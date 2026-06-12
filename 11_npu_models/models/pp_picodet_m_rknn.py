"""
PP-PicoDet-M 目标检测 RKNN 推理封装
====================================
PaddleDetection 出品的轻量级检测模型，anchor-free。
适配 PP-PicoDet 导出的ONNX格式。

用法:
    model = PPPicoDetMRKNN("pp_picodet_m.rknn", conf_thres=0.4, iou_thres=0.5)
    results = model.infer(image_bgr)
"""

import numpy as np
import cv2
import time
from typing import List, Tuple, Optional, Dict

try:
    from rknnlite.api import RKNNLite as RKNN
except ImportError:
    from rknn.api import RKNN


class PPPicoDetMRKNN:
    """PP-PicoDet-M 目标检测 RKNN 推理封装"""

    CLASSES = [
        'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
        'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
        'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
        'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
        'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
        'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
        'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
        'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
        'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
        'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
        'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
        'hair drier', 'toothbrush'
    ]

    # PicoDet 默认输出层名称 (根据实际导出调整)
    OUTPUT_NAMES = ['stride_8', 'stride_16', 'stride_32']

    def __init__(
        self,
        model_path: str,
        input_size: int = 416,
        conf_thres: float = 0.4,
        iou_thres: float = 0.5,
        classes: Optional[List[int]] = None,
        nms_top_k: int = 1000,
        core_mask: int = 0,
        perf_debug: bool = False,
    ):
        self.input_size = input_size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.classes = classes
        self.nms_top_k = nms_top_k
        self.perf_debug = perf_debug

        self.strides = [8, 16, 32]
        self.reg_max = 7  # DFL regression max

        self.rknn = RKNN(verbose=False)
        ret = self.rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError(f"加载RKNN模型失败: {model_path}, ret={ret}")
        ret = self.rknn.init_runtime(core_mask=core_mask)
        if ret != 0:
            raise RuntimeError(f"初始化RKNN运行时失败, ret={ret}")

        self._warmup()

    def _warmup(self, n: int = 3):
        dummy = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
        for _ in range(n):
            self._raw_infer(dummy)

    def _raw_infer(self, img: np.ndarray) -> List[np.ndarray]:
        return self.rknn.inference(inputs=[img])

    def _preprocess(self, img: np.ndarray) -> Tuple[np.ndarray, float, Tuple[int, int]]:
        h, w = img.shape[:2]
        new_size = self.input_size
        scale = min(new_size / w, new_size / h)
        nw, nh = int(w * scale), int(h * scale)
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        pad_w = (new_size - nw) // 2
        pad_h = (new_size - nh) // 2
        padded = np.full((new_size, new_size, 3), 114, dtype=np.uint8)
        padded[pad_h:pad_h + nh, pad_w:pad_w + nw] = resized
        return padded, scale, (pad_w, pad_h)

    @staticmethod
    def _sigmoid(x: np.ndarray) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-np.clip(x, -50, 50)))

    def _generate_anchors(self) -> np.ndarray:
        """生成anchor points (center-based)"""
        anchors = []
        for stride in self.strides:
            h = w = self.input_size // stride
            shift_x = (np.arange(w) + 0.5) * stride
            shift_y = (np.arange(h) + 0.5) * stride
            xv, yv = np.meshgrid(shift_x, shift_y)
            anchor = np.stack([xv.ravel(), yv.ravel()], axis=-1)
            anchors.append(anchor)
        return np.concatenate(anchors, axis=0)

    def _decode_dfl(self, reg_output: np.ndarray) -> np.ndarray:
        """DFL (Distribution Focal Loss) 解码为边界距离"""
        # reg_output: [N, 4*(reg_max+1)] -> [N, 4]
        n = reg_output.shape[0]
        reg_output = reg_output.reshape(n, 4, self.reg_max + 1)
        reg_output = self._sigmoid(reg_output)
        # 加权求和
        weights = np.arange(self.reg_max + 1, dtype=np.float32)
        decoded = np.sum(reg_output * weights, axis=-1)  # [N, 4]
        return decoded

    def _postprocess(self, outputs: List[np.ndarray], scale: float, pad: Tuple[int, int], orig_shape: Tuple[int, int]):
        """
        PicoDet 后处理:
        outputs[0]: cls_scores [1, total_anchors, num_classes]
        outputs[1]: reg_preds  [1, total_anchors, 4*(reg_max+1)]
        """
        if len(outputs) >= 2:
            cls_output = outputs[0][0]  # [total_anchors, nc]
            reg_output = outputs[1][0]  # [total_anchors, 4*(reg_max+1)]
        else:
            # 兼容合并输出格式
            combined = outputs[0][0]
            nc = 80
            cls_output = combined[:, :nc]
            reg_output = combined[:, nc:]

        anchors = self._generate_anchors()

        # 分类得分 sigmoid
        cls_scores = self._sigmoid(cls_output)
        max_scores = np.max(cls_scores, axis=1)
        class_ids = np.argmax(cls_scores, axis=1)

        # 置信度过滤
        mask = max_scores >= self.conf_thres
        cls_scores = cls_scores[mask]
        max_scores = max_scores[mask]
        class_ids = class_ids[mask]
        reg_preds = reg_output[mask]
        anchor_pts = anchors[mask]

        if self.classes is not None:
            cls_mask = np.isin(class_ids, self.classes)
            max_scores = max_scores[cls_mask]
            class_ids = class_ids[cls_mask]
            reg_preds = reg_preds[cls_mask]
            anchor_pts = anchor_pts[cls_mask]

        if len(max_scores) == 0:
            return []

        # DFL 解码
        dist = self._decode_dfl(reg_preds)  # [N, 4] -> left, top, right, bottom

        # anchor points + distance -> xyxy
        x1 = anchor_pts[:, 0] - dist[:, 0]
        y1 = anchor_pts[:, 1] - dist[:, 1]
        x2 = anchor_pts[:, 0] + dist[:, 2]
        y2 = anchor_pts[:, 1] + dist[:, 3]

        boxes = np.stack([x1, y1, x2, y2], axis=1)

        # Top-K
        if len(max_scores) > self.nms_top_k:
            top_k_idx = np.argsort(max_scores)[::-1][:self.nms_top_k]
            boxes = boxes[top_k_idx]
            max_scores = max_scores[top_k_idx]
            class_ids = class_ids[top_k_idx]

        # 去 letterbox
        pad_w, pad_h = pad
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_w) / scale
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_h) / scale

        oh, ow = orig_shape
        boxes = np.clip(boxes, [0, 0, 0, 0], [ow, oh, ow, oh])

        # NMS
        dets = np.column_stack([boxes, max_scores, class_ids])
        keep = self._nms(dets[:, :4], dets[:, 4], self.iou_thres)
        dets = dets[keep]

        results = []
        for det in dets:
            cid = int(det[5])
            results.append({
                'bbox': det[:4].tolist(),
                'confidence': float(det[4]),
                'class_id': cid,
                'class_name': self.CLASSES[cid] if cid < len(self.CLASSES) else str(cid),
            })
        return results

    @staticmethod
    def _nms(boxes, scores, iou_thres):
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(int(i))
            if order.size == 1:
                break
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-7)
            inds = np.where(iou <= iou_thres)[0]
            order = order[inds + 1]
        return keep

    def infer(self, img: np.ndarray) -> List[Dict]:
        t0 = time.perf_counter()
        orig_shape = img.shape[:2]
        padded, scale, pad = self._preprocess(img)
        t_pre = time.perf_counter()
        outputs = self._raw_infer(padded)
        t_infer = time.perf_counter()
        results = self._postprocess(outputs, scale, pad, orig_shape)
        t_post = time.perf_counter()
        if self.perf_debug:
            print(f"[PicoDet-M] 预处理:{(t_pre-t0)*1000:.1f}ms  "
                  f"推理:{(t_infer-t_pre)*1000:.1f}ms  "
                  f"后处理:{(t_post-t_infer)*1000:.1f}ms  "
                  f"总计:{(t_post-t0)*1000:.1f}ms  "
                  f"检测:{len(results)}个目标")
        return results

    def draw_results(self, img: np.ndarray, results: List[Dict], thickness: int = 2) -> np.ndarray:
        vis = img.copy()
        for r in results:
            x1, y1, x2, y2 = [int(v) for v in r['bbox']]
            label = f"{r['class_name']} {r['confidence']:.2f}"
            cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 0, 0), thickness)
            cv2.putText(vis, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        return vis

    def release(self):
        if self.rknn:
            self.rknn.release()


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 3:
        print("用法: python pp_picodet_m_rknn.py <model.rknn> <image.jpg>")
        sys.exit(1)
    model = PPPicoDetMRKNN(sys.argv[1], perf_debug=True)
    img = cv2.imread(sys.argv[2])
    results = model.infer(img)
    for r in results:
        print(f"{r['class_name']}: {r['confidence']:.3f}  bbox={r['bbox']}")
    cv2.imwrite("result_picodet_m.jpg", model.draw_results(img, results))
    print("结果已保存到 result_picodet_m.jpg")
    model.release()
