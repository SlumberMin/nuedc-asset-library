"""
PP-PicoDet-S 目标检测 RKNN 推理封装
PaddlePaddle PicoDet-S: 小目标检测优化版, 比PicoDet精度更高
适配 RK3588S NPU, 支持多头输出解码
"""
import numpy as np
import cv2
import time

try:
    from rknnlite.api import RKNNLite
except ImportError:
    print("[WARN] rknnlite 未安装, 将使用模拟模式")
    RKNNLite = None


class PPPicoDetSRKNN:
    """PP-PicoDet-S RKNN推理封装"""

    COCO_CLASSES = [
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

    def __init__(self, model_path, input_size=416, conf_thres=0.5, nms_thres=0.6,
                 num_classes=80, target_platform='rk3588', core_mask=None):
        """
        Args:
            model_path: RKNN模型路径
            input_size: 输入尺寸 (320/416/640)
            conf_thres: 置信度阈值
            nms_thres: NMS IoU阈值
            num_classes: 类别数
            target_platform: 目标平台
            core_mask: NPU核心掩码
        """
        self.model_path = model_path
        self.input_size = input_size
        self.conf_thres = conf_thres
        self.nms_thres = nms_thres
        self.num_classes = num_classes
        self.target_platform = target_platform
        self.core_mask = core_mask
        self.rknn = None
        self._load_time = 0

        # PicoDet-S 使用 SimOTA分配策略, 含4个FPN层
        self.num_fpn_levels = 4
        # 归一化参数 (PaddlePaddle标准)
        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

    def load(self):
        """加载RKNN模型"""
        t0 = time.time()
        if RKNNLite is None:
            print(f"[SIM] 模拟加载模型: {self.model_path}")
            self._load_time = time.time() - t0
            return True

        self.rknn = RKNNLite()
        ret = self.rknn.load_rknn(self.model_path)
        if ret != 0:
            print(f"[ERR] 加载RKNN模型失败: {self.model_path}")
            return False

        if self.core_mask is not None:
            ret = self.rknn.init_runtime(core_mask=self.core_mask)
        else:
            ret = self.rknn.init_runtime(target=None)
        if ret != 0:
            print("[ERR] 初始化运行时失败")
            return False

        self._load_time = time.time() - t0
        print(f"[OK] PP-PicoDet-S模型加载成功: {self.model_path} ({self._load_time:.2f}s)")
        return True

    def preprocess(self, img):
        """
        PicoDet-S预处理: Letterbox resize + 归一化
        Returns: (input_data, scale_info)
        """
        h, w = img.shape[:2]
        target = self.input_size

        # Letterbox (保持宽高比)
        ratio = min(target / h, target / w)
        new_h, new_w = int(h * ratio), int(w * ratio)
        img_resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        pad_h = (target - new_h) // 2
        pad_w = (target - new_w) // 2
        img_padded = np.full((target, target, 3), 114, dtype=np.uint8)
        img_padded[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = img_resized

        # 归一化: /255, 标准ImageNet均值方差
        img_rgb = cv2.cvtColor(img_padded, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        for c in range(3):
            img_rgb[:, :, c] = (img_rgb[:, :, c] - self.mean[c]) / self.std[c]

        input_data = np.transpose(img_rgb, (2, 0, 1))
        input_data = np.expand_dims(input_data, axis=0)

        scale_info = {
            'ratio': ratio,
            'pad_w': pad_w,
            'pad_h': pad_h,
            'orig_shape': (h, w),
        }
        return input_data, scale_info

    def preprocess_direct(self, img):
        """
        直接resize预处理 (无Letterbox)
        适用于某些ONNX导出格式
        """
        h, w = img.shape[:2]
        target = self.input_size
        img_resized = cv2.resize(img, (target, target))

        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        for c in range(3):
            img_rgb[:, :, c] = (img_rgb[:, :, c] - self.mean[c]) / self.std[c]

        input_data = np.transpose(img_rgb, (2, 0, 1))
        input_data = np.expand_dims(input_data, axis=0)

        scale_info = {'scale_x': target / w, 'scale_y': target / h}
        return input_data, scale_info

    def postprocess(self, outputs, scale_info):
        """
        PicoDet-S后处理
        支持两种输出格式:
        1. 单头输出: [1, N, 6] -> class_id, score, x1, y1, x2, y2
        2. 多头FPN输出: list of [1, Ni, 6]
        """
        if isinstance(outputs, list):
            # 多头输出合并
            all_boxes = []
            for out in outputs:
                out = out[0] if out.ndim == 3 else out
                if out.ndim == 2 and out.shape[-1] >= 6:
                    all_boxes.append(out)
            if all_boxes:
                output = np.concatenate(all_boxes, axis=0)
            else:
                output = outputs[0]
                output = output[0] if output.ndim == 3 else output
        else:
            output = outputs
            output = output[0] if output.ndim == 3 else output

        if output.ndim == 1:
            output = output.reshape(-1, 6)

        # 格式: class_id, score, x1, y1, x2, y2
        class_ids = output[:, 0].astype(int)
        scores = output[:, 1]
        boxes = output[:, 2:6].copy()

        # 置信度过滤
        mask = scores > self.conf_thres
        boxes = boxes[mask]
        scores = scores[mask]
        class_ids = class_ids[mask]

        if len(boxes) == 0:
            return [], [], []

        # 坐标映射回原图
        if 'ratio' in scale_info:
            # Letterbox 模式
            ratio = scale_info['ratio']
            pad_w = scale_info['pad_w']
            pad_h = scale_info['pad_h']
            boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_w) / ratio
            boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_h) / ratio
        else:
            # 直接resize模式
            boxes[:, [0, 2]] = boxes[:, [0, 2]] / scale_info['scale_x']
            boxes[:, [1, 3]] = boxes[:, [1, 3]] / scale_info['scale_y']

        # NMS
        keep = self._nms(boxes, scores, self.nms_thres)
        return boxes[keep].tolist(), scores[keep].tolist(), class_ids[keep].tolist()

    def _nms(self, boxes, scores, iou_threshold):
        """NMS"""
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
            order = order[np.where(iou <= iou_threshold)[0] + 1]
        return np.array(keep)

    def inference(self, img):
        """完整推理流程"""
        t0 = time.time()
        input_data, scale_info = self.preprocess(img)

        if self.rknn is None:
            # 模拟: 4个FPN头输出
            outputs = [np.random.randn(1, 100, 6).astype(np.float32) for _ in range(4)]
        else:
            outputs = self.rknn.inference(inputs=[input_data])

        t_infer = (time.time() - t0) * 1000
        boxes, scores, class_ids = self.postprocess(outputs, scale_info)
        return boxes, scores, class_ids, t_infer

    def inference_with_label(self, img):
        """推理并返回带类别名称的结果"""
        boxes, scores, class_ids, t = self.inference(img)
        results = []
        for box, score, cid in zip(boxes, scores, class_ids):
            results.append({
                'bbox': box,
                'score': score,
                'class_id': cid,
                'class_name': self.get_class_name(cid),
            })
        return results, t

    def get_class_name(self, class_id):
        if 0 <= class_id < len(self.COCO_CLASSES):
            return self.COCO_CLASSES[class_id]
        return f"class_{class_id}"

    def get_model_info(self):
        info = {
            'model_path': self.model_path,
            'input_size': self.input_size,
            'model_type': 'PP-PicoDet-S',
            'num_classes': self.num_classes,
            'conf_thres': self.conf_thres,
            'nms_thres': self.nms_thres,
            'num_fpn_levels': self.num_fpn_levels,
            'load_time': self._load_time,
        }
        return info

    def release(self):
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None
            print("[OK] PP-PicoDet-S资源已释放")


if __name__ == '__main__':
    model = PPPicoDetSRKNN('picodet_s.rknn', input_size=416)
    model.load()
    img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    boxes, scores, ids, t = model.inference(img)
    print(f"推理耗时: {t:.1f}ms, 检测到 {len(boxes)} 个目标")
    print(f"模型信息: {model.get_model_info()}")

    results, t2 = model.inference_with_label(img)
    for r in results[:5]:
        print(f"  {r['class_name']}: {r['score']:.3f} @ {r['bbox']}")
    model.release()
