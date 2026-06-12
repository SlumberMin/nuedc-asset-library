"""
PP-PicoDet 目标检测 RKNN 推理封装
PaddlePaddle轻量级检测模型, 适合边缘部署
"""
import numpy as np
import cv2
import time

try:
    from rknnlite.api import RKNNLite
except ImportError:
    print("[WARN] rknnlite 未安装, 将使用模拟模式")
    RKNNLite = None


class PPPicoDetRKNN:
    """PP-PicoDet RKNN推理封装"""

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

    def __init__(self, model_path, input_size=320, conf_thres=0.5, nms_thres=0.6,
                 num_classes=80, target_platform='rk3588'):
        self.model_path = model_path
        self.input_size = input_size
        self.conf_thres = conf_thres
        self.nms_thres = nms_thres
        self.num_classes = num_classes
        self.target_platform = target_platform
        self.rknn = None

    def load(self):
        """加载RKNN模型"""
        if RKNNLite is None:
            print("[SIM] 模拟加载模型:", self.model_path)
            return True

        self.rknn = RKNNLite()
        ret = self.rknn.load_rknn(self.model_path)
        if ret != 0:
            print(f"[ERR] 加载RKNN模型失败: {self.model_path}")
            return False
        ret = self.rknn.init_runtime(target=None)
        if ret != 0:
            print("[ERR] 初始化运行时失败")
            return False
        print(f"[OK] PP-PicoDet模型加载成功: {self.model_path}")
        return True

    def preprocess(self, img):
        """PicoDet预处理: resize + 归一化"""
        h, w = img.shape[:2]
        target = self.input_size

        # 直接resize到目标尺寸
        img_resized = cv2.resize(img, (target, target))

        # 归一化: /255, 标准ImageNet均值方差
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        for c in range(3):
            img_rgb[:, :, c] = (img_rgb[:, :, c] - mean[c]) / std[c]

        input_data = np.transpose(img_rgb, (2, 0, 1))
        input_data = np.expand_dims(input_data, axis=0)

        scale_x = target / w
        scale_y = target / h
        return input_data, (scale_x, scale_y)

    def postprocess(self, outputs, scale_info):
        """
        PicoDet后处理
        输出: [bboxes (N,6)] -> class_id, score, x1,y1,x2,y2
        或多头输出
        """
        scale_x, scale_y = scale_info

        if isinstance(outputs, list):
            # 多头输出合并
            all_boxes = []
            for out in outputs:
                out = out[0] if out.ndim == 3 else out
                if out.ndim == 2:
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
        boxes = output[:, 2:6]

        # 置信度过滤
        mask = scores > self.conf_thres
        boxes = boxes[mask]
        scores = scores[mask]
        class_ids = class_ids[mask]

        if len(boxes) == 0:
            return [], [], []

        # 坐标映射回原图
        boxes[:, [0, 2]] = boxes[:, [0, 2]] / scale_x
        boxes[:, [1, 3]] = boxes[:, [1, 3]] / scale_y

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
            iou = inter / (areas[i] + areas[order[1:]] - inter)
            order = order[np.where(iou <= iou_threshold)[0] + 1]
        return np.array(keep)

    def inference(self, img):
        """完整推理流程"""
        t0 = time.time()
        input_data, scale_info = self.preprocess(img)

        if self.rknn is None:
            outputs = [np.random.randn(1, 100, 6).astype(np.float32)]
        else:
            outputs = self.rknn.inference(inputs=[input_data])

        t_infer = (time.time() - t0) * 1000
        boxes, scores, class_ids = self.postprocess(outputs, scale_info)
        return boxes, scores, class_ids, t_infer

    def get_class_name(self, class_id):
        if 0 <= class_id < len(self.COCO_CLASSES):
            return self.COCO_CLASSES[class_id]
        return f"class_{class_id}"

    def release(self):
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None
            print("[OK] PP-PicoDet资源已释放")


if __name__ == '__main__':
    model = PPPicoDetRKNN('picodet_s.rknn', input_size=320)
    model.load()
    img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    boxes, scores, ids, t = model.inference(img)
    print(f"推理耗时: {t:.1f}ms, 检测到 {len(boxes)} 个目标")
    model.release()
