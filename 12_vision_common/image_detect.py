"""
目标检测V2模块 - HOG+SVM / YOLO / SSD 检测
适用场景: 电赛中目标检测、行人检测、通用物体检测
依赖: opencv-python, numpy
可选: onnxruntime (ONNX推理)
"""

import cv2
import numpy as np
import os


class ImageDetectV2:
    """目标检测工具集V2"""

    # ---- HOG + SVM 行人检测 ----

    @staticmethod
    def detect_pedestrian(image, win_stride=(8, 8), padding=(8, 8), scale=1.05):
        """
        HOG+SVM 行人检测 (OpenCV内置模型)
        :param image: BGR图像
        :param win_stride: 滑窗步长
        :param padding: 填充
        :param scale: 多尺度缩放因子
        :return: (检测框列表 [(x,y,w,h), ...], 权重列表)
        """
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        rects, weights = hog.detectMultiScale(
            image, winStride=win_stride, padding=padding, scale=scale
        )

        # 非极大值抑制
        if len(rects) > 0:
            rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
            weights = np.array(weights).flatten()
            keep = ImageDetectV2._nms(rects, weights, threshold=0.3)
            rects = rects[keep]
            weights = weights[keep]
            rects = [(x1, y1, x2 - x1, y2 - y1) for (x1, y1, x2, y2) in rects]

        return rects, weights.tolist()

    @staticmethod
    def detect_hog_custom(image, hog_descriptor, svm_detector, win_size=(64, 128),
                          win_stride=(8, 8), scale=1.05):
        """
        自定义 HOG+SVM 检测 (使用训练好的分类器)
        :param image: BGR图像
        :param hog_descriptor: cv2.HOGDescriptor 实例
        :param svm_detector: SVM检测器系数
        :param win_size: 检测窗口大小
        :param win_stride: 滑窗步长
        :param scale: 多尺度因子
        :return: 检测框列表
        """
        hog_descriptor.setSVMDetector(svm_detector)
        rects, weights = hog_descriptor.detectMultiScale(
            image, winStride=win_stride, scale=scale
        )
        return [(x, y, w, h) for (x, y, w, h) in rects], weights

    # ---- YOLO 检测 (ONNX) ----

    @staticmethod
    def yolo_load(model_path):
        """
        加载 YOLO ONNX 模型
        :param model_path: .onnx 模型文件路径
        :return: cv2.dnn.Net 对象
        """
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"模型文件不存在: {model_path}")
        net = cv2.dnn.readNetFromONNX(model_path)
        return net

    @staticmethod
    def yolo_detect(image, net, input_size=(640, 640), conf_threshold=0.5,
                    nms_threshold=0.4, class_names=None):
        """
        YOLOv5/v8 目标检测 (ONNX推理)
        :param image: BGR图像
        :param net: cv2.dnn.Net (已加载的YOLO模型)
        :param input_size: 输入尺寸 (width, height)
        :param conf_threshold: 置信度阈值
        :param nms_threshold: NMS阈值
        :param class_names: 类别名称列表
        :return: list of dict {'box': (x,y,w,h), 'confidence': float, 'class_id': int, 'class_name': str}
        """
        h, w = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, input_size, swapRB=True, crop=False)
        net.setInput(blob)

        outputs = net.forward(net.getUnconnectedOutLayersNames())

        # 解析输出 (YOLOv5 格式: [batch, num_detections, 5+num_classes])
        detections = []
        if len(outputs) > 0:
            output = outputs[0]
            if output.ndim == 3:
                output = output[0]  # 去除batch维度

            # 计算缩放比例
            scale_x = w / input_size[0]
            scale_y = h / input_size[1]

            for det in output:
                conf = det[4]
                if conf < conf_threshold:
                    continue

                class_scores = det[5:]
                class_id = int(np.argmax(class_scores))
                class_conf = class_scores[class_id] * conf

                if class_conf < conf_threshold:
                    continue

                # 中心坐标 -> 左上角坐标
                cx, cy, bw, bh = det[0:4]
                x1 = int((cx - bw / 2) * scale_x)
                y1 = int((cy - bh / 2) * scale_y)
                box_w = int(bw * scale_x)
                box_h = int(bh * scale_y)

                name = class_names[class_id] if class_names and class_id < len(class_names) else str(class_id)
                detections.append({
                    'box': (max(0, x1), max(0, y1), box_w, box_h),
                    'confidence': float(class_conf),
                    'class_id': class_id,
                    'class_name': name,
                })

        # NMS
        if detections:
            boxes = np.array([d['box'] for d in detections])
            scores = np.array([d['confidence'] for d in detections])
            x1y1 = boxes[:, :2]
            x2y2 = boxes[:, :2] + boxes[:, 2:]
            nms_boxes = np.hstack([x1y1, x2y2]).astype(np.int32)
            keep = cv2.dnn.NMSBoxes(
                nms_boxes.tolist(), scores.tolist(), conf_threshold, nms_threshold
            )
            if len(keep) > 0:
                detections = [detections[i] for i in keep.flatten()]

        return detections

    # ---- SSD 检测 (OpenCV DNN) ----

    @staticmethod
    def ssd_load_caffe(prototxt_path, model_path):
        """
        加载 Caffe SSD 模型
        :param prototxt_path: 部署文件 (.prototxt)
        :param model_path: 模型文件 (.caffemodel)
        :return: cv2.dnn.Net
        """
        return cv2.dnn.readNetFromCaffe(prototxt_path, model_path)

    @staticmethod
    def ssd_load_tf(pb_path, pbtxt_path):
        """
        加载 TensorFlow SSD 模型
        :param pb_path: 冻结图 (.pb)
        :param pbtxt_path: 配置文件 (.pbtxt)
        :return: cv2.dnn.Net
        """
        return cv2.dnn.readNetFromTensorflow(pb_path, pbtxt_path)

    @staticmethod
    def ssd_detect(image, net, input_size=(300, 300), conf_threshold=0.5,
                   class_names=None):
        """
        SSD 目标检测
        :param image: BGR图像
        :param net: cv2.dnn.Net
        :param input_size: 输入尺寸
        :param conf_threshold: 置信度阈值
        :param class_names: 类别名称列表 (如 VOC 20类)
        :return: list of dict
        """
        h, w = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 1.0, input_size, (104, 117, 123))
        net.setInput(blob)
        output = net.forward()

        detections = []
        for i in range(output.shape[2]):
            confidence = output[0, 0, i, 2]
            if confidence < conf_threshold:
                continue

            class_id = int(output[0, 0, i, 1])
            x1 = int(output[0, 0, i, 3] * w)
            y1 = int(output[0, 0, i, 4] * h)
            x2 = int(output[0, 0, i, 5] * w)
            y2 = int(output[0, 0, i, 6] * h)

            name = class_names[class_id] if class_names and class_id < len(class_names) else str(class_id)
            detections.append({
                'box': (x1, y1, x2 - x1, y2 - y1),
                'confidence': float(confidence),
                'class_id': class_id,
                'class_name': name,
            })

        return detections

    # ---- 绘制检测结果 ----

    @staticmethod
    def draw_detections(image, detections, color=(0, 255, 0), thickness=2, font_scale=0.6):
        """
        在图像上绘制检测结果
        :param image: 输入图像
        :param detections: 检测结果列表 (含 'box', 'class_name', 'confidence')
        :return: 绘制后的图像
        """
        vis = image.copy()
        for det in detections:
            x, y, w, h = det['box']
            label = f"{det.get('class_name', '')} {det.get('confidence', 0):.2f}"
            cv2.rectangle(vis, (x, y), (x + w, y + h), color, thickness)
            cv2.putText(vis, label, (x, max(0, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)
        return vis

    # ---- NMS 工具 ----

    @staticmethod
    def _nms(boxes, scores, threshold=0.3):
        """非极大值抑制 (numpy实现)"""
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
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
            inds = np.where(iou <= threshold)[0]
            order = order[inds + 1]

        return keep


# ======================== 快捷函数 ========================

def detect_pedestrian(image, scale=1.05):
    return ImageDetectV2.detect_pedestrian(image, scale=scale)

def yolo_detect(image, model_path, conf_threshold=0.5, class_names=None):
    net = ImageDetectV2.yolo_load(model_path)
    return ImageDetectV2.yolo_detect(image, net, conf_threshold=conf_threshold, class_names=class_names)

def ssd_detect(image, prototxt, caffemodel, conf_threshold=0.5, class_names=None):
    net = ImageDetectV2.ssd_load_caffe(prototxt, caffemodel)
    return ImageDetectV2.ssd_detect(image, net, conf_threshold=conf_threshold, class_names=class_names)
