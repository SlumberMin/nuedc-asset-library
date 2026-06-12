# -*- coding: utf-8 -*-
"""
模块1: 轻量级目标检测 - MobileNet-SSD / YOLOv8-nano
=====================================================
目标: Orange Pi 5 上 30fps+ 实时检测

技术方案:
  方案A: MobileNet-SSD (OpenCV DNN)  → 最简单, ~20fps
  方案B: YOLOv8-nano ONNX推理        → ~25fps
  方案C: YOLOv8-nano RKNN INT8推理   → ~40fps ★推荐

优化技巧:
  1. INT8量化: 精度损失<2%, 速度提升3x
  2. 输入320x320: 比640快4x, 小目标稍差
  3. NMS加速: 用cv2.dnn.NMSBoxes比手写快
  4. 多线程流水线: 采集/推理/显示各一线程

电赛场景:
  - 智能小车: 检测障碍物、目标物
  - 机械臂: 识别抓取目标
  - 无人机: 空中目标识别与追踪
"""

import cv2
import numpy as np
import time
import os


class MobileNetSSDDetector:
    """
    MobileNet-SSD 目标检测器 (OpenCV DNN)
    
    模型: MobileNetV2-SSD (COCO 80类)
    推理: OpenCV DNN后端, 支持CPU/NEON
    输入: 300x300 RGB
    输出: 类别, 置信度, 边界框
    
    用法:
        det = MobileNetSSDDetector()
        results = det.detect(frame)
        for cls, conf, box in results:
            print(f"{cls}: {conf:.2f} {box}")
    """
    
    # COCO 80类名称
    CLASSES = [
        'background', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
        'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
        'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
        'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
        'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
        'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
        'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
        'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
        'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
        'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
        'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
        'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven',
        'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
        'scissors', 'teddy bear', 'hair drier', 'toothbrush'
    ]
    
    def __init__(self, model_path=None, conf_threshold=0.5, input_size=300):
        """
        初始化检测器
        
        Args:
            model_path: 模型路径, None则使用OpenCV内置
            conf_threshold: 置信度阈值
            input_size: 输入尺寸(300或320)
        """
        self.conf_threshold = conf_threshold
        self.input_size = input_size
        
        if model_path and os.path.exists(model_path):
            # 自定义模型
            self.net = cv2.dnn.readNet(model_path)
        else:
            # 使用OpenCV内置MobileNet-SSD (caffe)
            prototxt = "deploy.prototxt"
            caffemodel = "mobilenet_iter_73000.caffemodel"
            if os.path.exists(prototxt) and os.path.exists(caffemodel):
                self.net = cv2.dnn.readNetFromCaffe(prototxt, caffemodel)
            else:
                # 用YOLOv8 ONNX作为后备
                self.net = None
                print("[检测器] 未找到模型, 使用颜色检测后备方案")
        
        # 尝试启用OpenCL加速
        self._setup_backend()
    
    def _setup_backend(self):
        """配置推理后端(OpenCL/NEON优化)"""
        if self.net is not None:
            try:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL)
                print("[检测器] 使用OpenCL加速")
            except Exception:
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                print("[检测器] 使用CPU推理")
    
    def detect(self, frame):
        """
        检测目标
        
        Args:
            frame: BGR输入图像
        Returns:
            results: [(class_name, confidence, (x,y,w,h)), ...]
        """
        if self.net is None:
            return self._fallback_color_detect(frame)
        
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame, 0.007843, (self.input_size, self.input_size), 
            127.5, swapRB=True, crop=False
        )
        self.net.setInput(blob)
        detections = self.net.forward()
        
        results = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.conf_threshold:
                class_id = int(detections[0, 0, i, 1])
                if class_id < len(self.CLASSES):
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    x1, y1, x2, y2 = box.astype(int)
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(w, x2), min(h, y2)
                    results.append((
                        self.CLASSES[class_id],
                        float(confidence),
                        (x1, y1, x2 - x1, y2 - y1)
                    ))
        
        # NMS去重
        if len(results) > 1:
            results = self._nms(results)
        return results
    
    def _nms(self, results, iou_threshold=0.45):
        """非极大值抑制"""
        boxes = [r[2] for r in results]
        scores = [r[1] for r in results]
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_threshold, iou_threshold)
        return [results[i] for i in indices.flatten()] if len(indices) > 0 else []
    
    def _fallback_color_detect(self, frame):
        """
        颜色检测后备方案(无需模型)
        检测红色/蓝色/绿色物体, 用于电赛调参
        
        Returns:
            [(color_name, area_ratio, (x,y,w,h)), ...]
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        results = []
        
        # 红色检测(电赛常见)
        mask_red = cv2.inRange(hsv, np.array([0,100,100]), np.array([10,255,255])) | \
                   cv2.inRange(hsv, np.array([160,100,100]), np.array([180,255,255]))
        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, bw, bh = cv2.boundingRect(cnt)
                results.append(("red", area/(h*w), (x,y,bw,bh)))
        
        # 蓝色检测
        mask_blue = cv2.inRange(hsv, np.array([100,100,100]), np.array([130,255,255]))
        contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, bw, bh = cv2.boundingRect(cnt)
                results.append(("blue", area/(h*w), (x,y,bw,bh)))
        
        # 绿色检测
        mask_green = cv2.inRange(hsv, np.array([35,100,100]), np.array([85,255,255]))
        contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, bw, bh = cv2.boundingRect(cnt)
                results.append(("green", area/(h*w), (x,y,bw,bh)))
        
        return results
    
    def draw_results(self, frame, results):
        """在图像上绘制检测结果"""
        for cls, conf, (x, y, w, h) in results:
            color = (0, 255, 0) if cls != "red" else (0, 0, 255)
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
            label = f"{cls}: {conf:.2f}"
            cv2.putText(frame, label, (x, y-5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return frame


class YOLOv8Detector:
    """
    YOLOv8-nano 检测器 (ONNX / RKNN)
    
    推荐部署路径:
      1. ultralytics导出: yolo export model=yolov8n.pt format=onnx
      2. RKNN转换: onnx→rknn (INT8量化)
      3. 或直接用onnxruntime推理
    
    性能参考(RK3588):
      - YOLOv8n 320x320 INT8: ~40fps
      - YOLOv8n 640x640 INT8: ~20fps
      - YOLOv8s 320x320 INT8: ~25fps
    """
    
    CLASSES = MobileNetSSDDetector.CLASSES  # 复用COCO类名
    
    def __init__(self, model_path="yolov8n.onnx", conf_threshold=0.45, 
                 input_size=320, use_rknn=False):
        self.conf_threshold = conf_threshold
        self.input_size = input_size
        self.use_rknn = use_rknn
        
        if use_rknn:
            from rknnlite.api import RKNNLite
            self.rknn = RKNNLite()
            ret = self.rknn.load_rknn(model_path.replace('.onnx', '.rknn'))
            if ret != 0:
                raise RuntimeError(f"RKNN模型加载失败: {model_path}")
            self.rknn.init_runtime()
            print("[YOLOv8] RKNN模型加载成功")
        else:
            self.net = cv2.dnn.readNetFromONNX(model_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            print("[YOLOv8] ONNX模型加载成功")
    
    def detect(self, frame):
        """
        YOLOv8推理
        
        Args:
            frame: BGR输入图像
        Returns:
            results: [(class_name, confidence, (x,y,w,h)), ...]
        """
        h, w = frame.shape[:2]
        img, ratio, (top, left) = self._preprocess(frame)
        
        if self.use_rknn:
            outputs = self.rknn.inference(inputs=[img])
            predictions = outputs[0]
        else:
            blob = cv2.dnn.blobFromImage(img, 1/255.0, swapRB=True, crop=False)
            self.net.setInput(blob)
            predictions = self.net.forward()
        
        return self._postprocess(predictions, h, w, ratio, top, left)
    
    def _preprocess(self, frame):
        """预处理: 缩放+填充"""
        from .platform_utils import resize_for_inference
        img, ratio, (top, left) = resize_for_inference(frame, self.input_size)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC→CHW
        img = np.expand_dims(img, 0)  # 添加batch维度
        return img, ratio, (top, left)
    
    def _postprocess(self, predictions, orig_h, orig_w, ratio, top, left):
        """后处理: 解码+NMS"""
        # YOLOv8输出格式: [1, 84, 8400] (4bbox + 80class)
        if len(predictions.shape) == 3:
            predictions = predictions[0]  # [84, 8400]
        
        # 转置: [8400, 84]
        predictions = predictions.T
        boxes = predictions[:, :4]
        scores = predictions[:, 4:]
        
        # 找到最大类别分数
        class_ids = np.argmax(scores, axis=1)
        confidences = scores[np.arange(len(scores)), class_ids]
        
        # 置信度过滤
        mask = confidences > self.conf_threshold
        boxes, confidences, class_ids = boxes[mask], confidences[mask], class_ids[mask]
        
        if len(boxes) == 0:
            return []
        
        # xywh → xyxy
        x1 = boxes[:, 0] - boxes[:, 2] / 2
        y1 = boxes[:, 1] - boxes[:, 3] / 2
        x2 = boxes[:, 0] + boxes[:, 2] / 2
        y2 = boxes[:, 1] + boxes[:, 3] / 2
        
        # 还原到原图坐标
        x1 = (x1 - left) / ratio
        y1 = (y1 - top) / ratio
        x2 = (x2 - left) / ratio
        y2 = (y2 - top) / ratio
        
        # 裁剪
        x1 = np.clip(x1, 0, orig_w)
        y1 = np.clip(y1, 0, orig_h)
        x2 = np.clip(x2, 0, orig_w)
        y2 = np.clip(y2, 0, orig_h)
        
        # NMS
        result_boxes = [[int(x1[i]), int(y1[i]), int(x2[i]-x1[i]), int(y2[i]-y1[i])] 
                        for i in range(len(x1))]
        indices = cv2.dnn.NMSBoxes(result_boxes, confidences.tolist(), 
                                    self.conf_threshold, 0.45)
        
        results = []
        for i in indices.flatten():
            cid = int(class_ids[i])
            if cid < len(self.CLASSES):
                results.append((self.CLASSES[cid], float(confidences[i]), 
                              tuple(result_boxes[i])))
        return results


# ===== 电赛应用示例 =====
def demo_obstacle_avoidance():
    """
    智能小车障碍物检测示例
    
    场景: 小车前进时检测前方障碍物
    输出: 障碍物位置 → 控制转向
    """
    from .platform_utils import CameraThread, FrameCounter, optimize_opencv
    optimize_opencv()
    
    cam = CameraThread(src=0, width=640, height=480).start()
    detector = MobileNetSSDDetector(conf_threshold=0.4)
    counter = FrameCounter()
    
    print("[智能小车] 障碍物检测启动...")
    while True:
        frame = cam.read()
        if frame is None:
            continue
        
        results = detector.detect(frame)
        frame = detector.draw_results(frame, results)
        
        # 判断障碍物方向
        h, w = frame.shape[:2]
        for cls, conf, (x, y, bw, bh) in results:
            cx = x + bw // 2
            if cx < w // 3:
                direction = "左侧"
            elif cx > w * 2 // 3:
                direction = "右侧"
            else:
                direction = "正前方"
            print(f"[障碍] {cls} 在{direction} 置信度:{conf:.2f}")
        
        counter.tick()
        cv2.putText(frame, f"FPS: {counter.fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Obstacle Detection", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cam.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo_obstacle_avoidance()
