"""
车辆检测模块 - Haar级联 / YOLOv4-tiny轻量级检测 + 目标跟踪
适用于：交通流量统计、停车场管理、电赛车流量检测等场景
依赖：pip install opencv-python numpy
"""
import cv2
import numpy as np
import time


class VehicleDetector:
    """车辆检测器，支持Haar级联和DNN方法"""

    def __init__(self, method='haar', model_path=None, config_path=None):
        """
        初始化车辆检测器
        Args:
            method: 'haar' 使用Haar级联, 'dnn' 使用YOLOv4-tiny等DNN
            model_path: DNN模型权重路径
            config_path: DNN配置文件路径
        """
        self.method = method

        if method == 'haar':
            # 使用OpenCV内置的车辆Haar级联分类器
            cascade_path = cv2.data.haarcascades + 'haarcascade_car.xml'
            # 备选：如果不存在则用俄罗斯车牌级联或自定义路径
            try:
                self.car_cascade = cv2.CascadeClassifier(cascade_path)
                if self.car_cascade.empty():
                    # 尝试下载或使用替代方案
                    print("提示：haarcascade_car.xml 未找到，使用 frontalface 作为占位测试")
                    self.car_cascade = cv2.CascadeClassifier(
                        cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
                    )
            except Exception:
                self.car_cascade = cv2.CascadeClassifier(
                    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
                )

        elif method == 'dnn':
            # YOLOv4-tiny 检测（需要下载模型文件）
            # 下载: https://github.com/AlexeyAB/darknet
            # 权重: yolov4-tiny.weights  配置: yolov4-tiny.cfg  类别: coco.names
            if model_path and config_path:
                self.net = cv2.dnn.readNetFromDarknet(config_path, model_path)
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                # COCO中车辆相关类别ID
                self.vehicle_classes = ['car', 'truck', 'bus', 'motorbike', 'bicycle']
                with open('coco.names', 'r') as f:
                    self.class_names = [line.strip() for line in f.readlines()]
                self.output_layers = self.net.getUnconnectedOutLayersNames()
            else:
                raise ValueError("DNN模式需要提供 model_path 和 config_path")

    def detect_haar(self, image, min_size=(80, 80), scale_factor=1.1, min_neighbors=3):
        """
        Haar级联车辆检测
        Args:
            image: BGR图像
            min_size: 最小检测尺寸
            scale_factor: 缩放因子
            min_neighbors: 最小邻居数（越大越严格）
        Returns:
            boxes: [[x, y, w, h], ...]
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # 直方图均衡化增强对比度
        gray = cv2.equalizeHist(gray)
        vehicles = self.car_cascade.detectMultiScale(
            gray,
            scaleFactor=scale_factor,
            minNeighbors=min_neighbors,
            minSize=min_size,
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        return [[int(x), int(y), int(w), int(h)] for (x, y, w, h) in vehicles]

    def detect_dnn(self, image, conf_threshold=0.5, nms_threshold=0.4):
        """
        YOLOv4-tiny DNN车辆检测
        Args:
            image: BGR图像
            conf_threshold: 置信度阈值
            nms_threshold: NMS阈值
        Returns:
            results: [{'box': [x,y,w,h], 'class': name, 'confidence': float}, ...]
        """
        h, w = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)

        boxes, confidences, class_ids = [], [], []
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                class_name = self.class_names[class_id]

                if confidence > conf_threshold and class_name in self.vehicle_classes:
                    center_x = int(detection[0] * w)
                    center_y = int(detection[1] * h)
                    dw = int(detection[2] * w)
                    dh = int(detection[3] * h)
                    x = int(center_x - dw / 2)
                    y = int(center_y - dh / 2)
                    boxes.append([x, y, dw, dh])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # NMS
        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
        results = []
        for i in indices.flatten():
            results.append({
                'box': boxes[i],
                'class': self.class_names[class_ids[i]],
                'confidence': confidences[i]
            })
        return results

    def detect(self, image):
        """统一检测接口"""
        if self.method == 'haar':
            return self.detect_haar(image)
        else:
            return self.detect_dnn(image)

    def draw_detections(self, image, detections, color=(0, 255, 0)):
        """绘制检测结果"""
        vis = image.copy()
        for det in detections:
            if isinstance(det, dict):
                x, y, w, h = det['box']
                label = f"{det['class']} {det['confidence']:.2f}"
            else:
                x, y, w, h = det[:4]
                label = "Vehicle"
            cv2.rectangle(vis, (x, y), (x + w, y + h), color, 2)
            cv2.putText(vis, label, (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return vis


class VehicleTracker:
    """
    简易车辆跟踪器（基于质心距离匹配）
    适合电赛场景的轻量级多目标跟踪
    """

    def __init__(self, max_disappeared=30, max_distance=80):
        """
        Args:
            max_disappeared: 目标消失多少帧后删除
            max_distance: 质心匹配最大距离（像素）
        """
        self.next_id = 0
        self.objects = {}         # id -> centroid
        self.disappeared = {}     # id -> 消失帧数
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance

    def register(self, centroid):
        """注册新目标"""
        self.objects[self.next_id] = centroid
        self.disappeared[self.next_id] = 0
        self.next_id += 1

    def deregister(self, object_id):
        """注销目标"""
        del self.objects[object_id]
        del self.disappeared[object_id]

    def update(self, boxes):
        """
        更新跟踪状态
        Args:
            boxes: [[x, y, w, h], ...] 检测到的目标框
        Returns:
            {id: (cx, cy), ...} 当前跟踪的目标及质心
        """
        if len(boxes) == 0:
            for obj_id in list(self.disappeared.keys()):
                self.disappeared[obj_id] += 1
                if self.disappeared[obj_id] > self.max_disappeared:
                    self.deregister(obj_id)
            return self.objects.copy()

        # 计算当前帧检测框的质心
        centroids = []
        for (x, y, w, h) in boxes:
            centroids.append((int(x + w / 2), int(y + h / 2)))
        centroids = np.array(centroids)

        if len(self.objects) == 0:
            for c in centroids:
                self.register(tuple(c))
        else:
            # 匹配已有目标和新检测
            obj_ids = list(self.objects.keys())
            obj_centroids = np.array(list(self.objects.values()))

            # 计算距离矩阵
            distances = np.linalg.norm(
                obj_centroids[:, np.newaxis] - centroids[np.newaxis, :],
                axis=2
            )

            # 贪心匹配（按距离从小到大）
            rows = distances.min(axis=1).argsort()
            cols = distances.argmin(axis=1)[rows]

            used_rows, used_cols = set(), set()
            for row, col in zip(rows, cols):
                if row in used_rows or col in used_cols:
                    continue
                if distances[row, col] > self.max_distance:
                    continue
                obj_id = obj_ids[row]
                self.objects[obj_id] = tuple(centroids[col])
                self.disappeared[obj_id] = 0
                used_rows.add(row)
                used_cols.add(col)

            # 未匹配的已有目标
            for row in set(range(len(obj_ids))) - used_rows:
                obj_id = obj_ids[row]
                self.disappeared[obj_id] += 1
                if self.disappeared[obj_id] > self.max_disappeared:
                    self.deregister(obj_id)

            # 未匹配的新检测 -> 注册
            for col in set(range(len(centroids))) - used_cols:
                self.register(tuple(centroids[col]))

        return self.objects.copy()


# ============== 使用示例 ==============
if __name__ == '__main__':
    print("=== 车辆检测 + 跟踪示例 ===")

    # 创建检测器和跟踪器
    detector = VehicleDetector(method='haar')
    tracker = VehicleTracker(max_disappeared=20, max_distance=100)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头，使用模拟数据演示")

        # 模拟60帧场景
        for frame_idx in range(60):
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            # 模拟移动车辆
            x = 50 + frame_idx * 5
            cv2.rectangle(img, (x, 200), (x + 100, 260), (200, 200, 200), -1)

            detections = detector.detect_haar(img)
            tracked = tracker.update(detections)

            vis = detector.draw_detections(img, detections)
            for obj_id, centroid in tracked.items():
                cv2.putText(vis, f'ID:{obj_id}', (centroid[0]-10, centroid[1]-20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.circle(vis, centroid, 4, (0, 0, 255), -1)

            cv2.putText(vis, f'Vehicles: {len(tracked)}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        print(f"跟踪结束，共跟踪 {tracker.next_id} 个目标")
        cv2.imwrite('vehicle_tracking_result.jpg', vis)
        print("最后一帧已保存")
    else:
        print("按 'q' 退出")
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            detections = detector.detect_haar(frame)
            tracked = tracker.update(detections)

            vis = detector.draw_detections(frame, detections)
            for obj_id, centroid in tracked.items():
                cv2.putText(vis, f'ID:{obj_id}', (centroid[0]-10, centroid[1]-20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.imshow('Vehicle Tracking', vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()
