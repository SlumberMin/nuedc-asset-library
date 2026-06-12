"""
行人检测模块 - HOG+SVM / DNN多尺度检测 + NMS
适用于：智能监控、自动驾驶辅助、电赛人流统计等场景
依赖：pip install opencv-python numpy
"""
import cv2
import numpy as np


class PedestrianDetector:
    """行人检测器，支持HOG+SVM和DNN两种方法"""

    def __init__(self, method='hog', confidence_threshold=0.5, nms_threshold=0.4):
        """
        初始化检测器
        Args:
            method: 'hog' 使用HOG+SVM, 'dnn' 使用DNN模型
            confidence_threshold: DNN置信度阈值
            nms_threshold: NMS非极大值抑制阈值
        """
        self.method = method
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold

        if method == 'hog':
            # HOG+SVM行人检测器（OpenCV内置，无需额外模型文件）
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        elif method == 'dnn':
            # 需要预下载 MobileNetSSD 模型，也可替换为YOLOv4-tiny
            # 下载地址：https://github.com/chuanqi305/MobileNet-SSD
            prototxt = 'MobileNetSSD_deploy.prototxt'
            model = 'MobileNetSSD_deploy.caffemodel'
            self.net = cv2.dnn.readNetFromCaffe(prototxt, model)
            # MobileNetSSD类别中 person=15
            self.person_class_id = 15
        else:
            raise ValueError("method 仅支持 'hog' 或 'dnn'")

    def detect_hog(self, image, win_stride=(8, 8), padding=(8, 8), scale=1.05):
        """
        HOG+SVM多尺度行人检测
        Args:
            image: BGR图像
            win_stride: 窗口滑动步长
            padding: 边界填充
            scale: 金字塔缩放因子
        Returns:
            boxes: [[x, y, w, h], ...]
        """
        # 多尺度检测
        boxes, weights = self.hog.detectMultiScale(
            image,
            winStride=win_stride,
            padding=padding,
            scale=scale
        )
        # 转为标准列表格式
        results = []
        for (x, y, w, h), w_conf in zip(boxes, weights):
            results.append([int(x), int(y), int(w), int(h), float(w_conf)])
        return results

    def detect_dnn(self, image):
        """
        DNN深度学习行人检测
        Args:
            image: BGR图像
        Returns:
            boxes: [[x1, y1, x2, y2, confidence], ...]
        """
        h, w = image.shape[:2]
        # 预处理：构建blob（300x300输入）
        blob = cv2.dnn.blobFromImage(
            image, scalefactor=0.007843,
            size=(300, 300), mean=(127.5, 127.5, 127.5)
        )
        self.net.setInput(blob)
        detections = self.net.forward()

        results = []
        for i in range(detections.shape[2]):
            class_id = int(detections[0, 0, i, 1])
            confidence = float(detections[0, 0, i, 2])

            if class_id == self.person_class_id and confidence > self.confidence_threshold:
                x1 = int(detections[0, 0, i, 3] * w)
                y1 = int(detections[0, 0, i, 4] * h)
                x2 = int(detections[0, 0, i, 5] * w)
                y2 = int(detections[0, 0, i, 6] * h)
                results.append([x1, y1, x2, y2, confidence])
        return results

    def non_max_suppression(self, boxes, scores=None):
        """
        非极大值抑制（NMS），去除重叠框
        Args:
            boxes: [[x1, y1, x2, y2], ...] 或 [[x, y, w, h], ...]
            scores: 对应置信度列表
        Returns:
            过滤后的boxes列表
        """
        if len(boxes) == 0:
            return []

        boxes_arr = np.array(boxes, dtype=np.float32)
        # 统一转为 x1,y1,x2,y2 格式（如果是x,y,w,h则转换）
        if boxes_arr.shape[1] == 4:
            # 判断是否为x,y,w,h格式（宽高为正数且x2可能>图像范围）
            # 简单判断：如果第三个值+第一个值 > 第三个值本身，认为是wh格式
            # 这里用更通用的方式：直接传给cv2.dnn.NMSBoxes
            pass

        if scores is None:
            scores = [1.0] * len(boxes)

        indices = cv2.dnn.NMSBoxes(
            boxes, scores,
            score_threshold=self.confidence_threshold,
            nms_threshold=self.nms_threshold
        )
        return [boxes[i] for i in indices.flatten()]

    def detect(self, image):
        """
        统一检测接口
        Args:
            image: BGR图像
        Returns:
            detections: 检测结果列表
        """
        if self.method == 'hog':
            return self.detect_hog(image)
        else:
            return self.detect_dnn(image)

    def draw_detections(self, image, detections, color=(0, 255, 0), thickness=2):
        """
        在图像上绘制检测结果
        Args:
            image: BGR图像（会被复制后绘制）
            detections: 检测结果
            color: 边框颜色 (B,G,R)
            thickness: 线条粗细
        Returns:
            绘制了检测框的图像
        """
        vis = image.copy()
        for det in detections:
            if len(det) == 5:
                # HOG格式: x, y, w, h, weight
                if self.method == 'hog':
                    x, y, w, h, conf = det
                    cv2.rectangle(vis, (x, y), (x + w, y + h), color, thickness)
                    cv2.putText(vis, f'{conf:.2f}', (x, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                else:
                    # DNN格式: x1, y1, x2, y2, confidence
                    x1, y1, x2, y2, conf = det
                    cv2.rectangle(vis, (x1, y1), (x2, y2), color, thickness)
                    cv2.putText(vis, f'Person {conf:.2f}', (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return vis

    def count_pedestrians(self, image):
        """
        统计图像中的行人数量
        Args:
            image: BGR图像
        Returns:
            count: 行人数量, annotated_image: 标注图像
        """
        detections = self.detect(image)
        # 对HOG结果应用NMS去除重叠
        if self.method == 'hog' and len(detections) > 0:
            boxes = [[d[0], d[1], d[0]+d[2], d[1]+d[3]] for d in detections]
            scores = [d[4] for d in detections]
            filtered = self.non_max_suppression(boxes, scores)
            count = len(filtered)
        else:
            count = len(detections)

        annotated = self.draw_detections(image, detections)
        cv2.putText(annotated, f'Count: {count}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return count, annotated


# ============== 使用示例 ==============
if __name__ == '__main__':
    # 示例1：HOG+SVM检测（无需额外模型）
    print("=== 行人检测示例（HOG+SVM）===")
    detector = PedestrianDetector(method='hog')

    # 从摄像头实时检测
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头，使用测试图像")
        # 创建测试图像
        test_img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(test_img, (200, 100), (280, 350), (180, 160, 140), -1)
        cv2.circle(test_img, (240, 90), 25, (180, 160, 140), -1)

        detections = detector.detect(test_img)
        count, result = detector.count_pedestrians(test_img)
        print(f"检测到 {count} 个行人")
        cv2.imwrite('pedestrian_result.jpg', result)
        print("结果已保存到 pedestrian_result.jpg")
    else:
        print("按 'q' 退出实时检测")
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            count, result = detector.count_pedestrians(frame)
            cv2.imshow('Pedestrian Detection', result)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

    # 示例2：图像文件检测
    # img = cv2.imread('street.jpg')
    # if img is not None:
    #     det = PedestrianDetector(method='hog')
    #     count, annotated = det.count_pedestrians(img)
    #     print(f"街景中检测到 {count} 位行人")
    #     cv2.imshow('Result', annotated)
    #     cv2.waitKey(0)
