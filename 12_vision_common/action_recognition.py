"""
动作识别模块 - 骨骼关键点 + 时序分类 + 手势/姿态识别
功能：基于OpenCV DNN的姿态估计、手势识别、动作分类
依赖：opencv-python, numpy
"""

import cv2
import numpy as np
from collections import deque


# ==================== 姿态估计（OpenPose轻量版） ====================

class PoseEstimator:
    """
    基于OpenCV DNN的人体姿态估计
    使用OpenPose的COCO模型或MobileNet模型
    """

    # COCO 18个关键点定义
    BODY_PARTS = {
        "Nose": 0, "Neck": 1, "RShoulder": 2, "RElbow": 3, "RWrist": 4,
        "LShoulder": 5, "LElbow": 6, "LWrist": 7, "RHip": 8, "RKnee": 9,
        "RAnkle": 10, "LHip": 11, "LKnee": 12, "LAnkle": 13, "REye": 14,
        "LEye": 15, "REar": 16, "LEar": 17
    }

    # 骨骼连接关系
    POSE_PAIRS = [
        ["Neck", "RShoulder"], ["Neck", "LShoulder"], ["RShoulder", "RElbow"],
        ["RElbow", "RWrist"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"],
        ["Neck", "RHip"], ["RHip", "RKnee"], ["RKnee", "RAnkle"],
        ["Neck", "LHip"], ["LHip", "LKnee"], ["LKnee", "LAnkle"],
        ["Neck", "Nose"], ["Nose", "REye"], ["REye", "REar"],
        ["Nose", "LEye"], ["LEye", "LEar"]
    ]

    def __init__(self, proto_path=None, model_path=None, threshold=0.1):
        """
        初始化姿态估计器
        
        参数:
            proto_path: OpenPose prototxt文件路径（可选，无则用简单方法）
            model_path: OpenPose caffemodel文件路径
            threshold: 关键点置信度阈值
        """
        self.threshold = threshold
        self.net = None

        if proto_path and model_path:
            self.net = cv2.dnn.readNetFromCaffe(proto_path, model_path)

    def estimate_pose(self, image):
        """
        估计图像中的人体姿态
        
        参数:
            image: 输入图像 (BGR)
            
        返回:
            keypoints: 关键点列表 [(x, y, confidence), ...]
            connections: 骨骼连接线列表
        """
        if self.net is None:
            # 无模型时返回空结果
            return [], []

        h, w = image.shape[:2]
        # 预处理：创建blob
        blob = cv2.dnn.blobFromImage(image, 1.0 / 255, (368, 368), (0, 0, 0),
                                      swapRB=False, crop=False)
        self.net.setInput(blob)
        output = self.net.forward()

        # 解析关键点
        keypoints = []
        for i in range(len(self.BODY_PARTS)):
            # 获取热力图
            prob_map = output[0, i, :, :]
            # 找到最大值位置
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(prob_map)
            # 缩放回原图尺寸
            x = int((w * max_loc[0]) / output.shape[3])
            y = int((h * max_loc[1]) / output.shape[2])

            if max_val > self.threshold:
                keypoints.append((x, y, float(max_val)))
            else:
                keypoints.append(None)

        # 构建骨骼连接
        connections = []
        for pair in self.POSE_PAIRS:
            part_from = self.BODY_PARTS[pair[0]]
            part_to = self.BODY_PARTS[pair[1]]

            if keypoints[part_from] and keypoints[part_to]:
                connections.append((keypoints[part_from][:2], keypoints[part_to][:2]))

        return keypoints, connections

    def draw_pose(self, image, keypoints, connections):
        """在图像上绘制骨骼"""
        result = image.copy()

        # 画连接线
        for pt_from, pt_to in connections:
            cv2.line(result, pt_from, pt_to, (0, 255, 0), 2)

        # 画关键点
        for kp in keypoints:
            if kp:
                cv2.circle(result, (kp[0], kp[1]), 4, (0, 0, 255), -1)

        return result

    def get_body_angles(self, keypoints):
        """
        计算关键关节角度（用于动作判断）
        
        返回:
            angles: 字典，包含各关节角度
        """
        angles = {}

        def calc_angle(p1, p2, p3):
            """计算三点构成的角度（p2为顶点）"""
            if not (p1 and p2 and p3):
                return None
            v1 = np.array(p1[:2]) - np.array(p2[:2])
            v2 = np.array(p3[:2]) - np.array(p2[:2])
            cos_val = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
            return np.degrees(np.arccos(np.clip(cos_val, -1, 1)))

        # 右肘角度
        angles['right_elbow'] = calc_angle(
            keypoints[self.BODY_PARTS['RShoulder']],
            keypoints[self.BODY_PARTS['RElbow']],
            keypoints[self.BODY_PARTS['RWrist']]
        )
        # 左肘角度
        angles['left_elbow'] = calc_angle(
            keypoints[self.BODY_PARTS['LShoulder']],
            keypoints[self.BODY_PARTS['LElbow']],
            keypoints[self.BODY_PARTS['LWrist']]
        )
        # 右膝角度
        angles['right_knee'] = calc_angle(
            keypoints[self.BODY_PARTS['RHip']],
            keypoints[self.BODY_PARTS['RKnee']],
            keypoints[self.BODY_PARTS['RAnkle']]
        )
        # 左膝角度
        angles['left_knee'] = calc_angle(
            keypoints[self.BODY_PARTS['LHip']],
            keypoints[self.BODY_PARTS['LKnee']],
            keypoints[self.BODY_PARTS['LAnkle']]
        )

        return angles


# ==================== 手势识别 ====================

class GestureRecognizer:
    """
    基于手指关键点的手势识别
    支持：握拳、张开、指向、点赞、OK等
    """

    def __init__(self):
        self.finger_tips = [4, 8, 12, 16, 20]  # 手指尖端关键点ID
        self.finger_pips = [3, 6, 10, 14, 18]   # 手指中间关节
        self.finger_mcps = [2, 5, 9, 13, 17]     # 手指根部

    def count_fingers(self, hand_landmarks):
        """
        数伸出的手指数量
        
        参数:
            hand_landmarks: 21个手部关键点坐标列表
            
        返回:
            fingers_up: 每根手指是否伸展 [拇指,食指,中指,无名指,小指]
        """
        if hand_landmarks is None or len(hand_landmarks) < 21:
            return [0, 0, 0, 0, 0]

        fingers = []

        # 拇指：比较x坐标（水平伸展）
        if hand_landmarks[self.finger_tips[0]][0] > hand_landmarks[self.finger_pips[0]][0]:
            fingers.append(1)
        else:
            fingers.append(0)

        # 其他四指：比较y坐标（垂直伸展，y轴向下）
        for i in range(1, 5):
            if hand_landmarks[self.finger_tips[i]][1] < hand_landmarks[self.finger_pips[i]][1]:
                fingers.append(1)
            else:
                fingers.append(0)

        return fingers

    def recognize_gesture(self, hand_landmarks):
        """
        识别手势类型
        
        返回:
            gesture: 手势名称字符串
        """
        fingers = self.count_fingers(hand_landmarks)
        total = sum(fingers)

        if total == 0:
            return "握拳 (Fist)"
        elif total == 5:
            return "张开 (Open Palm)"
        elif fingers == [0, 1, 0, 0, 0]:
            return "指向 (Pointing)"
        elif fingers == [0, 1, 1, 0, 0]:
            return "和平/剪刀手 (Peace)"
        elif fingers == [1, 0, 0, 0, 0]:
            return "点赞 (Thumbs Up)"
        elif fingers == [1, 1, 0, 0, 0]:
            return "数字2 (Two)"
        elif fingers == [1, 1, 1, 0, 0]:
            return "数字3 (Three)"
        elif fingers == [0, 1, 1, 1, 1]:
            return "数字4 (Four)"
        elif fingers == [1, 1, 1, 1, 1]:
            return "数字5 (Five)"
        else:
            return f"其他 (Other: {fingers})"


# ==================== 动作时序分类器 ====================

class ActionClassifier:
    """
    基于骨骼关键点序列的动作分类器
    使用滑动窗口 + 规则/模板匹配
    """

    def __init__(self, window_size=30):
        """
        参数:
            window_size: 时序窗口大小（帧数）
        """
        self.window_size = window_size
        self.keypoint_buffer = deque(maxlen=window_size)
        self.action_templates = {}

    def add_template(self, action_name, template_func):
        """
        注册动作模板
        
        参数:
            action_name: 动作名称
            template_func: 判别函数，接收关键点序列，返回bool
        """
        self.action_templates[action_name] = template_func

    def update(self, keypoints):
        """添加新的关键点帧"""
        self.keypoint_buffer.append(keypoints)

    def classify(self):
        """
        基于当前缓冲区识别动作
        
        返回:
            action: 识别到的动作名称，未识别返回None
        """
        if len(self.keypoint_buffer) < self.window_size // 2:
            return None

        for name, func in self.action_templates.items():
            if func(list(self.keypoint_buffer)):
                return name
        return None

    @staticmethod
    def detect_waving(keypoints_sequence):
        """检测挥手动作（左右手腕大幅摆动）"""
        if len(keypoints_sequence) < 10:
            return False
        # 提取右手腕x坐标序列
        wrist_x = []
        for kp in keypoints_sequence:
            if kp and len(kp) > 4 and kp[4]:
                wrist_x.append(kp[4][0])
        if len(wrist_x) < 10:
            return False
        # 计算摆动幅度
        x_range = max(wrist_x) - min(wrist_x)
        return x_range > 100  # 像素阈值

    @staticmethod
    def detect_jumping(keypoints_sequence):
        """检测跳跃动作（身体整体y坐标先降后升）"""
        if len(keypoints_sequence) < 10:
            return False
        hip_y = []
        for kp in keypoints_sequence:
            if kp and len(kp) > 8 and kp[1]:  # Neck关键点
                hip_y.append(kp[1][1])
        if len(hip_y) < 10:
            return False
        # y轴向下增大，跳跃时先减小后增大
        mid = len(hip_y) // 2
        first_half = np.mean(hip_y[:mid])
        second_half = np.mean(hip_y[mid:])
        min_y = min(hip_y)
        max_y = max(hip_y)
        return (first_half - min_y > 30) and (max_y - second_half > 20)

    @staticmethod
    def detect_squatting(keypoints_sequence):
        """检测下蹲动作（膝盖角度持续减小）"""
        if len(keypoints_sequence) < 10:
            return False
        knee_angles = []
        for kp in keypoints_sequence:
            if kp and len(kp) > 13 and kp[8] and kp[9] and kp[10]:
                v1 = np.array(kp[8][:2]) - np.array(kp[9][:2])
                v2 = np.array(kp[10][:2]) - np.array(kp[9][:2])
                cos_val = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
                angle = np.degrees(np.arccos(np.clip(cos_val, -1, 1)))
                knee_angles.append(angle)
        if len(knee_angles) < 10:
            return False
        # 角度从大到小变化
        return knee_angles[0] - min(knee_angles) > 40


# ==================== 简单运动检测 ====================

class MotionDetector:
    """
    基于帧差法的运动检测
    适用于无模型场景下的简单动作检测
    """

    def __init__(self, threshold=25, min_area=500):
        self.threshold = threshold
        self.min_area = min_area
        self.prev_gray = None

    def detect(self, frame):
        """
        检测帧间运动
        
        返回:
            motion_mask: 运动区域二值图
            contours: 运动轮廓列表
            motion_level: 运动程度 (0~1)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        if self.prev_gray is None:
            self.prev_gray = gray
            return None, [], 0.0

        # 帧差
        diff = cv2.absdiff(self.prev_gray, gray)
        _, motion_mask = cv2.threshold(diff, self.threshold, 255, cv2.THRESH_BINARY)

        # 形态学操作去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        motion_mask = cv2.dilate(motion_mask, kernel, iterations=2)
        motion_mask = cv2.erode(motion_mask, kernel, iterations=1)

        # 找轮廓
        contours, _ = cv2.findContours(motion_mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        # 过滤小面积
        contours = [c for c in contours if cv2.contourArea(c) > self.min_area]

        # 计算运动程度
        motion_pixels = cv2.countNonZero(motion_mask)
        total_pixels = motion_mask.shape[0] * motion_mask.shape[1]
        motion_level = motion_pixels / total_pixels

        self.prev_gray = gray
        return motion_mask, contours, motion_level


# ==================== 使用示例 ====================

def example_pose_estimation():
    """姿态估计示例"""
    # 初始化（无模型时使用简单方法）
    pose = PoseEstimator()

    # 读取图像
    image = cv2.imread("test_person.jpg")
    if image is None:
        # 创建测试图像
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(image, "Place test_person.jpg", (100, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # 估计姿态
    keypoints, connections = pose.estimate_pose(image)

    # 绘制结果
    result = pose.draw_pose(image, keypoints, connections)

    # 计算关节角度
    angles = pose.get_body_angles(keypoints)
    print("关节角度:", angles)

    cv2.imshow("Pose", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def example_gesture_recognition():
    """手势识别示例"""
    gesture = GestureRecognizer()

    # 模拟手部关键点（21个点）
    # 这里用None表示需要实际检测器提供
    test_hand = [(100, 200)] * 21
    result = gesture.recognize_gesture(test_hand)
    print(f"识别结果: {result}")


def example_motion_detection():
    """运动检测示例"""
    cap = cv2.VideoCapture(0)
    detector = MotionDetector(threshold=25, min_area=500)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        motion_mask, contours, level = detector.detect(frame)
        if motion_mask is not None:
            # 绘制运动区域
            for c in contours:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.putText(frame, f"Motion: {level:.2%}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow("Motion Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    print("=== 动作识别模块 ===")
    print("1. 姿态估计（OpenPose DNN）")
    print("2. 手势识别（手指关键点）")
    print("3. 动作时序分类（滑动窗口）")
    print("4. 运动检测（帧差法）")
    example_gesture_recognition()
    example_motion_detection()
