"""
手势控制模块 - 手指追踪 + 手势识别 + 命令映射
=================================================
功能：
  1. 手部检测与手指关键点追踪
  2. 手势识别（石头/剪刀/布/数字等）
  3. 手势到命令的映射控制

依赖：opencv-python, numpy
可选：mediapipe (用于高精度关键点检测)
"""

import cv2
import numpy as np
from enum import IntEnum


# ==================== 手势枚举定义 ====================
class Gesture(IntEnum):
    """手势类型枚举"""
    UNKNOWN = -1       # 未知手势
    FIST = 0           # 拳头（石头）
    ONE = 1            # 伸出一根手指
    TWO = 2            # 伸出两根手指（剪刀）
    THREE = 3          # 伸出三根手指
    FOUR = 4           # 伸出四根手指
    FIVE = 5           # 张开手掌（布）
    THUMBS_UP = 6      # 竖大拇指
    OK_SIGN = 7        # OK手势


class GestureController:
    """
    手势控制器
    使用OpenCV进行基于肤色分割的手部检测和手势识别
    """

    def __init__(self, hsv_lower=(0, 30, 60), hsv_upper=(20, 255, 255)):
        """
        初始化手势控制器

        参数:
            hsv_lower: HSV肤色下限
            hsv_upper: HSV肤色上限
        """
        # 肤色HSV范围（可调整适配不同肤色）
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)

        # 命令映射表
        self.command_map = {
            Gesture.FIST: "STOP",          # 拳头 → 停止
            Gesture.ONE: "MOVE_FORWARD",   # 一指 → 前进
            Gesture.TWO: "TURN_LEFT",      # 剪刀 → 左转
            Gesture.THREE: "TURN_RIGHT",   # 三指 → 右转
            Gesture.FOUR: "SPEED_UP",      # 四指 → 加速
            Gesture.FIVE: "START",         # 手掌 → 启动
            Gesture.THUMBS_UP: "CONFIRM",  # 拇指 → 确认
            Gesture.OK_SIGN: "CANCEL",     # OK → 取消
        }

        # 手势稳定缓冲区（防抖）
        self.buffer = []
        self.buffer_size = 5

    def detect_hand(self, frame):
        """
        检测手部区域（基于肤色分割）

        参数:
            frame: BGR输入图像

        返回:
            mask: 手部二值掩码
            contour: 最大手部轮廓
        """
        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 肤色检测
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # 形态学操作去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

        # 高斯模糊平滑
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        hand_contour = None
        if contours:
            # 取最大轮廓作为手部
            hand_contour = max(contours, key=cv2.contourArea)
            # 面积太小则认为无手
            if cv2.contourArea(hand_contour) < 3000:
                hand_contour = None

        return mask, hand_contour

    def count_fingers(self, contour):
        """
        基于凸包和凸缺陷计算伸出的手指数量

        参数:
            contour: 手部轮廓

        返回:
            finger_count: 伸出的手指数量
            defects: 凸缺陷信息
            hull: 凸包
        """
        if contour is None:
            return 0, None, None

        # 计算凸包
        hull = cv2.convexHull(contour, returnPoints=False)
        defects = cv2.convexityDefects(contour, hull)

        if defects is None:
            return 0, None, hull

        # 统计有效的凸缺陷（手指间的凹陷）
        finger_count = 0
        valid_defects = []

        for i in range(defects.shape[0]):
            s, e, f, d = defects[i, 0]
            start = tuple(contour[s][0])
            end = tuple(contour[e][0])
            far = tuple(contour[f][0])

            # 计算三角形三边长度
            a = np.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
            b = np.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
            c = np.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)

            # 余弦定理计算角度
            angle = np.arccos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c + 1e-6))

            # 角度小于90度且深度足够 → 有效手指间凹陷
            if angle < np.pi / 2 and d > 5000:
                finger_count += 1
                valid_defects.append((start, end, far))

        # 每个缺陷对应两根手指，加上最外侧一根
        finger_count = min(finger_count + 1, 5)
        return finger_count, valid_defects, hull

    def recognize_gesture(self, finger_count, contour, defects):
        """
        识别具体手势类型

        参数:
            finger_count: 手指数量
            contour: 手部轮廓
            defects: 凸缺陷

        返回:
            gesture: 手势类型
        """
        if contour is None:
            return Gesture.UNKNOWN

        # 基于手指数量的初步判断
        if finger_count == 0:
            return Gesture.FIST
        elif finger_count == 1:
            # 判断是单指还是竖拇指
            if self._is_thumbs_up(contour):
                return Gesture.THUMBS_UP
            return Gesture.ONE
        elif finger_count == 2:
            # 判断是否是OK手势
            if self._is_ok_sign(contour, defects):
                return Gesture.OK_SIGN
            return Gesture.TWO
        elif finger_count == 3:
            return Gesture.THREE
        elif finger_count == 4:
            return Gesture.FOUR
        elif finger_count >= 5:
            return Gesture.FIVE

        return Gesture.UNKNOWN

    def _is_thumbs_up(self, contour):
        """判断是否是竖大拇指（基于轮廓长宽比和朝向）"""
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = h / (w + 1e-6)
        # 竖拇指通常比较瘦长
        return aspect_ratio > 1.5

    def _is_ok_sign(self, contour, defects):
        """判断是否是OK手势（拇指和食指形成圆圈）"""
        if defects is None or len(defects) < 2:
            return False
        # 简化判断：凸缺陷较少且轮廓近似圆形
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        circularity = 4 * np.pi * area / (perimeter ** 2 + 1e-6)
        return circularity > 0.3

    def get_command(self, gesture):
        """
        将手势映射到控制命令

        参数:
            gesture: 手势类型

        返回:
            command: 控制命令字符串
        """
        return self.command_map.get(gesture, "UNKNOWN")

    def stabilize_gesture(self, gesture):
        """
        手势稳定化（防抖）

        参数:
            gesture: 当前帧识别的手势

        返回:
            stable_gesture: 稳定后的手势
        """
        self.buffer.append(gesture)
        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)

        # 取出现次数最多的手势
        from collections import Counter
        counter = Counter(self.buffer)
        stable_gesture = counter.most_common(1)[0][0]
        return stable_gesture

    def draw_debug(self, frame, contour, hull, defects, gesture, command):
        """
        绘制调试可视化信息

        参数:
            frame: 原始图像
            contour: 手部轮廓
            hull: 凸包
            defects: 凸缺陷
            gesture: 识别的手势
            command: 映射的命令

        返回:
            vis: 可视化图像
        """
        vis = frame.copy()

        if contour is not None:
            # 绘制手部轮廓
            cv2.drawContours(vis, [contour], -1, (0, 255, 0), 2)

            # 绘制凸包
            if hull is not None:
                hull_points = cv2.convexHull(contour)
                cv2.drawContours(vis, [hull_points], -1, (0, 0, 255), 2)

            # 绘制凸缺陷点
            if defects is not None:
                for start, end, far in defects:
                    cv2.circle(vis, far, 5, (255, 0, 0), -1)

        # 显示手势和命令
        gesture_name = Gesture(gesture).name if gesture >= 0 else "UNKNOWN"
        cv2.putText(vis, f"Gesture: {gesture_name}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(vis, f"Command: {command}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return vis

    def process_frame(self, frame):
        """
        处理单帧图像（完整流水线）

        参数:
            frame: BGR输入图像

        返回:
            result: 包含手势信息的字典
        """
        # 手部检测
        mask, contour = self.detect_hand(frame)

        # 手指计数
        finger_count, defects, hull = self.count_fingers(contour)

        # 手势识别
        gesture = self.recognize_gesture(finger_count, contour, defects)

        # 手势稳定化
        stable_gesture = self.stabilize_gesture(gesture)

        # 命令映射
        command = self.get_command(stable_gesture)

        return {
            'gesture': stable_gesture,
            'gesture_name': Gesture(stable_gesture).name if stable_gesture >= 0 else "UNKNOWN",
            'finger_count': finger_count,
            'command': command,
            'contour': contour,
            'hull': hull,
            'defects': defects,
            'mask': mask
        }


# ==================== MediaPipe增强版 ====================
class GestureControllerMediaPipe:
    """
    基于MediaPipe的高精度手势控制器
    需要安装: pip install mediapipe
    """

    def __init__(self):
        """初始化MediaPipe手势检测"""
        try:
            import mediapipe as mp
            self.mp = mp
            self.mp_hands = mp.solutions.hands
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=2,
                min_detection_confidence=0.7,
                min_tracking_confidence=0.5
            )
            self.mp_draw = mp.solutions.drawing_utils
            self.available = True
        except ImportError:
            print("[警告] mediapipe未安装，增强版不可用。pip install mediapipe")
            self.available = False

    def process_frame(self, frame):
        """
        使用MediaPipe处理帧

        参数:
            frame: BGR图像

        返回:
            results: 手部关键点结果
        """
        if not self.available:
            return None

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        hand_data = []
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # 提取21个关键点坐标
                landmarks = []
                for lm in hand_landmarks.landmark:
                    h, w, _ = frame.shape
                    landmarks.append((int(lm.x * w), int(lm.y * h), lm.z))

                # 判断手指是否伸出
                fingers = self._count_fingers(landmarks)
                hand_data.append({
                    'landmarks': landmarks,
                    'fingers': fingers,
                    'count': sum(fingers)
                })

                # 绘制关键点
                self.mp_draw.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        return hand_data

    def _count_fingers(self, landmarks):
        """
        基于关键点判断每根手指是否伸出

        参数:
            landmarks: 21个关键点列表

        返回:
            fingers: [拇指, 食指, 中指, 无名指, 小指] (1=伸直, 0=弯曲)
        """
        fingers = []

        # 拇指：比较指尖和IP关节的x坐标
        if landmarks[4][0] < landmarks[3][0]:  # 右手
            fingers.append(1)
        else:
            fingers.append(0)

        # 其余四指：比较指尖和PIP关节的y坐标
        tip_ids = [8, 12, 16, 20]
        pip_ids = [6, 10, 14, 18]
        for tip, pip in zip(tip_ids, pip_ids):
            fingers.append(1 if landmarks[tip][1] < landmarks[pip][1] else 0)

        return fingers


# ==================== 使用示例 ====================
def demo_camera():
    """摄像头手势识别演示"""
    controller = GestureController()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("手势控制演示 - 按ESC退出")
    print("手势映射:")
    for gesture, command in controller.command_map.items():
        print(f"  {Gesture(gesture).name} → {command}")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 水平翻转（镜像）
        frame = cv2.flip(frame, 1)

        # 处理帧
        result = controller.process_frame(frame)

        # 可视化
        vis = controller.draw_debug(
            frame, result['contour'], result['hull'],
            result['defects'], result['gesture'], result['command']
        )

        cv2.imshow("Gesture Control", vis)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


def demo_image(image_path):
    """
    单张图片手势识别

    参数:
        image_path: 图片路径
    """
    controller = GestureController()
    frame = cv2.imread(image_path)

    if frame is None:
        print(f"无法读取图片: {image_path}")
        return

    result = controller.process_frame(frame)

    print(f"手指数量: {result['finger_count']}")
    print(f"识别手势: {result['gesture_name']}")
    print(f"控制命令: {result['command']}")

    vis = controller.draw_debug(
        frame, result['contour'], result['hull'],
        result['defects'], result['gesture'], result['command']
    )
    cv2.imshow("Result", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo_camera()
