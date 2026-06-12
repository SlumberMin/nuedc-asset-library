"""
眼动追踪模块 - 瞳孔检测 + 注视点估计 + 眨眼识别
=================================================
功能：
  1. 人脸检测与眼部区域定位
  2. 瞳孔中心检测（基于阈值分割+质心计算）
  3. 注视方向估计
  4. 眨眼检测（EAR眼部纵横比）

依赖：opencv-python, numpy
"""

import cv2
import numpy as np
from collections import deque


class EyeTracker:
    """
    眼动追踪器
    使用OpenCV Haar级联分类器进行人脸/眼部检测
    """

    def __init__(self, face_cascade_path=None, eye_cascade_path=None):
        """
        初始化眼动追踪器

        参数:
            face_cascade_path: 人脸级联分类器路径（None则使用内置）
            eye_cascade_path: 眼部级联分类器路径（None则使用内置）
        """
        # 加载Haar级联分类器
        if face_cascade_path is None:
            face_cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        if eye_cascade_path is None:
            eye_cascade_path = cv2.data.haarcascades + 'haarcascade_eye.xml'

        self.face_cascade = cv2.CascadeClassifier(face_cascade_path)
        self.eye_cascade = cv2.CascadeClassifier(eye_cascade_path)

        # 瞳孔检测参数
        self.pupil_threshold = 30  # 瞳孔暗区阈值

        # 眨眼检测参数
        self.ear_threshold = 0.2   # EAR阈值（低于此值判定为眨眼）
        self.blink_counter = 0
        self.total_blinks = 0
        self.blink_history = deque(maxlen=100)

        # 注视点平滑缓冲
        self.gaze_buffer_x = deque(maxlen=10)
        self.gaze_buffer_y = deque(maxlen=10)

    def detect_face_and_eyes(self, gray):
        """
        检测人脸和眼部区域

        参数:
            gray: 灰度图像

        返回:
            faces: 人脸矩形列表
            eyes_per_face: 每张脸上的眼睛列表
        """
        # 人脸检测
        faces = self.face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(100, 100))

        eyes_per_face = []
        for (fx, fy, fw, fh) in faces:
            roi_gray = gray[fy:fy + fh, fx:fx + fw]
            eyes = self.eye_cascade.detectMultiScale(
                roi_gray, scaleFactor=1.1, minNeighbors=10, minSize=(20, 20))
            # 转换为原图坐标
            eyes_abs = [(fx + ex, fy + ey, ew, eh) for (ex, ey, ew, eh) in eyes]
            eyes_per_face.append(eyes_abs)

        return faces, eyes_per_face

    def detect_pupil(self, eye_roi):
        """
        检测瞳孔中心位置

        参数:
            eye_roi: 眼部灰度图像区域

        返回:
            center: (cx, cy) 瞳孔中心坐标（相对于眼部区域）
            radius: 瞳孔半径
        """
        if eye_roi is None or eye_roi.size == 0:
            return None, 0

        # 高斯模糊降噪
        blurred = cv2.GaussianBlur(eye_roi, (7, 7), 0)

        # 自适应阈值分割暗区（瞳孔）
        _, thresh = cv2.threshold(blurred, self.pupil_threshold, 255, cv2.THRESH_BINARY_INV)

        # 形态学操作清理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            # 降阈值重试
            _, thresh = cv2.threshold(blurred, self.pupil_threshold + 10, 255, cv2.THRESH_BINARY_INV)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, 0

        # 取最大轮廓作为瞳孔
        pupil_contour = max(contours, key=cv2.contourArea)

        # 计算质心
        M = cv2.moments(pupil_contour)
        if M["m00"] == 0:
            return None, 0

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # 估算半径
        area = cv2.contourArea(pupil_contour)
        radius = int(np.sqrt(area / np.pi))

        return (cx, cy), radius

    def estimate_gaze_direction(self, pupil_center, eye_size):
        """
        估计注视方向

        参数:
            pupil_center: 瞳孔中心 (cx, cy)
            eye_size: 眼部区域大小 (w, h)

        返回:
            direction: 注视方向字符串
            normalized: 归一化坐标 (nx, ny) 范围[-1, 1]
        """
        if pupil_center is None:
            return "UNKNOWN", (0, 0)

        cx, cy = pupil_center
        ew, eh = eye_size

        # 归一化到[-1, 1]
        nx = (cx / ew - 0.5) * 2
        ny = (cy / eh - 0.5) * 2

        # 平滑
        self.gaze_buffer_x.append(nx)
        self.gaze_buffer_y.append(ny)
        smooth_x = np.mean(self.gaze_buffer_x)
        smooth_y = np.mean(self.gaze_buffer_y)

        # 判断方向
        threshold = 0.3
        if abs(smooth_x) < threshold and abs(smooth_y) < threshold:
            direction = "CENTER"
        elif smooth_x < -threshold:
            direction = "LEFT"
        elif smooth_x > threshold:
            direction = "RIGHT"
        elif smooth_y < -threshold:
            direction = "UP"
        elif smooth_y > threshold:
            direction = "DOWN"
        else:
            direction = "CENTER"

        return direction, (smooth_x, smooth_y)

    def compute_ear(self, eye_points):
        """
        计算眼部纵横比 (Eye Aspect Ratio)

        参数:
            eye_points: 眼部6个关键点坐标 [(x1,y1), ..., (x6,y6)]
                        排列：左角, 上1, 上2, 右角, 下2, 下1

        返回:
            ear: 眼部纵横比值
        """
        if len(eye_points) < 6:
            return 0.3  # 默认值

        # 计算垂直距离
        v1 = np.linalg.norm(np.array(eye_points[1]) - np.array(eye_points[5]))
        v2 = np.linalg.norm(np.array(eye_points[2]) - np.array(eye_points[4]))

        # 计算水平距离
        h = np.linalg.norm(np.array(eye_points[0]) - np.array(eye_points[3]))

        # EAR公式
        ear = (v1 + v2) / (2.0 * h + 1e-6)
        return ear

    def detect_blink_simple(self, eye_roi):
        """
        基于眼部区域白色像素比例的简易眨眼检测

        参数:
            eye_roi: 眼部灰度图像

        返回:
            is_blinking: 是否在眨眼
        """
        if eye_roi is None or eye_roi.size == 0:
            return True

        h, w = eye_roi.shape
        total_pixels = h * w

        # 二值化
        _, thresh = cv2.threshold(eye_roi, 70, 255, cv2.THRESH_BINARY)

        # 计算白色像素比例
        white_ratio = cv2.countNonZero(thresh) / total_pixels

        # 眼睛闭合时白色像素很少
        is_blinking = white_ratio < 0.25
        return is_blinking

    def detect_blinks_ear(self, is_blinking_frame):
        """
        基于EAR序列的眨眼检测（防抖）

        参数:
            is_blinking_frame: 当前帧是否闭眼

        返回:
            blink_detected: 是否完成一次眨眼
            total_blinks: 总眨眼次数
        """
        self.blink_history.append(is_blinking_frame)

        if is_blinking_frame:
            self.blink_counter += 1
        else:
            if self.blink_counter >= 2:  # 至少连续2帧闭眼才算眨眼
                self.total_blinks += 1
            self.blink_counter = 0

        blink_detected = self.blink_counter == 2  # 刚完成眨眼
        return blink_detected, self.total_blinks

    def process_frame(self, frame):
        """
        处理单帧图像

        参数:
            frame: BGR输入图像

        返回:
            result: 追踪结果字典
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 检测人脸和眼睛
        faces, eyes_per_face = self.detect_face_and_eyes(gray)

        all_eye_data = []
        blink_detected = False

        for face_idx, (face, eyes) in enumerate(zip(faces, eyes_per_face)):
            for eye_idx, (ex, ey, ew, eh) in enumerate(eyes[:2]):  # 最多处理2只眼
                eye_roi = gray[ey:ey + eh, ex:ex + ew]

                # 瞳孔检测
                pupil_center, pupil_radius = self.detect_pupil(eye_roi)

                # 注视方向估计
                gaze_dir, gaze_norm = self.estimate_gaze_direction(pupil_center, (ew, eh))

                # 眨眼检测
                is_blinking = self.detect_blink_simple(eye_roi)
                bd, total = self.detect_blinks_ear(is_blinking)
                if bd:
                    blink_detected = True

                all_eye_data.append({
                    'face_idx': face_idx,
                    'eye_idx': eye_idx,
                    'bbox': (ex, ey, ew, eh),
                    'pupil_center': (ex + pupil_center[0], ey + pupil_center[1]) if pupil_center else None,
                    'pupil_radius': pupil_radius,
                    'gaze_direction': gaze_dir,
                    'gaze_normalized': gaze_norm,
                    'is_blinking': is_blinking,
                })

        return {
            'faces': faces,
            'eyes': all_eye_data,
            'blink_detected': blink_detected,
            'total_blinks': self.total_blinks,
            'num_faces': len(faces),
        }

    def draw_debug(self, frame, result):
        """
        绘制调试可视化

        参数:
            frame: 原始图像
            result: process_frame返回的结果

        返回:
            vis: 可视化图像
        """
        vis = frame.copy()

        # 绘制人脸框
        for (fx, fy, fw, fh) in result['faces']:
            cv2.rectangle(vis, (fx, fy), (fx + fw, fy + fh), (255, 0, 0), 2)

        # 绘制眼部信息
        for eye in result['eyes']:
            ex, ey, ew, eh = eye['bbox']

            # 眼部框
            color = (0, 0, 255) if eye['is_blinking'] else (0, 255, 0)
            cv2.rectangle(vis, (ex, ey), (ex + ew, ey + eh), color, 2)

            # 瞳孔中心
            if eye['pupil_center']:
                cv2.circle(vis, eye['pupil_center'], eye['pupil_radius'], (0, 255, 255), 2)
                cv2.circle(vis, eye['pupil_center'], 2, (0, 0, 255), -1)

            # 注视方向箭头
            if eye['pupil_center'] and eye['gaze_direction'] != "UNKNOWN":
                gx, gy = eye['gaze_normalized']
                px, py = eye['pupil_center']
                end_x = int(px + gx * 50)
                end_y = int(py + gy * 50)
                cv2.arrowedLine(vis, (px, py), (end_x, end_y), (255, 255, 0), 2)

        # 显示状态信息
        cv2.putText(vis, f"Blinks: {result['total_blinks']}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        if result['blink_detected']:
            cv2.putText(vis, "BLINK!", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 显示注视方向
        if result['eyes']:
            gaze = result['eyes'][0]['gaze_direction']
            cv2.putText(vis, f"Gaze: {gaze}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        return vis


# ==================== 使用示例 ====================
def demo_camera():
    """摄像头眼动追踪演示"""
    tracker = EyeTracker()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("眼动追踪演示 - 按ESC退出")
    print("功能：瞳孔检测 | 注视方向 | 眨眼计数")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        result = tracker.process_frame(frame)
        vis = tracker.draw_debug(frame, result)

        cv2.imshow("Eye Tracking", vis)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


def demo_image(image_path):
    """单张图片眼动追踪"""
    tracker = EyeTracker()
    frame = cv2.imread(image_path)

    if frame is None:
        print(f"无法读取图片: {image_path}")
        return

    result = tracker.process_frame(frame)

    print(f"检测到人脸: {result['num_faces']}")
    print(f"检测到眼睛: {len(result['eyes'])}")
    for eye in result['eyes']:
        print(f"  眼{eye['eye_idx']}: 注视{eye['gaze_direction']}, "
              f"眨眼={eye['is_blinking']}")

    vis = tracker.draw_debug(frame, result)
    cv2.imshow("Result", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo_camera()
