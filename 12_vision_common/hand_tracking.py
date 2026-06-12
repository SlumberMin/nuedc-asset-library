"""
手部追踪模块 - 肤色检测 + 轮廓 + 凸包 + 指尖检测
适用于电赛中基于摄像头的手势识别与手部定位任务。
"""

import cv2
import numpy as np
from threading import Thread, Lock
from collections import deque
import time


class HandTracker:
    """手部追踪器：基于肤色检测、轮廓分析、凸包和指尖检测。"""

    def __init__(
        self,
        src=0,
        width=640,
        height=480,
        skin_lower=np.array([0, 20, 70], dtype=np.uint8),
        skin_upper=np.array([20, 255, 255], dtype=np.uint8),
        min_area=3000,
        max_hands=2,
    ):
        """
        初始化手部追踪器。

        参数:
            src: 摄像头索引或视频路径
            width, height: 画面尺寸
            skin_lower, skin_upper: HSV肤色范围
            min_area: 最小轮廓面积阈值
            max_hands: 最大检测手数
        """
        self.src = src
        self.width = width
        self.height = height
        self.skin_lower = skin_lower
        self.skin_upper = skin_upper
        self.min_area = min_area
        self.max_hands = max_hands

        # 结果存储（线程安全）
        self._lock = Lock()
        self._frame = None
        self._hands = []  # 每只手的检测结果
        self._running = False
        self._fps = 0.0

        # 指尖历史（用于手势平滑）
        self._fingertip_history = deque(maxlen=10)

    def _detect_skin(self, frame):
        """
        肤色检测：在YCrCb和HSV空间中提取肤色区域并合并。

        参数:
            frame: BGR图像
        返回:
            二值掩码
        """
        # YCrCb空间肤色检测
        ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
        lower_ycrcb = np.array([0, 133, 77], dtype=np.uint8)
        upper_ycrcb = np.array([255, 173, 127], dtype=np.uint8)
        mask_ycrcb = cv2.inRange(ycrcb, lower_ycrcb, upper_ycrcb)

        # HSV空间肤色检测
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_hsv = cv2.inRange(hsv, self.skin_lower, self.skin_upper)

        # 合并两种检测结果
        mask = cv2.bitwise_or(mask_ycrcb, mask_hsv)

        # 形态学操作去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        return mask

    def _find_fingertips(self, hull, defects, contour):
        """
        凸包缺陷分析检测指尖位置。

        参数:
            hull: 凸包点集
            defects: 凸包缺陷
            contour: 手部轮廓
        返回:
            fingertips: 指尖坐标列表
        """
        fingertips = []
        if defects is None:
            return fingertips

        for i in range(defects.shape[0]):
            s, e, f, d = defects[i, 0]
            start = tuple(contour[s][0])
            end = tuple(contour[e][0])
            far = tuple(contour[f][0])

            # 计算三角形三边长度
            a = np.linalg.norm(np.array(end) - np.array(start))
            b = np.linalg.norm(np.array(far) - np.array(start))
            c = np.linalg.norm(np.array(end) - np.array(far))

            # 计算角度（使用余弦定理）
            if b * c == 0:
                continue
            angle = np.arccos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c))

            # 角度小于90度且深度大于阈值 → 可能是指尖
            if angle < np.pi / 2 and d > 500:
                fingertips.append(start)

        return fingertips

    def _process_frame(self, frame):
        """
        处理单帧图像，检测手部。

        参数:
            frame: BGR图像
        返回:
            hands: 手部检测结果列表
        """
        # 缩放并翻转
        frame = cv2.resize(frame, (self.width, self.height))
        frame = cv2.flip(frame, 1)

        # 肤色检测
        skin_mask = self._detect_skin(frame)

        # 查找轮廓
        contours, _ = cv2.findContours(
            skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # 按面积排序，取前N个
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        hands = []

        for contour in contours[: self.max_hands]:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            # 凸包与缺陷
            hull = cv2.convexHull(contour, returnPoints=False)
            try:
                defects = cv2.convexityDefects(contour, hull)
            except cv2.error:
                defects = None

            # 指尖检测
            hull_points = cv2.convexHull(contour)
            fingertips = self._find_fingertips(hull_points, defects, contour)

            # 手部中心（轮廓矩）
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

            # 边界矩形
            x, y, w, h = cv2.boundingRect(contour)

            hand_info = {
                "contour": contour,
                "hull": hull_points,
                "defects": defects,
                "fingertips": fingertips,
                "center": (cx, cy),
                "bbox": (x, y, w, h),
                "area": area,
                "num_fingers": len(fingertips),
            }
            hands.append(hand_info)

        return hands, frame, skin_mask

    def _draw_results(self, frame, hands, skin_mask):
        """
        在帧上绘制检测结果。

        参数:
            frame: 原始帧
            hands: 手部检测结果
            skin_mask: 肤色掩码
        返回:
            绘制了标注的帧
        """
        display = frame.copy()

        for hand in hands:
            # 绘制轮廓
            cv2.drawContours(display, [hand["contour"]], -1, (0, 255, 0), 2)
            # 绘制凸包
            cv2.drawContours(display, [hand["hull"]], -1, (0, 0, 255), 2)

            # 绘制指尖
            for tip in hand["fingertips"]:
                cv2.circle(display, tip, 10, (255, 0, 0), -1)
                cv2.putText(
                    display,
                    f"({tip[0]},{tip[1]})",
                    (tip[0] + 10, tip[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 255, 255),
                    1,
                )

            # 绘制中心点
            cv2.circle(display, hand["center"], 8, (0, 255, 255), -1)

            # 绘制边界框
            x, y, w, h = hand["bbox"]
            cv2.rectangle(display, (x, y), (x + w, y + h), (255, 255, 0), 2)

            # 显示手指数量
            cv2.putText(
                display,
                f"Fingers: {hand['num_fingers']}",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        # 显示FPS
        cv2.putText(
            display,
            f"FPS: {self._fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        return display

    def _capture_loop(self):
        """摄像头捕获与处理主循环（在独立线程中运行）。"""
        cap = cv2.VideoCapture(self.src)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not cap.isOpened():
            print("[错误] 无法打开摄像头")
            self._running = False
            return

        fps_counter = 0
        fps_timer = time.time()

        while self._running:
            ret, frame = cap.read()
            if not ret:
                break

            hands, processed_frame, skin_mask = self._process_frame(frame)
            display = self._draw_results(processed_frame, hands, skin_mask)

            # 更新FPS
            fps_counter += 1
            if time.time() - fps_timer >= 1.0:
                self._fps = fps_counter / (time.time() - fps_timer)
                fps_counter = 0
                fps_timer = time.time()

            with self._lock:
                self._frame = display
                self._hands = hands

        cap.release()

    def start(self):
        """启动手部追踪（多线程）。"""
        if self._running:
            return
        self._running = True
        self._thread = Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        print("[信息] 手部追踪已启动")

    def stop(self):
        """停止手部追踪。"""
        self._running = False
        if hasattr(self, "_thread"):
            self._thread.join(timeout=2.0)
        print("[信息] 手部追踪已停止")

    def get_frame(self):
        """获取最新处理帧。"""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_hands(self):
        """获取最新手部检测结果。"""
        with self._lock:
            return list(self._hands)

    def get_fps(self):
        """获取当前FPS。"""
        return self._fps


class GestureDetector:
    """手势识别器：基于指尖数量和手部形状的简单手势分类。"""

    @staticmethod
    def classify(hand_info):
        """
        根据手指数量和手部几何特征分类手势。

        参数:
            hand_info: 单只手的检测结果字典
        返回:
            gesture: 手势名称字符串
        """
        n = hand_info["num_fingers"]
        area = hand_info["area"]
        x, y, w, h = hand_info["bbox"]
        aspect = w / h if h > 0 else 0

        if n <= 1:
            if area < 5000:
                return "拳头(Fist)"
            return "一(One)"
        elif n == 2:
            return "二(Two/Peace)"
        elif n == 3:
            return "三(Three)"
        elif n == 4:
            return "四(Four)"
        elif n >= 5:
            return "五(Open Palm)"
        return "未知(Unknown)"


def main():
    """
    使用示例：实时摄像头手部追踪与手势识别。
    按 'q' 退出。
    """
    tracker = HandTracker(src=0, width=640, height=480)
    tracker.start()

    try:
        while True:
            frame = tracker.get_frame()
            if frame is not None:
                # 显示手势
                hands = tracker.get_hands()
                for i, hand in enumerate(hands):
                    gesture = GestureDetector.classify(hand)
                    cv2.putText(
                        frame,
                        gesture,
                        (10, 60 + i * 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 0, 255),
                        2,
                    )
                cv2.imshow("Hand Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        tracker.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
