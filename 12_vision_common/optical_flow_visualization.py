"""
光流可视化模块 - LK稀疏光流 + 稠密光流 + 运动矢量图
适用于电赛中运动检测、目标跟踪和场景流分析任务。
"""

import cv2
import numpy as np
from threading import Thread, Lock
import time


class OpticalFlowVisualizer:
    """
    光流可视化器。
    支持Lucas-Kanade稀疏光流和Farneback/Gunnar稠密光流。
    """

    def __init__(
        self,
        src=0,
        width=640,
        height=480,
        flow_method="farneback",
        lk_win_size=21,
        lk_max_level=3,
        feature_max_corners=200,
        feature_quality=0.01,
        feature_min_dist=10,
    ):
        """
        初始化光流可视化器。

        参数:
            src: 摄像头索引或视频路径
            width, height: 画面尺寸
            flow_method: 光流方法 ('lk'=稀疏, 'farneback'=稠密, 'rlof'=鲁棒)
            lk_win_size: LK光流窗口大小
            lk_max_level: LK金字塔层数
            feature_max_corners: 最大特征点数
            feature_quality: 特征点质量阈值
            feature_min_dist: 特征点最小间距
        """
        self.src = src
        self.width = width
        self.height = height
        self.flow_method = flow_method

        # LK稀疏光流参数
        self.lk_params = dict(
            winSize=(lk_win_size, lk_win_size),
            maxLevel=lk_max_level,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )

        # 特征点检测参数
        self.feature_params = dict(
            maxCorners=feature_max_corners,
            qualityLevel=feature_quality,
            minDistance=feature_min_dist,
            blockSize=7,
        )

        # Farneback稠密光流参数
        self.farneback_params = dict(
            pyr_scale=0.5,
            levels=3,
            winsize=15,
            iterations=3,
            poly_n=5,
            poly_sigma=1.2,
            flags=0,
        )

        # 结果存储
        self._lock = Lock()
        self._frame = None
        self._flow_result = None
        self._running = False
        self._fps = 0.0

        # 前一帧（用于光流计算）
        self._prev_gray = None

        # 稀疏光流轨迹历史
        self._tracks = []
        self._track_len = 30  # 轨迹最大长度
        self._detect_interval = 5  # 每隔N帧重新检测特征点

        # 运动统计
        self._motion_magnitude_avg = 0.0
        self._motion_direction_hist = np.zeros(36)  # 10度一档

    def _detect_features(self, gray, mask=None):
        """
        检测Shi-Tomasi角点作为光流追踪点。

        参数:
            gray: 灰度图像
            mask: 检测掩码
        返回:
            points: 特征点坐标数组
        """
        points = cv2.goodFeaturesToTrack(gray, mask=mask, **self.feature_params)
        return points

    def _compute_sparse_flow(self, prev_gray, curr_gray, prev_points):
        """
        计算Lucas-Kanade稀疏光流。

        参数:
            prev_gray, curr_gray: 前后帧灰度图
            prev_points: 前一帧特征点
        返回:
            good_new: 成功追踪的当前点
            good_old: 对应的前一帧点
            status: 追踪状态
        """
        if prev_points is None or len(prev_points) == 0:
            return [], [], None

        next_points, status, error = cv2.calcOpticalFlowPyrLK(
            prev_gray, curr_gray, prev_points, None, **self.lk_params
        )

        if next_points is None or status is None:
            return [], [], status

        # 筛选成功追踪的点
        good_new = next_points[status.ravel() == 1]
        good_old = prev_points[status.ravel() == 1]

        return good_new, good_old, status

    def _compute_dense_flow(self, prev_gray, curr_gray):
        """
        计算Farneback稠密光流。

        参数:
            prev_gray, curr_gray: 前后帧灰度图
        返回:
            flow: 光流场 (H, W, 2)，每个像素有(dx, dy)
        """
        flow = cv2.calcOpticalFlowFarneback(
            prev_gray, curr_gray, None, **self.farneback_params
        )
        return flow

    def _flow_to_hsv(self, flow):
        """
        将光流场转换为HSV色彩编码图像。

        颜色编码：H=方向, S=饱和度, V=亮度(速度大小)

        参数:
            flow: 光流场 (H, W, 2)
        返回:
            hsv_image: HSV编码的光流可视化图像
        """
        h, w = flow.shape[:2]
        hsv = np.zeros((h, w, 3), dtype=np.uint8)
        hsv[..., 1] = 255  # 饱和度设为最大

        # 计算极坐标（幅度和角度）
        magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])

        # 色调 = 方向（0-180映射到OpenCV的HSV）
        hsv[..., 0] = (angle * 180 / np.pi / 2).astype(np.uint8)
        # 亮度 = 速度大小（归一化到0-255）
        hsv[..., 2] = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX).astype(
            np.uint8
        )

        # HSV转BGR显示
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        return bgr, magnitude, angle

    def _compute_motion_statistics(self, flow):
        """
        计算运动统计信息。

        参数:
            flow: 光流场
        返回:
            stats: 运动统计字典
        """
        magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])

        # 平均运动幅度
        avg_mag = np.mean(magnitude)
        max_mag = np.max(magnitude)

        # 运动方向直方图（10度一档，共36档）
        angle_deg = (angle * 180 / np.pi).ravel()
        hist, _ = np.histogram(angle_deg, bins=36, range=(0, 360))
        hist = hist.astype(np.float64) / hist.sum() if hist.sum() > 0 else hist

        # 主运动方向
        dominant_dir = np.argmax(hist) * 10  # 度

        # 运动像素比例（幅度>阈值的像素占比）
        motion_pixels = np.sum(magnitude > 1.0) / magnitude.size

        stats = {
            "avg_magnitude": avg_mag,
            "max_magnitude": max_mag,
            "dominant_direction": dominant_dir,
            "direction_histogram": hist,
            "motion_pixel_ratio": motion_pixels,
        }
        return stats

    def _draw_sparse_flow(self, frame, tracks):
        """
        绘制稀疏光流轨迹和当前点。

        参数:
            frame: 原始帧
            tracks: 轨迹列表
        返回:
            display: 绘制了轨迹的帧
        """
        display = frame.copy()

        # 绘制轨迹线
        for track in tracks:
            if len(track) < 2:
                continue
            pts = np.int32(track)
            # 渐变颜色（越新的点越亮）
            for i in range(1, len(pts)):
                alpha = i / len(pts)
                color = (
                    int(255 * (1 - alpha)),
                    int(255 * alpha),
                    0,
                )  # 从红到绿
                cv2.line(display, tuple(pts[i - 1]), tuple(pts[i]), color, 2)

            # 当前点（绿色圆点）
            cv2.circle(display, tuple(pts[-1]), 5, (0, 255, 0), -1)

        return display

    def _draw_flow_arrows(self, frame, flow, step=20):
        """
        在帧上绘制光流矢量箭头。

        参数:
            frame: 原始帧
            flow: 稠密光流场
            step: 箭头采样步长
        返回:
            display: 绘制了箭头的帧
        """
        display = frame.copy()
        h, w = flow.shape[:2]

        for y in range(step // 2, h, step):
            for x in range(step // 2, w, step):
                dx, dy = flow[y, x]
                # 只绘制显著运动
                if np.sqrt(dx * dx + dy * dy) > 2.0:
                    cv2.arrowedLine(
                        display,
                        (x, y),
                        (int(x + dx * 3), int(y + dy * 3)),
                        (0, 255, 0),
                        1,
                        tipLength=0.3,
                    )

        return display

    def _process_frame(self, frame, frame_count):
        """
        处理单帧光流计算。

        参数:
            frame: BGR图像
            frame_count: 帧计数
        返回:
            结果字典
        """
        frame = cv2.resize(frame, (self.width, self.height))
        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        result = {
            "frame": frame,
            "sparse_display": None,
            "dense_display": None,
            "arrow_display": None,
            "flow_hsv": None,
            "stats": None,
        }

        if self._prev_gray is None:
            self._prev_gray = gray
            # 初始特征点
            if self.flow_method == "lk":
                pts = self._detect_features(gray)
                if pts is not None:
                    self._tracks = [[p[0]] for p in pts]
            return result

        if self.flow_method == "lk":
            # ===== 稀疏光流 =====
            # 定期重新检测特征点
            if frame_count % self._detect_interval == 0:
                mask = np.zeros_like(gray)
                # 在已有轨迹附近排除
                for track in self._tracks:
                    if len(track) > 0:
                        cv2.circle(mask, tuple(np.int32(track[-1])), 7, 255, -1)
                new_pts = self._detect_features(gray, mask=~mask)
                if new_pts is not None:
                    for p in new_pts:
                        self._tracks.append([p[0]])

            # 追踪
            if self._tracks:
                prev_pts = np.float32([t[-1] for t in self._tracks]).reshape(-1, 1, 2)
                good_new, good_old, status = self._compute_sparse_flow(
                    self._prev_gray, gray, prev_pts
                )

                new_tracks = []
                for i, track in enumerate(self._tracks):
                    if i < len(good_new):
                        track.append(good_new[i].ravel())
                        # 限制轨迹长度
                        if len(track) > self._track_len:
                            track.pop(0)
                        new_tracks.append(track)
                self._tracks = new_tracks

                # 稀疏光流可视化
                result["sparse_display"] = self._draw_sparse_flow(
                    frame, self._tracks
                )

        elif self.flow_method == "farneback":
            # ===== 稠密光流 =====
            flow = self._compute_dense_flow(self._prev_gray, gray)

            # HSV色彩编码
            flow_hsv, magnitude, angle = self._flow_to_hsv(flow)
            result["flow_hsv"] = flow_hsv

            # 箭头可视化
            result["arrow_display"] = self._draw_flow_arrows(frame, flow, step=25)

            # 运动统计
            stats = self._compute_motion_statistics(flow)
            result["stats"] = stats
            self._motion_magnitude_avg = stats["avg_magnitude"]

            # 叠加显示（半透明HSV + 原图）
            dense_overlay = cv2.addWeighted(frame, 0.6, flow_hsv, 0.4, 0)
            result["dense_display"] = dense_overlay

        self._prev_gray = gray
        return result

    def _draw_info(self, display, result):
        """在显示帧上绘制信息。"""
        if result.get("stats"):
            stats = result["stats"]
            cv2.putText(
                display,
                f"Avg Motion: {stats['avg_magnitude']:.2f} px/frame",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                display,
                f"Motion Pixels: {stats['motion_pixel_ratio']*100:.1f}%",
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                display,
                f"Dominant Dir: {stats['dominant_direction']}deg",
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        cv2.putText(
            display,
            f"FPS: {self._fps:.1f}",
            (10, display.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )

    def _capture_loop(self):
        """光流计算主循环。"""
        cap = cv2.VideoCapture(self.src)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not cap.isOpened():
            print("[错误] 无法打开摄像头")
            self._running = False
            return

        fps_counter = 0
        fps_timer = time.time()
        frame_count = 0

        while self._running:
            ret, frame = cap.read()
            if not ret:
                break

            result = self._process_frame(frame, frame_count)
            frame_count += 1

            fps_counter += 1
            if time.time() - fps_timer >= 1.0:
                self._fps = fps_counter / (time.time() - fps_timer)
                fps_counter = 0
                fps_timer = time.time()

            with self._lock:
                self._frame = result["frame"]
                self._flow_result = result

        cap.release()

    def start(self):
        """启动光流计算。"""
        if self._running:
            return
        self._running = True
        self._thread = Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        print(f"[信息] 光流可视化已启动 (方法: {self.flow_method})")

    def stop(self):
        """停止。"""
        self._running = False
        if hasattr(self, "_thread"):
            self._thread.join(timeout=2.0)
        print("[信息] 光流可视化已停止")

    def get_frame(self):
        """获取最新原始帧。"""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_flow_result(self):
        """获取最新光流结果。"""
        with self._lock:
            return self._flow_result

    def get_visualization(self):
        """获取最佳可视化帧。"""
        with self._lock:
            if self._flow_result is None:
                return None
            r = self._flow_result
            # 根据方法返回对应可视化
            if self.flow_method == "lk":
                return r.get("sparse_display")
            else:
                return r.get("dense_display") or r.get("arrow_display")


def compute_optical_flow_video(video_path, output_path=None, method="farneback"):
    """
    离线视频光流分析工具函数。

    参数:
        video_path: 输入视频路径
        output_path: 输出视频路径（可选）
        method: 光流方法
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[错误] 无法打开视频: {video_path}")
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    writer = None
    if output_path:
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    visualizer = OpticalFlowVisualizer(
        src=video_path, width=width, height=height, flow_method=method
    )

    prev_gray = None
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if prev_gray is not None:
            if method == "farneback":
                flow = cv2.calcOpticalFlowFarneback(
                    prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0
                )
                flow_bgr, _, _ = visualizer._flow_to_hsv(flow)
                overlay = cv2.addWeighted(frame, 0.6, flow_bgr, 0.4, 0)
            else:
                overlay = frame

            if writer:
                writer.write(overlay)

        prev_gray = gray
        frame_count += 1

        if frame_count % 100 == 0:
            print(f"  已处理 {frame_count} 帧")

    cap.release()
    if writer:
        writer.release()
    print(f"[信息] 处理完成，共 {frame_count} 帧")


def main():
    """
    使用示例：实时稠密光流可视化。
    按 'q' 退出，'m' 切换显示模式。
    """
    visualizer = OpticalFlowVisualizer(
        src=0, width=640, height=480, flow_method="farneback"
    )
    visualizer.start()

    show_mode = 0  # 0=叠加, 1=HSV, 2=箭头

    try:
        while True:
            result = visualizer.get_flow_result()
            if result is not None:
                if show_mode == 0:
                    display = result.get("dense_display")
                elif show_mode == 1:
                    display = result.get("flow_hsv")
                else:
                    display = result.get("arrow_display")

                if display is not None:
                    # 添加统计信息
                    visualizer._draw_info(display, result)
                    cv2.imshow("Optical Flow", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("m"):
                show_mode = (show_mode + 1) % 3
                modes = ["Overlay", "HSV Color", "Arrow Vector"]
                print(f"切换显示模式: {modes[show_mode]}")
    finally:
        visualizer.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
