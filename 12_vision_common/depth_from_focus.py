"""
景深估计模块 - 多焦点图像 + 清晰度图 + 深度重建
适用于电赛中景深测量、自动对焦、3D场景分析等任务。
"""

import cv2
import numpy as np
from threading import Thread, Lock
import time


class DepthFromFocus:
    """
    基于聚焦法的景深估计器。
    通过分析多张不同对焦距离图像的局部清晰度来重建深度图。
    """

    def __init__(
        self,
        focus_distances=None,
        sharpness_method="laplacian",
        block_size=16,
        smooth_sigma=3.0,
    ):
        """
        初始化景深估计器。

        参数:
            focus_distances: 各焦距图像对应的归一化焦距值列表 (0~1)
            sharpness_method: 清晰度度量方法 ('laplacian', 'variance', 'gradient', 'wavelet')
            block_size: 分块大小
            smooth_sigma: 高斯平滑sigma值
        """
        self.focus_distances = focus_distances
        self.sharpness_method = sharpness_method
        self.block_size = block_size
        self.smooth_sigma = smooth_sigma

        self._lock = Lock()
        self._depth_map = None
        self._sharpness_stack = None
        self._best_focus_map = None
        self._confidence_map = None

    def compute_sharpness_laplacian(self, gray):
        """
        拉普拉斯方差清晰度度量。

        参数:
            gray: 灰度图像
        返回:
            sharpness_map: 局部清晰度图
        """
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        sharpness_map = cv2.GaussianBlur(
            np.abs(laplacian), (self.block_size + 1, self.block_size + 1), 0
        )
        return sharpness_map

    def compute_sharpness_variance(self, gray):
        """
        局部方差清晰度度量。

        参数:
            gray: 灰度图像
        返回:
            sharpness_map: 局部方差图
        """
        mean = cv2.blur(gray.astype(np.float64), (self.block_size, self.block_size))
        sq_mean = cv2.blur(
            gray.astype(np.float64) ** 2, (self.block_size, self.block_size)
        )
        variance = sq_mean - mean ** 2
        variance = np.maximum(variance, 0)  # 数值稳定性
        return variance

    def compute_sharpness_gradient(self, gray):
        """
        梯度幅度清晰度度量。

        参数:
            gray: 灰度图像
        返回:
            sharpness_map: 梯度幅度图
        """
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        gradient_mag = np.sqrt(gx ** 2 + gy ** 2)
        smooth = cv2.GaussianBlur(
            gradient_mag, (self.block_size + 1, self.block_size + 1), 0
        )
        return smooth

    def compute_sharpness_wavelet(self, gray):
        """
        小波变换清晰度度量（使用Haar小波近似）。

        参数:
            gray: 灰度图像
        返回:
            sharpness_map: 高频能量图
        """
        img = gray.astype(np.float64)
        h, w = img.shape

        # 确保尺寸为偶数
        h = h - h % 2
        w = w - w % 2
        img = img[:h, :w]

        # 一级Haar小波分解（简化版）
        # 低通 + 高通
        low_pass = (img[0::2, :] + img[1::2, :]) / 2
        high_pass = np.abs(img[0::2, :] - img[1::2, :]) / 2

        # 水平方向
        ll = (low_pass[:, 0::2] + low_pass[:, 1::2]) / 2
        lh = np.abs(low_pass[:, 0::2] - low_pass[:, 1::2]) / 2
        hl = np.abs(high_pass[:, 0::2] + high_pass[:, 1::2]) / 2  # 修正
        hh = np.abs(high_pass[:, 0::2] - high_pass[:, 1::2]) / 2

        # 高频能量
        high_freq_energy = lh ** 2 + hl ** 2 + hh ** 2

        # 上采样回原尺寸
        sharpness_map = cv2.resize(high_freq_energy, (gray.shape[1], gray.shape[0]))

        return sharpness_map

    def _compute_sharpness(self, gray):
        """根据配置选择清晰度度量方法。"""
        if self.sharpness_method == "laplacian":
            return self.compute_sharpness_laplacian(gray)
        elif self.sharpness_method == "variance":
            return self.compute_sharpness_variance(gray)
        elif self.sharpness_method == "gradient":
            return self.compute_sharpness_gradient(gray)
        elif self.sharpness_method == "wavelet":
            return self.compute_sharpness_wavelet(gray)
        else:
            return self.compute_sharpness_laplacian(gray)

    def estimate_depth(self, images):
        """
        从多焦点图像序列估计深度图。

        原理：对每张图像计算局部清晰度，清晰度最高的焦距即为该像素的深度。

        参数:
            images: 多焦点图像列表（BGR格式，按焦距递增顺序）
        返回:
            depth_map: 归一化深度图 (0~255, uint8)
            sharpness_stack: 各图像清晰度堆栈 (N, H, W)
            confidence_map: 深度置信度图
        """
        if len(images) < 2:
            raise ValueError("至少需要2张不同焦距的图像")

        n_images = len(images)
        h, w = images[0].shape[:2]

        # 焦距值
        if self.focus_distances is None:
            self.focus_distances = np.linspace(0, 1, n_images)
        focus_arr = np.array(self.focus_distances, dtype=np.float64)

        # 计算每张图像的清晰度图
        sharpness_stack = np.zeros((n_images, h, w), dtype=np.float64)

        for i, img in enumerate(images):
            # 确保尺寸一致
            img_resized = cv2.resize(img, (w, h))
            gray = cv2.cvtColor(img_resized, cv2.COLOR_BGR2GRAY)
            sharpness_stack[i] = self._compute_sharpness(gray)

        # 对清晰度图进行高斯平滑
        for i in range(n_images):
            sharpness_stack[i] = cv2.GaussianBlur(
                sharpness_stack[i],
                (0, 0),
                self.smooth_sigma,
            )

        # 找到每像素最大清晰度对应的焦距索引
        best_focus_idx = np.argmax(sharpness_stack, axis=0)

        # 深度 = 对应焦距值
        depth_map = focus_arr[best_focus_idx]

        # 置信度 = 最大清晰度与次大清晰度的比值
        sorted_sharpness = np.sort(sharpness_stack, axis=0)
        max_sharp = sorted_sharpness[-1]
        second_sharp = sorted_sharpness[-2]

        # 避免除零
        confidence = np.where(
            second_sharp > 0, max_sharp / (second_sharp + 1e-10), 1.0
        )
        confidence = np.clip(confidence, 0, 10)
        confidence = confidence / confidence.max()  # 归一化

        # 归一化深度图到0-255
        depth_normalized = (depth_map * 255).astype(np.uint8)

        # 中值滤波去噪
        depth_normalized = cv2.medianBlur(depth_normalized, 5)

        with self._lock:
            self._depth_map = depth_normalized
            self._sharpness_stack = sharpness_stack
            self._best_focus_map = best_focus_idx
            self._confidence_map = confidence

        return depth_normalized, sharpness_stack, confidence

    def get_all_in_focus(self, images):
        """
        生成全焦图像（每个像素取最清晰的焦距）。

        参数:
            images: 多焦点图像列表
        返回:
            all_in_focus: 全焦图像
        """
        if self._best_focus_map is None:
            self.estimate_depth(images)

        h, w = images[0].shape[:2]
        all_in_focus = np.zeros((h, w, 3), dtype=np.uint8)

        with self._lock:
            best_idx = self._best_focus_map

        for i, img in enumerate(images):
            img_resized = cv2.resize(img, (w, h))
            mask = (best_idx == i).astype(np.uint8)
            for c in range(3):
                channel = img_resized[:, :, c]
                all_in_focus[:, :, c] = np.where(mask, channel, all_in_focus[:, :, c])

        return all_in_focus

    def visualize_depth(self, colormap=cv2.COLORMAP_JET):
        """
        可视化深度图。

        参数:
            colormap: OpenCV颜色映射
        返回:
            depth_color: 彩色深度图
        """
        with self._lock:
            if self._depth_map is None:
                return None
            depth = self._depth_map.copy()

        depth_color = cv2.applyColorMap(depth, colormap)
        return depth_color

    def visualize_confidence(self):
        """可视化置信度图。"""
        with self._lock:
            if self._confidence_map is None:
                return None
            conf = self._confidence_map.copy()

        conf_uint8 = (conf * 255).astype(np.uint8)
        conf_color = cv2.applyColorMap(conf_uint8, cv2.COLORMAP_VIRIDIS)
        return conf_color

    def visualize_sharpness_comparison(self, images, indices=None):
        """
        可视化多张图像的清晰度对比。

        参数:
            images: 原始图像列表
            indices: 要显示的图像索引列表（默认全部）
        返回:
            comparison: 拼接的对比图
        """
        if self._sharpness_stack is None:
            self.estimate_depth(images)

        if indices is None:
            indices = range(len(images))

        displays = []
        with self._lock:
            for i in indices:
                if i < self._sharpness_stack.shape[0]:
                    sharp = self._sharpness_stack[i]
                    sharp_norm = cv2.normalize(
                        sharp, None, 0, 255, cv2.NORM_MINMAX
                    ).astype(np.uint8)
                    sharp_color = cv2.applyColorMap(sharp_norm, cv2.COLORMAP_HOT)
                    # 缩放到统一尺寸
                    sharp_color = cv2.resize(sharp_color, (320, 240))
                    # 添加焦距标签
                    if self.focus_distances is not None and i < len(
                        self.focus_distances
                    ):
                        label = f"Focus: {self.focus_distances[i]:.2f}"
                    else:
                        label = f"Frame {i}"
                    cv2.putText(
                        sharp_color,
                        label,
                        (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                    )
                    displays.append(sharp_color)

        if not displays:
            return None

        # 拼接（每行最多4个）
        rows = []
        for i in range(0, len(displays), 4):
            row = np.hstack(displays[i : i + 4])
            rows.append(row)

        comparison = np.vstack(rows)
        return comparison


class AutoFocusHelper:
    """
    自动对焦辅助工具。
    通过分析单帧或视频流的清晰度来辅助对焦。
    """

    def __init__(self, roi=None):
        """
        参数:
            roi: 感兴趣区域 (x, y, w, h)，None表示全图
        """
        self.roi = roi
        self._lock = Lock()
        self._sharpness_history = []
        self._max_history = 100

    def compute_focus_score(self, frame):
        """
        计算当前帧的对焦分数。

        参数:
            frame: BGR图像
        返回:
            score: 对焦分数（越高越清晰）
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 如果有ROI，裁剪
        if self.roi is not None:
            x, y, w, h = self.roi
            gray = gray[y : y + h, x : x + w]

        # 综合多种度量
        # 1. 拉普拉斯方差
        lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()

        # 2. 梯度幅度
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        grad_mag = np.mean(gx ** 2 + gy ** 2)

        # 综合分数
        score = lap_var * 0.5 + grad_mag * 0.5

        with self._lock:
            self._sharpness_history.append(score)
            if len(self._sharpness_history) > self._max_history:
                self._sharpness_history.pop(0)

        return score

    def is_in_focus(self, frame, threshold=500.0):
        """判断当前帧是否对焦准确。"""
        score = self.compute_focus_score(frame)
        return score >= threshold

    def draw_focus_info(self, frame, threshold=500.0):
        """
        在帧上绘制对焦信息。

        参数:
            frame: BGR图像
            threshold: 对焦阈值
        返回:
            display: 绘制了信息的帧
        """
        display = frame.copy()
        score = self.compute_focus_score(frame)
        in_focus = score >= threshold

        # 状态文字
        status = "IN FOCUS" if in_focus else "OUT OF FOCUS"
        color = (0, 255, 0) if in_focus else (0, 0, 255)
        cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(
            display,
            f"Score: {score:.1f}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )

        # 对焦分数柱状图
        with self._lock:
            hist = list(self._sharpness_history)

        if len(hist) > 1:
            bar_x = frame.shape[1] - 220
            bar_y = 30
            bar_w = 200
            bar_h = 100

            # 背景
            cv2.rectangle(
                display, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (50, 50, 50), -1
            )

            # 绘制历史曲线
            max_val = max(hist) if max(hist) > 0 else 1
            n = len(hist)
            for i in range(1, n):
                x1 = bar_x + int((i - 1) / n * bar_w)
                y1 = bar_y + bar_h - int(hist[i - 1] / max_val * bar_h)
                x2 = bar_x + int(i / n * bar_w)
                y2 = bar_y + bar_h - int(hist[i] / max_val * bar_h)
                cv2.line(display, (x1, y1), (x2, y2), (0, 255, 255), 1)

            # 阈值线
            thresh_y = bar_y + bar_h - int(threshold / max_val * bar_h)
            cv2.line(
                display, (bar_x, thresh_y), (bar_x + bar_w, thresh_y), (0, 0, 255), 1
            )

        # ROI标记
        if self.roi is not None:
            x, y, w, h = self.roi
            cv2.rectangle(display, (x, y), (x + w, y + h), (255, 255, 0), 2)

        return display


class DepthFromFocusRealtime:
    """实时景深估计器（使用变焦摄像头或多摄像头）。"""

    def __init__(self, src=0, width=640, height=480, num_focus_steps=5):
        """
        参数:
            src: 摄像头索引
            width, height: 画面尺寸
            num_focus_steps: 对焦步数（采集帧数）
        """
        self.src = src
        self.width = width
        self.height = height
        self.num_focus_steps = num_focus_steps

        self.depth_estimator = DepthFromFocus(
            sharpness_method="laplacian", block_size=16
        )

        self._lock = Lock()
        self._frame = None
        self._depth_result = None
        self._running = False
        self._fps = 0.0

        # 采集的多焦距帧
        self._focus_frames = []
        self._focus_distances = []

    def _capture_loop(self):
        """实时采集与景深估计主循环。"""
        cap = cv2.VideoCapture(self.src)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not cap.isOpened():
            print("[错误] 无法打开摄像头")
            self._running = False
            return

        fps_counter = 0
        fps_timer = time.time()

        # 简化：使用连续帧模拟不同焦距（实际应控制摄像头对焦）
        frame_buffer = []
        buffer_size = self.num_focus_steps

        while self._running:
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.resize(frame, (self.width, self.height))
            frame = cv2.flip(frame, 1)

            # 缓存帧
            frame_buffer.append(frame.copy())
            if len(frame_buffer) > buffer_size:
                frame_buffer.pop(0)

            # 当缓存满时执行景深估计
            if len(frame_buffer) == buffer_size:
                # 使用不同锐度模拟多焦距（实际场景需改变对焦距离）
                sim_frames = []
                for i, f in enumerate(frame_buffer):
                    # 模拟不同焦距的模糊
                    blur_k = 2 * i + 1
                    blurred = cv2.GaussianBlur(f, (blur_k * 2 + 1, blur_k * 2 + 1), 0)
                    sim_frames.append(blurred)

                distances = np.linspace(0, 1, len(sim_frames))
                self.depth_estimator.focus_distances = distances

                depth_map, _, confidence = self.depth_estimator.estimate_depth(
                    sim_frames
                )
                depth_color = self.depth_estimator.visualize_depth()

                with self._lock:
                    self._depth_result = {
                        "depth_map": depth_map,
                        "depth_color": depth_color,
                        "confidence": confidence,
                    }

            fps_counter += 1
            if time.time() - fps_timer >= 1.0:
                self._fps = fps_counter / (time.time() - fps_timer)
                fps_counter = 0
                fps_timer = time.time()

            with self._lock:
                self._frame = frame

        cap.release()

    def start(self):
        """启动实时景深估计。"""
        if self._running:
            return
        self._running = True
        self._thread = Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        print("[信息] 景深估计已启动")

    def stop(self):
        """停止。"""
        self._running = False
        if hasattr(self, "_thread"):
            self._thread.join(timeout=2.0)
        print("[信息] 景深估计已停止")

    def get_frame(self):
        """获取最新原始帧。"""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_depth_result(self):
        """获取景深估计结果。"""
        with self._lock:
            return self._depth_result


def estimate_depth_from_files(image_paths, focus_distances=None, **kwargs):
    """
    从文件列表估计景深。

    参数:
        image_paths: 图像文件路径列表（按焦距顺序）
        focus_distances: 对应焦距值列表
        **kwargs: DepthFromFocus参数
    返回:
        depth_map: 深度图
        all_in_focus: 全焦图像
    """
    images = []
    for p in image_paths:
        img = cv2.imread(p)
        if img is not None:
            images.append(img)
        else:
            print(f"[警告] 无法读取: {p}")

    if len(images) < 2:
        print("[错误] 至少需要2张不同焦距的图像")
        return None, None

    estimator = DepthFromFocus(focus_distances=focus_distances, **kwargs)
    depth_map, sharpness_stack, confidence = estimator.estimate_depth(images)

    # 生成全焦图像
    all_in_focus = estimator.get_all_in_focus(images)

    print(f"[信息] 景深估计完成")
    print(f"  输入图像数: {len(images)}")
    print(f"  深度图尺寸: {depth_map.shape}")
    print(f"  平均置信度: {np.mean(confidence):.4f}")

    return depth_map, all_in_focus


def main():
    """
    使用示例：实时景深估计可视化。
    按 'q' 退出。
    """
    realtime = DepthFromFocusRealtime(
        src=0, width=640, height=480, num_focus_steps=5
    )
    realtime.start()

    try:
        while True:
            frame = realtime.get_frame()
            if frame is not None:
                cv2.imshow("Original", frame)

            result = realtime.get_depth_result()
            if result is not None:
                if result["depth_color"] is not None:
                    cv2.imshow("Depth Map", result["depth_color"])

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        realtime.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
