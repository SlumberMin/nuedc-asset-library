"""
运动模糊检测模块 - 拉普拉斯方差 + FFT频谱分析
适用于电赛中图像质量评估、自动对焦判断和运动检测任务。
"""

import cv2
import numpy as np
from threading import Thread, Lock
import time


class MotionBlurDetector:
    """
    运动模糊检测器。
    综合拉普拉斯方差法和FFT频谱分析法判断图像模糊程度。
    """

    def __init__(
        self,
        src=0,
        width=640,
        height=480,
        laplacian_threshold=100.0,
        fft_threshold=10.0,
        blur_direction=True,
    ):
        """
        初始化模糊检测器。

        参数:
            src: 摄像头索引或视频路径
            width, height: 画面尺寸
            laplacian_threshold: 拉普拉斯方差阈值（低于此值判定模糊）
            fft_threshold: FFT高频能量阈值（低于此值判定模糊）
            blur_direction: 是否检测模糊方向
        """
        self.src = src
        self.width = width
        self.height = height
        self.laplacian_threshold = laplacian_threshold
        self.fft_threshold = fft_threshold
        self.blur_direction = blur_direction

        self._lock = Lock()
        self._frame = None
        self._result = None
        self._running = False
        self._fps = 0.0

    def laplacian_variance(self, gray):
        """
        拉普拉斯方差法：计算图像二阶导数的方差。

        原理：模糊图像边缘不锐利，二阶导数变化小，方差低。

        参数:
            gray: 灰度图像
        返回:
            score: 模糊评分（越低越模糊）
        """
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        score = laplacian.var()
        return score

    def fft_frequency_analysis(self, gray):
        """
        FFT频谱分析法：分析图像高频分量能量。

        原理：模糊图像高频信息丢失，高频区域能量低。

        参数:
            gray: 灰度图像
        返回:
            energy_ratio: 高频能量占比
            magnitude: 频谱幅度图（可视化用）
        """
        # 执行2D FFT
        f_transform = np.fft.fft2(gray.astype(np.float64))
        f_shift = np.fft.fftshift(f_transform)

        # 计算幅度谱
        magnitude = np.log1p(np.abs(f_shift))

        # 计算总能量
        total_energy = np.sum(np.abs(f_shift) ** 2)

        if total_energy == 0:
            return 0.0, magnitude

        # 创建高通滤波器掩码（中心为低频，边缘为高频）
        rows, cols = gray.shape
        crow, ccol = rows // 2, cols // 2
        radius = min(rows, cols) // 6  # 高频半径阈值

        mask = np.ones((rows, cols), dtype=np.float64)
        Y, X = np.ogrid[:rows, :cols]
        dist = np.sqrt((X - ccol) ** 2 + (Y - crow) ** 2)
        mask[dist <= radius] = 0  # 屏蔽低频

        # 高频能量
        high_freq_energy = np.sum((np.abs(f_shift) * mask) ** 2)
        energy_ratio = high_freq_energy / total_energy

        return energy_ratio, magnitude

    def detect_blur_direction(self, gray):
        """
        检测运动模糊方向（基于方向梯度分析）。

        参数:
            gray: 灰度图像
        返回:
            angle: 主要模糊方向角度（度）
            strength: 该方向的模糊强度
        """
        # 使用多个方向的Sobel滤波器
        angles = []
        strengths = []

        for angle in range(0, 180, 15):
            # 计算该方向的梯度
            rad = np.deg2rad(angle)
            dx = np.cos(rad)
            dy = np.sin(rad)

            # 用Sobel近似方向梯度
            gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
            gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
            directional = gx * dx + gy * dy

            # 方向梯度的方差作为强度指标
            var_val = np.var(directional)
            angles.append(angle)
            strengths.append(var_val)

        # 方差最小的方向可能是模糊方向（该方向梯度被"抹平"）
        min_idx = np.argmin(strengths)
        return angles[min_idx], strengths[min_idx]

    def detect_motion_blur_psf(self, gray, estimated_length=20):
        """
        估计运动模糊的PSF（点扩散函数）长度。

        参数:
            gray: 灰度图像
            estimated_length: 估计的模糊核长度上限
        返回:
            estimated_len: 估计的模糊核长度
        """
        # 频谱分析中的暗条纹间距与模糊核长度相关
        f_transform = np.fft.fft2(gray.astype(np.float64))
        f_shift = np.fft.fftshift(f_transform)
        magnitude = np.log1p(np.abs(f_shift))

        # 二值化频谱
        _, binary = cv2.threshold(
            magnitude.astype(np.uint8), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        # 寻找频谱中的暗条纹（对应模糊核）
        # 简化：通过水平和垂直投影估算
        h_proj = np.sum(binary, axis=1)
        v_proj = np.sum(binary, axis=0)

        # 寻找投影中的凹陷
        h_min = np.min(h_proj)
        v_min = np.min(v_proj)

        if h_min < np.mean(h_proj) * 0.3:
            estimated_len = max(estimated_len, 10)
        else:
            estimated_len = 5

        return estimated_len

    def _process_frame(self, frame):
        """
        处理单帧，综合评估模糊程度。

        参数:
            frame: BGR图像
        返回:
            result: 包含各项指标的字典
        """
        frame = cv2.resize(frame, (self.width, self.height))
        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. 拉普拉斯方差
        lap_score = self.laplacian_variance(gray)

        # 2. FFT频谱分析
        fft_ratio, magnitude = self.fft_frequency_analysis(gray)

        # 3. 综合判断是否模糊
        is_blurry_lap = lap_score < self.laplacian_threshold
        is_blurry_fft = fft_ratio < self.fft_threshold / 100.0
        is_blurry = is_blurry_lap or is_blurry_fft

        # 4. 模糊方向检测
        blur_angle = None
        blur_strength = None
        if self.blur_direction and is_blurry:
            blur_angle, blur_strength = self.detect_blur_direction(gray)

        # 5. 模糊核长度估计
        psf_length = None
        if is_blurry:
            psf_length = self.detect_motion_blur_psf(gray)

        # 6. 生成清晰度热力图（局部清晰度）
        sharpness_map = self._local_sharpness_map(gray)

        result = {
            "laplacian_score": lap_score,
            "fft_energy_ratio": fft_ratio,
            "is_blurry": is_blurry,
            "blur_angle": blur_angle,
            "blur_strength": blur_strength,
            "psf_length": psf_length,
            "sharpness_map": sharpness_map,
            "fft_magnitude": magnitude,
        }

        return result, frame

    def _local_sharpness_map(self, gray, block_size=32):
        """
        计算局部清晰度热力图（分块拉普拉斯方差）。

        参数:
            gray: 灰度图像
            block_size: 分块大小
        返回:
            sharpness_map: 归一化的清晰度热力图
        """
        h, w = gray.shape
        map_h = h // block_size
        map_w = w // block_size
        sharpness_map = np.zeros((map_h, map_w), dtype=np.float64)

        for i in range(map_h):
            for j in range(map_w):
                block = gray[
                    i * block_size : (i + 1) * block_size,
                    j * block_size : (j + 1) * block_size,
                ]
                sharpness_map[i, j] = cv2.Laplacian(block, cv2.CV_64F).var()

        # 归一化到0-255
        if sharpness_map.max() > 0:
            sharpness_map = (sharpness_map / sharpness_map.max() * 255).astype(
                np.uint8
            )
        return sharpness_map

    def _draw_results(self, frame, result):
        """绘制检测结果到帧上。"""
        display = frame.copy()

        # 模糊状态文字
        status = "模糊(BLURRY)" if result["is_blurry"] else "清晰(SHARP)"
        color = (0, 0, 255) if result["is_blurry"] else (0, 255, 0)
        cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # 指标数值
        cv2.putText(
            display,
            f"Laplacian: {result['laplacian_score']:.1f}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            display,
            f"FFT Ratio: {result['fft_energy_ratio']:.4f}",
            (10, 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )

        # 模糊方向箭头
        if result["blur_angle"] is not None:
            angle = result["blur_angle"]
            cx, cy = self.width - 80, 80
            length = 50
            ex = int(cx + length * np.cos(np.deg2rad(angle)))
            ey = int(cy + length * np.sin(np.deg2rad(angle)))
            cv2.arrowedLine(display, (cx, cy), (ex, ey), (0, 255, 255), 3)
            cv2.putText(
                display,
                f"Dir: {angle}deg",
                (self.width - 150, 150),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )

        # 右侧小图：清晰度热力图
        sm = result["sharpness_map"]
        sm_resized = cv2.resize(sm, (160, 120))
        sm_color = cv2.applyColorMap(sm_resized, cv2.COLORMAP_JET)
        display[10:130, self.width - 170 : self.width - 10] = sm_color

        # FPS
        cv2.putText(
            display,
            f"FPS: {self._fps:.1f}",
            (10, self.height - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )

        return display

    def _capture_loop(self):
        """捕获与处理主循环。"""
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

            result, processed_frame = self._process_frame(frame)
            display = self._draw_results(processed_frame, result)

            fps_counter += 1
            if time.time() - fps_timer >= 1.0:
                self._fps = fps_counter / (time.time() - fps_timer)
                fps_counter = 0
                fps_timer = time.time()

            with self._lock:
                self._frame = display
                self._result = result

        cap.release()

    def start(self):
        """启动模糊检测（多线程）。"""
        if self._running:
            return
        self._running = True
        self._thread = Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        print("[信息] 运动模糊检测已启动")

    def stop(self):
        """停止检测。"""
        self._running = False
        if hasattr(self, "_thread"):
            self._thread.join(timeout=2.0)
        print("[信息] 运动模糊检测已停止")

    def get_frame(self):
        """获取最新处理帧。"""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_result(self):
        """获取最新检测结果。"""
        with self._lock:
            return self._result

    def is_blurry(self):
        """快速查询：当前画面是否模糊。"""
        with self._lock:
            return self._result["is_blurry"] if self._result else None


def analyze_single_image(image_path, **kwargs):
    """
    静态图像模糊分析工具函数。

    参数:
        image_path: 图像路径
        **kwargs: MotionBlurDetector参数
    返回:
        result: 分析结果字典
    """
    img = cv2.imread(image_path)
    if img is None:
        print(f"[错误] 无法读取图像: {image_path}")
        return None

    detector = MotionBlurDetector(**kwargs)
    result, frame = detector._process_frame(img)

    print(f"图像: {image_path}")
    print(f"  拉普拉斯方差: {result['laplacian_score']:.2f}")
    print(f"  FFT高频能量比: {result['fft_energy_ratio']:.4f}")
    print(f"  是否模糊: {'是' if result['is_blurry'] else '否'}")
    if result["blur_angle"] is not None:
        print(f"  模糊方向: {result['blur_angle']}度")

    return result


def main():
    """
    使用示例：实时运动模糊检测。
    按 'q' 退出。
    """
    detector = MotionBlurDetector(
        src=0, width=640, height=480, laplacian_threshold=100.0
    )
    detector.start()

    try:
        while True:
            frame = detector.get_frame()
            if frame is not None:
                cv2.imshow("Motion Blur Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        detector.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
