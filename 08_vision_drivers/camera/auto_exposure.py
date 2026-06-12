"""
自动曝光算法 - 区域加权 + 直方图均衡
针对电赛场景优化：场地光照变化大，需要快速收敛

算法:
1. 区域加权测光（中心权重高，边缘权重低）
2. 直方图分析（检测过曝/欠曝比例）
3. PID控制器快速收敛到目标亮度
4. 支持ROI优先曝光（如目标区域）
"""
import time
import logging
from typing import Optional, Tuple, List
from dataclasses import dataclass
import numpy as np

logger = logging.getLogger(__name__)

try:
    import cv2
    _has_cv2 = True
except ImportError:
    _has_cv2 = False


@dataclass
class AEState:
    """自动曝光状态"""
    current_exposure: int = 500
    current_gain: int = 16
    target_brightness: int = 128
    measured_brightness: float = 0.0
    overexposed_ratio: float = 0.0
    underexposed_ratio: float = 0.0
    converged: bool = False
    iterations: int = 0


class AutoExposure:
    """
    自动曝光控制器
    
    使用方式:
        ae = AutoExposure(camera, target_brightness=128)
        
        # 逐帧调用
        while True:
            frame = grab_frame()
            ae.update(frame)
            # ae会自动调整camera参数
        
        # 或者一次性收敛
        ae.converge(max_iterations=20)
    """

    # 曝光参数范围
    EXPOSURE_MIN = 1
    EXPOSURE_MAX = 10000
    GAIN_MIN = 1
    GAIN_MAX = 255

    def __init__(self, camera, target_brightness: int = 128,
                 roi: Optional[Tuple[int, int, int, int]] = None,
                 metering_mode: str = "center_weighted",
                 convergence_threshold: int = 3,
                 pid_kp: float = 0.5,
                 pid_ki: float = 0.05,
                 pid_kd: float = 0.1):
        """
        Args:
            camera: V4L2Camera实例
            target_brightness: 目标平均亮度 (0-255)
            roi: 感兴趣区域 (x, y, w, h)，None为全图
            metering_mode: 测光模式 "center_weighted" | "spot" | "matrix"
            convergence_threshold: 收敛判定阈值（亮度差）
            pid_kp/kd/ki: PID控制器参数
        """
        self.camera = camera
        self.target_brightness = target_brightness
        self.roi = roi
        self.metering_mode = metering_mode
        self.convergence_threshold = convergence_threshold
        
        # PID控制器
        self.kp = pid_kp
        self.ki = pid_ki
        self.kd = pid_kd
        self._integral = 0.0
        self._prev_error = 0.0
        
        # 状态
        self.state = AEState(
            target_brightness=target_brightness,
            current_exposure=camera.get_exposure() if hasattr(camera, 'get_exposure') else 500,
        )
        
        # 权重矩阵缓存
        self._weight_map = None
        self._weight_sum = 0.0

    def _build_weight_map(self, h: int, w: int) -> np.ndarray:
        """构建区域权重矩阵"""
        if self._weight_map is not None and self._weight_map.shape == (h, w):
            return self._weight_map
        
        if self.metering_mode == "center_weighted":
            # 高斯权重：中心权重最高
            y, x = np.mgrid[0:h, 0:w]
            cy, cx = h / 2, w / 2
            sigma_y, sigma_x = h / 3, w / 3
            self._weight_map = np.exp(-(
                (y - cy) ** 2 / (2 * sigma_y ** 2) +
                (x - cx) ** 2 / (2 * sigma_x ** 2)
            ))
        
        elif self.metering_mode == "spot":
            # 点测光：只有ROI区域有权重
            self._weight_map = np.zeros((h, w), dtype=np.float64)
            if self.roi:
                x, y, rw, rh = self.roi
                self._weight_map[y:y+rh, x:x+rw] = 1.0
            else:
                # 默认中心1/4区域
                y1, y2 = h // 4, 3 * h // 4
                x1, x2 = w // 4, 3 * w // 4
                self._weight_map[y1:y2, x1:x2] = 1.0
        
        elif self.metering_mode == "matrix":
            # 矩阵测光：分区权重
            self._weight_map = np.ones((h, w), dtype=np.float64)
            # 中心区域权重2x
            y1, y2 = h // 4, 3 * h // 4
            x1, x2 = w // 4, 3 * w // 4
            self._weight_map[y1:y2, x1:x2] = 2.0
            # 边缘降权
            self._weight_map[:h//8, :] *= 0.5
            self._weight_map[-h//8:, :] *= 0.5
            self._weight_map[:, :w//8] *= 0.5
            self._weight_map[:, -w//8:] *= 0.5
        
        self._weight_sum = self._weight_map.sum()
        return self._weight_map

    def measure_brightness(self, frame: np.ndarray) -> Tuple[float, float, float]:
        """
        测量帧亮度
        
        Returns:
            (weighted_mean, overexposed_ratio, underexposed_ratio)
        """
        if frame is None or frame.size == 0:
            return 0.0, 0.0, 0.0
        
        h, w = frame.shape[:2]
        
        # ROI裁剪
        if self.roi:
            x, y, rw, rh = self.roi
            roi_frame = frame[y:y+rh, x:x+rw]
            weights = np.ones_like(roi_frame, dtype=np.float64)
            weight_sum = roi_frame.size
        else:
            roi_frame = frame
            weights = self._build_weight_map(h, w)
            weight_sum = self._weight_sum
        
        # 加权平均亮度
        weighted_mean = np.sum(roi_frame.astype(np.float64) * weights) / max(weight_sum, 1)
        
        # 过曝/欠曝比例
        over_ratio = np.mean(frame > 250) if frame.dtype == np.uint8 else 0.0
        under_ratio = np.mean(frame < 5) if frame.dtype == np.uint8 else 0.0
        
        return weighted_mean, over_ratio, under_ratio

    def update(self, frame: np.ndarray) -> AEState:
        """
        帧级更新：测量亮度并调整曝光
        
        Args:
            frame: 当前帧（灰度，uint8）
        
        Returns:
            更新后的AE状态
        """
        # 1. 测量亮度
        brightness, over_ratio, under_ratio = self.measure_brightness(frame)
        self.state.measured_brightness = brightness
        self.state.overexposed_ratio = over_ratio
        self.state.underexposed_ratio = under_ratio
        self.state.iterations += 1
        
        # 2. 计算误差
        error = self.target_brightness - brightness
        
        # 3. PID计算
        self._integral += error
        # 积分限幅（防止windup）
        self._integral = max(-500, min(500, self._integral))
        
        derivative = error - self._prev_error
        self._prev_error = error
        
        adjustment = self.kp * error + self.ki * self._integral + self.kd * derivative
        
        # 4. 调整曝光
        new_exposure = int(self.state.current_exposure + adjustment)
        new_exposure = max(self.EXPOSURE_MIN, min(self.EXPOSURE_MAX, new_exposure))
        
        # 如果曝光值已到极限，调增益
        if new_exposure >= self.EXPOSURE_MAX and error > 0:
            new_gain = min(self.GAIN_MAX, self.state.current_gain + 4)
            self.camera.set_gain(new_gain)
            self.state.current_gain = new_gain
        elif new_exposure <= self.EXPOSURE_MIN and error < 0:
            new_gain = max(self.GAIN_MIN, self.state.current_gain - 4)
            self.camera.set_gain(new_gain)
            self.state.current_gain = new_gain
        
        self.camera.set_exposure(new_exposure)
        self.state.current_exposure = new_exposure
        
        # 5. 收敛判定
        self.state.converged = abs(error) < self.convergence_threshold
        
        return self.state

    def converge(self, max_iterations: int = 30, timeout_ms: float = 2000) -> AEState:
        """
        自动收敛到目标亮度
        
        Args:
            max_iterations: 最大迭代次数
            timeout_ms: 超时时间(ms)
        
        Returns:
            最终AE状态
        """
        t0 = time.perf_counter()
        
        for i in range(max_iterations):
            # 检查超时
            if (time.perf_counter() - t0) * 1000 > timeout_ms:
                logger.warning(f"AE convergence timeout after {i} iterations")
                break
            
            # 等待新帧
            time.sleep(1.0 / max(self.camera.get_fps() if hasattr(self.camera, 'get_fps') else 30, 1))
            
            # 获取帧
            raw_data, ts = self.camera.grab_frame()
            if raw_data is None:
                continue
            
            h, w = self.camera.height, self.camera.width
            img = np.frombuffer(raw_data, dtype=np.uint8)
            if len(raw_data) == h * w * 2:
                img = img.reshape(h, w * 2)[:, 0::2]
            else:
                img = img.reshape(h, w)
            
            # 更新
            state = self.update(img)
            
            logger.debug(f"AE iter {i}: exposure={state.current_exposure}, "
                         f"brightness={state.measured_brightness:.1f}, "
                         f"error={self._prev_error:.1f}")
            
            if state.converged:
                logger.info(f"AE converged after {i+1} iterations: "
                           f"exposure={state.current_exposure}, "
                           f"brightness={state.measured_brightness:.1f}")
                break
        
        return self.state

    def auto_histogram_equalize(self, frame: np.ndarray) -> np.ndarray:
        """
        自适应直方图均衡（CLAHE）
        
        用于光照不均匀场景的后处理
        """
        if not _has_cv2:
            # 简单全局均衡
            hist, bins = np.histogram(frame.flatten(), 256, [0, 256])
            cdf = hist.cumsum()
            cdf_normalized = cdf * 255 / cdf[-1]
            return cdf_normalized[frame].astype(np.uint8)
        
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return clahe.apply(frame)

    @property
    def is_converged(self) -> bool:
        return self.state.converged

    @property
    def current_exposure(self) -> int:
        return self.state.current_exposure

    @property
    def stats(self) -> dict:
        return {
            'target': self.state.target_brightness,
            'measured': round(self.state.measured_brightness, 1),
            'exposure': self.state.current_exposure,
            'gain': self.state.current_gain,
            'overexposed': f"{self.state.overexposed_ratio*100:.1f}%",
            'underexposed': f"{self.state.underexposed_ratio*100:.1f}%",
            'converged': self.state.converged,
            'iterations': self.state.iterations,
        }
