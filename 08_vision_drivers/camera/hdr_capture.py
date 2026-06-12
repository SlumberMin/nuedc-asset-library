"""
HDR多帧合成 - 扩展动态范围
利用全局快门相机拍摄多帧不同曝光，合成高动态范围图像

工作流程:
1. 快速连拍3-5帧不同曝光（短/中/长）
2. 基于曝光比进行对齐（全局快门无运动伪影）
3. 加权融合生成HDR图像
4. Tone mapping输出8-bit显示
"""
import time
import logging
from typing import List, Tuple, Optional
from dataclasses import dataclass
import numpy as np

logger = logging.getLogger(__name__)

# 尝试导入cv2
try:
    import cv2
    _has_cv2 = True
except ImportError:
    _has_cv2 = False
    logger.warning("OpenCV not available, HDR tone mapping disabled")


@dataclass
class HDRFrame:
    """单帧HDR数据"""
    image: np.ndarray          # 灰度或BGR图像
    exposure_time: float       # 曝光时间(us)
    gain: float                # 增益
    timestamp: float           # 时间戳


class HDRCapture:
    """
    HDR多帧合成采集器
    
    使用方式:
        hdr = HDRCapture(camera, ev_offsets=[-2, 0, 2])
        result = hdr.capture()  # 返回8-bit HDR合成图
    """

    # 默认EV包围曝光序列
    DEFAULT_EV_OFFSETS = [-2, 0, 2]  # 3帧: 欠曝/正常/过曝

    def __init__(self, camera, ev_offsets: List[int] = None,
                 base_exposure: int = 500, base_gain: int = 16,
                 merge_method: str = "linear_weighted"):
        """
        Args:
            camera: V4L2Camera实例
            ev_offsets: EV包围值列表，如[-2, 0, 2]
            base_exposure: 基础曝光值（100us单位）
            base_gain: 基础增益
            merge_method: 融合方法 "linear_weighted" | "debevec" | "mertens"
        """
        self.camera = camera
        self.ev_offsets = ev_offsets or self.DEFAULT_EV_OFFSETS
        self.base_exposure = base_exposure
        self.base_gain = base_gain
        self.merge_method = merge_method
        
        # EV到曝光倍率映射: 每1EV = 2x曝光
        self._ev_multipliers = [2.0 ** ev for ev in self.ev_offsets]

    def capture(self) -> Tuple[np.ndarray, List[HDRFrame]]:
        """
        执行HDR拍摄+合成
        
        Returns:
            (hdr_result, frames) - 合成结果和原始帧列表
        """
        frames = []
        t0 = time.perf_counter()
        
        # 1. 多帧不同曝光拍摄
        for i, (ev, mult) in enumerate(zip(self.ev_offsets, self._ev_multipliers)):
            exposure = int(self.base_exposure * mult)
            exposure = max(1, min(exposure, 10000))  # 限制范围
            
            self.camera.set_exposure(exposure)
            self.camera.set_gain(self.base_gain)
            
            # 等待参数生效（至少1帧延迟）
            time.sleep(1.0 / max(self.camera.get_fps(), 30))
            
            # 抓取帧
            raw_data, ts = self.camera.grab_frame()
            if raw_data is None:
                logger.warning(f"HDR frame {i} capture failed")
                continue
            
            img = np.frombuffer(raw_data, dtype=np.uint8)
            h, w = self.camera.height, self.camera.width
            if len(raw_data) == h * w * 2:  # YUYV
                img = img.reshape(h, w * 2)[:, 0::2]  # 取Y分量
            else:
                img = img.reshape(h, w)
            
            frames.append(HDRFrame(
                image=img.copy(),
                exposure_time=exposure,
                gain=self.base_gain,
                timestamp=ts or time.perf_counter() * 1e6
            ))
        
        # 恢复原始曝光
        self.camera.set_exposure(self.base_exposure)
        
        if len(frames) < 2:
            logger.error("HDR requires at least 2 frames")
            if frames:
                return frames[0].image, frames
            return np.zeros((self.camera.height, self.camera.width), dtype=np.uint8), frames
        
        # 2. HDR合成
        elapsed = (time.perf_counter() - t0) * 1000
        logger.info(f"HDR capture: {len(frames)} frames in {elapsed:.1f}ms")
        
        result = self._merge(frames)
        return result, frames

    def _merge(self, frames: List[HDRFrame]) -> np.ndarray:
        """HDR融合"""
        if self.merge_method == "linear_weighted":
            return self._merge_linear_weighted(frames)
        elif _has_cv2 and self.merge_method == "mertens":
            return self._merge_mertens(frames)
        else:
            return self._merge_linear_weighted(frames)

    def _merge_linear_weighted(self, frames: List[HDRFrame]) -> np.ndarray:
        """
        线性加权融合（纯numpy实现，无需OpenCV）
        
        权重策略:
        - 欠曝帧: 高亮区域权重高
        - 正常帧: 中间调权重高
        - 过曝帧: 暗部区域权重高
        """
        h, w = frames[0].image.shape
        total_weight = np.zeros((h, w), dtype=np.float64)
        weighted_sum = np.zeros((h, w), dtype=np.float64)
        
        for frame in frames:
            img = frame.image.astype(np.float64)
            exposure = frame.exposure_time
            
            # 权重函数: 三角权重，中间灰度权重最高
            # w(z) = z for z<128, w(z) = 255-z for z>=128
            weight = np.where(img < 128, img, 255.0 - img)
            weight = weight / 128.0  # 归一化到[0,1]
            weight = np.clip(weight, 0.01, 1.0)  # 避免零权重
            
            # 归一化曝光
            normalized = img / exposure
            
            weighted_sum += weight * normalized
            total_weight += weight
        
        # 归一化
        result = weighted_sum / np.maximum(total_weight, 1e-6)
        
        # Tone mapping: Reinhard全局算子
        result = self._tone_map_reinhard(result)
        
        return result.astype(np.uint8)

    def _tone_map_reinhard(self, hdr: np.ndarray) -> np.ndarray:
        """
        Reinhard全局tone mapping
        
        L_out = L / (1 + L)
        """
        # 归一化到[0, 1]
        l_min, l_max = hdr.min(), hdr.max()
        if l_max - l_min < 1e-6:
            return np.full_like(hdr, 128, dtype=np.uint8)
        
        normalized = (hdr - l_min) / (l_max - l_min)
        
        # Reinhard映射
        mapped = normalized / (1.0 + normalized)
        
        # 调整gamma
        gamma = 0.8
        mapped = np.power(mapped, 1.0 / gamma)
        
        return (mapped * 255).clip(0, 255).astype(np.uint8)

    def _merge_mertens(self, frames: List[HDRFrame]) -> np.ndarray:
        """OpenCV Mertens融合（无需标定，直接融合）"""
        images = [f.image for f in frames]
        merge = cv2.createMergeMertens(
            contrast_weight=1.0,
            saturation_weight=1.0,
            exposure_weight=0.0
        )
        result = merge.process(images)
        return (result * 255).clip(0, 255).astype(np.uint8)

    def capture_fast(self) -> np.ndarray:
        """
        快速HDR（仅2帧，最小延迟）
        
        一帧欠曝抓高光，一帧过曝抓暗部，直接融合
        """
        frames_data = []
        
        # 短曝光帧
        self.camera.set_exposure(int(self.base_exposure * 0.25))
        time.sleep(0.02)
        raw, _ = self.camera.grab_frame()
        if raw:
            img = np.frombuffer(raw, dtype=np.uint8).reshape(
                self.camera.height, self.camera.width) if len(raw) == self.camera.height * self.camera.width else \
                np.frombuffer(raw, dtype=np.uint8).reshape(self.camera.height, self.camera.width * 2)[:, 0::2]
            frames_data.append((img.copy(), self.base_exposure * 0.25))
        
        # 长曝光帧
        self.camera.set_exposure(int(self.base_exposure * 4))
        time.sleep(0.02)
        raw, _ = self.camera.grab_frame()
        if raw:
            img = np.frombuffer(raw, dtype=np.uint8).reshape(
                self.camera.height, self.camera.width) if len(raw) == self.camera.height * self.camera.width else \
                np.frombuffer(raw, dtype=np.uint8).reshape(self.camera.height, self.camera.width * 2)[:, 0::2]
            frames_data.append((img.copy(), self.base_exposure * 4))
        
        self.camera.set_exposure(self.base_exposure)
        
        if len(frames_data) < 2:
            return np.zeros((self.camera.height, self.camera.width), dtype=np.uint8)
        
        # 简单双帧融合
        dark, dark_exp = frames_data[0]
        bright, bright_exp = frames_data[1]
        
        dark_f = dark.astype(np.float64)
        bright_f = bright.astype(np.float64)
        
        # 暗帧取高光，亮帧取暗部
        mask = (dark_f > 200).astype(np.float64)
        result = dark_f * mask + bright_f * (1 - mask)
        
        return result.clip(0, 255).astype(np.uint8)
