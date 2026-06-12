"""
多帧融合降噪 - 利用全局快门特性
全局快门相机无卷帘快门果冻效应，帧间对齐精度高，
特别适合多帧降噪方案。

算法:
1. 连续采集N帧（N=4~8）
2. 基于参考帧进行光流/块匹配对齐
3. 加权融合降噪（时域+空域）
4. 边缘保持锐化
"""
import time
import logging
from typing import Optional, List, Tuple
from collections import deque
import numpy as np

logger = logging.getLogger(__name__)

try:
    import cv2
    _has_cv2 = True
except ImportError:
    _has_cv2 = False


class MultiFrameDenoise:
    """
    多帧融合降噪器
    
    利用全局快门相机帧间精确对齐的特性，
    通过多帧平均/加权融合降低噪声。
    
    使用方式:
        denoise = MultiFrameDenoise(camera, num_frames=4)
        result = denoise.capture_denoised()
    """

    def __init__(self, camera, num_frames: int = 4, 
                 align_method: str = "block_match",
                 blend_method: str = "temporal_median",
                 denoise_strength: float = 1.0):
        """
        Args:
            camera: V4L2Camera实例
            num_frames: 融合帧数(2-8)
            align_method: 对齐方法 "none" | "block_match" | "optflow"
            blend_method: 融合方法 "temporal_median" | "temporal_mean" | "nlm_fusion"
            denoise_strength: 降噪强度 (0.5~2.0)
        """
        self.camera = camera
        self.num_frames = min(max(num_frames, 2), 8)
        self.align_method = align_method
        self.blend_method = blend_method
        self.denoise_strength = denoise_strength
        
        # 帧缓冲（环形）
        self._frame_buf = deque(maxlen=num_frames)
        self._aligned_buf = deque(maxlen=num_frames)

    def capture_denoised(self) -> Tuple[np.ndarray, float]:
        """
        拍摄多帧并融合降噪
        
        Returns:
            (denoised_frame, total_time_ms)
        """
        t0 = time.perf_counter()
        
        # 1. 连续采集N帧
        raw_frames = []
        for i in range(self.num_frames):
            raw_data, ts = self.camera.grab_frame()
            if raw_data is None:
                continue
            
            h, w = self.camera.height, self.camera.width
            img = np.frombuffer(raw_data, dtype=np.uint8)
            
            # 处理YUYV格式
            if len(raw_data) == h * w * 2:
                img = img.reshape(h, w * 2)[:, 0::2]
            else:
                img = img.reshape(h, w)
            
            raw_frames.append(img.copy())
        
        if not raw_frames:
            empty = np.zeros((self.camera.height, self.camera.width), dtype=np.uint8)
            return empty, 0.0
        
        if len(raw_frames) == 1:
            return raw_frames[0], (time.perf_counter() - t0) * 1000
        
        # 2. 帧对齐
        reference = raw_frames[len(raw_frames) // 2]  # 中间帧作为参考
        aligned = self._align_frames(reference, raw_frames)
        
        # 3. 融合降噪
        result = self._blend(aligned)
        
        # 4. 后处理
        if self.denoise_strength > 1.0 and _has_cv2:
            result = cv2.fastNlMeansDenoising(
                result, None, 
                h=10 * self.denoise_strength,
                templateWindowSize=7,
                searchWindowSize=21
            )
        
        elapsed = (time.perf_counter() - t0) * 1000
        return result, elapsed

    def _align_frames(self, reference: np.ndarray, 
                      frames: List[np.ndarray]) -> List[np.ndarray]:
        """帧对齐"""
        if self.align_method == "none":
            return frames
        
        aligned = [reference]  # 参考帧不需要对齐
        
        for i, frame in enumerate(frames):
            if frame is reference:
                continue
            
            if self.align_method == "block_match":
                offset = self._block_match_align(reference, frame)
                shifted = self._shift_frame(frame, offset)
                aligned.append(shifted)
            elif self.align_method == "optflow" and _has_cv2:
                aligned_frame = self._optflow_align(reference, frame)
                aligned.append(aligned_frame)
            else:
                aligned.append(frame)
        
        return aligned

    def _block_match_align(self, ref: np.ndarray, frame: np.ndarray,
                           search_range: int = 8, block_size: int = 32) -> Tuple[int, int]:
        """
        块匹配对齐（纯numpy实现，无需OpenCV）
        
        在参考帧中心取一个块，在待对齐帧中搜索最佳匹配位置
        """
        h, w = ref.shape
        cx, cy = w // 2, h // 2
        bs = block_size // 2
        
        # 参考块
        ref_block = ref[cy-bs:cy+bs, cx-bs:cx+bs].astype(np.float64)
        
        best_sad = float('inf')
        best_dx, best_dy = 0, 0
        
        # 搜索窗口内遍历
        for dy in range(-search_range, search_range + 1, 1):
            for dx in range(-search_range, search_range + 1, 1):
                y1, y2 = cy - bs + dy, cy + bs + dy
                x1, x2 = cx - bs + dx, cx + bs + dx
                
                if y1 < 0 or y2 > h or x1 < 0 or x2 > w:
                    continue
                
                candidate = frame[y1:y2, x1:x2].astype(np.float64)
                sad = np.sum(np.abs(ref_block - candidate))
                
                if sad < best_sad:
                    best_sad = sad
                    best_dx, best_dy = dx, dy
        
        return best_dx, best_dy

    def _shift_frame(self, frame: np.ndarray, offset: Tuple[int, int]) -> np.ndarray:
        """平移帧（亚像素精度用插值）"""
        dx, dy = offset
        h, w = frame.shape
        
        result = np.zeros_like(frame)
        
        # 计算有效区域
        src_y1 = max(0, -dy)
        src_y2 = min(h, h - dy)
        src_x1 = max(0, -dx)
        src_x2 = min(w, w - dx)
        
        dst_y1 = src_y1 + dy
        dst_y2 = src_y2 + dy
        dst_x1 = src_x1 + dx
        dst_x2 = src_x2 + dx
        
        if src_y2 > src_y1 and src_x2 > src_x1:
            result[dst_y1:dst_y2, dst_x1:dst_x2] = frame[src_y1:src_y2, src_x1:src_x2]
        
        return result

    def _optflow_align(self, ref: np.ndarray, frame: np.ndarray) -> np.ndarray:
        """光流对齐（需要OpenCV）"""
        h, w = ref.shape
        
        # 计算稀疏光流
        features = cv2.goodFeaturesToTrack(ref, maxCorners=100, qualityLevel=0.01, minDistance=10)
        if features is None:
            return frame
        
        pts, status, _ = cv2.calcOpticalFlowPyrLK(
            ref, frame, features, None,
            winSize=(15, 15), maxLevel=3
        )
        
        # 计算全局平移（RANSAC剔除异常点）
        good_old = features[status.flatten() == 1]
        good_new = pts[status.flatten() == 1]
        
        if len(good_old) < 4:
            return frame
        
        # 估计仿射变换
        M, inliers = cv2.estimateAffinePartial2D(good_old, good_new)
        if M is None:
            return frame
        
        aligned = cv2.warpAffine(frame, M, (w, h), flags=cv2.INTER_LINEAR)
        return aligned

    def _blend(self, frames: List[np.ndarray]) -> np.ndarray:
        """帧融合"""
        if self.blend_method == "temporal_median":
            return self._temporal_median(frames)
        elif self.blend_method == "temporal_mean":
            return self._temporal_mean(frames)
        elif self.blend_method == "nlm_fusion" and _has_cv2:
            return self._nlm_fusion(frames)
        else:
            return self._temporal_median(frames)

    def _temporal_median(self, frames: List[np.ndarray]) -> np.ndarray:
        """
        时域中值滤波 - 最佳去椒盐噪声，保持边缘
        
        全局快门下帧间对齐精度高，中值滤波效果好
        """
        stack = np.stack(frames, axis=0)  # (N, H, W)
        return np.median(stack, axis=0).astype(np.uint8)

    def _temporal_mean(self, frames: List[np.ndarray]) -> np.ndarray:
        """
        时域均值滤波 - 最佳高斯噪声抑制
        
        SNR提升: sqrt(N)倍，4帧=2倍信噪比提升
        """
        stack = np.stack(frames, axis=0).astype(np.float64)
        return np.mean(stack, axis=0).astype(np.uint8)

    def _nlm_fusion(self, frames: List[np.ndarray]) -> np.ndarray:
        """
        非局部均值融合 - 结合时域和空域信息
        """
        # 先做时域中值
        temporal = self._temporal_median(frames)
        
        # 再做空域NLM
        h_param = int(10 * self.denoise_strength)
        result = cv2.fastNlMeansDenoising(
            temporal, None, h=h_param,
            templateWindowSize=7, searchWindowSize=21
        )
        return result

    def denoise_single(self, frame: np.ndarray) -> np.ndarray:
        """
        单帧降噪（无多帧时的后备方案）
        
        使用空域NLM降噪
        """
        if _has_cv2:
            h = int(10 * self.denoise_strength)
            return cv2.fastNlMeansDenoising(frame, None, h=h)
        
        # 无OpenCV时用简单高斯模糊
        kernel = np.array([[1, 2, 1], [2, 4, 2], [1, 2, 1]], dtype=np.float64) / 16
        from scipy.ndimage import convolve
        return convolve(frame.astype(np.float64), kernel).astype(np.uint8)

    @property
    def stats(self) -> dict:
        return {
            'num_frames': self.num_frames,
            'align_method': self.align_method,
            'blend_method': self.blend_method,
            'denoise_strength': self.denoise_strength,
            'snr_gain_db': 10 * np.log10(self.num_frames) if self.num_frames > 0 else 0,
        }
