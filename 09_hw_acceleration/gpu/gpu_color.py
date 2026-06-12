"""
Mali GPU 颜色处理模块
======================
GPU加速的颜色空间转换和颜色检测功能。

功能：
- 颜色空间转换 (BGR/RGB/HSV/LAB/YUV/HLS)
- GPU颜色范围检测 (HSV阈值分割)
- 多区域颜色同时检测
- 颜色直方图计算
- 颜色追踪
"""

import numpy as np
import cv2
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class ColorRange:
    """HSV颜色范围"""
    name: str
    lower: np.ndarray  # HSV下界
    upper: np.ndarray  # HSV上界

    def __post_init__(self):
        self.lower = np.array(self.lower, dtype=np.uint8)
        self.upper = np.array(self.upper, dtype=np.uint8)


# 预定义颜色范围 (HSV)
PRESET_COLORS = {
    'red': ColorRange('red', [0, 100, 100], [10, 255, 255]),
    'red2': ColorRange('red2', [170, 100, 100], [180, 255, 255]),  # 红色跨0度
    'orange': ColorRange('orange', [10, 100, 100], [25, 255, 255]),
    'yellow': ColorRange('yellow', [25, 100, 100], [35, 255, 255]),
    'green': ColorRange('green', [35, 80, 80], [85, 255, 255]),
    'cyan': ColorRange('cyan', [85, 80, 80], [100, 255, 255]),
    'blue': ColorRange('blue', [100, 100, 100], [130, 255, 255]),
    'purple': ColorRange('purple', [130, 80, 80], [170, 255, 255]),
    'white': ColorRange('white', [0, 0, 200], [180, 30, 255]),
    'black': ColorRange('black', [0, 0, 0], [180, 255, 50]),
}


class GpuColorProcessor:
    """
    GPU颜色处理器
    
    使用示例:
        color = GpuColorProcessor()
        
        # 颜色空间转换
        hsv = color.convert(img, 'bgr', 'hsv')
        
        # 颜色检测
        mask = color.detect_color(img, 'red')
        
        # 多颜色检测
        results = color.detect_multi(img, ['red', 'blue', 'green'])
        
        # 颜色追踪
        bbox = color.track_color(img, 'green')
    """

    def __init__(self):
        self._ocl_available = False
        self._cv_gpu_available = False
        self._custom_colors: Dict[str, ColorRange] = {}
        self._init()

    def _init(self):
        try:
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                self._cv_gpu_available = True
                return
        except Exception:
            pass
        try:
            if cv2.ocl.haveOpenCL():
                cv2.ocl.setUseOpenCL(True)
                self._ocl_available = True
        except Exception:
            pass

    @property
    def available(self) -> bool:
        return self._cv_gpu_available or self._ocl_available

    def _to_umat(self, img):
        return cv2.UMat(img)

    def _from_umat(self, umat):
        if isinstance(umat, cv2.UMat):
            return umat.get()
        return np.asarray(umat)

    def convert(self, img: np.ndarray, src_space: str, dst_space: str) -> np.ndarray:
        """
        GPU颜色空间转换
        
        Args:
            img: 输入图像
            src_space: 源空间 (bgr/rgb/hsv/lab/yuv/hls/gray)
            dst_space: 目标空间
            
        Returns:
            转换后的图像
        """
        code = self._get_cvt_code(src_space, dst_space)

        if not self.available:
            return cv2.cvtColor(img, code)

        if self._cv_gpu_available:
            gpu_img = cv2.cuda_GpuMat()
            gpu_img.upload(img)
            gpu_result = cv2.cuda.cvtColor(gpu_img, code)
            return gpu_result.download()

        umat = self._to_umat(img)
        result = cv2.cvtColor(umat, code)
        return self._from_umat(result)

    def _get_cvt_code(self, src: str, dst: str) -> int:
        """获取OpenCV颜色转换代码"""
        s = src.lower()
        d = dst.lower()

        codes = {
            ('bgr', 'hsv'): cv2.COLOR_BGR2HSV,
            ('bgr', 'lab'): cv2.COLOR_BGR2LAB,
            ('bgr', 'yuv'): cv2.COLOR_BGR2YUV,
            ('bgr', 'hls'): cv2.COLOR_BGR2HLS,
            ('bgr', 'gray'): cv2.COLOR_BGR2GRAY,
            ('bgr', 'rgb'): cv2.COLOR_BGR2RGB,
            ('rgb', 'bgr'): cv2.COLOR_RGB2BGR,
            ('rgb', 'hsv'): cv2.COLOR_RGB2HSV,
            ('rgb', 'lab'): cv2.COLOR_RGB2LAB,
            ('hsv', 'bgr'): cv2.COLOR_HSV2BGR,
            ('hsv', 'rgb'): cv2.COLOR_HSV2RGB,
            ('lab', 'bgr'): cv2.COLOR_LAB2BGR,
            ('lab', 'rgb'): cv2.COLOR_LAB2RGB,
            ('yuv', 'bgr'): cv2.COLOR_YUV2BGR,
            ('hls', 'bgr'): cv2.COLOR_HLS2BGR,
            ('gray', 'bgr'): cv2.COLOR_GRAY2BGR,
        }

        key = (s, d)
        if key in codes:
            return codes[key]
        raise ValueError(f"不支持的颜色空间转换: {s} -> {d}")

    def add_color(self, name: str, lower: List[int], upper: List[int]):
        """添加自定义颜色范围"""
        self._custom_colors[name] = ColorRange(name, lower, upper)

    def detect_color(self, img: np.ndarray, color_name: str,
                     morph_kernel: int = 5) -> np.ndarray:
        """
        GPU颜色检测
        
        Args:
            img: BGR输入图像
            color_name: 颜色名称 (预设或自定义)
            morph_kernel: 形态学核大小
            
        Returns:
            二值掩码
        """
        hsv = self.convert(img, 'bgr', 'hsv')

        color_range = self._get_color_range(color_name)
        if color_range is None:
            raise ValueError(f"未知颜色: {color_name}")

        if self.available:
            umat_hsv = self._to_umat(hsv)
            mask = cv2.inRange(umat_hsv, color_range.lower, color_range.upper)
            mask = self._from_umat(mask)
        else:
            mask = cv2.inRange(hsv, color_range.lower, color_range.upper)

        # 形态学开闭运算去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def _get_color_range(self, name: str) -> Optional[ColorRange]:
        """获取颜色范围"""
        if name in self._custom_colors:
            return self._custom_colors[name]
        return PRESET_COLORS.get(name)

    def detect_multi(self, img: np.ndarray,
                     color_names: List[str]) -> Dict[str, np.ndarray]:
        """多颜色同时检测"""
        hsv = self.convert(img, 'bgr', 'hsv')
        results = {}

        for name in color_names:
            cr = self._get_color_range(name)
            if cr is None:
                continue
            mask = cv2.inRange(hsv, cr.lower, cr.upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            results[name] = mask

        return results

    def find_color_contours(self, img: np.ndarray, color_name: str,
                            min_area: int = 100) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """
        检测颜色并返回轮廓和边界框
        
        Returns:
            [(contour, (x, y, w, h)), ...]
        """
        mask = self.detect_color(img, color_name)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= min_area:
                bbox = cv2.boundingRect(cnt)
                results.append((cnt, bbox))

        return results

    def track_color(self, img: np.ndarray, color_name: str,
                    min_area: int = 500) -> Optional[Tuple[int, int, int, int]]:
        """
        颜色追踪 - 返回最大目标的边界框
        
        Returns:
            (x, y, w, h) 或 None
        """
        contours = self.find_color_contours(img, color_name, min_area)
        if not contours:
            return None

        # 选择最大轮廓
        max_contour = max(contours, key=lambda x: cv2.contourArea(x[0]))
        return max_contour[1]

    def color_histogram(self, img: np.ndarray, mask: Optional[np.ndarray] = None,
                        bins: int = 64) -> Dict[str, np.ndarray]:
        """计算颜色直方图"""
        channels = {}
        names = ['blue', 'green', 'red'] if img.shape[2] == 3 else ['b', 'g', 'r', 'a']

        for i, name in enumerate(names[:img.shape[2]]):
            hist = cv2.calcHist([img], [i], mask, [bins], [0, 256])
            channels[name] = hist.flatten()

        return channels

    def dominant_color(self, img: np.ndarray,
                       mask: Optional[np.ndarray] = None,
                       k: int = 3) -> Tuple[np.ndarray, np.ndarray]:
        """
        提取主色调 (K-Means聚类)
        
        Returns:
            (colors, percentages) - K个主要颜色及其占比
        """
        data = img.reshape(-1, 3).astype(np.float32)
        if mask is not None:
            data = data[mask.flatten() > 0]

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
        _, labels, centers = cv2.kmeans(data, k, None, criteria, 3,
                                        cv2.KMEANS_PP_CENTERS)

        # 计算各颜色占比
        counts = np.bincount(labels.flatten())
        percentages = counts / counts.sum()

        # 按占比排序
        idx = np.argsort(-percentages)
        return centers[idx].astype(np.uint8), percentages[idx]

    def benchmark(self, img_size=(1920, 1080), iterations=100) -> dict:
        """性能测试"""
        import time
        img = np.random.randint(0, 255, (img_size[1], img_size[0], 3), dtype=np.uint8)
        results = {}

        ops = [
            ('bgr_to_hsv', lambda: self.convert(img, 'bgr', 'hsv')),
            ('hsv_to_bgr', lambda: self.convert(
                self.convert(img, 'bgr', 'hsv'), 'hsv', 'bgr')),
            ('bgr_to_lab', lambda: self.convert(img, 'bgr', 'lab')),
            ('bgr_to_gray', lambda: self.convert(img, 'bgr', 'gray')),
            ('detect_red', lambda: self.detect_color(img, 'red')),
            ('detect_multi', lambda: self.detect_multi(img, ['red', 'blue', 'green'])),
        ]

        for name, fn in ops:
            t0 = time.perf_counter()
            for _ in range(iterations):
                fn()
            elapsed = (time.perf_counter() - t0) / iterations * 1000
            results[name] = round(elapsed, 2)

        return results


if __name__ == '__main__':
    color = GpuColorProcessor()
    print(f"GPU颜色处理可用: {color.available}")

    test_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # 颜色空间转换
    hsv = color.convert(test_img, 'bgr', 'hsv')
    print(f"BGR->HSV: {hsv.shape}")

    lab = color.convert(test_img, 'bgr', 'lab')
    print(f"BGR->LAB: {lab.shape}")

    # 颜色检测
    mask = color.detect_color(test_img, 'red')
    print(f"红色检测: {np.count_nonzero(mask)} 像素")

    # 多颜色检测
    masks = color.detect_multi(test_img, ['red', 'blue', 'green'])
    for name, m in masks.items():
        print(f"  {name}: {np.count_nonzero(m)} 像素")

    # 主色调
    colors, pcts = color.dominant_color(test_img, k=3)
    for i, (c, p) in enumerate(zip(colors, pcts)):
        print(f"  主色{i+1}: BGR={c}, 占比={p:.1%}")
