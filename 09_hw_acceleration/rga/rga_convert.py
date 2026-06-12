"""
RGA 格式转换专用模块
=====================
RK3588S RGA硬件格式转换，支持常见视频/图像格式互转。
主要用于摄像头帧、视频解码帧的快速格式转换。

典型场景：
- YUYV (USB摄像头) → RGB/BGR (OpenCV处理)
- NV12 (VPU解码输出) → BGR (显示/处理)
- BGR (OpenCV) → NV12 (硬件编码器输入)
- RGB ↔ BGR 转换
"""

import numpy as np
import cv2
from typing import Optional, Tuple, Union
from enum import IntEnum
import logging
import time

logger = logging.getLogger(__name__)


class PixelFormat(IntEnum):
    """像素格式枚举"""
    YUYV = 0
    UYVY = 1
    NV12 = 2       # YUV420 Semi-Planar
    NV21 = 3       # YUV420 Semi-Planar (VU顺序)
    I420 = 4       # YUV420 Planar
    YV12 = 5
    RGB = 6
    RGBA = 7
    BGR = 8
    BGRA = 9
    GRAY = 10
    RGB565 = 11


# 格式转换映射表
_FORMAT_SIZE = {
    PixelFormat.YUYV: 2,    # 2 bytes/pixel
    PixelFormat.UYVY: 2,
    PixelFormat.NV12: 1.5,  # 12 bits/pixel
    PixelFormat.NV21: 1.5,
    PixelFormat.I420: 1.5,
    PixelFormat.YV12: 1.5,
    PixelFormat.RGB: 3,
    PixelFormat.RGBA: 4,
    PixelFormat.BGR: 3,
    PixelFormat.BGRA: 4,
    PixelFormat.GRAY: 1,
    PixelFormat.RGB565: 2,
}

# OpenCV颜色转换代码映射
_CVT_MAP = {
    (PixelFormat.YUYV, PixelFormat.BGR): cv2.COLOR_YUV2BGR_YUYV,
    (PixelFormat.YUYV, PixelFormat.RGB): cv2.COLOR_YUV2RGB_YUYV,
    (PixelFormat.UYVY, PixelFormat.BGR): cv2.COLOR_YUV2BGR_UYVY,
    (PixelFormat.NV12, PixelFormat.BGR): cv2.COLOR_YUV2BGR_NV12,
    (PixelFormat.NV12, PixelFormat.RGB): cv2.COLOR_YUV2RGB_NV12,
    (PixelFormat.NV21, PixelFormat.BGR): cv2.COLOR_YUV2BGR_NV21,
    (PixelFormat.NV21, PixelFormat.RGB): cv2.COLOR_YUV2RGB_NV21,
    (PixelFormat.I420, PixelFormat.BGR): cv2.COLOR_YUV2BGR_I420,
    (PixelFormat.I420, PixelFormat.RGB): cv2.COLOR_YUV2RGB_I420,
    (PixelFormat.BGR, PixelFormat.RGB): cv2.COLOR_BGR2RGB,
    (PixelFormat.RGB, PixelFormat.BGR): cv2.COLOR_RGB2BGR,
    (PixelFormat.BGR, PixelFormat.GRAY): cv2.COLOR_BGR2GRAY,
    (PixelFormat.RGB, PixelFormat.GRAY): cv2.COLOR_RGB2GRAY,
    (PixelFormat.GRAY, PixelFormat.BGR): cv2.COLOR_GRAY2BGR,
    (PixelFormat.BGR, PixelFormat.BGRA): cv2.COLOR_BGR2BGRA,
    (PixelFormat.RGB, PixelFormat.RGBA): cv2.COLOR_RGB2RGBA,
    (PixelFormat.BGRA, PixelFormat.BGR): cv2.COLOR_BGRA2BGR,
    (PixelFormat.RGBA, PixelFormat.RGB): cv2.COLOR_RGBA2RGB,
    (PixelFormat.NV12, PixelFormat.NV21): None,  # UV互换
    (PixelFormat.NV21, PixelFormat.NV12): None,
}


def get_buffer_size(fmt: PixelFormat, width: int, height: int) -> int:
    """计算指定格式的缓冲区大小"""
    bpp = _FORMAT_SIZE.get(fmt, 3)
    return int(width * height * bpp)


class RgaConverter:
    """
    RGA格式转换器
    
    使用示例:
        conv = RgaConverter()
        
        # USB摄像头YUYV转BGR
        bgr = conv.convert(yuyv_frame, PixelFormat.YUYV, PixelFormat.BGR, 640, 480)
        
        # VPU解码NV12转BGR
        bgr = conv.convert(nv12_frame, PixelFormat.NV12, PixelFormat.BGR, 1920, 1080)
        
        # OpenCV BGR转NV12给编码器
        nv12 = conv.convert(bgr_frame, PixelFormat.BGR, PixelFormat.NV12, 1920, 1080)
    """

    def __init__(self):
        self._rga = None
        try:
            from .rga_utils import RgaUtils, RgaFormat
            self._rga = RgaUtils()
            self._RgaFormat = RgaFormat
        except ImportError:
            try:
                from rga_utils import RgaUtils, RgaFormat
                self._rga = RgaUtils()
                self._RgaFormat = RgaFormat
            except ImportError:
                logger.warning("RGA模块不可用，使用软件转换")

    def convert(self, data: np.ndarray,
                src_fmt: PixelFormat, dst_fmt: PixelFormat,
                width: int, height: int) -> np.ndarray:
        """
        格式转换
        
        Args:
            data: 源数据 (可以是1D uint8数组或已reshape的2D/3D数组)
            src_fmt: 源格式
            dst_fmt: 目标格式
            width: 图像宽度
            height: 图像高度
            
        Returns:
            转换后的numpy数组
        """
        if src_fmt == dst_fmt:
            return data.copy()

        # 确保数据是连续的
        if not data.flags['C_CONTIGUOUS']:
            data = np.ascontiguousarray(data)

        # 尝试RGA硬件加速
        if self._rga and self._rga.available:
            try:
                return self._rga_convert(data, src_fmt, dst_fmt, width, height)
            except Exception as e:
                logger.debug(f"RGA转换失败，回退软件: {e}")

        # 软件转换
        return self._sw_convert(data, src_fmt, dst_fmt, width, height)

    def _rga_convert(self, data: np.ndarray,
                     src_fmt: PixelFormat, dst_fmt: PixelFormat,
                     width: int, height: int) -> np.ndarray:
        """RGA硬件加速转换"""
        fmt_map = {
            PixelFormat.YUYV: self._RgaFormat.YUV422_YUYV,
            PixelFormat.NV12: self._RgaFormat.YUV420_SP,
            PixelFormat.NV21: self._RgaFormat.YCrCb_420_SP,
            PixelFormat.I420: self._RgaFormat.YUV420_P,
            PixelFormat.RGB: self._RgaFormat.RGB_888,
            PixelFormat.BGR: self._RgaFormat.BGR_888,
            PixelFormat.RGBA: self._RgaFormat.RGBA_8888,
            PixelFormat.BGRA: self._RgaFormat.BGRA_8888,
        }

        rga_src = fmt_map.get(src_fmt)
        rga_dst = fmt_map.get(dst_fmt)

        if rga_src is None or rga_dst is None:
            raise ValueError(f"RGA不支持此格式转换: {src_fmt}->{dst_fmt}")

        return self._rga.convert(data, rga_src, rga_dst, width, height)

    def _sw_convert(self, data: np.ndarray,
                    src_fmt: PixelFormat, dst_fmt: PixelFormat,
                    width: int, height: int) -> np.ndarray:
        """CPU软件转换回退"""
        # reshape原始数据
        img = self._reshape_input(data, src_fmt, width, height)

        # NV12/NV21 互换 (只需交换UV平面)
        if (src_fmt == PixelFormat.NV12 and dst_fmt == PixelFormat.NV21) or \
           (src_fmt == PixelFormat.NV21 and dst_fmt == PixelFormat.NV12):
            result = img.copy()
            uv_start = height
            uv = result[uv_start:].reshape(-1, 2)
            uv[:, [0, 1]] = uv[:, [1, 0]]
            return result

        # 查找OpenCV转换码
        cvt_code = _CVT_MAP.get((src_fmt, dst_fmt))
        if cvt_code is not None:
            return cv2.cvtColor(img, cvt_code)

        # 通过中间格式转换 (BGR作为中间格式)
        if src_fmt != PixelFormat.BGR and dst_fmt != PixelFormat.BGR:
            mid_cvt = _CVT_MAP.get((src_fmt, PixelFormat.BGR))
            dst_cvt = _CVT_MAP.get((PixelFormat.BGR, dst_fmt))
            if mid_cvt and dst_cvt:
                mid = cv2.cvtColor(img, mid_cvt)
                return cv2.cvtColor(mid, dst_cvt)

        raise ValueError(f"不支持的转换: {src_fmt.name} -> {dst_fmt.name}")

    def _reshape_input(self, data: np.ndarray, fmt: PixelFormat,
                       width: int, height: int) -> np.ndarray:
        """将原始数据reshape为正确的图像形状"""
        if data.ndim >= 2:
            return data

        # 1D数据需要reshape
        if fmt in (PixelFormat.YUYV, PixelFormat.UYVY):
            return data.reshape(height, width, 2)
        elif fmt in (PixelFormat.NV12, PixelFormat.NV21):
            return data.reshape(height * 3 // 2, width)
        elif fmt in (PixelFormat.I420, PixelFormat.YV12):
            return data.reshape(height * 3 // 2, width)
        elif fmt in (PixelFormat.RGB, PixelFormat.BGR):
            return data.reshape(height, width, 3)
        elif fmt in (PixelFormat.RGBA, PixelFormat.BGRA):
            return data.reshape(height, width, 4)
        elif fmt == PixelFormat.GRAY:
            return data.reshape(height, width)
        elif fmt == PixelFormat.RGB565:
            return data.reshape(height, width).view(np.uint16)
        else:
            return data.reshape(height, width, 3)

    def yuyv_to_bgr(self, yuyv: np.ndarray, width: int, height: int) -> np.ndarray:
        """USB摄像头YUYV转BGR"""
        return self.convert(yuyv, PixelFormat.YUYV, PixelFormat.BGR, width, height)

    def nv12_to_bgr(self, nv12: np.ndarray, width: int, height: int) -> np.ndarray:
        """VPU解码NV12转BGR"""
        return self.convert(nv12, PixelFormat.NV12, PixelFormat.BGR, width, height)

    def nv12_to_rgb(self, nv12: np.ndarray, width: int, height: int) -> np.ndarray:
        """NV12转RGB（用于RKNPU推理）"""
        return self.convert(nv12, PixelFormat.NV12, PixelFormat.RGB, width, height)

    def bgr_to_nv12(self, bgr: np.ndarray) -> np.ndarray:
        """BGR转NV12（用于硬件编码器）"""
        h, w = bgr.shape[:2]
        return self.convert(bgr, PixelFormat.BGR, PixelFormat.NV12, w, h)

    def bgr_to_rgb(self, bgr: np.ndarray) -> np.ndarray:
        """BGR转RGB"""
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def rgb_to_bgr(self, rgb: np.ndarray) -> np.ndarray:
        """RGB转BGR"""
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def batch_convert(self, frames: list, src_fmt: PixelFormat,
                      dst_fmt: PixelFormat, width: int, height: int) -> list:
        """批量格式转换"""
        return [self.convert(f, src_fmt, dst_fmt, width, height) for f in frames]

    def benchmark(self, width: int = 1920, height: int = 1080,
                  iterations: int = 200) -> dict:
        """格式转换性能测试"""
        results = {}

        test_cases = [
            (PixelFormat.YUYV, PixelFormat.BGR, "YUYV->BGR"),
            (PixelFormat.NV12, PixelFormat.BGR, "NV12->BGR"),
            (PixelFormat.NV12, PixelFormat.RGB, "NV12->RGB"),
            (PixelFormat.BGR, PixelFormat.NV12, "BGR->NV12"),
            (PixelFormat.BGR, PixelFormat.RGB, "BGR->RGB"),
        ]

        for src_fmt, dst_fmt, label in test_cases:
            bpp = _FORMAT_SIZE[src_fmt]
            buf_size = int(width * height * bpp)
            test_data = np.random.randint(0, 255, buf_size, dtype=np.uint8)

            # 本次转换
            t0 = time.perf_counter()
            for _ in range(iterations):
                self.convert(test_data, src_fmt, dst_fmt, width, height)
            elapsed = (time.perf_counter() - t0) / iterations * 1000

            results[label] = {'ms': round(elapsed, 2)}

        return results


# 单例
_default_converter = None

def get_converter() -> RgaConverter:
    global _default_converter
    if _default_converter is None:
        _default_converter = RgaConverter()
    return _default_converter


if __name__ == '__main__':
    conv = RgaConverter()

    # YUYV转BGR测试
    w, h = 640, 480
    yuyv = np.random.randint(0, 255, (h, w, 2), dtype=np.uint8)
    bgr = conv.yuyv_to_bgr(yuyv, w, h)
    print(f"YUYV ({yuyv.shape}) -> BGR ({bgr.shape})")

    # NV12转BGR测试
    nv12 = np.random.randint(0, 255, (h * 3 // 2, w), dtype=np.uint8)
    bgr = conv.nv12_to_bgr(nv12, w, h)
    print(f"NV12 ({nv12.shape}) -> BGR ({bgr.shape})")

    # BGR转NV12测试
    bgr_in = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
    nv12_out = conv.bgr_to_nv12(bgr_in)
    print(f"BGR ({bgr_in.shape}) -> NV12 ({nv12_out.shape})")

    # 性能测试
    print("\n性能测试 (1080p):")
    results = conv.benchmark(1920, 1080, 100)
    for label, data in results.items():
        print(f"  {label}: {data['ms']} ms")
