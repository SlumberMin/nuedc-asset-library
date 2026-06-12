"""
高速抓拍模式 - 120fps优化，最小化延迟
针对Orange Pi 5 RK3588S + USB3.0全局快门相机
特性:
- 最小缓冲深度（1-2帧）
- 预分配numpy数组，零分配热路径
- 直接YUYV→灰度快速转换（跳过完整解码）
- 帧时间戳精确到微秒
"""
import os
import time
import mmap
import struct
import fcntl
import logging
import threading
from typing import Optional, Tuple, Callable
from collections import deque
import numpy as np

from .camera_v4l2 import V4L2Camera
from .camera_config import CameraParams

logger = logging.getLogger(__name__)

# V4L2常量
VIDIOC_S_PARM = 0xc0cc5616
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1


class FastCapture:
    """
    高速抓拍模式
    
    使用方式:
        cam = FastCapture("/dev/video0", width=640, height=480, target_fps=120)
        cam.start()
        frame, timestamp = cam.grab()  # 最小延迟获取一帧
        cam.stop()
    """

    def __init__(self, device: str, width: int = 640, height: int = 480,
                 target_fps: float = 120.0, buffer_count: int = 2,
                 pixel_format: str = "YUYV"):
        self.device = device
        self.width = width
        self.height = height
        self.target_fps = target_fps
        self.buffer_count = min(buffer_count, 2)  # 最小化延迟
        self.pixel_format = pixel_format
        
        # 预分配输出缓冲（避免热路径分配）
        self._gray_buf = np.empty((height, width), dtype=np.uint8)
        self._yuyv_buf = np.empty((height, width * 2), dtype=np.uint8)
        
        # 帧统计
        self._frame_count = 0
        self._drop_count = 0
        self._last_ts = 0.0
        self._fps_ema = 0.0  # 指数移动平均帧率
        
        # 相机实例
        self._cam: Optional[V4L2Camera] = None
        self._running = False
        
        # 回调模式支持
        self._callback: Optional[Callable] = None
        self._thread: Optional[threading.Thread] = None

    def _configure_high_speed(self):
        """配置高速模式参数"""
        # 设置最小曝光以支持高帧率
        max_exposure = int(1_000_000 / self.target_fps * 0.8)  # 80%帧周期
        self._cam.set_exposure(max_exposure)
        # 关闭自动曝光/白平衡以减少延迟
        self._cam.set_auto_exposure(False)
        self._cam.set_auto_white_balance(False)

    def start(self):
        """启动高速采集"""
        self._cam = V4L2Camera(
            self.device, self.width, self.height,
            fps=self.target_fps, buffer_count=self.buffer_count
        )
        self._cam.open()
        self._configure_high_speed()
        self._cam.start_stream()
        self._running = True
        logger.info(f"FastCapture started: {self.width}x{self.height}@{self.target_fps}fps, "
                     f"buffers={self.buffer_count}")

    def stop(self):
        """停止采集"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._cam:
            self._cam.stop_stream()
            self._cam.close()
            self._cam = None
        logger.info(f"FastCapture stopped. frames={self._frame_count}, drops={self._drop_count}")

    def grab(self) -> Tuple[np.ndarray, float]:
        """
        获取一帧（最小延迟路径）
        
        Returns:
            (gray_frame, timestamp_us) - 灰度图像和微秒时间戳
        """
        if not self._running or not self._cam:
            raise RuntimeError("FastCapture not started")
        
        t0 = time.perf_counter()
        
        # 直接从V4L2获取原始数据
        raw_data, timestamp = self._cam.grab_frame()
        
        # 快速YUYV→灰度：只取Y分量（偶数字节）
        if self.pixel_format == "YUYV" and raw_data is not None:
            # 利用numpy stride跳过UV分量，比完整转换快4x
            yuyv = np.frombuffer(raw_data, dtype=np.uint8).reshape(self.height, self.width * 2)
            # Y分量在byte0和byte2位置，即每4字节取前2字节
            self._gray_buf[:] = yuyv[:, 0::2]
            result = self._gray_buf
        elif raw_data is not None:
            result = np.frombuffer(raw_data, dtype=np.uint8).reshape(self.height, self.width)
        else:
            self._drop_count += 1
            return self._gray_buf, 0.0
        
        # 更新统计
        self._frame_count += 1
        now = time.perf_counter()
        dt = now - self._last_ts if self._last_ts > 0 else 1.0 / self.target_fps
        self._last_ts = now
        alpha = 0.1
        self._fps_ema = alpha * (1.0 / max(dt, 1e-6)) + (1 - alpha) * self._fps_ema
        
        latency_ms = (now - t0) * 1000
        return result, timestamp if timestamp else now * 1e6

    def grab_color(self) -> Tuple[Optional[np.ndarray], float]:
        """获取彩色帧（BGR格式，比grab慢）"""
        if not self._running or not self._cam:
            raise RuntimeError("FastCapture not started")
        
        raw_data, timestamp = self._cam.grab_frame()
        if raw_data is None:
            self._drop_count += 1
            return None, 0.0
        
        self._frame_count += 1
        yuyv = np.frombuffer(raw_data, dtype=np.uint8).reshape(self.height, self.width, 2)
        bgr = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUYV) if _has_cv2 else None
        return bgr, timestamp or time.perf_counter() * 1e6

    def start_callback(self, callback: Callable[[np.ndarray, float], None]):
        """启动回调模式（独立线程持续采集）"""
        self._callback = callback
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        """回调采集循环"""
        while self._running:
            frame, ts = self.grab()
            if self._callback:
                self._callback(frame, ts)

    @property
    def fps(self) -> float:
        """当前实际帧率"""
        return self._fps_ema

    @property
    def stats(self) -> dict:
        return {
            'fps': round(self._fps_ema, 1),
            'frame_count': self._frame_count,
            'drop_count': self._drop_count,
            'target_fps': self.target_fps,
        }

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()


# 尝试导入cv2（可选）
try:
    import cv2
    _has_cv2 = True
except ImportError:
    _has_cv2 = False
