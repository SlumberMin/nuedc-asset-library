"""
相机管理器 - Orange Pi 5 USB3.0全局快门相机统一管理
支持V4L2/libuvc/OpenCV三种后端，多线程采集，帧率统计
"""
import cv2
import time
import threading
import logging
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, Callable, Dict, List, Tuple
import numpy as np

logger = logging.getLogger(__name__)


class Backend(Enum):
    V4L2 = "v4l2"
    LIBUVC = "libuvc"
    OPENCV = "opencv"


@dataclass
class FrameStats:
    """帧率统计"""
    frame_count: int = 0
    fps: float = 0.0
    avg_latency_ms: float = 0.0
    drop_count: int = 0
    _timestamps: list = field(default_factory=list)
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def update(self, timestamp: float):
        with self._lock:
            self.frame_count += 1
            self._timestamps.append(timestamp)
            # 保留最近2秒的数据
            cutoff = timestamp - 2.0
            self._timestamps = [t for t in self._timestamps if t > cutoff]
            if len(self._timestamps) >= 2:
                dt = self._timestamps[-1] - self._timestamps[0]
                self.fps = (len(self._timestamps) - 1) / dt if dt > 0 else 0
                self.avg_latency_ms = dt / (len(self._timestamps) - 1) * 1000 if len(self._timestamps) > 1 else 0

    def record_drop(self):
        with self._lock:
            self.drop_count += 1


@dataclass
class CameraInfo:
    """相机设备信息"""
    device_path: str          # /dev/video0
    name: str                 # 设备名称
    vendor_id: int            # USB VID
    product_id: int           # USB PID
    is_usb3: bool             # 是否USB3.0
    supported_formats: list   # 支持的像素格式
    max_resolution: Tuple[int, int] = (0, 0)
    max_fps: float = 0.0


class CameraManager:
    """
    统一相机管理器
    - 自动检测USB3.0相机
    - 支持V4L2/libuvc/OpenCV三种采集后端
    - 多线程采集 + 帧率统计
    """

    def __init__(self, backend: Backend = Backend.V4L2, device: str = None,
                 width: int = 640, height: int = 480, fps: float = 60.0,
                 pixel_format: str = "YUYV"):
        self.backend = backend
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.pixel_format = pixel_format

        self._camera = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame_lock = threading.Lock()
        self._current_frame: Optional[np.ndarray] = None
        self._raw_buffer = None
        self.stats = FrameStats()
        self._callbacks: List[Callable] = []

        # 根据后端初始化
        self._driver = None

    @staticmethod
    def enumerate_cameras() -> List[CameraInfo]:
        """枚举系统中所有USB相机"""
        cameras = []
        import glob
        import subprocess

        for dev in sorted(glob.glob('/dev/video*')):
            try:
                # v4l2-ctl获取设备信息
                result = subprocess.run(
                    ['v4l2-ctl', '--device', dev, '--all'],
                    capture_output=True, text=True, timeout=2
                )
                info_text = result.stdout
                name = "Unknown"
                for line in info_text.split('\n'):
                    if 'Card type' in line:
                        name = line.split(':')[-1].strip()
                        break

                is_usb3 = 'usb' in info_text.lower() and ('3.0' in info_text or 'xHCI' in info_text)
                formats = []
                fmt_result = subprocess.run(
                    ['v4l2-ctl', '--device', dev, '--list-formats-ext'],
                    capture_output=True, text=True, timeout=2
                )
                for line in fmt_result.stdout.split('\n'):
                    if "'" in line:
                        fmt = line.split("'")[1] if "'" in line else ""
                        if fmt:
                            formats.append(fmt)

                cam = CameraInfo(
                    device_path=dev, name=name,
                    vendor_id=0, product_id=0,
                    is_usb3=is_usb3,
                    supported_formats=formats
                )
                cameras.append(cam)
            except Exception as e:
                logger.debug(f"无法查询 {dev}: {e}")

        return cameras

    def start(self) -> bool:
        """启动采集"""
        if self._running:
            logger.warning("相机已在运行")
            return True

        if not self.device:
            cameras = self.enumerate_cameras()
            if not cameras:
                logger.error("未检测到相机设备")
                return False
            self.device = cameras[0].device_path
            logger.info(f"自动选择相机: {self.device}")

        # 根据后端创建驱动
        try:
            if self.backend == Backend.V4L2:
                from .camera_v4l2 import V4L2Camera
                self._driver = V4L2Camera(
                    self.device, self.width, self.height, self.fps, self.pixel_format
                )
            elif self.backend == Backend.LIBUVC:
                from .camera_libuvc import LibUVCCamera
                self._driver = LibUVCCamera(
                    self.device, self.width, self.height, self.fps
                )
            else:
                self._driver = OpenCVDriver(self.device, self.width, self.height, self.fps)

            self._driver.open()
        except Exception as e:
            logger.error(f"打开相机失败 [{self.backend.value}]: {e}")
            # 降级到OpenCV
            if self.backend != Backend.OPENCV:
                logger.info("降级到OpenCV后端")
                self.backend = Backend.OPENCV
                try:
                    self._driver = OpenCVDriver(self.device, self.width, self.height, self.fps)
                    self._driver.open()
                except Exception as e2:
                    logger.error(f"OpenCV后端也失败: {e2}")
                    return False
            else:
                return False

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        logger.info(f"相机启动成功 [{self.backend.value}] {self.width}x{self.height}@{self.fps}fps")
        return True

    def stop(self):
        """停止采集"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._driver:
            self._driver.close()
        logger.info("相机已停止")

    def get_frame(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        """获取最新帧"""
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            with self._frame_lock:
                if self._current_frame is not None:
                    return self._current_frame.copy()
            time.sleep(0.001)
        return None

    def get_raw_buffer(self) -> Optional[bytes]:
        """获取原始YUYV缓冲区（零拷贝路径）"""
        with self._frame_lock:
            return self._raw_buffer

    def register_callback(self, callback: Callable):
        """注册帧回调"""
        self._callbacks.append(callback)

    def set_exposure(self, value: int):
        """设置曝光"""
        if self._driver and hasattr(self._driver, 'set_exposure'):
            self._driver.set_exposure(value)

    def set_gain(self, value: int):
        """设置增益"""
        if self._driver and hasattr(self._driver, 'set_gain'):
            self._driver.set_gain(value)

    def set_property(self, prop: str, value):
        """通用属性设置"""
        if self._driver and hasattr(self._driver, 'set_property'):
            self._driver.set_property(prop, value)

    def _capture_loop(self):
        """采集线程主循环"""
        logger.info("采集线程启动")
        while self._running:
            try:
                t0 = time.monotonic()
                frame = self._driver.read()
                if frame is None:
                    self.stats.record_drop()
                    time.sleep(0.001)
                    continue

                with self._frame_lock:
                    self._current_frame = frame
                    if hasattr(self._driver, 'get_raw'):
                        self._raw_buffer = self._driver.get_raw()

                self.stats.update(t0)

                for cb in self._callbacks:
                    try:
                        cb(frame, self.stats)
                    except Exception as e:
                        logger.error(f"回调异常: {e}")

            except Exception as e:
                logger.error(f"采集异常: {e}")
                time.sleep(0.01)

        logger.info("采集线程退出")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()


class OpenCVDriver:
    """OpenCV后端驱动（降级方案）"""

    def __init__(self, device, width, height, fps):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self._cap = None

    def open(self):
        dev_id = self.device
        if isinstance(dev_id, str) and '/dev/video' in dev_id:
            dev_id = int(dev_id.replace('/dev/video', ''))
        self._cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f"无法打开设备: {self.device}")
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read(self):
        ret, frame = self._cap.read()
        return frame if ret else None

    def close(self):
        if self._cap:
            self._cap.release()

    def set_exposure(self, value):
        self._cap.set(cv2.CAP_PROP_EXPOSURE, value)

    def set_gain(self, value):
        self._cap.set(cv2.CAP_PROP_GAIN, value)
