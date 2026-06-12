"""
libuvc驱动 - 低延迟USB摄像头访问
特性：
- 零拷贝帧获取
- 异步回调模式
- 支持UVC控件直接设置
"""
import time
import logging
import threading
from typing import Optional, Callable
import numpy as np

logger = logging.getLogger(__name__)

try:
    import libuvc
    HAS_LIBUVC = True
except ImportError:
    HAS_LIBUVC = False
    logger.warning("libuvc未安装，此驱动不可用。安装: pip install libuvc")


class LibUVCCamera:
    """
    libuvc后端驱动
    优势：
    - 比V4L2更低的延迟（绕过V4L2缓冲层）
    - 异步回调零拷贝
    - 精确的UVC控件控制
    """

    def __init__(self, device: str = None, width: int = 640, height: int = 480,
                 fps: float = 60.0, pixelformat: str = 'YUYV'):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.pixelformat = pixelformat

        self._ctx = None
        self._dev = None
        self._stream = None
        self._stream_ctrl = None
        self._frame_callback: Optional[Callable] = None
        self._running = False
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._latest_raw = None
        self._frame_event = threading.Event()

        # UVC像素格式映射
        self._uvc_formats = {
            'YUYV': 0x56595559 if not HAS_LIBUVC else None,
            'MJPEG': 0x47504A4D if not HAS_LIBUVC else None,
        }

    def open(self):
        """打开libuvc设备"""
        if not HAS_LIBUVC:
            raise RuntimeError("libuvc未安装")

        self._ctx = libuvc.UVCContext()
        self._dev = self._ctx.find_device()
        if self._dev is None:
            raise RuntimeError("未找到UVC设备")

        self._dev.open()
        logger.info(f"libuvc设备已打开: {self._dev.info}")

        # 配置流控参数
        self._stream_ctrl = self._dev.get_stream_ctrl_format_size(
            self.width, self.height, self.fps, self.pixelformat
        )

    def close(self):
        """关闭设备"""
        self.stop_streaming()
        if self._dev:
            self._dev.close()
        if self._ctx:
            self._ctx.close()
        logger.info("libuvc设备已关闭")

    def start_streaming(self, callback: Callable = None):
        """启动异步流传输"""
        if self._running:
            return

        self._frame_callback = callback
        self._dev.start_streaming(self._on_frame, self._stream_ctrl)
        self._running = True
        logger.info("libuvc流传输已启动")

    def stop_streaming(self):
        """停止流传输"""
        if self._running and self._dev:
            self._dev.stop_streaming()
            self._running = False

    def read(self) -> Optional[np.ndarray]:
        """同步读取一帧（阻塞等待）"""
        if not self._running:
            self.start_streaming()

        if self._frame_event.wait(timeout=1.0):
            self._frame_event.clear()
            with self._frame_lock:
                return self._latest_frame
        return None

    def get_raw(self) -> Optional[bytes]:
        """获取原始帧数据"""
        with self._frame_lock:
            return self._latest_raw

    def read_raw_yuyv(self) -> Optional[bytes]:
        """直接获取YUYV原始数据（零拷贝路径）"""
        if self._frame_event.wait(timeout=1.0):
            self._frame_event.clear()
            with self._frame_lock:
                return self._latest_raw
        return None

    def set_exposure(self, value: int):
        """设置曝光"""
        if self._dev:
            try:
                self._dev.set_ctrl(0x009a0902, value)  # EXPOSURE_ABSOLUTE
            except Exception as e:
                logger.warning(f"设置曝光失败: {e}")

    def set_gain(self, value: int):
        """设置增益"""
        if self._dev:
            try:
                self._dev.set_ctrl(0x009a0913, value)  # GAIN
            except Exception as e:
                logger.warning(f"设置增益失败: {e}")

    def set_property(self, prop: str, value):
        """通用属性设置"""
        prop_map = {
            'exposure': self.set_exposure,
            'gain': self.set_gain,
        }
        fn = prop_map.get(prop)
        if fn:
            fn(value)

    def _on_frame(self, frame):
        """libuvc帧回调"""
        try:
            data = frame.data
            with self._frame_lock:
                self._latest_raw = data
                # 根据格式转换
                if self.pixelformat == 'YUYV':
                    self._latest_frame = self._yuyv_to_bgr(data)
                elif self.pixelformat == 'MJPEG':
                    import cv2
                    arr = np.frombuffer(data, dtype=np.uint8)
                    self._latest_frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                else:
                    self._latest_frame = np.frombuffer(data, dtype=np.uint8).reshape(
                        self.height, self.width, 2
                    )

            self._frame_event.set()

            # 外部回调
            if self._frame_callback:
                self._frame_callback(self._latest_frame)

        except Exception as e:
            logger.error(f"帧处理异常: {e}")

    def _yuyv_to_bgr(self, data: bytes) -> np.ndarray:
        """YUYV转BGR"""
        import cv2
        yuyv = np.frombuffer(data, dtype=np.uint8).reshape(self.height, self.width, 2)
        return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUYV)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()
