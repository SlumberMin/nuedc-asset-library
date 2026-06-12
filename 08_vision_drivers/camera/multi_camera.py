"""
多相机支持 - 多线程采集 + 时间同步
适用于Orange Pi 5多USB3.0相机场景
"""
import time
import threading
import logging
from typing import Dict, List, Optional, Callable, Tuple
from dataclasses import dataclass, field
from concurrent.futures import ThreadPoolExecutor
import numpy as np

from .camera_manager import CameraManager, Backend, FrameStats

logger = logging.getLogger(__name__)


@dataclass
class SyncedFrame:
    """同步帧"""
    timestamp: float                    # 采集时间戳
    frames: Dict[str, np.ndarray]       # {camera_id: frame}
    stats: Dict[str, FrameStats]        # {camera_id: stats}
    time_diff_ms: float = 0.0           # 帧间最大时间差(ms)


class MultiCamera:
    """
    多相机管理器
    - 每个相机独立线程采集
    - 软件时间同步（时间戳对齐）
    - 统一接口获取同步帧
    """

    def __init__(self, backend: Backend = Backend.V4L2,
                 sync_tolerance_ms: float = 10.0):
        self.backend = backend
        self.sync_tolerance_ms = sync_tolerance_ms
        self._cameras: Dict[str, CameraManager] = {}
        self._frames: Dict[str, Tuple[float, np.ndarray]] = {}
        self._frame_lock = threading.Lock()
        self._running = False
        self._sync_thread: Optional[threading.Thread] = None
        self._sync_callbacks: List[Callable] = []
        self._executor: Optional[ThreadPoolExecutor] = None

    def add_camera(self, cam_id: str, device: str,
                   width: int = 640, height: int = 480,
                   fps: float = 60.0, pixel_format: str = "YUYV") -> bool:
        """添加相机"""
        if cam_id in self._cameras:
            logger.warning(f"相机 '{cam_id}' 已存在")
            return False

        cam = CameraManager(
            backend=self.backend, device=device,
            width=width, height=height, fps=fps,
            pixel_format=pixel_format,
        )
        self._cameras[cam_id] = cam
        logger.info(f"已添加相机: {cam_id} -> {device}")
        return True

    def remove_camera(self, cam_id: str):
        """移除相机"""
        if cam_id in self._cameras:
            self._cameras[cam_id].stop()
            del self._cameras[cam_id]
            logger.info(f"已移除相机: {cam_id}")

    def start(self) -> bool:
        """启动所有相机"""
        if self._running:
            return True

        self._executor = ThreadPoolExecutor(max_workers=len(self._cameras) + 2)
        self._running = True

        success_count = 0
        for cam_id, cam in self._cameras.items():
            cam.register_callback(lambda frame, stats, cid=cam_id: self._on_frame(cid, frame, stats))
            if cam.start():
                success_count += 1
            else:
                logger.error(f"相机 {cam_id} 启动失败")

        if success_count == 0:
            logger.error("没有相机启动成功")
            self._running = False
            return False

        # 启动同步线程
        self._sync_thread = threading.Thread(target=self._sync_loop, daemon=True)
        self._sync_thread.start()

        logger.info(f"多相机系统启动: {success_count}/{len(self._cameras)} 个相机")
        return True

    def stop(self):
        """停止所有相机"""
        self._running = False
        for cam in self._cameras.values():
            cam.stop()
        if self._sync_thread:
            self._sync_thread.join(timeout=3.0)
        if self._executor:
            self._executor.shutdown(wait=False)
        logger.info("多相机系统已停止")

    def get_synced_frame(self, timeout: float = 1.0) -> Optional[SyncedFrame]:
        """获取同步帧（阻塞等待所有相机就绪）"""
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            with self._frame_lock:
                if len(self._frames) >= len(self._cameras):
                    # 检查时间同步
                    timestamps = [t for t, _ in self._frames.values()]
                    max_diff = (max(timestamps) - min(timestamps)) * 1000

                    if max_diff <= self.sync_tolerance_ms:
                        frames = {cid: f for cid, (_, f) in self._frames.items()}
                        stats = {cid: cam.stats for cid, cam in self._cameras.items()}
                        return SyncedFrame(
                            timestamp=time.monotonic(),
                            frames=frames,
                            stats=stats,
                            time_diff_ms=max_diff,
                        )
            time.sleep(0.001)
        return None

    def get_latest_frames(self) -> Dict[str, np.ndarray]:
        """获取所有相机最新帧（不要求同步）"""
        with self._frame_lock:
            return {cid: f for cid, (_, f) in self._frames.items()}

    def get_frame(self, cam_id: str, timeout: float = 1.0) -> Optional[np.ndarray]:
        """获取单个相机的帧"""
        cam = self._cameras.get(cam_id)
        if cam:
            return cam.get_frame(timeout)
        return None

    def register_sync_callback(self, callback: Callable):
        """注册同步帧回调"""
        self._sync_callbacks.append(callback)

    def get_stats(self) -> Dict[str, dict]:
        """获取所有相机统计"""
        return {
            cid: {
                'fps': cam.stats.fps,
                'frame_count': cam.stats.frame_count,
                'drop_count': cam.stats.drop_count,
                'avg_latency_ms': cam.stats.avg_latency_ms,
            }
            for cid, cam in self._cameras.items()
        }

    def set_property(self, prop: str, value, cam_id: str = None):
        """设置相机属性（cam_id=None时设置所有）"""
        targets = [self._cameras[cam_id]] if cam_id else self._cameras.values()
        for cam in targets:
            cam.set_property(prop, value)

    def _on_frame(self, cam_id: str, frame: np.ndarray, stats: FrameStats):
        """帧回调"""
        with self._frame_lock:
            self._frames[cam_id] = (time.monotonic(), frame)

    def _sync_loop(self):
        """同步帧分发线程"""
        while self._running:
            synced = self.get_synced_frame(timeout=0.05)
            if synced:
                for cb in self._sync_callbacks:
                    try:
                        cb(synced)
                    except Exception as e:
                        logger.error(f"同步回调异常: {e}")
                # 清空已消费的帧
                with self._frame_lock:
                    self._frames.clear()
            time.sleep(0.001)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()
