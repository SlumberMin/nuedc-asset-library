#!/usr/bin/env python3
"""
高速抓拍单元测试
覆盖: 初始化参数、YUYV快速转换、帧统计、FPS计算、缓冲管理
注意: 模拟V4L2Camera，不依赖实际硬件
"""

import sys
import os
import unittest
import time
import numpy as np
from collections import deque
from typing import Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ── 模拟实现 ──────────────────────────────────────────────────

class MockV4L2Camera:
    """模拟V4L2相机"""

    def __init__(self, device, width, height, fps=30, buffer_count=2):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self._running = False
        self._exposure = 500
        self._auto_exposure = True
        self._auto_wb = True
        self._frame_idx = 0

    def open(self):
        pass

    def close(self):
        pass

    def start_stream(self):
        self._running = True

    def stop_stream(self):
        self._running = False

    def set_exposure(self, val):
        self._exposure = val

    def set_auto_exposure(self, val):
        self._auto_exposure = val

    def set_auto_white_balance(self, val):
        self._auto_wb = val

    def get_fps(self):
        return self.fps

    def grab_frame(self):
        if not self._running:
            return None, 0.0
        # 生成模拟YUYV数据
        h, w = self.height, self.width
        yuyv = np.random.randint(0, 255, (h, w * 2), dtype=np.uint8)
        self._frame_idx += 1
        ts = time.perf_counter() * 1e6
        return yuyv.tobytes(), ts


class FastCaptureSimulator:
    """高速抓拍模拟"""

    def __init__(self, device, width=640, height=480,
                 target_fps=120.0, buffer_count=2,
                 pixel_format="YUYV", camera_factory=None):
        self.device = device
        self.width = width
        self.height = height
        self.target_fps = target_fps
        self.buffer_count = min(buffer_count, 2)
        self.pixel_format = pixel_format

        self._gray_buf = np.empty((height, width), dtype=np.uint8)
        self._frame_count = 0
        self._drop_count = 0
        self._last_ts = 0.0
        self._fps_ema = 0.0
        self._running = False

        factory = camera_factory or MockV4L2Camera
        self._cam = None
        self._cam_factory = factory
        self._cam_args = (device, width, height, target_fps, self.buffer_count)

    def start(self):
        self._cam = self._cam_factory(*self._cam_args)
        self._cam.open()
        self._cam.set_auto_exposure(False)
        self._cam.set_auto_white_balance(False)
        self._cam.start_stream()
        self._running = True

    def stop(self):
        self._running = False
        if self._cam:
            self._cam.stop_stream()
            self._cam.close()
            self._cam = None

    def grab(self):
        if not self._running or not self._cam:
            raise RuntimeError("FastCapture not started")

        t0 = time.perf_counter()
        raw_data, timestamp = self._cam.grab_frame()

        if self.pixel_format == "YUYV" and raw_data is not None:
            yuyv = np.frombuffer(raw_data, dtype=np.uint8).reshape(
                self.height, self.width * 2)
            self._gray_buf[:] = yuyv[:, 0::2]
            result = self._gray_buf
        elif raw_data is not None:
            result = np.frombuffer(raw_data, dtype=np.uint8).reshape(
                self.height, self.width)
        else:
            self._drop_count += 1
            return self._gray_buf, 0.0

        self._frame_count += 1
        now = time.perf_counter()
        dt = now - self._last_ts if self._last_ts > 0 else 1.0 / self.target_fps
        self._last_ts = now
        alpha = 0.1
        self._fps_ema = alpha * (1.0 / max(dt, 1e-6)) + (1 - alpha) * self._fps_ema

        return result, timestamp if timestamp else now * 1e6

    @property
    def fps(self):
        return self._fps_ema

    @property
    def stats(self):
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


# ── 测试用例 ──────────────────────────────────────────────────

class TestFastCaptureInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        fc = FastCaptureSimulator("/dev/video0")
        self.assertEqual(fc.width, 640)
        self.assertEqual(fc.height, 480)
        self.assertEqual(fc.target_fps, 120.0)
        self.assertEqual(fc.pixel_format, "YUYV")

    def test_custom_params(self):
        fc = FastCaptureSimulator("/dev/video0", width=320, height=240,
                                  target_fps=60, pixel_format="GRAY")
        self.assertEqual(fc.width, 320)
        self.assertEqual(fc.height, 240)
        self.assertEqual(fc.target_fps, 60)

    def test_buffer_count_capped(self):
        fc = FastCaptureSimulator("/dev/video0", buffer_count=10)
        self.assertEqual(fc.buffer_count, 2)

    def test_initial_counters(self):
        fc = FastCaptureSimulator("/dev/video0")
        self.assertEqual(fc._frame_count, 0)
        self.assertEqual(fc._drop_count, 0)
        self.assertEqual(fc.fps, 0)


class TestFastCaptureStartStop(unittest.TestCase):
    """启停测试"""

    def test_start_sets_running(self):
        fc = FastCaptureSimulator("/dev/video0")
        fc.start()
        self.assertTrue(fc._running)
        fc.stop()

    def test_stop_clears_running(self):
        fc = FastCaptureSimulator("/dev/video0")
        fc.start()
        fc.stop()
        self.assertFalse(fc._running)

    def test_context_manager(self):
        with FastCaptureSimulator("/dev/video0") as fc:
            self.assertTrue(fc._running)
        self.assertFalse(fc._running)


class TestFastCaptureGrab(unittest.TestCase):
    """抓帧测试"""

    def test_grab_returns_frame_and_timestamp(self):
        fc = FastCaptureSimulator("/dev/video0")
        fc.start()
        frame, ts = fc.grab()
        self.assertEqual(frame.shape, (480, 640))
        self.assertGreater(ts, 0)
        fc.stop()

    def test_grab_before_start_raises(self):
        fc = FastCaptureSimulator("/dev/video0")
        with self.assertRaises(RuntimeError):
            fc.grab()

    def test_grab_increments_counter(self):
        fc = FastCaptureSimulator("/dev/video0")
        fc.start()
        fc.grab()
        fc.grab()
        self.assertEqual(fc._frame_count, 2)
        fc.stop()

    def test_grab_yuyv_to_gray(self):
        """YUYV格式应正确转换为灰度"""
        fc = FastCaptureSimulator("/dev/video0", width=100, height=50,
                                  pixel_format="YUYV")
        fc.start()
        frame, _ = fc.grab()
        self.assertEqual(frame.shape, (50, 100))
        self.assertEqual(frame.dtype, np.uint8)
        fc.stop()


class TestFastCaptureStats(unittest.TestCase):
    """统计测试"""

    def test_stats_format(self):
        fc = FastCaptureSimulator("/dev/video0")
        stats = fc.stats
        self.assertIn("fps", stats)
        self.assertIn("frame_count", stats)
        self.assertIn("drop_count", stats)
        self.assertIn("target_fps", stats)

    def test_stats_after_grabs(self):
        fc = FastCaptureSimulator("/dev/video0")
        fc.start()
        for _ in range(5):
            fc.grab()
        stats = fc.stats
        self.assertEqual(stats["frame_count"], 5)
        self.assertEqual(stats["drop_count"], 0)
        fc.stop()

    def test_fps_convergence(self):
        fc = FastCaptureSimulator("/dev/video0")
        fc.start()
        for _ in range(20):
            time.sleep(0.001)
            fc.grab()
        self.assertGreater(fc.fps, 0)
        fc.stop()


class TestFastCapturePreallocatedBuffer(unittest.TestCase):
    """预分配缓冲测试"""

    def test_gray_buf_preallocated(self):
        fc = FastCaptureSimulator("/dev/video0", width=320, height=240)
        self.assertEqual(fc._gray_buf.shape, (240, 320))
        self.assertEqual(fc._gray_buf.dtype, np.uint8)

    def test_grab_returns_same_buffer(self):
        """多次grab应返回同一预分配缓冲区"""
        fc = FastCaptureSimulator("/dev/video0")
        fc.start()
        frame1, _ = fc.grab()
        frame2, _ = fc.grab()
        self.assertIs(frame1, frame2)  # 同一numpy数组
        fc.stop()


if __name__ == '__main__':
    unittest.main()
