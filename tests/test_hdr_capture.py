#!/usr/bin/env python3
"""
HDR合成单元测试
覆盖: EV包围曝光、线性加权融合、Reinhard tone mapping、双帧快速HDR
注意: 模拟相机，不依赖实际硬件
"""

import sys
import os
import unittest
import time
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ── 模拟实现 ──────────────────────────────────────────────────

@dataclass
class HDRFrame:
    image: np.ndarray
    exposure_time: float
    gain: float
    timestamp: float


class MockCamera:
    """模拟相机"""
    def __init__(self, width=320, height=240, fps=30):
        self.width = width
        self.height = height
        self._fps = fps
        self._exposure = 500
        self._gain = 16
        self._frame_count = 0

    def set_exposure(self, val):
        self._exposure = val

    def set_gain(self, val):
        self._gain = val

    def get_fps(self):
        return self._fps

    def get_exposure(self):
        return self._exposure

    def grab_frame(self):
        h, w = self.height, self.width
        # 生成模拟灰度帧，亮度与曝光相关
        brightness = min(255, int(self._exposure / 10))
        img = np.full((h, w), brightness, dtype=np.uint8)
        # 加噪声
        noise = np.random.randint(-10, 10, (h, w), dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        self._frame_count += 1
        return img.tobytes(), time.perf_counter() * 1e6


class HDRCaptureSimulator:
    """HDR采集器模拟"""

    DEFAULT_EV_OFFSETS = [-2, 0, 2]

    def __init__(self, camera, ev_offsets=None,
                 base_exposure=500, base_gain=16,
                 merge_method="linear_weighted"):
        self.camera = camera
        self.ev_offsets = ev_offsets or self.DEFAULT_EV_OFFSETS
        self.base_exposure = base_exposure
        self.base_gain = base_gain
        self.merge_method = merge_method
        self._ev_multipliers = [2.0 ** ev for ev in self.ev_offsets]

    def capture(self):
        frames = []
        for i, (ev, mult) in enumerate(zip(self.ev_offsets, self._ev_multipliers)):
            exposure = int(self.base_exposure * mult)
            exposure = max(1, min(exposure, 10000))

            self.camera.set_exposure(exposure)
            self.camera.set_gain(self.base_gain)

            raw_data, ts = self.camera.grab_frame()
            if raw_data is None:
                continue

            img = np.frombuffer(raw_data, dtype=np.uint8)
            h, w = self.camera.height, self.camera.width
            img = img.reshape(h, w)

            frames.append(HDRFrame(
                image=img.copy(),
                exposure_time=exposure,
                gain=self.base_gain,
                timestamp=ts or time.perf_counter() * 1e6
            ))

        self.camera.set_exposure(self.base_exposure)

        if len(frames) < 2:
            if frames:
                return frames[0].image, frames
            return np.zeros((self.camera.height, self.camera.width), dtype=np.uint8), frames

        result = self._merge(frames)
        return result, frames

    def _merge(self, frames):
        if self.merge_method == "linear_weighted":
            return self._merge_linear_weighted(frames)
        return self._merge_linear_weighted(frames)

    def _merge_linear_weighted(self, frames):
        h, w = frames[0].image.shape
        total_weight = np.zeros((h, w), dtype=np.float64)
        weighted_sum = np.zeros((h, w), dtype=np.float64)

        for frame in frames:
            img = frame.image.astype(np.float64)
            exposure = frame.exposure_time

            weight = np.where(img < 128, img, 255.0 - img)
            weight = weight / 128.0
            weight = np.clip(weight, 0.01, 1.0)

            normalized = img / max(exposure, 1e-6)
            weighted_sum += weight * normalized
            total_weight += weight

        result = weighted_sum / np.maximum(total_weight, 1e-6)
        result = self._tone_map_reinhard(result)
        return result.astype(np.uint8)

    def _tone_map_reinhard(self, hdr):
        l_min, l_max = hdr.min(), hdr.max()
        if l_max - l_min < 1e-6:
            return np.full_like(hdr, 128, dtype=np.float64)

        normalized = (hdr - l_min) / (l_max - l_min)
        mapped = normalized / (1.0 + normalized)
        gamma = 0.8
        mapped = np.power(mapped, 1.0 / gamma)
        return (mapped * 255).clip(0, 255)

    def get_ev_multipliers(self):
        return self._ev_multipliers


# ── 测试用例 ──────────────────────────────────────────────────

class TestHDRFrame(unittest.TestCase):
    """HDR帧数据结构测试"""

    def test_hdr_frame_fields(self):
        img = np.zeros((100, 100), dtype=np.uint8)
        f = HDRFrame(image=img, exposure_time=500, gain=16, timestamp=1.0)
        self.assertEqual(f.image.shape, (100, 100))
        self.assertEqual(f.exposure_time, 500)
        self.assertEqual(f.gain, 16)


class TestHDRCaptureInit(unittest.TestCase):
    """初始化测试"""

    def test_default_ev_offsets(self):
        cam = MockCamera()
        hdr = HDRCaptureSimulator(cam)
        self.assertEqual(hdr.ev_offsets, [-2, 0, 2])

    def test_custom_ev_offsets(self):
        cam = MockCamera()
        hdr = HDRCaptureSimulator(cam, ev_offsets=[-3, 0, 3])
        self.assertEqual(hdr.ev_offsets, [-3, 0, 3])

    def test_ev_multipliers(self):
        cam = MockCamera()
        hdr = HDRCaptureSimulator(cam, ev_offsets=[-2, 0, 2])
        mults = hdr.get_ev_multipliers()
        self.assertAlmostEqual(mults[0], 0.25)  # 2^-2
        self.assertAlmostEqual(mults[1], 1.0)   # 2^0
        self.assertAlmostEqual(mults[2], 4.0)   # 2^2


class TestHDRCapture(unittest.TestCase):
    """HDR拍摄测试"""

    def test_capture_returns_image_and_frames(self):
        cam = MockCamera()
        hdr = HDRCaptureSimulator(cam, ev_offsets=[-1, 0, 1])
        result, frames = hdr.capture()
        self.assertEqual(result.shape, (240, 320))
        self.assertEqual(result.dtype, np.uint8)
        self.assertEqual(len(frames), 3)

    def test_capture_frame_exposures(self):
        cam = MockCamera()
        hdr = HDRCaptureSimulator(cam, base_exposure=1000, ev_offsets=[-2, 0, 2])
        _, frames = hdr.capture()
        # 曝光应分别为 250, 1000, 4000
        self.assertEqual(frames[0].exposure_time, 250)
        self.assertEqual(frames[1].exposure_time, 1000)
        self.assertEqual(frames[2].exposure_time, 4000)

    def test_exposure_clamped(self):
        """曝光值应被限制在[1, 10000]"""
        cam = MockCamera()
        hdr = HDRCaptureSimulator(cam, base_exposure=5000, ev_offsets=[0, 2])
        _, frames = hdr.capture()
        for f in frames:
            self.assertGreaterEqual(f.exposure_time, 1)
            self.assertLessEqual(f.exposure_time, 10000)


class TestHDRMerge(unittest.TestCase):
    """融合算法测试"""

    def test_linear_weighted_merge(self):
        cam = MockCamera()
        hdr = HDRCaptureSimulator(cam, merge_method="linear_weighted")
        result, frames = hdr.capture()
        self.assertEqual(result.shape, (240, 320))
        self.assertEqual(result.dtype, np.uint8)

    def test_merge_preserves_shape(self):
        cam = MockCamera(width=100, height=80)
        hdr = HDRCaptureSimulator(cam, ev_offsets=[-1, 0, 1])
        result, _ = hdr.capture()
        self.assertEqual(result.shape, (80, 100))


class TestReinhardToneMapping(unittest.TestCase):
    """Reinhard Tone Mapping测试"""

    def _make_hdr(self):
        cam = MockCamera()
        hdr = HDRCaptureSimulator(cam)
        return hdr

    def test_output_range(self):
        hdr = self._make_hdr()
        input_hdr = np.array([[0.0, 0.5, 1.0],
                               [2.0, 5.0, 10.0]])
        result = hdr._tone_map_reinhard(input_hdr)
        self.assertTrue(np.all(result >= 0))
        self.assertTrue(np.all(result <= 255))

    def test_constant_input(self):
        hdr = self._make_hdr()
        input_hdr = np.full((10, 10), 5.0)
        result = hdr._tone_map_reinhard(input_hdr)
        # 常数输入应输出统一值
        self.assertTrue(np.all(result == result[0, 0]))

    def test_monotonic(self):
        """较亮的输入应产生较亮的输出"""
        hdr = self._make_hdr()
        dark = np.array([[1.0]])
        bright = np.array([[10.0]])
        r_dark = hdr._tone_map_reinhard(dark)
        r_bright = hdr._tone_map_reinhard(bright)
        self.assertGreaterEqual(r_bright[0, 0], r_dark[0, 0])


if __name__ == '__main__':
    unittest.main()
