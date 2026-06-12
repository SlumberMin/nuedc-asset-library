#!/usr/bin/env python3
"""
多帧降噪单元测试
覆盖: 帧采集、块匹配对齐、帧平移、时域中值/均值融合、SNR提升
注意: 模拟相机，不依赖实际硬件
"""

import sys
import os
import unittest
import time
import numpy as np
from collections import deque
from typing import List, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ── 模拟实现 ──────────────────────────────────────────────────

class MockCameraForDenoise:
    """模拟相机（带噪声）"""
    def __init__(self, width=320, height=240, noise_level=20):
        self.width = width
        self.height = height
        self.noise_level = noise_level
        self._base_image = None

    def set_base_image(self, img):
        self._base_image = img

    def grab_frame(self):
        h, w = self.height, self.width
        if self._base_image is not None:
            base = self._base_image
        else:
            base = np.full((h, w), 128, dtype=np.uint8)
        noise = np.random.randint(-self.noise_level, self.noise_level + 1,
                                  (h, w), dtype=np.int16)
        noisy = np.clip(base.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        return noisy.tobytes(), time.perf_counter() * 1e6


class MultiFrameDenoiseSimulator:
    """多帧降噪模拟"""

    def __init__(self, camera, num_frames=4,
                 align_method="block_match",
                 blend_method="temporal_median",
                 denoise_strength=1.0):
        self.camera = camera
        self.num_frames = min(max(num_frames, 2), 8)
        self.align_method = align_method
        self.blend_method = blend_method
        self.denoise_strength = denoise_strength

    def capture_denoised(self):
        t0 = time.perf_counter()

        raw_frames = []
        for i in range(self.num_frames):
            raw_data, ts = self.camera.grab_frame()
            if raw_data is None:
                continue
            h, w = self.camera.height, self.camera.width
            img = np.frombuffer(raw_data, dtype=np.uint8).reshape(h, w)
            raw_frames.append(img.copy())

        if not raw_frames:
            empty = np.zeros((self.camera.height, self.camera.width), dtype=np.uint8)
            return empty, 0.0

        if len(raw_frames) == 1:
            return raw_frames[0], (time.perf_counter() - t0) * 1000

        reference = raw_frames[len(raw_frames) // 2]
        aligned = self._align_frames(reference, raw_frames)
        result = self._blend(aligned)

        elapsed = (time.perf_counter() - t0) * 1000
        return result, elapsed

    def _align_frames(self, reference, frames):
        if self.align_method == "none":
            return frames

        aligned = [reference]
        for i, frame in enumerate(frames):
            if frame is reference:
                continue
            if self.align_method == "block_match":
                offset = self._block_match_align(reference, frame)
                shifted = self._shift_frame(frame, offset)
                aligned.append(shifted)
            else:
                aligned.append(frame)
        return aligned

    def _block_match_align(self, ref, frame, search_range=8, block_size=32):
        h, w = ref.shape
        cx, cy = w // 2, h // 2
        bs = block_size // 2

        ref_block = ref[cy-bs:cy+bs, cx-bs:cx+bs].astype(np.float64)

        best_sad = float('inf')
        best_dx, best_dy = 0, 0

        for dy in range(-search_range, search_range + 1):
            for dx in range(-search_range, search_range + 1):
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

    def _shift_frame(self, frame, offset):
        dx, dy = offset
        h, w = frame.shape
        result = np.zeros_like(frame)

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

    def _blend(self, frames):
        if self.blend_method == "temporal_median":
            return self._temporal_median(frames)
        elif self.blend_method == "temporal_mean":
            return self._temporal_mean(frames)
        return self._temporal_median(frames)

    def _temporal_median(self, frames):
        stack = np.stack(frames, axis=0)
        return np.median(stack, axis=0).astype(np.uint8)

    def _temporal_mean(self, frames):
        stack = np.stack(frames, axis=0).astype(np.float64)
        return np.mean(stack, axis=0).astype(np.uint8)

    @property
    def stats(self):
        return {
            'num_frames': self.num_frames,
            'align_method': self.align_method,
            'blend_method': self.blend_method,
            'denoise_strength': self.denoise_strength,
            'snr_gain_db': 10 * np.log10(self.num_frames) if self.num_frames > 0 else 0,
        }


# ── 测试用例 ──────────────────────────────────────────────────

class TestDenoiseInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        cam = MockCameraForDenoise()
        dn = MultiFrameDenoiseSimulator(cam)
        self.assertEqual(dn.num_frames, 4)
        self.assertEqual(dn.align_method, "block_match")
        self.assertEqual(dn.blend_method, "temporal_median")

    def test_num_frames_clamped(self):
        cam = MockCameraForDenoise()
        dn = MultiFrameDenoiseSimulator(cam, num_frames=10)
        self.assertEqual(dn.num_frames, 8)
        dn2 = MultiFrameDenoiseSimulator(cam, num_frames=1)
        self.assertEqual(dn2.num_frames, 2)


class TestFrameShift(unittest.TestCase):
    """帧平移测试"""

    def test_shift_zero(self):
        cam = MockCameraForDenoise()
        dn = MultiFrameDenoiseSimulator(cam)
        frame = np.arange(100, dtype=np.uint8).reshape(10, 10)
        result = dn._shift_frame(frame, (0, 0))
        np.testing.assert_array_equal(result, frame)

    def test_shift_right(self):
        cam = MockCameraForDenoise()
        dn = MultiFrameDenoiseSimulator(cam)
        frame = np.ones((10, 10), dtype=np.uint8) * 5
        result = dn._shift_frame(frame, (3, 0))
        # 左边3列应为0，右边7列应为5
        self.assertTrue(np.all(result[:, :3] == 0))
        self.assertTrue(np.all(result[:, 3:] == 5))

    def test_shift_down(self):
        cam = MockCameraForDenoise()
        dn = MultiFrameDenoiseSimulator(cam)
        frame = np.ones((10, 10), dtype=np.uint8) * 5
        result = dn._shift_frame(frame, (0, 2))
        self.assertTrue(np.all(result[:2, :] == 0))
        self.assertTrue(np.all(result[2:, :] == 5))


class TestBlockMatch(unittest.TestCase):
    """块匹配对齐测试"""

    def test_identical_frames_zero_offset(self):
        cam = MockCameraForDenoise()
        dn = MultiFrameDenoiseSimulator(cam)
        frame = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
        dx, dy = dn._block_match_align(frame, frame)
        self.assertEqual(dx, 0)
        self.assertEqual(dy, 0)

    def test_shifted_frame_detected(self):
        cam = MockCameraForDenoise()
        dn = MultiFrameDenoiseSimulator(cam)
        frame = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
        shifted = dn._shift_frame(frame, (3, -2))
        dx, dy = dn._block_match_align(frame, shifted)
        self.assertEqual(dx, 3)
        self.assertEqual(dy, -2)


class TestTemporalFusion(unittest.TestCase):
    """时域融合测试"""

    def test_median_removes_outlier(self):
        """中值滤波应去除异常值"""
        cam = MockCameraForDenoise()
        dn = MultiFrameDenoiseSimulator(cam, blend_method="temporal_median")
        base = np.full((10, 10), 100, dtype=np.uint8)
        outlier = np.full((10, 10), 255, dtype=np.uint8)
        frames = [base, outlier, base.copy(), base.copy()]
        result = dn._temporal_median(frames)
        np.testing.assert_array_equal(result, base)

    def test_mean_reduces_noise(self):
        """均值融合应降低噪声"""
        np.random.seed(42)
        cam = MockCameraForDenoise()
        dn = MultiFrameDenoiseSimulator(cam, blend_method="temporal_mean")
        base = np.full((50, 50), 128, dtype=np.uint8)
        noisy_frames = []
        for _ in range(8):
            noise = np.random.randint(-30, 30, (50, 50), dtype=np.int16)
            noisy = np.clip(base.astype(np.int16) + noise, 0, 255).astype(np.uint8)
            noisy_frames.append(noisy)

        result = dn._temporal_mean(noisy_frames)
        var_input = np.var(noisy_frames[0].astype(np.float64))
        var_output = np.var(result.astype(np.float64))
        self.assertLess(var_output, var_input)


class TestCaptureDenoised(unittest.TestCase):
    """端到端降噪测试"""

    def test_capture_returns_result(self):
        cam = MockCameraForDenoise(width=64, height=48, noise_level=30)
        dn = MultiFrameDenoiseSimulator(cam, num_frames=4)
        result, elapsed = dn.capture_denoised()
        self.assertEqual(result.shape, (48, 64))
        self.assertGreater(elapsed, 0)

    def test_noise_reduction(self):
        """多帧降噪应显著降低噪声"""
        np.random.seed(42)
        base = np.full((64, 64), 128, dtype=np.uint8)
        cam = MockCameraForDenoise(width=64, height=64, noise_level=30)
        cam.set_base_image(base)
        dn = MultiFrameDenoiseSimulator(cam, num_frames=4, align_method="none")
        result, _ = dn.capture_denoised()
        var_result = np.var(result.astype(np.float64))
        self.assertLess(var_result, 100)  # 应比单帧噪声小很多

    def test_single_frame_fallback(self):
        """只有1帧时应直接返回"""
        cam = MockCameraForDenoise()
        # Mock: 只返回1帧
        original_grab = cam.grab_frame
        call_count = [0]
        def once_grab():
            call_count[0] += 1
            if call_count[0] <= 1:
                return original_grab()
            return None, 0.0
        cam.grab_frame = once_grab

        dn = MultiFrameDenoiseSimulator(cam, num_frames=4)
        result, elapsed = dn.capture_denoised()
        self.assertEqual(result.shape, (240, 320))


class TestDenoiseStats(unittest.TestCase):
    """统计信息测试"""

    def test_stats_format(self):
        cam = MockCameraForDenoise()
        dn = MultiFrameDenoiseSimulator(cam, num_frames=4)
        stats = dn.stats
        self.assertEqual(stats['num_frames'], 4)
        self.assertIn('snr_gain_db', stats)

    def test_snr_gain(self):
        cam = MockCameraForDenoise()
        dn = MultiFrameDenoiseSimulator(cam, num_frames=4)
        snr_db = dn.stats['snr_gain_db']
        # 4帧 -> 10*log10(4) ≈ 6dB
        self.assertAlmostEqual(snr_db, 6.0, delta=0.5)


if __name__ == '__main__':
    unittest.main()
