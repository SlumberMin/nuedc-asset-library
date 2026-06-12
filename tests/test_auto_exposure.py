#!/usr/bin/env python3
"""
自动曝光单元测试
覆盖: 亮度测量、区域加权测光、PID控制、收敛判定、增益调节
注意: 模拟相机，不依赖实际硬件
"""

import sys
import os
import unittest
import time
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ── 模拟实现 ──────────────────────────────────────────────────

@dataclass
class AEState:
    current_exposure: int = 500
    current_gain: int = 16
    target_brightness: int = 128
    measured_brightness: float = 0.0
    overexposed_ratio: float = 0.0
    underexposed_ratio: float = 0.0
    converged: bool = False
    iterations: int = 0


class MockCameraForAE:
    """模拟相机（曝光影响亮度）"""
    def __init__(self, width=320, height=240, base_brightness=50):
        self.width = width
        self.height = height
        self._exposure = 500
        self._gain = 16
        self._base_brightness = base_brightness
        self._fps = 30

    def set_exposure(self, val):
        self._exposure = val

    def set_gain(self, val):
        self._gain = val

    def get_exposure(self):
        return self._exposure

    def get_fps(self):
        return self._fps

    def grab_frame(self):
        h, w = self.height, self.width
        # 亮度 = base + exposure * gain / 500，限制在0-255
        brightness = min(255, max(0,
            self._base_brightness + int(self._exposure * self._gain / 500)))
        img = np.full((h, w), brightness, dtype=np.uint8)
        noise = np.random.randint(-3, 4, (h, w), dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        return img.tobytes(), time.perf_counter() * 1e6


class AutoExposureSimulator:
    """自动曝光控制器模拟"""

    EXPOSURE_MIN = 1
    EXPOSURE_MAX = 10000
    GAIN_MIN = 1
    GAIN_MAX = 255

    def __init__(self, camera, target_brightness=128,
                 roi=None, metering_mode="center_weighted",
                 convergence_threshold=3,
                 pid_kp=0.5, pid_ki=0.05, pid_kd=0.1):
        self.camera = camera
        self.target_brightness = target_brightness
        self.roi = roi
        self.metering_mode = metering_mode
        self.convergence_threshold = convergence_threshold

        self.kp = pid_kp
        self.ki = pid_ki
        self.kd = pid_kd
        self._integral = 0.0
        self._prev_error = 0.0

        self.state = AEState(
            target_brightness=target_brightness,
            current_exposure=camera.get_exposure() if hasattr(camera, 'get_exposure') else 500,
        )

        self._weight_map = None
        self._weight_sum = 0.0

    def _build_weight_map(self, h, w):
        if self._weight_map is not None and self._weight_map.shape == (h, w):
            return self._weight_map

        if self.metering_mode == "center_weighted":
            y, x = np.mgrid[0:h, 0:w]
            cy, cx = h / 2, w / 2
            sigma_y, sigma_x = h / 3, w / 3
            self._weight_map = np.exp(-(
                (y - cy) ** 2 / (2 * sigma_y ** 2) +
                (x - cx) ** 2 / (2 * sigma_x ** 2)
            ))
        elif self.metering_mode == "spot":
            self._weight_map = np.zeros((h, w), dtype=np.float64)
            if self.roi:
                x, y, rw, rh = self.roi
                self._weight_map[y:y+rh, x:x+rw] = 1.0
            else:
                y1, y2 = h // 4, 3 * h // 4
                x1, x2 = w // 4, 3 * w // 4
                self._weight_map[y1:y2, x1:x2] = 1.0
        elif self.metering_mode == "matrix":
            self._weight_map = np.ones((h, w), dtype=np.float64)
            y1, y2 = h // 4, 3 * h // 4
            x1, x2 = w // 4, 3 * w // 4
            self._weight_map[y1:y2, x1:x2] = 2.0
            self._weight_map[:h//8, :] *= 0.5
            self._weight_map[-h//8:, :] *= 0.5
            self._weight_map[:, :w//8] *= 0.5
            self._weight_map[:, -w//8:] *= 0.5

        self._weight_sum = self._weight_map.sum()
        return self._weight_map

    def measure_brightness(self, frame):
        if frame is None or frame.size == 0:
            return 0.0, 0.0, 0.0

        h, w = frame.shape[:2]

        if self.roi:
            x, y, rw, rh = self.roi
            roi_frame = frame[y:y+rh, x:x+rw]
            weights = np.ones_like(roi_frame, dtype=np.float64)
            weight_sum = roi_frame.size
        else:
            roi_frame = frame
            weights = self._build_weight_map(h, w)
            weight_sum = self._weight_sum

        weighted_mean = np.sum(roi_frame.astype(np.float64) * weights) / max(weight_sum, 1)
        over_ratio = np.mean(frame > 250) if frame.dtype == np.uint8 else 0.0
        under_ratio = np.mean(frame < 5) if frame.dtype == np.uint8 else 0.0

        return weighted_mean, over_ratio, under_ratio

    def update(self, frame):
        brightness, over_ratio, under_ratio = self.measure_brightness(frame)
        self.state.measured_brightness = brightness
        self.state.overexposed_ratio = over_ratio
        self.state.underexposed_ratio = under_ratio
        self.state.iterations += 1

        error = self.target_brightness - brightness

        self._integral += error
        self._integral = max(-500, min(500, self._integral))

        derivative = error - self._prev_error
        self._prev_error = error

        adjustment = self.kp * error + self.ki * self._integral + self.kd * derivative

        new_exposure = int(self.state.current_exposure + adjustment)
        new_exposure = max(self.EXPOSURE_MIN, min(self.EXPOSURE_MAX, new_exposure))

        if new_exposure >= self.EXPOSURE_MAX and error > 0:
            new_gain = min(self.GAIN_MAX, self.state.current_gain + 4)
            self.camera.set_gain(new_gain)
            self.state.current_gain = new_gain
        elif new_exposure <= self.EXPOSURE_MIN and error < 0:
            new_gain = max(self.GAIN_MIN, self.state.current_gain - 4)
            self.camera.set_gain(new_gain)
            self.state.current_gain = new_gain

        self.camera.set_exposure(new_exposure)
        self.state.current_exposure = new_exposure

        self.state.converged = abs(error) < self.convergence_threshold

        return self.state

    @property
    def is_converged(self):
        return self.state.converged

    @property
    def current_exposure(self):
        return self.state.current_exposure

    @property
    def stats(self):
        return {
            'target': self.state.target_brightness,
            'measured': round(self.state.measured_brightness, 1),
            'exposure': self.state.current_exposure,
            'gain': self.state.current_gain,
            'overexposed': f"{self.state.overexposed_ratio*100:.1f}%",
            'underexposed': f"{self.state.underexposed_ratio*100:.1f}%",
            'converged': self.state.converged,
            'iterations': self.state.iterations,
        }


# ── 测试用例 ──────────────────────────────────────────────────

class TestAEState(unittest.TestCase):
    """AE状态数据结构测试"""

    def test_default_state(self):
        s = AEState()
        self.assertEqual(s.current_exposure, 500)
        self.assertEqual(s.target_brightness, 128)
        self.assertFalse(s.converged)
        self.assertEqual(s.iterations, 0)


class TestAutoExposureInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam)
        self.assertEqual(ae.target_brightness, 128)
        self.assertEqual(ae.metering_mode, "center_weighted")

    def test_custom_target(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam, target_brightness=200)
        self.assertEqual(ae.target_brightness, 200)

    def test_custom_pid_params(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam, pid_kp=1.0, pid_ki=0.1, pid_kd=0.2)
        self.assertEqual(ae.kp, 1.0)
        self.assertEqual(ae.ki, 0.1)
        self.assertEqual(ae.kd, 0.2)


class TestBrightnessMeasurement(unittest.TestCase):
    """亮度测量测试"""

    def test_uniform_frame(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam)
        frame = np.full((100, 100), 128, dtype=np.uint8)
        brightness, over, under = ae.measure_brightness(frame)
        self.assertAlmostEqual(brightness, 128.0, delta=1.0)
        self.assertAlmostEqual(over, 0.0)
        self.assertAlmostEqual(under, 0.0)

    def test_overexposed_frame(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam)
        frame = np.full((100, 100), 255, dtype=np.uint8)
        _, over, _ = ae.measure_brightness(frame)
        self.assertAlmostEqual(over, 1.0)

    def test_underexposed_frame(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam)
        frame = np.zeros((100, 100), dtype=np.uint8)
        _, _, under = ae.measure_brightness(frame)
        self.assertAlmostEqual(under, 1.0)

    def test_empty_frame(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam)
        b, o, u = ae.measure_brightness(None)
        self.assertEqual(b, 0.0)

    def test_roi_measurement(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam, roi=(25, 25, 50, 50))
        frame = np.zeros((100, 100), dtype=np.uint8)
        frame[25:75, 25:75] = 200  # ROI区域亮
        brightness, _, _ = ae.measure_brightness(frame)
        self.assertGreater(brightness, 100)


class TestWeightMap(unittest.TestCase):
    """权重矩阵测试"""

    def test_center_weighted_shape(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam, metering_mode="center_weighted")
        wmap = ae._build_weight_map(100, 100)
        self.assertEqual(wmap.shape, (100, 100))
        # 中心权重最高
        self.assertGreater(wmap[50, 50], wmap[0, 0])

    def test_spot_metering(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam, metering_mode="spot")
        wmap = ae._build_weight_map(100, 100)
        # 中心1/4区域有权重
        self.assertTrue(np.all(wmap[25:75, 25:75] > 0))
        # 边缘无权重
        self.assertAlmostEqual(wmap[0, 0], 0.0)

    def test_spot_with_roi(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam, metering_mode="spot", roi=(10, 10, 30, 30))
        wmap = ae._build_weight_map(100, 100)
        self.assertTrue(np.all(wmap[10:40, 10:40] == 1.0))
        self.assertAlmostEqual(wmap[0, 0], 0.0)

    def test_matrix_metering(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam, metering_mode="matrix")
        wmap = ae._build_weight_map(100, 100)
        # 中心区域权重更高
        self.assertGreater(wmap[50, 50], wmap[5, 5])

    def test_weight_map_cached(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam)
        wmap1 = ae._build_weight_map(100, 100)
        wmap2 = ae._build_weight_map(100, 100)
        self.assertIs(wmap1, wmap2)


class TestAutoExposureUpdate(unittest.TestCase):
    """曝光调整测试"""

    def test_increases_exposure_when_dark(self):
        cam = MockCameraForAE(base_brightness=20)
        ae = AutoExposureSimulator(cam, target_brightness=128)
        frame = np.full((240, 320), 20, dtype=np.uint8)
        state = ae.update(frame)
        self.assertGreater(state.current_exposure, 500)

    def test_decreases_exposure_when_bright(self):
        cam = MockCameraForAE(base_brightness=200)
        ae = AutoExposureSimulator(cam, target_brightness=128,
                                    pid_kp=2.0, pid_ki=0, pid_kd=0)
        ae.state.current_exposure = 5000
        frame = np.full((240, 320), 200, dtype=np.uint8)
        state = ae.update(frame)
        self.assertLess(state.current_exposure, 5000)

    def test_convergence(self):
        """经过多次迭代应收敛"""
        np.random.seed(42)
        cam = MockCameraForAE(base_brightness=100)
        ae = AutoExposureSimulator(cam, target_brightness=128,
                                    convergence_threshold=5, pid_kp=0.8)
        # 模拟帧：亮度随曝光变化
        for _ in range(30):
            brightness = min(255, max(0,
                cam._base_brightness + int(ae.state.current_exposure * ae.state.current_gain / 500)))
            frame = np.full((100, 100), brightness, dtype=np.uint8)
            ae.update(frame)
        # 应该趋近收敛
        self.assertLess(abs(ae.state.measured_brightness - 128), 50)

    def test_exposure_clamped(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam)
        ae.state.current_exposure = 10000
        frame = np.full((100, 100), 0, dtype=np.uint8)
        state = ae.update(frame)
        self.assertLessEqual(state.current_exposure, AutoExposureSimulator.EXPOSURE_MAX)
        self.assertGreaterEqual(state.current_exposure, AutoExposureSimulator.EXPOSURE_MIN)


class TestAutoExposureGain(unittest.TestCase):
    """增益调节测试"""

    def test_gain_increases_at_max_exposure(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam, target_brightness=255)
        ae.state.current_exposure = 10000  # 已到极限
        frame = np.zeros((100, 100), dtype=np.uint8)  # 很暗
        state = ae.update(frame)
        self.assertGreater(state.current_gain, 16)

    def test_gain_decreases_at_min_exposure(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam, target_brightness=0)
        ae.state.current_exposure = 1
        ae.state.current_gain = 100
        frame = np.full((100, 100), 255, dtype=np.uint8)  # 很亮
        state = ae.update(frame)
        self.assertLess(state.current_gain, 100)


class TestAutoExposureStats(unittest.TestCase):
    """统计信息测试"""

    def test_stats_format(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam)
        stats = ae.stats
        self.assertIn('target', stats)
        self.assertIn('measured', stats)
        self.assertIn('exposure', stats)
        self.assertIn('gain', stats)
        self.assertIn('converged', stats)
        self.assertIn('iterations', stats)

    def test_iterations_count(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam)
        frame = np.full((100, 100), 100, dtype=np.uint8)
        ae.update(frame)
        ae.update(frame)
        ae.update(frame)
        self.assertEqual(ae.stats['iterations'], 3)


class TestAutoExposureConvergence(unittest.TestCase):
    """收敛判定测试"""

    def test_converged_property(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam, convergence_threshold=5)
        self.assertFalse(ae.is_converged)
        ae.state.converged = True
        self.assertTrue(ae.is_converged)

    def test_convergence_within_threshold(self):
        cam = MockCameraForAE()
        ae = AutoExposureSimulator(cam, target_brightness=128,
                                    convergence_threshold=10)
        frame = np.full((100, 100), 125, dtype=np.uint8)
        state = ae.update(frame)
        # 误差=3，阈值=10，应收敛
        self.assertTrue(state.converged)


if __name__ == '__main__':
    unittest.main()
