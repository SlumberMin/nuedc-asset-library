#!/usr/bin/env python3
"""
数学工具单元测试
覆盖: 限幅(Clamp)、映射(Map)、死区(DeadZone)、滑动平均(MovingAvg)、低通滤波(LowPassFilter)
注意: 使用纯 Python 模拟 C math_utils 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 ──────────────────────────────────────────

def clamp_f(value, lo, hi):
    return max(lo, min(hi, value))


def clamp_i(value, lo, hi):
    return max(lo, min(hi, value))


def map_f(value, in_min, in_max, out_min, out_max):
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)


def dead_zone_f(value, dz):
    return 0.0 if abs(value) < dz else value


def dead_zone_compensate_f(value, dz):
    if abs(value) < dz:
        return 0.0
    return value - math.copysign(dz, value)


class MovingAvgSimulator:
    """模拟滑动平均滤波器"""

    def __init__(self, size=8):
        self.size = min(size, 64)
        self.buffer = [0.0] * self.size
        self.index = 0
        self.count = 0
        self.sum = 0.0

    def update(self, value):
        if self.count < self.size:
            self.buffer[self.index] = value
            self.sum += value
            self.count += 1
        else:
            self.sum -= self.buffer[self.index]
            self.buffer[self.index] = value
            self.sum += value
        self.index = (self.index + 1) % self.size
        return self.sum / self.count

    def get_value(self):
        if self.count == 0:
            return 0.0
        return self.sum / self.count

    def reset(self):
        self.buffer = [0.0] * self.size
        self.index = 0
        self.count = 0
        self.sum = 0.0


class LowPassFilterSimulator:
    """模拟一阶低通滤波器"""

    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.filtered = 0.0
        self.initialized = False

    def update(self, value):
        if not self.initialized:
            self.filtered = value
            self.initialized = True
        else:
            self.filtered = self.alpha * value + (1 - self.alpha) * self.filtered
        return self.filtered

    def get_value(self):
        return self.filtered

    def reset(self):
        self.filtered = 0.0
        self.initialized = False


# ── 测试用例 ──────────────────────────────────────────────────

class TestClampF(unittest.TestCase):
    """浮点限幅测试"""

    def test_within_range(self):
        self.assertAlmostEqual(clamp_f(5, 0, 10), 5.0)

    def test_below_min(self):
        self.assertAlmostEqual(clamp_f(-5, 0, 10), 0.0)

    def test_above_max(self):
        self.assertAlmostEqual(clamp_f(15, 0, 10), 10.0)

    def test_at_min(self):
        self.assertAlmostEqual(clamp_f(0, 0, 10), 0.0)

    def test_at_max(self):
        self.assertAlmostEqual(clamp_f(10, 0, 10), 10.0)

    def test_negative_range(self):
        self.assertAlmostEqual(clamp_f(-5, -10, -1), -5.0)

    def test_large_value(self):
        self.assertAlmostEqual(clamp_f(1e6, -100, 100), 100.0)


class TestClampI(unittest.TestCase):
    """整数限幅测试"""

    def test_within_range(self):
        self.assertEqual(clamp_i(5, 0, 10), 5)

    def test_below_min(self):
        self.assertEqual(clamp_i(-5, 0, 10), 0)

    def test_above_max(self):
        self.assertEqual(clamp_i(15, 0, 10), 10)


class TestMapF(unittest.TestCase):
    """线性映射测试"""

    def test_basic_mapping(self):
        self.assertAlmostEqual(map_f(5, 0, 10, 0, 100), 50.0)

    def test_full_range(self):
        self.assertAlmostEqual(map_f(0, 0, 10, 0, 100), 0.0)
        self.assertAlmostEqual(map_f(10, 0, 10, 0, 100), 100.0)

    def test_reverse_mapping(self):
        """输出范围反向"""
        self.assertAlmostEqual(map_f(0, 0, 10, 100, 0), 100.0)
        self.assertAlmostEqual(map_f(10, 0, 10, 100, 0), 0.0)

    def test_adc_to_voltage(self):
        """ADC(0~4095) → 电压(0~3.3V)"""
        self.assertAlmostEqual(map_f(2048, 0, 4095, 0, 3.3), 1.65, places=2)

    def test_extrapolation(self):
        """超出范围应外推"""
        self.assertAlmostEqual(map_f(20, 0, 10, 0, 100), 200.0)


class TestDeadZoneF(unittest.TestCase):
    """死区测试"""

    def test_in_deadzone(self):
        self.assertAlmostEqual(dead_zone_f(2.0, 5.0), 0.0)

    def test_outside_deadzone_positive(self):
        self.assertAlmostEqual(dead_zone_f(10.0, 5.0), 10.0)

    def test_outside_deadzone_negative(self):
        self.assertAlmostEqual(dead_zone_f(-10.0, 5.0), -10.0)

    def test_boundary(self):
        """恰好等于死区阈值: 不在死区内(abs(5)==5, 不 < 5)"""
        self.assertAlmostEqual(dead_zone_f(5.0, 5.0), 5.0)

    def test_negative_in_deadzone(self):
        self.assertAlmostEqual(dead_zone_f(-3.0, 5.0), 0.0)


class TestDeadZoneCompensate(unittest.TestCase):
    """死区补偿测试"""

    def test_in_deadzone(self):
        self.assertAlmostEqual(dead_zone_compensate_f(2.0, 5.0), 0.0)

    def test_positive_outside(self):
        """正方向出死区: value - dz"""
        self.assertAlmostEqual(dead_zone_compensate_f(10.0, 5.0), 5.0)

    def test_negative_outside(self):
        """负方向出死区: value + dz"""
        self.assertAlmostEqual(dead_zone_compensate_f(-10.0, 5.0), -5.0)


class TestMovingAvg(unittest.TestCase):
    """滑动平均滤波器测试"""

    def test_initial_value(self):
        """首次输入应直接返回"""
        avg = MovingAvgSimulator(size=4)
        self.assertAlmostEqual(avg.update(10.0), 10.0)

    def test_average_of_two(self):
        """两次输入的平均"""
        avg = MovingAvgSimulator(size=4)
        avg.update(10.0)
        self.assertAlmostEqual(avg.update(20.0), 15.0)

    def test_window_fill(self):
        """窗口填满后滑动"""
        avg = MovingAvgSimulator(size=3)
        avg.update(1.0)
        avg.update(2.0)
        v = avg.update(3.0)
        self.assertAlmostEqual(v, 2.0)  # (1+2+3)/3

    def test_sliding_window(self):
        """窗口滑动后旧数据被替换"""
        avg = MovingAvgSimulator(size=3)
        avg.update(1.0)
        avg.update(2.0)
        avg.update(3.0)
        v = avg.update(4.0)
        # 窗口: [4, 2, 3] → 3.0
        self.assertAlmostEqual(v, 3.0)

    def test_constant_input(self):
        """常数输入应返回常数"""
        avg = MovingAvgSimulator(size=5)
        for _ in range(20):
            v = avg.update(42.0)
        self.assertAlmostEqual(v, 42.0)

    def test_reset(self):
        """重置后应重新开始"""
        avg = MovingAvgSimulator(size=4)
        avg.update(10.0)
        avg.update(20.0)
        avg.reset()
        self.assertEqual(avg.count, 0)
        self.assertAlmostEqual(avg.update(5.0), 5.0)

    def test_noise_reduction(self):
        """滤波应减少噪声"""
        import random
        random.seed(42)
        avg = MovingAvgSimulator(size=10)
        values = [100 + random.gauss(0, 10) for _ in range(50)]
        filtered = [avg.update(v) for v in values]
        # 滤波后方差应更小
        import statistics
        var_raw = statistics.variance(values[10:])
        var_filtered = statistics.variance(filtered[10:])
        self.assertLess(var_filtered, var_raw)


class TestLowPassFilter(unittest.TestCase):
    """一阶低通滤波器测试"""

    def test_first_value_passthrough(self):
        """首次输入应直接通过"""
        lpf = LowPassFilterSimulator(alpha=0.5)
        self.assertAlmostEqual(lpf.update(100.0), 100.0)

    def test_convergence(self):
        """应逐渐收敛到输入值"""
        lpf = LowPassFilterSimulator(alpha=0.5)
        lpf.update(0.0)
        for _ in range(50):
            v = lpf.update(100.0)
        self.assertGreater(v, 90.0)

    def test_strong_filter_slow_response(self):
        """小alpha(强滤波)应响应更慢"""
        lpf_fast = LowPassFilterSimulator(alpha=0.9)
        lpf_slow = LowPassFilterSimulator(alpha=0.1)
        lpf_fast.update(0.0)
        lpf_slow.update(0.0)
        for _ in range(5):
            v_fast = lpf_fast.update(100.0)
            v_slow = lpf_slow.update(100.0)
        self.assertGreater(v_fast, v_slow)

    def test_constant_input(self):
        """常数输入最终应完全收敛"""
        lpf = LowPassFilterSimulator(alpha=0.3)
        for _ in range(100):
            v = lpf.update(50.0)
        self.assertAlmostEqual(v, 50.0, places=3)

    def test_reset(self):
        """重置后应重新初始化"""
        lpf = LowPassFilterSimulator(alpha=0.5)
        lpf.update(100.0)
        lpf.reset()
        self.assertFalse(lpf.initialized)
        self.assertAlmostEqual(lpf.update(50.0), 50.0)

    def test_noise_smoothing(self):
        """应平滑噪声"""
        import random
        random.seed(42)
        lpf = LowPassFilterSimulator(alpha=0.3)
        raw = [100 + random.gauss(0, 20) for _ in range(100)]
        filtered = [lpf.update(v) for v in raw]
        import statistics
        var_raw = statistics.variance(raw[10:])
        var_filtered = statistics.variance(filtered[10:])
        self.assertLess(var_filtered, var_raw)


if __name__ == '__main__':
    unittest.main()
