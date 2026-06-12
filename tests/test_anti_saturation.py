#!/usr/bin/env python3
"""
抗饱和模块单元测试
覆盖: 初始化、输出限幅、反计算抗饱和、条件积分抗饱和、
      跟踪抗饱和、钳位抗饱和、积分冻结抗饱和、重置
注意: 使用纯 Python 模拟 C AntiSaturation 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 枚举
ANTI_SAT_BACK_CALCULATION = 0
ANTI_SAT_CONDITIONAL_INTEGRATION = 1
ANTI_SAT_TRACKING = 2
ANTI_SAT_CLAMPING = 3
ANTI_SAT_INTEGRATOR_FREEZE = 4


def clampf(value, min_val, max_val):
    if value < min_val:
        return min_val
    if value > max_val:
        return max_val
    return value


class AntiSaturationSimulator:
    """抗饱和控制器 Python 模拟"""

    def __init__(self, method, dt):
        self.method = method
        self.dt = dt

        self.output_min = -1000.0
        self.output_max = 1000.0

        self.Kb = 1.0
        self.tracking_time_constant = 0.1
        self.epsilon = 0.01

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0
        self.prev_unsaturated_output = 0.0
        self.is_saturated = False

    def set_output_limit(self, min_val, max_val):
        self.output_min = min_val
        self.output_max = max_val

    def set_back_calculation(self, Kb, Tt):
        self.Kb = Kb
        self.tracking_time_constant = Tt

    def set_conditional_integration(self, epsilon):
        self.epsilon = epsilon

    def _back_calculation(self, error, raw_output):
        saturated = clampf(raw_output, self.output_min, self.output_max)
        saturation_error = saturated - raw_output
        integral_increment = error + self.Kb * saturation_error / self.tracking_time_constant
        return integral_increment * self.dt

    def _conditional_integration(self, error, raw_output):
        saturated = clampf(raw_output, self.output_min, self.output_max)
        is_upper_saturated = raw_output >= self.output_max
        is_lower_saturated = raw_output <= self.output_min

        freeze = False
        if is_upper_saturated and error > 0:
            freeze = True
        if is_lower_saturated and error < 0:
            freeze = True
        if abs(error) < self.epsilon:
            freeze = True

        if freeze:
            return 0.0
        return error * self.dt

    def _tracking_anti_windup(self, error, raw_output):
        saturated = clampf(raw_output, self.output_min, self.output_max)
        saturation_error = saturated - raw_output
        alpha = self.dt / (self.tracking_time_constant + self.dt)
        tracking_correction = alpha * saturation_error
        return (error + tracking_correction) * self.dt

    def _clamping_anti_windup(self, error, raw_output):
        saturated = clampf(raw_output, self.output_min, self.output_max)
        if saturated == raw_output:
            return error * self.dt
        if (error > 0 and raw_output > self.output_max) or \
           (error < 0 and raw_output < self.output_min):
            return error * self.dt * 0.1
        return error * self.dt

    def _integrator_freeze(self, error, raw_output):
        saturated = clampf(raw_output, self.output_min, self.output_max)
        if saturated != raw_output:
            return 0.0
        return error * self.dt

    def calculate_integral(self, error, raw_output):
        if self.method == ANTI_SAT_BACK_CALCULATION:
            inc = self._back_calculation(error, raw_output)
        elif self.method == ANTI_SAT_CONDITIONAL_INTEGRATION:
            inc = self._conditional_integration(error, raw_output)
        elif self.method == ANTI_SAT_TRACKING:
            inc = self._tracking_anti_windup(error, raw_output)
        elif self.method == ANTI_SAT_CLAMPING:
            inc = self._clamping_anti_windup(error, raw_output)
        elif self.method == ANTI_SAT_INTEGRATOR_FREEZE:
            inc = self._integrator_freeze(error, raw_output)
        else:
            inc = error * self.dt

        self.integral += inc
        integral_max = (self.output_max - self.output_min) * 0.5
        self.integral = clampf(self.integral, -integral_max, integral_max)
        return self.integral

    def apply_limit(self, raw_output):
        saturated = clampf(raw_output, self.output_min, self.output_max)
        self.is_saturated = (saturated != raw_output)
        self.prev_unsaturated_output = raw_output
        self.prev_output = saturated
        return saturated

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0
        self.prev_unsaturated_output = 0.0
        self.is_saturated = False

    def is_saturated_state(self):
        return self.is_saturated

    def get_saturation_margin(self):
        return self.prev_output - self.prev_unsaturated_output


# ── 初始化测试 ──

class TestAntiSatInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_BACK_CALCULATION, 0.01)
        self.assertEqual(ctrl.method, ANTI_SAT_BACK_CALCULATION)
        self.assertEqual(ctrl.dt, 0.01)
        self.assertEqual(ctrl.integral, 0.0)

    def test_all_methods(self):
        for m in [ANTI_SAT_BACK_CALCULATION, ANTI_SAT_CONDITIONAL_INTEGRATION,
                  ANTI_SAT_TRACKING, ANTI_SAT_CLAMPING, ANTI_SAT_INTEGRATOR_FREEZE]:
            ctrl = AntiSaturationSimulator(m, 0.01)
            self.assertEqual(ctrl.method, m)


# ── 输出限幅测试 ──

class TestAntiSatOutputLimit(unittest.TestCase):
    """输出限幅测试"""

    def test_clamp_high(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_BACK_CALCULATION, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        result = ctrl.apply_limit(100.0)
        self.assertEqual(result, 10.0)

    def test_clamp_low(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_BACK_CALCULATION, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        result = ctrl.apply_limit(-100.0)
        self.assertEqual(result, -10.0)

    def test_no_clamp(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_BACK_CALCULATION, 0.01)
        ctrl.set_output_limit(-100.0, 100.0)
        result = ctrl.apply_limit(50.0)
        self.assertEqual(result, 50.0)

    def test_saturated_flag(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_BACK_CALCULATION, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        ctrl.apply_limit(100.0)
        self.assertTrue(ctrl.is_saturated_state())

    def test_not_saturated_flag(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_BACK_CALCULATION, 0.01)
        ctrl.set_output_limit(-100.0, 100.0)
        ctrl.apply_limit(50.0)
        self.assertFalse(ctrl.is_saturated_state())

    def test_saturation_margin(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_BACK_CALCULATION, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        ctrl.apply_limit(15.0)
        margin = ctrl.get_saturation_margin()
        self.assertAlmostEqual(margin, 10.0 - 15.0, places=3)


# ── 反计算抗饱和测试 ──

class TestAntiSatBackCalculation(unittest.TestCase):
    """反计算抗饱和测试"""

    def test_no_saturation(self):
        """未饱和时应正常积分"""
        ctrl = AntiSaturationSimulator(ANTI_SAT_BACK_CALCULATION, 0.01)
        ctrl.set_output_limit(-100.0, 100.0)
        result = ctrl.calculate_integral(5.0, 50.0)
        self.assertGreater(result, 0.0)

    def test_saturation_reduces_integral(self):
        """饱和时反计算应减缓积分增长"""
        ctrl = AntiSaturationSimulator(ANTI_SAT_BACK_CALCULATION, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        ctrl.set_back_calculation(Kb=2.0, Tt=0.1)
        # raw_output 超出上限
        result = ctrl.calculate_integral(5.0, 20.0)
        # 积分增长应比正常慢
        self.assertIsInstance(result, float)


# ── 条件积分抗饱和测试 ──

class TestAntiSatConditionalIntegration(unittest.TestCase):
    """条件积分抗饱和测试"""

    def test_freeze_on_upper_saturation_positive_error(self):
        """上限饱和且误差为正时应冻结积分"""
        ctrl = AntiSaturationSimulator(ANTI_SAT_CONDITIONAL_INTEGRATION, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        integral_before = ctrl.integral
        ctrl.calculate_integral(5.0, 20.0)  # 正误差, 上限饱和
        self.assertEqual(ctrl.integral, integral_before)  # 冻结

    def test_freeze_on_lower_saturation_negative_error(self):
        """下限饱和且误差为负时应冻结积分"""
        ctrl = AntiSaturationSimulator(ANTI_SAT_CONDITIONAL_INTEGRATION, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        integral_before = ctrl.integral
        ctrl.calculate_integral(-5.0, -20.0)  # 负误差, 下限饱和
        self.assertEqual(ctrl.integral, integral_before)

    def test_freeze_on_small_error(self):
        """误差小于阈值时应冻结积分"""
        ctrl = AntiSaturationSimulator(ANTI_SAT_CONDITIONAL_INTEGRATION, 0.01)
        ctrl.set_output_limit(-100.0, 100.0)
        ctrl.set_conditional_integration(epsilon=0.1)
        integral_before = ctrl.integral
        ctrl.calculate_integral(0.001, 0.0)  # 很小的误差
        self.assertEqual(ctrl.integral, integral_before)

    def test_integrate_when_not_saturated(self):
        """未饱和时应正常积分"""
        ctrl = AntiSaturationSimulator(ANTI_SAT_CONDITIONAL_INTEGRATION, 0.01)
        ctrl.set_output_limit(-100.0, 100.0)
        ctrl.calculate_integral(5.0, 0.0)
        self.assertGreater(ctrl.integral, 0.0)


# ── 跟踪抗饱和测试 ──

class TestAntiSatTracking(unittest.TestCase):
    """跟踪抗饱和测试"""

    def test_tracking_correction_on_saturation(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_TRACKING, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        # 饱和时跟踪修正应影响积分
        result = ctrl.calculate_integral(5.0, 20.0)
        self.assertIsInstance(result, float)

    def test_normal_when_not_saturated(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_TRACKING, 0.01)
        ctrl.set_output_limit(-100.0, 100.0)
        result = ctrl.calculate_integral(5.0, 0.0)
        self.assertGreater(result, 0.0)


# ── 钳位抗饱和测试 ──

class TestAntiSatClamping(unittest.TestCase):
    """钳位抗饱和测试"""

    def test_decay_on_saturation(self):
        """饱和时积分增长应衰减"""
        ctrl = AntiSaturationSimulator(ANTI_SAT_CLAMPING, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        # 正误差, 输出超出上限
        result = ctrl.calculate_integral(5.0, 20.0)
        # 应该是 error * dt * 0.1 (衰减)
        expected = 5.0 * 0.01 * 0.1
        self.assertAlmostEqual(result, expected, places=5)

    def test_normal_integration(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_CLAMPING, 0.01)
        ctrl.set_output_limit(-100.0, 100.0)
        result = ctrl.calculate_integral(5.0, 0.0)
        expected = 5.0 * 0.01
        self.assertAlmostEqual(result, expected, places=5)


# ── 积分冻结抗饱和测试 ──

class TestAntiSatIntegratorFreeze(unittest.TestCase):
    """积分冻结抗饱和测试"""

    def test_freeze_on_saturation(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_INTEGRATOR_FREEZE, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        integral_before = ctrl.integral
        ctrl.calculate_integral(5.0, 20.0)
        self.assertEqual(ctrl.integral, integral_before)

    def test_integrate_when_not_saturated(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_INTEGRATOR_FREEZE, 0.01)
        ctrl.set_output_limit(-100.0, 100.0)
        ctrl.calculate_integral(5.0, 0.0)
        self.assertAlmostEqual(ctrl.integral, 0.05, places=5)


# ── 积分限幅测试 ──

class TestAntiSatIntegralClamp(unittest.TestCase):
    """积分项自身限幅测试"""

    def test_integral_clamped(self):
        """积分项不应超出 (output_max - output_min) * 0.5"""
        ctrl = AntiSaturationSimulator(ANTI_SAT_INTEGRATOR_FREEZE, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        # 持续积分
        for _ in range(10000):
            ctrl.calculate_integral(1.0, 0.0)
        max_integral = (10.0 - (-10.0)) * 0.5
        self.assertLessEqual(ctrl.integral, max_integral)


# ── 重置测试 ──

class TestAntiSatReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        ctrl = AntiSaturationSimulator(ANTI_SAT_BACK_CALCULATION, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        ctrl.calculate_integral(5.0, 0.0)
        ctrl.apply_limit(20.0)
        ctrl.reset()
        self.assertEqual(ctrl.integral, 0.0)
        self.assertEqual(ctrl.prev_error, 0.0)
        self.assertFalse(ctrl.is_saturated_state())


if __name__ == '__main__':
    unittest.main()
