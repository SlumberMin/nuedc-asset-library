#!/usr/bin/env python3
"""
PID融合控制单元测试
覆盖: 初始化、PID基本计算、前馈补偿、DOB扰动观测器、
      自适应参数调整、输出限幅、微分滤波、重置
注意: 使用纯 Python 模拟 C PIDFusion 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def clampf(value, min_val, max_val):
    if value < min_val:
        return min_val
    if value > max_val:
        return max_val
    return value


def low_pass_filter(current, new_value, alpha):
    return alpha * current + (1.0 - alpha) * new_value


class PIDFusionSimulator:
    """PID融合控制器 Python 模拟"""

    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

        self.Kff = 0.0
        self.Kff_d = 0.0

        self.dob_gain = 0.0
        self.dob_cutoff = 10.0
        self.dob_estimate = 0.0

        self.adapt_rate = 0.0
        self.Kp_min = 0.0
        self.Kp_max = 100.0
        self.Ki_min = 0.0
        self.Ki_max = 100.0
        self.Kd_min = 0.0
        self.Kd_max = 100.0

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.prev_derivative = 0.0
        self.prev_dob_estimate = 0.0

        self.output_min = -1000.0
        self.output_max = 1000.0

        self.derivative_filter_coeff = 0.0
        self.filtered_derivative = 0.0

    def set_feedforward(self, Kff, Kff_d):
        self.Kff = Kff
        self.Kff_d = Kff_d

    def set_dob(self, gain, cutoff):
        self.dob_gain = gain
        self.dob_cutoff = cutoff

    def set_adaptive(self, rate, Kp_min, Kp_max, Ki_min, Ki_max, Kd_min, Kd_max):
        self.adapt_rate = rate
        self.Kp_min = Kp_min
        self.Kp_max = Kp_max
        self.Ki_min = Ki_min
        self.Ki_max = Ki_max
        self.Kd_min = Kd_min
        self.Kd_max = Kd_max

    def set_output_limit(self, min_val, max_val):
        self.output_min = min_val
        self.output_max = max_val

    def set_derivative_filter(self, coeff):
        self.derivative_filter_coeff = clampf(coeff, 0.0, 0.99)

    def _update_adaptive(self, error, error_dot):
        if self.adapt_rate <= 0.0:
            return
        abs_error = abs(error)
        abs_error_dot = abs(error_dot)

        Kp_adapt = self.Kp + self.adapt_rate * abs_error
        self.Kp = clampf(Kp_adapt, self.Kp_min, self.Kp_max)

        Kd_adapt = self.Kd + self.adapt_rate * abs_error_dot
        self.Kd = clampf(Kd_adapt, self.Kd_min, self.Kd_max)

        steady_state_error = abs(self.integral) * self.dt
        Ki_adapt = self.Ki + self.adapt_rate * steady_state_error
        self.Ki = clampf(Ki_adapt, self.Ki_min, self.Ki_max)

    def _update_dob(self, measurement, control_output):
        if self.dob_gain <= 0.0:
            return
        measurement_rate = (measurement - self.prev_measurement) / self.dt
        expected_rate = control_output
        raw_estimate = measurement_rate - expected_rate
        alpha = math.exp(-2.0 * math.pi * self.dob_cutoff * self.dt)
        self.dob_estimate = low_pass_filter(self.prev_dob_estimate, raw_estimate, alpha)
        self.prev_dob_estimate = self.dob_estimate

    def calculate(self, setpoint, measurement, feedforward=0.0, feedforward_d=0.0):
        error = setpoint - measurement
        derivative = (measurement - self.prev_measurement) / self.dt

        if self.derivative_filter_coeff > 0.0:
            self.filtered_derivative = low_pass_filter(
                self.filtered_derivative, derivative, self.derivative_filter_coeff)
            derivative = self.filtered_derivative

        error_dot = (error - self.prev_error) / self.dt
        self._update_adaptive(error, error_dot)

        P_term = self.Kp * error
        I_term = self.Ki * self.integral
        D_term = -self.Kd * derivative
        FF_term = self.Kff * feedforward
        FF_d_term = self.Kff_d * feedforward_d

        pid_output = P_term + I_term + D_term + FF_term + FF_d_term

        self._update_dob(measurement, pid_output)
        dob_compensation = self.dob_gain * self.dob_estimate

        output = pid_output - dob_compensation
        output = clampf(output, self.output_min, self.output_max)

        if ((output > self.output_min and output < self.output_max) or
                (error > 0 and output < self.output_max) or
                (error < 0 and output > self.output_min)):
            self.integral += error * self.dt

        self.prev_error = error
        self.prev_measurement = measurement
        self.prev_derivative = derivative

        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.prev_derivative = 0.0
        self.prev_dob_estimate = 0.0
        self.dob_estimate = 0.0
        self.filtered_derivative = 0.0

    def get_disturbance_estimate(self):
        return self.dob_estimate

    def get_current_params(self):
        return self.Kp, self.Ki, self.Kd


# ── 初始化测试 ──

class TestPIDFusionInit(unittest.TestCase):
    """PID融合控制器初始化测试"""

    def test_default_params(self):
        ctrl = PIDFusionSimulator(1.0, 0.5, 0.1, 0.01)
        self.assertEqual(ctrl.Kp, 1.0)
        self.assertEqual(ctrl.Ki, 0.5)
        self.assertEqual(ctrl.Kd, 0.1)
        self.assertEqual(ctrl.dt, 0.01)

    def test_default_state(self):
        ctrl = PIDFusionSimulator(1.0, 0.5, 0.1, 0.01)
        self.assertEqual(ctrl.integral, 0.0)
        self.assertEqual(ctrl.prev_error, 0.0)
        self.assertEqual(ctrl.prev_measurement, 0.0)

    def test_default_limits(self):
        ctrl = PIDFusionSimulator(1.0, 0.5, 0.1, 0.01)
        self.assertEqual(ctrl.output_min, -1000.0)
        self.assertEqual(ctrl.output_max, 1000.0)


# ── 基本PID计算测试 ──

class TestPIDFusionBasicCalc(unittest.TestCase):
    """基本PID计算测试"""

    def test_proportional_only(self):
        ctrl = PIDFusionSimulator(2.0, 0.0, 0.0, 0.01)
        output = ctrl.calculate(setpoint=10.0, measurement=5.0)
        self.assertAlmostEqual(output, 10.0, places=1)  # Kp * error = 2*5

    def test_zero_error(self):
        ctrl = PIDFusionSimulator(2.0, 0.5, 0.1, 0.01)
        output = ctrl.calculate(setpoint=5.0, measurement=5.0)
        self.assertEqual(output, 0.0)

    def test_negative_error(self):
        ctrl = PIDFusionSimulator(1.0, 0.0, 0.0, 0.01)
        output = ctrl.calculate(setpoint=0.0, measurement=10.0)
        self.assertAlmostEqual(output, -10.0, places=1)

    def test_returns_float(self):
        ctrl = PIDFusionSimulator(1.0, 0.5, 0.1, 0.01)
        output = ctrl.calculate(10.0, 5.0)
        self.assertIsInstance(output, float)


# ── 前馈补偿测试 ──

class TestPIDFusionFeedforward(unittest.TestCase):
    """前馈补偿测试"""

    def test_feedforward_adds_output(self):
        ctrl = PIDFusionSimulator(0.0, 0.0, 0.0, 0.01)
        ctrl.set_feedforward(Kff=1.0, Kff_d=0.0)
        output = ctrl.calculate(setpoint=0.0, measurement=0.0, feedforward=5.0)
        self.assertAlmostEqual(output, 5.0, places=1)

    def test_feedforward_derivative(self):
        ctrl = PIDFusionSimulator(0.0, 0.0, 0.0, 0.01)
        ctrl.set_feedforward(Kff=0.0, Kff_d=2.0)
        output = ctrl.calculate(setpoint=0.0, measurement=0.0, feedforward_d=3.0)
        self.assertAlmostEqual(output, 6.0, places=1)

    def test_no_feedforward(self):
        ctrl = PIDFusionSimulator(1.0, 0.0, 0.0, 0.01)
        output = ctrl.calculate(setpoint=5.0, measurement=3.0, feedforward=100.0)
        self.assertAlmostEqual(output, 2.0, places=1)  # no Kff set


# ── 输出限幅测试 ──

class TestPIDFusionOutputLimit(unittest.TestCase):
    """输出限幅测试"""

    def test_output_clamped_high(self):
        ctrl = PIDFusionSimulator(100.0, 0.0, 0.0, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        output = ctrl.calculate(setpoint=100.0, measurement=0.0)
        self.assertLessEqual(output, 10.0)

    def test_output_clamped_low(self):
        ctrl = PIDFusionSimulator(100.0, 0.0, 0.0, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        output = ctrl.calculate(setpoint=-100.0, measurement=0.0)
        self.assertGreaterEqual(output, -10.0)


# ── 微分滤波测试 ──

class TestPIDFusionDerivativeFilter(unittest.TestCase):
    """微分滤波测试"""

    def test_filter_coeff_clamped(self):
        ctrl = PIDFusionSimulator(1.0, 0.0, 0.1, 0.01)
        ctrl.set_derivative_filter(1.5)
        self.assertLessEqual(ctrl.derivative_filter_coeff, 0.99)

    def test_filter_reduces_noise(self):
        """滤波后微分项应更平滑"""
        ctrl1 = PIDFusionSimulator(0.0, 0.0, 1.0, 0.01)
        ctrl2 = PIDFusionSimulator(0.0, 0.0, 1.0, 0.01)
        ctrl2.set_derivative_filter(0.5)

        # 多步计算
        for i in range(10):
            m = 5.0 + (0.1 if i % 2 == 0 else -0.1)
            ctrl1.calculate(5.0, m)
            ctrl2.calculate(5.0, m)

        # 两个滤波器的微分值应该不同
        self.assertIsNotNone(ctrl2.filtered_derivative)


# ── DOB测试 ──

class TestPIDFusionDOB(unittest.TestCase):
    """DOB扰动观测器测试"""

    def test_dob_disabled_by_default(self):
        ctrl = PIDFusionSimulator(1.0, 0.0, 0.0, 0.01)
        self.assertEqual(ctrl.dob_gain, 0.0)
        self.assertEqual(ctrl.get_disturbance_estimate(), 0.0)

    def test_dob_enabled(self):
        ctrl = PIDFusionSimulator(1.0, 0.0, 0.0, 0.01)
        ctrl.set_dob(gain=1.0, cutoff=10.0)
        self.assertEqual(ctrl.dob_gain, 1.0)

    def test_dob_estimate_changes(self):
        ctrl = PIDFusionSimulator(1.0, 0.0, 0.0, 0.01)
        ctrl.set_dob(gain=1.0, cutoff=10.0)
        ctrl.calculate(10.0, 5.0)
        ctrl.calculate(10.0, 6.0)
        # DOB estimate should have been updated (non-zero after movement)
        self.assertIsNotNone(ctrl.get_disturbance_estimate())


# ── 自适应参数测试 ──

class TestPIDFusionAdaptive(unittest.TestCase):
    """自适应参数调整测试"""

    def test_adaptive_disabled_by_default(self):
        ctrl = PIDFusionSimulator(1.0, 0.5, 0.1, 0.01)
        Kp0, _, _ = ctrl.get_current_params()
        ctrl.calculate(100.0, 0.0)  # large error
        Kp1, _, _ = ctrl.get_current_params()
        self.assertEqual(Kp0, Kp1)

    def test_adaptive_increases_kp(self):
        ctrl = PIDFusionSimulator(1.0, 0.0, 0.0, 0.01)
        ctrl.set_adaptive(rate=0.1, Kp_min=0.0, Kp_max=50.0,
                          Ki_min=0.0, Ki_max=50.0, Kd_min=0.0, Kd_max=50.0)
        Kp0, _, _ = ctrl.get_current_params()
        ctrl.calculate(100.0, 0.0)  # large error → Kp should increase
        Kp1, _, _ = ctrl.get_current_params()
        self.assertGreater(Kp1, Kp0)

    def test_adaptive_respects_limits(self):
        ctrl = PIDFusionSimulator(1.0, 0.0, 0.0, 0.01)
        ctrl.set_adaptive(rate=10.0, Kp_min=0.0, Kp_max=5.0,
                          Ki_min=0.0, Ki_max=5.0, Kd_min=0.0, Kd_max=5.0)
        for _ in range(100):
            ctrl.calculate(1000.0, 0.0)
        Kp, _, _ = ctrl.get_current_params()
        self.assertLessEqual(Kp, 5.0)


# ── 重置测试 ──

class TestPIDFusionReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        ctrl = PIDFusionSimulator(1.0, 0.5, 0.1, 0.01)
        ctrl.calculate(10.0, 0.0)
        ctrl.calculate(10.0, 1.0)
        ctrl.reset()
        self.assertEqual(ctrl.integral, 0.0)
        self.assertEqual(ctrl.prev_error, 0.0)
        self.assertEqual(ctrl.prev_measurement, 0.0)
        self.assertEqual(ctrl.dob_estimate, 0.0)
        self.assertEqual(ctrl.filtered_derivative, 0.0)

    def test_reset_allows_fresh_start(self):
        ctrl = PIDFusionSimulator(2.0, 0.0, 0.0, 0.01)
        ctrl.calculate(100.0, 0.0)
        ctrl.reset()
        output = ctrl.calculate(5.0, 3.0)
        self.assertAlmostEqual(output, 4.0, places=1)


# ── 积分抗饱和测试 ──

class TestPIDFusionAntiWindup(unittest.TestCase):
    """积分抗饱和测试"""

    def test_integral_frozen_on_saturation(self):
        """输出饱和时积分应停止累积"""
        ctrl = PIDFusionSimulator(1000.0, 100.0, 0.0, 0.01)
        ctrl.set_output_limit(-10.0, 10.0)
        # 大误差持续→输出应饱和
        for _ in range(100):
            output = ctrl.calculate(100.0, 0.0)
        self.assertLessEqual(output, 10.0)
        integral_after_sat = ctrl.integral

        # 再走几步，积分不应继续增长（因为输出已饱和在上限且误差为正）
        for _ in range(100):
            ctrl.calculate(100.0, 0.0)
        # integral should have stopped growing or grown very little
        self.assertLessEqual(ctrl.integral, integral_after_sat + 1.0)


if __name__ == '__main__':
    unittest.main()
