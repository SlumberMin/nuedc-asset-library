#!/usr/bin/env python3
"""
自适应PID单元测试
覆盖: 初始化、梯度下降法自适应、MIT规则自适应、参数范围限制、
      学习率设置、死区、重置、参数获取、收敛性
注意: 使用纯 Python 模拟 C AdaptivePID 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 自适应方法枚举
ADAPTIVE_GRADIENT = 0
ADAPTIVE_MIT = 1


def _clamp(val, min_val, max_val):
    if val < min_val:
        return min_val
    if val > max_val:
        return max_val
    return val


class AdaptivePIDSimulator:
    """自适应PID控制器模拟 (对应 adaptive_pid.c)"""

    def __init__(self):
        # 默认参数范围
        self.Kp_init = 1.0
        self.Ki_init = 0.0
        self.Kd_init = 0.0
        self.Kp_min, self.Kp_max = 0.01, 100.0
        self.Ki_min, self.Ki_max = 0.0, 50.0
        self.Kd_min, self.Kd_max = 0.0, 10.0

        self.Kp = self.Kp_init
        self.Ki = self.Ki_init
        self.Kd = self.Kd_init

        # 自适应参数
        self.method = ADAPTIVE_GRADIENT
        self.learning_rate_p = 0.01
        self.learning_rate_i = 0.005
        self.learning_rate_d = 0.002
        self.alpha = 0.1  # MIT规则遗忘因子
        self.deadband = 0.01

        # 输出限幅
        self.out_min = -1000.0
        self.out_max = 1000.0
        self.integral_max = 500.0
        self.dt = 0.001

        # 内部状态
        self.error = 0.0
        self.error_last = 0.0
        self.error_prev = 0.0
        self.integral = 0.0
        self.output_last = 0.0
        self.plant_output_last = 0.0

    def set_method(self, method):
        self.method = method

    def set_param_range(self, kp_min, kp_max, ki_min, ki_max, kd_min, kd_max):
        self.Kp_min, self.Kp_max = kp_min, kp_max
        self.Ki_min, self.Ki_max = ki_min, ki_max
        self.Kd_min, self.Kd_max = kd_min, kd_max

    def set_learning_rate(self, lr_p, lr_i, lr_d):
        self.learning_rate_p = lr_p
        self.learning_rate_i = lr_i
        self.learning_rate_d = lr_d

    def _gradient_adapt(self, error):
        de = error - self.error_last
        dkp = -self.learning_rate_p * error * error
        dki = -self.learning_rate_i * error * self.integral
        dkd = -self.learning_rate_d * error * de
        self.Kp += dkp
        self.Ki += dki
        self.Kd += dkd
        self.Kp = _clamp(self.Kp, self.Kp_min, self.Kp_max)
        self.Ki = _clamp(self.Ki, self.Ki_min, self.Ki_max)
        self.Kd = _clamp(self.Kd, self.Kd_min, self.Kd_max)

    def _mit_adapt(self, error, plant_output):
        du = self.output_last
        dy = plant_output - self.plant_output_last
        jacobian = (dy / du) if abs(du) > 1e-6 else 0.0
        common = -self.alpha * error * jacobian
        self.Kp += common * error
        self.Ki += common * self.integral
        self.Kd += common * (error - self.error_last)
        self.Kp = _clamp(self.Kp, self.Kp_min, self.Kp_max)
        self.Ki = _clamp(self.Ki, self.Ki_min, self.Ki_max)
        self.Kd = _clamp(self.Kd, self.Kd_min, self.Kd_max)

    def compute(self, target, measurement):
        error = target - measurement
        do_adapt = abs(error) > self.deadband

        # 积分 (带限幅)
        self.integral += error * self.dt
        self.integral = _clamp(self.integral, -self.integral_max, self.integral_max)

        # 微分
        derivative = (error - self.error_last) / self.dt if self.dt > 0 else 0

        # PID输出
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = _clamp(output, self.out_min, self.out_max)

        # 自适应
        if do_adapt:
            if self.method == ADAPTIVE_GRADIENT:
                self._gradient_adapt(error)
            else:
                self._mit_adapt(error, measurement)

        # 更新历史
        self.error_prev = self.error_last
        self.error_last = error
        self.plant_output_last = measurement
        self.output_last = output

        return output

    def reset(self):
        self.error = 0.0
        self.error_last = 0.0
        self.error_prev = 0.0
        self.integral = 0.0
        self.output_last = 0.0
        self.plant_output_last = 0.0
        self.Kp = self.Kp_init
        self.Ki = self.Ki_init
        self.Kd = self.Kd_init

    def get_params(self):
        return self.Kp, self.Ki, self.Kd


class TestAdaptivePIDInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        pid = AdaptivePIDSimulator()
        self.assertEqual(pid.Kp, 1.0)
        self.assertEqual(pid.Ki, 0.0)
        self.assertEqual(pid.Kd, 0.0)

    def test_default_method(self):
        pid = AdaptivePIDSimulator()
        self.assertEqual(pid.method, ADAPTIVE_GRADIENT)

    def test_default_ranges(self):
        pid = AdaptivePIDSimulator()
        self.assertEqual(pid.Kp_min, 0.01)
        self.assertEqual(pid.Kp_max, 100.0)

    def test_default_learning_rates(self):
        pid = AdaptivePIDSimulator()
        self.assertEqual(pid.learning_rate_p, 0.01)
        self.assertEqual(pid.learning_rate_i, 0.005)


class TestAdaptivePIDCompute(unittest.TestCase):
    """计算测试"""

    def test_returns_float(self):
        """应返回数值"""
        pid = AdaptivePIDSimulator()
        output = pid.compute(target=10.0, measurement=5.0)
        self.assertIsInstance(output, float)

    def test_positive_error_positive_output(self):
        """正误差应产生正输出"""
        pid = AdaptivePIDSimulator()
        output = pid.compute(target=10.0, measurement=0.0)
        self.assertGreater(output, 0)

    def test_negative_error_negative_output(self):
        """负误差应产生负输出"""
        pid = AdaptivePIDSimulator()
        output = pid.compute(target=0.0, measurement=10.0)
        self.assertLess(output, 0)

    def test_output_clamped(self):
        """输出应被限幅"""
        pid = AdaptivePIDSimulator()
        pid.out_min = -50.0
        pid.out_max = 50.0
        pid.Kp = 1000.0
        output = pid.compute(target=100.0, measurement=0.0)
        self.assertLessEqual(output, 50.0)
        self.assertGreaterEqual(output, -50.0)


class TestAdaptivePIDGradient(unittest.TestCase):
    """梯度下降法自适应测试"""

    def test_kp_adapts(self):
        """Kp应随误差变化"""
        pid = AdaptivePIDSimulator()
        pid.method = ADAPTIVE_GRADIENT
        pid.learning_rate_p = 0.1
        kp_before = pid.Kp
        pid.compute(target=10.0, measurement=0.0)  # 大误差
        self.assertNotAlmostEqual(pid.Kp, kp_before, places=2)

    def test_params_clamped(self):
        """调整后参数应在范围内"""
        pid = AdaptivePIDSimulator()
        pid.set_param_range(0.1, 5.0, 0.0, 2.0, 0.0, 1.0)
        for _ in range(100):
            pid.compute(target=10.0, measurement=0.0)
        kp, ki, kd = pid.get_params()
        self.assertGreaterEqual(kp, 0.1)
        self.assertLessEqual(kp, 5.0)
        self.assertGreaterEqual(ki, 0.0)
        self.assertLessEqual(ki, 2.0)
        self.assertGreaterEqual(kd, 0.0)
        self.assertLessEqual(kd, 1.0)

    def test_adaptation_with_integral(self):
        """积分累积后Ki应有变化"""
        pid = AdaptivePIDSimulator()
        pid.learning_rate_i = 0.01
        pid.dt = 0.01
        ki_before = pid.Ki
        for _ in range(50):
            pid.compute(target=10.0, measurement=0.0)
        # Ki应有变化(可能增大也可能减小，取决于误差方向)
        self.assertNotAlmostEqual(pid.Ki, ki_before, places=3)


class TestAdaptivePIDMIT(unittest.TestCase):
    """MIT规则自适应测试"""

    def test_mit_method(self):
        """MIT规则应能运行"""
        pid = AdaptivePIDSimulator()
        pid.set_method(ADAPTIVE_MIT)
        output = pid.compute(target=10.0, measurement=5.0)
        self.assertIsNotNone(output)

    def test_mit_adapts_params(self):
        """MIT规则应调整参数"""
        pid = AdaptivePIDSimulator()
        pid.set_method(ADAPTIVE_MIT)
        pid.alpha = 0.5
        kp_before = pid.Kp
        # 需要多步才能看到明显变化
        for _ in range(10):
            pid.compute(target=10.0, measurement=0.0)
        self.assertNotAlmostEqual(pid.Kp, kp_before, places=2)


class TestAdaptivePIDDeadband(unittest.TestCase):
    """死区测试"""

    def test_no_adaptation_in_deadband(self):
        """死区内不应自适应"""
        pid = AdaptivePIDSimulator()
        pid.deadband = 5.0
        pid.learning_rate_p = 1.0  # 大学习率
        kp_before = pid.Kp
        pid.compute(target=2.0, measurement=0.0)  # |e|=2 < 5
        self.assertAlmostEqual(pid.Kp, kp_before, places=5)

    def test_adaptation_outside_deadband(self):
        """死区外应自适应"""
        pid = AdaptivePIDSimulator()
        pid.deadband = 1.0
        pid.learning_rate_p = 0.5
        kp_before = pid.Kp
        pid.compute(target=10.0, measurement=0.0)  # |e|=10 > 1
        self.assertNotAlmostEqual(pid.Kp, kp_before, places=3)


class TestAdaptivePIDIntegral(unittest.TestCase):
    """积分测试"""

    def test_integral_accumulation(self):
        """积分应累积"""
        pid = AdaptivePIDSimulator()
        pid.dt = 0.01
        pid.compute(target=10.0, measurement=0.0)
        self.assertGreater(pid.integral, 0)

    def test_integral_clamping(self):
        """积分应被限幅"""
        pid = AdaptivePIDSimulator()
        pid.dt = 0.01
        pid.integral_max = 5.0
        for _ in range(10000):
            pid.compute(target=100.0, measurement=0.0)
        self.assertLessEqual(pid.integral, 5.0)
        self.assertGreaterEqual(pid.integral, -5.0)


class TestAdaptivePIDSetParamRange(unittest.TestCase):
    """参数范围设置测试"""

    def test_set_param_range(self):
        pid = AdaptivePIDSimulator()
        pid.set_param_range(0.5, 10.0, 0.1, 5.0, 0.01, 2.0)
        self.assertEqual(pid.Kp_min, 0.5)
        self.assertEqual(pid.Kp_max, 10.0)
        self.assertEqual(pid.Ki_min, 0.1)
        self.assertEqual(pid.Ki_max, 5.0)
        self.assertEqual(pid.Kd_min, 0.01)
        self.assertEqual(pid.Kd_max, 2.0)


class TestAdaptivePIDSetLearningRate(unittest.TestCase):
    """学习率设置测试"""

    def test_set_learning_rate(self):
        pid = AdaptivePIDSimulator()
        pid.set_learning_rate(0.1, 0.05, 0.02)
        self.assertEqual(pid.learning_rate_p, 0.1)
        self.assertEqual(pid.learning_rate_i, 0.05)
        self.assertEqual(pid.learning_rate_d, 0.02)


class TestAdaptivePIDSetMethod(unittest.TestCase):
    """方法设置测试"""

    def test_set_gradient(self):
        pid = AdaptivePIDSimulator()
        pid.set_method(ADAPTIVE_GRADIENT)
        self.assertEqual(pid.method, ADAPTIVE_GRADIENT)

    def test_set_mit(self):
        pid = AdaptivePIDSimulator()
        pid.set_method(ADAPTIVE_MIT)
        self.assertEqual(pid.method, ADAPTIVE_MIT)


class TestAdaptivePIDReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        pid = AdaptivePIDSimulator()
        pid.compute(target=10.0, measurement=0.0)
        pid.reset()
        self.assertEqual(pid.error, 0.0)
        self.assertEqual(pid.error_last, 0.0)
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.output_last, 0.0)

    def test_reset_restores_initial_params(self):
        """重置应恢复初始参数"""
        pid = AdaptivePIDSimulator()
        pid.Kp_init = 2.0
        pid.Ki_init = 0.5
        pid.Kd_init = 0.1
        pid.Kp = pid.Kp_init
        pid.Ki = pid.Ki_init
        pid.Kd = pid.Kd_init

        # 运行使参数变化
        for _ in range(100):
            pid.compute(target=10.0, measurement=0.0)
        pid.reset()
        self.assertEqual(pid.Kp, 2.0)
        self.assertEqual(pid.Ki, 0.5)
        self.assertEqual(pid.Kd, 0.1)


class TestAdaptivePIDGetParams(unittest.TestCase):
    """参数获取测试"""

    def test_get_params(self):
        pid = AdaptivePIDSimulator()
        kp, ki, kd = pid.get_params()
        self.assertEqual(kp, 1.0)
        self.assertEqual(ki, 0.0)
        self.assertEqual(kd, 0.0)

    def test_get_params_after_compute(self):
        pid = AdaptivePIDSimulator()
        pid.compute(target=10.0, measurement=0.0)
        kp, ki, kd = pid.get_params()
        self.assertIsInstance(kp, float)
        self.assertIsInstance(ki, float)
        self.assertIsInstance(kd, float)


class TestAdaptivePIDConvergence(unittest.TestCase):
    """收敛性测试"""

    def test_gradient_converges(self):
        """梯度下降法阶跃响应应趋近目标"""
        pid = AdaptivePIDSimulator()
        pid.set_param_range(0.01, 10.0, 0.0, 5.0, 0.0, 2.0)
        pid.set_learning_rate(0.001, 0.0005, 0.0002)
        pid.dt = 0.01
        pid.out_min = -500
        pid.out_max = 500
        pid.integral_max = 200

        state = 0.0
        for _ in range(5000):
            output = pid.compute(target=100.0, measurement=state)
            state += output * 0.001

        # 应该趋近目标(不要求精确,因为自适应可能振荡)
        self.assertGreater(state, 30.0)

    def test_mit_converges(self):
        """MIT规则阶跃响应应趋近目标"""
        pid = AdaptivePIDSimulator()
        pid.set_method(ADAPTIVE_MIT)
        pid.alpha = 0.05
        pid.dt = 0.01
        pid.out_min = -500
        pid.out_max = 500

        state = 0.0
        for _ in range(5000):
            output = pid.compute(target=100.0, measurement=state)
            state += output * 0.001

        self.assertGreater(state, 20.0)


class TestAdaptivePIDHistoryUpdate(unittest.TestCase):
    """历史状态更新测试"""

    def test_error_history_updated(self):
        """误差历史应被更新"""
        pid = AdaptivePIDSimulator()
        pid.compute(target=10.0, measurement=5.0)
        self.assertAlmostEqual(pid.error_last, 5.0, places=3)

    def test_plant_output_history(self):
        """被控对象输出历史应被更新"""
        pid = AdaptivePIDSimulator()
        pid.compute(target=10.0, measurement=7.0)
        self.assertEqual(pid.plant_output_last, 7.0)

    def test_output_history(self):
        """输出历史应被更新"""
        pid = AdaptivePIDSimulator()
        output = pid.compute(target=10.0, measurement=5.0)
        self.assertEqual(pid.output_last, output)


if __name__ == '__main__':
    unittest.main()
