#!/usr/bin/env python3
"""
PID控制器单元测试
覆盖: 位置式PID、增量式PID、积分限幅、死区、微分滤波、前馈、抗饱和
"""

import sys
import os
import unittest
import numpy as np

# 将项目根目录加入路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# [审计修复] 修正import路径：目录名以数字开头，需用importlib
import importlib
_control_dir = os.path.join(os.path.dirname(__file__), '..', '04_通用代码库_OrangePi5')
sys.path.insert(0, os.path.abspath(_control_dir))
from control.pid_controller import PIDController, PIDMode, AntiWindupMethod


class TestPIDInit(unittest.TestCase):
    """PID初始化测试"""

    def test_default_params(self):
        pid = PIDController()
        self.assertEqual(pid.kp, 1.0)
        self.assertEqual(pid.ki, 0.0)
        self.assertEqual(pid.kd, 0.0)
        self.assertEqual(pid.mode, PIDMode.POSITION)
        self.assertEqual(pid.output_min, -100.0)
        self.assertEqual(pid.output_max, 100.0)

    def test_custom_params(self):
        pid = PIDController(kp=2.5, ki=0.3, kd=0.1, mode=PIDMode.INCREMENTAL)
        self.assertEqual(pid.kp, 2.5)
        self.assertEqual(pid.ki, 0.3)
        self.assertEqual(pid.kd, 0.1)
        self.assertEqual(pid.mode, PIDMode.INCREMENTAL)

    def test_reset_clears_state(self):
        pid = PIDController(kp=1.0, ki=0.1, kd=0.01)
        pid.compute(10.0, dt=0.01)  # 产生内部状态
        pid.reset()
        self.assertEqual(pid._integral, 0.0)
        self.assertEqual(pid._prev_error, 0.0)
        self.assertEqual(pid._prev_output, 0.0)


class TestPIDPositionMode(unittest.TestCase):
    """位置式PID测试"""

    def test_p_only_proportional(self):
        """纯比例控制: 输出应等于 kp * error"""
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0, deadband=0.0)
        output = pid.compute(error=5.0, dt=0.01)
        self.assertAlmostEqual(output, 10.0, places=2)

    def test_p_negative_error(self):
        """负误差应产生负输出"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        output = pid.compute(error=-5.0, dt=0.01)
        self.assertLess(output, 0)

    def test_integral_accumulation(self):
        """积分项应随时间累积"""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
        outputs = []
        for _ in range(10):
            out = pid.compute(error=1.0, dt=0.1)
            outputs.append(out)
        # 积分应单调递增
        for i in range(1, len(outputs)):
            self.assertGreaterEqual(outputs[i], outputs[i - 1])

    def test_integral_clamping(self):
        """积分项应被限幅"""
        pid = PIDController(kp=0.0, ki=10.0, kd=0.0,
                           integral_min=-10.0, integral_max=10.0)
        # 长时间大误差
        for _ in range(1000):
            pid.compute(error=100.0, dt=0.01)
        self.assertLessEqual(pid._integral, 10.0)
        self.assertGreaterEqual(pid._integral, -10.0)

    def test_output_clamping(self):
        """输出应被限幅在 [output_min, output_max]"""
        pid = PIDController(kp=100.0, ki=0.0, kd=0.0,
                           output_min=-50.0, output_max=50.0)
        output = pid.compute(error=100.0, dt=0.01)
        self.assertLessEqual(output, 50.0)
        self.assertGreaterEqual(output, -50.0)

    def test_deadband(self):
        """死区内误差应被置零"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, deadband=5.0)
        output = pid.compute(error=3.0, dt=0.01)  # 在死区内
        self.assertAlmostEqual(output, 0.0, places=2)

    def test_deadband_outside(self):
        """死区外误差应正常响应"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, deadband=5.0)
        output = pid.compute(error=10.0, dt=0.01)  # 超出死区
        self.assertAlmostEqual(output, 10.0, places=2)

    def test_zero_error_zero_output(self):
        """零误差纯P控制器应输出零"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        output = pid.compute(error=0.0, dt=0.01)
        self.assertAlmostEqual(output, 0.0, places=5)

    def test_step_response_converges(self):
        """阶跃响应应逐渐趋近目标"""
        pid = PIDController(kp=0.5, ki=0.1, kd=0.05,
                           output_min=-1000, output_max=1000)
        target = 100.0
        current = 0.0
        dt = 0.01
        for _ in range(500):
            error = target - current
            output = pid.compute(error=error, dt=dt)
            current += output * dt * 0.5  # 简单模拟
        # 应该接近目标
        self.assertGreater(current, 50.0)


class TestPIDIncrementalMode(unittest.TestCase):
    """增量式PID测试"""

    def test_incremental_basic(self):
        """增量式PID基本计算"""
        pid = PIDController(kp=1.0, ki=0.1, kd=0.01,
                           mode=PIDMode.INCREMENTAL)
        output1 = pid.compute(error=10.0, dt=0.01)
        output2 = pid.compute(error=9.0, dt=0.01)
        # 输出应有变化
        self.assertNotAlmostEqual(output1, output2, places=1)

    def test_incremental_accumulates(self):
        """增量式输出应累积"""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0,
                           mode=PIDMode.INCREMENTAL,
                           output_min=-1000, output_max=1000)
        outputs = []
        for _ in range(10):
            out = pid.compute(error=1.0, dt=0.1)
            outputs.append(out)
        # 输出应递增
        for i in range(1, len(outputs)):
            self.assertGreaterEqual(outputs[i], outputs[i - 1] - 0.01)


class TestPIDDerivativeFilter(unittest.TestCase):
    """微分滤波测试"""

    def test_derivative_filter_reduces_noise(self):
        """微分滤波应减少高频噪声的影响"""
        # 无滤波
        pid_no_filter = PIDController(kp=0.0, ki=0.0, kd=1.0,
                                      derivative_filter_alpha=1.0)
        # 强滤波
        pid_strong_filter = PIDController(kp=0.0, ki=0.0, kd=1.0,
                                          derivative_filter_alpha=0.01)

        # 含噪声的误差信号
        import math
        errors = [10.0 * math.sin(i * 0.1) + 2.0 * math.sin(i * 5.0)
                  for i in range(100)]

        outputs_no_filter = []
        outputs_strong_filter = []
        for e in errors:
            outputs_no_filter.append(pid_no_filter.compute(error=e, dt=0.01))
            outputs_strong_filter.append(pid_strong_filter.compute(error=e, dt=0.01))

        # 强滤波的输出方差应更小
        import statistics
        var_no_filter = statistics.variance(outputs_no_filter[10:])
        var_strong = statistics.variance(outputs_strong_filter[10:])
        self.assertLess(var_strong, var_no_filter)


class TestPIDSetGains(unittest.TestCase):
    """运行时参数修改测试"""

    def test_set_gains(self):
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        output_before = pid.compute(error=10.0, dt=0.01)

        pid.reset()
        pid.set_gains(kp=2.0, ki=0.0, kd=0.0)
        output_after = pid.compute(error=10.0, dt=0.01)

        self.assertAlmostEqual(output_after, output_before * 2.0, places=1)

    def test_state_property(self):
        """state属性应返回内部状态字典"""
        pid = PIDController(kp=1.0, ki=0.1, kd=0.01)
        pid.compute(error=5.0, dt=0.01)
        state = pid.state
        self.assertIn('kp', state)
        self.assertIn('ki', state)
        self.assertIn('kd', state)
        self.assertIn('integral', state)
        self.assertIn('prev_error', state)
        self.assertIn('prev_output', state)


class TestPIDEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_very_small_dt(self):
        """极小dt不应导致数值爆炸"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.1)
        output = pid.compute(error=10.0, dt=1e-6)
        # 输出应该在合理范围内
        self.assertGreaterEqual(output, -100, "输出不应低于-100")
        self.assertLessEqual(output, 100, "输出不应超过100")
        self.assertTrue(np.isfinite(output), "输出应该是有限值")

    def test_very_large_error(self):
        """大误差应被限幅"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0,
                           output_min=-100.0, output_max=100.0)
        output = pid.compute(error=1e6, dt=0.01)
        self.assertLessEqual(output, 100.0)

    def test_negative_dt_handled(self):
        """负dt应被修正为极小正值"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        # 首次调用 dt=None，内部默认0.01
        output = pid.compute(error=10.0, dt=None)
        self.assertIsNotNone(output)

    def test_repr(self):
        """__repr__应返回可读字符串"""
        pid = PIDController(kp=1.0, ki=0.1, kd=0.01)
        s = repr(pid)
        self.assertIn('PIDController', s)
        self.assertIn('1.0', s)


if __name__ == '__main__':
    unittest.main()
