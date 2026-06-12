#!/usr/bin/env python3
"""
PID + 滤波器单元测试
覆盖: 一阶低通滤波器、二阶Butterworth滤波器、PIDWithFilter控制器
      微分滤波效果、输出滤波效果、PV微分vs误差微分
"""
import unittest
import math
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '15_simulation'))
from pid_with_filter import LowPassFilter1, LowPassFilter2, PIDWithFilter, SecondOrderPlant


class TestLowPassFilter1(unittest.TestCase):
    """一阶低通滤波器测试"""

    def test_constant_input_converges(self):
        """恒定输入应收敛到该值"""
        f = LowPassFilter1(fc=50, dt=0.01)
        for _ in range(500):
            result = f.filter(1.0)
        self.assertAlmostEqual(result, 1.0, delta=0.05)

    def test_zero_input_stays_zero(self):
        """零输入应保持零输出"""
        f = LowPassFilter1(fc=50, dt=0.01)
        for _ in range(100):
            result = f.filter(0.0)
        self.assertAlmostEqual(result, 0.0, delta=1e-10)

    def test_reset(self):
        """重置后输出应归零"""
        f = LowPassFilter1(fc=50, dt=0.01)
        for _ in range(100):
            f.filter(1.0)
        f.reset()
        self.assertAlmostEqual(f.y_prev, 0.0)

    def test_low_fc_more_smoothing(self):
        """更低的截止频率应产生更多平滑"""
        f_high = LowPassFilter1(fc=100, dt=0.01)
        f_low = LowPassFilter1(fc=10, dt=0.01)

        # 输入阶跃
        for _ in range(10):
            out_high = f_high.filter(1.0)
            out_low = f_low.filter(1.0)

        # 低截止频率应更慢响应
        self.assertLess(out_low, out_high)

    def test_output_shape(self):
        f = LowPassFilter1(fc=50, dt=0.01)
        result = f.filter(1.0)
        self.assertIsInstance(result, float)


class TestLowPassFilter2(unittest.TestCase):
    """二阶Butterworth低通滤波器测试"""

    def test_constant_input_converges(self):
        f = LowPassFilter2(fc=50, dt=0.01)
        for _ in range(1000):
            result = f.filter(1.0)
        self.assertAlmostEqual(result, 1.0, delta=0.1)

    def test_zero_input_stays_zero(self):
        f = LowPassFilter2(fc=50, dt=0.01)
        for _ in range(100):
            result = f.filter(0.0)
        self.assertAlmostEqual(result, 0.0, delta=1e-10)

    def test_reset(self):
        f = LowPassFilter2(fc=50, dt=0.01)
        for _ in range(100):
            f.filter(1.0)
        f.reset()
        self.assertEqual(f.x, [0.0, 0.0, 0.0])
        self.assertEqual(f.y, [0.0, 0.0, 0.0])

    def test_noise_smoothing(self):
        """滤波器应平滑随机噪声"""
        np.random.seed(42)
        noise = np.random.randn(200)
        f = LowPassFilter2(fc=20, dt=0.01)
        filtered = np.array([f.filter(x) for x in noise])
        # 滤波后方差应更小
        self.assertLess(np.var(filtered), np.var(noise))

    def test_output_is_float(self):
        f = LowPassFilter2(fc=50, dt=0.01)
        result = f.filter(1.0)
        self.assertIsInstance(result, float)


class TestPIDWithFilterInit(unittest.TestCase):
    """PIDWithFilter初始化测试"""

    def test_default_init(self):
        pid = PIDWithFilter(Kp=1.0, Ki=0.5, Kd=0.1, dt=0.01)
        self.assertEqual(pid.Kp, 1.0)
        self.assertEqual(pid.Ki, 0.5)
        self.assertEqual(pid.Kd, 0.1)

    def test_lp1_filter_type(self):
        pid = PIDWithFilter(Kp=1.0, Ki=0.5, Kd=0.1, dt=0.01, filter_type='lp1', fc=50)
        self.assertEqual(pid.filter_type, 'lp1')
        self.assertTrue(hasattr(pid, 'd_filter'))

    def test_lp2_filter_type(self):
        pid = PIDWithFilter(Kp=1.0, Ki=0.5, Kd=0.1, dt=0.01, filter_type='lp2', fc=50)
        self.assertEqual(pid.filter_type, 'lp2')
        self.assertTrue(hasattr(pid, 'd_filter'))

    def test_incomplete_filter_type(self):
        pid = PIDWithFilter(Kp=1.0, Ki=0.5, Kd=0.1, dt=0.01, filter_type='incomplete', fc=50)
        self.assertEqual(pid.filter_type, 'incomplete')

    def test_none_filter_type(self):
        pid = PIDWithFilter(Kp=1.0, Ki=0.5, Kd=0.1, dt=0.01, filter_type='none')
        self.assertEqual(pid.filter_type, 'none')


class TestPIDWithFilterCompute(unittest.TestCase):
    """PIDWithFilter计算测试"""

    def test_positive_error_positive_output(self):
        pid = PIDWithFilter(Kp=1.0, Ki=0.0, Kd=0.0, dt=0.01, filter_type='none')
        u = pid.compute(1.0)
        self.assertGreater(u, 0)

    def test_negative_error_negative_output(self):
        pid = PIDWithFilter(Kp=1.0, Ki=0.0, Kd=0.0, dt=0.01, filter_type='none')
        u = pid.compute(-1.0)
        self.assertLess(u, 0)

    def test_output_clamped(self):
        pid = PIDWithFilter(Kp=100.0, Ki=0.0, Kd=0.0, dt=0.01, u_min=-5, u_max=5)
        u = pid.compute(10.0)
        self.assertLessEqual(u, 5.0)
        self.assertGreaterEqual(u, -5.0)

    def test_integral_accumulates(self):
        pid = PIDWithFilter(Kp=0.0, Ki=1.0, Kd=0.0, dt=0.01, filter_type='none')
        u1 = pid.compute(1.0)
        u2 = pid.compute(1.0)
        self.assertGreater(u2, u1)

    def test_reset_clears_state(self):
        pid = PIDWithFilter(Kp=1.0, Ki=1.0, Kd=0.5, dt=0.01, filter_type='lp1', fc=50)
        for _ in range(50):
            pid.compute(1.0)
        pid.reset()
        self.assertEqual(pid.e_sum, 0.0)
        self.assertEqual(pid.e_prev, 0.0)

    def test_lp1_filter_computes(self):
        pid = PIDWithFilter(Kp=1.0, Ki=0.0, Kd=0.5, dt=0.01, filter_type='lp1', fc=50)
        u = pid.compute(1.0)
        self.assertIsInstance(u, float)

    def test_lp2_filter_computes(self):
        pid = PIDWithFilter(Kp=1.0, Ki=0.0, Kd=0.5, dt=0.01, filter_type='lp2', fc=50)
        u = pid.compute(1.0)
        self.assertIsInstance(u, float)

    def test_incomplete_filter_computes(self):
        pid = PIDWithFilter(Kp=1.0, Ki=0.0, Kd=0.5, dt=0.01, filter_type='incomplete', fc=50)
        u = pid.compute(1.0)
        self.assertIsInstance(u, float)

    def test_filter_reduces_d_noise(self):
        """滤波应减少微分项噪声"""
        np.random.seed(42)
        noise = np.random.randn(200) * 0.05

        pid_no_filter = PIDWithFilter(Kp=1.0, Ki=0.0, Kd=2.0, dt=0.01, filter_type='none')
        pid_lp1 = PIDWithFilter(Kp=1.0, Ki=0.0, Kd=2.0, dt=0.01, filter_type='lp1', fc=50)

        outputs_no_filter = []
        outputs_lp1 = []
        for n in noise:
            outputs_no_filter.append(pid_no_filter.compute(n))
            outputs_lp1.append(pid_lp1.compute(n))

        # 滤波后输出方差应更小
        self.assertLess(np.var(outputs_lp1), np.var(outputs_no_filter))


class TestSecondOrderPlant(unittest.TestCase):
    """二阶系统测试"""

    def test_zero_input_stays_zero(self):
        plant = SecondOrderPlant()
        for _ in range(100):
            y = plant.update(0.0)
        self.assertAlmostEqual(y, 0.0, delta=1e-10)

    def test_step_response_converges(self):
        plant = SecondOrderPlant(K=1.0, T1=0.5, T2=0.2, dt=0.01)
        for _ in range(1000):
            y = plant.update(1.0)
        self.assertAlmostEqual(y, 1.0, delta=0.1)

    def test_output_is_float(self):
        plant = SecondOrderPlant()
        y = plant.update(0.5)
        self.assertIsInstance(y, float)


if __name__ == '__main__':
    unittest.main()
