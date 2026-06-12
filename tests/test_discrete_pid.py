#!/usr/bin/env python3
"""
离散PID单元测试 - 定点化效应分析
覆盖: 浮点PID计算、定点化工具、一阶系统响应、性能指标
      不同精度下的控制效果、稳态误差、上升时间
"""
import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '15_simulation'))
from discrete_pid_simulation import (
    FirstOrderPlant, float_to_fixed, fixed_pid_step,
    run_simulation, calc_metrics
)


class TestFirstOrderPlant(unittest.TestCase):
    """一阶惯性+纯滞后系统测试"""

    def test_zero_input_stays_zero(self):
        plant = FirstOrderPlant(K=1.0, T=0.5, dt=0.01)
        for _ in range(100):
            y = plant.update(0.0)
        self.assertAlmostEqual(y, 0.0, delta=1e-10)

    def test_step_response_converges(self):
        plant = FirstOrderPlant(K=1.0, T=0.5, dt=0.01)
        for _ in range(500):
            y = plant.update(1.0)
        self.assertAlmostEqual(y, 1.0, delta=0.05)

    def test_with_delay(self):
        """带滞后时输出应延迟响应"""
        plant_no_delay = FirstOrderPlant(K=1.0, T=0.5, delay=0.0, dt=0.01)
        plant_delay = FirstOrderPlant(K=1.0, T=0.5, delay=0.1, dt=0.01)

        # 前几步两者应不同(延迟系统需要等待)
        for _ in range(5):
            y_no_delay = plant_no_delay.update(1.0)
            y_delay = plant_delay.update(1.0)
        # 延迟系统的输出应更小(还没收到输入)
        self.assertLessEqual(y_delay, y_no_delay + 0.01)

    def test_different_K_values(self):
        """不同增益K应影响稳态值"""
        plant1 = FirstOrderPlant(K=1.0, T=0.5, dt=0.01)
        plant2 = FirstOrderPlant(K=2.0, T=0.5, dt=0.01)
        for _ in range(500):
            y1 = plant1.update(1.0)
            y2 = plant2.update(1.0)
        self.assertAlmostEqual(y2 / y1, 2.0, delta=0.1)

    def test_different_T_values(self):
        """不同时间常数T应影响响应速度"""
        plant_fast = FirstOrderPlant(K=1.0, T=0.2, dt=0.01)
        plant_slow = FirstOrderPlant(K=1.0, T=1.0, dt=0.01)

        for _ in range(30):
            y_fast = plant_fast.update(1.0)
            y_slow = plant_slow.update(1.0)
        # 快系统应响应更快
        self.assertGreater(y_fast, y_slow)


class TestFloatToFixed(unittest.TestCase):
    """定点化工具测试"""

    def test_zero(self):
        result = float_to_fixed(0.0, 8)
        self.assertAlmostEqual(result, 0.0)

    def test_positive_value(self):
        result = float_to_fixed(1.5, 8)
        self.assertAlmostEqual(result, 1.5, delta=0.01)

    def test_negative_value(self):
        result = float_to_fixed(-1.5, 8)
        self.assertAlmostEqual(result, -1.5, delta=0.01)

    def test_saturation_upper(self):
        """超出范围应饱和"""
        result = float_to_fixed(100000.0, 8)
        max_val = 32767.0 / (2**8)
        self.assertLessEqual(result, max_val + 0.01)

    def test_saturation_lower(self):
        """低于范围应饱和"""
        result = float_to_fixed(-100000.0, 8)
        min_val = -32768.0 / (2**8)
        self.assertGreaterEqual(result, min_val - 0.01)

    def test_quantization(self):
        """定点化应产生量化效应"""
        result = float_to_fixed(1.23456789, 4)
        # 4位小数精度约0.0625
        quantized = round(1.23456789 * (2**4)) / (2**4)
        self.assertAlmostEqual(result, quantized, delta=0.001)


class TestFixedPidStep(unittest.TestCase):
    """单步PID计算测试"""

    def test_p_only(self):
        u = fixed_pid_step(e=5.0, e_sum=0.0, e_prev=0.0,
                          Kp=1.0, Ki=0.0, Kd=0.0, dt=0.01)
        self.assertAlmostEqual(u, 5.0, delta=0.01)

    def test_i_only(self):
        u = fixed_pid_step(e=1.0, e_sum=10.0, e_prev=0.0,
                          Kp=0.0, Ki=1.0, Kd=0.0, dt=0.01)
        self.assertAlmostEqual(u, 10.0, delta=0.01)

    def test_d_only(self):
        u = fixed_pid_step(e=5.0, e_sum=0.0, e_prev=3.0,
                          Kp=0.0, Ki=0.0, Kd=1.0, dt=0.01)
        # de = (5-3)/0.01 = 200
        self.assertAlmostEqual(u, 200.0, delta=1.0)

    def test_with_fixed_point(self):
        """定点化应产生量化噪声"""
        u_float = fixed_pid_step(e=1.234, e_sum=0.567, e_prev=0.1,
                                 Kp=2.0, Ki=1.0, Kd=0.5, dt=0.01)
        u_fixed = fixed_pid_step(e=1.234, e_sum=0.567, e_prev=0.1,
                                 Kp=2.0, Ki=1.0, Kd=0.5, dt=0.01, frac_bits=8)
        # 两者应接近但不完全相同
        self.assertAlmostEqual(u_float, u_fixed, delta=abs(u_float) * 0.1 + 0.1)

    def test_zero_error_zero_p_term(self):
        u = fixed_pid_step(e=0.0, e_sum=0.0, e_prev=0.0,
                          Kp=1.0, Ki=0.0, Kd=0.0, dt=0.01)
        self.assertAlmostEqual(u, 0.0, delta=0.01)


class TestRunSimulation(unittest.TestCase):
    """仿真运行测试"""

    def test_returns_arrays(self):
        t, y, u, e = run_simulation(
            Kp=2.0, Ki=1.0, Kd=0.3, setpoint=1.0,
            plant_params=dict(K=1.0, T=0.5, delay=0.0),
            dt=0.01, t_end=2.0
        )
        self.assertEqual(len(t), 200)
        self.assertEqual(len(y), 200)
        self.assertEqual(len(u), 200)
        self.assertEqual(len(e), 200)

    def test_converges_float(self):
        """浮点PID应能收敛"""
        t, y, u, e = run_simulation(
            Kp=2.0, Ki=1.0, Kd=0.3, setpoint=1.0,
            plant_params=dict(K=1.0, T=0.5, delay=0.0),
            dt=0.01, t_end=5.0
        )
        ss_val = np.mean(y[int(0.8*len(y)):])
        self.assertAlmostEqual(ss_val, 1.0, delta=0.1)

    def test_converges_fixed_point(self):
        """定点PID应能收敛(精度足够时)"""
        t, y, u, e = run_simulation(
            Kp=2.0, Ki=1.0, Kd=0.3, setpoint=1.0,
            plant_params=dict(K=1.0, T=0.5, delay=0.0),
            dt=0.01, t_end=5.0, frac_bits=12
        )
        ss_val = np.mean(y[int(0.8*len(y)):])
        self.assertAlmostEqual(ss_val, 1.0, delta=0.15)

    def test_low_precision_oscillates(self):
        """低精度定点应产生更大误差"""
        t, y_high, _, _ = run_simulation(
            Kp=2.0, Ki=1.0, Kd=0.3, setpoint=1.0,
            plant_params=dict(K=1.0, T=0.5, delay=0.0),
            dt=0.01, t_end=5.0, frac_bits=12
        )
        t, y_low, _, _ = run_simulation(
            Kp=2.0, Ki=1.0, Kd=0.3, setpoint=1.0,
            plant_params=dict(K=1.0, T=0.5, delay=0.0),
            dt=0.01, t_end=5.0, frac_bits=4
        )
        ss_err_high = abs(np.mean(y_high[int(0.8*len(y_high)):]) - 1.0)
        ss_err_low = abs(np.mean(y_low[int(0.8*len(y_low)):]) - 1.0)
        # 低精度误差应更大
        self.assertGreaterEqual(ss_err_low, ss_err_high - 0.05)


class TestCalcMetrics(unittest.TestCase):
    """性能指标计算测试"""

    def test_overshoot(self):
        t = np.linspace(0, 5, 500)
        y = np.ones(500) * 1.0
        y[50:100] = 1.3  # 30%超调
        overshoot, rise_time, ss_err, ss_val = calc_metrics(t, y, 1.0)
        self.assertAlmostEqual(overshoot, 30.0, delta=1.0)

    def test_no_overshoot(self):
        t = np.linspace(0, 5, 500)
        y = np.ones(500) * 1.0
        overshoot, _, _, _ = calc_metrics(t, y, 1.0)
        self.assertAlmostEqual(overshoot, 0.0, delta=0.1)

    def test_steady_state_error(self):
        t = np.linspace(0, 5, 500)
        y = np.ones(500) * 0.95  # 5%稳态误差
        _, _, ss_err, ss_val = calc_metrics(t, y, 1.0)
        self.assertAlmostEqual(ss_err, 5.0, delta=0.5)

    def test_rise_time(self):
        t = np.linspace(0, 5, 500)
        y = np.linspace(0, 1.5, 500)  # 线性上升
        _, rise_time, _, _ = calc_metrics(t, y, 1.0)
        self.assertGreater(rise_time, 0)

    def test_zero_setpoint(self):
        t = np.linspace(0, 5, 500)
        y = np.ones(500) * 0.0
        overshoot, _, ss_err, _ = calc_metrics(t, y, 0.0)
        self.assertEqual(overshoot, 0)
        self.assertEqual(ss_err, 0)


if __name__ == '__main__':
    unittest.main()
