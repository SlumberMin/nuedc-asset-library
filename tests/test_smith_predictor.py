#!/usr/bin/env python3
"""
Smith预估器单元测试
覆盖: 初始化、模型设置、PID设置、延迟补偿、预估输出、
      修正反馈、复位、调试信息获取、收敛性
注意: 使用纯 Python 模拟 C SmithPredictor 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def _clamp(val, min_v, max_v):
    if val < min_v:
        return min_v
    if val > max_v:
        return max_v
    return val


class SmithPredictorSimulator:
    """Smith预估器模拟 (对应 smith_predictor.c)"""

    def __init__(self, dt=0.01):
        self.dt = dt

        # 模型参数
        self.Kp = 1.0
        self.T = 1.0
        self.L = 0.5

        # PID参数
        self.pid_Kp = 1.0
        self.pid_Ki = 0.5
        self.pid_Kd = 0.1

        # PID状态
        self.pid_error = 0.0
        self.pid_error_last = 0.0
        self.pid_integral = 0.0

        # 模型状态
        self.model_state = 0.0
        self.model_coeff_a = 0.0
        self.model_coeff_b = 0.0

        # 延迟缓冲区
        self.delay_buffer = [0.0] * 512
        self.delay_size = 512
        self.delay_index = 0
        self.delay_output = 0.0

        # 预估器输出
        self.predictor_output = 0.0
        self.compensated_feedback = 0.0

        # 限幅
        self.out_min = -1000.0
        self.out_max = 1000.0
        self.integral_max = 500.0

        # 初始化模型系数
        self.set_model(self.Kp, self.T, self.L)

    def set_model(self, Kp, T, L):
        self.Kp = Kp
        self.T = T
        self.L = L
        dt = self.dt
        self.model_coeff_b = Kp * dt / (T + dt)
        self.model_coeff_a = T / (T + dt)

        delay_samples = int(L / dt) + 1
        if delay_samples > 512:
            delay_samples = 512
        self.delay_size = delay_samples
        self.delay_index = 0
        self.delay_buffer = [0.0] * 512

    def set_pid(self, Kp, Ki, Kd):
        self.pid_Kp = Kp
        self.pid_Ki = Ki
        self.pid_Kd = Kd

    def compute(self, target, measurement):
        # Step 1: 更新无滞后模型
        self.model_state = (self.model_coeff_a * self.model_state
                           + self.model_coeff_b * measurement)
        ym = self.model_state

        # Step 2: 更新延迟模型
        self.delay_buffer[self.delay_index] = ym
        read_index = (self.delay_index + 1) % self.delay_size
        ym_delayed = self.delay_buffer[read_index]
        self.delay_index = read_index

        # Step 3: 预估补偿
        self.predictor_output = ym - ym_delayed

        # Step 4: 修正反馈
        self.compensated_feedback = measurement + self.predictor_output

        # Step 5: PID控制器
        error = target - self.compensated_feedback
        self.pid_integral += error * self.dt
        self.pid_integral = _clamp(self.pid_integral, -self.integral_max, self.integral_max)

        derivative = (error - self.pid_error_last) / self.dt if self.dt > 0 else 0.0

        output = (self.pid_Kp * error
                  + self.pid_Ki * self.pid_integral
                  + self.pid_Kd * derivative)

        output = _clamp(output, self.out_min, self.out_max)
        self.pid_error_last = error

        return output

    def reset(self):
        self.pid_error = 0.0
        self.pid_error_last = 0.0
        self.pid_integral = 0.0
        self.model_state = 0.0
        self.delay_output = 0.0
        self.predictor_output = 0.0
        self.compensated_feedback = 0.0
        self.delay_index = 0
        self.delay_buffer = [0.0] * 512

    def get_debug(self):
        return self.model_state, self.delay_output, self.compensated_feedback


class TestSmithInit(unittest.TestCase):
    """初始化测试"""

    def test_default_model_params(self):
        pred = SmithPredictorSimulator()
        self.assertEqual(pred.Kp, 1.0)
        self.assertEqual(pred.T, 1.0)
        self.assertEqual(pred.L, 0.5)

    def test_default_pid_params(self):
        pred = SmithPredictorSimulator()
        self.assertEqual(pred.pid_Kp, 1.0)
        self.assertEqual(pred.pid_Ki, 0.5)
        self.assertEqual(pred.pid_Kd, 0.1)

    def test_default_limits(self):
        pred = SmithPredictorSimulator()
        self.assertEqual(pred.out_min, -1000.0)
        self.assertEqual(pred.out_max, 1000.0)

    def test_model_coefficients_computed(self):
        """模型系数应在初始化时计算"""
        pred = SmithPredictorSimulator(dt=0.01)
        self.assertNotEqual(pred.model_coeff_a, 0.0)
        self.assertNotEqual(pred.model_coeff_b, 0.0)


class TestSmithSetModel(unittest.TestCase):
    """模型设置测试"""

    def test_set_model_updates_params(self):
        pred = SmithPredictorSimulator()
        pred.set_model(2.0, 0.5, 0.1)
        self.assertEqual(pred.Kp, 2.0)
        self.assertEqual(pred.T, 0.5)
        self.assertEqual(pred.L, 0.1)

    def test_model_coefficients(self):
        """模型系数应正确计算"""
        pred = SmithPredictorSimulator(dt=0.1)
        pred.set_model(2.0, 1.0, 0.5)
        # coeff_a = T/(T+dt) = 1.0/(1.0+0.1) = 0.909
        expected_a = 1.0 / (1.0 + 0.1)
        self.assertAlmostEqual(pred.model_coeff_a, expected_a, places=3)
        # coeff_b = Kp*dt/(T+dt) = 2.0*0.1/1.1 = 0.1818
        expected_b = 2.0 * 0.1 / (1.0 + 0.1)
        self.assertAlmostEqual(pred.model_coeff_b, expected_b, places=3)

    def test_delay_size_computed(self):
        """延迟缓冲区大小应根据L和dt计算"""
        pred = SmithPredictorSimulator(dt=0.01)
        pred.set_model(1.0, 1.0, 0.1)  # L=0.1, dt=0.01 -> 10+1=11
        self.assertEqual(pred.delay_size, 11)


class TestSmithSetPID(unittest.TestCase):
    """PID设置测试"""

    def test_set_pid(self):
        pred = SmithPredictorSimulator()
        pred.set_pid(2.0, 1.0, 0.3)
        self.assertEqual(pred.pid_Kp, 2.0)
        self.assertEqual(pred.pid_Ki, 1.0)
        self.assertEqual(pred.pid_Kd, 0.3)


class TestSmithCompute(unittest.TestCase):
    """计算测试"""

    def test_returns_float(self):
        pred = SmithPredictorSimulator()
        output = pred.compute(target=10.0, measurement=5.0)
        self.assertIsInstance(output, float)

    def test_positive_error_positive_output(self):
        pred = SmithPredictorSimulator()
        output = pred.compute(target=10.0, measurement=0.0)
        self.assertGreater(output, 0)

    def test_negative_error_negative_output(self):
        pred = SmithPredictorSimulator()
        output = pred.compute(target=0.0, measurement=10.0)
        self.assertLess(output, 0)

    def test_output_clamped(self):
        pred = SmithPredictorSimulator()
        pred.out_min = -50.0
        pred.out_max = 50.0
        pred.pid_Kp = 1000.0
        output = pred.compute(target=100.0, measurement=0.0)
        self.assertLessEqual(output, 50.0)
        self.assertGreaterEqual(output, -50.0)


class TestSmithCompensation(unittest.TestCase):
    """预估补偿测试"""

    def test_predictor_output_initially_zero(self):
        """初始时预估输出应为0"""
        pred = SmithPredictorSimulator()
        self.assertEqual(pred.predictor_output, 0.0)

    def test_compensation_develops_over_time(self):
        """经过延迟时间后补偿应生效"""
        pred = SmithPredictorSimulator(dt=0.01)
        pred.set_model(1.0, 1.0, 0.05)  # L=0.05 -> 5 steps
        for _ in range(20):
            pred.compute(target=10.0, measurement=5.0)
        # 补偿量应非零
        self.assertNotAlmostEqual(pred.predictor_output, 0.0, places=3)


class TestSmithDelayBuffer(unittest.TestCase):
    """延迟缓冲区测试"""

    def test_delay_circular_buffer(self):
        """环形缓冲区应正确工作"""
        pred = SmithPredictorSimulator(dt=0.01)
        pred.set_model(1.0, 1.0, 0.02)  # 2 steps delay
        for _ in range(10):
            pred.compute(target=10.0, measurement=5.0)
        # delay_index应循环
        self.assertLess(pred.delay_index, pred.delay_size)


class TestSmithReset(unittest.TestCase):
    """复位测试"""

    def test_reset_clears_state(self):
        pred = SmithPredictorSimulator()
        pred.compute(target=10.0, measurement=5.0)
        pred.reset()
        self.assertEqual(pred.pid_error, 0.0)
        self.assertEqual(pred.pid_error_last, 0.0)
        self.assertEqual(pred.pid_integral, 0.0)
        self.assertEqual(pred.model_state, 0.0)
        self.assertEqual(pred.predictor_output, 0.0)
        self.assertEqual(pred.compensated_feedback, 0.0)

    def test_reset_clears_buffer(self):
        """复位应清零延迟缓冲区"""
        pred = SmithPredictorSimulator()
        pred.compute(target=10.0, measurement=5.0)
        pred.reset()
        for v in pred.delay_buffer:
            self.assertEqual(v, 0.0)


class TestSmithDebug(unittest.TestCase):
    """调试信息测试"""

    def test_get_debug(self):
        pred = SmithPredictorSimulator()
        model_out, delay_out, comp_fb = pred.get_debug()
        self.assertIsInstance(model_out, float)
        self.assertIsInstance(delay_out, float)
        self.assertIsInstance(comp_fb, float)


class TestSmithIntegral(unittest.TestCase):
    """积分测试"""

    def test_integral_accumulation(self):
        pred = SmithPredictorSimulator(dt=0.01)
        pred.compute(target=10.0, measurement=0.0)
        self.assertGreater(pred.pid_integral, 0)

    def test_integral_clamping(self):
        pred = SmithPredictorSimulator(dt=0.01)
        pred.integral_max = 5.0
        for _ in range(10000):
            pred.compute(target=100.0, measurement=0.0)
        self.assertLessEqual(pred.pid_integral, 5.0)
        self.assertGreaterEqual(pred.pid_integral, -5.0)


class TestSmithConvergence(unittest.TestCase):
    """收敛性测试"""

    def test_step_response_converges(self):
        """阶跃响应应趋近目标"""
        pred = SmithPredictorSimulator(dt=0.01)
        pred.set_model(1.0, 1.0, 0.1)
        pred.set_pid(2.0, 1.0, 0.5)
        pred.out_min = -500
        pred.out_max = 500
        pred.integral_max = 200

        state = 0.0
        for _ in range(5000):
            output = pred.compute(target=100.0, measurement=state)
            state += output * 0.001

        self.assertGreater(state, 30.0)


if __name__ == '__main__':
    unittest.main()
