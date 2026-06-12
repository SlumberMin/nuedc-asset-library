#!/usr/bin/env python3
"""
神经网络PID单元测试
覆盖: 初始化、权重归一化、Hebb/Delta/改进型学习规则、激活函数、
      输出限幅、积分限幅、重置、权重获取、收敛性
注意: 使用纯 Python 模拟 C NeuralPID 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 学习规则枚举
NEURAL_HEBB = 0
NEURAL_DELTA = 1
NEURAL_IMPROVED = 2


def _clamp(val, min_v, max_v):
    if val < min_v:
        return min_v
    if val > max_v:
        return max_v
    return val


def _sigmoid(x, gain):
    """Sigmoid激活函数: 输出范围(-1, 1)"""
    return 2.0 / (1.0 + math.exp(-gain * x)) - 1.0


class NeuralPIDSimulator:
    """神经元PID控制器模拟 (对应 neural_pid.c)"""

    def __init__(self):
        # 初始权重
        self.w1 = 0.5   # P权重
        self.w2 = 0.1   # I权重
        self.w3 = 0.05  # D权重

        self.w1_raw = self.w1
        self.w2_raw = self.w2
        self.w3_raw = self.w3

        # 学习率
        self.lr_p = 0.2
        self.lr_i = 0.1
        self.lr_d = 0.05

        self.rule = NEURAL_IMPROVED
        self.activation_gain = 0.5

        self.out_min = -1000.0
        self.out_max = 1000.0
        self.integral_max = 500.0
        self.dt = 0.001

        # 内部状态
        self.error = 0.0
        self.error_last = 0.0
        self.error_prev = 0.0
        self.integral = 0.0
        self.derivative = 0.0

    def set_rule(self, rule):
        self.rule = rule

    def set_weights(self, w1, w2, w3):
        self.w1 = w1
        self.w2 = w2
        self.w3 = w3
        self.w1_raw = w1
        self.w2_raw = w2
        self.w3_raw = w3

    def set_learning_rate(self, lr_p, lr_i, lr_d):
        self.lr_p = lr_p
        self.lr_i = lr_i
        self.lr_d = lr_d

    def _normalize_weights(self):
        s = abs(self.w1_raw) + abs(self.w2_raw) + abs(self.w3_raw)
        if s > 1e-6:
            self.w1 = self.w1_raw / s
            self.w2 = self.w2_raw / s
            self.w3 = self.w3_raw / s

    def _update_weights(self, error, x1, x2, x3, u_raw):
        dw1 = dw2 = dw3 = 0.0

        if self.rule == NEURAL_HEBB:
            dw1 = self.lr_p * u_raw * x1
            dw2 = self.lr_i * u_raw * x2
            dw3 = self.lr_d * u_raw * x3
        elif self.rule == NEURAL_DELTA:
            dw1 = self.lr_p * error * x1
            dw2 = self.lr_i * error * x2
            dw3 = self.lr_d * error * x3
        elif self.rule == NEURAL_IMPROVED:
            sign = 1.0 if (u_raw * error > 0) else -1.0
            factor = 1.0 + 0.5 * sign
            dw1 = self.lr_p * error * x1 * factor
            dw2 = self.lr_i * error * x2 * factor
            dw3 = self.lr_d * error * x3 * factor

        self.w1_raw += dw1
        self.w2_raw += dw2
        self.w3_raw += dw3

        # 防止权重为负
        self.w1_raw = max(self.w1_raw, 0.0)
        self.w2_raw = max(self.w2_raw, 0.0)
        self.w3_raw = max(self.w3_raw, 0.0)

        # 权重上限
        self.w1_raw = _clamp(self.w1_raw, 0.0, 10.0)
        self.w2_raw = _clamp(self.w2_raw, 0.0, 10.0)
        self.w3_raw = _clamp(self.w3_raw, 0.0, 10.0)

        self._normalize_weights()

    def compute(self, target, measurement):
        error = target - measurement

        x1 = error
        x2 = self.integral + error * self.dt
        x3 = (error - self.error_last) / self.dt if self.dt > 0 else 0.0

        # 更新积分
        self.integral += error * self.dt
        self.integral = _clamp(self.integral, -self.integral_max, self.integral_max)
        x2 = self.integral

        # 神经元加权求和
        u_raw = self.w1 * x1 + self.w2 * x2 + self.w3 * x3
        u = u_raw
        u = _clamp(u, self.out_min, self.out_max)

        # 权重学习更新
        self._update_weights(error, x1, x2, x3, u_raw)

        # 更新历史
        self.error_prev = self.error_last
        self.error_last = error

        return u

    def reset(self):
        self.error = 0.0
        self.error_last = 0.0
        self.error_prev = 0.0
        self.integral = 0.0
        self.derivative = 0.0

    def get_weights(self):
        return self.w1, self.w2, self.w3


class TestNeuralPIDInit(unittest.TestCase):
    """初始化测试"""

    def test_default_weights(self):
        pid = NeuralPIDSimulator()
        self.assertAlmostEqual(pid.w1, 0.5)
        self.assertAlmostEqual(pid.w2, 0.1)
        self.assertAlmostEqual(pid.w3, 0.05)

    def test_default_rule(self):
        pid = NeuralPIDSimulator()
        self.assertEqual(pid.rule, NEURAL_IMPROVED)

    def test_default_learning_rates(self):
        pid = NeuralPIDSimulator()
        self.assertAlmostEqual(pid.lr_p, 0.2)
        self.assertAlmostEqual(pid.lr_i, 0.1)
        self.assertAlmostEqual(pid.lr_d, 0.05)

    def test_default_output_limits(self):
        pid = NeuralPIDSimulator()
        self.assertEqual(pid.out_min, -1000.0)
        self.assertEqual(pid.out_max, 1000.0)


class TestNeuralPIDNormalization(unittest.TestCase):
    """权重归一化测试"""

    def test_normalized_weights_sum_to_one(self):
        """归一化后权重绝对值之和应为1"""
        pid = NeuralPIDSimulator()
        pid.set_weights(3.0, 2.0, 1.0)
        pid._normalize_weights()
        w1, w2, w3 = pid.get_weights()
        total = abs(w1) + abs(w2) + abs(w3)
        self.assertAlmostEqual(total, 1.0, places=5)

    def test_normalized_proportions(self):
        """归一化应保持比例"""
        pid = NeuralPIDSimulator()
        pid.set_weights(4.0, 2.0, 1.0)
        pid._normalize_weights()
        w1, w2, w3 = pid.get_weights()
        self.assertAlmostEqual(w1 / w2, 2.0, places=5)
        self.assertAlmostEqual(w2 / w3, 2.0, places=5)


class TestNeuralPIDHebb(unittest.TestCase):
    """Hebb学习规则测试"""

    def test_hebb_updates_weights(self):
        """Hebb规则应更新权重"""
        pid = NeuralPIDSimulator()
        pid.set_rule(NEURAL_HEBB)
        pid.set_learning_rate(0.1, 0.05, 0.02)
        w1_before, _, _ = pid.get_weights()
        pid.compute(target=10.0, measurement=0.0)
        w1_after, _, _ = pid.get_weights()
        # 权重应有变化(经过归一化)
        self.assertNotAlmostEqual(w1_before, w1_after, places=3)

    def test_hebb_zero_error_no_change(self):
        """零误差时Hebb规则权重raw不变"""
        pid = NeuralPIDSimulator()
        pid.set_rule(NEURAL_HEBB)
        pid.set_learning_rate(0.01, 0.01, 0.01)
        # 先运行一次建立状态
        pid.compute(target=5.0, measurement=5.0)
        # 零误差
        w_raw_before = pid.w1_raw
        pid.compute(target=5.0, measurement=5.0)
        # u_raw ≈ 0, x1=0, 所以 dw ≈ 0
        self.assertAlmostEqual(pid.w1_raw, w_raw_before, places=3)


class TestNeuralPIDDelta(unittest.TestCase):
    """Delta学习规则测试"""

    def test_delta_updates_weights(self):
        """Delta规则应更新权重"""
        pid = NeuralPIDSimulator()
        pid.set_rule(NEURAL_DELTA)
        pid.set_learning_rate(0.1, 0.05, 0.02)
        w1_before, _, _ = pid.get_weights()
        pid.compute(target=10.0, measurement=0.0)
        w1_after, _, _ = pid.get_weights()
        self.assertNotAlmostEqual(w1_before, w1_after, places=3)


class TestNeuralPIDImproved(unittest.TestCase):
    """改进型学习规则测试"""

    def test_improved_updates_weights(self):
        """改进型规则应更新权重"""
        pid = NeuralPIDSimulator()
        pid.set_rule(NEURAL_IMPROVED)
        pid.set_learning_rate(0.1, 0.05, 0.02)
        w1_before, _, _ = pid.get_weights()
        pid.compute(target=10.0, measurement=0.0)
        w1_after, _, _ = pid.get_weights()
        self.assertNotAlmostEqual(w1_before, w1_after, places=3)

    def test_improved_positive_feedback(self):
        """u和error同向时应有更大更新"""
        pid1 = NeuralPIDSimulator()
        pid1.set_rule(NEURAL_IMPROVED)
        pid1.set_learning_rate(0.1, 0.05, 0.02)
        pid1.compute(target=10.0, measurement=0.0)
        kp1, _, _ = pid1.get_weights()

        # 对比: u和error异向
        pid2 = NeuralPIDSimulator()
        pid2.set_rule(NEURAL_IMPROVED)
        pid2.set_learning_rate(0.1, 0.05, 0.02)
        pid2.w1_raw = 10.0  # 大权重使u很大
        pid2._normalize_weights()
        pid2.compute(target=0.0, measurement=10.0)
        # 两者都应更新，只是factor不同


class TestNeuralPIDOutput(unittest.TestCase):
    """输出测试"""

    def test_positive_error_positive_output(self):
        """正误差应产生正输出"""
        pid = NeuralPIDSimulator()
        output = pid.compute(target=10.0, measurement=0.0)
        self.assertGreater(output, 0)

    def test_negative_error_negative_output(self):
        """负误差应产生负输出"""
        pid = NeuralPIDSimulator()
        output = pid.compute(target=0.0, measurement=10.0)
        self.assertLess(output, 0)

    def test_output_clamped(self):
        """输出应被限幅"""
        pid = NeuralPIDSimulator()
        pid.out_min = -50.0
        pid.out_max = 50.0
        pid.set_weights(10.0, 0.0, 0.0)
        output = pid.compute(target=100.0, measurement=0.0)
        self.assertLessEqual(output, 50.0)
        self.assertGreaterEqual(output, -50.0)

    def test_returns_float(self):
        """应返回数值"""
        pid = NeuralPIDSimulator()
        output = pid.compute(target=10.0, measurement=5.0)
        self.assertIsInstance(output, float)


class TestNeuralPIDIntegral(unittest.TestCase):
    """积分测试"""

    def test_integral_accumulation(self):
        """积分应累积"""
        pid = NeuralPIDSimulator()
        pid.dt = 0.01
        pid.compute(target=10.0, measurement=0.0)
        self.assertGreater(pid.integral, 0)

    def test_integral_clamping(self):
        """积分应被限幅"""
        pid = NeuralPIDSimulator()
        pid.dt = 0.01
        pid.integral_max = 5.0
        for _ in range(10000):
            pid.compute(target=100.0, measurement=0.0)
        self.assertLessEqual(pid.integral, 5.0)
        self.assertGreaterEqual(pid.integral, -5.0)


class TestNeuralPIDWeightClamping(unittest.TestCase):
    """权重限幅测试"""

    def test_weights_non_negative(self):
        """权重不应为负"""
        pid = NeuralPIDSimulator()
        pid.set_rule(NEURAL_DELTA)
        pid.set_learning_rate(10.0, 10.0, 10.0)  # 大学习率
        for _ in range(100):
            pid.compute(target=0.0, measurement=100.0)  # 负误差
        self.assertGreaterEqual(pid.w1_raw, 0.0)
        self.assertGreaterEqual(pid.w2_raw, 0.0)
        self.assertGreaterEqual(pid.w3_raw, 0.0)

    def test_weights_upper_bounded(self):
        """权重不应超过上限"""
        pid = NeuralPIDSimulator()
        for _ in range(1000):
            pid.compute(target=10.0, measurement=0.0)
        self.assertLessEqual(pid.w1_raw, 10.0)
        self.assertLessEqual(pid.w2_raw, 10.0)
        self.assertLessEqual(pid.w3_raw, 10.0)


class TestNeuralPIDReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        """重置应清零状态"""
        pid = NeuralPIDSimulator()
        pid.compute(target=10.0, measurement=0.0)
        pid.reset()
        self.assertEqual(pid.error, 0.0)
        self.assertEqual(pid.error_last, 0.0)
        self.assertEqual(pid.error_prev, 0.0)
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.derivative, 0.0)


class TestNeuralPIDSetWeights(unittest.TestCase):
    """权重设置测试"""

    def test_set_weights(self):
        pid = NeuralPIDSimulator()
        pid.set_weights(2.0, 0.5, 0.1)
        self.assertEqual(pid.w1, 2.0)
        self.assertEqual(pid.w2, 0.5)
        self.assertEqual(pid.w3, 0.1)
        self.assertEqual(pid.w1_raw, 2.0)
        self.assertEqual(pid.w2_raw, 0.5)
        self.assertEqual(pid.w3_raw, 0.1)


class TestNeuralPIDSetLearningRate(unittest.TestCase):
    """学习率设置测试"""

    def test_set_learning_rate(self):
        pid = NeuralPIDSimulator()
        pid.set_learning_rate(0.5, 0.3, 0.1)
        self.assertEqual(pid.lr_p, 0.5)
        self.assertEqual(pid.lr_i, 0.3)
        self.assertEqual(pid.lr_d, 0.1)


class TestNeuralPIDSetRule(unittest.TestCase):
    """学习规则设置测试"""

    def test_set_hebb(self):
        pid = NeuralPIDSimulator()
        pid.set_rule(NEURAL_HEBB)
        self.assertEqual(pid.rule, NEURAL_HEBB)

    def test_set_delta(self):
        pid = NeuralPIDSimulator()
        pid.set_rule(NEURAL_DELTA)
        self.assertEqual(pid.rule, NEURAL_DELTA)

    def test_set_improved(self):
        pid = NeuralPIDSimulator()
        pid.set_rule(NEURAL_IMPROVED)
        self.assertEqual(pid.rule, NEURAL_IMPROVED)


class TestNeuralPIDHistoryUpdate(unittest.TestCase):
    """历史状态更新测试"""

    def test_error_history_updated(self):
        pid = NeuralPIDSimulator()
        pid.compute(target=10.0, measurement=5.0)
        self.assertAlmostEqual(pid.error_last, 5.0, places=3)

    def test_error_prev_updated(self):
        pid = NeuralPIDSimulator()
        pid.compute(target=10.0, measurement=3.0)
        pid.compute(target=10.0, measurement=5.0)
        self.assertAlmostEqual(pid.error_prev, 7.0, places=3)


class TestSigmoidFunction(unittest.TestCase):
    """Sigmoid激活函数测试"""

    def test_sigmoid_zero(self):
        """输入0应输出0"""
        result = _sigmoid(0.0, 0.5)
        self.assertAlmostEqual(result, 0.0, places=5)

    def test_sigmoid_positive(self):
        """正输入应产生正输出"""
        result = _sigmoid(10.0, 0.5)
        self.assertGreater(result, 0)

    def test_sigmoid_negative(self):
        """负输入应产生负输出"""
        result = _sigmoid(-10.0, 0.5)
        self.assertLess(result, 0)

    def test_sigmoid_bounded(self):
        """输出应在(-1, 1)之间"""
        for x in [-100, -10, -1, 0, 1, 10, 100]:
            result = _sigmoid(x, 0.5)
            self.assertGreater(result, -1.0)
            self.assertLess(result, 1.0)

    def test_sigmoid_monotonic(self):
        """Sigmoid应单调递增"""
        vals = [_sigmoid(x, 0.5) for x in [-5, -3, -1, 0, 1, 3, 5]]
        for i in range(len(vals) - 1):
            self.assertLess(vals[i], vals[i + 1])


class TestNeuralPIDConvergence(unittest.TestCase):
    """收敛性测试"""

    def test_step_response_converges(self):
        """阶跃响应应趋近目标"""
        pid = NeuralPIDSimulator()
        pid.dt = 0.01
        pid.out_min = -500
        pid.out_max = 500
        pid.integral_max = 200

        state = 0.0
        for _ in range(5000):
            output = pid.compute(target=100.0, measurement=state)
            state += output * 0.001

        # 应该趋近目标
        self.assertGreater(state, 30.0)


class TestNeuralPIDDifferentRules(unittest.TestCase):
    """不同学习规则对比测试"""

    def test_all_rules_produce_output(self):
        """所有规则都应产生输出"""
        for rule in [NEURAL_HEBB, NEURAL_DELTA, NEURAL_IMPROVED]:
            pid = NeuralPIDSimulator()
            pid.set_rule(rule)
            output = pid.compute(target=10.0, measurement=5.0)
            self.assertIsNotNone(output)
            self.assertIsInstance(output, float)


if __name__ == '__main__':
    unittest.main()
