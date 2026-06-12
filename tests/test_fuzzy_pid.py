#!/usr/bin/env python3
"""
模糊PID单元测试
覆盖: 初始化、规则表、模糊推理、PID计算、参数自整定、输出限幅
注意: 使用纯 Python 模拟 C FuzzyPID 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 模糊语言值
NB, NM, NS, ZO, PS, PM, PB = range(7)


def _trapmf(x, a, b, c, d):
    """梯形隶属函数"""
    if x <= a or x >= d:
        return 0.0
    elif a < x < b:
        return (x - a) / (b - a) if b != a else 1.0
    elif b <= x <= c:
        return 1.0
    else:  # c < x < d
        return (d - x) / (d - c) if d != c else 1.0


def _trimf(x, a, b, c):
    """三角形隶属函数"""
    if x <= a or x >= c:
        return 0.0
    elif a < x <= b:
        return (x - a) / (b - a) if b != a else 1.0
    else:  # b < x < c
        return (c - x) / (c - b) if c != b else 1.0


class FuzzyMembership:
    """模糊隶属函数集合"""

    @staticmethod
    def nb(x): return _trapmf(x, -3, -3, -2.5, -1.5)

    @staticmethod
    def nm(x): return _trimf(x, -2.5, -1.5, -0.5)

    @staticmethod
    def ns(x): return _trimf(x, -1.5, -0.5, 0.0)

    @staticmethod
    def zo(x): return _trimf(x, -0.5, 0.0, 0.5)

    @staticmethod
    def ps(x): return _trimf(x, 0.0, 0.5, 1.5)

    @staticmethod
    def pm(x): return _trimf(x, 0.5, 1.5, 2.5)

    @staticmethod
    def pb(x): return _trapmf(x, 1.5, 2.5, 3, 3)

    @classmethod
    def fuzzify(cls, x):
        """模糊化: 返回7个语言值的隶属度"""
        return [cls.nb(x), cls.nm(x), cls.ns(x), cls.zo(x),
                cls.ps(x), cls.pm(x), cls.pb(x)]


class FuzzyPIDSimulator:
    """模糊PID控制器模拟"""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0):
        self.kp_base = kp
        self.ki_base = ki
        self.kd_base = kd
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.target = 0.0
        self.error = 0.0
        self.error_last = 0.0
        self.error_dot = 0.0
        self.integral = 0.0
        self.output = 0.0
        self.output_last = 0.0

        self.delta_kp_max = 0.5
        self.delta_ki_max = 0.1
        self.delta_kd_max = 0.1
        self.e_scale = 1.0
        self.ec_scale = 1.0

        self.output_max = 100.0
        self.output_min = -100.0
        self.integral_max = 50.0

        # 默认规则表 (经典7x7)
        self.rule_kp = [
            [PB, PB, PM, PM, PS, ZO, ZO],
            [PB, PB, PM, PS, PS, ZO, NS],
            [PM, PM, PM, PS, ZO, NS, NS],
            [PM, PM, PS, ZO, NS, NM, NM],
            [PS, PS, ZO, NS, NS, NM, NM],
            [PS, ZO, NS, NM, NM, NM, NB],
            [ZO, ZO, NM, NM, NM, NB, NB],
        ]
        self.rule_ki = [
            [NB, NB, NM, NM, NS, ZO, ZO],
            [NB, NB, NM, NS, NS, ZO, ZO],
            [NB, NM, NS, NS, ZO, PS, PS],
            [NM, NM, NS, ZO, PS, PM, PM],
            [NM, NS, ZO, PS, PS, PM, PB],
            [ZO, ZO, PS, PS, PM, PB, PB],
            [ZO, ZO, PS, PM, PM, PB, PB],
        ]
        self.rule_kd = [
            [PS, NS, NB, NB, NB, NM, PS],
            [PS, NS, NB, NM, NM, NS, ZO],
            [ZO, NS, NM, NM, NS, NS, ZO],
            [ZO, NS, NS, NS, NS, NS, ZO],
            [ZO, ZO, ZO, ZO, ZO, ZO, ZO],
            [PB, NS, PS, PS, PS, PS, PB],
            [PB, PM, PM, PM, PS, PS, PB],
        ]

    def _fuzzy_infer(self, e, ec):
        """模糊推理: 根据误差和误差变化率调整Kp/Ki/Kd"""
        e_fuzzy = FuzzyMembership.fuzzify(e)
        ec_fuzzy = FuzzyMembership.fuzzify(ec)

        delta_kp = 0.0
        delta_ki = 0.0
        delta_kd = 0.0
        w_sum = 0.0

        for i in range(7):
            for j in range(7):
                w = min(e_fuzzy[i], ec_fuzzy[j])
                if w > 0:
                    delta_kp += w * (self.rule_kp[i][j] - 3)
                    delta_ki += w * (self.rule_ki[i][j] - 3)
                    delta_kd += w * (self.rule_kd[i][j] - 3)
                    w_sum += w

        if w_sum > 0:
            delta_kp /= w_sum
            delta_ki /= w_sum
            delta_kd /= w_sum

        # 映射到调整范围
        dkp = delta_kp / 3.0 * self.delta_kp_max
        dki = delta_ki / 3.0 * self.delta_ki_max
        dkd = delta_kd / 3.0 * self.delta_kd_max

        self.kp = max(0, self.kp_base + dkp)
        self.ki = max(0, self.ki_base + dki)
        self.kd = max(0, self.kd_base + dkd)

    def compute(self, measurement, dt=0.01):
        """模糊PID计算"""
        self.error_last = self.error
        self.error = self.target - measurement
        self.error_dot = (self.error - self.error_last) / dt if dt > 0 else 0

        # 模糊推理自整定
        e_scaled = self.error * self.e_scale
        ec_scaled = self.error_dot * self.ec_scale
        self._fuzzy_infer(e_scaled, ec_scaled)

        # PID计算
        self.integral += self.error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))

        self.output = (self.kp * self.error +
                       self.ki * self.integral +
                       self.kd * self.error_dot)
        self.output = max(self.output_min, min(self.output_max, self.output))
        self.output_last = self.output
        return self.output

    def reset(self):
        self.error = 0.0
        self.error_last = 0.0
        self.error_dot = 0.0
        self.integral = 0.0
        self.output = 0.0
        self.output_last = 0.0
        self.kp = self.kp_base
        self.ki = self.ki_base
        self.kd = self.kd_base


# ── 测试用例 ──────────────────────────────────────────────────

class TestFuzzyPIDInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        """默认参数"""
        fp = FuzzyPIDSimulator()
        self.assertEqual(fp.kp_base, 1.0)
        self.assertEqual(fp.ki_base, 0.0)
        self.assertEqual(fp.kd_base, 0.0)

    def test_custom_params(self):
        """自定义参数"""
        fp = FuzzyPIDSimulator(kp=2.0, ki=0.5, kd=0.1)
        self.assertEqual(fp.kp_base, 2.0)
        self.assertEqual(fp.ki_base, 0.5)
        self.assertEqual(fp.kd_base, 0.1)


class TestFuzzyMembership(unittest.TestCase):
    """隶属函数测试"""

    def test_zo_at_zero(self):
        """ZO在零点隶属度应为1"""
        mu = FuzzyMembership.zo(0.0)
        self.assertAlmostEqual(mu, 1.0)

    def test_pb_at_large(self):
        """PB在大正值隶属度应为1"""
        mu = FuzzyMembership.pb(3.0)
        self.assertAlmostEqual(mu, 1.0)

    def test_nb_at_large_neg(self):
        """NB在大负值隶属度应为1"""
        mu = FuzzyMembership.nb(-3.0)
        self.assertAlmostEqual(mu, 1.0)

    def test_fuzzify_sum_positive(self):
        """模糊化后隶属度之和应>0"""
        mus = FuzzyMembership.fuzzify(0.5)
        self.assertGreater(sum(mus), 0)

    def test_fuzzify_symmetry(self):
        """对称点的隶属度应有对称性"""
        mus_pos = FuzzyMembership.fuzzify(1.0)
        mus_neg = FuzzyMembership.fuzzify(-1.0)
        # NB(-1)应≈PB(1), NM(-1)应≈PM(1), ...
        self.assertAlmostEqual(mus_pos[PB], mus_neg[NB], places=2)


class TestFuzzyPIDCompute(unittest.TestCase):
    """模糊PID计算测试"""

    def test_zero_error_zero_output(self):
        """零误差(目标=测量)应产生小输出"""
        fp = FuzzyPIDSimulator(kp=1.0, ki=0.0, kd=0.0)
        fp.target = 50.0
        output = fp.compute(50.0, dt=0.01)
        self.assertAlmostEqual(output, 0.0, delta=0.5)

    def test_positive_error_positive_output(self):
        """正误差应产生正输出"""
        fp = FuzzyPIDSimulator(kp=1.0, ki=0.0, kd=0.0)
        fp.target = 100.0
        output = fp.compute(50.0, dt=0.01)
        self.assertGreater(output, 0)

    def test_negative_error_negative_output(self):
        """负误差应产生负输出"""
        fp = FuzzyPIDSimulator(kp=1.0, ki=0.0, kd=0.0)
        fp.target = 0.0
        output = fp.compute(50.0, dt=0.01)
        self.assertLess(output, 0)

    def test_output_clamped(self):
        """输出应被限幅"""
        fp = FuzzyPIDSimulator(kp=100.0, ki=0.0, kd=0.0)
        fp.output_max = 50.0
        fp.output_min = -50.0
        fp.target = 1000.0
        output = fp.compute(0.0, dt=0.01)
        self.assertLessEqual(output, 50.0)


class TestFuzzyPIDIntegral(unittest.TestCase):
    """积分测试"""

    def test_integral_accumulation(self):
        """积分应累积"""
        fp = FuzzyPIDSimulator(kp=0.0, ki=1.0, kd=0.0)
        fp.target = 10.0
        outputs = []
        for _ in range(10):
            outputs.append(fp.compute(0.0, dt=0.1))
        # 输出应递增(积分累积)
        for i in range(1, len(outputs)):
            self.assertGreaterEqual(outputs[i], outputs[i-1] - 0.01)

    def test_integral_clamping(self):
        """积分应被限幅"""
        fp = FuzzyPIDSimulator(kp=0.0, ki=100.0, kd=0.0)
        fp.integral_max = 10.0
        fp.target = 100.0
        for _ in range(1000):
            fp.compute(0.0, dt=0.01)
        self.assertLessEqual(fp.integral, 10.0)


class TestFuzzyPIDSelfTuning(unittest.TestCase):
    """模糊自整定测试"""

    def test_kp_changes_with_error(self):
        """Kp应随误差变化"""
        fp = FuzzyPIDSimulator(kp=1.0, ki=0.0, kd=0.0)
        fp.delta_kp_max = 0.5
        fp.target = 100.0
        kp_before = fp.kp
        fp.compute(0.0, dt=0.01)  # 大误差
        kp_after = fp.kp
        # Kp应被调整
        self.assertNotAlmostEqual(kp_before, kp_after, places=3)

    def test_params_always_positive(self):
        """调整后参数应非负"""
        fp = FuzzyPIDSimulator(kp=0.1, ki=0.1, kd=0.1)
        fp.target = 100.0
        for _ in range(100):
            fp.compute(0.0, dt=0.01)
        self.assertGreaterEqual(fp.kp, 0)
        self.assertGreaterEqual(fp.ki, 0)
        self.assertGreaterEqual(fp.kd, 0)


class TestFuzzyPIDReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        """reset应清零内部状态"""
        fp = FuzzyPIDSimulator(kp=1.0, ki=0.1, kd=0.01)
        fp.target = 100.0
        fp.compute(50.0, dt=0.01)
        fp.reset()
        self.assertEqual(fp.error, 0.0)
        self.assertEqual(fp.integral, 0.0)
        self.assertEqual(fp.output, 0.0)
        self.assertEqual(fp.kp, fp.kp_base)


class TestFuzzyPIDConvergence(unittest.TestCase):
    """收敛性测试"""

    def test_step_response_converges(self):
        """阶跃响应应趋近目标"""
        fp = FuzzyPIDSimulator(kp=2.0, ki=0.5, kd=0.1)
        fp.output_max = 200.0
        fp.output_min = -200.0
        fp.integral_max = 200.0
        fp.target = 100.0
        state = 0.0
        dt = 0.01
        for _ in range(2000):
            output = fp.compute(state, dt=dt)
            state += output * dt * 0.1
        self.assertGreater(state, 50.0)


class TestFuzzyRuleTable(unittest.TestCase):
    """规则表测试"""

    def test_rule_table_size(self):
        """规则表应为7x7"""
        fp = FuzzyPIDSimulator()
        self.assertEqual(len(fp.rule_kp), 7)
        for row in fp.rule_kp:
            self.assertEqual(len(row), 7)

    def test_rule_values_in_range(self):
        """规则值应在[0,6]范围内"""
        fp = FuzzyPIDSimulator()
        for row in fp.rule_kp:
            for v in row:
                self.assertIn(v, range(7))


if __name__ == '__main__':
    unittest.main()
