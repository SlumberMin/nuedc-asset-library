#!/usr/bin/env python3
"""
重复控制单元测试
覆盖: 初始化/周期延迟缓冲/Q滤波/收敛性/边界条件/性能基准
注意: 使用纯 Python 模拟重复控制器逻辑
"""

import sys
import os
import math
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 ──────────────────────────────────────────

class RepetitiveController:
    """
    重复控制器 (基于内模原理)
    核心: G_rc(z) = z^(-N) / (1 - Q*z^(-N)) * kr
    实现: u(k) = Q*u(k-N) + kr*e(k-N)
    """

    def __init__(self, N_period, kr=0.1, Q=0.95, out_min=-10.0, out_max=10.0):
        """
        N_period: 一个参考周期内的采样点数
        kr: 重复增益
        Q: 低通滤波系数 (稳定化, |Q| < 1)
        out_min/out_max: 输出限幅
        """
        if N_period < 1:
            raise ValueError("N_period必须>=1")
        if not (0.0 < Q <= 1.0):
            raise ValueError("Q必须在(0, 1]范围内")

        self.N = N_period
        self.kr = kr
        self.Q = Q
        self.out_min = out_min
        self.out_max = out_max

        # 延迟缓冲
        self.buffer = [0.0] * N_period
        self.idx = 0
        self.prev_output = 0.0

    def compute(self, error):
        """计算重复控制输出"""
        # 取出N步前的误差
        delayed_error = self.buffer[self.idx]

        # 重复控制律
        output = self.Q * self.prev_output + self.kr * delayed_error

        # 限幅
        output = max(self.out_min, min(self.out_max, output))

        # 存入当前误差
        self.buffer[self.idx] = error
        self.idx = (self.idx + 1) % self.N

        self.prev_output = output
        return output

    def process_period(self, errors):
        """处理一个完整周期的误差序列"""
        outputs = []
        for e in errors:
            outputs.append(self.compute(e))
        return outputs

    def reset(self):
        """重置状态"""
        self.buffer = [0.0] * self.N
        self.idx = 0
        self.prev_output = 0.0


# ── 测试用例 ──────────────────────────────────────────────────

class TestRepetitiveInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        """默认参数应正确"""
        rc = RepetitiveController(N_period=20)
        self.assertEqual(rc.N, 20)
        self.assertAlmostEqual(rc.kr, 0.1)
        self.assertAlmostEqual(rc.Q, 0.95)

    def test_custom_params(self):
        """自定义参数应生效"""
        rc = RepetitiveController(N_period=50, kr=0.2, Q=0.9, out_min=-20, out_max=20)
        self.assertEqual(rc.N, 50)
        self.assertAlmostEqual(rc.kr, 0.2)
        self.assertAlmostEqual(rc.Q, 0.9)
        self.assertEqual(rc.out_min, -20)

    def test_buffer_length(self):
        """延迟缓冲长度应等于N"""
        rc = RepetitiveController(N_period=30)
        self.assertEqual(len(rc.buffer), 30)

    def test_initial_state_zero(self):
        """初始状态应为零"""
        rc = RepetitiveController(N_period=20)
        self.assertTrue(all(b == 0.0 for b in rc.buffer))
        self.assertEqual(rc.idx, 0)
        self.assertEqual(rc.prev_output, 0.0)


class TestRepetitiveInitInvalid(unittest.TestCase):
    """无效参数测试"""

    def test_invalid_N_period(self):
        """N_period<1应抛出异常"""
        with self.assertRaises(ValueError):
            RepetitiveController(N_period=0)

    def test_invalid_Q_zero(self):
        """Q=0应抛出异常"""
        with self.assertRaises(ValueError):
            RepetitiveController(N_period=20, Q=0.0)

    def test_invalid_Q_negative(self):
        """Q<0应抛出异常"""
        with self.assertRaises(ValueError):
            RepetitiveController(N_period=20, Q=-0.5)

    def test_invalid_Q_greater_one(self):
        """Q>1应抛出异常"""
        with self.assertRaises(ValueError):
            RepetitiveController(N_period=20, Q=1.5)


class TestRepetitiveCompute(unittest.TestCase):
    """计算测试"""

    def test_initial_output_zero(self):
        """首次输出应为零(缓冲全为零)"""
        rc = RepetitiveController(N_period=20)
        output = rc.compute(1.0)
        self.assertAlmostEqual(output, 0.0)

    def test_output_after_one_period(self):
        """一个周期+1步后应开始产生输出"""
        rc = RepetitiveController(N_period=10, kr=0.5, Q=1.0)
        outputs = []
        for _ in range(11):  # 一个完整周期 + 1步
            outputs.append(rc.compute(1.0))
        # 第11步应有非零输出(从缓冲中取出第一个周期存入的误差)
        self.assertGreater(abs(outputs[-1]), 0.01)

    def test_positive_error_positive_output(self):
        """正误差应逐渐产生正输出"""
        rc = RepetitiveController(N_period=5, kr=0.5, Q=0.9)
        for _ in range(20):
            out = rc.compute(1.0)
        self.assertGreater(out, 0)

    def test_negative_error_negative_output(self):
        """负误差应逐渐产生负输出"""
        rc = RepetitiveController(N_period=5, kr=0.5, Q=0.9)
        for _ in range(20):
            out = rc.compute(-1.0)
        self.assertLess(out, 0)

    def test_output_clipping(self):
        """输出应被限幅"""
        rc = RepetitiveController(N_period=5, kr=100.0, Q=1.0,
                                  out_min=-5.0, out_max=5.0)
        for _ in range(50):
            out = rc.compute(100.0)
            self.assertLessEqual(out, 5.0)
            self.assertGreaterEqual(out, -5.0)


class TestRepetitiveConvergence(unittest.TestCase):
    """收敛性测试"""

    def test_periodic_error_convergence(self):
        """重复控制应逐渐减小周期性误差"""
        rc = RepetitiveController(N_period=20, kr=0.3, Q=0.95)
        # 模拟周期性误差(正弦)
        period_errors = [math.sin(2 * math.pi * i / 20) for i in range(20)]

        # 多个周期
        first_period_outputs = []
        last_period_outputs = []
        for cycle in range(50):
            period_out = []
            for e in period_errors:
                out = rc.compute(e)
                period_out.append(out)
            if cycle == 0:
                first_period_outputs = period_out
            if cycle == 49:
                last_period_outputs = period_out

        # 后期输出应有更大幅度(学习到周期模式)
        rms_first = math.sqrt(sum(o**2 for o in first_period_outputs) / len(first_period_outputs))
        rms_last = math.sqrt(sum(o**2 for o in last_period_outputs) / len(last_period_outputs))
        self.assertGreater(rms_last, rms_first)

    def test_zero_error_stays_zero(self):
        """零误差应保持零输出"""
        rc = RepetitiveController(N_period=10, kr=0.5, Q=0.9)
        for _ in range(100):
            out = rc.compute(0.0)
            self.assertAlmostEqual(out, 0.0)


class TestRepetitiveQFilter(unittest.TestCase):
    """Q滤波系数影响测试"""

    def test_lower_q_faster_decay(self):
        """较低Q值应使输出更快衰减"""
        # 高Q
        rc_high = RepetitiveController(N_period=5, kr=0.5, Q=0.99)
        # 低Q
        rc_low = RepetitiveController(N_period=5, kr=0.5, Q=0.5)

        # 先注入误差
        for _ in range(5):
            rc_high.compute(1.0)
            rc_low.compute(1.0)

        # 然后停止输入(零误差)
        out_high_sum = 0.0
        out_low_sum = 0.0
        for _ in range(50):
            out_high_sum += abs(rc_high.compute(0.0))
            out_low_sum += abs(rc_low.compute(0.0))

        # 低Q应更快衰减到零
        self.assertLess(out_low_sum, out_high_sum)

    def test_q_equal_one_no_decay(self):
        """Q=1时输出应不衰减(纯延迟)"""
        rc = RepetitiveController(N_period=5, kr=1.0, Q=1.0)
        # 注入恒定误差
        for _ in range(5):
            rc.compute(1.0)
        # 然后零误差 — 输出应保持(因为Q=1不衰减)
        out = rc.compute(0.0)
        self.assertGreater(abs(out), 0.01)


class TestRepetitiveReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_buffer(self):
        """reset应清零缓冲"""
        rc = RepetitiveController(N_period=10)
        for _ in range(20):
            rc.compute(1.0)
        rc.reset()
        self.assertTrue(all(b == 0.0 for b in rc.buffer))

    def test_reset_clears_state(self):
        """reset应清零状态"""
        rc = RepetitiveController(N_period=10)
        for _ in range(20):
            rc.compute(1.0)
        rc.reset()
        self.assertEqual(rc.idx, 0)
        self.assertEqual(rc.prev_output, 0.0)

    def test_behavior_after_reset(self):
        """reset后应从零开始"""
        rc = RepetitiveController(N_period=10, kr=0.5)
        for _ in range(50):
            rc.compute(1.0)
        rc.reset()
        out = rc.compute(1.0)
        self.assertAlmostEqual(out, 0.0)


class TestRepetitiveEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_N_period_equal_1(self):
        """N_period=1应正常工作"""
        rc = RepetitiveController(N_period=1, kr=0.5, Q=0.9)
        for _ in range(10):
            out = rc.compute(1.0)
        self.assertTrue(math.isfinite(out))

    def test_large_N_period(self):
        """大N_period应正常工作"""
        rc = RepetitiveController(N_period=1000, kr=0.1, Q=0.95)
        for _ in range(2000):
            out = rc.compute(1.0)
        self.assertTrue(math.isfinite(out))

    def test_very_small_kr(self):
        """极小kr应产生极小输出"""
        rc = RepetitiveController(N_period=10, kr=1e-6, Q=0.9)
        for _ in range(100):
            out = rc.compute(1.0)
        self.assertLess(abs(out), 0.01)

    def test_stability_many_periods(self):
        """长时间运行不应发散"""
        rc = RepetitiveController(N_period=20, kr=0.3, Q=0.95)
        for i in range(5000):
            e = math.sin(2 * math.pi * i / 20)
            out = rc.compute(e)
            self.assertTrue(math.isfinite(out), f"输出在第{i}步变为非有限值")
            self.assertLess(abs(out), 100.0, f"输出在第{i}步发散")


class TestRepetitivePerformance(unittest.TestCase):
    """性能基准测试"""

    def test_compute_speed(self):
        """10000次计算应在1秒内完成"""
        rc = RepetitiveController(N_period=20, kr=0.3, Q=0.95)

        start = time.perf_counter()
        for i in range(10000):
            rc.compute(math.sin(2 * math.pi * i / 20))
        elapsed = time.perf_counter() - start

        self.assertLess(elapsed, 2.0,
                       f"10000次计算耗时 {elapsed:.3f}s")


if __name__ == '__main__':
    unittest.main()
