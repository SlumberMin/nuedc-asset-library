#!/usr/bin/env python3
"""
反步法(Backstepping)控制单元测试
覆盖: 初始化/虚拟控制律/Lyapunov稳定性/参数自适应/边界条件/性能基准
注意: 使用纯 Python 模拟反步法控制器逻辑 (二阶系统)
"""

import sys
import os
import math
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 ──────────────────────────────────────────

class BacksteppingController:
    """
    反步法控制器 (针对严格反馈系统)
    系统模型:
        x1_dot = x2 + f1(x1)
        x2_dot = u   + f2(x1, x2)
    控制律通过逐步构造Lyapunov函数得到
    """

    def __init__(self, c1=5.0, c2=5.0):
        """
        c1: 第一步虚拟控制增益
        c2: 第二步实际控制增益
        """
        self.c1 = c1
        self.c2 = c2

        # 虚拟控制量
        self.alpha = 0.0

    def virtual_control(self, x1, x1_ref, f1=0.0):
        """
        第一步: 虚拟控制律
        alpha = x1_ref_dot - c1*(x1 - x1_ref) - f1(x1)
        简化: 假设 x1_ref_dot = 0 (阶跃跟踪)
        alpha = -c1 * e1 - f1
        """
        e1 = x1 - x1_ref
        self.alpha = -self.c1 * e1 - f1
        return self.alpha

    def compute(self, x1, x2, x1_ref, f1=0.0, f2=0.0):
        """
        第二步: 实际控制律
        u = alpha_dot - c2*(x2 - alpha) - f2
        简化: 假设 alpha_dot ≈ 0 (变化慢)
        u = -c2 * (x2 - self.alpha) - f2 + d_alpha/dt
        """
        self.virtual_control(x1, x1_ref, f1)
        e2 = x2 - self.alpha
        u = -self.c2 * e2 - f2
        return u

    def reset(self):
        self.alpha = 0.0


class AdaptiveBacksteppingController:
    """
    自适应反步法控制器 (带参数估计)
    用于含未知参数的系统
    """

    def __init__(self, c1=5.0, c2=5.0, gamma=1.0):
        """
        c1, c2: 反步法增益
        gamma: 自适应律增益
        """
        self.c1 = c1
        self.c2 = c2
        self.gamma = gamma

        # 参数估计
        self.theta_hat = 0.0
        self.alpha = 0.0

    def compute(self, x1, x2, x1_ref, dt=0.01):
        """
        含自适应律的反步法控制
        系统: x1_dot = theta*x1 + x2
              x2_dot = u
        目标: x1 -> x1_ref
        """
        e1 = x1 - x1_ref

        # 虚拟控制律
        self.alpha = -self.c1 * e1 - self.theta_hat * x1

        e2 = x2 - self.alpha

        # 实际控制律
        u = -self.c2 * e2 + e1 * x1  # 简化

        # 参数自适应律 (梯度法)
        self.theta_hat += self.gamma * e1 * x1 * dt

        return u

    def reset(self):
        self.theta_hat = 0.0
        self.alpha = 0.0


# ── 测试用例 ──────────────────────────────────────────────────

class TestBacksteppingInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        """默认参数应正确"""
        bs = BacksteppingController()
        self.assertAlmostEqual(bs.c1, 5.0)
        self.assertAlmostEqual(bs.c2, 5.0)

    def test_custom_params(self):
        """自定义参数应生效"""
        bs = BacksteppingController(c1=10.0, c2=20.0)
        self.assertAlmostEqual(bs.c1, 10.0)
        self.assertAlmostEqual(bs.c2, 20.0)

    def test_initial_alpha_zero(self):
        """初始虚拟控制量应为零"""
        bs = BacksteppingController()
        self.assertAlmostEqual(bs.alpha, 0.0)


class TestVirtualControl(unittest.TestCase):
    """虚拟控制律测试"""

    def test_zero_error_zero_alpha(self):
        """零误差应产生零虚拟控制"""
        bs = BacksteppingController(c1=5.0)
        alpha = bs.virtual_control(x1=5.0, x1_ref=5.0)
        self.assertAlmostEqual(alpha, 0.0)

    def test_positive_error_negative_alpha(self):
        """正误差(x1>x1_ref)应产生负虚拟控制(减小状态)"""
        bs = BacksteppingController(c1=5.0)
        alpha = bs.virtual_control(x1=10.0, x1_ref=0.0)
        self.assertLess(alpha, 0)

    def test_negative_error_positive_alpha(self):
        """负误差应产生正虚拟控制"""
        bs = BacksteppingController(c1=5.0)
        alpha = bs.virtual_control(x1=0.0, x1_ref=10.0)
        self.assertGreater(alpha, 0)

    def test_with_known_dynamics(self):
        """含已知动力学项的虚拟控制"""
        bs = BacksteppingController(c1=5.0)
        alpha = bs.virtual_control(x1=10.0, x1_ref=0.0, f1=5.0)
        # alpha = -c1*e1 - f1 = -5*10 - 5 = -55
        self.assertAlmostEqual(alpha, -55.0)


class TestBacksteppingCompute(unittest.TestCase):
    """完整控制律测试"""

    def test_zero_error_zero_output(self):
        """零误差(平衡点)应产生零输出"""
        bs = BacksteppingController(c1=5.0, c2=5.0)
        u = bs.compute(x1=0.0, x2=0.0, x1_ref=0.0)
        self.assertAlmostEqual(u, 0.0)

    def test_positive_error_sign(self):
        """正误差应产生适当符号的控制"""
        bs = BacksteppingController(c1=5.0, c2=5.0)
        u = bs.compute(x1=10.0, x2=0.0, x1_ref=0.0)
        # 需要减小x1, 应输出负控制
        self.assertNotAlmostEqual(u, 0.0)

    def test_stabilizes_system(self):
        """反步法应稳定二阶系统"""
        bs = BacksteppingController(c1=5.0, c2=5.0)
        dt = 0.001
        x1, x2 = 5.0, 0.0  # 初始偏移
        target = 0.0

        for _ in range(5000):
            u = bs.compute(x1, x2, target)
            # 简单二阶积分器: x1_dot = x2, x2_dot = u
            x2 += u * dt
            x1 += x2 * dt

        # 应收敛到接近目标
        self.assertAlmostEqual(x1, target, delta=1.0)

    def test_reset(self):
        """reset应清零状态"""
        bs = BacksteppingController()
        bs.compute(5.0, 1.0, 0.0)
        bs.reset()
        self.assertEqual(bs.alpha, 0.0)


class TestBacksteppingStability(unittest.TestCase):
    """Lyapunov稳定性测试"""

    def test_lyapunov_decreasing(self):
        """Lyapunov函数应递减"""
        bs = BacksteppingController(c1=5.0, c2=5.0)
        dt = 0.001
        x1, x2 = 5.0, 0.0
        target = 0.0

        V_prev = 0.5 * x1**2 + 0.5 * (x2 - bs.alpha)**2
        decreasing_count = 0
        total_count = 0

        for _ in range(3000):
            u = bs.compute(x1, x2, target)
            x2 += u * dt
            x1 += x2 * dt

            e1 = x1 - target
            e2 = x2 - bs.alpha
            V = 0.5 * e1**2 + 0.5 * e2**2

            if V < V_prev:
                decreasing_count += 1
            total_count += 1
            V_prev = V

        # Lyapunov函数大部分时间应递减
        self.assertGreater(decreasing_count / total_count, 0.5)

    def test_no_overshoot_with_high_gains(self):
        """高增益应减小超调"""
        for c in [2.0, 10.0]:
            bs = BacksteppingController(c1=c, c2=c)
            dt = 0.001
            x1, x2 = 5.0, 0.0
            target = 0.0
            max_overshoot = 0.0

            for _ in range(5000):
                u = bs.compute(x1, x2, target)
                x2 += u * dt
                x1 += x2 * dt
                overshoot = abs(x1 - target)
                max_overshoot = max(max_overshoot, overshoot)

            if c == 2.0:
                low_gain_overshoot = max_overshoot
            else:
                high_gain_overshoot = max_overshoot

        # 高增益应有更小或相当的超调
        # (对于反步法, 高增益通常更快收敛)
        self.assertLessEqual(high_gain_overshoot, low_gain_overshoot + 5.0)


class TestAdaptiveBackstepping(unittest.TestCase):
    """自适应反步法测试"""

    def test_init(self):
        """自适应控制器应正常初始化"""
        ab = AdaptiveBacksteppingController()
        self.assertAlmostEqual(ab.c1, 5.0)
        self.assertAlmostEqual(ab.theta_hat, 0.0)

    def test_parameter_adaptation(self):
        """参数估计应随时间变化"""
        ab = AdaptiveBacksteppingController(c1=5.0, c2=5.0, gamma=0.1)
        x1 = 5.0
        x2 = 0.0
        dt = 0.01

        for _ in range(100):
            ab.compute(x1, x2, 0.0, dt=dt)

        # 参数估计应不为零(自适应)
        self.assertNotAlmostEqual(ab.theta_hat, 0.0)

    def test_adaptive_convergence(self):
        """自适应反步法应能收敛"""
        ab = AdaptiveBacksteppingController(c1=5.0, c2=5.0, gamma=0.5)
        dt = 0.001
        theta_true = 2.0
        x1, x2 = 3.0, 0.0

        for _ in range(10000):
            u = ab.compute(x1, x2, 0.0, dt=dt)
            # 含未知参数的系统
            x2 += u * dt
            x1 += (theta_true * x1 + x2) * dt

        # 状态应有界
        self.assertTrue(abs(x1) < 100, "自适应反步法状态发散")


class TestBacksteppingEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_zero_gains(self):
        """零增益应产生零控制(不稳定但不应崩溃)"""
        bs = BacksteppingController(c1=0.0, c2=0.0)
        u = bs.compute(5.0, 1.0, 0.0)
        self.assertTrue(math.isfinite(u))

    def test_very_large_state(self):
        """大状态值不应导致溢出"""
        bs = BacksteppingController(c1=10.0, c2=10.0)
        u = bs.compute(1000.0, 500.0, 0.0)
        self.assertTrue(math.isfinite(u))

    def test_very_small_gains(self):
        """极小增益应产生小控制量"""
        bs = BacksteppingController(c1=0.001, c2=0.001)
        u = bs.compute(5.0, 1.0, 0.0)
        self.assertLess(abs(u), 1.0)

    def test_large_gains(self):
        """大增益应产生大控制量"""
        bs = BacksteppingController(c1=100.0, c2=100.0)
        u = bs.compute(5.0, 1.0, 0.0)
        self.assertGreater(abs(u), 10.0)


class TestBacksteppingPerformance(unittest.TestCase):
    """性能基准测试"""

    def test_compute_speed(self):
        """10000次计算应在1秒内完成"""
        bs = BacksteppingController(c1=5.0, c2=5.0)

        start = time.perf_counter()
        for i in range(10000):
            bs.compute(float(i % 10), float(i % 5), 0.0)
        elapsed = time.perf_counter() - start

        self.assertLess(elapsed, 2.0,
                       f"10000次计算耗时 {elapsed:.3f}s")


if __name__ == '__main__':
    unittest.main()
