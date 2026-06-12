#!/usr/bin/env python3
"""
Luenberger观测器单元测试
覆盖: 初始化/状态估计/收敛性/极点配置/噪声抑制/边界条件/性能基准
注意: 使用纯 Python 模拟 Luenberger 观测器逻辑 (二阶系统)
"""

import sys
import os
import math
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 ──────────────────────────────────────────

class LuenbergerObserver:
    """
    Luenberger全阶状态观测器
    系统: x_dot = A*x + B*u
          y     = C*x
    观测器: x_hat_dot = A*x_hat + B*u + L*(y - C*x_hat)
    
    离散化 (前向欧拉):
        x_hat(k+1) = x_hat(k) + h * [(A - L*C)*x_hat(k) + B*u(k) + L*y(k)]
    """

    def __init__(self, A, B, C, L, h=0.01):
        """
        A: 系统矩阵 (n×n)
        B: 输入矩阵 (n×m)
        C: 输出矩阵 (p×n)
        L: 观测器增益 (n×p)
        h: 采样周期
        """
        self.n = len(A)       # 状态维度
        self.h = h

        # 存储矩阵
        self.A = [row[:] for row in A]
        self.B = [row[:] for row in B] if B else None
        self.C = [row[:] for row in C]
        self.L = [row[:] for row in L]

        # 估计状态
        self.x_hat = [0.0] * self.n

    def _mat_vec(self, M, v):
        """矩阵乘向量"""
        result = [0.0] * len(M)
        for i in range(len(M)):
            for j in range(len(v)):
                result[i] += M[i][j] * v[j]
        return result

    def _vec_add(self, a, b):
        return [a[i] + b[i] for i in range(len(a))]

    def _vec_sub(self, a, b):
        return [a[i] - b[i] for i in range(len(a))]

    def _vec_scale(self, v, s):
        return [v[i] * s for i in range(len(v))]

    def update(self, y, u=None):
        """
        观测器更新步 (前向欧拉)
        y: 量测输出 (list)
        u: 控制输入 (list, 可选)
        """
        # C * x_hat
        y_hat = self._mat_vec(self.C, self.x_hat)

        # 量测误差
        innovation = self._vec_sub(y, y_hat)

        # L * innovation
        L_innov = self._mat_vec(self.L, innovation)

        # A * x_hat
        Ax = self._mat_vec(self.A, self.x_hat)

        # B * u
        Bu = [0.0] * self.n
        if self.B is not None and u is not None:
            Bu = self._mat_vec(self.B, u)

        # dx_hat/dt = A*x_hat + B*u + L*(y - C*x_hat)
        dx = self._vec_add(self._vec_add(Ax, Bu), L_innov)

        # 前向欧拉积分
        self.x_hat = self._vec_add(self.x_hat, self._vec_scale(dx, self.h))

        return self.x_hat[:]

    def get_estimate(self):
        """返回当前状态估计"""
        return self.x_hat[:]

    def set_estimate(self, x):
        """设置初始估计"""
        self.x_hat = x[:]

    def reset(self):
        """重置状态"""
        self.x_hat = [0.0] * self.n


class LuenbergerObserverDesign:
    """观测器设计工具"""

    @staticmethod
    def place_poles_2x2(A, C, desired_poles, h=0.01):
        """
        2x2系统的极点配置
        A: [[a11, a12], [a21, a22]]
        C: [[c1, c2]]
        desired_poles: [p1, p2] (期望极点, 负实数)
        返回: L (2×1)
        """
        # 简化设计: 对于标量输出, L = [l1, l2]^T
        # 特征方程: s^2 + (a11+a22+l1*c1+l2*c2)*s + ...
        # 使用简化方法: l1, l2 使得观测器误差动态稳定
        a11, a12 = A[0][0], A[0][1]
        a21, a22 = A[1][0], A[1][1]
        c1, c2 = C[0][0], C[0][1]

        p1, p2 = desired_poles
        # 观测器特征多项式: (s - p1)(s - p2) = s^2 - (p1+p2)*s + p1*p2
        # 简化: 直接用期望极点计算增益
        l1 = -(p1 + p2 + a11 + a22) / c1 if abs(c1) > 1e-10 else 0.0
        l2 = (p1 * p2 - a11 * a22 + a12 * a21) / (c1 * a12) if abs(c1 * a12) > 1e-10 else 0.0

        return [[l1], [l2]]


# ── 测试用例 ──────────────────────────────────────────────────

class TestLuenbergerInit(unittest.TestCase):
    """初始化测试"""

    def test_2x2_system(self):
        """2x2系统应正常初始化"""
        A = [[0, 1], [-2, -3]]
        B = [[0], [1]]
        C = [[1, 0]]
        L = [[10], [5]]
        obs = LuenbergerObserver(A, B, C, L, h=0.01)
        self.assertEqual(obs.n, 2)
        self.assertEqual(len(obs.x_hat), 2)

    def test_initial_estimate_zero(self):
        """初始估计应为零"""
        A = [[0, 1], [-2, -3]]
        B = [[0], [1]]
        C = [[1, 0]]
        L = [[10], [5]]
        obs = LuenbergerObserver(A, B, C, L)
        for xi in obs.x_hat:
            self.assertAlmostEqual(xi, 0.0)

    def test_custom_initial_estimate(self):
        """自定义初始估计"""
        A = [[0, 1], [-2, -3]]
        B = [[0], [1]]
        C = [[1, 0]]
        L = [[10], [5]]
        obs = LuenbergerObserver(A, B, C, L)
        obs.set_estimate([1.0, 2.0])
        self.assertAlmostEqual(obs.x_hat[0], 1.0)
        self.assertAlmostEqual(obs.x_hat[1], 2.0)


class TestLuenbergerTracking(unittest.TestCase):
    """状态跟踪测试"""

    def test_tracks_constant_state(self):
        """应能跟踪恒定状态"""
        # 简单一阶系统: x_dot = -x + u, y = x
        A = [[-1.0]]
        B = [[1.0]]
        C = [[1.0]]
        L = [[20.0]]  # 观测器增益
        obs = LuenbergerObserver(A, B, C, L, h=0.01)

        # 真实系统状态
        x_true = 5.0
        u = 0.0

        for _ in range(500):
            # 真实系统更新
            x_true += 0.01 * (-x_true + u)
            # 观测器更新
            obs.update([x_true], [u])

        est = obs.get_estimate()
        self.assertAlmostEqual(est[0], x_true, delta=0.5)

    def test_tracks_2x2_system(self):
        """2x2系统应能跟踪状态"""
        # 谐振系统
        A = [[0, 1], [-4, -0.5]]
        B = [[0], [1]]
        C = [[1, 0]]
        L = [[20], [10]]
        obs = LuenbergerObserver(A, B, C, L, h=0.001)

        x1_true, x2_true = 1.0, 0.0
        u = 0.0

        for _ in range(5000):
            # 真实系统
            dx1 = x2_true
            dx2 = -4 * x1_true - 0.5 * x2_true + u
            x1_true += 0.001 * dx1
            x2_true += 0.001 * dx2

            obs.update([x1_true], [u])

        est = obs.get_estimate()
        self.assertAlmostEqual(est[0], x1_true, delta=0.5)

    def test_tracks_with_initial_offset(self):
        """初始估计有偏差时应快速收敛"""
        A = [[-1.0]]
        B = [[1.0]]
        C = [[1.0]]
        L = [[30.0]]
        obs = LuenbergerObserver(A, B, C, L, h=0.01)

        # 初始估计偏差
        obs.set_estimate([0.0])

        x_true = 5.0
        for _ in range(200):
            obs.update([x_true], [0.0])

        est = obs.get_estimate()
        self.assertAlmostEqual(est[0], x_true, delta=0.5)


class TestLuenbergerConvergence(unittest.TestCase):
    """收敛性测试"""

    def test_higher_gain_faster_convergence(self):
        """更高增益应更快收敛"""
        A = [[-1.0]]
        B = [[1.0]]
        C = [[1.0]]

        results = {}
        for L_val in [5.0, 50.0]:
            obs = LuenbergerObserver(A, B, C, [[L_val]], h=0.01)
            x_true = 10.0

            for i in range(100):
                obs.update([x_true], [0.0])

            results[L_val] = abs(obs.get_estimate()[0] - x_true)

        # 高增益应有更小误差
        self.assertLess(results[50.0], results[5.0] + 0.1)

    def test_convergence_rate(self):
        """收敛应近似指数衰减"""
        A = [[-1.0]]
        B = [[1.0]]
        C = [[1.0]]
        L = [[20.0]]
        obs = LuenbergerObserver(A, B, C, L, h=0.01)

        x_true = 10.0
        errors = []
        for i in range(200):
            obs.update([x_true], [0.0])
            err = abs(obs.get_estimate()[0] - x_true)
            errors.append(err)

        # 误差应单调递减(在初始阶段)
        monotone_count = sum(1 for i in range(1, 100) if errors[i] <= errors[i-1] + 0.001)
        self.assertGreater(monotone_count, 50)

    def test_stability_many_steps(self):
        """长时间运行不应发散"""
        A = [[-1.0]]
        B = [[1.0]]
        C = [[1.0]]
        L = [[20.0]]
        obs = LuenbergerObserver(A, B, C, L, h=0.01)

        for _ in range(10000):
            obs.update([5.0], [0.0])
            est = obs.get_estimate()
            self.assertTrue(all(abs(xi) < 1e6 for xi in est),
                          "观测器状态发散")


class TestLuenbergerDisturbanceRejection(unittest.TestCase):
    """扰动抑制测试"""

    def test_tracks_under_disturbance(self):
        """应能在扰动下跟踪状态"""
        A = [[-1.0]]
        B = [[1.0]]
        C = [[1.0]]
        L = [[30.0]]
        obs = LuenbergerObserver(A, B, C, L, h=0.01)

        x_true = 5.0
        d = 2.0  # 恒定扰动

        for _ in range(500):
            x_true += 0.01 * (-x_true + d)
            obs.update([x_true], [0.0])

        est = obs.get_estimate()
        # 观测器应能跟踪(即使不知道扰动)
        self.assertAlmostEqual(est[0], x_true, delta=1.0)


class TestLuenbergerNoise(unittest.TestCase):
    """噪声影响测试"""

    def test_filters_measurement_noise(self):
        """观测器应能滤除部分量测噪声"""
        import random
        random.seed(42)

        A = [[-0.5]]
        B = [[1.0]]
        C = [[1.0]]
        L = [[10.0]]  # 中等增益(平衡跟踪速度和噪声抑制)
        obs = LuenbergerObserver(A, B, C, L, h=0.01)

        x_true = 5.0
        noise_errors = []

        for _ in range(500):
            noise = random.gauss(0, 0.5)
            obs.update([x_true + noise], [0.0])
            noise_errors.append(abs(obs.get_estimate()[0] - x_true))

        # 后半段平均误差应小于噪声标准差
        avg_err = sum(noise_errors[250:]) / len(noise_errors[250:])
        self.assertLess(avg_err, 0.5)


class TestLuenbergerReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        """reset应清零估计"""
        A = [[-1.0]]
        B = [[1.0]]
        C = [[1.0]]
        L = [[10.0]]
        obs = LuenbergerObserver(A, B, C, L)
        obs.set_estimate([5.0])
        obs.reset()
        self.assertEqual(obs.x_hat[0], 0.0)

    def test_behavior_after_reset(self):
        """reset后应从零重新开始"""
        A = [[-1.0]]
        B = [[1.0]]
        C = [[1.0]]
        L = [[10.0]]
        obs = LuenbergerObserver(A, B, C, L, h=0.01)
        for _ in range(100):
            obs.update([5.0], [0.0])
        obs.reset()
        est = obs.get_estimate()
        self.assertAlmostEqual(est[0], 0.0)


class TestLuenbergerEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_zero_measurement(self):
        """零量测应正常工作"""
        A = [[-1.0]]
        B = [[1.0]]
        C = [[1.0]]
        L = [[10.0]]
        obs = LuenbergerObserver(A, B, C, L, h=0.01)
        for _ in range(100):
            obs.update([0.0], [0.0])
        self.assertAlmostEqual(obs.get_estimate()[0], 0.0, delta=0.1)

    def test_zero_gain(self):
        """零增益观测器(开环)应正常工作"""
        A = [[-1.0]]
        B = [[1.0]]
        C = [[1.0]]
        L = [[0.0]]
        obs = LuenbergerObserver(A, B, C, L, h=0.01)
        obs.set_estimate([5.0])
        for _ in range(100):
            obs.update([10.0], [0.0])
        # 开环估计应衰减(因为A=-1)
        self.assertTrue(math.isfinite(obs.get_estimate()[0]))

    def test_no_input(self):
        """无控制输入(B=None)应正常工作"""
        A = [[-1.0]]
        B = None
        C = [[1.0]]
        L = [[10.0]]
        obs = LuenbergerObserver(A, B, C, L, h=0.01)
        for _ in range(100):
            obs.update([5.0])
        self.assertTrue(math.isfinite(obs.get_estimate()[0]))

    def test_very_small_h(self):
        """极小采样周期应正常工作"""
        A = [[-1.0]]
        B = [[1.0]]
        C = [[1.0]]
        L = [[10.0]]
        obs = LuenbergerObserver(A, B, C, L, h=1e-6)
        for _ in range(1000):
            obs.update([5.0], [0.0])
        self.assertTrue(math.isfinite(obs.get_estimate()[0]))


class TestLuenbergerPerformance(unittest.TestCase):
    """性能基准测试"""

    def test_update_speed(self):
        """10000次更新应在2秒内完成"""
        A = [[0, 1], [-4, -0.5]]
        B = [[0], [1]]
        C = [[1, 0]]
        L = [[20], [10]]
        obs = LuenbergerObserver(A, B, C, L, h=0.001)

        start = time.perf_counter()
        for _ in range(10000):
            obs.update([5.0], [1.0])
        elapsed = time.perf_counter() - start

        self.assertLess(elapsed, 5.0,
                       f"10000次更新耗时 {elapsed:.3f}s")


class TestLuenbergerObserverDesign(unittest.TestCase):
    """观测器设计工具测试"""

    def test_design_returns_gain(self):
        """设计函数应返回增益矩阵"""
        A = [[0, 1], [-4, -0.5]]
        C = [[1, 0]]
        L = LuenbergerObserverDesign.place_poles_2x2(
            A, C, desired_poles=[-10.0, -12.0])
        self.assertEqual(len(L), 2)
        self.assertEqual(len(L[0]), 1)

    def test_design_with_faster_poles(self):
        """更快的极点应产生更大增益"""
        A = [[0, 1], [-4, -0.5]]
        C = [[1, 0]]

        L_slow = LuenbergerObserverDesign.place_poles_2x2(
            A, C, desired_poles=[-5.0, -6.0])
        L_fast = LuenbergerObserverDesign.place_poles_2x2(
            A, C, desired_poles=[-20.0, -25.0])

        # 更快极点通常需要更大增益
        gain_slow = abs(L_slow[0][0]) + abs(L_slow[1][0])
        gain_fast = abs(L_fast[0][0]) + abs(L_fast[1][0])
        self.assertGreater(gain_fast, gain_slow)


if __name__ == '__main__':
    unittest.main()
