#!/usr/bin/env python3
"""
LQR线性二次调节器单元测试
覆盖: Riccati方程求解、LQR增益计算、倒立摆模型、闭环稳定性、
      线性/非线性仿真、大角度鲁棒性
测试对象: 11_控制算法库/simulation/inverted_pendulum_lqr.py
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '11_控制算法库', 'simulation'))

from inverted_pendulum_lqr import (
    solve_riccati, lqr_gain, inverted_pendulum_model,
    simulate_closed_loop, simulate_nonlinear, check_stability
)


class TestRiccatiSolver(unittest.TestCase):
    """Riccati方程求解测试"""

    def test_riccati_returns_symmetric(self):
        """Riccati方程解P应为对称矩阵"""
        A = np.array([[0, 1], [0, -1]], dtype=float)
        B = np.array([[0], [1]], dtype=float)
        Q = np.eye(2)
        R = np.array([[1.0]])
        P = solve_riccati(A, B, Q, R)
        np.testing.assert_array_almost_equal(P, P.T)

    def test_riccati_positive_definite(self):
        """P应为正定矩阵"""
        A = np.array([[0, 1], [0, -1]], dtype=float)
        B = np.array([[0], [1]], dtype=float)
        Q = np.eye(2)
        R = np.array([[1.0]])
        P = solve_riccati(A, B, Q, R)
        eigenvalues = np.linalg.eigvals(P)
        self.assertTrue(np.all(eigenvalues > -1e-10))

    def test_riccati_satisfies_equation(self):
        """P应满足 CARE: A^T*P + P*A - P*B*R^{-1}*B^T*P + Q ≈ 0"""
        A = np.array([[0, 1], [0, -1]], dtype=float)
        B = np.array([[0], [1]], dtype=float)
        Q = np.diag([10.0, 1.0])
        R = np.array([[1.0]])
        P = solve_riccati(A, B, Q, R)
        R_inv = np.linalg.inv(R)
        residual = A.T @ P + P @ A - P @ B @ R_inv @ B.T @ P + Q
        np.testing.assert_array_almost_equal(residual, np.zeros_like(residual), decimal=6)


class TestLQRGain(unittest.TestCase):
    """LQR增益计算测试"""

    def test_lqr_gain_shape(self):
        """K应为 (1, 4) 矩阵"""
        A, B, C = inverted_pendulum_model()
        Q = np.diag([100, 10, 10, 10])
        R = np.array([[1.0]])
        K, P = lqr_gain(A, B, Q, R)
        self.assertEqual(K.shape, (1, 4))

    def test_lqr_gain_nonzero(self):
        """K不应为零矩阵"""
        A, B, C = inverted_pendulum_model()
        Q = np.diag([100, 10, 10, 10])
        R = np.array([[1.0]])
        K, _ = lqr_gain(A, B, Q, R)
        self.assertGreater(np.max(np.abs(K)), 0.1)


class TestInvertedPendulumModel(unittest.TestCase):
    """倒立摆模型测试"""

    def test_model_dimensions(self):
        """A应为4x4, B应为4x1"""
        A, B, C = inverted_pendulum_model()
        self.assertEqual(A.shape, (4, 4))
        self.assertEqual(B.shape, (4, 1))
        self.assertEqual(C.shape, (4, 4))

    def test_model_open_loop_unstable(self):
        """开环系统应不稳定(有正实部极点)"""
        A, B, C = inverted_pendulum_model()
        eigenvalues = np.linalg.eigvals(A)
        has_positive = np.any(eigenvalues.real > 0)
        self.assertTrue(has_positive, "倒立摆开环应为不稳定系统")

    def test_custom_parameters(self):
        """自定义参数应影响模型"""
        A1, B1, _ = inverted_pendulum_model(M=0.5, m=0.2, l=0.3)
        A2, B2, _ = inverted_pendulum_model(M=2.0, m=0.5, l=0.5)
        self.assertFalse(np.allclose(A1, A2))


class TestClosedLoopStability(unittest.TestCase):
    """闭环稳定性测试"""

    def test_closed_loop_poles_stable(self):
        """LQR闭环极点应全部在左半平面"""
        A, B, _ = inverted_pendulum_model()
        Q = np.diag([100, 10, 10, 10])
        R = np.array([[1.0]])
        K, _ = lqr_gain(A, B, Q, R)
        eigenvalues = check_stability(A, B, K)
        self.assertTrue(np.all(eigenvalues.real < 0),
                       "闭环极点应全部具有负实部")


class TestSimulateClosedLoop(unittest.TestCase):
    """闭环仿真测试"""

    def test_simulation_output_shape(self):
        """仿真输出形状应正确"""
        A, B, _ = inverted_pendulum_model()
        Q = np.diag([100, 10, 10, 10])
        R = np.array([[1.0]])
        K, _ = lqr_gain(A, B, Q, R)
        x0 = np.array([5 * np.pi / 180, 0, 0, 0])
        t, x, u = simulate_closed_loop(A, B, K, x0, (0, 2), dt=0.001)
        self.assertEqual(len(t), len(x))
        self.assertEqual(x.shape[1], 4)

    def test_small_angle_converges(self):
        """小角度应快速收敛到零"""
        A, B, _ = inverted_pendulum_model()
        Q = np.diag([100, 10, 10, 10])
        R = np.array([[1.0]])
        K, _ = lqr_gain(A, B, Q, R)
        x0 = np.array([5 * np.pi / 180, 0, 0, 0])
        t, x, u = simulate_closed_loop(A, B, K, x0, (0, 2), dt=0.001)
        # 最终摆角应接近零
        final_angle = abs(x[-1, 0])
        self.assertLess(final_angle, 0.01)


class TestSimulateNonlinear(unittest.TestCase):
    """非线性仿真测试"""

    def test_nonlinear_output_shape(self):
        """非线性仿真输出形状应正确"""
        M, m, l, g = 0.5, 0.2, 0.3, 9.81
        A, B, _ = inverted_pendulum_model(M, m, l, g)
        Q = np.diag([100, 10, 10, 10])
        R = np.array([[1.0]])
        K, _ = lqr_gain(A, B, Q, R)
        x0 = np.array([15 * np.pi / 180, 0, 0, 0])
        t, x, u = simulate_nonlinear(M, m, l, g, K, x0, (0, 2), dt=0.001)
        self.assertEqual(len(t), len(x))
        self.assertEqual(x.shape[1], 4)

    def test_nonlinear_small_angle_stable(self):
        """小角度非线性仿真应收敛"""
        M, m, l, g = 0.5, 0.2, 0.3, 9.81
        A, B, _ = inverted_pendulum_model(M, m, l, g)
        Q = np.diag([100, 10, 10, 10])
        R = np.array([[1.0]])
        K, _ = lqr_gain(A, B, Q, R)
        x0 = np.array([10 * np.pi / 180, 0, 0, 0])
        t, x, u = simulate_nonlinear(M, m, l, g, K, x0, (0, 3), dt=0.001)
        # 最终角度应接近零
        final_angle = abs(x[-1, 0])
        self.assertLess(final_angle, 0.1)


class TestLQREdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_zero_initial_condition(self):
        """零初始条件应保持平衡"""
        A, B, _ = inverted_pendulum_model()
        Q = np.diag([100, 10, 10, 10])
        R = np.array([[1.0]])
        K, _ = lqr_gain(A, B, Q, R)
        x0 = np.zeros(4)
        t, x, u = simulate_closed_loop(A, B, K, x0, (0, 1), dt=0.001)
        # 应保持在零附近
        self.assertLess(np.max(np.abs(x[:, 0])), 0.001)

    def test_different_Q_penalties(self):
        """改变Q应影响控制行为"""
        A, B, _ = inverted_pendulum_model()
        R = np.array([[1.0]])
        Q1 = np.diag([10, 1, 1, 1])
        Q2 = np.diag([1000, 1, 1, 1])
        K1, _ = lqr_gain(A, B, Q1, R)
        K2, _ = lqr_gain(A, B, Q2, R)
        # 更大的摆角权重应导致更大的第一增益分量
        self.assertGreater(abs(K2[0, 0]), abs(K1[0, 0]))


if __name__ == '__main__':
    unittest.main()
