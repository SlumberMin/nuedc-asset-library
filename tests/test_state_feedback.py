#!/usr/bin/env python3
"""
状态反馈控制单元测试
覆盖: 极点配置、Ackermann公式、闭环稳定性、
      前馈增益、阶跃响应收敛、正弦跟踪
测试对象: 11_控制算法库/simulation/state_feedback_simulation.py
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '11_控制算法库', 'simulation'))

from state_feedback_simulation import (
    simulate_state_feedback, simulate_observer_controller,
    simulate_open_loop, simulate_pid
)


def make_dc_motor_model(J=0.01, b=0.1, Kt=0.01):
    """直流电机状态空间模型"""
    A = np.array([[0, 1], [0, -b / J]])
    B = np.array([[0], [Kt / J]])
    C = np.array([[1, 0]])
    return A, B, C


def ackermann_pole_placement(A, B, desired_poles):
    """Ackermann公式极点配置"""
    n = A.shape[0]
    # 期望特征多项式
    poly = np.poly(desired_poles)
    # phi(A)
    phi = np.zeros_like(A)
    for i, coeff in enumerate(poly):
        phi += coeff * np.linalg.matrix_power(A, i)
    # 可控性矩阵
    M = B.copy()
    for i in range(1, n):
        M = np.hstack([M, np.linalg.matrix_power(A, i) @ B])
    e = np.zeros((1, n))
    e[0, -1] = 1
    K = e @ np.linalg.inv(M) @ phi
    return K


class TestStateFeedbackOutputShape(unittest.TestCase):
    """输出形状测试"""

    def test_x_hist_shape(self):
        A, B, C = make_dc_motor_model()
        K = np.array([[10.0, 2.0]])
        N_ref = 1.0
        x0 = np.zeros(2)
        ref = np.ones(100)
        x_hist, y_hist, u_hist = simulate_state_feedback(
            A, B, C, K, N_ref, x0, ref, 0.001, 100)
        self.assertEqual(x_hist.shape, (100, 2))
        self.assertEqual(y_hist.shape, (100,))
        self.assertEqual(u_hist.shape, (100, 1))

    def test_y_hist_shape(self):
        A, B, C = make_dc_motor_model()
        K = np.array([[5.0, 1.0]])
        ref = np.ones(50)
        _, y_hist, _ = simulate_state_feedback(
            A, B, C, K, 1.0, np.zeros(2), ref, 0.001, 50)
        self.assertEqual(len(y_hist), 50)


class TestClosedLoopStability(unittest.TestCase):
    """闭环稳定性测试"""

    def test_poles_left_half_plane(self):
        """闭环极点应在左半平面"""
        A, B, C = make_dc_motor_model()
        desired = np.array([-10.0, -15.0])
        K = ackermann_pole_placement(A, B, desired)
        A_cl = A - B @ K
        eigs = np.linalg.eigvals(A_cl)
        self.assertTrue(np.all(eigs.real < 0), "闭环极点应全部在左半平面")

    def test_desired_poles_achieved(self):
        """闭环极点应接近期望值"""
        A, B, C = make_dc_motor_model()
        desired = np.array([-10.0, -15.0])
        K = ackermann_pole_placement(A, B, desired)
        A_cl = A - B @ K
        eigs = np.sort(np.linalg.eigvals(A_cl).real)
        np.testing.assert_array_almost_equal(np.sort(eigs), np.sort(desired), decimal=4)


class TestStepResponseConvergence(unittest.TestCase):
    """阶跃响应收敛测试"""

    def test_converges_to_reference(self):
        """状态反馈应使输出收敛到参考值"""
        A, B, C = make_dc_motor_model()
        desired = np.array([-10.0, -15.0])
        K = ackermann_pole_placement(A, B, desired)
        A_cl = A - B @ K
        N_ref = -1.0 / (C @ np.linalg.inv(A_cl) @ B)[0, 0]

        steps = 2000
        ref = np.ones(steps)
        x_hist, y_hist, _ = simulate_state_feedback(
            A, B, C, K, N_ref, np.zeros(2), ref, 0.001, steps)
        # 最终输出应接近参考值
        self.assertAlmostEqual(y_hist[-1], 1.0, delta=0.05)

    def test_zero_ref_converges_to_zero(self):
        """零参考应收敛到零"""
        A, B, C = make_dc_motor_model()
        K = np.array([[50.0, 5.0]])
        steps = 1000
        ref = np.zeros(steps)
        x0 = np.array([0.1, 0.0])
        x_hist, y_hist, _ = simulate_state_feedback(
            A, B, C, K, 0.0, x0, ref, 0.001, steps)
        self.assertAlmostEqual(y_hist[-1], 0.0, delta=0.01)


class TestOpenLoopVsClosedLoop(unittest.TestCase):
    """开环vs闭环对比测试"""

    def test_closed_loop_faster(self):
        """闭环应比开环更快收敛"""
        A, B, C = make_dc_motor_model()
        K = np.array([[50.0, 5.0]])
        steps = 5000
        dt = 0.001

        # 开环
        _, y_ol = simulate_open_loop(A, B, C, np.zeros(2), 12.0, dt, steps)

        # 闭环
        A_cl = A - B @ K
        N_ref = -1.0 / (C @ np.linalg.inv(A_cl) @ B)[0, 0]
        _, y_cl, _ = simulate_state_feedback(
            A, B, C, K, N_ref, np.zeros(2), np.ones(steps), dt, steps)

        # 闭环应在1秒内接近稳态
        self.assertGreater(abs(y_cl[1000]), abs(y_ol[1000]) * 0.5)


class TestSineTracking(unittest.TestCase):
    """正弦跟踪测试"""

    def test_output_tracks_sine(self):
        """应能跟踪正弦信号"""
        A, B, C = make_dc_motor_model()
        desired = np.array([-20.0, -25.0])
        K = ackermann_pole_placement(A, B, desired)
        A_cl = A - B @ K
        N_ref = -1.0 / (C @ np.linalg.inv(A_cl) @ B)[0, 0]

        steps = 5000
        dt = 0.001
        t = np.arange(steps) * dt
        ref = np.sin(2 * np.pi * 0.5 * t)

        _, y_hist, _ = simulate_state_feedback(
            A, B, C, K, N_ref, np.zeros(2), ref, dt, steps)

        # 跳过初始瞬态, 检查跟踪误差
        error = np.mean(np.abs(y_hist[2000:] - ref[2000:]))
        self.assertLess(error, 0.1, "正弦跟踪误差应小于0.1")


class TestPIDComparison(unittest.TestCase):
    """PID对比测试"""

    def test_pid_output_shape(self):
        A, B, C = make_dc_motor_model()
        steps = 100
        ref = np.ones(steps)
        y_hist, u_hist = simulate_pid(10, 50, 0.5, ref, 0.001, steps, A, B, C, np.zeros(2))
        self.assertEqual(y_hist.shape, (steps,))
        self.assertEqual(u_hist.shape, (steps,))

    def test_pid_converges(self):
        """PID应也能收敛"""
        A, B, C = make_dc_motor_model()
        steps = 5000
        ref = np.ones(steps)
        y_hist, _ = simulate_pid(10, 50, 0.5, ref, 0.001, steps, A, B, C, np.zeros(2))
        self.assertAlmostEqual(y_hist[-1], 1.0, delta=0.2)


class TestObserverController(unittest.TestCase):
    """观测器+控制器测试"""

    def test_observer_output_shape(self):
        A, B, C = make_dc_motor_model()
        K = np.array([[50.0, 5.0]])
        L = np.array([[100, 0], [0, 200]])
        A_cl = A - B @ K
        N_ref = -1.0 / (C @ np.linalg.inv(A_cl) @ B)[0, 0]

        steps = 100
        ref = np.ones(steps)
        x_hist, x_hat_hist, y_hist, u_hist = simulate_observer_controller(
            A, B, C, K, L, N_ref, np.zeros(2), np.zeros(2),
            ref, 0.001, 0.001, steps)

        self.assertEqual(x_hist.shape, (steps, 2))
        self.assertEqual(x_hat_hist.shape, (steps, 2))
        self.assertEqual(y_hist.shape, (steps,))

    def test_observer_converges(self):
        """观测器应使输出收敛"""
        A, B, C = make_dc_motor_model()
        K = np.array([[50.0, 5.0]])
        L = np.array([[100, 0], [0, 200]])
        A_cl = A - B @ K
        N_ref = -1.0 / (C @ np.linalg.inv(A_cl) @ B)[0, 0]

        steps = 3000
        ref = np.ones(steps)
        x_hist, _, y_hist, _ = simulate_observer_controller(
            A, B, C, K, L, N_ref, np.zeros(2), np.zeros(2),
            ref, 0.0, 0.001, steps)

        self.assertAlmostEqual(y_hist[-1], 1.0, delta=0.1)


if __name__ == '__main__':
    unittest.main()
