#!/usr/bin/env python3
"""
观测器+控制器单元测试
覆盖: 观测器初始化、模型设置、增益设置、
      观测器更新、状态估计收敛、分离原理验证
测试对象: 11_控制算法库/common/observer_controller.c (Python仿真验证)
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class ObserverControllerSim:
    """观测器+控制器Python仿真 (对应observer_controller.c)"""

    def __init__(self, n=2, m=1, p=1, dt=0.001):
        self.n = n  # 状态维度
        self.m = m  # 输入维度
        self.p = p  # 输出维度
        self.dt = dt
        self.A = np.zeros((n, n))
        self.B = np.zeros((n, m))
        self.C = np.zeros((p, n))
        self.L = np.zeros((n, p))
        self.K = np.zeros((m, n))
        self.N = np.eye(m)
        self.x_hat = np.zeros(n)

    def set_model(self, A, B, C):
        self.A = np.array(A, dtype=float).reshape(self.n, self.n)
        self.B = np.array(B, dtype=float).reshape(self.n, self.m)
        self.C = np.array(C, dtype=float).reshape(self.p, self.n)

    def set_observer_gain(self, L):
        self.L = np.array(L, dtype=float).reshape(self.n, self.p)

    def set_controller_gain(self, K):
        self.K = np.array(K, dtype=float).reshape(self.m, self.n)

    def set_feedforward_gain(self, N):
        self.N = np.array(N, dtype=float).reshape(self.m, self.m)

    def set_initial_state(self, x0):
        self.x_hat = np.array(x0, dtype=float)

    def update(self, ref, y_meas):
        ref = np.array(ref, dtype=float).reshape(self.m)
        y_meas = np.array(y_meas, dtype=float).reshape(self.p)

        # 控制律: u = -K*x_hat + N*r
        u = -self.K @ self.x_hat + self.N @ ref

        # 观测器更新: x_hat' = x_hat + dt*(A*x_hat + B*u + L*(y - C*x_hat))
        e_y = y_meas - self.C @ self.x_hat
        self.x_hat = self.x_hat + self.dt * (
            self.A @ self.x_hat + self.B @ u + self.L @ e_y)

        return u

    def get_estimated_state(self, index):
        return self.x_hat[index]

    def reset(self):
        self.x_hat = np.zeros(self.n)


class TestOCInit(unittest.TestCase):
    def test_default_init(self):
        oc = ObserverControllerSim()
        self.assertEqual(oc.n, 2)
        self.assertEqual(oc.m, 1)
        self.assertEqual(oc.p, 1)

    def test_custom_dimensions(self):
        oc = ObserverControllerSim(n=3, m=2, p=2)
        self.assertEqual(oc.n, 3)
        self.assertEqual(oc.m, 2)

    def test_initial_state_zero(self):
        oc = ObserverControllerSim()
        self.assertAlmostEqual(oc.get_estimated_state(0), 0.0)
        self.assertAlmostEqual(oc.get_estimated_state(1), 0.0)


class TestOCSetModel(unittest.TestCase):
    def test_set_model(self):
        oc = ObserverControllerSim()
        A = [[0, 1], [0, -10]]
        B = [[0], [100]]
        C = [[1, 0]]
        oc.set_model(A, B, C)
        self.assertAlmostEqual(oc.A[1, 1], -10.0)
        self.assertAlmostEqual(oc.B[1, 0], 100.0)

    def test_set_gains(self):
        oc = ObserverControllerSim()
        oc.set_observer_gain([[50, 0], [0, 200]])
        oc.set_controller_gain([[10, 2]])
        self.assertAlmostEqual(oc.L[0, 0], 50)
        self.assertAlmostEqual(oc.K[0, 0], 10)


class TestOCUpdate(unittest.TestCase):
    def test_update_returns_control(self):
        oc = ObserverControllerSim()
        oc.set_model([[0, 1], [0, -10]], [[0], [100]], [[1, 0]])
        oc.set_observer_gain([[50, 0], [0, 200]])
        oc.set_controller_gain([[10, 2]])
        u = oc.update([1.0], [0.0])
        self.assertEqual(len(u), 1)

    def test_state_estimate_updates(self):
        oc = ObserverControllerSim()
        oc.set_model([[0, 1], [0, -10]], [[0], [100]], [[1, 0]])
        oc.set_observer_gain([[100, 0], [0, 500]])
        oc.set_controller_gain([[10, 2]])
        oc.set_initial_state([0.0, 0.0])

        # 运行几步
        for _ in range(100):
            oc.update([1.0], [0.5])

        # 状态估计应不为零
        self.assertGreater(abs(oc.get_estimated_state(0)), 0.0)


class TestOCConvergence(unittest.TestCase):
    def test_observer_tracks_true_state(self):
        """观测器状态估计应收敛到真实状态"""
        A = np.array([[0, 1], [0, -10]], dtype=float)
        B = np.array([[0], [100]], dtype=float)
        C = np.array([[1, 0]], dtype=float)
        K = np.array([[50, 5]], dtype=float)
        L = np.array([[200, 0], [0, 500]], dtype=float)

        oc = ObserverControllerSim()
        oc.set_model(A, B, C)
        oc.set_observer_gain(L)
        oc.set_controller_gain(K)
        oc.set_feedforward_gain([[1.0]])
        oc.set_initial_state([0.0, 0.0])

        dt = 0.001
        oc.dt = dt
        # 真实系统状态
        x_true = np.array([0.0, 0.0])
        steps = 3000

        for _ in range(steps):
            y = (C @ x_true)[0]
            u = oc.update([1.0], [y])
            x_true = x_true + dt * (A @ x_true + B.flatten() * u[0])

        # 估计应接近真实
        err = abs(oc.get_estimated_state(0) - x_true[0])
        self.assertLess(err, 0.1, "观测器估计应接近真实状态")


class TestOCReset(unittest.TestCase):
    def test_reset_clears_state(self):
        oc = ObserverControllerSim()
        oc.x_hat = np.array([999.0, 888.0])
        oc.reset()
        self.assertAlmostEqual(oc.get_estimated_state(0), 0.0)
        self.assertAlmostEqual(oc.get_estimated_state(1), 0.0)


class TestOCSeparationPrinciple(unittest.TestCase):
    def test_separate_design_valid(self):
        """分离原理: 观测器和控制器可独立设计"""
        A = np.array([[0, 1], [0, -10]], dtype=float)
        B = np.array([[0], [100]], dtype=float)
        C = np.array([[1, 0]], dtype=float)

        # 独立设计控制器
        K = np.array([[50, 5]])
        A_cl = A - B @ K
        eigs_cl = np.linalg.eigvals(A_cl)
        self.assertTrue(np.all(eigs_cl.real < 0), "闭环应稳定")

        # 独立设计观测器
        L = np.array([[200], [500]])
        A_obs = A - L @ C
        eigs_obs = np.linalg.eigvals(A_obs)
        self.assertTrue(np.all(eigs_obs.real < 0), "观测器应稳定")

        # 组合后仍应稳定 (分离原理)
        # A_aug = [[A-BK, BK], [0, A-LC]]
        n = A.shape[0]
        A_aug = np.block([
            [A - B @ K, B @ K],
            [np.zeros((n, n)), A - L @ C]
        ])
        eigs_aug = np.linalg.eigvals(A_aug)
        self.assertTrue(np.all(eigs_aug.real < 0), "组合系统应稳定")


if __name__ == '__main__':
    unittest.main()
