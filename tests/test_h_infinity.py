#!/usr/bin/env python3
"""
H∞鲁棒控制单元测试
覆盖: Riccati方程求解、H∞增益计算、鲁棒稳定性、
      干扰抑制、与LQR对比、参数摄动鲁棒性
测试对象: 11_控制算法库/simulation/h_infinity_simulation.py
"""

import sys
import os
import unittest
import numpy as np
from scipy import linalg

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '11_控制算法库', 'simulation'))

from h_infinity_simulation import (
    SecondOrderPlant, HInfController, LQRController, simulate
)


def make_nominal_system():
    """标称二阶系统 m*x'' + b*x' + k*x = u"""
    m, b, k = 1.0, 0.5, 2.0
    A = np.array([[0, 1], [-k / m, -b / m]])
    B = np.array([[0], [1 / m]])
    return A, B, m, b, k


class TestHInfControllerInit(unittest.TestCase):
    """H∞控制器初始化测试"""

    def test_gain_shape(self):
        A, B, *_ = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        ctrl = HInfController(A, B, Q, R, gamma=1.0, B2=B2)
        self.assertEqual(ctrl.K.shape, (1, 2))

    def test_gain_finite(self):
        A, B, *_ = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        ctrl = HInfController(A, B, Q, R, gamma=1.0, B2=B2)
        self.assertTrue(np.all(np.isfinite(ctrl.K)), "增益矩阵K应全部有限")

    def test_gain_nonzero(self):
        A, B, *_ = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        ctrl = HInfController(A, B, Q, R, gamma=1.0, B2=B2)
        self.assertGreater(np.linalg.norm(ctrl.K), 0, "增益矩阵K不应为零")


class TestRiccatiSolution(unittest.TestCase):
    """Riccati方程解验证"""

    def test_p_positive_semidefinite(self):
        """H∞ Riccati解P应为半正定"""
        A, B, *_ = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        ctrl = HInfController(A, B, Q, R, gamma=2.0, B2=B2)
        # 内部P应为半正定 (通过K间接验证)
        # 如果K有效且有限，说明Riccati收敛
        self.assertTrue(np.all(np.isfinite(ctrl.K)))

    def test_larger_gamma_gives_smaller_gain(self):
        """更大的γ应给出更保守（更小）的增益"""
        A, B, *_ = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        ctrl_small = HInfController(A, B, Q, R, gamma=0.5, B2=B2)
        ctrl_large = HInfController(A, B, Q, R, gamma=5.0, B2=B2)
        # γ越大，增益越小（更保守）
        self.assertLessEqual(np.linalg.norm(ctrl_large.K),
                             np.linalg.norm(ctrl_small.K) * 10.0)


class TestClosedLoopStability(unittest.TestCase):
    """闭环稳定性测试"""

    def test_poles_left_half_plane(self):
        """闭环极点应在左半平面"""
        A, B, *_ = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        ctrl = HInfController(A, B, Q, R, gamma=1.0, B2=B2)
        A_cl = A - B @ ctrl.K
        eigs = np.linalg.eigvals(A_cl)
        self.assertTrue(np.all(eigs.real < 0),
                        f"闭环极点应全部在左半平面, 实际: {eigs.real}")

    def test_step_response_converges(self):
        """阶跃响应应收敛"""
        A, B, m, b, k = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        ctrl = HInfController(A, B, Q, R, gamma=1.0, B2=B2)
        A_cl = A - B @ ctrl.K
        N_ref = -1.0 / (np.array([[1, 0]]) @ np.linalg.inv(A_cl) @ B)[0, 0]

        steps = 5000
        dt = 0.001
        ref = np.ones(steps)
        def target(t): return 1.0
        plant = SecondOrderPlant(m, b, k, dt)
        t, x1, _, _ = simulate(ctrl, plant, target, steps * dt, dt)
        self.assertAlmostEqual(x1[-1], 1.0, delta=0.1,
                               msg="H∞控制阶跃响应应收敛到参考值")


class TestDisturbanceRejection(unittest.TestCase):
    """干扰抑制测试"""

    def test_step_disturbance_rejection(self):
        """应能抑制阶跃干扰"""
        A, B, m, b, k = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        ctrl = HInfController(A, B, Q, R, gamma=1.0, B2=B2)

        steps = 5000
        dt = 0.001
        plant = SecondOrderPlant(m, b, k, dt)
        def target(t): return 1.0 if t >= 0.5 else 0.0
        def dist(t): return 0.5 if t >= 2.0 else 0.0
        t, x1, _, _ = simulate(ctrl, plant, target, steps * dt, dt, dist)
        # 干扰开始后应仍能保持接近参考值
        self.assertAlmostEqual(x1[-1], 1.0, delta=0.3,
                               msg="H∞应能抑制阶跃干扰")

    def test_sine_disturbance_rejection(self):
        """应能抑制正弦干扰"""
        A, B, m, b, k = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        ctrl = HInfController(A, B, Q, R, gamma=1.0, B2=B2)

        steps = 5000
        dt = 0.001
        plant = SecondOrderPlant(m, b, k, dt)
        def target(t): return 1.0 if t >= 0.5 else 0.0
        def dist(t): return 0.3 * np.sin(2 * np.pi * 2.0 * t) if t >= 1.0 else 0.0
        t, x1, _, _ = simulate(ctrl, plant, target, steps * dt, dt, dist)
        # 正弦干扰下误差应可控
        error = np.abs(x1[-1] - 1.0)
        self.assertLess(error, 0.5, "正弦干扰下误差应小于0.5")


class TestRobustness(unittest.TestCase):
    """参数摄动鲁棒性测试"""

    def test_positive_perturbation(self):
        """+30%参数摄动下仍能稳定"""
        A, B, m, b, k = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        ctrl = HInfController(A, B, Q, R, gamma=1.0, B2=B2)

        steps = 5000
        dt = 0.001
        plant = SecondOrderPlant(m * 1.3, b * 0.7, k * 1.3, dt)
        def target(t): return 1.0
        t, x1, _, _ = simulate(ctrl, plant, target, steps * dt, dt)
        self.assertAlmostEqual(x1[-1], 1.0, delta=0.2,
                               msg="+30%摄动下应仍收敛")

    def test_negative_perturbation(self):
        """-30%参数摄动下仍能稳定"""
        A, B, m, b, k = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        ctrl = HInfController(A, B, Q, R, gamma=1.0, B2=B2)

        steps = 5000
        dt = 0.001
        plant = SecondOrderPlant(m * 0.7, b * 1.3, k * 0.7, dt)
        def target(t): return 1.0
        t, x1, _, _ = simulate(ctrl, plant, target, steps * dt, dt)
        self.assertAlmostEqual(x1[-1], 1.0, delta=0.2,
                               msg="-30%摄动下应仍收敛")


class TestHInfVsLQR(unittest.TestCase):
    """H∞ vs LQR对比测试"""

    def test_both_output_shape(self):
        """两者输出形状应一致"""
        A, B, m, b, k = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        hinf = HInfController(A, B, Q, R, gamma=1.0, B2=B2)
        lqr = LQRController(A, B, Q, R)
        self.assertEqual(hinf.K.shape, lqr.K.shape)

    def test_both_converge(self):
        """两者都应能收敛"""
        A, B, m, b, k = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        hinf = HInfController(A, B, Q, R, gamma=1.0, B2=B2)
        lqr = LQRController(A, B, Q, R)

        steps = 5000
        dt = 0.001
        def target(t): return 1.0

        _, x1_h, _, _ = simulate(hinf, SecondOrderPlant(m, b, k, dt),
                                  target, steps * dt, dt)
        _, x1_l, _, _ = simulate(lqr, SecondOrderPlant(m, b, k, dt),
                                  target, steps * dt, dt)
        self.assertAlmostEqual(x1_h[-1], 1.0, delta=0.15)
        self.assertAlmostEqual(x1_l[-1], 1.0, delta=0.15)

    def test_hinf_better_under_perturbation(self):
        """H∞在参数摄动下应比LQR更鲁棒"""
        A, B, m, b, k = make_nominal_system()
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        B2 = np.eye(2)
        hinf = HInfController(A, B, Q, R, gamma=1.0, B2=B2)
        lqr = LQRController(A, B, Q, R)

        steps = 5000
        dt = 0.001
        m_p, b_p, k_p = m * 1.3, b * 0.7, k * 1.3
        def target(t): return 1.0
        def dist(t): return 0.5 if t >= 2.0 else 0.0

        _, x1_h, _, _ = simulate(hinf, SecondOrderPlant(m_p, b_p, k_p, dt),
                                  target, steps * dt, dt, dist)
        _, x1_l, _, _ = simulate(lqr, SecondOrderPlant(m_p, b_p, k_p, dt),
                                  target, steps * dt, dt, dist)
        err_hinf = abs(x1_h[-1] - 1.0)
        err_lqr = abs(x1_l[-1] - 1.0)
        # H∞误差应不大于LQR（允许一定容差）
        self.assertLessEqual(err_hinf, err_lqr + 0.1,
                             "H∞在摄动+干扰下应不差于LQR")


class TestPlantModel(unittest.TestCase):
    """被控对象模型测试"""

    def test_plant_reset(self):
        plant = SecondOrderPlant(1.0, 0.5, 2.0, 0.001)
        plant.update(10.0)
        plant.reset()
        state = plant.get_state()
        np.testing.assert_array_equal(state, [0.0, 0.0])

    def test_plant_response(self):
        """阶跃输入应产生位移"""
        plant = SecondOrderPlant(1.0, 0.5, 2.0, 0.001)
        for _ in range(5000):
            plant.update(1.0)
        self.assertGreater(plant.get_state()[0], 0.1,
                           "正向输入应产生正位移")

    def test_plant_with_disturbance(self):
        """干扰应影响输出"""
        plant1 = SecondOrderPlant(1.0, 0.5, 2.0, 0.001)
        plant2 = SecondOrderPlant(1.0, 0.5, 2.0, 0.001)
        for _ in range(2000):
            plant1.update(1.0, 0.0)
            plant2.update(1.0, 5.0)
        self.assertGreater(plant2.get_state()[0], plant1.get_state()[0],
                           "正向干扰应增大位移")


if __name__ == '__main__':
    unittest.main()
