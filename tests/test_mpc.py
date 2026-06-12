#!/usr/bin/env python3
"""
MPC模型预测控制单元测试
覆盖: 模型初始化、预测时域/控制时域、权重矩阵、约束、
      轨迹跟踪精度、BicycleModel、参考轨迹生成
注意: 直接测试 15_simulation/mpc_simulation.py 中的类
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '15_simulation'))

from mpc_simulation import BicycleModel, MPC, generate_ref_trajectory


class TestBicycleModel(unittest.TestCase):
    """自行车运动模型测试"""

    def test_initial_state(self):
        """初始状态应为零"""
        model = BicycleModel(dt=0.05)
        state = model.state()
        np.testing.assert_array_almost_equal(state, [0, 0, 0, 0])

    def test_step_with_acceleration(self):
        """加速应增加速度"""
        model = BicycleModel(dt=0.05)
        model.v = 1.0
        x, y, theta, v = model.step(a=1.0, delta=0.0)
        self.assertGreater(v, 1.0)

    def test_step_with_deceleration(self):
        """减速应降低速度"""
        model = BicycleModel(dt=0.05)
        model.v = 2.0
        x, y, theta, v = model.step(a=-1.0, delta=0.0)
        self.assertLess(v, 2.0)

    def test_speed_clamped_positive(self):
        """速度不应为负"""
        model = BicycleModel(dt=0.05)
        model.v = 0.1
        x, y, theta, v = model.step(a=-100.0, delta=0.0)
        self.assertGreaterEqual(v, 0.0)

    def test_speed_clamped_max(self):
        """速度不应超过 v_max"""
        model = BicycleModel(dt=0.05)
        model.v = 2.9
        x, y, theta, v = model.step(a=100.0, delta=0.0, v_max=3.0)
        self.assertLessEqual(v, 3.0)

    def test_straight_line_with_zero_steer(self):
        """零转向角应走直线"""
        model = BicycleModel(dt=0.05)
        model.v = 1.0
        model.theta = 0.0
        positions = []
        for _ in range(20):
            x, y, _, _ = model.step(a=0.0, delta=0.0)
            positions.append((x, y))
        # y 方向应基本不变
        for _, y in positions:
            self.assertAlmostEqual(y, 0.0, delta=0.05)

    def test_turn_with_steer(self):
        """非零转向角应产生转弯"""
        model = BicycleModel(dt=0.05)
        model.v = 1.0
        model.theta = 0.0
        for _ in range(20):
            model.step(a=0.0, delta=0.3)
        # 应偏离直线
        self.assertGreater(abs(model.y), 0.01)

    def test_state_array(self):
        """state() 应返回4维数组"""
        model = BicycleModel(dt=0.05)
        s = model.state()
        self.assertEqual(len(s), 4)


class TestMPCInit(unittest.TestCase):
    """MPC控制器初始化测试"""

    def test_default_params(self):
        mpc = MPC(N=20, dt=0.05)
        self.assertEqual(mpc.N, 20)
        self.assertEqual(mpc.dt, 0.05)
        self.assertEqual(mpc.nu, 2)
        self.assertAlmostEqual(mpc.L, 0.3)

    def test_custom_params(self):
        mpc = MPC(N=10, dt=0.01)
        self.assertEqual(mpc.N, 10)
        self.assertEqual(mpc.dt, 0.01)

    def test_weight_matrices_shape(self):
        """Q应为4x4, R应为2x2"""
        mpc = MPC()
        self.assertEqual(mpc.Q.shape, (4, 4))
        self.assertEqual(mpc.R.shape, (2, 2))
        self.assertEqual(mpc.Rd.shape, (2, 2))

    def test_weight_matrices_positive(self):
        """权重矩阵应正定"""
        mpc = MPC()
        self.assertTrue(np.all(np.diag(mpc.Q) > 0))
        self.assertTrue(np.all(np.diag(mpc.R) > 0))


class TestMPCPredict(unittest.TestCase):
    """MPC预测功能测试"""

    def test_predict_returns_correct_length(self):
        """预测应返回 N+1 个状态"""
        mpc = MPC(N=10, dt=0.05)
        x0 = np.array([0, 0, 0, 1.0])
        u_seq = np.zeros(mpc.N * mpc.nu)
        states = mpc.predict(x0, u_seq)
        self.assertEqual(len(states), mpc.N + 1)

    def test_predict_initial_state(self):
        """预测的第一个状态应等于初始状态"""
        mpc = MPC(N=10, dt=0.05)
        x0 = np.array([1.0, 2.0, 0.5, 1.0])
        u_seq = np.zeros(mpc.N * mpc.nu)
        states = mpc.predict(x0, u_seq)
        np.testing.assert_array_almost_equal(states[0], x0)

    def test_predict_zero_control_stays_stationary(self):
        """零控制输入和零初始速度应保持静止"""
        mpc = MPC(N=10, dt=0.05)
        x0 = np.array([0, 0, 0, 0.0])
        u_seq = np.zeros(mpc.N * mpc.nu)
        states = mpc.predict(x0, u_seq)
        for s in states:
            self.assertAlmostEqual(s[0], 0.0, places=5)
            self.assertAlmostEqual(s[1], 0.0, places=5)


class TestMPCCostFunction(unittest.TestCase):
    """MPC代价函数测试"""

    def test_cost_zero_when_matching(self):
        """完美跟踪时代价应接近零"""
        mpc = MPC(N=5, dt=0.05)
        x0 = np.array([1.0, 0.0, 0.0, 1.0])
        # 参考轨迹 = 预测的零控制轨迹
        u_seq = np.zeros(mpc.N * mpc.nu)
        ref = mpc.predict(x0, u_seq)
        cost = mpc.cost_function(u_seq, x0, ref)
        self.assertAlmostEqual(cost, 0.0, places=2)

    def test_cost_positive_when_mismatch(self):
        """跟踪误差应导致正代价"""
        mpc = MPC(N=5, dt=0.05)
        x0 = np.array([0, 0, 0, 1.0])
        u_seq = np.zeros(mpc.N * mpc.nu)
        ref = [np.array([10.0, 10.0, 0, 1.0])] * (mpc.N + 1)
        cost = mpc.cost_function(u_seq, x0, ref)
        self.assertGreater(cost, 0)


class TestMPCSolve(unittest.TestCase):
    """MPC求解器测试"""

    def test_solve_returns_correct_shape(self):
        """solve应返回 N x nu 控制序列"""
        mpc = MPC(N=5, dt=0.05)
        x0 = np.array([0, 0, 0, 1.0])
        ref = [np.array([1.0, 0.0, 0.0, 1.0])] * (mpc.N + 1)
        u_opt = mpc.solve(x0, ref)
        self.assertEqual(u_opt.shape, (mpc.N, mpc.nu))

    def test_solve_respects_constraints(self):
        """求解结果应满足约束"""
        mpc = MPC(N=5, dt=0.05)
        x0 = np.array([0, 0, 0, 1.0])
        ref = [np.array([5.0, 5.0, 0.5, 2.0])] * (mpc.N + 1)
        u_opt = mpc.solve(x0, ref)
        for u in u_opt:
            self.assertGreaterEqual(u[0], mpc.u_min[0] - 0.1)
            self.assertLessEqual(u[0], mpc.u_max[0] + 0.1)
            self.assertGreaterEqual(u[1], mpc.u_min[1] - 0.1)
            self.assertLessEqual(u[1], mpc.u_max[1] + 0.1)


class TestGenerateRefTrajectory(unittest.TestCase):
    """参考轨迹生成测试"""

    def test_circle_trajectory_length(self):
        """圆形轨迹长度应正确"""
        ref = generate_ref_trajectory(t_total=10.0, dt=0.05, pattern='circle')
        expected_len = int(10.0 / 0.05)
        self.assertEqual(len(ref), expected_len)

    def test_circle_trajectory_shape(self):
        """每个参考点应为4维"""
        ref = generate_ref_trajectory(t_total=5.0, dt=0.05, pattern='circle')
        self.assertEqual(ref.shape[1], 4)

    def test_figure8_trajectory(self):
        """8字形轨迹应可生成"""
        ref = generate_ref_trajectory(t_total=5.0, dt=0.05, pattern='figure8')
        self.assertGreater(len(ref), 0)

    def test_circle_is_circular(self):
        """圆形轨迹应大致在圆上"""
        ref = generate_ref_trajectory(t_total=10.0, dt=0.05, pattern='circle')
        R = 2.0
        for pt in ref[:50]:
            radius = np.sqrt(pt[0]**2 + pt[1]**2)
            self.assertAlmostEqual(radius, R, delta=0.5)


class TestMPCIntegration(unittest.TestCase):
    """MPC集成测试"""

    def test_mpc_reduces_cost_vs_zero_control(self):
        """MPC优化后代价应低于零控制"""
        mpc = MPC(N=10, dt=0.05)
        x0 = np.array([0, 0, 0, 1.0])
        ref = [np.array([2.0, 1.0, 0.3, 1.0])] * (mpc.N + 1)
        u_zero = np.zeros(mpc.N * mpc.nu)
        cost_zero = mpc.cost_function(u_zero, x0, ref)

        u_opt = mpc.solve(x0, ref)
        cost_opt = mpc.cost_function(u_opt.flatten(), x0, ref)

        self.assertLessEqual(cost_opt, cost_zero + 0.1)


if __name__ == '__main__':
    unittest.main()
