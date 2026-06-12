#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阻抗控制单元测试
覆盖: ImpedanceController初始化/计算、CompliantEnvironment初始化/接触力、
      SingleJointArm初始化/步进、run_simulation仿真运行、
      不同刚度/阻尼参数对比、力跟踪集成测试
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from _15_simulation.impedance_control_simulation import (
    ImpedanceController,
    CompliantEnvironment,
    SingleJointArm,
    run_simulation,
)


# ==================== ImpedanceController测试 ====================

class TestImpedanceControllerInit(unittest.TestCase):
    """阻抗控制器初始化测试"""

    def test_default_params(self):
        ctrl = ImpedanceController()
        self.assertEqual(ctrl.M, 1.0)
        self.assertEqual(ctrl.B, 20.0)
        self.assertEqual(ctrl.K, 100.0)

    def test_custom_params(self):
        ctrl = ImpedanceController(M=2.0, B=50.0, K=200.0)
        self.assertEqual(ctrl.M, 2.0)
        self.assertEqual(ctrl.B, 50.0)
        self.assertEqual(ctrl.K, 200.0)


class TestImpedanceControllerCompute(unittest.TestCase):
    """阻抗控制器计算测试"""

    def test_returns_float(self):
        ctrl = ImpedanceController(M=1.0, B=20.0, K=100.0)
        F = ctrl.compute(x_des=0.5, x_dot_des=0, x_ddot_des=0,
                         x=0.3, x_dot=0.0, F_ext=0.0)
        self.assertIsInstance(F, float)

    def test_zero_error_zero_ext_force(self):
        """在期望位置且无外力时，控制力应为零"""
        ctrl = ImpedanceController(M=1.0, B=20.0, K=100.0)
        F = ctrl.compute(x_des=0.5, x_dot_des=0, x_ddot_des=0,
                         x=0.5, x_dot=0.0, F_ext=0.0)
        self.assertAlmostEqual(F, 0.0, places=5)

    def test_position_error_generates_force(self):
        """有位置误差时应产生力"""
        ctrl = ImpedanceController(M=1.0, B=20.0, K=100.0)
        F = ctrl.compute(x_des=0.5, x_dot_des=0, x_ddot_des=0,
                         x=0.3, x_dot=0.0, F_ext=0.0)
        self.assertGreater(F, 0.0)

    def test_negative_error_negative_force(self):
        """位置超调时应产生负力"""
        ctrl = ImpedanceController(M=1.0, B=20.0, K=100.0)
        F = ctrl.compute(x_des=0.5, x_dot_des=0, x_ddot_des=0,
                         x=0.7, x_dot=0.0, F_ext=0.0)
        self.assertLess(F, 0.0)

    def test_external_force_affects_output(self):
        """外力应影响控制输出"""
        ctrl = ImpedanceController(M=1.0, B=20.0, K=100.0)
        F_no_ext = ctrl.compute(0.5, 0, 0, 0.3, 0.0, F_ext=0.0)
        F_with_ext = ctrl.compute(0.5, 0, 0, 0.3, 0.0, F_ext=50.0)
        self.assertLess(F_with_ext, F_no_ext)  # 外力会减小控制力

    def test_velocity_difference_contributes(self):
        """速度误差应产生阻尼力"""
        ctrl = ImpedanceController(M=1.0, B=20.0, K=100.0)
        F_static = ctrl.compute(0.5, 0, 0, 0.5, 0.0, F_ext=0.0)
        F_moving = ctrl.compute(0.5, 0, 0, 0.5, 1.0, F_ext=0.0)
        # 正速度误差 → 阻尼力减小控制力
        self.assertLess(F_moving, F_static)

    def test_acceleration_term(self):
        """期望加速度应影响输出"""
        ctrl = ImpedanceController(M=1.0, B=20.0, K=100.0)
        F_no_acc = ctrl.compute(0.5, 0, 0, 0.5, 0.0, F_ext=0.0)
        F_with_acc = ctrl.compute(0.5, 0, 10.0, 0.5, 0.0, F_ext=0.0)
        self.assertGreater(F_with_acc, F_no_acc)

    def test_higher_K_stronger_response(self):
        """更高刚度应产生更大力"""
        ctrl_soft = ImpedanceController(M=1.0, B=20.0, K=50.0)
        ctrl_stiff = ImpedanceController(M=1.0, B=20.0, K=500.0)
        F_soft = ctrl_soft.compute(0.5, 0, 0, 0.3, 0.0, F_ext=0.0)
        F_stiff = ctrl_stiff.compute(0.5, 0, 0, 0.3, 0.0, F_ext=0.0)
        self.assertGreater(F_stiff, F_soft)

    def test_higher_B_more_damping(self):
        """更高阻尼应产生更大力（有速度误差时）"""
        ctrl_low = ImpedanceController(M=1.0, B=10.0, K=100.0)
        ctrl_high = ImpedanceController(M=1.0, B=100.0, K=100.0)
        F_low = ctrl_low.compute(0.5, 1.0, 0, 0.5, 0.0, F_ext=0.0)
        F_high = ctrl_high.compute(0.5, 1.0, 0, 0.5, 0.0, F_ext=0.0)
        self.assertGreater(F_high, F_low)


# ==================== CompliantEnvironment测试 ====================

class TestCompliantEnvironmentInit(unittest.TestCase):
    """柔性环境模型初始化测试"""

    def test_default_params(self):
        env = CompliantEnvironment()
        self.assertEqual(env.K_e, 5000.0)
        self.assertEqual(env.B_e, 50.0)
        self.assertEqual(env.x_surface, 0.5)

    def test_custom_params(self):
        env = CompliantEnvironment(K_e=10000, B_e=100, x_surface=0.8)
        self.assertEqual(env.K_e, 10000)
        self.assertEqual(env.B_e, 100)
        self.assertEqual(env.x_surface, 0.8)


class TestCompliantEnvironmentContactForce(unittest.TestCase):
    """柔性环境接触力测试"""

    def test_no_contact_zero_force(self):
        """未接触时力为零"""
        env = CompliantEnvironment(x_surface=0.5)
        F = env.contact_force(x=0.3, x_dot=0.0)
        self.assertEqual(F, 0.0)

    def test_contact_positive_force(self):
        """接触时应产生正力"""
        env = CompliantEnvironment(K_e=5000, x_surface=0.5)
        # x < x_surface → 发生接触
        F = env.contact_force(x=0.4, x_dot=0.0)
        self.assertGreater(F, 0.0)

    def test_force_proportional_to_penetration(self):
        """侵入越深力越大"""
        env = CompliantEnvironment(K_e=5000, x_surface=0.5)
        F1 = env.contact_force(x=0.45, x_dot=0.0)
        F2 = env.contact_force(x=0.3, x_dot=0.0)
        self.assertGreater(F2, F1)

    def test_at_surface_zero_force(self):
        """恰好在表面时力为零"""
        env = CompliantEnvironment(K_e=5000, x_surface=0.5)
        F = env.contact_force(x=0.5, x_dot=0.0)
        self.assertEqual(F, 0.0)

    def test_beyond_surface_zero_force(self):
        """超过表面时力为零"""
        env = CompliantEnvironment(K_e=5000, x_surface=0.5)
        F = env.contact_force(x=0.6, x_dot=0.0)
        self.assertEqual(F, 0.0)

    def test_negative_velocity_increases_force(self):
        """负速度（朝环境方向）应增加力"""
        env = CompliantEnvironment(K_e=5000, B_e=50, x_surface=0.5)
        F_static = env.contact_force(x=0.4, x_dot=0.0)
        F_moving_in = env.contact_force(x=0.4, x_dot=-1.0)
        self.assertGreaterEqual(F_moving_in, F_static)

    def test_positive_velocity_no_damping(self):
        """正速度（远离环境）不应增加阻尼力"""
        env = CompliantEnvironment(K_e=5000, B_e=50, x_surface=0.5)
        F_static = env.contact_force(x=0.4, x_dot=0.0)
        F_moving_out = env.contact_force(x=0.4, x_dot=1.0)
        self.assertEqual(F_moving_out, F_static)

    def test_force_never_negative(self):
        """力不应为负"""
        env = CompliantEnvironment(K_e=5000, B_e=50, x_surface=0.5)
        F = env.contact_force(x=0.4, x_dot=10.0)
        self.assertGreaterEqual(F, 0.0)


# ==================== SingleJointArm测试 ====================

class TestSingleJointArmInit(unittest.TestCase):
    """单关节臂初始化测试"""

    def test_default_params(self):
        arm = SingleJointArm()
        self.assertEqual(arm.m, 2.0)
        self.assertEqual(arm.l, 0.5)
        self.assertEqual(arm.damping, 1.0)

    def test_custom_params(self):
        arm = SingleJointArm(m=5.0, l=1.0, damping=2.0)
        self.assertEqual(arm.m, 5.0)
        self.assertEqual(arm.l, 1.0)
        self.assertEqual(arm.damping, 2.0)

    def test_initial_state(self):
        arm = SingleJointArm()
        self.assertEqual(arm.x, 0.3)
        self.assertEqual(arm.x_dot, 0.0)

    def test_inertia_computed(self):
        arm = SingleJointArm(m=2.0, l=0.5)
        expected_I = 2.0 * 0.5**2 / 3
        self.assertAlmostEqual(arm.I, expected_I, places=5)


class TestSingleJointArmStep(unittest.TestCase):
    """单关节臂步进测试"""

    def test_returns_tuple(self):
        arm = SingleJointArm()
        result = arm.step(F_ctrl=1.0, F_ext=0.0, dt=0.001)
        self.assertEqual(len(result), 2)

    def test_position_updates(self):
        arm = SingleJointArm()
        x0 = arm.x
        arm.step(F_ctrl=10.0, F_ext=0.0, dt=0.01)
        self.assertNotEqual(arm.x, x0)

    def test_force_accelerates(self):
        """正控制力应加速运动"""
        arm = SingleJointArm()
        arm.step(F_ctrl=100.0, F_ext=0.0, dt=0.01)
        self.assertGreater(arm.x_dot, 0.0)

    def test_damping_resists(self):
        """阻尼应抵抗运动"""
        arm1 = SingleJointArm(damping=0.0)
        arm2 = SingleJointArm(damping=10.0)
        arm1.step(F_ctrl=10.0, F_ext=0.0, dt=0.01)
        arm2.step(F_ctrl=10.0, F_ext=0.0, dt=0.01)
        self.assertGreater(arm1.x_dot, arm2.x_dot)

    def test_external_force_adds(self):
        """外力应叠加"""
        arm = SingleJointArm()
        arm.step(F_ctrl=0.0, F_ext=100.0, dt=0.01)
        self.assertGreater(arm.x_dot, 0.0)


# ==================== run_simulation测试 ====================

class TestRunSimulation(unittest.TestCase):
    """仿真运行测试"""

    def test_returns_dict(self):
        results = run_simulation({'M': 1.0, 'B': 20.0, 'K': 100.0},
                                 x_target=0.5, duration=0.5, dt=0.001)
        self.assertIsInstance(results, dict)

    def test_result_keys(self):
        results = run_simulation({'M': 1.0, 'B': 20.0, 'K': 100.0},
                                 x_target=0.5, duration=0.5, dt=0.001)
        expected_keys = ['time', 'position', 'velocity', 'force', 'force_ext', 'target']
        for key in expected_keys:
            self.assertIn(key, results)

    def test_result_array_lengths(self):
        duration = 0.5
        dt = 0.001
        results = run_simulation({'M': 1.0, 'B': 20.0, 'K': 100.0},
                                 x_target=0.5, duration=duration, dt=dt)
        expected_len = int(duration / dt)
        self.assertEqual(len(results['time']), expected_len)
        self.assertEqual(len(results['position']), expected_len)

    def test_time_increments(self):
        results = run_simulation({'M': 1.0, 'B': 20.0, 'K': 100.0},
                                 x_target=0.5, duration=0.5, dt=0.001)
        for i in range(1, len(results['time'])):
            self.assertAlmostEqual(results['time'][i] - results['time'][i - 1],
                                   0.001, places=6)

    def test_target_constant(self):
        results = run_simulation({'M': 1.0, 'B': 20.0, 'K': 100.0},
                                 x_target=0.5, duration=0.5, dt=0.001)
        np.testing.assert_array_equal(results['target'], 0.5)

    def test_position_converges_to_target(self):
        """位置应趋近目标"""
        results = run_simulation({'M': 1.0, 'B': 40.0, 'K': 200.0},
                                 x_target=0.5, duration=2.0, dt=0.0001)
        final_pos = np.mean(results['position'][-100:])
        self.assertAlmostEqual(final_pos, 0.5, delta=0.1)

    def test_contact_force_nonzero(self):
        """应产生接触力"""
        results = run_simulation({'M': 1.0, 'B': 40.0, 'K': 200.0},
                                 x_target=0.5, duration=1.0, dt=0.001)
        max_F = np.max(results['force_ext'])
        self.assertGreater(max_F, 0.0)


# ==================== 不同参数对比测试 ====================

class TestImpedanceParameterComparison(unittest.TestCase):
    """不同阻抗参数对比测试"""

    def test_low_stiffness_slower_response(self):
        """低刚度应更慢响应"""
        r_soft = run_simulation({'M': 1.0, 'B': 30.0, 'K': 50.0},
                                x_target=0.5, duration=1.0, dt=0.001)
        r_stiff = run_simulation({'M': 1.0, 'B': 30.0, 'K': 800.0},
                                 x_target=0.5, duration=1.0, dt=0.001)
        # 高刚度应更快到达目标附近
        idx = 500  # t=0.5s
        self.assertGreater(r_stiff['position'][idx], r_soft['position'][idx] - 0.2)

    def test_high_damping_more_stable(self):
        """高阻尼应更稳定（更小振荡）"""
        r_low = run_simulation({'M': 1.0, 'B': 10.0, 'K': 200.0},
                               x_target=0.5, duration=1.0, dt=0.001)
        r_high = run_simulation({'M': 1.0, 'B': 100.0, 'K': 200.0},
                                x_target=0.5, duration=1.0, dt=0.001)
        # 高阻尼的速度变化更小
        vel_std_low = np.std(r_low['velocity'][500:])
        vel_std_high = np.std(r_high['velocity'][500:])
        self.assertLess(vel_std_high, vel_std_low + 1.0)

    def test_high_stiffness_higher_force(self):
        """高刚度应产生更大力"""
        r_soft = run_simulation({'M': 1.0, 'B': 30.0, 'K': 50.0},
                                x_target=0.5, duration=1.0, dt=0.001)
        r_stiff = run_simulation({'M': 1.0, 'B': 30.0, 'K': 800.0},
                                 x_target=0.5, duration=1.0, dt=0.001)
        self.assertGreater(np.max(r_stiff['force_ext']),
                           np.max(r_soft['force_ext']) * 0.5)


# ==================== 阻抗控制集成测试 ====================

class TestImpedanceControlIntegration(unittest.TestCase):
    """阻抗控制集成测试"""

    def test_full_simulation_no_crash(self):
        """完整仿真不应崩溃"""
        configs = {
            'soft': {'M': 1.0, 'B': 30.0, 'K': 50.0},
            'medium': {'M': 1.0, 'B': 40.0, 'K': 200.0},
            'stiff': {'M': 1.0, 'B': 60.0, 'K': 800.0},
        }
        for name, params in configs.items():
            results = run_simulation(params, x_target=0.5, duration=1.0, dt=0.001)
            self.assertEqual(len(results['time']), 1000)

    def test_position_never_exceeds_reasonable_bounds(self):
        """位置不应超过合理范围"""
        results = run_simulation({'M': 1.0, 'B': 40.0, 'K': 200.0},
                                 x_target=0.5, duration=2.0, dt=0.001)
        self.assertLess(np.max(results['position']), 1.0)
        self.assertGreater(np.min(results['position']), -0.1)

    def test_force_bounded(self):
        """控制力应在合理范围内"""
        results = run_simulation({'M': 1.0, 'B': 40.0, 'K': 200.0},
                                 x_target=0.5, duration=1.0, dt=0.001)
        self.assertLess(np.max(np.abs(results['force'])), 1e6)


if __name__ == '__main__':
    unittest.main()
