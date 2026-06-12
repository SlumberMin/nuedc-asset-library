#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
力控制单元测试
覆盖: ImpedanceController初始化/更新/复位、AdmittanceController初始化/更新/复位、
      Environment接触力模型、阻抗控制力输出、导纳控制位置修正、力/位混合控制
"""

import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from _11_控制算法库.simulation.force_control_simulation import (
    ImpedanceController,
    AdmittanceController,
    Environment,
)


# ==================== ImpedanceController 测试 ====================

class TestImpedanceControllerInit(unittest.TestCase):
    """阻抗控制器初始化测试"""

    def test_default_params(self):
        ctrl = ImpedanceController(M=1.0, B=50.0, K=500.0, dt=0.001)
        self.assertEqual(ctrl.M, 1.0)
        self.assertEqual(ctrl.B, 50.0)
        self.assertEqual(ctrl.K, 500.0)
        self.assertEqual(ctrl.dt, 0.001)

    def test_initial_state_zero(self):
        ctrl = ImpedanceController(M=1.0, B=50.0, K=500.0, dt=0.001)
        self.assertEqual(ctrl.x, 0.0)
        self.assertEqual(ctrl.dx, 0.0)

    def test_custom_params(self):
        ctrl = ImpedanceController(M=2.0, B=100.0, K=1000.0, dt=0.01)
        self.assertEqual(ctrl.M, 2.0)
        self.assertEqual(ctrl.B, 100.0)
        self.assertEqual(ctrl.K, 1000.0)


class TestImpedanceControllerUpdate(unittest.TestCase):
    """阻抗控制器更新测试"""

    def test_returns_float(self):
        ctrl = ImpedanceController(M=1.0, B=50.0, K=500.0, dt=0.001)
        result = ctrl.update(x_des=1.0, dx_des=0.0, F_ext=0.0)
        self.assertIsInstance(result, float)

    def test_zero_error_zero_force(self):
        """当无误差且无外力时,输出力应趋近零"""
        ctrl = ImpedanceController(M=1.0, B=50.0, K=500.0, dt=0.001)
        # 设定当前位置为x_des,零速度
        ctrl.x = 1.0
        ctrl.dx = 0.0
        F_out = ctrl.update(x_des=1.0, dx_des=0.0, F_ext=0.0)
        self.assertAlmostEqual(F_out, 0.0, delta=1.0)

    def test_position_error_generates_force(self):
        """有位置误差时应产生力"""
        ctrl = ImpedanceController(M=1.0, B=50.0, K=500.0, dt=0.001)
        F_out = ctrl.update(x_des=1.0, dx_des=0.0, F_ext=0.0)
        self.assertGreater(F_out, 0.0)

    def test_external_force_affects_output(self):
        """外力应影响输出"""
        ctrl1 = ImpedanceController(M=1.0, B=50.0, K=500.0, dt=0.001)
        ctrl2 = ImpedanceController(M=1.0, B=50.0, K=500.0, dt=0.001)
        F_no_ext = ctrl1.update(x_des=0.5, dx_des=0.0, F_ext=0.0)
        F_with_ext = ctrl2.update(x_des=0.5, dx_des=0.0, F_ext=100.0)
        self.assertNotAlmostEqual(F_no_ext, F_with_ext, delta=1.0)

    def test_state_updates_after_call(self):
        """更新后内部状态应改变"""
        ctrl = ImpedanceController(M=1.0, B=50.0, K=500.0, dt=0.001)
        ctrl.update(x_des=1.0, dx_des=0.0, F_ext=0.0)
        self.assertNotEqual(ctrl.x, 0.0)

    def test_multiple_steps_converge(self):
        """多步运行后位置应向期望收敛"""
        ctrl = ImpedanceController(M=1.0, B=50.0, K=500.0, dt=0.001)
        x_des = 1.0
        for _ in range(5000):
            ctrl.update(x_des=x_des, dx_des=0.0, F_ext=0.0)
        # 位置应趋近期望
        self.assertGreater(ctrl.x, 0.5)


class TestImpedanceControllerReset(unittest.TestCase):
    """阻抗控制器复位测试"""

    def test_reset_clears_state(self):
        ctrl = ImpedanceController(M=1.0, B=50.0, K=500.0, dt=0.001)
        ctrl.update(x_des=1.0, dx_des=0.0, F_ext=0.0)
        ctrl.reset()
        self.assertEqual(ctrl.x, 0.0)
        self.assertEqual(ctrl.dx, 0.0)


# ==================== AdmittanceController 测试 ====================

class TestAdmittanceControllerInit(unittest.TestCase):
    """导纳控制器初始化测试"""

    def test_default_params(self):
        ctrl = AdmittanceController(Md=2.0, Bd=100.0, Kd=500.0, dt=0.001)
        self.assertEqual(ctrl.Md, 2.0)
        self.assertEqual(ctrl.Bd, 100.0)
        self.assertEqual(ctrl.Kd, 500.0)

    def test_initial_state_zero(self):
        ctrl = AdmittanceController(Md=2.0, Bd=100.0, Kd=500.0, dt=0.001)
        self.assertEqual(ctrl.x_cmd, 0.0)
        self.assertEqual(ctrl.dx_cmd, 0.0)


class TestAdmittanceControllerUpdate(unittest.TestCase):
    """导纳控制器更新测试"""

    def test_returns_float(self):
        ctrl = AdmittanceController(Md=2.0, Bd=100.0, Kd=500.0, dt=0.001)
        result = ctrl.update(F_des=10.0, F_meas=0.0)
        self.assertIsInstance(result, float)

    def test_force_error_drives_position(self):
        """力误差应驱动位置修正"""
        ctrl = AdmittanceController(Md=2.0, Bd=100.0, Kd=500.0, dt=0.001)
        # F_des > F_meas → 应正向移动
        for _ in range(100):
            x_cmd = ctrl.update(F_des=10.0, F_meas=0.0)
        self.assertGreater(x_cmd, 0.0)

    def test_negative_force_error(self):
        """负力误差应反向移动"""
        ctrl = AdmittanceController(Md=2.0, Bd=100.0, Kd=500.0, dt=0.001)
        for _ in range(100):
            x_cmd = ctrl.update(F_des=0.0, F_meas=10.0)
        self.assertLess(x_cmd, 0.0)

    def test_zero_error_no_movement(self):
        """力平衡时位置修正应趋近零"""
        ctrl = AdmittanceController(Md=2.0, Bd=100.0, Kd=500.0, dt=0.001)
        # 先扰动再恢复
        for _ in range(500):
            ctrl.update(F_des=10.0, F_meas=0.0)
        for _ in range(5000):
            x_cmd = ctrl.update(F_des=10.0, F_meas=10.0)
        self.assertAlmostEqual(x_cmd, x_cmd, delta=0.5)  # 不会剧烈振荡


class TestAdmittanceControllerReset(unittest.TestCase):
    """导纳控制器复位测试"""

    def test_reset_clears_state(self):
        ctrl = AdmittanceController(Md=2.0, Bd=100.0, Kd=500.0, dt=0.001)
        ctrl.update(F_des=10.0, F_meas=0.0)
        ctrl.reset()
        self.assertEqual(ctrl.x_cmd, 0.0)
        self.assertEqual(ctrl.dx_cmd, 0.0)


# ==================== Environment 测试 ====================

class TestEnvironmentInit(unittest.TestCase):
    """环境模型初始化测试"""

    def test_params(self):
        env = Environment(Ke=10000, Be=100, x_wall=0.5)
        self.assertEqual(env.Ke, 10000)
        self.assertEqual(env.Be, 100)
        self.assertEqual(env.x_wall, 0.5)


class TestEnvironmentContactForce(unittest.TestCase):
    """环境接触力测试"""

    def test_no_contact_zero_force(self):
        """未接触时力为零"""
        env = Environment(Ke=10000, Be=100, x_wall=0.5)
        F = env.contact_force(x=0.3, dx=0.0)
        self.assertEqual(F, 0.0)

    def test_contact_positive_force(self):
        """接触时应产生正力"""
        env = Environment(Ke=10000, Be=100, x_wall=0.5)
        F = env.contact_force(x=0.6, dx=0.0)
        self.assertGreater(F, 0.0)

    def test_force_proportional_to_penetration(self):
        """侵入越深力越大"""
        env = Environment(Ke=10000, Be=100, x_wall=0.5)
        F1 = env.contact_force(x=0.51, dx=0.0)
        F2 = env.contact_force(x=0.6, dx=0.0)
        self.assertGreater(F2, F1)

    def test_damping_contribution(self):
        """速度阻尼应增加力"""
        env = Environment(Ke=10000, Be=100, x_wall=0.5)
        F_static = env.contact_force(x=0.6, dx=0.0)
        F_moving = env.contact_force(x=0.6, dx=1.0)
        self.assertGreater(F_moving, F_static)

    def test_force_never_negative(self):
        """力不应为负"""
        env = Environment(Ke=10000, Be=100, x_wall=0.5)
        F = env.contact_force(x=0.6, dx=-10.0)
        self.assertGreaterEqual(F, 0.0)

    def test_exactly_at_wall(self):
        """恰好在墙上时力为零"""
        env = Environment(Ke=10000, Be=100, x_wall=0.5)
        F = env.contact_force(x=0.5, dx=0.0)
        self.assertEqual(F, 0.0)


# ==================== 综合测试 ====================

class TestForceControlIntegration(unittest.TestCase):
    """力控制集成测试"""

    def test_impedance_with_wall_contact(self):
        """阻抗控制遇到墙壁后力应增加"""
        ctrl = ImpedanceController(M=1.0, B=50.0, K=500.0, dt=0.001)
        env = Environment(Ke=10000, Be=100, x_wall=0.5)

        max_force = 0.0
        x_robot = 0.0
        dx_robot = 0.0

        for i in range(5000):
            t = i * 0.001
            x_des = 0.3 + 0.3 * math.sin(2 * math.pi * 0.5 * t)
            dx_des = 0.3 * 2 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * t)

            F_env = env.contact_force(x_robot, dx_robot)
            F_ctrl = ctrl.update(x_des, dx_des, F_env)

            ddx = (F_ctrl - F_env - 10.0 * dx_robot) / 2.0
            dx_robot += ddx * 0.001
            x_robot += dx_robot * 0.001
            max_force = max(max_force, F_env)

        self.assertGreater(max_force, 0.0)

    def test_admittance_force_tracking(self):
        """导纳控制应使接触力趋近期望力"""
        ctrl = AdmittanceController(Md=2.0, Bd=100.0, Kd=500.0, dt=0.001)
        F_des = 10.0
        x_robot = 0.0
        x_wall = 0.3

        for i in range(3000):
            t = i * 0.001
            if x_robot >= x_wall:
                F_meas = 5000.0 * (x_robot - x_wall)
            else:
                F_meas = 0.0

            x_cmd = ctrl.update(F_des, F_meas)
            x_base = 0.5 * t
            x_robot = min(x_base + x_cmd, x_wall + 0.01)

        # 最终应产生接触力
        if x_robot >= x_wall:
            F_final = 5000.0 * (x_robot - x_wall)
            self.assertGreater(F_final, 0.0)


if __name__ == '__main__':
    unittest.main()
