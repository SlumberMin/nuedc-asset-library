#!/usr/bin/env python3
"""
扰动观测器单元测试
覆盖: 一阶DOB初始化/更新/复位、二阶DOB初始化/更新/复位、
      速度扰动观测器、互补滤波器、扰动估计收敛性
注意: 使用纯 Python 模拟 C DOB 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class DOB_FirstOrder_Simulator:
    """一阶扰动观测器模拟"""

    def __init__(self, Kn=1.0, an=1.0, q_tau=0.1, dt=0.01):
        self.Kn = Kn
        self.an = an
        self.q_tau = q_tau
        self.dt = dt
        self.q_alpha = dt / (q_tau + dt)
        self.q_state = 0.0
        self.disturbance_hat = 0.0
        self.prev_u = 0.0
        self.prev_y = 0.0
        self.initialized = True

    def update(self, u, y):
        if not self.initialized:
            return 0.0
        # 标称模型预测
        y_model = ((1.0 - self.an * self.dt) * self.prev_y
                   + self.Kn * self.dt * self.prev_u)
        # 模型误差
        e = y_model - y
        # Q滤波器
        self.q_state = (1.0 - self.q_alpha) * self.q_state + self.q_alpha * e
        self.disturbance_hat = self.q_state
        self.prev_u = u
        self.prev_y = y
        return self.disturbance_hat

    def get_disturbance(self):
        return self.disturbance_hat

    def reset(self):
        self.q_state = 0.0
        self.disturbance_hat = 0.0
        self.prev_u = 0.0
        self.prev_y = 0.0


class DOB_SecondOrder_Simulator:
    """二阶扰动观测器模拟"""

    def __init__(self, Kn=1.0, a1=2.0, a0=1.0, q_wn=50.0, dt=0.01):
        self.Kn = Kn
        self.a1 = a1
        self.a0 = a0
        self.q_wn = q_wn
        self.q_zeta = 0.707  # Butterworth
        self.dt = dt
        self.x1 = 0.0
        self.x2 = 0.0
        self.y_hat = 0.0
        self.disturbance_hat = 0.0
        self.prev_u = 0.0
        self.initialized = True

    def update(self, u, y):
        if not self.initialized:
            return 0.0
        dt = self.dt
        L1 = 2.0 * self.q_zeta * self.q_wn
        L2 = self.q_wn * self.q_wn

        e = y - self.x1
        dx1 = self.x2 + L1 * e
        dx2 = -self.a0 * self.x1 - self.a1 * self.x2 + self.Kn * u + L2 * e

        self.x1 += dx1 * dt
        self.x2 += dx2 * dt

        self.disturbance_hat = dx2 + self.a0 * self.x1 + self.a1 * self.x2 - self.Kn * u
        self.prev_u = u
        return self.disturbance_hat

    def get_disturbance(self):
        return self.disturbance_hat

    def reset(self):
        self.x1 = 0.0
        self.x2 = 0.0
        self.disturbance_hat = 0.0
        self.prev_u = 0.0


class DOB_Velocity_Simulator:
    """速度扰动观测器模拟"""

    def __init__(self, model_K=1.0, model_tau=0.1, alpha=0.1, dt=0.01):
        self.model_K = model_K
        self.model_tau = model_tau
        self.alpha = alpha
        self.dt = dt
        self.d_hat = 0.0
        self.prev_u = 0.0
        self.prev_y = 0.0
        self.initialized = True

    def update(self, u, y):
        if not self.initialized:
            return 0.0
        dt = self.dt
        alpha_model = dt / self.model_tau
        y_pred = ((1.0 - alpha_model) * self.prev_y
                  + self.model_K * alpha_model * self.prev_u)
        e = y_pred - y
        self.d_hat = (1.0 - self.alpha) * self.d_hat + self.alpha * e
        self.prev_u = u
        self.prev_y = y
        return self.d_hat

    def get_disturbance(self):
        return self.d_hat

    def reset(self):
        self.d_hat = 0.0
        self.prev_u = 0.0
        self.prev_y = 0.0


def complement_filter(raw, filtered, alpha):
    return alpha * raw + (1.0 - alpha) * filtered


# ── 一阶DOB测试 ──

class TestDOB_FirstOrder_Init(unittest.TestCase):
    """一阶DOB初始化测试"""

    def test_default_params(self):
        dob = DOB_FirstOrder_Simulator(Kn=2.0, an=1.0, q_tau=0.1, dt=0.01)
        self.assertEqual(dob.Kn, 2.0)
        self.assertEqual(dob.an, 1.0)
        self.assertEqual(dob.q_tau, 0.1)

    def test_alpha_computed(self):
        dob = DOB_FirstOrder_Simulator(dt=0.1, q_tau=0.1)
        # alpha = dt/(tau+dt) = 0.1/0.2 = 0.5
        self.assertAlmostEqual(dob.q_alpha, 0.5, places=5)

    def test_initialized_flag(self):
        dob = DOB_FirstOrder_Simulator()
        self.assertTrue(dob.initialized)


class TestDOB_FirstOrder_Update(unittest.TestCase):
    """一阶DOB更新测试"""

    def test_returns_float(self):
        dob = DOB_FirstOrder_Simulator()
        result = dob.update(u=1.0, y=0.5)
        self.assertIsInstance(result, float)

    def test_initial_disturbance_zero(self):
        """初始扰动估计应为0"""
        dob = DOB_FirstOrder_Simulator()
        self.assertEqual(dob.get_disturbance(), 0.0)

    def test_update_changes_state(self):
        dob = DOB_FirstOrder_Simulator()
        dob.update(u=1.0, y=0.0)
        self.assertNotEqual(dob.q_state, 0.0)

    def test_detects_constant_disturbance(self):
        """应能检测恒定扰动"""
        dob = DOB_FirstOrder_Simulator(Kn=1.0, an=1.0, q_tau=0.05, dt=0.01)
        # 模拟: y = model_output + d, d=2.0
        u = 1.0
        d = 2.0
        y = u + d  # 简化: 测量值包含扰动
        for _ in range(1000):
            dob.update(u, y)
        # 扰动估计应趋近d
        self.assertGreater(dob.get_disturbance(), 0.5)


class TestDOB_FirstOrder_Reset(unittest.TestCase):
    """一阶DOB复位测试"""

    def test_reset_clears_state(self):
        dob = DOB_FirstOrder_Simulator()
        dob.update(u=1.0, y=0.5)
        dob.reset()
        self.assertEqual(dob.q_state, 0.0)
        self.assertEqual(dob.disturbance_hat, 0.0)
        self.assertEqual(dob.prev_u, 0.0)
        self.assertEqual(dob.prev_y, 0.0)


# ── 二阶DOB测试 ──

class TestDOB_SecondOrder_Init(unittest.TestCase):
    """二阶DOB初始化测试"""

    def test_default_params(self):
        dob = DOB_SecondOrder_Simulator(Kn=1.0, a1=2.0, a0=1.0, q_wn=50.0)
        self.assertEqual(dob.Kn, 1.0)
        self.assertEqual(dob.a1, 2.0)
        self.assertEqual(dob.a0, 1.0)
        self.assertAlmostEqual(dob.q_zeta, 0.707, places=3)


class TestDOB_SecondOrder_Update(unittest.TestCase):
    """二阶DOB更新测试"""

    def test_returns_float(self):
        dob = DOB_SecondOrder_Simulator()
        result = dob.update(u=1.0, y=0.5)
        self.assertIsInstance(result, float)

    def test_tracks_state(self):
        """应能跟踪系统状态"""
        dob = DOB_SecondOrder_Simulator(Kn=1.0, a1=2.0, a0=1.0, q_wn=100.0, dt=0.001)
        # 恒定输出
        for _ in range(5000):
            dob.update(u=0.0, y=5.0)
        self.assertAlmostEqual(dob.x1, 5.0, delta=1.0)

    def test_estimates_disturbance(self):
        """应能估计扰动"""
        dob = DOB_SecondOrder_Simulator(Kn=1.0, a1=2.0, a0=1.0, q_wn=50.0, dt=0.001)
        d = 3.0
        for _ in range(5000):
            dob.update(u=0.0, y=d)
        # x1应趋近d
        self.assertAlmostEqual(dob.x1, d, delta=1.0)


class TestDOB_SecondOrder_Reset(unittest.TestCase):
    """二阶DOB复位测试"""

    def test_reset_clears_state(self):
        dob = DOB_SecondOrder_Simulator()
        dob.update(u=1.0, y=0.5)
        dob.reset()
        self.assertEqual(dob.x1, 0.0)
        self.assertEqual(dob.x2, 0.0)
        self.assertEqual(dob.disturbance_hat, 0.0)


# ── 速度扰动观测器测试 ──

class TestDOB_Velocity_Init(unittest.TestCase):
    """速度DOB初始化测试"""

    def test_default_params(self):
        dob = DOB_Velocity_Simulator(model_K=1.0, model_tau=0.1, alpha=0.1, dt=0.01)
        self.assertEqual(dob.model_K, 1.0)
        self.assertEqual(dob.model_tau, 0.1)
        self.assertAlmostEqual(dob.alpha, 0.1)


class TestDOB_Velocity_Update(unittest.TestCase):
    """速度DOB更新测试"""

    def test_returns_float(self):
        dob = DOB_Velocity_Simulator()
        result = dob.update(u=1.0, y=0.5)
        self.assertIsInstance(result, float)

    def test_initial_zero(self):
        dob = DOB_Velocity_Simulator()
        self.assertEqual(dob.get_disturbance(), 0.0)

    def test_detects_disturbance(self):
        """应能检测扰动"""
        dob = DOB_Velocity_Simulator(model_K=1.0, model_tau=0.1, alpha=0.2, dt=0.01)
        u = 1.0
        d = 2.0
        for _ in range(500):
            dob.update(u, u + d)
        self.assertGreater(abs(dob.get_disturbance()), 0.1)


class TestDOB_Velocity_Reset(unittest.TestCase):
    """速度DOB复位测试"""

    def test_reset_clears_state(self):
        dob = DOB_Velocity_Simulator()
        dob.update(u=1.0, y=0.5)
        dob.reset()
        self.assertEqual(dob.d_hat, 0.0)
        self.assertEqual(dob.prev_u, 0.0)
        self.assertEqual(dob.prev_y, 0.0)


# ── 互补滤波器测试 ──

class TestComplementFilter(unittest.TestCase):
    """互补滤波器测试"""

    def test_alpha_one_returns_raw(self):
        """alpha=1应返回原始值"""
        result = complement_filter(10.0, 5.0, 1.0)
        self.assertAlmostEqual(result, 10.0, places=5)

    def test_alpha_zero_returns_filtered(self):
        """alpha=0应返回滤波值"""
        result = complement_filter(10.0, 5.0, 0.0)
        self.assertAlmostEqual(result, 5.0, places=5)

    def test_alpha_half_mixed(self):
        """alpha=0.5应返回平均值"""
        result = complement_filter(10.0, 6.0, 0.5)
        self.assertAlmostEqual(result, 8.0, places=5)

    def test_interpolation(self):
        """应正确插值"""
        result = complement_filter(10.0, 0.0, 0.3)
        self.assertAlmostEqual(result, 3.0, places=5)


# ── 综合测试 ──

class TestDOB_Convergence(unittest.TestCase):
    """扰动估计收敛性测试"""

    def test_first_order_converges(self):
        """一阶DOB应收敛到真实扰动"""
        dob = DOB_FirstOrder_Simulator(Kn=1.0, an=1.0, q_tau=0.02, dt=0.001)
        u = 0.0
        d = 5.0
        for _ in range(10000):
            dob.update(u, u + d)
        # 应趋近扰动值
        self.assertAlmostEqual(dob.get_disturbance(), d, delta=1.0)

    def test_velocity_converges(self):
        """速度DOB应收敛"""
        dob = DOB_Velocity_Simulator(model_K=1.0, model_tau=0.1, alpha=0.3, dt=0.01)
        u = 0.0
        d = 3.0
        for _ in range(2000):
            dob.update(u, u + d)
        self.assertAlmostEqual(dob.get_disturbance(), d, delta=1.0)


class TestDOB_AllTypes(unittest.TestCase):
    """所有类型DOB基本功能测试"""

    def test_first_order_runs(self):
        dob = DOB_FirstOrder_Simulator()
        for _ in range(100):
            dob.update(u=1.0, y=0.5)
        self.assertIsNotNone(dob.get_disturbance())

    def test_second_order_runs(self):
        dob = DOB_SecondOrder_Simulator()
        for _ in range(100):
            dob.update(u=1.0, y=0.5)
        self.assertIsNotNone(dob.get_disturbance())

    def test_velocity_runs(self):
        dob = DOB_Velocity_Simulator()
        for _ in range(100):
            dob.update(u=1.0, y=0.5)
        self.assertIsNotNone(dob.get_disturbance())


if __name__ == '__main__':
    unittest.main()
