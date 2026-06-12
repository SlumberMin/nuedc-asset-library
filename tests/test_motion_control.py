#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
运动控制单元测试
覆盖: TrapezoidalProfile初始化/状态、SCurveProfile初始化/状态、
      CubicPolynomial初始化/状态、PIDController、点到点控制、轨迹边界条件
"""

import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from _11_控制算法库.simulation.motion_control_simulation import (
    TrapezoidalProfile,
    SCurveProfile,
    CubicPolynomial,
    PIDController,
)


# ==================== TrapezoidalProfile 测试 ====================

class TestTrapezoidalProfileInit(unittest.TestCase):
    """梯形速度规划初始化测试"""

    def test_basic_params(self):
        p = TrapezoidalProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, dt=0.001)
        self.assertEqual(p.q0, 0)
        self.assertEqual(p.qf, 10)
        self.assertEqual(p.v_max, 5.0)
        self.assertEqual(p.a_max, 10.0)

    def test_direction_positive(self):
        p = TrapezoidalProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, dt=0.001)
        self.assertEqual(p.dir, 1.0)

    def test_direction_negative(self):
        p = TrapezoidalProfile(q0=10, qf=0, v_max=5.0, a_max=10.0, dt=0.001)
        self.assertEqual(p.dir, -1.0)

    def test_total_time_positive(self):
        p = TrapezoidalProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, dt=0.001)
        self.assertGreater(p.T_total, 0.0)

    def test_triangle_profile(self):
        """当距离短无法达到v_max时,应为三角形"""
        p = TrapezoidalProfile(q0=0, qf=0.1, v_max=100.0, a_max=10.0, dt=0.001)
        self.assertEqual(p.T_const, 0.0)


class TestTrapezoidalProfileState(unittest.TestCase):
    """梯形速度规划状态测试"""

    def test_initial_state(self):
        p = TrapezoidalProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, dt=0.001)
        pos, vel, acc = p.get_state(0.0)
        self.assertAlmostEqual(pos, 0.0, delta=0.01)
        self.assertAlmostEqual(vel, 0.0, delta=0.01)

    def test_final_state(self):
        p = TrapezoidalProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, dt=0.001)
        pos, vel, acc = p.get_state(p.T_total + 0.1)
        self.assertAlmostEqual(pos, 10.0, delta=0.1)
        self.assertAlmostEqual(vel, 0.0, delta=0.1)

    def test_monotonically_increasing(self):
        """正向运动时位置应单调递增"""
        p = TrapezoidalProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, dt=0.001)
        prev_pos = -1.0
        for t in np.linspace(0, p.T_total, 100):
            pos, _, _ = p.get_state(t)
            self.assertGreaterEqual(pos, prev_pos - 0.01)
            prev_pos = pos

    def test_negative_direction(self):
        """反向运动时终点应在起点左侧"""
        p = TrapezoidalProfile(q0=10, qf=0, v_max=5.0, a_max=10.0, dt=0.001)
        pos, _, _ = p.get_state(p.T_total + 0.1)
        self.assertAlmostEqual(pos, 0.0, delta=0.1)

    def test_mid_acceleration_phase(self):
        """加速阶段速度应递增"""
        p = TrapezoidalProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, dt=0.001)
        _, vel1, _ = p.get_state(0.0)
        _, vel2, _ = p.get_state(p.T_accel * 0.5)
        self.assertGreater(vel2, vel1)

    def test_returns_tuple(self):
        p = TrapezoidalProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, dt=0.001)
        result = p.get_state(0.5)
        self.assertEqual(len(result), 3)


# ==================== SCurveProfile 测试 ====================

class TestSCurveProfileInit(unittest.TestCase):
    """S形速度规划初始化测试"""

    def test_basic_params(self):
        p = SCurveProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, j_max=50.0, dt=0.001)
        self.assertEqual(p.q0, 0)
        self.assertEqual(p.qf, 10)
        self.assertEqual(p.v_max, 5.0)
        self.assertEqual(p.a_max, 10.0)
        self.assertEqual(p.j_max, 50.0)

    def test_total_time_positive(self):
        p = SCurveProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, j_max=50.0, dt=0.001)
        self.assertGreater(p.T_total, 0.0)

    def test_seven_phases(self):
        """应有7段时间"""
        p = SCurveProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, j_max=50.0, dt=0.001)
        self.assertEqual(len(p.T), 7)


class TestSCurveProfileState(unittest.TestCase):
    """S形速度规划状态测试"""

    def test_initial_state(self):
        p = SCurveProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, j_max=50.0, dt=0.001)
        pos, vel, acc = p.get_state(0.0)
        self.assertAlmostEqual(pos, 0.0, delta=0.1)
        self.assertAlmostEqual(vel, 0.0, delta=0.1)

    def test_final_state(self):
        p = SCurveProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, j_max=50.0, dt=0.001)
        pos, vel, acc = p.get_state(p.T_total + 0.1)
        self.assertAlmostEqual(pos, 10.0, delta=0.5)
        self.assertAlmostEqual(vel, 0.0, delta=0.5)

    def test_returns_tuple(self):
        p = SCurveProfile(q0=0, qf=10, v_max=5.0, a_max=10.0, j_max=50.0, dt=0.001)
        result = p.get_state(0.5)
        self.assertEqual(len(result), 3)


# ==================== CubicPolynomial 测试 ====================

class TestCubicPolynomialInit(unittest.TestCase):
    """三次多项式轨迹初始化测试"""

    def test_coefficients(self):
        c = CubicPolynomial(q0=0, dq0=0, qf=10, dqf=0, T=2.0, dt=0.001)
        self.assertEqual(c.a0, 0)
        self.assertEqual(c.a1, 0)
        self.assertEqual(c.T, 2.0)

    def test_start_condition(self):
        c = CubicPolynomial(q0=0, dq0=0, qf=10, dqf=0, T=2.0, dt=0.001)
        q, dq, ddq = c.get_state(0.0)
        self.assertAlmostEqual(q, 0.0, delta=0.01)
        self.assertAlmostEqual(dq, 0.0, delta=0.01)

    def test_end_condition(self):
        c = CubicPolynomial(q0=0, dq0=0, qf=10, dqf=0, T=2.0, dt=0.001)
        q, dq, ddq = c.get_state(2.0)
        self.assertAlmostEqual(q, 10.0, delta=0.01)
        self.assertAlmostEqual(dq, 0.0, delta=0.01)

    def test_nonzero_initial_velocity(self):
        c = CubicPolynomial(q0=0, dq0=5, qf=10, dqf=0, T=2.0, dt=0.001)
        _, dq, _ = c.get_state(0.0)
        self.assertAlmostEqual(dq, 5.0, delta=0.01)


class TestCubicPolynomialState(unittest.TestCase):
    """三次多项式状态测试"""

    def test_returns_tuple(self):
        c = CubicPolynomial(q0=0, dq0=0, qf=10, dqf=0, T=2.0, dt=0.001)
        result = c.get_state(1.0)
        self.assertEqual(len(result), 3)

    def test_midpoint_between(self):
        """中点位置应在起点和终点之间"""
        c = CubicPolynomial(q0=0, dq0=0, qf=10, dqf=0, T=2.0, dt=0.001)
        q, _, _ = c.get_state(1.0)
        self.assertGreater(q, 0.0)
        self.assertLess(q, 10.0)

    def test_beyond_T_clamps(self):
        """超过T后应返回终点状态"""
        c = CubicPolynomial(q0=0, dq0=0, qf=10, dqf=0, T=2.0, dt=0.001)
        q, _, _ = c.get_state(5.0)
        self.assertAlmostEqual(q, 10.0, delta=0.01)

    def test_negative_motion(self):
        """反向运动"""
        c = CubicPolynomial(q0=10, dq0=0, qf=0, dqf=0, T=2.0, dt=0.001)
        q, _, _ = c.get_state(2.0)
        self.assertAlmostEqual(q, 0.0, delta=0.01)


# ==================== PIDController 测试 ====================

class TestPIDControllerInit(unittest.TestCase):
    """PID控制器初始化测试"""

    def test_params(self):
        pid = PIDController(Kp=50.0, Ki=10.0, Kd=5.0, dt=0.001)
        self.assertEqual(pid.Kp, 50.0)
        self.assertEqual(pid.Ki, 10.0)
        self.assertEqual(pid.Kd, 5.0)

    def test_initial_state(self):
        pid = PIDController(Kp=50.0, Ki=10.0, Kd=5.0, dt=0.001)
        self.assertEqual(pid.err_sum, 0.0)
        self.assertEqual(pid.err_prev, 0.0)


class TestPIDControllerUpdate(unittest.TestCase):
    """PID控制器更新测试"""

    def test_returns_float(self):
        pid = PIDController(Kp=50.0, Ki=10.0, Kd=5.0, dt=0.001)
        result = pid.update(ref=1.0, meas=0.5)
        self.assertIsInstance(result, float)

    def test_zero_error_zero_output(self):
        """零误差应产生零输出"""
        pid = PIDController(Kp=50.0, Ki=10.0, Kd=5.0, dt=0.001)
        # 第一次调用时err_prev=0,d_err也会计算
        result = pid.update(ref=1.0, meas=1.0)
        self.assertEqual(result, 0.0)

    def test_positive_error_positive_output(self):
        """正误差应产生正输出"""
        pid = PIDController(Kp=50.0, Ki=10.0, Kd=5.0, dt=0.001)
        result = pid.update(ref=1.0, meas=0.0)
        self.assertGreater(result, 0.0)

    def test_integral_accumulation(self):
        """积分项应累积"""
        pid = PIDController(Kp=0.0, Ki=10.0, Kd=0.0, dt=0.001)
        pid.update(ref=1.0, meas=0.0)
        pid.update(ref=1.0, meas=0.0)
        self.assertGreater(pid.err_sum, 0.0)

    def test_derivative_response(self):
        """微分项应响应误差变化"""
        pid = PIDController(Kp=0.0, Ki=0.0, Kd=10.0, dt=0.001)
        u1 = pid.update(ref=1.0, meas=0.0)  # err从0→1
        u2 = pid.update(ref=1.0, meas=0.0)  # err从1→1,d_err=0
        # 第一次调用有d_err,第二次没有
        self.assertNotAlmostEqual(u1, u2, delta=0.01)


# ==================== 综合测试 ====================

class TestMotionControlIntegration(unittest.TestCase):
    """运动控制集成测试"""

    def test_trap_profile_with_pid_tracking(self):
        """梯形规划+PID跟踪"""
        dt = 0.001
        trap = TrapezoidalProfile(q0=0, qf=5, v_max=3.0, a_max=10.0, dt=dt)
        pid = PIDController(Kp=50.0, Ki=10.0, Kd=5.0, dt=dt)

        m, b = 1.0, 2.0
        x, dx = 0.0, 0.0

        for i in range(3000):
            t = i * dt
            ref, _, _ = trap.get_state(t)
            u = pid.update(ref, x)
            ddx = (u - b * dx) / m
            dx += ddx * dt
            x += dx * dt

        # 应趋近目标
        self.assertGreater(x, 3.0)

    def test_cubic_with_pid_tracking(self):
        """三次多项式+PID跟踪"""
        dt = 0.001
        cubic = CubicPolynomial(q0=0, dq0=0, qf=5, dqf=0, T=2.0, dt=dt)
        pid = PIDController(Kp=50.0, Ki=10.0, Kd=5.0, dt=dt)

        m, b = 1.0, 2.0
        x, dx = 0.0, 0.0

        for i in range(2000):
            t = i * dt
            ref, _, _ = cubic.get_state(t)
            u = pid.update(ref, x)
            ddx = (u - b * dx) / m
            dx += ddx * dt
            x += dx * dt

        self.assertGreater(x, 2.0)


if __name__ == '__main__':
    unittest.main()
