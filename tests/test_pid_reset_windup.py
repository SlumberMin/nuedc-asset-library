#!/usr/bin/env python3
"""
积分重置抗饱和PID单元测试
覆盖: 无抗饱和/精确重置/条件积分/死区重置、双向饱和、收敛性
"""
import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '11_控制算法库', 'simulation'))
from pid_reset_windup_simulation import PIDResetWindup, SecondOrderPlant


class TestPIDResetWindupInit(unittest.TestCase):
    """初始化测试"""

    def test_default_state(self):
        pid = PIDResetWindup(kp=1.0, ki=0.5, kd=0.1, dt=0.01)
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)
        self.assertTrue(pid.first_run)

    def test_default_mode_is_exact_reset(self):
        pid = PIDResetWindup(kp=1.0, ki=0.5, kd=0.1, dt=0.01)
        self.assertEqual(pid.mode, PIDResetWindup.MODE_EXACT_RESET)

    def test_mode_constants(self):
        self.assertEqual(PIDResetWindup.MODE_NONE, 0)
        self.assertEqual(PIDResetWindup.MODE_EXACT_RESET, 1)
        self.assertEqual(PIDResetWindup.MODE_CONDITIONAL, 2)
        self.assertEqual(PIDResetWindup.MODE_DEADZONE, 3)


class TestPIDResetWindupNoAntiWindup(unittest.TestCase):
    """无抗饱和模式测试"""

    def test_output_clamped(self):
        pid = PIDResetWindup(kp=10.0, ki=50.0, kd=0.0, dt=0.01, out_min=-5, out_max=5)
        pid.mode = PIDResetWindup.MODE_NONE
        output, _, _, _ = pid.compute(setpoint=100.0, pv=0.0)
        self.assertLessEqual(output, 5.0)
        self.assertGreaterEqual(output, -5.0)

    def test_integral_winds_up(self):
        """无抗饱和时积分应持续增长"""
        pid = PIDResetWindup(kp=1.0, ki=10.0, kd=0.0, dt=0.01, out_min=-5, out_max=5)
        pid.mode = PIDResetWindup.MODE_NONE
        for _ in range(100):
            pid.compute(setpoint=100.0, pv=0.0)
        self.assertGreater(abs(pid.integral), 10.0)


class TestPIDResetWindupExactReset(unittest.TestCase):
    """精确重置抗饱和测试"""

    def test_output_within_limits(self):
        pid = PIDResetWindup(kp=4.0, ki=8.0, kd=0.3, dt=0.01, out_min=-3, out_max=3)
        pid.mode = PIDResetWindup.MODE_EXACT_RESET
        for _ in range(200):
            output, _, _, _ = pid.compute(setpoint=2.0, pv=0.0)
            self.assertLessEqual(output, 3.0 + 0.01)
            self.assertGreaterEqual(output, -3.0 - 0.01)

    def test_exact_reset_limits_integral(self):
        """精确重置应限制积分增长"""
        pid_no = PIDResetWindup(kp=4.0, ki=8.0, kd=0.0, dt=0.01, out_min=-3, out_max=3)
        pid_no.mode = PIDResetWindup.MODE_NONE

        pid_reset = PIDResetWindup(kp=4.0, ki=8.0, kd=0.0, dt=0.01, out_min=-3, out_max=3)
        pid_reset.mode = PIDResetWindup.MODE_EXACT_RESET

        for _ in range(100):
            pid_no.compute(setpoint=10.0, pv=0.0)
            pid_reset.compute(setpoint=10.0, pv=0.0)

        # 精确重置积分应更小
        self.assertLess(abs(pid_reset.integral), abs(pid_no.integral))

    def test_saturated_flag(self):
        pid = PIDResetWindup(kp=10.0, ki=50.0, kd=0.0, dt=0.01, out_min=-1, out_max=1)
        pid.mode = PIDResetWindup.MODE_EXACT_RESET
        output, _, _, _ = pid.compute(setpoint=100.0, pv=0.0)
        self.assertTrue(pid.saturated)

    def test_not_saturated_normal_operation(self):
        pid = PIDResetWindup(kp=1.0, ki=1.0, kd=0.0, dt=0.01, out_min=-100, out_max=100)
        pid.mode = PIDResetWindup.MODE_EXACT_RESET
        output, _, _, _ = pid.compute(setpoint=0.1, pv=0.0)
        self.assertFalse(pid.saturated)


class TestPIDResetWindupConditional(unittest.TestCase):
    """条件积分抗饱和测试"""

    def test_conditional_limits_integral(self):
        pid = PIDResetWindup(kp=4.0, ki=8.0, kd=0.0, dt=0.01, out_min=-3, out_max=3)
        pid.mode = PIDResetWindup.MODE_CONDITIONAL
        for _ in range(100):
            pid.compute(setpoint=10.0, pv=0.0)
        # 条件积分应比无抗饱和积分更小
        pid_no = PIDResetWindup(kp=4.0, ki=8.0, kd=0.0, dt=0.01, out_min=-3, out_max=3)
        pid_no.mode = PIDResetWindup.MODE_NONE
        for _ in range(100):
            pid_no.compute(setpoint=10.0, pv=0.0)
        self.assertLess(abs(pid.integral), abs(pid_no.integral))


class TestPIDResetWindupDeadzone(unittest.TestCase):
    """死区重置测试"""

    def test_deadzone_resets_integral(self):
        pid = PIDResetWindup(kp=1.0, ki=5.0, kd=0.0, dt=0.01, out_min=-100, out_max=100)
        pid.mode = PIDResetWindup.MODE_DEADZONE
        pid.deadzone = 0.1

        # 小误差应在死区内
        output, _, i_term, _ = pid.compute(setpoint=1.0, pv=0.95)
        # 误差=0.05 < 死区0.1, 积分应被重置
        self.assertAlmostEqual(pid.integral, 0.0, delta=1e-6)
        self.assertAlmostEqual(i_term, 0.0, delta=1e-6)

    def test_deadzone_outside_normal(self):
        pid = PIDResetWindup(kp=1.0, ki=5.0, kd=0.0, dt=0.01, out_min=-100, out_max=100)
        pid.mode = PIDResetWindup.MODE_DEADZONE
        pid.deadzone = 0.1

        # 大误差应正常工作
        output, _, i_term, _ = pid.compute(setpoint=1.0, pv=0.0)
        # 误差=1.0 > 死区0.1, 积分不应为0
        self.assertNotAlmostEqual(i_term, 0.0, places=1)


class TestPIDResetWindupReset(unittest.TestCase):
    """重置功能测试"""

    def test_reset_clears_state(self):
        pid = PIDResetWindup(kp=1.0, ki=1.0, kd=0.5, dt=0.01)
        for _ in range(50):
            pid.compute(setpoint=10.0, pv=0.0)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)
        self.assertTrue(pid.first_run)


class TestPIDResetWindupConvergence(unittest.TestCase):
    """收敛性测试"""

    def test_converges_with_exact_reset(self):
        """精确重置模式应能收敛"""
        plant = SecondOrderPlant(wn=15.0, zeta=0.3, dt=0.001)
        pid = PIDResetWindup(kp=3.0, ki=6.0, kd=0.2, dt=0.001,
                             out_min=-10, out_max=10)
        pid.mode = PIDResetWindup.MODE_EXACT_RESET
        setpoint = 1.0

        for _ in range(5000):
            pv = plant.x1
            output, _, _, _ = pid.compute(setpoint, pv)
            plant.update(output)

        self.assertAlmostEqual(plant.x1, setpoint, delta=0.2)

    def test_output_returns_tuple(self):
        pid = PIDResetWindup(kp=1.0, ki=0.5, kd=0.1, dt=0.01)
        result = pid.compute(setpoint=1.0, pv=0.0)
        self.assertEqual(len(result), 4)
        output, p_term, i_term, d_term = result
        self.assertIsInstance(output, float)


class TestSecondOrderPlant(unittest.TestCase):
    """二阶系统测试"""

    def test_zero_input_stays_zero(self):
        plant = SecondOrderPlant()
        for _ in range(100):
            y = plant.update(0.0)
        self.assertAlmostEqual(y, 0.0, delta=1e-10)

    def test_step_response(self):
        plant = SecondOrderPlant(wn=15.0, zeta=0.3, dt=0.001)
        for _ in range(5000):
            y = plant.update(1.0)
        self.assertAlmostEqual(y, 1.0, delta=0.15)

    def test_reset(self):
        plant = SecondOrderPlant()
        for _ in range(100):
            plant.update(1.0)
        plant.reset()
        self.assertEqual(plant.x1, 0.0)
        self.assertEqual(plant.x2, 0.0)


if __name__ == '__main__':
    unittest.main()
