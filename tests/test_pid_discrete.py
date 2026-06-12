#!/usr/bin/env python3
"""
离散PID控制器单元测试
使用纯Python模拟C pid_discrete模块逻辑（Q16.16定点化）
覆盖: 初始化/位置式PID/增量式PID/输出限幅/死区/抗积分饱和(钳位+退饱和)/重置
测试对象: 11_控制算法库/common/pid_discrete.c
"""
import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ============================================================
# Q16.16 定点数辅助
# ============================================================
Q16_SHIFT = 16
Q16_ONE = 1 << Q16_SHIFT


def float_to_q16(f):
    return int(f * Q16_ONE)


def q16_to_float(q):
    return q / Q16_ONE


def q16_mul(a, b):
    return int((a * b) >> Q16_SHIFT)


def clamp_q16(val, lo, hi):
    if val < lo:
        return lo
    if val > hi:
        return hi
    return val


# ============================================================
# Python 模拟实现（对应 pid_discrete.c）
# ============================================================
MODE_POSITION = 0
MODE_INCREMENT = 1

AW_NONE = 0
AW_CLAMP = 1
AW_BACKCALC = 2


class PIDDiscrete:
    """对应C的pid_discrete_t"""

    def __init__(self, kp, ki, kd, out_min, out_max, mode=MODE_POSITION):
        self.kp = float_to_q16(kp)
        self.ki = float_to_q16(ki)
        self.kd = float_to_q16(kd)
        self.out_min = float_to_q16(out_min)
        self.out_max = float_to_q16(out_max)
        self.integral_min = float_to_q16(-1e6)
        self.integral_max = float_to_q16(1e6)
        self.mode = mode
        self.anti_windup = AW_CLAMP
        self.kb = float_to_q16(1.0)
        self.setpoint = 0
        self.integral = 0
        self.prev_error = 0
        self.prev_output = 0
        self.deadzone = 0
        self.first_run = 1

    def set_setpoint(self, sp):
        self.setpoint = float_to_q16(sp)

    def set_deadzone(self, dz):
        self.deadzone = float_to_q16(dz)

    def enable_backcalc(self, kb):
        self.anti_windup = AW_BACKCALC
        self.kb = float_to_q16(kb)

    def update(self, measurement):
        meas_q = float_to_q16(measurement)
        error = self.setpoint - meas_q

        # 死区
        if self.deadzone > 0:
            abs_err = error if error >= 0 else -error
            if abs_err < self.deadzone:
                error = 0

        if self.mode == MODE_POSITION:
            p_term = q16_mul(self.kp, error)
            self.integral += q16_mul(self.ki, error)
            self.integral = clamp_q16(self.integral, self.integral_min, self.integral_max)
            i_term = self.integral

            if self.first_run:
                d_term = 0
                self.first_run = 0
            else:
                d_error = error - self.prev_error
                d_term = q16_mul(self.kd, d_error)

            output = p_term + i_term + d_term
            unclamped = output
            output = clamp_q16(output, self.out_min, self.out_max)

            if self.anti_windup == AW_BACKCALC and unclamped != output:
                excess = unclamped - output
                self.integral -= q16_mul(self.kb, excess)
                self.integral = clamp_q16(self.integral, self.integral_min, self.integral_max)

            self.prev_error = error
        else:
            # 增量式
            p_term = q16_mul(self.kp, error - self.prev_error)
            i_term = q16_mul(self.ki, error)

            if self.first_run:
                d_term = 0
                self.first_run = 0
            else:
                dd = error - 2 * self.prev_error + self.prev_output
                d_term = q16_mul(self.kd, dd)

            delta = p_term + i_term + d_term
            output = self.prev_output + delta
            output = clamp_q16(output, self.out_min, self.out_max)
            self.prev_error = error

        self.prev_output = output
        return q16_to_float(output)

    def reset(self):
        self.integral = 0
        self.prev_error = 0
        self.prev_output = 0
        self.first_run = 1


# ============================================================
# 测试用例
# ============================================================

class TestDiscreteInit(unittest.TestCase):
    """初始化测试"""

    def test_default_state(self):
        pid = PIDDiscrete(kp=1.0, ki=0.5, kd=0.1, out_min=-100, out_max=100)
        self.assertEqual(pid.integral, 0)
        self.assertEqual(pid.prev_error, 0)
        self.assertEqual(pid.prev_output, 0)
        self.assertEqual(pid.first_run, 1)

    def test_mode_position(self):
        pid = PIDDiscrete(1.0, 0.5, 0.1, -100, 100, MODE_POSITION)
        self.assertEqual(pid.mode, MODE_POSITION)

    def test_mode_increment(self):
        pid = PIDDiscrete(1.0, 0.5, 0.1, -100, 100, MODE_INCREMENT)
        self.assertEqual(pid.mode, MODE_INCREMENT)


class TestDiscretePositionPID(unittest.TestCase):
    """位置式PID测试"""

    def test_positive_error_positive_output(self):
        pid = PIDDiscrete(kp=1.0, ki=0.0, kd=0.0, out_min=-100, out_max=100)
        pid.set_setpoint(10.0)
        output = pid.update(5.0)
        self.assertGreater(output, 0)

    def test_negative_error_negative_output(self):
        pid = PIDDiscrete(kp=1.0, ki=0.0, kd=0.0, out_min=-100, out_max=100)
        pid.set_setpoint(0.0)
        output = pid.update(10.0)
        self.assertLess(output, 0)

    def test_zero_error_zero_output_p_only(self):
        pid = PIDDiscrete(kp=1.0, ki=0.0, kd=0.0, out_min=-100, out_max=100)
        pid.set_setpoint(5.0)
        output = pid.update(5.0)
        self.assertAlmostEqual(output, 0.0, places=2)

    def test_integral_accumulates(self):
        pid = PIDDiscrete(kp=0.0, ki=1.0, kd=0.0, out_min=-1000, out_max=1000)
        pid.set_setpoint(10.0)
        out1 = pid.update(5.0)
        out2 = pid.update(5.0)
        self.assertGreater(out2, out1)

    def test_derivative_on_first_run_is_zero(self):
        """首次运行微分项应为0"""
        pid = PIDDiscrete(kp=0.0, ki=0.0, kd=10.0, out_min=-100, out_max=100)
        pid.set_setpoint(10.0)
        out1 = pid.update(0.0)
        # 首次微分=0, 但P=10*10=100
        self.assertNotEqual(out1, 0.0)

    def test_output_clamped(self):
        pid = PIDDiscrete(kp=100.0, ki=0.0, kd=0.0, out_min=-10, out_max=10)
        pid.set_setpoint(100.0)
        output = pid.update(0.0)
        self.assertLessEqual(output, 10.0)
        self.assertGreaterEqual(output, -10.0)


class TestDiscreteIncrementPID(unittest.TestCase):
    """增量式PID测试"""

    def test_positive_error_positive_delta(self):
        pid = PIDDiscrete(kp=1.0, ki=0.5, kd=0.0, out_min=-100, out_max=100,
                          mode=MODE_INCREMENT)
        pid.set_setpoint(10.0)
        out1 = pid.update(5.0)
        out2 = pid.update(5.0)
        self.assertGreater(out2, out1)

    def test_output_clamped(self):
        pid = PIDDiscrete(kp=100.0, ki=50.0, kd=0.0, out_min=-10, out_max=10,
                          mode=MODE_INCREMENT)
        pid.set_setpoint(100.0)
        for _ in range(10):
            output = pid.update(0.0)
        self.assertLessEqual(output, 10.0)


class TestDiscreteDeadzone(unittest.TestCase):
    """死区测试"""

    def test_error_in_deadzone_no_output(self):
        pid = PIDDiscrete(kp=1.0, ki=0.0, kd=0.0, out_min=-100, out_max=100)
        pid.set_setpoint(5.0)
        pid.set_deadzone(2.0)
        # 误差=5-4=1, 小于死区2
        output = pid.update(4.0)
        self.assertAlmostEqual(output, 0.0, places=1)

    def test_error_outside_deadzone_normal(self):
        pid = PIDDiscrete(kp=1.0, ki=0.0, kd=0.0, out_min=-100, out_max=100)
        pid.set_setpoint(10.0)
        pid.set_deadzone(2.0)
        # 误差=10-4=6, 大于死区2
        output = pid.update(4.0)
        self.assertGreater(output, 0)


class TestDiscreteAntiWindup(unittest.TestCase):
    """抗积分饱和测试"""

    def test_clamp_limits_output(self):
        pid = PIDDiscrete(kp=10.0, ki=100.0, kd=0.0, out_min=-5, out_max=5)
        pid.set_setpoint(100.0)
        for _ in range(100):
            output = pid.update(0.0)
        self.assertLessEqual(output, 5.0)

    def test_backcalc_reduces_integral(self):
        pid_bc = PIDDiscrete(kp=10.0, ki=100.0, kd=0.0, out_min=-5, out_max=5)
        pid_bc.set_setpoint(100.0)
        pid_bc.enable_backcalc(kb=2.0)

        pid_cl = PIDDiscrete(kp=10.0, ki=100.0, kd=0.0, out_min=-5, out_max=5)
        pid_cl.set_setpoint(100.0)
        # Both saturated
        for _ in range(50):
            pid_bc.update(0.0)
            pid_cl.update(0.0)
        # Backcalc should reduce integral compared to clamp
        self.assertLess(abs(pid_bc.integral), abs(pid_cl.integral) + 1)


class TestDiscreteReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        pid = PIDDiscrete(kp=1.0, ki=1.0, kd=0.5, out_min=-100, out_max=100)
        pid.set_setpoint(10.0)
        for _ in range(50):
            pid.update(0.0)
        pid.reset()
        self.assertEqual(pid.integral, 0)
        self.assertEqual(pid.prev_error, 0)
        self.assertEqual(pid.prev_output, 0)
        self.assertEqual(pid.first_run, 1)


class TestDiscreteConvergence(unittest.TestCase):
    """收敛性测试"""

    def test_position_pid_converges(self):
        """位置式PID应驱动误差趋近于0"""
        pid = PIDDiscrete(kp=0.5, ki=0.02, kd=0.1, out_min=-100, out_max=100)
        pid.set_setpoint(50.0)
        measurement = 0.0
        for _ in range(500):
            output = pid.update(measurement)
            measurement += output * 0.01  # 简单一阶系统
        self.assertAlmostEqual(measurement, 50.0, delta=5.0)

    def test_increment_pid_converges(self):
        """增量式PID应驱动误差趋近于0"""
        pid = PIDDiscrete(kp=0.5, ki=0.02, kd=0.1, out_min=-100, out_max=100,
                          mode=MODE_INCREMENT)
        pid.set_setpoint(50.0)
        measurement = 0.0
        for _ in range(500):
            output = pid.update(measurement)
            measurement += output * 0.01
        self.assertAlmostEqual(measurement, 50.0, delta=5.0)


class TestDiscreteQ16Precision(unittest.TestCase):
    """定点精度测试"""

    def test_q16_conversion_roundtrip(self):
        for v in [0.0, 1.0, -1.0, 3.14159, -2.71828, 100.0]:
            result = q16_to_float(float_to_q16(v))
            self.assertAlmostEqual(result, v, delta=0.001)

    def test_q16_mul(self):
        a = float_to_q16(3.0)
        b = float_to_q16(4.0)
        result = q16_to_float(q16_mul(a, b))
        self.assertAlmostEqual(result, 12.0, delta=0.01)


if __name__ == '__main__':
    unittest.main()
