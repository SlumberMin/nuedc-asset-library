#!/usr/bin/env python3
"""
位置式PID单元测试
覆盖: 初始化、P/I/D项、积分分离、微分先行、微分滤波、死区、输出限幅、重置
注意: 使用纯 Python 模拟 C PositionPID 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class PositionPIDSimulator:
    """位置式PID控制器模拟 (对应 position_pid.c)"""

    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, out_min=-1000.0, out_max=1000.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.out_min = out_min
        self.out_max = out_max

        self.err = 0.0
        self.err_last = 0.0
        self.err_sum = 0.0

        self.integral_min = -1e30
        self.integral_max = 1e30
        self.integral_sep_threshold = 1e30  # 默认不启用积分分离

        self.derivative_on_feedback = False
        self.feedback_last = 0.0

        self.d_filter_alpha = 0.0
        self.d_filtered = 0.0

        self.dead_zone = 0.0
        self.output = 0.0

    def set_integral_limit(self, min_val, max_val):
        self.integral_min = min_val
        self.integral_max = max_val

    def set_integral_separation(self, threshold):
        self.integral_sep_threshold = max(0.0, threshold)

    def enable_derivative_on_feedback(self, enable):
        self.derivative_on_feedback = enable

    def set_d_filter(self, alpha):
        self.d_filter_alpha = max(0.0, min(0.99, alpha))

    def set_dead_zone(self, dz):
        self.dead_zone = max(0.0, dz)

    def compute(self, setpoint, feedback):
        error = setpoint - feedback

        # 死区
        if -self.dead_zone < error < self.dead_zone:
            error = 0.0

        self.err_last = self.err
        self.err = error

        # P项
        p_out = self.Kp * error

        # I项 (带积分分离)
        i_out = 0.0
        if abs(error) < self.integral_sep_threshold:
            self.err_sum += error
            self.err_sum = max(self.integral_min, min(self.integral_max, self.err_sum))
            i_out = self.Ki * self.err_sum

        # D项
        if self.derivative_on_feedback:
            d_raw = -self.Kd * (feedback - self.feedback_last)
            self.feedback_last = feedback
        else:
            d_raw = self.Kd * (self.err - self.err_last)

        # D项滤波
        self.d_filtered = (self.d_filter_alpha * self.d_filtered
                           + (1.0 - self.d_filter_alpha) * d_raw)
        d_out = self.d_filtered

        # 合成输出
        self.output = p_out + i_out + d_out

        # 输出限幅
        self.output = max(self.out_min, min(self.out_max, self.output))

        return self.output

    def reset(self):
        self.err = 0.0
        self.err_last = 0.0
        self.err_sum = 0.0
        self.d_filtered = 0.0
        self.feedback_last = 0.0
        self.output = 0.0


class TestPositionPIDInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        pid = PositionPIDSimulator()
        self.assertEqual(pid.Kp, 1.0)
        self.assertEqual(pid.Ki, 0.0)
        self.assertEqual(pid.Kd, 0.0)

    def test_custom_params(self):
        pid = PositionPIDSimulator(Kp=2.5, Ki=0.3, Kd=0.1,
                                    out_min=-500, out_max=500)
        self.assertEqual(pid.Kp, 2.5)
        self.assertEqual(pid.Ki, 0.3)
        self.assertEqual(pid.Kd, 0.1)
        self.assertEqual(pid.out_min, -500)

    def test_initial_state_zero(self):
        """初始状态应全为0"""
        pid = PositionPIDSimulator()
        self.assertEqual(pid.err, 0.0)
        self.assertEqual(pid.err_sum, 0.0)
        self.assertEqual(pid.output, 0.0)


class TestPositionPIDProportional(unittest.TestCase):
    """比例项测试"""

    def test_p_only(self):
        """纯P控制: output = Kp * error"""
        pid = PositionPIDSimulator(Kp=2.0, Ki=0.0, Kd=0.0)
        output = pid.compute(setpoint=10.0, feedback=5.0)
        self.assertAlmostEqual(output, 10.0, places=2)

    def test_p_negative_error(self):
        """负误差应产生负输出"""
        pid = PositionPIDSimulator(Kp=1.0, Ki=0.0, Kd=0.0)
        output = pid.compute(setpoint=0.0, feedback=10.0)
        self.assertLess(output, 0)

    def test_zero_error_zero_output(self):
        """零误差纯P应输出零"""
        pid = PositionPIDSimulator(Kp=1.0, Ki=0.0, Kd=0.0)
        output = pid.compute(setpoint=5.0, feedback=5.0)
        self.assertAlmostEqual(output, 0.0, places=5)


class TestPositionPIDIntegral(unittest.TestCase):
    """积分项测试"""

    def test_integral_accumulation(self):
        """积分应累积"""
        pid = PositionPIDSimulator(Kp=0.0, Ki=1.0, Kd=0.0,
                                    out_min=-10000, out_max=10000)
        outputs = []
        for _ in range(10):
            out = pid.compute(setpoint=10.0, feedback=0.0)
            outputs.append(out)
        # 积分应单调递增
        for i in range(1, len(outputs)):
            self.assertGreaterEqual(outputs[i], outputs[i - 1])

    def test_integral_clamping(self):
        """积分应被限幅"""
        pid = PositionPIDSimulator(Kp=0.0, Ki=10.0, Kd=0.0,
                                    out_min=-10000, out_max=10000)
        pid.set_integral_limit(-10.0, 10.0)
        for _ in range(1000):
            pid.compute(setpoint=100.0, feedback=0.0)
        self.assertLessEqual(pid.err_sum, 10.0)
        self.assertGreaterEqual(pid.err_sum, -10.0)

    def test_i_only_steady_state(self):
        """纯I控制应消除稳态误差"""
        pid = PositionPIDSimulator(Kp=0.0, Ki=2.0, Kd=0.0,
                                    out_min=-1000, out_max=1000)
        pid.set_integral_limit(-1000, 1000)
        state = 0.0
        for _ in range(5000):
            output = pid.compute(setpoint=100.0, feedback=state)
            state += output * 0.001
        self.assertAlmostEqual(state, 100.0, delta=5.0)


class TestPositionPIDDerivative(unittest.TestCase):
    """微分项测试"""

    def test_d_on_step_change(self):
        """阶跃变化应产生微分项"""
        pid = PositionPIDSimulator(Kp=0.0, Ki=0.0, Kd=1.0)
        pid.compute(setpoint=0.0, feedback=0.0)
        output = pid.compute(setpoint=10.0, feedback=0.0)
        # err从0变到10, d = Kd*(10-0) = 10
        self.assertAlmostEqual(output, 10.0, places=1)

    def test_d_on_constant_error(self):
        """恒定误差微分项应为零"""
        pid = PositionPIDSimulator(Kp=0.0, Ki=0.0, Kd=1.0)
        pid.compute(setpoint=10.0, feedback=0.0)
        output = pid.compute(setpoint=10.0, feedback=0.0)
        # err不变, d = Kd*(10-10) = 0
        self.assertAlmostEqual(output, 0.0, places=1)

    def test_d_filter_reduces_noise(self):
        """微分滤波应减少噪声"""
        pid_no = PositionPIDSimulator(Kp=0.0, Ki=0.0, Kd=1.0)
        pid_no.set_d_filter(0.0)

        pid_strong = PositionPIDSimulator(Kp=0.0, Ki=0.0, Kd=1.0)
        pid_strong.set_d_filter(0.9)

        outputs_no = []
        outputs_strong = []
        for i in range(100):
            error = 10.0 * math.sin(i * 0.1) + 2.0 * math.sin(i * 5.0)
            outputs_no.append(pid_no.compute(setpoint=error + 50, feedback=50))
            outputs_strong.append(pid_strong.compute(setpoint=error + 50, feedback=50))

        import statistics
        var_no = statistics.variance(outputs_no[10:])
        var_strong = statistics.variance(outputs_strong[10:])
        self.assertLess(var_strong, var_no)


class TestPositionPIDIntegralSeparation(unittest.TestCase):
    """积分分离测试"""

    def test_integral_stops_when_error_large(self):
        """误差大于阈值时应暂停积分"""
        pid = PositionPIDSimulator(Kp=0.0, Ki=1.0, Kd=0.0,
                                    out_min=-10000, out_max=10000)
        pid.set_integral_separation(5.0)

        # 大误差 (>5) 不应积分
        pid.compute(setpoint=100.0, feedback=0.0)  # e=100 > 5
        self.assertAlmostEqual(pid.err_sum, 0.0, places=3)

    def test_integral_resumes_when_error_small(self):
        """误差小于阈值时应恢复积分"""
        pid = PositionPIDSimulator(Kp=0.0, Ki=1.0, Kd=0.0,
                                    out_min=-10000, out_max=10000)
        pid.set_integral_separation(50.0)

        # 小误差 (<50) 应积分
        pid.compute(setpoint=3.0, feedback=0.0)  # e=3 < 50
        self.assertGreater(pid.err_sum, 0)


class TestPositionPIDDerivativeOnFeedback(unittest.TestCase):
    """微分先行测试"""

    def test_d_on_feedback(self):
        """微分先行应对反馈微分"""
        pid = PositionPIDSimulator(Kp=0.0, Ki=0.0, Kd=1.0)
        pid.enable_derivative_on_feedback(True)

        pid.compute(setpoint=10.0, feedback=0.0)
        output = pid.compute(setpoint=10.0, feedback=5.0)
        # d = -Kd*(feedback - feedback_last) = -1.0*(5-0) = -5.0
        self.assertAlmostEqual(output, -5.0, places=1)

    def test_d_on_feedback_no_setpoint_jump(self):
        """微分先行应避免设定值跳变冲击"""
        pid_normal = PositionPIDSimulator(Kp=0.0, Ki=0.0, Kd=1.0)
        pid_fb = PositionPIDSimulator(Kp=0.0, Ki=0.0, Kd=1.0)
        pid_fb.enable_derivative_on_feedback(True)

        # 两者的设定值都跳变
        pid_normal.compute(setpoint=0.0, feedback=5.0)
        out_normal = pid_normal.compute(setpoint=10.0, feedback=5.0)

        pid_fb.compute(setpoint=0.0, feedback=5.0)
        out_fb = pid_fb.compute(setpoint=10.0, feedback=5.0)

        # 普通微分: err从-5变到5, d=Kd*(5-(-5))=10
        # 微分先行: d=-Kd*(5-5)=0 (反馈不变)
        self.assertGreater(abs(out_normal), abs(out_fb))


class TestPositionPIDDeadZone(unittest.TestCase):
    """死区测试"""

    def test_deadzone_inside(self):
        """死区内误差应被置零"""
        pid = PositionPIDSimulator(Kp=1.0, Ki=0.0, Kd=0.0)
        pid.set_dead_zone(5.0)
        output = pid.compute(setpoint=10.0, feedback=8.0)  # |e|=2 < 5
        self.assertAlmostEqual(output, 0.0, places=2)

    def test_deadzone_outside(self):
        """死区外应正常响应"""
        pid = PositionPIDSimulator(Kp=1.0, Ki=0.0, Kd=0.0)
        pid.set_dead_zone(5.0)
        output = pid.compute(setpoint=20.0, feedback=0.0)  # |e|=20 > 5
        self.assertAlmostEqual(output, 20.0, places=1)


class TestPositionPIDOutputClamp(unittest.TestCase):
    """输出限幅测试"""

    def test_output_clamped_high(self):
        pid = PositionPIDSimulator(Kp=100.0, Ki=0.0, Kd=0.0,
                                    out_min=-50.0, out_max=50.0)
        output = pid.compute(setpoint=100.0, feedback=0.0)
        self.assertLessEqual(output, 50.0)

    def test_output_clamped_low(self):
        pid = PositionPIDSimulator(Kp=100.0, Ki=0.0, Kd=0.0,
                                    out_min=-50.0, out_max=50.0)
        output = pid.compute(setpoint=-100.0, feedback=0.0)
        self.assertGreaterEqual(output, -50.0)


class TestPositionPIDReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        pid = PositionPIDSimulator(Kp=1.0, Ki=0.1, Kd=0.01)
        pid.compute(setpoint=10.0, feedback=0.0)
        pid.reset()
        self.assertEqual(pid.err, 0.0)
        self.assertEqual(pid.err_sum, 0.0)
        self.assertEqual(pid.d_filtered, 0.0)
        self.assertEqual(pid.output, 0.0)
        self.assertEqual(pid.feedback_last, 0.0)

    def test_reset_allows_fresh_start(self):
        pid = PositionPIDSimulator(Kp=1.0, Ki=0.0, Kd=0.0)
        pid.compute(setpoint=10.0, feedback=0.0)
        pid.reset()
        output = pid.compute(setpoint=5.0, feedback=0.0)
        self.assertAlmostEqual(output, 5.0, places=2)


class TestPositionPIDConvergence(unittest.TestCase):
    """收敛性测试"""

    def test_step_response_converges(self):
        """PID阶跃响应应趋近目标"""
        pid = PositionPIDSimulator(Kp=0.5, Ki=0.1, Kd=0.05,
                                    out_min=-1000, out_max=1000)
        pid.set_integral_limit(-500, 500)
        state = 0.0
        dt = 0.01
        for _ in range(3000):
            output = pid.compute(setpoint=100.0, feedback=state)
            state += output * dt * 0.5
        self.assertGreater(state, 50.0)

    def test_overshoot_limited(self):
        """适当参数下超调应有限"""
        pid = PositionPIDSimulator(Kp=1.0, Ki=0.5, Kd=0.2,
                                    out_min=-100, out_max=100)
        pid.set_integral_limit(-50, 50)
        state = 0.0
        dt = 0.01
        max_state = 0.0
        for _ in range(5000):
            output = pid.compute(setpoint=100.0, feedback=state)
            state += output * dt * 0.3
            max_state = max(max_state, state)
        # 超调不应超过目标的2倍
        self.assertLess(max_state, 200.0)


class TestPositionPIDDFilterBounds(unittest.TestCase):
    """滤波参数边界测试"""

    def test_alpha_clamped_low(self):
        pid = PositionPIDSimulator()
        pid.set_d_filter(-0.5)
        self.assertEqual(pid.d_filter_alpha, 0.0)

    def test_alpha_clamped_high(self):
        pid = PositionPIDSimulator()
        pid.set_d_filter(1.5)
        self.assertAlmostEqual(pid.d_filter_alpha, 0.99, places=2)

    def test_dead_zone_negative(self):
        pid = PositionPIDSimulator()
        pid.set_dead_zone(-5.0)
        self.assertEqual(pid.dead_zone, 0.0)

    def test_integral_sep_negative(self):
        pid = PositionPIDSimulator()
        pid.set_integral_separation(-10.0)
        self.assertEqual(pid.integral_sep_threshold, 0.0)


if __name__ == '__main__':
    unittest.main()
