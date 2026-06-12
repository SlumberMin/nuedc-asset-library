#!/usr/bin/env python3
"""
速度PID(增量式PID)单元测试
覆盖: 初始化、增量计算、输出限幅、积分限幅、微分滤波、死区、重置
注意: 使用纯 Python 模拟 C VelocityPID 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class VelocityPIDSimulator:
    """速度式PID控制器模拟 (对应 velocity_pid.c)"""

    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, out_min=-1000.0, out_max=1000.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.out_min = out_min
        self.out_max = out_max

        self.err = [0.0, 0.0, 0.0]  # [当前, 上一次, 上上次]
        self.integral = 0.0
        self.integral_min = -1e30
        self.integral_max = 1e30
        self.d_filter_alpha = 0.0
        self.d_filtered = 0.0
        self.dead_zone = 0.0
        self.output = 0.0

    def set_integral_limit(self, min_val, max_val):
        self.integral_min = min_val
        self.integral_max = max_val

    def set_d_filter(self, alpha):
        self.d_filter_alpha = max(0.0, min(0.99, alpha))

    def set_dead_zone(self, dz):
        self.dead_zone = max(0.0, dz)

    def compute(self, setpoint, feedback):
        error = setpoint - feedback

        # 死区
        if -self.dead_zone < error < self.dead_zone:
            error = 0.0

        # 更新误差历史
        self.err[2] = self.err[1]
        self.err[1] = self.err[0]
        self.err[0] = error

        # 增量计算
        delta_p = self.Kp * (self.err[0] - self.err[1])
        delta_i = self.Ki * self.err[0]
        delta_d_raw = self.Kd * (self.err[0] - 2.0 * self.err[1] + self.err[2])

        # 微分滤波
        self.d_filtered = (self.d_filter_alpha * self.d_filtered
                           + (1.0 - self.d_filter_alpha) * delta_d_raw)
        delta_d = self.d_filtered

        # 积分累积并限幅
        self.integral += delta_i
        self.integral = max(self.integral_min, min(self.integral_max, self.integral))

        delta_u = delta_p + delta_i + delta_d

        # 输出累积
        self.output += delta_u

        # 输出限幅
        self.output = max(self.out_min, min(self.out_max, self.output))

        return self.output

    def reset(self):
        self.err = [0.0, 0.0, 0.0]
        self.integral = 0.0
        self.d_filtered = 0.0
        self.output = 0.0


class TestVelocityPIDInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        pid = VelocityPIDSimulator()
        self.assertEqual(pid.Kp, 1.0)
        self.assertEqual(pid.Ki, 0.0)
        self.assertEqual(pid.Kd, 0.0)

    def test_custom_params(self):
        pid = VelocityPIDSimulator(Kp=2.5, Ki=0.3, Kd=0.1)
        self.assertEqual(pid.Kp, 2.5)
        self.assertEqual(pid.Ki, 0.3)
        self.assertEqual(pid.Kd, 0.1)

    def test_output_limits(self):
        pid = VelocityPIDSimulator(out_min=-500, out_max=500)
        self.assertEqual(pid.out_min, -500)
        self.assertEqual(pid.out_max, 500)


class TestVelocityPIDProportional(unittest.TestCase):
    """比例项测试"""

    def test_p_only_first_step(self):
        """纯P控制第一步: output = Kp * error (因为e[-1]=0)"""
        pid = VelocityPIDSimulator(Kp=2.0, Ki=0.0, Kd=0.0)
        output = pid.compute(setpoint=10.0, feedback=5.0)
        # 第一步: delta = Kp*(e0-0) = 2.0*5.0 = 10.0
        self.assertAlmostEqual(output, 10.0, places=2)

    def test_p_only_steady_error(self):
        """纯P控制有稳态误差"""
        pid = VelocityPIDSimulator(Kp=1.0, Ki=0.0, Kd=0.0,
                                    out_min=-1000, out_max=1000)
        # 模拟一阶系统
        state = 0.0
        for _ in range(500):
            output = pid.compute(setpoint=100.0, feedback=state)
            state += output * 0.01
        # 纯P有稳态误差，不应精确到100
        self.assertLess(state, 100.0)

    def test_negative_error(self):
        """负误差应产生负增量"""
        pid = VelocityPIDSimulator(Kp=1.0, Ki=0.0, Kd=0.0)
        output = pid.compute(setpoint=0.0, feedback=10.0)
        self.assertLess(output, 0)


class TestVelocityPIDIntegral(unittest.TestCase):
    """积分项测试"""

    def test_integral_accumulation(self):
        """积分应使输出持续增长"""
        pid = VelocityPIDSimulator(Kp=0.0, Ki=1.0, Kd=0.0,
                                    out_min=-10000, out_max=10000)
        outputs = []
        for _ in range(10):
            out = pid.compute(setpoint=10.0, feedback=0.0)
            outputs.append(out)
        # 输出应递增
        for i in range(1, len(outputs)):
            self.assertGreater(outputs[i], outputs[i - 1])

    def test_integral_clamping(self):
        """积分项应被限幅"""
        pid = VelocityPIDSimulator(Kp=0.0, Ki=10.0, Kd=0.0,
                                    out_min=-10000, out_max=10000)
        pid.set_integral_limit(-5.0, 5.0)
        for _ in range(1000):
            pid.compute(setpoint=100.0, feedback=0.0)
        self.assertLessEqual(pid.integral, 5.0)
        self.assertGreaterEqual(pid.integral, -5.0)


class TestVelocityPIDDerivative(unittest.TestCase):
    """微分项测试"""

    def test_derivative_on_step_change(self):
        """阶跃变化应产生微分脉冲"""
        pid = VelocityPIDSimulator(Kp=0.0, Ki=0.0, Kd=1.0)
        pid.compute(setpoint=0.0, feedback=0.0)
        output = pid.compute(setpoint=10.0, feedback=0.0)
        # e从0跳到10, d项 = Kd*(10 - 2*0 + 0) = 10
        self.assertGreater(abs(output), 0)

    def test_d_filter_reduces_noise(self):
        """微分滤波应减少噪声"""
        # 无滤波
        pid_no = VelocityPIDSimulator(Kp=0.0, Ki=0.0, Kd=1.0)
        pid_no.set_d_filter(0.0)

        # 强滤波
        pid_strong = VelocityPIDSimulator(Kp=0.0, Ki=0.0, Kd=1.0)
        pid_strong.set_d_filter(0.9)

        # 含噪声信号
        outputs_no = []
        outputs_strong = []
        for i in range(100):
            error = 10.0 * math.sin(i * 0.1) + 2.0 * math.sin(i * 5.0)
            outputs_no.append(pid_no.compute(setpoint=error + 50, feedback=50))
            outputs_strong.append(pid_strong.compute(setpoint=error + 50, feedback=50))

        # 强滤波的输出方差应更小
        import statistics
        var_no = statistics.variance(outputs_no[10:])
        var_strong = statistics.variance(outputs_strong[10:])
        self.assertLess(var_strong, var_no)


class TestVelocityPIDDeadZone(unittest.TestCase):
    """死区测试"""

    def test_deadzone_zero_error(self):
        """死区内误差应被置零"""
        pid = VelocityPIDSimulator(Kp=1.0, Ki=0.0, Kd=0.0)
        pid.set_dead_zone(5.0)
        output = pid.compute(setpoint=10.0, feedback=8.0)  # |e|=2 < 5
        # 误差为0, 增量为0, 输出为0
        self.assertAlmostEqual(output, 0.0, places=2)

    def test_deadzone_outside(self):
        """死区外应正常响应"""
        pid = VelocityPIDSimulator(Kp=1.0, Ki=0.0, Kd=0.0)
        pid.set_dead_zone(5.0)
        output = pid.compute(setpoint=20.0, feedback=0.0)  # |e|=20 > 5
        self.assertNotAlmostEqual(output, 0.0, places=1)


class TestVelocityPIDOutputClamp(unittest.TestCase):
    """输出限幅测试"""

    def test_output_clamped_high(self):
        """输出不应超过上限"""
        pid = VelocityPIDSimulator(Kp=100.0, Ki=0.0, Kd=0.0,
                                    out_min=-50.0, out_max=50.0)
        output = pid.compute(setpoint=100.0, feedback=0.0)
        self.assertLessEqual(output, 50.0)

    def test_output_clamped_low(self):
        """输出不应低于下限"""
        pid = VelocityPIDSimulator(Kp=100.0, Ki=0.0, Kd=0.0,
                                    out_min=-50.0, out_max=50.0)
        output = pid.compute(setpoint=-100.0, feedback=0.0)
        self.assertGreaterEqual(output, -50.0)


class TestVelocityPIDReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        """reset应清零所有状态"""
        pid = VelocityPIDSimulator(Kp=1.0, Ki=0.1, Kd=0.01)
        pid.compute(setpoint=10.0, feedback=0.0)
        pid.reset()
        self.assertEqual(pid.err, [0.0, 0.0, 0.0])
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.d_filtered, 0.0)
        self.assertEqual(pid.output, 0.0)

    def test_reset_allows_fresh_start(self):
        """重置后应能重新开始"""
        pid = VelocityPIDSimulator(Kp=1.0, Ki=0.0, Kd=0.0)
        pid.compute(setpoint=10.0, feedback=0.0)
        pid.reset()
        output = pid.compute(setpoint=5.0, feedback=0.0)
        self.assertAlmostEqual(output, 5.0, places=2)


class TestVelocityPIDConvergence(unittest.TestCase):
    """收敛性测试"""

    def test_step_response_converges(self):
        """阶跃响应应趋近目标"""
        pid = VelocityPIDSimulator(Kp=0.5, Ki=0.5, Kd=0.05,
                                    out_min=-1000, out_max=1000)
        pid.set_integral_limit(-500, 500)
        state = 0.0
        dt = 0.01
        for _ in range(2000):
            output = pid.compute(setpoint=100.0, feedback=state)
            state += output * dt * 0.5
        self.assertGreater(state, 80.0)


class TestVelocityPIDDFilterBounds(unittest.TestCase):
    """微分滤波边界测试"""

    def test_alpha_clamped_low(self):
        """alpha<0应被限制为0"""
        pid = VelocityPIDSimulator()
        pid.set_d_filter(-0.5)
        self.assertEqual(pid.d_filter_alpha, 0.0)

    def test_alpha_clamped_high(self):
        """alpha>0.99应被限制为0.99"""
        pid = VelocityPIDSimulator()
        pid.set_d_filter(1.5)
        self.assertAlmostEqual(pid.d_filter_alpha, 0.99, places=2)

    def test_dead_zone_negative(self):
        """负死区应被限制为0"""
        pid = VelocityPIDSimulator()
        pid.set_dead_zone(-5.0)
        self.assertEqual(pid.dead_zone, 0.0)


if __name__ == '__main__':
    unittest.main()
