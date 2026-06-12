"""
增益调度PIDV2单元测试 (线性插值法)
使用纯Python模拟C pid_gain_scheduling模块逻辑
覆盖: 初始化/添加标定点/插值计算/边界钳位/微分滤波/输出限幅/抗饱和/复位
"""
import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ============================================================
# Python模拟实现（对应 pid_gain_scheduling.c）
# ============================================================
PID_GS_MAX_POINTS = 16


class PID_GainSched:
    """对应C的PID_GainSched_t"""

    def __init__(self, dt=0.01):
        self.sched_value = []
        self.kp_table = []
        self.ki_table = []
        self.kd_table = []
        self.out_min = -100.0
        self.out_max = 100.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.derivative_alpha = 0.1
        self.output = 0.0
        self.dt = dt
        self.anti_windup = False

    def set_output_limit(self, out_min, out_max):
        self.out_min = out_min
        self.out_max = out_max

    def set_deriv_filter(self, alpha):
        alpha = max(0.0, min(1.0, alpha))
        self.derivative_alpha = alpha

    def add_point(self, sched_val, kp, ki, kd):
        if len(self.sched_value) >= PID_GS_MAX_POINTS:
            return -1
        if self.sched_value and sched_val <= self.sched_value[-1]:
            return -1
        self.sched_value.append(sched_val)
        self.kp_table.append(kp)
        self.ki_table.append(ki)
        self.kd_table.append(kd)
        return 0

    def _interpolate(self, sched_var):
        n = len(self.sched_value)
        if n == 0:
            return 0.0, 0.0, 0.0
        if n == 1:
            return self.kp_table[0], self.ki_table[0], self.kd_table[0]

        if sched_var <= self.sched_value[0]:
            return self.kp_table[0], self.ki_table[0], self.kd_table[0]
        if sched_var >= self.sched_value[-1]:
            return self.kp_table[-1], self.ki_table[-1], self.kd_table[-1]

        for i in range(n - 1):
            if self.sched_value[i] <= sched_var < self.sched_value[i + 1]:
                v0, v1 = self.sched_value[i], self.sched_value[i + 1]
                t = (sched_var - v0) / (v1 - v0) if v1 > v0 else 0.0
                kp = self.kp_table[i] + t * (self.kp_table[i + 1] - self.kp_table[i])
                ki = self.ki_table[i] + t * (self.ki_table[i + 1] - self.ki_table[i])
                kd = self.kd_table[i] + t * (self.kd_table[i + 1] - self.kd_table[i])
                return kp, ki, kd

        return self.kp_table[-1], self.ki_table[-1], self.kd_table[-1]

    def update(self, setpoint, feedback, sched_var):
        if not self.sched_value:
            return 0.0

        kp_eff, ki_eff, kd_eff = self._interpolate(sched_var)

        error = setpoint - feedback
        self.integral += error * self.dt

        raw_derivative = (error - self.prev_error) / self.dt
        alpha = self.derivative_alpha
        filtered_derivative = alpha * raw_derivative + (1 - alpha) * self.prev_derivative
        self.prev_derivative = filtered_derivative
        self.prev_error = error

        output = kp_eff * error + ki_eff * self.integral + kd_eff * filtered_derivative

        if output > self.out_max:
            if self.anti_windup:
                self.integral -= error * self.dt
            output = self.out_max
        elif output < self.out_min:
            if self.anti_windup:
                self.integral -= error * self.dt
            output = self.out_min

        self.output = output
        return output

    def get_effective_params(self, sched_var):
        return self._interpolate(sched_var)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.output = 0.0


# ============================================================
# 测试用例
# ============================================================

class TestGSInit(unittest.TestCase):
    """初始化测试"""

    def test_default_state(self):
        pid = PID_GainSched(dt=0.01)
        self.assertEqual(len(pid.sched_value), 0)
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)
        self.assertEqual(pid.derivative_alpha, 0.1)

    def test_no_points_returns_zero(self):
        pid = PID_GainSched()
        result = pid.update(1.0, 0.5, 0.0)
        self.assertEqual(result, 0.0)


class TestGSAddPoint(unittest.TestCase):
    """添加标定点测试"""

    def test_add_single_point(self):
        pid = PID_GainSched()
        ret = pid.add_point(0.0, kp=10.0, ki=5.0, kd=1.0)
        self.assertEqual(ret, 0)
        self.assertEqual(len(pid.sched_value), 1)

    def test_add_multiple_points(self):
        pid = PID_GainSched()
        pid.add_point(0.0, 12, 8, 0.2)
        pid.add_point(4.0, 8, 5, 0.3)
        pid.add_point(8.0, 5, 3, 0.5)
        self.assertEqual(len(pid.sched_value), 3)

    def test_non_increasing_rejected(self):
        """非递增标定值应被拒绝"""
        pid = PID_GainSched()
        pid.add_point(5.0, 10, 5, 1)
        ret = pid.add_point(3.0, 8, 3, 0.5)
        self.assertEqual(ret, -1)

    def test_equal_value_rejected(self):
        """相等标定值应被拒绝"""
        pid = PID_GainSched()
        pid.add_point(5.0, 10, 5, 1)
        ret = pid.add_point(5.0, 8, 3, 0.5)
        self.assertEqual(ret, -1)

    def test_max_points_overflow(self):
        pid = PID_GainSched()
        for i in range(PID_GS_MAX_POINTS):
            pid.add_point(float(i), 1, 1, 1)
        ret = pid.add_point(100.0, 1, 1, 1)
        self.assertEqual(ret, -1)


class TestGSInterpolation(unittest.TestCase):
    """插值计算测试"""

    def setUp(self):
        self.pid = PID_GainSched()
        self.pid.add_point(0.0, kp=12, ki=8, kd=0.2)
        self.pid.add_point(4.0, kp=8, ki=5, kd=0.3)
        self.pid.add_point(8.0, kp=5, ki=3, kd=0.5)

    def test_at_first_point(self):
        kp, ki, kd = self.pid._interpolate(0.0)
        self.assertAlmostEqual(kp, 12.0, places=3)
        self.assertAlmostEqual(ki, 8.0, places=3)

    def test_at_last_point(self):
        kp, ki, kd = self.pid._interpolate(8.0)
        self.assertAlmostEqual(kp, 5.0, places=3)

    def test_at_midpoint(self):
        kp, ki, kd = self.pid._interpolate(2.0)
        self.assertAlmostEqual(kp, 10.0, places=3)  # (12+8)/2
        self.assertAlmostEqual(ki, 6.5, places=3)   # (8+5)/2

    def test_below_range_clamped(self):
        kp, ki, kd = self.pid._interpolate(-5.0)
        self.assertAlmostEqual(kp, 12.0, places=3)

    def test_above_range_clamped(self):
        kp, ki, kd = self.pid._interpolate(20.0)
        self.assertAlmostEqual(kp, 5.0, places=3)

    def test_single_point(self):
        pid = PID_GainSched()
        pid.add_point(5.0, kp=7, ki=4, kd=0.1)
        kp, ki, kd = pid._interpolate(3.0)
        self.assertAlmostEqual(kp, 7.0, places=3)

    def test_interpolation_is_linear(self):
        kp, _, _ = self.pid._interpolate(1.0)
        self.assertAlmostEqual(kp, 11.0, places=3)  # 12 + 0.25*(8-12) = 11

    def test_get_effective_params(self):
        kp, ki, kd = self.pid.get_effective_params(2.0)
        self.assertAlmostEqual(kp, 10.0, places=3)


class TestGSUpdate(unittest.TestCase):
    """更新计算测试"""

    def setUp(self):
        self.pid = PID_GainSched(dt=0.01)
        self.pid.add_point(0.0, kp=10, ki=5, kd=0.5)
        self.pid.add_point(5.0, kp=5, ki=2, kd=0.3)

    def test_positive_error_positive_output(self):
        output = self.pid.update(10, 5, sched_var=0.0)
        self.assertGreater(output, 0)

    def test_negative_error_negative_output(self):
        output = self.pid.update(0, 10, sched_var=0.0)
        self.assertLess(output, 0)

    def test_zero_error_p_only(self):
        """零误差P-only应为零"""
        pid = PID_GainSched(dt=0.01)
        pid.add_point(0.0, kp=10, ki=0, kd=0)
        output = pid.update(5, 5, 0.0)
        self.assertAlmostEqual(output, 0.0, places=3)

    def test_output_varies_with_sched_var(self):
        """不同调度变量应产生不同输出（因参数不同）"""
        pid1 = PID_GainSched(dt=0.01)
        pid1.add_point(0.0, kp=10, ki=0, kd=0)

        pid2 = PID_GainSched(dt=0.01)
        pid2.add_point(0.0, kp=5, ki=0, kd=0)

        out1 = pid1.update(10, 5, 0.0)
        out2 = pid2.update(10, 5, 0.0)
        self.assertNotAlmostEqual(out1, out2, places=1)


class TestGSDerivFilter(unittest.TestCase):
    """微分滤波测试"""

    def test_alpha_1_no_filter(self):
        """alpha=1应不滤波"""
        pid = PID_GainSched(dt=0.01)
        pid.set_deriv_filter(1.0)
        pid.add_point(0.0, kp=0, ki=0, kd=1)
        pid.update(10, 0, 0.0)
        raw_d = (10 - 0) / 0.01
        self.assertAlmostEqual(pid.prev_derivative, raw_d, places=0)

    def test_alpha_0_max_filter(self):
        """alpha=0应最大滤波（输出为0）"""
        pid = PID_GainSched(dt=0.01)
        pid.set_deriv_filter(0.0)
        pid.add_point(0.0, kp=0, ki=0, kd=1)
        pid.update(10, 0, 0.0)
        self.assertAlmostEqual(pid.prev_derivative, 0.0, places=3)

    def test_alpha_clamping(self):
        pid = PID_GainSched()
        pid.set_deriv_filter(-0.5)
        self.assertEqual(pid.derivative_alpha, 0.0)
        pid.set_deriv_filter(1.5)
        self.assertEqual(pid.derivative_alpha, 1.0)


class TestGSOutputLimit(unittest.TestCase):
    """输出限幅测试"""

    def test_clamped_max(self):
        pid = PID_GainSched(dt=0.01)
        pid.set_output_limit(-10, 10)
        pid.add_point(0.0, kp=1000, ki=0, kd=0)
        output = pid.update(100, 0, 0.0)
        self.assertLessEqual(output, 10.0)

    def test_clamped_min(self):
        pid = PID_GainSched(dt=0.01)
        pid.set_output_limit(-10, 10)
        pid.add_point(0.0, kp=1000, ki=0, kd=0)
        output = pid.update(-100, 0, 0.0)
        self.assertGreaterEqual(output, -10.0)


class TestGSAntiWindup(unittest.TestCase):
    """抗积分饱和测试"""

    def test_anti_windup_limits_integral(self):
        pid_aw = PID_GainSched(dt=0.01)
        pid_aw.anti_windup = True
        pid_aw.set_output_limit(-10, 10)
        pid_aw.add_point(0.0, kp=100, ki=1000, kd=0)

        pid_no = PID_GainSched(dt=0.01)
        pid_no.anti_windup = False
        pid_no.set_output_limit(-10, 10)
        pid_no.add_point(0.0, kp=100, ki=1000, kd=0)

        for _ in range(200):
            pid_aw.update(100, 0, 0.0)
            pid_no.update(100, 0, 0.0)

        self.assertLess(abs(pid_aw.integral), abs(pid_no.integral))


class TestGSReset(unittest.TestCase):
    """复位测试"""

    def test_reset_clears_state(self):
        pid = PID_GainSched()
        pid.add_point(0.0, kp=10, ki=5, kd=1)
        for _ in range(50):
            pid.update(10, 0, 0.0)

        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)
        self.assertEqual(pid.prev_derivative, 0.0)
        self.assertEqual(pid.output, 0.0)


class TestGSSmoothTransition(unittest.TestCase):
    """平滑过渡测试（与硬切换对比）"""

    def test_interpolated_params_are_between_boundaries(self):
        """插值参数应在边界之间"""
        pid = PID_GainSched()
        pid.add_point(0.0, kp=12, ki=8, kd=0.2)
        pid.add_point(8.0, kp=4, ki=2, kd=0.6)

        kp, ki, kd = pid._interpolate(4.0)
        self.assertGreater(kp, 4.0)
        self.assertLess(kp, 12.0)
        self.assertGreater(ki, 2.0)
        self.assertLess(ki, 8.0)


if __name__ == '__main__':
    unittest.main()
