"""
增益调度PID单元测试 (硬切换)
使用纯Python模拟C pid_scheduled模块逻辑
覆盖: 初始化/添加区间/查找区间/更新计算/切换重置积分/输出限幅/抗饱和/复位
"""
import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ============================================================
# Python模拟实现（对应 pid_scheduled.c）
# ============================================================
PID_SCHEDULED_MAX_REGIONS = 8


class PID_Scheduled_Region:
    __slots__ = ['threshold', 'kp', 'ki', 'kd', 'out_min', 'out_max']

    def __init__(self, threshold=0.0, kp=0.0, ki=0.0, kd=0.0,
                 out_min=-100.0, out_max=100.0):
        self.threshold = threshold
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max


class PID_Scheduled:
    """对应C的PID_Scheduled_t"""

    def __init__(self, dt=0.01):
        self.regions = []
        self.active_region = 0
        self.integral = 0.0
        self.prev_error = 0.0
        self.output = 0.0
        self.dt = dt
        self.reset_on_switch = True
        self.anti_windup = False

    def add_region(self, threshold, kp, ki, kd, out_min=-100.0, out_max=100.0):
        if len(self.regions) >= PID_SCHEDULED_MAX_REGIONS:
            return -1
        self.regions.append(PID_Scheduled_Region(
            threshold, kp, ki, kd, out_min, out_max))
        self.regions.sort(key=lambda r: r.threshold)
        return 0

    def _find_region(self, sched_var):
        for i in range(len(self.regions) - 1, -1, -1):
            if sched_var >= self.regions[i].threshold:
                return i
        return 0

    def update(self, setpoint, feedback, sched_var):
        if not self.regions:
            return 0.0

        new_region = self._find_region(sched_var)
        if new_region != self.active_region:
            if self.reset_on_switch:
                self.integral = 0.0
            self.active_region = new_region

        p = self.regions[self.active_region]
        error = setpoint - feedback
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        output = p.kp * error + p.ki * self.integral + p.kd * derivative

        if output > p.out_max:
            output = p.out_max
            if self.anti_windup:
                self.integral -= error * self.dt
        elif output < p.out_min:
            output = p.out_min
            if self.anti_windup:
                self.integral -= error * self.dt

        self.output = output
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.output = 0.0
        self.active_region = 0


# ============================================================
# 测试用例
# ============================================================

class TestScheduledPIDInit(unittest.TestCase):
    """初始化测试"""

    def test_default_state(self):
        pid = PID_Scheduled(dt=0.01)
        self.assertEqual(len(pid.regions), 0)
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)
        self.assertEqual(pid.output, 0.0)
        self.assertEqual(pid.dt, 0.01)
        self.assertTrue(pid.reset_on_switch)

    def test_no_regions_returns_zero(self):
        pid = PID_Scheduled()
        result = pid.update(1.0, 0.5, 0.0)
        self.assertEqual(result, 0.0)


class TestScheduledPIDAddRegion(unittest.TestCase):
    """添加区间测试"""

    def test_add_single_region(self):
        pid = PID_Scheduled()
        ret = pid.add_region(0.0, kp=10.0, ki=5.0, kd=1.0)
        self.assertEqual(ret, 0)
        self.assertEqual(len(pid.regions), 1)

    def test_add_multiple_regions(self):
        pid = PID_Scheduled()
        pid.add_region(0.0, 10, 5, 1)
        pid.add_region(5.0, 6, 3, 0.5)
        pid.add_region(10.0, 3, 1, 0.2)
        self.assertEqual(len(pid.regions), 3)

    def test_max_regions_overflow(self):
        pid = PID_Scheduled()
        for i in range(PID_SCHEDULED_MAX_REGIONS):
            pid.add_region(float(i), 1, 1, 1)
        ret = pid.add_region(100.0, 1, 1, 1)
        self.assertEqual(ret, -1)

    def test_regions_sorted_by_threshold(self):
        pid = PID_Scheduled()
        pid.add_region(10.0, 3, 1, 0.2)
        pid.add_region(0.0, 10, 5, 1)
        pid.add_region(5.0, 6, 3, 0.5)
        thresholds = [r.threshold for r in pid.regions]
        self.assertEqual(thresholds, sorted(thresholds))


class TestScheduledPIDFindRegion(unittest.TestCase):
    """查找区间测试"""

    def setUp(self):
        self.pid = PID_Scheduled()
        self.pid.add_region(0.0, kp=12, ki=8, kd=0.2)
        self.pid.add_region(3.0, kp=8, ki=5, kd=0.3)
        self.pid.add_region(6.0, kp=5, ki=3, kd=0.5)

    def test_below_all_thresholds(self):
        self.assertEqual(self.pid._find_region(-1.0), 0)

    def test_at_threshold(self):
        self.assertEqual(self.pid._find_region(3.0), 1)

    def test_between_thresholds(self):
        self.assertEqual(self.pid._find_region(4.5), 1)

    def test_above_all_thresholds(self):
        self.assertEqual(self.pid._find_region(10.0), 2)


class TestScheduledPIDUpdate(unittest.TestCase):
    """更新计算测试"""

    def setUp(self):
        self.pid = PID_Scheduled(dt=0.01)
        self.pid.add_region(0.0, kp=10, ki=5, kd=0.5, out_min=-100, out_max=100)
        self.pid.add_region(5.0, kp=5, ki=2, kd=0.3, out_min=-100, out_max=100)

    def test_positive_error_positive_output(self):
        """正误差应产生正输出"""
        output = self.pid.update(setpoint=10.0, feedback=5.0, sched_var=0.0)
        self.assertGreater(output, 0)

    def test_negative_error_negative_output(self):
        """负误差应产生负输出"""
        output = self.pid.update(setpoint=0.0, feedback=10.0, sched_var=0.0)
        self.assertLess(output, 0)

    def test_output_changes_with_region(self):
        """不同区间应使用不同参数"""
        pid1 = PID_Scheduled(dt=0.01)
        pid1.add_region(0.0, kp=10, ki=0, kd=0)
        out1 = pid1.update(10, 5, 0.0)

        pid2 = PID_Scheduled(dt=0.01)
        pid2.add_region(0.0, kp=5, ki=0, kd=0)
        out2 = pid2.update(10, 5, 0.0)

        self.assertNotAlmostEqual(out1, out2, places=1)

    def test_zero_error_zero_output_p_only(self):
        """零误差P-only应为零输出"""
        pid = PID_Scheduled(dt=0.01)
        pid.add_region(0.0, kp=10, ki=0, kd=0)
        # 首次调用prev_error=0, error=0 -> 输出=0
        output = pid.update(setpoint=5.0, feedback=5.0, sched_var=0.0)
        self.assertAlmostEqual(output, 0.0, places=3)


class TestScheduledPIDResetOnSwitch(unittest.TestCase):
    """切换区间时积分重置测试"""

    def test_reset_on_switch(self):
        pid = PID_Scheduled(dt=0.01)
        pid.reset_on_switch = True
        pid.add_region(0.0, kp=10, ki=5, kd=0)
        pid.add_region(5.0, kp=5, ki=2, kd=0)

        # 在区间0积累积分
        for _ in range(100):
            pid.update(10, 0, sched_var=0.0)
        self.assertNotEqual(pid.integral, 0.0)

        # 切换到区间1，积分应重置
        pid.update(10, 0, sched_var=6.0)
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.active_region, 1)

    def test_no_reset_on_switch(self):
        pid = PID_Scheduled(dt=0.01)
        pid.reset_on_switch = False
        pid.add_region(0.0, kp=10, ki=5, kd=0)
        pid.add_region(5.0, kp=5, ki=2, kd=0)

        for _ in range(100):
            pid.update(10, 0, sched_var=0.0)
        integral_before = pid.integral

        pid.update(10, 0, sched_var=6.0)
        # 积分应保留（加上本轮误差*dt）
        self.assertNotEqual(pid.integral, 0.0)


class TestScheduledPIDOutputLimit(unittest.TestCase):
    """输出限幅测试"""

    def test_output_clamped_max(self):
        pid = PID_Scheduled(dt=0.01)
        pid.add_region(0.0, kp=1000, ki=0, kd=0, out_min=-10, out_max=10)
        output = pid.update(100, 0, 0.0)
        self.assertLessEqual(output, 10.0)

    def test_output_clamped_min(self):
        pid = PID_Scheduled(dt=0.01)
        pid.add_region(0.0, kp=1000, ki=0, kd=0, out_min=-10, out_max=10)
        output = pid.update(-100, 0, 0.0)
        self.assertGreaterEqual(output, -10.0)


class TestScheduledPIDAntiWindup(unittest.TestCase):
    """抗积分饱和测试"""

    def test_anti_windup_limits_integral(self):
        pid = PID_Scheduled(dt=0.01)
        pid.anti_windup = True
        pid.add_region(0.0, kp=100, ki=1000, kd=0, out_min=-10, out_max=10)

        for _ in range(200):
            pid.update(100, 0, 0.0)

        # 积分不应无限增长（被回退）
        pid_no_aw = PID_Scheduled(dt=0.01)
        pid_no_aw.anti_windup = False
        pid_no_aw.add_region(0.0, kp=100, ki=1000, kd=0, out_min=-10, out_max=10)

        for _ in range(200):
            pid_no_aw.update(100, 0, 0.0)

        # 有抗饱和的积分应小于无抗饱和的
        self.assertLess(abs(pid.integral), abs(pid_no_aw.integral))


class TestScheduledPIDReset(unittest.TestCase):
    """复位测试"""

    def test_reset_clears_state(self):
        pid = PID_Scheduled()
        pid.add_region(0.0, kp=10, ki=5, kd=1)
        for _ in range(50):
            pid.update(10, 0, 0.0)

        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)
        self.assertEqual(pid.output, 0.0)
        self.assertEqual(pid.active_region, 0)


if __name__ == '__main__':
    unittest.main()
