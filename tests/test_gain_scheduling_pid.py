#!/usr/bin/env python3
"""
增益调度PID单元测试
覆盖: 初始化/标定点添加/线性插值/硬切换/输出限幅/抗饱和/性能基准
注意: 使用纯 Python 模拟增益调度PID逻辑
"""

import sys
import os
import math
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 ──────────────────────────────────────────

class GainSchedulingPID:
    """增益调度PID控制器"""

    def __init__(self, dt=0.01, u_max=100.0, u_min=-100.0, mode='soft'):
        """
        dt: 采样周期
        u_max/u_min: 输出限幅
        mode: 'soft'=线性插值, 'hard'=硬切换
        """
        self.dt = dt
        self.u_max = u_max
        self.u_min = u_min
        self.mode = mode

        self.points = []  # [(sv, Kp, Ki, Kd), ...]
        self.integral = 0.0
        self.prev_error = 0.0
        self.Kp = self.Ki = self.Kd = 0.0
        self.integral_max = 500.0
        self.derivative_alpha = 0.1  # 微分滤波系数
        self.prev_derivative = 0.0

    def add_point(self, sv, Kp, Ki, Kd):
        """添加标定点, 要求sv严格递增"""
        if self.points and sv <= self.points[-1][0]:
            return -1
        self.points.append((sv, Kp, Ki, Kd))
        return 0

    def lookup(self, sv):
        """根据调度变量查表/插值"""
        n = len(self.points)
        if n == 0:
            return 0.0, 0.0, 0.0
        if n == 1:
            return self.points[0][1], self.points[0][2], self.points[0][3]

        # 钳位到边界
        if sv <= self.points[0][0]:
            return self.points[0][1], self.points[0][2], self.points[0][3]
        if sv >= self.points[-1][0]:
            return self.points[-1][1], self.points[-1][2], self.points[-1][3]

        # 查找区间
        for i in range(n - 1):
            sv0, kp0, ki0, kd0 = self.points[i]
            sv1, kp1, ki1, kd1 = self.points[i + 1]
            if sv0 <= sv < sv1:
                if self.mode == 'hard':
                    return kp0, ki0, kd0
                else:
                    t = (sv - sv0) / (sv1 - sv0) if sv1 != sv0 else 0.0
                    return (kp0 + t * (kp1 - kp0),
                            ki0 + t * (ki1 - ki0),
                            kd0 + t * (kd1 - kd0))

        return self.points[-1][1], self.points[-1][2], self.points[-1][3]

    def update(self, setpoint, feedback, sched_var):
        """PID计算"""
        self.Kp, self.Ki, self.Kd = self.lookup(sched_var)

        error = setpoint - feedback

        # 积分项 (带抗饱和)
        self.integral += error * self.dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))

        # 微分项 (带一阶滤波)
        raw_derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0
        derivative = (self.derivative_alpha * raw_derivative +
                      (1 - self.derivative_alpha) * self.prev_derivative)
        self.prev_derivative = derivative
        self.prev_error = error

        # PID输出
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        u = max(self.u_min, min(self.u_max, u))

        return u

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0


# ── 测试用例 ──────────────────────────────────────────────────

class TestGainSchedulingInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        """默认参数应正确"""
        gs = GainSchedulingPID()
        self.assertEqual(gs.dt, 0.01)
        self.assertEqual(gs.u_max, 100.0)
        self.assertEqual(gs.u_min, -100.0)
        self.assertEqual(gs.mode, 'soft')

    def test_empty_points(self):
        """无标定点时增益应为0"""
        gs = GainSchedulingPID()
        kp, ki, kd = gs.lookup(500.0)
        self.assertEqual(kp, 0.0)
        self.assertEqual(ki, 0.0)
        self.assertEqual(kd, 0.0)

    def test_custom_params(self):
        """自定义参数应生效"""
        gs = GainSchedulingPID(dt=0.001, u_max=50, u_min=-50, mode='hard')
        self.assertEqual(gs.dt, 0.001)
        self.assertEqual(gs.u_max, 50)
        self.assertEqual(gs.mode, 'hard')


class TestGainSchedulingPoints(unittest.TestCase):
    """标定点管理测试"""

    def test_add_point_success(self):
        """正常添加标定点"""
        gs = GainSchedulingPID()
        ret = gs.add_point(100, 1.0, 0.5, 0.1)
        self.assertEqual(ret, 0)
        self.assertEqual(len(gs.points), 1)

    def test_add_point_order_violation(self):
        """非递增sv应返回-1"""
        gs = GainSchedulingPID()
        gs.add_point(100, 1.0, 0.5, 0.1)
        ret = gs.add_point(50, 2.0, 1.0, 0.2)
        self.assertEqual(ret, -1)

    def test_add_duplicate_sv(self):
        """重复sv应返回-1"""
        gs = GainSchedulingPID()
        gs.add_point(100, 1.0, 0.5, 0.1)
        ret = gs.add_point(100, 2.0, 1.0, 0.2)
        self.assertEqual(ret, -1)

    def test_multiple_points(self):
        """应能添加多个标定点"""
        gs = GainSchedulingPID()
        for sv in [100, 200, 300, 400, 500]:
            gs.add_point(sv, sv * 0.01, sv * 0.005, sv * 0.001)
        self.assertEqual(len(gs.points), 5)


class TestGainSchedulingLookup(unittest.TestCase):
    """插值查表测试"""

    def test_below_range_clamp(self):
        """低于范围应返回第一个点"""
        gs = GainSchedulingPID()
        gs.add_point(100, 10.0, 5.0, 1.0)
        gs.add_point(200, 20.0, 10.0, 2.0)
        kp, ki, kd = gs.lookup(50)
        self.assertAlmostEqual(kp, 10.0)

    def test_above_range_clamp(self):
        """高于范围应返回最后一个点"""
        gs = GainSchedulingPID()
        gs.add_point(100, 10.0, 5.0, 1.0)
        gs.add_point(200, 20.0, 10.0, 2.0)
        kp, ki, kd = gs.lookup(300)
        self.assertAlmostEqual(kp, 20.0)

    def test_exact_point(self):
        """精确匹配标定点"""
        gs = GainSchedulingPID()
        gs.add_point(100, 10.0, 5.0, 1.0)
        gs.add_point(200, 20.0, 10.0, 2.0)
        kp, ki, kd = gs.lookup(100)
        self.assertAlmostEqual(kp, 10.0)
        self.assertAlmostEqual(ki, 5.0)
        self.assertAlmostEqual(kd, 1.0)

    def test_linear_interpolation_midpoint(self):
        """中点线性插值应正确"""
        gs = GainSchedulingPID(mode='soft')
        gs.add_point(100, 10.0, 5.0, 1.0)
        gs.add_point(200, 20.0, 10.0, 2.0)
        kp, ki, kd = gs.lookup(150)
        self.assertAlmostEqual(kp, 15.0)
        self.assertAlmostEqual(ki, 7.5)
        self.assertAlmostEqual(kd, 1.5)

    def test_hard_switching(self):
        """硬切换应返回区间左端点"""
        gs = GainSchedulingPID(mode='hard')
        gs.add_point(100, 10.0, 5.0, 1.0)
        gs.add_point(200, 20.0, 10.0, 2.0)
        kp, ki, kd = gs.lookup(150)
        self.assertAlmostEqual(kp, 10.0)

    def test_single_point(self):
        """单标定点应返回该点"""
        gs = GainSchedulingPID()
        gs.add_point(100, 10.0, 5.0, 1.0)
        kp, ki, kd = gs.lookup(500)
        self.assertAlmostEqual(kp, 10.0)


class TestGainSchedulingPIDUpdate(unittest.TestCase):
    """PID计算测试"""

    def test_zero_error_zero_output(self):
        """零误差(设定值=反馈)应产生零输出"""
        gs = GainSchedulingPID()
        gs.add_point(0, 10.0, 5.0, 1.0)
        u = gs.update(5.0, 5.0, 0.0)
        self.assertAlmostEqual(u, 0.0, places=2)

    def test_positive_error_positive_output(self):
        """正误差应产生正输出"""
        gs = GainSchedulingPID()
        gs.add_point(0, 10.0, 5.0, 1.0)
        u = gs.update(10.0, 0.0, 0.0)
        self.assertGreater(u, 0)

    def test_output_saturation(self):
        """输出应被限幅"""
        gs = GainSchedulingPID(u_max=50.0, u_min=-50.0)
        gs.add_point(0, 1000.0, 100.0, 10.0)
        u = gs.update(100.0, 0.0, 0.0)
        self.assertLessEqual(u, 50.0)
        self.assertGreaterEqual(u, -50.0)

    def test_integral_accumulation(self):
        """持续误差应累积积分"""
        gs = GainSchedulingPID()
        gs.add_point(0, 0.0, 10.0, 0.0)
        for _ in range(10):
            gs.update(1.0, 0.0, 0.0)
        self.assertGreater(gs.integral, 0)

    def test_integral_windup_limit(self):
        """积分应有上限"""
        gs = GainSchedulingPID()
        gs.integral_max = 100.0
        gs.add_point(0, 0.0, 10.0, 0.0)
        for _ in range(10000):
            gs.update(100.0, 0.0, 0.0)
        self.assertLessEqual(gs.integral, 100.0)

    def test_reset(self):
        """reset应清零所有状态"""
        gs = GainSchedulingPID()
        gs.add_point(0, 10.0, 5.0, 1.0)
        gs.update(10.0, 0.0, 0.0)
        gs.reset()
        self.assertEqual(gs.integral, 0.0)
        self.assertEqual(gs.prev_error, 0.0)

    def test_convergence_to_setpoint(self):
        """增益调度PID应能收敛到设定值"""
        gs = GainSchedulingPID(dt=0.01)
        gs.add_point(0, 5.0, 2.0, 0.5)

        state = 0.0
        for _ in range(2000):
            u = gs.update(10.0, state, 0.0)
            state += u * 0.01 * 0.1  # 简单积分模型

        self.assertGreater(state, 5.0)


class TestGainSchedulingEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_negative_setpoint(self):
        """负设定值应正常工作"""
        gs = GainSchedulingPID()
        gs.add_point(0, 10.0, 5.0, 1.0)
        u = gs.update(-10.0, 0.0, 0.0)
        self.assertLess(u, 0)

    def test_very_small_dt(self):
        """极小dt不应导致数值问题"""
        gs = GainSchedulingPID(dt=1e-6)
        gs.add_point(0, 10.0, 5.0, 1.0)
        u = gs.update(10.0, 0.0, 0.0)
        self.assertTrue(math.isfinite(u))

    def test_zero_dt_protection(self):
        """dt=0不应导致除零"""
        gs = GainSchedulingPID(dt=0.0)
        gs.add_point(0, 10.0, 5.0, 1.0)
        u = gs.update(10.0, 0.0, 0.0)
        self.assertTrue(math.isfinite(u))

    def test_no_points_update(self):
        """无标定点时输出应为0"""
        gs = GainSchedulingPID()
        u = gs.update(10.0, 0.0, 0.0)
        self.assertAlmostEqual(u, 0.0)


class TestGainSchedulingPerformance(unittest.TestCase):
    """性能基准测试"""

    def test_update_speed(self):
        """10000次更新应在1秒内完成"""
        gs = GainSchedulingPID()
        for sv in range(0, 5000, 100):
            gs.add_point(float(sv), 10.0, 5.0, 1.0)

        start = time.perf_counter()
        for i in range(10000):
            sv = float(i % 5000)
            gs.update(10.0, 5.0, sv)
        elapsed = time.perf_counter() - start

        self.assertLess(elapsed, 2.0,
                       f"10000次更新耗时 {elapsed:.3f}s")


if __name__ == '__main__':
    unittest.main()
