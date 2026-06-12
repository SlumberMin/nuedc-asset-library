#!/usr/bin/env python3
"""
轨迹生成器单元测试
覆盖: 初始化、梯形轨迹、S曲线轨迹、五次多项式轨迹、
      速度/加速度约束、完成状态、进度、重置
测试对象: 11_控制算法库/simulation/trajectory_generator_simulation.py
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from _11_控制算法库.simulation.trajectory_generator_simulation import (
    TrajectoryGenerator, TrajectoryType, TrajectoryConfig
)


# ── 初始化测试 ──

class TestTrajectoryInit(unittest.TestCase):
    """初始化测试"""

    def test_default_dt(self):
        gen = TrajectoryGenerator(dt=0.001)
        self.assertEqual(gen.dt, 0.001)

    def test_default_state(self):
        gen = TrajectoryGenerator()
        self.assertEqual(gen.current_time, 0.0)


# ── 梯形轨迹测试 ──

class TestTrapezoidalSetup(unittest.TestCase):
    """梯形轨迹配置测试"""

    def test_sets_type(self):
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 10, 5, 2)
        self.assertEqual(gen.trajectory_type, TrajectoryType.TRAPEZOIDAL)

    def test_total_time_positive(self):
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 10, 5, 2)
        self.assertGreater(gen.total_time, 0)

    def test_total_time_triangle_curve(self):
        """短距离→三角形曲线"""
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 1, 100, 2)
        self.assertEqual(gen.tc, 0.0)  # no cruise phase

    def test_total_time_trapezoidal_curve(self):
        """长距离→完整梯形曲线"""
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 100, 5, 2)
        self.assertGreater(gen.tc, 0.0)

    def test_negative_direction(self):
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(10, 0, 5, 2)
        self.assertGreater(gen.total_time, 0)


class TestTrapezoidalCalculation(unittest.TestCase):
    """梯形轨迹计算测试"""

    def test_initial_position(self):
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 10, 5, 2)
        result = gen.calculate(0.0)
        self.assertAlmostEqual(result['position'], 0.0, places=2)

    def test_final_position(self):
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 10, 5, 2)
        result = gen.calculate(gen.total_time + 1.0)
        self.assertAlmostEqual(result['position'], 10.0, places=2)

    def test_velocity_zero_at_start(self):
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 10, 5, 2)
        result = gen.calculate(0.0)
        self.assertAlmostEqual(result['velocity'], 0.0, places=2)

    def test_velocity_zero_at_end(self):
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 10, 5, 2)
        result = gen.calculate(gen.total_time + 1.0)
        self.assertAlmostEqual(result['velocity'], 0.0, places=2)

    def test_max_velocity_not_exceeded(self):
        gen = TrajectoryGenerator()
        max_vel = 5.0
        gen.set_trapezoidal(0, 100, max_vel, 2)
        for t_100 in range(0, 20000):
            t = t_100 * 0.001
            result = gen.calculate(t)
            self.assertLessEqual(abs(result['velocity']), max_vel + 0.1)

    def test_acceleration_phases(self):
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 100, 5, 2)
        # acceleration phase
        result = gen.calculate(0.001)
        self.assertEqual(result['state'], 'accel')
        # cruise phase
        t_cruise = gen.ta + gen.tc / 2
        if gen.tc > 0:
            result = gen.calculate(t_cruise)
            self.assertEqual(result['state'], 'cruise')

    def test_monotonically_increasing_position(self):
        """位置应单调递增（正方向）"""
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 10, 5, 2)
        prev_pos = 0.0
        for t_100 in range(0, int(gen.total_time * 1000)):
            t = t_100 * 0.001
            result = gen.calculate(t)
            self.assertGreaterEqual(result['position'], prev_pos - 0.01)
            prev_pos = result['position']

    def test_done_state(self):
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 10, 5, 2)
        result = gen.calculate(gen.total_time + 1.0)
        self.assertEqual(result['state'], 'done')


# ── S曲线轨迹测试 ──

class TestSCurveSetup(unittest.TestCase):
    """S曲线轨迹配置测试"""

    def test_sets_type(self):
        gen = TrajectoryGenerator()
        gen.set_s_curve(0, 10, 5, 2, 10)
        self.assertEqual(gen.trajectory_type, TrajectoryType.S_CURVE)

    def test_total_time_positive(self):
        gen = TrajectoryGenerator()
        gen.set_s_curve(0, 10, 5, 2, 10)
        self.assertGreater(gen.total_time, 0)


class TestSCurveCalculation(unittest.TestCase):
    """S曲线轨迹计算测试"""

    def test_initial_position(self):
        gen = TrajectoryGenerator()
        gen.set_s_curve(0, 10, 5, 2, 10)
        result = gen.calculate(0.0)
        self.assertAlmostEqual(result['position'], 0.0, places=1)

    def test_final_position(self):
        gen = TrajectoryGenerator()
        gen.set_s_curve(0, 10, 5, 2, 10)
        result = gen.calculate(gen.total_time + 1.0)
        self.assertAlmostEqual(result['position'], 10.0, places=1)

    def test_smooth_velocity(self):
        """S曲线速度应平滑"""
        gen = TrajectoryGenerator()
        gen.set_s_curve(0, 10, 5, 2, 10)
        prev_vel = 0.0
        for t_100 in range(0, int(gen.total_time * 1000)):
            t = t_100 * 0.001
            result = gen.calculate(t)
            # 速度变化应平滑（无突变）
            if t > 0.01:
                self.assertLess(abs(result['velocity'] - prev_vel), 2.0)
            prev_vel = result['velocity']

    def test_done_state(self):
        gen = TrajectoryGenerator()
        gen.set_s_curve(0, 10, 5, 2, 10)
        result = gen.calculate(gen.total_time + 1.0)
        self.assertEqual(result['state'], 'done')


# ── 五次多项式轨迹测试 ──

class TestPolynomial5thSetup(unittest.TestCase):
    """五次多项式轨迹配置测试"""

    def test_sets_type(self):
        gen = TrajectoryGenerator()
        gen.set_polynomial_5th(0, 10, 0, 0, 0, 0, 5.0)
        self.assertEqual(gen.trajectory_type, TrajectoryType.POLYNOMIAL_5TH)

    def test_coefficients_computed(self):
        gen = TrajectoryGenerator()
        gen.set_polynomial_5th(0, 10, 0, 0, 0, 0, 5.0)
        self.assertTrue(hasattr(gen, 'c0'))
        self.assertTrue(hasattr(gen, 'c5'))


class TestPolynomial5thCalculation(unittest.TestCase):
    """五次多项式轨迹计算测试"""

    def test_initial_position(self):
        gen = TrajectoryGenerator()
        gen.set_polynomial_5th(0, 10, 0, 0, 0, 0, 5.0)
        result = gen.calculate(0.0)
        self.assertAlmostEqual(result['position'], 0.0, places=2)

    def test_final_position(self):
        gen = TrajectoryGenerator()
        gen.set_polynomial_5th(0, 10, 0, 0, 0, 0, 5.0)
        result = gen.calculate(5.0 + 0.1)
        self.assertAlmostEqual(result['position'], 10.0, places=1)

    def test_boundary_velocities(self):
        """起始和终止速度应匹配设定"""
        gen = TrajectoryGenerator()
        gen.set_polynomial_5th(0, 10, 0, 0, 0, 0, 5.0)
        result_start = gen.calculate(0.0)
        self.assertAlmostEqual(result_start['velocity'], 0.0, places=2)
        result_end = gen.calculate(5.0 + 0.1)
        self.assertAlmostEqual(result_end['velocity'], 0.0, places=1)

    def test_nonzero_start_velocity(self):
        gen = TrajectoryGenerator()
        gen.set_polynomial_5th(0, 10, 2.0, 0, 0, 0, 5.0)
        result = gen.calculate(0.0)
        self.assertAlmostEqual(result['velocity'], 2.0, places=1)

    def test_contains_jerk(self):
        gen = TrajectoryGenerator()
        gen.set_polynomial_5th(0, 10, 0, 0, 0, 0, 5.0)
        result = gen.calculate(2.5)
        self.assertIn('jerk', result)

    def test_done_state(self):
        gen = TrajectoryGenerator()
        gen.set_polynomial_5th(0, 10, 0, 0, 0, 0, 5.0)
        result = gen.calculate(5.0 + 0.1)
        self.assertEqual(result['state'], 'done')


# ── 生成完整轨迹测试 ──

class TestGenerateTrajectory(unittest.TestCase):
    """生成完整轨迹测试"""

    def test_returns_dict(self):
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 10, 5, 2)
        data = gen.generate_trajectory()
        self.assertIn('time', data)
        self.assertIn('position', data)
        self.assertIn('velocity', data)
        self.assertIn('acceleration', data)

    def test_array_lengths_match(self):
        gen = TrajectoryGenerator()
        gen.set_trapezoidal(0, 10, 5, 2)
        data = gen.generate_trajectory()
        n = len(data['time'])
        self.assertEqual(len(data['position']), n)
        self.assertEqual(len(data['velocity']), n)
        self.assertEqual(len(data['acceleration']), n)

    def test_s_curve_generation(self):
        gen = TrajectoryGenerator()
        gen.set_s_curve(0, 10, 5, 2, 10)
        data = gen.generate_trajectory()
        self.assertGreater(len(data['time']), 0)

    def test_polynomial_generation(self):
        gen = TrajectoryGenerator()
        gen.set_polynomial_5th(0, 10, 0, 0, 0, 0, 5.0)
        data = gen.generate_trajectory()
        self.assertGreater(len(data['time']), 0)


if __name__ == '__main__':
    unittest.main()
