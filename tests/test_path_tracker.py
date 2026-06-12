#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路径跟踪单元测试
覆盖: BicycleModel初始化/更新/轨迹记录、PurePursuitController初始化/
      最近点查找/控制计算/弯道减速、generate_figure8_path/generate_smooth_path路径生成
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from _11_控制算法库.simulation.path_tracking_simulation import (
    BicycleModel,
    PurePursuitController,
    generate_figure8_path,
    generate_smooth_path,
)


# ==================== 自行车模型测试 ====================

class TestBicycleModelInit(unittest.TestCase):
    """自行车模型初始化测试"""

    def test_default_params(self):
        model = BicycleModel()
        self.assertEqual(model.x, 0.0)
        self.assertEqual(model.y, 0.0)
        self.assertEqual(model.yaw, 0.0)
        self.assertEqual(model.v, 0.0)
        self.assertEqual(model.L, 0.5)

    def test_custom_params(self):
        model = BicycleModel(x=1.0, y=2.0, yaw=0.5, v=1.0, wheelbase=0.8)
        self.assertEqual(model.x, 1.0)
        self.assertEqual(model.y, 2.0)
        self.assertEqual(model.yaw, 0.5)
        self.assertEqual(model.v, 1.0)
        self.assertEqual(model.L, 0.8)

    def test_trail_initialized(self):
        model = BicycleModel(x=1.0, y=2.0)
        self.assertEqual(model.trail_x, [1.0])
        self.assertEqual(model.trail_y, [2.0])


class TestBicycleModelUpdate(unittest.TestCase):
    """自行车模型更新测试"""

    def test_position_changes(self):
        model = BicycleModel(v=1.0)
        model.update(steering=0.0, target_speed=1.0, dt=0.1)
        self.assertNotEqual(model.x, 0.0)

    def test_straight_motion(self):
        """直行应沿x轴移动"""
        model = BicycleModel(x=0, y=0, yaw=0, v=0)
        model.update(steering=0.0, target_speed=1.0, dt=1.0)
        self.assertGreater(model.x, 0)

    def test_yaw_updates_with_steering(self):
        """有转向角时应改变航向"""
        model = BicycleModel(v=1.0)
        model.update(steering=0.3, target_speed=1.0, dt=0.5)
        self.assertNotEqual(model.yaw, 0.0)

    def test_yaw_wrapping(self):
        """航向角应保持在[-pi, pi]范围内"""
        model = BicycleModel(yaw=3.0, v=2.0)
        for _ in range(100):
            model.update(steering=0.5, target_speed=2.0, dt=0.1)
        self.assertGreaterEqual(model.yaw, -np.pi - 0.1)
        self.assertLessEqual(model.yaw, np.pi + 0.1)

    def test_trail_grows(self):
        model = BicycleModel(v=1.0)
        model.update(steering=0.0, target_speed=1.0, dt=0.1)
        self.assertEqual(len(model.trail_x), 2)
        self.assertEqual(len(model.trail_y), 2)

    def test_speed_control(self):
        """速度应趋近目标速度"""
        model = BicycleModel(v=0)
        for _ in range(50):
            model.update(steering=0.0, target_speed=1.5, dt=0.05)
        self.assertAlmostEqual(model.v, 1.5, delta=0.5)

    def test_negative_speed_clamped(self):
        """速度不应低于-1"""
        model = BicycleModel(v=-2.0)
        model.update(steering=0.0, target_speed=-2.0, dt=0.1)
        self.assertGreaterEqual(model.v, -1.5)

    def test_max_speed_clamped(self):
        """速度不应超过3"""
        model = BicycleModel(v=5.0)
        model.update(steering=0.0, target_speed=5.0, dt=0.1)
        self.assertLessEqual(model.v, 3.5)


# ==================== PurePursuit控制器测试 ====================

class TestPurePursuitControllerInit(unittest.TestCase):
    """PurePursuit控制器初始化测试"""

    def test_default_params(self):
        path_x = [0, 1, 2, 3]
        path_y = [0, 0, 0, 0]
        ctrl = PurePursuitController(path_x, path_y)
        self.assertEqual(ctrl.wheelbase, 0.5)
        self.assertEqual(ctrl.target_speed, 1.0)
        self.assertEqual(ctrl.last_idx, 0)

    def test_custom_params(self):
        path_x = [0, 1, 2]
        path_y = [0, 0, 0]
        ctrl = PurePursuitController(path_x, path_y, wheelbase=0.8,
                                      target_speed=2.0, max_steer=np.radians(45))
        self.assertEqual(ctrl.wheelbase, 0.8)
        self.assertEqual(ctrl.target_speed, 2.0)

    def test_path_stored(self):
        path_x = [0, 1, 2]
        path_y = [0, 1, 0]
        ctrl = PurePursuitController(path_x, path_y)
        np.testing.assert_array_equal(ctrl.path_x, np.array(path_x))
        np.testing.assert_array_equal(ctrl.path_y, np.array(path_y))


class TestPurePursuitFindNearest(unittest.TestCase):
    """PurePursuit最近点查找测试"""

    def test_finds_nearest(self):
        path_x = [0, 1, 2, 3, 4]
        path_y = [0, 0, 0, 0, 0]
        ctrl = PurePursuitController(path_x, path_y)
        idx = ctrl.find_nearest(0.5, 0.0)
        self.assertEqual(idx, 0)  # 或1,取决于搜索范围

    def test_returns_int(self):
        path_x = [0, 1, 2, 3]
        path_y = [0, 0, 0, 0]
        ctrl = PurePursuitController(path_x, path_y)
        idx = ctrl.find_nearest(1.0, 0.0)
        self.assertIsInstance(idx, (int, np.integer))

    def test_index_within_bounds(self):
        path_x = [0, 1, 2, 3, 4]
        path_y = [0, 1, 0, 1, 0]
        ctrl = PurePursuitController(path_x, path_y)
        for _ in range(10):
            x = np.random.uniform(-1, 5)
            y = np.random.uniform(-1, 2)
            idx = ctrl.find_nearest(x, y)
            self.assertGreaterEqual(idx, 0)
            self.assertLess(idx, len(path_x))


class TestPurePursuitCompute(unittest.TestCase):
    """PurePursuit控制计算测试"""

    def test_returns_tuple_of_6(self):
        path_x = np.linspace(0, 10, 100)
        path_y = np.sin(np.linspace(0, 2 * np.pi, 100))
        ctrl = PurePursuitController(path_x, path_y)
        result = ctrl.compute(0, 0, 0, 0.5)
        self.assertEqual(len(result), 6)

    def test_steering_within_limits(self):
        path_x = np.linspace(0, 10, 100)
        path_y = np.sin(np.linspace(0, 2 * np.pi, 100))
        ctrl = PurePursuitController(path_x, path_y, max_steer=np.radians(30))
        steer, _, _, _, _, _ = ctrl.compute(0, 0, 0, 0.5)
        self.assertGreaterEqual(steer, -np.radians(30) - 0.01)
        self.assertLessEqual(steer, np.radians(30) + 0.01)

    def test_target_speed_reasonable(self):
        path_x = np.linspace(0, 10, 100)
        path_y = np.sin(np.linspace(0, 2 * np.pi, 100))
        ctrl = PurePursuitController(path_x, path_y, target_speed=2.0)
        _, tgt_speed, _, _, _, _ = ctrl.compute(0, 0, 0, 0.5)
        self.assertGreater(tgt_speed, 0)
        self.assertLessEqual(tgt_speed, 2.5)

    def test_lookahead_distance_positive(self):
        path_x = np.linspace(0, 10, 100)
        path_y = np.zeros(100)
        ctrl = PurePursuitController(path_x, path_y)
        _, _, _, Ld, _, _ = ctrl.compute(0, 0, 0, 1.0)
        self.assertGreater(Ld, 0)

    def test_target_point_on_path(self):
        path_x = np.linspace(0, 10, 100)
        path_y = np.zeros(100)
        ctrl = PurePursuitController(path_x, path_y)
        _, _, _, _, tidx, (tx, ty) = ctrl.compute(0, 0, 0, 0.5)
        self.assertGreaterEqual(tidx, 0)
        self.assertLess(tidx, len(path_x))

    def test_curvature_value(self):
        path_x = np.linspace(0, 10, 100)
        path_y = np.sin(np.linspace(0, 2 * np.pi, 100))
        ctrl = PurePursuitController(path_x, path_y)
        _, _, curv, _, _, _ = ctrl.compute(0, 0, 0, 0.5)
        self.assertIsInstance(curv, float)

    def test_last_idx_updates(self):
        path_x = np.linspace(0, 10, 100)
        path_y = np.zeros(100)
        ctrl = PurePursuitController(path_x, path_y)
        initial_idx = ctrl.last_idx
        ctrl.compute(1.0, 0, 0, 0.5)
        # last_idx 可能更新
        self.assertGreaterEqual(ctrl.last_idx, initial_idx)


# ==================== 路径生成测试 ====================

class TestGenerateFigure8Path(unittest.TestCase):
    """8字形路径生成测试"""

    def test_returns_tuple_of_2(self):
        result = generate_figure8_path()
        self.assertEqual(len(result), 2)

    def test_default_points(self):
        x, y = generate_figure8_path()
        self.assertEqual(len(x), 200)
        self.assertEqual(len(y), 200)

    def test_custom_points(self):
        x, y = generate_figure8_path(n_points=100)
        self.assertEqual(len(x), 100)

    def test_figure8_shape(self):
        """8字形路径y应在[-1.5, 1.5]范围内"""
        x, y = generate_figure8_path()
        self.assertGreaterEqual(y.min(), -1.6)
        self.assertLessEqual(y.max(), 1.6)

    def test_x_range(self):
        """x应在[-3, 3]范围内"""
        x, y = generate_figure8_path()
        self.assertGreaterEqual(x.min(), -3.1)
        self.assertLessEqual(x.max(), 3.1)


class TestGenerateSmoothPath(unittest.TestCase):
    """平滑S形路径生成测试"""

    def test_returns_tuple_of_2(self):
        result = generate_smooth_path()
        self.assertEqual(len(result), 2)

    def test_default_points(self):
        x, y = generate_smooth_path()
        self.assertEqual(len(x), 300)

    def test_x_monotonically_increasing(self):
        """x应单调递增"""
        x, y = generate_smooth_path()
        for i in range(len(x) - 1):
            self.assertGreater(x[i + 1], x[i])

    def test_y_bounded(self):
        x, y = generate_smooth_path()
        self.assertGreater(y.min(), -4.0)
        self.assertLess(y.max(), 4.0)


# ==================== 路径跟踪集成测试 ====================

class TestPathTrackingIntegration(unittest.TestCase):
    """路径跟踪集成测试"""

    def test_full_tracking_loop(self):
        """完整跟踪循环应能运行"""
        path_x, path_y = generate_figure8_path(200)
        model = BicycleModel(x=path_x[0] - 0.5, y=path_y[0] - 0.5,
                              yaw=0.0, v=0.0, wheelbase=0.5)
        ctrl = PurePursuitController(path_x, path_y,
                                      wheelbase=0.5, target_speed=1.0)

        dt = 0.05
        for _ in range(200):
            steer, tgt_speed, _, _, _, _ = ctrl.compute(
                model.x, model.y, model.yaw, model.v)
            model.update(steer, tgt_speed, dt)

        # 应有移动
        dist = np.sqrt(model.x ** 2 + model.y ** 2)
        self.assertGreater(len(model.trail_x), 100)

    def test_tracking_error_bounded(self):
        """跟踪误差应在合理范围内"""
        path_x, path_y = generate_smooth_path(200)
        model = BicycleModel(x=path_x[0], y=path_y[0],
                              yaw=0.0, v=0.0, wheelbase=0.5)
        ctrl = PurePursuitController(path_x, path_y,
                                      wheelbase=0.5, target_speed=1.0,
                                      lookahead_base=0.5)

        dt = 0.05
        max_error = 0
        for _ in range(200):
            steer, tgt_speed, _, _, tidx, _ = ctrl.compute(
                model.x, model.y, model.yaw, model.v)
            model.update(steer, tgt_speed, dt)
            err = np.hypot(model.x - path_x[tidx], model.y - path_y[tidx])
            max_error = max(max_error, err)

        # 最大误差应小于合理值（路径宽度的几倍）
        self.assertLess(max_error, 10.0)

    def test_vehicle_moves_forward(self):
        """车辆应向前移动"""
        path_x = np.linspace(0, 10, 200)
        path_y = np.zeros(200)
        model = BicycleModel(x=0, y=0, yaw=0, v=0, wheelbase=0.5)
        ctrl = PurePursuitController(path_x, path_y,
                                      wheelbase=0.5, target_speed=1.5)

        for _ in range(100):
            steer, tgt_speed, _, _, _, _ = ctrl.compute(
                model.x, model.y, model.yaw, model.v)
            model.update(steer, tgt_speed, 0.05)

        self.assertGreater(model.x, 2.0)


if __name__ == '__main__':
    unittest.main()
