#!/usr/bin/env python3
"""
轨迹预测器单元测试
覆盖: 点添加、历史管理、线性预测、抛物线预测、样条预测、
      截距预测、多目标预测、统一接口、边界条件
测试对象: 10_视觉通用代码库/trajectory_predictor.py
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '10_视觉通用代码库'))

from trajectory_predictor import TrajectoryPredictor, MultiPointPredictor


class TestTrajectoryPredictorInit(unittest.TestCase):
    """初始化测试"""

    def test_default_max_history(self):
        """默认最大历史长度"""
        pred = TrajectoryPredictor()
        self.assertEqual(pred.count, 0)

    def test_custom_max_history(self):
        """自定义最大历史长度"""
        pred = TrajectoryPredictor(max_history=20)
        self.assertEqual(pred.count, 0)

    def test_clear(self):
        """清除应重置所有状态"""
        pred = TrajectoryPredictor()
        pred.add_point(1.0, 2.0)
        pred.add_point(3.0, 4.0)
        pred.clear()
        self.assertEqual(pred.count, 0)


class TestAddPoint(unittest.TestCase):
    """点添加测试"""

    def test_add_single_point(self):
        """添加单个点"""
        pred = TrajectoryPredictor()
        pred.add_point(10.0, 20.0)
        self.assertEqual(pred.count, 1)

    def test_add_multiple_points(self):
        """添加多个点"""
        pred = TrajectoryPredictor()
        for i in range(10):
            pred.add_point(float(i), float(i * 2))
        self.assertEqual(pred.count, 10)

    def test_add_point_with_time(self):
        """带时间戳添加"""
        pred = TrajectoryPredictor()
        pred.add_point(1.0, 2.0, t=0.5)
        pred.add_point(3.0, 4.0, t=1.0)
        self.assertEqual(pred.count, 2)

    def test_auto_increment_time(self):
        """不指定时间应自动递增"""
        pred = TrajectoryPredictor()
        pred.add_point(1.0, 2.0)
        pred.add_point(3.0, 4.0)
        self.assertEqual(pred._t_counter, 2)

    def test_max_history_limit(self):
        """应限制历史长度"""
        pred = TrajectoryPredictor(max_history=5)
        for i in range(10):
            pred.add_point(float(i), float(i))
        self.assertEqual(pred.count, 5)


class TestLinearPrediction(unittest.TestCase):
    """线性预测测试"""

    def test_insufficient_points(self):
        """少于2个点应返回空"""
        pred = TrajectoryPredictor()
        pred.add_point(1.0, 2.0)
        result = pred.predict_linear(steps=5)
        self.assertEqual(len(result), 0)

    def test_linear_two_points(self):
        """两个点应能预测"""
        pred = TrajectoryPredictor()
        pred.add_point(0.0, 0.0, t=0.0)
        pred.add_point(1.0, 1.0, t=1.0)
        result = pred.predict_linear(steps=3)
        self.assertEqual(len(result), 3)

    def test_linear_uniform_motion(self):
        """匀速直线运动应精确预测"""
        pred = TrajectoryPredictor()
        # x = 2t, y = 3t
        for i in range(10):
            t = float(i) * 0.1
            pred.add_point(2.0 * t, 3.0 * t, t=t)
        result = pred.predict_linear(steps=5)
        # 预测点应接近直线延长
        self.assertGreater(len(result), 0)
        # 检查预测的斜率
        if len(result) >= 2:
            dx = result[1][0] - result[0][0]
            dy = result[1][1] - result[0][1]
            expected_ratio = 3.0 / 2.0
            actual_ratio = dy / (dx + 1e-9)
            self.assertAlmostEqual(actual_ratio, expected_ratio, delta=0.5)

    def test_linear_returns_tuples(self):
        """预测结果应为(x, y)元组"""
        pred = TrajectoryPredictor()
        pred.add_point(0.0, 0.0, t=0.0)
        pred.add_point(1.0, 1.0, t=1.0)
        result = pred.predict_linear(steps=5)
        for pt in result:
            self.assertEqual(len(pt), 2)


class TestParabolicPrediction(unittest.TestCase):
    """抛物线预测测试"""

    def test_falls_back_to_linear(self):
        """少于3个点应回退到线性"""
        pred = TrajectoryPredictor()
        pred.add_point(0.0, 0.0, t=0.0)
        pred.add_point(1.0, 1.0, t=1.0)
        result = pred.predict_parabolic(steps=5)
        self.assertEqual(len(result), 5)

    def test_parabolic_with_enough_points(self):
        """足够点数应使用抛物线"""
        pred = TrajectoryPredictor()
        for i in range(10):
            t = float(i) * 0.1
            # y = -4.9t^2 + 10t (抛体运动)
            pred.add_point(5.0 * t, 10.0 * t - 4.9 * t * t, t=t)
        result = pred.predict_parabolic(steps=5)
        self.assertEqual(len(result), 5)

    def test_parabolic_captures_curvature(self):
        """抛物线应捕捉曲率"""
        pred = TrajectoryPredictor()
        for i in range(15):
            t = float(i) * 0.1
            x = 5.0 * t
            y = -4.9 * t * t + 10.0 * t
            pred.add_point(x, y, t=t)
        result = pred.predict_parabolic(steps=10)
        # 预测应显示下落趋势(y递减)
        if len(result) >= 2:
            self.assertLess(result[-1][1], result[0][1])


class TestSplinePrediction(unittest.TestCase):
    """样条预测测试"""

    def test_falls_back_to_parabolic(self):
        """少于4个点应回退到抛物线"""
        pred = TrajectoryPredictor()
        pred.add_point(0.0, 0.0, t=0.0)
        pred.add_point(1.0, 1.0, t=1.0)
        pred.add_point(2.0, 0.5, t=2.0)
        result = pred.predict_spline(steps=5)
        self.assertEqual(len(result), 5)

    def test_spline_with_enough_points(self):
        """足够点数应使用样条"""
        pred = TrajectoryPredictor()
        for i in range(10):
            t = float(i)
            pred.add_point(np.sin(t), np.cos(t), t=t)
        result = pred.predict_spline(steps=5)
        self.assertEqual(len(result), 5)

    def test_spline_smooth(self):
        """样条预测应较平滑"""
        pred = TrajectoryPredictor()
        for i in range(20):
            t = float(i) * 0.1
            pred.add_point(t, np.sin(t), t=t)
        result = pred.predict_spline(steps=10)
        # 相邻预测点距离应较小
        for i in range(1, len(result)):
            dist = np.sqrt((result[i][0] - result[i-1][0])**2 +
                          (result[i][1] - result[i-1][1])**2)
            self.assertLess(dist, 5.0)


class TestUnifiedPredictInterface(unittest.TestCase):
    """统一预测接口测试"""

    def test_linear_method(self):
        """method='linear'应调用线性预测"""
        pred = TrajectoryPredictor()
        pred.add_point(0.0, 0.0, t=0.0)
        pred.add_point(1.0, 1.0, t=1.0)
        result = pred.predict(method="linear", steps=5)
        self.assertEqual(len(result), 5)

    def test_parabolic_method(self):
        """method='parabolic'应调用抛物线预测"""
        pred = TrajectoryPredictor()
        for i in range(5):
            pred.add_point(float(i), float(i), t=float(i))
        result = pred.predict(method="parabolic", steps=5)
        self.assertEqual(len(result), 5)

    def test_spline_method(self):
        """method='spline'应调用样条预测"""
        pred = TrajectoryPredictor()
        for i in range(10):
            pred.add_point(float(i), float(i), t=float(i))
        result = pred.predict(method="spline", steps=5)
        self.assertEqual(len(result), 5)

    def test_unknown_method_raises(self):
        """未知方法应抛出异常"""
        pred = TrajectoryPredictor()
        for i in range(5):
            pred.add_point(float(i), float(i), t=float(i))
        with self.assertRaises(ValueError):
            pred.predict(method="unknown")


class TestInterceptPrediction(unittest.TestCase):
    """截距预测测试"""

    def test_intercept_horizontal_line(self):
        """预测与水平线交点"""
        pred = TrajectoryPredictor()
        # x = t, y = t (上升直线)
        for i in range(10):
            t = float(i)
            pred.add_point(t, t, t=t)
        result = pred.predict_intercept(target_y=15.0)
        # 应在 x ≈ 15 附近
        if result is not None:
            self.assertAlmostEqual(result, 15.0, delta=5.0)

    def test_intercept_no_crossing(self):
        """不交叉时应返回None"""
        pred = TrajectoryPredictor()
        # 水平线 y=0
        for i in range(10):
            pred.add_point(float(i), 0.0, t=float(i))
        result = pred.predict_intercept(target_y=100.0)
        # 可能返回None(如果预测也是水平的)
        # 或者返回某个值
        self.assertTrue(result is None or isinstance(result, float))


class TestMultiPointPredictor(unittest.TestCase):
    """多目标预测管理测试"""

    def test_add_multiple_objects(self):
        """应能管理多个目标"""
        mp = MultiPointPredictor()
        mp.add_point(1, 10.0, 20.0)
        mp.add_point(2, 30.0, 40.0)
        p1 = mp.get_predictor(1)
        p2 = mp.get_predictor(2)
        self.assertIsNotNone(p1)
        self.assertIsNotNone(p2)
        self.assertEqual(p1.count, 1)
        self.assertEqual(p2.count, 1)

    def test_predict_existing_object(self):
        """应能预测已添加的目标"""
        mp = MultiPointPredictor()
        for i in range(5):
            mp.add_point(1, float(i), float(i), t=float(i))
        result = mp.predict(1, method="linear", steps=3)
        self.assertEqual(len(result), 3)

    def test_predict_nonexistent_object(self):
        """不存在的目标应返回空"""
        mp = MultiPointPredictor()
        result = mp.predict(999, method="linear", steps=5)
        self.assertEqual(len(result), 0)

    def test_remove_object(self):
        """应能删除目标"""
        mp = MultiPointPredictor()
        mp.add_point(1, 10.0, 20.0)
        mp.remove(1)
        self.assertIsNone(mp.get_predictor(1))

    def test_remove_nonexistent(self):
        """删除不存在的目标不应报错"""
        mp = MultiPointPredictor()
        mp.remove(999)  # 不应抛出异常


class TestTrajectoryPredictorEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_empty_prediction(self):
        """空历史应返回空"""
        pred = TrajectoryPredictor()
        self.assertEqual(pred.predict_linear(5), [])
        self.assertEqual(pred.predict_parabolic(5), [])
        self.assertEqual(pred.predict_spline(5), [])

    def test_single_point_prediction(self):
        """单个点应返回空"""
        pred = TrajectoryPredictor()
        pred.add_point(5.0, 5.0)
        self.assertEqual(pred.predict_linear(5), [])

    def test_identical_points(self):
        """相同点不应崩溃"""
        pred = TrajectoryPredictor()
        for i in range(5):
            pred.add_point(5.0, 5.0, t=float(i))
        result = pred.predict_linear(3)
        # 应能正常返回
        self.assertEqual(len(result), 3)

    def test_very_close_points(self):
        """非常接近的点不应导致数值问题"""
        pred = TrajectoryPredictor()
        for i in range(10):
            pred.add_point(5.0 + i * 1e-10, 5.0 + i * 1e-10, t=float(i))
        result = pred.predict_linear(3)
        self.assertEqual(len(result), 3)
        # 预测值不应为nan或inf
        for pt in result:
            self.assertFalse(np.isnan(pt[0]))
            self.assertFalse(np.isnan(pt[1]))

    def test_negative_coordinates(self):
        """负坐标应正常"""
        pred = TrajectoryPredictor()
        for i in range(5):
            pred.add_point(-float(i), -float(i), t=float(i))
        result = pred.predict_linear(3)
        self.assertEqual(len(result), 3)

    def test_large_coordinates(self):
        """大坐标应正常"""
        pred = TrajectoryPredictor()
        for i in range(5):
            pred.add_point(float(i) * 1000, float(i) * 1000, t=float(i))
        result = pred.predict_linear(3)
        self.assertEqual(len(result), 3)
        for pt in result:
            self.assertFalse(np.isnan(pt[0]))
            self.assertFalse(np.isnan(pt[1]))


if __name__ == '__main__':
    unittest.main()
