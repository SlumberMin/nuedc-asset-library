#!/usr/bin/env python3
"""
角度估计模块单元测试
覆盖: 最小外接矩形角度、归一化角度、矩方法、椭圆拟合、两点角度、线段角度、透视角度
"""

import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from _10_视觉通用代码库.angle_estimator import AngleEstimator


def make_rotated_rect_contour(center, size, angle_deg):
    """创建旋转矩形轮廓"""
    rect = (center, size, angle_deg)
    box = cv2.boxPoints(rect)
    return np.int0(box).reshape(-1, 1, 2)


class TestRotationAngleMinRect(unittest.TestCase):
    """最小外接矩形角度"""

    def test_horizontal_rect(self):
        """水平矩形角度应接近0或-90"""
        contour = make_rotated_rect_contour((100, 100), (80, 40), 0)
        angle = AngleEstimator.get_rotation_angle_minrect(contour)
        self.assertTrue(-95 < angle < 5)

    def test_returns_float(self):
        contour = make_rotated_rect_contour((100, 100), (80, 40), 15)
        angle = AngleEstimator.get_rotation_angle_minrect(contour)
        self.assertIsInstance(angle, float)


class TestNormalizedAngle(unittest.TestCase):
    """归一化角度 [-90, 90]"""

    def test_range(self):
        """角度应在 [-90, 90]"""
        for ang in [0, 15, 30, 45, 60, 75]:
            contour = make_rotated_rect_contour((200, 200), (120, 50), ang)
            result = AngleEstimator.get_normalized_angle(contour)
            self.assertGreaterEqual(result, -95)
            self.assertLessEqual(result, 95)


class TestOrientationByMoments(unittest.TestCase):
    """矩方法角度"""

    def test_horizontal_rect(self):
        """水平矩形主轴应接近0度"""
        contour = make_rotated_rect_contour((200, 200), (160, 60), 0)
        angle = AngleEstimator.get_orientation_by_moments(contour)
        self.assertAlmostEqual(abs(angle), 0, delta=5)

    def test_returns_float(self):
        contour = make_rotated_rect_contour((200, 200), (100, 50), 0)
        result = AngleEstimator.get_orientation_by_moments(contour)
        self.assertIsInstance(result, float)


class TestEstimateTiltFromEllipse(unittest.TestCase):
    """椭圆拟合"""

    def test_enough_points(self):
        """>=5个点应返回结果"""
        pts = make_rotated_rect_contour((200, 200), (120, 60), 20)
        result = AngleEstimator.estimate_tilt_from_ellipse(pts)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 3)

    def test_too_few_points(self):
        """<5个点应返回None"""
        contour = np.array([[[0, 0]], [[1, 1]], [[2, 0]]], dtype=np.int32)
        result = AngleEstimator.estimate_tilt_from_ellipse(contour)
        self.assertIsNone(result)


class TestAngleFromTwoPoints(unittest.TestCase):
    """两点角度"""

    def test_horizontal_right(self):
        angle = AngleEstimator.estimate_angle_from_two_points((0, 0), (10, 0))
        self.assertAlmostEqual(angle, 0.0, places=1)

    def test_vertical_down(self):
        angle = AngleEstimator.estimate_angle_from_two_points((0, 0), (0, 10))
        self.assertAlmostEqual(angle, 90.0, places=1)

    def test_horizontal_left(self):
        angle = AngleEstimator.estimate_angle_from_two_points((10, 0), (0, 0))
        self.assertAlmostEqual(abs(angle), 180.0, delta=1)

    def test_45_degrees(self):
        angle = AngleEstimator.estimate_angle_from_two_points((0, 0), (10, 10))
        self.assertAlmostEqual(angle, 45.0, places=1)


class TestAngleFromLines(unittest.TestCase):
    """线段角度"""

    def test_none_lines(self):
        result = AngleEstimator.estimate_angle_from_lines(None)
        self.assertIsNone(result)

    def test_empty_lines(self):
        result = AngleEstimator.estimate_angle_from_lines([])
        self.assertIsNone(result)

    def test_single_line(self):
        lines = np.array([[[0, 0, 10, 10]]], dtype=np.int32)
        result = AngleEstimator.estimate_angle_from_lines(lines)
        self.assertAlmostEqual(result, 45.0, places=0)

    def test_multiple_lines(self):
        lines = np.array([
            [[0, 0, 10, 10]],
            [[0, 0, 10, 0]],
        ], dtype=np.int32)
        result = AngleEstimator.estimate_angle_from_lines(lines)
        self.assertIsNotNone(result)


class TestLongestLineAngle(unittest.TestCase):
    """最长边角度"""

    def test_returns_float(self):
        contour = make_rotated_rect_contour((200, 200), (120, 60), 30)
        result = AngleEstimator.get_longest_line_angle(contour)
        self.assertIsInstance(result, float)


class TestEstimatePerspectiveAngle(unittest.TestCase):
    """透视角度"""

    def test_identity(self):
        """单位变换应返回接近0的角度"""
        src = np.array([[0, 0], [100, 0], [100, 100], [0, 100]], dtype=np.float32)
        dst = src.copy()
        H, pitch, yaw = AngleEstimator.estimate_perspective_angle(src, dst)
        self.assertIsNotNone(H)
        self.assertAlmostEqual(abs(pitch), 0, delta=1)
        self.assertAlmostEqual(abs(yaw), 0, delta=1)


if __name__ == '__main__':
    unittest.main()
