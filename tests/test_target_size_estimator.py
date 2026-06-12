#!/usr/bin/env python3
"""
目标大小测量模块单元测试
覆盖: 焦距标定、像素距离换算、边界框测量、参考物法、面积估计、轮廓测量
"""

import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from _10_视觉通用代码库.target_size_estimator import TargetSizeEstimator


class TestTargetSizeInit(unittest.TestCase):
    """初始化测试"""

    def test_basic_init(self):
        est = TargetSizeEstimator(focal_length_px=350)
        self.assertEqual(est.focal_length_px, 350)

    def test_mm_to_px_conversion(self):
        """物理焦距+传感器信息应自动换算为像素焦距"""
        est = TargetSizeEstimator(focal_length_px=3.6, sensor_width_mm=5.0, image_width_px=640)
        expected = 3.6 * 640 / 5.0
        self.assertAlmostEqual(est.focal_length_px, expected, places=1)


class TestCalibrateFocalLength(unittest.TestCase):
    """焦距标定"""

    def test_known_params(self):
        f = TargetSizeEstimator.calibrate_focal_length(1.0, 0.1, 350)
        self.assertAlmostEqual(f, 3500.0, places=1)

    def test_symmetry(self):
        """正反标定应一致"""
        f = TargetSizeEstimator.calibrate_focal_length(2.0, 0.2, 100)
        size = (100 * 2.0) / f
        self.assertAlmostEqual(size, 0.2, places=5)


class TestEstimateSizeByDistance(unittest.TestCase):
    """已知距离下的尺寸计算"""

    def test_basic(self):
        est = TargetSizeEstimator(focal_length_px=350)
        size = est.estimate_size_by_distance(known_distance_m=1.0, pixel_length=70)
        self.assertAlmostEqual(size, 70 * 1.0 / 350, places=5)

    def test_proportional(self):
        """像素越长，实际尺寸越大"""
        est = TargetSizeEstimator(focal_length_px=350)
        s1 = est.estimate_size_by_distance(1.0, 50)
        s2 = est.estimate_size_by_distance(1.0, 100)
        self.assertAlmostEqual(s2, 2 * s1, places=5)

    def test_distance_proportional(self):
        """距离越大，同像素对应的实际尺寸越大"""
        est = TargetSizeEstimator(focal_length_px=350)
        s1 = est.estimate_size_by_distance(1.0, 100)
        s2 = est.estimate_size_by_distance(2.0, 100)
        self.assertAlmostEqual(s2, 2 * s1, places=5)


class TestEstimateWidthHeight(unittest.TestCase):
    """边界框宽高测量"""

    def test_basic(self):
        est = TargetSizeEstimator(focal_length_px=350)
        w, h = est.estimate_width_height(1.0, (100, 100, 70, 50))
        self.assertAlmostEqual(w, 70.0 / 350, places=5)
        self.assertAlmostEqual(h, 50.0 / 350, places=5)

    def test_zero_bbox(self):
        est = TargetSizeEstimator(focal_length_px=350)
        w, h = est.estimate_width_height(1.0, (0, 0, 0, 0))
        self.assertEqual(w, 0.0)
        self.assertEqual(h, 0.0)


class TestEstimateFromTwoPoints(unittest.TestCase):
    """两点间距离测量"""

    def test_horizontal(self):
        est = TargetSizeEstimator(focal_length_px=350)
        d = est.estimate_from_two_points(1.0, (0, 0), (350, 0))
        self.assertAlmostEqual(d, 1.0, places=2)

    def test_diagonal(self):
        est = TargetSizeEstimator(focal_length_px=350)
        d = est.estimate_from_two_points(1.0, (0, 0), (3, 4))
        expected = 5.0 * 1.0 / 350
        self.assertAlmostEqual(d, expected, places=5)


class TestEstimateByReference(unittest.TestCase):
    """参考物法"""

    def test_same_size(self):
        est = TargetSizeEstimator(focal_length_px=350)
        size = est.estimate_by_reference(0.1, 100, 100)
        self.assertAlmostEqual(size, 0.1, places=5)

    def test_double_pixel(self):
        est = TargetSizeEstimator(focal_length_px=350)
        size = est.estimate_by_reference(0.1, 100, 200)
        self.assertAlmostEqual(size, 0.2, places=5)


class TestEstimateArea(unittest.TestCase):
    """面积估计"""

    def test_basic(self):
        est = TargetSizeEstimator(focal_length_px=350)
        area = est.estimate_area(1.0, 350 * 350)
        self.assertAlmostEqual(area, 1.0, places=2)

    def test_scale_squared(self):
        """面积缩放应为线性缩放的平方"""
        est = TargetSizeEstimator(focal_length_px=350)
        a1 = est.estimate_area(1.0, 1000)
        a2 = est.estimate_area(2.0, 1000)
        self.assertAlmostEqual(a2, 4 * a1, places=5)


if __name__ == '__main__':
    unittest.main()
