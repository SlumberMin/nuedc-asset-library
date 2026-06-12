#!/usr/bin/env python3
"""
直线交点计算单元测试
覆盖: 直线交点、极坐标交点、夹角计算、极坐标夹角、
      点到直线距离、所有交点计算、交点聚类
测试对象: 10_视觉通用代码库/line_intersection.py
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

if HAS_NUMPY:
    from _10_视觉通用代码库.line_intersection import (
        line_intersection,
        line_intersection_polar,
        angle_between_lines,
        angle_between_lines_polar,
        point_to_line_distance,
        find_all_intersections,
        cluster_intersections,
    )


@unittest.skipUnless(HAS_NUMPY, "numpy not installed")
class TestLineIntersection(unittest.TestCase):
    """直线交点计算测试"""

    def test_perpendicular_lines(self):
        """垂直线交点"""
        pt = line_intersection((0, 0, 10, 10), (0, 10, 10, 0))
        self.assertIsNotNone(pt)
        self.assertAlmostEqual(pt[0], 5.0, places=3)
        self.assertAlmostEqual(pt[1], 5.0, places=3)

    def test_horizontal_vertical(self):
        """水平线与垂直线交点"""
        pt = line_intersection((0, 5, 10, 5), (5, 0, 5, 10))
        self.assertIsNotNone(pt)
        self.assertAlmostEqual(pt[0], 5.0, places=3)
        self.assertAlmostEqual(pt[1], 5.0, places=3)

    def test_parallel_lines(self):
        """平行线应返回 None"""
        pt = line_intersection((0, 0, 10, 0), (0, 1, 10, 1))
        self.assertIsNone(pt)

    def test_coincident_lines(self):
        """重合线应返回 None"""
        pt = line_intersection((0, 0, 10, 10), (0, 0, 20, 20))
        self.assertIsNone(pt)

    def test_diagonal_intersection(self):
        """斜线交点"""
        pt = line_intersection((0, 0, 4, 0), (2, -2, 2, 2))
        self.assertIsNotNone(pt)
        self.assertAlmostEqual(pt[0], 2.0, places=3)
        self.assertAlmostEqual(pt[1], 0.0, places=3)

    def test_negative_coordinates(self):
        """负坐标交点"""
        pt = line_intersection((-10, 0, 10, 0), (0, -10, 0, 10))
        self.assertIsNotNone(pt)
        self.assertAlmostEqual(pt[0], 0.0, places=3)
        self.assertAlmostEqual(pt[1], 0.0, places=3)

    def test_returns_tuple(self):
        pt = line_intersection((0, 0, 1, 1), (0, 1, 1, 0))
        self.assertIsInstance(pt, tuple)
        self.assertEqual(len(pt), 2)


@unittest.skipUnless(HAS_NUMPY, "numpy not installed")
class TestLineIntersectionPolar(unittest.TestCase):
    """极坐标交点测试"""

    def test_perpendicular_polar(self):
        """垂直线极坐标交点"""
        pt = line_intersection_polar(5, 0, 5, np.pi / 2)
        self.assertIsNotNone(pt)
        self.assertAlmostEqual(pt[0], 5.0, places=3)
        self.assertAlmostEqual(pt[1], 5.0, places=3)

    def test_parallel_polar(self):
        """平行极坐标线应返回 None"""
        pt = line_intersection_polar(0, 0, 5, 0)
        self.assertIsNone(pt)

    def test_origin_intersection(self):
        """过原点的交点"""
        pt = line_intersection_polar(0, 0, 0, np.pi / 2)
        self.assertIsNotNone(pt)
        self.assertAlmostEqual(pt[0], 0.0, places=3)
        self.assertAlmostEqual(pt[1], 0.0, places=3)


@unittest.skipUnless(HAS_NUMPY, "numpy not installed")
class TestAngleBetweenLines(unittest.TestCase):
    """夹角计算测试"""

    def test_perpendicular_angle(self):
        """垂直线夹角 90°"""
        angle = angle_between_lines((0, 0, 10, 0), (0, 0, 0, 10))
        self.assertAlmostEqual(angle, 90.0, places=1)

    def test_parallel_angle(self):
        """平行线夹角 0°"""
        angle = angle_between_lines((0, 0, 10, 0), (0, 5, 10, 5))
        self.assertAlmostEqual(angle, 0.0, places=1)

    def test_45_degree(self):
        """45°夹角"""
        angle = angle_between_lines((0, 0, 10, 0), (0, 0, 10, 10))
        self.assertAlmostEqual(angle, 45.0, places=1)

    def test_60_degree(self):
        """60°夹角"""
        angle = angle_between_lines((0, 0, 10, 0), (0, 0, 5, 5 * math.sqrt(3)))
        self.assertAlmostEqual(angle, 60.0, places=1)

    def test_angle_range(self):
        """角度应在 0~90° 之间（锐角）"""
        angle = angle_between_lines((0, 0, 10, 0), (0, 0, -10, 10))
        self.assertGreaterEqual(angle, 0.0)
        self.assertLessEqual(angle, 90.0)

    def test_zero_length_line(self):
        """零长度线段应返回 0"""
        angle = angle_between_lines((5, 5, 5, 5), (0, 0, 10, 0))
        self.assertEqual(angle, 0.0)


@unittest.skipUnless(HAS_NUMPY, "numpy not installed")
class TestAngleBetweenLinesPolar(unittest.TestCase):
    """极坐标夹角测试"""

    def test_perpendicular(self):
        angle = angle_between_lines_polar(0, np.pi / 2)
        self.assertAlmostEqual(angle, 90.0, places=1)

    def test_parallel(self):
        angle = angle_between_lines_polar(0, 0)
        self.assertAlmostEqual(angle, 0.0, places=1)

    def test_45_deg(self):
        angle = angle_between_lines_polar(0, np.pi / 4)
        self.assertAlmostEqual(angle, 45.0, places=1)

    def test_supplementary(self):
        """互补角应取锐角"""
        angle = angle_between_lines_polar(0, 3 * np.pi / 4)
        self.assertAlmostEqual(angle, 45.0, places=1)


@unittest.skipUnless(HAS_NUMPY, "numpy not installed")
class TestPointToLineDistance(unittest.TestCase):
    """点到直线距离测试"""

    def test_point_on_line(self):
        """线上的点距离为 0"""
        dist = point_to_line_distance((5, 0), (0, 0, 10, 0))
        self.assertAlmostEqual(dist, 0.0, places=3)

    def test_horizontal_line(self):
        """点到水平线的距离"""
        dist = point_to_line_distance((5, 3), (0, 0, 10, 0))
        self.assertAlmostEqual(dist, 3.0, places=3)

    def test_vertical_line(self):
        """点到垂直线的距离"""
        dist = point_to_line_distance((3, 5), (0, 0, 0, 10))
        self.assertAlmostEqual(dist, 3.0, places=3)

    def test_diagonal_line(self):
        """点到斜线的距离"""
        dist = point_to_line_distance((0, 0), (0, 10, 10, 0))
        expected = 10.0 / math.sqrt(2)
        self.assertAlmostEqual(dist, expected, places=2)

    def test_zero_length_line(self):
        """零长度线段退化为点距"""
        dist = point_to_line_distance((3, 4), (0, 0, 0, 0))
        self.assertAlmostEqual(dist, 5.0, places=3)


@unittest.skipUnless(HAS_NUMPY, "numpy not installed")
class TestFindAllIntersections(unittest.TestCase):
    """所有交点计算测试"""

    def test_two_lines(self):
        lines = [(0, 0, 10, 10), (0, 10, 10, 0)]
        pts = find_all_intersections(lines)
        self.assertEqual(len(pts), 1)

    def test_three_lines(self):
        lines = [(0, 0, 10, 10), (0, 10, 10, 0), (5, 0, 5, 10)]
        pts = find_all_intersections(lines)
        self.assertEqual(len(pts), 3)

    def test_parallel_lines(self):
        lines = [(0, 0, 10, 0), (0, 1, 10, 1), (0, 2, 10, 2)]
        pts = find_all_intersections(lines)
        self.assertEqual(len(pts), 0)

    def test_image_shape_filter(self):
        """图像范围外的交点应被过滤"""
        lines = [(-100, -100, 200, 200), (-100, 200, 200, -100)]
        pts = find_all_intersections(lines, image_shape=(100, 100))
        # 交点 (50, 50) 在范围内
        self.assertEqual(len(pts), 1)


@unittest.skipUnless(HAS_NUMPY, "numpy not installed")
class TestClusterIntersections(unittest.TestCase):
    """交点聚类测试"""

    def test_empty_input(self):
        result = cluster_intersections([])
        self.assertEqual(len(result), 0)

    def test_merge_close_points(self):
        pts = [(10.0, 10.0), (11.0, 10.5), (100.0, 100.0)]
        result = cluster_intersections(pts, eps=5.0)
        self.assertEqual(len(result), 2)

    def test_no_merge_far_points(self):
        pts = [(0.0, 0.0), (100.0, 100.0), (200.0, 200.0)]
        result = cluster_intersections(pts, eps=10.0)
        self.assertEqual(len(result), 3)

    def test_single_point(self):
        pts = [(5.0, 5.0)]
        result = cluster_intersections(pts)
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0][0], 5.0, places=3)
        self.assertAlmostEqual(result[0][1], 5.0, places=3)


if __name__ == '__main__':
    unittest.main()
