#!/usr/bin/env python3
"""
多边形检测单元测试
覆盖: 边长计算、内角计算、规则度计算、形状识别、
      多边形检测、绘制
测试对象: 10_视觉通用代码库/polygon_detector.py
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import numpy as np
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

if HAS_CV2:
    from _10_视觉通用代码库.polygon_detector import (
        compute_side_lengths,
        compute_interior_angles,
        compute_regularity,
        identify_shape,
        detect_polygons,
        draw_polygons,
        PolygonInfo,
    )


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestComputeSideLengths(unittest.TestCase):
    """边长计算测试"""

    def test_square(self):
        """正方形边长应相等"""
        vertices = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float64)
        sides = compute_side_lengths(vertices)
        self.assertEqual(len(sides), 4)
        for s in sides:
            self.assertAlmostEqual(s, 10.0, places=1)

    def test_triangle(self):
        """三角形边长"""
        vertices = np.array([[0, 0], [10, 0], [5, 10]], dtype=np.float64)
        sides = compute_side_lengths(vertices)
        self.assertEqual(len(sides), 3)
        self.assertAlmostEqual(sides[0], 10.0, places=1)

    def test_equilateral_triangle(self):
        """等边三角形边长相等"""
        side = 10.0
        h = side * math.sqrt(3) / 2
        vertices = np.array([[0, 0], [side, 0], [side / 2, h]], dtype=np.float64)
        sides = compute_side_lengths(vertices)
        for s in sides:
            self.assertAlmostEqual(s, side, places=1)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestComputeInteriorAngles(unittest.TestCase):
    """内角计算测试"""

    def test_square_angles(self):
        """正方形各角应为 90°"""
        vertices = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float64)
        angles = compute_interior_angles(vertices)
        for a in angles:
            self.assertAlmostEqual(a, 90.0, places=1)

    def test_equilateral_triangle_angles(self):
        """等边三角形各角应为 60°"""
        side = 10.0
        h = side * math.sqrt(3) / 2
        vertices = np.array([[0, 0], [side, 0], [side / 2, h]], dtype=np.float64)
        angles = compute_interior_angles(vertices)
        for a in angles:
            self.assertAlmostEqual(a, 60.0, places=1)

    def test_angle_sum_triangle(self):
        """三角形内角和 180°"""
        vertices = np.array([[0, 0], [10, 0], [5, 8]], dtype=np.float64)
        angles = compute_interior_angles(vertices)
        self.assertAlmostEqual(sum(angles), 180.0, places=1)

    def test_angle_sum_quadrilateral(self):
        """四边形内角和 360°"""
        vertices = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float64)
        angles = compute_interior_angles(vertices)
        self.assertAlmostEqual(sum(angles), 360.0, places=1)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestComputeRegularity(unittest.TestCase):
    """规则度计算测试"""

    def test_regular_square(self):
        """正方形规则度应接近 1"""
        vertices = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float64)
        reg = compute_regularity(vertices)
        self.assertGreater(reg, 0.8)

    def test_irregular_polygon(self):
        """不规则多边形规则度应较低"""
        vertices = np.array([[0, 0], [10, 0], [15, 5], [3, 12]], dtype=np.float64)
        reg = compute_regularity(vertices)
        self.assertLess(reg, 0.9)

    def test_range(self):
        """规则度应在 0~1 范围内"""
        vertices = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float64)
        reg = compute_regularity(vertices)
        self.assertGreaterEqual(reg, 0.0)
        self.assertLessEqual(reg, 1.0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestIdentifyShape(unittest.TestCase):
    """形状识别测试"""

    def test_square(self):
        vertices = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float64)
        sides = compute_side_lengths(vertices)
        angles = compute_interior_angles(vertices)
        reg = compute_regularity(vertices)
        shape = identify_shape(4, reg, angles, sides)
        self.assertIn('正方形', shape)

    def test_rectangle(self):
        vertices = np.array([[0, 0], [20, 0], [20, 10], [0, 10]], dtype=np.float64)
        sides = compute_side_lengths(vertices)
        angles = compute_interior_angles(vertices)
        reg = compute_regularity(vertices)
        shape = identify_shape(4, reg, angles, sides)
        self.assertIn('矩形', shape)

    def test_equilateral_triangle(self):
        side = 10.0
        h = side * math.sqrt(3) / 2
        vertices = np.array([[0, 0], [side, 0], [side / 2, h]], dtype=np.float64)
        sides = compute_side_lengths(vertices)
        angles = compute_interior_angles(vertices)
        reg = compute_regularity(vertices)
        shape = identify_shape(3, reg, angles, sides)
        self.assertIn('等边三角形', shape)

    def test_pentagon(self):
        shape = identify_shape(5, 0.9, np.array([108]*5), np.array([10]*5))
        self.assertIn('五边形', shape)

    def test_hexagon(self):
        shape = identify_shape(6, 0.9, np.array([120]*6), np.array([10]*6))
        self.assertIn('六边形', shape)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDetectPolygons(unittest.TestCase):
    """多边形检测测试"""

    def test_detect_square(self):
        """应检测到正方形"""
        img = np.zeros((400, 400), dtype=np.uint8)
        cv2.rectangle(img, (100, 100), (200, 200), 255, -1)
        polygons = detect_polygons(img)
        self.assertGreater(len(polygons), 0)

    def test_detect_triangle(self):
        """应检测到三角形"""
        img = np.zeros((400, 400), dtype=np.uint8)
        pts = np.array([[200, 50], [100, 300], [300, 300]])
        cv2.fillPoly(img, [pts], 255)
        polygons = detect_polygons(img)
        self.assertGreater(len(polygons), 0)

    def test_detect_multiple_shapes(self):
        """应检测到多个形状"""
        img = np.zeros((400, 400), dtype=np.uint8)
        cv2.rectangle(img, (50, 50), (150, 150), 255, -1)
        pts = np.array([[300, 100], [250, 200], [350, 200]])
        cv2.fillPoly(img, [pts], 255)
        polygons = detect_polygons(img, min_area=200)
        self.assertGreaterEqual(len(polygons), 2)

    def test_empty_image(self):
        img = np.zeros((400, 400), dtype=np.uint8)
        polygons = detect_polygons(img)
        self.assertEqual(len(polygons), 0)

    def test_polygon_info_fields(self):
        """检测结果应包含所有字段"""
        img = np.zeros((400, 400), dtype=np.uint8)
        cv2.rectangle(img, (100, 100), (200, 200), 255, -1)
        polygons = detect_polygons(img)
        if polygons:
            p = polygons[0]
            self.assertTrue(hasattr(p, 'vertices'))
            self.assertTrue(hasattr(p, 'num_vertices'))
            self.assertTrue(hasattr(p, 'side_lengths'))
            self.assertTrue(hasattr(p, 'interior_angles'))
            self.assertTrue(hasattr(p, 'area'))
            self.assertTrue(hasattr(p, 'perimeter'))
            self.assertTrue(hasattr(p, 'centroid'))
            self.assertTrue(hasattr(p, 'shape_name'))

    def test_area_filter(self):
        """面积过滤"""
        img = np.zeros((400, 400), dtype=np.uint8)
        cv2.rectangle(img, (100, 100), (200, 200), 255, -1)  # 10000 像素
        polygons = detect_polygons(img, min_area=50000)  # 过大阈值
        self.assertEqual(len(polygons), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawPolygons(unittest.TestCase):
    """绘制多边形测试"""

    def test_output_shape(self):
        img = np.zeros((400, 400), dtype=np.uint8)
        cv2.rectangle(img, (100, 100), (200, 200), 255, -1)
        polygons = detect_polygons(img)
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        result = draw_polygons(img_color, polygons)
        self.assertEqual(result.shape, img_color.shape)

    def test_empty_polygons(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        result = draw_polygons(img, [])
        self.assertEqual(result.shape, img.shape)


if __name__ == '__main__':
    unittest.main()
