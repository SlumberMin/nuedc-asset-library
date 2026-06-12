#!/usr/bin/env python3
"""
图像测量单元测试
覆盖: 比例尺校准、距离测量、角度测量、面积测量、轮廓测量、
      多点测量、可视化绘制、轮廓辅助
测试对象: 10_视觉通用代码库/image_measurement.py
"""

import sys
import os
import unittest
import numpy as np
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

if HAS_CV2:
    from _10_视觉通用代码库.image_measurement import (
        calibrate_scale,
        measure_distance_px,
        measure_distance,
        measure_distance_contour,
        measure_line_length,
        measure_angle,
        measure_angle_three_points,
        measure_angle_from_lines,
        measure_contour_area_px,
        measure_contour_area,
        measure_min_area_rect,
        measure_bounding_box,
        measure_circle,
        measure_polyline,
        measure_polygon,
        draw_measurement,
        draw_angle_arc,
        draw_scale_bar,
        find_measurement_contours,
        measure_all_contours,
    )


# ── 比例尺校准测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestCalibrateScale(unittest.TestCase):
    """比例尺校准测试"""

    def test_basic_calibration(self):
        """50mm / 200px = 0.25 mm/px"""
        scale = calibrate_scale(50, 200)
        self.assertAlmostEqual(scale, 0.25)

    def test_one_to_one(self):
        scale = calibrate_scale(100, 100)
        self.assertAlmostEqual(scale, 1.0)

    def test_zero_pixels_raises(self):
        with self.assertRaises(ValueError):
            calibrate_scale(50, 0)

    def test_negative_pixels_raises(self):
        with self.assertRaises(ValueError):
            calibrate_scale(50, -10)

    def test_small_object(self):
        scale = calibrate_scale(1, 1000)
        self.assertAlmostEqual(scale, 0.001)


# ── 距离测量测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMeasureDistance(unittest.TestCase):
    """距离测量测试"""

    def test_known_distance_px(self):
        """(0,0)到(3,4)的距离应为5"""
        d = measure_distance_px((0, 0), (3, 4))
        self.assertAlmostEqual(d, 5.0)

    def test_same_point_zero(self):
        d = measure_distance_px((100, 200), (100, 200))
        self.assertAlmostEqual(d, 0.0)

    def test_distance_with_scale(self):
        d = measure_distance((0, 0), (100, 0), scale=0.5)
        self.assertAlmostEqual(d, 50.0)

    def test_distance_scale_default(self):
        d = measure_distance((0, 0), (3, 4))
        self.assertAlmostEqual(d, 5.0)

    def test_contour_perimeter(self):
        """正方形轮廓周长"""
        contour = np.array([[0, 0], [100, 0], [100, 100], [0, 100]], dtype=np.int32)
        contour = contour.reshape(-1, 1, 2)
        p = measure_distance_contour(contour, scale=0.5)
        # 周长 = 4*100=400px, *0.5=200mm
        self.assertAlmostEqual(p, 200.0, delta=1.0)

    def test_line_length_with_draw(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        d = measure_line_length(img, (0, 0), (100, 0), scale=0.5, draw=True)
        self.assertAlmostEqual(d, 50.0)


# ── 角度测量测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMeasureAngle(unittest.TestCase):
    """角度测量测试"""

    def test_right_angle(self):
        """直角 = 90°"""
        angle = measure_angle((0, 100), (0, 0), (100, 0))
        self.assertAlmostEqual(angle, 90.0, delta=0.1)

    def test_straight_line(self):
        """直线 = 180°"""
        angle = measure_angle((-1, 0), (0, 0), (1, 0))
        self.assertAlmostEqual(angle, 180.0, delta=0.1)

    def test_zero_angle(self):
        """同方向 = 0°"""
        angle = measure_angle((1, 0), (0, 0), (2, 0))
        self.assertAlmostEqual(angle, 0.0, delta=0.1)

    def test_45_degree(self):
        """45度角"""
        angle = measure_angle((1, 0), (0, 0), (1, 1))
        self.assertAlmostEqual(angle, 45.0, delta=0.1)

    def test_angle_three_points_with_draw(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        angle = measure_angle_three_points((0, 100), (0, 0), (100, 0), img=img)
        self.assertAlmostEqual(angle, 90.0, delta=0.1)

    def test_angle_from_lines(self):
        """两条线段夹角"""
        line1 = ((0, 0), (100, 0))
        line2 = ((0, 0), (0, 100))
        angle = measure_angle_from_lines(line1, line2)
        self.assertAlmostEqual(angle, 90.0, delta=0.1)


# ── 面积测量测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMeasureArea(unittest.TestCase):
    """面积测量测试"""

    def test_square_area_px(self):
        """100x100正方形像素面积"""
        contour = np.array([[0, 0], [100, 0], [100, 100], [0, 100]], dtype=np.int32)
        contour = contour.reshape(-1, 1, 2)
        area = measure_contour_area_px(contour)
        self.assertAlmostEqual(area, 10000.0, delta=1.0)

    def test_area_with_scale(self):
        contour = np.array([[0, 0], [100, 0], [100, 100], [0, 100]], dtype=np.int32)
        contour = contour.reshape(-1, 1, 2)
        area = measure_contour_area(contour, scale=0.5)
        # 10000 * 0.25 = 2500
        self.assertAlmostEqual(area, 2500.0, delta=1.0)

    def test_min_area_rect(self):
        contour = np.array([[0, 0], [100, 0], [100, 50], [0, 50]], dtype=np.int32)
        contour = contour.reshape(-1, 1, 2)
        result = measure_min_area_rect(contour, scale=1.0)
        self.assertIn('center', result)
        self.assertIn('width_mm', result)
        self.assertIn('height_mm', result)
        self.assertIn('area_mm2', result)
        self.assertIn('angle', result)

    def test_bounding_box(self):
        contour = np.array([[10, 20], [110, 20], [110, 70], [10, 70]], dtype=np.int32)
        contour = contour.reshape(-1, 1, 2)
        result = measure_bounding_box(contour, scale=0.5)
        self.assertEqual(result['x'], 10)
        self.assertEqual(result['y'], 20)
        self.assertAlmostEqual(result['width_mm'], 50.0)
        self.assertAlmostEqual(result['height_mm'], 25.0)

    def test_circle_measurement(self):
        """拟合圆测量"""
        # 创建圆形轮廓
        angles = np.linspace(0, 2 * np.pi, 100)
        pts = np.array([[int(50 + 30 * np.cos(a)), int(50 + 30 * np.sin(a))] for a in angles])
        contour = pts.reshape(-1, 1, 2).astype(np.int32)
        result = measure_circle(contour, scale=0.5)
        self.assertIsNotNone(result)
        self.assertIn('radius_mm', result)
        self.assertIn('diameter_mm', result)
        self.assertIn('area_mm2', result)
        self.assertAlmostEqual(result['radius_mm'], 15.0, delta=1.0)

    def test_circle_too_few_points(self):
        """少于5个点应返回None"""
        contour = np.array([[0, 0], [1, 1], [2, 2]], dtype=np.int32).reshape(-1, 1, 2)
        result = measure_circle(contour)
        self.assertIsNone(result)


# ── 多点测量测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMeasurePolyline(unittest.TestCase):
    """折线测量测试"""

    def test_straight_line(self):
        pts = [(0, 0), (100, 0)]
        length = measure_polyline(pts, closed=False, scale=1.0)
        self.assertAlmostEqual(length, 100.0)

    def test_triangle_closed(self):
        """闭合三角形周长"""
        pts = [(0, 0), (100, 0), (50, 86.6)]
        length = measure_polyline(pts, closed=True, scale=1.0)
        # 三边各约100
        self.assertAlmostEqual(length, 300.0, delta=1.0)

    def test_with_scale(self):
        pts = [(0, 0), (200, 0)]
        length = measure_polyline(pts, closed=False, scale=0.25)
        self.assertAlmostEqual(length, 50.0)

    def test_polygon(self):
        """多边形面积和周长"""
        pts = [(0, 0), (100, 0), (100, 50), (0, 50)]
        result = measure_polygon(pts, scale=1.0)
        self.assertIn('area_mm2', result)
        self.assertIn('perimeter_mm', result)
        self.assertAlmostEqual(result['area_mm2'], 5000.0, delta=1.0)
        self.assertAlmostEqual(result['perimeter_mm'], 300.0, delta=1.0)


# ── 可视化绘制测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawMeasurement(unittest.TestCase):
    """测量可视化测试"""

    def test_draw_measurement_no_crash(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        result = draw_measurement(img, (10, 10), (100, 100), "50.0mm")
        self.assertEqual(result.shape, img.shape)

    def test_draw_angle_arc_no_crash(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        result = draw_angle_arc(img, (0, 100), (0, 0), (100, 0))
        self.assertEqual(result.shape, img.shape)

    def test_draw_scale_bar(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        result = draw_scale_bar(img, scale=0.5, bar_length_mm=10)
        self.assertEqual(result.shape, img.shape)


# ── 轮廓辅助测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFindMeasurementContours(unittest.TestCase):
    """测量轮廓查找测试"""

    def test_returns_list(self):
        img = np.ones((200, 200, 3), dtype=np.uint8) * 255
        contours = find_measurement_contours(img)
        self.assertIsInstance(contours, list)

    def test_with_objects(self):
        """有物体的图像应检测到轮廓"""
        img = np.ones((400, 400, 3), dtype=np.uint8) * 255
        cv2.rectangle(img, (50, 50), (150, 150), 0, -1)
        cv2.rectangle(img, (200, 200), (350, 350), 0, -1)
        contours = find_measurement_contours(img, min_area=100)
        self.assertGreater(len(contours), 0)

    def test_area_filter(self):
        """应按面积过滤"""
        img = np.ones((400, 400, 3), dtype=np.uint8) * 255
        cv2.rectangle(img, (50, 50), (150, 150), 0, -1)  # 10000 px area
        contours = find_measurement_contours(img, min_area=50000)
        self.assertEqual(len(contours), 0)


# ── 批量测量测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMeasureAllContours(unittest.TestCase):
    """批量测量测试"""

    def test_returns_list(self):
        img = np.ones((400, 400, 3), dtype=np.uint8) * 255
        cv2.rectangle(img, (50, 50), (150, 150), 0, -1)
        results = measure_all_contours(img, scale=0.5, min_area=100)
        self.assertIsInstance(results, list)

    def test_result_structure(self):
        img = np.ones((400, 400, 3), dtype=np.uint8) * 255
        cv2.rectangle(img, (50, 50), (150, 150), 0, -1)
        results = measure_all_contours(img, scale=0.5, min_area=100)
        if results:
            r = results[0]
            self.assertIn('index', r)
            self.assertIn('area_mm2', r)
            self.assertIn('perimeter_mm', r)
            self.assertIn('bounding_box', r)
            self.assertIn('min_rect', r)
            self.assertIn('circle', r)


if __name__ == '__main__':
    unittest.main()
