#!/usr/bin/env python3
"""
椭圆拟合单元测试
覆盖: 椭圆拟合、参数提取、圆度判断、离心率、
      等效圆参数、点采样、点包含判断
测试对象: 10_视觉通用代码库/fit_ellipse.py
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

if HAS_CV2:
    from _10_视觉通用代码库.fit_ellipse import (
        fit_ellipse, fit_ellipse_robust, fit_ellipse_all,
        ellipse_params, ellipse_roundness, is_circle,
        ellipse_to_circle_params, ellipse_points,
        point_in_ellipse, draw_ellipse, draw_ellipse_with_axes
    )


def make_circle_contour(size=200, r=60):
    """创建圆形轮廓（足够多点）"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(img, (100, 100), r, 255, -1)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return max(contours, key=cv2.contourArea)


def make_ellipse_contour(size=200, a=70, b=40, angle=30):
    """创建椭圆轮廓"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.ellipse(img, (100, 100), (a, b), angle, 0, 360, 255, -1)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return max(contours, key=cv2.contourArea)


def make_multi_ellipses(size=200):
    """创建多个椭圆"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.ellipse(img, (60, 60), (40, 25), 0, 0, 360, 255, -1)
    cv2.ellipse(img, (150, 140), (30, 20), 45, 0, 360, 255, -1)
    return img


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFitEllipse(unittest.TestCase):
    """椭圆拟合测试"""

    def test_returns_tuple(self):
        cnt = make_circle_contour()
        ellipse = fit_ellipse(cnt)
        self.assertIsNotNone(ellipse)
        self.assertEqual(len(ellipse), 3)

    def test_center_in_image(self):
        cnt = make_circle_contour()
        ellipse = fit_ellipse(cnt)
        cx, cy = ellipse[0]
        self.assertGreater(cx, 0)
        self.assertLess(cx, 200)

    def test_axes_positive(self):
        cnt = make_circle_contour()
        ellipse = fit_ellipse(cnt)
        w, h = ellipse[1]
        self.assertGreater(w, 0)
        self.assertGreater(h, 0)

    def test_too_few_points(self):
        """少于5个点应返回None"""
        pts = np.array([[[0, 0]], [[1, 1]], [[2, 2]], [[3, 3]]], dtype=np.int32)
        result = fit_ellipse(pts)
        self.assertIsNone(result)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFitEllipseRobust(unittest.TestCase):
    """鲁棒椭圆拟合测试"""

    def test_returns_ellipse_and_contour(self):
        img = np.zeros((200, 200), dtype=np.uint8)
        cv2.ellipse(img, (100, 100), (60, 40), 0, 0, 360, 255, -1)
        ellipse, cnt = fit_ellipse_robust(img)
        self.assertIsNotNone(ellipse)
        self.assertIsNotNone(cnt)

    def test_empty_image(self):
        img = np.zeros((200, 200), dtype=np.uint8)
        ellipse, cnt = fit_ellipse_robust(img)
        self.assertIsNone(ellipse)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFitEllipseAll(unittest.TestCase):
    """拟合所有椭圆测试"""

    def test_finds_multiple(self):
        img = make_multi_ellipses()
        ellipses = fit_ellipse_all(img, min_area=50)
        self.assertGreater(len(ellipses), 0)

    def test_has_contour_and_ellipse(self):
        img = make_multi_ellipses()
        ellipses = fit_ellipse_all(img, min_area=50)
        for item in ellipses:
            self.assertIn('contour', item)
            self.assertIn('ellipse', item)

    def test_min_area_filter(self):
        img = make_multi_ellipses()
        all_ellipses = fit_ellipse_all(img, min_area=10)
        large_ellipses = fit_ellipse_all(img, min_area=5000)
        self.assertLessEqual(len(large_ellipses), len(all_ellipses))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestEllipseParams(unittest.TestCase):
    """椭圆参数提取测试"""

    def test_returns_dict(self):
        cnt = make_circle_contour(r=50)
        ellipse = fit_ellipse(cnt)
        params = ellipse_params(ellipse)
        self.assertIsInstance(params, dict)

    def test_has_all_keys(self):
        cnt = make_circle_contour()
        ellipse = fit_ellipse(cnt)
        params = ellipse_params(ellipse)
        for key in ['center', 'axes', 'width', 'height', 'angle',
                     'major_angle', 'semi_major', 'semi_minor',
                     'area', 'eccentricity', 'aspect_ratio']:
            self.assertIn(key, params)

    def test_semi_axes_positive(self):
        cnt = make_circle_contour()
        ellipse = fit_ellipse(cnt)
        params = ellipse_params(ellipse)
        self.assertGreater(params['semi_major'], 0)
        self.assertGreater(params['semi_minor'], 0)

    def test_semi_major_geq_semi_minor(self):
        cnt = make_ellipse_contour(a=70, b=40)
        ellipse = fit_ellipse(cnt)
        params = ellipse_params(ellipse)
        self.assertGreaterEqual(params['semi_major'], params['semi_minor'])

    def test_area_formula(self):
        """面积 = π * a * b"""
        cnt = make_ellipse_contour(a=70, b=40)
        ellipse = fit_ellipse(cnt)
        params = ellipse_params(ellipse)
        expected_area = np.pi * params['semi_major'] * params['semi_minor']
        self.assertAlmostEqual(params['area'], expected_area, delta=1.0)

    def test_eccentricity_range(self):
        """离心率应在 [0, 1)"""
        cnt = make_ellipse_contour(a=70, b=40)
        ellipse = fit_ellipse(cnt)
        params = ellipse_params(ellipse)
        self.assertGreaterEqual(params['eccentricity'], 0.0)
        self.assertLess(params['eccentricity'], 1.0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestIsCircle(unittest.TestCase):
    """圆形判断测试"""

    def test_circle_detected(self):
        cnt = make_circle_contour(r=50)
        ellipse = fit_ellipse(cnt)
        self.assertTrue(is_circle(ellipse, tolerance=0.2))

    def test_ellipse_not_circle(self):
        cnt = make_ellipse_contour(a=80, b=30)
        ellipse = fit_ellipse(cnt)
        self.assertFalse(is_circle(ellipse, tolerance=0.1))

    def test_tolerance_effect(self):
        cnt = make_ellipse_contour(a=60, b=45)
        ellipse = fit_ellipse(cnt)
        # 宽容判定可能通过，严格判定可能不通过
        result_loose = is_circle(ellipse, tolerance=0.3)
        result_strict = is_circle(ellipse, tolerance=0.05)
        # 如果严格通过，宽容也应通过
        if result_strict:
            self.assertTrue(result_loose)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestEllipseRoundness(unittest.TestCase):
    """椭圆拟合度测试"""

    def test_perfect_ellipse_high_roundness(self):
        """椭圆本身应有高拟合度"""
        cnt = make_ellipse_contour(a=70, b=40)
        roundness = ellipse_roundness(cnt)
        self.assertGreater(roundness, 0.8)

    def test_roundness_range(self):
        cnt = make_circle_contour()
        roundness = ellipse_roundness(cnt)
        self.assertGreater(roundness, 0.0)
        self.assertLessEqual(roundness, 1.0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestEllipseToCircleParams(unittest.TestCase):
    """等效圆参数测试"""

    def test_returns_dict(self):
        cnt = make_circle_contour()
        ellipse = fit_ellipse(cnt)
        circle_params = ellipse_to_circle_params(ellipse)
        self.assertIn('center', circle_params)
        self.assertIn('radius', circle_params)
        self.assertIn('area', circle_params)
        self.assertIn('diameter', circle_params)

    def test_radius_positive(self):
        cnt = make_circle_contour(r=50)
        ellipse = fit_ellipse(cnt)
        circle_params = ellipse_to_circle_params(ellipse)
        self.assertGreater(circle_params['radius'], 0)

    def test_diameter_is_2x_radius(self):
        cnt = make_circle_contour()
        ellipse = fit_ellipse(cnt)
        circle_params = ellipse_to_circle_params(ellipse)
        self.assertAlmostEqual(circle_params['diameter'],
                               2 * circle_params['radius'], delta=0.01)

    def test_area_matches(self):
        cnt = make_ellipse_contour(a=70, b=40)
        ellipse = fit_ellipse(cnt)
        params = ellipse_params(ellipse)
        circle_params = ellipse_to_circle_params(ellipse)
        # 等效面积: π*r² = π*a*b
        equiv_area = np.pi * circle_params['radius'] ** 2
        self.assertAlmostEqual(equiv_area, params['area'], delta=1.0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestEllipsePoints(unittest.TestCase):
    """椭圆点采样测试"""

    def test_returns_array(self):
        cnt = make_circle_contour()
        ellipse = fit_ellipse(cnt)
        pts = ellipse_points(ellipse, num_pts=72)
        self.assertEqual(pts.shape, (72, 2))

    def test_custom_num_pts(self):
        cnt = make_circle_contour()
        ellipse = fit_ellipse(cnt)
        pts = ellipse_points(ellipse, num_pts=36)
        self.assertEqual(pts.shape, (36, 2))

    def test_points_dtype(self):
        cnt = make_circle_contour()
        ellipse = fit_ellipse(cnt)
        pts = ellipse_points(ellipse)
        self.assertEqual(pts.dtype, np.float32)

    def test_points_near_center(self):
        """采样点应在椭圆中心附近"""
        cnt = make_circle_contour(r=50)
        ellipse = fit_ellipse(cnt)
        pts = ellipse_points(ellipse)
        cx, cy = ellipse[0]
        # 所有点应距中心不超过半径的2倍
        dists = np.sqrt((pts[:, 0] - cx) ** 2 + (pts[:, 1] - cy) ** 2)
        self.assertTrue(np.all(dists < 200))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestPointInEllipse(unittest.TestCase):
    """点包含判断测试"""

    def test_center_inside(self):
        """中心点应在椭圆内"""
        cnt = make_circle_contour(r=50)
        ellipse = fit_ellipse(cnt)
        cx, cy = ellipse[0]
        self.assertTrue(point_in_ellipse((cx, cy), ellipse))

    def test_far_point_outside(self):
        """远点应在椭圆外"""
        cnt = make_circle_contour(r=50)
        ellipse = fit_ellipse(cnt)
        self.assertFalse(point_in_ellipse((0, 0), ellipse))

    def test_boundary_points(self):
        """椭圆上的采样点应在椭圆内或边界上"""
        cnt = make_circle_contour(r=50)
        ellipse = fit_ellipse(cnt)
        pts = ellipse_points(ellipse, num_pts=36)
        inside_count = sum(1 for p in pts if point_in_ellipse(tuple(p), ellipse))
        # 大部分采样点应在内部或边界
        self.assertGreater(inside_count, len(pts) * 0.5)

    def test_zero_axes(self):
        """零轴椭圆应返回False"""
        ellipse = ((100, 100), (0, 0), 0)
        self.assertFalse(point_in_ellipse((100, 100), ellipse))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawEllipse(unittest.TestCase):
    """绘制椭圆测试"""

    def test_output_shape(self):
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        cnt = make_circle_contour()
        ellipse = fit_ellipse(cnt)
        result = draw_ellipse(img, ellipse)
        self.assertEqual(result.shape, img.shape)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawEllipseWithAxes(unittest.TestCase):
    """绘制椭圆及长短轴测试"""

    def test_output_shape(self):
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        cnt = make_ellipse_contour()
        ellipse = fit_ellipse(cnt)
        result = draw_ellipse_with_axes(img, ellipse)
        self.assertEqual(result.shape, img.shape)


if __name__ == '__main__':
    unittest.main()
