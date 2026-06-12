#!/usr/bin/env python3
"""
圆拟合单元测试
覆盖: 最小二乘法拟合、代数方法拟合、RANSAC圆拟合、
      噪声鲁棒性、离群点鲁棒性、边界条件
测试对象: 10_视觉通用代码库/circle_fit.py
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
    from _10_视觉通用代码库.circle_fit import (
        fit_circle_least_squares,
        fit_circle_algebraic,
        fit_circle_ransac,
        detect_circles_hough,
        detect_circle_contour,
        fit_ellipse_to_points,
    )


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFitCircleLeastSquares(unittest.TestCase):
    """最小二乘法圆拟合测试"""

    def test_perfect_circle(self):
        """完美圆上的点应精确拟合"""
        true_cx, true_cy, true_r = 100, 150, 50
        angles = np.linspace(0, 2 * np.pi, 36, endpoint=False)
        pts = np.column_stack([
            true_cx + true_r * np.cos(angles),
            true_cy + true_r * np.sin(angles)
        ])
        cx, cy, r = fit_circle_least_squares(pts)
        self.assertAlmostEqual(cx, true_cx, places=1)
        self.assertAlmostEqual(cy, true_cy, places=1)
        self.assertAlmostEqual(r, true_r, places=1)

    def test_noisy_circle(self):
        """带噪声的圆拟合应接近真实值"""
        np.random.seed(42)
        true_cx, true_cy, true_r = 100, 150, 50
        angles = np.random.uniform(0, 2 * np.pi, 50)
        noise = np.random.normal(0, 1, 50)
        pts = np.column_stack([
            true_cx + (true_r + noise) * np.cos(angles),
            true_cy + (true_r + noise) * np.sin(angles)
        ])
        cx, cy, r = fit_circle_least_squares(pts)
        self.assertAlmostEqual(cx, true_cx, delta=5)
        self.assertAlmostEqual(cy, true_cy, delta=5)
        self.assertAlmostEqual(r, true_r, delta=5)

    def test_returns_tuple(self):
        angles = np.linspace(0, 2 * np.pi, 20, endpoint=False)
        pts = np.column_stack([10 + 5 * np.cos(angles), 10 + 5 * np.sin(angles)])
        result = fit_circle_least_squares(pts)
        self.assertEqual(len(result), 3)
        self.assertIsInstance(result[0], float)

    def test_unit_circle(self):
        """单位圆拟合"""
        angles = np.linspace(0, 2 * np.pi, 30, endpoint=False)
        pts = np.column_stack([np.cos(angles), np.sin(angles)])
        cx, cy, r = fit_circle_least_squares(pts)
        self.assertAlmostEqual(cx, 0.0, places=1)
        self.assertAlmostEqual(cy, 0.0, places=1)
        self.assertAlmostEqual(r, 1.0, places=1)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFitCircleAlgebraic(unittest.TestCase):
    """代数方法圆拟合测试"""

    def test_perfect_circle(self):
        true_cx, true_cy, true_r = 50, 60, 30
        angles = np.linspace(0, 2 * np.pi, 36, endpoint=False)
        pts = np.column_stack([
            true_cx + true_r * np.cos(angles),
            true_cy + true_r * np.sin(angles)
        ])
        cx, cy, r = fit_circle_algebraic(pts)
        self.assertAlmostEqual(cx, true_cx, places=1)
        self.assertAlmostEqual(cy, true_cy, places=1)
        self.assertAlmostEqual(r, true_r, places=1)

    def test_consistency_with_least_squares(self):
        """代数方法和最小二乘应给出类似结果"""
        angles = np.linspace(0, 2 * np.pi, 30, endpoint=False)
        pts = np.column_stack([100 + 50 * np.cos(angles), 100 + 50 * np.sin(angles)])
        cx1, cy1, r1 = fit_circle_least_squares(pts)
        cx2, cy2, r2 = fit_circle_algebraic(pts)
        self.assertAlmostEqual(r1, r2, delta=2)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFitCircleRANSAC(unittest.TestCase):
    """RANSAC圆拟合测试"""

    def test_clean_data(self):
        """无离群点数据"""
        np.random.seed(42)
        true_cx, true_cy, true_r = 100, 150, 50
        angles = np.random.uniform(0, 2 * np.pi, 50)
        noise = np.random.normal(0, 0.5, 50)
        pts = np.column_stack([
            true_cx + (true_r + noise) * np.cos(angles),
            true_cy + (true_r + noise) * np.sin(angles)
        ])
        cx, cy, r, mask = fit_circle_ransac(pts, n_iterations=500, inlier_threshold=3.0)
        self.assertAlmostEqual(cx, true_cx, delta=5)
        self.assertAlmostEqual(cy, true_cy, delta=5)
        self.assertAlmostEqual(r, true_r, delta=5)

    def test_with_outliers(self):
        """有离群点时应鲁棒"""
        np.random.seed(42)
        true_cx, true_cy, true_r = 100, 150, 50
        angles = np.random.uniform(0, 2 * np.pi, 50)
        noise = np.random.normal(0, 0.5, 50)
        pts = np.column_stack([
            true_cx + (true_r + noise) * np.cos(angles),
            true_cy + (true_r + noise) * np.sin(angles)
        ])
        # 添加离群点
        outliers = np.array([[0, 0], [500, 500], [10, 300]])
        pts_with_outliers = np.vstack([pts, outliers])

        cx, cy, r, mask = fit_circle_ransac(pts_with_outliers, n_iterations=1000, inlier_threshold=3.0)
        self.assertAlmostEqual(cx, true_cx, delta=10)
        self.assertAlmostEqual(cy, true_cy, delta=10)
        self.assertAlmostEqual(r, true_r, delta=10)

    def test_inlier_mask(self):
        """内点掩码应正确"""
        np.random.seed(42)
        true_cx, true_cy, true_r = 100, 150, 50
        angles = np.random.uniform(0, 2 * np.pi, 30)
        pts = np.column_stack([
            true_cx + true_r * np.cos(angles),
            true_cy + true_r * np.sin(angles)
        ])
        outliers = np.array([[0, 0], [500, 500]])
        pts_all = np.vstack([pts, outliers])
        _, _, _, mask = fit_circle_ransac(pts_all, n_iterations=500, inlier_threshold=5.0)
        self.assertEqual(len(mask), len(pts_all))
        self.assertTrue(mask.dtype == bool)

    def test_too_few_points_raises(self):
        """少于3个点应抛出异常"""
        pts = np.array([[0, 0], [1, 1]])
        with self.assertRaises(ValueError):
            fit_circle_ransac(pts)

    def test_returns_tuple_of_4(self):
        angles = np.linspace(0, 2 * np.pi, 20, endpoint=False)
        pts = np.column_stack([10 + 5 * np.cos(angles), 10 + 5 * np.sin(angles)])
        result = fit_circle_ransac(pts, n_iterations=100)
        self.assertEqual(len(result), 4)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDetectCircleContour(unittest.TestCase):
    """轮廓圆检测测试"""

    def test_detect_circle_in_image(self):
        """应能检测到绘制的圆"""
        img = np.zeros((400, 400), dtype=np.uint8)
        cv2.circle(img, (200, 200), 80, 255, -1)
        circles = detect_circle_contour(img, min_area=1000, circularity_threshold=0.7)
        self.assertGreater(len(circles), 0)

    def test_no_circle_in_empty_image(self):
        img = np.zeros((400, 400), dtype=np.uint8)
        circles = detect_circle_contour(img)
        self.assertEqual(len(circles), 0)

    def test_rect_filtered_out(self):
        """矩形不应被检测为圆"""
        img = np.zeros((400, 400), dtype=np.uint8)
        cv2.rectangle(img, (50, 50), (350, 100), 255, -1)
        circles = detect_circle_contour(img, min_area=500, circularity_threshold=0.8)
        self.assertEqual(len(circles), 0)

    def test_returns_list_of_tuples(self):
        img = np.zeros((400, 400), dtype=np.uint8)
        cv2.circle(img, (200, 200), 80, 255, -1)
        circles = detect_circle_contour(img, min_area=100, circularity_threshold=0.5)
        if circles:
            self.assertEqual(len(circles[0]), 3)  # (cx, cy, r)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFitEllipseToPoints(unittest.TestCase):
    """椭圆拟合测试"""

    def test_perfect_ellipse(self):
        """完美椭圆上的点"""
        pts = []
        for angle in np.linspace(0, 2 * np.pi, 50, endpoint=False):
            x = 100 + 50 * np.cos(angle)
            y = 100 + 30 * np.sin(angle)
            pts.append([x, y])
        pts = np.array(pts)
        ellipse = fit_ellipse_to_points(pts)
        center, axes, angle = ellipse
        self.assertAlmostEqual(center[0], 100, delta=5)
        self.assertAlmostEqual(center[1], 100, delta=5)

    def test_too_few_points_raises(self):
        pts = np.array([[0, 0], [1, 1], [2, 2], [3, 3]])
        with self.assertRaises(ValueError):
            fit_ellipse_to_points(pts)


if __name__ == '__main__':
    unittest.main()
