#!/usr/bin/env python3
"""
凸包计算单元测试
覆盖: 凸包计算、面积比、周长比、凸性判断、
      凸缺陷检测、缺陷过滤、手指计数
测试对象: 10_视觉通用代码库/convex_hull.py
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
    from _10_视觉通用代码库.convex_hull import (
        compute_convex_hull, convex_hull_from_mask,
        convex_hull_area_ratio, convex_hull_perimeter_ratio,
        is_convex, convexity_defects,
        filter_defects_by_depth, filter_defects_by_angle,
        count_fingers, draw_hull_and_defects
    )


def make_square_contour(size=200):
    """创建正方形轮廓"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.rectangle(img, (40, 40), (160, 160), 255, -1)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea)


def make_circle_contour(size=200, r=60):
    """创建圆形轮廓"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(img, (100, 100), r, 255, -1)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea)


def make_L_contour(size=200):
    """创建L形轮廓（非凸）"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.rectangle(img, (30, 30), (80, 170), 255, -1)
    cv2.rectangle(img, (30, 120), (170, 170), 255, -1)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea)


def make_star_contour(size=200, cx=100, cy=100, outer_r=70, inner_r=30, points=5):
    """创建星形轮廓（有凸缺陷）"""
    img = np.zeros((size, size), dtype=np.uint8)
    pts = []
    for i in range(points * 2):
        angle = np.pi / 2 + i * np.pi / points
        r = outer_r if i % 2 == 0 else inner_r
        pts.append([int(cx + r * np.cos(angle)), int(cy - r * np.sin(angle))])
    pts = np.array(pts, dtype=np.int32)
    cv2.fillPoly(img, [pts], 255)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestComputeConvexHull(unittest.TestCase):
    """凸包计算测试"""

    def test_returns_contour(self):
        cnt = make_square_contour()
        hull = compute_convex_hull(cnt)
        self.assertGreater(len(hull), 0)

    def test_hull_points(self):
        """正方形凸包应有4个顶点"""
        cnt = make_square_contour()
        hull = compute_convex_hull(cnt)
        self.assertEqual(len(hull), 4)

    def test_clockwise_option(self):
        cnt = make_square_contour()
        hull_cw = compute_convex_hull(cnt, clockwise=True)
        hull_ccw = compute_convex_hull(cnt, clockwise=False)
        self.assertEqual(len(hull_cw), len(hull_ccw))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestConvexHullFromMask(unittest.TestCase):
    """从mask计算凸包测试"""

    def test_returns_contour_and_hull(self):
        img = np.zeros((200, 200), dtype=np.uint8)
        cv2.circle(img, (100, 100), 50, 255, -1)
        cnt, hull = convex_hull_from_mask(img)
        self.assertIsNotNone(cnt)
        self.assertIsNotNone(hull)

    def test_empty_mask(self):
        img = np.zeros((200, 200), dtype=np.uint8)
        cnt, hull = convex_hull_from_mask(img)
        self.assertIsNone(cnt)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestConvexHullAreaRatio(unittest.TestCase):
    """凸包面积比（实心度）测试"""

    def test_convex_shape_ratio_near_one(self):
        """凸形状的面积比应接近1"""
        cnt = make_square_contour()
        ratio = convex_hull_area_ratio(cnt)
        self.assertAlmostEqual(ratio, 1.0, delta=0.05)

    def test_circle_high_ratio(self):
        cnt = make_circle_contour()
        ratio = convex_hull_area_ratio(cnt)
        self.assertGreater(ratio, 0.9)

    def test_L_shape_lower_ratio(self):
        """L形面积比应小于1"""
        cnt = make_L_contour()
        ratio = convex_hull_area_ratio(cnt)
        self.assertLess(ratio, 0.9)

    def test_ratio_range(self):
        cnt = make_L_contour()
        ratio = convex_hull_area_ratio(cnt)
        self.assertGreater(ratio, 0.0)
        self.assertLessEqual(ratio, 1.0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestConvexHullPerimeterRatio(unittest.TestCase):
    """凸包周长比测试"""

    def test_convex_perimeter_ratio(self):
        cnt = make_square_contour()
        ratio = convex_hull_perimeter_ratio(cnt)
        self.assertGreater(ratio, 0.9)

    def test_non_convex_perimeter_ratio(self):
        cnt = make_L_contour()
        ratio = convex_hull_perimeter_ratio(cnt)
        self.assertGreater(ratio, 1.0)  # 非凸轮廓周长 > 凸包周长


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestIsConvex(unittest.TestCase):
    """凸性判断测试"""

    def test_square_is_convex(self):
        cnt = make_square_contour()
        self.assertTrue(is_convex(cnt))

    def test_L_is_not_convex(self):
        cnt = make_L_contour()
        self.assertFalse(is_convex(cnt))

    def test_circle_is_convex(self):
        cnt = make_circle_contour()
        self.assertTrue(is_convex(cnt))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestConvexityDefects(unittest.TestCase):
    """凸缺陷检测测试"""

    def test_convex_no_defects(self):
        """凸形状无缺陷"""
        cnt = make_square_contour()
        defects = convexity_defects(cnt)
        self.assertEqual(len(defects), 0)

    def test_L_has_defects(self):
        """L形应有凸缺陷"""
        cnt = make_L_contour()
        defects = convexity_defects(cnt)
        self.assertGreater(len(defects), 0)

    def test_defect_has_keys(self):
        cnt = make_L_contour()
        defects = convexity_defects(cnt)
        if defects:
            d = defects[0]
            self.assertIn('start', d)
            self.assertIn('end', d)
            self.assertIn('far', d)
            self.assertIn('depth', d)

    def test_star_has_defects(self):
        """星形应有多个缺陷"""
        cnt = make_star_contour()
        defects = convexity_defects(cnt)
        self.assertGreater(len(defects), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterDefectsByDepth(unittest.TestCase):
    """按深度过滤缺陷测试"""

    def test_filters_shallow(self):
        cnt = make_L_contour()
        defects = convexity_defects(cnt)
        filtered = filter_defects_by_depth(defects, min_depth=10)
        self.assertLessEqual(len(filtered), len(defects))

    def test_all_pass_high_depth(self):
        """min_depth=0应全部通过"""
        cnt = make_L_contour()
        defects = convexity_defects(cnt)
        filtered = filter_defects_by_depth(defects, min_depth=0)
        self.assertEqual(len(filtered), len(defects))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterDefectsByAngle(unittest.TestCase):
    """按角度过滤缺陷测试"""

    def test_returns_list(self):
        cnt = make_L_contour()
        defects = convexity_defects(cnt)
        filtered = filter_defects_by_angle(defects, cnt, 20, 160)
        self.assertIsInstance(filtered, list)

    def test_angle_range(self):
        cnt = make_L_contour()
        defects = convexity_defects(cnt)
        filtered = filter_defects_by_angle(defects, cnt, 20, 160)
        for d in filtered:
            if 'angle' in d:
                self.assertGreaterEqual(d['angle'], 20)
                self.assertLessEqual(d['angle'], 160)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestCountFingers(unittest.TestCase):
    """手指计数测试"""

    def test_returns_count_and_list(self):
        cnt = make_star_contour()
        n_fingers, defects = count_fingers(cnt, min_depth=5)
        self.assertIsInstance(n_fingers, int)
        self.assertIsInstance(defects, list)

    def test_square_zero_fingers(self):
        """正方形无手指"""
        cnt = make_square_contour()
        n_fingers, _ = count_fingers(cnt, min_depth=20)
        self.assertEqual(n_fingers, 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawHullAndDefects(unittest.TestCase):
    """可视化测试"""

    def test_output_shape(self):
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        cnt = make_L_contour()
        result = draw_hull_and_defects(img, cnt)
        self.assertEqual(result.shape, img.shape)


if __name__ == '__main__':
    unittest.main()
