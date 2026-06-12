#!/usr/bin/env python3
"""
轮廓操作单元测试 (image_contour.py)
覆盖: 查找轮廓 / 绘制轮廓 / 多边形近似 / 凸包 / 形状匹配 /
      几何特征 / 过滤轮廓 / 形状分类
测试对象: 10_视觉通用代码库/image_contour.py
"""
import sys
import os
import unittest
import math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

if HAS_CV2:
    from _10_视觉通用代码库.image_contour import (
        find_contours, draw_contours, approx_contour, convex_hull,
        match_shapes, match_shape_template, contour_features,
        filter_contours, classify_shape,
    )


def make_circle_contour(r=80, size=400):
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(img, (size // 2, size // 2), r, 255, -1)
    contours, _ = find_contours(img)
    return max(contours, key=cv2.contourArea) if contours else None


def make_rect_contour(w=200, h=100, size=400):
    img = np.zeros((size, size), dtype=np.uint8)
    x = (size - w) // 2
    y = (size - h) // 2
    cv2.rectangle(img, (x, y), (x + w, y + h), 255, -1)
    contours, _ = find_contours(img)
    return max(contours, key=cv2.contourArea) if contours else None


def make_triangle_contour(size=400):
    img = np.zeros((size, size), dtype=np.uint8)
    pts = np.array([[size // 2, 50], [80, size - 80], [size - 80, size - 80]])
    cv2.fillPoly(img, [pts], 255)
    contours, _ = find_contours(img)
    return max(contours, key=cv2.contourArea) if contours else None


def make_multi_contours(size=400):
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(img, (100, 100), 50, 255, -1)
    cv2.circle(img, (300, 300), 80, 255, -1)
    cv2.rectangle(img, (50, 200), (200, 350), 255, -1)
    contours, _ = find_contours(img)
    return contours


# ── 查找轮廓 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFindContours(unittest.TestCase):

    def test_finds_circle(self):
        cnt = make_circle_contour()
        self.assertIsNotNone(cnt)
        self.assertGreater(len(cnt), 10)

    def test_returns_hierarchy(self):
        img = np.zeros((200, 200), dtype=np.uint8)
        cv2.circle(img, (100, 100), 60, 255, -1)
        contours, hierarchy = find_contours(img)
        self.assertIsNotNone(hierarchy)

    def test_mode_external(self):
        img = np.zeros((200, 200), dtype=np.uint8)
        cv2.rectangle(img, (20, 20), (180, 180), 255, -1)
        cv2.rectangle(img, (50, 50), (150, 150), 0, -1)  # 内孔
        contours_ext, _ = find_contours(img, mode='external')
        contours_tree, _ = find_contours(img, mode='tree')
        self.assertLessEqual(len(contours_ext), len(contours_tree))

    def test_bgr_input(self):
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        cv2.circle(img, (100, 100), 60, (255, 255, 255), -1)
        contours, _ = find_contours(img)
        self.assertGreater(len(contours), 0)


# ── 绘制轮廓 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawContours(unittest.TestCase):

    def test_output_shape(self):
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        cnt = make_circle_contour(size=200)
        result = draw_contours(img, [cnt])
        self.assertEqual(result.shape, img.shape)

    def test_does_not_modify_original(self):
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        cnt = make_circle_contour(size=200)
        draw_contours(img, [cnt])
        self.assertEqual(np.sum(img), 0)


# ── 多边形近似 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestApproxContour(unittest.TestCase):

    def test_circle_many_vertices(self):
        cnt = make_circle_contour()
        approx = approx_contour(cnt, epsilon_ratio=0.02)
        self.assertGreater(len(approx), 6)

    def test_rect_four_vertices(self):
        cnt = make_rect_contour(w=200, h=100)
        approx = approx_contour(cnt, epsilon_ratio=0.04)
        self.assertEqual(len(approx), 4)

    def test_triangle_three_vertices(self):
        cnt = make_triangle_contour()
        approx = approx_contour(cnt, epsilon_ratio=0.04)
        self.assertEqual(len(approx), 3)


# ── 凸包 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestConvexHull(unittest.TestCase):

    def test_returns_array(self):
        cnt = make_circle_contour()
        hull = convex_hull(cnt)
        self.assertIsNotNone(hull)
        self.assertGreater(len(hull), 0)

    def test_hull_area_ge_contour_area(self):
        cnt = make_circle_contour()
        hull = convex_hull(cnt)
        self.assertGreaterEqual(cv2.contourArea(hull), cv2.contourArea(cnt) * 0.95)


# ── 形状匹配 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMatchShapes(unittest.TestCase):

    def test_identical_contour_zero_distance(self):
        cnt = make_circle_contour()
        score = match_shapes(cnt, cnt)
        self.assertAlmostEqual(score, 0.0, places=5)

    def test_different_shapes_positive(self):
        c1 = make_circle_contour()
        c2 = make_rect_contour()
        score = match_shapes(c1, c2)
        self.assertGreater(score, 0.0)

    def test_method_c(self):
        c1 = make_circle_contour()
        c2 = make_circle_contour(r=60)
        score = match_shapes(c1, c2, method='c')
        self.assertGreaterEqual(score, 0.0)


# ── 匹配模板轮廓 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMatchShapeTemplate(unittest.TestCase):

    def test_finds_best_match(self):
        contours = make_multi_contours()
        template = make_circle_contour()
        idx, score = match_shape_template(contours, template)
        self.assertGreaterEqual(idx, 0)
        self.assertGreaterEqual(score, 0.0)


# ── 几何特征 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestContourFeatures(unittest.TestCase):

    def test_circle_features(self):
        cnt = make_circle_contour(r=80)
        feat = contour_features(cnt)
        expected_area = math.pi * 80 * 80
        self.assertAlmostEqual(feat['area'], expected_area, delta=expected_area * 0.05)
        self.assertGreater(feat['perimeter'], 0)
        self.assertIn('center', feat)
        self.assertIn('bounding_rect', feat)
        self.assertIn('aspect_ratio', feat)
        self.assertIn('solidity', feat)

    def test_rect_features(self):
        cnt = make_rect_contour(w=200, h=100)
        feat = contour_features(cnt)
        self.assertAlmostEqual(feat['aspect_ratio'], 2.0, delta=0.1)
        self.assertGreater(feat['extent'], 0.8)

    def test_returns_all_keys(self):
        cnt = make_circle_contour()
        feat = contour_features(cnt)
        expected = ['area', 'perimeter', 'center', 'bounding_rect',
                    'aspect_ratio', 'extent', 'solidity',
                    'min_enclosing_radius', 'equivalent_diameter', 'fit_ellipse']
        for key in expected:
            self.assertIn(key, feat)


# ── 过滤轮廓 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterContours(unittest.TestCase):

    def test_filter_by_area(self):
        contours = make_multi_contours()
        filtered = filter_contours(contours, min_area=10000)
        self.assertLess(len(filtered), len(contours))

    def test_filter_by_solidity(self):
        contours = make_multi_contours()
        filtered = filter_contours(contours, min_solidity=0.9)
        self.assertIsInstance(filtered, list)

    def test_filter_by_aspect(self):
        contours = make_multi_contours()
        filtered = filter_contours(contours, min_aspect=0.8, max_aspect=1.2)
        self.assertIsInstance(filtered, list)

    def test_no_filter_returns_all(self):
        contours = make_multi_contours()
        filtered = filter_contours(contours)
        self.assertEqual(len(filtered), len(contours))


# ── 形状分类 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestClassifyShape(unittest.TestCase):

    def test_classify_triangle(self):
        cnt = make_triangle_contour()
        self.assertEqual(classify_shape(cnt), "triangle")

    def test_classify_rectangle(self):
        cnt = make_rect_contour(w=200, h=100)
        self.assertIn(classify_shape(cnt), ["rectangle", "square"])

    def test_classify_circle(self):
        cnt = make_circle_contour(r=80)
        self.assertEqual(classify_shape(cnt), "circle")


if __name__ == '__main__':
    unittest.main()
