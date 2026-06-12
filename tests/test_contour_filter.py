#!/usr/bin/env python3
"""
轮廓过滤单元测试
覆盖: 面积过滤、圆度过滤、长宽比过滤、凸性过滤、
      周长过滤、填充率过滤、顶点数过滤、多条件组合、
      排序、提取最大/最小/最圆、特征分析
测试对象: 10_视觉通用代码库/contour_filter.py
"""

import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

if HAS_CV2:
    from _10_视觉通用代码库.contour_filter import ContourFilter


def get_contour(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea) if contours else None


def make_circle(cx=200, cy=200, r=80):
    img = np.zeros((400, 400, 3), dtype=np.uint8)
    cv2.circle(img, (cx, cy), r, (255, 255, 255), -1)
    return get_contour(img)


def make_rect(x=50, y=50, w=200, h=100):
    img = np.zeros((400, 400, 3), dtype=np.uint8)
    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 255), -1)
    return get_contour(img)


def make_triangle():
    img = np.zeros((400, 400, 3), dtype=np.uint8)
    pts = np.array([[200, 50], [100, 300], [300, 300]])
    cv2.fillPoly(img, [pts], (255, 255, 255))
    return get_contour(img)


def make_small_circle():
    return make_circle(cx=200, cy=200, r=3)


def make_multiple_contours():
    """创建多个不同大小的轮廓"""
    img = np.zeros((400, 400, 3), dtype=np.uint8)
    cv2.circle(img, (100, 100), 50, (255, 255, 255), -1)   # 小圆
    cv2.circle(img, (300, 300), 80, (255, 255, 255), -1)   # 大圆
    cv2.rectangle(img, (50, 200), (150, 350), (255, 255, 255), -1)  # 矩形
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterByArea(unittest.TestCase):
    def test_filters_small(self):
        c = make_small_circle()
        result = ContourFilter.filter_by_area([c], min_area=1000)
        self.assertEqual(len(result), 0)

    def test_keeps_large(self):
        c = make_circle(r=80)
        result = ContourFilter.filter_by_area([c], min_area=100)
        self.assertEqual(len(result), 1)

    def test_max_area(self):
        c = make_circle(r=80)
        result = ContourFilter.filter_by_area([c], max_area=10)
        self.assertEqual(len(result), 0)

    def test_range(self):
        c = make_circle(r=80)
        area = cv2.contourArea(c)
        result = ContourFilter.filter_by_area([c], min_area=area * 0.9, max_area=area * 1.1)
        self.assertEqual(len(result), 1)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterByCircularity(unittest.TestCase):
    def test_circle_passes(self):
        c = make_circle(r=80)
        result = ContourFilter.filter_by_circularity([c], min_circ=0.8)
        self.assertEqual(len(result), 1)

    def test_rect_filtered(self):
        c = make_rect(w=300, h=10)
        result = ContourFilter.filter_by_circularity([c], min_circ=0.8)
        self.assertEqual(len(result), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterByAspectRatio(unittest.TestCase):
    def test_square_passes(self):
        c = make_rect(w=100, h=100)
        result = ContourFilter.filter_by_aspect_ratio([c], min_ratio=0.9, max_ratio=1.1)
        self.assertEqual(len(result), 1)

    def test_long_rect_filtered(self):
        c = make_rect(w=300, h=10)
        result = ContourFilter.filter_by_aspect_ratio([c], min_ratio=0.9, max_ratio=1.1)
        self.assertEqual(len(result), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterBySolidity(unittest.TestCase):
    def test_convex_passes(self):
        c = make_circle(r=80)
        result = ContourFilter.filter_by_solidity([c], min_solidity=0.9)
        self.assertEqual(len(result), 1)

    def test_triangle_passes(self):
        c = make_triangle()
        result = ContourFilter.filter_by_solidity([c], min_solidity=0.9)
        self.assertEqual(len(result), 1)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterByPerimeter(unittest.TestCase):
    def test_keeps_large_perimeter(self):
        c = make_circle(r=80)
        peri = cv2.arcLength(c, True)
        result = ContourFilter.filter_by_perimeter([c], min_peri=peri * 0.9)
        self.assertEqual(len(result), 1)

    def test_filters_small_perimeter(self):
        c = make_small_circle()
        result = ContourFilter.filter_by_perimeter([c], min_peri=1000)
        self.assertEqual(len(result), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterByExtent(unittest.TestCase):
    def test_rect_high_extent(self):
        c = make_rect(w=200, h=100)
        result = ContourFilter.filter_by_extent([c], min_extent=0.9)
        self.assertEqual(len(result), 1)

    def test_circle_lower_extent(self):
        c = make_circle(r=80)
        result = ContourFilter.filter_by_extent([c], min_extent=0.95)
        self.assertEqual(len(result), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterByVertices(unittest.TestCase):
    def test_circle_high_vertices(self):
        c = make_circle(r=80)
        result = ContourFilter.filter_by_vertices([c], min_vertices=8)
        self.assertEqual(len(result), 1)

    def test_triangle_3_vertices(self):
        c = make_triangle()
        result = ContourFilter.filter_by_vertices([c], min_vertices=3, max_vertices=4)
        self.assertEqual(len(result), 1)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterMulti(unittest.TestCase):
    def test_combined_filter(self):
        contours = make_multiple_contours()
        result = ContourFilter.filter_multi(contours, min_area=500, min_circularity=0.5)
        self.assertIsInstance(result, list)
        self.assertLessEqual(len(result), len(contours))

    def test_no_filter_returns_all(self):
        contours = make_multiple_contours()
        result = ContourFilter.filter_multi(contours)
        self.assertEqual(len(result), len(contours))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestSortByArea(unittest.TestCase):
    def test_reverse_sort(self):
        contours = make_multiple_contours()
        sorted_c = ContourFilter.sort_by_area(contours, reverse=True)
        areas = [cv2.contourArea(c) for c in sorted_c]
        self.assertEqual(areas, sorted(areas, reverse=True))

    def test_forward_sort(self):
        contours = make_multiple_contours()
        sorted_c = ContourFilter.sort_by_area(contours, reverse=False)
        areas = [cv2.contourArea(c) for c in sorted_c]
        self.assertEqual(areas, sorted(areas))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestSortByPosition(unittest.TestCase):
    def test_sort_by_x(self):
        contours = make_multiple_contours()
        if len(contours) >= 2:
            sorted_c = ContourFilter.sort_by_position(contours, axis='x')
            self.assertEqual(len(sorted_c), len(contours))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGetLargest(unittest.TestCase):
    def test_returns_1(self):
        contours = make_multiple_contours()
        result = ContourFilter.get_largest(contours, n=1)
        self.assertEqual(len(result), 1)

    def test_largest_is_actually_largest(self):
        contours = make_multiple_contours()
        result = ContourFilter.get_largest(contours, n=1)
        max_area = max(cv2.contourArea(c) for c in contours)
        self.assertAlmostEqual(cv2.contourArea(result[0]), max_area, delta=1)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGetMostCircular(unittest.TestCase):
    def test_returns_1(self):
        contours = make_multiple_contours()
        result = ContourFilter.get_most_circular(contours, n=1)
        self.assertEqual(len(result), 1)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestAnalyzeContour(unittest.TestCase):
    def test_returns_all_keys(self):
        c = make_circle(r=80)
        info = ContourFilter.analyze_contour(c)
        for key in ['area', 'perimeter', 'circularity', 'aspect_ratio',
                     'solidity', 'extent', 'bounding_rect', 'center',
                     'enclosing_radius', 'n_vertices', 'ellipse']:
            self.assertIn(key, info)

    def test_area_correct(self):
        c = make_circle(r=80)
        info = ContourFilter.analyze_contour(c)
        expected = math.pi * 80 * 80
        self.assertAlmostEqual(info['area'], expected, delta=expected * 0.05)

    def test_circularity_near_one(self):
        c = make_circle(r=80)
        info = ContourFilter.analyze_contour(c)
        self.assertGreater(info['circularity'], 0.85)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawFiltered(unittest.TestCase):
    def test_output_shape(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        c = make_circle(r=80)
        result = ContourFilter.draw_filtered(img, [c], [c])
        self.assertEqual(result.shape, img.shape)


if __name__ == '__main__':
    unittest.main()
