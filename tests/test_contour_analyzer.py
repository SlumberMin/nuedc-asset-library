#!/usr/bin/env python3
"""
轮廓分析模块单元测试
覆盖: 面积、周长、圆度、矩形度、长宽比、延展度、实心度、紧凑度、
      离心率、Hu矩、质心、边界框、全面分析、形状分类、特征筛选
"""

import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from _10_视觉通用代码库.contour_analyzer import ContourAnalyzer


def get_contour(img):
    """从二值图获取最大轮廓"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea) if contours else None


def make_circle_contour(cx=200, cy=200, r=80):
    img = np.zeros((400, 400, 3), dtype=np.uint8)
    cv2.circle(img, (cx, cy), r, (255, 255, 255), -1)
    return get_contour(img)


def make_rect_contour(x=50, y=50, w=200, h=100):
    img = np.zeros((400, 400, 3), dtype=np.uint8)
    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 255), -1)
    return get_contour(img)


def make_triangle_contour():
    img = np.zeros((400, 400, 3), dtype=np.uint8)
    pts = np.array([[200, 50], [100, 300], [300, 300]])
    cv2.fillPoly(img, [pts], (255, 255, 255))
    return get_contour(img)


class TestInit(unittest.TestCase):
    def test_default(self):
        a = ContourAnalyzer()
        self.assertEqual(a.min_area, 100)

    def test_custom(self):
        a = ContourAnalyzer(min_area=50)
        self.assertEqual(a.min_area, 50)


class TestFilterContours(unittest.TestCase):
    def test_filters_small(self):
        a = ContourAnalyzer(min_area=1000)
        c = make_circle_contour(r=5)  # very small
        if c is not None:
            result = a.filter_contours([c])
            self.assertEqual(len(result), 0)

    def test_keeps_large(self):
        a = ContourAnalyzer(min_area=100)
        c = make_circle_contour(r=80)
        result = a.filter_contours([c])
        self.assertEqual(len(result), 1)


class TestGetArea(unittest.TestCase):
    def test_circle_area(self):
        a = ContourAnalyzer()
        c = make_circle_contour(r=80)
        area = a.get_area(c)
        expected = math.pi * 80 * 80
        self.assertAlmostEqual(area, expected, delta=expected * 0.05)

    def test_rect_area(self):
        a = ContourAnalyzer()
        c = make_rect_contour(w=200, h=100)
        area = a.get_area(c)
        self.assertAlmostEqual(area, 200 * 100, delta=500)


class TestGetPerimeter(unittest.TestCase):
    def test_rect_perimeter(self):
        a = ContourAnalyzer()
        c = make_rect_contour(w=200, h=100)
        peri = a.get_perimeter(c)
        self.assertAlmostEqual(peri, 2 * (200 + 100), delta=10)


class TestCircularity(unittest.TestCase):
    def test_circle_close_to_one(self):
        a = ContourAnalyzer()
        c = make_circle_contour(r=80)
        circ = a.get_circularity(c)
        self.assertGreater(circ, 0.85)

    def test_rectangle_less_than_circle(self):
        a = ContourAnalyzer()
        c = make_rect_contour(w=300, h=10)
        circ = a.get_circularity(c)
        self.assertLess(circ, 0.6)

    def test_range(self):
        a = ContourAnalyzer()
        for c in [make_circle_contour(r=50), make_rect_contour(w=200, h=100)]:
            circ = a.get_circularity(c)
            self.assertGreaterEqual(circ, 0.0)
            self.assertLessEqual(circ, 1.01)


class TestRectangularity(unittest.TestCase):
    def test_rectangle_close_to_one(self):
        a = ContourAnalyzer()
        c = make_rect_contour(w=200, h=100)
        rect = a.get_rectangularity(c)
        self.assertGreater(rect, 0.9)

    def test_circle_less_than_one(self):
        a = ContourAnalyzer()
        c = make_circle_contour(r=80)
        rect = a.get_rectangularity(c)
        self.assertLess(rect, 0.9)


class TestAspectRatio(unittest.TestCase):
    def test_square(self):
        a = ContourAnalyzer()
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        cv2.rectangle(img, (100, 100), (300, 300), (255, 255, 255), -1)
        c = get_contour(img)
        ar = a.get_aspect_ratio(c)
        self.assertGreater(ar, 0.9)

    def test_long_rect(self):
        a = ContourAnalyzer()
        c = make_rect_contour(w=300, h=50)
        ar = a.get_aspect_ratio(c)
        self.assertLess(ar, 0.3)


class TestExtent(unittest.TestCase):
    def test_rectangle_high_extent(self):
        a = ContourAnalyzer()
        c = make_rect_contour(w=200, h=100)
        ext = a.get_extent(c)
        self.assertGreater(ext, 0.9)


class TestSolidity(unittest.TestCase):
    def test_convex_shape(self):
        a = ContourAnalyzer()
        c = make_circle_contour(r=80)
        sol = a.get_solidity(c)
        self.assertGreater(sol, 0.9)

    def test_triangle_solidity(self):
        a = ContourAnalyzer()
        c = make_triangle_contour()
        sol = a.get_solidity(c)
        self.assertGreater(sol, 0.9)  # triangle is convex


class TestCompactness(unittest.TestCase):
    def test_positive(self):
        a = ContourAnalyzer()
        c = make_circle_contour(r=50)
        comp = a.get_compactness(c)
        self.assertGreater(comp, 0)


class TestEccentricity(unittest.TestCase):
    def test_circle_near_zero(self):
        a = ContourAnalyzer()
        c = make_circle_contour(r=80)
        ecc = a.get_eccentricity(c)
        self.assertLess(ecc, 0.2)

    def test_ellipse_high(self):
        a = ContourAnalyzer()
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        cv2.ellipse(img, (200, 200), (150, 30), 0, 0, 360, (255, 255, 255), -1)
        c = get_contour(img)
        ecc = a.get_eccentricity(c)
        self.assertGreater(ecc, 0.8)


class TestHuMoments(unittest.TestCase):
    def test_returns_7(self):
        a = ContourAnalyzer()
        c = make_circle_contour(r=80)
        hu = a.get_hu_moments(c)
        self.assertEqual(len(hu), 7)


class TestCentroid(unittest.TestCase):
    def test_circle_centroid(self):
        a = ContourAnalyzer()
        c = make_circle_contour(cx=200, cy=200, r=80)
        cx, cy = a.get_centroid(c)
        self.assertAlmostEqual(cx, 200, delta=5)
        self.assertAlmostEqual(cy, 200, delta=5)


class TestBoundingInfo(unittest.TestCase):
    def test_keys(self):
        a = ContourAnalyzer()
        c = make_rect_contour(x=50, y=50, w=200, h=100)
        info = a.get_bounding_info(c)
        self.assertIn('bbox', info)
        self.assertIn('min_rect', info)
        self.assertIn('min_rect_area', info)
        self.assertIn('center', info)


class TestAnalyze(unittest.TestCase):
    def test_returns_all_keys(self):
        a = ContourAnalyzer()
        c = make_circle_contour(r=80)
        info = a.analyze(c)
        for key in ['area', 'perimeter', 'circularity', 'rectangularity',
                     'aspect_ratio', 'extent', 'solidity', 'compactness',
                     'eccentricity', 'centroid', 'bbox', 'min_rect',
                     'min_rect_area', 'center', 'hu_moments']:
            self.assertIn(key, info)


class TestClassifyShape(unittest.TestCase):
    def test_circle(self):
        a = ContourAnalyzer()
        c = make_circle_contour(r=80)
        shape = a.classify_shape(c)
        self.assertEqual(shape, 'circle')

    def test_rectangle(self):
        a = ContourAnalyzer()
        c = make_rect_contour(w=300, h=100)
        shape = a.classify_shape(c)
        self.assertIn(shape, ['rectangle', 'polygon_4'])

    def test_triangle(self):
        a = ContourAnalyzer()
        c = make_triangle_contour()
        shape = a.classify_shape(c)
        self.assertIn(shape, ['triangle', 'polygon_3'])


class TestCompareContours(unittest.TestCase):
    def test_same_shape(self):
        a = ContourAnalyzer()
        c1 = make_circle_contour(cx=200, cy=200, r=80)
        c2 = make_circle_contour(cx=200, cy=200, r=80)
        dist = a.compare_contours(c1, c2)
        self.assertLess(dist, 0.01)

    def test_different_shape(self):
        a = ContourAnalyzer()
        c1 = make_circle_contour(r=80)
        c2 = make_rect_contour(w=300, h=50)
        dist = a.compare_contours(c1, c2)
        self.assertGreater(dist, 0.01)


class TestAnalyzeAll(unittest.TestCase):
    def test_multiple(self):
        a = ContourAnalyzer()
        contours = [make_circle_contour(r=80), make_rect_contour(w=200, h=100)]
        results = a.analyze_all(contours)
        self.assertEqual(len(results), 2)
        for r in results:
            self.assertIn('area', r)


class TestFindByFeature(unittest.TestCase):
    def test_find_circles(self):
        a = ContourAnalyzer()
        contours = [make_circle_contour(r=80), make_rect_contour(w=300, h=10)]
        circles = a.find_by_feature(contours, 'circularity', min_val=0.8)
        self.assertEqual(len(circles), 1)

    def test_find_by_area(self):
        a = ContourAnalyzer()
        contours = [make_circle_contour(r=80), make_rect_contour(w=200, h=100)]
        big = a.find_by_feature(contours, 'area', min_val=10000)
        self.assertEqual(len(big), 2)


if __name__ == '__main__':
    unittest.main()
