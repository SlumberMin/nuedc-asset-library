#!/usr/bin/env python3
"""
形状匹配单元测试
覆盖: Hu矩计算、Hu矩匹配、批量匹配、轮廓匹配、
      模板匹配(多种方法)、旋转不变匹配、形状分类、ORB匹配
测试对象: 10_视觉通用代码库/shape_matcher.py
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

if HAS_CV2:
    from _10_视觉通用代码库.shape_matcher import ShapeMatcher


def get_contour(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea) if contours else None


def make_circle_contour(r=80):
    img = np.zeros((400, 400), dtype=np.uint8)
    cv2.circle(img, (200, 200), r, 255, -1)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea) if contours else None


def make_rect_contour(w=200, h=100):
    img = np.zeros((400, 400), dtype=np.uint8)
    cv2.rectangle(img, (100, 150), (100 + w, 150 + h), 255, -1)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea) if contours else None


def make_square_contour():
    img = np.zeros((400, 400), dtype=np.uint8)
    cv2.rectangle(img, (100, 100), (300, 300), 255, -1)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea) if contours else None


def make_triangle_contour():
    img = np.zeros((400, 400), dtype=np.uint8)
    pts = np.array([[200, 50], [100, 300], [300, 300]])
    cv2.fillPoly(img, [pts], 255)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea) if contours else None


def make_pentagon_contour():
    img = np.zeros((400, 400), dtype=np.uint8)
    pts = []
    for i in range(5):
        angle = i * 2 * 3.14159 / 5 - 3.14159 / 2
        pts.append([int(200 + 100 * np.cos(angle)), int(200 + 100 * np.sin(angle))])
    cv2.fillPoly(img, [np.array(pts)], 255)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea) if contours else None


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestHuMoments(unittest.TestCase):
    """Hu矩计算测试"""

    def test_returns_7_values(self):
        c = make_circle_contour()
        hu = ShapeMatcher.hu_moments(c)
        self.assertEqual(len(hu), 7)

    def test_circle_hu_moments(self):
        c = make_circle_contour(r=80)
        hu = ShapeMatcher.hu_moments(c)
        # Hu矩应为有限值
        self.assertTrue(np.all(np.isfinite(hu)))

    def test_different_shapes_different_hu(self):
        c1 = make_circle_contour()
        c2 = make_rect_contour(w=300, h=10)
        hu1 = ShapeMatcher.hu_moments(c1)
        hu2 = ShapeMatcher.hu_moments(c2)
        self.assertFalse(np.allclose(hu1, hu2, atol=0.1))

    def test_same_shape_similar_hu(self):
        c1 = make_circle_contour(r=80)
        c2 = make_circle_contour(r=80)
        hu1 = ShapeMatcher.hu_moments(c1)
        hu2 = ShapeMatcher.hu_moments(c2)
        np.testing.assert_array_almost_equal(hu1, hu2, decimal=3)

    def test_from_grayscale_image(self):
        img = np.zeros((100, 100), dtype=np.uint8)
        cv2.circle(img, (50, 50), 30, 255, -1)
        hu = ShapeMatcher.hu_moments(img)
        self.assertEqual(len(hu), 7)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestHuMatch(unittest.TestCase):
    """Hu矩匹配测试"""

    def test_same_shape_zero_distance(self):
        c1 = make_circle_contour(r=80)
        c2 = make_circle_contour(r=80)
        score = ShapeMatcher.hu_match(c1, c2)
        self.assertLess(score, 0.01)

    def test_different_shapes_nonzero(self):
        c1 = make_circle_contour()
        c2 = make_rect_contour(w=300, h=10)
        score = ShapeMatcher.hu_match(c1, c2)
        self.assertGreater(score, 0.01)

    def test_different_methods(self):
        c1 = make_circle_contour()
        c2 = make_rect_contour()
        s1 = ShapeMatcher.hu_match(c1, c2, method=cv2.CONTOURS_MATCH_I1)
        s2 = ShapeMatcher.hu_match(c1, c2, method=cv2.CONTOURS_MATCH_I2)
        # 不同方法应返回不同分数
        self.assertNotAlmostEqual(s1, s2, places=1)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestHuMatchBatch(unittest.TestCase):
    """批量匹配测试"""

    def test_returns_sorted(self):
        target = make_circle_contour(r=80)
        candidates = [make_circle_contour(r=80), make_rect_contour(w=300, h=10),
                      make_triangle_contour()]
        results = ShapeMatcher.hu_match_batch(target, candidates)
        scores = [r[1] for r in results]
        self.assertEqual(scores, sorted(scores))

    def test_best_match_is_circle(self):
        target = make_circle_contour(r=80)
        candidates = [make_rect_contour(w=300, h=10), make_circle_contour(r=80),
                      make_triangle_contour()]
        results = ShapeMatcher.hu_match_batch(target, candidates)
        # 最佳匹配应为索引1(圆)
        self.assertEqual(results[0][0], 1)

    def test_returns_tuples(self):
        target = make_circle_contour()
        results = ShapeMatcher.hu_match_batch(target, [make_rect_contour()])
        self.assertEqual(len(results), 1)
        self.assertEqual(len(results[0]), 3)  # (index, score, contour)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestContourMatch(unittest.TestCase):
    """轮廓匹配测试"""

    def test_same_as_hu_match(self):
        c1 = make_circle_contour()
        c2 = make_rect_contour()
        s1 = ShapeMatcher.hu_match(c1, c2)
        s2 = ShapeMatcher.contour_match(c1, c2)
        self.assertAlmostEqual(s1, s2, places=10)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestContourMatchTemplate(unittest.TestCase):
    """模板轮廓匹配测试"""

    def test_returns_best_match(self):
        c = make_circle_contour(r=80)
        templates = {'circle': make_circle_contour(r=80),
                     'rect': make_rect_contour(w=300, h=10),
                     'tri': make_triangle_contour()}
        name, score = ShapeMatcher.contour_match_template(c, templates)
        self.assertEqual(name, 'circle')
        self.assertLess(score, 0.01)

    def test_rect_matches_rect(self):
        c = make_rect_contour(w=200, h=100)
        templates = {'circle': make_circle_contour(r=80),
                     'rect': make_rect_contour(w=200, h=100)}
        name, _ = ShapeMatcher.contour_match_template(c, templates)
        self.assertEqual(name, 'rect')


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestTemplateMatch(unittest.TestCase):
    """模板匹配测试"""

    def test_finds_template(self):
        img = np.zeros((200, 200), dtype=np.uint8)
        img[50:100, 50:100] = 255
        template = np.ones((50, 50), dtype=np.uint8) * 255
        result = ShapeMatcher.template_match(img, template)
        self.assertIn('max_val', result)
        self.assertIn('max_loc', result)
        self.assertIn('locations', result)

    def test_ccoeff_method(self):
        img = np.zeros((200, 200), dtype=np.uint8)
        img[50:100, 50:100] = 255
        template = np.ones((50, 50), dtype=np.uint8) * 255
        result = ShapeMatcher.template_match(img, template, method='ccoeff_normed')
        self.assertGreater(result['max_val'], 0.5)

    def test_multi_match(self):
        img = np.zeros((200, 300), dtype=np.uint8)
        img[50:100, 50:100] = 255
        img[50:100, 200:250] = 255
        template = np.ones((50, 50), dtype=np.uint8) * 255
        result = ShapeMatcher.template_match(img, template, multi=True, threshold=0.8)
        self.assertIn('locations', result)

    def test_color_image_input(self):
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        img[50:100, 50:100] = [255, 255, 255]
        template = np.ones((50, 50, 3), dtype=np.uint8) * 255
        result = ShapeMatcher.template_match(img, template)
        self.assertIn('max_val', result)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestClassifyShape(unittest.TestCase):
    """形状分类测试"""

    def test_circle(self):
        c = make_circle_contour(r=80)
        shape = ShapeMatcher.classify_shape(c)
        self.assertEqual(shape, 'circle')

    def test_triangle(self):
        c = make_triangle_contour()
        shape = ShapeMatcher.classify_shape(c)
        self.assertEqual(shape, 'triangle')

    def test_square(self):
        c = make_square_contour()
        shape = ShapeMatcher.classify_shape(c)
        self.assertEqual(shape, 'square')

    def test_rectangle(self):
        c = make_rect_contour(w=300, h=100)
        shape = ShapeMatcher.classify_shape(c)
        self.assertEqual(shape, 'rectangle')

    def test_pentagon(self):
        c = make_pentagon_contour()
        shape = ShapeMatcher.classify_shape(c)
        self.assertEqual(shape, 'pentagon')


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestClassifyShapes(unittest.TestCase):
    """批量形状分类测试"""

    def test_returns_list(self):
        contours = [make_circle_contour(), make_triangle_contour(), make_rect_contour()]
        results = ShapeMatcher.classify_shapes(contours)
        self.assertEqual(len(results), 3)

    def test_returns_tuples(self):
        contours = [make_circle_contour()]
        results = ShapeMatcher.classify_shapes(contours)
        self.assertEqual(len(results[0]), 2)  # (shape, circularity)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawMatchResult(unittest.TestCase):
    def test_output_shape(self):
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        match_result = {'max_loc': (50, 50), 'template_size': (30, 30), 'max_val': 0.95}
        result = ShapeMatcher.draw_match_result(img, match_result)
        self.assertEqual(result.shape, img.shape)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawClassifiedShapes(unittest.TestCase):
    def test_output_shape(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        contours = [make_circle_contour(), make_triangle_contour()]
        result = ShapeMatcher.draw_classified_shapes(img, contours)
        self.assertEqual(result.shape, img.shape)


if __name__ == '__main__':
    unittest.main()
