#!/usr/bin/env python3
"""
最小外接矩形单元测试
覆盖: 旋转矩形、正外接矩形、参数提取、角度规范化、
      最小外接圆、直线拟合、矩形相似度、多目标提取
测试对象: 10_视觉通用代码库/min_area_rect.py
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
    from _10_视觉通用代码库.min_area_rect import (
        min_area_rect, min_area_rect_box, rect_to_params,
        normalize_angle, bounding_rect, bounding_rect_from_mask,
        min_enclosing_circle, fit_line_angle,
        rectangle_similarity, find_rotated_rects,
        draw_rotated_rect, draw_all_rects
    )


def make_rect_contour(size=200, angle=0):
    """创建矩形轮廓"""
    img = np.zeros((size, size), dtype=np.uint8)
    cx, cy = size // 2, size // 2
    w, h = 80, 40
    pts = np.array([[-w // 2, -h // 2], [w // 2, -h // 2],
                     [w // 2, h // 2], [-w // 2, h // 2]], dtype=np.float32)
    # 旋转
    angle_rad = np.radians(angle)
    R = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                  [np.sin(angle_rad), np.cos(angle_rad)]])
    pts = pts @ R.T + np.array([cx, cy])
    pts = pts.astype(np.int32)
    cv2.fillPoly(img, [pts], 255)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea)


def make_circle_contour(size=200, r=50):
    """创建圆形轮廓"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(img, (100, 100), r, 255, -1)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea)


def make_multi_objects(size=200):
    """创建多个目标的二值图"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.rectangle(img, (20, 20), (80, 60), 255, -1)
    cv2.rectangle(img, (120, 100), (180, 160), 255, -1)
    cv2.circle(img, (60, 150), 25, 255, -1)
    return img


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMinAreaRect(unittest.TestCase):
    """最小外接矩形测试"""

    def test_returns_tuple(self):
        cnt = make_rect_contour()
        rect = min_area_rect(cnt)
        self.assertEqual(len(rect), 3)  # center, size, angle

    def test_center_in_image(self):
        cnt = make_rect_contour(200)
        rect = min_area_rect(cnt)
        cx, cy = rect[0]
        self.assertGreater(cx, 0)
        self.assertLess(cx, 200)
        self.assertGreater(cy, 0)
        self.assertLess(cy, 200)

    def test_size_positive(self):
        cnt = make_rect_contour()
        rect = min_area_rect(cnt)
        w, h = rect[1]
        self.assertGreater(w, 0)
        self.assertGreater(h, 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMinAreaRectBox(unittest.TestCase):
    """旋转矩形顶点测试"""

    def test_returns_4_points(self):
        cnt = make_rect_contour()
        box, rect = min_area_rect_box(cnt)
        self.assertEqual(box.shape, (4, 2))

    def test_box_is_integer(self):
        cnt = make_rect_contour()
        box, _ = min_area_rect_box(cnt)
        self.assertTrue(np.issubdtype(box.dtype, np.integer))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestRectToParams(unittest.TestCase):
    """参数提取测试"""

    def test_returns_dict(self):
        cnt = make_rect_contour()
        rect = min_area_rect(cnt)
        params = rect_to_params(rect)
        self.assertIsInstance(params, dict)

    def test_has_required_keys(self):
        cnt = make_rect_contour()
        rect = min_area_rect(cnt)
        params = rect_to_params(rect)
        for key in ['center', 'size', 'angle', 'width', 'height',
                     'aspect_ratio', 'area']:
            self.assertIn(key, params)

    def test_width_leq_height(self):
        """width应为短边"""
        cnt = make_rect_contour()
        rect = min_area_rect(cnt)
        params = rect_to_params(rect)
        self.assertLessEqual(params['width'], params['height'])

    def test_area_positive(self):
        cnt = make_rect_contour()
        rect = min_area_rect(cnt)
        params = rect_to_params(rect)
        self.assertGreater(params['area'], 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestNormalizeAngle(unittest.TestCase):
    """角度规范化测试"""

    def test_angle_in_range(self):
        cnt = make_rect_contour(angle=30)
        rect = min_area_rect(cnt)
        _, _, angle = normalize_angle(rect)
        self.assertGreaterEqual(angle, -90)
        self.assertLessEqual(angle, 90)

    def test_width_leq_height(self):
        cnt = make_rect_contour(angle=45)
        rect = min_area_rect(cnt)
        _, (w, h), _ = normalize_angle(rect)
        self.assertLessEqual(w, h)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestBoundingRect(unittest.TestCase):
    """正外接矩形测试"""

    def test_returns_4_values(self):
        cnt = make_rect_contour()
        x, y, w, h = bounding_rect(cnt)
        self.assertIsInstance(x, (int, np.integer))
        self.assertGreater(w, 0)
        self.assertGreater(h, 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestBoundingRectFromMask(unittest.TestCase):
    """从mask获取外接矩形测试"""

    def test_returns_rect(self):
        binary = make_multi_objects()
        rect = bounding_rect_from_mask(binary)
        self.assertIsNotNone(rect)
        self.assertEqual(len(rect), 4)

    def test_empty_mask(self):
        binary = np.zeros((200, 200), dtype=np.uint8)
        rect = bounding_rect_from_mask(binary)
        self.assertIsNone(rect)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMinEnclosingCircle(unittest.TestCase):
    """最小外接圆测试"""

    def test_returns_center_radius(self):
        cnt = make_circle_contour()
        center, radius = min_enclosing_circle(cnt)
        self.assertEqual(len(center), 2)
        self.assertGreater(radius, 0)

    def test_circle_radius(self):
        """50像素半径的圆，外接圆半径应接近50"""
        cnt = make_circle_contour(r=50)
        _, radius = min_enclosing_circle(cnt)
        self.assertAlmostEqual(radius, 50, delta=5)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFitLineAngle(unittest.TestCase):
    """直线拟合角度测试"""

    def test_returns_angle(self):
        cnt = make_rect_contour(angle=0)
        angle, pt, vec = fit_line_angle(cnt)
        self.assertIsInstance(angle, float)

    def test_returns_tuple(self):
        cnt = make_rect_contour()
        angle, pt, vec = fit_line_angle(cnt)
        self.assertEqual(len(pt), 2)
        self.assertEqual(len(vec), 2)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestRectangleSimilarity(unittest.TestCase):
    """矩形相似度测试"""

    def test_rectangle_high_similarity(self):
        """矩形应有高相似度"""
        cnt = make_rect_contour()
        ratio, is_rect = rectangle_similarity(cnt)
        self.assertGreater(ratio, 0.8)

    def test_circle_lower_similarity(self):
        """圆形矩形相似度应较低"""
        cnt = make_circle_contour()
        ratio, _ = rectangle_similarity(cnt)
        self.assertLess(ratio, 0.95)

    def test_returns_bool(self):
        cnt = make_rect_contour()
        _, is_rect = rectangle_similarity(cnt, thresh_ratio=0.85)
        self.assertIsInstance(is_rect, bool)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFindRotatedRects(unittest.TestCase):
    """多目标旋转矩形提取测试"""

    def test_finds_multiple(self):
        binary = make_multi_objects()
        rects = find_rotated_rects(binary, min_area=100)
        self.assertGreater(len(rects), 0)

    def test_has_required_keys(self):
        binary = make_multi_objects()
        rects = find_rotated_rects(binary, min_area=100)
        for r in rects:
            self.assertIn('contour', r)
            self.assertIn('rect', r)
            self.assertIn('box', r)
            self.assertIn('params', r)

    def test_min_area_filter(self):
        binary = make_multi_objects()
        rects_loose = find_rotated_rects(binary, min_area=10)
        rects_strict = find_rotated_rects(binary, min_area=5000)
        self.assertLessEqual(len(rects_strict), len(rects_loose))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawRotatedRect(unittest.TestCase):
    """绘制旋转矩形测试"""

    def test_output_shape(self):
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        cnt = make_rect_contour()
        rect = min_area_rect(cnt)
        result = draw_rotated_rect(img, rect)
        self.assertEqual(result.shape, img.shape)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawAllRects(unittest.TestCase):
    """绘制多矩形测试"""

    def test_output_shape(self):
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        binary = make_multi_objects()
        rects = find_rotated_rects(binary, min_area=100)
        result = draw_all_rects(img, rects)
        self.assertEqual(result.shape, img.shape)


if __name__ == '__main__':
    unittest.main()
