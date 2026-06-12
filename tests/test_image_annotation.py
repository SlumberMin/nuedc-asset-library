"""
图像标注单元测试
覆盖: 画矩形/旋转矩形/圆/椭圆/直线/箭头/文字/十字准星/轮廓/批量框/关键点
"""
import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from importlib import import_module

mod = import_module('10_视觉通用代码库.image_annotation')


class TestDrawRect(unittest.TestCase):
    """画矩形框测试"""

    def setUp(self):
        self.img = np.zeros((200, 300, 3), dtype=np.uint8)

    def test_output_shape(self):
        result = mod.draw_rect(self.img, (10, 10), (100, 80))
        self.assertEqual(result.shape, self.img.shape)

    def test_does_not_modify_original(self):
        """不应修改原图"""
        original = self.img.copy()
        mod.draw_rect(self.img, (10, 10), (100, 80))
        np.testing.assert_array_equal(self.img, original)

    def test_custom_color(self):
        result = mod.draw_rect(self.img, (10, 10), (100, 80), color=(255, 0, 0))
        # 矩形区域应有红色像素
        self.assertTrue(np.any(result[:, :, 2] > 0))

    def test_output_type(self):
        result = mod.draw_rect(self.img, (10, 10), (100, 80))
        self.assertEqual(result.dtype, np.uint8)


class TestDrawRotatedRect(unittest.TestCase):
    """画旋转矩形测试"""

    def test_output_shape(self):
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        result = mod.draw_rotated_rect(img, (150, 100), (80, 40), 30)
        self.assertEqual(result.shape, img.shape)


class TestDrawCircle(unittest.TestCase):
    """画圆测试"""

    def setUp(self):
        self.img = np.zeros((200, 200, 3), dtype=np.uint8)

    def test_output_shape(self):
        result = mod.draw_circle(self.img, (100, 100), 50)
        self.assertEqual(result.shape, self.img.shape)

    def test_does_not_modify_original(self):
        original = self.img.copy()
        mod.draw_circle(self.img, (100, 100), 50)
        np.testing.assert_array_equal(self.img, original)

    def test_circle_has_pixels(self):
        result = mod.draw_circle(self.img, (100, 100), 50, color=(0, 255, 0))
        self.assertTrue(np.any(result[:, :, 1] > 0))


class TestDrawEllipse(unittest.TestCase):
    """画椭圆测试"""

    def test_output_shape(self):
        img = np.zeros((200, 200, 3), dtype=np.uint8)
        result = mod.draw_ellipse(img, (100, 100), (80, 40), 0, 0, 360)
        self.assertEqual(result.shape, img.shape)


class TestDrawLine(unittest.TestCase):
    """画直线测试"""

    def test_output_shape(self):
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        result = mod.draw_line(img, (10, 10), (290, 190))
        self.assertEqual(result.shape, img.shape)

    def test_does_not_modify_original(self):
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        original = img.copy()
        mod.draw_line(img, (10, 10), (290, 190))
        np.testing.assert_array_equal(img, original)

    def test_custom_thickness(self):
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        result = mod.draw_line(img, (0, 100), (299, 100), thickness=5)
        self.assertTrue(np.any(result > 0))


class TestDrawArrow(unittest.TestCase):
    """画箭头测试"""

    def test_output_shape(self):
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        result = mod.draw_arrow(img, (50, 100), (250, 100))
        self.assertEqual(result.shape, img.shape)

    def test_arrow_has_pixels(self):
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        result = mod.draw_arrow(img, (50, 100), (250, 100), color=(0, 255, 0))
        self.assertTrue(np.any(result > 0))


class TestDrawText(unittest.TestCase):
    """画文字测试"""

    def setUp(self):
        self.img = np.zeros((200, 300, 3), dtype=np.uint8)

    def test_output_shape(self):
        result = mod.draw_text(self.img, 'Hello', (50, 100))
        self.assertEqual(result.shape, self.img.shape)

    def test_text_has_pixels(self):
        result = mod.draw_text(self.img, 'Test', (50, 100), color=(255, 255, 255))
        self.assertTrue(np.any(result > 0))

    def test_with_background(self):
        result = mod.draw_text(self.img, 'BG', (50, 100),
                               color=(255, 255, 255), bg_color=(0, 0, 128))
        self.assertTrue(np.any(result > 0))

    def test_does_not_modify_original(self):
        original = self.img.copy()
        mod.draw_text(self.img, 'Test', (50, 100))
        np.testing.assert_array_equal(self.img, original)


class TestDrawCrosshair(unittest.TestCase):
    """画十字准星测试"""

    def test_output_shape(self):
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        result = mod.draw_crosshair(img, (150, 100))
        self.assertEqual(result.shape, img.shape)

    def test_custom_size(self):
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        result = mod.draw_crosshair(img, (150, 100), size=40)
        self.assertTrue(np.any(result > 0))


class TestDrawContour(unittest.TestCase):
    """画轮廓测试"""

    def test_output_shape(self):
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        contour = np.array([[10, 10], [100, 10], [100, 80], [10, 80]])
        result = mod.draw_contour(img, contour)
        self.assertEqual(result.shape, img.shape)


class TestDrawBboxes(unittest.TestCase):
    """批量画框测试"""

    def setUp(self):
        self.img = np.zeros((200, 300, 3), dtype=np.uint8)

    def test_single_bbox(self):
        bboxes = [(10, 10, 100, 80)]
        result = mod.draw_bboxes(self.img, bboxes)
        self.assertEqual(result.shape, self.img.shape)

    def test_multiple_bboxes(self):
        bboxes = [(10, 10, 50, 50), (100, 100, 200, 150)]
        result = mod.draw_bboxes(self.img, bboxes)
        self.assertEqual(result.shape, self.img.shape)

    def test_with_labels(self):
        bboxes = [(10, 10, 100, 80), (120, 50, 250, 180)]
        labels = ['cat', 'dog']
        result = mod.draw_bboxes(self.img, bboxes, labels=labels)
        self.assertTrue(np.any(result > 0))

    def test_labels_shorter_than_bboxes(self):
        """标签数少于框数不应报错"""
        bboxes = [(10, 10, 50, 50), (100, 100, 200, 150)]
        labels = ['only_one']
        result = mod.draw_bboxes(self.img, bboxes, labels=labels)
        self.assertEqual(result.shape, self.img.shape)


class TestDrawKeypoints(unittest.TestCase):
    """画关键点测试"""

    def test_output_shape(self):
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        kps = [(50, 50), (100, 100), (200, 150)]
        result = mod.draw_keypoints(img, kps)
        self.assertEqual(result.shape, img.shape)

    def test_custom_radius(self):
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        kps = [(150, 100)]
        result = mod.draw_keypoints(img, kps, radius=10, color=(0, 0, 255))
        self.assertTrue(np.any(result[:, :, 2] > 0))


if __name__ == '__main__':
    unittest.main()
