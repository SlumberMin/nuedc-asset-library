#!/usr/bin/env python3
"""
图像分割单元测试
覆盖: 二值化、Otsu、自适应阈值、HSV颜色范围、
      单通道分割、区域生长、分水岭
测试对象: 10_视觉通用代码库/image_segmentation.py
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
    from _10_视觉通用代码库.image_segmentation import (
        threshold_binary, threshold_otsu, threshold_adaptive,
        threshold_color_range, threshold_multi_channel,
        region_growing, region_growing_multi_seed,
        watershed_segmentation
    )


def make_gradient_gray(size=200):
    """创建灰度渐变图"""
    img = np.zeros((size, size), dtype=np.uint8)
    for i in range(size):
        img[:, i] = int(255 * i / size)
    return img


def make_binary_objects(size=200):
    """创建含多个白色目标的灰度图"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(img, (50, 50), 20, 200, -1)
    cv2.circle(img, (150, 150), 30, 180, -1)
    cv2.rectangle(img, (100, 20), (170, 60), 160, -1)
    return img


def make_hsv_image(size=200):
    """创建BGR图像用于HSV测试"""
    img = np.zeros((size, size, 3), dtype=np.uint8)
    img[:, :size // 2] = [0, 0, 255]   # 红色
    img[:, size // 2:] = [255, 0, 0]    # 蓝色
    return img


def make_uniform_region(size=200):
    """创建含均匀区域的灰度图"""
    img = np.full((size, size), 50, dtype=np.uint8)
    cv2.circle(img, (100, 100), 40, 200, -1)
    return img


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestThresholdBinary(unittest.TestCase):
    """二值化分割测试"""

    def test_output_shape(self):
        gray = make_gradient_gray()
        mask = threshold_binary(gray, 127)
        self.assertEqual(mask.shape, gray.shape)

    def test_output_dtype(self):
        gray = make_gradient_gray()
        mask = threshold_binary(gray, 127)
        self.assertEqual(mask.dtype, np.uint8)

    def test_threshold_effect(self):
        """阈值分割应正确分离"""
        gray = make_gradient_gray()
        mask = threshold_binary(gray, 127)
        # 左半部分应为黑, 右半部分应为白
        self.assertTrue(np.all(mask[:, :64] == 0))
        self.assertTrue(np.all(mask[:, 128:] == 255))

    def test_custom_max_val(self):
        gray = make_gradient_gray()
        mask = threshold_binary(gray, 127, max_val=200)
        self.assertEqual(mask.max(), 200)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestThresholdOtsu(unittest.TestCase):
    """Otsu自动阈值测试"""

    def test_output_shape(self):
        gray = make_binary_objects()
        mask, thresh = threshold_otsu(gray)
        self.assertEqual(mask.shape, gray.shape)

    def test_thresh_value_range(self):
        gray = make_binary_objects()
        mask, thresh = threshold_otsu(gray)
        self.assertGreater(thresh, 0)
        self.assertLess(thresh, 255)

    def test_detects_objects(self):
        gray = make_binary_objects()
        mask, _ = threshold_otsu(gray)
        self.assertGreater(np.sum(mask > 0), 100)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestThresholdAdaptive(unittest.TestCase):
    """自适应阈值测试"""

    def test_output_shape(self):
        gray = make_gradient_gray()
        mask = threshold_adaptive(gray)
        self.assertEqual(mask.shape, gray.shape)

    def test_works_on_gradient(self):
        """渐变图自适应应能产生分割"""
        gray = make_gradient_gray()
        mask = threshold_adaptive(gray, block_size=11, C=2)
        self.assertGreater(np.sum(mask > 0), 0)

    def test_different_block_size(self):
        gray = make_gradient_gray(100)
        m1 = threshold_adaptive(gray, block_size=11, C=2)
        m2 = threshold_adaptive(gray, block_size=31, C=2)
        self.assertFalse(np.array_equal(m1, m2))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestThresholdColorRange(unittest.TestCase):
    """HSV颜色范围分割测试"""

    def test_detects_red(self):
        bgr = make_hsv_image()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = threshold_color_range(hsv, [0, 50, 50], [10, 255, 255])
        self.assertGreater(np.sum(mask > 0), 0)

    def test_output_shape(self):
        bgr = make_hsv_image()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = threshold_color_range(hsv, [0, 50, 50], [10, 255, 255])
        self.assertEqual(mask.shape[:2], bgr.shape[:2])


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestThresholdMultiChannel(unittest.TestCase):
    """单通道范围分割测试"""

    def test_single_channel(self):
        gray = make_gradient_gray()
        mask = threshold_multi_channel(gray, 'h', (50, 200))
        self.assertEqual(mask.shape, gray.shape)

    def test_bgr_channel(self):
        bgr = make_hsv_image()
        mask = threshold_multi_channel(bgr, 'r', (100, 255))
        self.assertEqual(mask.shape[:2], bgr.shape[:2])


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestRegionGrowing(unittest.TestCase):
    """区域生长测试"""

    def test_output_shape(self):
        gray = make_uniform_region()
        mask = region_growing(gray, (100, 100), thresh_diff=30)
        self.assertEqual(mask.shape, gray.shape)

    def test_grows_similar_region(self):
        """应生长相似灰度区域"""
        gray = make_uniform_region()
        mask = region_growing(gray, (100, 100), thresh_diff=30)
        # 种子点在亮圆内，应生长出区域
        self.assertGreater(np.sum(mask > 0), 100)

    def test_seed_point_labeled(self):
        """种子点应被标记"""
        gray = make_uniform_region()
        mask = region_growing(gray, (100, 100), thresh_diff=30)
        self.assertEqual(mask[100, 100], 255)

    def test_threshold_effect(self):
        """严格阈值应产生更小区域"""
        gray = make_uniform_region()
        mask_loose = region_growing(gray, (100, 100), thresh_diff=100)
        mask_strict = region_growing(gray, (100, 100), thresh_diff=5)
        self.assertGreaterEqual(np.sum(mask_loose > 0),
                                np.sum(mask_strict > 0))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestRegionGrowingMultiSeed(unittest.TestCase):
    """多种子区域生长测试"""

    def test_multiple_seeds(self):
        gray = make_uniform_region()
        seeds = [(100, 100), (30, 30)]
        mask = region_growing_multi_seed(gray, seeds, thresh_diff=30)
        self.assertEqual(mask.shape, gray.shape)

    def test_more_coverage(self):
        """多种子应覆盖更多区域"""
        gray = make_uniform_region()
        mask_single = region_growing(gray, (100, 100), thresh_diff=30)
        mask_multi = region_growing_multi_seed(
            gray, [(100, 100), (30, 30)], thresh_diff=30)
        self.assertGreaterEqual(np.sum(mask_multi > 0),
                                np.sum(mask_single > 0))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestWatershed(unittest.TestCase):
    """分水岭分割测试"""

    def test_output_shape(self):
        bgr = make_hsv_image()
        markers, result = watershed_segmentation(bgr)
        self.assertEqual(markers.shape[:2], bgr.shape[:2])

    def test_result_is_bgr(self):
        bgr = make_hsv_image()
        _, result = watershed_segmentation(bgr)
        self.assertEqual(len(result.shape), 3)
        self.assertEqual(result.shape[2], 3)

    def test_markers_have_labels(self):
        bgr = make_hsv_image()
        markers, _ = watershed_segmentation(bgr)
        # 应至少有背景和前景标签
        self.assertGreater(markers.max(), 0)

    def test_with_custom_mask(self):
        bgr = make_hsv_image()
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        markers, result = watershed_segmentation(bgr, mask=mask)
        self.assertEqual(markers.shape[:2], bgr.shape[:2])


if __name__ == '__main__':
    unittest.main()
