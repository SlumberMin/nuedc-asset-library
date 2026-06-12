#!/usr/bin/env python3
"""
阈值分割单元测试 (image_threshold.py)
覆盖: 全局阈值 / Otsu / 三角法 / 自适应均值 / 自适应高斯 / 多级 / Otsu+孔洞 / 分通道Otsu
测试对象: 10_视觉通用代码库/image_threshold.py
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
    from _10_视觉通用代码库.image_threshold import (
        threshold_global, threshold_otsu, threshold_triangle,
        threshold_adaptive_mean, threshold_adaptive_gaussian,
        threshold_multi_level, otsu_with_holes, split_channels_otsu,
    )


def make_bimodal(size=200):
    """双峰分布图像(暗背景+亮前景)"""
    img = np.zeros((size, size), dtype=np.uint8)
    img[60:140, 60:140] = 220
    img = cv2.GaussianBlur(img, (5, 5), 0)
    return img


def make_gradient(size=200):
    return np.tile(np.linspace(20, 230, size, dtype=np.uint8), (size, 1))


def make_bgr(size=200):
    gray = make_bimodal(size)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


# ── 全局阈值 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestThresholdGlobal(unittest.TestCase):

    def test_binary(self):
        result = threshold_global(make_bimodal(), thresh=100)
        self.assertTrue(set(np.unique(result)).issubset({0, 255}))

    def test_binary_inv(self):
        result = threshold_global(make_bimodal(), thresh=100, threshold_type='binary_inv')
        self.assertTrue(set(np.unique(result)).issubset({0, 255}))

    def test_trunc(self):
        result = threshold_global(make_bimodal(), thresh=100, threshold_type='trunc')
        self.assertLessEqual(result.max(), 100)

    def test_tozero(self):
        result = threshold_global(make_bimodal(), thresh=100, threshold_type='tozero')
        self.assertEqual(result[0, 0], 0)  # 暗区域变为0

    def test_tozero_inv(self):
        result = threshold_global(make_bimodal(), thresh=100, threshold_type='tozero_inv')
        self.assertEqual(result.dtype, np.uint8)

    def test_output_shape(self):
        result = threshold_global(make_bimodal(), thresh=127)
        self.assertEqual(result.shape, (200, 200))

    def test_bgr_input(self):
        result = threshold_global(make_bgr(), thresh=100)
        self.assertEqual(result.shape, (200, 200))

    def test_custom_maxval(self):
        result = threshold_global(make_bimodal(), thresh=100, maxval=128)
        unique = set(np.unique(result))
        self.assertTrue(unique.issubset({0, 128}))


# ── Otsu ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestThresholdOtsu(unittest.TestCase):

    def test_returns_binary_and_thresh(self):
        binary, thresh_val = threshold_otsu(make_bimodal())
        self.assertEqual(binary.dtype, np.uint8)
        self.assertIsInstance(thresh_val, (float, np.floating))

    def test_thresh_between_0_255(self):
        _, thresh_val = threshold_otsu(make_bimodal())
        self.assertGreater(thresh_val, 0)
        self.assertLess(thresh_val, 255)

    def test_binary_only_0_255(self):
        binary, _ = threshold_otsu(make_bimodal())
        unique = set(np.unique(binary))
        self.assertTrue(unique.issubset({0, 255}))

    def test_bgr_input(self):
        binary, _ = threshold_otsu(make_bgr())
        self.assertEqual(binary.shape, (200, 200))


# ── 三角法 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestThresholdTriangle(unittest.TestCase):

    def test_returns_binary_and_thresh(self):
        binary, thresh_val = threshold_triangle(make_gradient())
        self.assertEqual(binary.dtype, np.uint8)
        self.assertIsInstance(thresh_val, (float, np.floating))

    def test_output_shape(self):
        binary, _ = threshold_triangle(make_gradient())
        self.assertEqual(binary.shape, (200, 200))


# ── 自适应均值阈值 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestAdaptiveMean(unittest.TestCase):

    def test_output_shape(self):
        result = threshold_adaptive_mean(make_bimodal())
        self.assertEqual(result.shape, (200, 200))

    def test_dtype_uint8(self):
        result = threshold_adaptive_mean(make_bimodal())
        self.assertEqual(result.dtype, np.uint8)

    def test_even_block_size_handled(self):
        """偶数block_size应自动修正为奇数"""
        result = threshold_adaptive_mean(make_bimodal(), block_size=10)
        self.assertEqual(result.shape, (200, 200))

    def test_custom_params(self):
        result = threshold_adaptive_mean(make_bimodal(), block_size=21, C=5)
        self.assertEqual(result.shape, (200, 200))


# ── 自适应高斯阈值 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestAdaptiveGaussian(unittest.TestCase):

    def test_output_shape(self):
        result = threshold_adaptive_gaussian(make_bimodal())
        self.assertEqual(result.shape, (200, 200))

    def test_even_block_size_handled(self):
        result = threshold_adaptive_gaussian(make_bimodal(), block_size=12)
        self.assertEqual(result.shape, (200, 200))

    def test_different_from_mean(self):
        """高斯自适应与均值自适应结果应不同"""
        img = make_bimodal()
        r_mean = threshold_adaptive_mean(img, block_size=11, C=2)
        r_gauss = threshold_adaptive_gaussian(img, block_size=11, C=2)
        self.assertFalse(np.array_equal(r_mean, r_gauss))


# ── 多级阈值(K-means) ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMultiLevel(unittest.TestCase):

    def test_output_shape(self):
        result = threshold_multi_level(make_gradient(), levels=3)
        self.assertEqual(result.shape, (200, 200))

    def test_dtype_uint8(self):
        result = threshold_multi_level(make_gradient(), levels=3)
        self.assertEqual(result.dtype, np.uint8)

    def test_levels_affect_output(self):
        r2 = threshold_multi_level(make_gradient(), levels=2)
        r4 = threshold_multi_level(make_gradient(), levels=4)
        self.assertFalse(np.array_equal(r2, r4))


# ── Otsu + 形态学闭合 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestOtsuWithHoles(unittest.TestCase):

    def test_output_shape(self):
        result = otsu_with_holes(make_bimodal())
        self.assertEqual(result.shape, (200, 200))

    def test_fills_holes(self):
        """闭合操作应填充小孔"""
        img = np.zeros((100, 100), dtype=np.uint8)
        img[20:80, 20:80] = 255
        img[40:60, 40:60] = 0  # 小孔
        result = otsu_with_holes(img)
        # 闭合后中心区域应被填充
        self.assertGreater(result[50, 50], 0)


# ── 分通道Otsu ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestSplitChannelsOtsu(unittest.TestCase):

    def test_returns_three(self):
        results = split_channels_otsu(make_bgr())
        self.assertEqual(len(results), 3)

    def test_each_is_binary(self):
        results = split_channels_otsu(make_bgr())
        for r in results:
            unique = set(np.unique(r))
            self.assertTrue(unique.issubset({0, 255}))

    def test_shape_matches(self):
        results = split_channels_otsu(make_bgr())
        for r in results:
            self.assertEqual(r.shape, (200, 200))


if __name__ == '__main__':
    unittest.main()
