#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Haar特征单元测试
覆盖: HaarFeatureExtractor初始化、积分图、矩形区域求和、
      各类型Haar特征、多尺度提取、特征向量、特征计数、便捷函数
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from _10_视觉通用代码库.haar_feature import (
    HaarFeatureExtractor,
    extract_haar_feature,
)


def make_test_image(size=100):
    """创建含矩形的测试图像"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.rectangle(img, (20, 20), (80, 80), 180, -1)
    cv2.rectangle(img, (35, 35), (65, 65), 100, -1)
    return img


# ==================== 初始化测试 ====================

class TestHaarFeatureExtractorInit(unittest.TestCase):
    """初始化测试"""

    def test_default_feature_types(self):
        ext = HaarFeatureExtractor()
        self.assertEqual(len(ext.feature_types), 5)
        self.assertIn('two_horizontal', ext.feature_types)
        self.assertIn('two_vertical', ext.feature_types)
        self.assertIn('three_horizontal', ext.feature_types)
        self.assertIn('three_vertical', ext.feature_types)
        self.assertIn('four', ext.feature_types)

    def test_custom_feature_types(self):
        ext = HaarFeatureExtractor(feature_types=['two_horizontal', 'four'])
        self.assertEqual(len(ext.feature_types), 2)


# ==================== 积分图测试 ====================

class TestIntegralImage(unittest.TestCase):
    """积分图计算测试"""

    def test_returns_ndarray(self):
        ext = HaarFeatureExtractor()
        img = np.zeros((50, 50), dtype=np.uint8)
        integral = ext._compute_integral(img)
        self.assertIsInstance(integral, np.ndarray)

    def test_shape_plus_one(self):
        """积分图应比原图大1"""
        ext = HaarFeatureExtractor()
        img = np.zeros((50, 50), dtype=np.uint8)
        integral = ext._compute_integral(img)
        self.assertEqual(integral.shape, (51, 51))

    def test_dtype_float64(self):
        ext = HaarFeatureExtractor()
        img = np.zeros((50, 50), dtype=np.uint8)
        integral = ext._compute_integral(img)
        self.assertEqual(integral.dtype, np.float64)

    def test_sum_correctness(self):
        """积分图右下角应等于全图像素和"""
        ext = HaarFeatureExtractor()
        img = np.ones((50, 50), dtype=np.uint8) * 10
        integral = ext._compute_integral(img)
        total = integral[50, 50]
        self.assertAlmostEqual(total, 50 * 50 * 10, delta=1.0)


# ==================== 矩形区域求和测试 ====================

class TestRectSum(unittest.TestCase):
    """矩形区域求和测试"""

    def test_full_image_sum(self):
        ext = HaarFeatureExtractor()
        img = np.ones((50, 50), dtype=np.uint8) * 10
        integral = ext._compute_integral(img)
        s = ext._rect_sum(integral, 0, 0, 50, 50)
        self.assertAlmostEqual(s, 50 * 50 * 10, delta=1.0)

    def test_subregion_sum(self):
        ext = HaarFeatureExtractor()
        img = np.zeros((50, 50), dtype=np.uint8)
        img[10:20, 10:20] = 100
        integral = ext._compute_integral(img)
        s = ext._rect_sum(integral, 10, 10, 10, 10)
        self.assertAlmostEqual(s, 10 * 10 * 100, delta=1.0)

    def test_single_pixel(self):
        ext = HaarFeatureExtractor()
        img = np.zeros((50, 50), dtype=np.uint8)
        img[25, 25] = 200
        integral = ext._compute_integral(img)
        s = ext._rect_sum(integral, 25, 25, 1, 1)
        self.assertAlmostEqual(s, 200.0, delta=1.0)


# ==================== 单位置特征提取测试 ====================

class TestExtractAtPosition(unittest.TestCase):
    """单位置Haar特征提取测试"""

    def test_returns_list(self):
        ext = HaarFeatureExtractor()
        img = make_test_image()
        integral = ext._compute_integral(img)
        feats = ext.extract_at_position(integral, 10, 10, 24, 24)
        self.assertIsInstance(feats, list)

    def test_default_returns_five(self):
        """默认5种特征类型应返回5个值"""
        ext = HaarFeatureExtractor()
        img = make_test_image()
        integral = ext._compute_integral(img)
        feats = ext.extract_at_position(integral, 10, 10, 24, 24)
        self.assertEqual(len(feats), 5)

    def test_custom_types(self):
        ext = HaarFeatureExtractor(feature_types=['two_horizontal'])
        img = make_test_image()
        integral = ext._compute_integral(img)
        feats = ext.extract_at_position(integral, 10, 10, 24, 24)
        self.assertEqual(len(feats), 1)

    def test_uniform_image_zero(self):
        """均匀图像的Haar特征应为零"""
        ext = HaarFeatureExtractor()
        img = np.full((50, 50), 128, dtype=np.uint8)
        integral = ext._compute_integral(img)
        feats = ext.extract_at_position(integral, 0, 0, 24, 24)
        for f in feats:
            self.assertAlmostEqual(f, 0.0, delta=1.0)

    def test_asymmetric_image_nonzero(self):
        """非对称图像应产生非零特征"""
        ext = HaarFeatureExtractor()
        img = make_test_image()
        integral = ext._compute_integral(img)
        feats = ext.extract_at_position(integral, 10, 10, 24, 24)
        nonzero = sum(1 for f in feats if abs(f) > 1.0)
        self.assertGreater(nonzero, 0)


# ==================== 多尺度提取测试 ====================

class TestExtractAll(unittest.TestCase):
    """多尺度Haar特征提取测试"""

    def test_returns_list(self):
        ext = HaarFeatureExtractor()
        img = make_test_image(200)
        results = ext.extract_all(img, window_size=(24, 24), step=10,
                                  scale_factor=1.5, n_scales=2)
        self.assertIsInstance(results, list)

    def test_each_element_format(self):
        ext = HaarFeatureExtractor()
        img = make_test_image(200)
        results = ext.extract_all(img, window_size=(24, 24), step=20,
                                  scale_factor=1.5, n_scales=2)
        if len(results) > 0:
            x, y, scale, feats = results[0]
            self.assertIsInstance(x, (int, np.integer))
            self.assertIsInstance(y, (int, np.integer))
            self.assertIsInstance(feats, list)

    def test_bgr_input(self):
        """BGR输入应自动转灰度"""
        ext = HaarFeatureExtractor()
        img = np.random.randint(0, 256, (200, 200, 3), dtype=np.uint8)
        results = ext.extract_all(img, window_size=(24, 24), step=50,
                                  scale_factor=1.5, n_scales=2)
        self.assertIsInstance(results, list)


# ==================== 特征向量测试 ====================

class TestExtractFeatureVector(unittest.TestCase):
    """特征向量提取测试"""

    def test_returns_ndarray(self):
        ext = HaarFeatureExtractor()
        img = make_test_image(100)
        feat = ext.extract_feature_vector(img, window_size=(24, 24))
        self.assertIsInstance(feat, np.ndarray)

    def test_dtype_float32(self):
        ext = HaarFeatureExtractor()
        img = make_test_image(100)
        feat = ext.extract_feature_vector(img, window_size=(24, 24))
        self.assertEqual(feat.dtype, np.float32)

    def test_nonzero_length(self):
        ext = HaarFeatureExtractor()
        img = make_test_image(100)
        feat = ext.extract_feature_vector(img, window_size=(24, 24))
        self.assertGreater(len(feat), 0)

    def test_bgr_input(self):
        ext = HaarFeatureExtractor()
        img = np.random.randint(0, 256, (100, 100, 3), dtype=np.uint8)
        feat = ext.extract_feature_vector(img, window_size=(24, 24))
        self.assertGreater(len(feat), 0)


# ==================== 特征计数测试 ====================

class TestCountFeatures(unittest.TestCase):
    """Haar特征计数测试"""

    def test_returns_int(self):
        ext = HaarFeatureExtractor()
        count = ext.count_features((24, 24))
        self.assertIsInstance(count, int)

    def test_positive_count(self):
        ext = HaarFeatureExtractor()
        count = ext.count_features((24, 24))
        self.assertGreater(count, 0)

    def test_more_types_more_features(self):
        """更多特征类型应产生更多特征"""
        ext5 = HaarFeatureExtractor()
        ext2 = HaarFeatureExtractor(feature_types=['two_horizontal', 'two_vertical'])
        count5 = ext5.count_features((24, 24))
        count2 = ext2.count_features((24, 24))
        self.assertGreater(count5, count2)

    def test_larger_window_more_features(self):
        """更大窗口应有更多特征"""
        ext = HaarFeatureExtractor()
        count_small = ext.count_features((12, 12))
        count_large = ext.count_features((24, 24))
        self.assertGreater(count_large, count_small)


# ==================== 便捷函数测试 ====================

class TestConvenienceFunctions(unittest.TestCase):
    """便捷函数测试"""

    def test_extract_haar_feature(self):
        img = make_test_image(100)
        feat = extract_haar_feature(img, window_size=(24, 24))
        self.assertIsInstance(feat, np.ndarray)
        self.assertGreater(len(feat), 0)


if __name__ == '__main__':
    unittest.main()
