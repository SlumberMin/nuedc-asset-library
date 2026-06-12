#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LBP特征单元测试
覆盖: LBPFeatureExtractor初始化、默认LBP、均匀LBP、旋转不变LBP、
      LBP直方图、特征比较、便捷函数
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from _10_视觉通用代码库.lbp_feature import (
    LBPFeatureExtractor,
    extract_lbp_feature,
    compute_lbp_image,
)


def make_horizontal_stripes(h=100, w=100, stripe_h=10):
    """水平条纹纹理"""
    img = np.zeros((h, w), dtype=np.uint8)
    for i in range(0, h, stripe_h * 2):
        img[i:i + stripe_h, :] = 200
    return img


def make_checkerboard(h=100, w=100, block=20):
    """棋盘格纹理"""
    img = np.zeros((h, w), dtype=np.uint8)
    for i in range(0, h, block):
        for j in range(0, w, block):
            if (i // block + j // block) % 2 == 0:
                img[i:i + block, j:j + block] = 200
    return img


# ==================== 初始化测试 ====================

class TestLBPFeatureExtractorInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        ext = LBPFeatureExtractor()
        self.assertEqual(ext.radius, 1)
        self.assertEqual(ext.n_points, 8)
        self.assertEqual(ext.method, 'uniform')

    def test_custom_params(self):
        ext = LBPFeatureExtractor(radius=2, n_points=16, method='default')
        self.assertEqual(ext.radius, 2)
        self.assertEqual(ext.n_points, 16)
        self.assertEqual(ext.method, 'default')


# ==================== LBP计算测试 ====================

class TestLBPCompute(unittest.TestCase):
    """LBP图像计算测试"""

    def test_default_method(self):
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='default')
        img = make_horizontal_stripes()
        lbp = ext.compute(img)
        self.assertIsNotNone(lbp)
        self.assertEqual(len(lbp.shape), 2)

    def test_uniform_method(self):
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')
        img = make_horizontal_stripes()
        lbp = ext.compute(img)
        self.assertIsNotNone(lbp)

    def test_ri_method(self):
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='ri')
        img = make_horizontal_stripes()
        lbp = ext.compute(img)
        self.assertIsNotNone(lbp)

    def test_riu2_method(self):
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='riu2')
        img = make_horizontal_stripes()
        lbp = ext.compute(img)
        self.assertIsNotNone(lbp)

    def test_bgr_input(self):
        """BGR输入应自动转灰度"""
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')
        img = np.random.randint(0, 256, (100, 100, 3), dtype=np.uint8)
        lbp = ext.compute(img)
        self.assertIsNotNone(lbp)

    def test_output_shape_smaller(self):
        """输出应比输入小(边界裁剪)"""
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='default')
        img = np.random.randint(0, 256, (100, 100), dtype=np.uint8)
        lbp = ext.compute(img)
        self.assertLess(lbp.shape[0], img.shape[0])
        self.assertLess(lbp.shape[1], img.shape[1])

    def test_unknown_method_raises(self):
        ext = LBPFeatureExtractor(method='invalid')
        img = np.zeros((50, 50), dtype=np.uint8)
        with self.assertRaises(ValueError):
            ext.compute(img)

    def test_uniform_output_range(self):
        """均匀LBP输出应在合理范围内"""
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')
        img = np.random.randint(0, 256, (100, 100), dtype=np.uint8)
        lbp = ext.compute(img)
        self.assertGreaterEqual(lbp.min(), 0)
        self.assertLessEqual(lbp.max(), 59)  # 8点均匀模式最多58+1类


# ==================== 均匀模式判断测试 ====================

class TestIsUniform(unittest.TestCase):
    """均匀模式判断测试"""

    def test_all_zeros_is_uniform(self):
        ext = LBPFeatureExtractor(n_points=8)
        self.assertTrue(ext._is_uniform(0))  # 00000000

    def test_all_ones_is_uniform(self):
        ext = LBPFeatureExtractor(n_points=8)
        self.assertTrue(ext._is_uniform(255))  # 11111111

    def test_single_transition_is_uniform(self):
        """00001111 只有一次跳变"""
        ext = LBPFeatureExtractor(n_points=8)
        self.assertTrue(ext._is_uniform(15))  # 00001111

    def test_two_transitions_is_uniform(self):
        """00111100 两次跳变"""
        ext = LBPFeatureExtractor(n_points=8)
        self.assertTrue(ext._is_uniform(60))  # 00111100

    def test_alternating_not_uniform(self):
        """01010101 多次跳变,非均匀"""
        ext = LBPFeatureExtractor(n_points=8)
        self.assertFalse(ext._is_uniform(85))  # 01010101


# ==================== 直方图提取测试 ====================

class TestExtractHistogram(unittest.TestCase):
    """LBP直方图提取测试"""

    def test_returns_ndarray(self):
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')
        img = make_horizontal_stripes()
        feat = ext.extract_histogram(img)
        self.assertIsInstance(feat, np.ndarray)

    def test_single_block(self):
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')
        img = make_horizontal_stripes()
        feat = ext.extract_histogram(img, n_div_x=1, n_div_y=1)
        self.assertGreater(len(feat), 0)

    def test_multi_block(self):
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')
        img = make_horizontal_stripes()
        feat_single = ext.extract_histogram(img, n_div_x=1, n_div_y=1)
        feat_multi = ext.extract_histogram(img, n_div_x=4, n_div_y=4)
        # 分块后特征更长
        self.assertGreater(len(feat_multi), len(feat_single))

    def test_normalized_histogram(self):
        """单块直方图应归一化"""
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')
        img = make_horizontal_stripes()
        feat = ext.extract_histogram(img, n_div_x=1, n_div_y=1)
        self.assertAlmostEqual(np.sum(feat), 1.0, delta=0.01)

    def test_bgr_input(self):
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')
        img = np.random.randint(0, 256, (100, 100, 3), dtype=np.uint8)
        feat = ext.extract_histogram(img)
        self.assertGreater(len(feat), 0)


# ==================== 特征比较测试 ====================

class TestLBPCompare(unittest.TestCase):
    """LBP特征比较测试"""

    def test_chi_square_self(self):
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')
        img = make_horizontal_stripes()
        feat = ext.extract_histogram(img)
        dist = ext.compare(feat, feat, method='chi_square')
        self.assertAlmostEqual(dist, 0.0, delta=0.01)

    def test_euclidean_self_zero(self):
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')
        img = make_horizontal_stripes()
        feat = ext.extract_histogram(img)
        dist = ext.compare(feat, feat, method='euclidean')
        self.assertAlmostEqual(dist, 0.0, delta=0.01)

    def test_hist_intersection_self(self):
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')
        img = make_horizontal_stripes()
        feat = ext.extract_histogram(img)
        dist = ext.compare(feat, feat, method='hist_intersection')
        self.assertAlmostEqual(dist, 0.0, delta=0.01)

    def test_different_textures(self):
        """不同纹理应有较大距离"""
        ext = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')
        feat1 = ext.extract_histogram(make_horizontal_stripes())
        feat2 = ext.extract_histogram(make_checkerboard())
        dist = ext.compare(feat1, feat2, method='chi_square')
        self.assertGreater(dist, 0.0)

    def test_unknown_method_raises(self):
        ext = LBPFeatureExtractor()
        feat = ext.extract_histogram(make_horizontal_stripes())
        with self.assertRaises(ValueError):
            ext.compare(feat, feat, method='unknown')


# ==================== 便捷函数测试 ====================

class TestConvenienceFunctions(unittest.TestCase):
    """便捷函数测试"""

    def test_extract_lbp_feature(self):
        img = make_horizontal_stripes()
        feat = extract_lbp_feature(img, radius=1, n_points=8, grid_x=4, grid_y=4)
        self.assertIsInstance(feat, np.ndarray)
        self.assertGreater(len(feat), 0)

    def test_compute_lbp_image(self):
        img = make_horizontal_stripes()
        lbp = compute_lbp_image(img, radius=1, n_points=8)
        self.assertIsNotNone(lbp)
        self.assertEqual(len(lbp.shape), 2)


if __name__ == '__main__':
    unittest.main()
