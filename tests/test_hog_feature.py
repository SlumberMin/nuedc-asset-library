#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HOG特征单元测试
覆盖: HOGFeatureExtractor初始化、特征提取、密集提取、
      特征比较、灰度/彩色输入、便捷函数
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from _10_视觉通用代码库.hog_feature import (
    HOGFeatureExtractor,
    extract_hog,
)


# ==================== 初始化测试 ====================

class TestHOGFeatureExtractorInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        ext = HOGFeatureExtractor()
        self.assertEqual(ext.win_size, (64, 128))
        self.assertEqual(ext.block_size, (16, 16))
        self.assertEqual(ext.block_stride, (8, 8))
        self.assertEqual(ext.cell_size, (8, 8))
        self.assertEqual(ext.nbins, 9)

    def test_custom_params(self):
        ext = HOGFeatureExtractor(win_size=(32, 64), nbins=12)
        self.assertEqual(ext.win_size, (32, 64))
        self.assertEqual(ext.nbins, 12)

    def test_hog_object_created(self):
        ext = HOGFeatureExtractor()
        self.assertIsNotNone(ext.hog)


# ==================== 特征提取测试 ====================

class TestHOGExtract(unittest.TestCase):
    """特征提取测试"""

    def test_returns_ndarray(self):
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.zeros((200, 100, 3), dtype=np.uint8)
        feat = ext.extract(img)
        self.assertIsInstance(feat, np.ndarray)

    def test_output_dim_consistent(self):
        """相同参数应产生相同维度"""
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img1 = np.random.randint(0, 256, (200, 100, 3), dtype=np.uint8)
        img2 = np.random.randint(0, 256, (150, 80, 3), dtype=np.uint8)
        feat1 = ext.extract(img1)
        feat2 = ext.extract(img2)
        self.assertEqual(feat1.shape, feat2.shape)

    def test_bgr_input(self):
        """BGR输入应正常工作"""
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.random.randint(0, 256, (200, 100, 3), dtype=np.uint8)
        feat = ext.extract(img)
        self.assertGreater(len(feat), 0)

    def test_grayscale_input(self):
        """灰度输入应正常工作"""
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.random.randint(0, 256, (200, 100), dtype=np.uint8)
        feat = ext.extract(img)
        self.assertGreater(len(feat), 0)

    def test_solid_image_features(self):
        """纯色图像特征应全为零(无梯度)"""
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.full((200, 100, 3), 128, dtype=np.uint8)
        feat = ext.extract(img)
        self.assertAlmostEqual(np.sum(np.abs(feat)), 0.0, delta=1.0)

    def test_edge_image_nonzero(self):
        """有边缘的图像应产生非零特征"""
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.zeros((200, 100, 3), dtype=np.uint8)
        cv2.rectangle(img, (30, 50), (70, 150), (255, 255, 255), -1)
        feat = ext.extract(img)
        self.assertGreater(np.sum(np.abs(feat)), 0.0)

    def test_custom_win_size(self):
        ext = HOGFeatureExtractor(win_size=(32, 64))
        img = np.random.randint(0, 256, (100, 80, 3), dtype=np.uint8)
        feat = ext.extract(img)
        self.assertGreater(len(feat), 0)


# ==================== 密集提取测试 ====================

class TestHOGExtractDense(unittest.TestCase):
    """密集HOG提取测试"""

    def test_returns_list(self):
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.zeros((300, 200, 3), dtype=np.uint8)
        features = ext.extract_dense(img, step_size=32)
        self.assertIsInstance(features, list)

    def test_each_element_is_tuple(self):
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.zeros((300, 200, 3), dtype=np.uint8)
        features = ext.extract_dense(img, step_size=64)
        if len(features) > 0:
            self.assertEqual(len(features[0]), 3)  # (x, y, feat)

    def test_positions_within_bounds(self):
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.zeros((300, 200, 3), dtype=np.uint8)
        features = ext.extract_dense(img, step_size=32)
        h, w = img.shape[:2]
        for x, y, feat in features:
            self.assertGreaterEqual(x, 0)
            self.assertGreaterEqual(y, 0)
            self.assertLessEqual(x + 64, w)
            self.assertLessEqual(y + 128, h)


# ==================== 特征比较测试 ====================

class TestHOGCompare(unittest.TestCase):
    """HOG特征比较测试"""

    def test_cosine_self_zero(self):
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.random.randint(0, 256, (200, 100, 3), dtype=np.uint8)
        feat = ext.extract(img)
        dist = ext.compare(feat, feat, method='cosine')
        self.assertAlmostEqual(dist, 0.0, delta=0.01)

    def test_euclidean_self_zero(self):
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.random.randint(0, 256, (200, 100, 3), dtype=np.uint8)
        feat = ext.extract(img)
        dist = ext.compare(feat, feat, method='euclidean')
        self.assertAlmostEqual(dist, 0.0, delta=0.01)

    def test_correlation_self_zero(self):
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.random.randint(0, 256, (200, 100, 3), dtype=np.uint8)
        feat = ext.extract(img)
        dist = ext.compare(feat, feat, method='correlation')
        self.assertAlmostEqual(dist, 0.0, delta=0.01)

    def test_different_images_positive_distance(self):
        """不同图像应有正距离"""
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img1 = np.zeros((200, 100, 3), dtype=np.uint8)
        cv2.rectangle(img1, (20, 40), (80, 160), 255, -1)
        img2 = np.zeros((200, 100, 3), dtype=np.uint8)
        cv2.circle(img2, (50, 100), 40, 255, -1)
        feat1 = ext.extract(img1)
        feat2 = ext.extract(img2)
        dist = ext.compare(feat1, feat2, method='cosine')
        self.assertGreater(dist, 0.0)

    def test_unknown_method_raises(self):
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.zeros((200, 100, 3), dtype=np.uint8)
        feat = ext.extract(img)
        with self.assertRaises(ValueError):
            ext.compare(feat, feat, method='unknown')


# ==================== 可视化测试 ====================

class TestHOGVisualize(unittest.TestCase):
    """HOG可视化测试"""

    def test_returns_image(self):
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.zeros((200, 100, 3), dtype=np.uint8)
        vis = ext.visualize(img)
        self.assertIsNotNone(vis)
        self.assertEqual(len(vis.shape), 2)  # 灰度图

    def test_scaled_output(self):
        ext = HOGFeatureExtractor(win_size=(64, 128))
        img = np.zeros((200, 100, 3), dtype=np.uint8)
        vis1 = ext.visualize(img, scale=1.0)
        vis2 = ext.visualize(img, scale=2.0)
        self.assertGreater(vis2.shape[0], vis1.shape[0])


# ==================== 便捷函数测试 ====================

class TestConvenienceFunctions(unittest.TestCase):
    """便捷函数测试"""

    def test_extract_hog(self):
        img = np.zeros((200, 100, 3), dtype=np.uint8)
        feat = extract_hog(img, win_size=(64, 128))
        self.assertIsInstance(feat, np.ndarray)
        self.assertGreater(len(feat), 0)

    def test_extract_hog_grayscale(self):
        img = np.zeros((200, 100), dtype=np.uint8)
        feat = extract_hog(img, win_size=(64, 128))
        self.assertGreater(len(feat), 0)


if __name__ == '__main__':
    unittest.main()
