#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
颜色矩特征单元测试
覆盖: ColorMomentExtractor初始化、颜色空间转换、单通道矩计算、
      特征提取、掩码提取、ROI提取、特征比较、描述输出、便捷函数
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from _10_视觉通用代码库.color_moment import (
    ColorMomentExtractor,
    extract_color_moments,
    compare_by_color_moment,
)


def make_solid_image(bgr, size=100):
    """创建纯色图像"""
    img = np.zeros((size, size, 3), dtype=np.uint8)
    img[:] = bgr
    return img


# ==================== 初始化测试 ====================

class TestColorMomentExtractorInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        ext = ColorMomentExtractor()
        self.assertEqual(ext.channels, 'BGR')
        self.assertEqual(ext.n_moments, 3)

    def test_hsv_channels(self):
        ext = ColorMomentExtractor(channels='HSV')
        self.assertEqual(ext.channels, 'HSV')

    def test_lab_channels(self):
        ext = ColorMomentExtractor(channels='LAB')
        self.assertEqual(ext.channels, 'LAB')

    def test_two_moments(self):
        ext = ColorMomentExtractor(n_moments=2)
        self.assertEqual(ext.n_moments, 2)


# ==================== 颜色空间转换测试 ====================

class TestColorConversion(unittest.TestCase):
    """颜色空间转换测试"""

    def test_bgr_identity(self):
        ext = ColorMomentExtractor(channels='BGR')
        img = make_solid_image((100, 150, 200))
        converted = ext._convert_color(img)
        np.testing.assert_array_equal(converted, img)

    def test_hsv_conversion(self):
        ext = ColorMomentExtractor(channels='HSV')
        img = make_solid_image((100, 150, 200))
        converted = ext._convert_color(img)
        self.assertEqual(converted.shape, img.shape)

    def test_lab_conversion(self):
        ext = ColorMomentExtractor(channels='LAB')
        img = make_solid_image((100, 150, 200))
        converted = ext._convert_color(img)
        self.assertEqual(converted.shape, img.shape)


# ==================== 单通道矩计算测试 ====================

class TestSingleChannelMoments(unittest.TestCase):
    """单通道矩计算测试"""

    def test_uniform_channel(self):
        """均匀通道: 均值应等于该值, 标准差为0"""
        ext = ColorMomentExtractor(n_moments=3)
        channel = np.full((100, 100), 128, dtype=np.uint8)
        moments = ext._calc_moments_single_channel(channel)
        self.assertAlmostEqual(moments[0], 128.0, delta=0.1)
        self.assertAlmostEqual(moments[1], 0.0, delta=0.1)
        self.assertAlmostEqual(moments[2], 0.0, delta=0.1)

    def test_one_moment_only(self):
        """仅一阶矩"""
        ext = ColorMomentExtractor(n_moments=1)
        channel = np.full((50, 50), 200, dtype=np.uint8)
        moments = ext._calc_moments_single_channel(channel)
        self.assertEqual(len(moments), 1)
        self.assertAlmostEqual(moments[0], 200.0, delta=0.1)

    def test_two_moments(self):
        """一阶+二阶矩"""
        ext = ColorMomentExtractor(n_moments=2)
        channel = np.random.randint(0, 256, (50, 50), dtype=np.uint8)
        moments = ext._calc_moments_single_channel(channel)
        self.assertEqual(len(moments), 2)

    def test_three_moments(self):
        """一阶+二阶+三阶矩"""
        ext = ColorMomentExtractor(n_moments=3)
        channel = np.random.randint(0, 256, (50, 50), dtype=np.uint8)
        moments = ext._calc_moments_single_channel(channel)
        self.assertEqual(len(moments), 3)


# ==================== 特征提取测试 ====================

class TestExtract(unittest.TestCase):
    """特征提取测试"""

    def test_output_shape_bgr(self):
        ext = ColorMomentExtractor(channels='BGR', n_moments=3)
        img = make_solid_image((100, 150, 200))
        feat = ext.extract(img)
        self.assertEqual(feat.shape, (9,))  # 3通道 * 3矩

    def test_output_shape_two_moments(self):
        ext = ColorMomentExtractor(channels='BGR', n_moments=2)
        img = make_solid_image((100, 150, 200))
        feat = ext.extract(img)
        self.assertEqual(feat.shape, (6,))  # 3通道 * 2矩

    def test_output_dtype(self):
        ext = ColorMomentExtractor()
        img = make_solid_image((100, 150, 200))
        feat = ext.extract(img)
        self.assertEqual(feat.dtype, np.float32)

    def test_pure_red_mean(self):
        """纯红图像: B=0, G=0, R=255"""
        ext = ColorMomentExtractor(channels='BGR', n_moments=1)
        img = make_solid_image((0, 0, 255))
        feat = ext.extract(img)
        self.assertAlmostEqual(feat[0], 0.0, delta=1.0)    # B
        self.assertAlmostEqual(feat[1], 0.0, delta=1.0)    # G
        self.assertAlmostEqual(feat[2], 255.0, delta=1.0)  # R

    def test_pure_green_mean(self):
        ext = ColorMomentExtractor(channels='BGR', n_moments=1)
        img = make_solid_image((0, 255, 0))
        feat = ext.extract(img)
        self.assertAlmostEqual(feat[1], 255.0, delta=1.0)  # G

    def test_std_of_uniform_is_zero(self):
        """纯色图标准差应为零"""
        ext = ColorMomentExtractor(channels='BGR', n_moments=2)
        img = make_solid_image((128, 128, 128))
        feat = ext.extract(img)
        # 第二个值是B通道标准差
        self.assertAlmostEqual(feat[1], 0.0, delta=0.1)

    def test_with_mask(self):
        """掩码提取"""
        ext = ColorMomentExtractor()
        img = make_solid_image((0, 0, 255))
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[25:75, 25:75] = 255
        feat = ext.extract(img, mask=mask)
        self.assertEqual(feat.shape[0], 9)

    def test_empty_mask(self):
        """空掩码应返回全零"""
        ext = ColorMomentExtractor()
        img = make_solid_image((0, 0, 255))
        mask = np.zeros((100, 100), dtype=np.uint8)
        feat = ext.extract(img, mask=mask)
        np.testing.assert_array_equal(feat, 0.0)


# ==================== ROI提取测试 ====================

class TestExtractFromROI(unittest.TestCase):
    """ROI提取测试"""

    def test_rect_roi(self):
        ext = ColorMomentExtractor()
        img = make_solid_image((0, 0, 255))
        feat = ext.extract_from_roi(img, (10, 10, 50, 50))
        self.assertEqual(feat.shape[0], 9)

    def test_polygon_roi(self):
        ext = ColorMomentExtractor()
        img = make_solid_image((0, 0, 255))
        pts = np.array([[10, 10], [60, 10], [60, 60], [10, 60]])
        feat = ext.extract_from_roi(img, pts)
        self.assertEqual(feat.shape[0], 9)


# ==================== 特征比较测试 ====================

class TestCompare(unittest.TestCase):
    """特征比较测试"""

    def test_euclidean_self_zero(self):
        ext = ColorMomentExtractor()
        img = make_solid_image((100, 150, 200))
        feat = ext.extract(img)
        dist = ext.compare(feat, feat, method='euclidean')
        self.assertAlmostEqual(dist, 0.0, delta=0.01)

    def test_cosine_self_similarity(self):
        ext = ColorMomentExtractor()
        img = make_solid_image((100, 150, 200))
        feat = ext.extract(img)
        dist = ext.compare(feat, feat, method='cosine')
        self.assertAlmostEqual(dist, 0.0, delta=0.01)

    def test_manhattan_self_zero(self):
        ext = ColorMomentExtractor()
        img = make_solid_image((100, 150, 200))
        feat = ext.extract(img)
        dist = ext.compare(feat, feat, method='manhattan')
        self.assertAlmostEqual(dist, 0.0, delta=0.01)

    def test_different_colors_high_distance(self):
        """不同颜色应有较大距离"""
        ext = ColorMomentExtractor()
        feat_red = ext.extract(make_solid_image((0, 0, 255)))
        feat_blue = ext.extract(make_solid_image((255, 0, 0)))
        dist = ext.compare(feat_red, feat_blue, method='euclidean')
        self.assertGreater(dist, 10.0)

    def test_unknown_method_raises(self):
        ext = ColorMomentExtractor()
        feat = ext.extract(make_solid_image((100, 100, 100)))
        with self.assertRaises(ValueError):
            ext.compare(feat, feat, method='unknown')


# ==================== 描述测试 ====================

class TestDescribe(unittest.TestCase):
    """描述输出测试"""

    def test_returns_dict(self):
        ext = ColorMomentExtractor(channels='BGR', n_moments=3)
        feat = ext.extract(make_solid_image((100, 150, 200)))
        desc = ext.describe(feat)
        self.assertIsInstance(desc, dict)

    def test_channel_keys(self):
        ext = ColorMomentExtractor(channels='BGR', n_moments=3)
        feat = ext.extract(make_solid_image((100, 150, 200)))
        desc = ext.describe(feat)
        self.assertIn('B', desc)
        self.assertIn('G', desc)
        self.assertIn('R', desc)

    def test_moment_keys(self):
        ext = ColorMomentExtractor(channels='BGR', n_moments=3)
        feat = ext.extract(make_solid_image((100, 150, 200)))
        desc = ext.describe(feat)
        for ch in desc:
            self.assertIn('均值', desc[ch])
            self.assertIn('标准差', desc[ch])
            self.assertIn('偏度', desc[ch])


# ==================== 便捷函数测试 ====================

class TestConvenienceFunctions(unittest.TestCase):
    """便捷函数测试"""

    def test_extract_color_moments(self):
        img = make_solid_image((100, 150, 200))
        feat = extract_color_moments(img, space='BGR', n=3)
        self.assertEqual(feat.shape[0], 9)

    def test_compare_by_color_moment(self):
        img1 = make_solid_image((100, 150, 200))
        img2 = make_solid_image((100, 150, 200))
        dist = compare_by_color_moment(img1, img2)
        self.assertAlmostEqual(dist, 0.0, delta=1.0)

    def test_different_images_nonzero(self):
        img1 = make_solid_image((0, 0, 255))
        img2 = make_solid_image((255, 0, 0))
        dist = compare_by_color_moment(img1, img2)
        self.assertGreater(dist, 0.0)


if __name__ == '__main__':
    unittest.main()
