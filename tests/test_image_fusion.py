#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
图像融合单元测试
覆盖: laplacian_pyramid_blend金字塔融合、multi_focus_fusion多聚焦融合、
      exposure_fusion多曝光融合、hdr_tone_mapping HDR合成、
      weighted_fusion加权融合、simple_hdr_from_bracket
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from _10_视觉通用代码库.image_fusion import (
    laplacian_pyramid_blend,
    multi_focus_fusion,
    exposure_fusion,
    hdr_tone_mapping,
    weighted_fusion,
    simple_hdr_from_bracket,
)


# ==================== 辅助函数 ====================

def _make_image_circle(h=256, w=256):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    cv2.circle(img, (w // 2, h // 2), 80, (255, 100, 50), -1)
    return img


def _make_image_rect(h=256, w=256):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    cv2.rectangle(img, (40, 40), (w - 40, h - 40), (50, 100, 255), -1)
    return img


def _make_mask(h=256, w=256):
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[:, :w // 2] = 255
    return mask


def _make_multi_exposure(n=3):
    """创建不同曝光的测试图像序列"""
    base = np.random.randint(0, 200, (64, 64, 3), dtype=np.uint8)
    images = []
    for i in range(n):
        factor = 0.5 + i * 0.5
        img = np.clip(base.astype(np.float32) * factor, 0, 255).astype(np.uint8)
        images.append(img)
    return images


# ==================== 拉普拉斯金字塔融合测试 ====================

class TestLaplacianPyramidBlend(unittest.TestCase):
    """拉普拉斯金字塔融合测试"""

    def test_returns_ndarray(self):
        img1 = _make_image_circle()
        img2 = _make_image_rect()
        mask = _make_mask()
        result = laplacian_pyramid_blend(img1, img2, mask)
        self.assertIsInstance(result, np.ndarray)

    def test_output_shape(self):
        img1 = _make_image_circle(128, 128)
        img2 = _make_image_rect(128, 128)
        mask = _make_mask(128, 128)
        result = laplacian_pyramid_blend(img1, img2, mask)
        self.assertEqual(result.shape, img1.shape)

    def test_output_dtype_uint8(self):
        img1 = _make_image_circle()
        img2 = _make_image_rect()
        mask = _make_mask()
        result = laplacian_pyramid_blend(img1, img2, mask)
        self.assertEqual(result.dtype, np.uint8)

    def test_left_region_uses_img1(self):
        """mask白色区域应主要取img1"""
        h, w = 64, 64
        img1 = np.full((h, w, 3), 200, dtype=np.uint8)
        img2 = np.full((h, w, 3), 50, dtype=np.uint8)
        mask = np.full((h, w), 255, dtype=np.uint8)  # 全白 → 取img1
        result = laplacian_pyramid_blend(img1, img2, mask, levels=3)
        # 整体均值应更接近img1
        self.assertGreater(np.mean(result), 100)

    def test_right_region_uses_img2(self):
        """mask黑色区域应主要取img2"""
        h, w = 64, 64
        img1 = np.full((h, w, 3), 200, dtype=np.uint8)
        img2 = np.full((h, w, 3), 50, dtype=np.uint8)
        mask = np.zeros((h, w), dtype=np.uint8)  # 全黑 → 取img2
        result = laplacian_pyramid_blend(img1, img2, mask, levels=3)
        self.assertLess(np.mean(result), 100)

    def test_different_levels(self):
        img1 = _make_image_circle(128, 128)
        img2 = _make_image_rect(128, 128)
        mask = _make_mask(128, 128)
        for lv in [2, 3, 4]:
            result = laplacian_pyramid_blend(img1, img2, mask, levels=lv)
            self.assertEqual(result.shape, img1.shape)

    def test_size_mismatch_raises(self):
        img1 = _make_image_circle(64, 64)
        img2 = _make_image_rect(128, 128)
        mask = _make_mask(64, 64)
        with self.assertRaises(AssertionError):
            laplacian_pyramid_blend(img1, img2, mask)


# ==================== 多聚焦融合测试 ====================

class TestMultiFocusFusion(unittest.TestCase):
    """多聚焦融合测试"""

    def test_returns_tuple_of_2(self):
        img1 = _make_image_circle()
        img2 = _make_image_rect()
        result = multi_focus_fusion(img1, img2)
        self.assertEqual(len(result), 2)

    def test_fused_shape(self):
        img1 = _make_image_circle()
        img2 = _make_image_rect()
        fused, mask = multi_focus_fusion(img1, img2)
        self.assertEqual(fused.shape, img1.shape)

    def test_mask_is_uint8(self):
        img1 = _make_image_circle()
        img2 = _make_image_rect()
        _, mask = multi_focus_fusion(img1, img2)
        self.assertEqual(mask.dtype, np.uint8)

    def test_mask_shape(self):
        img1 = _make_image_circle()
        img2 = _make_image_rect()
        _, mask = multi_focus_fusion(img1, img2)
        self.assertEqual(mask.shape[:2], img1.shape[:2])

    def test_grayscale_input(self):
        img1 = np.random.randint(0, 255, (64, 64), dtype=np.uint8)
        img2 = np.random.randint(0, 255, (64, 64), dtype=np.uint8)
        fused, mask = multi_focus_fusion(img1, img2, block_size=11)
        self.assertEqual(fused.shape, img1.shape + (3,))  # 输出仍可能是彩色


# ==================== 加权融合测试 ====================

class TestWeightedFusion(unittest.TestCase):
    """加权融合测试"""

    def test_returns_ndarray(self):
        img1 = _make_image_circle()
        img2 = _make_image_rect()
        result = weighted_fusion(img1, img2, alpha=0.5)
        self.assertIsInstance(result, np.ndarray)

    def test_output_shape(self):
        img1 = _make_image_circle()
        img2 = _make_image_rect()
        result = weighted_fusion(img1, img2, alpha=0.5)
        self.assertEqual(result.shape, img1.shape)

    def test_alpha_1_returns_img1(self):
        """alpha=1时应完全取img1"""
        img1 = np.full((64, 64, 3), 200, dtype=np.uint8)
        img2 = np.full((64, 64, 3), 50, dtype=np.uint8)
        result = weighted_fusion(img1, img2, alpha=1.0)
        np.testing.assert_array_equal(result, img1)

    def test_alpha_0_returns_img2(self):
        """alpha=0时应完全取img2"""
        img1 = np.full((64, 64, 3), 200, dtype=np.uint8)
        img2 = np.full((64, 64, 3), 50, dtype=np.uint8)
        result = weighted_fusion(img1, img2, alpha=0.0)
        np.testing.assert_array_equal(result, img2)

    def test_alpha_half(self):
        """alpha=0.5时应取均值"""
        img1 = np.full((64, 64, 3), 200, dtype=np.uint8)
        img2 = np.full((64, 64, 3), 100, dtype=np.uint8)
        result = weighted_fusion(img1, img2, alpha=0.5)
        self.assertAlmostEqual(np.mean(result), 150, delta=2)


# ==================== 多曝光融合测试 ====================

class TestExposureFusion(unittest.TestCase):
    """多曝光融合测试"""

    def test_returns_ndarray(self):
        images = _make_multi_exposure(3)
        result = exposure_fusion(images)
        self.assertIsInstance(result, np.ndarray)

    def test_output_shape(self):
        images = _make_multi_exposure(3)
        result = exposure_fusion(images)
        self.assertEqual(result.shape, images[0].shape)

    def test_output_dtype_uint8(self):
        images = _make_multi_exposure(3)
        result = exposure_fusion(images)
        self.assertEqual(result.dtype, np.uint8)

    def test_different_weight_params(self):
        images = _make_multi_exposure(3)
        result = exposure_fusion(images, contrast_weight=2.0, saturation_weight=0.5)
        self.assertEqual(result.shape, images[0].shape)


# ==================== HDR色调映射测试 ====================

class TestHDRToneMapping(unittest.TestCase):
    """HDR色调映射测试"""

    def test_returns_ndarray(self):
        images = _make_multi_exposure(3)
        times = [0.01, 0.04, 0.16]
        result = hdr_tone_mapping(images, times, method="reinhard")
        self.assertIsInstance(result, np.ndarray)

    def test_output_dtype_uint8(self):
        images = _make_multi_exposure(3)
        times = [0.01, 0.04, 0.16]
        result = hdr_tone_mapping(images, times, method="reinhard")
        self.assertEqual(result.dtype, np.uint8)

    def test_different_methods(self):
        images = _make_multi_exposure(3)
        times = [0.01, 0.04, 0.16]
        for method in ["reinhard", "drago", "linear"]:
            result = hdr_tone_mapping(images, times, method=method)
            self.assertEqual(result.shape, images[0].shape)


# ==================== 简化HDR测试 ====================

class TestSimpleHDR(unittest.TestCase):
    """简化版HDR测试"""

    def test_returns_ndarray(self):
        images = _make_multi_exposure(3)
        result = simple_hdr_from_bracket(images)
        self.assertIsInstance(result, np.ndarray)

    def test_output_shape(self):
        images = _make_multi_exposure(3)
        result = simple_hdr_from_bracket(images)
        self.assertEqual(result.shape, images[0].shape)


if __name__ == '__main__':
    unittest.main()
