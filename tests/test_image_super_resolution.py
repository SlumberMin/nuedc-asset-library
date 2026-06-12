#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
图像超分辨率单元测试
覆盖: bicubic_upscale双三次上采样、lanczos_upscale Lanczos上采样、
      area_downscale区域下采样、upscale_with_sharpen锐化放大、
      upscale_edge_guided边缘引导上采样、upscale_multi_pass多次放大、
      compute_psnr PSNR计算、create_test_degraded退化测试图像生成、
      upscale_dnn_espcn/edsr DNN回退测试
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from _10_视觉通用代码库.image_super_resolution import (
    bicubic_upscale,
    lanczos_upscale,
    area_downscale,
    upscale_with_sharpen,
    upscale_edge_guided,
    upscale_multi_pass,
    upscale_dnn_espcn,
    upscale_dnn_edsr,
    compute_psnr,
    create_test_degraded,
)


# ==================== 辅助函数 ====================

def _make_test_image(h=128, w=128):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    cv2.rectangle(img, (10, 10), (60, 60), (200, 100, 50), -1)
    cv2.circle(img, (90, 40), 25, (50, 200, 100), -1)
    cv2.putText(img, "SR", (30, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return img


# ==================== 双三次插值上采样测试 ====================

class TestBicubicUpscale(unittest.TestCase):
    """双三次插值上采样测试"""

    def test_returns_ndarray(self):
        img = _make_test_image(64, 64)
        result = bicubic_upscale(img, scale_factor=2)
        self.assertIsInstance(result, np.ndarray)

    def test_2x_size(self):
        img = _make_test_image(64, 64)
        result = bicubic_upscale(img, scale_factor=2)
        self.assertEqual(result.shape[:2], (128, 128))

    def test_4x_size(self):
        img = _make_test_image(32, 32)
        result = bicubic_upscale(img, scale_factor=4)
        self.assertEqual(result.shape[:2], (128, 128))

    def test_channels_preserved(self):
        img = _make_test_image(64, 64)
        result = bicubic_upscale(img, scale_factor=2)
        self.assertEqual(result.shape[2], 3)

    def test_dtype_preserved(self):
        img = _make_test_image(64, 64)
        result = bicubic_upscale(img, scale_factor=2)
        self.assertEqual(result.dtype, np.uint8)


# ==================== Lanczos上采样测试 ====================

class TestLanczosUpscale(unittest.TestCase):
    """Lanczos插值上采样测试"""

    def test_returns_ndarray(self):
        img = _make_test_image(64, 64)
        result = lanczos_upscale(img, scale_factor=2)
        self.assertIsInstance(result, np.ndarray)

    def test_2x_size(self):
        img = _make_test_image(64, 64)
        result = lanczos_upscale(img, scale_factor=2)
        self.assertEqual(result.shape[:2], (128, 128))

    def test_channels_preserved(self):
        img = _make_test_image(64, 64)
        result = lanczos_upscale(img, scale_factor=2)
        self.assertEqual(result.shape[2], 3)


# ==================== 区域下采样测试 ====================

class TestAreaDownscale(unittest.TestCase):
    """区域插值下采样测试"""

    def test_returns_ndarray(self):
        img = _make_test_image(128, 128)
        result = area_downscale(img, scale_factor=4)
        self.assertIsInstance(result, np.ndarray)

    def test_quarter_size(self):
        img = _make_test_image(128, 128)
        result = area_downscale(img, scale_factor=4)
        self.assertEqual(result.shape[:2], (32, 32))

    def test_half_size(self):
        img = _make_test_image(128, 128)
        result = area_downscale(img, scale_factor=2)
        self.assertEqual(result.shape[:2], (64, 64))


# ==================== 锐化放大测试 ====================

class TestUpscaleWithSharpen(unittest.TestCase):
    """插值放大+锐化测试"""

    def test_returns_ndarray(self):
        img = _make_test_image(64, 64)
        result = upscale_with_sharpen(img, scale_factor=2)
        self.assertIsInstance(result, np.ndarray)

    def test_2x_size(self):
        img = _make_test_image(64, 64)
        result = upscale_with_sharpen(img, scale_factor=2)
        self.assertEqual(result.shape[:2], (128, 128))

    def test_dtype_uint8(self):
        img = _make_test_image(64, 64)
        result = upscale_with_sharpen(img, scale_factor=2)
        self.assertEqual(result.dtype, np.uint8)

    def test_custom_sharpen_strength(self):
        img = _make_test_image(64, 64)
        result = upscale_with_sharpen(img, scale_factor=2, sharpen_strength=1.0)
        self.assertEqual(result.shape[:2], (128, 128))

    def test_more_sharpening_higher_contrast(self):
        """更强锐化应产生更高对比度"""
        img = _make_test_image(64, 64)
        result_low = upscale_with_sharpen(img, scale_factor=2, sharpen_strength=0.3)
        result_high = upscale_with_sharpen(img, scale_factor=2, sharpen_strength=2.0)
        lap_low = cv2.Laplacian(result_low, cv2.CV_64F).std()
        lap_high = cv2.Laplacian(result_high, cv2.CV_64F).std()
        self.assertGreater(lap_high, lap_low)


# ==================== 边缘引导上采样测试 ====================

class TestUpscaleEdgeGuided(unittest.TestCase):
    """边缘引导上采样测试"""

    def test_returns_ndarray(self):
        img = _make_test_image(64, 64)
        result = upscale_edge_guided(img, scale_factor=2)
        self.assertIsInstance(result, np.ndarray)

    def test_2x_size(self):
        img = _make_test_image(64, 64)
        result = upscale_edge_guided(img, scale_factor=2)
        self.assertEqual(result.shape[:2], (128, 128))

    def test_channels_preserved(self):
        img = _make_test_image(64, 64)
        result = upscale_edge_guided(img, scale_factor=2)
        self.assertEqual(result.shape[2], 3)


# ==================== 多次放大测试 ====================

class TestUpscaleMultiPass(unittest.TestCase):
    """多次小幅放大测试"""

    def test_returns_ndarray(self):
        img = _make_test_image(32, 32)
        result = upscale_multi_pass(img, target_scale=4, pass_scale=2)
        self.assertIsInstance(result, np.ndarray)

    def test_4x_size(self):
        img = _make_test_image(32, 32)
        result = upscale_multi_pass(img, target_scale=4, pass_scale=2)
        self.assertEqual(result.shape[:2], (128, 128))

    def test_2x_as_single_pass(self):
        img = _make_test_image(64, 64)
        result = upscale_multi_pass(img, target_scale=2, pass_scale=2)
        self.assertEqual(result.shape[:2], (128, 128))

    def test_dtype_uint8(self):
        img = _make_test_image(32, 32)
        result = upscale_multi_pass(img, target_scale=4, pass_scale=2)
        self.assertEqual(result.dtype, np.uint8)


# ==================== DNN回退测试 ====================

class TestUpscaleDNNEspcn(unittest.TestCase):
    """ESPCN DNN上采样测试（无模型时应回退到插值+锐化）"""

    def test_returns_ndarray_no_model(self):
        img = _make_test_image(64, 64)
        result = upscale_dnn_espcn(img, scale_factor=2, model_path="nonexistent.pb")
        self.assertIsInstance(result, np.ndarray)

    def test_size_no_model(self):
        img = _make_test_image(64, 64)
        result = upscale_dnn_espcn(img, scale_factor=2, model_path="nonexistent.pb")
        self.assertEqual(result.shape[:2], (128, 128))


class TestUpscaleDNNEdsr(unittest.TestCase):
    """EDSR DNN上采样测试（无模型时应回退到双三次插值）"""

    def test_returns_ndarray_no_model(self):
        img = _make_test_image(64, 64)
        result = upscale_dnn_edsr(img, scale_factor=2, model_path="nonexistent.pb")
        self.assertIsInstance(result, np.ndarray)

    def test_size_no_model(self):
        img = _make_test_image(64, 64)
        result = upscale_dnn_edsr(img, scale_factor=2, model_path="nonexistent.pb")
        self.assertEqual(result.shape[:2], (128, 128))


# ==================== PSNR计算测试 ====================

class TestComputePSNR(unittest.TestCase):
    """PSNR计算测试"""

    def test_identical_images_infinite(self):
        img = _make_test_image()
        psnr = compute_psnr(img, img)
        self.assertEqual(psnr, float('inf'))

    def test_returns_float(self):
        img1 = _make_test_image()
        img2 = _make_test_image()
        img2[0, 0, 0] = 128
        psnr = compute_psnr(img1, img2)
        self.assertIsInstance(psnr, float)

    def test_small_difference_high_psnr(self):
        img1 = np.full((64, 64, 3), 128, dtype=np.uint8)
        img2 = np.full((64, 64, 3), 128, dtype=np.uint8)
        img2[0, 0, 0] = 130  # 微小差异
        psnr = compute_psnr(img1, img2)
        self.assertGreater(psnr, 30.0)

    def test_large_difference_low_psnr(self):
        img1 = np.zeros((64, 64, 3), dtype=np.uint8)
        img2 = np.full((64, 64, 3), 255, dtype=np.uint8)
        psnr = compute_psnr(img1, img2)
        self.assertLess(psnr, 10.0)

    def test_symmetric(self):
        img1 = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        img2 = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        psnr1 = compute_psnr(img1, img2)
        psnr2 = compute_psnr(img2, img1)
        self.assertAlmostEqual(psnr1, psnr2, places=5)


# ==================== 退化测试图像生成测试 ====================

class TestCreateTestDegraded(unittest.TestCase):
    """退化测试图像生成测试"""

    def test_returns_tuple_of_2(self):
        img = _make_test_image(128, 128)
        result = create_test_degraded(img, scale_factor=4)
        self.assertEqual(len(result), 2)

    def test_lr_size(self):
        img = _make_test_image(128, 128)
        lr, hr = create_test_degraded(img, scale_factor=4)
        self.assertEqual(lr.shape[:2], (32, 32))

    def test_hr_is_original(self):
        img = _make_test_image(128, 128)
        lr, hr = create_test_degraded(img, scale_factor=4)
        np.testing.assert_array_equal(hr, img)

    def test_lr_smaller_than_hr(self):
        img = _make_test_image(128, 128)
        lr, hr = create_test_degraded(img, scale_factor=4)
        self.assertLess(lr.shape[0], hr.shape[0])

    def test_with_noise(self):
        img = _make_test_image(128, 128)
        lr_noisy, _ = create_test_degraded(img, scale_factor=4, noise_sigma=10)
        lr_clean, _ = create_test_degraded(img, scale_factor=4, noise_sigma=0)
        # 加噪后应不完全相同
        self.assertFalse(np.array_equal(lr_noisy, lr_clean))

    def test_without_noise(self):
        img = _make_test_image(128, 128)
        lr1, _ = create_test_degraded(img, scale_factor=4, noise_sigma=0)
        lr2, _ = create_test_degraded(img, scale_factor=4, noise_sigma=0)
        np.testing.assert_array_equal(lr1, lr2)

    def test_channels_preserved(self):
        img = _make_test_image(128, 128)
        lr, _ = create_test_degraded(img, scale_factor=4)
        self.assertEqual(lr.shape[2], 3)


# ==================== 超分辨率质量对比测试 ====================

class TestSRQualityComparison(unittest.TestCase):
    """超分辨率方法质量对比集成测试"""

    def test_all_methods_same_output_size(self):
        """所有方法输出尺寸应一致"""
        img = _make_test_image(64, 64)
        methods = {
            'bicubic': bicubic_upscale(img, 2),
            'lanczos': lanczos_upscale(img, 2),
            'sharpen': upscale_with_sharpen(img, 2),
            'edge_guided': upscale_edge_guided(img, 2),
            'multi_pass': upscale_multi_pass(img, 2),
        }
        for name, result in methods.items():
            self.assertEqual(result.shape[:2], (128, 128),
                             f"{name} 输出尺寸不正确")

    def test_roundtrip_psnr_reasonable(self):
        """下采样再上采样的PSNR应在合理范围"""
        img = _make_test_image(128, 128)
        lr, hr = create_test_degraded(img, scale_factor=4, noise_sigma=0)
        sr = bicubic_upscale(lr, scale_factor=4)
        sr_resized = cv2.resize(sr, (128, 128))
        psnr = compute_psnr(hr, sr_resized)
        self.assertGreater(psnr, 15.0)


if __name__ == '__main__':
    unittest.main()
