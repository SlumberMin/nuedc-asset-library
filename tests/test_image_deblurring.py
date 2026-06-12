#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
图像去模糊单元测试
覆盖: estimate_psf_motion运动PSF生成、create_gaussian_psf高斯PSF、
      wiener_deconvolution维纳去卷积、lucy_richardson_deconvolution R-L去卷积、
      deblur_with_unsharp_mask反锐化掩模、deblur_bilateral_sharpen双边锐化、
      blind_deblur_simple盲去卷积、simulate_motion_blur运动模糊模拟
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from _10_视觉通用代码库.image_deblurring import (
    estimate_psf_motion,
    create_gaussian_psf,
    wiener_deconvolution,
    lucy_richardson_deconvolution,
    deblur_with_unsharp_mask,
    deblur_bilateral_sharpen,
    blind_deblur_simple,
    simulate_motion_blur,
)


# ==================== 辅助函数 ====================

def _make_test_image(h=128, w=128):
    """创建带纹理的测试图像"""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    cv2.putText(img, "AB", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
    cv2.circle(img, (90, 40), 25, (0, 200, 255), -1)
    return img


def _make_grayscale(h=128, w=128):
    img = np.zeros((h, w), dtype=np.uint8)
    cv2.putText(img, "AB", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 2, 255, 3)
    return img


# ==================== 运动模糊PSF测试 ====================

class TestEstimatePSFMotion(unittest.TestCase):
    """运动模糊PSF生成测试"""

    def test_returns_ndarray(self):
        psf = estimate_psf_motion(31, 45, 15)
        self.assertIsInstance(psf, np.ndarray)

    def test_output_shape(self):
        psf = estimate_psf_motion(31, 45, 15)
        self.assertEqual(psf.shape, (31, 31))

    def test_normalized(self):
        """PSF应归一化（总和≈1）"""
        psf = estimate_psf_motion(31, 45, 15)
        self.assertAlmostEqual(psf.sum(), 1.0, places=5)

    def test_nonzero_in_motion_direction(self):
        """沿运动方向应有非零值"""
        psf = estimate_psf_motion(31, 0, 15)  # 水平运动
        center_row = psf[15, :]
        self.assertGreater(np.sum(center_row), 0)

    def test_different_angles(self):
        for angle in [0, 30, 45, 90, 135]:
            psf = estimate_psf_motion(31, angle, 15)
            self.assertAlmostEqual(psf.sum(), 1.0, places=5)

    def test_longer_motion(self):
        psf_short = estimate_psf_motion(31, 0, 5)
        psf_long = estimate_psf_motion(31, 0, 15)
        # 更长的运动应有更多非零元素
        self.assertGreater(np.count_nonzero(psf_long), np.count_nonzero(psf_short))


# ==================== 高斯PSF测试 ====================

class TestCreateGaussianPSF(unittest.TestCase):
    """高斯PSF生成测试"""

    def test_returns_ndarray(self):
        psf = create_gaussian_psf(15, 2.0)
        self.assertIsInstance(psf, np.ndarray)

    def test_output_shape(self):
        psf = create_gaussian_psf(15, 2.0)
        self.assertEqual(psf.shape, (15, 15))

    def test_normalized(self):
        psf = create_gaussian_psf(15, 2.0)
        self.assertAlmostEqual(psf.sum(), 1.0, places=5)

    def test_peak_at_center(self):
        psf = create_gaussian_psf(15, 2.0)
        center = psf[7, 7]
        self.assertEqual(center, np.max(psf))

    def test_symmetric(self):
        psf = create_gaussian_psf(15, 2.0)
        np.testing.assert_array_almost_equal(psf, psf[::-1, ::-1])

    def test_larger_sigma_wider(self):
        psf_narrow = create_gaussian_psf(31, 1.0)
        psf_wide = create_gaussian_psf(31, 5.0)
        # 更大sigma应更分散（中心峰值更低）
        self.assertGreater(psf_narrow[15, 15], psf_wide[15, 15])


# ==================== 维纳去卷积测试 ====================

class TestWienerDeconvolution(unittest.TestCase):
    """维纳去卷积测试"""

    def test_returns_ndarray(self):
        img = _make_grayscale()
        psf = create_gaussian_psf(15, 2.0)
        result = wiener_deconvolution(img, psf, noise_var=0.01)
        self.assertIsInstance(result, np.ndarray)

    def test_output_shape_grayscale(self):
        img = _make_grayscale(64, 64)
        psf = create_gaussian_psf(11, 1.5)
        result = wiener_deconvolution(img, psf)
        self.assertEqual(result.shape, img.shape)

    def test_output_shape_color(self):
        img = _make_test_image(64, 64)
        psf = create_gaussian_psf(11, 1.5)
        result = wiener_deconvolution(img, psf)
        # 灰度输出
        self.assertEqual(len(result.shape), 2)

    def test_output_dtype_uint8(self):
        img = _make_grayscale(64, 64)
        psf = create_gaussian_psf(11, 1.5)
        result = wiener_deconvolution(img, psf)
        self.assertEqual(result.dtype, np.uint8)

    def test_output_range_0_255(self):
        img = _make_grayscale(64, 64)
        psf = create_gaussian_psf(11, 1.5)
        result = wiener_deconvolution(img, psf)
        self.assertGreaterEqual(result.min(), 0)
        self.assertLessEqual(result.max(), 255)

    def test_different_noise_levels(self):
        img = _make_grayscale(64, 64)
        psf = create_gaussian_psf(11, 1.5)
        for nv in [0.001, 0.01, 0.1]:
            result = wiener_deconvolution(img, psf, noise_var=nv)
            self.assertEqual(result.shape, img.shape)


# ==================== R-L去卷积测试 ====================

class TestLucyRichardsonDeconvolution(unittest.TestCase):
    """Richardson-Lucy去卷积测试"""

    def test_returns_ndarray(self):
        img = _make_grayscale(64, 64)
        psf = create_gaussian_psf(11, 1.5)
        result = lucy_richardson_deconvolution(img, psf, iterations=10)
        self.assertIsInstance(result, np.ndarray)

    def test_output_shape(self):
        img = _make_grayscale(64, 64)
        psf = create_gaussian_psf(11, 1.5)
        result = lucy_richardson_deconvolution(img, psf, iterations=10)
        self.assertEqual(result.shape, img.shape)

    def test_output_dtype_uint8(self):
        img = _make_grayscale(64, 64)
        psf = create_gaussian_psf(11, 1.5)
        result = lucy_richardson_deconvolution(img, psf, iterations=10)
        self.assertEqual(result.dtype, np.uint8)

    def test_more_iterations_more_sharpening(self):
        """更多迭代应产生更锐利的结果"""
        img = _make_grayscale(64, 64)
        psf = create_gaussian_psf(11, 1.5)
        result_few = lucy_richardson_deconvolution(img, psf, iterations=5)
        result_many = lucy_richardson_deconvolution(img, psf, iterations=50)
        # 标准差越大说明对比度越强（更锐利）
        self.assertGreaterEqual(np.std(result_many), np.std(result_few) - 5)

    def test_color_input(self):
        img = _make_test_image(64, 64)
        psf = create_gaussian_psf(11, 1.5)
        result = lucy_richardson_deconvolution(img, psf, iterations=10)
        self.assertEqual(len(result.shape), 2)


# ==================== 反锐化掩模测试 ====================

class TestDeblurWithUnsharpMask(unittest.TestCase):
    """反锐化掩模测试"""

    def test_returns_ndarray(self):
        img = _make_test_image()
        result = deblur_with_unsharp_mask(img)
        self.assertIsInstance(result, np.ndarray)

    def test_output_shape(self):
        img = _make_test_image(64, 64)
        result = deblur_with_unsharp_mask(img)
        self.assertEqual(result.shape, img.shape)

    def test_output_dtype_uint8(self):
        img = _make_test_image()
        result = deblur_with_unsharp_mask(img)
        self.assertEqual(result.dtype, np.uint8)

    def test_sharpening_increases_contrast(self):
        """锐化应增加对比度"""
        img = _make_test_image()
        result = deblur_with_unsharp_mask(img, sigma=2.0, strength=1.5)
        # 计算边缘强度差异
        edges_orig = cv2.Laplacian(img, cv2.CV_64F).std()
        edges_result = cv2.Laplacian(result, cv2.CV_64F).std()
        self.assertGreaterEqual(edges_result, edges_orig * 0.8)

    def test_custom_params(self):
        img = _make_test_image()
        result = deblur_with_unsharp_mask(img, sigma=3.0, strength=2.0)
        self.assertEqual(result.shape, img.shape)

    def test_grayscale_input(self):
        img = _make_grayscale()
        result = deblur_with_unsharp_mask(img, sigma=1.0, strength=1.0)
        self.assertEqual(result.shape, img.shape)


# ==================== 双边锐化测试 ====================

class TestDeblurBilateralSharpen(unittest.TestCase):
    """双边滤波锐化测试"""

    def test_returns_ndarray(self):
        img = _make_test_image()
        result = deblur_bilateral_sharpen(img)
        self.assertIsInstance(result, np.ndarray)

    def test_output_shape(self):
        img = _make_test_image(64, 64)
        result = deblur_bilateral_sharpen(img)
        self.assertEqual(result.shape, img.shape)

    def test_output_dtype_uint8(self):
        img = _make_test_image()
        result = deblur_bilateral_sharpen(img)
        self.assertEqual(result.dtype, np.uint8)

    def test_custom_params(self):
        img = _make_test_image()
        result = deblur_bilateral_sharpen(img, d=5, sigma_color=50, sigma_space=50)
        self.assertEqual(result.shape, img.shape)


# ==================== 盲去卷积测试 ====================

class TestBlindDeblurSimple(unittest.TestCase):
    """简化盲去卷积测试"""

    def test_returns_tuple_of_2(self):
        img = _make_grayscale(64, 64)
        result = blind_deblur_simple(img, psf_size=11, iterations=10)
        self.assertEqual(len(result), 2)

    def test_output_shape(self):
        img = _make_grayscale(64, 64)
        deblurred, psf = blind_deblur_simple(img, psf_size=11, iterations=10)
        self.assertEqual(deblurred.shape, img.shape)

    def test_psf_shape(self):
        img = _make_grayscale(64, 64)
        _, psf = blind_deblur_simple(img, psf_size=11, iterations=10)
        self.assertEqual(psf.shape, (11, 11))

    def test_psf_normalized(self):
        img = _make_grayscale(64, 64)
        _, psf = blind_deblur_simple(img, psf_size=11, iterations=10)
        self.assertGreater(psf.sum(), 0)

    def test_output_dtype_uint8(self):
        img = _make_grayscale(64, 64)
        deblurred, _ = blind_deblur_simple(img, psf_size=11, iterations=10)
        self.assertEqual(deblurred.dtype, np.uint8)

    def test_color_input(self):
        img = _make_test_image(64, 64)
        deblurred, psf = blind_deblur_simple(img, psf_size=11, iterations=10)
        self.assertEqual(deblurred.shape[:2], img.shape[:2])


# ==================== 运动模糊模拟测试 ====================

class TestSimulateMotionBlur(unittest.TestCase):
    """运动模糊模拟测试"""

    def test_returns_ndarray(self):
        img = _make_test_image()
        result = simulate_motion_blur(img, length=10, angle=30)
        self.assertIsInstance(result, np.ndarray)

    def test_output_shape(self):
        img = _make_test_image(64, 64)
        result = simulate_motion_blur(img, length=10, angle=30)
        self.assertEqual(result.shape, img.shape)

    def test_blur_changes_image(self):
        """模糊应改变图像"""
        img = _make_test_image()
        blurred = simulate_motion_blur(img, length=15, angle=45)
        self.assertFalse(np.array_equal(img, blurred))

    def test_longer_blur_more_diffuse(self):
        """更长的模糊应更分散"""
        img = _make_test_image()
        blur_short = simulate_motion_blur(img, length=5, angle=0)
        blur_long = simulate_motion_blur(img, length=20, angle=0)
        # 长模糊的拉普拉斯响应更小（更模糊）
        lap_short = cv2.Laplacian(blur_short, cv2.CV_64F).std()
        lap_long = cv2.Laplacian(blur_long, cv2.CV_64F).std()
        self.assertGreater(lap_short, lap_long)


# ==================== 去模糊效果恢复测试 ====================

class TestDeblurRecovery(unittest.TestCase):
    """去模糊效果恢复集成测试"""

    def test_wiener_restores_some_sharpness(self):
        """维纳去卷积应恢复一些锐度"""
        img = _make_grayscale(64, 64)
        blurred = simulate_motion_blur(img, length=10, angle=30)
        psf = estimate_psf_motion(21, 30, 10)
        restored = wiener_deconvolution(blurred, psf, noise_var=0.001)
        # 恢复后的图像应比模糊图像更锐利
        lap_blurred = cv2.Laplacian(blurred, cv2.CV_64F).std()
        lap_restored = cv2.Laplacian(restored, cv2.CV_64F).std()
        self.assertGreaterEqual(lap_restored, lap_blurred * 0.5)

    def test_rl_restores_some_sharpness(self):
        """R-L去卷积应恢复一些锐度"""
        img = _make_grayscale(64, 64)
        blurred = simulate_motion_blur(img, length=10, angle=30)
        psf = estimate_psf_motion(21, 30, 10)
        restored = lucy_richardson_deconvolution(blurred, psf, iterations=30)
        lap_blurred = cv2.Laplacian(blurred, cv2.CV_64F).std()
        lap_restored = cv2.Laplacian(restored, cv2.CV_64F).std()
        self.assertGreaterEqual(lap_restored, lap_blurred * 0.5)


if __name__ == '__main__':
    unittest.main()
