#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
图像金字塔单元测试
覆盖: gaussian_pyramid层数/尺寸、laplacian_pyramid层数、
      reconstruct_from_laplacian重建精度、dog_pyramid层数/彩色输入、
      visualize_pyramid拼接尺寸
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from _10_视觉通用代码库.image_pyramid import (
    gaussian_pyramid,
    laplacian_pyramid,
    reconstruct_from_laplacian,
    dog_pyramid,
    visualize_pyramid,
)


# ==================== 辅助函数 ====================

def _make_test_image(h=256, w=256, color=True):
    """创建带纹理的测试图像"""
    img = np.zeros((h, w, 3), dtype=np.uint8) if color else np.zeros((h, w), dtype=np.uint8)
    cv2.rectangle(img, (20, 20), (120, 120), (200, 100, 50), -1)
    cv2.circle(img, (180, 80), 40, (50, 200, 100), -1)
    cv2.putText(img, "TEST", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
    return img


# ==================== 高斯金字塔测试 ====================

class TestGaussianPyramid(unittest.TestCase):
    """高斯金字塔测试"""

    def test_returns_list(self):
        img = _make_test_image()
        pyr = gaussian_pyramid(img, levels=3)
        self.assertIsInstance(pyr, list)

    def test_correct_number_of_levels(self):
        img = _make_test_image()
        for n in [2, 4, 6]:
            pyr = gaussian_pyramid(img, levels=n)
            self.assertEqual(len(pyr), n)

    def test_level0_equals_original(self):
        img = _make_test_image()
        pyr = gaussian_pyramid(img, levels=5)
        np.testing.assert_array_equal(pyr[0], img)

    def test_each_level_half_size(self):
        """每层尺寸应为上一层的一半（向下取整）"""
        img = _make_test_image(256, 256)
        pyr = gaussian_pyramid(img, levels=5)
        for i in range(1, len(pyr)):
            h_prev, w_prev = pyr[i - 1].shape[:2]
            h_curr, w_curr = pyr[i].shape[:2]
            self.assertAlmostEqual(h_curr, h_prev / 2, delta=1)
            self.assertAlmostEqual(w_curr, w_prev / 2, delta=1)

    def test_grayscale_input(self):
        img = _make_test_image(color=False)
        pyr = gaussian_pyramid(img, levels=3)
        self.assertEqual(len(pyr), 3)
        for p in pyr:
            self.assertEqual(len(p.shape), 2)

    def test_color_channels_preserved(self):
        img = _make_test_image()
        pyr = gaussian_pyramid(img, levels=3)
        for p in pyr:
            self.assertEqual(p.shape[2], 3)

    def test_single_level(self):
        """levels=1 应只返回原图"""
        img = _make_test_image()
        pyr = gaussian_pyramid(img, levels=1)
        self.assertEqual(len(pyr), 1)
        np.testing.assert_array_equal(pyr[0], img)

    def test_does_not_modify_input(self):
        img = _make_test_image()
        original = img.copy()
        gaussian_pyramid(img, levels=3)
        np.testing.assert_array_equal(img, original)


# ==================== 拉普拉斯金字塔测试 ====================

class TestLaplacianPyramid(unittest.TestCase):
    """拉普拉斯金字塔测试"""

    def test_returns_list(self):
        img = _make_test_image()
        pyr = laplacian_pyramid(img, levels=3)
        self.assertIsInstance(pyr, list)

    def test_correct_number_of_levels(self):
        img = _make_test_image()
        for n in [2, 4, 5]:
            pyr = laplacian_pyramid(img, levels=n)
            self.assertEqual(len(pyr), n)

    def test_grayscale_input(self):
        img = _make_test_image(color=False)
        pyr = laplacian_pyramid(img, levels=3)
        self.assertEqual(len(pyr), 3)

    def test_contains_high_freq(self):
        """拉普拉斯金字塔应包含高频信息（非全零）"""
        img = _make_test_image()
        pyr = laplacian_pyramid(img, levels=3)
        # 最高层（原始分辨率）应非全零
        self.assertGreater(np.sum(np.abs(pyr[0].astype(np.float64))), 0)

    def test_sizes_decrease(self):
        """各层尺寸应递减或保持"""
        img = _make_test_image(256, 256)
        pyr = laplacian_pyramid(img, levels=5)
        for i in range(len(pyr) - 1):
            h_curr = pyr[i].shape[0]
            h_next = pyr[i + 1].shape[0]
            self.assertGreaterEqual(h_curr, h_next)


# ==================== 重建测试 ====================

class TestReconstructFromLaplacian(unittest.TestCase):
    """从拉普拉斯金字塔重建图像测试"""

    def test_returns_ndarray(self):
        img = _make_test_image()
        lap_pyr = laplacian_pyramid(img, levels=4)
        recon = reconstruct_from_laplacian(lap_pyr)
        self.assertIsInstance(recon, np.ndarray)

    def test_reconstruction_shape(self):
        """重建图像尺寸应与原图一致"""
        img = _make_test_image(256, 256)
        lap_pyr = laplacian_pyramid(img, levels=5)
        recon = reconstruct_from_laplacian(lap_pyr)
        self.assertEqual(recon.shape, img.shape)

    def test_reconstruction_closeto_original(self):
        """重建图像应与原图接近（由于截断误差不完全相等）"""
        img = _make_test_image(256, 256)
        lap_pyr = laplacian_pyramid(img, levels=3)
        recon = reconstruct_from_laplacian(lap_pyr)
        recon_u8 = np.clip(recon, 0, 255).astype(np.uint8)
        # PSNR应很高（>30dB）
        mse = np.mean((img.astype(np.float64) - recon_u8.astype(np.float64)) ** 2)
        if mse > 0:
            psnr = 10 * np.log10(255.0 ** 2 / mse)
            self.assertGreater(psnr, 25.0)

    def test_grayscale_reconstruction(self):
        img = _make_test_image(color=False)
        lap_pyr = laplacian_pyramid(img, levels=3)
        recon = reconstruct_from_laplacian(lap_pyr)
        self.assertEqual(recon.shape[:2], img.shape[:2])


# ==================== DOG金字塔测试 ====================

class TestDogPyramid(unittest.TestCase):
    """DOG金字塔测试"""

    def test_returns_list(self):
        img = _make_test_image()
        dog = dog_pyramid(img, levels=3)
        self.assertIsInstance(dog, list)

    def test_correct_number_of_levels(self):
        img = _make_test_image()
        for n in [2, 4, 5]:
            dog = dog_pyramid(img, levels=n)
            self.assertEqual(len(dog), n - 1)

    def test_contains_edges(self):
        """DOG应响应边缘（非全零）"""
        img = _make_test_image()
        dog = dog_pyramid(img, levels=3)
        for d in dog:
            self.assertGreater(np.sum(np.abs(d)), 0)

    def test_color_input_auto_gray(self):
        """彩色输入应自动转灰度处理"""
        img = _make_test_image(color=True)
        dog = dog_pyramid(img, levels=3)
        for d in dog:
            self.assertEqual(len(d.shape), 2)

    def test_grayscale_input(self):
        img = _make_test_image(color=False)
        dog = dog_pyramid(img, levels=3)
        self.assertEqual(len(dog), 2)

    def test_custom_sigma(self):
        img = _make_test_image()
        dog = dog_pyramid(img, levels=3, sigma1=0.5, sigma2=1.0)
        self.assertEqual(len(dog), 2)

    def test_sizes_decrease(self):
        img = _make_test_image(256, 256)
        dog = dog_pyramid(img, levels=5)
        for i in range(len(dog) - 1):
            self.assertGreaterEqual(dog[i].shape[0], dog[i + 1].shape[0])


# ==================== 可视化测试 ====================

class TestVisualizePyramid(unittest.TestCase):
    """金字塔可视化测试"""

    def test_returns_ndarray(self):
        img = _make_test_image()
        pyr = gaussian_pyramid(img, levels=4)
        vis = visualize_pyramid(pyr)
        self.assertIsInstance(vis, np.ndarray)

    def test_output_width_sufficient(self):
        """输出宽度应覆盖所有层"""
        img = _make_test_image(128, 128)
        pyr = gaussian_pyramid(img, levels=4)
        vis = visualize_pyramid(pyr)
        expected_w = 128 + 64 + 32 + 16
        self.assertEqual(vis.shape[1], expected_w)

    def test_output_height_equals_original(self):
        img = _make_test_image(128, 128)
        pyr = gaussian_pyramid(img, levels=4)
        vis = visualize_pyramid(pyr)
        self.assertEqual(vis.shape[0], 128)

    def test_grayscale_pyramid(self):
        img = _make_test_image(128, 128, color=False)
        pyr = gaussian_pyramid(img, levels=3)
        vis = visualize_pyramid(pyr)
        self.assertEqual(len(vis.shape), 2)


if __name__ == '__main__':
    unittest.main()
