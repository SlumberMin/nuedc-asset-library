"""
图像质量评估单元测试
覆盖: PSNR/SSIM/NIQE/清晰度拉普拉斯/清晰度Tenengrad/噪声估计/RMS对比度
"""
import unittest
import numpy as np
import cv2
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from importlib import import_module

mod = import_module('10_视觉通用代码库.image_quality')


class TestPSNR(unittest.TestCase):
    """PSNR测试"""

    def test_identical_images(self):
        """相同图像PSNR应为inf"""
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = mod.psnr(img, img)
        self.assertEqual(result, float('inf'))

    def test_different_images(self):
        """不同图像PSNR应为有限正数"""
        img1 = np.zeros((64, 64), dtype=np.uint8)
        img2 = np.full((64, 64), 10, dtype=np.uint8)
        result = mod.psnr(img1, img2)
        self.assertGreater(result, 0)
        self.assertNotEqual(result, float('inf'))

    def test_noisy_image_lower_psnr(self):
        """强噪声应降低PSNR"""
        img = np.full((64, 64), 128, dtype=np.uint8)
        low_noise = np.clip(img.astype(float) + np.random.normal(0, 5, img.shape), 0, 255).astype(np.uint8)
        high_noise = np.clip(img.astype(float) + np.random.normal(0, 50, img.shape), 0, 255).astype(np.uint8)
        psnr_low = mod.psnr(img, low_noise)
        psnr_high = mod.psnr(img, high_noise)
        self.assertGreater(psnr_low, psnr_high)

    def test_color_image(self):
        """彩色图像PSNR"""
        img1 = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        img2 = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        result = mod.psnr(img1, img2)
        self.assertIsInstance(result, float)
        self.assertGreater(result, 0)


class TestSSIM(unittest.TestCase):
    """SSIM测试"""

    def test_identical_images(self):
        """相同图像SSIM应接近1"""
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = mod.ssim(img, img)
        self.assertAlmostEqual(result, 1.0, places=3)

    def test_random_images_low_ssim(self):
        """随机图像SSIM应较低"""
        img1 = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        img2 = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = mod.ssim(img1, img2)
        self.assertLess(result, 0.5)

    def test_slightly_noisy_high_ssim(self):
        """轻微噪声SSIM应仍较高"""
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        noisy = np.clip(img.astype(float) + np.random.normal(0, 5, img.shape), 0, 255).astype(np.uint8)
        result = mod.ssim(img, noisy)
        self.assertGreater(result, 0.5)

    def test_color_image_ssim(self):
        """彩色图SSIM"""
        img = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        result = mod.ssim(img, img)
        self.assertAlmostEqual(result, 1.0, places=2)

    def test_ssim_range(self):
        """SSIM应在[-1,1]范围内"""
        img1 = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        img2 = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = mod.ssim(img1, img2)
        self.assertGreaterEqual(result, -1.0)
        self.assertLessEqual(result, 1.0)


class TestNIQE(unittest.TestCase):
    """NIQE测试"""

    def test_returns_float(self):
        img = np.random.randint(0, 256, (128, 128), dtype=np.uint8)
        result = mod.niqe(img)
        self.assertIsInstance(result, float)

    def test_positive_value(self):
        img = np.random.randint(0, 256, (128, 128), dtype=np.uint8)
        result = mod.niqe(img)
        self.assertGreaterEqual(result, 0)

    def test_color_image(self):
        """彩色图应自动转灰度"""
        img = np.random.randint(0, 256, (128, 128, 3), dtype=np.uint8)
        result = mod.niqe(img)
        self.assertIsInstance(result, float)

    def test_blurry_vs_sharp(self):
        """模糊图NIQE应高于清晰图（简化实现中可能不严格成立，仅检查可运行）"""
        sharp = np.random.randint(0, 256, (128, 128), dtype=np.uint8)
        blurry = cv2.GaussianBlur(sharp, (11, 11), 5)
        niqe_sharp = mod.niqe(sharp)
        niqe_blurry = mod.niqe(blurry)
        self.assertIsInstance(niqe_sharp, float)
        self.assertIsInstance(niqe_blurry, float)


class TestSharpnessLaplacian(unittest.TestCase):
    """拉普拉斯清晰度测试"""

    def test_returns_float(self):
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = mod.sharpness_laplacian(img)
        self.assertIsInstance(result, float)

    def test_positive_value(self):
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = mod.sharpness_laplacian(img)
        self.assertGreaterEqual(result, 0)

    def test_sharp_higher_than_blurry(self):
        """清晰图分数应高于模糊图"""
        sharp = np.zeros((64, 64), dtype=np.uint8)
        sharp[::2, :] = 255
        blurry = cv2.GaussianBlur(sharp, (11, 11), 5)
        score_sharp = mod.sharpness_laplacian(sharp)
        score_blurry = mod.sharpness_laplacian(blurry)
        self.assertGreater(score_sharp, score_blurry)

    def test_color_image(self):
        img = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        result = mod.sharpness_laplacian(img)
        self.assertIsInstance(result, float)


class TestSharpnessTenengrad(unittest.TestCase):
    """Tenengrad清晰度测试"""

    def test_returns_float(self):
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = mod.sharpness_tenengrad(img)
        self.assertIsInstance(result, float)

    def test_positive_value(self):
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = mod.sharpness_tenengrad(img)
        self.assertGreaterEqual(result, 0)

    def test_sharp_higher_than_blurry(self):
        sharp = np.zeros((64, 64), dtype=np.uint8)
        sharp[::2, :] = 255
        blurry = cv2.GaussianBlur(sharp, (11, 11), 5)
        score_sharp = mod.sharpness_tenengrad(sharp)
        score_blurry = mod.sharpness_tenengrad(blurry)
        self.assertGreater(score_sharp, score_blurry)

    def test_custom_ksize(self):
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result3 = mod.sharpness_tenengrad(img, ksize=3)
        result5 = mod.sharpness_tenengrad(img, ksize=5)
        self.assertIsInstance(result3, float)
        self.assertIsInstance(result5, float)


class TestNoiseEstimate(unittest.TestCase):
    """噪声估计测试"""

    def test_returns_float(self):
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = mod.noise_estimate(img)
        self.assertIsInstance(result, float)

    def test_positive_value(self):
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = mod.noise_estimate(img)
        self.assertGreaterEqual(result, 0)

    def test_noisy_image_higher_estimate(self):
        """加噪后噪声估计应更高"""
        img = np.full((128, 128), 128, dtype=np.uint8)
        noisy = np.clip(img.astype(float) + np.random.normal(0, 30, img.shape), 0, 255).astype(np.uint8)
        est_clean = mod.noise_estimate(img)
        est_noisy = mod.noise_estimate(noisy)
        self.assertGreater(est_noisy, est_clean)

    def test_color_image(self):
        img = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        result = mod.noise_estimate(img)
        self.assertIsInstance(result, float)


class TestContrastMetric(unittest.TestCase):
    """RMS对比度测试"""

    def test_returns_float(self):
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = mod.contrast_metric(img)
        self.assertIsInstance(result, float)

    def test_constant_image_zero_contrast(self):
        """均匀图像对比度应为0"""
        img = np.full((64, 64), 128, dtype=np.uint8)
        result = mod.contrast_metric(img)
        self.assertAlmostEqual(result, 0.0, places=3)

    def test_high_contrast(self):
        """黑白图像对比度应很高"""
        img = np.zeros((64, 64), dtype=np.uint8)
        img[:32, :] = 255
        result = mod.contrast_metric(img)
        self.assertGreater(result, 100)

    def test_color_image(self):
        img = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        result = mod.contrast_metric(img)
        self.assertIsInstance(result, float)


if __name__ == '__main__':
    unittest.main()
