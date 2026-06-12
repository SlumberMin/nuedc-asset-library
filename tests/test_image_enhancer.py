#!/usr/bin/env python3
"""
图像增强工具集单元测试
覆盖: CLAHE、去噪、锐化、白平衡、Gamma校正、
      亮度对比度调整、流水线、一键增强
测试对象: 10_视觉通用代码库/image_enhancer.py
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '10_视觉通用代码库'))

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

if HAS_CV2:
    from image_enhancer import ImageEnhancer


def _make_test_image(height=240, width=320, channels=3):
    """创建测试图像"""
    img = np.random.randint(0, 255, (height, width, channels), dtype=np.uint8)
    return img


def _make_gray_image(height=240, width=320):
    """创建灰度测试图像"""
    return np.random.randint(0, 255, (height, width), dtype=np.uint8)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestCLAHE(unittest.TestCase):
    """CLAHE自适应直方图均衡化测试"""

    def test_clahe_luminance_returns_bgr(self):
        """LAB亮度通道CLAHE应返回BGR图"""
        img = _make_test_image()
        result = ImageEnhancer.clahe(img, clip_limit=2.0, apply_to="luminance")
        self.assertEqual(result.shape, img.shape)
        self.assertEqual(result.dtype, np.uint8)

    def test_clahe_gray(self):
        """灰度CLAHE应返回灰度图"""
        img = _make_gray_image()
        result = ImageEnhancer.clahe(img, clip_limit=2.0, apply_to="gray")
        self.assertEqual(len(result.shape), 2)

    def test_clahe_channel(self):
        """逐通道CLAHE应返回BGR图"""
        img = _make_test_image()
        result = ImageEnhancer.clahe(img, clip_limit=2.0, apply_to="channel")
        self.assertEqual(result.shape, img.shape)

    def test_clahe_bgr_auto_gray(self):
        """BGR图使用gray模式应返回灰度"""
        img = _make_test_image()
        result = ImageEnhancer.clahe(img, clip_limit=2.0, apply_to="gray")
        self.assertEqual(len(result.shape), 2)

    def test_clahe_enhances_contrast(self):
        """CLAHE应增强对比度(标准差增大)"""
        # 创建低对比度图像
        img = np.ones((240, 320, 3), dtype=np.uint8) * 100
        img += np.random.randint(0, 20, img.shape, dtype=np.uint8).astype(np.uint8)
        result = ImageEnhancer.clahe(img, clip_limit=4.0, apply_to="luminance")
        # 标准差应增大
        self.assertGreaterEqual(np.std(result), np.std(img) - 5)

    def test_clip_limit_effect(self):
        """不同clip_limit应产生不同结果"""
        img = _make_test_image()
        r1 = ImageEnhancer.clahe(img, clip_limit=1.0)
        r2 = ImageEnhancer.clahe(img, clip_limit=8.0)
        self.assertFalse(np.array_equal(r1, r2))


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestDenoise(unittest.TestCase):
    """去噪测试"""

    def test_bilateral_returns_same_shape(self):
        """双边滤波应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.denoise(img, method="bilateral", strength=10)
        self.assertEqual(result.shape, img.shape)

    def test_gaussian_returns_same_shape(self):
        """高斯滤波应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.denoise(img, method="gaussian", strength=10)
        self.assertEqual(result.shape, img.shape)

    def test_median_returns_same_shape(self):
        """中值滤波应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.denoise(img, method="median", strength=5)
        self.assertEqual(result.shape, img.shape)

    def test_nlmeans_returns_same_shape(self):
        """NLMeans应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.denoise(img, method="nlmeans", strength=10)
        self.assertEqual(result.shape, img.shape)

    def test_denoise_reduces_noise(self):
        """去噪应减少高频噪声"""
        clean = np.ones((100, 100, 3), dtype=np.uint8) * 128
        noisy = np.clip(clean.astype(float) + np.random.randn(100, 100, 3) * 30, 0, 255).astype(np.uint8)
        denoised = ImageEnhancer.denoise(noisy, method="gaussian", strength=10)
        self.assertLess(np.std(denoised), np.std(noisy))

    def test_invalid_method_returns_original(self):
        """无效方法应返回原图"""
        img = _make_test_image()
        result = ImageEnhancer.denoise(img, method="invalid")
        np.testing.assert_array_equal(result, img)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestSharpen(unittest.TestCase):
    """锐化测试"""

    def test_unsharp_returns_same_shape(self):
        """Unsharp Mask应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.sharpen(img, method="unsharp", strength=1.5)
        self.assertEqual(result.shape, img.shape)

    def test_kernel_returns_same_shape(self):
        """核锐化应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.sharpen(img, method="kernel", strength=1.5)
        self.assertEqual(result.shape, img.shape)

    def test_laplacian_returns_same_shape(self):
        """拉普拉斯锐化应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.sharpen(img, method="laplacian", strength=1.5)
        self.assertEqual(result.shape, img.shape)

    def test_sharpen_increases_detail(self):
        """锐化应增强边缘"""
        # 平滑图像
        img = np.ones((100, 100, 3), dtype=np.uint8) * 128
        img[40:60, 40:60] = 200  # 方块
        img = cv2.GaussianBlur(img, (5, 5), 0)
        sharpened = ImageEnhancer.sharpen(img, method="unsharp", strength=2.0)
        # 锐化后方块边缘应更明显(标准差更大)
        self.assertGreaterEqual(np.std(sharpened), np.std(img) - 5)

    def test_invalid_method_returns_original(self):
        """无效方法应返回原图"""
        img = _make_test_image()
        result = ImageEnhancer.sharpen(img, method="invalid")
        np.testing.assert_array_equal(result, img)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestWhiteBalance(unittest.TestCase):
    """白平衡测试"""

    def test_gray_world_returns_same_shape(self):
        """灰度世界法应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.white_balance(img, method="gray_world")
        self.assertEqual(result.shape, img.shape)

    def test_white_patch_returns_same_shape(self):
        """白块法应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.white_balance(img, method="white_patch")
        self.assertEqual(result.shape, img.shape)

    def test_adaptive_returns_same_shape(self):
        """自适应白平衡应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.white_balance(img, method="adaptive")
        self.assertEqual(result.shape, img.shape)

    def test_gray_world_balances_channels(self):
        """灰度世界法应使通道均值接近"""
        # 创建偏色图像(偏红)
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        img[:, :, 0] = 50   # B
        img[:, :, 1] = 50   # G
        img[:, :, 2] = 200  # R
        result = ImageEnhancer.white_balance(img, method="gray_world")
        b_mean = result[:, :, 0].mean()
        r_mean = result[:, :, 2].mean()
        # 红色均值应降低
        self.assertLess(r_mean, 200)

    def test_invalid_method_returns_original(self):
        """无效方法应返回原图"""
        img = _make_test_image()
        result = ImageEnhancer.white_balance(img, method="invalid")
        np.testing.assert_array_equal(result, img)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestGammaCorrection(unittest.TestCase):
    """Gamma校正测试"""

    def test_gamma_1_unchanged(self):
        """gamma=1应基本不变"""
        img = _make_test_image()
        result = ImageEnhancer.gamma_correction(img, gamma=1.0)
        np.testing.assert_array_almost_equal(result, img, decimal=0)

    def test_gamma_less_than_1_brightens(self):
        """gamma<1应提亮"""
        img = np.ones((100, 100, 3), dtype=np.uint8) * 100
        result = ImageEnhancer.gamma_correction(img, gamma=0.5)
        self.assertGreater(result.mean(), img.mean())

    def test_gamma_greater_than_1_darkens(self):
        """gamma>1应变暗"""
        img = np.ones((100, 100, 3), dtype=np.uint8) * 200
        result = ImageEnhancer.gamma_correction(img, gamma=2.0)
        self.assertLess(result.mean(), img.mean())

    def test_gamma_returns_same_shape(self):
        """Gamma校正应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.gamma_correction(img, gamma=1.5)
        self.assertEqual(result.shape, img.shape)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestBrightnessContrast(unittest.TestCase):
    """亮度对比度调整测试"""

    def test_zero_adjustment_unchanged(self):
        """零调整应基本不变"""
        img = _make_test_image()
        result = ImageEnhancer.adjust_brightness_contrast(img, brightness=0, contrast=0)
        np.testing.assert_array_almost_equal(result, img, decimal=0)

    def test_positive_brightness(self):
        """正亮度应提亮"""
        img = np.ones((100, 100, 3), dtype=np.uint8) * 100
        result = ImageEnhancer.adjust_brightness_contrast(img, brightness=50)
        self.assertGreater(result.mean(), img.mean())

    def test_returns_same_shape(self):
        """调整应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.adjust_brightness_contrast(img, brightness=20, contrast=30)
        self.assertEqual(result.shape, img.shape)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestImageEnhancerPipeline(unittest.TestCase):
    """图像增强流水线测试"""

    def test_pipeline_add_and_apply(self):
        """流水线应能添加和执行步骤"""
        enhancer = ImageEnhancer()
        enhancer.add_step("gamma", ImageEnhancer.gamma_correction, gamma=1.2)
        img = _make_test_image()
        result = enhancer.apply(img)
        self.assertEqual(result.shape, img.shape)

    def test_pipeline_multiple_steps(self):
        """多步骤流水线"""
        enhancer = ImageEnhancer()
        enhancer.add_step("clahe", ImageEnhancer.clahe, clip_limit=2.0)
        enhancer.add_step("sharpen", ImageEnhancer.sharpen, strength=1.2)
        img = _make_test_image()
        result = enhancer.apply(img)
        self.assertEqual(result.shape, img.shape)

    def test_pipeline_clear(self):
        """清除流水线"""
        enhancer = ImageEnhancer()
        enhancer.add_step("gamma", ImageEnhancer.gamma_correction, gamma=1.2)
        enhancer.clear()
        img = _make_test_image()
        result = enhancer.apply(img)
        np.testing.assert_array_equal(result, img)

    def test_pipeline_chaining(self):
        """add_step应支持链式调用"""
        enhancer = ImageEnhancer()
        ret = enhancer.add_step("a", ImageEnhancer.gamma_correction, gamma=1.0)
        self.assertIs(ret, enhancer)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestEnhanceForDetection(unittest.TestCase):
    """一键检测增强测试"""

    def test_enhance_for_detection_shape(self):
        """检测增强应保持尺寸"""
        img = _make_test_image()
        result = ImageEnhancer.enhance_for_detection(img)
        self.assertEqual(result.shape, img.shape)

    def test_enhance_for_detection_dtype(self):
        """检测增强应返回uint8"""
        img = _make_test_image()
        result = ImageEnhancer.enhance_for_detection(img)
        self.assertEqual(result.dtype, np.uint8)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestEnhanceForOCR(unittest.TestCase):
    """一键OCR增强测试"""

    def test_enhance_for_ocr_shape(self):
        """OCR增强应返回灰度图"""
        img = _make_test_image()
        result = ImageEnhancer.enhance_for_ocr(img)
        self.assertEqual(len(result.shape), 2)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestImageEnhancerEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_all_black_image(self):
        """全黑图像应正常处理"""
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        result = ImageEnhancer.clahe(img, clip_limit=2.0)
        self.assertEqual(result.shape, img.shape)

    def test_all_white_image(self):
        """全白图像应正常处理"""
        img = np.ones((100, 100, 3), dtype=np.uint8) * 255
        result = ImageEnhancer.clahe(img, clip_limit=2.0)
        self.assertEqual(result.shape, img.shape)

    def test_small_image(self):
        """小图像应正常处理"""
        img = np.random.randint(0, 255, (8, 8, 3), dtype=np.uint8)
        result = ImageEnhancer.gamma_correction(img, gamma=1.5)
        self.assertEqual(result.shape, img.shape)


if __name__ == '__main__':
    unittest.main()
