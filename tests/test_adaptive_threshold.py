#!/usr/bin/env python3
"""
自适应阈值单元测试
覆盖: OpenCV自适应阈值(均值/高斯)、Otsu大津法、
      Niblack二值化、Sauvola二值化、局部Otsu、
      渐变阈值、自动选择最佳方法
测试对象: 10_视觉通用代码库/adaptive_threshold.py
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

if HAS_CV2:
    from _10_视觉通用代码库.adaptive_threshold import AdaptiveThreshold


def make_uniform_gray(val=128, size=100):
    """均匀灰度图"""
    return np.full((size, size), val, dtype=np.uint8)


def make_bimodal(size=100):
    """双峰分布图(暗区+亮区)"""
    img = np.zeros((size, size), dtype=np.uint8)
    img[:, :size // 2] = 50
    img[:, size // 2:] = 200
    return img


def make_uneven_illumination(size=200):
    """光照不均匀图"""
    img = np.zeros((size, size), dtype=np.uint8)
    for i in range(size):
        for j in range(size):
            base = 100 + 80 * np.sin(i * 0.03) * np.cos(j * 0.03)
            img[i, j] = int(np.clip(base, 0, 255))
    return img


def make_text_like(size=200):
    """模拟文字图像"""
    img = np.full((size, size), 220, dtype=np.uint8)
    # 画几条暗线模拟文字
    for y in range(50, 150, 15):
        img[y:y + 5, 30:170] = 20
    return img


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestAdaptiveMean(unittest.TestCase):
    """OpenCV自适应均值阈值测试"""

    def test_output_shape(self):
        gray = make_uniform_gray()
        result = AdaptiveThreshold.cv_adaptive_mean(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_output_dtype(self):
        gray = make_uniform_gray()
        result = AdaptiveThreshold.cv_adaptive_mean(gray)
        self.assertEqual(result.dtype, np.uint8)

    def test_binary_output(self):
        """输出应为二值(0或255)"""
        gray = make_bimodal()
        result = AdaptiveThreshold.cv_adaptive_mean(gray)
        unique = np.unique(result)
        self.assertTrue(set(unique).issubset({0, 255}))

    def test_custom_block_size(self):
        gray = make_bimodal()
        r1 = AdaptiveThreshold.cv_adaptive_mean(gray, block_size=11)
        r2 = AdaptiveThreshold.cv_adaptive_mean(gray, block_size=31)
        # 不同block size应产生不同结果
        self.assertFalse(np.array_equal(r1, r2))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestAdaptiveGaussian(unittest.TestCase):
    """OpenCV自适应高斯阈值测试"""

    def test_output_shape(self):
        gray = make_uniform_gray()
        result = AdaptiveThreshold.cv_adaptive_gaussian(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_binary_output(self):
        gray = make_bimodal()
        result = AdaptiveThreshold.cv_adaptive_gaussian(gray)
        unique = np.unique(result)
        self.assertTrue(set(unique).issubset({0, 255}))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestOtsu(unittest.TestCase):
    """Otsu大津法测试"""

    def test_output_binary(self):
        gray = make_bimodal()
        result = AdaptiveThreshold.otsu(gray)
        unique = np.unique(result)
        self.assertTrue(set(unique).issubset({0, 255}))

    def test_bimodal_segments_correctly(self):
        """双峰图应正确分割"""
        gray = make_bimodal(size=100)
        result = AdaptiveThreshold.otsu(gray)
        # 左半(50)应为黑, 右半(200)应为白(或反之,取决于阈值)
        left_mean = result[:, :50].mean()
        right_mean = result[:, 50:].mean()
        # 两侧应有明显差异
        self.assertGreater(abs(left_mean - right_mean), 100)

    def test_no_blur(self):
        gray = make_bimodal()
        result = AdaptiveThreshold.otsu(gray, blur_size=0)
        self.assertEqual(result.dtype, np.uint8)

    def test_otsu_inv(self):
        gray = make_bimodal()
        result = AdaptiveThreshold.otsu_inv(gray)
        # 反相应与原结果互补
        normal = AdaptiveThreshold.otsu(gray)
        # 大部分像素应反转
        match = np.sum(result == normal) / result.size
        self.assertLess(match, 0.2)  # 大部分应不同


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestNiblack(unittest.TestCase):
    """Niblack二值化测试"""

    def test_output_shape(self):
        gray = make_text_like()
        result = AdaptiveThreshold.niblack(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_output_binary(self):
        gray = make_text_like()
        result = AdaptiveThreshold.niblack(gray)
        unique = np.unique(result)
        self.assertTrue(set(unique).issubset({0, 255}))

    def test_detects_text(self):
        """应能检测到文字线条"""
        gray = make_text_like()
        result = AdaptiveThreshold.niblack(gray, window_size=15, k=-0.2)
        # 文字区域(50:55, 30:170)应被检测为黑色(0)
        text_region = result[50:55, 30:170]
        self.assertLess(text_region.mean(), 128)

    def test_different_k_values(self):
        gray = make_text_like()
        r1 = AdaptiveThreshold.niblack(gray, k=-0.1)
        r2 = AdaptiveThreshold.niblack(gray, k=-0.5)
        self.assertFalse(np.array_equal(r1, r2))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestSauvola(unittest.TestCase):
    """Sauvola二值化测试"""

    def test_output_shape(self):
        gray = make_uneven_illumination()
        result = AdaptiveThreshold.sauvola(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_handles_uneven_illumination(self):
        """应能处理光照不均匀"""
        gray = make_uneven_illumination()
        result = AdaptiveThreshold.sauvola(gray)
        # 应产生合理的二值结果(非全黑或全白)
        white_ratio = np.sum(result == 255) / result.size
        self.assertGreater(white_ratio, 0.1)
        self.assertLess(white_ratio, 0.9)

    def test_output_binary(self):
        gray = make_text_like()
        result = AdaptiveThreshold.sauvola(gray)
        unique = np.unique(result)
        self.assertTrue(set(unique).issubset({0, 255}))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLocalOtsu(unittest.TestCase):
    """局部Otsu测试"""

    def test_output_shape(self):
        gray = make_uneven_illumination(size=200)
        result = AdaptiveThreshold.local_otsu(gray, block_size=101)
        self.assertEqual(result.shape, gray.shape)

    def test_output_binary(self):
        gray = make_bimodal(size=200)
        result = AdaptiveThreshold.local_otsu(gray, block_size=51)
        unique = np.unique(result)
        self.assertTrue(set(unique).issubset({0, 255}))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGradientThreshold(unittest.TestCase):
    """渐变阈值测试"""

    def test_output_shape(self):
        gray = make_uneven_illumination()
        result = AdaptiveThreshold.gradient_threshold(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_output_binary(self):
        gray = make_uneven_illumination()
        result = AdaptiveThreshold.gradient_threshold(gray)
        unique = np.unique(result)
        self.assertTrue(set(unique).issubset({0, 255}))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestAutoBest(unittest.TestCase):
    """自动选择最佳方法测试"""

    def test_returns_binary(self):
        gray = make_bimodal()
        result = AdaptiveThreshold.auto_best(gray)
        unique = np.unique(result)
        self.assertTrue(set(unique).issubset({0, 255}))

    def test_handles_uneven(self):
        gray = make_uneven_illumination()
        result = AdaptiveThreshold.auto_best(gray)
        self.assertEqual(result.shape, gray.shape)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestCompareAll(unittest.TestCase):
    """全方法对比测试"""

    def test_returns_dict(self):
        gray = make_bimodal(size=100)
        results = AdaptiveThreshold.compare_all(gray)
        self.assertIsInstance(results, dict)
        expected_keys = ['AdaptiveMean', 'AdaptiveGauss', 'Otsu',
                         'Niblack', 'Sauvola', 'Gradient', 'AutoBest']
        for key in expected_keys:
            self.assertIn(key, results)

    def test_all_same_shape(self):
        gray = make_bimodal(size=100)
        results = AdaptiveThreshold.compare_all(gray)
        for name, result in results.items():
            self.assertEqual(result.shape, gray.shape, f"{name} shape mismatch")


if __name__ == '__main__':
    unittest.main()
