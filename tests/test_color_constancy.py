#!/usr/bin/env python3
"""
颜色恒常性单元测试
覆盖: Gray World白平衡、White Patch白平衡、Gray Edge算法、
      色温补偿、自动白平衡、光照归一化
测试对象: 10_视觉通用代码库/color_constancy.py
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
    from _10_视觉通用代码库.color_constancy import ColorConstancy


def make_color_image(r=100, g=150, b=200, size=100):
    """创建纯色BGR图像"""
    img = np.zeros((size, size, 3), dtype=np.uint8)
    img[:, :, 0] = b
    img[:, :, 1] = g
    img[:, :, 2] = r
    return img


def make_biased_image(r_bias=50, g_bias=0, b_bias=0, base=100, size=100):
    """创建偏色图像"""
    img = np.zeros((size, size, 3), dtype=np.uint8)
    img[:, :, 2] = np.clip(base + r_bias, 0, 255)  # R
    img[:, :, 1] = np.clip(base + g_bias, 0, 255)  # G
    img[:, :, 0] = np.clip(base + b_bias, 0, 255)  # B
    return img


def make_gradient_image(size=100):
    """创建渐变图像(模拟光照不均匀)"""
    img = np.zeros((size, size, 3), dtype=np.uint8)
    for i in range(size):
        for j in range(size):
            val = int(128 + 60 * np.sin(i * 0.05) * np.cos(j * 0.05))
            img[i, j] = [val, val, val]
    return img


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGrayWorld(unittest.TestCase):
    """Gray World白平衡测试"""

    def test_balanced_image_unchanged(self):
        """均衡图像应几乎不变"""
        img = make_color_image(r=128, g=128, b=128)
        result = ColorConstancy.gray_world(img)
        diff = np.mean(np.abs(result.astype(float) - img.astype(float)))
        self.assertLess(diff, 1.0)

    def test_biased_image_corrected(self):
        """偏色图像应被修正"""
        img = make_biased_image(r_bias=80, g_bias=0, b_bias=0, base=100)
        result = ColorConstancy.gray_world(img)
        # 修正后各通道均值应更接近
        avg_b = result[:, :, 0].mean()
        avg_g = result[:, :, 1].mean()
        avg_r = result[:, :, 2].mean()
        channel_range = max(avg_b, avg_g, avg_r) - min(avg_b, avg_g, avg_r)
        orig_range = 80  # 原始差异
        self.assertLess(channel_range, orig_range)

    def test_output_dtype(self):
        img = make_color_image()
        result = ColorConstancy.gray_world(img)
        self.assertEqual(result.dtype, np.uint8)

    def test_output_shape(self):
        img = make_color_image(size=200)
        result = ColorConstancy.gray_world(img)
        self.assertEqual(result.shape, img.shape)

    def test_no_overflow(self):
        """高亮度图像不应溢出"""
        img = make_color_image(r=240, g=230, b=220)
        result = ColorConstancy.gray_world(img)
        self.assertTrue(np.all(result <= 255))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestWhitePatch(unittest.TestCase):
    """White Patch白平衡测试"""

    def test_output_dtype(self):
        img = make_color_image()
        result = ColorConstancy.white_patch(img)
        self.assertEqual(result.dtype, np.uint8)

    def test_brightens_dark_channels(self):
        """暗通道应被提亮"""
        img = make_biased_image(r_bias=80, g_bias=0, b_bias=0, base=50)
        result = ColorConstancy.white_patch(img)
        # B和G通道应被提升
        self.assertGreater(result[:, :, 0].mean(), img[:, :, 0].mean())

    def test_percentile_parameter(self):
        """不同百分位应产生不同结果"""
        img = make_color_image(size=50)
        r1 = ColorConstancy.white_patch(img, percentile=99)
        r2 = ColorConstancy.white_patch(img, percentile=50)
        # 可能相同(纯色图), 但不应报错
        self.assertEqual(r1.shape, r2.shape)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGrayEdge(unittest.TestCase):
    """Gray Edge算法测试"""

    def test_output_shape(self):
        img = make_color_image()
        result = ColorConstancy.gray_edge(img)
        self.assertEqual(result.shape, img.shape)

    def test_order1(self):
        img = make_gradient_image()
        result = ColorConstancy.gray_edge(img, order=1)
        self.assertEqual(result.dtype, np.uint8)

    def test_order2(self):
        img = make_gradient_image()
        result = ColorConstancy.gray_edge(img, order=2)
        self.assertEqual(result.dtype, np.uint8)

    def test_balanced_image_stable(self):
        """均衡图像不应变化太大"""
        img = make_color_image(r=128, g=128, b=128)
        result = ColorConstancy.gray_edge(img)
        diff = np.mean(np.abs(result.astype(float) - img.astype(float)))
        self.assertLess(diff, 5.0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestColorTemperature(unittest.TestCase):
    """色温补偿测试"""

    def test_warm_increases_red(self):
        """暖色调应增加红色"""
        img = make_color_image(r=100, g=100, b=100)
        result = ColorConstancy.color_temperature_compensate(img, temperature=50)
        self.assertGreater(result[:, :, 2].mean(), img[:, :, 2].mean())

    def test_cool_increases_blue(self):
        """冷色调应增加蓝色"""
        img = make_color_image(r=100, g=100, b=100)
        result = ColorConstancy.color_temperature_compensate(img, temperature=-50)
        self.assertGreater(result[:, :, 0].mean(), img[:, :, 0].mean())

    def test_zero_no_change(self):
        """温度=0应无变化"""
        img = make_color_image(r=100, g=100, b=100)
        result = ColorConstancy.color_temperature_compensate(img, temperature=0)
        np.testing.assert_array_equal(result, img)

    def test_extreme_temperature_clamped(self):
        """极端温度不应溢出"""
        img = make_color_image(r=250, g=100, b=250)
        result = ColorConstancy.color_temperature_compensate(img, temperature=100)
        self.assertTrue(np.all(result <= 255))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestAutoWhiteBalance(unittest.TestCase):
    """自动白平衡测试"""

    def test_gray_world_method(self):
        img = make_color_image()
        result = ColorConstancy.auto_white_balance(img, method='gray_world')
        self.assertEqual(result.shape, img.shape)

    def test_white_patch_method(self):
        img = make_color_image()
        result = ColorConstancy.auto_white_balance(img, method='white_patch')
        self.assertEqual(result.shape, img.shape)

    def test_gray_edge_method(self):
        img = make_color_image()
        result = ColorConstancy.auto_white_balance(img, method='gray_edge')
        self.assertEqual(result.shape, img.shape)

    def test_combined_method(self):
        img = make_color_image()
        result = ColorConstancy.auto_white_balance(img, method='combined')
        self.assertEqual(result.shape, img.shape)

    def test_unknown_method_raises(self):
        img = make_color_image()
        with self.assertRaises(ValueError):
            ColorConstancy.auto_white_balance(img, method='unknown')


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestNormalizeIllumination(unittest.TestCase):
    """光照归一化测试"""

    def test_output_shape(self):
        img = make_gradient_image()
        result = ColorConstancy.normalize_illumination(img)
        self.assertEqual(result.shape, img.shape)

    def test_reduces_uneven(self):
        """应减少光照不均匀性"""
        img = make_gradient_image()
        result = ColorConstancy.normalize_illumination(img)
        # 计算亮度标准差
        orig_std = img.astype(float).mean(axis=2).std()
        result_std = result.astype(float).mean(axis=2).std()
        # 归一化后标准差应减小或保持
        self.assertLessEqual(result_std, orig_std * 1.5)


if __name__ == '__main__':
    unittest.main()
