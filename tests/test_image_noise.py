#!/usr/bin/env python3
"""
图像噪声添加单元测试
覆盖: 高斯/椒盐/泊松/斑点噪声、统一接口、批量生成、输出属性验证
"""
import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '10_视觉通用代码库'))
from image_noise import ImageNoise, add_gaussian_noise, add_salt_pepper_noise, add_poisson_noise, add_speckle_noise


class TestGaussianNoise(unittest.TestCase):
    """高斯噪声测试"""

    def setUp(self):
        self.image = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        self.color_image = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)

    def test_output_dtype_uint8(self):
        """输出应为uint8"""
        result = ImageNoise.gaussian_noise(self.image)
        self.assertEqual(result.dtype, np.uint8)

    def test_output_shape_preserved(self):
        """输出形状应与输入一致"""
        result = ImageNoise.gaussian_noise(self.color_image)
        self.assertEqual(result.shape, self.color_image.shape)

    def test_output_range_0_255(self):
        """输出值应限制在[0,255]"""
        result = ImageNoise.gaussian_noise(self.image, mean=0, sigma=100)
        self.assertTrue(np.all(result >= 0))
        self.assertTrue(np.all(result <= 255))

    def test_noise_changes_image(self):
        """添加噪声后图像应有变化"""
        result = ImageNoise.gaussian_noise(self.image, sigma=25)
        diff = np.mean(np.abs(self.image.astype(float) - result.astype(float)))
        self.assertGreater(diff, 1.0)

    def test_stronger_noise_more_change(self):
        """更强的噪声应产生更大的变化"""
        weak = ImageNoise.gaussian_noise(self.image, sigma=10)
        strong = ImageNoise.gaussian_noise(self.image, sigma=50)
        diff_weak = np.mean(np.abs(self.image.astype(float) - weak.astype(float)))
        diff_strong = np.mean(np.abs(self.image.astype(float) - strong.astype(float)))
        self.assertGreater(diff_strong, diff_weak)

    def test_zero_sigma_no_change(self):
        """sigma=0时应无变化"""
        result = ImageNoise.gaussian_noise(self.image, sigma=0)
        np.testing.assert_array_equal(result, self.image)


class TestSaltPepperNoise(unittest.TestCase):
    """椒盐噪声测试"""

    def setUp(self):
        self.image = np.full((100, 100), 128, dtype=np.uint8)

    def test_output_dtype(self):
        result = ImageNoise.salt_pepper_noise(self.image)
        self.assertEqual(result.dtype, np.uint8)

    def test_has_salt_pixels(self):
        """应包含白点(255)"""
        result = ImageNoise.salt_pepper_noise(self.image, salt_prob=0.1, pepper_prob=0.0)
        salt_count = np.sum(result == 255)
        self.assertGreater(salt_count, 0)

    def test_has_pepper_pixels(self):
        """应包含黑点(0)"""
        result = ImageNoise.salt_pepper_noise(self.image, salt_prob=0.0, pepper_prob=0.1)
        pepper_count = np.sum(result == 0)
        self.assertGreater(pepper_count, 0)

    def test_color_image_salt_pepper(self):
        """彩色图像椒盐噪声"""
        color_img = np.full((64, 64, 3), 128, dtype=np.uint8)
        result = ImageNoise.salt_pepper_noise(color_img, salt_prob=0.05, pepper_prob=0.05)
        self.assertEqual(result.shape, color_img.shape)

    def test_zero_prob_no_change(self):
        """概率为0时应无变化"""
        result = ImageNoise.salt_pepper_noise(self.image, salt_prob=0.0, pepper_prob=0.0)
        np.testing.assert_array_equal(result, self.image)


class TestPoissonNoise(unittest.TestCase):
    """泊松噪声测试"""

    def setUp(self):
        self.image = np.random.randint(50, 200, (64, 64), dtype=np.uint8)

    def test_output_dtype(self):
        result = ImageNoise.poisson_noise(self.image)
        self.assertEqual(result.dtype, np.uint8)

    def test_output_range(self):
        result = ImageNoise.poisson_noise(self.image)
        self.assertTrue(np.all(result >= 0))
        self.assertTrue(np.all(result <= 255))

    def test_noise_changes_image(self):
        result = ImageNoise.poisson_noise(self.image)
        self.assertFalse(np.array_equal(result, self.image))


class TestSpeckleNoise(unittest.TestCase):
    """斑点噪声测试"""

    def setUp(self):
        self.image = np.random.randint(50, 200, (64, 64), dtype=np.uint8)

    def test_output_dtype(self):
        result = ImageNoise.speckle_noise(self.image)
        self.assertEqual(result.dtype, np.uint8)

    def test_output_range(self):
        result = ImageNoise.speckle_noise(self.image, intensity=0.5)
        self.assertTrue(np.all(result >= 0))
        self.assertTrue(np.all(result <= 255))

    def test_stronger_intensity_more_change(self):
        weak = ImageNoise.speckle_noise(self.image, intensity=0.05)
        strong = ImageNoise.speckle_noise(self.image, intensity=0.5)
        diff_weak = np.mean(np.abs(self.image.astype(float) - weak.astype(float)))
        diff_strong = np.mean(np.abs(self.image.astype(float) - strong.astype(float)))
        self.assertGreater(diff_strong, diff_weak)


class TestAddNoiseInterface(unittest.TestCase):
    """统一噪声接口测试"""

    def setUp(self):
        self.image = np.random.randint(0, 256, (64, 64), dtype=np.uint8)

    def test_all_noise_types(self):
        """所有噪声类型应正常工作"""
        for nt in ['gaussian', 'salt_pepper', 'poisson', 'speckle']:
            result = ImageNoise.add_noise(self.image, nt)
            self.assertEqual(result.shape, self.image.shape)
            self.assertEqual(result.dtype, np.uint8)

    def test_unsupported_noise_type(self):
        """不支持的噪声类型应抛出ValueError"""
        with self.assertRaises(ValueError):
            ImageNoise.add_noise(self.image, 'invalid_noise')


class TestGenerateNoisyDataset(unittest.TestCase):
    """批量生成噪声数据集测试"""

    def setUp(self):
        self.image = np.random.randint(0, 256, (64, 64), dtype=np.uint8)

    def test_default_generates_all(self):
        result = ImageNoise.generate_noisy_dataset(self.image)
        self.assertEqual(len(result), 4)
        for nt in ['gaussian', 'salt_pepper', 'poisson', 'speckle']:
            self.assertIn(nt, result)

    def test_custom_noise_types(self):
        result = ImageNoise.generate_noisy_dataset(self.image, noise_types=['gaussian', 'poisson'])
        self.assertEqual(len(result), 2)
        self.assertIn('gaussian', result)
        self.assertIn('poisson', result)

    def test_output_shapes(self):
        result = ImageNoise.generate_noisy_dataset(self.image)
        for nt, img in result.items():
            self.assertEqual(img.shape, self.image.shape)


class TestShortcutFunctions(unittest.TestCase):
    """快捷函数测试"""

    def setUp(self):
        self.image = np.random.randint(0, 256, (64, 64), dtype=np.uint8)

    def test_add_gaussian_noise(self):
        result = add_gaussian_noise(self.image, sigma=15)
        self.assertEqual(result.dtype, np.uint8)
        self.assertEqual(result.shape, self.image.shape)

    def test_add_salt_pepper_noise(self):
        result = add_salt_pepper_noise(self.image, salt_prob=0.02, pepper_prob=0.02)
        self.assertEqual(result.dtype, np.uint8)

    def test_add_poisson_noise(self):
        result = add_poisson_noise(self.image)
        self.assertEqual(result.dtype, np.uint8)

    def test_add_speckle_noise(self):
        result = add_speckle_noise(self.image, intensity=0.1)
        self.assertEqual(result.dtype, np.uint8)


if __name__ == '__main__':
    unittest.main()
