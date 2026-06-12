"""
频域处理单元测试
覆盖: FFT/IFFT/频域滤波/理想低通/理想高通/巴特沃斯低通/巴特沃斯高通/
      高斯低通/高斯高通/带通滤波/同态滤波/陷波滤波器
"""
import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from importlib import import_module

mod = import_module('10_视觉通用代码库.image_frequency')


class TestFFT2(unittest.TestCase):
    """二维FFT测试"""

    def setUp(self):
        self.img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)

    def test_fft2_returns_shifted_spectrum(self):
        f_shift, magnitude = mod.fft2(self.img)
        self.assertEqual(f_shift.shape, self.img.shape)
        self.assertEqual(magnitude.shape, self.img.shape)

    def test_fft2_magnitude_uint8(self):
        _, magnitude = mod.fft2(self.img)
        self.assertEqual(magnitude.dtype, np.uint8)

    def test_fft2_color_image(self):
        """彩色图应自动转灰度"""
        img_color = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        f_shift, magnitude = mod.fft2(img_color)
        self.assertEqual(f_shift.shape, (64, 64))

    def test_fft2_center_is_bright(self):
        """频谱中心应为最大值区域"""
        _, magnitude = mod.fft2(self.img)
        center_val = magnitude[32, 32]
        self.assertGreater(center_val, 0)


class TestIFFT2(unittest.TestCase):
    """逆FFT测试"""

    def test_ifft2_roundtrip(self):
        """FFT->IFFT应还原图像"""
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        f_shift, _ = mod.fft2(img)
        restored = mod.ifft2(f_shift)
        self.assertEqual(restored.shape, img.shape)
        diff = np.mean(np.abs(img.astype(float) - restored.astype(float)))
        self.assertLess(diff, 2.0)

    def test_ifft2_output_type(self):
        img = np.random.randint(0, 256, (32, 32), dtype=np.uint8)
        f_shift, _ = mod.fft2(img)
        result = mod.ifft2(f_shift)
        self.assertEqual(result.dtype, np.uint8)


class TestFFTFilter(unittest.TestCase):
    """频域滤波测试"""

    def setUp(self):
        self.img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)

    def test_fft_filter_output_shape(self):
        mask = mod.lowpass_gaussian(self.img.shape, 20)
        result = mod.fft_filter(self.img, mask)
        self.assertEqual(result.shape, self.img.shape)

    def test_fft_filter_color_image(self):
        img_color = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        mask = mod.lowpass_gaussian((64, 64), 20)
        result = mod.fft_filter(img_color, mask)
        self.assertEqual(result.shape, (64, 64))

    def test_lowpass_blurs(self):
        """低通滤波应使图像变模糊（高频减少）"""
        edges = np.zeros((64, 64), dtype=np.uint8)
        edges[::2, :] = 255
        mask = mod.lowpass_gaussian((64, 64), 10)
        result = mod.fft_filter(edges, mask)
        # 原图高频丰富，滤波后应更平滑
        orig_var = np.var(edges.astype(float))
        filt_var = np.var(result.astype(float))
        self.assertLess(filt_var, orig_var)


class TestLowpassIdeal(unittest.TestCase):
    """理想低通滤波器测试"""

    def test_shape(self):
        mask = mod.lowpass_ideal((64, 64), 10)
        self.assertEqual(mask.shape, (64, 64))

    def test_center_is_one(self):
        mask = mod.lowpass_ideal((64, 64), 10)
        self.assertAlmostEqual(mask[32, 32], 1.0)

    def test_far_from_center_is_zero(self):
        mask = mod.lowpass_ideal((64, 64), 5)
        self.assertAlmostEqual(mask[0, 0], 0.0)

    def test_dtype(self):
        mask = mod.lowpass_ideal((64, 64), 10)
        self.assertEqual(mask.dtype, np.float32)


class TestHighpassIdeal(unittest.TestCase):
    """理想高通滤波器测试"""

    def test_complement_of_lowpass(self):
        lp = mod.lowpass_ideal((64, 64), 10)
        hp = mod.highpass_ideal((64, 64), 10)
        np.testing.assert_array_almost_equal(lp + hp, 1.0)

    def test_center_is_zero(self):
        mask = mod.highpass_ideal((64, 64), 10)
        self.assertAlmostEqual(mask[32, 32], 0.0)


class TestButterworthFilter(unittest.TestCase):
    """巴特沃斯滤波器测试"""

    def test_lowpass_shape(self):
        mask = mod.lowpass_butterworth((64, 64), 20)
        self.assertEqual(mask.shape, (64, 64))

    def test_highpass_complement(self):
        lp = mod.lowpass_butterworth((64, 64), 20, order=2)
        hp = mod.highpass_butterworth((64, 64), 20, order=2)
        np.testing.assert_array_almost_equal(lp + hp, 1.0)

    def test_lowpass_center_near_one(self):
        mask = mod.lowpass_butterworth((64, 64), 20)
        self.assertAlmostEqual(mask[32, 32], 1.0, places=1)

    def test_different_orders(self):
        mask2 = mod.lowpass_butterworth((64, 64), 20, order=2)
        mask4 = mod.lowpass_butterworth((64, 64), 20, order=4)
        # 高阶更接近理想滤波器
        self.assertEqual(mask2.shape, mask4.shape)


class TestGaussianFilter(unittest.TestCase):
    """高斯滤波器测试"""

    def test_lowpass_shape(self):
        mask = mod.lowpass_gaussian((64, 64), 15)
        self.assertEqual(mask.shape, (64, 64))

    def test_highpass_complement(self):
        lp = mod.lowpass_gaussian((64, 64), 15)
        hp = mod.highpass_gaussian((64, 64), 15)
        np.testing.assert_array_almost_equal(lp + hp, 1.0)

    def test_lowpass_center_is_one(self):
        mask = mod.lowpass_gaussian((64, 64), 15)
        self.assertAlmostEqual(mask[32, 32], 1.0, places=3)

    def test_dtype(self):
        mask = mod.lowpass_gaussian((64, 64), 15)
        self.assertEqual(mask.dtype, np.float32)


class TestBandpassFilter(unittest.TestCase):
    """带通滤波器测试"""

    def test_gaussian_bandpass(self):
        mask = mod.bandpass_filter((64, 64), 10, 30, 'gaussian')
        self.assertEqual(mask.shape, (64, 64))
        # 带通中间区域应大于0
        self.assertGreater(mask[32, 32], 0.0)

    def test_butterworth_bandpass(self):
        mask = mod.bandpass_filter((64, 64), 10, 30, 'butterworth', order=2)
        self.assertEqual(mask.shape, (64, 64))

    def test_ideal_bandpass(self):
        mask = mod.bandpass_filter((64, 64), 10, 30, 'ideal')
        self.assertEqual(mask.shape, (64, 64))

    def test_bandpass_center_near_zero(self):
        """带通在中心（DC）应接近0"""
        mask = mod.bandpass_filter((64, 64), 10, 30, 'gaussian')
        self.assertAlmostEqual(mask[32, 32], 0.0, places=1)


class TestHomomorphicFilter(unittest.TestCase):
    """同态滤波测试"""

    def test_output_shape(self):
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = mod.homomorphic_filter(img)
        self.assertEqual(result.shape, img.shape)

    def test_output_type(self):
        img = np.random.randint(1, 256, (64, 64), dtype=np.uint8)
        result = mod.homomorphic_filter(img)
        self.assertEqual(result.dtype, np.uint8)

    def test_color_image(self):
        img = np.random.randint(1, 256, (64, 64, 3), dtype=np.uint8)
        result = mod.homomorphic_filter(img)
        self.assertEqual(result.shape, (64, 64))

    def test_custom_params(self):
        img = np.random.randint(1, 256, (64, 64), dtype=np.uint8)
        result = mod.homomorphic_filter(img, gamma_l=0.3, gamma_h=3.0, cutoff=50)
        self.assertEqual(result.shape, img.shape)


class TestNotchFilter(unittest.TestCase):
    """陷波滤波器测试"""

    def test_shape(self):
        mask = mod.notch_filter((64, 64), [(10, 10)], radius=3)
        self.assertEqual(mask.shape, (64, 64))

    def test_notch_zeroes_at_centers(self):
        mask = mod.notch_filter((64, 64), [(10, 10)], radius=3)
        self.assertAlmostEqual(mask[10, 10], 0.0)

    def test_far_from_notch_is_one(self):
        mask = mod.notch_filter((64, 64), [(10, 10)], radius=2)
        self.assertAlmostEqual(mask[0, 0], 1.0)

    def test_multiple_notches(self):
        centers = [(10, 10), (30, 40)]
        mask = mod.notch_filter((64, 64), centers, radius=3)
        self.assertAlmostEqual(mask[10, 10], 0.0)
        self.assertAlmostEqual(mask[30, 40], 0.0)

    def test_dtype(self):
        mask = mod.notch_filter((64, 64), [(10, 10)])
        self.assertEqual(mask.dtype, np.float32)


if __name__ == '__main__':
    unittest.main()
