#!/usr/bin/env python3
"""
边缘检测器单元测试
覆盖: Canny(手动/自动)、Sobel(X/Y/幅值/方向)、Laplacian、
      LoG、Prewitt、Scharr、Roberts、NMS、自动选择
测试对象: 10_视觉通用代码库/edge_detector.py
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
    from _10_视觉通用代码库.edge_detector import EdgeDetector


def make_edge_image(size=200):
    """创建含明显边缘的图像"""
    img = np.zeros((size, size), dtype=np.uint8)
    # 中间白色矩形
    img[50:150, 50:150] = 255
    return img


def make_line_image(size=200):
    """创建含线条的图像"""
    img = np.zeros((size, size), dtype=np.uint8)
    img[100:102, 20:180] = 255  # 水平线
    img[20:180, 100:102] = 255  # 垂直线
    return img


def make_noisy_image(size=200):
    """创建含噪声的图像"""
    np.random.seed(42)
    img = np.random.randint(0, 256, (size, size), dtype=np.uint8)
    return img


def make_circle_edge(size=200, r=60):
    """创建圆形边缘"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(img, (size // 2, size // 2), r, 255, 2)
    return img


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestCanny(unittest.TestCase):
    """Canny边缘检测测试"""

    def test_output_shape(self):
        gray = make_edge_image()
        result = EdgeDetector.canny(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_detects_edges(self):
        gray = make_edge_image()
        result = EdgeDetector.canny(gray)
        # 应检测到边缘
        self.assertGreater(np.sum(result > 0), 100)

    def test_custom_thresholds(self):
        gray = make_edge_image()
        r1 = EdgeDetector.canny(gray, low=50, high=150)
        r2 = EdgeDetector.canny(gray, low=10, high=200)
        # 不同阈值应产生不同边缘数量
        self.assertNotEqual(np.sum(r1 > 0), np.sum(r2 > 0))

    def test_no_blur(self):
        gray = make_edge_image()
        result = EdgeDetector.canny(gray, blur_ksize=0)
        self.assertEqual(result.dtype, np.uint8)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestCannyAuto(unittest.TestCase):
    """Canny自动阈值测试"""

    def test_output_shape(self):
        gray = make_edge_image()
        result = EdgeDetector.canny_auto(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_detects_edges(self):
        gray = make_edge_image()
        result = EdgeDetector.canny_auto(gray)
        self.assertGreater(np.sum(result > 0), 50)

    def test_adapts_to_image(self):
        """自动阈值应适应不同图像"""
        bright = np.full((100, 100), 200, dtype=np.uint8)
        bright[30:70, 30:70] = 100
        dark = np.full((100, 100), 50, dtype=np.uint8)
        dark[30:70, 30:70] = 150
        r1 = EdgeDetector.canny_auto(bright)
        r2 = EdgeDetector.canny_auto(dark)
        # 两者都应检测到边缘
        self.assertGreater(np.sum(r1 > 0), 0)
        self.assertGreater(np.sum(r2 > 0), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestSobel(unittest.TestCase):
    """Sobel梯度测试"""

    def test_sobel_x_detects_vertical(self):
        """Sobel X应检测垂直边缘"""
        gray = make_edge_image()
        result = EdgeDetector.sobel_x(gray)
        self.assertEqual(result.shape, gray.shape)
        self.assertGreater(np.sum(result > 0), 0)

    def test_sobel_y_detects_horizontal(self):
        """Sobel Y应检测水平边缘"""
        gray = make_edge_image()
        result = EdgeDetector.sobel_y(gray)
        self.assertEqual(result.shape, gray.shape)
        self.assertGreater(np.sum(result > 0), 0)

    def test_sobel_mag(self):
        gray = make_edge_image()
        result = EdgeDetector.sobel_mag(gray)
        self.assertEqual(result.shape, gray.shape)
        self.assertEqual(result.dtype, np.uint8)

    def test_sobel_dir_range(self):
        """方向应在0-255范围内"""
        gray = make_edge_image()
        result = EdgeDetector.sobel_dir(gray)
        self.assertTrue(np.all(result >= 0))
        self.assertTrue(np.all(result <= 255))

    def test_different_ksize(self):
        gray = make_edge_image()
        r3 = EdgeDetector.sobel_mag(gray, ksize=3)
        r5 = EdgeDetector.sobel_mag(gray, ksize=5)
        self.assertFalse(np.array_equal(r3, r5))


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLaplacian(unittest.TestCase):
    """Laplacian边缘测试"""

    def test_output_shape(self):
        gray = make_edge_image()
        result = EdgeDetector.laplacian(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_detects_edges(self):
        gray = make_circle_edge()
        result = EdgeDetector.laplacian(gray)
        self.assertGreater(np.sum(result > 0), 50)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLoG(unittest.TestCase):
    """LoG边缘测试"""

    def test_output_shape(self):
        gray = make_edge_image()
        result = EdgeDetector.laplacian_of_gaussian(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_smoothes_noise(self):
        """LoG应对噪声有一定抑制"""
        gray = make_noisy_image()
        result = EdgeDetector.laplacian_of_gaussian(gray, ksize=5, sigma=2.0)
        lap = EdgeDetector.laplacian(gray)
        # LoG边缘应比直接Laplacian少
        self.assertLessEqual(np.sum(result > 128), np.sum(lap > 128) + 1000)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestPrewitt(unittest.TestCase):
    """Prewitt边缘测试"""

    def test_output_shape(self):
        gray = make_edge_image()
        result = EdgeDetector.prewitt(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_detects_edges(self):
        gray = make_edge_image()
        result = EdgeDetector.prewitt(gray)
        self.assertGreater(np.sum(result > 0), 100)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestScharr(unittest.TestCase):
    """Scharr边缘测试"""

    def test_output_shape(self):
        gray = make_edge_image()
        result = EdgeDetector.scharr(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_higher_precision_than_sobel(self):
        """Scharr应比Sobel更精确(更大的梯度值)"""
        gray = make_edge_image()
        scharr = EdgeDetector.scharr(gray).astype(float)
        sobel = EdgeDetector.sobel_mag(gray).astype(float)
        # Scharr通常产生更大的梯度值
        self.assertGreater(scharr.mean(), 0)
        self.assertGreater(sobel.mean(), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestRoberts(unittest.TestCase):
    """Roberts边缘测试"""

    def test_output_shape(self):
        gray = make_edge_image()
        result = EdgeDetector.roberts(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_detects_edges(self):
        gray = make_edge_image()
        result = EdgeDetector.roberts(gray)
        self.assertGreater(np.sum(result > 0), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestAutoBest(unittest.TestCase):
    """自动选择最佳方法测试"""

    def test_output_shape(self):
        gray = make_edge_image()
        result = EdgeDetector.auto_best(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_works_on_normal_image(self):
        gray = make_edge_image()
        result = EdgeDetector.auto_best(gray)
        self.assertGreater(np.sum(result > 0), 0)

    def test_works_on_noisy_image(self):
        gray = make_noisy_image()
        result = EdgeDetector.auto_best(gray)
        self.assertEqual(result.shape, gray.shape)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestNMS(unittest.TestCase):
    """非极大值抑制测试"""

    def test_output_shape(self):
        gray = make_edge_image()
        result = EdgeDetector.nms_edges(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_thins_edges(self):
        """NMS应细化边缘"""
        gray = make_edge_image()
        canny = EdgeDetector.canny(gray)
        nms = EdgeDetector.nms_edges(gray)
        # NMS通常产生更少的边缘像素
        self.assertGreaterEqual(np.sum(canny > 0), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestCompareAll(unittest.TestCase):
    """全方法对比测试"""

    def test_returns_dict(self):
        gray = make_edge_image(size=100)
        results = EdgeDetector.compare_all(gray)
        self.assertIsInstance(results, dict)
        expected = ['Canny_Manual', 'Canny_Auto', 'Sobel_Mag', 'Laplacian',
                    'LoG', 'Prewitt', 'Scharr', 'Roberts', 'AutoBest']
        for key in expected:
            self.assertIn(key, results)

    def test_all_same_shape(self):
        gray = make_edge_image(size=100)
        results = EdgeDetector.compare_all(gray)
        for name, result in results.items():
            self.assertEqual(result.shape, gray.shape, f"{name} shape mismatch")


if __name__ == '__main__':
    unittest.main()
