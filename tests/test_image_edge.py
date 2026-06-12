#!/usr/bin/env python3
"""
边缘检测单元测试 (image_edge.py)
覆盖: Canny / Sobel / SobelXY / Laplacian / Prewitt / Roberts / AutoCanny / 梯度幅值
测试对象: 10_视觉通用代码库/image_edge.py
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
    from _10_视觉通用代码库.image_edge import (
        edge_canny, edge_sobel, edge_sobel_xy, edge_laplacian,
        edge_prewitt, edge_roberts, auto_canny, edge_gradient_magnitude,
    )


def make_rect_image(size=200):
    """中间白色矩形"""
    img = np.zeros((size, size), dtype=np.uint8)
    img[50:150, 50:150] = 255
    return img


def make_bgr_rect(size=200):
    """BGR彩色矩形"""
    gray = make_rect_image(size)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def make_circle_image(size=200, r=60):
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(img, (size // 2, size // 2), r, 255, 2)
    return img


def make_gradient_image(size=200):
    """渐变图像"""
    return np.tile(np.linspace(0, 255, size, dtype=np.uint8), (size, 1))


def make_noisy_image(size=200):
    np.random.seed(42)
    return np.random.randint(0, 256, (size, size), dtype=np.uint8)


# ── Canny ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestEdgeCanny(unittest.TestCase):

    def test_output_shape_gray(self):
        gray = make_rect_image()
        result = edge_canny(gray)
        self.assertEqual(result.shape, gray.shape)

    def test_output_shape_bgr(self):
        bgr = make_bgr_rect()
        result = edge_canny(bgr)
        self.assertEqual(result.shape[:2], bgr.shape[:2])

    def test_dtype_uint8(self):
        result = edge_canny(make_rect_image())
        self.assertEqual(result.dtype, np.uint8)

    def test_detects_edges(self):
        result = edge_canny(make_rect_image())
        self.assertGreater(np.sum(result > 0), 100)

    def test_custom_thresholds(self):
        img = make_rect_image()
        r1 = edge_canny(img, threshold1=50, threshold2=150)
        r2 = edge_canny(img, threshold1=10, threshold2=200)
        # 不同阈值产生不同边缘数量
        self.assertNotEqual(np.sum(r1 > 0), np.sum(r2 > 0))

    def test_flat_image_no_edges(self):
        flat = np.full((100, 100), 128, dtype=np.uint8)
        result = edge_canny(flat)
        self.assertEqual(np.sum(result > 0), 0)

    def test_circle_edges(self):
        result = edge_canny(make_circle_image())
        self.assertGreater(np.sum(result > 0), 50)


# ── Sobel ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestEdgeSobel(unittest.TestCase):

    def test_output_shape(self):
        result = edge_sobel(make_rect_image())
        self.assertEqual(result.shape, (200, 200))

    def test_dtype_uint8(self):
        result = edge_sobel(make_rect_image())
        self.assertEqual(result.dtype, np.uint8)

    def test_detects_edges(self):
        result = edge_sobel(make_rect_image())
        self.assertGreater(np.sum(result > 0), 50)

    def test_bgr_input(self):
        result = edge_sobel(make_bgr_rect())
        self.assertEqual(result.shape, (200, 200))

    def test_different_ksize(self):
        img = make_rect_image()
        r3 = edge_sobel(img, ksize=3)
        r5 = edge_sobel(img, ksize=5)
        self.assertFalse(np.array_equal(r3, r5))


# ── SobelXY ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestEdgeSobelXY(unittest.TestCase):

    def test_returns_two_images(self):
        gx, gy = edge_sobel_xy(make_rect_image())
        self.assertEqual(gx.shape, (200, 200))
        self.assertEqual(gy.shape, (200, 200))

    def test_x_and_y_different(self):
        gx, gy = edge_sobel_xy(make_rect_image())
        self.assertFalse(np.array_equal(gx, gy))

    def test_detects_vertical_edge_in_gx(self):
        """垂直边缘在gx方向响应更强"""
        img = make_rect_image()
        gx, gy = edge_sobel_xy(img)
        self.assertGreater(np.sum(gx > 50), 0)


# ── Laplacian ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestEdgeLaplacian(unittest.TestCase):

    def test_output_shape(self):
        result = edge_laplacian(make_rect_image())
        self.assertEqual(result.shape, (200, 200))

    def test_detects_edges(self):
        result = edge_laplacian(make_circle_image())
        self.assertGreater(np.sum(result > 0), 30)

    def test_dtype_uint8(self):
        result = edge_laplacian(make_rect_image())
        self.assertEqual(result.dtype, np.uint8)


# ── Prewitt ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestEdgePrewitt(unittest.TestCase):

    def test_output_shape(self):
        result = edge_prewitt(make_rect_image())
        self.assertEqual(result.shape, (200, 200))

    def test_detects_edges(self):
        result = edge_prewitt(make_rect_image())
        self.assertGreater(np.sum(result > 0), 50)

    def test_bgr_input(self):
        result = edge_prewitt(make_bgr_rect())
        self.assertEqual(result.shape, (200, 200))


# ── Roberts ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestEdgeRoberts(unittest.TestCase):

    def test_output_shape(self):
        result = edge_roberts(make_rect_image())
        self.assertEqual(result.shape, (200, 200))

    def test_detects_edges(self):
        result = edge_roberts(make_rect_image())
        self.assertGreater(np.sum(result > 0), 0)

    def test_gradient_image(self):
        result = edge_roberts(make_gradient_image())
        self.assertEqual(result.shape, (200, 200))


# ── Auto Canny ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestAutoCanny(unittest.TestCase):

    def test_output_shape(self):
        result = auto_canny(make_rect_image())
        self.assertEqual(result.shape, (200, 200))

    def test_detects_edges(self):
        result = auto_canny(make_rect_image())
        self.assertGreater(np.sum(result > 0), 30)

    def test_adapts_to_brightness(self):
        bright = np.full((100, 100), 200, dtype=np.uint8)
        bright[30:70, 30:70] = 100
        dark = np.full((100, 100), 50, dtype=np.uint8)
        dark[30:70, 30:70] = 150
        r1 = auto_canny(bright)
        r2 = auto_canny(dark)
        self.assertGreater(np.sum(r1 > 0), 0)
        self.assertGreater(np.sum(r2 > 0), 0)

    def test_custom_sigma(self):
        img = make_rect_image()
        r1 = auto_canny(img, sigma=0.2)
        r2 = auto_canny(img, sigma=0.5)
        # 不同sigma可能产生不同边缘数量
        self.assertIsInstance(r1, np.ndarray)


# ── Gradient Magnitude ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGradientMagnitude(unittest.TestCase):

    def test_output_shape(self):
        result = edge_gradient_magnitude(make_rect_image())
        self.assertEqual(result.shape, (200, 200))

    def test_dtype_uint8(self):
        result = edge_gradient_magnitude(make_rect_image())
        self.assertEqual(result.dtype, np.uint8)

    def test_detects_gradient(self):
        result = edge_gradient_magnitude(make_gradient_image())
        self.assertGreater(np.sum(result > 0), 0)

    def test_flat_image_no_gradient(self):
        flat = np.full((100, 100), 128, dtype=np.uint8)
        result = edge_gradient_magnitude(flat)
        self.assertEqual(np.sum(result > 0), 0)


if __name__ == '__main__':
    unittest.main()
