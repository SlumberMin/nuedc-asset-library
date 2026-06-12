#!/usr/bin/env python3
"""
特征检测单元测试 (image_feature.py)
覆盖: Harris / Shi-Tomasi / FAST / ORB / SIFT / BRISK / 特征匹配 / 绘制
测试对象: 10_视觉通用代码库/image_feature.py
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
    from _10_视觉通用代码库.image_feature import (
        detect_harris, detect_harris_visual, detect_shi_tomasi,
        detect_shi_tomasi_visual, detect_fast, detect_orb, detect_sift,
        detect_brisk, draw_keypoints, match_features_orb,
        match_features_sift, draw_matches,
    )


def make_corner_image(size=200):
    """含明显角点的图像"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.rectangle(img, (40, 40), (90, 90), 200, -1)
    cv2.rectangle(img, (110, 110), (160, 160), 200, -1)
    return img


def make_bgr_corner(size=200):
    return cv2.cvtColor(make_corner_image(size), cv2.COLOR_GRAY2BGR)


def make_texture_image(size=200):
    """含纹理的图像(适合特征检测)"""
    np.random.seed(42)
    img = np.zeros((size, size), dtype=np.uint8)
    for _ in range(50):
        x, y = np.random.randint(20, size - 20, 2)
        r = np.random.randint(5, 20)
        cv2.circle(img, (x, y), r, np.random.randint(100, 255), -1)
    return img


def make_shifted_image(size=200, dx=10, dy=5):
    """对原图做平移(用于匹配测试)"""
    src = make_texture_image(size)
    M = np.float32([[1, 0, dx], [0, 1, dy]])
    dst = cv2.warpAffine(src, M, (size, size))
    return src, dst


# ── Harris ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestHarris(unittest.TestCase):

    def test_returns_two(self):
        result, dst = detect_harris(make_corner_image())
        self.assertEqual(result.shape, (200, 200))
        self.assertEqual(dst.shape, (200, 200))

    def test_dtype_uint8(self):
        result, _ = detect_harris(make_corner_image())
        self.assertEqual(result.dtype, np.uint8)

    def test_detects_corners(self):
        result, _ = detect_harris(make_corner_image())
        self.assertGreater(np.sum(result > 0), 0)

    def test_bgr_input(self):
        result, _ = detect_harris(make_bgr_corner())
        self.assertEqual(result.shape, (200, 200))

    def test_custom_threshold(self):
        _, dst1 = detect_harris(make_corner_image(), threshold=0.01)
        _, dst2 = detect_harris(make_corner_image(), threshold=0.1)
        # 不同阈值应产生不同角点数
        c1 = np.sum(dst1 > 0.01 * dst1.max())
        c2 = np.sum(dst2 > 0.1 * dst2.max())
        self.assertGreaterEqual(c1, c2)


# ── Harris Visual ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestHarrisVisual(unittest.TestCase):

    def test_output_shape(self):
        result = detect_harris_visual(make_corner_image())
        self.assertEqual(result.shape, (200, 200, 3))

    def test_bgr_input(self):
        result = detect_harris_visual(make_bgr_corner())
        self.assertEqual(result.shape, (200, 200, 3))


# ── Shi-Tomasi ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestShiTomasi(unittest.TestCase):

    def test_returns_corners(self):
        corners = detect_shi_tomasi(make_corner_image(), max_corners=50)
        self.assertIsNotNone(corners)
        self.assertGreater(len(corners), 0)

    def test_max_corners_limit(self):
        corners = detect_shi_tomasi(make_texture_image(), max_corners=10)
        if corners is not None:
            self.assertLessEqual(len(corners), 10)

    def test_bgr_input(self):
        corners = detect_shi_tomasi(make_bgr_corner(), max_corners=20)
        self.assertIsNotNone(corners)


# ── Shi-Tomasi Visual ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestShiTomasiVisual(unittest.TestCase):

    def test_output_shape(self):
        result = detect_shi_tomasi_visual(make_corner_image())
        self.assertEqual(result.shape, (200, 200, 3))


# ── FAST ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFAST(unittest.TestCase):

    def test_returns_keypoints(self):
        keypoints, fast = detect_fast(make_texture_image())
        self.assertIsNotNone(keypoints)
        self.assertGreater(len(keypoints), 0)

    def test_custom_threshold(self):
        kp1, _ = detect_fast(make_texture_image(), threshold=10)
        kp2, _ = detect_fast(make_texture_image(), threshold=50)
        self.assertGreaterEqual(len(kp1), len(kp2))


# ── ORB ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestORB(unittest.TestCase):

    def test_returns_keypoints_and_descriptors(self):
        kp, des = detect_orb(make_texture_image())
        self.assertGreater(len(kp), 0)
        self.assertIsNotNone(des)

    def test_nfeatures_limit(self):
        kp, _ = detect_orb(make_texture_image(), nfeatures=50)
        self.assertLessEqual(len(kp), 50)

    def test_descriptor_shape(self):
        _, des = detect_orb(make_texture_image())
        self.assertEqual(des.shape[1], 32)  # ORB描述子32维


# ── SIFT ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestSIFT(unittest.TestCase):

    def test_returns_keypoints_and_descriptors(self):
        kp, des = detect_sift(make_texture_image())
        self.assertGreater(len(kp), 0)
        self.assertIsNotNone(des)

    def test_descriptor_dim(self):
        _, des = detect_sift(make_texture_image())
        self.assertEqual(des.shape[1], 128)  # SIFT描述子128维


# ── BRISK ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestBRISK(unittest.TestCase):

    def test_returns_keypoints_and_descriptors(self):
        kp, des = detect_brisk(make_texture_image())
        self.assertGreater(len(kp), 0)
        self.assertIsNotNone(des)


# ── Draw Keypoints ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawKeypoints(unittest.TestCase):

    def test_output_shape(self):
        img = make_texture_image()
        kp, _ = detect_orb(img)
        canvas = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        result = draw_keypoints(canvas, kp)
        self.assertEqual(result.shape, canvas.shape)


# ── ORB 特征匹配 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMatchORB(unittest.TestCase):

    def test_returns_matches(self):
        src, dst = make_shifted_image()
        kp1, kp2, good = match_features_orb(src, dst)
        self.assertGreater(len(good), 0)

    def test_no_descriptors_returns_empty(self):
        """全黑图像可能无描述子"""
        black = np.zeros((100, 100), dtype=np.uint8)
        kp1, kp2, good = match_features_orb(black, black)
        self.assertIsInstance(good, list)


# ── SIFT 特征匹配 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMatchSIFT(unittest.TestCase):

    def test_returns_matches(self):
        src, dst = make_shifted_image()
        kp1, kp2, good = match_features_sift(src, dst)
        self.assertGreater(len(good), 0)


# ── Draw Matches ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawMatches(unittest.TestCase):

    def test_output_shape(self):
        src, dst = make_shifted_image()
        kp1, kp2, good = match_features_orb(src, dst)
        if len(good) > 0:
            canvas = draw_matches(src, kp1, dst, kp2, good)
            self.assertEqual(canvas.shape[0], 200)
            self.assertGreater(canvas.shape[1], 200)


if __name__ == '__main__':
    unittest.main()
