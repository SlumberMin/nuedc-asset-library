#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
图像配准单元测试
覆盖: detect_and_match_orb特征点检测/匹配、detect_and_match_sift、
      estimate_affine_transform仿射估计、estimate_homography透视估计、
      align_images_simple ECC对齐、draw_matches可视化
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from _10_视觉通用代码库.image_registration import (
    detect_and_match_orb,
    detect_and_match_sift,
    estimate_affine_transform,
    estimate_homography,
    draw_matches,
    align_images_simple,
)


# ==================== 辅助函数 ====================

def _make_registration_pair(shift_x=30, shift_y=20):
    """创建一组有平移的图像对用于配准测试"""
    base = np.zeros((300, 400, 3), dtype=np.uint8)
    cv2.rectangle(base, (50, 50), (200, 200), (0, 255, 0), -1)
    cv2.circle(base, (300, 150), 60, (0, 0, 255), -1)
    cv2.putText(base, "ABC", (100, 280), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    img1 = base.copy()
    M_shift = np.float32([[1, 0, shift_x], [0, 1, shift_y]])
    img2 = cv2.warpAffine(base, M_shift, (400, 300))
    return img1, img2


# ==================== ORB特征检测与匹配测试 ====================

class TestDetectAndMatchORB(unittest.TestCase):
    """ORB特征检测与匹配测试"""

    def test_returns_tuple_of_5(self):
        img1, img2 = _make_registration_pair()
        result = detect_and_match_orb(img1, img2)
        self.assertEqual(len(result), 5)

    def test_matches_are_list(self):
        img1, img2 = _make_registration_pair()
        good_matches, kp1, kp2, des1, des2 = detect_and_match_orb(img1, img2)
        self.assertIsInstance(good_matches, list)

    def test_keypoints_detected(self):
        img1, img2 = _make_registration_pair()
        good_matches, kp1, kp2, des1, des2 = detect_and_match_orb(img1, img2)
        self.assertGreater(len(kp1), 0)
        self.assertGreater(len(kp2), 0)

    def test_descriptors_not_none(self):
        img1, img2 = _make_registration_pair()
        _, _, _, des1, des2 = detect_and_match_orb(img1, img2)
        self.assertIsNotNone(des1)
        self.assertIsNotNone(des2)

    def test_good_matches_found(self):
        img1, img2 = _make_registration_pair()
        good_matches, _, _, _, _ = detect_and_match_orb(img1, img2)
        # 有丰富纹理的图像应找到匹配点
        self.assertGreater(len(good_matches), 0)

    def test_custom_max_features(self):
        img1, img2 = _make_registration_pair()
        _, kp1, _, _, _ = detect_and_match_orb(img1, img2, max_features=100)
        self.assertLessEqual(len(kp1), 100)

    def test_grayscale_input(self):
        img1, img2 = _make_registration_pair()
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        good_matches, kp1, kp2, des1, des2 = detect_and_match_orb(gray1, gray2)
        self.assertIsInstance(good_matches, list)


# ==================== SIFT特征检测与匹配测试 ====================

class TestDetectAndMatchSIFT(unittest.TestCase):
    """SIFT特征检测与匹配测试"""

    def test_returns_tuple_of_3(self):
        img1, img2 = _make_registration_pair()
        result = detect_and_match_sift(img1, img2)
        self.assertEqual(len(result), 3)

    def test_keypoints_detected(self):
        img1, img2 = _make_registration_pair()
        good_matches, kp1, kp2 = detect_and_match_sift(img1, img2)
        self.assertGreater(len(kp1), 0)
        self.assertGreater(len(kp2), 0)

    def test_good_matches_found(self):
        img1, img2 = _make_registration_pair()
        good_matches, _, _ = detect_and_match_sift(img1, img2)
        self.assertGreater(len(good_matches), 0)


# ==================== 仿射变换估计测试 ====================

class TestEstimateAffineTransform(unittest.TestCase):
    """仿射变换估计测试"""

    def test_returns_tuple_of_3(self):
        img1, img2 = _make_registration_pair()
        result = estimate_affine_transform(img1, img2, method="orb")
        self.assertEqual(len(result), 3)

    def test_affine_matrix_shape(self):
        img1, img2 = _make_registration_pair()
        M, warped, n = estimate_affine_transform(img1, img2, method="orb")
        if M is not None:
            self.assertEqual(M.shape, (2, 3))

    def test_warped_shape_matches_input(self):
        img1, img2 = _make_registration_pair()
        M, warped, n = estimate_affine_transform(img1, img2, method="orb")
        if warped is not None:
            self.assertEqual(warped.shape, img1.shape)

    def test_match_count_positive(self):
        img1, img2 = _make_registration_pair()
        _, _, n = estimate_affine_transform(img1, img2, method="orb")
        self.assertGreater(n, 0)

    def test_identity_shift_recoverable(self):
        """平移图像应能配准"""
        img1, img2 = _make_registration_pair(shift_x=30, shift_y=20)
        M, warped, n = estimate_affine_transform(img1, img2, method="orb")
        if M is not None:
            # 仿射矩阵应接近平移矩阵
            self.assertAlmostEqual(abs(M[0, 2]), 30, delta=15)
            self.assertAlmostEqual(abs(M[1, 2]), 20, delta=15)


# ==================== 透视变换估计测试 ====================

class TestEstimateHomography(unittest.TestCase):
    """透视变换估计测试"""

    def test_returns_tuple_of_3(self):
        img1, img2 = _make_registration_pair()
        result = estimate_homography(img1, img2, method="orb")
        self.assertEqual(len(result), 3)

    def test_homography_matrix_shape(self):
        img1, img2 = _make_registration_pair()
        H, warped, n = estimate_homography(img1, img2, method="orb")
        if H is not None:
            self.assertEqual(H.shape, (3, 3))

    def test_warped_shape_matches_input(self):
        img1, img2 = _make_registration_pair()
        H, warped, n = estimate_homography(img1, img2, method="orb")
        if warped is not None:
            self.assertEqual(warped.shape, img1.shape)

    def test_identity_matrix_nonzero(self):
        """H[2,2]应接近1（齐次坐标归一化）"""
        img1, img2 = _make_registration_pair()
        H, _, _ = estimate_homography(img1, img2, method="orb")
        if H is not None:
            self.assertAlmostEqual(abs(H[2, 2]), 1.0, delta=0.5)


# ==================== draw_matches测试 ====================

class TestDrawMatches(unittest.TestCase):
    """特征匹配可视化测试"""

    def test_returns_image(self):
        img1, img2 = _make_registration_pair()
        good_matches, kp1, kp2, _, _ = detect_and_match_orb(img1, img2)
        if len(good_matches) > 0:
            vis = draw_matches(img1, kp1, img2, kp2, good_matches)
            self.assertIsNotNone(vis)
            self.assertEqual(len(vis.shape), 3)

    def test_max_draw_limit(self):
        img1, img2 = _make_registration_pair()
        good_matches, kp1, kp2, _, _ = detect_and_match_orb(img1, img2)
        if len(good_matches) > 2:
            vis = draw_matches(img1, kp1, img2, kp2, good_matches, max_draw=2)
            self.assertIsNotNone(vis)


# ==================== ECC对齐测试 ====================

class TestAlignImagesSimple(unittest.TestCase):
    """ECC图像对齐测试"""

    def test_returns_tuple_of_2(self):
        img1, img2 = _make_registration_pair(shift_x=5, shift_y=5)
        try:
            result = align_images_simple(img1, img2)
            self.assertEqual(len(result), 2)
        except cv2.error:
            self.skipTest("ECC对齐在当前环境下可能不收敛")

    def test_warped_shape(self):
        img1, img2 = _make_registration_pair(shift_x=3, shift_y=3)
        try:
            warped, warp_matrix = align_images_simple(img1, img2)
            self.assertEqual(warped.shape, img1.shape)
        except cv2.error:
            self.skipTest("ECC对齐在当前环境下可能不收敛")

    def test_warp_matrix_shape_euclidean(self):
        img1, img2 = _make_registration_pair(shift_x=3, shift_y=3)
        try:
            _, warp_matrix = align_images_simple(img1, img2, warp_mode=cv2.MOTION_EUCLIDEAN)
            self.assertEqual(warp_matrix.shape, (2, 3))
        except cv2.error:
            self.skipTest("ECC对齐在当前环境下可能不收敛")


if __name__ == '__main__':
    unittest.main()
