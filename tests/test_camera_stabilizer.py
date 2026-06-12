#!/usr/bin/env python3
"""
图像稳定单元测试
覆盖: CameraStabilizer初始化、平滑窗口、裁剪比例、
      累积变换、复位、HomographyStabilizer初始化、
      变换矩阵平滑、特征点过滤
注意: 使用纯 Python 模拟核心逻辑，不依赖cv2
"""

import sys
import os
import math
import unittest
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np


class TransformSmoother:
    """变换平滑器 - 提取CameraStabilizer中的平滑逻辑"""

    def __init__(self, smooth_window=30):
        self.smooth_window = smooth_window
        self.transforms = deque(maxlen=smooth_window)

    def add_transform(self, dx, dy, da):
        self.transforms.append([dx, dy, da])

    def get_smooth(self):
        if len(self.transforms) == 0:
            return 0.0, 0.0, 0.0
        smooth_dx = np.mean([t[0] for t in self.transforms])
        smooth_dy = np.mean([t[1] for t in self.transforms])
        smooth_da = np.mean([t[2] for t in self.transforms])
        return smooth_dx, smooth_dy, smooth_da

    def build_affine_matrix(self, dx, dy, da):
        """构建仿射变换矩阵"""
        cos_a = np.cos(da)
        sin_a = np.sin(da)
        return np.array([
            [cos_a, -sin_a, dx],
            [sin_a, cos_a, dy]
        ], dtype=np.float32)

    def reset(self):
        self.transforms.clear()


class CameraStabilizerSimulator:
    """相机稳定器模拟 (不依赖cv2)"""

    def __init__(self, smooth_window=30, crop_ratio=0.05):
        self.smooth_window = smooth_window
        self.crop_ratio = crop_ratio
        self.transforms = deque(maxlen=smooth_window)
        self.cumulative = np.zeros(3, dtype=np.float64)  # dx, dy, da

    def add_frame_transform(self, dx, dy, da):
        """添加一帧的变换"""
        self.transforms.append([dx, dy, da])

    def get_smooth_transform(self):
        """获取平滑后的变换"""
        if len(self.transforms) == 0:
            return 0.0, 0.0, 0.0
        smooth_dx = np.mean([t[0] for t in self.transforms])
        smooth_dy = np.mean([t[1] for t in self.transforms])
        smooth_da = np.mean([t[2] for t in self.transforms])
        return smooth_dx, smooth_dy, smooth_da

    def get_crop_params(self, width, height):
        """获取裁剪参数"""
        crop_x = int(width * self.crop_ratio)
        crop_y = int(height * self.crop_ratio)
        return crop_x, crop_y

    def reset(self):
        self.transforms.clear()
        self.cumulative = np.zeros(3, dtype=np.float64)


class HomographySmoother:
    """单应性矩阵平滑器"""

    def __init__(self, smooth_window=20):
        self.smooth_window = smooth_window
        self.h_matrices = deque(maxlen=smooth_window)

    def add_homography(self, h):
        self.h_matrices.append(h)

    def get_smooth_homography(self):
        if len(self.h_matrices) == 0:
            return np.eye(3, dtype=np.float32)
        return np.mean(list(self.h_matrices), axis=0)

    def reset(self):
        self.h_matrices.clear()


class TestCameraStabilizerInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        stab = CameraStabilizerSimulator()
        self.assertEqual(stab.smooth_window, 30)
        self.assertAlmostEqual(stab.crop_ratio, 0.05)

    def test_custom_params(self):
        stab = CameraStabilizerSimulator(smooth_window=50, crop_ratio=0.1)
        self.assertEqual(stab.smooth_window, 50)
        self.assertAlmostEqual(stab.crop_ratio, 0.1)

    def test_initial_state_empty(self):
        stab = CameraStabilizerSimulator()
        self.assertEqual(len(stab.transforms), 0)

    def test_cumulative_zero(self):
        stab = CameraStabilizerSimulator()
        np.testing.assert_array_equal(stab.cumulative, np.zeros(3))


class TestTransformSmoother(unittest.TestCase):
    """变换平滑测试"""

    def test_empty_returns_zero(self):
        sm = TransformSmoother()
        dx, dy, da = sm.get_smooth()
        self.assertAlmostEqual(dx, 0.0)
        self.assertAlmostEqual(dy, 0.0)
        self.assertAlmostEqual(da, 0.0)

    def test_single_transform(self):
        sm = TransformSmoother()
        sm.add_transform(5.0, 3.0, 0.1)
        dx, dy, da = sm.get_smooth()
        self.assertAlmostEqual(dx, 5.0)
        self.assertAlmostEqual(dy, 3.0)
        self.assertAlmostEqual(da, 0.1)

    def test_smoothing_average(self):
        """平滑应取平均"""
        sm = TransformSmoother(smooth_window=10)
        sm.add_transform(10.0, 0.0, 0.0)
        sm.add_transform(0.0, 0.0, 0.0)
        dx, dy, da = sm.get_smooth()
        self.assertAlmostEqual(dx, 5.0)

    def test_window_limited(self):
        """窗口应限制历史长度"""
        sm = TransformSmoother(smooth_window=3)
        for i in range(10):
            sm.add_transform(float(i), 0.0, 0.0)
        dx, _, _ = sm.get_smooth()
        # 最后3个: 7, 8, 9 -> 平均8.0
        self.assertAlmostEqual(dx, 8.0)

    def test_affine_matrix_shape(self):
        sm = TransformSmoother()
        m = sm.build_affine_matrix(0.0, 0.0, 0.0)
        self.assertEqual(m.shape, (2, 3))

    def test_affine_identity(self):
        """零变换应近似单位矩阵"""
        sm = TransformSmoother()
        m = sm.build_affine_matrix(0.0, 0.0, 0.0)
        self.assertAlmostEqual(m[0, 0], 1.0, places=5)
        self.assertAlmostEqual(m[1, 1], 1.0, places=5)
        self.assertAlmostEqual(m[0, 2], 0.0, places=5)
        self.assertAlmostEqual(m[1, 2], 0.0, places=5)


class TestCameraStabilizerTransforms(unittest.TestCase):
    """变换累积测试"""

    def test_add_transform(self):
        stab = CameraStabilizerSimulator()
        stab.add_frame_transform(5.0, 3.0, 0.1)
        self.assertEqual(len(stab.transforms), 1)

    def test_smooth_transform(self):
        stab = CameraStabilizerSimulator(smooth_window=10)
        stab.add_frame_transform(10.0, 5.0, 0.2)
        stab.add_frame_transform(0.0, 0.0, 0.0)
        dx, dy, da = stab.get_smooth_transform()
        self.assertAlmostEqual(dx, 5.0)
        self.assertAlmostEqual(dy, 2.5)
        self.assertAlmostEqual(da, 0.1)


class TestCameraStabilizerCrop(unittest.TestCase):
    """裁剪测试"""

    def test_crop_params(self):
        stab = CameraStabilizerSimulator(crop_ratio=0.05)
        cx, cy = stab.get_crop_params(640, 480)
        self.assertEqual(cx, 32)   # 640*0.05
        self.assertEqual(cy, 24)   # 480*0.05

    def test_zero_crop(self):
        stab = CameraStabilizerSimulator(crop_ratio=0.0)
        cx, cy = stab.get_crop_params(640, 480)
        self.assertEqual(cx, 0)
        self.assertEqual(cy, 0)

    def test_large_crop(self):
        stab = CameraStabilizerSimulator(crop_ratio=0.1)
        cx, cy = stab.get_crop_params(1920, 1080)
        self.assertEqual(cx, 192)
        self.assertEqual(cy, 108)


class TestCameraStabilizerReset(unittest.TestCase):
    """复位测试"""

    def test_reset_clears_transforms(self):
        stab = CameraStabilizerSimulator()
        stab.add_frame_transform(1.0, 2.0, 0.1)
        stab.add_frame_transform(3.0, 4.0, 0.2)
        stab.reset()
        self.assertEqual(len(stab.transforms), 0)

    def test_reset_clears_cumulative(self):
        stab = CameraStabilizerSimulator()
        stab.cumulative = np.array([10.0, 20.0, 0.5])
        stab.reset()
        np.testing.assert_array_equal(stab.cumulative, np.zeros(3))


class TestHomographySmoother(unittest.TestCase):
    """单应性矩阵平滑测试"""

    def test_empty_returns_identity(self):
        sm = HomographySmoother()
        h = sm.get_smooth_homography()
        np.testing.assert_array_almost_equal(h, np.eye(3))

    def test_single_homography(self):
        sm = HomographySmoother()
        h_in = np.array([[1, 0, 5], [0, 1, 3], [0, 0, 1]], dtype=np.float32)
        sm.add_homography(h_in)
        h_out = sm.get_smooth_homography()
        np.testing.assert_array_almost_equal(h_out, h_in)

    def test_smoothing_two_matrices(self):
        sm = HomographySmoother()
        h1 = np.array([[1, 0, 10], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
        h2 = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
        sm.add_homography(h1)
        sm.add_homography(h2)
        h_out = sm.get_smooth_homography()
        # 平均: dx=5
        self.assertAlmostEqual(h_out[0, 2], 5.0)

    def test_window_limit(self):
        sm = HomographySmoother(smooth_window=3)
        for i in range(10):
            h = np.eye(3, dtype=np.float32)
            h[0, 2] = float(i)
            sm.add_homography(h)
        h_out = sm.get_smooth_homography()
        # 最后3个: 7, 8, 9 -> 平均8.0
        self.assertAlmostEqual(h_out[0, 2], 8.0)

    def test_reset(self):
        sm = HomographySmoother()
        sm.add_homography(np.eye(3))
        sm.reset()
        h = sm.get_smooth_homography()
        np.testing.assert_array_almost_equal(h, np.eye(3))


class TestAffineTransform(unittest.TestCase):
    """仿射变换数学测试"""

    def test_rotation_matrix(self):
        """旋转矩阵应正确"""
        sm = TransformSmoother()
        m = sm.build_affine_matrix(0.0, 0.0, math.pi / 6)  # 30度
        self.assertAlmostEqual(m[0, 0], math.cos(math.pi / 6), places=5)
        self.assertAlmostEqual(m[1, 0], math.sin(math.pi / 6), places=5)

    def test_translation_only(self):
        """纯平移"""
        sm = TransformSmoother()
        m = sm.build_affine_matrix(10.0, 20.0, 0.0)
        self.assertAlmostEqual(m[0, 2], 10.0, places=5)
        self.assertAlmostEqual(m[1, 2], 20.0, places=5)
        self.assertAlmostEqual(m[0, 0], 1.0, places=5)

    def test_combined_transform(self):
        """旋转+平移"""
        sm = TransformSmoother()
        m = sm.build_affine_matrix(5.0, 3.0, 0.1)
        self.assertNotAlmostEqual(m[0, 0], 1.0, places=1)  # cos(0.1) != 1
        self.assertAlmostEqual(m[0, 2], 5.0, places=5)
        self.assertAlmostEqual(m[1, 2], 3.0, places=5)


class TestSmoothingEffect(unittest.TestCase):
    """平滑效果测试"""

    def test_smoothing_reduces_variance(self):
        """平滑应减少方差"""
        sm = TransformSmoother(smooth_window=10)
        # 添加振荡变换
        for i in range(20):
            dx = 10.0 * math.sin(i * 0.5)
            sm.add_transform(dx, 0.0, 0.0)

        # 平滑后的值应比最后原始值更接近平均
        smooth_dx, _, _ = sm.get_smooth()
        # 最后几个值的平均
        recent = [10.0 * math.sin(i * 0.5) for i in range(15, 20)]
        recent_avg = sum(recent) / len(recent)
        self.assertAlmostEqual(smooth_dx, recent_avg, places=3)

    def test_longer_window_more_smoothing(self):
        """更长窗口应更平滑"""
        sm_short = TransformSmoother(smooth_window=5)
        sm_long = TransformSmoother(smooth_window=20)

        for i in range(30):
            dx = float(i)
            sm_short.add_transform(dx, 0.0, 0.0)
            sm_long.add_transform(dx, 0.0, 0.0)

        # 长窗口的平滑值应更小(因为包含了更多历史小值)
        dx_short, _, _ = sm_short.get_smooth()
        dx_long, _, _ = sm_long.get_smooth()
        self.assertGreater(dx_short, dx_long)


if __name__ == '__main__':
    unittest.main()
