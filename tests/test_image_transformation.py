"""
图像变换单元测试
覆盖: 仿射变换/仿射旋转/仿射缩放/仿射剪切/透视变换/透视校正/
      极坐标变换/对数极坐标变换/极坐标逆变换/对数变换/伽马变换
"""
import unittest
import numpy as np
import cv2
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from importlib import import_module

mod = import_module('10_视觉通用代码库.image_transformation')


class TestAffineTransform(unittest.TestCase):
    """仿射变换测试"""

    def setUp(self):
        self.img = np.random.randint(0, 256, (100, 150, 3), dtype=np.uint8)

    def test_affine_transform_output_shape(self):
        """仿射变换保持输出尺寸"""
        src = np.float32([[0, 0], [100, 0], [0, 100]])
        dst = np.float32([[10, 10], [110, 5], [5, 110]])
        result = mod.affine_transform(self.img, src, dst)
        self.assertEqual(result.shape, self.img.shape)

    def test_affine_transform_custom_size(self):
        """仿射变换自定义输出尺寸"""
        src = np.float32([[0, 0], [100, 0], [0, 100]])
        dst = np.float32([[10, 10], [110, 5], [5, 110]])
        result = mod.affine_transform(self.img, src, dst, size=(200, 80))
        self.assertEqual(result.shape[:2], (80, 200))

    def test_affine_identity(self):
        """单位仿射变换应保持图像基本不变"""
        src = np.float32([[0, 0], [150, 0], [0, 100]])
        dst = np.float32([[0, 0], [150, 0], [0, 100]])
        result = mod.affine_transform(self.img, src, dst)
        self.assertEqual(result.shape, self.img.shape)


class TestAffineRotate(unittest.TestCase):
    """仿射旋转测试"""

    def setUp(self):
        self.img = np.random.randint(0, 256, (100, 100, 3), dtype=np.uint8)

    def test_rotate_output_shape(self):
        """旋转后尺寸不变"""
        result = mod.affine_rotate(self.img, 45)
        self.assertEqual(result.shape, self.img.shape)

    def test_rotate_zero_degrees(self):
        """旋转0度应基本不变"""
        result = mod.affine_rotate(self.img, 0)
        np.testing.assert_array_equal(result, self.img)

    def test_rotate_custom_center(self):
        """自定义旋转中心"""
        result = mod.affine_rotate(self.img, 30, center=(50, 50))
        self.assertEqual(result.shape, self.img.shape)

    def test_rotate_with_scale(self):
        """旋转+缩放"""
        result = mod.affine_rotate(self.img, 45, scale=0.5)
        self.assertEqual(result.shape, self.img.shape)


class TestAffineScale(unittest.TestCase):
    """仿射缩放测试"""

    def test_scale_up(self):
        img = np.zeros((50, 60, 3), dtype=np.uint8)
        result = mod.affine_scale(img, 2.0)
        self.assertEqual(result.shape[:2], (100, 120))

    def test_scale_down(self):
        img = np.zeros((100, 120, 3), dtype=np.uint8)
        result = mod.affine_scale(img, 0.5)
        self.assertEqual(result.shape[:2], (50, 60))

    def test_scale_asymmetric(self):
        img = np.zeros((80, 60, 3), dtype=np.uint8)
        result = mod.affine_scale(img, 2.0, 0.5)
        self.assertEqual(result.shape[:2], (40, 120))


class TestAffineShear(unittest.TestCase):
    """仿射剪切测试"""

    def test_shear_zero(self):
        img = np.random.randint(0, 256, (50, 60, 3), dtype=np.uint8)
        result = mod.affine_shear(img, 0, 0)
        self.assertEqual(result.shape[:2], (50, 60))

    def test_shear_expands_image(self):
        img = np.random.randint(0, 256, (50, 60, 3), dtype=np.uint8)
        result = mod.affine_shear(img, shear_x=0.5)
        self.assertGreaterEqual(result.shape[1], img.shape[1])


class TestPerspectiveTransform(unittest.TestCase):
    """透视变换测试"""

    def test_perspective_output_shape(self):
        img = np.random.randint(0, 256, (100, 150, 3), dtype=np.uint8)
        src = np.float32([[0, 0], [150, 0], [150, 100], [0, 100]])
        dst = np.float32([[10, 20], [140, 10], [160, 110], [5, 90]])
        result = mod.perspective_transform(img, src, dst)
        self.assertEqual(result.shape, img.shape)

    def test_perspective_custom_size(self):
        img = np.random.randint(0, 256, (100, 150, 3), dtype=np.uint8)
        src = np.float32([[0, 0], [150, 0], [150, 100], [0, 100]])
        dst = np.float32([[0, 0], [200, 0], [200, 150], [0, 150]])
        result = mod.perspective_transform(img, src, dst, size=(200, 150))
        self.assertEqual(result.shape[:2], (150, 200))


class TestPerspectiveCorrect(unittest.TestCase):
    """透视校正测试"""

    def test_correct_rectangle(self):
        img = np.random.randint(0, 256, (200, 300, 3), dtype=np.uint8)
        pts = np.float32([[10, 10], [290, 10], [290, 190], [10, 190]])
        result = mod.perspective_correct(img, pts)
        self.assertGreater(result.shape[0], 0)
        self.assertGreater(result.shape[1], 0)


class TestPolarTransform(unittest.TestCase):
    """极坐标变换测试"""

    def test_polar_output_shape(self):
        img = np.random.randint(0, 256, (100, 100, 3), dtype=np.uint8)
        result = mod.polar_transform(img)
        self.assertGreater(result.shape[0], 0)
        self.assertGreater(result.shape[1], 0)

    def test_log_polar_output_shape(self):
        img = np.random.randint(0, 256, (100, 100, 3), dtype=np.uint8)
        result = mod.log_polar_transform(img)
        self.assertGreater(result.shape[0], 0)

    def test_inverse_polar(self):
        img = np.random.randint(0, 256, (100, 100), dtype=np.uint8)
        center = (50.0, 50.0)
        max_radius = 50.0
        polar = mod.polar_transform(img, center=center, max_radius=max_radius)
        result = mod.inverse_polar(polar, center, max_radius,
                                   (img.shape[1], img.shape[0]))
        self.assertEqual(result.shape, img.shape)


class TestLogTransform(unittest.TestCase):
    """对数变换测试"""

    def test_log_transform_output_type(self):
        img = np.random.randint(0, 256, (50, 60), dtype=np.uint8)
        result = mod.log_transform(img)
        self.assertEqual(result.dtype, np.uint8)

    def test_log_transform_output_shape(self):
        img = np.random.randint(1, 256, (50, 60), dtype=np.uint8)
        result = mod.log_transform(img)
        self.assertEqual(result.shape, img.shape)

    def test_log_transform_with_constant(self):
        img = np.random.randint(0, 256, (50, 60), dtype=np.uint8)
        result = mod.log_transform(img, c=100)
        self.assertEqual(result.dtype, np.uint8)


class TestGammaTransform(unittest.TestCase):
    """伽马变换测试"""

    def test_gamma_1_no_change(self):
        """gamma=1不应改变图像"""
        img = np.random.randint(0, 256, (50, 60), dtype=np.uint8)
        result = mod.gamma_transform(img, gamma=1.0)
        np.testing.assert_array_equal(result, img)

    def test_gamma_lt1_brightens(self):
        """gamma<1应提亮"""
        img = np.full((50, 60), 100, dtype=np.uint8)
        result = mod.gamma_transform(img, gamma=0.5)
        self.assertGreater(result[0, 0], 100)

    def test_gamma_gt1_darkens(self):
        """gamma>1应压暗"""
        img = np.full((50, 60), 200, dtype=np.uint8)
        result = mod.gamma_transform(img, gamma=2.0)
        self.assertLess(result[0, 0], 200)

    def test_gamma_output_type(self):
        img = np.random.randint(0, 256, (50, 60, 3), dtype=np.uint8)
        result = mod.gamma_transform(img, gamma=1.5)
        self.assertEqual(result.dtype, np.uint8)
        self.assertEqual(result.shape, img.shape)


if __name__ == '__main__':
    unittest.main()
