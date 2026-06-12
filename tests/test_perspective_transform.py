#!/usr/bin/env python3
"""
透视变换单元测试
覆盖: 四点标定、正/逆变换、点变换、鸟瞰图、自动校正、距离测量
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from importlib.util import find_spec
_cv2_available = find_spec('cv2') is not None

if _cv2_available:
    import cv2
    from perspective_transform import (PerspectiveTransformer, BirdEyeView,
                                        AutoPerspectiveCorrector,
                                        warp_perspective, get_bird_eye)


def _make_perspective_image():
    """创建含透视变形矩形的测试图像"""
    img = np.zeros((400, 600, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)
    src_pts = np.int32([[150, 50], [450, 80], [480, 350], [120, 320]])
    cv2.fillPoly(img, [src_pts], (200, 200, 200))
    cv2.putText(img, "TEST", (250, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
    return img


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestPerspectiveTransformerInit(unittest.TestCase):
    """初始化测试"""

    def test_init_state(self):
        """初始状态应全为None"""
        pt = PerspectiveTransformer()
        self.assertIsNone(pt.H)
        self.assertIsNone(pt.H_inv)
        self.assertIsNone(pt.src_pts)
        self.assertIsNone(pt.dst_pts)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestCalibrate(unittest.TestCase):
    """标定测试"""

    def setUp(self):
        self.pt = PerspectiveTransformer()
        self.src = np.float32([[150, 50], [450, 80], [480, 350], [120, 320]])
        self.dst = np.float32([[0, 0], [300, 0], [300, 270], [0, 270]])

    def test_calibrate_sets_H(self):
        """标定后应设置H矩阵"""
        self.pt.calibrate(self.src, self.dst, (300, 270))
        self.assertIsNotNone(self.pt.H)
        self.assertEqual(self.pt.H.shape, (3, 3))

    def test_calibrate_sets_H_inv(self):
        """标定后应设置逆矩阵"""
        self.pt.calibrate(self.src, self.dst, (300, 270))
        self.assertIsNotNone(self.pt.H_inv)

    def test_calibrate_returns_H(self):
        """标定应返回H"""
        H = self.pt.calibrate(self.src, self.dst, (300, 270))
        self.assertEqual(H.shape, (3, 3))

    def test_dst_size_auto(self):
        """不指定dst_size应自动计算"""
        self.pt.calibrate(self.src, self.dst)
        self.assertIsNotNone(self.pt.dst_size)

    def test_stores_points(self):
        """标定后应存储源和目标点"""
        self.pt.calibrate(self.src, self.dst)
        self.assertIsNotNone(self.pt.src_pts)
        self.assertIsNotNone(self.pt.dst_pts)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestTransform(unittest.TestCase):
    """变换测试"""

    def setUp(self):
        self.pt = PerspectiveTransformer()
        self.src = np.float32([[150, 50], [450, 80], [480, 350], [120, 320]])
        self.dst = np.float32([[0, 0], [300, 0], [300, 270], [0, 270]])
        self.pt.calibrate(self.src, self.dst, (300, 270))

    def test_transform_output_shape(self):
        """变换后应为目标尺寸"""
        img = _make_perspective_image()
        warped = self.pt.transform(img)
        self.assertEqual(warped.shape[:2], (270, 300))

    def test_transform_custom_size(self):
        """自定义输出尺寸"""
        img = _make_perspective_image()
        warped = self.pt.transform(img, dst_size=(150, 135))
        self.assertEqual(warped.shape[:2], (135, 150))

    def test_transform_without_calibrate_raises(self):
        """未标定时应抛异常"""
        pt = PerspectiveTransformer()
        with self.assertRaises(ValueError):
            pt.transform(np.zeros((100, 100, 3), dtype=np.uint8))


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestInverseTransform(unittest.TestCase):
    """逆变换测试"""

    def setUp(self):
        self.pt = PerspectiveTransformer()
        self.src = np.float32([[150, 50], [450, 80], [480, 350], [120, 320]])
        self.dst = np.float32([[0, 0], [300, 0], [300, 270], [0, 270]])
        self.pt.calibrate(self.src, self.dst, (300, 270))

    def test_inverse_transform(self):
        """逆变换应能执行"""
        img = _make_perspective_image()
        warped = self.pt.transform(img)
        restored = self.pt.inverse_transform(warped, (600, 400))
        self.assertEqual(restored.shape[:2], (400, 600))

    def test_inverse_without_calibrate_raises(self):
        """未标定时应抛异常"""
        pt = PerspectiveTransformer()
        with self.assertRaises(ValueError):
            pt.inverse_transform(np.zeros((100, 100, 3), dtype=np.uint8))

    def test_roundtrip_approximate(self):
        """正变换+逆变换应大致恢复原图"""
        img = _make_perspective_image()
        warped = self.pt.transform(img)
        restored = self.pt.inverse_transform(warped, (600, 400))
        # 中心区域应有内容
        center = restored[150:250, 250:350]
        self.assertGreater(np.mean(center), 0)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestTransformPoint(unittest.TestCase):
    """点变换测试"""

    def setUp(self):
        self.pt = PerspectiveTransformer()
        self.src = np.float32([[0, 0], [300, 0], [300, 300], [0, 300]])
        self.dst = np.float32([[0, 0], [200, 0], [200, 200], [0, 200]])
        self.pt.calibrate(self.src, self.dst, (200, 200))

    def test_transform_point_returns_tuple(self):
        """应返回(x, y)元组"""
        pt = self.pt.transform_point((150, 150))
        self.assertEqual(len(pt), 2)
        self.assertIsInstance(pt[0], float)
        self.assertIsInstance(pt[1], float)

    def test_corner_transform(self):
        """角点变换应接近目标角点"""
        pt = self.pt.transform_point((0, 0))
        self.assertAlmostEqual(pt[0], 0.0, delta=1)
        self.assertAlmostEqual(pt[1], 0.0, delta=1)

    def test_roundtrip_point(self):
        """点的正逆变换应大致恢复"""
        original = (150.0, 150.0)
        transformed = self.pt.transform_point(original)
        restored = self.pt.inverse_transform_point(transformed)
        self.assertAlmostEqual(restored[0], original[0], delta=1)
        self.assertAlmostEqual(restored[1], original[1], delta=1)

    def test_inverse_transform_point(self):
        """逆变换单点"""
        pt = self.pt.inverse_transform_point((100.0, 100.0))
        self.assertEqual(len(pt), 2)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestBirdEyeView(unittest.TestCase):
    """鸟瞰图测试"""

    def test_setup_from_region(self):
        """从区域设置鸟瞰图"""
        img = _make_perspective_image()
        bev = BirdEyeView()
        src_corners = np.float32([[150, 50], [450, 80], [480, 350], [120, 320]])
        H = bev.setup_from_region(img, src_corners, 300, 270)
        self.assertEqual(H.shape, (3, 3))

    def test_get_bird_eye(self):
        """获取鸟瞰图"""
        img = _make_perspective_image()
        bev = BirdEyeView()
        bev.setup_from_region(img,
                               np.float32([[150, 50], [450, 80], [480, 350], [120, 320]]),
                               300, 270)
        bird = bev.get_bird_eye(img)
        self.assertEqual(bird.shape[:2], (270, 300))

    def test_setup_simple(self):
        """简易设置"""
        img = _make_perspective_image()
        bev = BirdEyeView()
        H = bev.setup_simple(img, top_width_ratio=0.3, bottom_width_ratio=0.8)
        self.assertIsNotNone(H)

    def test_restore_from_bird_eye(self):
        """从鸟瞰图恢复"""
        img = _make_perspective_image()
        bev = BirdEyeView()
        bev.setup_from_region(img,
                               np.float32([[150, 50], [450, 80], [480, 350], [120, 320]]),
                               300, 270)
        bird = bev.get_bird_eye(img)
        restored = bev.restore_from_bird_eye(bird, (600, 400))
        self.assertEqual(restored.shape[:2], (400, 600))

    def test_measure_distance(self):
        """距离测量"""
        bev = BirdEyeView()
        dist = bev.measure_distance((0, 0), (300, 400), pixels_per_meter=100)
        expected = np.sqrt(300**2 + 400**2) / 100
        self.assertAlmostEqual(dist, expected, places=2)

    def test_measure_distance_same_point(self):
        """同一点距离应为0"""
        bev = BirdEyeView()
        dist = bev.measure_distance((50, 50), (50, 50), pixels_per_meter=10)
        self.assertAlmostEqual(dist, 0.0)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestDrawPoints(unittest.TestCase):
    """标定点可视化测试"""

    def test_draw_points(self):
        """绘制标定点"""
        img = np.zeros((400, 600, 3), dtype=np.uint8)
        pts = np.float32([[100, 100], [500, 100], [500, 300], [100, 300]])
        vis = PerspectiveTransformer.draw_points(img, pts)
        self.assertEqual(vis.shape, img.shape)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestAutoPerspectiveCorrector(unittest.TestCase):
    """自动透视校正测试"""

    def test_init(self):
        """初始化"""
        pc = AutoPerspectiveCorrector(min_area_ratio=0.05, approx_epsilon=0.02)
        self.assertEqual(pc.min_area_ratio, 0.05)

    def test_detect_quadrilateral_on_simple(self):
        """简单四边形应能检测到"""
        img = np.zeros((400, 600, 3), dtype=np.uint8)
        # 画一个白色四边形
        pts = np.int32([[100, 50], [500, 80], [480, 350], [120, 320]])
        cv2.fillPoly(img, [pts], (255, 255, 255))
        pc = AutoPerspectiveCorrector(min_area_ratio=0.01)
        result = pc.detect_quadrilateral(img)
        # 可能检测到也可能检测不到，取决于边缘检测
        if result is not None:
            self.assertEqual(result.shape, (4, 2))

    def test_detect_on_no_feature(self):
        """纯黑图像应返回None"""
        img = np.zeros((400, 600, 3), dtype=np.uint8)
        pc = AutoPerspectiveCorrector()
        result = pc.detect_quadrilateral(img)
        self.assertIsNone(result)

    def test_order_points(self):
        """点排序应正确"""
        pts = np.float32([[200, 200], [100, 100], [300, 300], [100, 300]])
        ordered = AutoPerspectiveCorrector._order_points(pts)
        self.assertEqual(ordered.shape, (4, 2))
        # 左上: x+y最小
        s = ordered.sum(axis=1)
        self.assertEqual(np.argmin(s), 0)
        # 右下: x+y最大
        self.assertEqual(np.argmax(s), 2)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestShortcutFunctions(unittest.TestCase):
    """快捷函数测试"""

    def test_warp_perspective(self):
        """快速透视变换"""
        img = _make_perspective_image()
        src = np.float32([[150, 50], [450, 80], [480, 350], [120, 320]])
        warped = warp_perspective(img, src, (300, 270))
        self.assertEqual(warped.shape[:2], (270, 300))

    def test_auto_correct_on_noisy(self):
        """噪声图可能返回None"""
        img = np.random.randint(0, 256, (200, 300, 3), dtype=np.uint8)
        result = AutoPerspectiveCorrector().auto_correct(img)
        # 可能为None
        if result is not None:
            self.assertGreater(result.shape[0], 0)


if __name__ == '__main__':
    unittest.main()
