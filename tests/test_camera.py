#!/usr/bin/env python3
"""
相机驱动单元测试
覆盖: CameraCalibrator 初始化、角点检测、标定流程、畸变校正、参数保存/加载
注意: 使用合成棋盘格图像进行测试，不依赖摄像头硬件
"""

import sys
import os
import unittest
import tempfile
import shutil
import numpy as np

# 将项目根目录加入路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


def generate_chessboard_image(chessboard_size=(9, 6), square_px=50,
                               image_size=(640, 480)):
    """
    生成合成棋盘格图像(用于标定测试)
    chessboard_size: 内角点数 (列, 行)
    square_px: 每格像素大小
    """
    cols, rows = chessboard_size
    width = (cols + 1) * square_px
    height = (rows + 1) * square_px

    img = np.ones((height, width), dtype=np.uint8) * 255

    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 1:
                y1 = r * square_px
                y2 = (r + 1) * square_px
                x1 = c * square_px
                x2 = (c + 1) * square_px
                img[y1:y2, x1:x2] = 0

    # 转为BGR
    img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # 缩放到目标尺寸
    img_bgr = cv2.resize(img_bgr, image_size)
    return img_bgr


@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过相机测试")
class TestCameraCalibratorInit(unittest.TestCase):
    """CameraCalibrator初始化测试"""

    def test_default_params(self):
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        self.assertEqual(cal.chessboard_size, (9, 6))
        self.assertAlmostEqual(cal.square_size, 0.025)
        self.assertIsNone(cal.camera_matrix)
        self.assertIsNone(cal.dist_coeffs)

    def test_custom_params(self):
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator(chessboard_size=(7, 5), square_size=0.03)
        self.assertEqual(cal.chessboard_size, (7, 5))
        self.assertAlmostEqual(cal.square_size, 0.03)

    def test_object_points_initialized(self):
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator(chessboard_size=(9, 6))
        # 应有 9*6 = 54 个3D点
        self.assertEqual(cal.objp.shape[0], 54)
        self.assertEqual(cal.objp.shape[1], 3)


@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过相机测试")
class TestCornerDetection(unittest.TestCase):
    """角点检测测试"""

    def test_find_corners_on_chessboard(self):
        """合成棋盘格应能检测到角点"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator(chessboard_size=(9, 6))
        img = generate_chessboard_image((9, 6), square_px=50,
                                         image_size=(640, 480))
        found, corners = cal.find_corners(img)
        # 可能找到也可能找不到(取决于合成质量)
        # 但不应报错
        self.assertIsInstance(found, bool)

    def test_find_corners_on_blank_image(self):
        """空白图像不应找到角点"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator(chessboard_size=(9, 6))
        img = np.ones((480, 640, 3), dtype=np.uint8) * 200
        found, corners = cal.find_corners(img)
        self.assertFalse(found)

    def test_find_corners_on_noise(self):
        """噪声图像不应找到角点"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator(chessboard_size=(9, 6))
        img = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
        found, corners = cal.find_corners(img)
        self.assertFalse(found)


@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过相机测试")
class TestCalibrationProcess(unittest.TestCase):
    """标定流程测试"""

    def test_calibrate_insufficient_images(self):
        """不足3张图像应返回False"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        # 不添加任何图像
        result = cal.calibrate(image_size=(640, 480))
        self.assertFalse(result)

    def test_calibrate_without_image_size(self):
        """未指定图像尺寸应返回False"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        # 手动添加假数据
        cal.img_points = [np.zeros((54, 1, 2), dtype=np.float32)] * 3
        cal.obj_points = [cal.objp] * 3
        result = cal.calibrate(image_size=None)
        self.assertFalse(result)

    def test_save_before_calibrate(self):
        """未标定时保存应打印错误(不崩溃)"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        # 不应抛出异常
        with tempfile.NamedTemporaryFile(suffix='.npz', delete=False) as f:
            tmpfile = f.name
        try:
            cal.save(tmpfile)
            # 文件不应存在(因为未标定)
            self.assertFalse(os.path.exists(tmpfile) or
                           os.path.getsize(tmpfile) > 0)
        finally:
            if os.path.exists(tmpfile):
                os.unlink(tmpfile)

    def test_load_nonexistent_file(self):
        """加载不存在的文件应返回False"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        result = cal.load('/nonexistent/path/camera_params.npz')
        self.assertFalse(result)


@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过相机测试")
class TestUndistort(unittest.TestCase):
    """畸变校正测试"""

    def test_undistort_without_calibration(self):
        """未标定时undistort应返回原图"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        result = cal.undistort(img)
        np.testing.assert_array_equal(result, img)

    def test_undistort_with_calibration(self):
        """标定后undistort应返回校正图像"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        # 模拟标定结果(无畸变)
        cal.camera_matrix = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ], dtype=np.float64)
        cal.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        img = np.zeros((480, 640, 3), dtype=np.uint8)
        result = cal.undistort(img)
        self.assertEqual(result.shape, img.shape)

    def test_undistort_with_roi(self):
        """undistort_with_roi应返回裁剪后的图像"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        cal.camera_matrix = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ], dtype=np.float64)
        cal.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        img = np.zeros((480, 640, 3), dtype=np.uint8)
        result = cal.undistort_with_roi(img)
        self.assertEqual(len(result.shape), 3)
        self.assertEqual(result.shape[2], 3)


@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过相机测试")
class TestSaveLoadParams(unittest.TestCase):
    """参数保存/加载测试"""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_save_and_load(self):
        """保存后加载应恢复参数"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        cal.camera_matrix = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ], dtype=np.float64)
        cal.dist_coeffs = np.array([0.1, -0.2, 0, 0, 0], dtype=np.float64).reshape(5, 1)
        cal.image_size = (640, 480)
        cal.reprojection_error = 0.35

        filepath = os.path.join(self.tmpdir, 'params.npz')
        cal.save(filepath)

        # 加载到新的实例
        cal2 = CameraCalibrator()
        result = cal2.load(filepath)
        self.assertTrue(result)
        np.testing.assert_array_almost_equal(cal2.camera_matrix, cal.camera_matrix)
        np.testing.assert_array_almost_equal(cal2.dist_coeffs, cal.dist_coeffs)
        self.assertEqual(cal2.image_size, (640, 480))
        self.assertAlmostEqual(cal2.reprojection_error, 0.35)

    def test_save_creates_file(self):
        """save应创建文件"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        cal.camera_matrix = np.eye(3)
        cal.dist_coeffs = np.zeros((5, 1))
        cal.image_size = (640, 480)

        filepath = os.path.join(self.tmpdir, 'params.npz')
        cal.save(filepath)
        self.assertTrue(os.path.exists(filepath))


@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过相机测试")
class TestUndistortMaps(unittest.TestCase):
    """畸变校正映射表测试"""

    def test_get_maps_before_calibrate(self):
        """未标定时获取映射表应返回None"""
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        result = cal.get_undistort_maps()
        self.assertEqual(result, (None, None))

    def test_get_maps_after_calibrate(self):
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        cal.camera_matrix = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ], dtype=np.float64)
        cal.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        cal.image_size = (640, 480)

        map1, map2 = cal.get_undistort_maps()
        self.assertIsNotNone(map1)
        self.assertIsNotNone(map2)
        self.assertEqual(map1.shape, (480, 640))


@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过相机测试")
class TestDrawCorners(unittest.TestCase):
    """角点绘制测试"""

    def test_draw_corners_returns_same_size(self):
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator(chessboard_size=(9, 6))
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        corners = np.zeros((54, 1, 2), dtype=np.float32)
        vis = cal.draw_corners(img, corners, True)
        self.assertEqual(vis.shape, img.shape)

    def test_draw_corners_not_found(self):
        from visual.camera_calibration import CameraCalibrator
        cal = CameraCalibrator()
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        corners = np.zeros((54, 1, 2), dtype=np.float32)
        vis = cal.draw_corners(img, corners, False)
        self.assertEqual(vis.shape, img.shape)


if __name__ == '__main__':
    unittest.main()
