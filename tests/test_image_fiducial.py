#!/usr/bin/env python3
"""
基准标记检测单元测试
覆盖: ArUco检测、AprilTag检测、统一接口、标记查找、角点排序、
      中心获取、距离计算、标记生成、标记板生成、姿态工具、绘制
测试对象: 10_视觉通用代码库/image_fiducial.py
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
    from _10_视觉通用代码库.image_fiducial import (
        detect_aruco,
        detect_apriltag,
        detect_fiducials,
        find_marker_by_id,
        get_marker_corners_ordered,
        get_marker_center,
        marker_distance,
        generate_aruco_marker,
        generate_aruco_board,
        create_camera_matrix,
        get_pose_euler,
        draw_fiducials,
        _ARUCO_DICTS,
    )


# ── ArUco字典测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestArUcoDicts(unittest.TestCase):
    """ArUco字典注册测试"""

    def test_contains_common_dicts(self):
        for name in ['DICT_4X4_50', 'DICT_5X5_50', 'DICT_6X6_50', 'DICT_7X7_50']:
            self.assertIn(name, _ARUCO_DICTS)

    def test_dict_count(self):
        self.assertGreaterEqual(len(_ARUCO_DICTS), 10)


# ── ArUco标记生成测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGenerateArucoMarker(unittest.TestCase):
    """ArUco标记生成测试"""

    def test_returns_ndarray(self):
        marker = generate_aruco_marker('DICT_4X4_50', marker_id=0, size=200)
        self.assertIsInstance(marker, np.ndarray)

    def test_output_size(self):
        marker = generate_aruco_marker('DICT_4X4_50', marker_id=0, size=300)
        self.assertEqual(marker.shape, (300, 300))

    def test_grayscale_output(self):
        marker = generate_aruco_marker('DICT_4X4_50', marker_id=0, size=200)
        self.assertEqual(len(marker.shape), 2)

    def test_different_ids_different_images(self):
        """不同ID应生成不同图像"""
        m1 = generate_aruco_marker('DICT_4X4_50', marker_id=0, size=200)
        m2 = generate_aruco_marker('DICT_4X4_50', marker_id=1, size=200)
        self.assertFalse(np.array_equal(m1, m2))


# ── ArUco标记板生成测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGenerateArucoBoard(unittest.TestCase):
    """ArUco标记板生成测试"""

    def test_returns_ndarray(self):
        board = generate_aruco_board('DICT_4X4_50', cols=3, rows=2, marker_size=100, spacing=20)
        self.assertIsInstance(board, np.ndarray)

    def test_color_output(self):
        """标记板应为BGR图像"""
        board = generate_aruco_board('DICT_4X4_50', cols=3, rows=2, marker_size=100, spacing=20)
        self.assertEqual(len(board.shape), 3)
        self.assertEqual(board.shape[2], 3)

    def test_output_dimensions(self):
        cols, rows, ms, sp = 4, 3, 100, 20
        expected_w = cols * ms + (cols + 1) * sp
        expected_h = rows * ms + (rows + 1) * sp
        board = generate_aruco_board('DICT_4X4_50', cols=cols, rows=rows,
                                     marker_size=ms, spacing=sp)
        self.assertEqual(board.shape[1], expected_w)
        self.assertEqual(board.shape[0], expected_h)


# ── ArUco检测测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDetectAruco(unittest.TestCase):
    """ArUco检测测试"""

    def test_returns_list(self):
        img = np.ones((400, 400), dtype=np.uint8) * 255
        results = detect_aruco(img, dict_name='DICT_4X4_50')
        self.assertIsInstance(results, list)

    def test_empty_image_no_markers(self):
        """空白图像不应检测到标记"""
        img = np.ones((400, 400), dtype=np.uint8) * 255
        results = detect_aruco(img, dict_name='DICT_4X4_50')
        self.assertEqual(len(results), 0)

    def test_generated_board_detected(self):
        """生成的标记板应能检测到标记"""
        board = generate_aruco_board('DICT_4X4_50', cols=4, rows=3,
                                     marker_size=100, spacing=30)
        results = detect_aruco(board, dict_name='DICT_4X4_50')
        self.assertGreater(len(results), 0)

    def test_result_structure(self):
        """结果应包含必要字段"""
        board = generate_aruco_board('DICT_4X4_50', cols=2, rows=2,
                                     marker_size=100, spacing=30)
        results = detect_aruco(board, dict_name='DICT_4X4_50')
        for r in results:
            self.assertIn('id', r)
            self.assertIn('corners', r)
            self.assertIn('center', r)
            self.assertIn('rvec', r)
            self.assertIn('tvec', r)

    def test_marker_ids_unique(self):
        """检测到的标记ID应唯一"""
        board = generate_aruco_board('DICT_4X4_50', cols=3, rows=2,
                                     marker_size=100, spacing=30)
        results = detect_aruco(board, dict_name='DICT_4X4_50')
        ids = [r['id'] for r in results]
        self.assertEqual(len(ids), len(set(ids)))


# ── 统一接口测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDetectFiducials(unittest.TestCase):
    """统一接口测试"""

    def test_aruco_type(self):
        board = generate_aruco_board('DICT_4X4_50', cols=2, rows=2,
                                     marker_size=100, spacing=30)
        results = detect_fiducials(board, marker_type='aruco', dict_name='DICT_4X4_50')
        self.assertIsInstance(results, list)

    def test_invalid_type_raises(self):
        img = np.ones((400, 400), dtype=np.uint8) * 255
        with self.assertRaises(ValueError):
            detect_fiducials(img, marker_type='invalid')

    def test_string_path_input(self):
        """字符串路径输入应尝试读取图像"""
        with self.assertRaises(Exception):
            detect_fiducials('nonexistent_image.png', marker_type='aruco')


# ── 标记查找测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFindMarkerById(unittest.TestCase):
    """按ID查找标记测试"""

    def test_found(self):
        markers = [
            {'id': 0, 'center': (100, 100)},
            {'id': 1, 'center': (200, 200)},
            {'id': 2, 'center': (300, 300)},
        ]
        result = find_marker_by_id(markers, 1)
        self.assertIsNotNone(result)
        self.assertEqual(result['center'], (200, 200))

    def test_not_found(self):
        markers = [{'id': 0, 'center': (100, 100)}]
        result = find_marker_by_id(markers, 99)
        self.assertIsNone(result)

    def test_empty_list(self):
        result = find_marker_by_id([], 0)
        self.assertIsNone(result)


# ── 角点排序测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGetMarkerCornersOrdered(unittest.TestCase):
    """角点排序测试"""

    def test_returns_4x2(self):
        marker = {
            'corners': [[100, 100], [200, 100], [200, 200], [100, 200]]
        }
        result = get_marker_corners_ordered(marker)
        self.assertEqual(result.shape, (4, 2))

    def test_ordering(self):
        """左上、右上、右下、左下"""
        marker = {
            'corners': [[200, 200], [100, 100], [200, 100], [100, 200]]
        }
        result = get_marker_corners_ordered(marker)
        # 左上 x+y最小
        self.assertAlmostEqual(result[0][0], 100, delta=1)
        self.assertAlmostEqual(result[0][1], 100, delta=1)


# ── 中心获取测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGetMarkerCenter(unittest.TestCase):
    """标记中心获取测试"""

    def test_returns_center(self):
        marker = {'center': (150, 200)}
        self.assertEqual(get_marker_center(marker), (150, 200))

    def test_no_center(self):
        marker = {'id': 0}
        self.assertIsNone(get_marker_center(marker))


# ── 距离计算测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMarkerDistance(unittest.TestCase):
    """标记距离计算测试"""

    def test_known_distance(self):
        m1 = {'center': (0, 0)}
        m2 = {'center': (3, 4)}
        dist = marker_distance(m1, m2)
        self.assertAlmostEqual(dist, 5.0, delta=0.01)

    def test_same_point_zero(self):
        m = {'center': (100, 100)}
        self.assertAlmostEqual(marker_distance(m, m), 0.0)


# ── 相机矩阵测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestCreateCameraMatrix(unittest.TestCase):
    """相机矩阵构造测试"""

    def test_shape(self):
        K = create_camera_matrix(500, 500, 320, 240)
        self.assertEqual(K.shape, (3, 3))

    def test_diagonal_values(self):
        K = create_camera_matrix(500, 600, 320, 240)
        self.assertAlmostEqual(K[0, 0], 500.0)
        self.assertAlmostEqual(K[1, 1], 600.0)
        self.assertAlmostEqual(K[0, 2], 320.0)
        self.assertAlmostEqual(K[1, 2], 240.0)

    def test_last_row(self):
        K = create_camera_matrix(500, 500, 320, 240)
        self.assertAlmostEqual(K[2, 0], 0.0)
        self.assertAlmostEqual(K[2, 1], 0.0)
        self.assertAlmostEqual(K[2, 2], 1.0)


# ── 欧拉角测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGetPoseEuler(unittest.TestCase):
    """旋转向量转欧拉角测试"""

    def test_returns_array(self):
        rvec = [0.0, 0.0, 0.0]
        euler = get_pose_euler(rvec)
        self.assertEqual(len(euler), 3)

    def test_zero_rotation(self):
        """零旋转应返回接近零的角度"""
        rvec = [0.0, 0.0, 0.0]
        euler = get_pose_euler(rvec)
        for angle in euler:
            self.assertAlmostEqual(angle, 0.0, delta=1.0)


# ── 绘制测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawFiducials(unittest.TestCase):
    """绘制基准标记测试"""

    def test_output_shape(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        markers = [{
            'id': 0,
            'corners': [[100, 100], [200, 100], [200, 200], [100, 200]],
            'center': (150, 150),
        }]
        vis = draw_fiducials(img, markers)
        self.assertEqual(vis.shape, img.shape)

    def test_empty_markers(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        vis = draw_fiducials(img, [])
        self.assertEqual(vis.shape, img.shape)

    def test_does_not_modify_input(self):
        img = np.zeros((400, 400, 3), dtype=np.uint8)
        img_copy = img.copy()
        markers = [{
            'id': 0,
            'corners': [[100, 100], [200, 100], [200, 200], [100, 200]],
            'center': (150, 150),
        }]
        draw_fiducials(img, markers)
        np.testing.assert_array_equal(img, img_copy)


if __name__ == '__main__':
    unittest.main()
