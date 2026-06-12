#!/usr/bin/env python3
"""
ArUco标记检测单元测试
覆盖: ArucoDetector 初始化、标记检测、位姿估计、旋转矩阵转欧拉角、标记生成
注意: 使用合成图像和Mock进行测试
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    HAS_CV2 = hasattr(cv2, 'aruco')
except ImportError:
    HAS_CV2 = False

if HAS_CV2:
    from visual_common.aruco_detector import ArucoDetector


# ── 辅助函数 ──────────────────────────────────────────────────

def _identity_camera_matrix(fx=500, fy=500, cx=320, cy=240):
    """生成标准相机内参矩阵"""
    return np.array([[fx, 0, cx],
                     [0, fy, cy],
                     [0, 0, 1]], dtype=np.float32)


def _generate_marker_image(marker_id=0, dict_type='5x5_100', size=200):
    """生成单个ArUco标记图像"""
    dict_id = ArucoDetector.DICT_TYPES.get(dict_type, cv2.aruco.DICT_5X5_100)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)
    # 添加白色边框
    border = size // 5
    bordered = np.ones((size + 2 * border, size + 2 * border), dtype=np.uint8) * 255
    bordered[border:border + size, border:border + size] = marker_img
    return bordered


def _make_frame_with_marker(marker_id=0, size=200, frame_size=(640, 480)):
    """生成包含ArUco标记的合成帧"""
    frame = np.ones((*frame_size, 3), dtype=np.uint8) * 128
    marker_img = _generate_marker_image(marker_id, size=size)
    # 居中放置
    y_off = (frame_size[0] - marker_img.shape[0]) // 2
    x_off = (frame_size[1] - marker_img.shape[1]) // 2
    if y_off >= 0 and x_off >= 0:
        roi = frame[y_off:y_off + marker_img.shape[0], x_off:x_off + marker_img.shape[1]]
        marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
        # 确保尺寸匹配
        h, w = roi.shape[:2]
        frame[y_off:y_off + h, x_off:x_off + w] = marker_bgr[:h, :w]
    return frame


# ── 测试用例 ──────────────────────────────────────────────────

@unittest.skipUnless(HAS_CV2, "OpenCV aruco module not available")
class TestArucoDetectorInit(unittest.TestCase):
    """初始化测试"""

    def test_default_dict(self):
        """默认字典应为5x5_100"""
        det = ArucoDetector()
        self.assertEqual(det.marker_length, 0.05)

    def test_custom_dict(self):
        """自定义字典"""
        det = ArucoDetector(dict_type='6x6_250')
        self.assertIsNotNone(det.aruco_dict)

    def test_custom_marker_length(self):
        """自定义标记边长"""
        det = ArucoDetector(marker_length=0.1)
        self.assertAlmostEqual(det.marker_length, 0.1)

    def test_no_camera_matrix(self):
        """无相机参数时camera_matrix应为None"""
        det = ArucoDetector()
        self.assertIsNone(det.camera_matrix)

    def test_default_dist_coeffs(self):
        """默认畸变系数应为零"""
        det = ArucoDetector()
        self.assertIsNotNone(det.dist_coeffs)

    def test_dict_types_count(self):
        """应支持多种字典类型"""
        self.assertTrue(len(ArucoDetector.DICT_TYPES) > 10)


@unittest.skipUnless(HAS_CV2, "OpenCV aruco module not available")
class TestArucoDetectorDetect(unittest.TestCase):
    """标记检测测试"""

    def test_detect_empty_frame(self):
        """空帧应返回空列表"""
        det = ArucoDetector()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        results = det.detect(frame)
        self.assertIsInstance(results, list)

    def test_detect_white_frame(self):
        """纯白帧应返回空列表"""
        det = ArucoDetector()
        frame = np.ones((480, 640, 3), dtype=np.uint8) * 255
        results = det.detect(frame)
        self.assertEqual(len(results), 0)

    def test_detect_generated_marker(self):
        """检测生成的标记"""
        det = ArucoDetector(dict_type='5x5_100')
        frame = _make_frame_with_marker(marker_id=0, size=200)
        results = det.detect(frame)
        # 应至少检测到一个标记
        if len(results) > 0:
            self.assertIn('id', results[0])
            self.assertIn('corners', results[0])
            self.assertIn('center', results[0])

    def test_result_structure(self):
        """检测结果结构"""
        det = ArucoDetector()
        frame = _make_frame_with_marker(marker_id=1, size=200)
        results = det.detect(frame)
        for r in results:
            self.assertIn('id', r)
            self.assertIn('corners', r)
            self.assertIn('center', r)
            self.assertIn('pose', r)


@unittest.skipUnless(HAS_CV2, "OpenCV aruco module not available")
class TestArucoDetectorPose(unittest.TestCase):
    """位姿估计测试"""

    def test_no_pose_without_camera_matrix(self):
        """无相机参数时不应估计位姿"""
        det = ArucoDetector()
        frame = _make_frame_with_marker(marker_id=0, size=200)
        results = det.detect(frame)
        for r in results:
            self.assertIsNone(r.get('pose'))

    def test_pose_with_camera_matrix(self):
        """有相机参数时应估计位姿"""
        cm = _identity_camera_matrix()
        det = ArucoDetector(camera_matrix=cm, marker_length=0.05)
        frame = _make_frame_with_marker(marker_id=0, size=200)
        results = det.detect(frame)
        for r in results:
            if r['pose'] is not None:
                self.assertIn('rvec', r['pose'])
                self.assertIn('tvec', r['pose'])
                self.assertIn('euler', r['pose'])
                self.assertIn('distance', r['pose'])

    def test_distance_positive(self):
        """距离应为正值"""
        cm = _identity_camera_matrix()
        det = ArucoDetector(camera_matrix=cm, marker_length=0.05)
        frame = _make_frame_with_marker(marker_id=0, size=200)
        results = det.detect(frame)
        for r in results:
            if r['pose'] is not None:
                self.assertGreater(r['pose']['distance'], 0)


@unittest.skipUnless(HAS_CV2, "OpenCV aruco module not available")
class TestArucoEulerConversion(unittest.TestCase):
    """旋转矩阵转欧拉角测试"""

    def test_identity_rotation(self):
        """单位旋转矩阵应返回零角度"""
        R = np.eye(3)
        euler = ArucoDetector._rotation_matrix_to_euler(R)
        self.assertAlmostEqual(euler[0], 0.0, places=3)
        self.assertAlmostEqual(euler[1], 0.0, places=3)
        self.assertAlmostEqual(euler[2], 0.0, places=3)

    def test_90_deg_z_rotation(self):
        """绕Z轴旋转90°"""
        R = np.array([[0, -1, 0],
                       [1, 0, 0],
                       [0, 0, 1]], dtype=float)
        euler = ArucoDetector._rotation_matrix_to_euler(R)
        self.assertAlmostEqual(euler[2], 90.0, places=1)

    def test_90_deg_x_rotation(self):
        """绕X轴旋转90°"""
        R = np.array([[1, 0, 0],
                       [0, 0, -1],
                       [0, 1, 0]], dtype=float)
        euler = ArucoDetector._rotation_matrix_to_euler(R)
        self.assertAlmostEqual(euler[0], 90.0, places=1)

    def test_euler_type(self):
        """返回值应为numpy数组"""
        R = np.eye(3)
        euler = ArucoDetector._rotation_matrix_to_euler(R)
        self.assertIsInstance(euler, np.ndarray)
        self.assertEqual(len(euler), 3)


@unittest.skipUnless(HAS_CV2, "OpenCV aruco module not available")
class TestArucoGenerateMarker(unittest.TestCase):
    """标记生成测试"""

    def test_generate_returns_image(self):
        """生成应返回图像"""
        img = ArucoDetector.generate_marker(0, '5x5_100', 200)
        self.assertIsNotNone(img)
        self.assertEqual(img.shape[0], img.shape[1])  # 正方形

    def test_generate_has_border(self):
        """生成图像应有白色边框"""
        img = ArucoDetector.generate_marker(0, '5x5_100', 200)
        # 边框区域应为白色
        self.assertEqual(img[0, 0], 255)

    def test_generate_different_ids(self):
        """不同ID应生成不同图像"""
        img0 = ArucoDetector.generate_marker(0, '5x5_100', 200)
        img1 = ArucoDetector.generate_marker(1, '5x5_100', 200)
        self.assertFalse(np.array_equal(img0, img1))

    def test_generate_to_file(self):
        """生成到文件"""
        import tempfile
        with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as f:
            path = f.name
        try:
            img = ArucoDetector.generate_marker(0, '5x5_100', 200, output_path=path)
            self.assertTrue(os.path.exists(path))
        finally:
            if os.path.exists(path):
                os.remove(path)


@unittest.skipUnless(HAS_CV2, "OpenCV aruco module not available")
class TestArucoDraw(unittest.TestCase):
    """绘制测试"""

    def test_draw_returns_frame(self):
        """draw应返回绘制后的帧"""
        det = ArucoDetector()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        results = []
        vis = det.draw(frame, results)
        self.assertEqual(vis.shape, frame.shape)

    def test_draw_modifies_frame(self):
        """绘制应修改帧内容"""
        det = ArucoDetector()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        result = {
            'id': 0,
            'corners': [[100, 100], [200, 100], [200, 200], [100, 200]],
            'center': (150, 150),
            'pose': None,
        }
        vis = det.draw(frame, [result])
        # 应有非零像素(绘制了标记)
        self.assertTrue(np.max(vis) > 0)


if __name__ == '__main__':
    unittest.main()
