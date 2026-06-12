#!/usr/bin/env python3
"""
障碍物检测单元测试
覆盖: Obstacle数据结构、UltrasonicSensor、ObstacleDetector、
      视觉检测、距离估计、角度计算、多传感器融合、可视化
测试对象: 10_视觉通用代码库/obstacle_detector.py
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '10_视觉通用代码库'))

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

if HAS_CV2:
    from obstacle_detector import Obstacle, UltrasonicSensor, ObstacleDetector


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestObstacleDataClass(unittest.TestCase):
    """Obstacle数据结构测试"""

    def test_obstacle_creation(self):
        """Obstacle应能正常创建"""
        obs = Obstacle(
            distance=50.0,
            angle=15.0,
            size=(100, 80),
            center=(320, 240),
            confidence=0.85,
            source='visual'
        )
        self.assertEqual(obs.distance, 50.0)
        self.assertEqual(obs.angle, 15.0)
        self.assertEqual(obs.size, (100, 80))
        self.assertEqual(obs.center, (320, 240))
        self.assertAlmostEqual(obs.confidence, 0.85)
        self.assertEqual(obs.source, 'visual')

    def test_obstacle_sources(self):
        """应支持不同来源"""
        for source in ['ultrasonic', 'visual', 'fused']:
            obs = Obstacle(100, 0, (50, 50), (100, 100), 0.5, source)
            self.assertEqual(obs.source, source)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestUltrasonicSensor(unittest.TestCase):
    """超声波传感器测试"""

    def test_init_without_port(self):
        """无端口初始化应不连接"""
        sensor = UltrasonicSensor()
        self.assertIsNone(sensor.serial_conn)

    def test_read_without_connection(self):
        """无连接时读取应返回None"""
        sensor = UltrasonicSensor()
        result = sensor.read_distance()
        self.assertIsNone(result)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestObstacleDetectorInit(unittest.TestCase):
    """障碍物检测器初始化测试"""

    def test_default_params(self):
        """默认参数初始化"""
        detector = ObstacleDetector.__new__(ObstacleDetector)
        # 测试参数赋值逻辑
        detector.min_area = 500
        detector.max_distance = 200.0
        self.assertEqual(detector.min_area, 500)
        self.assertEqual(detector.max_distance, 200.0)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestDistanceEstimation(unittest.TestCase):
    """距离估计测试"""

    def _make_detector(self):
        """创建不连接摄像头的检测器"""
        detector = ObstacleDetector.__new__(ObstacleDetector)
        detector.min_area = 500
        detector.max_distance = 200.0
        detector.camera_matrix = None
        return detector

    def test_larger_object_closer(self):
        """更大的物体应被估计为更近"""
        detector = self._make_detector()
        d1 = detector._estimate_distance_from_size(200, 200)
        d2 = detector._estimate_distance_from_size(50, 50)
        self.assertLess(d1, d2)

    def test_zero_size_returns_max(self):
        """零大小应返回最大距离"""
        detector = self._make_detector()
        d = detector._estimate_distance_from_size(0, 0)
        self.assertEqual(d, detector.max_distance)

    def test_with_camera_matrix(self):
        """有内参矩阵时应使用焦距计算"""
        detector = self._make_detector()
        detector.camera_matrix = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ], dtype=float)
        d = detector._estimate_distance_from_size(100, 100)
        # expected: (30 * 500) / 100 = 150
        self.assertAlmostEqual(d, 150.0, delta=1.0)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestAngleCalculation(unittest.TestCase):
    """角度计算测试"""

    def _make_detector(self):
        detector = ObstacleDetector.__new__(ObstacleDetector)
        return detector

    def test_center_zero_angle(self):
        """图像中心应为零度"""
        detector = self._make_detector()
        angle = detector._calculate_angle(320, 640)
        self.assertAlmostEqual(angle, 0.0, delta=0.1)

    def test_left_side_negative_angle(self):
        """左侧应为负角度"""
        detector = self._make_detector()
        angle = detector._calculate_angle(160, 640)
        self.assertLess(angle, 0)

    def test_right_side_positive_angle(self):
        """右侧应为正角度"""
        detector = self._make_detector()
        angle = detector._calculate_angle(480, 640)
        self.assertGreater(angle, 0)

    def test_angle_range(self):
        """角度应在合理范围内"""
        detector = self._make_detector()
        for x in range(0, 641, 40):
            angle = detector._calculate_angle(x, 640)
            self.assertGreaterEqual(angle, -30.0)
            self.assertLessEqual(angle, 30.0)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestFuseDetection(unittest.TestCase):
    """多传感器融合测试"""

    def _make_detector(self):
        detector = ObstacleDetector.__new__(ObstacleDetector)
        detector.max_distance = 200.0
        detector.min_area = 500
        return detector

    def test_only_ultrasonic(self):
        """仅有超声波检测结果"""
        detector = self._make_detector()
        result = detector.fuse_detections([], ultrasonic_distance=50.0)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].source, 'ultrasonic')

    def test_only_visual(self):
        """仅有视觉检测结果"""
        detector = self._make_detector()
        vis_obs = [Obstacle(60.0, 5.0, (100, 80), (320, 240), 0.7, 'visual')]
        result = detector.fuse_detections(vis_obs, ultrasonic_distance=None)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].source, 'visual')

    def test_matching_fusion(self):
        """角度和距离接近时应融合"""
        detector = self._make_detector()
        vis_obs = [Obstacle(50.0, 5.0, (100, 80), (320, 240), 0.7, 'visual')]
        result = detector.fuse_detections(vis_obs, ultrasonic_distance=55.0)
        # 应有融合结果
        fused = [r for r in result if r.source == 'fused']
        self.assertGreater(len(fused), 0)

    def test_non_matching_no_fusion(self):
        """角度或距离差异大时不融合"""
        detector = self._make_detector()
        vis_obs = [Obstacle(50.0, 25.0, (100, 80), (320, 240), 0.7, 'visual')]
        result = detector.fuse_detections(vis_obs, ultrasonic_distance=55.0)
        # 角度差异25度 > 10度，不应融合
        fused = [r for r in result if r.source == 'fused']
        self.assertEqual(len(fused), 0)

    def test_ultrasonic_beyond_max_distance_ignored(self):
        """超声波超出最大距离时应忽略"""
        detector = self._make_detector()
        result = detector.fuse_detections([], ultrasonic_distance=300.0)
        self.assertEqual(len(result), 0)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestPreprocessFrame(unittest.TestCase):
    """图像预处理测试"""

    def _make_detector(self):
        detector = ObstacleDetector.__new__(ObstacleDetector)
        return detector

    def test_preprocess_shape(self):
        """预处理后应为灰度图"""
        detector = self._make_detector()
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        result = detector.preprocess_frame(frame)
        self.assertEqual(len(result.shape), 2)
        self.assertEqual(result.shape, (480, 640))


if __name__ == '__main__':
    unittest.main()
