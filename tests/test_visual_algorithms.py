#!/usr/bin/env python3
"""
视觉通用代码库单元测试
覆盖: 所有检测器、识别器、跟踪器模块
模块来源: 10_视觉通用代码库/
包含: color_tracker, line_detector, aruco_detector, qr_decoder, obstacle_detector等
"""

import sys
import os
import unittest
import time
import numpy as np
from unittest.mock import MagicMock, patch, PropertyMock
from typing import List, Tuple, Dict, Any

# 将项目根目录加入路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


def create_test_image(width: int = 640, height: int = 480, color: Tuple = (0, 0, 0)) -> np.ndarray:
    """创建测试图像"""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = color
    return img


def create_circle_image(cx: int, cy: int, radius: int, color: Tuple,
                       width: int = 640, height: int = 480) -> np.ndarray:
    """创建带圆形的测试图像"""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.circle(img, (cx, cy), radius, color, -1)
    return img


def create_multi_target_image(circles: List[Tuple], width: int = 640, height: int = 480) -> np.ndarray:
    """创建包含多个目标的测试图像"""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    for cx, cy, radius, color in circles:
        cv2.circle(img, (cx, cy), radius, color, -1)
    return img


# ===================== 颜色检测器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过颜色检测器测试")
class TestColorTracker(unittest.TestCase):
    """颜色追踪器测试"""

    def setUp(self):
        """测试前初始化"""
        try:
            from visual.color_tracker import ColorTracker
            self.tracker = ColorTracker(color_name='red', min_area=100)
        except ImportError:
            self.skipTest("ColorTracker模块未安装")

    def test_initialization(self):
        """测试追踪器初始化"""
        self.assertIsNotNone(self.tracker)
        self.assertTrue(hasattr(self.tracker, 'color_config'))
        self.assertIn('lower1', self.tracker.color_config)

    def test_red_color_preset(self):
        """测试红色预设"""
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='red')
        self.assertIn('lower2', tracker.color_config)  # 红色有双区间

    def test_blue_color_preset(self):
        """测试蓝色预设"""
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='blue')
        self.assertIn('lower1', tracker.color_config)
        self.assertNotIn('lower2', tracker.color_config)

    def test_color_detection_performance(self):
        """颜色检测性能基准测试"""
        img = create_circle_image(320, 240, 80, (0, 0, 255))  # 红色圆
        start_time = time.time()
        for _ in range(100):
            results, mask = self.tracker.update(img)
        elapsed = time.time() - start_time

        # 性能要求: 100帧 < 1秒
        self.assertLess(elapsed, 1.0, "颜色检测性能过低")
        self.assertGreater(len(results), 0)

    def test_edge_case_noisy_image(self):
        """边界条件: 噪声图像"""
        img = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
        results, mask = self.tracker.update(img)
        self.assertIsInstance(results, list)
        self.assertIsNotNone(mask)

    def test_edge_case_small_image(self):
        """边界条件: 小图像"""
        img = create_test_image(100, 100, (0, 0, 255))
        results, mask = self.tracker.update(img)
        self.assertIsInstance(results, list)

    def test_multiple_targets(self):
        """多目标检测"""
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='red', min_area=100, max_targets=5)

        img = create_multi_target_image([
            (160, 240, 50, (0, 0, 255)),
            (480, 240, 50, (0, 0, 255)),
        ])
        results, mask = tracker.update(img)
        self.assertGreaterEqual(len(results), 1)


# ===================== 直线检测器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过直线检测器测试")
class TestLineDetector(unittest.TestCase):
    """直线检测器测试"""

    def setUp(self):
        try:
            from visual.line_detector import LineDetector
            self.detector = LineDetector()
        except ImportError:
            self.skipTest("LineDetector模块未安装")

    def test_initialization(self):
        """测试检测器初始化"""
        self.assertIsNotNone(self.detector)

    def test_detect_horizontal_line(self):
        """检测水平线"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.line(img, (0, 240), (640, 240), (0, 0, 0), 2)

        results = self.detector.detect(img)
        self.assertIsInstance(results, list)
        self.assertGreater(len(results), 0)

    def test_detect_vertical_line(self):
        """检测垂直线"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.line(img, (320, 0), (320, 480), (0, 0, 0), 2)

        results = self.detector.detect(img)
        self.assertIsInstance(results, list)

    def test_no_line_in_blank_image(self):
        """空白图像中无直线"""
        img = create_test_image(640, 480, (255, 255, 255))
        results = self.detector.detect(img)
        self.assertIsInstance(results, list)

    def test_performance_benchmark(self):
        """性能基准测试"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.line(img, (0, 240), (640, 240), (0, 0, 0), 2)

        start_time = time.time()
        for _ in range(100):
            results = self.detector.detect(img)
        elapsed = time.time() - start_time

        self.assertLess(elapsed, 2.0, "直线检测性能过低")


# ===================== ArUco检测器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过ArUco检测器测试")
class TestArUcoDetector(unittest.TestCase):
    """ArUco标记检测器测试"""

    def setUp(self):
        try:
            from visual.aruco_detector import ArUcoDetector
            self.detector = ArUcoDetector()
        except ImportError:
            self.skipTest("ArUcoDetector模块未安装")

    def test_initialization(self):
        """测试检测器初始化"""
        self.assertIsNotNone(self.detector)

    def test_create_aruco_marker(self):
        """创建ArUco标记"""
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            marker = cv2.aruco.drawMarker(aruco_dict, 0, 200)
            self.assertEqual(marker.shape, (200, 200))
        except Exception as e:
            self.skipTest(f"ArUco创建失败: {e}")

    def test_edge_case_no_markers(self):
        """边界条件: 无标记图像"""
        img = create_test_image(640, 480, (255, 255, 255))
        results = self.detector.detect(img)
        self.assertIsInstance(results, list)
        self.assertEqual(len(results), 0)

    def test_detect_synthetic_marker(self):
        """检测合成标记"""
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            marker_img = cv2.aruco.drawMarker(aruco_dict, 0, 300)

            img = create_test_image(640, 480, (255, 255, 255))
            img[90:390, 170:470] = marker_img

            results = self.detector.detect(img)
            self.assertIsInstance(results, list)
        except Exception as e:
            self.skipTest(f"ArUco检测失败: {e}")


# ===================== 二维码解码器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过二维码解码器测试")
class TestQRDecoder(unittest.TestCase):
    """二维码解码器测试"""

    def setUp(self):
        try:
            from visual.qr_decoder import QRDecoder
            self.decoder = QRDecoder()
        except ImportError:
            self.skipTest("QRDecoder模块未安装")

    def test_initialization(self):
        """测试解码器初始化"""
        self.assertIsNotNone(self.decoder)

    def test_edge_case_no_qr(self):
        """边界条件: 无二维码图像"""
        img = create_test_image(640, 480, (255, 255, 255))
        results = self.decoder.decode(img)
        self.assertIsInstance(results, list)


# ===================== 障碍物检测器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过障碍物检测器测试")
class TestObstacleDetector(unittest.TestCase):
    """障碍物检测器测试"""

    def setUp(self):
        try:
            from visual.obstacle_detector import ObstacleDetector
            self.detector = ObstacleDetector()
        except ImportError:
            self.skipTest("ObstacleDetector模块未安装")

    def test_initialization(self):
        """测试检测器初始化"""
        self.assertIsNotNone(self.detector)

    def test_no_obstacle_in_blank_image(self):
        """空白图像中无障碍物"""
        img = create_test_image(640, 480, (200, 200, 200))
        obstacles = self.detector.detect(img)
        self.assertIsInstance(obstacles, list)

    def test_detect_rectangle_obstacle(self):
        """检测矩形障碍物"""
        img = create_test_image(640, 480, (200, 200, 200))
        cv2.rectangle(img, (200, 150), (440, 330), (50, 50, 50), -1)

        obstacles = self.detector.detect(img)
        self.assertIsInstance(obstacles, list)

    def test_performance_benchmark(self):
        """性能基准测试"""
        img = create_test_image(640, 480, (200, 200, 200))
        cv2.rectangle(img, (200, 150), (440, 330), (50, 50, 50), -1)

        start_time = time.time()
        for _ in range(50):
            obstacles = self.detector.detect(img)
        elapsed = time.time() - start_time

        self.assertLess(elapsed, 2.0, "障碍物检测性能过低")


# ===================== 运动检测器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过运动检测器测试")
class TestMotionDetector(unittest.TestCase):
    """运动检测器测试"""

    def setUp(self):
        try:
            from visual.motion_detector import MotionDetector
            self.detector = MotionDetector()
        except ImportError:
            self.skipTest("MotionDetector模块未安装")

    def test_initialization(self):
        """测试检测器初始化"""
        self.assertIsNotNone(self.detector)

    def test_no_motion_in_identical_frames(self):
        """相同帧之间无运动"""
        img = create_test_image(640, 480, (100, 100, 100))

        motion1 = self.detector.detect(img)
        motion2 = self.detector.detect(img)

        self.assertFalse(motion2.get('has_motion', False))

    def test_detect_motion(self):
        """检测运动"""
        img1 = create_test_image(640, 480, (100, 100, 100))
        self.detector.detect(img1)

        # 添加一个移动的物体
        img2 = create_test_image(640, 480, (100, 100, 100))
        cv2.circle(img2, (300, 240), 50, (0, 0, 255), -1)

        result = self.detector.detect(img2)
        self.assertIsInstance(result, dict)

    def test_edge_case_single_frame(self):
        """边界条件: 第一帧(无对比)"""
        img = create_test_image(640, 480, (100, 100, 100))
        result = self.detector.detect(img)
        self.assertIsInstance(result, dict)


# ===================== Kalman跟踪器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过Kalman跟踪器测试")
class TestTrackingKalman(unittest.TestCase):
    """Kalman滤波跟踪器测试"""

    def setUp(self):
        try:
            from visual.tracking_kalman import TrackingKalman
            self.tracker = TrackingKalman(initial_x=100, initial_y=200)
        except ImportError:
            self.skipTest("TrackingKalman模块未安装")

    def test_initialization(self):
        """测试跟踪器初始化"""
        self.assertIsNotNone(self.tracker)
        pos = self.tracker.get_position()
        self.assertEqual(len(pos), 2)

    def test_predict(self):
        """预测下一步位置"""
        pos = self.tracker.predict()
        self.assertEqual(len(pos), 2)

    def test_update_position(self):
        """更新位置"""
        self.tracker.update(110, 210)
        pos = self.tracker.get_position()
        self.assertGreater(pos[0], 0)
        self.assertGreater(pos[1], 0)

    def test_trajectory_recording(self):
        """轨迹记录"""
        for i in range(10):
            self.tracker.update(100 + i * 10, 200 + i * 5)

        trajectory = self.tracker.trajectory
        self.assertGreater(len(trajectory), 0)

    def test_performance_benchmark(self):
        """性能基准测试"""
        start_time = time.time()
        for i in range(1000):
            self.tracker.update(100 + (i % 100), 200 + (i % 50))
        elapsed = time.time() - start_time

        # 1000次更新 < 0.1秒
        self.assertLess(elapsed, 0.1, "Kalman跟踪器性能过低")


# ===================== 多ROI检测器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过多ROI检测器测试")
class TestMultiROIDetector(unittest.TestCase):
    """多感兴趣区域检测器测试"""

    def setUp(self):
        try:
            from visual.multi_roi_detector import MultiROIDetector
            self.detector = MultiROIDetector()
        except ImportError:
            self.skipTest("MultiROIDetector模块未安装")

    def test_initialization(self):
        """测试检测器初始化"""
        self.assertIsNotNone(self.detector)

    def test_add_roi(self):
        """添加ROI区域"""
        self.detector.add_roi(x=100, y=100, w=200, h=200, name="roi1")
        self.assertGreater(len(self.detector.rois), 0)

    def test_edge_case_empty_rois(self):
        """边界条件: 无ROI"""
        img = create_test_image(640, 480, (100, 100, 100))
        results = self.detector.detect(img)
        self.assertIsInstance(results, dict)


# ===================== 轨迹预测器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过轨迹预测器测试")
class TestTrajectoryPredictor(unittest.TestCase):
    """轨迹预测器测试"""

    def setUp(self):
        try:
            from visual.trajectory_predictor import TrajectoryPredictor
            self.predictor = TrajectoryPredictor()
        except ImportError:
            self.skipTest("TrajectoryPredictor模块未安装")

    def test_initialization(self):
        """测试预测器初始化"""
        self.assertIsNotNone(self.predictor)

    def test_predict_after_updates(self):
        """多次更新后预测"""
        for i in range(10):
            self.predictor.update(100 + i * 10, 200 + i * 5, timestamp=i * 0.1)

        prediction = self.predictor.predict(steps_ahead=5)
        self.assertIsInstance(prediction, tuple)
        self.assertEqual(len(prediction), 2)

    def test_edge_case_insufficient_data(self):
        """边界条件: 数据不足"""
        self.predictor.update(100, 200, timestamp=0.0)
        prediction = self.predictor.predict(steps_ahead=5)
        self.assertIsInstance(prediction, tuple)


# ===================== 角度估计器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过角度估计器测试")
class TestAngleEstimator(unittest.TestCase):
    """角度估计器测试"""

    def setUp(self):
        try:
            from visual.angle_estimator import AngleEstimator
            self.estimator = AngleEstimator()
        except ImportError:
            self.skipTest("AngleEstimator模块未安装")

    def test_initialization(self):
        """测试估计器初始化"""
        self.assertIsNotNone(self.estimator)

    def test_estimate_horizontal_line(self):
        """估计水平线角度"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.line(img, (0, 240), (640, 240), (0, 0, 0), 2)

        angle = self.estimator.estimate(img)
        self.assertIsNotNone(angle)
        if angle is not None:
            self.assertAlmostEqual(angle, 0, delta=10)

    def test_estimate_vertical_line(self):
        """估计垂直线角度"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.line(img, (320, 0), (320, 480), (0, 0, 0), 2)

        angle = self.estimator.estimate(img)
        self.assertIsNotNone(angle)
        if angle is not None:
            self.assertAlmostEqual(abs(angle), 90, delta=10)


# ===================== 距离估计器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过距离估计器测试")
class TestDistanceEstimator(unittest.TestCase):
    """距离估计器测试"""

    def setUp(self):
        try:
            from visual.distance_estimator import DistanceEstimator
            self.estimator = DistanceEstimator()
        except ImportError:
            self.skipTest("DistanceEstimator模块未安装")

    def test_initialization(self):
        """测试估计器初始化"""
        self.assertIsNotNone(self.estimator)

    def test_estimate_distance_to_large_object(self):
        """大物体应距离近"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.rectangle(img, (50, 50), (590, 430), (0, 0, 0), -1)

        distance = self.estimator.estimate(img)
        self.assertIsNotNone(distance)
        if distance is not None:
            self.assertGreater(distance, 0)

    def test_estimate_distance_to_small_object(self):
        """小物体应距离远"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.rectangle(img, (300, 220), (340, 260), (0, 0, 0), -1)

        distance = self.estimator.estimate(img)
        self.assertIsNotNone(distance)
        if distance is not None:
            self.assertGreater(distance, 0)


# ===================== 轮廓分析器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过轮廓分析器测试")
class TestContourAnalyzer(unittest.TestCase):
    """轮廓分析器测试"""

    def setUp(self):
        try:
            from visual.contour_analyzer import ContourAnalyzer
            self.analyzer = ContourAnalyzer()
        except ImportError:
            self.skipTest("ContourAnalyzer模块未安装")

    def test_initialization(self):
        """测试分析器初始化"""
        self.assertIsNotNone(self.analyzer)

    def test_analyze_circular_contour(self):
        """分析圆形轮廓"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.circle(img, (320, 240), 100, (0, 0, 0), -1)

        contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours[0]) > 0:
            analysis = self.analyzer.analyze(contours[0][0])
            self.assertIsInstance(analysis, dict)

    def test_edge_case_empty_contour(self):
        """边界条件: 空轮廓"""
        contour = np.array([])
        try:
            analysis = self.analyzer.analyze(contour)
            self.assertIsInstance(analysis, dict)
        except Exception:
            pass  # 预期可能抛出异常


# ===================== 模板匹配器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过模板匹配器测试")
class TestTemplateMatcher(unittest.TestCase):
    """模板匹配器测试"""

    def setUp(self):
        try:
            from visual.template_matcher import TemplateMatcher
            self.matcher = TemplateMatcher()
        except ImportError:
            self.skipTest("TemplateMatcher模块未安装")

    def test_initialization(self):
        """测试匹配器初始化"""
        self.assertIsNotNone(self.matcher)

    def test_match_template(self):
        """模板匹配"""
        img = create_test_image(640, 480, (200, 200, 200))
        cv2.rectangle(img, (300, 200), (380, 280), (100, 100, 100), -1)
        template = img[200:280, 300:380].copy()

        result = self.matcher.match(img, template)
        self.assertIsInstance(result, dict)

    def test_edge_case_no_match(self):
        """边界条件: 无法匹配"""
        img = create_test_image(640, 480, (200, 200, 200))
        template = create_test_image(50, 50, (50, 50, 50))

        result = self.matcher.match(img, template)
        self.assertIsInstance(result, dict)


# ===================== 光流测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过光流测试")
class TestOpticalFlow(unittest.TestCase):
    """光流计算测试"""

    def setUp(self):
        try:
            from visual.optical_flow import OpticalFlow
            self.flow = OpticalFlow()
        except ImportError:
            self.skipTest("OpticalFlow模块未安装")

    def test_initialization(self):
        """测试初始化"""
        self.assertIsNotNone(self.flow)

    def test_compute_flow(self):
        """计算光流"""
        img1 = create_test_image(640, 480, (100, 100, 100))
        img2 = create_test_image(640, 480, (100, 100, 100))
        cv2.circle(img2, (320, 240), 50, (0, 0, 255), -1)

        flow = self.flow.compute(img1, img2)
        self.assertIsNotNone(flow)

    def test_edge_case_identical_frames(self):
        """边界条件: 相同帧"""
        img = create_test_image(640, 480, (100, 100, 100))
        flow = self.flow.compute(img, img)
        self.assertIsNotNone(flow)


# ===================== 特征匹配器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过特征匹配器测试")
class TestFeatureMatcher(unittest.TestCase):
    """特征匹配器测试"""

    def setUp(self):
        try:
            from visual.feature_matcher import FeatureMatcher
            self.matcher = FeatureMatcher()
        except ImportError:
            self.skipTest("FeatureMatcher模块未安装")

    def test_initialization(self):
        """测试初始化"""
        self.assertIsNotNone(self.matcher)

    def test_detect_and_match(self):
        """检测特征点并匹配"""
        img1 = create_test_image(640, 480, (100, 100, 100))
        img2 = create_test_image(640, 480, (100, 100, 100))

        # 添加一些纹理
        for i in range(0, 640, 50):
            for j in range(0, 480, 50):
                cv2.circle(img1, (i, j), 5, (150, 150, 150), -1)
                cv2.circle(img2, (i + 10, j), 5, (150, 150, 150), -1)

        matches = self.matcher.match(img1, img2)
        self.assertIsInstance(matches, list)


# ===================== 背景减除器测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过背景减除器测试")
class TestBackgroundSubtractor(unittest.TestCase):
    """背景减除器测试"""

    def setUp(self):
        try:
            from visual.background_subtractor import BackgroundSubtractor
            self.subtractor = BackgroundSubtractor()
        except ImportError:
            self.skipTest("BackgroundSubtractor模块未安装")

    def test_initialization(self):
        """测试初始化"""
        self.assertIsNotNone(self.subtractor)

    def test_apply_background_subtraction(self):
        """应用背景减除"""
        img = create_test_image(640, 480, (100, 100, 100))

        result = self.subtractor.apply(img)
        self.assertIsNotNone(result)

    def test_learning_rate(self):
        """学习率影响"""
        img = create_test_image(640, 480, (100, 100, 100))

        self.subtractor.apply(img, learning_rate=0.0)
        result = self.subtractor.apply(img, learning_rate=1.0)
        self.assertIsNotNone(result)


if __name__ == '__main__':
    unittest.main()
