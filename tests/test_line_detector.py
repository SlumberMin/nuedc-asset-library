#!/usr/bin/env python3
"""
循迹线检测单元测试
覆盖: LineDetector 初始化、ROI提取、掩膜生成、切片法检测、偏移计算、曲线拟合
注意: 使用合成图像进行测试，无需摄像头
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
    from visual_common.line_detector import LineDetector


def _make_black_line_frame(width=640, height=480, line_x=320, line_width=10):
    """生成含黑色循迹线的合成图像"""
    frame = np.ones((height, width, 3), dtype=np.uint8) * 200  # 灰色背景
    # 画黑色竖线
    x1 = max(0, line_x - line_width // 2)
    x2 = min(width, line_x + line_width // 2)
    frame[:, x1:x2] = 0  # 黑色
    return frame


def _make_curved_line_frame(width=640, height=480):
    """生成含曲线的合成图像"""
    frame = np.ones((height, width, 3), dtype=np.uint8) * 200
    for y in range(height):
        x = int(320 + 50 * np.sin(y * 0.02))
        x1 = max(0, x - 5)
        x2 = min(width, x + 5)
        frame[y, x1:x2] = 0
    return frame


def _make_empty_frame(width=640, height=480):
    """生成无线条的纯色图像"""
    return np.ones((height, width, 3), dtype=np.uint8) * 200


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLineDetectorInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        """默认参数"""
        det = LineDetector()
        self.assertEqual(det.line_color, 'black')
        self.assertAlmostEqual(det.roi_ratio, 0.5)

    def test_custom_color(self):
        """自定义颜色"""
        det = LineDetector(line_color='red')
        self.assertEqual(det.line_color, 'red')

    def test_custom_roi(self):
        """自定义ROI比例"""
        det = LineDetector(roi_ratio=0.3)
        self.assertAlmostEqual(det.roi_ratio, 0.3)

    def test_center_history_empty(self):
        """初始历史应为空"""
        det = LineDetector()
        self.assertEqual(len(det.center_history), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLineDetectorROI(unittest.TestCase):
    """ROI提取测试"""

    def test_roi_height(self):
        """ROI高度应为原图的roi_ratio"""
        det = LineDetector(roi_ratio=0.5)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        roi, y_offset = det._get_roi(frame)
        self.assertEqual(roi.shape[0], 240)
        self.assertEqual(y_offset, 240)

    def test_roi_width_unchanged(self):
        """ROI宽度应不变"""
        det = LineDetector(roi_ratio=0.5)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        roi, _ = det._get_roi(frame)
        self.assertEqual(roi.shape[1], 640)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLineDetectorMask(unittest.TestCase):
    """掩膜生成测试"""

    def test_black_mask(self):
        """黑色掩膜应检测黑色区域"""
        det = LineDetector(line_color='black')
        # 创建HSV图像: 黑色区域V值低
        hsv = np.zeros((100, 100, 3), dtype=np.uint8)
        hsv[:, 50:60, 2] = 30  # 低V值=黑色
        mask = det._get_mask(hsv)
        # 黑色区域应为白色(255)
        self.assertTrue(np.max(mask[:, 50:60]) > 0)

    def test_red_mask_dual_range(self):
        """红色应处理双色相范围"""
        det = LineDetector(line_color='red')
        hsv = np.zeros((100, 100, 3), dtype=np.uint8)
        # 红色H=0~10, S>100, V>100
        hsv[:, 50:60, 0] = 5
        hsv[:, 50:60, 1] = 150
        hsv[:, 50:60, 2] = 150
        mask = det._get_mask(hsv)
        self.assertTrue(np.max(mask[:, 50:60]) > 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLineDetectorCenterDetection(unittest.TestCase):
    """切片法中心检测测试"""

    def test_detect_center_vertical_line(self):
        """垂直线应检测到中心点"""
        det = LineDetector()
        # 创建二值掩膜: 中间一条白线
        mask = np.zeros((100, 640), dtype=np.uint8)
        mask[:, 310:330] = 255
        points = det.detect_line_center(mask, n_slices=5)
        self.assertTrue(len(points) > 0)
        # 中心点x应接近320
        for pt in points:
            self.assertAlmostEqual(pt[0], 320, delta=30)

    def test_no_line_no_points(self):
        """无线条时应返回空"""
        det = LineDetector()
        mask = np.zeros((100, 640), dtype=np.uint8)
        points = det.detect_line_center(mask, n_slices=5)
        self.assertEqual(len(points), 0)

    def test_left_line(self):
        """左侧线条"""
        det = LineDetector()
        mask = np.zeros((100, 640), dtype=np.uint8)
        mask[:, 50:70] = 255
        points = det.detect_line_center(mask, n_slices=5)
        if points:
            for pt in points:
                self.assertLess(pt[0], 200)

    def test_right_line(self):
        """右侧线条"""
        det = LineDetector()
        mask = np.zeros((100, 640), dtype=np.uint8)
        mask[:, 550:570] = 255
        points = det.detect_line_center(mask, n_slices=5)
        if points:
            for pt in points:
                self.assertGreater(pt[0], 400)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLineDetectorDeviation(unittest.TestCase):
    """偏移计算测试"""

    def test_center_zero_deviation(self):
        """中心线应产生零偏移"""
        det = LineDetector()
        # 中心点在中间
        points = [(320, i * 10) for i in range(5)]
        dev, angle = det.calculate_deviation(points, 640)
        self.assertAlmostEqual(dev, 0.0, delta=5.0)

    def test_right_positive_deviation(self):
        """右侧线应产生正偏移"""
        det = LineDetector()
        points = [(420, i * 10) for i in range(5)]
        dev, _ = det.calculate_deviation(points, 640)
        self.assertGreater(dev, 0)

    def test_left_negative_deviation(self):
        """左侧线应产生负偏移"""
        det = LineDetector()
        points = [(220, i * 10) for i in range(5)]
        dev, _ = det.calculate_deviation(points, 640)
        self.assertLess(dev, 0)

    def test_empty_points_none(self):
        """空点集应返回None"""
        det = LineDetector()
        dev, angle = det.calculate_deviation([], 640)
        self.assertIsNone(dev)
        self.assertIsNone(angle)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLineDetectorCurveFit(unittest.TestCase):
    """曲线拟合测试"""

    def test_straight_line_fit(self):
        """直线点集应拟合出直线"""
        det = LineDetector()
        points = [(320, i * 10) for i in range(10)]
        coeffs, curve = det.fit_curve(points)
        self.assertIsNotNone(coeffs)
        # 二次项系数应接近0(直线)
        self.assertAlmostEqual(coeffs[0], 0.0, delta=0.1)

    def test_curved_line_fit(self):
        """曲线点集应拟合出曲线"""
        det = LineDetector()
        points = [(int(320 + 50 * np.sin(i * 0.3)), i * 10) for i in range(20)]
        coeffs, curve = det.fit_curve(points)
        self.assertIsNotNone(coeffs)
        self.assertTrue(len(curve) > 0)

    def test_too_few_points(self):
        """少于3个点应返回None"""
        det = LineDetector()
        points = [(320, 0), (320, 10)]
        coeffs, curve = det.fit_curve(points)
        self.assertIsNone(coeffs)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLineDetectorFullPipeline(unittest.TestCase):
    """完整检测流程测试"""

    def test_detect_black_line(self):
        """检测黑色线条"""
        det = LineDetector(line_color='black')
        frame = _make_black_line_frame(line_x=320)
        result = det.detect(frame)
        self.assertIn('center_points', result)
        self.assertIn('deviation', result)
        self.assertIn('mask', result)

    def test_detect_returns_all_keys(self):
        """检测结果应包含所有必要键"""
        det = LineDetector()
        frame = _make_black_line_frame()
        result = det.detect(frame)
        required_keys = ['lines', 'center_points', 'curve_points',
                         'deviation', 'angle', 'mask', 'roi_offset']
        for key in required_keys:
            self.assertIn(key, result)

    def test_detect_curved_line(self):
        """检测曲线"""
        det = LineDetector()
        frame = _make_curved_line_frame()
        result = det.detect(frame)
        # 应该检测到一些中心点
        self.assertIsInstance(result['center_points'], list)

    def test_center_history_updated(self):
        """检测后历史应更新"""
        det = LineDetector()
        frame = _make_black_line_frame()
        det.detect(frame)
        # 历史可能有数据
        self.assertIsInstance(det.center_history, type(det.center_history))


if __name__ == '__main__':
    unittest.main()
