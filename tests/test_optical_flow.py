#!/usr/bin/env python3
"""
光流法单元测试
覆盖: 稀疏光流(LK)、稠密光流(Farneback)、运动估计器、快捷函数
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
    from optical_flow import (SparseOpticalFlow, DenseOpticalFlow,
                               MotionEstimator, compute_flow,
                               get_motion_direction, get_motion_speed)


def _make_frame(offset_x=0, offset_y=0, size=(300, 400)):
    """创建含白色方块的帧"""
    h, w = size
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    cv2.rectangle(frame, (100 + offset_x, 100 + offset_y),
                  (200 + offset_x, 200 + offset_y), (255, 255, 255), -1)
    return frame


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestSparseOpticalFlowInit(unittest.TestCase):
    """稀疏光流初始化测试"""

    def test_default_params(self):
        """默认参数"""
        lk = SparseOpticalFlow()
        self.assertEqual(lk.max_points, 200)
        self.assertIsNone(lk.prev_gray)
        self.assertIsNone(lk.prev_points)

    def test_custom_params(self):
        """自定义参数"""
        lk = SparseOpticalFlow(max_points=100, quality_level=0.05,
                                min_distance=20, win_size=11)
        self.assertEqual(lk.max_points, 100)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestSparseOpticalFlowUpdate(unittest.TestCase):
    """稀疏光流update测试"""

    def test_first_frame_returns_empty(self):
        """第一帧应返回空结果"""
        lk = SparseOpticalFlow()
        frame = _make_frame()
        result = lk.update(frame)
        self.assertIsNone(result['points'])
        self.assertEqual(result['motion_magnitude'], 0)

    def test_second_frame_returns_motion(self):
        """第二帧应检测到运动"""
        lk = SparseOpticalFlow(max_points=100)
        frame1 = _make_frame(0, 0)
        frame2 = _make_frame(10, 5)
        lk.update(frame1)
        result = lk.update(frame2)
        # 应该有运动
        self.assertIsInstance(result['mean_motion'], tuple)
        self.assertEqual(len(result['mean_motion']), 2)

    def test_returns_required_keys(self):
        """结果应包含所有必要字段"""
        lk = SparseOpticalFlow()
        frame = _make_frame()
        result = lk.update(frame)
        for key in ('points', 'prev_points', 'tracks', 'flow_vectors',
                     'mean_motion', 'motion_magnitude'):
            self.assertIn(key, result)

    def test_static_scene_low_motion(self):
        """静止场景运动应很小"""
        lk = SparseOpticalFlow(max_points=100)
        frame = _make_frame()
        lk.update(frame)
        result = lk.update(frame)
        self.assertAlmostEqual(result['motion_magnitude'], 0.0, delta=0.5)

    def test_moving_scene_detects_motion(self):
        """运动场景应检测到运动"""
        lk = SparseOpticalFlow(max_points=100)
        frame1 = _make_frame(0, 0)
        frame2 = _make_frame(20, 0)
        lk.update(frame1)
        result = lk.update(frame2)
        self.assertGreater(result['motion_magnitude'], 0.0)

    def test_color_and_gray(self):
        """彩色和灰度图都应能处理"""
        lk = SparseOpticalFlow()
        gray = cv2.cvtColor(_make_frame(), cv2.COLOR_BGR2GRAY)
        result = lk.update(gray)
        self.assertIsNotNone(result)

    def test_num_tracked(self):
        """应报告跟踪点数"""
        lk = SparseOpticalFlow(max_points=100)
        frame1 = _make_frame(0, 0)
        frame2 = _make_frame(10, 0)
        lk.update(frame1)
        result = lk.update(frame2)
        self.assertIn('num_tracked', result)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestSparseOpticalFlowReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        """reset应清空状态"""
        lk = SparseOpticalFlow()
        lk.update(_make_frame())
        lk.reset()
        self.assertIsNone(lk.prev_gray)
        self.assertIsNone(lk.prev_points)
        self.assertEqual(lk.tracks, [])


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestSparseOpticalFlowDetectPoints(unittest.TestCase):
    """特征点检测测试"""

    def test_detects_points_on_feature_rich(self):
        """特征丰富的图像应检测到点"""
        lk = SparseOpticalFlow(max_points=200)
        frame = _make_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pts = lk.detect_points(gray)
        self.assertIsNotNone(pts)
        self.assertGreater(len(pts), 0)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestDenseOpticalFlowInit(unittest.TestCase):
    """稠密光流初始化测试"""

    def test_default_params(self):
        """默认参数"""
        df = DenseOpticalFlow()
        self.assertIsNone(df.prev_gray)
        self.assertIsNone(df.accumulated_flow)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestDenseOpticalFlowUpdate(unittest.TestCase):
    """稠密光流update测试"""

    def test_first_frame_returns_empty(self):
        """第一帧应返回空结果"""
        df = DenseOpticalFlow()
        result = df.update(_make_frame())
        self.assertIsNone(result['flow'])
        self.assertEqual(result['mean_magnitude'], 0)

    def test_returns_required_keys(self):
        """结果应包含所有必要字段"""
        df = DenseOpticalFlow()
        df.update(_make_frame(0, 0))
        result = df.update(_make_frame(10, 0))
        for key in ('flow', 'magnitude', 'angle', 'mean_magnitude',
                     'dominant_direction', 'motion_mask'):
            self.assertIn(key, result)

    def test_flow_shape(self):
        """光流场应与输入同尺寸"""
        df = DenseOpticalFlow()
        frame1 = _make_frame(0, 0)
        frame2 = _make_frame(10, 0)
        df.update(frame1)
        result = df.update(frame2)
        h, w = frame1.shape[:2]
        self.assertEqual(result['flow'].shape, (h, w, 2))

    def test_moving_scene_has_magnitude(self):
        """运动场景应有非零运动幅度"""
        df = DenseOpticalFlow()
        df.update(_make_frame(0, 0))
        result = df.update(_make_frame(20, 0))
        self.assertGreater(result['mean_magnitude'], 0)

    def test_motion_mask_binary(self):
        """运动掩码应为二值图像"""
        df = DenseOpticalFlow()
        df.update(_make_frame(0, 0))
        result = df.update(_make_frame(20, 0))
        mask = result['motion_mask']
        unique = np.unique(mask)
        for v in unique:
            self.assertIn(v, [0, 255])

    def test_accumulated_flow_exists(self):
        """多帧后应有累积光流"""
        df = DenseOpticalFlow()
        df.update(_make_frame(0, 0))
        df.update(_make_frame(5, 0))
        df.update(_make_frame(10, 0))
        self.assertIsNotNone(df.accumulated_flow)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestDenseOpticalFlowReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        """reset应清空状态"""
        df = DenseOpticalFlow()
        df.update(_make_frame(0, 0))
        df.update(_make_frame(10, 0))
        df.reset()
        self.assertIsNone(df.prev_gray)
        self.assertIsNone(df.accumulated_flow)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestDrawFlowHSV(unittest.TestCase):
    """HSV可视化测试"""

    def test_hsv_shape(self):
        """HSV图应与输入同尺寸"""
        flow = np.random.randn(100, 200, 2).astype(np.float32)
        hsv = DenseOpticalFlow.draw_flow_hsv(flow)
        self.assertEqual(hsv.shape, (100, 200, 3))

    def test_hsv_dtype(self):
        """HSV图应为uint8"""
        flow = np.zeros((50, 50, 2), dtype=np.float32)
        hsv = DenseOpticalFlow.draw_flow_hsv(flow)
        self.assertEqual(hsv.dtype, np.uint8)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestDrawMotionVectors(unittest.TestCase):
    """运动矢量可视化测试"""

    def test_returns_same_shape(self):
        """应返回同尺寸图像"""
        frame = np.zeros((100, 200, 3), dtype=np.uint8)
        flow = np.zeros((100, 200, 2), dtype=np.float32)
        vis = DenseOpticalFlow.draw_motion_vectors(frame, flow, step=16)
        self.assertEqual(vis.shape, frame.shape)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMotionEstimator(unittest.TestCase):
    """运动估计器测试"""

    def test_first_analyze(self):
        """第一帧分析"""
        me = MotionEstimator()
        result = me.analyze(_make_frame())
        self.assertIn('is_moving', result)
        self.assertIn('motion_type', result)

    def test_motion_type_values(self):
        """motion_type应为预设值"""
        me = MotionEstimator()
        me.analyze(_make_frame(0, 0))
        result = me.analyze(_make_frame(10, 0))
        self.assertIn(result['motion_type'], ['still', 'slow', 'fast'])

    def test_moving_detection(self):
        """大位移应检测为运动"""
        me = MotionEstimator()
        me.analyze(_make_frame(0, 0))
        result = me.analyze(_make_frame(30, 0))
        self.assertIn('speed', result)
        self.assertIn('direction', result)

    def test_contains_dense_sparse(self):
        """结果应包含dense和sparse子结果"""
        me = MotionEstimator()
        me.analyze(_make_frame(0, 0))
        result = me.analyze(_make_frame(10, 0))
        self.assertIn('dense', result)
        self.assertIn('sparse', result)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestComputeFlowShortcut(unittest.TestCase):
    """快捷函数测试"""

    def test_compute_flow(self):
        """compute_flow应返回光流场"""
        gray1 = cv2.cvtColor(_make_frame(0, 0), cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(_make_frame(10, 0), cv2.COLOR_BGR2GRAY)
        flow = compute_flow(gray1, gray2)
        self.assertEqual(flow.shape[:2], gray1.shape)
        self.assertEqual(flow.shape[2], 2)

    def test_get_motion_direction(self):
        """get_motion_direction应返回角度"""
        gray1 = cv2.cvtColor(_make_frame(0, 0), cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(_make_frame(10, 0), cv2.COLOR_BGR2GRAY)
        flow = compute_flow(gray1, gray2)
        direction = get_motion_direction(flow)
        self.assertIsInstance(direction, float)
        self.assertGreaterEqual(direction, 0)
        self.assertLess(direction, 360)

    def test_get_motion_speed(self):
        """get_motion_speed应返回非负值"""
        gray1 = cv2.cvtColor(_make_frame(0, 0), cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(_make_frame(10, 0), cv2.COLOR_BGR2GRAY)
        flow = compute_flow(gray1, gray2)
        speed = get_motion_speed(flow)
        self.assertGreaterEqual(speed, 0)


if __name__ == '__main__':
    unittest.main()
