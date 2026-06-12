#!/usr/bin/env python3
"""
运动检测单元测试
覆盖: FrameDiffDetector、BgSubtractDetector、MultiFrameDiffDetector、
      OpticalFlowDetector、AccumulatedDiffDetector、MotionDetector、
      MotionRegion数据结构、检测方法切换、重置、可视化
测试对象: 10_视觉通用代码库/motion_detector.py
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
    from motion_detector import (
        MotionMethod, MotionRegion, FrameDiffDetector,
        MultiFrameDiffDetector, BgSubtractDetector,
        OpticalFlowDetector, AccumulatedDiffDetector, MotionDetector
    )


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestMotionRegion(unittest.TestCase):
    """MotionRegion数据结构测试"""

    def test_creation(self):
        """MotionRegion应能正常创建"""
        region = MotionRegion(
            bbox=(10, 20, 100, 80),
            center=(60, 60),
            area=8000,
            velocity=(1.5, -0.5),
            direction=45.0
        )
        self.assertEqual(region.bbox, (10, 20, 100, 80))
        self.assertEqual(region.center, (60, 60))
        self.assertEqual(region.area, 8000)
        self.assertEqual(region.velocity, (1.5, -0.5))
        self.assertEqual(region.direction, 45.0)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestMotionMethodEnum(unittest.TestCase):
    """运动检测方法枚举测试"""

    def test_all_methods_exist(self):
        """所有方法应存在"""
        methods = [
            MotionMethod.FRAME_DIFF,
            MotionMethod.BG_SUBTRACT_MOG2,
            MotionMethod.BG_SUBTRACT_KNN,
            MotionMethod.OPTICAL_FLOW,
            MotionMethod.ACCUM_DIFF,
        ]
        for m in methods:
            self.assertIsNotNone(m.value)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestFrameDiffDetector(unittest.TestCase):
    """帧差法测试"""

    def test_first_frame_no_motion(self):
        """首帧不应检测到运动"""
        detector = FrameDiffDetector(threshold=30, min_area=500)
        frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        mask, regions = detector.detect(frame)
        self.assertEqual(len(regions), 0)

    def test_identical_frames_no_motion(self):
        """相同帧不应检测到运动"""
        detector = FrameDiffDetector(threshold=30, min_area=100)
        frame = np.ones((240, 320, 3), dtype=np.uint8) * 128
        detector.detect(frame)
        mask, regions = detector.detect(frame)
        self.assertEqual(len(regions), 0)

    def test_different_frames_detect_motion(self):
        """不同帧应检测到运动"""
        detector = FrameDiffDetector(threshold=20, min_area=100)
        frame1 = np.zeros((240, 320, 3), dtype=np.uint8)
        frame2 = np.zeros((240, 320, 3), dtype=np.uint8)
        # 在frame2中添加一个白色方块
        frame2[50:150, 100:200] = 255
        detector.detect(frame1)
        mask, regions = detector.detect(frame2)
        self.assertGreater(len(regions), 0)

    def test_mask_shape(self):
        """掩码形状应与输入一致"""
        detector = FrameDiffDetector()
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        mask, _ = detector.detect(frame)
        self.assertEqual(mask.shape[:2], (240, 320))

    def test_min_area_filter(self):
        """小面积应被过滤"""
        detector = FrameDiffDetector(threshold=20, min_area=5000)
        frame1 = np.zeros((240, 320, 3), dtype=np.uint8)
        frame2 = np.zeros((240, 320, 3), dtype=np.uint8)
        frame2[10:20, 10:20] = 255  # 小方块
        detector.detect(frame1)
        _, regions = detector.detect(frame2)
        self.assertEqual(len(regions), 0)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestMultiFrameDiffDetector(unittest.TestCase):
    """多帧差分法测试"""

    def test_needs_multiple_frames(self):
        """需要多帧才能检测"""
        detector = MultiFrameDiffDetector(num_frames=3, threshold=30, min_area=100)
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        _, regions = detector.detect(frame)
        self.assertEqual(len(regions), 0)

    def test_detection_after_enough_frames(self):
        """足够帧后应能检测"""
        detector = MultiFrameDiffDetector(num_frames=3, threshold=20, min_area=100)
        frame_black = np.zeros((240, 320, 3), dtype=np.uint8)
        frame_white = np.ones((240, 320, 3), dtype=np.uint8) * 255
        detector.detect(frame_black)
        detector.detect(frame_black)
        _, regions = detector.detect(frame_white)
        # 应检测到运动
        self.assertGreater(len(regions), 0)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestBgSubtractDetector(unittest.TestCase):
    """背景减除法测试"""

    def test_mog2_method(self):
        """MOG2方法应能初始化"""
        detector = BgSubtractDetector(method='mog2', min_area=500)
        frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        mask, regions = detector.detect(frame)
        self.assertIsNotNone(mask)

    def test_knn_method(self):
        """KNN方法应能初始化"""
        detector = BgSubtractDetector(method='knn', min_area=500)
        frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        mask, regions = detector.detect(frame)
        self.assertIsNotNone(mask)

    def test_invalid_method_raises(self):
        """无效方法应抛出异常"""
        with self.assertRaises(ValueError):
            BgSubtractDetector(method='invalid')


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestAccumulatedDiffDetector(unittest.TestCase):
    """累积帧差法测试"""

    def test_first_frame_returns_empty(self):
        """首帧应返回空结果"""
        detector = AccumulatedDiffDetector(threshold=30, min_area=500)
        frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        mask, regions = detector.detect(frame)
        self.assertEqual(len(regions), 0)
        self.assertEqual(np.max(mask), 0)

    def test_gradual_change(self):
        """渐变应能被检测"""
        detector = AccumulatedDiffDetector(threshold=20, alpha=0.1, min_area=100)
        # 初始背景
        bg = np.ones((240, 320, 3), dtype=np.uint8) * 128
        detector.detect(bg)
        # 逐步变化
        for i in range(5):
            frame = bg.copy()
            frame[50:150, 100:200] = 128 + (i + 1) * 25
            detector.detect(frame)
        # 最终应检测到运动
        final = bg.copy()
        final[50:150, 100:200] = 255
        _, regions = detector.detect(final)
        self.assertGreater(len(regions), 0)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestMotionDetector(unittest.TestCase):
    """综合运动检测器测试"""

    def test_frame_diff_method(self):
        """帧差法应正常工作"""
        detector = MotionDetector(method=MotionMethod.FRAME_DIFF, min_area=100)
        frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        mask, regions = detector.detect(frame)
        self.assertIsNotNone(mask)

    def test_mog2_method(self):
        """MOG2方法应正常工作"""
        detector = MotionDetector(method=MotionMethod.BG_SUBTRACT_MOG2, min_area=100)
        frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        mask, regions = detector.detect(frame)
        self.assertIsNotNone(mask)

    def test_invalid_method_raises(self):
        """无效方法应抛出异常"""
        with self.assertRaises((ValueError, KeyError)):
            MotionDetector(method="invalid_method")

    def test_reset(self):
        """重置应清除状态"""
        detector = MotionDetector(method=MotionMethod.FRAME_DIFF, min_area=100)
        frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        detector.detect(frame)
        detector.reset()
        # 重置后再检测首帧应无运动
        _, regions = detector.detect(frame)
        self.assertEqual(len(regions), 0)

    def test_visualize_returns_image(self):
        """可视化应返回图像"""
        detector = MotionDetector(method=MotionMethod.FRAME_DIFF, min_area=100)
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        mask = np.zeros((240, 320), dtype=np.uint8)
        regions = []
        vis = detector.visualize(frame, mask, regions)
        self.assertEqual(vis.shape, frame.shape)


@unittest.skipUnless(HAS_CV2, "需要安装 opencv-python")
class TestMotionDetectorEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_grayscale_input(self):
        """灰度图输入应正常"""
        detector = FrameDiffDetector(threshold=30, min_area=100)
        frame = np.random.randint(0, 255, (240, 320), dtype=np.uint8)
        mask, regions = detector.detect(frame)
        self.assertIsNotNone(mask)

    def test_small_image(self):
        """小图像应正常处理"""
        detector = FrameDiffDetector(threshold=30, min_area=10)
        frame = np.random.randint(0, 255, (32, 32, 3), dtype=np.uint8)
        mask, regions = detector.detect(frame)
        self.assertIsNotNone(mask)


if __name__ == '__main__':
    unittest.main()
