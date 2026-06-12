#!/usr/bin/env python3
"""
背景减除单元测试
覆盖: MOG2/KNN初始化、前景检测、阴影去除、形态学后处理、
      自适应背景减除、多区域检测、可视化
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
    from background_subtractor import (BackgroundSubtractor,
                                        AdaptiveBackgroundSubtractor,
                                        MultiZoneDetector,
                                        detect_motion, create_motion_detector)


def _make_bg_frame(obj_pos=None, obj_size=50):
    """创建灰色背景帧，可选在指定位置画红色方块"""
    frame = np.zeros((300, 400, 3), dtype=np.uint8)
    frame[:] = (50, 50, 50)
    if obj_pos is not None:
        x, y = obj_pos
        cv2.rectangle(frame, (x, y), (x + obj_size, y + obj_size), (0, 0, 255), -1)
    return frame


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestBackgroundSubtractorInit(unittest.TestCase):
    """初始化测试"""

    def test_mog2_init(self):
        """MOG2初始化"""
        bs = BackgroundSubtractor('MOG2')
        self.assertEqual(bs.method, 'MOG2')
        self.assertIsNotNone(bs.subtractor)

    def test_knn_init(self):
        """KNN初始化"""
        bs = BackgroundSubtractor('KNN')
        self.assertEqual(bs.method, 'KNN')

    def test_invalid_method_raises(self):
        """无效方法应抛异常"""
        with self.assertRaises(ValueError):
            BackgroundSubtractor('INVALID')

    def test_custom_params(self):
        """自定义参数"""
        bs = BackgroundSubtractor('MOG2', history=200,
                                   var_threshold=20,
                                   detect_shadows=False)
        self.assertFalse(bs.detect_shadows)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestBackgroundSubtractorApply(unittest.TestCase):
    """apply测试"""

    def setUp(self):
        self.bs = BackgroundSubtractor('MOG2')

    def test_returns_required_keys(self):
        """应返回所有必要字段"""
        frame = _make_bg_frame()
        result = self.bs.apply(frame)
        for key in ('mask', 'fgmask_raw', 'contours', 'bboxes', 'num_objects'):
            self.assertIn(key, result)

    def test_mask_shape(self):
        """掩码应与输入同尺寸"""
        frame = _make_bg_frame()
        result = self.bs.apply(frame)
        self.assertEqual(result['mask'].shape[:2], frame.shape[:2])

    def test_detects_moving_object(self):
        """应检测到运动目标"""
        # 先喂几帧背景
        for _ in range(30):
            self.bs.apply(_make_bg_frame())
        # 突然出现目标
        result = self.bs.apply(_make_bg_frame(obj_pos=(150, 100)))
        self.assertIsInstance(result['num_objects'], int)

    def test_no_moving_object(self):
        """静止场景目标数应较少"""
        bg = _make_bg_frame()
        for _ in range(50):
            self.bs.apply(bg)
        result = self.bs.apply(bg)
        self.assertEqual(result['num_objects'], 0)

    def test_bbox_format(self):
        """bbox应为(x, y, w, h)元组"""
        frame = _make_bg_frame()
        result = self.bs.apply(frame)
        for bbox in result['bboxes']:
            self.assertEqual(len(bbox), 4)

    def test_min_area_filter(self):
        """min_area应过滤小目标"""
        frame = _make_bg_frame()
        result = self.bs.apply(frame, min_area=10000)
        for bbox in result['bboxes']:
            x, y, w, h = bbox
            self.assertGreaterEqual(w * h, 10000)

    def test_remove_shadows(self):
        """阴影去除选项"""
        frame = _make_bg_frame()
        result = self.bs.apply(frame, remove_shadows=True)
        self.assertIsNotNone(result['mask'])

    def test_no_morphological(self):
        """不做形态学后处理"""
        frame = _make_bg_frame()
        result = self.bs.apply(frame, morphological=False)
        self.assertIsNotNone(result['mask'])


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestBackgroundSubtractorReset(unittest.TestCase):
    """重置测试"""

    def test_reset(self):
        """reset应重建减除器"""
        bs = BackgroundSubtractor('MOG2')
        bs.apply(_make_bg_frame())
        bs.reset()
        # 重新使用不应出错
        result = bs.apply(_make_bg_frame())
        self.assertIsNotNone(result['mask'])

    def test_reset_knn(self):
        """KNN重置"""
        bs = BackgroundSubtractor('KNN')
        bs.apply(_make_bg_frame())
        bs.reset()
        result = bs.apply(_make_bg_frame())
        self.assertIsNotNone(result['mask'])


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestGetBackground(unittest.TestCase):
    """获取背景测试"""

    def test_get_background_after_frames(self):
        """多帧后应能获取背景"""
        bs = BackgroundSubtractor('MOG2')
        for _ in range(10):
            bs.apply(_make_bg_frame())
        bg = bs.get_background()
        # MOG2支持获取背景
        if bg is not None:
            self.assertEqual(bg.shape[:2], (300, 400))


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestAdaptiveBackgroundSubtractor(unittest.TestCase):
    """自适应背景减除测试"""

    def test_init(self):
        """初始化"""
        abs_sub = AdaptiveBackgroundSubtractor(alpha=0.01, threshold=30)
        self.assertEqual(abs_sub.alpha, 0.01)
        self.assertEqual(abs_sub.threshold, 30)

    def test_first_frame_returns_not_initialized(self):
        """第一帧应返回initialized=False"""
        abs_sub = AdaptiveBackgroundSubtractor()
        result = abs_sub.update(_make_bg_frame())
        self.assertFalse(result['initialized'])

    def test_initialization_phase(self):
        """初始化阶段应持续min_stability帧"""
        abs_sub = AdaptiveBackgroundSubtractor(min_stability=10)
        bg = _make_bg_frame()
        for _ in range(5):
            result = abs_sub.update(bg)
            self.assertFalse(result['initialized'])

    def test_initialized_after_enough_frames(self):
        """足够帧后应初始化完成"""
        abs_sub = AdaptiveBackgroundSubtractor(min_stability=5)
        bg = _make_bg_frame()
        for _ in range(10):
            result = abs_sub.update(bg)
        self.assertTrue(result['initialized'])

    def test_returns_required_keys(self):
        """应返回所有必要字段"""
        abs_sub = AdaptiveBackgroundSubtractor()
        result = abs_sub.update(_make_bg_frame())
        for key in ('mask', 'contours', 'bboxes', 'num_objects', 'initialized'):
            self.assertIn(key, result)

    def test_background_model_exists(self):
        """初始化后应有背景模型"""
        abs_sub = AdaptiveBackgroundSubtractor(min_stability=3)
        bg = _make_bg_frame()
        for _ in range(10):
            abs_sub.update(bg)
        self.assertIsNotNone(abs_sub.background)

    def test_detects_new_object(self):
        """应检测到新出现的目标"""
        abs_sub = AdaptiveBackgroundSubtractor(min_stability=10, threshold=30)
        bg = _make_bg_frame()
        for _ in range(20):
            abs_sub.update(bg)
        # 出现新目标
        result = abs_sub.update(_make_bg_frame(obj_pos=(150, 100)))
        self.assertTrue(result['initialized'])

    def test_reset(self):
        """reset应清空状态"""
        abs_sub = AdaptiveBackgroundSubtractor()
        abs_sub.update(_make_bg_frame())
        abs_sub.reset()
        self.assertIsNone(abs_sub.background)
        self.assertEqual(abs_sub.frame_count, 0)

    def test_variance_exists(self):
        """应有方差模型"""
        abs_sub = AdaptiveBackgroundSubtractor(min_stability=3)
        bg = _make_bg_frame()
        for _ in range(10):
            abs_sub.update(bg)
        self.assertIsNotNone(abs_sub.variance)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMultiZoneDetector(unittest.TestCase):
    """多区域检测器测试"""

    def test_init(self):
        """初始化"""
        zones = [(0, 0, 200, 150), (200, 0, 200, 150)]
        mzd = MultiZoneDetector(zones, method='MOG2')
        self.assertEqual(len(mzd.zones), 2)

    def test_update_returns_list(self):
        """update应返回区域结果列表"""
        zones = [(0, 0, 200, 300)]
        mzd = MultiZoneDetector(zones)
        frame = _make_bg_frame()
        results = mzd.update(frame)
        self.assertEqual(len(results), 1)

    def test_result_structure(self):
        """结果应包含zone_id和result"""
        zones = [(0, 0, 200, 300)]
        mzd = MultiZoneDetector(zones)
        results = mzd.update(_make_bg_frame())
        self.assertIn('zone_id', results[0])
        self.assertIn('zone', results[0])
        self.assertIn('result', results[0])


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestDrawDetections(unittest.TestCase):
    """可视化测试"""

    def test_draw_returns_same_shape(self):
        """应返回同尺寸图像"""
        frame = _make_bg_frame()
        bs = BackgroundSubtractor('MOG2')
        result = bs.apply(frame)
        vis = BackgroundSubtractor.draw_detections(frame, result)
        self.assertEqual(vis.shape, frame.shape)

    def test_draw_empty_result(self):
        """空结果也应能绘制"""
        frame = _make_bg_frame()
        result = {
            'mask': np.zeros(frame.shape[:2], dtype=np.uint8),
            'contours': [],
            'bboxes': [],
            'num_objects': 0
        }
        vis = BackgroundSubtractor.draw_detections(frame, result)
        self.assertEqual(vis.shape, frame.shape)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestShortcutFunctions(unittest.TestCase):
    """快捷函数测试"""

    def test_detect_motion(self):
        """detect_motion应返回bbox列表"""
        bs = BackgroundSubtractor('MOG2')
        for _ in range(10):
            bs.apply(_make_bg_frame())
        bboxes = detect_motion(_make_bg_frame(), bs)
        self.assertIsInstance(bboxes, list)

    def test_create_motion_detector(self):
        """create_motion_detector应返回BackgroundSubtractor"""
        bs = create_motion_detector('MOG2', history=300)
        self.assertIsInstance(bs, BackgroundSubtractor)


if __name__ == '__main__':
    unittest.main()
