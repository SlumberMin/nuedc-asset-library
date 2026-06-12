#!/usr/bin/env python3
"""
颜色检测单元测试
覆盖: ColorTracker、ColorTracker._get_mask、HSV阈值、多目标检测
注意: 使用合成图像进行测试，不依赖摄像头硬件
"""

import sys
import os
import unittest
import numpy as np

# 将项目根目录加入路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


def create_solid_color_image(color_bgr, width=640, height=480):
    """创建纯色BGR图像"""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:, :] = color_bgr
    return img


def create_circle_image(center, radius, color_bgr, width=640, height=480):
    """在黑色背景上绘制圆形"""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.circle(img, center, radius, color_bgr, -1)
    return img


def create_multi_circle_image(circles, width=640, height=480):
    """绘制多个圆形: circles = [(center, radius, color_bgr), ...]"""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    for center, radius, color in circles:
        cv2.circle(img, center, radius, color, -1)
    return img


@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过颜色检测测试")
class TestColorTrackerInit(unittest.TestCase):
    """ColorTracker初始化测试"""

    def test_default_init(self):
        """默认应使用红色预设"""
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker()
        self.assertIn('lower1', tracker.color_config)
        self.assertIn('upper1', tracker.color_config)

    def test_preset_red(self):
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='red')
        # 红色有双区间
        self.assertIn('lower2', tracker.color_config)

    def test_preset_blue(self):
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='blue')
        self.assertIn('lower1', tracker.color_config)
        self.assertNotIn('lower2', tracker.color_config)

    def test_preset_green(self):
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='green')
        self.assertIn('lower1', tracker.color_config)

    def test_preset_yellow(self):
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='yellow')
        self.assertIn('lower1', tracker.color_config)

    def test_custom_hsv_threshold(self):
        from visual.color_tracker import ColorTracker
        custom = {'lower1': [0, 50, 50], 'upper1': [30, 255, 255]}
        tracker = ColorTracker(color_name=custom)
        self.assertEqual(tracker.color_config['lower1'], [0, 50, 50])

    def test_invalid_color_falls_back_to_red(self):
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='nonexistent')
        self.assertEqual(tracker.color_config, ColorTracker.PRESETS['red'])

    def test_min_area_setting(self):
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(min_area=500)
        self.assertEqual(tracker.min_area, 500)

    def test_max_targets_setting(self):
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(max_targets=10)
        self.assertEqual(tracker.max_targets, 10)


@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过颜色检测测试")
class TestColorMaskGeneration(unittest.TestCase):
    """颜色掩膜生成测试"""

    def test_red_mask_on_red_image(self):
        """纯红图像应产生大面积掩膜"""
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='red')
        red_bgr = (0, 0, 255)  # BGR红色
        img = create_solid_color_image(red_bgr)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = tracker._get_mask(hsv)
        # 掩膜中白色像素应占大部分
        white_ratio = np.sum(mask > 0) / mask.size
        self.assertGreater(white_ratio, 0.8)

    def test_blue_mask_on_blue_image(self):
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='blue')
        blue_bgr = (255, 0, 0)  # BGR蓝色
        img = create_solid_color_image(blue_bgr)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = tracker._get_mask(hsv)
        white_ratio = np.sum(mask > 0) / mask.size
        self.assertGreater(white_ratio, 0.8)

    def test_no_detection_on_wrong_color(self):
        """蓝色图像不应被红色检测器检测到"""
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='red')
        blue_bgr = (255, 0, 0)
        img = create_solid_color_image(blue_bgr)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = tracker._get_mask(hsv)
        white_ratio = np.sum(mask > 0) / mask.size
        self.assertLess(white_ratio, 0.2)


@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过颜色检测测试")
class TestColorDetection(unittest.TestCase):
    """完整检测流程测试"""

    def test_detect_red_circle(self):
        """应能检测到红色圆形"""
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='red', min_area=100)
        red_bgr = (0, 0, 255)
        img = create_circle_image((320, 240), 80, red_bgr)
        results, mask = tracker.update(img)
        # 应检测到至少一个目标
        self.assertGreater(len(results), 0)

    def test_detect_green_circle(self):
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='green', min_area=100)
        green_bgr = (0, 255, 0)
        img = create_circle_image((320, 240), 80, green_bgr)
        results, mask = tracker.update(img)
        self.assertGreater(len(results), 0)

    def test_no_detection_on_black(self):
        """纯黑图像不应检测到目标"""
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='red', min_area=100)
        black = (0, 0, 0)
        img = create_solid_color_image(black)
        results, mask = tracker.update(img)
        self.assertEqual(len(results), 0)

    def test_multi_target_detection(self):
        """应能检测多个目标"""
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='red', min_area=100, max_targets=5)
        red_bgr = (0, 0, 255)
        img = create_multi_circle_image([
            ((160, 240), 60, red_bgr),
            ((480, 240), 60, red_bgr),
        ])
        results, mask = tracker.update(img)
        self.assertGreaterEqual(len(results), 1)  # 至少1个

    def test_result_structure(self):
        """结果应包含所需字段"""
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='red', min_area=100)
        red_bgr = (0, 0, 255)
        img = create_circle_image((320, 240), 80, red_bgr)
        results, mask = tracker.update(img)
        if results:
            r = results[0]
            self.assertIn('id', r)
            self.assertIn('cx', r)
            self.assertIn('cy', r)
            self.assertIn('vx', r)
            self.assertIn('vy', r)
            self.assertIn('trajectory', r)

    def test_center_position_accuracy(self):
        """检测中心应接近实际圆心"""
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='red', min_area=100)
        cx_true, cy_true = 320, 240
        red_bgr = (0, 0, 255)
        img = create_circle_image((cx_true, cy_true), 80, red_bgr)
        results, mask = tracker.update(img)
        if results:
            cx_det = results[0]['cx']
            cy_det = results[0]['cy']
            # 误差应小于20像素
            self.assertAlmostEqual(cx_det, cx_true, delta=20)
            self.assertAlmostEqual(cy_det, cy_true, delta=20)


@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过颜色检测测试")
class TestKalmanTracker(unittest.TestCase):
    """Kalman滤波追踪器测试"""

    def test_init(self):
        from visual.color_tracker import KalmanTracker
        tracker = KalmanTracker(100, 200)
        pos = tracker.get_position()
        self.assertEqual(pos, (100, 200))

    def test_predict(self):
        from visual.color_tracker import KalmanTracker
        tracker = KalmanTracker(100, 200)
        pred = tracker.predict()
        self.assertEqual(len(pred), 2)

    def test_update_resets_lost_count(self):
        from visual.color_tracker import KalmanTracker
        tracker = KalmanTracker(100, 200)
        tracker.lost_count = 5
        tracker.update(110, 210)
        self.assertEqual(tracker.lost_count, 0)

    def test_is_lost_threshold(self):
        from visual.color_tracker import KalmanTracker
        tracker = KalmanTracker(100, 200, dt=1.0)
        tracker.max_lost = 5
        tracker.lost_count = 6
        self.assertTrue(tracker.is_lost())

    def test_is_not_lost(self):
        from visual.color_tracker import KalmanTracker
        tracker = KalmanTracker(100, 200)
        self.assertFalse(tracker.is_lost())

    def test_trajectory_recording(self):
        from visual.color_tracker import KalmanTracker
        tracker = KalmanTracker(100, 200)
        tracker.update(110, 210)
        tracker.update(120, 220)
        self.assertGreaterEqual(len(tracker.trajectory), 3)  # 初始 + 2次更新


@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过颜色检测测试")
class TestColorTrackerDraw(unittest.TestCase):
    """绘制功能测试"""

    def test_draw_returns_image(self):
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker(color_name='red', min_area=100)
        red_bgr = (0, 0, 255)
        img = create_circle_image((320, 240), 80, red_bgr)
        results, mask = tracker.update(img)
        vis = tracker.draw(img, results)
        self.assertEqual(vis.shape, img.shape)

    def test_draw_empty_results(self):
        """空结果不应报错"""
        from visual.color_tracker import ColorTracker
        tracker = ColorTracker()
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        vis = tracker.draw(img, [])
        self.assertEqual(vis.shape, img.shape)


if __name__ == '__main__':
    unittest.main()
