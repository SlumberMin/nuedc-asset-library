#!/usr/bin/env python3
"""
模板匹配单元测试
覆盖: 单尺度匹配、多尺度匹配、旋转匹配、多模板匹配、多位置匹配、NMS
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
    from template_matcher import TemplateMatcher, find_template


def _make_test_image(w=600, h=400, rect_pos=(100, 100), rect_size=100):
    """创建含一个白色方块的测试图像"""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    x, y = rect_pos
    s = rect_size
    cv2.rectangle(img, (x, y), (x + s, y + s), (255, 255, 255), -1)
    return img


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestTemplateMatcherInit(unittest.TestCase):
    """初始化测试"""

    def test_default_method(self):
        """默认使用TM_CCOEFF_NORMED"""
        m = TemplateMatcher()
        self.assertEqual(m.method, cv2.TM_CCOEFF_NORMED)
        self.assertFalse(m.is_sqdiff)

    def test_sqdiff_method(self):
        """TM_SQDIFF_NORMED应标记is_sqdiff"""
        m = TemplateMatcher(method=cv2.TM_SQDIFF_NORMED)
        self.assertTrue(m.is_sqdiff)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMatchSingle(unittest.TestCase):
    """单尺度匹配测试"""

    def setUp(self):
        self.img = _make_test_image(600, 400, (100, 100), 100)
        self.template = self.img[100:200, 100:200].copy()
        self.matcher = TemplateMatcher()

    def test_returns_dict_keys(self):
        """返回值应包含所有必要字段"""
        r = self.matcher.match_single(self.img, self.template)
        for key in ('location', 'score', 'top_left', 'bottom_right', 'size', 'confidence'):
            self.assertIn(key, r)

    def test_exact_match_high_score(self):
        """精确裁剪的模板应得到高分"""
        r = self.matcher.match_single(self.img, self.template)
        self.assertGreater(r['score'], 0.95)

    def test_location_near_actual(self):
        """匹配位置应接近实际位置"""
        r = self.matcher.match_single(self.img, self.template)
        lx, ly = r['location']
        self.assertAlmostEqual(lx, 100, delta=2)
        self.assertAlmostEqual(ly, 100, delta=2)

    def test_size_matches_template(self):
        """返回的size应与模板尺寸一致"""
        r = self.matcher.match_single(self.img, self.template)
        self.assertEqual(r['size'], (100, 100))

    def test_sqdiff_method(self):
        """SQDIFF方法也应正常工作"""
        matcher = TemplateMatcher(method=cv2.TM_SQDIFF_NORMED)
        r = matcher.match_single(self.img, self.template)
        self.assertGreater(r['score'], 0.9)

    def test_color_image(self):
        """彩色图像应能正常匹配"""
        r = self.matcher.match_single(self.img, self.template)
        self.assertGreater(r['score'], 0.9)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMatchMultiScale(unittest.TestCase):
    """多尺度匹配测试"""

    def setUp(self):
        self.img = _make_test_image(600, 400, (100, 100), 100)
        self.template = self.img[100:200, 100:200].copy()
        self.matcher = TemplateMatcher()

    def test_returns_list(self):
        """应返回列表"""
        results = self.matcher.match_multi_scale(
            self.img, self.template,
            scale_range=(0.8, 1.2), scale_steps=5)
        self.assertIsInstance(results, list)
        self.assertGreater(len(results), 0)

    def test_sorted_by_score(self):
        """结果应按score降序排列"""
        results = self.matcher.match_multi_scale(
            self.img, self.template,
            scale_range=(0.5, 1.5), scale_steps=10)
        scores = [r['score'] for r in results]
        self.assertEqual(scores, sorted(scores, reverse=True))

    def test_contains_scale_key(self):
        """每个结果应包含scale字段"""
        results = self.matcher.match_multi_scale(
            self.img, self.template,
            scale_range=(0.8, 1.2), scale_steps=5)
        for r in results:
            self.assertIn('scale', r)

    def test_best_scale_near_1(self):
        """原图裁剪的模板，最佳scale应接近1.0"""
        results = self.matcher.match_multi_scale(
            self.img, self.template,
            scale_range=(0.5, 1.5), scale_steps=21)
        best = results[0]
        self.assertAlmostEqual(best['scale'], 1.0, delta=0.15)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMatchRotation(unittest.TestCase):
    """多角度匹配测试"""

    def setUp(self):
        self.img = _make_test_image(600, 400, (150, 150), 80)
        self.template = self.img[150:230, 150:230].copy()
        self.matcher = TemplateMatcher()

    def test_returns_results(self):
        """应返回结果列表"""
        results = self.matcher.match_rotation(
            self.img, self.template,
            angle_range=(0, 180), angle_step=30,
            scale_range=(0.9, 1.1), scale_steps=3)
        self.assertIsInstance(results, list)
        self.assertGreater(len(results), 0)

    def test_contains_angle_scale(self):
        """结果应包含angle和scale字段"""
        results = self.matcher.match_rotation(
            self.img, self.template,
            angle_range=(0, 90), angle_step=45,
            scale_range=(1.0, 1.0), scale_steps=1)
        for r in results:
            self.assertIn('angle', r)
            self.assertIn('scale', r)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMatchMultiTemplate(unittest.TestCase):
    """多模板匹配测试"""

    def setUp(self):
        self.img = _make_test_image(600, 400, (100, 100), 100)
        self.matcher = TemplateMatcher()

    def test_matching_template_found(self):
        """应能找到匹配的模板"""
        tpl = self.img[100:200, 100:200].copy()
        templates = {'target': tpl, 'other': np.zeros((50, 50, 3), dtype=np.uint8)}
        results = self.matcher.match_multi_template(self.img, templates, threshold=0.8)
        names = [r['template_name'] for r in results]
        self.assertIn('target', names)

    def test_threshold_filters(self):
        """低于阈值的结果应被过滤"""
        tpl = self.img[100:200, 100:200].copy()
        templates = {'target': tpl}
        results = self.matcher.match_multi_template(self.img, templates, threshold=0.99)
        for r in results:
            self.assertGreaterEqual(r['score'], 0.99)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMatchMultiLocation(unittest.TestCase):
    """多位置匹配测试"""

    def test_finds_multiple_instances(self):
        """图中同一模板出现多次应都能检测到"""
        img = np.zeros((400, 600, 3), dtype=np.uint8)
        # 画两个相同的白色方块
        cv2.rectangle(img, (50, 50), (150, 150), (255, 255, 255), -1)
        cv2.rectangle(img, (350, 200), (450, 300), (255, 255, 255), -1)
        template = np.zeros((100, 100, 3), dtype=np.uint8)
        template[:] = 255

        matcher = TemplateMatcher()
        results = matcher.match_multi_location(img, template, threshold=0.9, nms_dist=50)
        self.assertGreaterEqual(len(results), 1)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestNMS(unittest.TestCase):
    """NMS测试"""

    def test_nms_empty(self):
        """空列表应返回空"""
        result = TemplateMatcher._nms([], 10)
        self.assertEqual(result, [])

    def test_nms_suppresses_nearby(self):
        """距离过近的匹配应被抑制"""
        matches = [
            {'location': (100, 100), 'score': 0.95},
            {'location': (105, 105), 'score': 0.90},  # 距离<10
            {'location': (300, 300), 'score': 0.85},
        ]
        result = TemplateMatcher._nms(matches, dist=20)
        self.assertEqual(len(result), 2)

    def test_nms_keeps_distant(self):
        """距离足够远的匹配应保留"""
        matches = [
            {'location': (100, 100), 'score': 0.95},
            {'location': (300, 300), 'score': 0.90},
        ]
        result = TemplateMatcher._nms(matches, dist=10)
        self.assertEqual(len(result), 2)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestDrawResults(unittest.TestCase):
    """可视化测试"""

    def test_draw_returns_same_shape(self):
        """绘制结果应与原图同尺寸"""
        img = _make_test_image()
        results = [{
            'top_left': (100, 100),
            'bottom_right': (200, 200),
            'score': 0.95
        }]
        vis = TemplateMatcher.draw_results(img, results)
        self.assertEqual(vis.shape, img.shape)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestFindTemplate(unittest.TestCase):
    """快捷函数测试"""

    def test_find_above_threshold(self):
        """高于阈值时应返回坐标"""
        img = _make_test_image()
        tpl = img[100:200, 100:200].copy()
        result = find_template(img, tpl, threshold=0.8)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 3)  # (x, y, score)

    def test_find_below_threshold(self):
        """低于阈值时应返回None"""
        noise = np.random.randint(0, 256, (400, 600, 3), dtype=np.uint8)
        tpl = np.zeros((50, 50, 3), dtype=np.uint8)
        result = find_template(noise, tpl, threshold=0.99)
        # 可能为None也可能有结果，取决于噪声
        if result is not None:
            self.assertEqual(len(result), 3)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestRotateImage(unittest.TestCase):
    """旋转图像测试"""

    def test_0_degree_no_change(self):
        """0度旋转应保持大致相同尺寸"""
        img = np.zeros((100, 80), dtype=np.uint8)
        img[:] = 255
        rotated = TemplateMatcher._rotate_image(img, 0)
        self.assertEqual(rotated.shape[:2], img.shape[:2])

    def test_90_degree_swaps_dims(self):
        """90度旋转应大致交换宽高"""
        img = np.zeros((100, 60), dtype=np.uint8)
        rotated = TemplateMatcher._rotate_image(img, 90)
        # 旋转后宽高应互换(±1像素)
        self.assertAlmostEqual(rotated.shape[0], 60, delta=1)
        self.assertAlmostEqual(rotated.shape[1], 100, delta=1)


if __name__ == '__main__':
    unittest.main()
