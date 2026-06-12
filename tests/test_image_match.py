#!/usr/bin/env python3
"""
模板匹配单元测试 (image_match.py)
覆盖: 单尺度匹配 / 多尺度匹配 / 旋转匹配 / 多目标匹配 / 可视化 / 亮度归一化
测试对象: 10_视觉通用代码库/image_match.py
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
    from _10_视觉通用代码库.image_match import (
        match_template, match_template_visual, match_template_multiscale,
        match_template_rotation, match_template_multi,
        match_template_multi_visual, normalize_brightness,
    )


def make_scene_with_rect(pos=(50, 60), size=(80, 60), scene_size=300):
    """在场景中嵌入一个矩形目标"""
    scene = np.zeros((scene_size, scene_size), dtype=np.uint8)
    x, y = pos
    w, h = size
    scene[y:y + h, x:x + w] = 200
    template = scene[y:y + h, x:x + w].copy()
    return scene, template


def make_bgr_scene(pos=(50, 60), size=(80, 60), scene_size=300):
    scene, tpl = make_scene_with_rect(pos, size, scene_size)
    return cv2.cvtColor(scene, cv2.COLOR_GRAY2BGR), cv2.cvtColor(tpl, cv2.COLOR_GRAY2BGR)


def make_multi_target_scene(scene_size=400, tpl_size=50):
    """场景中嵌入多个相同目标"""
    scene = np.zeros((scene_size, scene_size), dtype=np.uint8)
    positions = [(50, 50), (200, 100), (100, 250)]
    for (x, y) in positions:
        scene[y:y + tpl_size, x:x + tpl_size] = 200
    template = np.zeros((tpl_size, tpl_size), dtype=np.uint8)
    template[:, :] = 200
    return scene, template, positions


# ── 单尺度模板匹配 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMatchTemplate(unittest.TestCase):

    def test_finds_exact_match(self):
        scene, tpl = make_scene_with_rect()
        tl, br, score, result = match_template(scene, tpl)
        self.assertGreater(score, 0.99)

    def test_returns_four_values(self):
        scene, tpl = make_scene_with_rect()
        ret = match_template(scene, tpl)
        self.assertEqual(len(ret), 4)

    def test_bgr_input(self):
        scene, tpl = make_bgr_scene()
        tl, br, score, _ = match_template(scene, tpl)
        self.assertGreater(score, 0.99)

    def test_different_methods(self):
        scene, tpl = make_scene_with_rect()
        for method in ['sqdiff', 'sqdiff_normed', 'ccorr', 'ccorr_normed',
                        'ccoeff', 'ccoeff_normed']:
            tl, br, score, _ = match_template(scene, tpl, method=method)
            self.assertIsInstance(score, (float, np.floating))

    def test_bottom_right_correct(self):
        scene, tpl = make_scene_with_rect(pos=(50, 60), size=(80, 60))
        tl, br, score, _ = match_template(scene, tpl)
        self.assertEqual(br[0] - tl[0], 80)
        self.assertEqual(br[1] - tl[1], 60)


# ── 可视化 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMatchTemplateVisual(unittest.TestCase):

    def test_returns_canvas_and_coords(self):
        scene, tpl = make_scene_with_rect()
        canvas, tl, br, score = match_template_visual(scene, tpl)
        self.assertEqual(canvas.shape, scene.shape + (3,) if len(scene.shape) == 2 else scene.shape)

    def test_score_in_range(self):
        scene, tpl = make_scene_with_rect()
        _, _, score, _ = match_template_visual(scene, tpl)
        self.assertGreater(score, 0.9)


# ── 多尺度匹配 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMatchMultiscale(unittest.TestCase):

    def test_finds_same_scale(self):
        """scale=1.0时应找到精确匹配"""
        scene, tpl = make_scene_with_rect()
        tl, br, score, scale = match_template_multiscale(
            scene, tpl, scale_range=(0.8, 1.2, 0.1))
        self.assertIsNotNone(tl)
        self.assertAlmostEqual(scale, 1.0, delta=0.15)

    def test_finds_scaled_target(self):
        """缩小模板应能在场景中找到"""
        scene = np.zeros((300, 300), dtype=np.uint8)
        scene[100:150, 100:150] = 200
        tpl = np.zeros((80, 80), dtype=np.uint8)
        tpl[:, :] = 200
        tl, br, score, scale = match_template_multiscale(
            scene, tpl, scale_range=(0.3, 1.0, 0.1))
        self.assertIsNotNone(tl)
        self.assertLess(scale, 1.0)


# ── 旋转匹配 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMatchRotation(unittest.TestCase):

    def test_finds_zero_rotation(self):
        """0度旋转应直接匹配"""
        scene, tpl = make_scene_with_rect(pos=(80, 80), size=(60, 60), scene_size=300)
        tl, br, score, angle = match_template_rotation(
            scene, tpl, angle_range=(0, 30, 10))
        self.assertIsNotNone(tl)
        self.assertGreater(score, 0.8)


# ── 多目标匹配 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMatchMulti(unittest.TestCase):

    def test_finds_multiple(self):
        scene, tpl, positions = make_multi_target_scene()
        matches = match_template_multi(scene, tpl, threshold=0.8)
        self.assertGreater(len(matches), 0)

    def test_returns_rect_score_pairs(self):
        scene, tpl, _ = make_multi_target_scene()
        matches = match_template_multi(scene, tpl, threshold=0.8)
        for rect, score in matches:
            self.assertEqual(len(rect), 4)  # (x, y, w, h)
            self.assertGreater(score, 0.0)

    def test_threshold_affects_count(self):
        scene, tpl, _ = make_multi_target_scene()
        m1 = match_template_multi(scene, tpl, threshold=0.5)
        m2 = match_template_multi(scene, tpl, threshold=0.95)
        self.assertGreaterEqual(len(m1), len(m2))


# ── 多目标可视化 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestMatchMultiVisual(unittest.TestCase):

    def test_returns_canvas_and_results(self):
        scene, tpl, _ = make_multi_target_scene()
        canvas, results = match_template_multi_visual(scene, tpl, threshold=0.8)
        self.assertEqual(canvas.shape, scene.shape + (3,) if len(scene.shape) == 2 else scene.shape)
        self.assertIsInstance(results, list)


# ── 亮度归一化 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestNormalizeBrightness(unittest.TestCase):

    def test_returns_two_images(self):
        img = np.random.randint(0, 256, (100, 100), dtype=np.uint8)
        tpl = np.random.randint(50, 200, (50, 50), dtype=np.uint8)
        n_img, n_tpl = normalize_brightness(img, tpl)
        self.assertEqual(n_img.shape, img.shape)
        self.assertEqual(n_tpl.shape, tpl.shape)

    def test_dtype_uint8(self):
        img = np.random.randint(0, 256, (100, 100), dtype=np.uint8)
        tpl = np.random.randint(0, 256, (50, 50), dtype=np.uint8)
        n_img, n_tpl = normalize_brightness(img, tpl)
        self.assertEqual(n_img.dtype, np.uint8)
        self.assertEqual(n_tpl.dtype, np.uint8)

    def test_reduces_brightness_difference(self):
        """归一化后两张图像的均值应更接近"""
        bright = np.full((100, 100), 200, dtype=np.uint8)
        dark = np.full((100, 100), 50, dtype=np.uint8)
        n_bright, n_dark = normalize_brightness(bright, dark)
        diff_before = abs(int(bright.mean()) - int(dark.mean()))
        diff_after = abs(int(n_bright.mean()) - int(n_dark.mean()))
        self.assertLess(diff_after, diff_before)


if __name__ == '__main__':
    unittest.main()
