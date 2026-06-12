#!/usr/bin/env python3
"""
颜色直方图模块单元测试
覆盖: HS直方图、H直方图、RGB直方图、直方图比较、模板匹配、颜色模型、
      反向投影、颜色分类、主色调、颜色占比、直方图绘制
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from _10_视觉通用代码库.color_histogram import ColorHistogramAnalyzer


def make_solid_image(bgr, size=100):
    img = np.zeros((size, size, 3), dtype=np.uint8)
    img[:] = bgr
    return img


class TestInit(unittest.TestCase):
    def test_defaults(self):
        a = ColorHistogramAnalyzer()
        self.assertEqual(a.h_bins, 30)
        self.assertEqual(a.s_bins, 32)

    def test_custom(self):
        a = ColorHistogramAnalyzer(h_bins=60, s_bins=64)
        self.assertEqual(a.h_bins, 60)
        self.assertEqual(a.s_bins, 64)


class TestCalcHSHistogram(unittest.TestCase):
    def test_shape(self):
        a = ColorHistogramAnalyzer(h_bins=30, s_bins=32)
        hist = a.calc_hs_histogram(make_solid_image((0, 0, 255)))
        self.assertEqual(hist.shape, (30, 32))

    def test_normalized(self):
        """归一化后最大值应为1"""
        a = ColorHistogramAnalyzer()
        hist = a.calc_hs_histogram(make_solid_image((0, 0, 255)))
        self.assertAlmostEqual(float(np.max(hist)), 1.0, places=5)

    def test_with_mask(self):
        a = ColorHistogramAnalyzer()
        img = make_solid_image((255, 0, 0))
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[25:75, 25:75] = 255
        hist = a.calc_hs_histogram(img, mask)
        self.assertEqual(hist.shape[0], a.h_bins)


class TestCalcHHistogram(unittest.TestCase):
    def test_shape(self):
        a = ColorHistogramAnalyzer(h_bins=30)
        hist = a.calc_h_histogram(make_solid_image((0, 0, 255)))
        self.assertEqual(hist.shape, (30,))


class TestCalcRGBHistogram(unittest.TestCase):
    def test_returns_three(self):
        a = ColorHistogramAnalyzer()
        hists = a.calc_rgb_histogram(make_solid_image((100, 150, 200)))
        self.assertEqual(len(hists), 3)

    def test_each_normalized(self):
        a = ColorHistogramAnalyzer()
        hists = a.calc_rgb_histogram(make_solid_image((0, 0, 255)))
        for h in hists:
            self.assertAlmostEqual(float(np.max(h)), 1.0, places=5)


class TestCompareHistograms(unittest.TestCase):
    def test_self_similarity_bhattacharyya(self):
        """自身比较巴氏距离应为0"""
        a = ColorHistogramAnalyzer()
        hist = a.calc_hs_histogram(make_solid_image((0, 0, 255)))
        dist = a.compare_histograms(hist, hist, 'bhattacharyya')
        self.assertAlmostEqual(dist, 0.0, places=3)

    def test_self_correlation(self):
        """自身比较相关性应为1"""
        a = ColorHistogramAnalyzer()
        hist = a.calc_hs_histogram(make_solid_image((0, 0, 255)))
        corr = a.compare_histograms(hist, hist, 'correlation')
        self.assertAlmostEqual(corr, 1.0, places=3)

    def test_different_colors(self):
        """红蓝应有差异"""
        a = ColorHistogramAnalyzer()
        red_hist = a.calc_hs_histogram(make_solid_image((0, 0, 255)))
        blue_hist = a.calc_hs_histogram(make_solid_image((255, 0, 0)))
        dist = a.compare_histograms(red_hist, blue_hist, 'bhattacharyya')
        self.assertGreater(dist, 0.1)

    def test_all_methods(self):
        a = ColorHistogramAnalyzer()
        h1 = a.calc_hs_histogram(make_solid_image((0, 0, 255)))
        h2 = a.calc_hs_histogram(make_solid_image((255, 0, 0)))
        for method in ['correlation', 'chi_square', 'intersection', 'bhattacharyya']:
            result = a.compare_histograms(h1, h2, method)
            self.assertIsNotNone(result)


class TestMatchTemplateByHistogram(unittest.TestCase):
    def test_self_match(self):
        a = ColorHistogramAnalyzer()
        img = make_solid_image((0, 0, 255))
        hist = a.calc_hs_histogram(img)
        score = a.match_template_by_histogram(hist, img)
        self.assertGreater(score, 0.9)


class TestCreateColorModel(unittest.TestCase):
    def test_single_image(self):
        a = ColorHistogramAnalyzer()
        imgs = [make_solid_image((0, 0, 255))]
        model = a.create_color_model(imgs)
        self.assertEqual(model.shape, (a.h_bins, a.s_bins))

    def test_multiple_images(self):
        a = ColorHistogramAnalyzer()
        imgs = [make_solid_image((0, 0, 255)), make_solid_image((0, 0, 200))]
        model = a.create_color_model(imgs)
        self.assertEqual(model.shape, (a.h_bins, a.s_bins))


class TestBackproject(unittest.TestCase):
    def test_returns_correct_shape(self):
        a = ColorHistogramAnalyzer()
        img = make_solid_image((0, 0, 255))
        hist = a.calc_hs_histogram(img)
        bp = a.backproject(img, hist)
        self.assertEqual(bp.shape, (100, 100))


class TestFindColorRegions(unittest.TestCase):
    def test_returns_mask_and_contours(self):
        a = ColorHistogramAnalyzer()
        img = make_solid_image((0, 0, 255))
        hist = a.calc_hs_histogram(img)
        mask, contours = a.find_color_regions(img, hist, threshold=50)
        self.assertEqual(mask.shape[:2], img.shape[:2])


class TestClassifyByColor(unittest.TestCase):
    def test_best_match(self):
        a = ColorHistogramAnalyzer()
        red = make_solid_image((0, 0, 255))
        blue = make_solid_image((255, 0, 0))
        models = {
            'red': a.calc_hs_histogram(red),
            'blue': a.calc_hs_histogram(blue),
        }
        best, scores = a.classify_by_color(red, models)
        self.assertEqual(best, 'red')
        self.assertIn('red', scores)
        self.assertIn('blue', scores)


class TestGetDominantColorHSV(unittest.TestCase):
    def test_red_dominant(self):
        a = ColorHistogramAnalyzer()
        h, s, v = a.get_dominant_color_hsv(make_solid_image((0, 0, 255)))
        # BGR红色 -> HSV H≈0
        self.assertTrue(h <= 10 or h >= 170)

    def test_blue_dominant(self):
        a = ColorHistogramAnalyzer()
        h, s, v = a.get_dominant_color_hsv(make_solid_image((255, 0, 0)))
        # BGR蓝色 -> HSV H≈110-130
        self.assertTrue(90 <= h <= 140)


class TestGetColorProportions(unittest.TestCase):
    def test_half_half(self):
        a = ColorHistogramAnalyzer()
        img = np.zeros((100, 200, 3), dtype=np.uint8)
        img[:, :100] = (0, 0, 255)   # 红
        img[:, 100:] = (255, 0, 0)   # 蓝
        ranges = {
            'red': (np.array([0, 100, 100]), np.array([10, 255, 255])),
            'blue': (np.array([100, 100, 100]), np.array([130, 255, 255])),
        }
        props = a.get_color_proportions(img, ranges)
        self.assertIn('red', props)
        self.assertIn('blue', props)
        self.assertAlmostEqual(props['red'], 0.5, delta=0.15)


class TestDrawHistogram(unittest.TestCase):
    def test_returns_image(self):
        a = ColorHistogramAnalyzer()
        img = make_solid_image((0, 0, 255))
        hist = a.calc_h_histogram(img)
        canvas = a.draw_histogram(img, hist, hist_h=200, hist_w=512, channel=0)
        self.assertEqual(canvas.shape, (200, 512, 3))


if __name__ == '__main__':
    unittest.main()
