#!/usr/bin/env python3
"""
连通域分析单元测试
覆盖: 标记、统计、过滤、面积/宽高比/实心度过滤
测试对象: 10_视觉通用代码库/connected_components.py
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
    from _10_视觉通用代码库.connected_components import (
        label_components, label_components_visual,
        get_component_stats, get_component_contours,
        filter_by_area, filter_by_bbox, filter_by_aspect_ratio,
        filter_by_solidity, filter_components
    )


def make_multi_object_binary(size=200):
    """创建含多个白色目标的二值图"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(img, (50, 50), 20, 255, -1)       # 小圆
    cv2.circle(img, (150, 150), 35, 255, -1)      # 大圆
    cv2.rectangle(img, (100, 20), (170, 60), 255, -1)  # 矩形
    return img


def make_single_object_binary(size=200):
    """创建含单个目标的二值图"""
    img = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(img, (100, 100), 50, 255, -1)
    return img


def make_empty_binary(size=200):
    """创建全黑二值图"""
    return np.zeros((size, size), dtype=np.uint8)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLabelComponents(unittest.TestCase):
    """连通域标记测试"""

    def test_output_shape(self):
        binary = make_multi_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        self.assertEqual(labels.shape, binary.shape)

    def test_detects_objects(self):
        binary = make_multi_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        # 背景 + 3个目标 = 4
        self.assertGreaterEqual(num_labels, 4)

    def test_background_label(self):
        """背景应为label 0"""
        binary = make_multi_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        # 左上角(0,0)是黑色背景
        self.assertEqual(labels[0, 0], 0)

    def test_stats_shape(self):
        binary = make_multi_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        self.assertEqual(stats.shape[0], num_labels)
        self.assertEqual(stats.shape[1], 5)  # x, y, w, h, area

    def test_centroids_shape(self):
        binary = make_multi_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        self.assertEqual(centroids.shape, (num_labels, 2))

    def test_empty_image(self):
        """全黑图像应只有一个标签（背景）"""
        binary = make_empty_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        self.assertEqual(num_labels, 1)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLabelComponentsVisual(unittest.TestCase):
    """可视化标记测试"""

    def test_output_shape(self):
        binary = make_multi_object_binary()
        color_map, num_labels, labels, stats, centroids = label_components_visual(binary)
        self.assertEqual(color_map.shape[:2], binary.shape)
        self.assertEqual(color_map.shape[2], 3)

    def test_background_is_black(self):
        binary = make_multi_object_binary()
        color_map, *_ = label_components_visual(binary)
        np.testing.assert_array_equal(color_map[0, 0], [0, 0, 0])


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGetComponentStats(unittest.TestCase):
    """连通域统计测试"""

    def test_excludes_background(self):
        binary = make_multi_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        components = get_component_stats(stats, centroids, num_labels)
        # 不应包含背景
        for c in components:
            self.assertGreater(c['label'], 0)

    def test_has_area(self):
        binary = make_single_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        components = get_component_stats(stats, centroids, num_labels)
        self.assertGreater(len(components), 0)
        self.assertGreater(components[0]['area'], 0)

    def test_has_center(self):
        binary = make_single_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        components = get_component_stats(stats, centroids, num_labels)
        cx, cy = components[0]['center']
        self.assertGreater(cx, 0)
        self.assertGreater(cy, 0)

    def test_aspect_ratio(self):
        binary = make_multi_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        components = get_component_stats(stats, centroids, num_labels)
        for c in components:
            self.assertGreater(c['aspect_ratio'], 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGetComponentContours(unittest.TestCase):
    """连通域轮廓测试"""

    def test_returns_dict(self):
        binary = make_multi_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        contours_dict = get_component_contours(labels, num_labels)
        self.assertIsInstance(contours_dict, dict)

    def test_has_contours(self):
        binary = make_single_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        contours_dict = get_component_contours(labels, num_labels)
        self.assertGreater(len(contours_dict), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterByArea(unittest.TestCase):
    """面积过滤测试"""

    def test_filters_small(self):
        binary = make_multi_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        valid = filter_by_area(stats, num_labels, min_area=1000)
        # 大面积目标应被保留
        self.assertGreater(len(valid), 0)

    def test_min_area_only(self):
        binary = make_multi_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        valid = filter_by_area(stats, num_labels, min_area=0)
        # 所有前景目标都应保留
        self.assertEqual(len(valid), num_labels - 1)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterByBbox(unittest.TestCase):
    """包围框过滤测试"""

    def test_filter(self):
        binary = make_multi_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        valid = filter_by_bbox(stats, num_labels, min_w=10, min_h=10)
        self.assertIsInstance(valid, list)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterByAspectRatio(unittest.TestCase):
    """宽高比过滤测试"""

    def test_filter(self):
        binary = make_multi_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        valid = filter_by_aspect_ratio(stats, num_labels, min_ratio=0.3, max_ratio=3.0)
        self.assertIsInstance(valid, list)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterBySolidity(unittest.TestCase):
    """实心度过滤测试"""

    def test_circle_high_solidity(self):
        """圆形应有高实心度"""
        binary = make_single_object_binary()
        num_labels, labels, stats, centroids = label_components(binary)
        valid = filter_by_solidity(labels, num_labels, min_solidity=0.5)
        self.assertGreater(len(valid), 0)


@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestFilterComponents(unittest.TestCase):
    """综合过滤测试"""

    def test_output_shape(self):
        binary = make_multi_object_binary()
        mask, valid, num_labels, stats, centroids = filter_components(
            binary, min_area=100, min_w=5, min_h=5)
        self.assertEqual(mask.shape, binary.shape)

    def test_mask_is_binary(self):
        binary = make_multi_object_binary()
        mask, valid, *_ = filter_components(binary, min_area=100)
        unique = set(np.unique(mask))
        self.assertTrue(unique.issubset({0, 255}))

    def test_filters_small_area(self):
        binary = make_multi_object_binary()
        _, valid_loose, *_ = filter_components(binary, min_area=100)
        _, valid_strict, *_ = filter_components(binary, min_area=5000)
        self.assertLessEqual(len(valid_strict), len(valid_loose))


if __name__ == '__main__':
    unittest.main()
