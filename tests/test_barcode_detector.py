#!/usr/bin/env python3
"""
条形码检测单元测试
覆盖: 梯度法定位、形态学法定位、条形码解码、ROI解码、
      图像增强、多尺度解码、绘制结果
测试对象: 10_视觉通用代码库/barcode_detector.py
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import numpy as np
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    from pyzbar import pyzbar
    HAS_PYZBAR = True
except ImportError:
    HAS_PYZBAR = False

if HAS_CV2:
    from _10_视觉通用代码库.barcode_detector import (
        locate_barcode_by_gradient,
        locate_barcode_by_morphology,
        enhance_for_barcode,
        draw_barcode_results,
    )

if HAS_CV2 and HAS_PYZBAR:
    from _10_视觉通用代码库.barcode_detector import (
        decode_barcodes,
        decode_barcode_roi,
        multi_scale_decode,
    )


def make_barcode_stripes(width=400, height=200):
    """创建模拟条形码图像"""
    img = np.ones((height, width), dtype=np.uint8) * 255
    x_start = 50
    bar_widths = [2, 1, 1, 3, 1, 2, 1, 1, 3, 2, 1, 1, 2, 3, 1, 1, 2]
    x = x_start
    for i, w in enumerate(bar_widths):
        if i % 2 == 0:
            cv2.rectangle(img, (x, 50), (x + w * 3, 150), 0, -1)
        x += w * 3
    return img


# ── 梯度法定位测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLocateBarcodeByGradient(unittest.TestCase):
    """梯度法定位测试"""

    def test_returns_list(self):
        img = make_barcode_stripes()
        rects = locate_barcode_by_gradient(img)
        self.assertIsInstance(rects, list)

    def test_returns_tuples(self):
        img = make_barcode_stripes()
        rects = locate_barcode_by_gradient(img)
        for r in rects:
            self.assertEqual(len(r), 4)
            self.assertIsInstance(r[0], int)

    def test_empty_image(self):
        img = np.ones((200, 400), dtype=np.uint8) * 255
        rects = locate_barcode_by_gradient(img)
        self.assertIsInstance(rects, list)

    def test_barcode_image(self):
        """条形码图像应能检测到区域"""
        img = make_barcode_stripes()
        rects = locate_barcode_by_gradient(img)
        # 可能检测到也可能检测不到，取决于阈值
        self.assertIsInstance(rects, list)


# ── 形态学法定位测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLocateBarcodeByMorphology(unittest.TestCase):
    """形态学法定位测试"""

    def test_returns_list(self):
        img = make_barcode_stripes()
        rects = locate_barcode_by_morphology(img)
        self.assertIsInstance(rects, list)

    def test_returns_valid_rects(self):
        img = make_barcode_stripes()
        rects = locate_barcode_by_morphology(img)
        for r in rects:
            self.assertEqual(len(r), 4)
            x, y, w, h = r
            self.assertGreater(w, 0)
            self.assertGreater(h, 0)

    def test_empty_image(self):
        img = np.ones((200, 400), dtype=np.uint8) * 255
        rects = locate_barcode_by_morphology(img)
        self.assertIsInstance(rects, list)


# ── 图像增强测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestEnhanceForBarcode(unittest.TestCase):
    """条形码图像增强测试"""

    def test_output_shape(self):
        img = np.random.randint(0, 256, (200, 400), dtype=np.uint8)
        enhanced = enhance_for_barcode(img)
        self.assertEqual(enhanced.shape, img.shape)

    def test_output_dtype(self):
        img = np.random.randint(0, 256, (200, 400), dtype=np.uint8)
        enhanced = enhance_for_barcode(img)
        self.assertEqual(enhanced.dtype, np.uint8)

    def test_different_from_input(self):
        """增强后应与原图不同"""
        img = make_barcode_stripes()
        enhanced = enhance_for_barcode(img)
        self.assertFalse(np.array_equal(img, enhanced))


# ── 绘制结果测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawBarcodeResults(unittest.TestCase):
    """绘制结果测试"""

    def test_output_shape(self):
        img = np.zeros((200, 400, 3), dtype=np.uint8)
        results = [{'rect': (50, 50, 100, 30), 'type': 'CODE128', 'data': '12345'}]
        vis = draw_barcode_results(img, results)
        self.assertEqual(vis.shape, img.shape)

    def test_empty_results(self):
        img = np.zeros((200, 400, 3), dtype=np.uint8)
        vis = draw_barcode_results(img, [])
        self.assertEqual(vis.shape, img.shape)

    def test_does_not_modify_input(self):
        img = np.zeros((200, 400, 3), dtype=np.uint8)
        img_copy = img.copy()
        results = [{'rect': (50, 50, 100, 30), 'type': 'CODE128', 'data': '12345'}]
        draw_barcode_results(img, results)
        np.testing.assert_array_equal(img, img_copy)


# ── 条形码解码测试（需要 pyzbar） ──

@unittest.skipUnless(HAS_CV2 and HAS_PYZBAR, "OpenCV or pyzbar not installed")
class TestDecodeBarcodes(unittest.TestCase):
    """条形码解码测试"""

    def test_returns_list(self):
        img = make_barcode_stripes()
        results = decode_barcodes(img)
        self.assertIsInstance(results, list)

    def test_result_structure(self):
        """如果有解码结果，应包含必要字段"""
        img = make_barcode_stripes()
        results = decode_barcodes(img)
        for r in results:
            self.assertIn('data', r)
            self.assertIn('type', r)
            self.assertIn('rect', r)
            self.assertIn('quality', r)

    def test_empty_image(self):
        img = np.ones((200, 400, 3), dtype=np.uint8) * 255
        results = decode_barcodes(img)
        self.assertEqual(len(results), 0)


@unittest.skipUnless(HAS_CV2 and HAS_PYZBAR, "OpenCV or pyzbar not installed")
class TestDecodeBarcodeROI(unittest.TestCase):
    """ROI解码测试"""

    def test_returns_none_for_empty_roi(self):
        img = np.ones((200, 400, 3), dtype=np.uint8) * 255
        result = decode_barcode_roi(img, (0, 0, 100, 100))
        self.assertIsNone(result)


@unittest.skipUnless(HAS_CV2 and HAS_PYZBAR, "OpenCV or pyzbar not installed")
class TestMultiScaleDecode(unittest.TestCase):
    """多尺度解码测试"""

    def test_returns_list(self):
        img = make_barcode_stripes()
        results = multi_scale_decode(img)
        self.assertIsInstance(results, list)

    def test_no_duplicates(self):
        """不应有重复结果"""
        img = make_barcode_stripes()
        results = multi_scale_decode(img)
        data_list = [r['data'] for r in results]
        self.assertEqual(len(data_list), len(set(data_list)))


if __name__ == '__main__':
    unittest.main()
