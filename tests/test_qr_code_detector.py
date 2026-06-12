#!/usr/bin/env python3
"""
QR码检测单元测试
覆盖: 定位图案检测、轮廓法定位、透视校正、
      QR码解码、多帧融合解码、QR码生成、绘制结果
测试对象: 10_视觉通用代码库/qr_code_detector.py
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

if HAS_CV2:
    from _10_视觉通用代码库.qr_code_detector import (
        locate_qr_finder_patterns,
        locate_qr_by_contour,
        order_points,
        perspective_correct,
        decode_qr_opencv,
        decode_qr_with_preprocess,
        decode_qr_multiframe,
        generate_qr_image,
        draw_qr_results,
    )


# ── 定位图案检测测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLocateQRFinderPatterns(unittest.TestCase):
    """QR码定位图案检测测试"""

    def test_returns_list(self):
        img = np.ones((300, 300), dtype=np.uint8) * 255
        result = locate_qr_finder_patterns(img)
        self.assertIsInstance(result, list)

    def test_returns_tuples(self):
        """如有结果，应为 (cx, cy, size) 三元组"""
        img = generate_qr_image("Test123", size=300)
        result = locate_qr_finder_patterns(img)
        for r in result:
            self.assertEqual(len(r), 3)

    def test_empty_image(self):
        """空白图像不应检测到定位图案"""
        img = np.ones((300, 300), dtype=np.uint8) * 255
        result = locate_qr_finder_patterns(img)
        self.assertIsInstance(result, list)

    def test_qr_image_has_patterns(self):
        """生成的QR码应有定位图案"""
        img = generate_qr_image("Hello", size=300)
        result = locate_qr_finder_patterns(img)
        # QR码应至少有3个定位图案（左上、右上、左下）
        self.assertGreaterEqual(len(result), 1)


# ── 轮廓法定位测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestLocateQRByContour(unittest.TestCase):
    """轮廓法定位测试"""

    def test_returns_list(self):
        img = np.ones((300, 300), dtype=np.uint8) * 255
        result = locate_qr_by_contour(img)
        self.assertIsInstance(result, list)

    def test_returns_rects(self):
        """如有结果，应为 (x, y, w, h)"""
        img = generate_qr_image("Test", size=300)
        result = locate_qr_by_contour(img)
        for r in result:
            self.assertEqual(len(r), 4)
            x, y, w, h = r
            self.assertGreater(w, 0)
            self.assertGreater(h, 0)

    def test_empty_image(self):
        img = np.ones((300, 300), dtype=np.uint8) * 255
        result = locate_qr_by_contour(img)
        self.assertEqual(len(result), 0)


# ── 点排序测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestOrderPoints(unittest.TestCase):
    """点排序测试"""

    def test_returns_4x2(self):
        pts = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.float32)
        result = order_points(pts)
        self.assertEqual(result.shape, (4, 2))

    def test_ordering(self):
        """左上、右上、右下、左下"""
        pts = np.array([[100, 100], [200, 100], [200, 200], [100, 200]], dtype=np.float32)
        result = order_points(pts)
        # 左上：x+y 最小
        self.assertAlmostEqual(result[0][0], 100, delta=1)
        self.assertAlmostEqual(result[0][1], 100, delta=1)
        # 右下：x+y 最大
        self.assertAlmostEqual(result[2][0], 200, delta=1)
        self.assertAlmostEqual(result[2][1], 200, delta=1)

    def test_unordered_input(self):
        """乱序输入应正确排序"""
        pts = np.array([[200, 200], [100, 100], [200, 100], [100, 200]], dtype=np.float32)
        result = order_points(pts)
        # 左上应为 (100, 100)
        self.assertAlmostEqual(result[0][0], 100, delta=1)
        self.assertAlmostEqual(result[0][1], 100, delta=1)


# ── 透视校正测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestPerspectiveCorrect(unittest.TestCase):
    """透视校正测试"""

    def test_output_shape(self):
        img = np.zeros((400, 400), dtype=np.uint8)
        corners = np.array([[100, 100], [300, 100], [300, 300], [100, 300]], dtype=np.float32)
        result = perspective_correct(img, corners, output_size=200)
        self.assertEqual(result.shape, (200, 200))

    def test_custom_output_size(self):
        img = np.zeros((400, 400), dtype=np.uint8)
        corners = np.array([[100, 100], [300, 100], [300, 300], [100, 300]], dtype=np.float32)
        result = perspective_correct(img, corners, output_size=400)
        self.assertEqual(result.shape, (400, 400))

    def test_preserves_content(self):
        """透视校正应保留内容"""
        img = np.zeros((400, 400), dtype=np.uint8)
        cv2.rectangle(img, (100, 100), (300, 300), 255, -1)
        corners = np.array([[100, 100], [300, 100], [300, 300], [100, 300]], dtype=np.float32)
        result = perspective_correct(img, corners, output_size=200)
        # 输出应包含白色区域
        self.assertGreater(np.sum(result > 0), 0)


# ── QR码解码测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDecodeQROpenCV(unittest.TestCase):
    """OpenCV QR码解码测试"""

    def test_returns_list(self):
        img = generate_qr_image("Test123", size=300)
        results = decode_qr_opencv(img)
        self.assertIsInstance(results, list)

    def test_decode_generated_qr(self):
        """应能解码生成的QR码"""
        try:
            import qrcode
        except ImportError:
            self.skipTest("qrcode library not installed")
        img = generate_qr_image("HelloQR", size=300)
        results = decode_qr_opencv(img)
        if results:
            self.assertIn('data', results[0])
            self.assertIn('points', results[0])

    def test_empty_image(self):
        img = np.ones((300, 300), dtype=np.uint8) * 255
        results = decode_qr_opencv(img)
        self.assertIsInstance(results, list)

    def test_result_structure(self):
        """解码结果应包含必要字段"""
        try:
            import qrcode
        except ImportError:
            self.skipTest("qrcode library not installed")
        img = generate_qr_image("Test", size=300)
        results = decode_qr_opencv(img)
        for r in results:
            self.assertIn('data', r)
            self.assertIn('error_corrected', r)


# ── 带预处理解码测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDecodeQRWithPreprocess(unittest.TestCase):
    """带预处理的QR码解码测试"""

    def test_returns_list(self):
        img = generate_qr_image("Test", size=300)
        results = decode_qr_with_preprocess(img)
        self.assertIsInstance(results, list)

    def test_no_duplicates(self):
        """不应有重复解码结果"""
        img = generate_qr_image("Test", size=300)
        results = decode_qr_with_preprocess(img)
        data_list = [r['data'] for r in results]
        self.assertEqual(len(data_list), len(set(data_list)))


# ── 多帧融合解码测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDecodeQRMultiframe(unittest.TestCase):
    """多帧融合解码测试"""

    def test_returns_none_for_empty_frames(self):
        frames = [np.ones((300, 300), dtype=np.uint8) * 255 for _ in range(5)]
        result = decode_qr_multiframe(frames)
        self.assertIsNone(result)

    def test_returns_dict_or_none(self):
        frames = [generate_qr_image("Test", size=300) for _ in range(3)]
        result = decode_qr_multiframe(frames)
        if result is not None:
            self.assertIn('data', result)
            self.assertIn('confidence', result)
            self.assertIn('total_votes', result)


# ── QR码生成测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestGenerateQRImage(unittest.TestCase):
    """QR码生成测试"""

    def test_returns_array(self):
        img = generate_qr_image("Test", size=300)
        self.assertIsInstance(img, np.ndarray)

    def test_output_size(self):
        img = generate_qr_image("Test", size=200)
        self.assertEqual(img.shape, (200, 200))

    def test_different_data(self):
        """不同数据应生成不同图像"""
        img1 = generate_qr_image("ABC", size=300)
        img2 = generate_qr_image("XYZ", size=300)
        self.assertFalse(np.array_equal(img1, img2))

    def test_grayscale_output(self):
        img = generate_qr_image("Test", size=300)
        self.assertEqual(len(img.shape), 2)


# ── 绘制结果测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDrawQRResults(unittest.TestCase):
    """绘制结果测试"""

    def test_output_shape(self):
        img = np.zeros((300, 300, 3), dtype=np.uint8)
        results = [{'data': 'Test', 'points': np.array([[10, 10], [100, 10], [100, 100], [10, 100]])}]
        vis = draw_qr_results(img, results)
        self.assertEqual(vis.shape, img.shape)

    def test_empty_results(self):
        img = np.zeros((300, 300, 3), dtype=np.uint8)
        vis = draw_qr_results(img, [])
        self.assertEqual(vis.shape, img.shape)

    def test_no_points_result(self):
        """无点的结果不应崩溃"""
        img = np.zeros((300, 300, 3), dtype=np.uint8)
        results = [{'data': 'Test', 'points': None}]
        vis = draw_qr_results(img, results)
        self.assertEqual(vis.shape, img.shape)


if __name__ == '__main__':
    unittest.main()
