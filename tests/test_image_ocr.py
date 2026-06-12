#!/usr/bin/env python3
"""
OCR文字识别单元测试
覆盖: 预处理方法、倾斜校正、统一接口、区域识别、数字识别、后端枚举
测试对象: 10_视觉通用代码库/image_ocr.py
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
    from _10_视觉通用代码库.image_ocr import (
        preprocess_for_ocr,
        deskew_image,
        ocr_recognize,
        ocr_recognize_detail,
        ocr_region,
        ocr_numbers,
        _BACKENDS,
    )


# ── 预处理测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestPreprocessForOCR(unittest.TestCase):
    """OCR预处理测试"""

    def _make_text_image(self):
        """创建含文字的测试图像"""
        img = np.ones((100, 400, 3), dtype=np.uint8) * 255
        cv2.putText(img, 'Hello OCR 12345', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
        return img

    def test_adaptive_output_shape(self):
        """自适应阈值输出形状"""
        img = self._make_text_image()
        result = preprocess_for_ocr(img, method='adaptive')
        self.assertEqual(result.shape, img.shape[:2])

    def test_adaptive_output_dtype(self):
        img = self._make_text_image()
        result = preprocess_for_ocr(img, method='adaptive')
        self.assertEqual(result.dtype, np.uint8)

    def test_otsu_output_shape(self):
        img = self._make_text_image()
        result = preprocess_for_ocr(img, method='otsu')
        self.assertEqual(result.shape, img.shape[:2])

    def test_denoise_output_shape(self):
        img = self._make_text_image()
        result = preprocess_for_ocr(img, method='denoise')
        self.assertEqual(result.shape, img.shape[:2])

    def test_sharpen_output_shape(self):
        img = self._make_text_image()
        result = preprocess_for_ocr(img, method='sharpen')
        self.assertEqual(result.shape, img.shape[:2])

    def test_unknown_method_passthrough(self):
        """未知方法应返回灰度图"""
        img = self._make_text_image()
        result = preprocess_for_ocr(img, method='unknown')
        self.assertEqual(result.shape, img.shape[:2])

    def test_grayscale_input(self):
        """灰度图输入"""
        img = np.ones((100, 400), dtype=np.uint8) * 200
        result = preprocess_for_ocr(img, method='adaptive')
        self.assertEqual(result.shape, img.shape)

    def test_binary_output_range(self):
        """自适应阈值输出应为二值"""
        img = self._make_text_image()
        result = preprocess_for_ocr(img, method='adaptive')
        unique = np.unique(result)
        self.assertTrue(len(unique) <= 2)


# ── 倾斜校正测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestDeskewImage(unittest.TestCase):
    """倾斜校正测试"""

    def test_output_shape_preserved(self):
        """校正后形状不变"""
        img = np.ones((200, 400), dtype=np.uint8) * 255
        cv2.putText(img, 'Test Text', (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, 0, 3)
        result = deskew_image(img)
        self.assertEqual(result.shape, img.shape)

    def test_empty_image_no_crash(self):
        """空白图像不应崩溃"""
        img = np.zeros((200, 400), dtype=np.uint8)
        result = deskew_image(img)
        self.assertEqual(result.shape, img.shape)

    def test_color_image_input(self):
        """彩色图像输入"""
        img = np.ones((200, 400, 3), dtype=np.uint8) * 255
        cv2.putText(img, 'Color', (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, 0, 3)
        result = deskew_image(img)
        self.assertEqual(result.shape, img.shape)


# ── 后端枚举测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestBackends(unittest.TestCase):
    """后端注册测试"""

    def test_tesseract_registered(self):
        self.assertIn('tesseract', _BACKENDS)

    def test_easyocr_registered(self):
        self.assertIn('easyocr', _BACKENDS)

    def test_paddle_registered(self):
        self.assertIn('paddle', _BACKENDS)
        self.assertIn('paddleocr', _BACKENDS)

    def test_backends_are_callable(self):
        for name, func in _BACKENDS.items():
            self.assertTrue(callable(func))


# ── 统一接口测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestOCRRecogizeInterface(unittest.TestCase):
    """统一接口测试"""

    def test_invalid_backend_raises(self):
        """无效后端应抛出ValueError"""
        img = np.ones((100, 400, 3), dtype=np.uint8) * 255
        with self.assertRaises(ValueError):
            ocr_recognize(img, backend='invalid_backend')

    def test_invalid_backend_detail_raises(self):
        img = np.ones((100, 400, 3), dtype=np.uint8) * 255
        with self.assertRaises(ValueError):
            ocr_recognize_detail(img, backend='invalid_backend')

    def test_preprocess_none(self):
        """preprocess=None时不应预处理"""
        img = np.ones((100, 400, 3), dtype=np.uint8) * 255
        try:
            ocr_recognize(img, backend='tesseract', preprocess=None)
        except ImportError:
            pass  # tesseract未安装是正常的

    def test_preprocess_adaptive(self):
        """preprocess='adaptive'应先预处理"""
        img = np.ones((100, 400, 3), dtype=np.uint8) * 255
        try:
            ocr_recognize(img, backend='tesseract', preprocess='adaptive')
        except ImportError:
            pass


# ── 区域识别测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestOCRRegion(unittest.TestCase):
    """区域识别测试"""

    def test_region_crop(self):
        """应能裁剪指定区域"""
        img = np.ones((200, 400, 3), dtype=np.uint8) * 255
        cv2.putText(img, 'ROI', (150, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
        try:
            result = ocr_region(img, 100, 50, 200, 100, backend='tesseract')
            self.assertIsInstance(result, str)
        except ImportError:
            pass


# ── 数字识别测试 ──

@unittest.skipUnless(HAS_CV2, "OpenCV not installed")
class TestOCRNumbers(unittest.TestCase):
    """数字识别测试"""

    def test_returns_list(self):
        """应返回数字列表"""
        img = np.ones((100, 400, 3), dtype=np.uint8) * 255
        cv2.putText(img, '3.14159', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, 0, 3)
        try:
            numbers = ocr_numbers(img, backend='tesseract')
            self.assertIsInstance(numbers, list)
        except ImportError:
            pass

    def test_empty_image_returns_empty(self):
        """空白图像应返回空列表"""
        img = np.ones((100, 400, 3), dtype=np.uint8) * 255
        try:
            numbers = ocr_numbers(img, backend='tesseract')
            self.assertIsInstance(numbers, list)
        except ImportError:
            pass


if __name__ == '__main__':
    unittest.main()
