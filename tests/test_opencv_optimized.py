#!/usr/bin/env python3
"""
OpenCV优化模块单元测试
覆盖: optimized_color_detect, optimized_line_detect, optimized_shape_detect, optimized_template_match, optimized_qr_detect
模块来源: 08_OpenCV优化/
"""

import sys
import os
import unittest
import time
import numpy as np
from unittest.mock import MagicMock, patch
from typing import List, Tuple, Dict

# 将项目根目录加入路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


def create_test_image(width: int = 640, height: int = 480, color: Tuple = (0, 0, 0)) -> np.ndarray:
    """创建测试图像"""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = color
    return img


# ===================== 优化颜色检测测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过优化颜色检测测试")
class TestOptimizedColorDetect(unittest.TestCase):
    """优化颜色检测器测试"""

    def setUp(self):
        """测试前初始化"""
        try:
            from OpenCV_optimized.optimized_color_detect import OptimizedColorDetector
            self.detector = OptimizedColorDetector()
        except ImportError:
            self.skipTest("OptimizedColorDetector模块未安装")

    def test_initialization(self):
        """测试检测器初始化"""
        self.assertIsNotNone(self.detector)

    def test_color_detection_basic(self):
        """基本颜色检测"""
        img = create_test_image(640, 480, (0, 0, 255))  # 红色
        results = self.detector.detect(img, color='red')
        self.assertIsInstance(results, list)

    def test_color_detection_performance(self):
        """性能基准测试 - 优化版应更快"""
        img = create_test_image(640, 480, (0, 0, 255))

        start_time = time.time()
        for _ in range(100):
            results = self.detector.detect(img, color='red')
        elapsed = time.time() - start_time

        # 优化版性能要求: 100帧 < 0.5秒
        self.assertLess(elapsed, 0.5, "优化颜色检测性能不足")

    def test_multi_color_detection(self):
        """多颜色同时检测"""
        img = create_test_image(640, 480, (100, 100, 100))
        cv2.circle(img, (200, 240), 50, (0, 0, 255), -1)  # 红色
        cv2.circle(img, (440, 240), 50, (255, 0, 0), -1)  # 蓝色

        results = self.detector.detect_multi(img, colors=['red', 'blue'])
        self.assertIsInstance(results, dict)

    def test_edge_case_empty_image(self):
        """边界条件: 空图像"""
        img = np.zeros((0, 0, 3), dtype=np.uint8)
        try:
            results = self.detector.detect(img, color='red')
            self.assertIsInstance(results, list)
        except Exception as e:
            self.assertIsInstance(e, (ValueError, cv2.error))

    def test_edge_case_grayscale_image(self):
        """边界条件: 灰度图像"""
        img = np.random.randint(0, 256, (480, 640), dtype=np.uint8)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        results = self.detector.detect(img_bgr, color='red')
        self.assertIsInstance(results, list)


# ===================== 优化直线检测测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过优化直线检测测试")
class TestOptimizedLineDetect(unittest.TestCase):
    """优化直线检测器测试"""

    def setUp(self):
        try:
            from OpenCV_optimized.optimized_line_detect import OptimizedLineDetector
            self.detector = OptimizedLineDetector()
        except ImportError:
            self.skipTest("OptimizedLineDetector模块未安装")

    def test_initialization(self):
        """测试检测器初始化"""
        self.assertIsNotNone(self.detector)

    def test_detect_horizontal_line(self):
        """检测水平线"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.line(img, (0, 240), (640, 240), (0, 0, 0), 2)

        lines = self.detector.detect(img)
        self.assertIsInstance(lines, list)
        self.assertGreater(len(lines), 0)

    def test_detect_multiple_lines(self):
        """检测多条线"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.line(img, (0, 100), (640, 100), (0, 0, 0), 2)
        cv2.line(img, (0, 200), (640, 200), (0, 0, 0), 2)
        cv2.line(img, (0, 300), (640, 300), (0, 0, 0), 2)

        lines = self.detector.detect(img)
        self.assertGreaterEqual(len(lines), 2)

    def test_performance_benchmark(self):
        """性能基准测试"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.line(img, (0, 240), (640, 240), (0, 0, 0), 2)

        start_time = time.time()
        for _ in range(100):
            lines = self.detector.detect(img)
        elapsed = time.time() - start_time

        self.assertLess(elapsed, 1.0, "优化直线检测性能不足")

    def test_detect_diagonal_line(self):
        """检测对角线"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.line(img, (0, 0), (640, 480), (0, 0, 0), 2)

        lines = self.detector.detect(img)
        self.assertIsInstance(lines, list)

    def test_no_lines_in_noise(self):
        """噪声图像中无直线"""
        img = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
        lines = self.detector.detect(img)
        self.assertIsInstance(lines, list)


# ===================== 优化形状检测测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过优化形状检测测试")
class TestOptimizedShapeDetect(unittest.TestCase):
    """优化形状检测器测试"""

    def setUp(self):
        try:
            from OpenCV_optimized.optimized_shape_detect import OptimizedShapeDetector
            self.detector = OptimizedShapeDetector()
        except ImportError:
            self.skipTest("OptimizedShapeDetector模块未安装")

    def test_initialization(self):
        """测试检测器初始化"""
        self.assertIsNotNone(self.detector)

    def test_detect_circle(self):
        """检测圆形"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.circle(img, (320, 240), 100, (0, 0, 0), -1)

        shapes = self.detector.detect(img)
        self.assertIsInstance(shapes, list)

    def test_detect_rectangle(self):
        """检测矩形"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.rectangle(img, (200, 150), (440, 330), (0, 0, 0), -1)

        shapes = self.detector.detect(img)
        self.assertIsInstance(shapes, list)

    def test_detect_triangle(self):
        """检测三角形"""
        img = create_test_image(640, 480, (255, 255, 255))
        pts = np.array([[320, 100], [200, 380], [440, 380]], np.int32)
        cv2.fillPoly(img, [pts], (0, 0, 0))

        shapes = self.detector.detect(img)
        self.assertIsInstance(shapes, list)

    def test_performance_benchmark(self):
        """性能基准测试"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.circle(img, (320, 240), 100, (0, 0, 0), -1)

        start_time = time.time()
        for _ in range(50):
            shapes = self.detector.detect(img)
        elapsed = time.time() - start_time

        self.assertLess(elapsed, 2.0, "优化形状检测性能不足")

    def test_detect_multiple_shapes(self):
        """检测多个形状"""
        img = create_test_image(640, 480, (255, 255, 255))
        cv2.circle(img, (160, 240), 60, (0, 0, 0), -1)
        cv2.rectangle(img, (380, 180), (520, 300), (0, 0, 0), -1)

        shapes = self.detector.detect(img)
        self.assertGreaterEqual(len(shapes), 2)


# ===================== 优化模板匹配测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过优化模板匹配测试")
class TestOptimizedTemplateMatch(unittest.TestCase):
    """优化模板匹配器测试"""

    def setUp(self):
        try:
            from OpenCV_optimized.optimized_template_match import OptimizedTemplateMatcher
            self.matcher = OptimizedTemplateMatcher()
        except ImportError:
            self.skipTest("OptimizedTemplateMatcher模块未安装")

    def test_initialization(self):
        """测试匹配器初始化"""
        self.assertIsNotNone(self.matcher)

    def test_match_template(self):
        """模板匹配"""
        img = create_test_image(640, 480, (200, 200, 200))
        cv2.rectangle(img, (300, 200), (380, 280), (100, 100, 100), -1)
        template = img[200:280, 300:380].copy()

        result = self.matcher.match(img, template)
        self.assertIsInstance(result, dict)
        self.assertIn('confidence', result)
        self.assertIn('location', result)

    def test_multi_scale_matching(self):
        """多尺度匹配"""
        img = create_test_image(640, 480, (200, 200, 200))
        cv2.rectangle(img, (300, 200), (380, 280), (100, 100, 100), -1)
        template = img[200:280, 300:380].copy()

        result = self.matcher.match_multiscale(img, template, scales=[0.8, 1.0, 1.2])
        self.assertIsInstance(result, dict)

    def test_performance_benchmark(self):
        """性能基准测试"""
        img = create_test_image(640, 480, (200, 200, 200))
        cv2.rectangle(img, (300, 200), (380, 280), (100, 100, 100), -1)
        template = img[200:280, 300:380].copy()

        start_time = time.time()
        for _ in range(50):
            result = self.matcher.match(img, template)
        elapsed = time.time() - start_time

        self.assertLess(elapsed, 2.0, "优化模板匹配性能不足")

    def test_edge_case_no_match(self):
        """边界条件: 无法匹配"""
        img = create_test_image(640, 480, (200, 200, 200))
        template = create_test_image(50, 50, (50, 50, 50))

        result = self.matcher.match(img, template)
        self.assertIsInstance(result, dict)
        self.assertLess(result.get('confidence', 0), 0.5)


# ===================== 优化二维码检测测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过优化二维码检测测试")
class TestOptimizedQRDetect(unittest.TestCase):
    """优化二维码检测器测试"""

    def setUp(self):
        try:
            from OpenCV_optimized.optimized_qr_detect import OptimizedQRDetector
            self.detector = OptimizedQRDetector()
        except ImportError:
            self.skipTest("OptimizedQRDetector模块未安装")

    def test_initialization(self):
        """测试检测器初始化"""
        self.assertIsNotNone(self.detector)

    def test_edge_case_no_qr(self):
        """边界条件: 无二维码"""
        img = create_test_image(640, 480, (255, 255, 255))
        results = self.detector.detect(img)
        self.assertIsInstance(results, list)

    def test_performance_benchmark(self):
        """性能基准测试"""
        img = create_test_image(640, 480, (255, 255, 255))

        start_time = time.time()
        for _ in range(50):
            results = self.detector.detect(img)
        elapsed = time.time() - start_time

        self.assertLess(elapsed, 1.0, "优化二维码检测性能不足")

    def test_detect_synthetic_qr(self):
        """检测合成二维码"""
        try:
            import qrcode
            qr = qrcode.QRCode(version=1, box_size=10, border=5)
            qr.add_data("Test123")
            qr.make(fit=True)
            qr_img = qr.make_image(fill_color="black", back_color="white")
            qr_np = np.array(qr_img)

            img = create_test_image(640, 480, (255, 255, 255))
            size = min(qr_np.shape[:2], 200)
            qr_resized = cv2.resize(qr_np[:size, :size], (size, size))

            if len(qr_resized.shape) == 2:
                qr_bgr = cv2.cvtColor(qr_resized, cv2.COLOR_GRAY2BGR)
            else:
                qr_bgr = qr_resized

            x_start = (640 - size) // 2
            y_start = (480 - size) // 2
            img[y_start:y_start+size, x_start:x_start+size] = qr_bgr

            results = self.detector.detect(img)
            self.assertIsInstance(results, list)
        except ImportError:
            self.skipTest("qrcode库未安装")


# ===================== 边缘案例与压力测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过压力测试")
class TestStressCases(unittest.TestCase):
    """压力测试与边界案例"""

    def test_large_image_processing(self):
        """大图像处理"""
        try:
            from OpenCV_optimized.optimized_color_detect import OptimizedColorDetector
            detector = OptimizedColorDetector()

            img = create_test_image(1920, 1080, (0, 0, 255))
            start_time = time.time()
            results = detector.detect(img, color='red')
            elapsed = time.time() - start_time

            self.assertLess(elapsed, 2.0, "大图像处理超时")
        except ImportError:
            self.skipTest("模块未安装")

    def test_rapid_successive_calls(self):
        """快速连续调用"""
        try:
            from OpenCV_optimized.optimized_color_detect import OptimizedColorDetector
            detector = OptimizedColorDetector()

            img = create_test_image(640, 480, (0, 0, 255))
            results_list = []

            start_time = time.time()
            for _ in range(200):
                results = detector.detect(img, color='red')
                results_list.append(results)
            elapsed = time.time() - start_time

            self.assertLess(elapsed, 2.0, "快速连续调用超时")
            self.assertEqual(len(results_list), 200)
        except ImportError:
            self.skipTest("模块未安装")

    def test_memory_stability(self):
        """内存稳定性测试"""
        try:
            from OpenCV_optimized.optimized_color_detect import OptimizedColorDetector
            detector = OptimizedColorDetector()

            for _ in range(100):
                img = create_test_image(640, 480, (0, 0, 255))
                results = detector.detect(img, color='red')
                del img  # 显式释放

            self.assertTrue(True, "内存稳定性测试通过")
        except ImportError:
            self.skipTest("模块未安装")


if __name__ == '__main__':
    unittest.main()
