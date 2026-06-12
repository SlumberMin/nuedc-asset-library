"""
直方图分析单元测试
"""
import unittest
import numpy as np


class TestImageHistogram(unittest.TestCase):
    """直方图分析测试"""

    def setUp(self):
        self.gray_image = np.random.randint(0, 256, (128, 128), dtype=np.uint8)
        self.color_image = np.random.randint(0, 256, (128, 128, 3), dtype=np.uint8)

    def test_histogram_calculation(self):
        """基本直方图计算"""
        import cv2
        hist = cv2.calcHist([self.gray_image], [0], None, [256], [0, 256])
        self.assertEqual(hist.shape, (256, 1))
        self.assertEqual(int(np.sum(hist)), self.gray_image.size)

    def test_histogram_sum(self):
        """直方图总和等于像素数"""
        import cv2
        hist = cv2.calcHist([self.gray_image], [0], None, [256], [0, 256])
        self.assertEqual(int(np.sum(hist)), 128 * 128)

    def test_color_histogram(self):
        """彩色图像各通道直方图"""
        import cv2
        for ch in range(3):
            hist = cv2.calcHist([self.color_image], [ch], None, [256], [0, 256])
            self.assertEqual(hist.shape, (256, 1))

    def test_histogram_equalization(self):
        """直方图均衡化"""
        import cv2
        equalized = cv2.equalizeHist(self.gray_image)
        self.assertEqual(equalized.shape, self.gray_image.shape)
        # 均衡化后标准差应增大（对比度增强）
        self.assertGreater(np.std(equalized), np.std(self.gray_image) * 0.5)

    def test_histogram_clahe(self):
        """CLAHE自适应直方图均衡化"""
        import cv2
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        result = clahe.apply(self.gray_image)
        self.assertEqual(result.shape, self.gray_image.shape)

    def test_histogram_backprojection(self):
        """直方图反向投影"""
        import cv2
        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        roi_hist = cv2.calcHist([hsv[:32, :32]], [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)
        hsv_full = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        bp = cv2.calcBackProject([hsv_full], [0, 1], roi_hist, [0, 180, 0, 256], 1)
        self.assertEqual(bp.shape, (128, 128))

    def test_histogram_compare(self):
        """直方图比较"""
        import cv2
        hist1 = cv2.calcHist([self.gray_image], [0], None, [256], [0, 256])
        hist2 = cv2.calcHist([self.gray_image], [0], None, [256], [0, 256])
        similarity = cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)
        self.assertAlmostEqual(similarity, 1.0, places=5)

    def test_uniform_image_histogram(self):
        """均匀图像直方图集中在一个值"""
        uniform = np.ones((64, 64), dtype=np.uint8) * 128
        hist, _ = np.histogram(uniform, bins=256, range=(0, 256))
        self.assertEqual(np.sum(hist > 0), 1)

    def test_histogram_2d(self):
        """二维直方图"""
        import cv2
        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
        self.assertEqual(hist.shape, (30, 32))

    def test_normalized_histogram(self):
        """归一化直方图"""
        import cv2
        hist = cv2.calcHist([self.gray_image], [0], None, [256], [0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        self.assertAlmostEqual(float(np.max(hist)), 1.0, places=5)

    def test_otsu_threshold(self):
        """Otsu阈值分割"""
        import cv2
        _, binary = cv2.threshold(self.gray_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        self.assertEqual(binary.shape, self.gray_image.shape)
        unique = np.unique(binary)
        self.assertTrue(len(unique) <= 2)

    def test_histogram_with_mask(self):
        """带掩码的直方图"""
        import cv2
        mask = np.zeros_like(self.gray_image)
        mask[32:96, 32:96] = 255
        hist = cv2.calcHist([self.gray_image], [0], mask, [256], [0, 256])
        self.assertEqual(int(np.sum(hist)), 64 * 64)


if __name__ == '__main__':
    unittest.main()
