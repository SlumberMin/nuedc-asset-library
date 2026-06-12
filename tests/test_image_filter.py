"""
图像滤波单元测试
"""
import unittest
import numpy as np


class TestImageFilter(unittest.TestCase):
    """图像滤波器测试"""

    def setUp(self):
        self.test_image = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        self.noisy_image = np.clip(
            self.test_image.astype(float) + np.random.normal(0, 25, self.test_image.shape), 0, 255
        ).astype(np.uint8)

    def test_gaussian_blur_reduces_noise(self):
        """高斯模糊应降低噪声"""
        import cv2
        blurred = cv2.GaussianBlur(self.noisy_image, (5, 5), 1.0)
        diff_before = np.mean(np.abs(self.test_image.astype(float) - self.noisy_image.astype(float)))
        diff_after = np.mean(np.abs(self.test_image.astype(float) - blurred.astype(float)))
        self.assertLess(diff_after, diff_before)

    def test_median_filter_removes_salt_pepper(self):
        """中值滤波去除椒盐噪声"""
        import cv2
        img = self.test_image.copy()
        # 添加椒盐噪声
        coords = np.random.randint(0, img.size, 500)
        img.flat[coords[:250]] = 0
        img.flat[coords[250:]] = 255
        filtered = cv2.medianBlur(img, 3)
        diff = np.mean(np.abs(self.test_image.astype(float) - filtered.astype(float)))
        diff_noisy = np.mean(np.abs(self.test_image.astype(float) - img.astype(float)))
        self.assertLess(diff, diff_noisy)

    def test_bilateral_filter_preserves_edges(self):
        """双边滤波应保持边缘"""
        import cv2
        img = np.zeros((64, 64), dtype=np.uint8)
        img[:32, :] = 128
        img[32:, :] = 200
        filtered = cv2.bilateralFilter(img, 9, 75, 75)
        # 边缘应保持
        self.assertEqual(filtered[0, 0], filtered[0, 31])
        self.assertNotEqual(filtered[0, 0], filtered[33, 0])

    def test_box_filter(self):
        """均值滤波"""
        import cv2
        kernel = np.ones((3, 3), np.float32) / 9
        filtered = cv2.filter2D(self.test_image, -1, kernel)
        self.assertEqual(filtered.shape, self.test_image.shape)

    def test_sobel_edge_detection(self):
        """Sobel边缘检测"""
        import cv2
        sobelx = cv2.Sobel(self.test_image, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(self.test_image, cv2.CV_64F, 0, 1, ksize=3)
        magnitude = np.sqrt(sobelx ** 2 + sobely ** 2)
        self.assertTrue(np.max(magnitude) > 0)

    def test_laplacian_edge_detection(self):
        """拉普拉斯边缘检测"""
        import cv2
        laplacian = cv2.Laplacian(self.test_image, cv2.CV_64F)
        self.assertEqual(laplacian.shape, self.test_image.shape)

    def test_canny_edge_detection(self):
        """Canny边缘检测"""
        import cv2
        edges = cv2.Canny(self.test_image, 50, 150)
        self.assertEqual(edges.shape, self.test_image.shape)
        self.assertTrue(np.max(edges) > 0)

    def test_custom_kernel_filter(self):
        """自定义卷积核滤波"""
        import cv2
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.float32)
        sharpened = cv2.filter2D(self.test_image, -1, kernel)
        self.assertEqual(sharpened.shape, self.test_image.shape)

    def test_blur_output_type(self):
        """模糊滤波输出类型正确"""
        import cv2
        blurred = cv2.blur(self.test_image, (3, 3))
        self.assertEqual(blurred.dtype, np.uint8)

    def test_filter_preserves_size(self):
        """各种滤波器保持图像尺寸"""
        import cv2
        for ksize in [3, 5, 7]:
            blurred = cv2.GaussianBlur(self.test_image, (ksize, ksize), 0)
            self.assertEqual(blurred.shape, self.test_image.shape)


if __name__ == '__main__':
    unittest.main()
