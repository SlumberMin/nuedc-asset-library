"""
形态学操作单元测试
"""
import unittest
import numpy as np


class TestImageMorphology(unittest.TestCase):
    """形态学操作测试"""

    def setUp(self):
        self.binary_image = np.zeros((64, 64), dtype=np.uint8)
        self.binary_image[20:44, 20:44] = 255  # 中心白色方块

    def test_erosion_shrinks(self):
        """腐蚀应缩小白色区域"""
        import cv2
        kernel = np.ones((5, 5), np.uint8)
        eroded = cv2.erode(self.binary_image, kernel)
        white_before = np.sum(self.binary_image > 0)
        white_after = np.sum(eroded > 0)
        self.assertLess(white_after, white_before)

    def test_dilation_expands(self):
        """膨胀应扩大白色区域"""
        import cv2
        kernel = np.ones((5, 5), np.uint8)
        dilated = cv2.dilate(self.binary_image, kernel)
        white_before = np.sum(self.binary_image > 0)
        white_after = np.sum(dilated > 0)
        self.assertGreater(white_after, white_before)

    def test_opening_removes_noise(self):
        """开运算去除小噪点"""
        import cv2
        noisy = self.binary_image.copy()
        noisy[5, 5] = 255
        noisy[10, 10] = 255
        kernel = np.ones((3, 3), np.uint8)
        opened = cv2.morphologyEx(noisy, cv2.MORPH_OPEN, kernel)
        # 小噪点应被去除
        self.assertEqual(opened[5, 5], 0)
        self.assertEqual(opened[10, 10], 0)

    def test_closing_fills_gaps(self):
        """闭运算填补小空洞"""
        import cv2
        img = self.binary_image.copy()
        img[30, 30] = 0  # 小空洞
        kernel = np.ones((3, 3), np.uint8)
        closed = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
        self.assertEqual(closed[30, 30], 255)

    def test_gradient_edge_detection(self):
        """形态学梯度检测边缘"""
        import cv2
        kernel = np.ones((3, 3), np.uint8)
        gradient = cv2.morphologyEx(self.binary_image, cv2.MORPH_GRADIENT, kernel)
        # 边缘区域应有非零值
        self.assertTrue(np.max(gradient) > 0)

    def test_tophat(self):
        """顶帽变换提取亮细节"""
        import cv2
        img = np.ones((64, 64), dtype=np.uint8) * 100
        img[30:34, 30:34] = 200
        kernel = np.ones((7, 7), np.uint8)
        tophat = cv2.morphologyEx(img, cv2.MORPH_TOPHAT, kernel)
        self.assertTrue(np.max(tophat) > 0)

    def test_blackhat(self):
        """黑帽变换提取暗细节"""
        import cv2
        img = np.ones((64, 64), dtype=np.uint8) * 200
        img[30:34, 30:34] = 50
        kernel = np.ones((7, 7), np.uint8)
        blackhat = cv2.morphologyEx(img, cv2.MORPH_BLACKHAT, kernel)
        self.assertTrue(np.max(blackhat) > 0)

    def test_erosion_dilation_inverse(self):
        """连续腐蚀膨胀不一定恢复原图"""
        import cv2
        kernel = np.ones((3, 3), np.uint8)
        eroded = cv2.erode(self.binary_image, kernel)
        dilated = cv2.dilate(eroded, kernel)
        # 开运算结果，白色面积应 <= 原图
        self.assertLessEqual(np.sum(dilated > 0), np.sum(self.binary_image > 0))

    def test_cross_kernel(self):
        """十字形结构元素"""
        import cv2
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5))
        eroded = cv2.erode(self.binary_image, kernel)
        self.assertEqual(eroded.shape, self.binary_image.shape)

    def test_ellipse_kernel(self):
        """椭圆形结构元素"""
        import cv2
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        dilated = cv2.dilate(self.binary_image, kernel)
        self.assertEqual(dilated.shape, self.binary_image.shape)

    def test_morphology_output_dtype(self):
        """形态学操作输出类型正确"""
        import cv2
        kernel = np.ones((3, 3), np.uint8)
        result = cv2.morphologyEx(self.binary_image, cv2.MORPH_OPEN, kernel)
        self.assertEqual(result.dtype, np.uint8)

    def test_large_kernel(self):
        """大尺寸结构元素"""
        import cv2
        kernel = np.ones((15, 15), np.uint8)
        eroded = cv2.erode(self.binary_image, kernel)
        self.assertLess(np.sum(eroded > 0), np.sum(self.binary_image > 0))


if __name__ == '__main__':
    unittest.main()
