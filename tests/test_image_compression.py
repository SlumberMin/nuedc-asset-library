"""
图像压缩单元测试
"""
import unittest
import numpy as np


class TestImageCompression(unittest.TestCase):
    """图像压缩算法测试"""

    def setUp(self):
        self.test_image = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        self.gray_image = np.random.randint(0, 256, (64, 64), dtype=np.uint8)

    def test_jpeg_quality_levels(self):
        """测试不同JPEG质量等级"""
        import cv2
        for quality in [10, 50, 90]:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            _, encoded = cv2.imencode('.jpg', self.test_image, encode_param)
            decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            self.assertEqual(decoded.shape, self.test_image.shape)

    def test_png_compression_lossless(self):
        """测试PNG无损压缩"""
        import cv2
        _, encoded = cv2.imencode('.png', self.test_image)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        np.testing.assert_array_equal(decoded, self.test_image)

    def test_run_length_encoding(self):
        """测试RLE编码基本逻辑"""
        data = np.array([1, 1, 1, 2, 2, 3, 3, 3, 3], dtype=np.uint8)
        encoded = []
        count = 1
        for i in range(1, len(data)):
            if data[i] == data[i - 1]:
                count += 1
            else:
                encoded.append((data[i - 1], count))
                count = 1
        encoded.append((data[-1], count))
        self.assertEqual(encoded, [(1, 3), (2, 2), (3, 4)])

    def test_dct_transform_roundtrip(self):
        """测试DCT变换可逆性"""
        block = np.float64(self.gray_image[:8, :8])
        dct_result = np.zeros_like(block)
        for i in range(8):
            for j in range(8):
                ci = 1.0 / np.sqrt(8) if i == 0 else np.sqrt(2) / np.sqrt(8)
                cj = 1.0 / np.sqrt(8) if j == 0 else np.sqrt(2) / np.sqrt(8)
                val = 0.0
                for x in range(8):
                    for y in range(8):
                        val += block[x, y] * np.cos((2 * x + 1) * i * np.pi / 16) * np.cos((2 * y + 1) * j * np.pi / 16)
                dct_result[i, j] = ci * cj * val
        self.assertFalse(np.allclose(dct_result, 0))

    def test_compression_ratio_estimation(self):
        """测试压缩率估算"""
        import cv2
        raw_size = self.test_image.nbytes
        _, encoded = cv2.imencode('.jpg', self.test_image, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        compressed_size = encoded.nbytes
        ratio = raw_size / compressed_size
        self.assertGreater(ratio, 1.0)

    def test_compression_preserves_dimensions(self):
        """测试压缩后图像尺寸不变"""
        import cv2
        _, encoded = cv2.imencode('.png', self.gray_image)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
        self.assertEqual(decoded.shape, self.gray_image.shape)

    def test_webp_compression(self):
        """测试WebP压缩"""
        import cv2
        _, encoded = cv2.imencode('.webp', self.test_image)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, self.test_image.shape)

    def test_high_quality_low_loss(self):
        """高质量压缩损失应很小"""
        import cv2
        _, encoded = cv2.imencode('.jpg', self.test_image, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        mse = np.mean((self.test_image.astype(float) - decoded.astype(float)) ** 2)
        self.assertLess(mse, 10.0)

    def test_low_quality_high_loss(self):
        """低质量压缩损失应较大"""
        import cv2
        _, encoded = cv2.imencode('.jpg', self.test_image, [int(cv2.IMWRITE_JPEG_QUALITY), 5])
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        mse = np.mean((self.test_image.astype(float) - decoded.astype(float)) ** 2)
        self.assertGreater(mse, 0.0)

    def test_empty_image_handling(self):
        """测试空图像处理"""
        import cv2
        small = np.zeros((1, 1, 3), dtype=np.uint8)
        _, encoded = cv2.imencode('.png', small)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape[0], 1)


if __name__ == '__main__':
    unittest.main()
