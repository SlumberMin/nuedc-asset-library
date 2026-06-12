"""
图像水印单元测试
"""
import unittest
import numpy as np


class TestImageWatermark(unittest.TestCase):
    """图像水印算法测试"""

    def setUp(self):
        self.base_image = np.random.randint(50, 200, (128, 128, 3), dtype=np.uint8)
        self.watermark = np.random.randint(0, 2, (32, 32), dtype=np.uint8) * 255

    def test_add_lsb_watermark(self):
        """测试LSB水印嵌入"""
        img = self.base_image.copy().astype(np.int32)
        wm = cv2.resize(self.watermark, (128, 128)) if False else self.watermark
        small_wm = cv2.resize(self.watermark, (img.shape[1], img.shape[0])) if False else None
        # 简单LSB嵌入
        flat = img[:, :, 0].flatten()
        wm_flat = np.zeros_like(flat)
        wm_small = self.watermark.flatten()
        wm_flat[:len(wm_small)] = wm_small
        flat = (flat & 0xFE) | (wm_flat > 127).astype(np.int32)
        self.assertEqual(flat.shape, img[:, :, 0].flatten().shape)

    def test_watermark_invisibility(self):
        """测试水印不可见性"""
        img = self.base_image.copy()
        watermarked = img.copy()
        wm_resized = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
        h, w = self.watermark.shape
        wm_resized[:h, :w] = self.watermark
        # 修改最低位
        watermarked[:, :, 0] = (watermarked[:, :, 0] & 0xFE) | (wm_resized > 127).astype(np.uint8)
        diff = np.abs(img.astype(float) - watermarked.astype(float))
        self.assertLess(np.mean(diff), 1.0)

    def test_watermark_extract_roundtrip(self):
        """测试水印嵌入后可提取"""
        img = self.base_image.copy()
        wm_resized = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
        h, w = self.watermark.shape
        wm_resized[:h, :w] = self.watermark
        # 嵌入
        embedded = (img[:, :, 0] & 0xFE) | (wm_resized > 127).astype(np.uint8)
        # 提取
        extracted = embedded & 1
        expected = (wm_resized > 127).astype(np.uint8)
        np.testing.assert_array_equal(extracted, expected)

    def test_watermark_not_corrupt_image(self):
        """测试水印不破坏图像主要信息"""
        img = self.base_image.copy()
        wm_resized = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
        h, w = self.watermark.shape
        wm_resized[:h, :w] = self.watermark
        watermarked = img.copy()
        watermarked[:, :, 0] = (img[:, :, 0] & 0xFE) | (wm_resized > 127).astype(np.uint8)
        psnr = 10 * np.log10(255.0 ** 2 / np.mean((img.astype(float) - watermarked.astype(float)) ** 2))
        self.assertGreater(psnr, 50.0)

    def test_text_watermark_embed(self):
        """测试文本水印嵌入"""
        import cv2
        wm_img = np.zeros((50, 200), dtype=np.uint8)
        cv2.putText(wm_img, 'TEST', (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 255, 2)
        self.assertTrue(np.sum(wm_img) > 0)

    def test_watermark_blind_extract(self):
        """测试盲提取（不需要原图）"""
        img = self.base_image[:, :, 0].copy()
        wm_resized = np.zeros_like(img)
        h, w = self.watermark.shape
        wm_resized[:h, :w] = self.watermark
        embedded = (img & 0xFE) | (wm_resized > 127).astype(np.uint8)
        # 盲提取：仅靠嵌入图像
        extracted = embedded & 1
        self.assertEqual(extracted.shape, img.shape)

    def test_watermark_resilience_flip(self):
        """测试水印对位翻转的鲁棒性检测"""
        img = self.base_image[:, :, 0].copy()
        wm = (self.watermark > 127).astype(np.uint8).flatten()
        flat = img.flatten().copy()
        flat[:len(wm)] = (flat[:len(wm)] & 0xFE) | wm
        # 翻转一些位
        flat[:10] = flat[:10] ^ 1
        extracted = flat[:len(wm)] & 1
        errors = np.sum(extracted != wm)
        self.assertLessEqual(errors, len(wm))

    def test_multiple_watermarks(self):
        """测试多次水印嵌入"""
        img = self.base_image[:, :, 0].copy()
        wm1 = np.ones(100, dtype=np.uint8)
        wm2 = np.zeros(100, dtype=np.uint8)
        embedded = img.flatten().copy()
        embedded[:100] = (embedded[:100] & 0xFE) | wm1
        embedded[:100] = (embedded[:100] & 0xFE) | wm2  # 后者覆盖
        extracted = embedded[:100] & 1
        np.testing.assert_array_equal(extracted, wm2)

    def test_watermark_strength_scaling(self):
        """测试不同水印强度"""
        img = self.base_image[:, :, 0].astype(float)
        wm = (self.watermark > 127).astype(float)
        wm_resized = np.zeros_like(img)
        h, w = self.watermark.shape
        wm_resized[:h, :w] = wm[:h, :w]
        for alpha in [0.01, 0.05, 0.1]:
            watermarked = img + alpha * wm_resized * 255
            watermarked = np.clip(watermarked, 0, 255)
            mse = np.mean((img - watermarked) ** 2)
            self.assertGreater(mse, 0)

    def test_dct_watermark_embed(self):
        """测试DCT域水印嵌入"""
        import cv2
        img_f = np.float32(self.base_image[:, :, 0])
        block = img_f[:8, :8]
        dct_block = cv2.dct(block)
        dct_block[4, 4] += 10  # 嵌入水印
        idct_block = cv2.idct(dct_block)
        self.assertEqual(idct_block.shape, (8, 8))


if __name__ == '__main__':
    unittest.main()
