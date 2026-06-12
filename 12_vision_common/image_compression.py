"""
图像压缩模块 - JPEG/PNG/WebP 格式压缩与质量控制
依赖: opencv-python, numpy
"""

import cv2
import numpy as np
import os


class ImageCompressor:
    """图像压缩器，支持 JPEG/PNG/WebP 格式"""

    # ---- JPEG 压缩 ----

    @staticmethod
    def compress_jpeg(image, quality=75):
        """
        JPEG 压缩
        :param image: BGR 图像 (numpy.ndarray)
        :param quality: 压缩质量 1-100
        :return: (压缩后图像, 压缩后字节数)
        """
        params = [cv2.IMWRITE_JPEG_QUALITY, int(np.clip(quality, 1, 100))]
        success, encoded = cv2.imencode('.jpg', image, params)
        if not success:
            raise ValueError("JPEG 编码失败")
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        return decoded, len(encoded.tobytes())

    @staticmethod
    def compress_jpeg_to_file(image, path, quality=75):
        """JPEG 压缩并保存到文件"""
        params = [cv2.IMWRITE_JPEG_QUALITY, int(np.clip(quality, 1, 100))]
        cv2.imwrite(path, image, params)

    # ---- PNG 压缩 ----

    @staticmethod
    def compress_png(image, compression_level=9):
        """
        PNG 压缩 (无损)
        :param image: BGR/BGRA 图像
        :param compression_level: 0-9, 越高压缩率越大但越慢
        :return: (压缩后图像, 压缩后字节数)
        """
        level = int(np.clip(compression_level, 0, 9))
        params = [cv2.IMWRITE_PNG_COMPRESSION, level]
        success, encoded = cv2.imencode('.png', image, params)
        if not success:
            raise ValueError("PNG 编码失败")
        decoded = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
        return decoded, len(encoded.tobytes())

    @staticmethod
    def compress_png_to_file(image, path, compression_level=9):
        """PNG 压缩并保存到文件"""
        level = int(np.clip(compression_level, 0, 9))
        params = [cv2.IMWRITE_PNG_COMPRESSION, level]
        cv2.imwrite(path, image, params)

    # ---- WebP 压缩 ----

    @staticmethod
    def compress_webp(image, quality=80):
        """
        WebP 压缩
        :param image: BGR 图像
        :param quality: 0-100 (0=有损最大, 100=无损)
        :return: (压缩后图像, 压缩后字节数)
        """
        q = int(np.clip(quality, 0, 100))
        params = [cv2.IMWRITE_WEBP_QUALITY, q]
        success, encoded = cv2.imencode('.webp', image, params)
        if not success:
            raise ValueError("WebP 编码失败")
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        return decoded, len(encoded.tobytes())

    @staticmethod
    def compress_webp_to_file(image, path, quality=80):
        """WebP 压缩并保存到文件"""
        q = int(np.clip(quality, 0, 100))
        params = [cv2.IMWRITE_WEBP_QUALITY, q]
        cv2.imwrite(path, image, params)

    # ---- 通用工具 ----

    @staticmethod
    def get_raw_size(image):
        """获取图像原始字节数 (未压缩)"""
        return image.nbytes

    @staticmethod
    def get_compression_ratio(original_bytes, compressed_bytes):
        """计算压缩率"""
        if compressed_bytes == 0:
            return float('inf')
        return original_bytes / compressed_bytes

    @staticmethod
    def calc_psnr(original, compressed):
        """计算 PSNR (峰值信噪比)"""
        mse = np.mean((original.astype(np.float64) - compressed.astype(np.float64)) ** 2)
        if mse == 0:
            return float('inf')
        return 10 * np.log10(255.0 ** 2 / mse)

    @staticmethod
    def calc_ssim_simple(original, compressed):
        """
        简化版 SSIM 计算 (基于均值/方差)
        :return: SSIM 值 (0~1)
        """
        orig = original.astype(np.float64)
        comp = compressed.astype(np.float64)
        mu_o = np.mean(orig)
        mu_c = np.mean(comp)
        sigma_o2 = np.var(orig)
        sigma_c2 = np.var(comp)
        sigma_oc = np.mean((orig - mu_o) * (comp - mu_c))

        C1 = (0.01 * 255) ** 2
        C2 = (0.03 * 255) ** 2

        numerator = (2 * mu_o * mu_c + C1) * (2 * sigma_oc + C2)
        denominator = (mu_o ** 2 + mu_c ** 2 + C1) * (sigma_o2 + sigma_c2 + C2)
        return numerator / denominator

    @staticmethod
    def auto_compress(image, target_kb, fmt='jpeg'):
        """
        自动压缩到目标大小
        :param image: BGR 图像
        :param target_kb: 目标大小 (KB)
        :param fmt: 格式 'jpeg' / 'webp'
        :return: (压缩后图像, 最终大小KB, 使用的质量)
        """
        target_bytes = target_kb * 1024
        lo, hi = 1, 100
        best_img, best_q = None, 75

        for _ in range(20):
            mid = (lo + hi) // 2
            if fmt == 'jpeg':
                img, size = ImageCompressor.compress_jpeg(image, mid)
            else:
                img, size = ImageCompressor.compress_webp(image, mid)
            best_img, best_q = img, mid
            if size > target_bytes:
                hi = mid - 1
            else:
                lo = mid + 1
            if lo > hi:
                break

        return best_img, size / 1024, best_q


# ---- 便捷函数 ----

def compress_image(image, fmt='jpeg', quality=75):
    """通用压缩接口"""
    fmt = fmt.lower()
    if fmt == 'jpeg' or fmt == 'jpg':
        return ImageCompressor.compress_jpeg(image, quality)
    elif fmt == 'png':
        return ImageCompressor.compress_png(image, quality)
    elif fmt == 'webp':
        return ImageCompressor.compress_webp(image, quality)
    else:
        raise ValueError(f"不支持的格式: {fmt}")


if __name__ == '__main__':
    # 测试示例
    img = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
    comp_jpeg, size_j = ImageCompressor.compress_jpeg(img, 50)
    comp_png, size_p = ImageCompressor.compress_png(img, 6)
    comp_webp, size_w = ImageCompressor.compress_webp(img, 50)
    print(f"原始大小: {ImageCompressor.get_raw_size(img)} bytes")
    print(f"JPEG@50: {size_j} bytes, 压缩率: {ImageCompressor.get_compression_ratio(img.nbytes, size_j):.1f}x")
    print(f"PNG@6:   {size_p} bytes, 压缩率: {ImageCompressor.get_compression_ratio(img.nbytes, size_p):.1f}x")
    print(f"WebP@50: {size_w} bytes, 压缩率: {ImageCompressor.get_compression_ratio(img.nbytes, size_w):.1f}x")
    print(f"JPEG PSNR: {ImageCompressor.calc_psnr(img, comp_jpeg):.2f} dB")
