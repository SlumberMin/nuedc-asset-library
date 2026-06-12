"""
图像滤波器模块 - 均值/高斯/中值/双边/NLM 滤波
依赖: opencv-python, numpy
"""

import cv2
import numpy as np


class ImageFilter:
    """图像滤波器集合"""

    # ---- 均值滤波 ----

    @staticmethod
    def mean_filter(image, ksize=5):
        """
        均值滤波 (平滑去噪)
        :param image: BGR 或灰度图
        :param ksize: 核大小 (奇数)
        :return: 滤波后图像
        """
        k = ksize if ksize % 2 == 1 else ksize + 1
        return cv2.blur(image, (k, k))

    # ---- 高斯滤波 ----

    @staticmethod
    def gaussian_filter(image, ksize=5, sigma_x=0, sigma_y=0):
        """
        高斯滤波 (加权平滑)
        :param ksize: 核大小 (奇数)
        :param sigma_x: X 方向标准差, 0=自动
        :param sigma_y: Y 方向标准差, 0=自动
        """
        k = ksize if ksize % 2 == 1 else ksize + 1
        return cv2.GaussianBlur(image, (k, k), sigmaX=sigma_x, sigmaY=sigma_y)

    # ---- 中值滤波 ----

    @staticmethod
    def median_filter(image, ksize=5):
        """
        中值滤波 (椒盐噪声去除)
        :param ksize: 核大小 (奇数, >=3)
        """
        k = ksize if ksize % 2 == 1 else ksize + 1
        k = max(k, 3)
        return cv2.medianBlur(image, k)

    # ---- 双边滤波 ----

    @staticmethod
    def bilateral_filter(image, d=9, sigma_color=75, sigma_space=75):
        """
        双边滤波 (保边去噪)
        :param d: 邻域直径, 负值则由 sigma_space 计算
        :param sigma_color: 颜色空间标准差
        :param sigma_space: 坐标空间标准差
        """
        return cv2.bilateralFilter(image, d, sigma_color, sigma_space)

    # ---- NLM 去噪 (Non-Local Means) ----

    @staticmethod
    def nlm_filter(image, h=10, template_window=7, search_window=21):
        """
        非局部均值去噪 (效果最好但最慢)
        :param h: 滤波强度, 越大去噪越强 (推荐 3~15)
        :param template_window: 模板窗口大小 (奇数)
        :param search_window: 搜索窗口大小 (奇数)
        """
        if len(image.shape) == 2:
            return cv2.fastNlMeansDenoising(image, None, h, template_window, search_window)
        else:
            return cv2.fastNlMeansDenoisingColored(image, None, h, h,
                                                    template_window, search_window)

    # ---- 锐化滤波 ----

    @staticmethod
    def sharpen(image, strength=1.0):
        """
        锐化 (Unsharp Mask)
        :param strength: 锐化强度
        """
        blurred = cv2.GaussianBlur(image, (0, 0), 3)
        return cv2.addWeighted(image, 1 + strength, blurred, -strength, 0)

    @staticmethod
    def laplacian_sharpen(image, ksize=3):
        """拉普拉斯锐化"""
        lap = cv2.Laplacian(image, cv2.CV_64F, ksize=ksize)
        sharpened = cv2.convertScaleAbs(image.astype(np.float64) - 0.7 * lap)
        return sharpened

    # ---- 自定义卷积核 ----

    @staticmethod
    def custom_filter(image, kernel):
        """
        自定义卷积核滤波
        :param kernel: numpy 二维数组 (如 3x3)
        """
        kernel = np.array(kernel, dtype=np.float32)
        return cv2.filter2D(image, -1, kernel)

    # ---- 边缘保持滤波 ----

    @staticmethod
    def guided_filter(image, radius=8, eps=0.01):
        """
        导向滤波 (保边平滑, 用于 HDR/去雾/抠图)
        :param radius: 窗口半径
        :param eps: 正则化参数
        """
        src = image.astype(np.float32)
        if len(src.shape) == 3:
            # 分通道处理
            guide = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        else:
            guide = src

        mean_I = cv2.boxFilter(guide, -1, (radius, radius))
        mean_src = cv2.boxFilter(src, -1, (radius, radius))
        mean_Isrc = cv2.boxFilter(guide[:, :, None] * src if len(src.shape) == 3
                                  else guide * src, -1, (radius, radius))
        mean_II = cv2.boxFilter(guide * guide, -1, (radius, radius))

        cov_IS = mean_Isrc - mean_I[:, :, None] * mean_src if len(src.shape) == 3 \
            else mean_Isrc - mean_I * mean_src
        var_I = mean_II - mean_I * mean_I

        a = cov_IS / (var_I[:, :, None] + eps) if len(src.shape) == 3 \
            else cov_IS / (var_I + eps)
        b = mean_src - a * mean_I[:, :, None] if len(src.shape) == 3 \
            else mean_src - a * mean_I

        mean_a = cv2.boxFilter(a, -1, (radius, radius))
        mean_b = cv2.boxFilter(b, -1, (radius, radius))

        result = mean_a * guide[:, :, None] + mean_b if len(src.shape) == 3 \
            else mean_a * guide + mean_b
        return np.clip(result, 0, 255).astype(np.uint8)

    # ---- 批量对比 ----

    @staticmethod
    def compare_all(image, ksize=5):
        """
        应用所有滤波器并返回字典
        :return: dict {名称: 滤波结果}
        """
        return {
            'original': image,
            'mean': ImageFilter.mean_filter(image, ksize),
            'gaussian': ImageFilter.gaussian_filter(image, ksize),
            'median': ImageFilter.median_filter(image, ksize),
            'bilateral': ImageFilter.bilateral_filter(image),
            'nlm': ImageFilter.nlm_filter(image, h=10),
            'sharpen': ImageFilter.sharpen(image),
        }


# ---- 预定义卷积核 ----

KERNELS = {
    'edge_detect': np.array([[-1, -1, -1], [-1, 8, -1], [-1, -1, -1]]),
    'emboss':      np.array([[-2, -1, 0], [-1, 1, 1], [0, 1, 2]]),
    'sharpen':     np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]),
    'box_blur':    np.ones((3, 3)) / 9,
}


if __name__ == '__main__':
    img = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
    results = ImageFilter.compare_all(img)
    for name, result in results.items():
        print(f"{name}: shape={result.shape}, dtype={result.dtype}")
