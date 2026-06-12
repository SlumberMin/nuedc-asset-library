"""
图像增强V2模块 - 对比度/亮度/锐化/去雾
适用场景: 电赛中图像预处理、弱光增强、雾霾场景处理
依赖: opencv-python, numpy
"""

import cv2
import numpy as np


class ImageEnhanceV2:
    """图像增强工具集V2"""

    # ---- 对比度增强 ----

    @staticmethod
    def adjust_contrast(image, alpha=1.5):
        """
        调整对比度
        :param image: 输入图像
        :param alpha: 对比度系数 (>1 增强, <1 降低)
        :return: 增强后图像
        """
        return cv2.convertScaleAbs(image, alpha=alpha, beta=0)

    @staticmethod
    def clahe_enhance(image, clip_limit=2.0, tile_grid_size=(8, 8)):
        """
        CLAHE 自适应直方图均衡化 (局部对比度增强)
        :param image: 灰度图或BGR图
        :param clip_limit: 对比度限制
        :param tile_grid_size: 网格大小
        :return: 增强后图像
        """
        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
        if len(image.shape) == 3:
            # 在LAB空间的L通道操作, 保持颜色不失真
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            lab[:, :, 0] = clahe.apply(lab[:, :, 0])
            return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        else:
            return clahe.apply(image)

    @staticmethod
    def histogram_equalize(image):
        """
        全局直方图均衡化 (仅灰度图)
        :param image: 灰度图
        :return: 均衡化后图像
        """
        if len(image.shape) == 3:
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            lab[:, :, 0] = cv2.equalizeHist(lab[:, :, 0])
            return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        else:
            return cv2.equalizeHist(image)

    # ---- 亮度调整 ----

    @staticmethod
    def adjust_brightness(image, beta=30):
        """
        调整亮度
        :param image: 输入图像
        :param beta: 亮度偏移值 (>0 变亮, <0 变暗)
        :return: 调整后图像
        """
        return cv2.convertScaleAbs(image, alpha=1.0, beta=beta)

    @staticmethod
    def gamma_correction(image, gamma=1.0):
        """
        Gamma 校正 (非线性亮度调节)
        :param image: 输入图像 (uint8)
        :param gamma: <1 变亮, >1 变暗
        :return: 校正后图像
        """
        inv_gamma = 1.0 / gamma
        table = np.array([(i / 255.0) ** inv_gamma * 255
                          for i in range(256)]).astype(np.uint8)
        return cv2.LUT(image, table)

    # ---- 锐化 ----

    @staticmethod
    def sharpen(image, strength=1.0):
        """
        卷积核锐化
        :param image: 输入图像
        :param strength: 锐化强度 (0~2)
        :return: 锐化后图像
        """
        kernel = np.array([[0, -1, 0],
                           [-1, 4 + strength, -1],
                           [0, -1, 0]], dtype=np.float32) / (strength + 1)
        return cv2.filter2D(image, -1, kernel)

    @staticmethod
    def unsharp_mask(image, sigma=1.0, strength=1.5):
        """
        USM 锐化 (反锐化掩模, 效果更自然)
        :param image: 输入图像
        :param sigma: 高斯模糊半径
        :param strength: 锐化强度
        :return: 锐化后图像
        """
        blurred = cv2.GaussianBlur(image, (0, 0), sigma)
        sharpened = cv2.addWeighted(image, 1 + strength, blurred, -strength, 0)
        return np.clip(sharpened, 0, 255).astype(np.uint8)

    # ---- 去雾 (暗通道先验) ----

    @staticmethod
    def dehaze(image, omega=0.95, t_min=0.1, guided_radius=60, guided_eps=1e-3):
        """
        暗通道先验去雾 (He et al.)
        :param image: 输入BGR图像 (有雾)
        :param omega: 去雾程度 (0~1, 越大去雾越强)
        :param t_min: 透射率下限
        :param guided_radius: 引导滤波半径
        :param guided_eps: 引导滤波正则化参数
        :return: 去雾后图像
        """
        img_float = image.astype(np.float64) / 255.0
        h, w = image.shape[:2]

        # 1. 暗通道计算
        min_channel = np.min(img_float, axis=2)
        kernel_size = max(15, int(max(h, w) * 0.02)) | 1  # 确保奇数
        dark_channel = cv2.erode(min_channel, np.ones((kernel_size, kernel_size)))

        # 2. 估计大气光值
        num_pixels = h * w
        num_brightest = int(num_pixels * 0.001)
        flat_dark = dark_channel.ravel()
        indices = np.argsort(flat_dark)[-num_brightest:]
        atmospheric = np.max(img_float.reshape(num_pixels, 3)[indices], axis=0)

        # 3. 估计透射率
        transmission = 1 - omega * dark_channel / atmospheric
        transmission = np.maximum(transmission, t_min)

        # 4. 引导滤波细化透射率 (或用高斯模糊近似)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float64) / 255.0
        try:
            transmission_refined = cv2.ximgproc.guidedFilter(
                guide=gray.astype(np.float32),
                src=transmission.astype(np.float32),
                radius=guided_radius, eps=guided_eps
            )
        except AttributeError:
            # 无 ximgproc 模块时回退到高斯模糊
            k = guided_radius * 2 + 1
            transmission_refined = cv2.GaussianBlur(transmission, (k, k), 0)

        transmission_refined = np.maximum(transmission_refined, t_min)

        # 5. 恢复无雾图像
        result = np.zeros_like(img_float)
        for c in range(3):
            result[:, :, c] = (img_float[:, :, c] - atmospheric[c]) / transmission_refined + atmospheric[c]

        return np.clip(result * 255, 0, 255).astype(np.uint8)

    @staticmethod
    def dehaze_simple(image, clip_limit=3.0):
        """
        简易去雾 (CLAHE方法, 速度快, 适合实时场景)
        :param image: 输入图像
        :param clip_limit: CLAHE对比度限制
        :return: 去雾效果图像
        """
        return ImageEnhanceV2.clahe_enhance(image, clip_limit=clip_limit)


# ======================== 快捷函数 ========================

def adjust_contrast(image, alpha=1.5):
    return ImageEnhanceV2.adjust_contrast(image, alpha)

def adjust_brightness(image, beta=30):
    return ImageEnhanceV2.adjust_brightness(image, beta)

def gamma_correction(image, gamma=1.0):
    return ImageEnhanceV2.gamma_correction(image, gamma)

def sharpen(image, strength=1.0):
    return ImageEnhanceV2.sharpen(image, strength)

def unsharp_mask(image, sigma=1.0, strength=1.5):
    return ImageEnhanceV2.unsharp_mask(image, sigma, strength)

def dehaze(image, omega=0.95, t_min=0.1):
    return ImageEnhanceV2.dehaze(image, omega, t_min)
