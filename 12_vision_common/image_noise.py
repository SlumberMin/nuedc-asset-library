"""
图像噪声添加模块 - 高斯/椒盐/泊松/斑点噪声
适用场景: 数据增强、算法鲁棒性测试、去噪算法验证
依赖: opencv-python, numpy
"""

import cv2
import numpy as np


class ImageNoise:
    """图像噪声添加器"""

    # ---- 高斯噪声 ----

    @staticmethod
    def gaussian_noise(image, mean=0, sigma=25):
        """
        添加高斯噪声
        :param image: 输入图像 (uint8)
        :param mean: 噪声均值
        :param sigma: 噪声标准差 (越大噪声越强)
        :return: 添加噪声后的图像
        """
        noise = np.random.normal(mean, sigma, image.shape).astype(np.float64)
        noisy = np.clip(image.astype(np.float64) + noise, 0, 255)
        return noisy.astype(np.uint8)

    # ---- 椒盐噪声 ----

    @staticmethod
    def salt_pepper_noise(image, salt_prob=0.01, pepper_prob=0.01):
        """
        添加椒盐噪声 (随机白点和黑点)
        :param image: 输入图像
        :param salt_prob: 白点(盐)出现概率
        :param pepper_prob: 黑点(椒)出现概率
        :return: 添加噪声后的图像
        """
        noisy = image.copy()
        total_pixels = image.size

        # 盐噪声 (白点)
        num_salt = int(total_pixels * salt_prob)
        coords = tuple(np.random.randint(0, d, num_salt) for d in image.shape[:2])
        if len(image.shape) == 3:
            noisy[coords[0], coords[1], :] = 255
        else:
            noisy[coords] = 255

        # 椒噪声 (黑点)
        num_pepper = int(total_pixels * pepper_prob)
        coords = tuple(np.random.randint(0, d, num_pepper) for d in image.shape[:2])
        if len(image.shape) == 3:
            noisy[coords[0], coords[1], :] = 0
        else:
            noisy[coords] = 0

        return noisy

    # ---- 泊松噪声 ----

    @staticmethod
    def poisson_noise(image):
        """
        添加泊松噪声 (与信号强度相关, 适合模拟传感器噪声)
        :param image: 输入图像 (uint8)
        :return: 添加噪声后的图像
        """
        noisy = np.random.poisson(image.astype(np.float64))
        return np.clip(noisy, 0, 255).astype(np.uint8)

    # ---- 斑点噪声 (Speckle) ----

    @staticmethod
    def speckle_noise(image, intensity=0.1):
        """
        添加斑点噪声 (乘性噪声, 模拟雷达/超声图像)
        :param image: 输入图像
        :param intensity: 噪声强度 (0~1)
        :return: 添加噪声后的图像
        """
        noise = np.random.randn(*image.shape) * intensity
        noisy = image.astype(np.float64) * (1 + noise)
        return np.clip(noisy, 0, 255).astype(np.uint8)

    # ---- 综合添加 ----

    @classmethod
    def add_noise(cls, image, noise_type='gaussian', **kwargs):
        """
        统一噪声添加接口
        :param image: 输入图像
        :param noise_type: 噪声类型 'gaussian'|'salt_pepper'|'poisson'|'speckle'
        :return: 添加噪声后的图像
        """
        noise_map = {
            'gaussian': cls.gaussian_noise,
            'salt_pepper': cls.salt_pepper_noise,
            'poisson': cls.poisson_noise,
            'speckle': cls.speckle_noise,
        }
        func = noise_map.get(noise_type)
        if func is None:
            raise ValueError(f"不支持的噪声类型: {noise_type}, 可选: {list(noise_map.keys())}")
        return func(image, **kwargs)

    # ---- 批量生成 ----

    @classmethod
    def generate_noisy_dataset(cls, image, noise_types=None):
        """
        对同一图像生成多种噪声版本 (用于数据增强)
        :param image: 输入图像
        :param noise_types: 噪声类型列表, None 则生成全部
        :return: dict {noise_type: noisy_image}
        """
        if noise_types is None:
            noise_types = ['gaussian', 'salt_pepper', 'poisson', 'speckle']
        result = {}
        for nt in noise_types:
            result[nt] = cls.add_noise(image, nt)
        return result


# ======================== 快捷函数 ========================

def add_gaussian_noise(image, mean=0, sigma=25):
    return ImageNoise.gaussian_noise(image, mean, sigma)

def add_salt_pepper_noise(image, salt_prob=0.01, pepper_prob=0.01):
    return ImageNoise.salt_pepper_noise(image, salt_prob, pepper_prob)

def add_poisson_noise(image):
    return ImageNoise.poisson_noise(image)

def add_speckle_noise(image, intensity=0.1):
    return ImageNoise.speckle_noise(image, intensity)
