"""
颜色矩特征提取模块 - 一阶/二阶/三阶矩
适用于电赛中的颜色特征描述、目标分类和颜色匹配
"""

import cv2
import numpy as np


class ColorMomentExtractor:
    """颜色矩特征提取器"""

    def __init__(self, channels='BGR', n_moments=3):
        """
        初始化

        Args:
            channels: 颜色空间 'BGR', 'HSV', 'LAB'
            n_moments: 阶数(1=均值, +方差, +偏度)
        """
        self.channels = channels
        self.n_moments = n_moments

    def _convert_color(self, image):
        """转换颜色空间"""
        if self.channels == 'HSV':
            return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        elif self.channels == 'LAB':
            return cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        return image.copy()

    def _calc_moments_single_channel(self, channel):
        """
        计算单通道的颜色矩

        Args:
            channel: 单通道图像(HxW)

        Returns:
            list: [一阶矩(均值), 二阶矩(标准差), 三阶矩(偏度)]
        """
        pixels = channel.flatten().astype(np.float64)
        moments = []

        # 一阶矩: 均值
        mu = np.mean(pixels)
        moments.append(mu)

        if self.n_moments < 2:
            return moments

        # 二阶矩: 标准差
        sigma = np.std(pixels)
        moments.append(sigma)

        if self.n_moments < 3:
            return moments

        # 三阶矩: 偏度(skewness)
        if sigma > 1e-10:
            skewness = np.mean(((pixels - mu) / sigma) ** 3)
        else:
            skewness = 0.0
        moments.append(skewness)

        return moments

    def extract(self, image, mask=None):
        """
        提取颜色矩特征

        Args:
            image: BGR图像
            mask: 掩码(可选), 仅计算掩码区域

        Returns:
            numpy.ndarray: 颜色矩特征向量(长度 = 通道数 * n_moments)
        """
        converted = self._convert_color(image)
        channels = cv2.split(converted)

        feature = []
        for ch in channels:
            if mask is not None:
                pixels = ch[mask > 0]
                if len(pixels) == 0:
                    feature.extend([0.0] * self.n_moments)
                    continue
                ch_flat = pixels.astype(np.float64)
                moments = []
                mu = np.mean(ch_flat)
                moments.append(mu)
                if self.n_moments >= 2:
                    sigma = np.std(ch_flat)
                    moments.append(sigma)
                if self.n_moments >= 3:
                    if sigma > 1e-10:
                        moments.append(np.mean(((ch_flat - mu) / sigma) ** 3))
                    else:
                        moments.append(0.0)
                feature.extend(moments)
            else:
                feature.extend(self._calc_moments_single_channel(ch))

        return np.array(feature, dtype=np.float32)

    def extract_from_roi(self, image, roi):
        """
        从ROI区域提取颜色矩

        Args:
            image: BGR图像
            roi: (x, y, w, h) 或 4点坐标数组

        Returns:
            numpy.ndarray: 颜色矩特征
        """
        if isinstance(roi, (tuple, list)) and len(roi) == 4 and not isinstance(roi[0], (list, np.ndarray)):
            x, y, w, h = roi
            cropped = image[y:y+h, x:x+w]
            return self.extract(cropped)
        else:
            pts = np.array(roi, dtype=np.int32)
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
            cv2.fillConvexPoly(mask, pts, 255)
            return self.extract(image, mask=mask)

    def compare(self, feat1, feat2, method='euclidean'):
        """
        比较两个颜色矩特征

        Args:
            feat1, feat2: 颜色矩特征向量
            method: 'euclidean', 'cosine', 'manhattan'

        Returns:
            float: 距离/相似度(越小越相似)
        """
        f1 = np.array(feat1, dtype=np.float64)
        f2 = np.array(feat2, dtype=np.float64)

        if method == 'euclidean':
            return np.sqrt(np.sum((f1 - f2) ** 2))
        elif method == 'cosine':
            dot = np.dot(f1, f2)
            norm = np.linalg.norm(f1) * np.linalg.norm(f2)
            if norm < 1e-10:
                return 1.0
            return 1.0 - dot / norm
        elif method == 'manhattan':
            return np.sum(np.abs(f1 - f2))
        else:
            raise ValueError(f"未知方法: {method}")

    def describe(self, feature):
        """
        将特征向量转为可读描述

        Args:
            feature: 颜色矩特征

        Returns:
            dict: 每个通道的矩描述
        """
        channel_names = list(self.channels)
        moment_names = ['均值', '标准差', '偏度'][:self.n_moments]
        desc = {}
        for i, cname in enumerate(channel_names):
            channel_desc = {}
            for j, mname in enumerate(moment_names):
                idx = i * self.n_moments + j
                if idx < len(feature):
                    channel_desc[mname] = round(float(feature[idx]), 4)
            desc[cname] = channel_desc
        return desc


# ==================== 便捷函数 ====================

def extract_color_moments(image, mask=None, space='BGR', n=3):
    """
    便捷函数: 提取颜色矩

    Args:
        image: BGR图像
        mask: 掩码(可选)
        space: 颜色空间
        n: 阶数

    Returns:
        numpy.ndarray: 特征向量
    """
    extractor = ColorMomentExtractor(channels=space, n_moments=n)
    return extractor.extract(image, mask=mask)


def compare_by_color_moment(img1, img2, space='BGR'):
    """
    便捷函数: 用颜色矩比较两张图像的相似度

    Args:
        img1, img2: BGR图像

    Returns:
        float: 欧氏距离
    """
    extractor = ColorMomentExtractor(channels=space)
    f1 = extractor.extract(img1)
    f2 = extractor.extract(img2)
    return extractor.compare(f1, f2, method='euclidean')


# ==================== 测试 ====================

if __name__ == '__main__':
    # 创建测试图像
    img_red = np.zeros((100, 100, 3), dtype=np.uint8)
    img_red[:] = (0, 0, 255)  # 纯红

    img_green = np.zeros((100, 100, 3), dtype=np.uint8)
    img_green[:] = (0, 255, 0)  # 纯绿

    img_mixed = np.random.randint(0, 256, (100, 100, 3), dtype=np.uint8)

    extractor = ColorMomentExtractor(channels='BGR', n_moments=3)

    feat_red = extractor.extract(img_red)
    feat_green = extractor.extract(img_green)
    feat_mixed = extractor.extract(img_mixed)

    print("红色图像颜色矩:", feat_red)
    print("绿色图像颜色矩:", feat_green)
    print("随机图像颜色矩:", feat_mixed)

    print("\n红色 vs 绿色:", extractor.compare(feat_red, feat_green))
    print("红色 vs 随机:", extractor.compare(feat_red, feat_mixed))

    print("\n红色描述:", extractor.describe(feat_red))
