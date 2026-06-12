"""
LBP特征提取模块 - 局部二值模式
适用于电赛中的纹理分类、材质识别和表面缺陷检测
"""

import cv2
import numpy as np


class LBPFeatureExtractor:
    """LBP(局部二值模式)特征提取器"""

    def __init__(self, radius=1, n_points=8, method='uniform'):
        """
        初始化

        Args:
            radius: 采样半径
            n_points: 采样点数
            method: 'default', 'uniform', 'ri'(旋转不变), 'riu2'(旋转不变均匀)
        """
        self.radius = radius
        self.n_points = n_points
        self.method = method

    def _lbp_default(self, gray):
        """
        计算默认LBP

        Args:
            gray: 灰度图(HxW)

        Returns:
            numpy.ndarray: LBP图像
        """
        h, w = gray.shape
        lbp = np.zeros((h - 2 * self.radius, w - 2 * self.radius), dtype=np.uint8)

        for i in range(self.radius, h - self.radius):
            for j in range(self.radius, w - self.radius):
                center = gray[i, j]
                code = 0
                for k in range(self.n_points):
                    # 等间隔采样点坐标
                    angle = 2 * np.pi * k / self.n_points
                    y = i + self.radius * np.sin(angle)
                    x = j + self.radius * np.cos(angle)

                    # 双线性插值
                    y0, x0 = int(np.floor(y)), int(np.floor(x))
                    y1, x1 = y0 + 1, x0 + 1
                    wy = y - y0
                    wx = x - x0

                    val = (gray[y0, x0] * (1-wy) * (1-wx) +
                           gray[y0, x1] * (1-wy) * wx +
                           gray[y1, x0] * wy * (1-wx) +
                           gray[y1, x1] * wy * wx)

                    if val >= center:
                        code |= (1 << k)

                lbp[i - self.radius, j - self.radius] = code

        return lbp

    def _lbp_uniform(self, gray):
        """
        计算均匀LBP(Uniform LBP)
        只保留至多2次0/1跳变的模式

        Args:
            gray: 灰度图

        Returns:
            numpy.ndarray: LBP图像(值为均匀模式编号)
        """
        lbp_raw = self._lbp_default(gray)

        # 构建均匀模式查找表
        n_patterns = 2 ** self.n_points
        lookup = np.zeros(n_patterns, dtype=np.int32)
        uniform_count = 0

        for i in range(n_patterns):
            if self._is_uniform(i):
                lookup[i] = uniform_count
                uniform_count += 1
            else:
                lookup[i] = uniform_count  # 所有非均匀模式归为同一类

        return lookup[lbp_raw].astype(np.uint8)

    def _is_uniform(self, pattern):
        """判断是否为均匀模式(0/1跳变不超过2次)"""
        binary = format(pattern, f'0{self.n_points}b')
        transitions = 0
        for i in range(len(binary)):
            if binary[i] != binary[(i + 1) % len(binary)]:
                transitions += 1
        return transitions <= 2

    def _lbp_rotation_invariant(self, gray):
        """旋转不变LBP"""
        h, w = gray.shape
        lbp = np.zeros((h - 2 * self.radius, w - 2 * self.radius), dtype=np.uint8)

        for i in range(self.radius, h - self.radius):
            for j in range(self.radius, w - self.radius):
                center = gray[i, j]
                code = 0
                for k in range(self.n_points):
                    angle = 2 * np.pi * k / self.n_points
                    y = i + self.radius * np.sin(angle)
                    x = j + self.radius * np.cos(angle)
                    y0, x0 = int(np.floor(y)), int(np.floor(x))
                    y1, x1 = y0 + 1, x0 + 1
                    wy = y - y0
                    wx = x - x0
                    val = (gray[y0, x0] * (1-wy) * (1-wx) +
                           gray[y0, x1] * (1-wy) * wx +
                           gray[y1, x0] * wy * (1-wx) +
                           gray[y1, x1] * wy * wx)
                    if val >= center:
                        code |= (1 << k)

                # 旋转不变: 所有循环移位中取最小值
                min_code = code
                for k in range(1, self.n_points):
                    rotated = ((code >> k) | (code << (self.n_points - k))) & ((1 << self.n_points) - 1)
                    min_code = min(min_code, rotated)

                lbp[i - self.radius, j - self.radius] = min_code

        return lbp

    def compute(self, image):
        """
        计算LBP图像

        Args:
            image: 灰度或BGR图像

        Returns:
            numpy.ndarray: LBP编码图
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        if self.method == 'default':
            return self._lbp_default(gray)
        elif self.method == 'uniform':
            return self._lbp_uniform(gray)
        elif self.method == 'ri':
            return self._lbp_rotation_invariant(gray)
        elif self.method == 'riu2':
            lbp_ri = self._lbp_rotation_invariant(gray)
            # 进一步合并非均匀模式
            n_patterns = 2 ** self.n_points
            lookup = np.zeros(n_patterns, dtype=np.int32)
            count = 0
            for i in range(n_patterns):
                if self._is_uniform(i):
                    lookup[i] = count
                    count += 1
                else:
                    lookup[i] = count
            return lookup[lbp_ri].astype(np.uint8)
        else:
            raise ValueError(f"未知方法: {self.method}")

    def extract_histogram(self, image, n_div_x=1, n_div_y=1):
        """
        提取LBP直方图特征(分块直方图)

        Args:
            image: 输入图像
            n_div_x: 水平分块数
            n_div_y: 垂直分块数

        Returns:
            numpy.ndarray: 分块LBP直方图拼接的特征向量
        """
        lbp = self.compute(image)
        h, w = lbp.shape

        # 计算直方图bin数
        if self.method == 'default':
            n_bins = 2 ** self.n_points
        elif self.method == 'uniform':
            n_bins = self._count_uniform_patterns() + 1
        elif self.method == 'ri':
            n_bins = 2 ** self.n_points  # 上界
        else:
            n_bins = self._count_uniform_patterns() + 1

        block_h = h // n_div_y
        block_w = w // n_div_x

        features = []
        for by in range(n_div_y):
            for bx in range(n_div_x):
                y1 = by * block_h
                y2 = (by + 1) * block_h if by < n_div_y - 1 else h
                x1 = bx * block_w
                x2 = (bx + 1) * block_w if bx < n_div_x - 1 else w
                block = lbp[y1:y2, x1:x2]
                hist, _ = np.histogram(block.flatten(),
                                       bins=min(n_bins, 256),
                                       range=(0, 255))
                hist = hist.astype(np.float64)
                if hist.sum() > 0:
                    hist /= hist.sum()
                features.append(hist)

        return np.concatenate(features)

    def _count_uniform_patterns(self):
        """计算均匀模式数"""
        count = 0
        for i in range(2 ** self.n_points):
            if self._is_uniform(i):
                count += 1
        return count

    def compare(self, feat1, feat2, method='chi_square'):
        """
        比较两个LBP特征

        Args:
            feat1, feat2: LBP直方图特征
            method: 'chi_square', 'hist_intersection', 'euclidean'

        Returns:
            float: 距离(越小越相似)
        """
        f1 = np.array(feat1, dtype=np.float64) + 1e-10
        f2 = np.array(feat2, dtype=np.float64) + 1e-10

        if method == 'chi_square':
            return 0.5 * np.sum((f1 - f2) ** 2 / (f1 + f2))
        elif method == 'hist_intersection':
            return 1.0 - np.sum(np.minimum(f1, f2)) / np.sum(f1)
        elif method == 'euclidean':
            return np.sqrt(np.sum((f1 - f2) ** 2))
        else:
            raise ValueError(f"未知方法: {method}")


# ==================== 便捷函数 ====================

def extract_lbp_feature(image, radius=1, n_points=8, grid_x=4, grid_y=4):
    """
    便捷函数: 提取分块LBP直方图特征

    Args:
        image: 输入图像
        radius: 采样半径
        n_points: 采样点数
        grid_x, grid_y: 分块数

    Returns:
        numpy.ndarray: 特征向量
    """
    extractor = LBPFeatureExtractor(radius=radius, n_points=n_points, method='uniform')
    return extractor.extract_histogram(image, n_div_x=grid_x, n_div_y=grid_y)


def compute_lbp_image(image, radius=1, n_points=8):
    """
    便捷函数: 计算LBP编码图

    Args:
        image: 输入图像
        radius: 采样半径
        n_points: 采样点数

    Returns:
        numpy.ndarray: LBP图
    """
    extractor = LBPFeatureExtractor(radius=radius, n_points=n_points)
    return extractor.compute(image)


# ==================== 测试 ====================

if __name__ == '__main__':
    # 创建不同纹理的测试图像
    h, w = 100, 100

    # 纹理1: 水平条纹
    img1 = np.zeros((h, w), dtype=np.uint8)
    for i in range(0, h, 10):
        img1[i:i+5, :] = 200

    # 纹理2: 棋盘格
    img2 = np.zeros((h, w), dtype=np.uint8)
    for i in range(0, h, 20):
        for j in range(0, w, 20):
            if (i // 20 + j // 20) % 2 == 0:
                img2[i:i+20, j:j+20] = 200

    # 纹理3: 随机噪声
    img3 = np.random.randint(0, 256, (h, w), dtype=np.uint8)

    extractor = LBPFeatureExtractor(radius=1, n_points=8, method='uniform')

    # 计算LBP图
    lbp1 = extractor.compute(img1)
    lbp2 = extractor.compute(img2)
    lbp3 = extractor.compute(img3)

    print(f"条纹纹理LBP范围: [{lbp1.min()}, {lbp1.max()}]")
    print(f"棋盘纹理LBP范围: [{lbp2.min()}, {lbp2.max()}]")

    # 提取分块特征
    feat1 = extractor.extract_histogram(img1, n_div_x=4, n_div_y=4)
    feat2 = extractor.extract_histogram(img2, n_div_x=4, n_div_y=4)
    feat3 = extractor.extract_histogram(img3, n_div_x=4, n_div_y=4)

    print(f"\nLBP特征维度: {feat1.shape}")
    print(f"条纹 vs 棋盘 距离: {extractor.compare(feat1, feat2):.4f}")
    print(f"条纹 vs 噪声 距离: {extractor.compare(feat1, feat3):.4f}")
    print(f"条纹 vs 条纹 距离: {extractor.compare(feat1, feat1):.4f}")
