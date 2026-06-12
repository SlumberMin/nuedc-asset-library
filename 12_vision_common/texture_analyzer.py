"""
纹理分析器 - LBP + GLCM 特征
适用于: 材质识别、表面缺陷检测、纹理分类
"""

import cv2
import numpy as np
from enum import Enum


class TextureAnalyzer:
    """纹理分析器，支持LBP和GLCM特征"""

    def __init__(self):
        pass

    # ========== LBP (Local Binary Pattern) ==========

    @staticmethod
    def lbp_basic(image, radius=1, n_points=8):
        """
        基本LBP特征
        Args:
            image: 灰度图像
            radius: 半径
            n_points: 采样点数
        Returns:
            lbp_map: LBP图
        """
        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        h, w = image.shape
        lbp = np.zeros((h - 2 * radius, w - 2 * radius), dtype=np.uint8)

        for i in range(radius, h - radius):
            for j in range(radius, w - radius):
                center = image[i, j]
                code = 0
                for k in range(n_points):
                    angle = 2 * np.pi * k / n_points
                    y = int(i + radius * np.sin(angle))
                    x = int(j + radius * np.cos(angle))
                    y = np.clip(y, 0, h - 1)
                    x = np.clip(x, 0, w - 1)
                    code |= (1 << k) if image[y, x] >= center else 0
                lbp[i - radius, j - radius] = code

        return lbp

    @staticmethod
    def lbp_histogram(image, radius=1, n_points=8, grid_x=8, grid_y=8):
        """
        分块LBP直方图特征(空间增强)
        Args:
            image: 灰度图像
            grid_x, grid_y: 网格分块数
        Returns:
            feature: 拼接的LBP直方图向量
        """
        lbp = TextureAnalyzer.lbp_basic(image, radius, n_points)
        h, w = lbp.shape
        cell_h, cell_w = h // grid_y, w // grid_x

        features = []
        for gy in range(grid_y):
            for gx in range(grid_x):
                cell = lbp[gy * cell_h:(gy + 1) * cell_h, gx * cell_w:(gx + 1) * cell_w]
                hist, _ = np.histogram(cell.ravel(), bins=2 ** n_points, range=(0, 2 ** n_points))
                hist = hist.astype(np.float64)
                hist /= (hist.sum() + 1e-7)
                features.append(hist)

        return np.concatenate(features)

    # ========== GLCM (Gray-Level Co-occurrence Matrix) ==========

    @staticmethod
    def glcm_matrix(image, distance=1, angle=0, levels=64):
        """
        计算灰度共生矩阵(GLCM)
        Args:
            image: 灰度图像
            distance: 像素间距
            angle: 角度(0, 45, 90, 135)
            levels: 灰度量化级数
        Returns:
            glcm: 归一化共生矩阵
        """
        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 量化灰度级
        image = (image * levels // 256).astype(np.int32)

        h, w = image.shape
        glcm = np.zeros((levels, levels), dtype=np.float64)

        # 方向偏移
        angle_rad = np.radians(angle)
        dy = int(round(distance * np.sin(angle_rad)))
        dx = int(round(distance * np.cos(angle_rad)))

        for i in range(max(0, -dy), min(h, h - dy)):
            for j in range(max(0, -dx), min(w, w - dx)):
                row = image[i, j]
                col = image[i + dy, j + dx]
                glcm[row, col] += 1
                glcm[col, row] += 1  # 对称

        # 归一化
        total = glcm.sum()
        if total > 0:
            glcm /= total

        return glcm

    @staticmethod
    def glcm_features(image, distances=(1, 2), angles=(0, 45, 90, 135), levels=64):
        """
        提取GLCM纹理特征
        Returns:
            features: [contrast, correlation, energy, homogeneity, entropy, ...]
        """
        all_features = []

        for d in distances:
            for a in angles:
                glcm = TextureAnalyzer.glcm_matrix(image, d, a, levels)
                props = TextureAnalyzer._glcm_properties(glcm)
                all_features.extend(props)

        return np.array(all_features)

    @staticmethod
    def _glcm_properties(glcm):
        """计算GLCM统计属性"""
        levels = glcm.shape[0]

        # 生成坐标网格
        i, j = np.meshgrid(np.arange(levels), np.arange(levels), indexing='ij')

        # 对比度 Contrast
        contrast = np.sum(glcm * (i - j) ** 2)

        # 能量(角二阶矩) Energy / ASM
        energy = np.sum(glcm ** 2)

        # 同质性 Homogeneity
        homogeneity = np.sum(glcm / (1.0 + np.abs(i - j)))

        # 熵 Entropy
        nonzero = glcm[glcm > 0]
        entropy = -np.sum(nonzero * np.log2(nonzero))

        # 相关性 Correlation
        mu_i = np.sum(i * glcm)
        mu_j = np.sum(j * glcm)
        sigma_i = np.sqrt(np.sum((i - mu_i) ** 2 * glcm))
        sigma_j = np.sqrt(np.sum((j - mu_j) ** 2 * glcm))
        if sigma_i > 0 and sigma_j > 0:
            correlation = np.sum((i - mu_i) * (j - mu_j) * glcm) / (sigma_i * sigma_j)
        else:
            correlation = 0.0

        return [contrast, correlation, energy, homogeneity, entropy]

    # ========== 综合分析 ==========

    def analyze(self, image):
        """
        综合纹理分析
        Returns:
            dict: 包含LBP和GLCM特征
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image

        result = {
            'lbp_feature': self.lbp_histogram(gray),
            'glcm_feature': self.glcm_features(gray),
            'lbp_map': self.lbp_basic(gray),
        }
        return result

    def compare(self, image1, image2, method='lbp'):
        """
        比较两张图像的纹理相似度
        Returns:
            similarity: 0~1
        """
        if method == 'lbp':
            f1 = self.lbp_histogram(cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY) if len(image1.shape) == 3 else image1)
            f2 = self.lbp_histogram(cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY) if len(image2.shape) == 3 else image2)
            # 卡方距离
            chi2 = np.sum((f1 - f2) ** 2 / (f1 + f2 + 1e-10))
            return 1.0 / (1.0 + chi2)
        elif method == 'glcm':
            f1 = self.glcm_features(image1)
            f2 = self.glcm_features(image2)
            # 余弦相似度
            dot = np.dot(f1, f2)
            norm = np.linalg.norm(f1) * np.linalg.norm(f2)
            return dot / (norm + 1e-10) if norm > 0 else 0
        return 0

    def detect_anomaly(self, image, reference_features, threshold=0.3):
        """
        基于纹理特征的异常检测
        Args:
            image: 待检测图像
            reference_features: 正常样本的特征向量
            threshold: 异常阈值
        Returns:
            (is_anomaly, distance)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        feature = self.lbp_histogram(gray)

        dist = np.linalg.norm(feature - reference_features)
        return dist > threshold, dist


def visualize_lbp(image):
    """可视化LBP图"""
    analyzer = TextureAnalyzer()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    lbp = analyzer.lbp_basic(gray)

    cv2.imshow("Original", image if len(image.shape) == 3 else cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))
    cv2.imshow("LBP", lbp)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def demo():
    """纹理分析演示"""
    # 创建合成纹理
    h, w = 128, 128
    # 纹理1: 水平条纹
    tex1 = np.zeros((h, w), dtype=np.uint8)
    for i in range(0, h, 4):
        tex1[i:i+2, :] = 200
    tex1 = cv2.GaussianBlur(tex1, (3, 3), 0)

    # 纹理2: 垂直条纹
    tex2 = np.zeros((h, w), dtype=np.uint8)
    for j in range(0, w, 4):
        tex2[:, j:j+2] = 200
    tex2 = cv2.GaussianBlur(tex2, (3, 3), 0)

    # 纹理3: 随机噪声
    tex3 = np.random.randint(0, 256, (h, w), dtype=np.uint8)

    analyzer = TextureAnalyzer()

    for name, tex in [("水平条纹", tex1), ("垂直条纹", tex2), ("随机噪声", tex3)]:
        f = analyzer.glcm_features(tex, distances=(1,), angles=(0, 90), levels=32)
        print(f"\n{name}:")
        print(f"  Contrast:    {f[0]:.4f}")
        print(f"  Correlation: {f[1]:.4f}")
        print(f"  Energy:      {f[2]:.4f}")
        print(f"  Homogeneity: {f[3]:.4f}")
        print(f"  Entropy:     {f[4]:.4f}")

    # 比较
    sim = analyzer.compare(cv2.cvtColor(tex1, cv2.COLOR_GRAY2BGR),
                           cv2.cvtColor(tex2, cv2.COLOR_GRAY2BGR), method='lbp')
    print(f"\n水平vs垂直相似度: {sim:.4f}")

    sim = analyzer.compare(cv2.cvtColor(tex1, cv2.COLOR_GRAY2BGR),
                           cv2.cvtColor(tex1, cv2.COLOR_GRAY2BGR), method='lbp')
    print(f"水平vs水平相似度: {sim:.4f}")


if __name__ == "__main__":
    demo()
