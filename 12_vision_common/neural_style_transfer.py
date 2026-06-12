#!/usr/bin/env python3
"""
神经风格迁移模块 - 轻量级前馈网络 + 实时风格化
适用于电赛图像艺术化、实时视频风格化等任务
依赖: numpy, opencv-python
"""

import cv2
import numpy as np
from typing import Optional, Tuple


class NeuralStyleTransfer:
    """神经风格迁移：支持传统优化方法和快速前馈近似"""

    # 预设风格类型
    STYLE_PRESETS = {
        'oil_painting': '油画风格',
        'sketch': '素描风格',
        'watercolor': '水彩风格',
        'cartoon': '卡通风格',
        'impressionist': '印象派风格',
    }

    def __init__(self, output_size: Tuple = (512, 512)):
        """
        初始化
        Args:
            output_size: 输出图像尺寸 (W, H)
        """
        self.output_size = output_size

    def gram_matrix(self, features: np.ndarray) -> np.ndarray:
        """
        计算Gram矩阵（风格特征表示）
        Args:
            features: (C, H*W) 特征图
        Returns:
            (C, C) Gram矩阵
        """
        c, hw = features.shape
        G = features @ features.T / hw
        return G

    def extract_features(self, image: np.ndarray,
                         layers: list = None) -> dict:
        """
        用VGG-like特征提取（轻量级实现）
        Args:
            image: (H,W,3) BGR图像
            layers: 提取层名称
        Returns:
            各层特征字典
        """
        if layers is None:
            layers = ['conv1', 'conv2', 'conv3', 'conv4']

        # 简化的多尺度特征提取（不依赖VGG权重）
        img = image.astype(np.float32) / 255.0
        features = {}

        # 多尺度卷积特征（模拟VGG不同层的感受野）
        kernels = [
            # conv1: 边缘/纹理
            [np.array([[-1, -1, -1], [-1, 8, -1], [-1, -1, -1]], dtype=np.float32)],
            # conv2: 角点/简单图案
            [cv2.getGaussianKernel(3, 0.5) @ cv2.getGaussianKernel(3, 0.5).T],
            # conv3: 局部结构
            [cv2.getGaussianKernel(5, 1.0) @ cv2.getGaussianKernel(5, 1.0).T],
            # conv4: 全局布局
            [cv2.getGaussianKernel(7, 1.5) @ cv2.getGaussianKernel(7, 1.5).T],
        ]

        # 转灰度用于特征提取
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0

        for i, name in enumerate(layers):
            kernel = kernels[i][0]
            feat = cv2.filter2D(gray, -1, kernel)
            # 下采样模拟深层特征
            scale = 2 ** i
            feat = cv2.resize(feat, (feat.shape[1] // scale, feat.shape[0] // scale))
            features[name] = feat

        return features

    def extract_style_features(self, style_image: np.ndarray) -> dict:
        """提取风格图像的Gram矩阵特征"""
        features = self.extract_features(style_image)
        style_grams = {}
        for name, feat in features.items():
            flat = feat.reshape(1, -1)
            style_grams[name] = self.gram_matrix(flat)
        return style_grams

    def transfer_optimize(self, content_img: np.ndarray, style_img: np.ndarray,
                          iterations: int = 100,
                          content_weight: float = 1.0,
                          style_weight: float = 1e5,
                          learning_rate: float = 0.01) -> np.ndarray:
        """
        基于优化的风格迁移（迭代法）
        Args:
            content_img: 内容图 (H,W,3) BGR
            style_img: 风格图 (H,W,3) BGR
            iterations: 迭代次数
            content_weight: 内容损失权重
            style_weight: 风格损失权重
            learning_rate: 学习率
        Returns:
            风格化后的图像
        """
        # 统一尺寸
        h, w = content_img.shape[:2]
        style_resized = cv2.resize(style_img, (w, h))

        # 提取内容和风格特征
        content_features = self.extract_features(content_img)
        style_grams = self.extract_style_features(style_resized)

        # 用内容图初始化输出
        output = content_img.astype(np.float32) / 255.0
        gray = cv2.cvtColor(content_img, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0

        for it in range(iterations):
            # 计算当前输出的特征
            out_features = self.extract_features(
                (output * 255).astype(np.uint8)
            )

            # 内容损失梯度（简化：直接对灰度图操作）
            content_grad = np.zeros_like(gray)
            for name in content_features:
                out_feat = out_features[name]
                cont_feat = cv2.resize(
                    content_features[name],
                    (out_feat.shape[1], out_feat.shape[0])
                )
                diff = out_feat - cont_feat
                content_grad += cv2.resize(diff, (w, h))

            # 风格损失梯度（简化）
            style_grad = np.zeros_like(gray)
            for name in style_grams:
                out_feat = out_features[name]
                flat = out_feat.reshape(1, -1)
                out_gram = self.gram_matrix(flat)
                target_gram = style_grams[name]
                # 简化的梯度
                gram_diff = out_gram - target_gram[:out_gram.shape[0], :out_gram.shape[1]]
                style_energy = np.sum(gram_diff ** 2)
                style_grad += cv2.resize(
                    np.ones_like(out_feat) * style_energy * 0.001,
                    (w, h)
                )

            # 梯度下降更新
            total_grad = content_weight * content_grad + style_weight * style_grad
            output -= learning_rate * total_grad[:, :, None] * np.ones(3)[None, None, :]
            output = np.clip(output, 0, 1)

        return (output * 255).astype(np.uint8)

    def style_preset(self, image: np.ndarray, style_name: str,
                     intensity: float = 1.0) -> np.ndarray:
        """
        应用预设风格（快速，不需要风格图像）
        Args:
            image: 输入图像 (H,W,3) BGR
            style_name: 风格名称
            intensity: 强度 0~1
        Returns:
            风格化图像
        """
        if style_name not in self.STYLE_PRESETS:
            raise ValueError(f"未知风格: {style_name}, 可选: {list(self.STYLE_PRESETS.keys())}")

        result = image.copy()
        h, w = image.shape[:2]

        if style_name == 'oil_painting':
            # 油画：边缘平滑 + 颜色饱和度增强 + 笔触纹理
            result = cv2.bilateralFilter(result, 9, 75, 75)
            # 增强饱和度
            hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * (1 + 0.5 * intensity), 0, 255)
            result = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
            # 添加笔触纹理
            noise = np.random.randint(0, 15, image.shape, dtype=np.uint8)
            result = cv2.add(result, noise)

        elif style_name == 'sketch':
            # 素描：反转灰度 + 高斯模糊
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            inv = 255 - gray
            blur = cv2.GaussianBlur(inv, (21, 21), 0)
            sketch = cv2.divide(gray, 255 - blur, scale=256)
            result = cv2.cvtColor(sketch, cv2.COLOR_GRAY2BGR)
            # 与原图混合
            alpha = intensity
            result = cv2.addWeighted(image, 1 - alpha, result, alpha, 0)

        elif style_name == 'watercolor':
            # 水彩：多次双边滤波 + 边缘叠加
            for _ in range(int(3 + 7 * intensity)):
                result = cv2.bilateralFilter(result, 9, 50, 50)
            # 边缘
            gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
            edges = cv2.adaptiveThreshold(
                cv2.medianBlur(gray, 5), 255,
                cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 7, 3
            )
            edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            result = cv2.bitwise_and(result, edges)

        elif style_name == 'cartoon':
            # 卡通：边缘提取 + 颜色量化
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            gray = cv2.medianBlur(gray, 7)
            edges = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 9, 5
            )
            # 颜色量化
            data = result.reshape((-1, 3)).astype(np.float32)
            n_colors = max(2, int(12 - 8 * intensity))
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
            _, labels, centers = cv2.kmeans(data, n_colors, None, criteria, 10,
                                            cv2.KMEANS_RANDOM_CENTERS)
            quantized = centers[labels.flatten()].reshape(image.shape).astype(np.uint8)
            edges_3ch = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            result = cv2.bitwise_and(quantized, edges_3ch)

        elif style_name == 'impressionist':
            # 印象派：色彩抖动 + 轻微模糊
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[:, :, 0] += np.random.normal(0, 10 * intensity, hsv.shape[:2])
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * (1.3 + 0.3 * intensity), 0, 255)
            result = cv2.cvtColor(np.clip(hsv, 0, 255).astype(np.uint8), cv2.COLOR_HSV2BGR)
            ksize = int(3 + 4 * intensity) | 1
            result = cv2.GaussianBlur(result, (ksize, ksize), 0)
            # 模拟笔触方向
            kernel = np.ones((3, 1), np.float32) / 3
            result = cv2.filter2D(result, -1, kernel)

        return result

    def real_time_style(self, frame: np.ndarray, style_name: str = 'cartoon',
                        intensity: float = 0.7) -> np.ndarray:
        """
        实时风格化（针对视频流优化）
        Args:
            frame: 视频帧
            style_name: 风格名
            intensity: 强度
        Returns:
            风格化帧
        """
        # 缩小处理再放大（速度优化）
        h, w = frame.shape[:2]
        small = cv2.resize(frame, (w // 2, h // 2))
        styled = self.style_preset(small, style_name, intensity)
        return cv2.resize(styled, (w, h))


if __name__ == '__main__':
    # ==================== 使用示例 ====================
    print("=== 神经风格迁移模块使用示例 ===\n")

    # 创建测试图像
    test_img = np.zeros((256, 256, 3), dtype=np.uint8)
    cv2.rectangle(test_img, (50, 50), (200, 200), (255, 100, 50), -1)
    cv2.circle(test_img, (128, 128), 60, (50, 255, 100), -1)
    cv2.putText(test_img, "Test", (80, 150),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

    st = NeuralStyleTransfer()

    # 1. 预设风格应用
    print("1. 应用预设风格:")
    for style_name in NeuralStyleTransfer.STYLE_PRESETS:
        result = st.style_preset(test_img, style_name, intensity=0.8)
        print(f"   {style_name} ({NeuralStyleTransfer.STYLE_PRESETS[style_name]}): "
              f"输出尺寸{result.shape}")

    # 2. 实时风格化
    print("\n2. 实时风格化:")
    styled = st.real_time_style(test_img, 'cartoon', 0.7)
    print(f"   卡通风格输出: {styled.shape}")

    # 3. 提取特征
    print("\n3. 特征提取:")
    features = st.extract_features(test_img)
    for name, feat in features.items():
        print(f"   {name}: {feat.shape}")

    # 4. Gram矩阵
    print("\n4. 风格Gram矩阵:")
    style_grams = st.extract_style_features(test_img)
    for name, gram in style_grams.items():
        print(f"   {name}: Gram {gram.shape}")

    print("\n示例完成！")
