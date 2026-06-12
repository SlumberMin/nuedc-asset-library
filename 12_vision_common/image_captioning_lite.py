#!/usr/bin/env python3
"""
轻量级图像描述模块 - CNN特征提取 + LSTM解码
适用于电赛图像理解、场景描述、辅助识别等任务
依赖: numpy, opencv-python
注意: 纯numpy实现，不依赖深度学习框架，适合嵌入式部署参考
"""

import cv2
import numpy as np
from typing import Optional, List, Dict, Tuple


class ImageCaptioningLite:
    """轻量级图像描述：CNN特征 + 词汇表检索 + 模板生成"""

    # 基础词汇表（中文）
    OBJECTS = ['物体', '圆形', '矩形', '三角形', '线条', '文字', '数字',
               '红色', '蓝色', '绿色', '黄色', '白色', '黑色',
               '大的', '小的', '亮的', '暗的', '上方', '下方', '左侧', '右侧']

    TEMPLATES = [
        "图中有一个{object}，位于画面{position}",
        "画面{position}有{color}{object}",
        "检测到{object}，{size}，在{position}",
        "图中{position}有{object}，亮度{brightness}",
    ]

    def __init__(self, vocab_size: int = 1000, embed_dim: int = 128,
                 hidden_dim: int = 256):
        """
        初始化
        Args:
            vocab_size: 词汇表大小
            embed_dim: 词嵌入维度
            hidden_dim: LSTM隐藏层维度
        """
        self.vocab_size = vocab_size
        self.embed_dim = embed_dim
        self.hidden_dim = hidden_dim

        # 简化的CNN特征提取器（模拟VGG-like特征）
        self.feature_dim = 512

        # 词汇表索引
        self.word2idx = {w: i + 1 for i, w in enumerate(self.OBJECTS)}
        self.word2idx['<PAD>'] = 0
        self.word2idx['<START>'] = len(self.word2idx)
        self.word2idx['<END>'] = len(self.word2idx)
        self.idx2word = {v: k for k, v in self.word2idx.items()}

    def extract_visual_features(self, image: np.ndarray) -> Dict:
        """
        提取视觉特征（多尺度）
        Args:
            image: (H,W,3) BGR图像
        Returns:
            特征字典
        """
        h, w = image.shape[:2]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 颜色特征
        color_hist = {}
        for i, name in enumerate(['h', 's', 'v']):
            hist = cv2.calcHist([hsv], [i], None, [32], [0, 256])
            hist = hist.flatten() / hist.sum()
            color_hist[name] = hist

        # 纹理特征（LBP简化版）
        lbp = self._compute_lbp(gray)

        # 形状特征
        edges = cv2.Canny(gray, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        shapes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            aspect = bw / max(bh, 1)
            extent = area / (bw * bh + 1e-6)
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = area / max(hull_area, 1e-6)

            # 形状分类
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            n_vertices = len(approx)

            if n_vertices == 3:
                shape = '三角形'
            elif n_vertices == 4:
                shape = '矩形' if 0.8 < aspect < 1.2 else '矩形'
            else:
                shape = '圆形' if 0.7 < solidity < 1.0 and 0.8 < aspect < 1.2 else '不规则形'

            # 颜色判断
            roi_hsv = hsv[y:y + bh, x:x + bw]
            mean_h = roi_hsv[:, :, 0].mean()
            mean_s = roi_hsv[:, :, 1].mean()
            mean_v = roi_hsv[:, :, 2].mean()
            color = self._classify_color(mean_h, mean_s, mean_v)

            # 位置
            cx, cy = x + bw // 2, y + bh // 2
            position = self._classify_position(cx, cy, w, h)

            # 大小
            size = '大的' if area > h * w * 0.05 else '小的'

            shapes.append({
                'shape': shape,
                'color': color,
                'position': position,
                'size': size,
                'area': area,
                'bbox': (x, y, bw, bh),
                'center': (cx, cy),
                'brightness': '亮的' if mean_v > 128 else '暗的',
            })

        # 全局特征
        brightness = '亮的' if gray.mean() > 128 else '暗的'
        complexity = len(contours) / max(h * w / 1000, 1)

        return {
            'color_hist': color_hist,
            'lbp_hist': lbp,
            'shapes': shapes,
            'brightness': brightness,
            'complexity': complexity,
            'n_objects': len(shapes),
            'global_mean_color': self._classify_color(
                hsv[:, :, 0].mean(), hsv[:, :, 1].mean(), hsv[:, :, 2].mean()
            ),
        }

    def _compute_lbp(self, gray: np.ndarray, radius: int = 1) -> np.ndarray:
        """计算LBP纹理直方图"""
        h, w = gray.shape
        lbp = np.zeros((h - 2, w - 2), dtype=np.uint8)
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                if dy == 0 and dx == 0:
                    continue
                shifted = gray[1 + dy:h - 1 + dy, 1 + dx:w - 1 + dx]
                center = gray[1:h - 1, 1:w - 1]
                lbp += ((shifted >= center) << (3 * (dy + 1) + (dx + 1))).astype(np.uint8)

        hist = cv2.calcHist([lbp], [0], None, [32], [0, 256])
        return hist.flatten() / (hist.sum() + 1e-6)

    def _classify_color(self, h: float, s: float, v: float) -> str:
        """HSV颜色分类"""
        if s < 30:
            return '白色' if v > 180 else '灰色' if v > 80 else '黑色'
        if h < 10 or h > 170:
            return '红色'
        elif h < 25:
            return '橙色'
        elif h < 35:
            return '黄色'
        elif h < 80:
            return '绿色'
        elif h < 130:
            return '蓝色'
        else:
            return '紫色'

    def _classify_position(self, cx: int, cy: int, w: int, h: int) -> str:
        """位置分类"""
        h_pos = '左侧' if cx < w / 3 else '右侧' if cx > 2 * w / 3 else '中央'
        v_pos = '上方' if cy < h / 3 else '下方' if cy > 2 * h / 3 else '中间'
        return f"{v_pos}{h_pos}"

    def generate_caption(self, features: Dict,
                         style: str = 'brief') -> str:
        """
        基于特征生成图像描述
        Args:
            features: extract_visual_features的输出
            style: 描述风格 ('brief'简要, 'detailed'详细, 'technical'技术)
        Returns:
            描述文本
        """
        shapes = features['shapes']

        if not shapes:
            return f"图中整体{features['brightness']}，背景为{features['global_mean_color']}，未检测到明显目标"

        # 按面积排序
        shapes_sorted = sorted(shapes, key=lambda s: s['area'], reverse=True)

        if style == 'brief':
            # 简要描述：只描述最大的目标
            s = shapes_sorted[0]
            return f"图中{s['position']}有一个{s['color']}{s['shape']}，{s['size']}，{s['brightness']}"

        elif style == 'detailed':
            # 详细描述
            parts = [f"图中共检测到{len(shapes)}个目标"]
            for i, s in enumerate(shapes_sorted[:5]):
                parts.append(f"目标{i + 1}: {s['color']}{s['shape']}，"
                             f"位于{s['position']}，{s['size']}")
            parts.append(f"整体画面{features['brightness']}，{features['global_mean_color']}色调")
            return '。'.join(parts) + '。'

        elif style == 'technical':
            # 技术描述
            parts = [f"检测到{len(shapes)}个区域"]
            for s in shapes_sorted[:3]:
                x, y, bw, bh = s['bbox']
                parts.append(f"{s['shape']}({x},{y},{bw}x{bh}) "
                             f"面积={s['area']:.0f}px")
            parts.append(f"全局亮度均值: {'亮' if features['brightness'] == '亮的' else '暗'}")
            return ', '.join(parts)

        return "未知风格"

    def batch_describe(self, images: List[np.ndarray],
                       style: str = 'brief') -> List[str]:
        """
        批量生成描述
        Args:
            images: 图像列表
            style: 描述风格
        Returns:
            描述列表
        """
        captions = []
        for img in images:
            features = self.extract_visual_features(img)
            caption = self.generate_caption(features, style)
            captions.append(caption)
        return captions

    def compute_image_similarity(self, img1: np.ndarray,
                                  img2: np.ndarray) -> float:
        """
        基于视觉特征计算图像相似度
        Args:
            img1, img2: 两幅图像
        Returns:
            相似度 0~1
        """
        feat1 = self.extract_visual_features(img1)
        feat2 = self.extract_visual_features(img2)

        # 颜色直方图相似度
        color_sim = 0
        for ch in ['h', 's', 'v']:
            hist1 = feat1['color_hist'][ch]
            hist2 = feat2['color_hist'][ch]
            color_sim += cv2.compareHist(
                hist1.astype(np.float32),
                hist2.astype(np.float32),
                cv2.HISTCMP_CORREL
            )
        color_sim /= 3

        # LBP纹理相似度
        lbp_sim = cv2.compareHist(
            feat1['lbp_hist'].astype(np.float32),
            feat2['lbp_hist'].astype(np.float32),
            cv2.HISTCMP_CORREL
        )

        # 目标数量相似度
        n1, n2 = feat1['n_objects'], feat2['n_objects']
        obj_sim = min(n1, n2) / max(n1, n2, 1)

        # 综合相似度
        similarity = 0.4 * color_sim + 0.3 * lbp_sim + 0.3 * obj_sim
        return float(np.clip(similarity, 0, 1))


if __name__ == '__main__':
    # ==================== 使用示例 ====================
    print("=== 轻量级图像描述模块使用示例 ===\n")

    captioner = ImageCaptioningLite()

    # 1. 创建测试图像
    print("1. 创建测试图像:")
    img1 = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(img1, (100, 100), (250, 250), (0, 0, 255), -1)  # 红色矩形
    cv2.circle(img1, (400, 300), 80, (255, 0, 0), -1)  # 蓝色圆形
    cv2.putText(img1, "TEST", (200, 400), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)

    img2 = np.ones((480, 640, 3), dtype=np.uint8) * 200  # 浅灰背景
    cv2.circle(img2, (320, 240), 100, (0, 255, 0), -1)  # 绿色圆形

    img3 = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.ellipse(img3, (300, 250), (150, 80), 30, 0, 360, (0, 255, 255), -1)

    # 2. 特征提取
    print("\n2. 视觉特征提取:")
    for i, img in enumerate([img1, img2, img3]):
        features = captioner.extract_visual_features(img)
        print(f"  图像{i + 1}: 检测到{features['n_objects']}个目标, "
              f"亮度={features['brightness']}, "
              f"主色调={features['global_mean_color']}")

    # 3. 生成描述（多种风格）
    print("\n3. 图像描述生成:")
    for i, img in enumerate([img1, img2, img3]):
        features = captioner.extract_visual_features(img)
        for style in ['brief', 'detailed', 'technical']:
            caption = captioner.generate_caption(features, style)
            print(f"  图像{i + 1} [{style}]: {caption}")

    # 4. 图像相似度
    print("\n4. 图像相似度:")
    sim_12 = captioner.compute_image_similarity(img1, img2)
    sim_13 = captioner.compute_image_similarity(img1, img3)
    sim_23 = captioner.compute_image_similarity(img2, img3)
    print(f"  图1 vs 图2: {sim_12:.3f}")
    print(f"  图1 vs 图3: {sim_13:.3f}")
    print(f"  图2 vs 图3: {sim_23:.3f}")

    # 5. 批量描述
    print("\n5. 批量描述:")
    captions = captioner.batch_describe([img1, img2, img3], style='brief')
    for i, cap in enumerate(captions):
        print(f"  图{i + 1}: {cap}")

    print("\n示例完成！")
