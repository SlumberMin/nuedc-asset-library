"""
颜色恒常性模块 - 不同光照条件下的颜色一致性处理
====================================================
功能:
  - Gray World 假设白平衡
  - White Patch (Retinex) 白平衡
  - Gray Edge 算法
  - 简单色温补偿
  - 自动白平衡 (combined)

适用场景:
  - 电赛中不同光照环境下颜色识别的一致性
  - 室内外光照切换时保持颜色阈值稳定
  - 摄像头自动白平衡效果不佳时的软件补偿

用法:
  cc = ColorConstancy()
  balanced = cc.auto_white_balance(img)
  balanced = cc.gray_world(img)
"""

import cv2
import numpy as np


class ColorConstancy:
    """颜色恒常性处理器"""

    # ──────────────── Gray World ────────────────
    @staticmethod
    def gray_world(img: np.ndarray) -> np.ndarray:
        """
        Gray World 假设: 整个场景的平均颜色应为灰色。
        简单高效，适合大多数自然场景。
        """
        img_f = img.astype(np.float64)
        avg_b, avg_g, avg_r = [img_f[:, :, c].mean() for c in range(3)]
        avg_gray = (avg_b + avg_g + avg_r) / 3.0
        # 避免除零
        scale_b = avg_gray / max(avg_b, 1e-6)
        scale_g = avg_gray / max(avg_g, 1e-6)
        scale_r = avg_gray / max(avg_r, 1e-6)
        result = img_f.copy()
        result[:, :, 0] *= scale_b
        result[:, :, 1] *= scale_g
        result[:, :, 2] *= scale_r
        return np.clip(result, 0, 255).astype(np.uint8)

    # ──────────────── White Patch (Retinex) ────────────────
    @staticmethod
    def white_patch(img: np.ndarray, percentile: float = 99.0) -> np.ndarray:
        """
        White Patch 假设: 场景中最亮的点应为白色。
        percentile: 使用第N百分位代替最大值, 减少噪声影响。
        """
        img_f = img.astype(np.float64)
        for c in range(3):
            max_val = np.percentile(img_f[:, :, c], percentile)
            if max_val > 0:
                img_f[:, :, c] = img_f[:, :, c] * (255.0 / max_val)
        return np.clip(img_f, 0, 255).astype(np.uint8)

    # ──────────────── Gray Edge ────────────────
    @staticmethod
    def gray_edge(img: np.ndarray, order: int = 1, kernel_size: int = 3) -> np.ndarray:
        """
        Gray Edge 算法: 假设场景中反射率变化的平均值在各通道相等。
        order=1: 一阶导数 (类似Sobel)
        order=2: 二阶导数 (类似Laplacian)
        """
        img_f = img.astype(np.float64)
        # 计算各通道的导数幅值均值
        edge_means = []
        for c in range(3):
            channel = img_f[:, :, c]
            if order == 1:
                gx = cv2.Sobel(channel, cv2.CV_64F, 1, 0, ksize=kernel_size)
                gy = cv2.Sobel(channel, cv2.CV_64F, 0, 1, ksize=kernel_size)
                edge_mag = np.sqrt(gx ** 2 + gy ** 2)
            else:
                edge_mag = np.abs(cv2.Laplacian(channel, cv2.CV_64F, ksize=kernel_size))
            edge_means.append(edge_mag.mean())

        avg_edge = np.mean(edge_means)
        result = img_f.copy()
        for c in range(3):
            if edge_means[c] > 0:
                result[:, :, c] *= (avg_edge / edge_means[c])
        return np.clip(result, 0, 255).astype(np.uint8)

    # ──────────────── 色温补偿 ────────────────
    @staticmethod
    def color_temperature_compensate(img: np.ndarray, temperature: int = 0) -> np.ndarray:
        """
        色温补偿: 手动调整色温偏移。
        temperature: -100 (偏蓝/冷色) ~ +100 (偏黄/暖色)
        适用于已知光源色温的场景。
        """
        img_f = img.astype(np.float64)
        offset = temperature / 100.0 * 30  # 最大偏移30
        # 暖色: R增 B减; 冷色: R减 B增
        img_f[:, :, 2] = np.clip(img_f[:, :, 2] + offset, 0, 255)  # R
        img_f[:, :, 0] = np.clip(img_f[:, :, 0] - offset, 0, 255)  # B
        return img_f.astype(np.uint8)

    # ──────────────── 自动白平衡 ────────────────
    @staticmethod
    def auto_white_balance(img: np.ndarray, method: str = 'gray_world') -> np.ndarray:
        """
        统一入口, 选择白平衡方法。
        method: 'gray_world' | 'white_patch' | 'gray_edge' | 'combined'
        """
        cc = ColorConstancy()
        if method == 'gray_world':
            return cc.gray_world(img)
        elif method == 'white_patch':
            return cc.white_patch(img)
        elif method == 'gray_edge':
            return cc.gray_edge(img)
        elif method == 'combined':
            # 融合多种方法取平均
            r1 = cc.gray_world(img).astype(np.float64)
            r2 = cc.white_patch(img).astype(np.float64)
            r3 = cc.gray_edge(img).astype(np.float64)
            combined = (r1 + r2 + r3) / 3.0
            return np.clip(combined, 0, 255).astype(np.uint8)
        else:
            raise ValueError(f"未知方法: {method}")

    # ──────────────── 光照归一化 ────────────────
    @staticmethod
    def normalize_illumination(img: np.ndarray, clip_limit: float = 2.0) -> np.ndarray:
        """
        光照归一化: CLAHE + 灰度世界联合处理。
        适合光照不均匀的场景。
        """
        # 转LAB, 仅对L通道做CLAHE
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
        l = clahe.apply(l)
        lab = cv2.merge([l, a, b])
        result = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        # 再做一次灰度世界
        result = ColorConstancy.gray_world(result)
        return result


# ──────────────── Demo ────────────────
if __name__ == '__main__':
    import sys

    img_path = sys.argv[1] if len(sys.argv) > 1 else 'test.jpg'
    img = cv2.imread(img_path)
    if img is None:
        print(f"无法读取图像: {img_path}")
        sys.exit(1)

    cc = ColorConstancy()

    results = {
        'Original': img,
        'GrayWorld': cc.gray_world(img),
        'WhitePatch': cc.white_patch(img),
        'GrayEdge': cc.gray_edge(img),
        'Warm+30': cc.color_temperature_compensate(img, 30),
        'Cool-30': cc.color_temperature_compensate(img, -30),
        'IllumNorm': cc.normalize_illumination(img),
    }

    # 拼接显示
    h, w = img.shape[:2]
    thumb_w, thumb_h = 320, int(320 * h / w)
    panels = []
    for name, result in results.items():
        thumb = cv2.resize(result, (thumb_w, thumb_h))
        cv2.putText(thumb, name, (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        panels.append(thumb)

    # 4列排列
    rows = []
    for i in range(0, len(panels), 2):
        row = np.hstack(panels[i:i + 2])
        rows.append(row)
    grid = np.vstack(rows)
    cv2.imshow('Color Constancy Demo', grid)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
