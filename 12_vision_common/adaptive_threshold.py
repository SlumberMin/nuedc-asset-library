"""
自适应阈值模块 - 多种自适应二值化方法
=========================================
功能:
  - OpenCV 自适应阈值 (均值/高斯)
  - 大津法 (Otsu)
  - 局部大津法 (Sauvola)
  - Niblack 二值化
  - 自适应组合方法 (自动选择最佳)

适用场景:
  - 光照不均匀条件下的二值化
  - 文档/二维码识别前的预处理
  - 低对比度图像的分割

用法:
  at = AdaptiveThreshold()
  binary = at.auto_best(gray)
  binary = at.sauvola(gray, window_size=25, k=0.2)
"""

import cv2
import numpy as np


class AdaptiveThreshold:
    """自适应阈值处理器"""

    # ──────────────── OpenCV 自适应阈值 ────────────────
    @staticmethod
    def cv_adaptive_mean(gray: np.ndarray, block_size: int = 11, C: float = 2) -> np.ndarray:
        """OpenCV 自适应均值阈值"""
        return cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY, block_size, C
        )

    @staticmethod
    def cv_adaptive_gaussian(gray: np.ndarray, block_size: int = 11, C: float = 2) -> np.ndarray:
        """OpenCV 自适应高斯阈值"""
        return cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, block_size, C
        )

    # ──────────────── Otsu ────────────────
    @staticmethod
    def otsu(gray: np.ndarray, blur_size: int = 5) -> np.ndarray:
        """
        大津法全局阈值, 自动选择最佳阈值。
        适合双峰分布的直方图。
        """
        if blur_size > 0:
            gray = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return binary

    @staticmethod
    def otsu_inv(gray: np.ndarray, blur_size: int = 5) -> np.ndarray:
        """大津法反相"""
        if blur_size > 0:
            gray = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        return binary

    # ──────────────── Niblack ────────────────
    @staticmethod
    def niblack(gray: np.ndarray, window_size: int = 15, k: float = -0.2) -> np.ndarray:
        """
        Niblack 二值化:
        T(x,y) = mean + k * std
        k 通常取 -0.2 ~ -0.1 (负值使阈值低于均值)
        """
        img = gray.astype(np.float64)
        mean = cv2.blur(img, (window_size, window_size))
        sq_mean = cv2.blur(img ** 2, (window_size, window_size))
        std = np.sqrt(np.maximum(sq_mean - mean ** 2, 0))
        threshold = mean + k * std
        binary = np.where(img >= threshold, 255, 0).astype(np.uint8)
        return binary

    # ──────────────── Sauvola ────────────────
    @staticmethod
    def sauvola(gray: np.ndarray, window_size: int = 25, k: float = 0.2, R: float = 128) -> np.ndarray:
        """
        Sauvola 二值化 (Niblack 改进版):
        T(x,y) = mean * (1 + k * (std / R - 1))
        R: 标准差的动态范围, 通常取128
        k: 通常取 0.1 ~ 0.5
        对光照不均匀场景效果好。
        """
        img = gray.astype(np.float64)
        mean = cv2.blur(img, (window_size, window_size))
        sq_mean = cv2.blur(img ** 2, (window_size, window_size))
        std = np.sqrt(np.maximum(sq_mean - mean ** 2, 0))
        threshold = mean * (1.0 + k * (std / R - 1.0))
        binary = np.where(img >= threshold, 255, 0).astype(np.uint8)
        return binary

    # ──────────────── 局部Otsu ────────────────
    @staticmethod
    def local_otsu(gray: np.ndarray, block_size: int = 101) -> np.ndarray:
        """
        局部大津法: 将图像分块, 每块独立做Otsu。
        block_size: 分块大小 (奇数)
        """
        h, w = gray.shape[:2]
        binary = np.zeros_like(gray)
        half = block_size // 2
        # 用padding处理边界
        padded = cv2.copyMakeBorder(gray, half, half, half, half, cv2.BORDER_REFLECT)
        for y in range(0, h, block_size):
            for x in range(0, w, block_size):
                patch = padded[y:y + block_size, x:x + block_size]
                _, local_bin = cv2.threshold(patch, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                # 裁剪回实际区域
                ey = min(y + block_size, h)
                ex = min(x + block_size, w)
                ph = ey - y
                pw = ex - x
                binary[y:ey, x:ex] = local_bin[half:half + ph, half:half + pw]
        return binary

    # ──────────────── 渐变阈值 ────────────────
    @staticmethod
    def gradient_threshold(gray: np.ndarray, direction: str = 'horizontal') -> np.ndarray:
        """
        渐变阈值: 适用于光照沿某方向渐变的场景。
        direction: 'horizontal' | 'vertical'
        """
        h, w = gray.shape[:2]
        # 计算局部均值作为基准
        local_mean = cv2.blur(gray.astype(np.float64), (51, 51))
        diff = gray.astype(np.float64) - local_mean
        binary = np.where(diff >= 0, 255, 0).astype(np.uint8)
        return binary

    # ──────────────── 自动选择最佳 ────────────────
    @staticmethod
    def auto_best(gray: np.ndarray) -> np.ndarray:
        """
        自动选择最佳二值化方法。
        基于图像的直方图分布和局部方差来决策。
        """
        # 计算直方图双峰性
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256]).flatten()
        hist_norm = hist / hist.sum()

        # 计算局部方差均值 (反映光照均匀性)
        mean_img = cv2.blur(gray.astype(np.float64), (31, 31))
        sq_mean = cv2.blur(gray.astype(np.float64) ** 2, (31, 31))
        local_var = np.sqrt(np.maximum(sq_mean - mean_img ** 2, 0))
        avg_local_var = local_var.mean()

        # 全局方差
        global_var = gray.astype(np.float64).std()

        # 判断: 光照均匀 + 双峰分布 -> Otsu
        var_ratio = avg_local_var / max(global_var, 1)
        if var_ratio < 0.8:
            # 光照较均匀 -> Otsu
            return AdaptiveThreshold.otsu(gray)
        else:
            # 光照不均匀 -> Sauvola
            return AdaptiveThreshold.sauvola(gray)

    # ──────────────── 所有方法对比 ────────────────
    @staticmethod
    def compare_all(gray: np.ndarray) -> dict:
        """返回所有方法的结果字典, 方便对比。"""
        at = AdaptiveThreshold()
        return {
            'AdaptiveMean': at.cv_adaptive_mean(gray),
            'AdaptiveGauss': at.cv_adaptive_gaussian(gray),
            'Otsu': at.otsu(gray),
            'Niblack': at.niblack(gray),
            'Sauvola': at.sauvola(gray),
            'Gradient': at.gradient_threshold(gray),
            'AutoBest': at.auto_best(gray),
        }


# ──────────────── Demo ────────────────
if __name__ == '__main__':
    import sys

    img_path = sys.argv[1] if len(sys.argv) > 1 else 'test.jpg'
    img = cv2.imread(img_path)
    if img is None:
        print(f"无法读取图像: {img_path}")
        sys.exit(1)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    at = AdaptiveThreshold()
    results = at.compare_all(gray)

    h, w = gray.shape[:2]
    thumb_w, thumb_h = 320, int(320 * h / w)

    panels = []
    for name, binary in results.items():
        thumb = cv2.resize(binary, (thumb_w, thumb_h))
        cv2.putText(thumb, name, (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        panels.append(thumb)

    # 拼接: 每行3个
    rows = []
    for i in range(0, len(panels), 3):
        row_panels = panels[i:i + 3]
        while len(row_panels) < 3:
            row_panels.append(np.zeros((thumb_h, thumb_w), dtype=np.uint8))
        rows.append(np.hstack(row_panels))
    grid = np.vstack(rows)
    cv2.imshow('Adaptive Threshold Compare', grid)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
