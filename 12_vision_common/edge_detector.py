"""
边缘检测器模块 - Canny/Sobel/Laplacian/Prewitt 等
====================================================
功能:
  - Canny 边缘检测 (自动/手动阈值)
  - Sobel 梯度 (X/Y/幅值/方向)
  - Laplacian 边缘
  - Prewitt 边缘
  - Scharr 边缘 (高精度一阶导)
  - Roberts 边缘
  - 自动边缘检测 (根据图像选择最佳参数)

适用场景:
  - 形状检测前的边缘提取
  - 轮廓检测的预处理
  - 缺陷检测
  - 车道线/路径检测

用法:
  ed = EdgeDetector()
  edges = ed.canny_auto(gray)
  edges = ed.sobel_mag(gray)
"""

import cv2
import numpy as np


class EdgeDetector:
    """边缘检测器"""

    # ──────────────── Canny ────────────────
    @staticmethod
    def canny(gray: np.ndarray, low: int = 50, high: int = 150,
              aperture_size: int = 3, blur_ksize: int = 3) -> np.ndarray:
        """
        Canny 边缘检测 (手动阈值)。
        low/high: 低/高阈值
        aperture_size: Sobel核大小 (3/5/7)
        blur_ksize: 预模糊核大小, 0=不模糊
        """
        if blur_ksize > 0:
            gray = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0)
        return cv2.Canny(gray, low, high, apertureSize=aperture_size)

    @staticmethod
    def canny_auto(gray: np.ndarray, sigma: float = 0.33, blur_ksize: int = 3) -> np.ndarray:
        """
        Canny 自动阈值: 基于中位数自动计算高低阈值。
        sigma: 控制阈值范围的系数 (越大边缘越少)
        """
        if blur_ksize > 0:
            gray = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0)
        median = np.median(gray)
        low = int(max(0, (1.0 - sigma) * median))
        high = int(min(255, (1.0 + sigma) * median))
        return cv2.Canny(gray, low, high)

    # ──────────────── Sobel ────────────────
    @staticmethod
    def sobel_x(gray: np.ndarray, ksize: int = 3) -> np.ndarray:
        """Sobel X方向梯度 (检测垂直边缘)"""
        return cv2.convertScaleAbs(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=ksize))

    @staticmethod
    def sobel_y(gray: np.ndarray, ksize: int = 3) -> np.ndarray:
        """Sobel Y方向梯度 (检测水平边缘)"""
        return cv2.convertScaleAbs(cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=ksize))

    @staticmethod
    def sobel_mag(gray: np.ndarray, ksize: int = 3) -> np.ndarray:
        """Sobel 梯度幅值"""
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=ksize)
        gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=ksize)
        mag = np.sqrt(gx ** 2 + gy ** 2)
        return cv2.convertScaleAbs(mag)

    @staticmethod
    def sobel_dir(gray: np.ndarray, ksize: int = 3) -> np.ndarray:
        """Sobel 梯度方向 (0-180度, 映射到0-255)"""
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=ksize)
        gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=ksize)
        direction = np.arctan2(gy, gx) * 180 / np.pi % 180
        return (direction / 180 * 255).astype(np.uint8)

    # ──────────────── Laplacian ────────────────
    @staticmethod
    def laplacian(gray: np.ndarray, ksize: int = 3) -> np.ndarray:
        """Laplacian 边缘 (二阶导数, 各向同性)"""
        return cv2.convertScaleAbs(cv2.Laplacian(gray, cv2.CV_64F, ksize=ksize))

    @staticmethod
    def laplacian_of_gaussian(gray: np.ndarray, ksize: int = 5, sigma: float = 1.4) -> np.ndarray:
        """LoG: 先高斯平滑再Laplacian, 减少噪声"""
        blurred = cv2.GaussianBlur(gray, (ksize, ksize), sigma)
        return cv2.convertScaleAbs(cv2.Laplacian(blurred, cv2.CV_64F, ksize=ksize))

    # ──────────────── Prewitt ────────────────
    @staticmethod
    def prewitt(gray: np.ndarray) -> np.ndarray:
        """Prewitt 边缘检测 (自定义核)"""
        kernel_x = np.array([[-1, 0, 1], [-1, 0, 1], [-1, 0, 1]], dtype=np.float64)
        kernel_y = np.array([[-1, -1, -1], [0, 0, 0], [1, 1, 1]], dtype=np.float64)
        gx = cv2.filter2D(gray.astype(np.float64), -1, kernel_x)
        gy = cv2.filter2D(gray.astype(np.float64), -1, kernel_y)
        mag = np.sqrt(gx ** 2 + gy ** 2)
        return cv2.convertScaleAbs(mag)

    # ──────────────── Scharr ────────────────
    @staticmethod
    def scharr(gray: np.ndarray) -> np.ndarray:
        """Scharr 边缘 (比Sobel更精确的3x3一阶导数)"""
        gx = cv2.Scharr(gray, cv2.CV_64F, 1, 0)
        gy = cv2.Scharr(gray, cv2.CV_64F, 0, 1)
        mag = np.sqrt(gx ** 2 + gy ** 2)
        return cv2.convertScaleAbs(mag)

    # ──────────────── Roberts ────────────────
    @staticmethod
    def roberts(gray: np.ndarray) -> np.ndarray:
        """Roberts 交叉边缘检测 (2x2核)"""
        kernel_x = np.array([[1, 0], [0, -1]], dtype=np.float64)
        kernel_y = np.array([[0, 1], [-1, 0]], dtype=np.float64)
        gx = cv2.filter2D(gray.astype(np.float64), -1, kernel_x)
        gy = cv2.filter2D(gray.astype(np.float64), -1, kernel_y)
        mag = np.sqrt(gx ** 2 + gy ** 2)
        return cv2.convertScaleAbs(mag)

    # ──────────────── 自动选择 ────────────────
    @staticmethod
    def auto_best(gray: np.ndarray) -> np.ndarray:
        """
        自动选择最佳边缘检测方法和参数。
        基于图像噪声水平和对比度。
        """
        # 估计噪声水平 (用中值绝对偏差)
        median_val = np.median(gray)
        mad = np.median(np.abs(gray.astype(np.float64) - median_val))
        noise_level = mad / median_val if median_val > 0 else 0.1

        # 估计对比度
        contrast = gray.astype(np.float64).std()

        if noise_level > 0.05 or contrast < 30:
            # 高噪声或低对比度 -> 先增强再Canny
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            return EdgeDetector.canny_auto(enhanced, sigma=0.4, blur_ksize=5)
        else:
            # 正常图像 -> Canny自动阈值
            return EdgeDetector.canny_auto(gray)

    # ──────────────── 非极大值抑制 ────────────────
    @staticmethod
    def nms_edges(gray: np.ndarray, ksize: int = 3) -> np.ndarray:
        """
        手动非极大值抑制边缘细化。
        先计算梯度幅值和方向, 再沿梯度方向非极大值抑制。
        """
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=ksize)
        gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=ksize)
        mag = np.sqrt(gx ** 2 + gy ** 2)
        angle = np.arctan2(gy, gx) * 180 / np.pi
        angle[angle < 0] += 180

        h, w = gray.shape
        nms = np.zeros_like(mag)
        for i in range(1, h - 1):
            for j in range(1, w - 1):
                a = angle[i, j]
                # 量化到4个方向
                if (0 <= a < 22.5) or (157.5 <= a <= 180):
                    n1, n2 = mag[i, j - 1], mag[i, j + 1]
                elif 22.5 <= a < 67.5:
                    n1, n2 = mag[i - 1, j + 1], mag[i + 1, j - 1]
                elif 67.5 <= a < 112.5:
                    n1, n2 = mag[i - 1, j], mag[i + 1, j]
                else:
                    n1, n2 = mag[i - 1, j - 1], mag[i + 1, j + 1]
                if mag[i, j] >= n1 and mag[i, j] >= n2:
                    nms[i, j] = mag[i, j]
        return cv2.convertScaleAbs(nms)

    # ──────────────── 所有方法对比 ────────────────
    @staticmethod
    def compare_all(gray: np.ndarray) -> dict:
        """返回所有方法的结果字典。"""
        ed = EdgeDetector()
        return {
            'Canny_Manual': ed.canny(gray),
            'Canny_Auto': ed.canny_auto(gray),
            'Sobel_Mag': ed.sobel_mag(gray),
            'Laplacian': ed.laplacian(gray),
            'LoG': ed.laplacian_of_gaussian(gray),
            'Prewitt': ed.prewitt(gray),
            'Scharr': ed.scharr(gray),
            'Roberts': ed.roberts(gray),
            'AutoBest': ed.auto_best(gray),
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
    ed = EdgeDetector()
    results = ed.compare_all(gray)

    h, w = gray.shape[:2]
    thumb_w, thumb_h = 320, int(320 * h / w)

    panels = []
    for name, edge in results.items():
        thumb = cv2.resize(edge, (thumb_w, thumb_h))
        cv2.putText(thumb, name, (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        panels.append(thumb)

    # 3列排列
    rows = []
    for i in range(0, len(panels), 3):
        row_panels = panels[i:i + 3]
        while len(row_panels) < 3:
            row_panels.append(np.zeros((thumb_h, thumb_w), dtype=np.uint8))
        rows.append(np.hstack(row_panels))
    grid = np.vstack(rows)
    cv2.imshow('Edge Detector Compare', grid)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
