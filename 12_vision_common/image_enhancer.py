"""
图像增强工具集 - CLAHE / 去噪 / 锐化 / 白平衡
适用于电赛中低光照、偏色、模糊等场景的图像预处理
"""
import cv2
import numpy as np
from typing import Optional, Tuple


class ImageEnhancer:
    """图像增强流水线"""

    def __init__(self):
        self._pipeline = []

    def add_step(self, name, fn, **kwargs):
        """添加增强步骤"""
        self._pipeline.append((name, fn, kwargs))
        return self

    def clear(self):
        self._pipeline.clear()

    def apply(self, image):
        """按顺序执行所有增强步骤"""
        result = image.copy()
        for name, fn, kwargs in self._pipeline:
            result = fn(result, **kwargs)
        return result

    # ========== 静态增强方法 ==========

    @staticmethod
    def clahe(image, clip_limit=2.0, grid_size=(8, 8),
              apply_to="luminance"):
        """
        CLAHE 自适应直方图均衡化
        apply_to: "luminance"(LAB空间) / "gray"(灰度) / "channel"(BGR各通道)
        """
        if apply_to == "gray" or len(image.shape) == 2:
            gray = image if len(image.shape) == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=grid_size)
            return clahe.apply(gray)

        elif apply_to == "luminance":
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=grid_size)
            l = clahe.apply(l)
            return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)

        elif apply_to == "channel":
            channels = cv2.split(image)
            clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=grid_size)
            enhanced = [clahe.apply(ch) for ch in channels]
            return cv2.merge(enhanced)

        return image

    @staticmethod
    def denoise(image, method="bilateral", strength=10):
        """
        去噪
        method: "bilateral" / "gaussian" / "nlmeans" / "median"
        """
        if method == "bilateral":
            return cv2.bilateralFilter(image, d=9,
                                       sigmaColor=strength * 7.5,
                                       sigmaSpace=strength * 7.5)

        elif method == "gaussian":
            ksize = max(3, int(strength / 2) * 2 + 1)
            return cv2.GaussianBlur(image, (ksize, ksize), 0)

        elif method == "nlmeans":
            if len(image.shape) == 3:
                return cv2.fastNlMeansDenoisingColored(
                    image, None, strength, strength, 7, 21)
            else:
                return cv2.fastNlMeansDenoising(image, None, strength, 7, 21)

        elif method == "median":
            ksize = max(3, int(strength / 3) * 2 + 1)
            ksize = min(ksize, 31)
            return cv2.medianBlur(image, ksize)

        return image

    @staticmethod
    def sharpen(image, method="unsharp", strength=1.5):
        """
        锐化
        method: "unsharp" / "kernel" / "laplacian"
        strength: 锐化强度 (0.5~3.0 推荐)
        """
        if method == "unsharp":
            blurred = cv2.GaussianBlur(image, (0, 0), 3)
            return cv2.addWeighted(image, strength, blurred,
                                   -(strength - 1), 0)

        elif method == "kernel":
            k = np.array([
                [0, -1, 0],
                [-1, 5, -1],
                [0, -1, 0]
            ], dtype=np.float32)
            k = k * strength / 5.0
            k[1, 1] = 1 + (strength - 1) * 0.8
            return cv2.filter2D(image, -1, k)

        elif method == "laplacian":
            lap = cv2.Laplacian(image, cv2.CV_64F)
            lap = np.clip(lap, 0, 255).astype(np.uint8)
            return cv2.addWeighted(image, 1.0, lap, strength * 0.3, 0)

        return image

    @staticmethod
    def white_balance(image, method="gray_world"):
        """
        白平衡
        method: "gray_world" / "white_patch" / "adaptive"
        """
        if method == "gray_world":
            b, g, r = cv2.split(image.astype(np.float64))
            avg_b, avg_g, avg_r = b.mean(), g.mean(), r.mean()
            avg_all = (avg_b + avg_g + avg_r) / 3
            if avg_b > 0:
                b = b * (avg_all / avg_b)
            if avg_g > 0:
                g = g * (avg_all / avg_g)
            if avg_r > 0:
                r = r * (avg_all / avg_r)
            result = cv2.merge([
                np.clip(b, 0, 255).astype(np.uint8),
                np.clip(g, 0, 255).astype(np.uint8),
                np.clip(r, 0, 255).astype(np.uint8)
            ])
            return result

        elif method == "white_patch":
            b, g, r = cv2.split(image.astype(np.float64))
            max_b = np.percentile(b, 99) or 1
            max_g = np.percentile(g, 99) or 1
            max_r = np.percentile(r, 99) or 1
            b = b * (255.0 / max_b)
            g = g * (255.0 / max_g)
            r = r * (255.0 / max_r)
            result = cv2.merge([
                np.clip(b, 0, 255).astype(np.uint8),
                np.clip(g, 0, 255).astype(np.uint8),
                np.clip(r, 0, 255).astype(np.uint8)
            ])
            return result

        elif method == "adaptive":
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            l, a, b_ch = cv2.split(lab)
            # 自适应调整a和b通道
            a = cv2.normalize(a, None, 100, 155, cv2.NORM_MINMAX)
            b_ch = cv2.normalize(b_ch, None, 100, 155, cv2.NORM_MINMAX)
            return cv2.cvtColor(cv2.merge([l, a, b_ch]), cv2.COLOR_LAB2BGR)

        return image

    @staticmethod
    def gamma_correction(image, gamma=1.0):
        """Gamma校正, gamma<1提亮, gamma>1变暗"""
        inv_gamma = 1.0 / max(gamma, 0.01)
        table = np.array([
            ((i / 255.0) ** inv_gamma) * 255
            for i in range(256)
        ]).astype(np.uint8)
        return cv2.LUT(image, table)

    @staticmethod
    def adjust_brightness_contrast(image, brightness=0, contrast=0):
        """
        亮度/对比度调整
        brightness: -100~100
        contrast: -100~100
        """
        b = brightness / 100.0
        c = contrast / 100.0
        if c != 0:
            f = 131 * (c + 127) / (127 * (131 - c))
        else:
            f = 1.0
        img = image.astype(np.float32)
        img = f * (img - 128) + 128 + b * 128
        return np.clip(img, 0, 255).astype(np.uint8)

    @staticmethod
    def enhance_for_detection(image, brightness=0, clahe_clip=2.0,
                              denoise_strength=5, sharpen_strength=1.2,
                              white_balance_method="gray_world"):
        """一键增强流水线（面向目标检测优化）"""
        result = image.copy()
        if brightness != 0:
            result = ImageEnhancer.adjust_brightness_contrast(
                result, brightness=brightness)
        result = ImageEnhancer.white_balance(result, method=white_balance_method)
        result = ImageEnhancer.clahe(result, clip_limit=clahe_clip)
        if denoise_strength > 0:
            result = ImageEnhancer.denoise(result, method="bilateral",
                                           strength=denoise_strength)
        if sharpen_strength > 1.0:
            result = ImageEnhancer.sharpen(result, method="unsharp",
                                           strength=sharpen_strength)
        return result

    @staticmethod
    def enhance_for_ocr(image):
        """一键增强流水线（面向OCR/文字识别优化）"""
        result = image.copy()
        result = ImageEnhancer.white_balance(result, method="white_patch")
        result = ImageEnhancer.clahe(result, clip_limit=3.0, apply_to="gray")
        if len(result.shape) == 2:
            result = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)
        result = ImageEnhancer.sharpen(result, method="kernel", strength=2.0)
        _, result = cv2.threshold(
            cv2.cvtColor(result, cv2.COLOR_BGR2GRAY),
            0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return result


def compare_enhancements(image):
    """对比不同增强方法的效果"""
    results = OrderedDict()
    results["Original"] = image
    results["CLAHE(L)"] = ImageEnhancer.clahe(image, clip_limit=2.0)
    results["CLAHE(4.0)"] = ImageEnhancer.clahe(image, clip_limit=4.0)
    results["Bilateral"] = ImageEnhancer.denoise(image, method="bilateral")
    results["NLMeans"] = ImageEnhancer.denoise(image, method="nlmeans")
    results["Sharpen"] = ImageEnhancer.sharpen(image, strength=1.5)
    results["WhiteBal"] = ImageEnhancer.white_balance(image)
    results["Gamma(0.5)"] = ImageEnhancer.gamma_correction(image, gamma=0.5)
    results["Gamma(1.5)"] = ImageEnhancer.gamma_correction(image, gamma=1.5)

    cells = []
    for name, img in results.items():
        cell = cv2.resize(img, (320, 240))
        cv2.putText(cell, name, (5, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cells.append(cell)

    row1 = np.hstack(cells[:3])
    row2 = np.hstack(cells[3:6])
    row3 = np.hstack(cells[6:9])
    return np.vstack([row1, row2, row3])


if __name__ == "__main__":
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else 0
    img = cv2.imread(str(path)) if isinstance(path, str) else None
    if img is None:
        cap = cv2.VideoCapture(int(path) if isinstance(path, int) else 0)
        ret, img = cap.read()
        cap.release()
    if img is not None:
        comparison = compare_enhancements(img)
        cv2.imshow("Enhancement Comparison", comparison)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("无法读取图像")
