"""
直方图分析模块 - 计算/均衡化/匹配/比较
依赖: opencv-python, numpy
"""

import cv2
import numpy as np


class HistogramAnalyzer:
    """直方图分析工具"""

    # ---- 直方图计算 ----

    @staticmethod
    def calc_histogram(image, channel=0, bins=256, range_min=0, range_max=256, normalize=False):
        """
        计算单通道直方图
        :param image: 图像 (灰度或 BGR)
        :param channel: 通道索引 (0=B/Gray, 1=G, 2=R), -1=灰度
        :param bins: bin 数量
        :param normalize: 是否归一化
        :return: (hist, bin_edges)
        """
        if channel == -1 or len(image.shape) == 2:
            src = image if len(image.shape) == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            src = image[:, :, channel]

        hist = cv2.calcHist([src], [0], None, [bins], [range_min, range_max]).flatten()

        if normalize:
            hist = hist / hist.sum()

        bin_edges = np.linspace(range_min, range_max, bins + 1)
        return hist, bin_edges

    @staticmethod
    def calc_histogram_all_channels(image, bins=256):
        """计算 BGR 三通道直方图"""
        result = {}
        names = ['blue', 'green', 'red']
        for i, name in enumerate(names):
            hist, edges = HistogramAnalyzer.calc_histogram(image, i, bins)
            result[name] = {'hist': hist, 'edges': edges}
        return result

    @staticmethod
    def calc_2d_histogram(image1, image2, bins=256):
        """
        计算 2D 联合直方图 (用于分析两幅图关系)
        """
        hist = cv2.calcHist([image1, image2], [0, 1], None, [bins, bins], [0, 256, 0, 256])
        return hist

    # ---- 直方图均衡化 ----

    @staticmethod
    def equalize_histogram(image):
        """
        直方图均衡化 (灰度图)
        :param image: 灰度图
        :return: 均衡化后图像
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        return cv2.equalizeHist(gray)

    @staticmethod
    def equalize_histogram_color(image):
        """
        彩色直方图均衡化 (在 YCrCb 空间)
        """
        ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
        ycrcb[:, :, 0] = cv2.equalizeHist(ycrcb[:, :, 0])
        return cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2BGR)

    @staticmethod
    def clahe(image, clip_limit=2.0, grid_size=(8, 8)):
        """
        CLAHE 自适应直方图均衡化 (推荐)
        :param clip_limit: 对比度限制
        :param grid_size: 网格大小
        :return: 均衡化后图像
        """
        clahe_obj = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=grid_size)
        if len(image.shape) == 3:
            ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
            ycrcb[:, :, 0] = clahe_obj.apply(ycrcb[:, :, 0])
            return cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2BGR)
        else:
            return clahe_obj.apply(image)

    @staticmethod
    def clahe_color_per_channel(image, clip_limit=2.0, grid_size=(8, 8)):
        """
        CLAHE 分通道均衡化 (在 LAB 空间)
        """
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        clahe_obj = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=grid_size)
        lab[:, :, 0] = clahe_obj.apply(lab[:, :, 0])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # ---- 直方图匹配 (规定化) ----

    @staticmethod
    def match_histogram(source, reference):
        """
        直方图匹配 - 使 source 的直方图分布匹配 reference
        适用于: 风格迁移、亮度对齐、颜色校正
        :param source: 源图 (BGR 或灰度)
        :param reference: 参考图 (BGR 或灰度)
        :return: 匹配后的图像
        """
        is_color = len(source.shape) == 3

        if is_color:
            src_lab = cv2.cvtColor(source, cv2.COLOR_BGR2LAB)
            ref_lab = cv2.cvtColor(reference, cv2.COLOR_BGR2LAB)
            result_lab = src_lab.copy()
            for ch in range(3):
                result_lab[:, :, ch] = HistogramAnalyzer._match_channel(
                    src_lab[:, :, ch], ref_lab[:, :, ch])
            return cv2.cvtColor(result_lab, cv2.COLOR_LAB2BGR)
        else:
            return HistogramAnalyzer._match_channel(source, reference)

    @staticmethod
    def _match_channel(src, ref):
        """单通道直方图匹配"""
        src_hist, _ = np.histogram(src.flatten(), 256, [0, 256])
        ref_hist, _ = np.histogram(ref.flatten(), 256, [0, 256])

        # 计算 CDF
        src_cdf = src_hist.cumsum().astype(np.float64)
        ref_cdf = ref_hist.cumsum().astype(np.float64)
        src_cdf /= src_cdf[-1] if src_cdf[-1] > 0 else 1
        ref_cdf /= ref_cdf[-1] if ref_cdf[-1] > 0 else 1

        # 建立映射表
        lut = np.zeros(256, dtype=np.uint8)
        for i in range(256):
            j = np.searchsorted(ref_cdf, src_cdf[i])
            lut[i] = min(j, 255)

        return lut[src]

    # ---- 直方图比较 ----

    @staticmethod
    def compare_histograms(hist1, hist2, method='correlation'):
        """
        比较两个直方图的相似度
        :param method: 'correlation' / 'chi_square' / 'intersection' / 'bhattacharyya'
        :return: 相似度值 (越大越相似, chi_square 除外)
        """
        method_map = {
            'correlation':   cv2.HISTCMP_CORREL,
            'chi_square':    cv2.HISTCMP_CHISQR,
            'intersection':  cv2.HISTCMP_INTERSECT,
            'bhattacharyya': cv2.HISTCMP_BHATTACHYRYA,
        }
        cv_method = method_map.get(method, cv2.HISTCMP_CORREL)
        return cv2.compareHist(hist1.astype(np.float32),
                                hist2.astype(np.float32), cv_method)

    @staticmethod
    def compare_images_by_histogram(image1, image2, method='correlation'):
        """
        通过直方图比较两幅图像的相似度
        :return: 相似度分数
        """
        hist1 = cv2.calcHist([image1], [0], None, [256], [0, 256]).flatten()
        hist2 = cv2.calcHist([image2], [0], None, [256], [0, 256]).flatten()

        hist1 = hist1 / (hist1.sum() + 1e-10)
        hist2 = hist2 / (hist2.sum() + 1e-10)

        return HistogramAnalyzer.compare_histograms(hist1, hist2, method)

    @staticmethod
    def compare_multi_method(image1, image2):
        """用多种方法比较直方图, 返回所有分数"""
        methods = ['correlation', 'chi_square', 'intersection', 'bhattacharyya']
        return {m: HistogramAnalyzer.compare_images_by_histogram(image1, image2, m)
                for m in methods}

    # ---- 直方图统计 ----

    @staticmethod
    def histogram_stats(image):
        """
        计算图像统计信息
        :return: dict {mean, std, min, max, median, entropy}
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        hist = cv2.calcHist([gray], [0], None, [256], [0, 256]).flatten()
        hist_norm = hist / hist.sum()

        # 熵
        nonzero = hist_norm[hist_norm > 0]
        entropy = -np.sum(nonzero * np.log2(nonzero))

        return {
            'mean':    float(np.mean(gray)),
            'std':     float(np.std(gray)),
            'min':     int(np.min(gray)),
            'max':     int(np.max(gray)),
            'median':  float(np.median(gray)),
            'entropy': float(entropy),
        }

    @staticmethod
    def otsu_threshold(image):
        """
        Otsu 自动阈值分割
        :return: (阈值, 二值图)
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        thresh, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return thresh, binary


if __name__ == '__main__':
    img1 = np.random.randint(50, 200, (480, 640), dtype=np.uint8)
    img2 = np.random.randint(100, 255, (480, 640), dtype=np.uint8)

    hist1, _ = HistogramAnalyzer.calc_histogram(img1, -1)
    hist2, _ = HistogramAnalyzer.calc_histogram(img2, -1)

    print(f"Hist1 sum: {hist1.sum()}, Hist2 sum: {hist2.sum()}")
    print(f"Stats: {HistogramAnalyzer.histogram_stats(img1)}")
    print(f"Compare: {HistogramAnalyzer.compare_multi_method(img1, img2)}")

    equalized = HistogramAnalyzer.clahe(img1)
    print(f"CLAHE: {equalized.shape}")

    matched = HistogramAnalyzer.match_histogram(img1, img2)
    print(f"Match: {matched.shape}")
