"""
颜色直方图分析模块 - 用于颜色匹配和分类
适用于电赛中的颜色识别、目标分类和匹配
"""

import cv2
import numpy as np


class ColorHistogramAnalyzer:
    """颜色直方图分析器"""

    def __init__(self, h_bins=30, s_bins=32):
        """
        初始化

        Args:
            h_bins: H通道bin数
            s_bins: S通道bin数
        """
        self.h_bins = h_bins
        self.s_bins = s_bins

    def calc_hs_histogram(self, image, mask=None):
        """
        计算HS(色相-饱和度)直方图

        Args:
            image: BGR图像
            mask: 掩码(可选)

        Returns:
            归一化直方图
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist(
            [hsv], [0, 1], mask,
            [self.h_bins, self.s_bins],
            [0, 180, 0, 256]
        )
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        return hist

    def calc_h_histogram(self, image, mask=None):
        """
        仅计算色相(H)直方图

        Args:
            image: BGR图像
            mask: 掩码

        Returns:
            归一化直方图
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0], mask, [self.h_bins], [0, 180])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        return hist

    def calc_rgb_histogram(self, image, mask=None, bins=32):
        """
        计算RGB三通道直方图

        Args:
            image: BGR图像
            mask: 掩码
            bins: 每通道bin数

        Returns:
            (hist_b, hist_g, hist_r)
        """
        hists = []
        for i in range(3):
            h = cv2.calcHist([image], [i], mask, [bins], [0, 256])
            cv2.normalize(h, h, 0, 1, cv2.NORM_MINMAX)
            hists.append(h)
        return tuple(hists)

    def compare_histograms(self, hist1, hist2, method='bhattacharyya'):
        """
        比较两个直方图的相似度

        Args:
            hist1, hist2: 直方图
            method: 比较方法
                'correlation' - 相关性 (越接近1越相似)
                'chi_square' - 卡方 (越小越相似)
                'intersection' - 交叉 (越大越相似)
                'bhattacharyya' - 巴氏距离 (越小越相似, 0=完全相同)

        Returns:
            相似度值
        """
        methods = {
            'correlation': cv2.HISTCMP_CORREL,
            'chi_square': cv2.HISTCMP_CHISQR,
            'intersection': cv2.HISTCMP_INTERSECT,
            'bhattacharyya': cv2.HISTCMP_BHATTACHARYYA
        }
        method_flag = methods.get(method, cv2.HISTCMP_BHATTACHARYYA)
        return cv2.compareHist(hist1, hist2, method_flag)

    def match_template_by_histogram(self, template_hist, roi_image):
        """
        通过直方图匹配判断ROI与模板的相似度

        Args:
            template_hist: 模板直方图
            roi_image: 感兴趣区域图像

        Returns:
            相似度分数 (0~1, 越大越相似)
        """
        roi_hist = self.calc_hs_histogram(roi_image)
        # 巴氏距离 -> 相似度
        dist = cv2.compareHist(template_hist, roi_hist, cv2.HISTCMP_BHATTACHARYYA)
        return 1.0 - dist

    def create_color_model(self, sample_images, masks=None):
        """
        从样本图像创建颜色模型(直方图模板)

        Args:
            sample_images: 样本图像列表
            masks: 对应掩码列表(可选)

        Returns:
            平均直方图
        """
        hists = []
        for i, img in enumerate(sample_images):
            mask = masks[i] if masks else None
            hist = self.calc_hs_histogram(img, mask)
            hists.append(hist)

        # 取平均
        avg_hist = np.mean(hists, axis=0)
        # 归一化
        cv2.normalize(avg_hist, avg_hist, 0, 1, cv2.NORM_MINMAX)
        return avg_hist

    def backproject(self, image, model_hist):
        """
        反向投影 - 用颜色模型在图像中找到匹配区域

        Args:
            image: 输入BGR图像
            model_hist: 模型直方图

        Returns:
            反向投影图(单通道, 0~255)
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        bp = cv2.calcBackProject([hsv], [0, 1], model_hist, [0, 180, 0, 256], 1)
        return bp

    def find_color_regions(self, image, model_hist, threshold=50):
        """
        通过反向投影找到颜色匹配区域

        Args:
            image: 输入BGR图像
            model_hist: 模型直方图
            threshold: 二值化阈值

        Returns:
            (mask, contours) - 匹配掩码和轮廓
        """
        bp = self.backproject(image, model_hist)

        # 形态学处理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        bp = cv2.filter2D(bp, -1, kernel)
        _, mask = cv2.threshold(bp, threshold, 255, cv2.THRESH_BINARY)

        # 开闭运算去噪
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return mask, contours

    def classify_by_color(self, image, color_models):
        """
        根据颜色模型对图像区域进行分类

        Args:
            image: 输入BGR图像
            color_models: dict {类别名: 模型直方图}

        Returns:
            (best_class, scores_dict) - 最佳类别和所有得分
        """
        roi_hist = self.calc_hs_histogram(image)
        scores = {}
        for name, model_hist in color_models.items():
            dist = cv2.compareHist(model_hist, roi_hist, cv2.HISTCMP_BHATTACHARYYA)
            scores[name] = 1.0 - dist

        best_class = max(scores, key=scores.get)
        return best_class, scores

    def get_dominant_color_hsv(self, image, mask=None):
        """
        获取图像的主色调(HSV)

        Args:
            image: BGR图像
            mask: 掩码

        Returns:
            (H, S, V) - 主色调HSV值
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h_hist = cv2.calcHist([hsv], [0], mask, [180], [0, 180])
        s_hist = cv2.calcHist([hsv], [1], mask, [256], [0, 256])
        v_hist = cv2.calcHist([hsv], [2], mask, [256], [0, 256])

        h_dom = int(np.argmax(h_hist))
        s_dom = int(np.argmax(s_hist))
        v_dom = int(np.argmax(v_hist))

        return h_dom, s_dom, v_dom

    def get_color_proportions(self, image, color_ranges):
        """
        计算各颜色在图像中的占比

        Args:
            image: BGR图像
            color_ranges: dict {颜色名: (lower_hsv, upper_hsv)}

        Returns:
            dict {颜色名: 占比(0~1)}
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        total_pixels = image.shape[0] * image.shape[1]

        proportions = {}
        for name, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            count = cv2.countNonZero(mask)
            proportions[name] = count / total_pixels

        return proportions

    def draw_histogram(self, image, hist, hist_h=200, hist_w=512, channel=0):
        """
        绘制直方图可视化

        Args:
            image: 原图(用于标题)
            hist: 直方图
            hist_h, hist_w: 画布尺寸
            channel: 通道(0=H, 1=S, 2=V)

        Returns:
            直方图图像
        """
        canvas = np.zeros((hist_h, hist_w, 3), dtype=np.uint8)
        bin_w = int(round(hist_w / len(hist)))

        cv2.normalize(hist, hist, 0, hist_h, cv2.NORM_MINMAX)

        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        color = colors[channel % 3]

        for i in range(1, len(hist)):
            cv2.line(canvas,
                     (bin_w * (i - 1), hist_h - int(hist[i - 1])),
                     (bin_w * i, hist_h - int(hist[i])),
                     color, 2)

        return canvas


# ==================== 使用示例 ====================
if __name__ == '__main__':
    analyzer = ColorHistogramAnalyzer()

    # 创建测试图像
    red_img = np.zeros((100, 100, 3), dtype=np.uint8)
    red_img[:] = (0, 0, 255)  # BGR红色

    blue_img = np.zeros((100, 100, 3), dtype=np.uint8)
    blue_img[:] = (255, 0, 0)  # BGR蓝色

    red_hist = analyzer.calc_hs_histogram(red_img)
    blue_hist = analyzer.calc_hs_histogram(blue_img)

    sim = analyzer.compare_histograms(red_hist, blue_hist, 'bhattacharyya')
    print(f"红色 vs 蓝色 巴氏距离: {sim:.4f}")

    sim2 = analyzer.compare_histograms(red_hist, red_hist, 'bhattacharyya')
    print(f"红色 vs 红色 巴氏距离: {sim2:.4f}")

    # 主色调
    h, s, v = analyzer.get_dominant_color_hsv(red_img)
    print(f"红色图像主色调 HSV: ({h}, {s}, {v})")

    # 颜色占比
    ranges = {
        'red': (np.array([0, 100, 100]), np.array([10, 255, 255])),
        'blue': (np.array([100, 100, 100]), np.array([130, 255, 255])),
    }
    # 需要一个混合图像
    mix = np.zeros((100, 200, 3), dtype=np.uint8)
    mix[:, :100] = (0, 0, 255)
    mix[:, 100:] = (255, 0, 0)
    props = analyzer.get_color_proportions(mix, ranges)
    print(f"颜色占比: {props}")
