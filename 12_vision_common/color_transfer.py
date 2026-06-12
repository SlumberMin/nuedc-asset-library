"""
颜色迁移模块 - 直方图匹配 + Reinhard方法 + 风格化
功能：将一张图像的颜色风格迁移到另一张图像
依赖：opencv-python, numpy
"""

import cv2
import numpy as np


# ==================== 基础颜色迁移 ====================

class ColorTransfer:
    """颜色迁移算法集合"""

    @staticmethod
    def reinhard_transfer(source, reference):
        """
        Reinhard颜色迁移算法
        将参考图像的颜色统计特征迁移到源图像
        
        原理：在LAB颜色空间中，对L/A/B三通道分别做
              target = (src - src_mean) * (ref_std / src_std) + ref_mean
        
        参数:
            source: 源图像（待迁移）
            reference: 参考图像（目标风格）
            
        返回:
            result: 迁移后的图像
        """
        # 转换到LAB颜色空间（亮度和颜色分离）
        src_lab = cv2.cvtColor(source, cv2.COLOR_BGR2LAB).astype(np.float32)
        ref_lab = cv2.cvtColor(reference, cv2.COLOR_BGR2LAB).astype(np.float32)

        # 计算各通道均值和标准差
        src_mean, src_std = cv2.meanStdDev(src_lab)
        ref_mean, ref_std = cv2.meanStdDev(ref_lab)

        # 防止除零
        src_std = np.maximum(src_std, 1e-6)

        # 逐通道迁移
        result = np.zeros_like(src_lab)
        for i in range(3):
            result[:, :, i] = (src_lab[:, :, i] - src_mean[i][0]) * \
                               (ref_std[i][0] / src_std[i][0]) + ref_mean[i][0]

        result = np.clip(result, 0, 255).astype(np.uint8)
        return cv2.cvtColor(result, cv2.COLOR_LAB2BGR)

    @staticmethod
    def histogram_transfer(source, reference, bins=256):
        """
        直方图匹配（直方图规定化）
        使源图像的直方图分布匹配参考图像
        
        参数:
            source: 源图像
            reference: 参考图像
            bins: 直方图bin数量
            
        返回:
            result: 匹配后的图像
        """
        # 转换到HSV空间
        src_hsv = cv2.cvtColor(source, cv2.COLOR_BGR2HSV)
        ref_hsv = cv2.cvtColor(reference, cv2.COLOR_BGR2HSV)

        result = np.zeros_like(src_hsv)

        for ch in range(3):
            src_ch = src_hsv[:, :, ch].ravel()
            ref_ch = ref_hsv[:, :, ch].ravel()

            # 计算累积分布函数(CDF)
            src_hist, _ = np.histogram(src_ch, bins, (0, bins))
            ref_hist, _ = np.histogram(ref_ch, bins, (0, bins))

            src_cdf = src_hist.cumsum().astype(np.float64)
            ref_cdf = ref_hist.cumsum().astype(np.float64)

            # 归一化
            src_cdf /= src_cdf[-1] if src_cdf[-1] > 0 else 1
            ref_cdf /= ref_cdf[-1] if ref_cdf[-1] > 0 else 1

            # 建立映射关系：对每个源像素值，找到CDF最接近的参考值
            mapping = np.zeros(bins, dtype=np.uint8)
            for src_val in range(bins):
                # 找到ref_cdf中与src_cdf[src_val]最接近的索引
                diff = np.abs(ref_cdf - src_cdf[src_val])
                mapping[src_val] = np.argmin(diff)

            # 应用映射
            result[:, :, ch] = mapping[src_hsv[:, :, ch]]

        return cv2.cvtColor(result, cv2.COLOR_HSV2BGR)

    @staticmethod
    def selective_color_transfer(source, reference, target_color='skin'):
        """
        选择性颜色迁移：只迁移特定颜色区域
        
        参数:
            source: 源图像
            reference: 参考图像
            target_color: 目标颜色类型 ('skin','sky','green','warm','cool')
            
        返回:
            result: 迁移后的图像
        """
        # 定义HSV颜色范围
        color_ranges = {
            'skin': ((0, 30, 60), (25, 200, 255)),      # 肤色
            'sky': ((90, 30, 100), (130, 255, 255)),     # 天空蓝
            'green': ((35, 30, 30), (85, 255, 255)),     # 绿色
            'warm': ((0, 50, 50), (30, 255, 255)),       # 暖色
            'cool': ((90, 30, 30), (130, 255, 255)),     # 冷色
        }

        if target_color not in color_ranges:
            target_color = 'skin'

        lower, upper = color_ranges[target_color]

        # 创建源图像中目标颜色区域的掩码
        src_hsv = cv2.cvtColor(source, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(src_hsv, np.array(lower), np.array(upper))

        # 形态学处理平滑掩码
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.GaussianBlur(mask, (21, 21), 10)

        # 对整个图像做颜色迁移
        full_transfer = ColorTransfer.reinhard_transfer(source, reference)

        # 只在目标区域应用迁移结果
        mask_3ch = cv2.merge([mask, mask, mask]).astype(np.float32) / 255.0
        result = source.astype(np.float32) * (1 - mask_3ch) + \
                 full_transfer.astype(np.float32) * mask_3ch

        return np.clip(result, 0, 255).astype(np.uint8)


# ==================== 风格化迁移 ====================

class StyleTransfer:
    """基于颜色统计的风格化方法"""

    @staticmethod
    def vintage_style(image):
        """
        复古风格：降低饱和度，增加暖色调
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
        # 降低饱和度
        hsv[:, :, 1] *= 0.6
        # 偏暖色调
        hsv[:, :, 0] = np.clip(hsv[:, :, 0] * 0.8 + 10, 0, 180)
        # 稍微降低亮度
        hsv[:, :, 2] = np.clip(hsv[:, :, 2] * 0.9, 0, 255)
        result = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

        # 添加轻微的棕褐色调
        sepia = np.array([[0.272, 0.534, 0.131],
                          [0.349, 0.686, 0.168],
                          [0.393, 0.769, 0.189]])
        sepia_img = cv2.transform(result, sepia.T)
        result = cv2.addWeighted(result, 0.6, sepia_img, 0.4, 0)

        return np.clip(result, 0, 255).astype(np.uint8)

    @staticmethod
    def cold_style(image):
        """
        冷色调风格：增强蓝色通道
        """
        b, g, r = cv2.split(image.astype(np.float32))
        b = np.clip(b * 1.2 + 10, 0, 255)
        r = np.clip(r * 0.8, 0, 255)
        return cv2.merge([b, g, r]).astype(np.uint8)

    @staticmethod
    def warm_style(image):
        """
        暖色调风格：增强红/黄通道
        """
        b, g, r = cv2.split(image.astype(np.float32))
        r = np.clip(r * 1.2 + 10, 0, 255)
        g = np.clip(g * 1.05, 0, 255)
        b = np.clip(b * 0.8, 0, 255)
        return cv2.merge([b, g, r]).astype(np.uint8)

    @staticmethod
    def high_contrast_style(image, factor=1.5):
        """
        高对比度风格
        """
        mean = np.mean(image, axis=(0, 1), keepdims=True)
        result = (image.astype(np.float32) - mean) * factor + mean
        return np.clip(result, 0, 255).astype(np.uint8)

    @staticmethod
    def cross_process(image):
        """
        交叉冲洗效果：模拟胶片交叉冲洗
        """
        b, g, r = cv2.split(image)

        # 创建查找表
        lut_r = np.clip(np.arange(256) * 1.1, 0, 255).astype(np.uint8)
        lut_g = np.clip(np.arange(256) * 1.0 + 20, 0, 255).astype(np.uint8)
        lut_b = np.clip(np.arange(256) * 0.9, 0, 255).astype(np.uint8)

        # 应用LUT
        r = cv2.LUT(r, lut_r)
        g = cv2.LUT(g, lut_g)
        b = cv2.LUT(b, lut_b)

        result = cv2.merge([b, g, r])

        # 增加对比度
        result = StyleTransfer.high_contrast_style(result, 1.2)

        return result


# ==================== 颜色空间工具 ====================

class ColorSpaceUtils:
    """颜色空间转换和分析工具"""

    @staticmethod
    def analyze_color_distribution(image):
        """
        分析图像颜色分布
        
        返回:
            stats: 各通道统计信息字典
        """
        b, g, r = cv2.split(image)
        stats = {
            'R': {'mean': np.mean(r), 'std': np.std(r),
                  'min': np.min(r), 'max': np.max(r)},
            'G': {'mean': np.mean(g), 'std': np.std(g),
                  'min': np.min(g), 'max': np.max(g)},
            'B': {'mean': np.mean(b), 'std': np.std(b),
                  'min': np.min(b), 'max': np.max(b)},
        }

        # HSV分析
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        stats['H'] = {'mean': np.mean(hsv[:, :, 0]), 'dominant': int(
            np.bincount(hsv[:, :, 0].ravel()).argmax())}
        stats['S'] = {'mean': np.mean(hsv[:, :, 1])}
        stats['V'] = {'mean': np.mean(hsv[:, :, 2])}

        return stats

    @staticmethod
    def dominant_colors(image, k=5):
        """
        提取图像主色调（K-means聚类）
        
        参数:
            image: BGR图像
            k: 聚类数量
            
        返回:
            colors: 主色调RGB值列表
            percentages: 各颜色占比
        """
        # 缩小图像加速计算
        small = cv2.resize(image, (100, 100))
        data = small.reshape(-1, 3).astype(np.float32)

        # K-means聚类
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
        _, labels, centers = cv2.kmeans(data, k, None, criteria, 10,
                                         cv2.KMEANS_RANDOM_CENTERS)

        # 计算各颜色占比
        colors = centers.astype(np.uint8).tolist()
        percentages = []
        for i in range(k):
            pct = np.sum(labels == i) / len(labels)
            percentages.append(float(pct))

        # 按占比排序
        sorted_idx = np.argsort(percentages)[::-1]
        colors = [colors[i] for i in sorted_idx]
        percentages = [percentages[i] for i in sorted_idx]

        return colors, percentages


# ==================== 使用示例 ====================

def example_reinhard_transfer():
    """Reinhard颜色迁移示例"""
    src = cv2.imread("source.jpg")
    ref = cv2.imread("reference.jpg")

    if src is None or ref is None:
        # 创建测试图像
        src = np.zeros((200, 300, 3), dtype=np.uint8)
        src[:, :] = (180, 100, 50)  # 蓝色调
        cv2.circle(src, (150, 100), 50, (50, 200, 180), -1)

        ref = np.zeros((200, 300, 3), dtype=np.uint8)
        ref[:, :] = (50, 100, 200)  # 红色调
        cv2.circle(ref, (150, 100), 50, (200, 180, 50), -1)

    # Reinhard迁移
    result = ColorTransfer.reinhard_transfer(src, ref)

    # 直方图匹配
    result_hist = ColorTransfer.histogram_transfer(src, ref)

    cv2.imshow("Source", src)
    cv2.imshow("Reference", ref)
    cv2.imshow("Reinhard Result", result)
    cv2.imshow("Histogram Match", result_hist)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def example_style_effects():
    """风格化效果示例"""
    image = cv2.imread("test.jpg")
    if image is None:
        image = np.random.randint(50, 200, (300, 400, 3), dtype=np.uint8)

    # 应用不同风格
    vintage = StyleTransfer.vintage_style(image)
    cold = StyleTransfer.cold_style(image)
    warm = StyleTransfer.warm_style(image)
    cross = StyleTransfer.cross_process(image)

    # 分析颜色分布
    stats = ColorSpaceUtils.analyze_color_distribution(image)
    print("颜色分布:", stats)

    # 提取主色调
    colors, pcts = ColorSpaceUtils.dominant_colors(image, k=5)
    print("主色调:", list(zip(colors, [f"{p:.1%}" for p in pcts])))

    cv2.imshow("Original", image)
    cv2.imshow("Vintage", vintage)
    cv2.imshow("Cold", cold)
    cv2.imshow("Warm", warm)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    print("=== 颜色迁移模块 ===")
    print("算法: Reinhard迁移 / 直方图匹配 / 选择性迁移 / 风格化")
    example_reinhard_transfer()
