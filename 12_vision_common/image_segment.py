"""
图像分割V2模块 - GrabCut/Watershed/MeanShift 分割
适用场景: 前景提取、粘连目标分离、颜色聚类分割
依赖: opencv-python, numpy
"""

import cv2
import numpy as np


class ImageSegmentV2:
    """图像分割工具集V2"""

    # ---- GrabCut 分割 ----

    @staticmethod
    def grabcut_segment(image, rect=None, mask_init=None, iter_count=5):
        """
        GrabCut 前景分割 (交互式, 需要初始矩形或掩模)
        :param image: BGR图像
        :param rect: 前景矩形 (x, y, w, h), 与 mask_init 二选一
        :param mask_init: 初始掩模 (0=背景, 1=前景), 可选
        :param iter_count: 迭代次数
        :return: (前景图像, 前景掩模)
        """
        h, w = image.shape[:2]
        mask = np.zeros((h, w), np.uint8)
        bgd_model = np.zeros((1, 65), np.float64)
        fgd_model = np.zeros((1, 65), np.float64)

        if mask_init is not None:
            mask[mask_init > 0] = cv2.GC_FGD
            mask[mask_init == 0] = cv2.GC_BGD
            cv2.grabCut(image, mask, None, bgd_model, fgd_model, iter_count, cv2.GC_INIT_WITH_MASK)
        elif rect is not None:
            cv2.grabCut(image, mask, rect, bgd_model, fgd_model, iter_count, cv2.GC_INIT_WITH_RECT)
        else:
            raise ValueError("必须提供 rect 或 mask_init 之一")

        # 生成最终掩模: 确定前景 + 可能前景
        result_mask = np.where((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)
        foreground = cv2.bitwise_and(image, image, mask=result_mask)
        return foreground, result_mask

    # ---- Watershed 分水岭 ----

    @staticmethod
    def watershed_segment(image, sure_fg_area=0.01, kernel_size=3):
        """
        Watershed 分水岭分割 (自动分离粘连目标)
        :param image: BGR图像
        :param sure_fg_area: 前景面积比 (相对总面积, 用于开运算)
        :param kernel_size: 形态学核大小
        :return: (labels, markers, 分割可视化)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image.copy()

        # 1. 阈值化 + 形态学处理
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        # 开运算去除噪点
        opening = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=2)

        # 确定背景区域 (膨胀)
        sure_bg = cv2.dilate(opening, kernel, iterations=3)

        # 2. 距离变换确定前景
        dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
        _, sure_fg = cv2.threshold(dist_transform, sure_fg_area * dist_transform.max(), 255, 0)
        sure_fg = sure_fg.astype(np.uint8)

        # 3. 未知区域
        unknown = cv2.subtract(sure_bg, sure_fg)

        # 4. 连通域标记
        num_labels, markers = cv2.connectedComponents(sure_fg)
        markers = markers + 1  # 背景标记为1
        markers[unknown == 255] = 0  # 未知区域标记为0

        # 5. Watershed
        markers = cv2.watershed(image, markers)

        # 可视化
        vis = image.copy()
        vis[markers == -1] = [0, 0, 255]  # 边界标红

        return markers, vis

    # ---- MeanShift 均值漂移分割 ----

    @staticmethod
    def meanshift_segment(image, spatial_radius=10, color_radius=30, min_density=50):
        """
        MeanShift 均值漂移分割 (颜色聚类, 无需预设类别数)
        :param image: BGR图像
        :param spatial_radius: 空间窗口半径
        :param color_radius: 颜色窗口半径
        :param min_density: 最小密度阈值
        :return: 分割后图像 (颜色聚类结果)
        """
        # OpenCV pyrMeanShiftFiltering
        shifted = cv2.pyrMeanShiftFiltering(image, spatial_radius, color_radius, min_density)
        return shifted

    @staticmethod
    def meanshift_labels(image, spatial_radius=10, color_radius=30, min_density=50):
        """
        MeanShift 分割并返回聚类标签
        :return: (segmented_image, label_map, num_regions)
        """
        shifted = ImageSegmentV2.meanshift_segment(image, spatial_radius, color_radius, min_density)
        gray = cv2.cvtColor(shifted, cv2.COLOR_BGR2GRAY)
        num_labels, labels = cv2.connectedComponents(cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1])
        return shifted, labels, num_labels

    # ---- 综合分割接口 ----

    @classmethod
    def segment(cls, image, method='watershed', **kwargs):
        """
        统一分割接口
        :param image: 输入BGR图像
        :param method: 'grabcut'|'watershed'|'meanshift'
        :return: 分割结果
        """
        method_map = {
            'grabcut': cls.grabcut_segment,
            'watershed': cls.watershed_segment,
            'meanshift': cls.meanshift_segment,
        }
        func = method_map.get(method)
        if func is None:
            raise ValueError(f"不支持的分割方法: {method}, 可选: {list(method_map.keys())}")
        return func(image, **kwargs)


# ======================== 快捷函数 ========================

def grabcut_segment(image, rect=None, mask_init=None, iter_count=5):
    return ImageSegmentV2.grabcut_segment(image, rect, mask_init, iter_count)

def watershed_segment(image, sure_fg_area=0.01):
    return ImageSegmentV2.watershed_segment(image, sure_fg_area)

def meanshift_segment(image, spatial_radius=10, color_radius=30):
    return ImageSegmentV2.meanshift_segment(image, spatial_radius, color_radius)
