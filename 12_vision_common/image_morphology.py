"""
形态学操作模块 - 腐蚀/膨胀/开闭/梯度/顶帽/黑帽
依赖: opencv-python, numpy
"""

import cv2
import numpy as np


class MorphologyProcessor:
    """形态学操作处理器"""

    @staticmethod
    def _get_kernel(kernel_size, kernel_shape='rect'):
        """
        生成形态学核
        :param kernel_size: 核大小 (int 或 (h, w))
        :param kernel_shape: 'rect' / 'ellipse' / 'cross'
        """
        if isinstance(kernel_size, int):
            ksize = (kernel_size, kernel_size)
        else:
            ksize = kernel_size

        shape_map = {
            'rect':   cv2.MORPH_RECT,
            'ellipse': cv2.MORPH_ELLIPSE,
            'cross':  cv2.MORPH_CROSS,
        }
        shape = shape_map.get(kernel_shape, cv2.MORPH_RECT)
        return cv2.getStructuringElement(shape, ksize)

    # ---- 基本操作 ----

    @staticmethod
    def erode(image, kernel_size=3, iterations=1, kernel_shape='rect'):
        """
        腐蚀 - 消除小噪点, 缩小前景
        :param iterations: 迭代次数
        """
        kernel = MorphologyProcessor._get_kernel(kernel_size, kernel_shape)
        return cv2.erode(image, kernel, iterations=iterations)

    @staticmethod
    def dilate(image, kernel_size=3, iterations=1, kernel_shape='rect'):
        """
        膨胀 - 填充小孔洞, 扩大前景
        """
        kernel = MorphologyProcessor._get_kernel(kernel_size, kernel_shape)
        return cv2.dilate(image, kernel, iterations=iterations)

    # ---- 复合操作 ----

    @staticmethod
    def opening(image, kernel_size=3, iterations=1, kernel_shape='rect'):
        """
        开运算 (先腐蚀后膨胀) - 去除小噪点, 断开细连接
        """
        kernel = MorphologyProcessor._get_kernel(kernel_size, kernel_shape)
        return cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel, iterations=iterations)

    @staticmethod
    def closing(image, kernel_size=3, iterations=1, kernel_shape='rect'):
        """
        闭运算 (先膨胀后腐蚀) - 填充小孔洞, 连接近邻
        """
        kernel = MorphologyProcessor._get_kernel(kernel_size, kernel_shape)
        return cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel, iterations=iterations)

    @staticmethod
    def gradient(image, kernel_size=3, kernel_shape='rect'):
        """
        形态学梯度 (膨胀 - 腐蚀) - 提取边缘
        """
        kernel = MorphologyProcessor._get_kernel(kernel_size, kernel_shape)
        return cv2.morphologyEx(image, cv2.MORPH_GRADIENT, kernel)

    @staticmethod
    def tophat(image, kernel_size=15, kernel_shape='rect'):
        """
        顶帽 / 礼帽 (原图 - 开运算) - 提取亮细节
        """
        kernel = MorphologyProcessor._get_kernel(kernel_size, kernel_shape)
        return cv2.morphologyEx(image, cv2.MORPH_TOPHAT, kernel)

    @staticmethod
    def blackhat(image, kernel_size=15, kernel_shape='rect'):
        """
        黑帽 (闭运算 - 原图) - 提取暗细节
        """
        kernel = MorphologyProcessor._get_kernel(kernel_size, kernel_shape)
        return cv2.morphologyEx(image, cv2.MORPH_BLACKHAT, kernel)

    # ---- 高级操作 ----

    @staticmethod
    def hit_or_miss(image, kernel=None):
        """
        击中击不中变换 - 模式匹配
        :param kernel: 核 (-1/0/1), 默认 3x3 十字
        """
        if kernel is None:
            kernel = np.array([[0, 1, 0],
                               [1, -1, 1],
                               [0, 1, 0]], dtype=np.int32)
        else:
            kernel = np.array(kernel, dtype=np.int32)
        return cv2.morphologyEx(image, cv2.MORPH_HITMISS, kernel)

    @staticmethod
    def skeleton(image, threshold=127):
        """
        骨架提取 (细化)
        :param image: 灰度图
        :param threshold: 二值化阈值
        :return: 骨架图像
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
        binary = binary.astype(np.uint8)

        skeleton = np.zeros_like(binary)
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

        temp = binary.copy()
        while True:
            eroded = cv2.erode(temp, kernel)
            opened = cv2.dilate(eroded, kernel)
            subset = cv2.subtract(temp, opened)
            skeleton = cv2.bitwise_or(skeleton, subset)
            temp = eroded.copy()
            if cv2.countNonZero(temp) == 0:
                break

        return skeleton

    @staticmethod
    def reconstruct_by_dilation(marker, mask, kernel_size=3):
        """
        膨胀形态学重建
        :param marker: 标记图 (<= mask)
        :param mask: 掩码图
        """
        kernel = MorphologyProcessor._get_kernel(kernel_size)
        result = marker.copy()
        while True:
            dilated = cv2.dilate(result, kernel)
            new_result = cv2.bitwise_and(dilated, mask)
            if np.array_equal(new_result, result):
                break
            result = new_result
        return result

    @staticmethod
    def remove_small_objects(binary, min_size=100):
        """
        移除二值图中小于 min_size 的连通域
        :param binary: 二值图 (0/255)
        :param min_size: 最小面积
        """
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            binary, connectivity=8)
        result = np.zeros_like(binary)
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] >= min_size:
                result[labels == i] = 255
        return result

    # ---- 批量对比 ----

    @staticmethod
    def compare_all(image, kernel_size=5):
        """应用所有形态学操作并返回字典"""
        return {
            'original':  image,
            'erode':     MorphologyProcessor.erode(image, kernel_size),
            'dilate':    MorphologyProcessor.dilate(image, kernel_size),
            'opening':   MorphologyProcessor.opening(image, kernel_size),
            'closing':   MorphologyProcessor.closing(image, kernel_size),
            'gradient':  MorphologyProcessor.gradient(image, kernel_size),
            'tophat':    MorphologyProcessor.tophat(image, kernel_size),
            'blackhat':  MorphologyProcessor.blackhat(image, kernel_size),
        }


if __name__ == '__main__':
    # 测试
    img = np.zeros((200, 200), dtype=np.uint8)
    cv2.circle(img, (60, 60), 30, 255, -1)
    cv2.circle(img, (140, 140), 30, 255, -1)
    cv2.rectangle(img, (80, 80), (120, 120), 255, -1)

    results = MorphologyProcessor.compare_all(img, 5)
    for name, r in results.items():
        print(f"{name}: nonzero={np.count_nonZero(r)}")

    skel = MorphologyProcessor.skeleton(img)
    print(f"skeleton: nonzero={np.count_nonZero(skel)}")
