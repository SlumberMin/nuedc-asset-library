"""
Haar特征提取模块 - Haar-like特征
适用于电赛中的人脸检测、目标检测和级联分类器应用
"""

import cv2
import numpy as np


class HaarFeatureExtractor:
    """Haar-like特征提取器"""

    def __init__(self, feature_types=None):
        """
        初始化

        Args:
            feature_types: Haar特征类型列表
                'two_horizontal' - 两矩形水平(左白右黑)
                'two_vertical'   - 两矩形垂直(上白下黑)
                'three_horizontal' - 三矩形水平
                'three_vertical'   - 三矩形垂直
                'four' - 四矩形对角
        """
        if feature_types is None:
            self.feature_types = [
                'two_horizontal',
                'two_vertical',
                'three_horizontal',
                'three_vertical',
                'four'
            ]
        else:
            self.feature_types = feature_types

    def _compute_integral(self, gray):
        """计算积分图"""
        integral = cv2.integral(gray)
        return integral.astype(np.float64)

    def _rect_sum(self, integral, x, y, w, h):
        """利用积分图快速计算矩形区域和"""
        return (integral[y+h, x+w] - integral[y, x+w]
                - integral[y+h, x] + integral[y, x])

    def extract_at_position(self, integral, x, y, w, h):
        """
        在指定位置提取Haar特征

        Args:
            integral: 积分图
            x, y: 左上角坐标
            w, h: 窗口宽高

        Returns:
            list: 各类型Haar特征值
        """
        features = []
        hw, hh = w // 2, h // 2

        for ftype in self.feature_types:
            if ftype == 'two_horizontal' and w >= 2:
                white = self._rect_sum(integral, x, y, hw, h)
                black = self._rect_sum(integral, x + hw, y, hw, h)
                features.append(white - black)

            elif ftype == 'two_vertical' and h >= 2:
                white = self._rect_sum(integral, x, y, w, hh)
                black = self._rect_sum(integral, x, y + hh, w, hh)
                features.append(white - black)

            elif ftype == 'three_horizontal' and w >= 3:
                sw = w // 3
                left = self._rect_sum(integral, x, y, sw, h)
                center = self._rect_sum(integral, x + sw, y, sw, h)
                right = self._rect_sum(integral, x + 2*sw, y, sw, h)
                features.append(left + right - 2 * center)

            elif ftype == 'three_vertical' and h >= 3:
                sh = h // 3
                top = self._rect_sum(integral, x, y, w, sh)
                mid = self._rect_sum(integral, x, y + sh, w, sh)
                bot = self._rect_sum(integral, x, y + 2*sh, w, sh)
                features.append(top + bot - 2 * mid)

            elif ftype == 'four' and w >= 2 and h >= 2:
                tl = self._rect_sum(integral, x, y, hw, hh)
                tr = self._rect_sum(integral, x + hw, y, hw, hh)
                bl = self._rect_sum(integral, x, y + hh, hw, hh)
                br = self._rect_sum(integral, x + hw, y + hh, hw, hh)
                features.append((tl + br) - (tr + bl))

        return features

    def extract_all(self, image, window_size=(24, 24), step=1,
                    scale_factor=1.25, n_scales=3):
        """
        多尺度提取所有Haar特征

        Args:
            image: 输入图像(灰度)
            window_size: 扫描窗口大小(w, h)
            step: 滑动步长
            scale_factor: 尺度缩放因子
            n_scales: 尺度数

        Returns:
            list: [(x, y, scale, features), ...]
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        gray = gray.astype(np.float64)

        all_features = []
        win_w, win_h = window_size

        for s in range(n_scales):
            scale = scale_factor ** s
            sw = int(win_w * scale)
            sh = int(win_h * scale)

            if sw > gray.shape[1] or sh > gray.shape[0]:
                break

            # 缩放图像
            if s > 0:
                scaled = cv2.resize(gray.astype(np.uint8),
                                    (int(gray.shape[1] / scale),
                                     int(gray.shape[0] / scale)))
                scaled = scaled.astype(np.float64)
            else:
                scaled = gray

            integral = self._compute_integral(scaled.astype(np.uint8))
            h, w = scaled.shape

            for y in range(0, h - win_h + 1, step):
                for x in range(0, w - win_w + 1, step):
                    feats = self.extract_at_position(integral, x, y, win_w, win_h)
                    all_features.append((x, y, scale, feats))

        return all_features

    def extract_feature_vector(self, image, window_size=(24, 24)):
        """
        提取单个窗口的Haar特征向量

        Args:
            image: 输入图像(灰度或BGR)
            window_size: 窗口大小

        Returns:
            numpy.ndarray: Haar特征向量
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        resized = cv2.resize(gray, window_size).astype(np.uint8)
        integral = self._compute_integral(resized)
        w, h = window_size

        features = []
        # 遍历多种窗口大小
        for ww in range(2, w + 1, 2):
            for wh in range(2, h + 1, 2):
                for x in range(0, w - ww + 1, max(1, ww // 2)):
                    for y in range(0, h - wh + 1, max(1, wh // 2)):
                        feats = self.extract_at_position(integral, x, y, ww, wh)
                        features.extend(feats)

        return np.array(features, dtype=np.float32)

    def use_cascade_detector(self, cascade_path='haarcascade_frontalface_default.xml'):
        """
        使用OpenCV级联分类器(预训练Haar模型)

        Args:
            cascade_path: cascade XML文件路径

        Returns:
            cv2.CascadeClassifier
        """
        cascade = cv2.CascadeClassifier(cascade_path)
        if cascade.empty():
            # 尝试OpenCV自带路径
            default_path = cv2.data.haarcascades + cascade_path
            cascade = cv2.CascadeClassifier(default_path)
        if cascade.empty():
            raise FileNotFoundError(f"无法加载cascade文件: {cascade_path}")
        return cascade

    def detect_faces(self, image, scale_factor=1.1, min_neighbors=5,
                     min_size=(30, 30)):
        """
        人脸检测(使用预训练Haar级联)

        Args:
            image: BGR图像
            scale_factor: 尺度缩放因子
            min_neighbors: 最小邻居数
            min_size: 最小检测尺寸

        Returns:
            list: 人脸矩形 [(x, y, w, h), ...]
        """
        cascade = self.use_cascade_detector()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = cascade.detectMultiScale(
            gray, scaleFactor=scale_factor,
            minNeighbors=min_neighbors, minSize=min_size
        )
        return [tuple(f) for f in faces]

    def detect_with_custom_cascade(self, image, cascade_path,
                                   scale_factor=1.1, min_neighbors=5):
        """
        使用自定义级联分类器检测

        Args:
            image: BGR图像
            cascade_path: cascade文件路径
            scale_factor: 尺度因子
            min_neighbors: 最小邻居数

        Returns:
            list: 检测到的矩形
        """
        cascade = self.use_cascade_detector(cascade_path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        objects = cascade.detectMultiScale(
            gray, scaleFactor=scale_factor,
            minNeighbors=min_neighbors
        )
        if len(objects) == 0:
            return []
        return [tuple(o) for o in objects]

    def count_features(self, image_size=(24, 24)):
        """
        计算给定窗口大小的Haar特征总数

        Args:
            image_size: (w, h)

        Returns:
            int: 特征数
        """
        w, h = image_size
        count = 0
        for ww in range(2, w + 1, 2):
            for wh in range(2, h + 1, 2):
                for x in range(0, w - ww + 1, max(1, ww // 2)):
                    for y in range(0, h - wh + 1, max(1, wh // 2)):
                        count += len(self.feature_types)
        return count


# ==================== 便捷函数 ====================

def detect_faces_haar(image, scale=1.1, min_neighbors=5):
    """
    便捷函数: Haar人脸检测

    Args:
        image: BGR图像
        scale: 尺度因子
        min_neighbors: 最小邻居数

    Returns:
        list: 人脸矩形列表
    """
    extractor = HaarFeatureExtractor()
    return extractor.detect_faces(image, scale_factor=scale,
                                  min_neighbors=min_neighbors)


def extract_haar_feature(image, window_size=(24, 24)):
    """
    便捷函数: 提取Haar特征向量

    Args:
        image: 输入图像
        window_size: 窗口大小

    Returns:
        numpy.ndarray: 特征向量
    """
    extractor = HaarFeatureExtractor()
    return extractor.extract_feature_vector(image, window_size=window_size)


# ==================== 测试 ====================

if __name__ == '__main__':
    # 创建测试图像
    img = np.zeros((100, 100), dtype=np.uint8)
    cv2.rectangle(img, (20, 20), (80, 80), 180, -1)
    cv2.rectangle(img, (35, 35), (65, 65), 100, -1)

    extractor = HaarFeatureExtractor()

    # 测试积分图 + 单位置特征
    integral = cv2.integral(img).astype(np.float64)
    feats = extractor.extract_at_position(integral, 10, 10, 24, 24)
    print(f"单窗口Haar特征值: {feats}")
    print(f"特征类型数: {len(extractor.feature_types)}")

    # 特征数统计
    n = extractor.count_features((24, 24))
    print(f"24x24窗口Haar特征总数: {n}")

    # 人脸检测测试(需要OpenCV级联文件)
    try:
        faces = detect_faces_haar(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
        print(f"检测到人脸数: {len(faces)}")
    except Exception as e:
        print(f"人脸检测测试跳过: {e}")
