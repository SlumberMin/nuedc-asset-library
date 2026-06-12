"""
HOG特征提取模块 - 方向梯度直方图
适用于电赛中的目标检测、行人检测和形状分类
"""

import cv2
import numpy as np


class HOGFeatureExtractor:
    """HOG(方向梯度直方图)特征提取器"""

    def __init__(self, win_size=(64, 128), block_size=(16, 16),
                 block_stride=(8, 8), cell_size=(8, 8), nbins=9):
        """
        初始化HOG描述符

        Args:
            win_size: 检测窗口大小(w, h)
            block_size: 块大小(w, h)
            block_stride: 块步长(w, h)
            cell_size: 单元格大小(w, h)
            nbins: 方向bin数
        """
        self.win_size = win_size
        self.block_size = block_size
        self.block_stride = block_stride
        self.cell_size = cell_size
        self.nbins = nbins

        self.hog = cv2.HOGDescriptor(win_size, block_size, block_stride,
                                      cell_size, nbins)

    def extract(self, image):
        """
        提取HOG特征

        Args:
            image: 灰度图或BGR图

        Returns:
            numpy.ndarray: HOG特征向量
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        resized = cv2.resize(gray, self.win_size)
        descriptor = self.hog.compute(resized)
        return descriptor.flatten()

    def extract_dense(self, image, step_size=8):
        """
        密集HOG特征提取(滑窗方式)

        Args:
            image: 输入图像
            step_size: 滑窗步长

        Returns:
            list: [(x, y, feature), ...] 每个位置的HOG特征
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        h, w = gray.shape
        win_w, win_h = self.win_size
        features = []

        for y in range(0, h - win_h + 1, step_size):
            for x in range(0, w - win_w + 1, step_size):
                patch = gray[y:y+win_h, x:x+win_w]
                feat = self.hog.compute(patch).flatten()
                features.append((x, y, feat))

        return features

    def visualize(self, image, scale=1.0):
        """
        可视化HOG特征

        Args:
            image: 输入图像
            scale: 缩放因子

        Returns:
            numpy.ndarray: HOG可视化图像
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        resized = cv2.resize(gray, self.win_size)
        descriptor = self.hog.compute(resized)

        # 绘制HOG可视化
        cell_w, cell_h = self.cell_size
        n_cells_x = self.win_size[0] // cell_w
        n_cells_y = self.win_size[1] // cell_h

        vis = np.zeros((self.win_size[1], self.win_size[0]), dtype=np.float32)

        # 重建每个cell的梯度方向
        for cy in range(n_cells_y):
            for cx in range(n_cells_x):
                # 每个cell有nbins个方向
                cell_center_x = cx * cell_w + cell_w // 2
                cell_center_y = cy * cell_h + cell_h // 2

                # 获取该cell在描述符中的贡献
                bin_idx = 0
                for b in range(self.nbins):
                    angle = b * (180.0 / self.nbins)
                    rad = np.radians(angle)
                    dx = int(np.cos(rad) * cell_w / 2)
                    dy = int(np.sin(rad) * cell_h / 2)
                    x1 = max(0, cell_center_x - dx)
                    y1 = max(0, cell_center_y - dy)
                    x2 = min(self.win_size[0]-1, cell_center_x + dx)
                    y2 = min(self.win_size[1]-1, cell_center_y + dy)
                    cv2.line(vis, (x1, y1), (x2, y2), 0.5, 1)

        vis = cv2.resize(vis, None, fx=scale, fy=scale)
        vis = (vis * 255).astype(np.uint8)
        return vis

    def compare(self, feat1, feat2, method='cosine'):
        """
        比较两个HOG特征

        Args:
            feat1, feat2: HOG特征
            method: 'cosine', 'euclidean', 'correlation'

        Returns:
            float: 距离值(越小越相似)
        """
        f1 = np.array(feat1, dtype=np.float64)
        f2 = np.array(feat2, dtype=np.float64)

        if method == 'cosine':
            dot = np.dot(f1, f2)
            norm = np.linalg.norm(f1) * np.linalg.norm(f2)
            if norm < 1e-10:
                return 1.0
            return 1.0 - dot / norm
        elif method == 'euclidean':
            return np.sqrt(np.sum((f1 - f2) ** 2))
        elif method == 'correlation':
            if np.std(f1) < 1e-10 or np.std(f2) < 1e-10:
                return 1.0
            corr = np.corrcoef(f1, f2)[0, 1]
            return 1.0 - abs(corr)
        else:
            raise ValueError(f"未知方法: {method}")

    def get_default_detector(self):
        """
        获取默认的HOG行人检测器

        Returns:
            cv2.HOGDescriptor: 配置好的检测器
        """
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        return self.hog

    def detect_pedestrians(self, image, win_stride=(8, 8),
                           padding=(8, 8), scale=1.05, hit_threshold=0):
        """
        检测行人

        Args:
            image: BGR图像
            win_stride: 窗口步长
            padding: 填充
            scale: 尺度缩放因子
            hit_threshold: SVM阈值

        Returns:
            list: 行人边界框 [(x, y, w, h), ...]
            list: 权重值
        """
        self.get_default_detector()
        rects, weights = self.hog.detectMultiScale(
            image, hit_threshold=hit_threshold,
            win_stride=win_stride, padding=padding, scale=scale
        )
        # 转换格式
        boxes = []
        for r in rects:
            boxes.append(tuple(r))
        return boxes, weights.tolist()


# ==================== 便捷函数 ====================

def extract_hog(image, win_size=(64, 128)):
    """
    便捷函数: 提取HOG特征

    Args:
        image: 输入图像
        win_size: 窗口大小

    Returns:
        numpy.ndarray: HOG特征向量
    """
    extractor = HOGFeatureExtractor(win_size=win_size)
    return extractor.extract(image)


def detect_people(image, scale=1.05):
    """
    便捷函数: HOG行人检测

    Args:
        image: BGR图像
        scale: 尺度因子

    Returns:
        list: 边界框列表
    """
    extractor = HOGFeatureExtractor()
    boxes, weights = extractor.detect_pedestrians(image, scale=scale)
    return boxes


# ==================== 测试 ====================

if __name__ == '__main__':
    # 创建测试图像
    img = np.zeros((200, 100, 3), dtype=np.uint8)
    # 画一个简单的人形轮廓
    cv2.rectangle(img, (30, 10), (70, 50), (255, 255, 255), -1)  # 头
    cv2.rectangle(img, (25, 50), (75, 120), (200, 200, 200), -1)  # 身体
    cv2.rectangle(img, (30, 120), (50, 180), (180, 180, 180), -1)  # 左腿
    cv2.rectangle(img, (50, 120), (70, 180), (180, 180, 180), -1)  # 右腿

    extractor = HOGFeatureExtractor(win_size=(64, 128))

    feat = extractor.extract(img)
    print(f"HOG特征维度: {feat.shape}")
    print(f"HOG特征前20维: {feat[:20]}")

    # 比较测试
    feat2 = extractor.extract(img)
    dist = extractor.compare(feat, feat2, method='cosine')
    print(f"相同图像距离: {dist}")
