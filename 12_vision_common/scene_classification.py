"""
场景分类模块 - CNN特征 + 纹理特征 + 场景标签
=================================================
功能：
  1. 颜色特征提取（颜色直方图、颜色矩）
  2. 纹理特征提取（LBP、Gabor滤波器）
  3. 形状特征提取（Hu矩、边缘特征）
  4. 基于特征的场景分类

依赖：opencv-python, numpy
"""

import cv2
import numpy as np


# ==================== 场景类别定义 ====================
SCENE_CATEGORIES = {
    0: "室内-办公室",
    1: "室内-客厅",
    2: "室内-厨房",
    3: "室外-街道",
    4: "室外-公园",
    5: "室外-建筑",
    6: "自然-森林",
    7: "自然-水域",
    8: "交通-公路",
    9: "其他",
}


class SceneClassifier:
    """
    场景分类器
    使用多特征融合进行场景识别
    """

    def __init__(self):
        """初始化场景分类器"""
        # 场景特征模板（预定义的特征描述）
        self.scene_templates = self._build_scene_templates()

    def _build_scene_templates(self):
        """构建场景特征模板"""
        templates = {
            # 室内场景特征：纹理复杂度中等，色彩较均匀
            "indoor": {
                'texture_complexity': (0.2, 0.6),
                'color_uniformity': (0.3, 0.8),
                'edge_density': (0.05, 0.25),
                'dominant_colors': ['white', 'gray', 'brown'],
            },
            # 室外场景特征：纹理复杂度高，色彩丰富
            "outdoor": {
                'texture_complexity': (0.3, 0.8),
                'color_uniformity': (0.1, 0.5),
                'edge_density': (0.1, 0.4),
                'dominant_colors': ['blue', 'green', 'gray'],
            },
            # 自然场景特征：绿色为主，纹理自然
            "nature": {
                'texture_complexity': (0.4, 0.9),
                'color_uniformity': (0.1, 0.4),
                'edge_density': (0.05, 0.3),
                'dominant_colors': ['green', 'brown', 'blue'],
            },
            # 交通场景特征：灰暗色彩，直线边缘多
            "traffic": {
                'texture_complexity': (0.2, 0.5),
                'color_uniformity': (0.2, 0.6),
                'edge_density': (0.15, 0.4),
                'dominant_colors': ['gray', 'black', 'white'],
            },
        }
        return templates

    def extract_color_features(self, frame):
        """
        提取颜色特征

        参数:
            frame: BGR图像

        返回:
            features: 颜色特征字典
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]

        # 1. 颜色直方图（HSV各通道）
        hist_h = cv2.calcHist([hsv], [0], None, [30], [0, 180])
        hist_s = cv2.calcHist([hsv], [1], None, [32], [0, 256])
        hist_v = cv2.calcHist([hsv], [2], None, [32], [0, 256])

        # 归一化
        cv2.normalize(hist_h, hist_h)
        cv2.normalize(hist_s, hist_s)
        cv2.normalize(hist_v, hist_v)

        # 2. 颜色矩（均值、标准差、偏度）
        channels = cv2.split(hsv)
        color_moments = []
        for ch in channels:
            mean = np.mean(ch)
            std = np.std(ch)
            skewness = np.mean((ch - mean) ** 3) / (std ** 3 + 1e-6)
            color_moments.extend([mean, std, skewness])

        # 3. 主要颜色统计
        # 将H通道划分为8个色区
        h_channel = hsv[:, :, 0]
        color_bins = np.histogram(h_channel, bins=8, range=(0, 180))[0]
        color_bins = color_bins / (h * w)

        # 4. 颜色均匀性
        s_channel = hsv[:, :, 1].astype(float)
        color_uniformity = 1.0 - np.std(s_channel) / 128.0

        # 5. 蓝天/绿植/灰暗比例
        sky_ratio = np.sum((h_channel > 90) & (h_channel < 130) &
                          (s_channel > 50)) / (h * w)
        green_ratio = np.sum((h_channel > 35) & (h_channel < 85) &
                            (s_channel > 30)) / (h * w)
        gray_ratio = np.sum(s_channel < 30) / (h * w)

        return {
            'hist_h': hist_h.flatten(),
            'hist_s': hist_s.flatten(),
            'hist_v': hist_v.flatten(),
            'color_moments': np.array(color_moments),
            'color_bins': color_bins,
            'color_uniformity': color_uniformity,
            'sky_ratio': sky_ratio,
            'green_ratio': green_ratio,
            'gray_ratio': gray_ratio,
        }

    def extract_texture_features(self, frame):
        """
        提取纹理特征（LBP + Gabor）

        参数:
            frame: BGR图像

        返回:
            features: 纹理特征字典
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        # 1. LBP特征（局部二值模式）
        lbp = self._compute_lbp(gray)
        lbp_hist = cv2.calcHist([lbp.astype(np.uint8)], [0], None, [256], [0, 256])
        lbp_hist = lbp_hist / (h * w + 1e-6)

        # 纹理复杂度（LBP直方图的熵）
        lbp_prob = lbp_hist.flatten() + 1e-10
        texture_entropy = -np.sum(lbp_prob * np.log2(lbp_prob))

        # 2. Gabor滤波器特征
        gabor_features = self._compute_gabor_features(gray)

        # 3. 边缘特征
        edges = cv2.Canny(gray, 50, 150)
        edge_density = np.count_nonzero(edges) / (h * w)

        # 4. 梯度方向直方图（简化HOG）
        gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        mag = np.sqrt(gx ** 2 + gy ** 2)
        angle = np.arctan2(gy, gx) * 180 / np.pi + 180

        hog_hist = np.histogram(angle, bins=18, range=(0, 360), weights=mag)[0]
        hog_hist = hog_hist / (np.sum(hog_hist) + 1e-6)

        return {
            'lbp_hist': lbp_hist.flatten(),
            'texture_entropy': texture_entropy,
            'gabor_features': gabor_features,
            'edge_density': edge_density,
            'hog_hist': hog_hist,
            'mean_gradient': np.mean(mag),
        }

    def _compute_lbp(self, gray, radius=1, n_points=8):
        """
        计算局部二值模式（LBP）

        参数:
            gray: 灰度图像
            radius: 邻域半径
            n_points: 采样点数

        返回:
            lbp: LBP图像
        """
        h, w = gray.shape
        lbp = np.zeros_like(gray)

        for i in range(radius, h - radius):
            for j in range(radius, w - radius):
                center = gray[i, j]
                code = 0
                # 8邻域采样
                neighbors = [
                    gray[i - 1, j - 1], gray[i - 1, j], gray[i - 1, j + 1],
                    gray[i, j + 1], gray[i + 1, j + 1], gray[i + 1, j],
                    gray[i + 1, j - 1], gray[i, j - 1]
                ]
                for k, neighbor in enumerate(neighbors):
                    if neighbor >= center:
                        code |= (1 << k)
                lbp[i, j] = code

        return lbp

    def _compute_gabor_features(self, gray):
        """
        计算Gabor滤波器特征

        参数:
            gray: 灰度图像

        返回:
            features: Gabor特征向量
        """
        features = []
        # 多尺度多方向Gabor滤波器
        ksize = 31
        for theta in np.arange(0, np.pi, np.pi / 4):  # 4个方向
            for sigma in [3, 5]:  # 2个尺度
                for lambd in [10, 20]:
                    kernel = cv2.getGaborKernel(
                        (ksize, ksize), sigma, theta, lambd, 0.5, 0, ktype=cv2.CV_32F)
                    filtered = cv2.filter2D(gray, cv2.CV_8UC3, kernel)
                    features.append(np.mean(filtered))
                    features.append(np.std(filtered))

        return np.array(features)

    def extract_shape_features(self, frame):
        """
        提取形状特征

        参数:
            frame: BGR图像

        返回:
            features: 形状特征字典
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        # Hu矩（7个不变矩）
        moments = cv2.moments(binary)
        hu_moments = cv2.HuMoments(moments).flatten()

        # 对数变换使数值范围合理
        hu_moments = -np.sign(hu_moments) * np.log10(np.abs(hu_moments) + 1e-10)

        # 直线检测（霍夫变换）
        edges = cv2.Canny(gray, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50,
                                minLineLength=50, maxLineGap=10)
        line_count = len(lines) if lines is not None else 0

        # 轮廓特征
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_count = len(contours)

        return {
            'hu_moments': hu_moments,
            'line_count': line_count,
            'contour_count': contour_count,
        }

    def classify(self, frame):
        """
        场景分类

        参数:
            frame: BGR输入图像

        返回:
            result: 分类结果字典
        """
        # 提取各类特征
        color_feat = self.extract_color_features(frame)
        texture_feat = self.extract_texture_features(frame)
        shape_feat = self.extract_shape_features(frame)

        # 基于规则的场景分类
        scores = {}
        sky = color_feat['sky_ratio']
        green = color_feat['green_ratio']
        gray_ratio = color_feat['gray_ratio']
        edge_d = texture_feat['edge_density']
        entropy = texture_feat['texture_entropy']
        lines = shape_feat['line_count']
        uniformity = color_feat['color_uniformity']

        h, w = frame.shape[:2]
        img_size = h * w

        # 自然场景：高绿色比、中等纹理
        scores['nature'] = (green * 3 + (0.3 < entropy < 5) * 0.5 +
                           (sky > 0.1) * 0.5)

        # 室外-天空场景
        scores['outdoor_sky'] = (sky * 2 + green * 0.5 + (edge_d > 0.05) * 0.3)

        # 室内场景：高均匀性、低天空比
        scores['indoor'] = (uniformity * 0.5 + (sky < 0.05) * 0.5 +
                           (green < 0.1) * 0.3 + (gray_ratio > 0.1) * 0.3)

        # 交通场景：高边缘密度、多直线
        line_ratio = min(lines / 100, 1.0)
        scores['traffic'] = (edge_d * 2 + line_ratio * 0.5 + gray_ratio * 0.5)

        # 取最高分
        best_scene = max(scores, key=scores.get)
        confidence = scores[best_scene] / (sum(scores.values()) + 1e-6)

        # 映射到具体类别
        scene_map = {
            'nature': 6 if green > sky else 7,
            'outdoor_sky': 5 if edge_d > 0.15 else 4,
            'indoor': 0,
            'traffic': 8,
        }
        scene_id = scene_map.get(best_scene, 9)

        return {
            'scene_id': scene_id,
            'scene_name': SCENE_CATEGORIES.get(scene_id, "未知"),
            'scene_type': best_scene,
            'confidence': confidence,
            'scores': scores,
            'color_features': color_feat,
            'texture_features': texture_feat,
            'shape_features': shape_feat,
        }

    def draw_debug(self, frame, result):
        """
        绘制调试可视化

        参数:
            frame: 原始图像
            result: classify返回的结果

        返回:
            vis: 可视化图像
        """
        vis = frame.copy()
        h, w = vis.shape[:2]

        # 显示场景标签
        scene_name = result['scene_name']
        confidence = result['confidence']
        text = f"{scene_name} ({confidence:.2f})"
        cv2.putText(vis, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 显示各维度得分
        y_offset = 60
        for name, score in result['scores'].items():
            bar_width = int(score * 100)
            cv2.rectangle(vis, (10, y_offset), (10 + bar_width, y_offset + 15),
                          (255, 100, 0), -1)
            cv2.putText(vis, f"{name}: {score:.2f}", (120, y_offset + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            y_offset += 20

        # 显示关键特征
        cf = result['color_features']
        tf = result['texture_features']
        info = f"Sky:{cf['sky_ratio']:.2f} Green:{cf['green_ratio']:.2f} Edge:{tf['edge_density']:.2f}"
        cv2.putText(vis, info, (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        return vis


# ==================== 使用示例 ====================
def demo_camera():
    """摄像头场景分类演示"""
    classifier = SceneClassifier()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("场景分类演示 - 按ESC退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        result = classifier.classify(frame)
        vis = classifier.draw_debug(frame, result)

        cv2.imshow("Scene Classification", vis)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


def demo_image(image_path):
    """单张图片场景分类"""
    classifier = SceneClassifier()
    frame = cv2.imread(image_path)

    if frame is None:
        print(f"无法读取图片: {image_path}")
        return

    result = classifier.classify(frame)

    print(f"场景类别: {result['scene_name']}")
    print(f"场景类型: {result['scene_type']}")
    print(f"置信度: {result['confidence']:.3f}")
    print(f"各维度得分: {result['scores']}")

    vis = classifier.draw_debug(frame, result)
    cv2.imshow("Result", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        demo_image(sys.argv[1])
    else:
        demo_camera()
