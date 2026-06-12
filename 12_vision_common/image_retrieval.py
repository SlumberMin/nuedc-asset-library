"""
图像检索模块 - 特征提取 + 相似度匹配 + 视觉词袋
=================================================
功能：
  1. 多种图像特征提取（颜色、纹理、形状）
  2. 特征相似度计算（多种距离度量）
  3. 视觉词袋模型（BoVW）
  4. 图像检索与排序

依赖：opencv-python, numpy
"""

import cv2
import numpy as np
import os


class ImageRetriever:
    """
    图像检索器
    支持多种特征描述子和相似度度量
    """

    def __init__(self, feature_type='hybrid'):
        """
        初始化图像检索器

        参数:
            feature_type: 特征类型 'color'/'texture'/'shape'/'orb'/'hybrid'
        """
        self.feature_type = feature_type

        # ORB特征检测器
        self.orb = cv2.ORB_create(nfeatures=500)

        # 特征数据库
        self.feature_db = {}  # {image_id: feature_vector}
        self.image_db = {}    # {image_id: image_path}
        self.next_id = 0

        # 视觉词袋
        self.vocabulary = None
        self.bovw_histograms = {}
        self.vocab_size = 64  # 词典大小

    def extract_color_histogram(self, frame, bins=64):
        """
        提取颜色直方图特征

        参数:
            frame: BGR图像
            bins: 每通道直方图bin数

        返回:
            feature: 颜色直方图特征向量
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 多通道直方图
        hist_h = cv2.calcHist([hsv], [0], None, [bins], [0, 180])
        hist_s = cv2.calcHist([hsv], [1], None, [bins], [0, 256])
        hist_v = cv2.calcHist([hsv], [2], None, [bins], [0, 256])

        # 归一化并拼接
        feature = np.concatenate([
            hist_h.flatten(), hist_s.flatten(), hist_v.flatten()
        ])
        feature = feature / (np.sum(feature) + 1e-10)
        return feature

    def extract_texture_lbp(self, frame, grid_x=4, grid_y=4):
        """
        提取分块LBP纹理特征

        参数:
            frame: BGR图像
            grid_x: 水平分块数
            grid_y: 垂直分块数

        返回:
            feature: LBP纹理特征向量
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        # 计算LBP图
        lbp = self._compute_lbp_fast(gray)

        # 分块统计直方图
        features = []
        bh = h // grid_y
        bw = w // grid_x

        for gy in range(grid_y):
            for gx in range(grid_x):
                y1 = gy * bh
                x1 = gx * bw
                block = lbp[y1:y1 + bh, x1:x1 + bw]
                hist, _ = np.histogram(block, bins=36, range=(0, 256))
                hist = hist / (np.sum(hist) + 1e-10)
                features.append(hist)

        return np.concatenate(features)

    def _compute_lbp_fast(self, gray):
        """快速LBP计算（向量化）"""
        h, w = gray.shape
        lbp = np.zeros((h - 2, w - 2), dtype=np.uint8)
        center = gray[1:-1, 1:-1]

        offsets = [(-1, -1), (-1, 0), (-1, 1), (0, 1),
                   (1, 1), (1, 0), (1, -1), (0, -1)]

        for k, (dy, dx) in enumerate(offsets):
            neighbor = gray[1 + dy:h - 1 + dy, 1 + dx:w - 1 + dx]
            lbp |= ((neighbor >= center) << k).astype(np.uint8)

        return lbp

    def extract_shape_hu(self, frame):
        """
        提取Hu矩形状特征

        参数:
            frame: BGR图像

        返回:
            feature: Hu矩特征向量
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        moments = cv2.moments(binary)
        hu = cv2.HuMoments(moments).flatten()
        # 对数变换
        hu = -np.sign(hu) * np.log10(np.abs(hu) + 1e-10)
        return hu

    def extract_orb_features(self, frame):
        """
        提取ORB关键点和描述子

        参数:
            frame: BGR图像

        返回:
            keypoints: 关键点列表
            descriptors: 描述子数组
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        return keypoints, descriptors

    def extract_hybrid_feature(self, frame):
        """
        混合特征提取（颜色+纹理+形状）

        参数:
            frame: BGR图像

        返回:
            feature: 混合特征向量
        """
        color = self.extract_color_histogram(frame, bins=32)
        texture = self.extract_texture_lbp(frame, grid_x=3, grid_y=3)
        shape = self.extract_shape_hu(frame)

        feature = np.concatenate([color, texture, shape])
        return feature

    def extract_feature(self, frame):
        """
        提取特征（统一接口）

        参数:
            frame: BGR图像

        返回:
            feature: 特征向量
        """
        if self.feature_type == 'color':
            return self.extract_color_histogram(frame)
        elif self.feature_type == 'texture':
            return self.extract_texture_lbp(frame)
        elif self.feature_type == 'shape':
            return self.extract_shape_hu(frame)
        elif self.feature_type == 'orb':
            _, desc = self.extract_orb_features(frame)
            return desc
        else:  # hybrid
            return self.extract_hybrid_feature(frame)

    # ==================== 相似度计算 ====================

    def compute_similarity(self, feat1, feat2, metric='cosine'):
        """
        计算两个特征向量的相似度

        参数:
            feat1, feat2: 特征向量
            metric: 距离度量 'cosine'/'euclidean'/'chi_square'/'bhattacharyya'/'hist_intersect'

        返回:
            similarity: 相似度（越大越相似）
        """
        feat1 = np.array(feat1, dtype=np.float64).flatten()
        feat2 = np.array(feat2, dtype=np.float64).flatten()

        if metric == 'cosine':
            dot = np.dot(feat1, feat2)
            norm = np.linalg.norm(feat1) * np.linalg.norm(feat2) + 1e-10
            return dot / norm

        elif metric == 'euclidean':
            dist = np.linalg.norm(feat1 - feat2)
            return 1.0 / (1.0 + dist)

        elif metric == 'chi_square':
            chi = np.sum((feat1 - feat2) ** 2 / (feat1 + feat2 + 1e-10))
            return 1.0 / (1.0 + chi)

        elif metric == 'bhattacharyya':
            bc = np.sum(np.sqrt(feat1 * feat2))
            return bc

        elif metric == 'hist_intersect':
            return np.sum(np.minimum(feat1, feat2))

        return 0

    def match_orb_features(self, desc1, desc2, ratio_thresh=0.75):
        """
        ORB特征匹配（BFMatcher + 比率测试）

        参数:
            desc1, desc2: ORB描述子
            ratio_thresh: 比率测试阈值

        返回:
            good_matches: 好匹配列表
            match_score: 匹配分数
        """
        if desc1 is None or desc2 is None:
            return [], 0

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        matches = bf.knnMatch(desc1, desc2, k=2)

        # 比率测试
        good_matches = []
        for m_n in matches:
            if len(m_n) == 2:
                m, n = m_n
                if m.distance < ratio_thresh * n.distance:
                    good_matches.append(m)

        match_score = len(good_matches) / max(len(desc1), 1)
        return good_matches, match_score

    # ==================== 视觉词袋 ====================

    def build_vocabulary(self, image_paths, vocab_size=64):
        """
        构建视觉词典（K-Means聚类）

        参数:
            image_paths: 训练图像路径列表
            vocab_size: 词典大小

        返回:
            vocabulary: 视觉词典 (vocab_size x feature_dim)
        """
        self.vocab_size = vocab_size

        # 收集所有描述子
        all_descriptors = []
        for path in image_paths:
            img = cv2.imread(path)
            if img is None:
                continue
            _, desc = self.extract_orb_features(img)
            if desc is not None:
                all_descriptors.append(desc)

        if not all_descriptors:
            print("[错误] 未提取到有效描述子")
            return None

        # 合并所有描述子
        all_desc = np.vstack(all_descriptors).astype(np.float32)

        # K-Means聚类
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.01)
        _, labels, centers = cv2.kmeans(
            all_desc, vocab_size, None, criteria, 10, cv2.KMEANS_PP_CENTERS)

        self.vocabulary = centers
        print(f"视觉词典构建完成：{vocab_size}个词汇，{len(all_desc)}个描述子")
        return centers

    def compute_bovw_histogram(self, frame):
        """
        计算图像的视觉词袋直方图

        参数:
            frame: BGR图像

        返回:
            histogram: BoVW直方图
        """
        if self.vocabulary is None:
            print("[错误] 视觉词典未构建")
            return None

        _, desc = self.extract_orb_features(frame)
        if desc is None:
            return np.zeros(self.vocab_size)

        # 将每个描述子映射到最近的视觉词
        desc_float = desc.astype(np.float32)
        histogram = np.zeros(self.vocab_size, dtype=np.float32)

        for d in desc_float:
            # 计算到每个视觉词的距离
            dists = np.linalg.norm(self.vocabulary - d, axis=1)
            nearest = np.argmin(dists)
            histogram[nearest] += 1

        # 归一化
        histogram = histogram / (np.sum(histogram) + 1e-10)
        return histogram

    # ==================== 索引和检索 ====================

    def add_to_index(self, image_path, image_id=None):
        """
        将图像添加到检索索引

        参数:
            image_path: 图像路径
            image_id: 图像ID（None则自增）

        返回:
            image_id: 分配的ID
        """
        frame = cv2.imread(image_path)
        if frame is None:
            print(f"[警告] 无法读取: {image_path}")
            return None

        if image_id is None:
            image_id = self.next_id
            self.next_id += 1

        # 提取特征
        feature = self.extract_feature(frame)
        self.feature_db[image_id] = feature
        self.image_db[image_id] = image_path

        # 如果有视觉词典，也计算BoVW
        if self.vocabulary is not None:
            self.bovw_histograms[image_id] = self.compute_bovw_histogram(frame)

        return image_id

    def build_index(self, image_dir, extensions=('.jpg', '.png', '.bmp', '.jpeg')):
        """
        从目录构建图像索引

        参数:
            image_dir: 图像目录
            extensions: 支持的扩展名
        """
        count = 0
        for fname in os.listdir(image_dir):
            if any(fname.lower().endswith(ext) for ext in extensions):
                path = os.path.join(image_dir, fname)
                if self.add_to_index(path) is not None:
                    count += 1
        print(f"索引构建完成：{count}张图像")

    def search(self, query_frame, top_k=5, metric='cosine'):
        """
        检索相似图像

        参数:
            query_frame: 查询图像（BGR）
            top_k: 返回前K个结果
            metric: 相似度度量

        返回:
            results: [(image_id, similarity, path), ...] 按相似度降序
        """
        query_feat = self.extract_feature(query_frame)

        # 计算与所有索引图像的相似度
        similarities = []
        for img_id, feat in self.feature_db.items():
            sim = self.compute_similarity(query_feat, feat, metric)
            similarities.append((img_id, sim))

        # 排序
        similarities.sort(key=lambda x: x[1], reverse=True)

        # 返回top_k
        results = []
        for img_id, sim in similarities[:top_k]:
            results.append({
                'image_id': img_id,
                'similarity': sim,
                'path': self.image_db.get(img_id, ""),
            })

        return results

    def search_by_image(self, query_path, top_k=5, metric='cosine'):
        """
        以图搜图

        参数:
            query_path: 查询图像路径
            top_k: 返回前K个结果

        返回:
            results: 检索结果列表
        """
        frame = cv2.imread(query_path)
        if frame is None:
            print(f"无法读取查询图像: {query_path}")
            return []
        return self.search(frame, top_k, metric)

    def visualize_results(self, query_frame, results, max_display=5):
        """
        可视化检索结果

        参数:
            query_frame: 查询图像
            results: search返回的结果
            max_display: 最多显示几张

        返回:
            vis: 拼接的结果可视化图像
        """
        h, w = 200, 200

        # 缩放查询图
        query_resized = cv2.resize(query_frame, (w, h))

        result_images = [query_resized]
        labels = ["Query"]

        for r in results[:max_display]:
            img = cv2.imread(r['path'])
            if img is not None:
                img = cv2.resize(img, (w, h))
                result_images.append(img)
                labels.append(f"Sim:{r['similarity']:.3f}")

        # 水平拼接
        canvas = np.zeros((h + 30, w * len(result_images), 3), dtype=np.uint8)

        for i, (img, label) in enumerate(zip(result_images, labels)):
            x = i * w
            canvas[30:30 + h, x:x + w] = img
            cv2.putText(canvas, label, (x + 5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return canvas


# ==================== 使用示例 ====================
def demo_directory(image_dir):
    """
    目录图像检索演示

    参数:
        image_dir: 图像目录路径
    """
    retriever = ImageRetriever(feature_type='hybrid')

    # 构建索引
    print(f"正在索引目录: {image_dir}")
    retriever.build_index(image_dir)

    if not retriever.feature_db:
        print("目录中未找到图像")
        return

    # 用第一张图作为查询
    first_path = list(retriever.image_db.values())[0]
    query = cv2.imread(first_path)

    print(f"\n查询图像: {first_path}")
    results = retriever.search(query, top_k=5)

    print("检索结果:")
    for i, r in enumerate(results):
        print(f"  {i + 1}. {os.path.basename(r['path'])} "
              f"(相似度: {r['similarity']:.4f})")

    # 可视化
    vis = retriever.visualize_results(query, results)
    cv2.imshow("Image Retrieval Results", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def demo_compare(image_path1, image_path2):
    """比较两张图像的相似度"""
    retriever = ImageRetriever(feature_type='hybrid')

    img1 = cv2.imread(image_path1)
    img2 = cv2.imread(image_path2)

    if img1 is None or img2 is None:
        print("无法读取图像")
        return

    feat1 = retriever.extract_feature(img1)
    feat2 = retriever.extract_feature(img2)

    metrics = ['cosine', 'euclidean', 'chi_square', 'bhattacharyya']
    print(f"比较: {os.path.basename(image_path1)} vs {os.path.basename(image_path2)}")
    for m in metrics:
        sim = retriever.compute_similarity(feat1, feat2, m)
        print(f"  {m}: {sim:.4f}")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        if os.path.isdir(sys.argv[1]):
            demo_directory(sys.argv[1])
        elif len(sys.argv) > 2:
            demo_compare(sys.argv[1], sys.argv[2])
        else:
            demo_directory(os.path.dirname(sys.argv[1]))
    else:
        print("用法:")
        print("  python image_retrieval.py <图像目录>")
        print("  python image_retrieval.py <图像1> <图像2>")
