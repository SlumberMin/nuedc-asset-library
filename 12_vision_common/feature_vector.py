"""
特征向量管理模块 - 特征提取、存储和匹配
适用于电赛中的目标识别、模板匹配和图像检索
"""

import cv2
import numpy as np
import os
import json
import time


class FeatureVectorManager:
    """特征向量管理器"""

    def __init__(self, feature_type='orb', max_features=500):
        """
        初始化

        Args:
            feature_type: 特征类型 'orb', 'sift', 'surf', 'hog', 'lbp', 'color_moment'
            max_features: 最大特征点数(用于orb/sift)
        """
        self.feature_type = feature_type
        self.max_features = max_features
        self.feature_db = {}  # {name: {'features': ..., 'descriptors': ..., 'keypoints': ...}}

        self._init_extractor()

    def _init_extractor(self):
        """初始化底层特征提取器"""
        if self.feature_type == 'orb':
            self.extractor = cv2.ORB_create(nfeatures=self.max_features)
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        elif self.feature_type == 'sift':
            self.extractor = cv2.SIFT_create(nfeatures=self.max_features)
            self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        elif self.feature_type in ('hog', 'lbp', 'color_moment'):
            self.extractor = None
            self.matcher = None
        else:
            raise ValueError(f"不支持的特征类型: {self.feature_type}")

    def extract(self, image, mask=None):
        """
        提取特征

        Args:
            image: 输入图像(BGR)
            mask: 掩码(可选)

        Returns:
            对于orb/sift: (keypoints, descriptors)
            对于hog/lbp/color_moment: numpy.ndarray 特征向量
        """
        if self.feature_type in ('orb', 'sift'):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            kps, descs = self.extractor.detectAndCompute(gray, mask)
            return kps, descs

        elif self.feature_type == 'hog':
            from hog_feature import HOGFeatureExtractor
            hog = HOGFeatureExtractor()
            return hog.extract(image)

        elif self.feature_type == 'lbp':
            from lbp_feature import LBPFeatureExtractor
            lbp = LBPFeatureExtractor()
            return lbp.extract_histogram(image)

        elif self.feature_type == 'color_moment':
            from color_moment import ColorMomentExtractor
            cm = ColorMomentExtractor()
            return cm.extract(image, mask=mask)

        else:
            raise ValueError(f"未知特征类型: {self.feature_type}")

    def register(self, name, image, mask=None):
        """
        注册一个模板/目标的特征

        Args:
            name: 目标名称
            image: 模板图像
            mask: 掩码(可选)

        Returns:
            注册的特征数据
        """
        result = self.extract(image, mask=mask)
        self.feature_db[name] = {
            'result': result,
            'image_size': image.shape[:2],
            'timestamp': time.time()
        }
        return result

    def match(self, image, method='bf', ratio_threshold=0.75, top_k=5):
        """
        将输入图像与已注册的目标匹配

        Args:
            image: 查询图像
            method: 'bf'(暴力匹配), 'flann', 'cosine', 'euclidean'
            ratio_threshold: 比率测试阈值(用于orb/sift)
            top_k: 返回前k个匹配结果

        Returns:
            list: [(name, score, detail), ...] 按匹配度排序
        """
        if len(self.feature_db) == 0:
            return []

        query_result = self.extract(image)
        results = []

        for name, data in self.feature_db.items():
            registered_result = data['result']

            if self.feature_type in ('orb', 'sift'):
                score, detail = self._match_keypoints(
                    query_result, registered_result,
                    method=method, ratio_threshold=ratio_threshold
                )
            else:
                score = self._match_vectors(query_result, registered_result, method=method)
                detail = {'score': score}

            results.append((name, score, detail))

        results.sort(key=lambda x: x[1], reverse=True)
        return results[:top_k]

    def _match_keypoints(self, query_result, registered_result,
                         method='bf', ratio_threshold=0.75):
        """
        关键点特征匹配

        Args:
            query_result: (keypoints, descriptors)
            registered_result: (keypoints, descriptors)

        Returns:
            float: 匹配分数(好的匹配数)
            dict: 匹配详情
        """
        kps_q, descs_q = query_result
        kps_r, descs_r = registered_result

        if descs_q is None or descs_r is None:
            return 0, {'good_matches': 0, 'total_matches': 0}

        if method in ('bf', 'flann'):
            if len(descs_q) < 2 or len(descs_r) < 2:
                return 0, {'good_matches': 0, 'total_matches': 0}

            matches = self.matcher.knnMatch(descs_q, descs_r, k=2)

            good_matches = []
            for m_pair in matches:
                if len(m_pair) == 2:
                    m, n = m_pair
                    if m.distance < ratio_threshold * n.distance:
                        good_matches.append(m)

            score = len(good_matches)
            detail = {
                'good_matches': len(good_matches),
                'total_matches': len(matches),
                'ratio': len(good_matches) / max(len(matches), 1),
                'homography': None
            }

            # 计算单应性矩阵
            if len(good_matches) >= 4:
                src_pts = np.float32([kps_q[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kps_r[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                if M is not None:
                    inliers = mask.ravel().sum()
                    detail['homography'] = M
                    detail['inliers'] = int(inliers)
                    score = inliers

            return score, detail
        else:
            return 0, {}

    def _match_vectors(self, vec1, vec2, method='euclidean'):
        """
        向量特征匹配

        Args:
            vec1, vec2: 特征向量
            method: 'euclidean', 'cosine', 'manhattan'

        Returns:
            float: 相似度得分(越大越相似)
        """
        v1 = np.array(vec1, dtype=np.float64).flatten()
        v2 = np.array(vec2, dtype=np.float64).flatten()

        # 长度对齐
        min_len = min(len(v1), len(v2))
        v1 = v1[:min_len]
        v2 = v2[:min_len]

        if method == 'euclidean':
            dist = np.sqrt(np.sum((v1 - v2) ** 2))
            return 1.0 / (1.0 + dist)
        elif method == 'cosine':
            dot = np.dot(v1, v2)
            norm = np.linalg.norm(v1) * np.linalg.norm(v2)
            if norm < 1e-10:
                return 0.0
            return dot / norm
        elif method == 'manhattan':
            dist = np.sum(np.abs(v1 - v2))
            return 1.0 / (1.0 + dist)
        else:
            raise ValueError(f"未知匹配方法: {method}")

    def match_flann(self, query_result, registered_result, ratio_threshold=0.75):
        """FLANN匹配(适用于大规模特征)"""
        kps_q, descs_q = query_result
        kps_r, descs_r = registered_result

        if descs_q is None or descs_r is None:
            return 0, {}

        if self.feature_type == 'sift':
            index_params = dict(algorithm=1, trees=5)
            search_params = dict(checks=50)
        else:
            index_params = dict(algorithm=6, table_number=6,
                                key_size=12, multi_probe_level=2)
            search_params = dict(checks=50)

        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(descs_q, descs_r, k=2)

        good = []
        for m_pair in matches:
            if len(m_pair) == 2:
                m, n = m_pair
                if m.distance < ratio_threshold * n.distance:
                    good.append(m)

        return len(good), {'good_matches': len(good), 'total_matches': len(matches)}

    def save(self, filepath):
        """
        保存特征库到文件

        Args:
            filepath: 保存路径(.npz或.json)
        """
        save_data = {}
        for name, data in self.feature_db.items():
            result = data['result']
            if self.feature_type in ('orb', 'sift'):
                kps, descs = result
                kp_data = [(kp.pt, kp.size, kp.angle, kp.response,
                            kp.octave, kp.class_id) for kp in kps]
                save_data[name] = {
                    'keypoints': kp_data,
                    'descriptors': descs if descs is not None else np.array([]),
                    'image_size': data['image_size']
                }
            else:
                save_data[name] = {
                    'vector': result,
                    'image_size': data['image_size']
                }

        np.savez(filepath, **{k: json.dumps(v, default=str) for k, v in save_data.items()})

    def load(self, filepath):
        """
        从文件加载特征库

        Args:
            filepath: .npz文件路径
        """
        data = np.load(filepath, allow_pickle=True)
        for key in data.files:
            raw = json.loads(data[key])
            self.feature_db[key] = {
                'result': raw,
                'image_size': raw.get('image_size', (0, 0)),
                'timestamp': time.time()
            }

    def get_info(self):
        """获取特征库信息"""
        info = {
            'feature_type': self.feature_type,
            'registered_count': len(self.feature_db),
            'registered_names': list(self.feature_db.keys()),
        }
        return info

    def remove(self, name):
        """移除已注册的目标"""
        if name in self.feature_db:
            del self.feature_db[name]

    def clear(self):
        """清空特征库"""
        self.feature_db.clear()


# ==================== 便捷函数 ====================

def quick_match(query_image, template_images, feature_type='orb'):
    """
    便捷函数: 快速匹配查询图像与多个模板

    Args:
        query_image: 查询图像
        template_images: {name: image} 字典
        feature_type: 特征类型

    Returns:
        list: [(name, score, detail), ...]
    """
    manager = FeatureVectorManager(feature_type=feature_type)
    for name, img in template_images.items():
        manager.register(name, img)
    return manager.match(query_image)


def extract_feature_vector(image, feature_type='orb', max_features=500):
    """
    便捷函数: 提取特征向量

    Args:
        image: 输入图像
        feature_type: 特征类型
        max_features: 最大特征点数

    Returns:
        特征数据
    """
    manager = FeatureVectorManager(feature_type=feature_type,
                                   max_features=max_features)
    return manager.extract(image)


# ==================== 测试 ====================

if __name__ == '__main__':
    # 创建测试图像
    template = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.rectangle(template, (20, 20), (80, 80), (0, 255, 0), -1)
    cv2.circle(template, (50, 50), 20, (0, 0, 255), -1)

    query = np.zeros((120, 120, 3), dtype=np.uint8)
    cv2.rectangle(query, (30, 30), (90, 90), (0, 255, 0), -1)
    cv2.circle(query, (60, 60), 20, (0, 0, 255), -1)

    # 测试ORB特征管理
    print("=== ORB特征管理 ===")
    manager = FeatureVectorManager(feature_type='orb', max_features=200)
    manager.register('target_A', template)

    results = manager.match(query)
    for name, score, detail in results:
        print(f"  {name}: score={score}, detail={detail}")

    print(f"\n特征库信息: {manager.get_info()}")

    # 测试HOG特征管理
    print("\n=== HOG特征管理 ===")
    manager_hog = FeatureVectorManager(feature_type='hog')
    manager_hog.register('target_B', template)
    results_hog = manager_hog.match(query, method='euclidean')
    for name, score, detail in results_hog:
        print(f"  {name}: score={score:.4f}")

    # 测试颜色矩特征管理
    print("\n=== 颜色矩特征管理 ===")
    manager_cm = FeatureVectorManager(feature_type='color_moment')
    manager_cm.register('red', template)
    results_cm = manager_cm.match(query, method='cosine')
    for name, score, detail in results_cm:
        print(f"  {name}: score={score:.4f}")
