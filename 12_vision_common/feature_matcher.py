"""
特征匹配模块 - ORB/SIFT/AKAZE + FLANN/暴力匹配
适用于电赛中目标识别、图像拼接、视觉定位等场景

功能:
- ORB/SIFT/AKAZE 特征提取
- BFMatcher / FLANN 特征匹配
- 比率测试 + RANSAC 过滤
- 单应性矩阵估计
- 特征匹配可视化
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict


class FeatureMatcher:
    """特征匹配器"""

    # 支持的特征类型
    DETECTOR_TYPES = ['ORB', 'SIFT', 'AKAZE']

    def __init__(self, detector_type: str = 'ORB', matcher_type: str = 'BF',
                 max_features: int = 1000):
        """
        初始化特征匹配器
        Args:
            detector_type: 特征检测器 'ORB' / 'SIFT' / 'AKAZE'
            matcher_type: 匹配器 'BF' (暴力) / 'FLANN'
            max_features: 最大特征点数
        """
        self.detector_type = detector_type.upper()
        self.matcher_type = matcher_type.upper()
        self.max_features = max_features

        # 创建检测器
        self.detector = self._create_detector()

        # 创建匹配器
        self.matcher = self._create_matcher()

    def _create_detector(self):
        """创建特征检测器"""
        if self.detector_type == 'ORB':
            return cv2.ORB_create(nfeatures=self.max_features)
        elif self.detector_type == 'SIFT':
            return cv2.SIFT_create(nfeatures=self.max_features)
        elif self.detector_type == 'AKAZE':
            return cv2.AKAZE_create()
        else:
            raise ValueError(f"不支持的检测器: {self.detector_type}, 可选: {self.DETECTOR_TYPES}")

    def _create_matcher(self):
        """创建匹配器"""
        if self.matcher_type == 'BF':
            if self.detector_type in ['ORB', 'AKAZE']:
                return cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
            else:  # SIFT
                return cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        elif self.matcher_type == 'FLANN':
            if self.detector_type in ['ORB', 'AKAZE']:
                # 二进制描述子用 LSH
                index_params = dict(algorithm=6, table_number=6,
                                    key_size=12, multi_probe_level=1)
            else:
                # 浮点描述子用 KDTree
                index_params = dict(algorithm=1, trees=5)
            search_params = dict(checks=50)
            return cv2.FlannBasedMatcher(index_params, search_params)
        else:
            raise ValueError(f"不支持的匹配器: {self.matcher_type}, 可选: ['BF', 'FLANN']")

    def detect_and_compute(self, image: np.ndarray) -> Tuple[List, np.ndarray]:
        """
        检测特征点并计算描述子
        Args:
            image: 输入图像 (灰度或彩色)
        Returns:
            (keypoints, descriptors)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        kp, des = self.detector.detectAndCompute(gray, None)
        return kp, des

    def match(self, desc_train: np.ndarray, desc_query: np.ndarray,
              ratio_thresh: float = 0.75) -> List[cv2.DMatch]:
        """
        特征匹配 (含比率测试)
        Args:
            desc_train: 训练描述子 (模板/参考图)
            desc_query: 查询描述子 (待搜索图)
            ratio_thresh: 比率测试阈值 (越小越严格)
        Returns:
            通过比率测试的好匹配
        """
        if desc_train is None or desc_query is None:
            return []
        if len(desc_train) < 2 or len(desc_query) < 2:
            return []

        matches = self.matcher.knnMatch(desc_train, desc_query, k=2)

        # Lowe's 比率测试
        good_matches = []
        for m_pair in matches:
            if len(m_pair) == 2:
                m, n = m_pair
                if m.distance < ratio_thresh * n.distance:
                    good_matches.append(m)

        return good_matches

    def match_image(self, template: np.ndarray, image: np.ndarray,
                    ratio_thresh: float = 0.75,
                    ransac_thresh: float = 5.0,
                    min_matches: int = 10) -> Dict:
        """
        完整的图像匹配流程
        Args:
            template: 模板/参考图像
            image: 搜索图像
            ratio_thresh: 比率测试阈值
            ransac_thresh: RANSAC重投影误差阈值
            min_matches: 最少匹配数
        Returns:
            dict: {
                'homography': 单应性矩阵 (None if failed),
                'good_matches': 好匹配列表,
                'inliers': 内点数,
                'corners': 模板四角在搜索图中的位置,
                'kp_template': 模板关键点,
                'kp_image': 搜索图关键点,
                'success': 是否成功
            }
        """
        kp1, des1 = self.detect_and_compute(template)
        kp2, des2 = self.detect_and_compute(image)

        good_matches = self.match(des1, des2, ratio_thresh)

        result = {
            'homography': None,
            'good_matches': good_matches,
            'inliers': 0,
            'corners': None,
            'kp_template': kp1,
            'kp_image': kp2,
            'des_template': des1,
            'des_image': des2,
            'success': False
        }

        if len(good_matches) < min_matches:
            return result

        # 提取匹配点坐标
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # RANSAC估计单应性矩阵
        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, ransac_thresh)

        if H is not None:
            inliers = int(mask.sum())
            result['homography'] = H
            result['inliers'] = inliers
            result['inlier_mask'] = mask.ravel().tolist()

            # 计算模板四角在搜索图中的位置
            h, w = template.shape[:2]
            corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
            transformed = cv2.perspectiveTransform(corners, H)
            result['corners'] = transformed.reshape(-1, 2)
            result['success'] = True

        return result

    def match_with_filter(self, template: np.ndarray, image: np.ndarray,
                          ratio_thresh: float = 0.7,
                          min_inlier_ratio: float = 0.3) -> Dict:
        """
        更严格的匹配 (适合电赛精确定位)
        """
        return self.match_image(template, image,
                                ratio_thresh=ratio_thresh,
                                ransac_thresh=3.0,
                                min_matches=max(8, int(min_inlier_ratio * 20)))

    def match_and_track(self, template: np.ndarray, image: np.ndarray,
                        roi: Optional[Tuple[int, int, int, int]] = None) -> Dict:
        """
        带ROI限制的匹配 (提高速度和精度)
        Args:
            template: 模板
            image: 当前帧
            roi: (x, y, w, h) 搜索区域, None则全图搜索
        """
        if roi is not None:
            x, y, w, h = roi
            search_img = image[y:y + h, x:x + w]
        else:
            search_img = image
            x, y = 0, 0

        result = self.match_image(template, search_img)

        # 偏移回原图坐标
        if result['success'] and result['corners'] is not None:
            result['corners'][:, 0] += x
            result['corners'][:, 1] += y

        return result

    def match_bf_crosscheck(self, template: np.ndarray, image: np.ndarray) -> List[cv2.DMatch]:
        """
        暴力匹配 + 交叉检验 (更可靠但匹配数少)
        """
        kp1, des1 = self.detect_and_compute(template)
        kp2, des2 = self.detect_and_compute(image)

        if des1 is None or des2 is None:
            return []

        norm = cv2.NORM_HAMMING if self.detector_type in ['ORB', 'AKAZE'] else cv2.NORM_L2
        bf = cv2.BFMatcher(norm, crossCheck=True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)
        return matches

    @staticmethod
    def draw_matches(image1: np.ndarray, kp1: List,
                     image2: np.ndarray, kp2: List,
                     matches: List[cv2.DMatch],
                     inlier_mask: Optional[List] = None,
                     max_draw: int = 50) -> np.ndarray:
        """可视化特征匹配"""
        draw_params = dict(
            matchColor=(0, 255, 0),
            singlePointColor=(0, 0, 255),
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )

        if inlier_mask is not None:
            matches_to_draw = [m for m, flag in zip(matches[:max_draw], inlier_mask[:max_draw]) if flag]
        else:
            matches_to_draw = matches[:max_draw]

        vis = cv2.drawMatches(image1, kp1, image2, kp2,
                              matches_to_draw, None, **draw_params)
        return vis

    @staticmethod
    def draw_homography(image: np.ndarray, corners: np.ndarray,
                        color: Tuple[int, int, int] = (0, 255, 0),
                        thickness: int = 3) -> np.ndarray:
        """在图上绘制单应性变换后的边界"""
        vis = image.copy()
        pts = corners.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(vis, [pts], True, color, thickness)

        # 标注角点
        for i, pt in enumerate(corners):
            cv2.circle(vis, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)
            cv2.putText(vis, str(i), (int(pt[0]) + 5, int(pt[1]) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        return vis


class MultiFeatureMatcher:
    """多特征检测器组合匹配"""

    def __init__(self, detectors: List[str] = None):
        """
        Args:
            detectors: 使用的检测器列表, 如 ['ORB', 'SIFT']
        """
        if detectors is None:
            detectors = ['ORB', 'SIFT']
        self.matchers = {d: FeatureMatcher(d) for d in detectors}

    def match_image(self, template: np.ndarray, image: np.ndarray,
                    strategy: str = 'best') -> Dict:
        """
        多检测器匹配
        Args:
            strategy: 'best' 取最佳, 'voting' 投票
        """
        all_results = {}
        for name, matcher in self.matchers.items():
            result = matcher.match_image(template, image)
            all_results[name] = result

        if strategy == 'best':
            best_name = max(all_results,
                            key=lambda k: len(all_results[k]['good_matches']) if all_results[k]['success'] else 0)
            return all_results[best_name]
        elif strategy == 'voting':
            # 多数投票取一致的匹配
            successful = {k: v for k, v in all_results.items() if v['success']}
            if not successful:
                return {'success': False, 'good_matches': []}
            best = max(successful.values(), key=lambda v: v['inliers'])
            return best

        return all_results


# ==================== 快捷函数 ====================

def match_orb(template: np.ndarray, image: np.ndarray,
              min_matches: int = 10) -> Optional[np.ndarray]:
    """快速ORB匹配, 返回单应性矩阵或None"""
    matcher = FeatureMatcher('ORB')
    result = matcher.match_image(template, image, min_matches=min_matches)
    return result['homography'] if result['success'] else None


def match_sift(template: np.ndarray, image: np.ndarray,
               min_matches: int = 10) -> Optional[np.ndarray]:
    """快速SIFT匹配"""
    matcher = FeatureMatcher('SIFT')
    result = matcher.match_image(template, image, min_matches=min_matches)
    return result['homography'] if result['success'] else None


def locate_object(template: np.ndarray, image: np.ndarray,
                  method: str = 'ORB') -> Optional[Tuple[int, int, int, int]]:
    """定位目标, 返回 (cx, cy, w, h) 或 None"""
    matcher = FeatureMatcher(method)
    result = matcher.match_image(template, image)
    if result['success'] and result['corners'] is not None:
        corners = result['corners']
        cx = int(corners[:, 0].mean())
        cy = int(corners[:, 1].mean())
        w = int(np.linalg.norm(corners[1] - corners[0]))
        h = int(np.linalg.norm(corners[3] - corners[0]))
        return (cx, cy, w, h)
    return None


# ==================== 示例与测试 ====================

if __name__ == '__main__':
    # 创建测试图像
    img1 = np.zeros((300, 400, 3), dtype=np.uint8)
    cv2.rectangle(img1, (50, 50), (200, 200), (255, 255, 255), -1)
    cv2.circle(img1, (125, 125), 30, (0, 0, 0), -1)
    cv2.putText(img1, "TEST", (60, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 3)

    # 模拟旋转+缩放+平移后的图像
    M = cv2.getRotationMatrix2D((200, 150), 15, 0.9)
    M[0, 2] += 50
    M[1, 2] += 30
    img2 = cv2.warpAffine(img1, M, (500, 400))

    for det_type in ['ORB', 'SIFT', 'AKAZE']:
        print(f"\n=== {det_type} 特征匹配 ===")
        matcher = FeatureMatcher(det_type)

        kp1, des1 = matcher.detect_and_compute(img1)
        kp2, des2 = matcher.detect_and_compute(img2)
        print(f"  模板关键点: {len(kp1)}, 搜索图关键点: {len(kp2)}")

        result = matcher.match_image(img1, img2)
        print(f"  好匹配数: {len(result['good_matches'])}")
        print(f"  RANSAC内点: {result['inliers']}")
        print(f"  匹配成功: {result['success']}")

        if result['success']:
            print(f"  目标角点:\n{result['corners']}")

    print("\n特征匹配模块测试完成!")
