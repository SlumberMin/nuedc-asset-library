#!/usr/bin/env python3
"""
特征匹配单元测试
覆盖: 检测器初始化、特征检测、匹配、比率测试、单应性估计、ROI匹配
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from importlib.util import find_spec
_cv2_available = find_spec('cv2') is not None

if _cv2_available:
    import cv2
    from feature_matcher import FeatureMatcher, MultiFeatureMatcher, match_orb


def _make_feature_image():
    """创建含丰富特征的测试图像"""
    img = np.zeros((300, 400, 3), dtype=np.uint8)
    cv2.rectangle(img, (50, 50), (200, 200), (255, 255, 255), -1)
    cv2.circle(img, (125, 125), 30, (0, 0, 0), -1)
    cv2.putText(img, "TEST", (60, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 3)
    cv2.circle(img, (300, 100), 40, (200, 200, 200), -1)
    cv2.rectangle(img, (250, 200), (380, 280), (150, 150, 150), -1)
    return img


def _make_transformed_image(img1):
    """创建经过旋转+平移的图像"""
    M = cv2.getRotationMatrix2D((200, 150), 15, 0.9)
    M[0, 2] += 50
    M[1, 2] += 30
    return cv2.warpAffine(img1, M, (500, 400))


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestFeatureMatcherInit(unittest.TestCase):
    """初始化测试"""

    def test_orb_detector(self):
        """ORB检测器初始化"""
        m = FeatureMatcher('ORB')
        self.assertEqual(m.detector_type, 'ORB')
        self.assertIsNotNone(m.detector)

    def test_sift_detector(self):
        """SIFT检测器初始化"""
        m = FeatureMatcher('SIFT')
        self.assertEqual(m.detector_type, 'SIFT')

    def test_akaze_detector(self):
        """AKAZE检测器初始化"""
        m = FeatureMatcher('AKAZE')
        self.assertEqual(m.detector_type, 'AKAZE')

    def test_invalid_detector_raises(self):
        """无效检测器应抛异常"""
        with self.assertRaises(ValueError):
            FeatureMatcher('INVALID')

    def test_bf_matcher(self):
        """BF匹配器初始化"""
        m = FeatureMatcher('ORB', 'BF')
        self.assertEqual(m.matcher_type, 'BF')

    def test_flann_matcher(self):
        """FLANN匹配器初始化"""
        m = FeatureMatcher('ORB', 'FLANN')
        self.assertEqual(m.matcher_type, 'FLANN')

    def test_invalid_matcher_raises(self):
        """无效匹配器应抛异常"""
        with self.assertRaises(ValueError):
            FeatureMatcher('ORB', 'INVALID')

    def test_max_features(self):
        """最大特征数应正确设置"""
        m = FeatureMatcher('ORB', max_features=500)
        self.assertEqual(m.max_features, 500)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestDetectAndCompute(unittest.TestCase):
    """特征检测测试"""

    def test_orb_detects_keypoints(self):
        """ORB应检测到关键点"""
        img = _make_feature_image()
        m = FeatureMatcher('ORB')
        kp, des = m.detect_and_compute(img)
        self.assertGreater(len(kp), 0)
        self.assertIsNotNone(des)

    def test_sift_detects_keypoints(self):
        """SIFT应检测到关键点"""
        img = _make_feature_image()
        m = FeatureMatcher('SIFT')
        kp, des = m.detect_and_compute(img)
        self.assertGreater(len(kp), 0)

    def test_grayscale_input(self):
        """灰度图应能正常检测"""
        img = cv2.cvtColor(_make_feature_image(), cv2.COLOR_BGR2GRAY)
        m = FeatureMatcher('ORB')
        kp, des = m.detect_and_compute(img)
        self.assertGreater(len(kp), 0)

    def test_empty_image(self):
        """纯黑图像也应返回结果(可能为空)"""
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        m = FeatureMatcher('ORB')
        kp, des = m.detect_and_compute(img)
        # 可能检测到0个关键点，但不应崩溃
        self.assertIsNotNone(kp)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMatch(unittest.TestCase):
    """特征匹配测试"""

    def setUp(self):
        self.img1 = _make_feature_image()
        self.img2 = _make_transformed_image(self.img1)
        self.matcher = FeatureMatcher('ORB')

    def test_match_returns_list(self):
        """match应返回DMatch列表"""
        kp1, des1 = self.matcher.detect_and_compute(self.img1)
        kp2, des2 = self.matcher.detect_and_compute(self.img2)
        matches = self.matcher.match(des1, des2)
        self.assertIsInstance(matches, list)

    def test_match_none_descriptors(self):
        """None描述子应返回空列表"""
        matches = self.matcher.match(None, np.zeros((5, 32), dtype=np.uint8))
        self.assertEqual(matches, [])

    def test_match_short_descriptors(self):
        """少于2个描述子应返回空列表"""
        des = np.zeros((1, 32), dtype=np.uint8)
        matches = self.matcher.match(des, des)
        self.assertEqual(matches, [])

    def test_ratio_thresh_effect(self):
        """更严格的比率阈值应产生更少匹配"""
        kp1, des1 = self.matcher.detect_and_compute(self.img1)
        kp2, des2 = self.matcher.detect_and_compute(self.img2)
        matches_loose = self.matcher.match(des1, des2, ratio_thresh=0.9)
        matches_strict = self.matcher.match(des1, des2, ratio_thresh=0.5)
        self.assertGreaterEqual(len(matches_loose), len(matches_strict))


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMatchImage(unittest.TestCase):
    """完整图像匹配测试"""

    def setUp(self):
        self.img1 = _make_feature_image()
        self.img2 = _make_transformed_image(self.img1)
        self.matcher = FeatureMatcher('ORB')

    def test_match_image_returns_dict(self):
        """应返回包含所有字段的字典"""
        result = self.matcher.match_image(self.img1, self.img2)
        for key in ('homography', 'good_matches', 'inliers', 'success'):
            self.assertIn(key, result)

    def test_success_with_transformed(self):
        """变换后的图像匹配应成功"""
        result = self.matcher.match_image(self.img1, self.img2, min_matches=5)
        self.assertTrue(result['success'])

    def test_homography_shape(self):
        """成功匹配时单应性矩阵应为3x3"""
        result = self.matcher.match_image(self.img1, self.img2, min_matches=5)
        if result['success']:
            self.assertEqual(result['homography'].shape, (3, 3))

    def test_corners_provided_on_success(self):
        """成功匹配时应提供角点"""
        result = self.matcher.match_image(self.img1, self.img2, min_matches=5)
        if result['success']:
            self.assertIsNotNone(result['corners'])
            self.assertEqual(result['corners'].shape, (4, 2))

    def test_min_matches_filter(self):
        """min_matches过大时应失败"""
        result = self.matcher.match_image(self.img1, self.img2, min_matches=10000)
        self.assertFalse(result['success'])


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMatchWithFilter(unittest.TestCase):
    """严格匹配测试"""

    def test_stricter_filter(self):
        """严格匹配应返回结果"""
        img1 = _make_feature_image()
        img2 = _make_transformed_image(img1)
        m = FeatureMatcher('ORB')
        result = m.match_with_filter(img1, img2)
        self.assertIn('success', result)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMatchAndTrack(unittest.TestCase):
    """ROI匹配测试"""

    def test_roi_match(self):
        """ROI限制的匹配应正常工作"""
        img1 = _make_feature_image()
        img2 = _make_transformed_image(img1)
        m = FeatureMatcher('ORB')
        # 使用全图ROI
        result = m.match_and_track(img1, img2, roi=(0, 0, 500, 400))
        self.assertIn('success', result)

    def test_no_roi(self):
        """无ROI应搜索全图"""
        img1 = _make_feature_image()
        img2 = _make_transformed_image(img1)
        m = FeatureMatcher('ORB')
        result = m.match_and_track(img1, img2, roi=None)
        self.assertIn('success', result)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestCrossCheckMatch(unittest.TestCase):
    """交叉检验匹配测试"""

    def test_crosscheck_returns_list(self):
        """交叉检验应返回排序的匹配列表"""
        img1 = _make_feature_image()
        img2 = _make_transformed_image(img1)
        m = FeatureMatcher('ORB')
        matches = m.match_bf_crosscheck(img1, img2)
        self.assertIsInstance(matches, list)
        if len(matches) > 1:
            # 应按distance升序
            self.assertLessEqual(matches[0].distance, matches[1].distance)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMultiFeatureMatcher(unittest.TestCase):
    """多特征匹配器测试"""

    def test_init_default(self):
        """默认应创建ORB和SIFT"""
        m = MultiFeatureMatcher()
        self.assertIn('ORB', m.matchers)
        self.assertIn('SIFT', m.matchers)

    def test_custom_detectors(self):
        """自定义检测器列表"""
        m = MultiFeatureMatcher(['ORB'])
        self.assertEqual(len(m.matchers), 1)

    def test_best_strategy(self):
        """best策略应返回结果"""
        img1 = _make_feature_image()
        img2 = _make_transformed_image(img1)
        m = MultiFeatureMatcher(['ORB'])
        result = m.match_image(img1, img2, strategy='best')
        self.assertIn('success', result)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestDrawFunctions(unittest.TestCase):
    """可视化测试"""

    def test_draw_homography(self):
        """绘制单应性边界"""
        img = _make_feature_image()
        corners = np.float32([[10, 10], [100, 10], [100, 100], [10, 100]])
        vis = FeatureMatcher.draw_homography(img, corners)
        self.assertEqual(vis.shape, img.shape)


@unittest.skipUnless(_cv2_available, 'opencv-python not installed')
class TestMatchOrbShortcut(unittest.TestCase):
    """快捷函数测试"""

    def test_match_orb_success(self):
        """match_orb在成功时应返回矩阵"""
        img1 = _make_feature_image()
        img2 = _make_transformed_image(img1)
        H = match_orb(img1, img2, min_matches=5)
        if H is not None:
            self.assertEqual(H.shape, (3, 3))

    def test_match_orb_failure(self):
        """match_orb在失败时应返回None"""
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        H = match_orb(img, img, min_matches=10000)
        self.assertIsNone(H)


if __name__ == '__main__':
    unittest.main()
