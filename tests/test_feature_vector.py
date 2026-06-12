#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
特征向量管理单元测试
覆盖: FeatureVectorManager初始化、ORB特征提取与注册、
      向量匹配、特征库管理(注册/移除/清空)、信息查询、便捷函数
注意: feature_vector 内部依赖 hog_feature / lbp_feature / color_moment,
      如果链式导入失败则自动跳过对应测试
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np

# 尝试导入，失败则标记跳过
_import_ok = False
_import_err = ''
try:
    from _10_视觉通用代码库.feature_vector import (
        FeatureVectorManager,
        extract_feature_vector,
    )
    _import_ok = True
except Exception as e:
    _import_err = str(e)

_skip_reason = f'feature_vector 导入失败: {_import_err}' if not _import_ok else ''


def make_template(size=100):
    """创建模板图像"""
    img = np.zeros((size, size, 3), dtype=np.uint8)
    cv2.rectangle(img, (20, 20), (80, 80), (0, 255, 0), -1)
    cv2.circle(img, (50, 50), 20, (0, 0, 255), -1)
    return img


def make_query(size=120):
    """创建查询图像"""
    img = np.zeros((size, size, 3), dtype=np.uint8)
    cv2.rectangle(img, (30, 30), (90, 90), (0, 255, 0), -1)
    cv2.circle(img, (60, 60), 20, (0, 0, 255), -1)
    return img


@unittest.skipIf(not _import_ok, _skip_reason)
class TestFeatureVectorManagerInit(unittest.TestCase):
    """初始化测试"""

    def test_orb_type(self):
        mgr = FeatureVectorManager(feature_type='orb')
        self.assertEqual(mgr.feature_type, 'orb')
        self.assertIsNotNone(mgr.extractor)

    def test_sift_type(self):
        mgr = FeatureVectorManager(feature_type='sift')
        self.assertEqual(mgr.feature_type, 'sift')
        self.assertIsNotNone(mgr.extractor)

    def test_hog_type(self):
        mgr = FeatureVectorManager(feature_type='hog')
        self.assertEqual(mgr.feature_type, 'hog')

    def test_lbp_type(self):
        mgr = FeatureVectorManager(feature_type='lbp')
        self.assertEqual(mgr.feature_type, 'lbp')

    def test_color_moment_type(self):
        mgr = FeatureVectorManager(feature_type='color_moment')
        self.assertEqual(mgr.feature_type, 'color_moment')

    def test_invalid_type_raises(self):
        with self.assertRaises(ValueError):
            FeatureVectorManager(feature_type='invalid')

    def test_max_features(self):
        mgr = FeatureVectorManager(feature_type='orb', max_features=100)
        self.assertEqual(mgr.max_features, 100)

    def test_empty_feature_db(self):
        mgr = FeatureVectorManager(feature_type='orb')
        self.assertEqual(len(mgr.feature_db), 0)


@unittest.skipIf(not _import_ok, _skip_reason)
class TestORBExtract(unittest.TestCase):
    """ORB特征提取测试"""

    def test_returns_keypoints_and_descriptors(self):
        mgr = FeatureVectorManager(feature_type='orb', max_features=100)
        img = make_template()
        kps, descs = mgr.extract(img)
        self.assertIsInstance(kps, (list, tuple))
        self.assertIsNotNone(descs)

    def test_max_features_limit(self):
        mgr = FeatureVectorManager(feature_type='orb', max_features=50)
        img = make_template()
        kps, descs = mgr.extract(img)
        self.assertLessEqual(len(kps), 50)

    def test_descriptor_shape(self):
        mgr = FeatureVectorManager(feature_type='orb', max_features=100)
        img = make_template()
        kps, descs = mgr.extract(img)
        if descs is not None:
            self.assertEqual(descs.shape[1], 32)  # ORB描述子32维


@unittest.skipIf(not _import_ok, _skip_reason)
class TestSIFTExtract(unittest.TestCase):
    """SIFT特征提取测试"""

    def test_returns_keypoints_and_descriptors(self):
        mgr = FeatureVectorManager(feature_type='sift', max_features=100)
        img = make_template()
        kps, descs = mgr.extract(img)
        self.assertIsNotNone(descs)

    def test_descriptor_dim(self):
        mgr = FeatureVectorManager(feature_type='sift', max_features=100)
        img = make_template()
        kps, descs = mgr.extract(img)
        if descs is not None and len(descs) > 0:
            self.assertEqual(descs.shape[1], 128)  # SIFT描述子128维


@unittest.skipIf(not _import_ok, _skip_reason)
class TestRegisterAndMatch(unittest.TestCase):
    """注册与匹配测试"""

    def test_register_adds_to_db(self):
        mgr = FeatureVectorManager(feature_type='orb', max_features=100)
        img = make_template()
        mgr.register('target_A', img)
        self.assertIn('target_A', mgr.feature_db)

    def test_match_returns_results(self):
        mgr = FeatureVectorManager(feature_type='orb', max_features=100)
        template = make_template()
        query = make_query()
        mgr.register('target_A', template)
        results = mgr.match(query)
        self.assertIsInstance(results, list)

    def test_match_empty_db(self):
        """空数据库应返回空列表"""
        mgr = FeatureVectorManager(feature_type='orb', max_features=100)
        query = make_query()
        results = mgr.match(query)
        self.assertEqual(len(results), 0)

    def test_match_multiple_targets(self):
        mgr = FeatureVectorManager(feature_type='orb', max_features=100)
        template = make_template()
        query = make_query()
        mgr.register('target_A', template)
        mgr.register('target_B', template)
        results = mgr.match(query, top_k=2)
        self.assertEqual(len(results), 2)

    def test_match_result_format(self):
        mgr = FeatureVectorManager(feature_type='orb', max_features=100)
        template = make_template()
        query = make_query()
        mgr.register('target_A', template)
        results = mgr.match(query)
        if len(results) > 0:
            name, score, detail = results[0]
            self.assertIsInstance(name, str)
            self.assertIsInstance(score, (int, float))
            self.assertIsInstance(detail, dict)

    def test_same_image_high_score(self):
        """相同图像应有高匹配分"""
        mgr = FeatureVectorManager(feature_type='orb', max_features=200)
        img = make_template()
        mgr.register('target', img)
        results = mgr.match(img)
        if len(results) > 0:
            _, score, _ = results[0]
            self.assertGreater(score, 5)


@unittest.skipIf(not _import_ok, _skip_reason)
class TestVectorMatch(unittest.TestCase):
    """向量匹配测试"""

    def test_match_vectors_euclidean(self):
        mgr = FeatureVectorManager(feature_type='color_moment')
        v1 = np.array([1.0, 2.0, 3.0])
        v2 = np.array([1.0, 2.0, 3.0])
        score = mgr._match_vectors(v1, v2, method='euclidean')
        self.assertAlmostEqual(score, 1.0, delta=0.01)

    def test_match_vectors_cosine(self):
        mgr = FeatureVectorManager(feature_type='color_moment')
        v1 = np.array([1.0, 0.0, 0.0])
        v2 = np.array([1.0, 0.0, 0.0])
        score = mgr._match_vectors(v1, v2, method='cosine')
        self.assertAlmostEqual(score, 1.0, delta=0.01)

    def test_match_vectors_manhattan(self):
        mgr = FeatureVectorManager(feature_type='color_moment')
        v1 = np.array([1.0, 2.0, 3.0])
        v2 = np.array([1.0, 2.0, 3.0])
        score = mgr._match_vectors(v1, v2, method='manhattan')
        self.assertAlmostEqual(score, 1.0, delta=0.01)

    def test_different_vectors_lower_score(self):
        mgr = FeatureVectorManager(feature_type='color_moment')
        v1 = np.array([1.0, 0.0, 0.0])
        v2 = np.array([0.0, 0.0, 1.0])
        score = mgr._match_vectors(v1, v2, method='cosine')
        self.assertLess(score, 1.0)

    def test_unknown_method_raises(self):
        mgr = FeatureVectorManager(feature_type='color_moment')
        with self.assertRaises(ValueError):
            mgr._match_vectors(np.array([1.0]), np.array([1.0]), method='unknown')


@unittest.skipIf(not _import_ok, _skip_reason)
class TestFeatureDBManagement(unittest.TestCase):
    """特征库管理测试"""

    def test_remove(self):
        mgr = FeatureVectorManager(feature_type='orb', max_features=100)
        mgr.register('target', make_template())
        self.assertIn('target', mgr.feature_db)
        mgr.remove('target')
        self.assertNotIn('target', mgr.feature_db)

    def test_remove_nonexistent(self):
        """移除不存在的目标不应报错"""
        mgr = FeatureVectorManager(feature_type='orb', max_features=100)
        mgr.remove('nonexistent')

    def test_clear(self):
        mgr = FeatureVectorManager(feature_type='orb', max_features=100)
        mgr.register('a', make_template())
        mgr.register('b', make_template())
        mgr.clear()
        self.assertEqual(len(mgr.feature_db), 0)

    def test_get_info(self):
        mgr = FeatureVectorManager(feature_type='orb', max_features=100)
        mgr.register('target', make_template())
        info = mgr.get_info()
        self.assertEqual(info['feature_type'], 'orb')
        self.assertEqual(info['registered_count'], 1)
        self.assertIn('target', info['registered_names'])


@unittest.skipIf(not _import_ok, _skip_reason)
class TestConvenienceFunctions(unittest.TestCase):
    """便捷函数测试"""

    def test_extract_feature_vector_orb(self):
        img = make_template()
        kps, descs = extract_feature_vector(img, feature_type='orb', max_features=100)
        self.assertIsNotNone(kps)
        self.assertIsNotNone(descs)

    def test_extract_feature_vector_color_moment(self):
        img = make_template()
        feat = extract_feature_vector(img, feature_type='color_moment')
        self.assertIsNotNone(feat)


if __name__ == '__main__':
    unittest.main()
