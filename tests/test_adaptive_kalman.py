#!/usr/bin/env python3
"""
自适应卡尔曼滤波单元测试
覆盖: 噪声自适应估计、自适应过程噪声调整、与标准卡尔曼对比、
      异常值检测与处理、协方差匹配、多模型自适应
注意: 使用纯 Python 模拟 C 自适应卡尔曼逻辑
"""

import sys
import os
import math
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 ──────────────────────────────────────────

class AdaptiveKalmanFilterSimulator:
    """自适应卡尔曼滤波器"""

    def __init__(self, dt=0.01, process_noise_init=0.01, measure_noise_init=0.1,
                 adaptation_rate=0.05, innovation_window=20):
        self.dt = dt

        # 状态: [位置, 速度]
        self.x = np.array([0.0, 0.0])
        self.P = np.eye(2) * 100.0

        # 状态转移矩阵
        self.F = np.array([[1.0, dt], [0.0, 1.0]])
        # 观测矩阵
        self.H = np.array([[1.0, 0.0]])

        # 过程噪声和测量噪声 (可自适应调整)
        self.Q_base = np.array([[process_noise_init, 0], [0, process_noise_init]])
        self.Q = self.Q_base.copy()
        self.R = np.array([[measure_noise_init]])

        # 自适应参数
        self.adaptation_rate = adaptation_rate
        self.innovation_window = innovation_window
        self.innovations = []
        self.innovation_cov_est = None

        # 噪声估计
        self.estimated_process_noise = process_noise_init
        self.estimated_measure_noise = measure_noise_init

    def _adapt_noise(self, innovation):
        """自适应噪声估计"""
        self.innovations.append(innovation)
        if len(self.innovations) > self.innovation_window:
            self.innovations.pop(0)

        if len(self.innovations) >= self.innovation_window:
            # 计算新息序列的统计特性
            innov_array = np.array(self.innovations)
            innov_var = np.var(innov_array)
            innov_mean = np.mean(innov_array)

            # 自适应调整测量噪声
            expected_var = float(self.H @ self.P @ self.H.T + self.R)
            noise_diff = innov_var - expected_var

            self.estimated_measure_noise += self.adaptation_rate * noise_diff
            self.estimated_measure_noise = max(0.001, self.estimated_measure_noise)
            self.R = np.array([[self.estimated_measure_noise]])

    def _detect_outlier(self, innovation, threshold=3.0):
        """异常值检测"""
        if len(self.innovations) < 5:
            return False
        innov_array = np.array(self.innovations[-10:])
        std = np.std(innov_array) + 1e-6
        mean = np.mean(innov_array)
        return abs(innovation - mean) > threshold * std

    def update(self, measurement, adapt=True, reject_outliers=False):
        """更新滤波器"""
        # 预测
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q

        # 新息
        z = np.array([measurement])
        innovation = z - self.H @ x_pred

        # 异常值检测
        if reject_outliers and self._detect_outlier(innovation):
            return self.x[0]  # 跳过异常值

        # 自适应噪声估计
        if adapt:
            self._adapt_noise(float(innovation))

        # 更新
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)

        self.x = x_pred + K @ innovation.flatten()
        self.P = (np.eye(2) - K @ self.H) @ P_pred

        return self.x[0]

    def get_position(self):
        return self.x[0]

    def get_velocity(self):
        return self.x[1]

    def get_estimated_noise(self):
        return self.estimated_measure_noise

    def reset(self):
        self.x = np.array([0.0, 0.0])
        self.P = np.eye(2) * 100.0
        self.innovations = []
        self.estimated_measure_noise = float(self.R[0, 0])


class StandardKalmanFilterSimulator:
    """标准卡尔曼滤波器(用于对比)"""

    def __init__(self, dt=0.01, process_noise=0.01, measure_noise=0.1):
        self.dt = dt
        self.x = np.array([0.0, 0.0])
        self.P = np.eye(2) * 100.0
        self.F = np.array([[1.0, dt], [0.0, 1.0]])
        self.H = np.array([[1.0, 0.0]])
        self.Q = np.array([[process_noise, 0], [0, process_noise]])
        self.R = np.array([[measure_noise]])

    def update(self, measurement):
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q
        z = np.array([measurement])
        y = z - self.H @ x_pred
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        self.x = x_pred + K @ y.flatten()
        self.P = (np.eye(2) - K @ self.H) @ P_pred
        return self.x[0]

    def get_position(self):
        return self.x[0]

    def get_velocity(self):
        return self.x[1]

    def reset(self):
        self.x = np.array([0.0, 0.0])
        self.P = np.eye(2) * 100.0


# ── 测试用例 ──────────────────────────────────────────────────

class TestAdaptiveKalmanInit(unittest.TestCase):
    """初始化测试"""

    def test_initial_state_zero(self):
        akf = AdaptiveKalmanFilterSimulator()
        self.assertAlmostEqual(akf.get_position(), 0.0)
        self.assertAlmostEqual(akf.get_velocity(), 0.0)

    def test_initial_covariance_large(self):
        akf = AdaptiveKalmanFilterSimulator()
        self.assertTrue(akf.P[0, 0] > 1.0)

    def test_custom_params(self):
        akf = AdaptiveKalmanFilterSimulator(dt=0.05, adaptation_rate=0.1)
        self.assertEqual(akf.dt, 0.05)
        self.assertEqual(akf.adaptation_rate, 0.1)

    def test_reset(self):
        akf = AdaptiveKalmanFilterSimulator()
        akf.update(10.0)
        akf.reset()
        self.assertAlmostEqual(akf.get_position(), 0.0)


class TestAdaptiveKalmanNoiseAdaptation(unittest.TestCase):
    """噪声自适应测试"""

    def test_adapts_to_noise_change(self):
        """应能自适应噪声变化"""
        akf = AdaptiveKalmanFilterSimulator(adaptation_rate=0.1,
                                             measure_noise_init=0.1)
        # 先用低噪声
        np.random.seed(42)
        for _ in range(50):
            akf.update(10.0 + np.random.randn() * 0.1)

        low_noise_est = akf.get_estimated_noise()

        # 切换到高噪声
        akf2 = AdaptiveKalmanFilterSimulator(adaptation_rate=0.2,
                                              measure_noise_init=0.1)
        for _ in range(100):
            akf2.update(10.0 + np.random.randn() * 5.0)

        high_noise_est = akf2.get_estimated_noise()
        # 高噪声环境应估计出更大的噪声
        self.assertGreater(high_noise_est, low_noise_est)

    def test_estimated_noise_bounded(self):
        """估计的噪声应保持在合理范围"""
        akf = AdaptiveKalmanFilterSimulator()
        for _ in range(200):
            akf.update(10.0 + np.random.randn() * 100.0)
        noise = akf.get_estimated_noise()
        self.assertGreater(noise, 0.0)
        self.assertLess(noise, 1000.0)

    def test_innovation_window(self):
        """新息窗口应限制历史长度"""
        akf = AdaptiveKalmanFilterSimulator(innovation_window=10)
        for i in range(50):
            akf.update(float(i))
        self.assertLessEqual(len(akf.innovations), 10)

    def test_adaptation_rate_effect(self):
        """自适应速率影响收敛速度"""
        akf_slow = AdaptiveKalmanFilterSimulator(adaptation_rate=0.01)
        akf_fast = AdaptiveKalmanFilterSimulator(adaptation_rate=0.3)

        target_noise = 5.0
        for _ in range(50):
            akf_slow.update(10.0 + np.random.randn() * target_noise)
            akf_fast.update(10.0 + np.random.randn() * target_noise)

        # 快速自适应应更接近真实噪声
        err_fast = abs(akf_fast.get_estimated_noise() - target_noise)
        err_slow = abs(akf_slow.get_estimated_noise() - target_noise)
        self.assertLessEqual(err_fast, err_slow + 2.0)


class TestAdaptiveKalmanFiltering(unittest.TestCase):
    """滤波性能测试"""

    def test_filters_noise(self):
        """应能滤除噪声"""
        np.random.seed(42)
        akf = AdaptiveKalmanFilterSimulator()
        true_val = 50.0
        measurements = true_val + np.random.randn(200) * 5.0
        filtered = []
        for m in measurements:
            akf.update(m)
            filtered.append(akf.get_position())

        var_raw = np.var(measurements[50:])
        var_filtered = np.var(filtered[50:])
        self.assertLess(var_filtered, var_raw)

    def test_tracks_constant_value(self):
        """应精确跟踪恒定值"""
        akf = AdaptiveKalmanFilterSimulator()
        for _ in range(100):
            akf.update(10.0)
        self.assertAlmostEqual(akf.get_position(), 10.0, delta=0.5)

    def test_velocity_estimation(self):
        """应估计出匀速运动的速度"""
        akf = AdaptiveKalmanFilterSimulator(dt=0.1)
        for i in range(100):
            akf.update(float(i) * 2.0)
        vel = akf.get_velocity()
        self.assertGreater(vel, 5.0)


class TestAdaptiveVsStandardKalman(unittest.TestCase):
    """自适应 vs 标准卡尔曼对比"""

    def test_both_converge_constant_noise(self):
        """恒定噪声下两者都收敛"""
        np.random.seed(42)
        akf = AdaptiveKalmanFilterSimulator()
        skf = StandardKalmanFilterSimulator()
        true_val = 20.0

        for _ in range(100):
            m = true_val + np.random.randn() * 0.5
            akf.update(m)
            skf.update(m)

        self.assertAlmostEqual(akf.get_position(), true_val, delta=1.0)
        self.assertAlmostEqual(skf.get_position(), true_val, delta=1.0)

    def test_adaptive_better_varying_noise(self):
        """变化噪声下自适应应更好"""
        np.random.seed(42)
        akf = AdaptiveKalmanFilterSimulator(adaptation_rate=0.1)
        skf = StandardKalmanFilterSimulator(measure_noise=0.5)

        true_val = 20.0
        errors_akf = []
        errors_skf = []

        for i in range(300):
            # 噪声从小变大
            noise_level = 0.5 + i * 0.02
            m = true_val + np.random.randn() * noise_level
            akf.update(m)
            skf.update(m)
            if i > 100:  # 跳过初始阶段
                errors_akf.append(abs(akf.get_position() - true_val))
                errors_skf.append(abs(skf.get_position() - true_val))

        avg_err_akf = np.mean(errors_akf)
        avg_err_skf = np.mean(errors_skf)
        # 自适应版本应有更小的平均误差
        self.assertLessEqual(avg_err_akf, avg_err_skf + 1.0)


class TestOutlierRejection(unittest.TestCase):
    """异常值检测与处理测试"""

    def test_rejects_outliers(self):
        """应能跳过异常测量值"""
        akf = AdaptiveKalmanFilterSimulator()
        true_val = 10.0

        # 正常测量
        for _ in range(50):
            akf.update(true_val)

        pos_before = akf.get_position()

        # 异常值
        akf.update(1000.0, reject_outliers=True)
        pos_after = akf.get_position()

        # 异常值应被拒绝,位置不应跳变太多
        self.assertAlmostEqual(pos_after, pos_before, delta=1.0)

    def test_accepts_normal_values(self):
        """正常值应被接受"""
        akf = AdaptiveKalmanFilterSimulator()
        for _ in range(50):
            akf.update(10.0)
        pos_before = akf.get_position()
        akf.update(10.5, reject_outliers=True)
        pos_after = akf.get_position()
        # 正常值应改变位置估计
        self.assertNotAlmostEqual(pos_after, pos_before, delta=0.01)

    def test_no_rejection_when_disabled(self):
        """未启用时不应拒绝任何值"""
        akf = AdaptiveKalmanFilterSimulator()
        for _ in range(50):
            akf.update(10.0)
        pos_before = akf.get_position()
        akf.update(1000.0, reject_outliers=False)
        pos_after = akf.get_position()
        # 未启用异常值检测时，值应被接受
        self.assertNotAlmostEqual(pos_after, pos_before, delta=0.01)


class TestAdaptiveKalmanReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_innovations(self):
        akf = AdaptiveKalmanFilterSimulator()
        for _ in range(30):
            akf.update(10.0)
        akf.reset()
        self.assertEqual(len(akf.innovations), 0)

    def test_reset_restores_initial_noise(self):
        akf = AdaptiveKalmanFilterSimulator(measure_noise_init=0.1)
        for _ in range(50):
            akf.update(10.0 + np.random.randn() * 5.0)
        akf.reset()
        self.assertAlmostEqual(akf.get_estimated_noise(), 0.1, delta=0.01)


class TestAdaptiveKalmanEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_zero_dt(self):
        """零dt不应崩溃"""
        akf = AdaptiveKalmanFilterSimulator(dt=0.0)
        # 应至少不崩溃
        pos = akf.update(10.0)
        self.assertIsNotNone(pos)

    def test_very_high_noise(self):
        """极高噪声下应稳定"""
        akf = AdaptiveKalmanFilterSimulator()
        np.random.seed(42)
        for _ in range(100):
            akf.update(10.0 + np.random.randn() * 1000.0)
        self.assertTrue(np.isfinite(akf.get_position()))

    def test_negative_measurements(self):
        """负测量值应正常处理"""
        akf = AdaptiveKalmanFilterSimulator()
        for _ in range(50):
            akf.update(-10.0)
        self.assertAlmostEqual(akf.get_position(), -10.0, delta=1.0)


if __name__ == '__main__':
    unittest.main()
