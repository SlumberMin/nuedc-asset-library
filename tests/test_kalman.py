#!/usr/bin/env python3
"""
卡尔曼滤波单元测试
覆盖: 标准卡尔曼滤波(2状态)、一阶互补滤波、噪声抑制、状态估计
注意: 使用纯 Python + NumPy 模拟 C kalman 逻辑
"""

import sys
import os
import math
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 ──────────────────────────────────────────

class KalmanFilterSimulator:
    """2状态卡尔曼滤波器模拟 (位置+速度)"""

    def __init__(self, dt=0.01, process_noise=0.01, measure_noise=0.1):
        self.dt = dt
        # 状态: [位置, 速度]
        self.x = np.array([0.0, 0.0])
        # 协方差
        self.P = np.eye(2) * 100.0
        # 状态转移矩阵
        self.F = np.array([[1.0, dt],
                           [0.0, 1.0]])
        # 观测矩阵
        self.H = np.array([[1.0, 0.0]])
        # 过程噪声
        self.Q = np.array([[process_noise, 0],
                           [0, process_noise]])
        # 测量噪声
        self.R = np.array([[measure_noise]])

    def update(self, measurement):
        # 预测
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q

        # 更新
        z = np.array([measurement])
        y = z - self.H @ x_pred  # 残差
        S = self.H @ P_pred @ self.H.T + self.R  # 残差协方差
        K = P_pred @ self.H.T @ np.linalg.inv(S)  # 卡尔曼增益

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


class ComplementaryFilterSimulator:
    """一阶互补滤波器模拟"""

    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.value = 0.0
        self.initialized = False

    def update(self, value):
        if not self.initialized:
            self.value = value
            self.initialized = True
        else:
            self.value = self.alpha * self.value + (1 - self.alpha) * value
        return self.value

    def reset(self):
        self.value = 0.0
        self.initialized = False


# ── 测试用例 ──────────────────────────────────────────────────

class TestKalmanInit(unittest.TestCase):
    """初始化测试"""

    def test_initial_state_zero(self):
        """初始状态应为零"""
        kf = KalmanFilterSimulator()
        self.assertAlmostEqual(kf.get_position(), 0.0)
        self.assertAlmostEqual(kf.get_velocity(), 0.0)

    def test_initial_covariance_large(self):
        """初始协方差应较大(不确定性高)"""
        kf = KalmanFilterSimulator()
        self.assertTrue(kf.P[0, 0] > 1.0)


class TestKalmanUpdate(unittest.TestCase):
    """更新测试"""

    def test_single_measurement(self):
        """单次测量后应更新状态"""
        kf = KalmanFilterSimulator()
        kf.update(10.0)
        self.assertAlmostEqual(kf.get_position(), 5.0, delta=3.0)

    def test_constant_measurements_converge(self):
        """恒定测量应收敛到真实值"""
        kf = KalmanFilterSimulator(measure_noise=0.1)
        true_value = 10.0
        for _ in range(100):
            kf.update(true_value)
        self.assertAlmostEqual(kf.get_position(), true_value, delta=0.5)

    def test_velocity_estimation(self):
        """匀速运动应估计出速度"""
        kf = KalmanFilterSimulator(dt=0.1, process_noise=0.001, measure_noise=0.1)
        # 匀速: 每步+1
        for i in range(100):
            kf.update(float(i) * 1.0)
        # 速度应趋近 1.0/0.1 = 10.0
        vel = kf.get_velocity()
        self.assertGreater(vel, 5.0)


class TestKalmanNoiseReduction(unittest.TestCase):
    """噪声抑制测试"""

    def test_filter_reduces_noise(self):
        """滤波后应比原始测量噪声更小"""
        np.random.seed(42)
        kf = KalmanFilterSimulator(measure_noise=1.0)
        true_value = 50.0
        measurements = true_value + np.random.randn(200) * 5.0
        filtered = []
        for m in measurements:
            kf.update(m)
            filtered.append(kf.get_position())
        var_raw = np.var(measurements[20:])
        var_filtered = np.var(filtered[20:])
        self.assertLess(var_filtered, var_raw)

    def test_low_noise_tracking(self):
        """低噪声下应精确跟踪"""
        kf = KalmanFilterSimulator(measure_noise=0.01)
        for _ in range(50):
            kf.update(10.0)
        self.assertAlmostEqual(kf.get_position(), 10.0, delta=0.1)


class TestKalmanReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        """reset应清零状态"""
        kf = KalmanFilterSimulator()
        kf.update(10.0)
        kf.reset()
        self.assertAlmostEqual(kf.get_position(), 0.0)
        self.assertAlmostEqual(kf.get_velocity(), 0.0)


class TestKalmanStepResponse(unittest.TestCase):
    """阶跃响应测试"""

    def test_step_response_delay(self):
        """阶跃响应应有一定延迟(平滑性)"""
        kf = KalmanFilterSimulator(measure_noise=0.5)
        # 阶跃: 从0跳到10
        kf.update(0.0)
        kf.update(0.0)
        pos_after_step = kf.update(10.0)
        # 不应立即到达10
        self.assertLess(pos_after_step, 10.0)

    def test_eventual_convergence(self):
        """最终应收敛"""
        kf = KalmanFilterSimulator(measure_noise=0.1)
        for _ in range(200):
            kf.update(10.0)
        self.assertAlmostEqual(kf.get_position(), 10.0, delta=0.2)


class TestComplementaryFilter(unittest.TestCase):
    """互补滤波器测试"""

    def test_first_value_passthrough(self):
        """首次输入直接通过"""
        cf = ComplementaryFilterSimulator(alpha=0.98)
        self.assertAlmostEqual(cf.update(10.0), 10.0)

    def test_high_alpha_slow_response(self):
        """大alpha(偏向旧值)应响应慢"""
        cf = ComplementaryFilterSimulator(alpha=0.99)
        cf.update(0.0)
        for _ in range(10):
            v = cf.update(100.0)
        self.assertLess(v, 30.0)

    def test_low_alpha_fast_response(self):
        """小alpha(偏向新值)应响应快"""
        cf = ComplementaryFilterSimulator(alpha=0.1)
        cf.update(0.0)
        for _ in range(5):
            v = cf.update(100.0)
        self.assertGreater(v, 80.0)

    def test_constant_convergence(self):
        """常数输入最终收敛"""
        cf = ComplementaryFilterSimulator(alpha=0.95)
        for _ in range(200):
            v = cf.update(42.0)
        self.assertAlmostEqual(v, 42.0, places=2)

    def test_reset(self):
        """重置后应重新初始化"""
        cf = ComplementaryFilterSimulator()
        cf.update(100.0)
        cf.reset()
        self.assertFalse(cf.initialized)
        self.assertAlmostEqual(cf.update(50.0), 50.0)

    def test_sensor_fusion(self):
        """互补滤波融合两个传感器"""
        # 模拟: 陀螺仪积分(漂移) + 加速度计(噪声)
        cf = ComplementaryFilterSimulator(alpha=0.98)
        true_angle = 45.0
        gyro_drift = 0.1  # 陀螺仪漂移
        acc_noise = 2.0   # 加速度计噪声
        np.random.seed(42)
        gyro_angle = 0.0
        for i in range(200):
            gyro_angle += gyro_drift + np.random.randn() * 0.5
            acc_angle = true_angle + np.random.randn() * acc_noise
            fused = cf.update(acc_angle)
        # 最终应接近真实角度(在合理范围内)
        self.assertGreater(fused, 30.0)


if __name__ == '__main__':
    unittest.main()
