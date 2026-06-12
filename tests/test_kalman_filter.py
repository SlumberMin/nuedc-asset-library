#!/usr/bin/env python3
"""
卡尔曼滤波器单元测试
覆盖: 初始化、预测更新、测量更新、收敛性、协方差矩阵、性能基准

V2修复: import wrappers.py中的生产代码逻辑，而非自行重写
对应C源: 02_mspm0g3507/drivers/kalman_filter.c
"""

import sys
import os
import unittest
import time
import random

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from tests.wrappers import KalmanFilter


class TestKalmanFilter(unittest.TestCase):
    """二维卡尔曼滤波器测试（与C版本KalmanFilter_t逻辑一致）"""

    def test_initialization(self):
        """测试初始化"""
        kf = KalmanFilter(dt=0.1, proc_noise=1.0, meas_noise=1.0)
        self.assertEqual(kf.x, [0.0, 0.0])
        self.assertEqual(kf.P, [[1.0, 0.0], [0.0, 1.0]])
        self.assertTrue(kf.initialized)

    def test_state_transition_matrix(self):
        """测试状态转移矩阵A正确（恒速模型）"""
        kf = KalmanFilter(dt=0.05)
        self.assertAlmostEqual(kf.A[0][0], 1.0)
        self.assertAlmostEqual(kf.A[0][1], 0.05)
        self.assertAlmostEqual(kf.A[1][0], 0.0)
        self.assertAlmostEqual(kf.A[1][1], 1.0)

    def test_control_input_matrix(self):
        """测试控制输入矩阵B正确"""
        kf = KalmanFilter(dt=0.1)
        self.assertAlmostEqual(kf.B[0], 0.5 * 0.01)
        self.assertAlmostEqual(kf.B[1], 0.1)

    def test_predict_constant_velocity(self):
        """测试匀速预测：位置应增加速度*dt"""
        kf = KalmanFilter(dt=0.1)
        kf.x = [0.0, 10.0]
        kf.predict()
        self.assertAlmostEqual(kf.x[0], 1.0)
        self.assertAlmostEqual(kf.x[1], 10.0)

    def test_predict_covariance_grows(self):
        """测试预测后协方差增长（无观测时不确定性增加）"""
        kf = KalmanFilter(dt=0.1)
        p_before = kf.P[0][0]
        kf.predict()
        self.assertGreater(kf.P[0][0], p_before)

    def test_update_reduces_uncertainty(self):
        """测试更新减少不确定性"""
        kf = KalmanFilter(dt=0.1, proc_noise=1.0, meas_noise=1.0)
        kf.predict()
        uncertainty_before = kf.get_uncertainty()
        kf.update_1d(5.0)
        uncertainty_after = kf.get_uncertainty()
        self.assertLess(uncertainty_after, uncertainty_before)

    def test_set_state(self):
        """测试设置初始状态"""
        kf = KalmanFilter(dt=0.1)
        kf.set_state(5.0, 2.0)
        self.assertEqual(kf.x[0], 5.0)
        self.assertEqual(kf.x[1], 2.0)

    def test_step_convenience(self):
        """测试一步操作（predict+update）"""
        kf = KalmanFilter(dt=0.1)
        pos, vel = kf.step(5.0)
        self.assertIsInstance(pos, float)
        self.assertIsInstance(vel, float)

    def test_predict_with_input(self):
        """测试带控制输入的预测"""
        kf = KalmanFilter(dt=0.1)
        kf.predict_with_input(10.0)
        # B[0]=0.005, B[1]=0.1, u=10 → x[0]+=0.05, x[1]+=1.0
        self.assertAlmostEqual(kf.x[0], 0.05, places=4)
        self.assertAlmostEqual(kf.x[1], 1.0, places=4)

    def test_tracking_constant_velocity(self):
        """测试跟踪匀速运动（收敛性）"""
        kf = KalmanFilter(dt=0.1, proc_noise=0.001, meas_noise=0.1)
        true_velocity = 5.0
        position = 0.0
        random.seed(42)
        for _ in range(100):
            position += true_velocity * 0.1
            measurement = position + random.gauss(0, 0.3)
            kf.step(measurement)
        self.assertAlmostEqual(kf.x[1], true_velocity, delta=1.0)

    def test_covariance_positive_definite(self):
        """测试协方差矩阵正定"""
        kf = KalmanFilter(dt=0.1)
        for i in range(100):
            kf.step(float(i))
        self.assertGreater(kf.P[0][0], 0)
        self.assertGreater(kf.P[1][1], 0)

    def test_reset(self):
        """测试重置"""
        kf = KalmanFilter(dt=0.1)
        kf.set_state(10.0, 5.0)
        kf.reset()
        self.assertEqual(kf.x, [0.0, 0.0])
        self.assertEqual(kf.P, [[1.0, 0.0], [0.0, 1.0]])

    def test_get_position_velocity(self):
        """测试获取位置和速度"""
        kf = KalmanFilter(dt=0.1)
        kf.set_state(3.0, 7.0)
        self.assertEqual(kf.get_position(), 3.0)
        self.assertEqual(kf.get_velocity(), 7.0)

    def test_convergence_speed(self):
        """测试收敛速度"""
        kf = KalmanFilter(dt=0.1, proc_noise=0.01, meas_noise=0.1)
        kf.x = [0.0, 0.0]
        kf.P = [[10.0, 0.0], [0.0, 10.0]]
        true_value = 5.0
        for _ in range(50):
            kf.step(true_value)
        self.assertAlmostEqual(kf.x[0], true_value, delta=0.5)

    def test_performance_benchmark(self):
        """性能基准: 2D卡尔曼滤波"""
        kf = KalmanFilter(dt=0.1)
        iterations = 50000
        start = time.perf_counter()
        for i in range(iterations):
            kf.step(float(i % 100))
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 50.0, "2D卡尔曼应<50μs")


if __name__ == '__main__':
    unittest.main()
