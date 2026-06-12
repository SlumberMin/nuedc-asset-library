#!/usr/bin/env python3
"""
速度估计器单元测试
覆盖: 基于编码器的速度估计、M/T法、卡尔曼速度估计
测试: 正常速度估计、零速、反转、高/低速、噪声、性能基准
"""

import sys
import os
import unittest
import math
import time
import random

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class EncoderSpeedEstimator:
    """基于编码器的速度估计器(M/T法)"""

    def __init__(self, ppr=1000, dt=0.01, filter_alpha=0.3):
        self.ppr = ppr          # 每转脉冲数
        self.dt = dt            # 采样周期
        self.filter_alpha = filter_alpha
        self.prev_count = 0
        self.speed_rpm = 0.0
        self.speed_filtered = 0.0
        self.direction = 1      # 1=正转, -1=反转

    def update(self, count, dt_actual=None):
        """更新速度估计"""
        if dt_actual is None:
            dt_actual = self.dt
        if dt_actual <= 0:
            return self.speed_filtered

        delta = count - self.prev_count
        self.prev_count = count

        # 转换为RPM
        rpm = (delta / self.ppr) / dt_actual * 60.0

        # 方向检测
        if delta > 0:
            self.direction = 1
        elif delta < 0:
            self.direction = -1

        self.speed_rpm = rpm

        # 低通滤波
        self.speed_filtered = (self.filter_alpha * rpm +
                               (1 - self.filter_alpha) * self.speed_filtered)
        return self.speed_filtered

    def get_speed_rpm(self):
        """获取滤波后速度(RPM)"""
        return self.speed_filtered

    def get_speed_rad_s(self):
        """获取滤波后速度(rad/s)"""
        return self.speed_filtered * 2 * math.pi / 60.0

    def get_direction(self):
        """获取旋转方向"""
        return self.direction

    def reset(self):
        """重置"""
        self.prev_count = 0
        self.speed_rpm = 0.0
        self.speed_filtered = 0.0


class FrequencySpeedEstimator:
    """基于频率的速度估计器(测周法/测频法)"""

    def __init__(self, ppr=1000, min_period_s=0.0001):
        self.ppr = ppr
        self.min_period_s = min_period_s
        self.last_timestamp = 0
        self.speed_rpm = 0.0
        self.pulse_count = 0

    def on_pulse(self, timestamp_s):
        """脉冲回调"""
        if self.last_timestamp > 0:
            period = timestamp_s - self.last_timestamp
            if period >= self.min_period_s:
                freq = 1.0 / period
                self.speed_rpm = freq / self.ppr * 60.0
        self.last_timestamp = timestamp_s
        self.pulse_count += 1
        return self.speed_rpm

    def update_counting(self, count, window_s):
        """测频法: 在时间窗口内计数"""
        if window_s <= 0:
            return 0.0
        freq = count / window_s
        self.speed_rpm = freq / self.ppr * 60.0
        return self.speed_rpm

    def get_speed_rpm(self):
        """获取速度(RPM)"""
        return self.speed_rpm

    def reset(self):
        """重置"""
        self.last_timestamp = 0
        self.speed_rpm = 0.0
        self.pulse_count = 0


class KalmanSpeedEstimator:
    """基于卡尔曼滤波的速度估计器"""

    def __init__(self, dt=0.01, q=0.01, r=0.1):
        self.dt = dt
        self.q = q      # 过程噪声
        self.r = r      # 测量噪声
        self.x = 0.0    # 速度状态
        self.p = 1.0    # 估计协方差
        self.k = 0.0    # 卡尔曼增益

    def update(self, measured_speed):
        """卡尔曼滤波更新"""
        # 预测(假设速度恒定)
        self.p += self.q
        # 更新
        self.k = self.p / (self.p + self.r)
        self.x += self.k * (measured_speed - self.x)
        self.p = (1 - self.k) * self.p
        return self.x

    def get_speed_rpm(self):
        """获取估计速度"""
        return self.x

    def reset(self):
        """重置"""
        self.x = 0.0
        self.p = 1.0
        self.k = 0.0


class TestEncoderSpeedEstimator(unittest.TestCase):
    """编码器速度估计器测试"""

    def test_initialization(self):
        """测试初始化"""
        est = EncoderSpeedEstimator(ppr=1000, dt=0.01)
        self.assertEqual(est.ppr, 1000)
        self.assertEqual(est.dt, 0.01)

    def test_zero_speed(self):
        """测试零速"""
        est = EncoderSpeedEstimator(ppr=1000, dt=0.01)
        est.update(0)
        est.update(0)
        self.assertAlmostEqual(est.get_speed_rpm(), 0.0, delta=1.0)

    def test_constant_speed(self):
        """测试匀速"""
        est = EncoderSpeedEstimator(ppr=1000, dt=0.01, filter_alpha=1.0)
        # 1000脉冲/转，0.01s内100脉冲 = 100/1000/0.01*60 = 600 RPM
        count = 0
        for _ in range(50):
            count += 100
            est.update(count, dt_actual=0.01)
        self.assertAlmostEqual(est.get_speed_rpm(), 600.0, delta=50.0)

    def test_reverse_direction(self):
        """测试反转"""
        est = EncoderSpeedEstimator(ppr=1000, dt=0.01, filter_alpha=1.0)
        est.update(100, 0.01)
        self.assertEqual(est.get_direction(), 1)
        est.update(50, 0.01)
        self.assertEqual(est.get_direction(), -1)

    def test_speed_filtering(self):
        """测试速度滤波"""
        est = EncoderSpeedEstimator(ppr=1000, dt=0.01, filter_alpha=0.1)
        count = 0
        for i in range(100):
            count += 100
            est.update(count, 0.01)
        # 滤波后应逐渐趋近真实值
        self.assertGreater(est.get_speed_rpm(), 0)

    def test_speed_conversion(self):
        """测试速度单位转换"""
        est = EncoderSpeedEstimator(ppr=1000, dt=0.01, filter_alpha=1.0)
        est.update(100, 0.01)
        rpm = est.get_speed_rpm()
        rad_s = est.get_speed_rad_s()
        expected_rad_s = rpm * 2 * math.pi / 60.0
        self.assertAlmostEqual(rad_s, expected_rad_s, places=3)

    def test_reset(self):
        """测试重置"""
        est = EncoderSpeedEstimator()
        est.update(100, 0.01)
        est.reset()
        self.assertEqual(est.speed_filtered, 0.0)
        self.assertEqual(est.prev_count, 0)

    def test_zero_dt(self):
        """测试零采样周期"""
        est = EncoderSpeedEstimator()
        est.update(100, 0.0)
        # 应返回上一次的速度值
        self.assertEqual(est.get_speed_rpm(), 0.0)

    def test_performance_benchmark(self):
        """性能基准: 速度估计性能"""
        est = EncoderSpeedEstimator(ppr=1000, dt=0.01)
        iterations = 100000
        start = time.perf_counter()
        for i in range(iterations):
            est.update(i * 100, 0.01)
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 5.0, "速度估计应<5μs")


class TestFrequencySpeedEstimator(unittest.TestCase):
    """频率速度估计器测试"""

    def test_initialization(self):
        """测试初始化"""
        est = FrequencySpeedEstimator(ppr=1000)
        self.assertEqual(est.ppr, 1000)

    def test_on_pulse(self):
        """测试脉冲回调"""
        est = FrequencySpeedEstimator(ppr=100)
        est.on_pulse(0.001)  # 第一个脉冲(非零时间)
        speed = est.on_pulse(0.011)  # 0.01s后第二个脉冲
        # freq=100Hz, speed=100/100*60=60RPM
        self.assertAlmostEqual(speed, 60.0, delta=5.0)

    def test_counting_method(self):
        """测试测频法"""
        est = FrequencySpeedEstimator(ppr=1000)
        speed = est.update_counting(100, 0.01)
        # 100脉冲/0.01s=10000Hz, 10000/1000*60=600RPM
        self.assertAlmostEqual(speed, 600.0)

    def test_zero_window(self):
        """测试零时间窗口"""
        est = FrequencySpeedEstimator()
        speed = est.update_counting(100, 0.0)
        self.assertEqual(speed, 0.0)

    def test_reset(self):
        """测试重置"""
        est = FrequencySpeedEstimator()
        est.on_pulse(0.0)
        est.on_pulse(0.01)
        est.reset()
        self.assertEqual(est.speed_rpm, 0.0)

    def test_performance_benchmark(self):
        """性能基准: 频率速度估计"""
        est = FrequencySpeedEstimator(ppr=1000)
        iterations = 100000
        start = time.perf_counter()
        for i in range(iterations):
            est.update_counting(i % 100, 0.01)
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 2.0, "频率估计应<2μs")


class TestKalmanSpeedEstimator(unittest.TestCase):
    """卡尔曼速度估计器测试"""

    def test_initialization(self):
        """测试初始化"""
        est = KalmanSpeedEstimator()
        self.assertEqual(est.x, 0.0)

    def test_filtering_noisy_input(self):
        """测试噪声过滤"""
        est = KalmanSpeedEstimator(q=0.001, r=1.0)
        true_speed = 500.0
        random.seed(42)
        for _ in range(100):
            noisy = true_speed + random.gauss(0, 50.0)
            est.update(noisy)
        self.assertAlmostEqual(est.x, true_speed, delta=20.0)

    def test_convergence(self):
        """测试收敛"""
        est = KalmanSpeedEstimator(q=0.01, r=0.1)
        for _ in range(50):
            est.update(100.0)
        self.assertAlmostEqual(est.x, 100.0, delta=1.0)

    def test_kalman_gain_range(self):
        """测试卡尔曼增益范围"""
        est = KalmanSpeedEstimator()
        est.update(50.0)
        self.assertGreaterEqual(est.k, 0.0)
        self.assertLessEqual(est.k, 1.0)

    def test_reset(self):
        """测试重置"""
        est = KalmanSpeedEstimator()
        est.update(100.0)
        est.reset()
        self.assertEqual(est.x, 0.0)
        self.assertEqual(est.p, 1.0)

    def test_step_response(self):
        """测试阶跃响应"""
        est = KalmanSpeedEstimator(q=0.1, r=0.5)
        est.update(0.0)
        for _ in range(20):
            result = est.update(100.0)
        self.assertAlmostEqual(result, 100.0, delta=5.0)

    def test_performance_benchmark(self):
        """性能基准: 卡尔曼速度估计"""
        est = KalmanSpeedEstimator()
        iterations = 100000
        start = time.perf_counter()
        for i in range(iterations):
            est.update(float(i % 1000))
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 3.0, "卡尔曼速度估计应<3μs")


if __name__ == '__main__':
    unittest.main()
