#!/usr/bin/env python3
"""
超声波 V3 测试 — 多传感器融合测试
覆盖: V2全部 + 多传感器实例、中值滤波、多路测距一致性、
      距离阈值决策、连续测距统计、异常脉宽处理
对应C源文件: 02_mspm0g3507/drivers/ultrasonic.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #1:  除零保护 (US_PER_CM)
  #14: 多路超声波融合避障，需覆盖多实例与滤波逻辑
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    Ultrasonic, ULTRASONIC_US_PER_CM, ULTRASONIC_MIN_CM,
    ULTRASONIC_MAX_CM, ULTRASONIC_TIMEOUT_US,
)


def median_filter(values):
    """中值滤波器 — 用于超声波数据预处理"""
    if not values:
        return 0.0
    sorted_vals = sorted(values)
    n = len(sorted_vals)
    if n % 2 == 1:
        return sorted_vals[n // 2]
    return (sorted_vals[n // 2 - 1] + sorted_vals[n // 2]) / 2.0


def obstacle_decision(distances, threshold=30.0):
    """
    多路超声波避障决策
    返回: 'forward', 'turn_left', 'turn_right', 'stop'
    假设distances = [left, front, right]
    """
    if len(distances) < 3:
        return 'stop'
    left, front, right = distances
    if front > threshold:
        return 'forward'
    if left > right:
        return 'turn_left'
    return 'turn_right'


class TestUltrasonicV3(unittest.TestCase):
    """超声波V3 — 多传感器融合测试"""

    def setUp(self):
        self.us = Ultrasonic()
        self.us.init()

    # ── 基础功能 ──

    def test_init_state(self):
        """初始化后状态正确"""
        self.assertTrue(self.us.initialized)
        self.assertEqual(self.us.last_distance, 0.0)
        self.assertEqual(self.us.measure_count, 0)
        self.assertEqual(self.us.timeout_count, 0)

    def test_measure_100cm(self):
        """100cm距离: 脉宽 = 100 * 58 = 5800µs"""
        pulse_us = int(100.0 * ULTRASONIC_US_PER_CM)
        ok, dist = self.us.measure(pulse_us)
        self.assertTrue(ok)
        self.assertAlmostEqual(dist, 100.0, places=1)

    def test_measure_10cm(self):
        """10cm距离"""
        pulse_us = int(10.0 * ULTRASONIC_US_PER_CM)
        ok, dist = self.us.measure(pulse_us)
        self.assertTrue(ok)
        self.assertAlmostEqual(dist, 10.0, places=1)

    def test_measure_400cm(self):
        """400cm最大距离"""
        pulse_us = int(400.0 * ULTRASONIC_US_PER_CM)
        ok, dist = self.us.measure(pulse_us)
        self.assertTrue(ok)
        self.assertAlmostEqual(dist, 400.0, places=0)

    def test_measure_2cm_min(self):
        """2cm最小距离"""
        pulse_us = int(2.0 * ULTRASONIC_US_PER_CM)
        ok, dist = self.us.measure(pulse_us)
        self.assertTrue(ok)
        self.assertAlmostEqual(dist, 2.0, places=1)

    def test_measure_timeout_zero(self):
        """脉宽为0应超时"""
        ok, dist = self.us.measure(0)
        self.assertFalse(ok)
        self.assertEqual(dist, 0.0)
        self.assertEqual(self.us.timeout_count, 1)

    def test_measure_timeout_negative(self):
        """负脉宽应超时"""
        ok, dist = self.us.measure(-100)
        self.assertFalse(ok)

    def test_measure_timeout_exceeded(self):
        """超过最大超时应失败"""
        ok, dist = self.us.measure(ULTRASONIC_TIMEOUT_US + 1)
        self.assertFalse(ok)

    def test_measure_count_increments(self):
        """测量计数递增"""
        self.us.measure(1000)
        self.us.measure(2000)
        self.us.measure(3000)
        self.assertEqual(self.us.measure_count, 3)

    def test_measure_raw_success(self):
        """原始脉宽测量成功"""
        ok, pulse = self.us.measure_raw(5800)
        self.assertTrue(ok)
        self.assertEqual(pulse, 5800)

    def test_measure_raw_timeout(self):
        """原始脉宽超时"""
        ok, pulse = self.us.measure_raw(0)
        self.assertFalse(ok)
        self.assertEqual(pulse, 0)

    def test_last_distance_updated(self):
        """last_distance跟踪"""
        self.us.measure(int(50.0 * ULTRASONIC_US_PER_CM))
        self.assertAlmostEqual(self.us.get_last_distance(), 50.0, places=1)

    def test_constants_consistency(self):
        """常量一致性"""
        self.assertAlmostEqual(ULTRASONIC_US_PER_CM, 58.0)
        self.assertEqual(ULTRASONIC_MIN_CM, 2.0)
        self.assertEqual(ULTRASONIC_MAX_CM, 400.0)
        self.assertEqual(ULTRASONIC_TIMEOUT_US, 30000)

    # ── V3: 多传感器实例 ──

    def test_multi_sensor_independent(self):
        """多传感器实例互不干扰"""
        us_left = Ultrasonic()
        us_front = Ultrasonic()
        us_right = Ultrasonic()
        us_left.init()
        us_front.init()
        us_right.init()

        us_left.measure(int(50.0 * ULTRASONIC_US_PER_CM))
        us_front.measure(int(100.0 * ULTRASONIC_US_PER_CM))
        us_right.measure(int(30.0 * ULTRASONIC_US_PER_CM))

        self.assertAlmostEqual(us_left.get_last_distance(), 50.0, places=1)
        self.assertAlmostEqual(us_front.get_last_distance(), 100.0, places=1)
        self.assertAlmostEqual(us_right.get_last_distance(), 30.0, places=1)

    def test_multi_sensor_count_independent(self):
        """多传感器计数独立"""
        us1 = Ultrasonic()
        us2 = Ultrasonic()
        us1.init()
        us2.init()

        us1.measure(5800)
        us1.measure(5800)
        us2.measure(5800)

        self.assertEqual(us1.measure_count, 2)
        self.assertEqual(us2.measure_count, 1)

    # ── V3: 中值滤波 ──

    def test_median_filter_odd(self):
        """中值滤波: 奇数个样本"""
        values = [10.0, 30.0, 20.0]
        self.assertAlmostEqual(median_filter(values), 20.0)

    def test_median_filter_even(self):
        """中值滤波: 偶数个样本"""
        values = [10.0, 20.0, 30.0, 40.0]
        self.assertAlmostEqual(median_filter(values), 25.0)

    def test_median_filter_single(self):
        """中值滤波: 单个样本"""
        self.assertAlmostEqual(median_filter([42.0]), 42.0)

    def test_median_filter_empty(self):
        """中值滤波: 空列表"""
        self.assertAlmostEqual(median_filter([]), 0.0)

    def test_median_filter_rejects_outlier(self):
        """中值滤波: 能抑制异常值"""
        # 正常值约50cm，一个异常值300cm
        values = [49.0, 50.0, 300.0, 51.0, 50.0]
        result = median_filter(values)
        self.assertAlmostEqual(result, 50.0, places=0)

    # ── V3: 避障决策 ──

    def test_obstacle_forward(self):
        """前方无障碍: 直行"""
        self.assertEqual(obstacle_decision([50, 100, 50]), 'forward')

    def test_obstacle_turn_left(self):
        """前方有障, 左侧更空: 左转"""
        self.assertEqual(obstacle_decision([80, 10, 20]), 'turn_left')

    def test_obstacle_turn_right(self):
        """前方有障, 右侧更空: 右转"""
        self.assertEqual(obstacle_decision([20, 10, 80]), 'turn_right')

    def test_obstacle_stop(self):
        """参数不足: 停止"""
        self.assertEqual(obstacle_decision([10, 20]), 'stop')

    def test_obstacle_equal_sides(self):
        """两侧等距: 默认右转"""
        self.assertEqual(obstacle_decision([30, 10, 30]), 'turn_right')

    # ── V3: 连续测距统计 ──

    def test_consecutive_measures(self):
        """连续测距计数正确"""
        for i in range(10):
            self.us.measure(int((i + 1) * 58))
        self.assertEqual(self.us.measure_count, 10)

    def test_consecutive_timeouts_count(self):
        """连续超时计数"""
        for _ in range(5):
            self.us.measure(0)
        self.assertEqual(self.us.timeout_count, 5)
        self.assertEqual(self.us.measure_count, 5)

    # ── V3: 边界脉宽值 ──

    def test_measure_exact_max_distance_boundary(self):
        """恰好在最大距离边界(400cm)"""
        pulse_us = int(400.0 * ULTRASONIC_US_PER_CM)
        ok, dist = self.us.measure(pulse_us)
        self.assertTrue(ok)
        self.assertAlmostEqual(dist, 400.0, places=0)

    def test_measure_one_over_timeout(self):
        """超时边界+1"""
        ok, dist = self.us.measure(ULTRASONIC_TIMEOUT_US + 1)
        self.assertFalse(ok)

    def test_measure_pulse_min_distance(self):
        """恰好最小距离(2cm)的脉宽"""
        pulse_us = int(2.0 * ULTRASONIC_US_PER_CM)
        ok, dist = self.us.measure(pulse_us)
        self.assertTrue(ok)
        self.assertAlmostEqual(dist, 2.0, places=1)


if __name__ == '__main__':
    unittest.main()
