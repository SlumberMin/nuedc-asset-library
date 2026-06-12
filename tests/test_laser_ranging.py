#!/usr/bin/env python3
"""
激光测距机器人 V2 测试 — 基于wrappers.py包装层
覆盖: VL53L0X传感器 + 舵机云台 + 卡尔曼滤波 + 测距避障逻辑
模拟场景: 舵机带动激光测距仪扫描，卡尔曼滤波平滑距离数据
对应C源文件: 02_mspm0g3507/drivers/vl53l0x.c
              02_mspm0g3507/drivers/servo.c
              02_mspm0g3507/drivers/kalman_filter.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    VL53L0X, VL53L0X_MODE_CONTINUOUS, VL53L0X_ACCURACY_HIGH,
    VL53L0X_MAX_RANGE_MM, VL53L0X_MIN_RANGE_MM,
    Servo, SERVO_MAX_ANGLE,
    KalmanFilter,
)


class TestLaserRangingInit(unittest.TestCase):
    """激光测距机器人初始化测试"""

    def test_sensor_init(self):
        """VL53L0X传感器初始化成功"""
        sensor = VL53L0X()
        self.assertTrue(sensor.init())
        self.assertTrue(sensor.initialized)

    def test_servo_init(self):
        """舵机初始化成功，默认居中90°"""
        servo = Servo()
        servo.init()
        self.assertEqual(servo.get_angle(), 90)

    def test_kalman_init(self):
        """卡尔曼滤波器初始化成功"""
        kf = KalmanFilter(dt=0.1, proc_noise=1.0, meas_noise=1.0)
        self.assertAlmostEqual(kf.x[0], 0.0)
        self.assertAlmostEqual(kf.x[1], 0.0)

    def test_full_system_init(self):
        """完整系统初始化：传感器+舵机+滤波器"""
        sensor = VL53L0X()
        servo = Servo()
        kf = KalmanFilter()
        self.assertTrue(sensor.init())
        servo.init()
        self.assertTrue(sensor.initialized)
        self.assertEqual(servo.get_angle(), 90)


class TestLaserRangingMeasurement(unittest.TestCase):
    """激光测距测量测试"""

    def setUp(self):
        """初始化传感器和滤波器"""
        self.sensor = VL53L0X()
        self.sensor.init()
        self.sensor.set_mode(VL53L0X_MODE_CONTINUOUS)
        self.sensor.set_accuracy(VL53L0X_ACCURACY_HIGH)
        self.kf = KalmanFilter(dt=0.1, proc_noise=1.0, meas_noise=5.0)

    def test_basic_distance_read(self):
        """基本距离读取"""
        self.sensor.set_simulated(500)
        self.assertEqual(self.sensor.read_distance(), 500)

    def test_filtered_distance(self):
        """卡尔曼滤波后距离更平滑"""
        # 输入有噪声的距离数据
        distances = [480, 520, 490, 510, 500]
        filtered = []
        for d in distances:
            self.sensor.set_simulated(d)
            raw = self.sensor.read_distance()
            self.kf.predict()
            self.kf.update_1d(raw)
            filtered.append(self.kf.x[0])

        # 滤波后最后一个值应接近真实值500
        self.assertAlmostEqual(filtered[-1], 500, delta=250)

    def test_distance_out_of_range(self):
        """超范围距离被截断"""
        self.sensor.set_simulated(VL53L0X_MAX_RANGE_MM + 500)
        dist = self.sensor.read_distance()
        self.assertEqual(dist, VL53L0X_MAX_RANGE_MM)

    def test_minimum_range(self):
        """最小测量范围边界"""
        self.sensor.set_simulated(VL53L0X_MIN_RANGE_MM)
        self.assertEqual(self.sensor.read_distance(), VL53L0X_MIN_RANGE_MM)

    def test_below_minimum_range(self):
        """低于最小范围返回0"""
        self.sensor.set_simulated(VL53L0X_MIN_RANGE_MM - 10)
        self.assertEqual(self.sensor.read_distance(), 0)


class TestServoScan(unittest.TestCase):
    """舵机扫描控制测试"""

    def setUp(self):
        self.servo = Servo()
        self.servo.init()

    def test_scan_left_to_right(self):
        """从左到右扫描"""
        angles = []
        for angle in range(0, SERVO_MAX_ANGLE + 1, 15):
            self.servo.set_angle(angle)
            angles.append(self.servo.get_angle())
        # 第一个角度为0，最后一个为180
        self.assertEqual(angles[0], 0)
        self.assertEqual(angles[-1], SERVO_MAX_ANGLE)

    def test_scan_right_to_left(self):
        """从右到左扫描"""
        angles = []
        for angle in range(SERVO_MAX_ANGLE, -1, -15):
            self.servo.set_angle(angle)
            angles.append(self.servo.get_angle())
        self.assertEqual(angles[0], SERVO_MAX_ANGLE)
        self.assertEqual(angles[-1], 0)

    def test_servo_over_range_clamped(self):
        """超过180°被截断"""
        self.servo.set_angle(200)
        self.assertEqual(self.servo.get_angle(), SERVO_MAX_ANGLE)

    def test_servo_center_position(self):
        """居中位置90°"""
        self.servo.set_angle(90)
        self.assertEqual(self.servo.get_angle(), 90)

    def test_servo_stop(self):
        """舵机停止"""
        self.servo.stop()
        self.assertFalse(self.servo.running)


class TestLaserRangingObstacle(unittest.TestCase):
    """激光测距避障逻辑测试"""

    def setUp(self):
        self.sensor = VL53L0X()
        self.sensor.init()
        self.OBSTACLE_THRESHOLD = 300  # 300mm避障阈值

    def test_no_obstacle(self):
        """无障碍物（距离>阈值）"""
        self.sensor.set_simulated(1000)
        dist = self.sensor.read_distance()
        self.assertGreater(dist, self.OBSTACLE_THRESHOLD)

    def test_obstacle_detected(self):
        """检测到障碍物（距离<阈值）"""
        self.sensor.set_simulated(200)
        dist = self.sensor.read_distance()
        self.assertLess(dist, self.OBSTACLE_THRESHOLD)

    def test_obstacle_at_threshold(self):
        """障碍物恰好在阈值处"""
        self.sensor.set_simulated(self.OBSTACLE_THRESHOLD)
        dist = self.sensor.read_distance()
        self.assertEqual(dist, self.OBSTACLE_THRESHOLD)

    def test_scan_find_obstacle(self):
        """扫描过程中发现障碍物"""
        # 模拟扫描：大部分距离远，某个角度有障碍物
        scan_distances = [800, 750, 600, 200, 500, 700, 900]
        obstacle_angles = []
        for i, dist in enumerate(scan_distances):
            self.sensor.set_simulated(dist)
            d = self.sensor.read_distance()
            if d < self.OBSTACLE_THRESHOLD:
                obstacle_angles.append(i)
        # 索引3处有障碍物
        self.assertEqual(obstacle_angles, [3])


class TestLaserRangingSignalQuality(unittest.TestCase):
    """信号质量测试"""

    def setUp(self):
        self.sensor = VL53L0X()
        self.sensor.init()

    def test_signal_strength(self):
        """读取信号强度"""
        self.sensor.set_simulated(500, signal=1500)
        self.assertEqual(self.sensor.read_signal(), 1500)

    def test_ambient_light(self):
        """读取环境光"""
        self.sensor.set_simulated(500, ambient=250)
        self.assertEqual(self.sensor.read_ambient(), 250)

    def test_weak_signal_scenario(self):
        """弱信号场景：远距离信号弱"""
        self.sensor.set_simulated(1800, signal=100)
        dist = self.sensor.read_distance()
        signal = self.sensor.read_signal()
        self.assertEqual(dist, 1800)
        self.assertEqual(signal, 100)

    def test_not_initialized_returns_none(self):
        """未初始化读取返回None"""
        sensor2 = VL53L0X()
        self.assertIsNone(sensor2.read_distance())
        self.assertIsNone(sensor2.read_signal())
        self.assertIsNone(sensor2.read_ambient())


if __name__ == '__main__':
    unittest.main()
