#!/usr/bin/env python3
"""
VL53L0X 激光测距传感器 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、I2C地址、测量模式、精度设置、距离读取、
      范围限制、信号强度、环境光、边界条件
对应C源文件: 02_mspm0g3507/drivers/vl53l0x.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    VL53L0X, VL53L0X_ADDR_DEFAULT, VL53L0X_ADDR_ALTERNATE,
    VL53L0X_MODE_SINGLE, VL53L0X_MODE_CONTINUOUS,
    VL53L0X_ACCURACY_DEFAULT, VL53L0X_ACCURACY_HIGH,
    VL53L0X_ACCURACY_LONG, VL53L0X_ACCURACY_HIGH_SPEED,
    VL53L0X_MAX_RANGE_MM, VL53L0X_MIN_RANGE_MM,
)


class TestVL53L0XInit(unittest.TestCase):
    """VL53L0X初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        sensor = VL53L0X()
        self.assertTrue(sensor.init())
        self.assertTrue(sensor.initialized)

    def test_default_address(self):
        """默认I2C地址0x29"""
        sensor = VL53L0X()
        self.assertEqual(sensor.addr, VL53L0X_ADDR_DEFAULT)

    def test_custom_address(self):
        """自定义I2C地址"""
        sensor = VL53L0X(VL53L0X_ADDR_ALTERNATE)
        self.assertEqual(sensor.addr, VL53L0X_ADDR_ALTERNATE)

    def test_default_mode(self):
        """默认单次测量模式"""
        sensor = VL53L0X()
        self.assertEqual(sensor.mode, VL53L0X_MODE_SINGLE)

    def test_default_accuracy(self):
        """默认精度模式"""
        sensor = VL53L0X()
        self.assertEqual(sensor.accuracy, VL53L0X_ACCURACY_DEFAULT)


class TestVL53L0XMode(unittest.TestCase):
    """VL53L0X测量模式测试"""

    def setUp(self):
        self.sensor = VL53L0X()
        self.sensor.init()

    def test_set_single_mode(self):
        """设置单次测量模式"""
        self.assertTrue(self.sensor.set_mode(VL53L0X_MODE_SINGLE))
        self.assertEqual(self.sensor.mode, VL53L0X_MODE_SINGLE)

    def test_set_continuous_mode(self):
        """设置连续测量模式"""
        self.assertTrue(self.sensor.set_mode(VL53L0X_MODE_CONTINUOUS))
        self.assertEqual(self.sensor.mode, VL53L0X_MODE_CONTINUOUS)

    def test_set_invalid_mode(self):
        """无效模式失败"""
        self.assertFalse(self.sensor.set_mode(0x02))
        self.assertFalse(self.sensor.set_mode(-1))


class TestVL53L0XAccuracy(unittest.TestCase):
    """VL53L0X精度模式测试"""

    def setUp(self):
        self.sensor = VL53L0X()
        self.sensor.init()

    def test_set_all_accuracy_modes(self):
        """所有精度模式均可设置"""
        modes = [VL53L0X_ACCURACY_DEFAULT, VL53L0X_ACCURACY_HIGH,
                 VL53L0X_ACCURACY_LONG, VL53L0X_ACCURACY_HIGH_SPEED]
        for mode in modes:
            self.assertTrue(self.sensor.set_accuracy(mode))
            self.assertEqual(self.sensor.accuracy, mode)

    def test_set_invalid_accuracy(self):
        """无效精度模式失败"""
        self.assertFalse(self.sensor.set_accuracy(0x04))
        self.assertFalse(self.sensor.set_accuracy(-1))


class TestVL53L0XDistance(unittest.TestCase):
    """VL53L0X距离读取测试"""

    def setUp(self):
        self.sensor = VL53L0X()
        self.sensor.init()

    def test_read_default_distance(self):
        """默认距离值为0"""
        self.assertEqual(self.sensor.read_distance(), 0)

    def test_read_normal_distance(self):
        """正常距离读取"""
        self.sensor.set_simulated(500)
        self.assertEqual(self.sensor.read_distance(), 500)

    def test_read_short_distance(self):
        """低于最小范围返回0"""
        self.sensor.set_simulated(10)  # < 30mm
        self.assertEqual(self.sensor.read_distance(), 0)

    def test_read_min_range(self):
        """最小范围边界"""
        self.sensor.set_simulated(VL53L0X_MIN_RANGE_MM)
        self.assertEqual(self.sensor.read_distance(), VL53L0X_MIN_RANGE_MM)

    def test_read_max_range(self):
        """最大范围边界"""
        self.sensor.set_simulated(VL53L0X_MAX_RANGE_MM)
        self.assertEqual(self.sensor.read_distance(), VL53L0X_MAX_RANGE_MM)

    def test_read_over_range_clamped(self):
        """超过最大范围被截断"""
        self.sensor.set_simulated(3000)
        self.assertEqual(self.sensor.read_distance(), VL53L0X_MAX_RANGE_MM)

    def test_read_not_initialized(self):
        """未初始化返回None"""
        sensor2 = VL53L0X()
        self.assertIsNone(sensor2.read_distance())


class TestVL53L0XSignalAmbient(unittest.TestCase):
    """VL53L0X信号强度和环境光测试"""

    def setUp(self):
        self.sensor = VL53L0X()
        self.sensor.init()

    def test_read_signal_default(self):
        """默认信号强度为0"""
        self.assertEqual(self.sensor.read_signal(), 0)

    def test_read_signal_value(self):
        """读取信号强度"""
        self.sensor.set_simulated(500, signal=1200)
        self.assertEqual(self.sensor.read_signal(), 1200)

    def test_read_ambient_default(self):
        """默认环境光为0"""
        self.assertEqual(self.sensor.read_ambient(), 0)

    def test_read_ambient_value(self):
        """读取环境光强度"""
        self.sensor.set_simulated(500, ambient=300)
        self.assertEqual(self.sensor.read_ambient(), 300)

    def test_signal_not_initialized(self):
        """未初始化读取信号返回None"""
        sensor2 = VL53L0X()
        self.assertIsNone(sensor2.read_signal())

    def test_ambient_not_initialized(self):
        """未初始化读取环境光返回None"""
        sensor2 = VL53L0X()
        self.assertIsNone(sensor2.read_ambient())


class TestVL53L0XRangeConstants(unittest.TestCase):
    """VL53L0X范围常量测试"""

    def test_max_range(self):
        """最大测量距离2000mm"""
        self.assertEqual(VL53L0X_MAX_RANGE_MM, 2000)

    def test_min_range(self):
        """最小测量距离30mm"""
        self.assertEqual(VL53L0X_MIN_RANGE_MM, 30)

    def test_address_values(self):
        """I2C地址值正确"""
        self.assertEqual(VL53L0X_ADDR_DEFAULT, 0x29)
        self.assertEqual(VL53L0X_ADDR_ALTERNATE, 0x30)


if __name__ == '__main__':
    unittest.main()
