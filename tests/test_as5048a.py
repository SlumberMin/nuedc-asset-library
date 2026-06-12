#!/usr/bin/env python3
"""
AS5048A 磁编码器 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、I2C地址、原始角度读取、角度转换、
      零点偏移、AGC、磁场强度、边界条件
对应C源文件: 02_mspm0g3507/drivers/as5048a.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    AS5048A, AS5048A_ADDR_DEFAULT, AS5048A_ADDR_ALT1,
    AS5048A_ADDR_ALT2, AS5048A_ADDR_ALT3,
    AS5048A_MAX_RAW, AS5048A_DEGREES,
)


class TestAS5048AInit(unittest.TestCase):
    """AS5048A初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        enc = AS5048A()
        self.assertTrue(enc.init())
        self.assertTrue(enc.initialized)

    def test_default_address(self):
        """默认I2C地址0x40"""
        enc = AS5048A()
        self.assertEqual(enc.addr, AS5048A_ADDR_DEFAULT)

    def test_custom_address(self):
        """自定义I2C地址"""
        enc = AS5048A(AS5048A_ADDR_ALT1)
        self.assertEqual(enc.addr, AS5048A_ADDR_ALT1)

    def test_all_addresses(self):
        """所有备用地址均可设置"""
        for addr in (AS5048A_ADDR_DEFAULT, AS5048A_ADDR_ALT1,
                     AS5048A_ADDR_ALT2, AS5048A_ADDR_ALT3):
            enc = AS5048A(addr)
            self.assertEqual(enc.addr, addr)


class TestAS5048ARaw(unittest.TestCase):
    """AS5048A原始角度读取测试"""

    def setUp(self):
        self.enc = AS5048A()
        self.enc.init()

    def test_read_raw_default(self):
        """默认原始值为0"""
        self.assertEqual(self.enc.read_raw(), 0)

    def test_read_raw_value(self):
        """读取模拟原始值"""
        self.enc.set_simulated(8192)
        self.assertEqual(self.enc.read_raw(), 8192)

    def test_read_raw_max(self):
        """最大原始值16383"""
        self.enc.set_simulated(AS5048A_MAX_RAW)
        self.assertEqual(self.enc.read_raw(), AS5048A_MAX_RAW)

    def test_read_raw_mask(self):
        """原始值掩码为14位"""
        self.enc.set_simulated(0x7FFF)  # 15位值
        self.assertEqual(self.enc.read_raw(), 0x3FFF)  # 截断为14位

    def test_read_raw_not_initialized(self):
        """未初始化返回None"""
        enc2 = AS5048A()
        self.assertIsNone(enc2.read_raw())


class TestAS5048AAngle(unittest.TestCase):
    """AS5048A角度转换测试"""

    def setUp(self):
        self.enc = AS5048A()
        self.enc.init()

    def test_angle_zero(self):
        """原始值0对应角度0"""
        self.enc.set_simulated(0)
        self.assertAlmostEqual(self.enc.read_angle(), 0.0, places=2)

    def test_angle_half(self):
        """半量程对应180度"""
        self.enc.set_simulated(AS5048A_MAX_RAW // 2)
        angle = self.enc.read_angle()
        self.assertAlmostEqual(angle, 180.0, places=1)

    def test_angle_max(self):
        """最大原始值对应接近360度"""
        self.enc.set_simulated(AS5048A_MAX_RAW)
        angle = self.enc.read_angle()
        self.assertAlmostEqual(angle, 360.0, places=1)

    def test_angle_range(self):
        """角度在0-360范围内"""
        for raw in (0, 4096, 8192, 12288, 16383):
            self.enc.set_simulated(raw)
            angle = self.enc.read_angle()
            self.assertGreaterEqual(angle, 0.0)
            self.assertLessEqual(angle, 360.0)

    def test_angle_not_initialized(self):
        """未初始化返回None"""
        enc2 = AS5048A()
        self.assertIsNone(enc2.read_angle())


class TestAS5048AOffset(unittest.TestCase):
    """AS5048A零点偏移测试"""

    def setUp(self):
        self.enc = AS5048A()
        self.enc.init()

    def test_default_no_offset(self):
        """默认无偏移时角度一致"""
        self.enc.set_simulated(8192)
        angle = self.enc.read_angle()
        angle_offset = self.enc.read_angle_with_offset()
        self.assertAlmostEqual(angle, angle_offset, places=2)

    def test_set_zero_offset(self):
        """设置零点偏移"""
        self.enc.set_simulated(8192)
        angle_before = self.enc.read_angle()
        self.enc.set_zero(angle_before)
        angle_after = self.enc.read_angle_with_offset()
        self.assertAlmostEqual(angle_after, 0.0, places=1)

    def test_offset_wraps(self):
        """偏移后角度环绕到0-360"""
        self.enc.set_zero(350.0)
        self.enc.set_simulated(0)  # 角度约0度
        angle = self.enc.read_angle_with_offset()
        self.assertGreaterEqual(angle, 0.0)
        self.assertLess(angle, 360.0)

    def test_set_zero_return_true(self):
        """set_zero返回True"""
        self.assertTrue(self.enc.set_zero(90.0))

    def test_offset_not_initialized(self):
        """未初始化读取偏移角度返回None"""
        enc2 = AS5048A()
        self.assertIsNone(enc2.read_angle_with_offset())


class TestAS5048ADiagnostics(unittest.TestCase):
    """AS5048A诊断测试"""

    def setUp(self):
        self.enc = AS5048A()
        self.enc.init()

    def test_read_agc(self):
        """读取AGC值"""
        self.enc.set_simulated(8192, agc=128)
        self.assertEqual(self.enc.read_agc(), 128)

    def test_read_magnitude(self):
        """读取磁场强度"""
        self.enc.set_simulated(8192, magnitude=5000)
        self.assertEqual(self.enc.read_magnitude(), 5000)

    def test_agc_not_initialized(self):
        """未初始化读取AGC返回None"""
        enc2 = AS5048A()
        self.assertIsNone(enc2.read_agc())

    def test_magnitude_not_initialized(self):
        """未初始化读取磁场返回None"""
        enc2 = AS5048A()
        self.assertIsNone(enc2.read_magnitude())


class TestAS5048AConstants(unittest.TestCase):
    """AS5048A常量测试"""

    def test_max_raw(self):
        """14位最大值16383"""
        self.assertEqual(AS5048A_MAX_RAW, 0x3FFF)

    def test_degrees(self):
        """角度范围360度"""
        self.assertEqual(AS5048A_DEGREES, 360.0)

    def test_address_values(self):
        """I2C地址值正确"""
        self.assertEqual(AS5048A_ADDR_DEFAULT, 0x40)
        self.assertEqual(AS5048A_ADDR_ALT1, 0x41)
        self.assertEqual(AS5048A_ADDR_ALT2, 0x42)
        self.assertEqual(AS5048A_ADDR_ALT3, 0x43)


if __name__ == '__main__':
    unittest.main()
