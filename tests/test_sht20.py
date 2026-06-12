#!/usr/bin/env python3
"""
SHT20 温湿度传感器 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、温度读取、湿度读取、原始值转换、边界条件、用户寄存器、软复位
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    SHT20, SHT20_ADDR, SHT20_CMD_TEMP_HOLD, SHT20_CMD_HUMI_HOLD,
    SHT20_CMD_SOFT_RESET,
)


class TestSHT20V2Init(unittest.TestCase):
    """初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        sht = SHT20()
        ok = sht.init()
        self.assertTrue(ok)
        self.assertTrue(sht.initialized)

    def test_default_addr(self):
        """默认I2C地址0x40"""
        sht = SHT20()
        self.assertEqual(sht.addr, SHT20_ADDR)


class TestSHT20V2Temperature(unittest.TestCase):
    """温度读取测试"""

    def test_temperature_25c(self):
        """25°C对应的原始值: raw = (25 + 46.85) / 175.72 * 65536 ≈ 26736"""
        sht = SHT20()
        sht.init()
        raw = int((25.0 + 46.85) / 175.72 * 65536)
        sht.set_raw_values(raw, 0)
        ok, temp = sht.read_temperature()
        self.assertTrue(ok)
        self.assertAlmostEqual(temp, 25.0, places=1)

    def test_temperature_0c(self):
        """0°C"""
        sht = SHT20()
        sht.init()
        raw = int((0.0 + 46.85) / 175.72 * 65536)
        sht.set_raw_values(raw, 0)
        ok, temp = sht.read_temperature()
        self.assertTrue(ok)
        self.assertAlmostEqual(temp, 0.0, places=1)

    def test_temperature_negative(self):
        """-10°C"""
        sht = SHT20()
        sht.init()
        raw = int((-10.0 + 46.85) / 175.72 * 65536)
        sht.set_raw_values(raw, 0)
        ok, temp = sht.read_temperature()
        self.assertTrue(ok)
        self.assertAlmostEqual(temp, -10.0, places=1)

    def test_temperature_high(self):
        """80°C（高温）"""
        sht = SHT20()
        sht.init()
        raw = int((80.0 + 46.85) / 175.72 * 65536)
        raw = min(raw, 0xFFFC)
        sht.set_raw_values(raw, 0)
        ok, temp = sht.read_temperature()
        self.assertTrue(ok)
        self.assertAlmostEqual(temp, 80.0, places=0)

    def test_temperature_not_initialized(self):
        """未初始化应失败"""
        sht = SHT20()
        ok, _ = sht.read_temperature()
        self.assertFalse(ok)


class TestSHT20V2Humidity(unittest.TestCase):
    """湿度读取测试"""

    def test_humidity_50pct(self):
        """50%RH对应的原始值: raw = (50 + 6) / 125 * 65536 ≈ 29286"""
        sht = SHT20()
        sht.init()
        raw = int((50.0 + 6.0) / 125.0 * 65536)
        sht.set_raw_values(0, raw)
        ok, humi = sht.read_humidity()
        self.assertTrue(ok)
        self.assertAlmostEqual(humi, 50.0, places=1)

    def test_humidity_0pct(self):
        """0%RH"""
        sht = SHT20()
        sht.init()
        raw = int((0.0 + 6.0) / 125.0 * 65536)
        sht.set_raw_values(0, raw)
        ok, humi = sht.read_humidity()
        self.assertTrue(ok)
        self.assertAlmostEqual(humi, 0.0, places=0)

    def test_humidity_100pct(self):
        """100%RH"""
        sht = SHT20()
        sht.init()
        raw = int((100.0 + 6.0) / 125.0 * 65536)
        raw = min(raw, 0xFFFC)
        sht.set_raw_values(0, raw)
        ok, humi = sht.read_humidity()
        self.assertTrue(ok)
        self.assertAlmostEqual(humi, 100.0, places=0)

    def test_humidity_clamp(self):
        """湿度应限幅到0~100"""
        sht = SHT20()
        sht.init()
        # 原始值为0时: -6.0 + 125*0/65536 = -6.0 → 应钳位到0
        sht.set_raw_values(0, 0)
        _, humi = sht.read_humidity()
        self.assertGreaterEqual(humi, 0.0)

    def test_humidity_not_initialized(self):
        """未初始化应失败"""
        sht = SHT20()
        ok, _ = sht.read_humidity()
        self.assertFalse(ok)


class TestSHT20V2UserReg(unittest.TestCase):
    """用户寄存器测试"""

    def test_read_default_reg(self):
        """默认用户寄存器=0x02"""
        sht = SHT20()
        sht.init()
        ok, val = sht.read_user_reg()
        self.assertTrue(ok)
        self.assertEqual(val, 0x02)

    def test_write_read_reg(self):
        """写入后读出一致"""
        sht = SHT20()
        sht.init()
        sht.write_user_reg(0x3A)
        _, val = sht.read_user_reg()
        self.assertEqual(val, 0x3A)

    def test_soft_reset(self):
        """软复位恢复默认"""
        sht = SHT20()
        sht.init()
        sht.write_user_reg(0xFF)
        sht.soft_reset()
        _, val = sht.read_user_reg()
        self.assertEqual(val, 0x02)


class TestSHT20V2Constants(unittest.TestCase):
    """常量一致性"""

    def test_constants(self):
        self.assertEqual(SHT20_ADDR, 0x40)
        self.assertEqual(SHT20_CMD_TEMP_HOLD, 0xE3)
        self.assertEqual(SHT20_CMD_HUMI_HOLD, 0xE5)
        self.assertEqual(SHT20_CMD_SOFT_RESET, 0xFE)


if __name__ == '__main__':
    unittest.main()
