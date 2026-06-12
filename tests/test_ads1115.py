#!/usr/bin/env python3
"""
ADS1115 16位ADC V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、PGA设置、MUX设置、数据速率、原始读取、
      电压转换、连续/单次模式、地址、边界条件
对应C源文件: 02_mspm0g3507/drivers/ads1115.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    ADS1115, ADS1115_ADDR_GND, ADS1115_ADDR_VDD,
    ADS1115_PGA_6_144V, ADS1115_PGA_4_096V, ADS1115_PGA_2_048V,
    ADS1115_PGA_1_024V, ADS1115_PGA_0_512V, ADS1115_PGA_0_256V,
    ADS1115_PGA_FSR,
    ADS1115_MUX_AIN0_AIN1, ADS1115_MUX_AIN0_GND, ADS1115_MUX_AIN3_GND,
    ADS1115_DR_8SPS, ADS1115_DR_250SPS, ADS1115_DR_860SPS,
)


class TestADS1115Init(unittest.TestCase):
    """ADS1115初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        adc = ADS1115()
        self.assertTrue(adc.init())
        self.assertTrue(adc.initialized)

    def test_default_address(self):
        """默认I2C地址0x48"""
        adc = ADS1115()
        self.assertEqual(adc.addr, ADS1115_ADDR_GND)

    def test_custom_address(self):
        """自定义I2C地址"""
        adc = ADS1115(ADS1115_ADDR_VDD)
        self.assertEqual(adc.addr, ADS1115_ADDR_VDD)

    def test_default_pga(self):
        """默认PGA为±2.048V"""
        adc = ADS1115()
        self.assertEqual(adc.pga, ADS1115_PGA_2_048V)

    def test_default_mux(self):
        """默认MUX为AIN0单端"""
        adc = ADS1115()
        self.assertEqual(adc.mux, ADS1115_MUX_AIN0_GND)


class TestADS1115PGA(unittest.TestCase):
    """ADS1115 PGA增益测试"""

    def setUp(self):
        self.adc = ADS1115()
        self.adc.init()

    def test_set_pga_all(self):
        """所有PGA选项均可设置"""
        for pga in ADS1115_PGA_FSR:
            self.assertTrue(self.adc.set_pga(pga))
            self.assertEqual(self.adc.pga, pga)

    def test_set_pga_invalid(self):
        """无效PGA值失败"""
        self.assertFalse(self.adc.set_pga(0x06))
        self.assertFalse(self.adc.set_pga(-1))

    def test_pga_fsr_values(self):
        """PGA满量程电压映射正确"""
        self.assertAlmostEqual(ADS1115_PGA_FSR[ADS1115_PGA_6_144V], 6.144)
        self.assertAlmostEqual(ADS1115_PGA_FSR[ADS1115_PGA_2_048V], 2.048)
        self.assertAlmostEqual(ADS1115_PGA_FSR[ADS1115_PGA_0_256V], 0.256)

    def test_get_pga_fsr(self):
        """获取当前PGA满量程"""
        self.adc.set_pga(ADS1115_PGA_6_144V)
        self.assertAlmostEqual(self.adc.get_pga_fsr(), 6.144)


class TestADS1115MUX(unittest.TestCase):
    """ADS1115多路复用器测试"""

    def setUp(self):
        self.adc = ADS1115()
        self.adc.init()

    def test_set_mux_valid(self):
        """有效MUX设置"""
        for m in range(8):
            self.assertTrue(self.adc.set_mux(m))
            self.assertEqual(self.adc.mux, m)

    def test_set_mux_invalid(self):
        """无效MUX设置"""
        self.assertFalse(self.adc.set_mux(-1))
        self.assertFalse(self.adc.set_mux(8))


class TestADS1115DataRate(unittest.TestCase):
    """ADS1115数据速率测试"""

    def setUp(self):
        self.adc = ADS1115()
        self.adc.init()

    def test_set_data_rate_valid(self):
        """有效数据速率"""
        for r in range(8):
            self.assertTrue(self.adc.set_data_rate(r))

    def test_set_data_rate_invalid(self):
        """无效数据速率"""
        self.assertFalse(self.adc.set_data_rate(-1))
        self.assertFalse(self.adc.set_data_rate(8))


class TestADS1115Read(unittest.TestCase):
    """ADS1115读取测试"""

    def setUp(self):
        self.adc = ADS1115()
        self.adc.init()

    def test_read_raw_default(self):
        """默认原始值为0"""
        self.assertEqual(self.adc.read_raw(), 0)

    def test_read_raw_simulated(self):
        """设置模拟原始值"""
        self.adc.set_simulated_raw(12345)
        self.assertEqual(self.adc.read_raw(), 12345)

    def test_read_raw_not_initialized(self):
        """未初始化读取返回None"""
        adc2 = ADS1115()
        self.assertIsNone(adc2.read_raw())

    def test_read_raw_mask_16bit(self):
        """原始值掩码为16位"""
        self.adc.set_simulated_raw(0x1FFFF)
        self.assertEqual(self.adc.read_raw(), 0xFFFF)


class TestADS1115Voltage(unittest.TestCase):
    """ADS1115电压转换测试"""

    def setUp(self):
        self.adc = ADS1115()
        self.adc.init()

    def test_read_voltage_zero(self):
        """原始值0对应电压0"""
        self.adc.set_simulated_raw(0)
        v = self.adc.read_voltage()
        self.assertAlmostEqual(v, 0.0, places=4)

    def test_read_voltage_positive(self):
        """正电压"""
        self.adc.set_pga(ADS1115_PGA_2_048V)
        # 正半量程: 16383/32767 * 2.048 ≈ 1.024V
        self.adc.set_simulated_raw(16383)
        v = self.adc.read_voltage()
        self.assertGreater(v, 0)
        self.assertLess(v, 2.048)

    def test_read_voltage_max(self):
        """满量程正电压"""
        self.adc.set_pga(ADS1115_PGA_6_144V)
        self.adc.set_simulated_raw(0x7FFF)
        v = self.adc.read_voltage()
        self.assertAlmostEqual(v, 6.144, places=2)

    def test_read_voltage_negative(self):
        """负电压（差分模式）"""
        self.adc.set_pga(ADS1115_PGA_2_048V)
        # 负值: 0xFFFF = -1
        self.adc.set_simulated_raw(0xFFFF)
        v = self.adc.read_voltage()
        self.assertLess(v, 0)

    def test_read_voltage_not_initialized(self):
        """未初始化返回None"""
        adc2 = ADS1115()
        self.assertIsNone(adc2.read_voltage())

    def test_raw_to_voltage(self):
        """raw_to_voltage工具函数"""
        self.adc.set_pga(ADS1115_PGA_4_096V)
        v = self.adc.raw_to_voltage(0)
        self.assertAlmostEqual(v, 0.0, places=4)

    def test_different_pga_voltage(self):
        """不同PGA下电压不同"""
        self.adc.set_simulated_raw(10000)
        self.adc.set_pga(ADS1115_PGA_1_024V)
        v1 = self.adc.read_voltage()
        self.adc.set_pga(ADS1115_PGA_6_144V)
        v2 = self.adc.read_voltage()
        # 更大PGA → 更大电压
        self.assertGreater(abs(v2), abs(v1))


class TestADS1115Mode(unittest.TestCase):
    """ADS1115模式测试"""

    def test_continuous_default_false(self):
        """默认非连续模式"""
        adc = ADS1115()
        self.assertFalse(adc.continuous)

    def test_set_continuous(self):
        """设置连续模式"""
        adc = ADS1115()
        adc.init()
        adc.set_continuous(True)
        self.assertTrue(adc.continuous)

    def test_set_single_shot(self):
        """设置单次模式"""
        adc = ADS1115()
        adc.init()
        adc.set_continuous(True)
        adc.set_continuous(False)
        self.assertFalse(adc.continuous)


if __name__ == '__main__':
    unittest.main()
