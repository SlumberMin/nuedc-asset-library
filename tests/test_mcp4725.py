#!/usr/bin/env python3
"""
MCP4725 12位DAC V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、DAC值设置/读取、电压设置/读取、掉电模式、
      EEPROM写入、状态读取、边界条件
对应C源文件: 02_mspm0g3507/drivers/mcp4725.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    MCP4725, MCP4725_ADDR_A0_GND, MCP4725_ADDR_A0_VDD,
    MCP4725_MAX_VALUE, MCP4725_VREF_DEFAULT,
    MCP4725_PD_NONE, MCP4725_PD_1K, MCP4725_PD_100K, MCP4725_PD_500K,
)


class TestMCP4725Init(unittest.TestCase):
    """MCP4725初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        dac = MCP4725()
        self.assertTrue(dac.init())
        self.assertTrue(dac.initialized)

    def test_default_address(self):
        """默认I2C地址0x60"""
        dac = MCP4725()
        self.assertEqual(dac.addr, MCP4725_ADDR_A0_GND)

    def test_custom_address(self):
        """自定义地址"""
        dac = MCP4725(MCP4725_ADDR_A0_VDD)
        self.assertEqual(dac.addr, MCP4725_ADDR_A0_VDD)

    def test_default_vref(self):
        """默认参考电压3.3V"""
        dac = MCP4725()
        self.assertAlmostEqual(dac.vref, 3.3)

    def test_custom_vref(self):
        """自定义参考电压"""
        dac = MCP4725(vref=5.0)
        self.assertAlmostEqual(dac.vref, 5.0)

    def test_init_value_zero(self):
        """初始化后DAC值为0"""
        dac = MCP4725()
        dac.init()
        self.assertEqual(dac.get_value(), 0)

    def test_init_power_down_none(self):
        """初始化后掉电模式为无"""
        dac = MCP4725()
        dac.init()
        self.assertEqual(dac.power_down, MCP4725_PD_NONE)


class TestMCP4725SetValue(unittest.TestCase):
    """MCP4725 DAC值设置测试"""

    def setUp(self):
        self.dac = MCP4725()
        self.dac.init()

    def test_set_value_zero(self):
        """设置值为0"""
        self.assertTrue(self.dac.set_value(0))
        self.assertEqual(self.dac.get_value(), 0)

    def test_set_value_max(self):
        """设置最大值4095"""
        self.assertTrue(self.dac.set_value(MCP4725_MAX_VALUE))
        self.assertEqual(self.dac.get_value(), MCP4725_MAX_VALUE)

    def test_set_value_mid(self):
        """设置中间值"""
        self.assertTrue(self.dac.set_value(2048))
        self.assertEqual(self.dac.get_value(), 2048)

    def test_set_value_negative_fail(self):
        """负值失败"""
        self.assertFalse(self.dac.set_value(-1))

    def test_set_value_overflow_fail(self):
        """超范围失败"""
        self.assertFalse(self.dac.set_value(4096))

    def test_set_value_not_initialized(self):
        """未初始化设置失败"""
        dac2 = MCP4725()
        self.assertFalse(dac2.set_value(100))

    def test_set_value_all_12bit(self):
        """12位范围内所有边界"""
        for val in [0, 1, 2047, 2048, 4094, 4095]:
            self.assertTrue(self.dac.set_value(val))
            self.assertEqual(self.dac.get_value(), val)


class TestMCP4725Voltage(unittest.TestCase):
    """MCP4725电压设置测试"""

    def setUp(self):
        self.dac = MCP4725(vref=3.3)
        self.dac.init()

    def test_set_voltage_zero(self):
        """设置0V"""
        self.assertTrue(self.dac.set_voltage(0.0))
        self.assertEqual(self.dac.get_value(), 0)

    def test_set_voltage_max(self):
        """设置满电压"""
        self.assertTrue(self.dac.set_voltage(3.3))
        self.assertEqual(self.dac.get_value(), MCP4725_MAX_VALUE)

    def test_set_voltage_mid(self):
        """设置中间电压"""
        self.assertTrue(self.dac.set_voltage(1.65))
        # 1.65/3.3 * 4095 ≈ 2048
        self.assertAlmostEqual(self.dac.get_value(), 2048, delta=1)

    def test_set_voltage_negative_fail(self):
        """负电压失败"""
        self.assertFalse(self.dac.set_voltage(-0.1))

    def test_set_voltage_over_vref_fail(self):
        """超过参考电压失败"""
        self.assertFalse(self.dac.set_voltage(3.4))

    def test_get_voltage(self):
        """获取电压"""
        self.dac.set_value(2048)
        v = self.dac.get_voltage()
        self.assertAlmostEqual(v, 2048 / 4095 * 3.3, places=3)

    def test_get_voltage_zero(self):
        """0对应0V"""
        self.dac.set_value(0)
        self.assertAlmostEqual(self.dac.get_voltage(), 0.0)

    def test_get_voltage_max(self):
        """满值对应参考电压"""
        self.dac.set_value(MCP4725_MAX_VALUE)
        self.assertAlmostEqual(self.dac.get_voltage(), 3.3, places=2)

    def test_set_voltage_5v_ref(self):
        """5V参考电压"""
        dac = MCP4725(vref=5.0)
        dac.init()
        dac.set_voltage(2.5)
        self.assertAlmostEqual(dac.get_value(), 2048, delta=1)


class TestMCP4725PowerDown(unittest.TestCase):
    """MCP4725掉电模式测试"""

    def setUp(self):
        self.dac = MCP4725()
        self.dac.init()

    def test_set_power_down_none(self):
        """正常模式"""
        self.assertTrue(self.dac.set_power_down(MCP4725_PD_NONE))
        self.assertEqual(self.dac.power_down, MCP4725_PD_NONE)

    def test_set_power_down_1k(self):
        """1K下拉"""
        self.assertTrue(self.dac.set_power_down(MCP4725_PD_1K))
        self.assertEqual(self.dac.power_down, MCP4725_PD_1K)

    def test_set_power_down_100k(self):
        """100K下拉"""
        self.assertTrue(self.dac.set_power_down(MCP4725_PD_100K))

    def test_set_power_down_500k(self):
        """500K下拉"""
        self.assertTrue(self.dac.set_power_down(MCP4725_PD_500K))

    def test_set_power_down_invalid(self):
        """无效掉电模式"""
        self.assertFalse(self.dac.set_power_down(4))
        self.assertFalse(self.dac.set_power_down(-1))


class TestMCP4725EEPROM(unittest.TestCase):
    """MCP4725 EEPROM测试"""

    def setUp(self):
        self.dac = MCP4725()
        self.dac.init()

    def test_write_eeprom_value(self):
        """写入EEPROM DAC值"""
        self.assertTrue(self.dac.write_eeprom(value=2048))
        self.assertEqual(self.dac.eeprom_dac, 2048)

    def test_write_eeprom_pd(self):
        """写入EEPROM掉电模式"""
        self.assertTrue(self.dac.write_eeprom(pd_mode=MCP4725_PD_1K))
        self.assertEqual(self.dac.eeprom_pd, MCP4725_PD_1K)

    def test_write_eeprom_both(self):
        """同时写入值和掉电模式"""
        self.assertTrue(self.dac.write_eeprom(value=1000, pd_mode=MCP4725_PD_100K))
        self.assertEqual(self.dac.eeprom_dac, 1000)
        self.assertEqual(self.dac.eeprom_pd, MCP4725_PD_100K)

    def test_write_eeprom_invalid_value(self):
        """无效EEPROM值"""
        self.assertFalse(self.dac.write_eeprom(value=5000))
        self.assertFalse(self.dac.write_eeprom(value=-1))

    def test_write_eeprom_not_initialized(self):
        """未初始化EEPROM写入失败"""
        dac2 = MCP4725()
        self.assertFalse(dac2.write_eeprom(value=100))


class TestMCP4725Status(unittest.TestCase):
    """MCP4725状态读取测试"""

    def test_read_status(self):
        """状态字典包含所有字段"""
        dac = MCP4725()
        dac.init()
        status = dac.read_status()
        self.assertIn('ready', status)
        self.assertIn('por', status)
        self.assertIn('dac_value', status)
        self.assertIn('power_down', status)
        self.assertIn('eeprom_dac', status)
        self.assertIn('eeprom_pd', status)

    def test_status_ready(self):
        """初始化后ready为True"""
        dac = MCP4725()
        dac.init()
        status = dac.read_status()
        self.assertTrue(status['ready'])

    def test_status_dac_value(self):
        """状态中DAC值正确"""
        dac = MCP4725()
        dac.init()
        dac.set_value(1234)
        status = dac.read_status()
        self.assertEqual(status['dac_value'], 1234)


class TestMCP4725Constants(unittest.TestCase):
    """MCP4725常量一致性"""

    def test_max_value(self):
        """12位最大值4095"""
        self.assertEqual(MCP4725_MAX_VALUE, 4095)

    def test_vref_default(self):
        """默认参考电压3.3V"""
        self.assertAlmostEqual(MCP4725_VREF_DEFAULT, 3.3)


if __name__ == '__main__':
    unittest.main()
