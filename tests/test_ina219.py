#!/usr/bin/env python3
"""
INA219 电流/功率传感器 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、I2C地址、校准、分流电压、总线电压、
      电流读取、功率读取、配置参数、边界条件
对应C源文件: 02_mspm0g3507/drivers/ina219.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    INA219, INA219_ADDR_0, INA219_ADDR_1, INA219_ADDR_2, INA219_ADDR_3,
    INA219_BUS_RANGE_16V, INA219_BUS_RANGE_32V,
    INA219_PGA_40MV, INA219_PGA_80MV, INA219_PGA_160MV, INA219_PGA_320MV,
    INA219_ADC_9BIT, INA219_ADC_10BIT, INA219_ADC_11BIT, INA219_ADC_12BIT,
)


class TestINA219Init(unittest.TestCase):
    """INA219初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        sensor = INA219()
        self.assertTrue(sensor.init())
        self.assertTrue(sensor.initialized)

    def test_default_address(self):
        """默认I2C地址0x40"""
        sensor = INA219()
        self.assertEqual(sensor.addr, INA219_ADDR_0)

    def test_custom_address(self):
        """自定义I2C地址"""
        sensor = INA219(INA219_ADDR_3)
        self.assertEqual(sensor.addr, INA219_ADDR_3)

    def test_default_bus_range(self):
        """默认总线电压范围32V"""
        sensor = INA219()
        self.assertEqual(sensor.bus_range, INA219_BUS_RANGE_32V)

    def test_default_pga(self):
        """默认PGA增益±320mV"""
        sensor = INA219()
        self.assertEqual(sensor.pga, INA219_PGA_320MV)

    def test_default_adc(self):
        """默认ADC分辨率12位"""
        sensor = INA219()
        self.assertEqual(sensor.shunt_adc, INA219_ADC_12BIT)
        self.assertEqual(sensor.bus_adc, INA219_ADC_12BIT)


class TestINA219Calibrate(unittest.TestCase):
    """INA219校准测试"""

    def setUp(self):
        self.sensor = INA219()
        self.sensor.init()

    def test_calibrate_success(self):
        """校准成功"""
        self.assertTrue(self.sensor.calibrate(3.2, 0.1))

    def test_calibrate_sets_current_lsb(self):
        """校准设置电流LSB"""
        self.sensor.calibrate(3.2, 0.1)
        expected_lsb = 3.2 / 32767.0
        self.assertAlmostEqual(self.sensor._current_lsb, expected_lsb, places=8)

    def test_calibrate_sets_power_lsb(self):
        """校准设置功率LSB为20倍电流LSB"""
        self.sensor.calibrate(3.2, 0.1)
        expected_power_lsb = 20.0 * (3.2 / 32767.0)
        self.assertAlmostEqual(self.sensor._power_lsb, expected_power_lsb, places=8)

    def test_calibrate_sets_cal_value(self):
        """校准计算校准寄存器值"""
        self.sensor.calibrate(3.2, 0.1)
        self.assertGreater(self.sensor.get_cal_value(), 0)

    def test_calibrate_invalid_current(self):
        """无效电流值校准失败"""
        self.assertFalse(self.sensor.calibrate(0, 0.1))
        self.assertFalse(self.sensor.calibrate(-1, 0.1))

    def test_calibrate_invalid_shunt(self):
        """无效分流电阻校准失败"""
        self.assertFalse(self.sensor.calibrate(3.2, 0))
        self.assertFalse(self.sensor.calibrate(3.2, -0.1))

    def test_calibrate_not_initialized(self):
        """未初始化校准失败"""
        sensor2 = INA219()
        self.assertFalse(sensor2.calibrate(3.2, 0.1))

    def test_calibrate_different_values(self):
        """不同参数校准值不同"""
        self.sensor.calibrate(1.0, 0.1)
        cal1 = self.sensor.get_cal_value()
        self.sensor.calibrate(3.2, 0.1)
        cal2 = self.sensor.get_cal_value()
        self.assertNotEqual(cal1, cal2)


class TestINA219Readings(unittest.TestCase):
    """INA219读数测试"""

    def setUp(self):
        self.sensor = INA219()
        self.sensor.init()
        self.sensor.calibrate(3.2, 0.1)

    def test_read_shunt_voltage(self):
        """读取分流电压"""
        self.sensor.set_simulated(50.0, 12.0, 0.5, 6.0)
        self.assertAlmostEqual(self.sensor.read_shunt_voltage(), 50.0)

    def test_read_bus_voltage(self):
        """读取总线电压"""
        self.sensor.set_simulated(50.0, 12.0, 0.5, 6.0)
        self.assertAlmostEqual(self.sensor.read_bus_voltage(), 12.0)

    def test_read_current(self):
        """读取电流"""
        self.sensor.set_simulated(50.0, 12.0, 0.5, 6.0)
        self.assertAlmostEqual(self.sensor.read_current(), 0.5)

    def test_read_power(self):
        """读取功率"""
        self.sensor.set_simulated(50.0, 12.0, 0.5, 6.0)
        self.assertAlmostEqual(self.sensor.read_power(), 6.0)

    def test_read_default_values(self):
        """默认读数为0"""
        self.assertAlmostEqual(self.sensor.read_shunt_voltage(), 0.0)
        self.assertAlmostEqual(self.sensor.read_bus_voltage(), 0.0)
        self.assertAlmostEqual(self.sensor.read_current(), 0.0)
        self.assertAlmostEqual(self.sensor.read_power(), 0.0)


class TestINA219NotInitialized(unittest.TestCase):
    """INA219未初始化测试"""

    def test_read_shunt_not_init(self):
        """未初始化读取分流电压返回None"""
        sensor = INA219()
        self.assertIsNone(sensor.read_shunt_voltage())

    def test_read_bus_not_init(self):
        """未初始化读取总线电压返回None"""
        sensor = INA219()
        self.assertIsNone(sensor.read_bus_voltage())

    def test_read_current_not_init(self):
        """未初始化读取电流返回None"""
        sensor = INA219()
        self.assertIsNone(sensor.read_current())

    def test_read_power_not_init(self):
        """未初始化读取功率返回None"""
        sensor = INA219()
        self.assertIsNone(sensor.read_power())


class TestINA219Config(unittest.TestCase):
    """INA219配置测试"""

    def setUp(self):
        self.sensor = INA219()
        self.sensor.init()

    def test_bus_range_16v(self):
        """设置16V总线范围"""
        self.sensor.bus_range = INA219_BUS_RANGE_16V
        self.assertEqual(self.sensor.bus_range, INA219_BUS_RANGE_16V)

    def test_bus_range_32v(self):
        """设置32V总线范围"""
        self.sensor.bus_range = INA219_BUS_RANGE_32V
        self.assertEqual(self.sensor.bus_range, INA219_BUS_RANGE_32V)

    def test_pga_options(self):
        """所有PGA选项"""
        for pga in (INA219_PGA_40MV, INA219_PGA_80MV,
                    INA219_PGA_160MV, INA219_PGA_320MV):
            self.sensor.pga = pga
            self.assertEqual(self.sensor.pga, pga)

    def test_adc_resolutions(self):
        """所有ADC分辨率选项"""
        for adc in (INA219_ADC_9BIT, INA219_ADC_10BIT,
                    INA219_ADC_11BIT, INA219_ADC_12BIT):
            self.sensor.shunt_adc = adc
            self.sensor.bus_adc = adc
            self.assertEqual(self.sensor.shunt_adc, adc)
            self.assertEqual(self.sensor.bus_adc, adc)


class TestINA219Constants(unittest.TestCase):
    """INA219常量测试"""

    def test_address_values(self):
        """I2C地址值正确"""
        self.assertEqual(INA219_ADDR_0, 0x40)
        self.assertEqual(INA219_ADDR_1, 0x41)
        self.assertEqual(INA219_ADDR_2, 0x44)
        self.assertEqual(INA219_ADDR_3, 0x45)

    def test_bus_range_values(self):
        """总线范围常量"""
        self.assertEqual(INA219_BUS_RANGE_16V, 0x00)
        self.assertEqual(INA219_BUS_RANGE_32V, 0x01)

    def test_pga_values(self):
        """PGA增益常量"""
        self.assertEqual(INA219_PGA_40MV, 0x00)
        self.assertEqual(INA219_PGA_320MV, 0x03)

    def test_adc_values(self):
        """ADC分辨率常量"""
        self.assertEqual(INA219_ADC_9BIT, 0x00)
        self.assertEqual(INA219_ADC_12BIT, 0x03)


class TestINA219Realistic(unittest.TestCase):
    """INA219实际使用场景测试"""

    def setUp(self):
        self.sensor = INA219()
        self.sensor.init()

    def test_3v3_motor_scenario(self):
        """3.3V电机驱动场景"""
        self.sensor.calibrate(2.0, 0.05)  # 2A, 50mΩ
        self.sensor.set_simulated(
            shunt_mv=75.0,   # 75mV分流电压
            bus_v=3.3,       # 3.3V总线
            current_a=1.5,   # 1.5A电流
            power_w=4.95     # 4.95W功率
        )
        self.assertAlmostEqual(self.sensor.read_current(), 1.5)
        self.assertAlmostEqual(self.sensor.read_power(), 4.95)
        self.assertGreater(self.sensor.get_cal_value(), 0)

    def test_12v_led_scenario(self):
        """12V LED驱动场景"""
        self.sensor.calibrate(1.0, 0.1)  # 1A, 100mΩ
        self.sensor.set_simulated(
            shunt_mv=30.0,
            bus_v=12.0,
            current_a=0.3,
            power_w=3.6
        )
        self.assertAlmostEqual(self.sensor.read_bus_voltage(), 12.0)
        self.assertAlmostEqual(self.sensor.read_current(), 0.3)


if __name__ == '__main__':
    unittest.main()
