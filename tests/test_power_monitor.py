#!/usr/bin/env python3
"""
功率监测器 V2 测试 — 基于wrappers.py包装层
覆盖: INA219电流/功率传感器 + ADS1115 ADC + LCD1602显示
模拟场景: 电源功率实时监测、过流保护、数据显示
对应C源文件: 02_mspm0g3507/drivers/ina219.c
              02_mspm0g3507/drivers/ads1115.c
              02_mspm0g3507/drivers/lcd1602.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    INA219, INA219_ADDR_0, INA219_ADDR_1,
    INA219_BUS_RANGE_16V, INA219_BUS_RANGE_32V,
    INA219_PGA_40MV, INA219_PGA_320MV,
    INA219_ADC_12BIT, INA219_ADC_10BIT,
    ADS1115, ADS1115_ADDR_GND, ADS1115_ADDR_VDD,
    ADS1115_PGA_2_048V, ADS1115_PGA_4_096V, ADS1115_PGA_6_144V,
    ADS1115_MUX_AIN0_GND, ADS1115_MUX_AIN1_GND,
    ADS1115_DR_250SPS, ADS1115_DR_860SPS,
    LCD1602, LCD1602_COLS, LCD1602_ROWS,
)


class TestINA219Init(unittest.TestCase):
    """INA219初始化测试"""

    def test_init_success(self):
        """INA219初始化成功"""
        ina = INA219()
        self.assertTrue(ina.init())
        self.assertTrue(ina.initialized)

    def test_default_address(self):
        """默认I2C地址0x40"""
        ina = INA219()
        self.assertEqual(ina.addr, INA219_ADDR_0)

    def test_custom_address(self):
        """自定义I2C地址"""
        ina = INA219(INA219_ADDR_1)
        self.assertEqual(ina.addr, INA219_ADDR_1)

    def test_default_config(self):
        """默认配置：32V量程，±320mV PGA，12位ADC"""
        ina = INA219()
        self.assertEqual(ina.bus_range, INA219_BUS_RANGE_32V)
        self.assertEqual(ina.pga, INA219_PGA_320MV)
        self.assertEqual(ina.bus_adc, INA219_ADC_12BIT)
        self.assertEqual(ina.shunt_adc, INA219_ADC_12BIT)


class TestINA219Calibration(unittest.TestCase):
    """INA219校准测试"""

    def setUp(self):
        self.ina = INA219()
        self.ina.init()

    def test_calibrate_success(self):
        """校准成功"""
        self.assertTrue(self.ina.calibrate(3.2, 0.1))
        self.assertGreater(self.ina.get_cal_value(), 0)

    def test_calibrate_different_resistor(self):
        """不同分流电阻校准"""
        self.assertTrue(self.ina.calibrate(1.0, 0.05))
        cal1 = self.ina.get_cal_value()

        ina2 = INA219()
        ina2.init()
        self.assertTrue(ina2.calibrate(1.0, 0.1))
        cal2 = ina2.get_cal_value()

        # 不同分流电阻应得到不同校准值
        self.assertNotEqual(cal1, cal2)

    def test_calibrate_invalid_current(self):
        """无效电流校准失败"""
        self.assertFalse(self.ina.calibrate(0, 0.1))
        self.assertFalse(self.ina.calibrate(-1.0, 0.1))

    def test_calibrate_invalid_resistor(self):
        """无效电阻校准失败"""
        self.assertFalse(self.ina.calibrate(1.0, 0))
        self.assertFalse(self.ina.calibrate(1.0, -0.1))

    def test_calibrate_not_initialized(self):
        """未初始化校准失败"""
        ina2 = INA219()
        self.assertFalse(ina2.calibrate(1.0, 0.1))


class TestINA219Readings(unittest.TestCase):
    """INA219读数测试"""

    def setUp(self):
        self.ina = INA219()
        self.ina.init()
        self.ina.calibrate(3.2, 0.1)

    def test_read_bus_voltage(self):
        """读取总线电压"""
        self.ina.set_simulated(50.0, 12.0, 1.5, 18.0)
        self.assertAlmostEqual(self.ina.read_bus_voltage(), 12.0)

    def test_read_shunt_voltage(self):
        """读取分流电压"""
        self.ina.set_simulated(150.0, 5.0, 0.5, 2.5)
        self.assertAlmostEqual(self.ina.read_shunt_voltage(), 150.0)

    def test_read_current(self):
        """读取电流"""
        self.ina.set_simulated(100.0, 5.0, 2.0, 10.0)
        self.assertAlmostEqual(self.ina.read_current(), 2.0)

    def test_read_power(self):
        """读取功率"""
        self.ina.set_simulated(100.0, 12.0, 3.0, 36.0)
        self.assertAlmostEqual(self.ina.read_power(), 36.0)

    def test_zero_readings(self):
        """零值读数"""
        self.ina.set_simulated(0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(self.ina.read_bus_voltage(), 0.0)
        self.assertAlmostEqual(self.ina.read_current(), 0.0)
        self.assertAlmostEqual(self.ina.read_power(), 0.0)

    def test_not_initialized(self):
        """未初始化读取返回None"""
        ina2 = INA219()
        self.assertIsNone(ina2.read_bus_voltage())
        self.assertIsNone(ina2.read_shunt_voltage())
        self.assertIsNone(ina2.read_current())
        self.assertIsNone(ina2.read_power())


class TestPowerMonitorOvercurrent(unittest.TestCase):
    """功率监测过流保护测试"""

    def setUp(self):
        self.ina = INA219()
        self.ina.init()
        self.ina.calibrate(5.0, 0.1)
        self.CURRENT_LIMIT = 3.0  # 3A过流阈值

    def test_normal_current(self):
        """正常电流不触发保护"""
        self.ina.set_simulated(50.0, 12.0, 1.5, 18.0)
        current = self.ina.read_current()
        self.assertLess(current, self.CURRENT_LIMIT)

    def test_overcurrent_detected(self):
        """过流检测"""
        self.ina.set_simulated(400.0, 12.0, 4.0, 48.0)
        current = self.ina.read_current()
        self.assertGreater(current, self.CURRENT_LIMIT)

    def test_power_calculation_consistency(self):
        """功率计算一致性检查"""
        bus_v = 12.0
        current = 2.0
        expected_power = bus_v * current
        self.ina.set_simulated(200.0, bus_v, current, expected_power)
        self.assertAlmostEqual(self.ina.read_power(), expected_power)

    def test_multi_channel_monitoring(self):
        """多通道功率监测"""
        # 创建两个INA219用于不同通道
        ina_ch1 = INA219(INA219_ADDR_0)
        ina_ch2 = INA219(INA219_ADDR_1)
        ina_ch1.init()
        ina_ch2.init()
        ina_ch1.calibrate(3.0, 0.1)
        ina_ch2.calibrate(3.0, 0.1)

        # 通道1: 5V/1A, 通道2: 12V/0.5A
        ina_ch1.set_simulated(100.0, 5.0, 1.0, 5.0)
        ina_ch2.set_simulated(50.0, 12.0, 0.5, 6.0)

        total_power = ina_ch1.read_power() + ina_ch2.read_power()
        self.assertAlmostEqual(total_power, 11.0)


class TestADS1115Init(unittest.TestCase):
    """ADS1115初始化测试"""

    def test_init_success(self):
        """ADS1115初始化成功"""
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


class TestADS1115Config(unittest.TestCase):
    """ADS1115配置测试"""

    def setUp(self):
        self.adc = ADS1115()
        self.adc.init()

    def test_set_pga(self):
        """设置PGA增益"""
        self.assertTrue(self.adc.set_pga(ADS1115_PGA_4_096V))
        self.assertEqual(self.adc.pga, ADS1115_PGA_4_096V)

    def test_set_invalid_pga(self):
        """无效PGA失败"""
        self.assertFalse(self.adc.set_pga(0xFF))

    def test_get_pga_fsr(self):
        """获取PGA满量程电压"""
        self.adc.set_pga(ADS1115_PGA_2_048V)
        self.assertAlmostEqual(self.adc.get_pga_fsr(), 2.048)

    def test_set_mux(self):
        """设置输入多路复用器"""
        self.assertTrue(self.adc.set_mux(ADS1115_MUX_AIN0_GND))
        self.assertEqual(self.adc.mux, ADS1115_MUX_AIN0_GND)

    def test_set_invalid_mux(self):
        """无效MUX失败"""
        self.assertFalse(self.adc.set_mux(8))
        self.assertFalse(self.adc.set_mux(-1))

    def test_set_data_rate(self):
        """设置数据速率"""
        self.assertTrue(self.adc.set_data_rate(ADS1115_DR_860SPS))
        self.assertEqual(self.adc.data_rate, ADS1115_DR_860SPS)

    def test_set_invalid_data_rate(self):
        """无效数据速率失败"""
        self.assertFalse(self.adc.set_data_rate(8))


class TestADS1115Readings(unittest.TestCase):
    """ADS1115读数测试"""

    def setUp(self):
        self.adc = ADS1115()
        self.adc.init()
        self.adc.set_pga(ADS1115_PGA_2_048V)

    def test_read_voltage(self):
        """读取电压"""
        self.adc.set_simulated_raw(16384)  # 半量程
        voltage = self.adc.read_voltage()
        self.assertIsNotNone(voltage)
        # 半量程对应FSR/2
        self.assertAlmostEqual(voltage, 1.024, delta=0.1)

    def test_read_voltage_zero(self):
        """零电压读数"""
        self.adc.set_simulated_raw(0)
        voltage = self.adc.read_voltage()
        self.assertIsNotNone(voltage)
        self.assertAlmostEqual(voltage, 0.0, delta=0.01)

    def test_read_voltage_full_scale(self):
        """满量程电压读数"""
        self.adc.set_simulated_raw(32767)
        voltage = self.adc.read_voltage()
        self.assertIsNotNone(voltage)
        self.assertAlmostEqual(voltage, 2.048, delta=0.01)

    def test_read_raw(self):
        """读取原始ADC值"""
        self.adc.set_simulated_raw(12345)
        raw = self.adc.read_raw()
        self.assertEqual(raw, 12345)

    def test_raw_to_voltage(self):
        """原始值转电压"""
        voltage = self.adc.raw_to_voltage(0)
        self.assertAlmostEqual(voltage, 0.0)
        voltage = self.adc.raw_to_voltage(32767)
        self.assertAlmostEqual(voltage, 2.048, delta=0.01)

    def test_not_initialized(self):
        """未初始化读取返回None"""
        adc2 = ADS1115()
        self.assertIsNone(adc2.read_voltage())
        self.assertIsNone(adc2.read_raw())


class TestLCD1602Display(unittest.TestCase):
    """LCD1602显示测试"""

    def setUp(self):
        self.lcd = LCD1602()
        self.lcd.init()

    def test_init(self):
        """LCD1602初始化成功"""
        self.assertTrue(self.lcd.initialized)
        self.assertTrue(self.lcd.display_on)

    def test_clear(self):
        """清除显示"""
        self.lcd.write_string("Hello")
        self.lcd.clear()
        line = self.lcd.get_line(0)
        self.assertEqual(line.strip(), '')

    def test_write_string(self):
        """写入字符串"""
        count = self.lcd.write_string("12.0V 1.5A")
        self.assertEqual(count, 10)

    def test_print_line(self):
        """在指定行显示"""
        count = self.lcd.print_line(0, "Voltage: 12V")
        self.assertGreater(count, 0)
        line = self.lcd.get_line(0)
        self.assertIn('V', line)

    def test_set_cursor(self):
        """设置光标位置"""
        self.assertTrue(self.lcd.set_cursor(0, 5))
        self.assertEqual(self.lcd.cursor_row, 0)
        self.assertEqual(self.lcd.cursor_col, 5)

    def test_set_cursor_invalid(self):
        """无效光标位置失败"""
        self.assertFalse(self.lcd.set_cursor(2, 0))
        self.assertFalse(self.lcd.set_cursor(0, 16))

    def test_set_backlight(self):
        """设置背光"""
        self.lcd.set_backlight(False)
        self.assertFalse(self.lcd.backlight)
        self.lcd.set_backlight(True)
        self.assertTrue(self.lcd.backlight)

    def test_display_on_off(self):
        """显示开关"""
        self.lcd.display_on_off(False)
        self.assertFalse(self.lcd.display_on)
        self.lcd.display_on_off(True)
        self.assertTrue(self.lcd.display_on)

    def test_cursor_on_off(self):
        """光标开关"""
        self.lcd.cursor_on_off(True)
        self.assertTrue(self.lcd.cursor_on)

    def test_blink_on_off(self):
        """闪烁开关"""
        self.lcd.blink_on_off(True)
        self.assertTrue(self.lcd.blink_on)

    def test_home(self):
        """光标回原点"""
        self.lcd.set_cursor(1, 10)
        self.lcd.home()
        self.assertEqual(self.lcd.cursor_row, 0)
        self.assertEqual(self.lcd.cursor_col, 0)

    def test_get_display_text(self):
        """获取全部显示内容"""
        self.lcd.print_line(0, "Hello")
        self.lcd.print_line(1, "World")
        lines = self.lcd.get_display_text()
        self.assertEqual(len(lines), 2)
        self.assertIn('H', lines[0])
        self.assertIn('W', lines[1])

    def test_not_initialized(self):
        """未初始化操作失败"""
        lcd2 = LCD1602()
        self.assertFalse(lcd2.write_string("test"))
        self.assertFalse(lcd2.write_char('x'))


if __name__ == '__main__':
    unittest.main()
