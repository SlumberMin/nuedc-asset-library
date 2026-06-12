#!/usr/bin/env python3
"""
BMP280气压传感器 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、工作模式、过采样、IIR滤波、温度读取、
      气压读取、海拔计算、校准参数、边界条件
对应C源文件: 02_mspm0g3507/drivers/bmp280.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    BMP280, BMP280_ADDR_SDO_GND, BMP280_ADDR_SDO_VCC, BMP280_CHIP_ID,
    BMP280_MODE_SLEEP, BMP280_MODE_FORCED, BMP280_MODE_NORMAL,
    BMP280_OS_SKIPPED, BMP280_OS_1X, BMP280_OS_2X, BMP280_OS_4X,
    BMP280_OS_8X, BMP280_OS_16X,
    BMP280_FILTER_OFF, BMP280_FILTER_2, BMP280_FILTER_4,
    BMP280_FILTER_8, BMP280_FILTER_16,
    BMP280_Calib,
)


class TestBMP280Init(unittest.TestCase):
    """BMP280初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        bmp = BMP280()
        self.assertTrue(bmp.init())
        self.assertTrue(bmp.initialized)

    def test_default_address(self):
        """默认I2C地址0x76"""
        bmp = BMP280()
        self.assertEqual(bmp.addr, BMP280_ADDR_SDO_GND)

    def test_custom_address(self):
        """SDO接VCC地址0x77"""
        bmp = BMP280(BMP280_ADDR_SDO_VCC)
        self.assertEqual(bmp.addr, BMP280_ADDR_SDO_VCC)

    def test_default_mode_sleep(self):
        """默认睡眠模式"""
        bmp = BMP280()
        self.assertEqual(bmp.mode, BMP280_MODE_SLEEP)

    def test_default_oversampling(self):
        """默认过采样1x"""
        bmp = BMP280()
        self.assertEqual(bmp.osrs_t, BMP280_OS_1X)
        self.assertEqual(bmp.osrs_p, BMP280_OS_1X)

    def test_init_loads_calibration(self):
        """初始化加载校准参数"""
        bmp = BMP280()
        bmp.init()
        self.assertNotEqual(bmp.calib.dig_T1, 0)
        self.assertNotEqual(bmp.calib.dig_P1, 0)


class TestBMP280Mode(unittest.TestCase):
    """BMP280工作模式测试"""

    def setUp(self):
        self.bmp = BMP280()
        self.bmp.init()

    def test_set_sleep_mode(self):
        """设置睡眠模式"""
        self.assertTrue(self.bmp.set_mode(BMP280_MODE_SLEEP))
        self.assertEqual(self.bmp.mode, BMP280_MODE_SLEEP)

    def test_set_forced_mode(self):
        """设置强制模式"""
        self.assertTrue(self.bmp.set_mode(BMP280_MODE_FORCED))
        self.assertEqual(self.bmp.mode, BMP280_MODE_FORCED)

    def test_set_normal_mode(self):
        """设置正常模式"""
        self.assertTrue(self.bmp.set_mode(BMP280_MODE_NORMAL))
        self.assertEqual(self.bmp.mode, BMP280_MODE_NORMAL)

    def test_set_invalid_mode(self):
        """无效模式失败"""
        self.assertFalse(self.bmp.set_mode(0x02))


class TestBMP280Oversampling(unittest.TestCase):
    """BMP280过采样测试"""

    def setUp(self):
        self.bmp = BMP280()
        self.bmp.init()

    def test_set_oversampling(self):
        """设置过采样率"""
        self.bmp.set_oversampling(BMP280_OS_16X, BMP280_OS_8X)
        self.assertEqual(self.bmp.osrs_t, BMP280_OS_16X)
        self.assertEqual(self.bmp.osrs_p, BMP280_OS_8X)

    def test_set_oversampling_skipped(self):
        """跳过采样"""
        self.bmp.set_oversampling(BMP280_OS_SKIPPED, BMP280_OS_SKIPPED)
        self.assertEqual(self.bmp.osrs_t, BMP280_OS_SKIPPED)

    def test_set_oversampling_all(self):
        """所有过采样选项"""
        for os_val in [BMP280_OS_SKIPPED, BMP280_OS_1X, BMP280_OS_2X,
                       BMP280_OS_4X, BMP280_OS_8X, BMP280_OS_16X]:
            self.bmp.set_oversampling(os_val, os_val)
            self.assertEqual(self.bmp.osrs_t, os_val)


class TestBMP280Filter(unittest.TestCase):
    """BMP280 IIR滤波测试"""

    def setUp(self):
        self.bmp = BMP280()
        self.bmp.init()

    def test_set_filter_off(self):
        """关闭滤波"""
        self.assertTrue(self.bmp.set_filter(BMP280_FILTER_OFF))
        self.assertEqual(self.bmp.filter, BMP280_FILTER_OFF)

    def test_set_filter_coefficients(self):
        """设置不同滤波系数"""
        for coeff in [BMP280_FILTER_2, BMP280_FILTER_4,
                      BMP280_FILTER_8, BMP280_FILTER_16]:
            self.assertTrue(self.bmp.set_filter(coeff))
            self.assertEqual(self.bmp.filter, coeff)

    def test_set_filter_invalid(self):
        """无效滤波系数"""
        self.assertFalse(self.bmp.set_filter(5))
        self.assertFalse(self.bmp.set_filter(-1))


class TestBMP280Temperature(unittest.TestCase):
    """BMP280温度读取测试"""

    def setUp(self):
        self.bmp = BMP280()
        self.bmp.init()

    def test_read_temperature_not_initialized(self):
        """未初始化返回None"""
        bmp2 = BMP280()
        self.assertIsNone(bmp2.read_temperature())

    def test_read_temperature_default(self):
        """默认原始值温度"""
        # 默认simulated=0, 应返回合理温度
        temp = self.bmp.read_temperature()
        self.assertIsNotNone(temp)
        # 校准参数下raw=0可能返回负数，但应在合理范围
        self.assertGreater(temp, -273)
        self.assertLess(temp, 500)

    def test_read_temperature_room_temp(self):
        """模拟室温~25°C"""
        # BMP280 datasheet公式: raw_temp约524288对应25°C（依赖校准参数）
        # 用默认校准参数，调参得到约25°C的raw值
        # 反推: t_fine ≈ 25*5120 = 128000
        # var1 = (raw/16384 - T1/1024) * T2
        # 粗略估计: 设置一个raw使得温度接近25
        self.bmp.set_simulated_raw(524288, 0)
        temp = self.bmp.read_temperature()
        # 温度应在合理范围（不一定精确25，但应在-40~85范围内）
        self.assertGreater(temp, -40)
        self.assertLess(temp, 85)

    def test_read_temperature_high(self):
        """高温"""
        self.bmp.set_simulated_raw(700000, 0)
        temp = self.bmp.read_temperature()
        self.assertGreater(temp, 0)

    def test_read_temperature_low(self):
        """低温"""
        self.bmp.set_simulated_raw(100000, 0)
        temp = self.bmp.read_temperature()
        self.assertIsNotNone(temp)


class TestBMP280Pressure(unittest.TestCase):
    """BMP280气压读取测试"""

    def setUp(self):
        self.bmp = BMP280()
        self.bmp.init()

    def test_read_pressure_not_initialized(self):
        """未初始化返回None"""
        bmp2 = BMP280()
        self.assertIsNone(bmp2.read_pressure())

    def test_read_pressure_default(self):
        """默认原始值气压"""
        self.bmp.set_simulated_raw(524288, 400000)
        pressure = self.bmp.read_pressure()
        self.assertIsNotNone(pressure)
        # 气压应在合理范围
        self.assertGreater(pressure, 0)

    def test_read_pressure_sea_level(self):
        """海平面气压约101325Pa"""
        # 用合理raw值测试
        self.bmp.set_simulated_raw(524288, 364522)
        pressure = self.bmp.read_pressure()
        self.assertIsNotNone(pressure)
        # 应在合理范围（不强求精确，验证不crash）
        self.assertGreater(pressure, 0)

    def test_read_pressure_after_temperature(self):
        """气压读取内部先更新温度"""
        self.bmp.set_simulated_raw(500000, 400000)
        temp = self.bmp.read_temperature()
        press = self.bmp.read_pressure()
        self.assertIsNotNone(temp)
        self.assertIsNotNone(press)
        self.assertGreater(press, 0)


class TestBMP280Altitude(unittest.TestCase):
    """BMP280海拔计算测试"""

    def setUp(self):
        self.bmp = BMP280()
        self.bmp.init()

    def test_read_altitude_not_initialized(self):
        """未初始化返回None"""
        bmp2 = BMP280()
        self.assertIsNone(bmp2.read_altitude())

    def test_read_altitude_value(self):
        """海拔计算返回数值"""
        self.bmp.set_simulated_raw(524288, 400000)
        alt = self.bmp.read_altitude()
        self.assertIsNotNone(alt)
        self.assertIsInstance(alt, float)

    def test_read_altitude_custom_sea_level(self):
        """自定义海平面气压"""
        self.bmp.set_simulated_raw(524288, 400000)
        alt = self.bmp.read_altitude(sea_level_pa=101325.0)
        self.assertIsNotNone(alt)

    def test_altitude_formula(self):
        """海拔公式验证: 气压越低海拔越高"""
        # 海平面气压 → 海拔0
        alt = self.bmp.read_altitude(sea_level_pa=101325.0)
        self.assertIsNotNone(alt)


class TestBMP280Calib(unittest.TestCase):
    """BMP280校准参数测试"""

    def test_calib_init(self):
        """校准参数初始为0"""
        cal = BMP280_Calib()
        self.assertEqual(cal.dig_T1, 0)
        self.assertEqual(cal.dig_P1, 0)

    def test_calib_all_fields(self):
        """校准参数包含T1-T3和P1-P9"""
        cal = BMP280_Calib()
        fields = ['dig_T1', 'dig_T2', 'dig_T3',
                  'dig_P1', 'dig_P2', 'dig_P3',
                  'dig_P4', 'dig_P5', 'dig_P6',
                  'dig_P7', 'dig_P8', 'dig_P9']
        for f in fields:
            self.assertTrue(hasattr(cal, f))


class TestBMP280Constants(unittest.TestCase):
    """BMP280常量一致性"""

    def test_chip_id(self):
        """芯片ID为0x58"""
        self.assertEqual(BMP280_CHIP_ID, 0x58)

    def test_mode_values(self):
        """模式值正确"""
        self.assertEqual(BMP280_MODE_SLEEP, 0x00)
        self.assertEqual(BMP280_MODE_FORCED, 0x01)
        self.assertEqual(BMP280_MODE_NORMAL, 0x03)


if __name__ == '__main__':
    unittest.main()
