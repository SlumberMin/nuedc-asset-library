#!/usr/bin/env python3
"""
多路ADC采样V2测试 — 基于wrappers.py包装层
覆盖: MultiADC内部ADC + MCP3421高精度ADC
模拟场景: 多通道采样、过采样滤波、电压转换、统计分析
对应C源文件: 02_mspm0g3507/examples/multi_adc_sampler.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    MultiADC, ADC_CHANNELS, ADC_RESOLUTION, VREF_MV,
    MCP3421, MCP3421_ADDR,
    MCP3421_12BIT, MCP3421_14BIT, MCP3421_16BIT, MCP3421_18BIT,
    MCP3421_GAIN_1X, MCP3421_GAIN_2X, MCP3421_GAIN_4X, MCP3421_GAIN_8X,
)


class TestMultiADCInit(unittest.TestCase):
    """多路ADC初始化测试"""

    def test_init_success(self):
        """ADC初始化成功"""
        adc = MultiADC()
        self.assertTrue(adc.init())
        self.assertTrue(adc.initialized)

    def test_default_config(self):
        """默认配置：4通道、12位、3.3V参考"""
        adc = MultiADC()
        self.assertEqual(adc.channels, ADC_CHANNELS)
        self.assertEqual(adc.resolution, ADC_RESOLUTION)
        self.assertEqual(adc.vref_mv, VREF_MV)

    def test_custom_channels(self):
        """自定义通道数"""
        adc = MultiADC(channels=8, resolution=4096)
        self.assertEqual(adc.channels, 8)


class TestMultiADCRead(unittest.TestCase):
    """多路ADC读取测试"""

    def setUp(self):
        self.adc = MultiADC()
        self.adc.init()

    def test_read_raw(self):
        """读取原始ADC值"""
        self.adc.set_simulated_raw(0, 2048)
        self.assertEqual(self.adc.read_raw(0), 2048)

    def test_read_raw_all_channels(self):
        """读取所有通道"""
        for ch in range(ADC_CHANNELS):
            self.adc.set_simulated_raw(ch, ch * 1000)
        for ch in range(ADC_CHANNELS):
            self.assertEqual(self.adc.read_raw(ch), ch * 1000)

    def test_read_raw_not_initialized(self):
        """未初始化读取返回None"""
        adc2 = MultiADC()
        self.assertIsNone(adc2.read_raw(0))

    def test_read_raw_invalid_channel(self):
        """无效通道返回None"""
        self.assertIsNone(self.adc.read_raw(-1))
        self.assertIsNone(self.adc.read_raw(ADC_CHANNELS))

    def test_voltage_conversion(self):
        """原始值转电压：mid-scale = 1650mV"""
        # 半量程: 2048 * 3300 / 4096 = 1650mV
        self.assertAlmostEqual(self.adc.raw_to_voltage_mv(2048), 1650.0)

    def test_voltage_zero(self):
        """零值电压"""
        self.assertAlmostEqual(self.adc.raw_to_voltage_mv(0), 0.0)

    def test_voltage_full_scale(self):
        """满量程电压"""
        self.assertAlmostEqual(self.adc.raw_to_voltage_mv(4095), 3300.0, delta=1.0)

    def test_read_voltage_mv(self):
        """读取通道电压"""
        self.adc.set_simulated_raw(0, 2048)
        self.assertAlmostEqual(self.adc.read_voltage_mv(0), 1650.0)

    def test_voltage_not_initialized(self):
        """未初始化读取电压返回None"""
        adc2 = MultiADC()
        self.assertIsNone(adc2.read_voltage_mv(0))


class TestMultiADCFilter(unittest.TestCase):
    """多路ADC滤波器测试"""

    def setUp(self):
        self.adc = MultiADC()
        self.adc.init()
        self.adc.filter_depth = 4

    def test_filter_stabilizes(self):
        """滤波器收敛到常数值"""
        self.adc.set_simulated_raw(0, 2048)
        for _ in range(10):
            filtered = self.adc.read_filtered_mv(0)
        self.assertAlmostEqual(filtered, 1650.0, delta=1.0)

    def test_filter_smoothing(self):
        """滤波器平滑噪声"""
        # 先预热滤波器到稳态
        self.adc.set_simulated_raw(0, int(2050 * 4096 / 3300))
        for _ in range(10):
            self.adc.read_filtered_mv(0)
        # 交替输入高/低值
        values = [2100, 2000, 2100, 2000, 2100, 2000]
        results = []
        for v in values:
            self.adc.set_simulated_raw(0, int(v * 4096 / 3300))
            results.append(self.adc.read_filtered_mv(0))
        # 滤波后的值应比原始波动小
        raw_range = max(values) - min(values)
        filtered_range = max(results) - min(results)
        self.assertLessEqual(filtered_range, raw_range)


class TestMultiADCStatistics(unittest.TestCase):
    """多路ADC统计测试"""

    def setUp(self):
        self.adc = MultiADC()
        self.adc.init()

    def test_min_max_tracking(self):
        """最小/最大值跟踪"""
        self.adc.set_simulated_raw(0, 1000)
        self.adc.process_sample()
        self.adc.set_simulated_raw(0, 3000)
        self.adc.process_sample()
        self.adc.set_simulated_raw(0, 2000)
        self.adc.process_sample()

        min_mv = self.adc.get_min_mv(0)
        max_mv = self.adc.get_max_mv(0)
        self.assertAlmostEqual(min_mv, self.adc.raw_to_voltage_mv(1000), delta=1.0)
        self.assertAlmostEqual(max_mv, self.adc.raw_to_voltage_mv(3000), delta=1.0)

    def test_sample_count(self):
        """采样计数"""
        self.adc.set_simulated_raw(0, 2048)
        for i in range(5):
            self.adc.process_sample()
        self.assertEqual(self.adc.get_sample_count(0), 5)

    def test_oversample_config(self):
        """过采样配置"""
        self.assertTrue(self.adc.set_oversample_count(32))
        self.assertEqual(self.adc.oversample_count, 32)

    def test_oversample_min(self):
        """过采样次数最小为1"""
        self.adc.set_oversample_count(0)
        self.assertEqual(self.adc.oversample_count, 1)


class TestMCP3421Init(unittest.TestCase):
    """MCP3421高精度ADC初始化测试"""

    def test_init_success(self):
        """MCP3421初始化成功"""
        adc = MCP3421()
        self.assertTrue(adc.init())
        self.assertTrue(adc.initialized)

    def test_default_address(self):
        """默认I2C地址0x6E"""
        adc = MCP3421()
        self.assertEqual(adc.addr, MCP3421_ADDR)

    def test_default_config(self):
        """默认配置：18位、1x增益"""
        adc = MCP3421()
        self.assertEqual(adc.get_resolution_bits(), 18)


class TestMCP3421Config(unittest.TestCase):
    """MCP3421配置测试"""

    def setUp(self):
        self.adc = MCP3421()
        self.adc.init()

    def test_set_resolution_12bit(self):
        """设置12位分辨率"""
        self.assertTrue(self.adc.set_resolution(MCP3421_12BIT))
        self.assertEqual(self.adc.get_resolution_bits(), 12)

    def test_set_resolution_14bit(self):
        """设置14位分辨率"""
        self.assertTrue(self.adc.set_resolution(MCP3421_14BIT))
        self.assertEqual(self.adc.get_resolution_bits(), 14)

    def test_set_resolution_16bit(self):
        """设置16位分辨率"""
        self.assertTrue(self.adc.set_resolution(MCP3421_16BIT))
        self.assertEqual(self.adc.get_resolution_bits(), 16)

    def test_set_resolution_18bit(self):
        """设置18位分辨率"""
        self.assertTrue(self.adc.set_resolution(MCP3421_18BIT))
        self.assertEqual(self.adc.get_resolution_bits(), 18)

    def test_set_gain_1x(self):
        """设置1x增益"""
        self.assertTrue(self.adc.set_gain(MCP3421_GAIN_1X))
        self.assertEqual(self.adc.config & 0x03, MCP3421_GAIN_1X)

    def test_set_gain_8x(self):
        """设置8x增益"""
        self.assertTrue(self.adc.set_gain(MCP3421_GAIN_8X))
        self.assertEqual(self.adc.config & 0x03, MCP3421_GAIN_8X)

    def test_invalid_resolution(self):
        """无效分辨率失败"""
        self.assertFalse(self.adc.set_resolution(0xFF))

    def test_invalid_gain(self):
        """无效增益失败"""
        self.assertFalse(self.adc.set_gain(0xFF))


class TestMCP3421Readings(unittest.TestCase):
    """MCP3421读数测试"""

    def setUp(self):
        self.adc = MCP3421()
        self.adc.init()
        self.adc.set_resolution(MCP3421_18BIT)
        self.adc.set_gain(MCP3421_GAIN_1X)

    def test_lsb_18bit(self):
        """18位分辨率LSB = 15.625uV"""
        self.assertAlmostEqual(self.adc.get_lsb_uv(), 15.625)

    def test_lsb_16bit(self):
        """16位分辨率LSB = 62.5uV"""
        self.adc.set_resolution(MCP3421_16BIT)
        self.assertAlmostEqual(self.adc.get_lsb_uv(), 62.5)

    def test_lsb_12bit(self):
        """12位分辨率LSB = 1000uV"""
        self.adc.set_resolution(MCP3421_12BIT)
        self.assertAlmostEqual(self.adc.get_lsb_uv(), 1000.0)

    def test_lsb_with_gain(self):
        """增益影响LSB：8x增益时LSB减小"""
        self.adc.set_gain(MCP3421_GAIN_8X)
        self.assertAlmostEqual(self.adc.get_lsb_uv(), 15.625 / 8)

    def test_read_voltage(self):
        """读取电压"""
        self.adc.set_simulated_raw(1000)
        voltage = self.adc.read_voltage_mv()
        self.assertAlmostEqual(voltage, 1000 * 15.625 / 1000.0)

    def test_read_raw(self):
        """读取原始值"""
        self.adc.set_simulated_raw(12345)
        self.assertEqual(self.adc.read_raw(), 12345)

    def test_not_initialized(self):
        """未初始化读取返回None"""
        adc2 = MCP3421()
        self.assertIsNone(adc2.read_voltage_mv())
        self.assertIsNone(adc2.read_raw())

    def test_is_ready(self):
        """数据就绪状态"""
        self.assertTrue(self.adc.is_ready())


class TestMultiADCSync(unittest.TestCase):
    """多路ADC同步采样场景测试"""

    def test_multi_channel_scenario(self):
        """模拟4通道同步采样场景"""
        adc = MultiADC()
        adc.init()

        # 模拟4通道不同电压
        test_values = [1000, 2000, 3000, 4000]
        for ch, raw in enumerate(test_values):
            adc.set_simulated_raw(ch, raw)

        # 验证各通道独立读取
        for ch, raw in enumerate(test_values):
            self.assertEqual(adc.read_raw(ch), raw)
            expected_mv = raw * VREF_MV / ADC_RESOLUTION
            self.assertAlmostEqual(adc.read_voltage_mv(ch), expected_mv, delta=1.0)


if __name__ == '__main__':
    unittest.main()
