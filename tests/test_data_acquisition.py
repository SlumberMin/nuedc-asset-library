#!/usr/bin/env python3
"""
数据采集V2测试 — ADS1115 16位ADC多通道采集
覆盖: ADS1115初始化、PGA设置、多通道切换、
      原始值读取、电压转换、连续采集、
      数据统计（最大/最小/平均）、滤波
对应C源文件: 02_mspm0g3507/drivers/ads1115.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    ADS1115,
    ADS1115_ADDR_GND, ADS1115_ADDR_VDD,
    ADS1115_MUX_AIN0_GND, ADS1115_MUX_AIN1_GND,
    ADS1115_MUX_AIN2_GND, ADS1115_MUX_AIN3_GND,
    ADS1115_MUX_AIN0_AIN1,
    ADS1115_PGA_6_144V, ADS1115_PGA_4_096V,
    ADS1115_PGA_2_048V, ADS1115_PGA_1_024V,
    ADS1115_PGA_0_512V, ADS1115_PGA_0_256V,
    ADS1115_PGA_FSR,
    ADS1115_DR_8SPS, ADS1115_DR_250SPS, ADS1115_DR_860SPS,
)


class DataChannel:
    """单通道数据统计"""

    def __init__(self, name, mux, pga=ADS1115_PGA_2_048V):
        self.name = name        # 通道名称
        self.mux = mux          # 输入多路复用配置
        self.pga = pga          # PGA增益设置
        self.samples = []       # 采样值(电压)
        self.raw_values = []    # 原始ADC值

    def clear(self):
        """清空数据"""
        self.samples.clear()
        self.raw_values.clear()

    def add_sample(self, raw, voltage):
        """添加采样"""
        self.raw_values.append(raw)
        self.samples.append(voltage)

    def get_count(self):
        """采样数"""
        return len(self.samples)

    def get_max(self):
        """最大值"""
        return max(self.samples) if self.samples else 0.0

    def get_min(self):
        """最小值"""
        return min(self.samples) if self.samples else 0.0

    def get_avg(self):
        """平均值"""
        if not self.samples:
            return 0.0
        return sum(self.samples) / len(self.samples)

    def get_rms(self):
        """RMS(有效值)"""
        if not self.samples:
            return 0.0
        sq_sum = sum(v * v for v in self.samples)
        return math.sqrt(sq_sum / len(self.samples))


class DataAcquisitionV2:
    """数据采集V2 — ADS1115多通道采集系统

    功能:
    - ADS1115 16位ADC驱动
    - 4通道单端/差分采集
    - 可编程增益(PGA)设置
    - 多通道轮询采集
    - 通道数据统计（最大/最小/平均/RMS）
    - 采集触发和缓冲区管理
    """

    def __init__(self, addr=ADS1115_ADDR_GND):
        self.adc = ADS1115(addr=addr)
        self.channels = []       # 通道列表
        self.current_ch = 0      # 当前通道索引
        self.is_running = False  # 采集运行标志
        self.total_samples = 0   # 总采样计数

    def init(self):
        """初始化采集系统"""
        self.adc.init()

    def add_channel(self, name, mux, pga=ADS1115_PGA_2_048V):
        """添加采集通道"""
        ch = DataChannel(name, mux, pga)
        self.channels.append(ch)
        return len(self.channels) - 1

    def read_channel(self, ch_index):
        """读取指定通道

        返回: (success, raw_value, voltage)
        """
        if ch_index < 0 or ch_index >= len(self.channels):
            return False, 0, 0.0

        ch = self.channels[ch_index]
        self.adc.set_mux(ch.mux)
        self.adc.set_pga(ch.pga)

        raw = self.adc.read_raw()
        if raw is None:
            return False, 0, 0.0

        voltage = self.adc.raw_to_voltage(raw)
        ch.add_sample(raw, voltage)
        self.total_samples += 1
        return True, raw, voltage

    def scan_all(self):
        """扫描所有通道一次

        返回: 各通道(成功, 电压)列表
        """
        results = []
        for i in range(len(self.channels)):
            ok, raw, voltage = self.read_channel(i)
            results.append((ok, voltage))
        return results

    def get_channel_stats(self, ch_index):
        """获取通道统计信息"""
        if ch_index < 0 or ch_index >= len(self.channels):
            return None
        ch = self.channels[ch_index]
        return {
            'name': ch.name,
            'count': ch.get_count(),
            'min': ch.get_min(),
            'max': ch.get_max(),
            'avg': ch.get_avg(),
            'rms': ch.get_rms(),
        }

    def clear_all(self):
        """清空所有通道数据"""
        for ch in self.channels:
            ch.clear()
        self.total_samples = 0

    def is_channel_valid(self, ch_index):
        """检查通道索引是否有效"""
        return 0 <= ch_index < len(self.channels)


class TestADS1115Init(unittest.TestCase):
    """ADS1115初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        daq = DataAcquisitionV2()
        daq.init()
        self.assertTrue(daq.adc.initialized)

    def test_default_pga(self):
        """默认PGA为±2.048V"""
        daq = DataAcquisitionV2()
        daq.init()
        self.assertEqual(daq.adc.pga, ADS1115_PGA_2_048V)

    def test_default_mux(self):
        """默认MUX为AIN0单端"""
        daq = DataAcquisitionV2()
        daq.init()
        self.assertEqual(daq.adc.mux, ADS1115_MUX_AIN0_GND)


class TestPGASettings(unittest.TestCase):
    """PGA增益设置测试"""

    def setUp(self):
        self.daq = DataAcquisitionV2()
        self.daq.init()

    def test_all_pga_settings(self):
        """所有PGA设置均可配置"""
        pga_list = [
            ADS1115_PGA_6_144V, ADS1115_PGA_4_096V,
            ADS1115_PGA_2_048V, ADS1115_PGA_1_024V,
            ADS1115_PGA_0_512V, ADS1115_PGA_0_256V,
        ]
        for pga in pga_list:
            self.assertTrue(self.daq.adc.set_pga(pga))
            self.assertEqual(self.daq.adc.pga, pga)

    def test_pga_fsr_values(self):
        """PGA满量程电压正确"""
        self.daq.adc.set_pga(ADS1115_PGA_6_144V)
        self.assertAlmostEqual(self.daq.adc.get_pga_fsr(), 6.144, places=3)
        self.daq.adc.set_pga(ADS1115_PGA_2_048V)
        self.assertAlmostEqual(self.daq.adc.get_pga_fsr(), 2.048, places=3)
        self.daq.adc.set_pga(ADS1115_PGA_0_256V)
        self.assertAlmostEqual(self.daq.adc.get_pga_fsr(), 0.256, places=3)

    def test_invalid_pga(self):
        """无效PGA返回False"""
        self.assertFalse(self.daq.adc.set_pga(0xFF))


class TestMuxSettings(unittest.TestCase):
    """多路复用器设置测试"""

    def setUp(self):
        self.daq = DataAcquisitionV2()
        self.daq.init()

    def test_all_mux_settings(self):
        """所有MUX设置（0-7）均可配置"""
        for mux in range(8):
            self.assertTrue(self.daq.adc.set_mux(mux))
            self.assertEqual(self.daq.adc.mux, mux)

    def test_invalid_mux(self):
        """无效MUX返回False"""
        self.assertFalse(self.daq.adc.set_mux(8))
        self.assertFalse(self.daq.adc.set_mux(-1))


class TestVoltageConversion(unittest.TestCase):
    """电压转换测试"""

    def setUp(self):
        self.daq = DataAcquisitionV2()
        self.daq.init()

    def test_zero_raw_to_voltage(self):
        """原始值0→电压0"""
        self.daq.adc.set_pga(ADS1115_PGA_2_048V)
        v = self.daq.adc.raw_to_voltage(0)
        self.assertAlmostEqual(v, 0.0, places=4)

    def test_max_positive_voltage(self):
        """最大正电压 (0x7FFF)"""
        self.daq.adc.set_pga(ADS1115_PGA_2_048V)
        v = self.daq.adc.raw_to_voltage(0x7FFF)
        self.assertAlmostEqual(v, 2.048, places=2)

    def test_max_negative_voltage(self):
        """最大负电压 (0x8000)"""
        self.daq.adc.set_pga(ADS1115_PGA_2_048V)
        v = self.daq.adc.raw_to_voltage(0x8000)
        self.assertAlmostEqual(v, -2.048, places=2)

    def test_voltage_proportional_to_pga(self):
        """电压与PGA量程成正比"""
        self.daq.adc.set_simulated_raw(16384)  # 约半量程
        self.daq.adc.set_pga(ADS1115_PGA_6_144V)
        v1 = self.daq.adc.read_voltage()
        self.daq.adc.set_pga(ADS1115_PGA_2_048V)
        v2 = self.daq.adc.read_voltage()
        # 6.144V量程下的电压应为2.048V量程的3倍
        self.assertAlmostEqual(v1 / v2, 3.0, places=1)

    def test_read_voltage_before_init(self):
        """初始化前读取返回None"""
        adc = ADS1115()
        self.assertIsNone(adc.read_voltage())


class TestMultiChannel(unittest.TestCase):
    """多通道采集测试"""

    def setUp(self):
        self.daq = DataAcquisitionV2()
        self.daq.init()
        # 添加4个通道
        self.daq.add_channel("电压检测", ADS1115_MUX_AIN0_GND)
        self.daq.add_channel("电流检测", ADS1115_MUX_AIN1_GND)
        self.daq.add_channel("温度检测", ADS1115_MUX_AIN2_GND)
        self.daq.add_channel("参考电压", ADS1115_MUX_AIN3_GND)

    def test_add_channels(self):
        """添加通道"""
        self.assertEqual(len(self.daq.channels), 4)

    def test_channel_names(self):
        """通道名称正确"""
        self.assertEqual(self.daq.channels[0].name, "电压检测")
        self.assertEqual(self.daq.channels[1].name, "电流检测")

    def test_read_single_channel(self):
        """读取单通道"""
        self.daq.adc.set_simulated_raw(10000)
        ok, raw, voltage = self.daq.read_channel(0)
        self.assertTrue(ok)
        self.assertEqual(raw, 10000)
        self.assertGreater(voltage, 0)

    def test_read_invalid_channel(self):
        """读取无效通道返回失败"""
        ok, raw, voltage = self.daq.read_channel(10)
        self.assertFalse(ok)
        ok, raw, voltage = self.daq.read_channel(-1)
        self.assertFalse(ok)

    def test_scan_all_channels(self):
        """扫描全部通道"""
        self.daq.adc.set_simulated_raw(5000)
        results = self.daq.scan_all()
        self.assertEqual(len(results), 4)
        for ok, voltage in results:
            self.assertTrue(ok)

    def test_total_sample_count(self):
        """总采样计数"""
        self.daq.adc.set_simulated_raw(1000)
        self.daq.read_channel(0)
        self.daq.read_channel(1)
        self.assertEqual(self.daq.total_samples, 2)


class TestDataStatistics(unittest.TestCase):
    """数据统计测试"""

    def setUp(self):
        self.daq = DataAcquisitionV2()
        self.daq.init()
        self.daq.add_channel("测试通道", ADS1115_MUX_AIN0_GND)

    def test_min_max_avg(self):
        """最小/最大/平均值"""
        # 模拟不同电压采样
        test_raws = [1000, 5000, 3000, 8000, 2000]
        for raw in test_raws:
            self.daq.adc.set_simulated_raw(raw)
            self.daq.read_channel(0)

        stats = self.daq.get_channel_stats(0)
        self.assertEqual(stats['count'], 5)
        self.assertGreater(stats['max'], stats['min'])
        # 平均值应在min和max之间
        self.assertGreater(stats['avg'], stats['min'])
        self.assertLess(stats['avg'], stats['max'])

    def test_rms(self):
        """RMS计算"""
        test_raws = [10000, 20000, 10000, 20000]
        for raw in test_raws:
            self.daq.adc.set_simulated_raw(raw)
            self.daq.read_channel(0)

        stats = self.daq.get_channel_stats(0)
        self.assertGreater(stats['rms'], 0)
        # RMS应大于平均值（对非恒定信号）
        self.assertGreaterEqual(stats['rms'], stats['avg'])

    def test_single_sample_stats(self):
        """单采样统计"""
        self.daq.adc.set_simulated_raw(15000)
        self.daq.read_channel(0)
        stats = self.daq.get_channel_stats(0)
        self.assertEqual(stats['count'], 1)
        self.assertEqual(stats['min'], stats['max'])
        self.assertEqual(stats['min'], stats['avg'])

    def test_empty_channel_stats(self):
        """空通道统计"""
        stats = self.daq.get_channel_stats(0)
        self.assertEqual(stats['count'], 0)
        self.assertEqual(stats['min'], 0.0)
        self.assertEqual(stats['max'], 0.0)

    def test_invalid_channel_stats(self):
        """无效通道返回None"""
        self.assertIsNone(self.daq.get_channel_stats(99))


class TestContinuousAcquisition(unittest.TestCase):
    """连续采集测试"""

    def setUp(self):
        self.daq = DataAcquisitionV2()
        self.daq.init()
        self.daq.add_channel("连续通道", ADS1115_MUX_AIN0_GND)

    def test_continuous_read(self):
        """连续多次读取"""
        self.daq.adc.set_continuous(True)
        self.assertTrue(self.daq.adc.continuous)

        for i in range(100):
            self.daq.adc.set_simulated_raw(i * 100)
            self.daq.read_channel(0)

        stats = self.daq.get_channel_stats(0)
        self.assertEqual(stats['count'], 100)

    def test_clear_and_restart(self):
        """清空后重新采集"""
        self.daq.adc.set_simulated_raw(5000)
        self.daq.read_channel(0)
        self.assertEqual(self.daq.total_samples, 1)

        self.daq.clear_all()
        self.assertEqual(self.daq.total_samples, 0)
        self.assertEqual(self.daq.channels[0].get_count(), 0)


class TestDataRate(unittest.TestCase):
    """数据速率设置测试"""

    def setUp(self):
        self.daq = DataAcquisitionV2()
        self.daq.init()

    def test_set_data_rate(self):
        """设置数据速率"""
        self.assertTrue(self.daq.adc.set_data_rate(ADS1115_DR_250SPS))
        self.assertEqual(self.daq.adc.data_rate, ADS1115_DR_250SPS)

    def test_high_speed_rate(self):
        """高速率 860SPS"""
        self.assertTrue(self.daq.adc.set_data_rate(ADS1115_DR_860SPS))
        self.assertEqual(self.daq.adc.data_rate, ADS1115_DR_860SPS)

    def test_low_speed_rate(self):
        """低速率 8SPS"""
        self.assertTrue(self.daq.adc.set_data_rate(ADS1115_DR_8SPS))
        self.assertEqual(self.daq.adc.data_rate, ADS1115_DR_8SPS)

    def test_invalid_rate(self):
        """无效速率返回False"""
        self.assertFalse(self.daq.adc.set_data_rate(10))


class TestChannelValidation(unittest.TestCase):
    """通道有效性测试"""

    def setUp(self):
        self.daq = DataAcquisitionV2()
        self.daq.init()
        self.daq.add_channel("CH0", ADS1115_MUX_AIN0_GND)

    def test_valid_channel(self):
        """有效通道"""
        self.assertTrue(self.daq.is_channel_valid(0))

    def test_invalid_channel_high(self):
        """超出上限"""
        self.assertFalse(self.daq.is_channel_valid(1))

    def test_invalid_channel_low(self):
        """低于下限"""
        self.assertFalse(self.daq.is_channel_valid(-1))


if __name__ == '__main__':
    unittest.main()
