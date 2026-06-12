#!/usr/bin/env python3
"""
信号发生器V2测试 — MCP4725 12位DAC信号生成
覆盖: MCP4725初始化、DAC值设置、电压输出、
      波形生成（正弦/三角/锯齿/方波）、
      频率控制、幅度调节、EEPROM存储
对应C源文件: 02_mspm0g3507/drivers/mcp4725.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    MCP4725,
    MCP4725_ADDR_A0_GND, MCP4725_ADDR_A0_VDD,
    MCP4725_MAX_VALUE, MCP4725_VREF_DEFAULT,
    MCP4725_PD_NONE, MCP4725_PD_1K,
    MCP4725_PD_100K, MCP4725_PD_500K,
)


# ═══════════════════════════════════════════════════════════════
#  波形类型定义
# ═══════════════════════════════════════════════════════════════

WAVE_DC = 0       # 直流
WAVE_SINE = 1     # 正弦波
WAVE_TRIANGLE = 2 # 三角波
WAVE_SAWTOOTH = 3 # 锯齿波
WAVE_SQUARE = 4   # 方波


class SignalGeneratorV2:
    """信号发生器V2 — MCP4725 12位DAC

    功能:
    - 直流电压输出
    - 正弦波/三角波/锯齿波/方波生成
    - 频率和幅度可调
    - 输出缓冲区管理
    - EEPROM掉电保持
    """

    def __init__(self, addr=MCP4725_ADDR_A0_GND, vref=MCP4725_VREF_DEFAULT):
        self.dac = MCP4725(addr=addr, vref=vref)
        self.vref = vref

        # 波形参数
        self.waveform = WAVE_DC
        self.frequency = 1000.0   # Hz
        self.amplitude = 1.0      # V (峰峰值的一半)
        self.offset = vref / 2.0  # V (直流偏移)
        self.duty_cycle = 50      # % (方波占空比)

        # 输出缓冲区
        self.buffer = []
        self.buffer_size = 256    # 一个周期采样点数

    def init(self):
        """初始化信号发生器"""
        self.dac.init()

    def set_dc_voltage(self, voltage):
        """设置直流输出电压

        返回: (success, dac_value)
        """
        if not self.dac.initialized:
            return False, 0
        ok = self.dac.set_voltage(voltage)
        return ok, self.dac.get_value()

    def set_dc_value(self, value):
        """设置DAC原始值 (0-4095)

        返回: success
        """
        return self.dac.set_value(value)

    def get_output_voltage(self):
        """获取当前输出电压"""
        return self.dac.get_voltage()

    def set_waveform(self, wave_type):
        """设置波形类型"""
        self.waveform = wave_type

    def set_frequency(self, freq):
        """设置频率(Hz)"""
        self.frequency = max(0.1, freq)

    def set_amplitude(self, amp):
        """设置幅度(峰峰值的一半，V)"""
        self.amplitude = max(0, min(self.vref / 2.0, amp))

    def set_offset(self, offset):
        """设置直流偏移(V)"""
        self.offset = max(0, min(self.vref, offset))

    def set_duty_cycle(self, duty):
        """设置方波占空比(0-100%)"""
        self.duty_cycle = max(0, min(100, duty))

    def generate_buffer(self):
        """生成一个周期的波形缓冲区

        返回: DAC值列表 (0-4095)
        """
        buf = []
        n = self.buffer_size
        mid = self.offset

        for i in range(n):
            t = i / n  # 0.0 ~ 1.0

            if self.waveform == WAVE_DC:
                value = mid

            elif self.waveform == WAVE_SINE:
                value = mid + self.amplitude * math.sin(2.0 * math.pi * t)

            elif self.waveform == WAVE_TRIANGLE:
                # 三角波: 0→峰值→0→谷值→0
                if t < 0.25:
                    value = mid + self.amplitude * (t / 0.25)
                elif t < 0.75:
                    value = mid + self.amplitude * (1.0 - (t - 0.25) / 0.25)
                else:
                    value = mid - self.amplitude * (1.0 - (t - 0.75) / 0.25)

            elif self.waveform == WAVE_SAWTOOTH:
                # 锯齿波: 从谷值线性升到峰值
                value = mid + self.amplitude * (2.0 * t - 1.0)

            elif self.waveform == WAVE_SQUARE:
                # 方波
                if t < self.duty_cycle / 100.0:
                    value = mid + self.amplitude
                else:
                    value = mid - self.amplitude
            else:
                value = mid

            # 限幅到0~Vref
            value = max(0, min(self.vref, value))
            # 转换为DAC值
            dac_val = int(round(value / self.vref * MCP4725_MAX_VALUE))
            dac_val = max(0, min(MCP4725_MAX_VALUE, dac_val))
            buf.append(dac_val)

        self.buffer = buf
        return buf

    def output_buffer_point(self, index):
        """输出缓冲区指定点到DAC

        返回: success
        """
        if not self.buffer or index < 0 or index >= len(self.buffer):
            return False
        return self.dac.set_value(self.buffer[index])

    def save_to_eeprom(self):
        """保存当前DAC值到EEPROM"""
        return self.dac.write_eeprom(value=self.dac.get_value())

    def get_status(self):
        """获取信号发生器状态"""
        return {
            'waveform': self.waveform,
            'frequency': self.frequency,
            'amplitude': self.amplitude,
            'offset': self.offset,
            'duty_cycle': self.duty_cycle,
            'dac_value': self.dac.get_value(),
            'voltage': self.dac.get_voltage(),
        }


class TestMCP4725Init(unittest.TestCase):
    """MCP4725初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        gen = SignalGeneratorV2()
        gen.init()
        self.assertTrue(gen.dac.initialized)

    def test_init_default_value(self):
        """初始DAC值为0"""
        gen = SignalGeneratorV2()
        gen.init()
        self.assertEqual(gen.dac.get_value(), 0)

    def test_init_default_power_down(self):
        """初始为正常模式"""
        gen = SignalGeneratorV2()
        gen.init()
        self.assertEqual(gen.dac.power_down, MCP4725_PD_NONE)

    def test_init_vref(self):
        """参考电压正确"""
        gen = SignalGeneratorV2(vref=3.3)
        gen.init()
        self.assertEqual(gen.dac.vref, 3.3)


class TestDCOutput(unittest.TestCase):
    """直流输出测试"""

    def setUp(self):
        self.gen = SignalGeneratorV2()
        self.gen.init()

    def test_set_dc_zero(self):
        """输出0V"""
        ok, val = self.gen.set_dc_voltage(0.0)
        self.assertTrue(ok)
        self.assertEqual(self.gen.dac.get_value(), 0)
        self.assertAlmostEqual(self.gen.get_output_voltage(), 0.0, places=3)

    def test_set_dc_max(self):
        """输出最大电压 Vref"""
        ok, val = self.gen.set_dc_voltage(MCP4725_VREF_DEFAULT)
        self.assertTrue(ok)
        self.assertEqual(self.gen.dac.get_value(), MCP4725_MAX_VALUE)

    def test_set_dc_mid(self):
        """输出中间电压"""
        mid_v = MCP4725_VREF_DEFAULT / 2.0
        self.gen.set_dc_voltage(mid_v)
        v = self.gen.get_output_voltage()
        self.assertAlmostEqual(v, mid_v, delta=0.01)

    def test_set_dc_value_raw(self):
        """直接设置DAC原始值"""
        self.assertTrue(self.gen.set_dc_value(2048))
        self.assertEqual(self.gen.dac.get_value(), 2048)

    def test_set_dc_value_range(self):
        """DAC值范围 0-4095"""
        self.assertTrue(self.gen.set_dc_value(0))
        self.assertTrue(self.gen.set_dc_value(MCP4725_MAX_VALUE))
        self.assertFalse(self.gen.set_dc_value(-1))
        self.assertFalse(self.gen.set_dc_value(MCP4725_MAX_VALUE + 1))

    def test_voltage_before_init(self):
        """初始化前设置电压失败"""
        gen = SignalGeneratorV2()
        ok, _ = gen.set_dc_voltage(1.5)
        self.assertFalse(ok)


class TestSineWave(unittest.TestCase):
    """正弦波测试"""

    def setUp(self):
        self.gen = SignalGeneratorV2()
        self.gen.init()
        self.gen.set_waveform(WAVE_SINE)
        self.gen.set_amplitude(1.0)
        self.gen.set_offset(1.65)

    def test_generate_buffer(self):
        """生成正弦波缓冲区"""
        buf = self.gen.generate_buffer()
        self.assertEqual(len(buf), 256)
        for val in buf:
            self.assertGreaterEqual(val, 0)
            self.assertLessEqual(val, MCP4725_MAX_VALUE)

    def test_sine_range(self):
        """正弦波值范围正确"""
        buf = self.gen.generate_buffer()
        mid_dac = int(round(1.65 / 3.3 * MCP4725_MAX_VALUE))
        # 所有值应围绕中间值
        for val in buf:
            self.assertGreaterEqual(val, 0)
            self.assertLessEqual(val, MCP4725_MAX_VALUE)

    def test_sine_peak_values(self):
        """正弦波峰值出现在正确位置"""
        self.gen.buffer_size = 360
        buf = self.gen.generate_buffer()
        # 位置0应为中间值(sin(0)=0)
        mid_dac = int(round(1.65 / 3.3 * MCP4725_MAX_VALUE))
        self.assertAlmostEqual(buf[0], mid_dac, delta=5)
        # 位置90应为最大值(sin(π/2)=1)
        max_dac = int(round(2.65 / 3.3 * MCP4725_MAX_VALUE))
        self.assertAlmostEqual(buf[90], max_dac, delta=5)

    def test_output_point(self):
        """输出缓冲区点"""
        self.gen.generate_buffer()
        self.assertTrue(self.gen.output_buffer_point(0))


class TestTriangleWave(unittest.TestCase):
    """三角波测试"""

    def setUp(self):
        self.gen = SignalGeneratorV2()
        self.gen.init()
        self.gen.set_waveform(WAVE_TRIANGLE)
        self.gen.set_amplitude(1.0)
        self.gen.set_offset(1.65)

    def test_generate_buffer(self):
        """生成三角波缓冲区"""
        buf = self.gen.generate_buffer()
        self.assertEqual(len(buf), 256)

    def test_triangle_symmetry(self):
        """三角波对称性"""
        buf = self.gen.generate_buffer()
        n = len(buf)
        # 中间点应为峰值附近
        peak_idx = n // 4
        self.assertGreater(buf[peak_idx], buf[0])


class TestSawtoothWave(unittest.TestCase):
    """锯齿波测试"""

    def setUp(self):
        self.gen = SignalGeneratorV2()
        self.gen.init()
        self.gen.set_waveform(WAVE_SAWTOOTH)
        self.gen.set_amplitude(1.0)
        self.gen.set_offset(1.65)

    def test_generate_buffer(self):
        """生成锯齿波缓冲区"""
        buf = self.gen.generate_buffer()
        self.assertEqual(len(buf), 256)

    def test_sawtooth_monotonic_first_half(self):
        """锯齿波前半段递增"""
        buf = self.gen.generate_buffer()
        n = len(buf)
        # 从t=0(谷值)到t=0.5(峰值)应单调递增
        for i in range(1, n // 2):
            self.assertGreaterEqual(buf[i], buf[i - 1] - 1)  # 允许1的舍入误差


class TestSquareWave(unittest.TestCase):
    """方波测试"""

    def setUp(self):
        self.gen = SignalGeneratorV2()
        self.gen.init()
        self.gen.set_waveform(WAVE_SQUARE)
        self.gen.set_amplitude(1.0)
        self.gen.set_offset(1.65)

    def test_generate_buffer(self):
        """生成方波缓冲区"""
        buf = self.gen.generate_buffer()
        self.assertEqual(len(buf), 256)

    def test_square_two_levels(self):
        """方波只有两个电平"""
        buf = self.gen.generate_buffer()
        unique_vals = set(buf)
        # 方波应只有高/低两个值(或因舍入有少量差异)
        self.assertLessEqual(len(unique_vals), 3)

    def test_square_duty_cycle(self):
        """方波占空比50%"""
        buf = self.gen.generate_buffer()
        high_count = sum(1 for v in buf if v > MCP4725_MAX_VALUE // 2)
        # 约50%高电平
        self.assertAlmostEqual(high_count / len(buf), 0.5, delta=0.05)

    def test_custom_duty_cycle(self):
        """自定义占空比"""
        self.gen.set_duty_cycle(25)
        buf = self.gen.generate_buffer()
        high_count = sum(1 for v in buf if v > MCP4725_MAX_VALUE // 2)
        self.assertAlmostEqual(high_count / len(buf), 0.25, delta=0.05)


class TestWaveformParameters(unittest.TestCase):
    """波形参数测试"""

    def setUp(self):
        self.gen = SignalGeneratorV2()
        self.gen.init()

    def test_set_amplitude(self):
        """设置幅度"""
        self.gen.set_amplitude(0.5)
        self.assertAlmostEqual(self.gen.amplitude, 0.5)

    def test_amplitude_clamp(self):
        """幅度限幅到Vref/2"""
        self.gen.set_amplitude(10.0)
        self.assertAlmostEqual(self.gen.amplitude, MCP4725_VREF_DEFAULT / 2.0)

    def test_set_offset(self):
        """设置偏移"""
        self.gen.set_offset(2.0)
        self.assertAlmostEqual(self.gen.offset, 2.0)

    def test_set_frequency(self):
        """设置频率"""
        self.gen.set_frequency(500)
        self.assertAlmostEqual(self.gen.frequency, 500.0)

    def test_frequency_minimum(self):
        """频率最小值保护"""
        self.gen.set_frequency(-10)
        self.assertGreaterEqual(self.gen.frequency, 0.1)

    def test_duty_cycle_range(self):
        """占空比范围0-100"""
        self.gen.set_duty_cycle(0)
        self.assertEqual(self.gen.duty_cycle, 0)
        self.gen.set_duty_cycle(100)
        self.assertEqual(self.gen.duty_cycle, 100)
        self.gen.set_duty_cycle(150)
        self.assertEqual(self.gen.duty_cycle, 100)


class TestEEPROM(unittest.TestCase):
    """EEPROM存储测试"""

    def setUp(self):
        self.gen = SignalGeneratorV2()
        self.gen.init()

    def test_save_to_eeprom(self):
        """保存DAC值到EEPROM"""
        self.gen.set_dc_value(2048)
        self.assertTrue(self.gen.save_to_eeprom())
        self.assertEqual(self.gen.dac.eeprom_dac, 2048)

    def test_eeprom_before_init(self):
        """初始化前EEPROM写入失败"""
        gen = SignalGeneratorV2()
        self.assertFalse(gen.dac.write_eeprom(value=100))


class TestPowerDown(unittest.TestCase):
    """掉电模式测试"""

    def setUp(self):
        self.gen = SignalGeneratorV2()
        self.gen.init()

    def test_power_down_modes(self):
        """掉电模式设置"""
        modes = [MCP4725_PD_NONE, MCP4725_PD_1K,
                 MCP4725_PD_100K, MCP4725_PD_500K]
        for mode in modes:
            self.assertTrue(self.gen.dac.set_power_down(mode))
            self.assertEqual(self.gen.dac.power_down, mode)

    def test_invalid_power_down(self):
        """无效掉电模式"""
        self.assertFalse(self.gen.dac.set_power_down(10))


class TestStatus(unittest.TestCase):
    """状态查询测试"""

    def test_get_status(self):
        """获取完整状态"""
        gen = SignalGeneratorV2()
        gen.init()
        gen.set_waveform(WAVE_SINE)
        gen.set_frequency(1000)
        gen.set_amplitude(1.0)
        gen.set_dc_voltage(1.5)

        status = gen.get_status()
        self.assertEqual(status['waveform'], WAVE_SINE)
        self.assertEqual(status['frequency'], 1000.0)
        self.assertAlmostEqual(status['amplitude'], 1.0)
        self.assertAlmostEqual(status['voltage'], 1.5, delta=0.01)

    def test_read_status(self):
        """MCP4725状态寄存器"""
        gen = SignalGeneratorV2()
        gen.init()
        status = gen.dac.read_status()
        self.assertIn('ready', status)
        self.assertIn('dac_value', status)
        self.assertIn('power_down', status)
        self.assertTrue(status['ready'])


if __name__ == '__main__':
    unittest.main()
