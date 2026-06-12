#!/usr/bin/env python3
"""
DDS信号发生器V2测试 — 基于wrappers.py包装层
覆盖: AD9833 DDS芯片驱动（SPI接口）
模拟场景: 正弦波/三角波/方波输出、频率/相位调节、预设频率、频率扫描
对应C源文件: 02_mspm0g3507/examples/dds_signal_generator.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    AD9833,
    WAVE_SINE, WAVE_TRIANGLE, WAVE_SQUARE, WAVE_NAMES,
    FREQ_STEPS, PRESET_FREQS,
    AD9833_MAX_FREQ, AD9833_MIN_FREQ, AD9833_MCLK,
    AD9833_B28, AD9833_MODE, AD9833_OPBITEN, AD9833_DIV2,
    AD9833_FREQ0_REG, AD9833_PHASE_REG,
)


class TestAD9833Init(unittest.TestCase):
    """AD9833初始化测试"""

    def test_init_success(self):
        """AD9833初始化成功"""
        dds = AD9833()
        self.assertTrue(dds.init())
        self.assertTrue(dds.initialized)

    def test_default_params(self):
        """默认参数：正弦波、1kHz、0度相位"""
        dds = AD9833()
        dds.init()
        self.assertEqual(dds.get_waveform(), WAVE_SINE)
        self.assertAlmostEqual(dds.get_frequency(), 1000.0)
        self.assertAlmostEqual(dds.get_phase(), 0.0)

    def test_default_waveform_name(self):
        """默认波形名称为Sine"""
        dds = AD9833()
        dds.init()
        self.assertEqual(dds.get_waveform_name(), "Sine")


class TestAD9833Frequency(unittest.TestCase):
    """AD9833频率控制测试"""

    def setUp(self):
        self.dds = AD9833()
        self.dds.init()

    def test_set_frequency_1khz(self):
        """设置1kHz频率"""
        self.assertTrue(self.dds.set_frequency(1000.0))
        self.assertAlmostEqual(self.dds.get_frequency(), 1000.0)

    def test_set_frequency_1mhz(self):
        """设置1MHz频率"""
        self.assertTrue(self.dds.set_frequency(1000000.0))
        self.assertAlmostEqual(self.dds.get_frequency(), 1000000.0)

    def test_set_frequency_min(self):
        """设置最小频率0.1Hz"""
        self.assertTrue(self.dds.set_frequency(0.1))
        self.assertAlmostEqual(self.dds.get_frequency(), 0.1)

    def test_set_frequency_max(self):
        """设置最大频率12.5MHz"""
        self.assertTrue(self.dds.set_frequency(12500000.0))
        self.assertAlmostEqual(self.dds.get_frequency(), 12500000.0)

    def test_set_frequency_too_low(self):
        """频率低于最小值失败"""
        self.assertFalse(self.dds.set_frequency(0.05))

    def test_set_frequency_too_high(self):
        """频率高于最大值失败"""
        self.assertFalse(self.dds.set_frequency(13000000.0))

    def test_set_frequency_not_initialized(self):
        """未初始化设置频率失败"""
        dds2 = AD9833()
        self.assertFalse(dds2.set_frequency(1000.0))

    def test_freq_word_calculation(self):
        """频率字计算验证：freq_word = freq * 2^28 / MCLK"""
        self.dds.set_frequency(1000.0)
        # 1000 * 2^28 / 25000000 = 10737.41824 ≈ 10737
        expected_word = int(1000.0 * (1 << 28) / AD9833_MCLK)
        log = self.dds.get_spi_log()
        # 第一条是控制字(B28|RESET)，后两条是freq_lsb和freq_msb
        freq_lsb = log[1][1] & 0x3FFF
        freq_msb = log[2][1] & 0x3FFF
        actual_word = freq_lsb | (freq_msb << 14)
        self.assertEqual(actual_word, expected_word)

    def test_freq_up(self):
        """频率增加（步进100Hz）"""
        self.dds.set_frequency(1000.0)
        self.dds.set_step_index(3)  # 100Hz步进
        self.dds.freq_up()
        self.assertAlmostEqual(self.dds.get_frequency(), 1100.0)

    def test_freq_down(self):
        """频率减少"""
        self.dds.set_frequency(1000.0)
        self.dds.set_step_index(3)  # 100Hz步进
        self.dds.freq_down()
        self.assertAlmostEqual(self.dds.get_frequency(), 900.0)

    def test_freq_up_clamp(self):
        """频率增加不超过最大值"""
        self.dds.set_frequency(AD9833_MAX_FREQ - 10.0)
        self.dds.set_step_index(3)
        self.dds.freq_up()
        self.assertLessEqual(self.dds.get_frequency(), AD9833_MAX_FREQ)

    def test_freq_down_clamp(self):
        """频率减少不低于最小值"""
        self.dds.set_frequency(0.2)
        self.dds.set_step_index(3)  # 100Hz步进，远大于当前频率
        self.dds.freq_down()
        self.assertGreaterEqual(self.dds.get_frequency(), AD9833_MIN_FREQ)


class TestAD9833Phase(unittest.TestCase):
    """AD9833相位控制测试"""

    def setUp(self):
        self.dds = AD9833()
        self.dds.init()

    def test_set_phase_0(self):
        """设置0度相位"""
        self.assertTrue(self.dds.set_phase(0.0))
        self.assertAlmostEqual(self.dds.get_phase(), 0.0)

    def test_set_phase_180(self):
        """设置180度相位"""
        self.assertTrue(self.dds.set_phase(180.0))
        self.assertAlmostEqual(self.dds.get_phase(), 180.0)

    def test_set_phase_360(self):
        """设置360度相位"""
        self.assertTrue(self.dds.set_phase(360.0))
        self.assertAlmostEqual(self.dds.get_phase(), 360.0)

    def test_set_phase_negative(self):
        """负相位失败"""
        self.assertFalse(self.dds.set_phase(-10.0))

    def test_set_phase_over_360(self):
        """超过360度相位失败"""
        self.assertFalse(self.dds.set_phase(400.0))

    def test_phase_word_calculation(self):
        """相位字计算：phase_word = phase/360 * 4096"""
        self.dds.set_phase(180.0)
        log = self.dds.get_spi_log()
        # 相位字 = 180/360 * 4096 = 2048
        phase_word = log[0][1] & 0x0FFF
        self.assertEqual(phase_word, 2048)


class TestAD9833Waveform(unittest.TestCase):
    """AD9833波形控制测试"""

    def setUp(self):
        self.dds = AD9833()
        self.dds.init()

    def test_set_sine(self):
        """设置正弦波"""
        self.assertTrue(self.dds.set_waveform(WAVE_SINE))
        self.assertEqual(self.dds.get_waveform(), WAVE_SINE)
        self.assertEqual(self.dds.get_waveform_name(), "Sine")

    def test_set_triangle(self):
        """设置三角波"""
        self.assertTrue(self.dds.set_waveform(WAVE_TRIANGLE))
        self.assertEqual(self.dds.get_waveform(), WAVE_TRIANGLE)
        self.assertEqual(self.dds.get_waveform_name(), "Triangle")

    def test_set_square(self):
        """设置方波"""
        self.assertTrue(self.dds.set_waveform(WAVE_SQUARE))
        self.assertEqual(self.dds.get_waveform(), WAVE_SQUARE)
        self.assertEqual(self.dds.get_waveform_name(), "Square")

    def test_waveform_control_word_sine(self):
        """正弦波控制字：仅B28"""
        self.dds.set_waveform(WAVE_SINE)
        log = self.dds.get_spi_log()
        control = log[0][1]
        self.assertEqual(control, AD9833_B28)

    def test_waveform_control_word_triangle(self):
        """三角波控制字：B28 | MODE"""
        self.dds.set_waveform(WAVE_TRIANGLE)
        log = self.dds.get_spi_log()
        control = log[0][1]
        self.assertEqual(control, AD9833_B28 | AD9833_MODE)

    def test_waveform_control_word_square(self):
        """方波控制字：B28 | OPBITEN | DIV2"""
        self.dds.set_waveform(WAVE_SQUARE)
        log = self.dds.get_spi_log()
        control = log[0][1]
        self.assertEqual(control, AD9833_B28 | AD9833_OPBITEN | AD9833_DIV2)

    def test_invalid_waveform(self):
        """无效波形类型失败"""
        self.assertFalse(self.dds.set_waveform(99))


class TestAD9833StepAndPreset(unittest.TestCase):
    """AD9833频率步进和预设测试"""

    def setUp(self):
        self.dds = AD9833()
        self.dds.init()

    def test_set_step_index(self):
        """设置频率步进档位"""
        for i in range(len(FREQ_STEPS)):
            self.assertTrue(self.dds.set_step_index(i))
            self.assertEqual(self.dds.get_step_value(), FREQ_STEPS[i])

    def test_invalid_step_index(self):
        """无效步进档位失败"""
        self.assertFalse(self.dds.set_step_index(-1))
        self.assertFalse(self.dds.set_step_index(100))

    def test_apply_preset(self):
        """应用预设频率"""
        for i, freq in enumerate(PRESET_FREQS):
            self.assertTrue(self.dds.apply_preset(i))
            self.assertAlmostEqual(self.dds.get_frequency(), freq)

    def test_invalid_preset(self):
        """无效预设索引失败"""
        self.assertFalse(self.dds.apply_preset(-1))
        self.assertFalse(self.dds.apply_preset(100))

    def test_all_waveform_names(self):
        """所有波形名称验证"""
        self.assertEqual(WAVE_NAMES, ["Sine", "Triangle", "Square"])


class TestAD9833SPI(unittest.TestCase):
    """AD9833 SPI传输记录测试"""

    def setUp(self):
        self.dds = AD9833()
        self.dds.init()

    def test_spi_log_frequency(self):
        """频率设置产生3条SPI记录（控制字+LSB+MSB）"""
        self.dds.set_frequency(1000.0)
        log = self.dds.get_spi_log()
        self.assertEqual(len(log), 3)
        self.assertEqual(log[0][0], 'freq')  # 控制字
        self.assertEqual(log[1][0], 'freq')  # LSB
        self.assertEqual(log[2][0], 'freq')  # MSB

    def test_spi_log_contains_freq_reg(self):
        """频率字包含FREQ0_REG标识"""
        self.dds.set_frequency(1000.0)
        log = self.dds.get_spi_log()
        self.assertTrue(log[1][1] & AD9833_FREQ0_REG)
        self.assertTrue(log[2][1] & AD9833_FREQ0_REG)

    def test_enable_output(self):
        """使能/禁用输出"""
        self.dds.enable_output(False)
        self.assertFalse(self.dds.output_enabled)
        self.dds.enable_output(True)
        self.assertTrue(self.dds.output_enabled)


if __name__ == '__main__':
    unittest.main()
