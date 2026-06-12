#!/usr/bin/env python3
"""
时钟发生器V2测试 — 基于wrappers.py包装层
覆盖: MSPM0G3507时钟系统（时钟源、分频、PLL、外设时钟）
模拟场景: 系统时钟配置、外设时钟分频、定时器周期计算、波特率分频
对应C源文件: 02_mspm0g3507/examples/clock_generator.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    ClockGen,
    CLK_SRC_HFXT, CLK_SRC_HFOSC, CLK_SRC_LFXT, CLK_SRC_LFOSC,
    CLK_SOURCES,
    CLK_FREQ_HFXT, CLK_FREQ_HFOSC, CLK_FREQ_LFXT, CLK_FREQ_LFOSC,
)


class TestClockGenInit(unittest.TestCase):
    """时钟发生器初始化测试"""

    def test_init_success(self):
        """时钟系统初始化成功"""
        clk = ClockGen()
        self.assertTrue(clk.init())
        self.assertTrue(clk.initialized)

    def test_default_clock_source(self):
        """默认时钟源为HFXT(外部高速晶振)"""
        clk = ClockGen()
        clk.init()
        self.assertEqual(clk.sysclk_src, CLK_SRC_HFXT)
        self.assertEqual(clk.get_clock_source_name(), "HFXT")

    def test_default_frequency(self):
        """默认系统时钟32MHz"""
        clk = ClockGen()
        clk.init()
        self.assertEqual(clk.get_sysclk_freq(), CLK_FREQ_HFXT)


class TestClockGenSource(unittest.TestCase):
    """时钟源切换测试"""

    def setUp(self):
        self.clk = ClockGen()
        self.clk.init()

    def test_switch_to_hfosc(self):
        """切换到内部高速振荡器(4MHz)"""
        self.assertTrue(self.clk.set_sysclk_source(CLK_SRC_HFOSC))
        self.assertEqual(self.clk.get_sysclk_freq(), CLK_FREQ_HFOSC)
        self.assertEqual(self.clk.get_clock_source_name(), "HFOSC")

    def test_switch_to_lfxt(self):
        """切换到外部低速晶振(32.768kHz)"""
        self.assertTrue(self.clk.set_sysclk_source(CLK_SRC_LFXT))
        self.assertEqual(self.clk.get_sysclk_freq(), CLK_FREQ_LFXT)
        self.assertEqual(self.clk.get_clock_source_name(), "LFXT")

    def test_switch_to_lfosc(self):
        """切换到内部低速振荡器(32.768kHz)"""
        self.assertTrue(self.clk.set_sysclk_source(CLK_SRC_LFOSC))
        self.assertEqual(self.clk.get_sysclk_freq(), CLK_FREQ_LFOSC)

    def test_invalid_source(self):
        """无效时钟源失败"""
        self.assertFalse(self.clk.set_sysclk_source(99))

    def test_all_sources(self):
        """所有时钟源频率验证"""
        expected = {
            CLK_SRC_HFXT: CLK_FREQ_HFXT,
            CLK_SRC_HFOSC: CLK_FREQ_HFOSC,
            CLK_SRC_LFXT: CLK_FREQ_LFXT,
            CLK_SRC_LFOSC: CLK_FREQ_LFOSC,
        }
        for src, freq in expected.items():
            self.assertTrue(self.clk.set_sysclk_source(src))
            self.assertEqual(self.clk.get_sysclk_freq(), freq)


class TestClockGenDivider(unittest.TestCase):
    """时钟分频测试"""

    def setUp(self):
        self.clk = ClockGen()
        self.clk.init()

    def test_sysclk_div_2(self):
        """系统时钟2分频"""
        self.assertTrue(self.clk.set_sysclk_divider(2))
        self.assertEqual(self.clk.get_sysclk_freq(), CLK_FREQ_HFXT // 2)

    def test_sysclk_div_4(self):
        """系统时钟4分频"""
        self.assertTrue(self.clk.set_sysclk_divider(4))
        self.assertEqual(self.clk.get_sysclk_freq(), CLK_FREQ_HFXT // 4)

    def test_sysclk_div_256(self):
        """系统时钟256分频"""
        self.assertTrue(self.clk.set_sysclk_divider(256))
        self.assertEqual(self.clk.get_sysclk_freq(), CLK_FREQ_HFXT // 256)

    def test_sysclk_div_1(self):
        """系统时钟1分频（不分频）"""
        self.assertTrue(self.clk.set_sysclk_divider(1))
        self.assertEqual(self.clk.get_sysclk_freq(), CLK_FREQ_HFXT)

    def test_sysclk_div_invalid_zero(self):
        """0分频失败"""
        self.assertFalse(self.clk.set_sysclk_divider(0))

    def test_sysclk_div_invalid_over(self):
        """超过256分频失败"""
        self.assertFalse(self.clk.set_sysclk_divider(257))

    def test_peripheral_divider(self):
        """外设时钟分频"""
        self.assertTrue(self.clk.set_peripheral_divider(2))
        self.assertEqual(self.clk.get_timer_freq(), CLK_FREQ_HFXT // 2)
        self.assertEqual(self.clk.get_uart_freq(), CLK_FREQ_HFXT // 2)
        self.assertEqual(self.clk.get_i2c_freq(), CLK_FREQ_HFXT // 2)
        self.assertEqual(self.clk.get_spi_freq(), CLK_FREQ_HFXT // 2)

    def test_peripheral_divider_invalid(self):
        """无效外设分频失败"""
        self.assertFalse(self.clk.set_peripheral_divider(0))
        self.assertFalse(self.clk.set_peripheral_divider(300))


class TestClockGenPLL(unittest.TestCase):
    """PLL测试"""

    def setUp(self):
        self.clk = ClockGen()
        self.clk.init()

    def test_enable_pll(self):
        """使能PLL"""
        self.assertTrue(self.clk.enable_pll(2))
        self.assertEqual(self.clk.get_sysclk_freq(), CLK_FREQ_HFXT * 2)

    def test_pll_multiplier_range(self):
        """PLL倍频范围1~16"""
        for m in range(1, 17):
            self.assertTrue(self.clk.enable_pll(m))
            self.assertEqual(self.clk.get_sysclk_freq(), CLK_FREQ_HFXT * m)

    def test_pll_invalid_zero(self):
        """PLL倍频0失败"""
        self.assertFalse(self.clk.enable_pll(0))

    def test_pll_invalid_over(self):
        """PLL倍频超过16失败"""
        self.assertFalse(self.clk.enable_pll(17))

    def test_disable_pll(self):
        """禁用PLL恢复原频率"""
        self.clk.enable_pll(4)
        self.assertEqual(self.clk.get_sysclk_freq(), CLK_FREQ_HFXT * 4)
        self.clk.disable_pll()
        self.assertEqual(self.clk.get_sysclk_freq(), CLK_FREQ_HFXT)

    def test_pll_with_divider(self):
        """PLL+分频组合"""
        self.clk.enable_pll(4)
        self.clk.set_sysclk_divider(2)
        self.assertEqual(self.clk.get_sysclk_freq(), CLK_FREQ_HFXT * 4 // 2)


class TestClockGenPeripheral(unittest.TestCase):
    """外设时钟测试"""

    def setUp(self):
        self.clk = ClockGen()
        self.clk.init()

    def test_default_peripheral_freq(self):
        """默认外设时钟等于系统时钟"""
        self.assertEqual(self.clk.get_timer_freq(), CLK_FREQ_HFXT)
        self.assertEqual(self.clk.get_uart_freq(), CLK_FREQ_HFXT)
        self.assertEqual(self.clk.get_i2c_freq(), CLK_FREQ_HFXT)
        self.assertEqual(self.clk.get_spi_freq(), CLK_FREQ_HFXT)

    def test_peripheral_after_source_change(self):
        """切换时钟源后外设时钟同步更新"""
        self.clk.set_sysclk_source(CLK_SRC_HFOSC)
        self.assertEqual(self.clk.get_timer_freq(), CLK_FREQ_HFOSC)


class TestClockGenCalc(unittest.TestCase):
    """时钟计算测试"""

    def setUp(self):
        self.clk = ClockGen()
        self.clk.init()

    def test_calc_timer_period_1khz(self):
        """计算1kHz定时器周期"""
        period = self.clk.calc_timer_period(1000)
        self.assertEqual(period, CLK_FREQ_HFXT // 1000)

    def test_calc_timer_period_1hz(self):
        """计算1Hz定时器周期"""
        period = self.clk.calc_timer_period(1)
        self.assertEqual(period, CLK_FREQ_HFXT)

    def test_calc_timer_period_invalid(self):
        """无效频率返回0"""
        self.assertEqual(self.clk.calc_timer_period(0), 0)
        self.assertEqual(self.clk.calc_timer_period(-1), 0)

    def test_calc_uart_baud_115200(self):
        """计算115200波特率分频"""
        div = self.clk.calc_uart_baud_div(115200)
        self.assertGreater(div, 0)

    def test_calc_uart_baud_9600(self):
        """计算9600波特率分频"""
        div = self.clk.calc_uart_baud_div(9600)
        expected = CLK_FREQ_HFXT // (16 * 9600)
        self.assertEqual(div, expected)

    def test_calc_uart_baud_invalid(self):
        """无效波特率返回0"""
        self.assertEqual(self.clk.calc_uart_baud_div(0), 0)


if __name__ == '__main__':
    unittest.main()
