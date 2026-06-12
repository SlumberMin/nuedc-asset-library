#!/usr/bin/env python3
"""
系统诊断V2测试 — 基于wrappers.py包装层
覆盖: MSPM0G3507系统诊断工具（CPU/内存/时钟/GPIO/外设检测）
模拟场景: 全系统健康检查、单项诊断、诊断报告生成
对应C源文件: 02_mspm0g3507/examples/system_diagnostics.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    SystemDiag,
    DIAG_CPU, DIAG_MEMORY, DIAG_CLOCK, DIAG_GPIO,
    DIAG_ADC, DIAG_UART, DIAG_I2C, DIAG_SPI, DIAG_TIMER, DIAG_DMA,
    DIAG_NAMES,
    DIAG_PASS, DIAG_FAIL, DIAG_SKIP, DIAG_WARN,
    DIAG_STATUS_NAMES,
)


class TestSystemDiagInit(unittest.TestCase):
    """系统诊断初始化测试"""

    def test_init_success(self):
        """诊断工具初始化成功"""
        diag = SystemDiag()
        self.assertTrue(diag.init())
        self.assertTrue(diag.initialized)

    def test_default_state(self):
        """默认状态：无诊断结果"""
        diag = SystemDiag()
        diag.init()
        self.assertEqual(len(diag.get_all_results()), 0)


class TestSystemDiagSingle(unittest.TestCase):
    """单项诊断测试"""

    def setUp(self):
        self.diag = SystemDiag()
        self.diag.init()

    def test_run_cpu_diag(self):
        """运行CPU诊断"""
        status, msg = self.diag.run_single(DIAG_CPU)
        self.assertEqual(status, DIAG_PASS)
        self.assertEqual(msg, "OK")

    def test_run_memory_diag(self):
        """运行内存诊断"""
        status, msg = self.diag.run_single(DIAG_MEMORY)
        self.assertEqual(status, DIAG_PASS)

    def test_run_clock_diag(self):
        """运行时钟诊断"""
        status, msg = self.diag.run_single(DIAG_CLOCK)
        self.assertEqual(status, DIAG_PASS)

    def test_run_gpio_diag(self):
        """运行GPIO诊断"""
        status, msg = self.diag.run_single(DIAG_GPIO)
        self.assertEqual(status, DIAG_PASS)

    def test_run_adc_diag(self):
        """运行ADC诊断"""
        status, msg = self.diag.run_single(DIAG_ADC)
        self.assertEqual(status, DIAG_PASS)

    def test_run_uart_diag(self):
        """运行UART诊断"""
        status, msg = self.diag.run_single(DIAG_UART)
        self.assertEqual(status, DIAG_PASS)

    def test_run_i2c_diag(self):
        """运行I2C诊断"""
        status, msg = self.diag.run_single(DIAG_I2C)
        self.assertEqual(status, DIAG_PASS)

    def test_run_spi_diag(self):
        """运行SPI诊断"""
        status, msg = self.diag.run_single(DIAG_SPI)
        self.assertEqual(status, DIAG_PASS)

    def test_run_timer_diag(self):
        """运行定时器诊断"""
        status, msg = self.diag.run_single(DIAG_TIMER)
        self.assertEqual(status, DIAG_PASS)

    def test_run_dma_diag(self):
        """运行DMA诊断"""
        status, msg = self.diag.run_single(DIAG_DMA)
        self.assertEqual(status, DIAG_PASS)

    def test_run_invalid_diag(self):
        """无效诊断项失败"""
        status, msg = self.diag.run_single(-1)
        self.assertEqual(status, DIAG_FAIL)

        status, msg = self.diag.run_single(100)
        self.assertEqual(status, DIAG_FAIL)

    def test_run_not_initialized(self):
        """未初始化运行诊断失败"""
        diag2 = SystemDiag()
        status, msg = diag2.run_single(DIAG_CPU)
        self.assertEqual(status, DIAG_FAIL)
        self.assertEqual(msg, "未初始化")


class TestSystemDiagAll(unittest.TestCase):
    """全部诊断测试"""

    def setUp(self):
        self.diag = SystemDiag()
        self.diag.init()

    def test_run_all(self):
        """运行全部诊断"""
        self.assertTrue(self.diag.run_all())

    def test_run_all_not_initialized(self):
        """未初始化运行全部失败"""
        diag2 = SystemDiag()
        self.assertFalse(diag2.run_all())

    def test_all_results_count(self):
        """全部诊断结果数量"""
        self.diag.run_all()
        results = self.diag.get_all_results()
        self.assertEqual(len(results), len(DIAG_NAMES))

    def test_all_pass(self):
        """全部诊断通过"""
        self.diag.run_all()
        self.assertEqual(self.diag.get_pass_count(), len(DIAG_NAMES))
        self.assertEqual(self.diag.get_fail_count(), 0)


class TestSystemDiagResults(unittest.TestCase):
    """诊断结果查询测试"""

    def setUp(self):
        self.diag = SystemDiag()
        self.diag.init()
        self.diag.run_all()

    def test_get_result(self):
        """获取单项诊断结果"""
        status, msg = self.diag.get_result(DIAG_CPU)
        self.assertEqual(status, DIAG_PASS)
        self.assertEqual(msg, "OK")

    def test_get_result_not_executed(self):
        """未执行的诊断项返回SKIP"""
        diag2 = SystemDiag()
        diag2.init()
        status, msg = diag2.get_result(DIAG_CPU)
        self.assertEqual(status, DIAG_SKIP)
        self.assertEqual(msg, "未执行")

    def test_get_all_results(self):
        """获取全部结果"""
        results = self.diag.get_all_results()
        self.assertIsInstance(results, dict)
        for diag_id in range(len(DIAG_NAMES)):
            self.assertIn(diag_id, results)

    def test_get_summary(self):
        """获取诊断摘要"""
        summary = self.diag.get_summary()
        self.assertEqual(summary['total'], len(DIAG_NAMES))
        self.assertEqual(summary['passed'], len(DIAG_NAMES))
        self.assertEqual(summary['failed'], 0)
        self.assertAlmostEqual(summary['pass_rate'], 100.0)

    def test_summary_empty(self):
        """空诊断摘要"""
        diag2 = SystemDiag()
        diag2.init()
        summary = diag2.get_summary()
        self.assertEqual(summary['total'], 0)
        self.assertEqual(summary['pass_rate'], 0.0)


class TestSystemDiagInfo(unittest.TestCase):
    """系统信息测试"""

    def setUp(self):
        self.diag = SystemDiag()
        self.diag.init()

    def test_chip_id(self):
        """芯片ID"""
        self.assertEqual(self.diag.get_chip_id(), 0x3507)

    def test_cpu_freq(self):
        """CPU频率32MHz"""
        self.assertEqual(self.diag.get_cpu_freq(), 32000000)

    def test_memory_info(self):
        """内存信息"""
        info = self.diag.get_memory_info()
        self.assertEqual(info['flash_kb'], 256)
        self.assertEqual(info['ram_kb'], 32)


class TestSystemDiagTiming(unittest.TestCase):
    """系统时序测试"""

    def setUp(self):
        self.diag = SystemDiag()
        self.diag.init()

    def test_boot_time(self):
        """启动时间"""
        self.diag.set_boot_time(150)
        self.assertEqual(self.diag.get_boot_time(), 150)

    def test_uptime(self):
        """运行时间"""
        self.diag.set_uptime(60000)
        self.assertEqual(self.diag.get_uptime(), 60000)

    def test_reset_count(self):
        """复位次数"""
        self.diag.set_reset_count(5)
        self.assertEqual(self.diag.get_reset_count(), 5)

    def test_default_timing(self):
        """默认时序值"""
        self.assertEqual(self.diag.get_boot_time(), 0)
        self.assertEqual(self.diag.get_uptime(), 0)
        self.assertEqual(self.diag.get_reset_count(), 0)


class TestSystemDiagConstants(unittest.TestCase):
    """诊断常量验证"""

    def test_diag_names(self):
        """诊断项名称"""
        self.assertEqual(len(DIAG_NAMES), 10)
        self.assertEqual(DIAG_NAMES[DIAG_CPU], "CPU")
        self.assertEqual(DIAG_NAMES[DIAG_MEMORY], "Memory")
        self.assertEqual(DIAG_NAMES[DIAG_CLOCK], "Clock")
        self.assertEqual(DIAG_NAMES[DIAG_DMA], "DMA")

    def test_diag_status_names(self):
        """诊断状态名称"""
        self.assertEqual(DIAG_STATUS_NAMES[DIAG_PASS], "PASS")
        self.assertEqual(DIAG_STATUS_NAMES[DIAG_FAIL], "FAIL")
        self.assertEqual(DIAG_STATUS_NAMES[DIAG_SKIP], "SKIP")
        self.assertEqual(DIAG_STATUS_NAMES[DIAG_WARN], "WARN")

    def test_diag_ids_unique(self):
        """诊断ID唯一"""
        ids = [DIAG_CPU, DIAG_MEMORY, DIAG_CLOCK, DIAG_GPIO,
               DIAG_ADC, DIAG_UART, DIAG_I2C, DIAG_SPI,
               DIAG_TIMER, DIAG_DMA]
        self.assertEqual(len(ids), len(set(ids)))


if __name__ == '__main__':
    unittest.main()
