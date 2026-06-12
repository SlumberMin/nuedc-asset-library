#!/usr/bin/env python3
"""
PCF8574 8位IO扩展器 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、I2C地址、引脚读写、端口读写、
      引脚翻转、准双向特性、边界条件
对应C源文件: 02_mspm0g3507/drivers/pcf8574.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    PCF8574, PCF8574_ADDR_0, PCF8574_ADDR_1,
    PCF8574_ADDR_2, PCF8574_ADDR_7,
    PCF8574_ADDR_BASE, PCF8574_ADDR_MAX,
)


class TestPCF8574Init(unittest.TestCase):
    """PCF8574初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        io = PCF8574()
        self.assertTrue(io.init())
        self.assertTrue(io.initialized)

    def test_default_address(self):
        """默认I2C地址0x20"""
        io = PCF8574()
        self.assertEqual(io.addr, PCF8574_ADDR_0)

    def test_custom_address(self):
        """自定义I2C地址"""
        io = PCF8574(PCF8574_ADDR_7)
        self.assertEqual(io.addr, PCF8574_ADDR_7)

    def test_default_output_all_high(self):
        """上电默认输出全1（输入模式）"""
        io = PCF8574()
        self.assertEqual(io._output, 0xFF)

    def test_default_input_all_high(self):
        """默认模拟输入全1"""
        io = PCF8574()
        self.assertEqual(io._input, 0xFF)


class TestPCF8574WritePin(unittest.TestCase):
    """PCF8574引脚写入测试"""

    def setUp(self):
        self.io = PCF8574()
        self.io.init()

    def test_write_pin_low(self):
        """写低电平（输出模式）"""
        self.assertTrue(self.io.write_pin(0, 0))
        self.assertEqual(self.io._output & 0x01, 0)

    def test_write_pin_high(self):
        """写高电平（输入/上拉模式）"""
        self.io.write_pin(0, 0)  # 先拉低
        self.assertTrue(self.io.write_pin(0, 1))
        self.assertEqual(self.io._output & 0x01, 1)

    def test_write_all_pins(self):
        """逐个写入所有引脚"""
        for pin in range(8):
            self.assertTrue(self.io.write_pin(pin, 0))

    def test_write_invalid_pin_low(self):
        """无效引脚号（负数）失败"""
        self.assertFalse(self.io.write_pin(-1, 0))

    def test_write_invalid_pin_high(self):
        """无效引脚号（>7）失败"""
        self.assertFalse(self.io.write_pin(8, 0))

    def test_write_not_initialized(self):
        """未初始化写入失败"""
        io2 = PCF8574()
        self.assertFalse(io2.write_pin(0, 0))


class TestPCF8574ReadPin(unittest.TestCase):
    """PCF8574引脚读取测试"""

    def setUp(self):
        self.io = PCF8574()
        self.io.init()

    def test_read_default_high(self):
        """默认输入全高"""
        for pin in range(8):
            self.assertEqual(self.io.read_pin(pin), 1)

    def test_read_simulated_low(self):
        """读取模拟低电平"""
        self.io.set_simulated_input(0x00)
        for pin in range(8):
            self.assertEqual(self.io.read_pin(pin), 0)

    def test_read_simulated_pattern(self):
        """读取模拟模式"""
        self.io.set_simulated_input(0xA5)  # 10100101
        self.assertEqual(self.io.read_pin(0), 1)  # bit0
        self.assertEqual(self.io.read_pin(1), 0)  # bit1
        self.assertEqual(self.io.read_pin(2), 1)  # bit2
        self.assertEqual(self.io.read_pin(3), 0)  # bit3
        self.assertEqual(self.io.read_pin(4), 0)  # bit4
        self.assertEqual(self.io.read_pin(5), 1)  # bit5
        self.assertEqual(self.io.read_pin(6), 0)  # bit6
        self.assertEqual(self.io.read_pin(7), 1)  # bit7

    def test_read_invalid_pin(self):
        """无效引脚返回None"""
        self.assertIsNone(self.io.read_pin(-1))
        self.assertIsNone(self.io.read_pin(8))

    def test_read_not_initialized(self):
        """未初始化返回None"""
        io2 = PCF8574()
        self.assertIsNone(io2.read_pin(0))


class TestPCF8574Port(unittest.TestCase):
    """PCF8574端口操作测试"""

    def setUp(self):
        self.io = PCF8574()
        self.io.init()

    def test_write_port(self):
        """写入整个端口"""
        self.assertTrue(self.io.write_port(0x55))
        self.assertEqual(self.io._output, 0x55)

    def test_write_port_mask(self):
        """端口值掩码为8位"""
        self.io.write_port(0x1FF)
        self.assertEqual(self.io._output, 0xFF)

    def test_read_port(self):
        """读取整个端口"""
        self.io.set_simulated_input(0x3C)
        self.assertEqual(self.io.read_port(), 0x3C)

    def test_write_port_not_initialized(self):
        """未初始化写入端口失败"""
        io2 = PCF8574()
        self.assertFalse(io2.write_port(0xFF))

    def test_read_port_not_initialized(self):
        """未初始化读取端口返回None"""
        io2 = PCF8574()
        self.assertIsNone(io2.read_port())


class TestPCF8574Toggle(unittest.TestCase):
    """PCF8574引脚翻转测试"""

    def setUp(self):
        self.io = PCF8574()
        self.io.init()

    def test_toggle_pin(self):
        """翻转引脚"""
        # 默认全1，翻转pin0应变为0xFE
        self.assertTrue(self.io.toggle_pin(0))
        self.assertEqual(self.io._output, 0xFE)

    def test_toggle_pin_twice(self):
        """翻转两次恢复原值"""
        original = self.io._output
        self.io.toggle_pin(3)
        self.io.toggle_pin(3)
        self.assertEqual(self.io._output, original)

    def test_toggle_all_pins(self):
        """逐个翻转所有引脚"""
        for pin in range(8):
            self.assertTrue(self.io.toggle_pin(pin))
        # 全部翻转后应为0x00
        self.assertEqual(self.io._output, 0x00)

    def test_toggle_invalid_pin(self):
        """无效引脚翻转失败"""
        self.assertFalse(self.io.toggle_pin(-1))
        self.assertFalse(self.io.toggle_pin(8))

    def test_toggle_not_initialized(self):
        """未初始化翻转失败"""
        io2 = PCF8574()
        self.assertFalse(io2.toggle_pin(0))


class TestPCF8574QuasiBidirectional(unittest.TestCase):
    """PCF8574准双向特性测试"""

    def setUp(self):
        self.io = PCF8574()
        self.io.init()

    def test_write_1_is_input_mode(self):
        """写1后引脚处于输入模式（内部上拉）"""
        self.io.write_pin(0, 1)
        # 输出寄存器bit0=1，表示输入/上拉模式
        self.assertTrue(self.io._output & 0x01)

    def test_write_0_is_output_low(self):
        """写0后引脚输出低电平"""
        self.io.write_pin(0, 0)
        self.assertFalse(self.io._output & 0x01)

    def test_mixed_mode(self):
        """混合输入输出模式"""
        # pin 0-3 输出低, pin 4-7 输入/上拉
        self.io.write_port(0xF0)
        self.assertEqual(self.io._output, 0xF0)


class TestPCF8574Constants(unittest.TestCase):
    """PCF8574常量测试"""

    def test_address_range(self):
        """地址范围0x20-0x27"""
        self.assertEqual(PCF8574_ADDR_BASE, 0x20)
        self.assertEqual(PCF8574_ADDR_MAX, 0x27)

    def test_all_addresses(self):
        """所有可用地址"""
        self.assertEqual(PCF8574_ADDR_0, 0x20)
        self.assertEqual(PCF8574_ADDR_1, 0x21)
        self.assertEqual(PCF8574_ADDR_2, 0x22)
        self.assertEqual(PCF8574_ADDR_7, 0x27)


if __name__ == '__main__':
    unittest.main()
