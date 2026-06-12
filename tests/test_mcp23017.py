#!/usr/bin/env python3
"""
MCP23017 16位IO扩展器 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、I2C地址、引脚方向、端口方向、
      引脚读写、端口读写、模拟输入、边界条件
对应C源文件: 02_mspm0g3507/drivers/mcp23017.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    MCP23017, MCP23017_ADDR_0, MCP23017_ADDR_1,
    MCP23017_ADDR_2, MCP23017_ADDR_7,
    MCP23017_PORTA, MCP23017_PORTB,
)


class TestMCP23017Init(unittest.TestCase):
    """MCP23017初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        io = MCP23017()
        self.assertTrue(io.init())
        self.assertTrue(io.initialized)

    def test_default_address(self):
        """默认I2C地址0x20"""
        io = MCP23017()
        self.assertEqual(io.addr, MCP23017_ADDR_0)

    def test_custom_address(self):
        """自定义I2C地址"""
        io = MCP23017(MCP23017_ADDR_7)
        self.assertEqual(io.addr, MCP23017_ADDR_7)

    def test_default_direction_all_input(self):
        """上电默认全部为输入(0xFF)"""
        io = MCP23017()
        self.assertEqual(io.iodir[0], 0xFF)
        self.assertEqual(io.iodir[1], 0xFF)

    def test_default_gpio_zero(self):
        """上电默认GPIO为0"""
        io = MCP23017()
        self.assertEqual(io.gpio[0], 0x00)
        self.assertEqual(io.gpio[1], 0x00)


class TestMCP23017Direction(unittest.TestCase):
    """MCP23017引脚方向测试"""

    def setUp(self):
        self.io = MCP23017()
        self.io.init()

    def test_set_pin_output(self):
        """设置引脚为输出"""
        self.assertTrue(self.io.set_direction(MCP23017_PORTA, 0, 0))
        self.assertEqual(self.io.get_direction(MCP23017_PORTA, 0), 0)

    def test_set_pin_input(self):
        """设置引脚为输入"""
        self.io.set_direction(MCP23017_PORTA, 0, 0)  # 先设为输出
        self.assertTrue(self.io.set_direction(MCP23017_PORTA, 0, 1))
        self.assertEqual(self.io.get_direction(MCP23017_PORTA, 0), 1)

    def test_set_all_pins(self):
        """设置所有引脚方向"""
        for pin in range(8):
            self.assertTrue(self.io.set_direction(MCP23017_PORTA, pin, 0))
            self.assertEqual(self.io.get_direction(MCP23017_PORTA, pin), 0)

    def test_set_invalid_port(self):
        """无效端口失败"""
        self.assertFalse(self.io.set_direction(2, 0, 0))
        self.assertIsNone(self.io.get_direction(2, 0))

    def test_set_invalid_pin(self):
        """无效引脚失败"""
        self.assertFalse(self.io.set_direction(MCP23017_PORTA, -1, 0))
        self.assertFalse(self.io.set_direction(MCP23017_PORTA, 8, 0))

    def test_set_port_direction(self):
        """设置整个端口方向"""
        self.assertTrue(self.io.set_port_direction(MCP23017_PORTA, 0x0F))
        self.assertEqual(self.io.iodir[MCP23017_PORTA], 0x0F)

    def test_set_port_direction_not_initialized(self):
        """未初始化设置方向失败"""
        io2 = MCP23017()
        self.assertFalse(io2.set_direction(MCP23017_PORTA, 0, 0))


class TestMCP23017WritePin(unittest.TestCase):
    """MCP23017引脚写入测试"""

    def setUp(self):
        self.io = MCP23017()
        self.io.init()

    def test_write_pin_high(self):
        """写入高电平"""
        self.io.set_direction(MCP23017_PORTA, 0, 0)  # 设为输出
        self.assertTrue(self.io.write_pin(MCP23017_PORTA, 0, 1))
        self.assertEqual(self.io.gpio[MCP23017_PORTA] & 0x01, 1)

    def test_write_pin_low(self):
        """写入低电平"""
        self.io.set_direction(MCP23017_PORTA, 0, 0)
        self.io.write_pin(MCP23017_PORTA, 0, 1)  # 先写高
        self.assertTrue(self.io.write_pin(MCP23017_PORTA, 0, 0))
        self.assertEqual(self.io.gpio[MCP23017_PORTA] & 0x01, 0)

    def test_write_input_pin_fails(self):
        """输入模式引脚写入失败"""
        # 默认为输入模式(0xFF)
        self.assertFalse(self.io.write_pin(MCP23017_PORTA, 0, 1))

    def test_write_invalid_port(self):
        """无效端口写入失败"""
        self.assertFalse(self.io.write_pin(2, 0, 1))

    def test_write_invalid_pin(self):
        """无效引脚写入失败"""
        self.assertFalse(self.io.write_pin(MCP23017_PORTA, -1, 1))
        self.assertFalse(self.io.write_pin(MCP23017_PORTA, 8, 1))

    def test_write_not_initialized(self):
        """未初始化写入失败"""
        io2 = MCP23017()
        self.assertFalse(io2.write_pin(MCP23017_PORTA, 0, 1))


class TestMCP23017ReadPin(unittest.TestCase):
    """MCP23017引脚读取测试"""

    def setUp(self):
        self.io = MCP23017()
        self.io.init()

    def test_read_default_zero(self):
        """默认读取为0"""
        self.assertEqual(self.io.read_pin(MCP23017_PORTA, 0), 0)

    def test_read_simulated_input(self):
        """读取模拟输入"""
        self.io.set_simulated_input(MCP23017_PORTA, 0xA5)
        self.assertEqual(self.io.read_pin(MCP23017_PORTA, 0), 1)  # bit0=1
        self.assertEqual(self.io.read_pin(MCP23017_PORTA, 1), 0)  # bit1=0
        self.assertEqual(self.io.read_pin(MCP23017_PORTA, 5), 1)  # bit5=1
        self.assertEqual(self.io.read_pin(MCP23017_PORTA, 7), 1)  # bit7=1

    def test_read_invalid_port(self):
        """无效端口返回None"""
        self.assertIsNone(self.io.read_pin(2, 0))

    def test_read_invalid_pin(self):
        """无效引脚返回None"""
        self.assertIsNone(self.io.read_pin(MCP23017_PORTA, -1))
        self.assertIsNone(self.io.read_pin(MCP23017_PORTA, 8))

    def test_read_not_initialized(self):
        """未初始化返回None"""
        io2 = MCP23017()
        self.assertIsNone(io2.read_pin(MCP23017_PORTA, 0))


class TestMCP23017Port(unittest.TestCase):
    """MCP23017端口操作测试"""

    def setUp(self):
        self.io = MCP23017()
        self.io.init()

    def test_write_port(self):
        """写入整个端口"""
        self.io.set_port_direction(MCP23017_PORTA, 0x00)  # 全部输出
        self.assertTrue(self.io.write_port(MCP23017_PORTA, 0xAA))
        self.assertEqual(self.io.gpio[MCP23017_PORTA], 0xAA)

    def test_read_port(self):
        """读取整个端口"""
        self.io.set_simulated_input(MCP23017_PORTB, 0x55)
        self.assertEqual(self.io.read_port(MCP23017_PORTB), 0x55)

    def test_write_port_not_initialized(self):
        """未初始化写入端口失败"""
        io2 = MCP23017()
        self.assertFalse(io2.write_port(MCP23017_PORTA, 0xFF))

    def test_read_port_not_initialized(self):
        """未初始化读取端口返回None"""
        io2 = MCP23017()
        self.assertIsNone(io2.read_port(MCP23017_PORTA))

    def test_both_ports(self):
        """A和B端口独立操作"""
        self.io.set_port_direction(MCP23017_PORTA, 0x00)
        self.io.set_port_direction(MCP23017_PORTB, 0x00)
        self.io.write_port(MCP23017_PORTA, 0x12)
        self.io.write_port(MCP23017_PORTB, 0x34)
        self.assertEqual(self.io.gpio[MCP23017_PORTA], 0x12)
        self.assertEqual(self.io.gpio[MCP23017_PORTB], 0x34)


class TestMCP23017Constants(unittest.TestCase):
    """MCP23017常量测试"""

    def test_address_values(self):
        """I2C地址值正确"""
        self.assertEqual(MCP23017_ADDR_0, 0x20)
        self.assertEqual(MCP23017_ADDR_7, 0x27)

    def test_port_values(self):
        """端口常量正确"""
        self.assertEqual(MCP23017_PORTA, 0)
        self.assertEqual(MCP23017_PORTB, 1)


if __name__ == '__main__':
    unittest.main()
