#!/usr/bin/env python3
"""
IO扩展键盘 V2 测试 — 基于wrappers.py包装层
覆盖: MCP23017 16位IO扩展器 + PCF8574 8位IO扩展器
模拟场景: 矩阵键盘扫描、LED指示灯控制、中断检测
对应C源文件: 02_mspm0g3507/drivers/mcp23017.c
              02_mspm0g3507/drivers/pcf8574.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    MCP23017, MCP23017_ADDR_0, MCP23017_ADDR_1,
    MCP23017_PORTA, MCP23017_PORTB,
    PCF8574, PCF8574_ADDR_0, PCF8574_ADDR_1,
)


class TestMCP23017Init(unittest.TestCase):
    """MCP23017初始化测试"""

    def test_init_success(self):
        """MCP23017初始化成功"""
        io = MCP23017()
        self.assertTrue(io.init())
        self.assertTrue(io.initialized)

    def test_default_address(self):
        """默认I2C地址0x20"""
        io = MCP23017()
        self.assertEqual(io.addr, MCP23017_ADDR_0)

    def test_custom_address(self):
        """自定义I2C地址"""
        io = MCP23017(MCP23017_ADDR_1)
        self.assertEqual(io.addr, MCP23017_ADDR_1)

    def test_default_direction(self):
        """默认所有引脚为输入（方向寄存器全1）"""
        io = MCP23017()
        io.init()
        # 所有引脚默认输入
        for pin in range(8):
            self.assertEqual(io.get_direction(MCP23017_PORTA, pin), 1)
            self.assertEqual(io.get_direction(MCP23017_PORTB, pin), 1)


class TestMCP23017Direction(unittest.TestCase):
    """MCP23017引脚方向测试"""

    def setUp(self):
        self.io = MCP23017()
        self.io.init()

    def test_set_output_direction(self):
        """设置引脚为输出"""
        self.assertTrue(self.io.set_direction(MCP23017_PORTA, 0, 0))
        self.assertEqual(self.io.get_direction(MCP23017_PORTA, 0), 0)

    def test_set_input_direction(self):
        """设置引脚为输入"""
        self.io.set_direction(MCP23017_PORTA, 3, 0)  # 先设为输出
        self.io.set_direction(MCP23017_PORTA, 3, 1)  # 再设为输入
        self.assertEqual(self.io.get_direction(MCP23017_PORTA, 3), 1)

    def test_set_port_direction(self):
        """设置整个端口方向"""
        self.assertTrue(self.io.set_port_direction(MCP23017_PORTA, 0xF0))
        # 0xF0 = 11110000: 低4位输出(0)，高4位输入(1)
        for pin in range(4):
            self.assertEqual(self.io.get_direction(MCP23017_PORTA, pin), 0)
        for pin in range(4, 8):
            self.assertEqual(self.io.get_direction(MCP23017_PORTA, pin), 1)

    def test_invalid_port(self):
        """无效端口失败"""
        self.assertIsNone(self.io.get_direction(2, 0))
        self.assertFalse(self.io.set_direction(2, 0, 1))

    def test_invalid_pin(self):
        """无效引脚失败"""
        self.assertIsNone(self.io.get_direction(MCP23017_PORTA, -1))
        self.assertIsNone(self.io.get_direction(MCP23017_PORTA, 8))
        self.assertFalse(self.io.set_direction(MCP23017_PORTA, -1, 1))
        self.assertFalse(self.io.set_direction(MCP23017_PORTA, 8, 1))


class TestMCP23017ReadWrite(unittest.TestCase):
    """MCP23017读写测试"""

    def setUp(self):
        self.io = MCP23017()
        self.io.init()
        # 端口A低4位设为输出
        for pin in range(4):
            self.io.set_direction(MCP23017_PORTA, pin, 0)

    def test_write_pin_high(self):
        """写入引脚高电平"""
        self.assertTrue(self.io.write_pin(MCP23017_PORTA, 0, 1))
        self.assertEqual(self.io.read_pin(MCP23017_PORTA, 0), 1)

    def test_write_pin_low(self):
        """写入引脚低电平"""
        self.io.write_pin(MCP23017_PORTA, 1, 1)
        self.assertTrue(self.io.write_pin(MCP23017_PORTA, 1, 0))
        self.assertEqual(self.io.read_pin(MCP23017_PORTA, 1), 0)

    def test_write_input_pin_fails(self):
        """写入输入模式引脚失败"""
        # 端口A pin7默认输入
        self.assertFalse(self.io.write_pin(MCP23017_PORTA, 7, 1))

    def test_write_port(self):
        """写入整个端口"""
        self.assertTrue(self.io.write_port(MCP23017_PORTA, 0x0F))
        # 输出引脚应读到对应值
        for pin in range(4):
            self.assertEqual(self.io.read_pin(MCP23017_PORTA, pin), 1)

    def test_read_port(self):
        """读取整个端口"""
        self.io.set_simulated_input(MCP23017_PORTB, 0xA5)
        self.assertEqual(self.io.read_port(MCP23017_PORTB), 0xA5)

    def test_simulated_input(self):
        """模拟外部输入"""
        self.io.set_simulated_input(MCP23017_PORTA, 0x55)
        # 读取单个引脚
        self.assertEqual(self.io.read_pin(MCP23017_PORTA, 0), 1)
        self.assertEqual(self.io.read_pin(MCP23017_PORTA, 1), 0)
        self.assertEqual(self.io.read_pin(MCP23017_PORTA, 4), 1)

    def test_not_initialized(self):
        """未初始化操作失败"""
        io2 = MCP23017()
        self.assertFalse(io2.write_pin(MCP23017_PORTA, 0, 1))
        self.assertIsNone(io2.read_pin(MCP23017_PORTA, 0))
        self.assertFalse(io2.write_port(MCP23017_PORTA, 0xFF))
        self.assertIsNone(io2.read_port(MCP23017_PORTA))


class TestMCP23017KeypadMatrix(unittest.TestCase):
    """MCP23017矩阵键盘扫描测试"""

    def setUp(self):
        """配置4x4矩阵键盘：PA低4位输出（行），PB低4位输入（列）"""
        self.io = MCP23017()
        self.io.init()
        # 行引脚：PA0-PA3 输出
        for pin in range(4):
            self.io.set_direction(MCP23017_PORTA, pin, 0)
        # 列引脚：PB0-PB3 输入（默认即为输入）
        # 高4位也设为输出用于LED指示
        for pin in range(4, 8):
            self.io.set_direction(MCP23017_PORTA, pin, 0)

    def test_scan_no_key(self):
        """无按键按下时扫描返回-1"""
        self.io.set_simulated_input(MCP23017_PORTB, 0x00)
        # 逐行扫描
        found = False
        for row in range(4):
            # 拉低当前行
            self.io.write_port(MCP23017_PORTA, ~(1 << row) & 0x0F)
            cols = self.io.read_port(MCP23017_PORTB) & 0x0F
            if cols != 0:
                found = True
                break
        self.assertFalse(found)

    def test_scan_key_pressed(self):
        """按键按下检测"""
        # 模拟第1行第2列按下
        # 扫描第1行（PA1拉低）时，PB2读到高
        self.io.write_port(MCP23017_PORTA, 0x0D)  # PA1=0, 其他=1
        self.io.set_simulated_input(MCP23017_PORTB, 0x04)  # PB2=1
        cols = self.io.read_port(MCP23017_PORTB) & 0x0F
        self.assertEqual(cols, 0x04)  # 检测到PB2

    def test_led_indicator(self):
        """LED指示灯控制"""
        # PA4-PA7控制LED
        for pin in range(4, 8):
            self.io.write_pin(MCP23017_PORTA, pin, 1)
        # 验证高4位输出
        port_val = self.io.read_port(MCP23017_PORTA)
        self.assertEqual(port_val & 0xF0, 0xF0)

    def test_toggle_led(self):
        """LED翻转"""
        self.io.write_pin(MCP23017_PORTA, 4, 1)
        self.assertEqual(self.io.read_pin(MCP23017_PORTA, 4), 1)
        self.io.write_pin(MCP23017_PORTA, 4, 0)
        self.assertEqual(self.io.read_pin(MCP23017_PORTA, 4), 0)


class TestPCF8574Init(unittest.TestCase):
    """PCF8574初始化测试"""

    def test_init_success(self):
        """PCF8574初始化成功"""
        io = PCF8574()
        self.assertTrue(io.init())
        self.assertTrue(io.initialized)

    def test_default_address(self):
        """默认I2C地址0x20"""
        io = PCF8574()
        self.assertEqual(io.addr, PCF8574_ADDR_0)

    def test_custom_address(self):
        """自定义I2C地址"""
        io = PCF8574(PCF8574_ADDR_1)
        self.assertEqual(io.addr, PCF8574_ADDR_1)


class TestPCF8574ReadWrite(unittest.TestCase):
    """PCF8574读写测试"""

    def setUp(self):
        self.io = PCF8574()
        self.io.init()

    def test_write_pin(self):
        """写入引脚"""
        self.assertTrue(self.io.write_pin(0, 0))  # 输出低
        self.assertTrue(self.io.write_pin(1, 1))  # 输入/上拉

    def test_read_pin(self):
        """读取引脚"""
        self.io.set_simulated_input(0xA5)
        self.assertEqual(self.io.read_pin(0), 1)
        self.assertEqual(self.io.read_pin(1), 0)
        self.assertEqual(self.io.read_pin(5), 1)

    def test_write_port(self):
        """写入整个端口"""
        self.assertTrue(self.io.write_port(0x55))

    def test_read_port(self):
        """读取整个端口"""
        self.io.set_simulated_input(0xFF)
        self.assertEqual(self.io.read_port(), 0xFF)

    def test_toggle_pin(self):
        """翻转引脚"""
        self.io.write_port(0x00)
        self.io.toggle_pin(3)
        # toggle后pin3应为1
        output = self.io._output
        self.assertTrue(output & (1 << 3))

    def test_invalid_pin(self):
        """无效引脚失败"""
        self.assertFalse(self.io.write_pin(-1, 1))
        self.assertFalse(self.io.write_pin(8, 1))
        self.assertIsNone(self.io.read_pin(-1))
        self.assertIsNone(self.io.read_pin(8))

    def test_not_initialized(self):
        """未初始化操作失败"""
        io2 = PCF8574()
        self.assertFalse(io2.write_pin(0, 1))
        self.assertIsNone(io2.read_pin(0))
        self.assertIsNone(io2.read_port())

    def test_toggle_not_initialized(self):
        """未初始化翻转失败"""
        io2 = PCF8574()
        self.assertFalse(io2.toggle_pin(0))


if __name__ == '__main__':
    unittest.main()
