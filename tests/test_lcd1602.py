#!/usr/bin/env python3
"""
LCD1602液晶显示 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、清屏、光标定位、字符/字符串写入、显示控制
对应C源文件: 02_mspm0g3507/drivers/lcd1602.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #7:  16列边界溢出保护
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    LCD1602,
    LCD1602_COLS, LCD1602_ROWS,
)


class TestLCD1602V2(unittest.TestCase):
    """LCD1602液晶显示V2 — 基于wrappers.py包装层"""

    def setUp(self):
        self.lcd = LCD1602()
        self.lcd.init()

    def test_init_state(self):
        """初始化后状态正确"""
        self.assertTrue(self.lcd.initialized)
        self.assertTrue(self.lcd.display_on)
        self.assertFalse(self.lcd.cursor_on)
        self.assertFalse(self.lcd.blink_on)
        self.assertEqual(self.lcd.cursor_row, 0)
        self.assertEqual(self.lcd.cursor_col, 0)
        self.assertTrue(self.lcd.backlight)

    def test_clear(self):
        """清屏后缓冲区全空格"""
        self.lcd.write_string("Hello")
        self.lcd.clear()
        for row in range(LCD1602_ROWS):
            self.assertEqual(self.lcd.get_line(row), ' ' * LCD1602_COLS)
        self.assertEqual(self.lcd.cursor_row, 0)
        self.assertEqual(self.lcd.cursor_col, 0)

    def test_home(self):
        """光标回原点"""
        self.lcd.set_cursor(1, 5)
        self.lcd.home()
        self.assertEqual(self.lcd.cursor_row, 0)
        self.assertEqual(self.lcd.cursor_col, 0)

    def test_set_cursor_valid(self):
        """有效光标设置"""
        self.assertTrue(self.lcd.set_cursor(0, 0))
        self.assertTrue(self.lcd.set_cursor(1, 15))
        self.assertTrue(self.lcd.set_cursor(1, 0))

    def test_set_cursor_invalid(self):
        """无效光标设置"""
        self.assertFalse(self.lcd.set_cursor(-1, 0))
        self.assertFalse(self.lcd.set_cursor(2, 0))
        self.assertFalse(self.lcd.set_cursor(0, 16))

    def test_write_char(self):
        """写入单个字符"""
        self.lcd.write_char('A')
        line = self.lcd.get_line(0)
        self.assertEqual(line[0], 'A')
        self.assertEqual(self.lcd.cursor_col, 1)

    def test_write_string(self):
        """写入字符串"""
        self.lcd.write_string("Hello World")
        line = self.lcd.get_line(0)
        self.assertEqual(line[:11], "Hello World")

    def test_write_string_overflow(self):
        """字符串超过16列自动截断"""
        count = self.lcd.write_string("ABCDEFGHIJKLMNOPQRSTUVWXYZ")
        self.assertEqual(count, 16)
        self.assertEqual(self.lcd.cursor_col, 16)

    def test_print_line_row0(self):
        """第0行显示"""
        count = self.lcd.print_line(0, "Line 0 Text")
        self.assertEqual(count, 11)
        self.assertEqual(self.lcd.get_line(0)[:11], "Line 0 Text")

    def test_print_line_row1(self):
        """第1行显示"""
        self.lcd.print_line(1, "Line 1 Text")
        self.assertEqual(self.lcd.get_line(1)[:11], "Line 1 Text")

    def test_print_line_cleans_old(self):
        """print_line清除旧行内容"""
        self.lcd.print_line(0, "Old Content XXXXX")
        self.lcd.print_line(0, "New")
        line = self.lcd.get_line(0)
        self.assertEqual(line[:3], "New")
        # 第4个字符应为空格（旧行被清除）
        self.assertEqual(line[3], ' ')

    def test_print_line_invalid_row(self):
        """无效行号返回0"""
        self.assertEqual(self.lcd.print_line(-1, "test"), 0)
        self.assertEqual(self.lcd.print_line(2, "test"), 0)

    def test_get_display_text(self):
        """获取全部显示内容"""
        self.lcd.print_line(0, "Hello")
        self.lcd.print_line(1, "World")
        lines = self.lcd.get_display_text()
        self.assertEqual(len(lines), 2)
        self.assertEqual(lines[0][:5], "Hello")
        self.assertEqual(lines[1][:5], "World")

    def test_display_on_off(self):
        """显示开关控制"""
        self.lcd.display_on_off(False)
        self.assertFalse(self.lcd.display_on)
        self.lcd.display_on_off(True)
        self.assertTrue(self.lcd.display_on)

    def test_cursor_on_off(self):
        """光标开关控制"""
        self.lcd.cursor_on_off(True)
        self.assertTrue(self.lcd.cursor_on)
        self.lcd.cursor_on_off(False)
        self.assertFalse(self.lcd.cursor_on)

    def test_blink_on_off(self):
        """闪烁开关控制"""
        self.lcd.blink_on_off(True)
        self.assertTrue(self.lcd.blink_on)
        self.lcd.blink_on_off(False)
        self.assertFalse(self.lcd.blink_on)

    def test_set_backlight(self):
        """背光控制"""
        self.lcd.set_backlight(False)
        self.assertFalse(self.lcd.backlight)
        self.lcd.set_backlight(True)
        self.assertTrue(self.lcd.backlight)

    def test_write_char_not_initialized(self):
        """未初始化时写入失败"""
        lcd2 = LCD1602()
        self.assertFalse(lcd2.write_char('X'))

    def test_write_string_not_initialized(self):
        """未初始化时写入字符串失败"""
        lcd2 = LCD1602()
        self.assertFalse(lcd2.write_string("test"))

    def test_two_lines_independent(self):
        """两行独立显示"""
        self.lcd.print_line(0, "AAAA")
        self.lcd.print_line(1, "BBBB")
        self.assertEqual(self.lcd.get_line(0)[:4], "AAAA")
        self.assertEqual(self.lcd.get_line(1)[:4], "BBBB")

    def test_empty_string(self):
        """空字符串不影响显示"""
        self.lcd.print_line(0, "Hello")
        self.lcd.print_line(0, "")
        self.assertEqual(self.lcd.get_line(0)[0], ' ')

    def test_full_line(self):
        """恰好写满一行（16字符）"""
        text = "1234567890123456"
        count = self.lcd.print_line(0, text)
        self.assertEqual(count, 16)
        self.assertEqual(self.lcd.get_line(0), text)

    def test_chinese_like_chars(self):
        """多字节字符模拟写入"""
        self.lcd.write_string("AB12")
        line = self.lcd.get_line(0)
        self.assertEqual(line[:4], "AB12")


if __name__ == '__main__':
    unittest.main()
