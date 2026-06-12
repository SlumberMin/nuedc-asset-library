#!/usr/bin/env python3
"""
TM1637数码管 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、段码设置、数字显示、字符串显示、亮度控制、
      冒号控制、缓冲区、边界条件
对应C源文件: 02_mspm0g3507/drivers/tm1637.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    TM1637, TM1637_MAX_DIGITS, TM1637_BRIGHTNESS_MAX, TM1637_SEG_MAP,
)


class TestTM1637Init(unittest.TestCase):
    """TM1637初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        tm = TM1637(4)
        self.assertTrue(tm.init())
        self.assertTrue(tm.initialized)

    def test_init_brightness_max(self):
        """初始化后亮度为7"""
        tm = TM1637(4)
        tm.init()
        self.assertEqual(tm.brightness, TM1637_BRIGHTNESS_MAX)

    def test_init_display_on(self):
        """初始化后显示开启"""
        tm = TM1637(4)
        tm.init()
        self.assertTrue(tm.display_on)

    def test_init_buffer_cleared(self):
        """初始化后缓冲区全零"""
        tm = TM1637(4)
        tm.init()
        buf = tm.get_buffer()
        self.assertEqual(buf, [0x00, 0x00, 0x00, 0x00])

    def test_default_4_digits(self):
        """默认4位数码管"""
        tm = TM1637()
        self.assertEqual(tm.num_digits, 4)

    def test_6_digits(self):
        """6位数码管"""
        tm = TM1637(6)
        self.assertEqual(tm.num_digits, 6)

    def test_max_digits_clamp(self):
        """超过最大位数被限制"""
        tm = TM1637(10)
        self.assertEqual(tm.num_digits, TM1637_MAX_DIGITS)

    def test_min_digits_clamp(self):
        """0位修正为1"""
        tm = TM1637(0)
        self.assertEqual(tm.num_digits, 1)


class TestTM1637SetDigit(unittest.TestCase):
    """TM1637段码设置测试"""

    def setUp(self):
        self.tm = TM1637(4)
        self.tm.init()

    def test_set_digit_valid(self):
        """有效位置设置段码"""
        self.assertTrue(self.tm.set_digit(0, 0x3F))
        self.assertEqual(self.tm.get_digit(0), 0x3F)

    def test_set_digit_all_positions(self):
        """所有位置可设置"""
        for i in range(4):
            self.assertTrue(self.tm.set_digit(i, 0x06))

    def test_set_digit_out_of_range(self):
        """越界位置失败"""
        self.assertFalse(self.tm.set_digit(-1, 0x00))
        self.assertFalse(self.tm.set_digit(4, 0x00))

    def test_set_digit_not_initialized(self):
        """未初始化时失败"""
        tm2 = TM1637(4)
        self.assertFalse(tm2.set_digit(0, 0x3F))

    def test_set_digit_mask_8bit(self):
        """段码仅保留低8位"""
        self.tm.set_digit(0, 0x1FF)
        self.assertEqual(self.tm.get_digit(0), 0xFF)


class TestTM1637DisplayNumber(unittest.TestCase):
    """TM1637数字显示测试"""

    def setUp(self):
        self.tm = TM1637(4)
        self.tm.init()

    def test_display_zero(self):
        """显示0"""
        self.assertTrue(self.tm.display_number(0))
        self.assertEqual(self.tm.get_digit(3), TM1637_SEG_MAP['0'])

    def test_display_single_digit(self):
        """显示个位数"""
        self.tm.display_number(5)
        self.assertEqual(self.tm.get_digit(3), TM1637_SEG_MAP['5'])

    def test_display_two_digits(self):
        """显示两位数"""
        self.tm.display_number(42)
        self.assertEqual(self.tm.get_digit(2), TM1637_SEG_MAP['4'])
        self.assertEqual(self.tm.get_digit(3), TM1637_SEG_MAP['2'])

    def test_display_max_4_digits(self):
        """显示最大4位数9999"""
        self.assertTrue(self.tm.display_number(9999))
        self.assertEqual(self.tm.get_digit(0), TM1637_SEG_MAP['9'])

    def test_display_overflow_dash(self):
        """超出范围显示横杠"""
        self.assertFalse(self.tm.display_number(10000))
        for i in range(4):
            self.assertEqual(self.tm.get_digit(i), TM1637_SEG_MAP['-'])

    def test_display_negative(self):
        """显示负数"""
        self.tm.display_number(-42)
        self.assertEqual(self.tm.get_digit(0), TM1637_SEG_MAP['-'])
        self.assertEqual(self.tm.get_digit(2), TM1637_SEG_MAP['4'])
        self.assertEqual(self.tm.get_digit(3), TM1637_SEG_MAP['2'])

    def test_display_pad_with_zero(self):
        """前导零填充"""
        self.tm.display_number(5, pad_with_zero=True)
        self.assertEqual(self.tm.get_digit(0), TM1637_SEG_MAP['0'])
        self.assertEqual(self.tm.get_digit(1), TM1637_SEG_MAP['0'])
        self.assertEqual(self.tm.get_digit(2), TM1637_SEG_MAP['0'])
        self.assertEqual(self.tm.get_digit(3), TM1637_SEG_MAP['5'])

    def test_display_number_not_initialized(self):
        """未初始化显示失败"""
        tm2 = TM1637(4)
        self.assertFalse(tm2.display_number(123))


class TestTM1637DisplayString(unittest.TestCase):
    """TM1637字符串显示测试"""

    def setUp(self):
        self.tm = TM1637(4)
        self.tm.init()

    def test_display_string(self):
        """显示字符串"""
        self.assertTrue(self.tm.display_string("1234"))
        self.assertEqual(self.tm.get_digit(0), TM1637_SEG_MAP['1'])
        self.assertEqual(self.tm.get_digit(3), TM1637_SEG_MAP['4'])

    def test_display_string_partial(self):
        """短字符串左侧填充空白"""
        self.tm.display_string("Ab")
        self.assertEqual(self.tm.get_digit(0), TM1637_SEG_MAP['A'])
        self.assertEqual(self.tm.get_digit(1), TM1637_SEG_MAP['b'])
        self.assertEqual(self.tm.get_digit(2), 0x00)

    def test_display_string_truncate(self):
        """超长字符串截断"""
        self.tm.display_string("12345678")
        # 仅前4位有效
        self.assertEqual(self.tm.get_digit(3), TM1637_SEG_MAP['4'])

    def test_display_dash(self):
        """显示横杠"""
        self.tm.display_string("----")
        for i in range(4):
            self.assertEqual(self.tm.get_digit(i), TM1637_SEG_MAP['-'])

    def test_display_unknown_char(self):
        """未知字符段码为0"""
        self.tm.display_string("X")
        self.assertEqual(self.tm.get_digit(0), 0x00)


class TestTM1637Colon(unittest.TestCase):
    """TM1637冒号控制测试"""

    def setUp(self):
        self.tm = TM1637(4)
        self.tm.init()

    def test_colon_on(self):
        """开启冒号"""
        self.tm.set_colon(True)
        # 冒号在digit[1]的最高位
        self.assertTrue(self.tm.get_digit(1) & 0x80)

    def test_colon_off(self):
        """关闭冒号"""
        self.tm.set_colon(True)
        self.tm.set_colon(False)
        self.assertFalse(self.tm.get_digit(1) & 0x80)

    def test_colon_preserves_seg(self):
        """冒号不影响段数据"""
        self.tm.set_digit(1, 0x06)
        self.tm.set_colon(True)
        self.assertEqual(self.tm.get_digit(1), 0x06 | 0x80)

    def test_colon_2_digits(self):
        """2位数码管也有冒号"""
        tm2 = TM1637(2)
        tm2.init()
        tm2.set_colon(True)
        self.assertTrue(tm2.get_digit(1) & 0x80)

    def test_colon_1_digit_no_effect(self):
        """1位数码管无冒号"""
        tm1 = TM1637(1)
        tm1.init()
        tm1.set_colon(True)  # 不应报错


class TestTM1637Brightness(unittest.TestCase):
    """TM1637亮度测试"""

    def test_set_brightness_normal(self):
        """设置亮度"""
        tm = TM1637(4)
        tm.init()
        tm.set_brightness(3)
        self.assertEqual(tm.brightness, 3)

    def test_set_brightness_clamp_max(self):
        """亮度超上限被限幅"""
        tm = TM1637(4)
        tm.init()
        tm.set_brightness(10)
        self.assertEqual(tm.brightness, TM1637_BRIGHTNESS_MAX)

    def test_set_brightness_clamp_min(self):
        """亮度低于0被限幅"""
        tm = TM1637(4)
        tm.init()
        tm.set_brightness(-1)
        self.assertEqual(tm.brightness, 0)


class TestTM1637Raw(unittest.TestCase):
    """TM1637原始数据测试"""

    def test_display_raw(self):
        """直接设置原始段数据"""
        tm = TM1637(4)
        tm.init()
        data = [0x3F, 0x06, 0x5B, 0x4F]
        self.assertTrue(tm.display_raw(data))
        self.assertEqual(tm.get_buffer(), data)

    def test_display_raw_partial(self):
        """部分数据填充"""
        tm = TM1637(4)
        tm.init()
        tm.display_raw([0xFF, 0xFF])
        self.assertEqual(tm.get_digit(0), 0xFF)
        self.assertEqual(tm.get_digit(1), 0xFF)
        self.assertEqual(tm.get_digit(2), 0x00)


class TestTM1637Clear(unittest.TestCase):
    """TM1637清除测试"""

    def test_clear(self):
        """清除后缓冲区全零"""
        tm = TM1637(4)
        tm.init()
        tm.display_number(8888)
        tm.clear()
        self.assertEqual(tm.get_buffer(), [0x00, 0x00, 0x00, 0x00])


class TestTM1637SegMap(unittest.TestCase):
    """TM1637段码表测试"""

    def test_seg_map_digits(self):
        """0-9段码非零"""
        for d in range(10):
            self.assertNotEqual(TM1637_SEG_MAP[chr(ord('0') + d)], 0)

    def test_seg_map_space(self):
        """空格段码为0"""
        self.assertEqual(TM1637_SEG_MAP[' '], 0x00)

    def test_seg_map_dash(self):
        """横杠段码"""
        self.assertEqual(TM1637_SEG_MAP['-'], 0x40)


if __name__ == '__main__':
    unittest.main()
