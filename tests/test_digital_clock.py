#!/usr/bin/env python3
"""
数字时钟V2测试 — TM1637四位数码管 + 时间管理
覆盖: TM1637初始化、数字显示、字符串显示、
      亮度控制、时间格式化、闹钟功能、
      秒表/倒计时、格式化显示
对应C源文件: 02_mspm0g3507/drivers/tm1637.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    TM1637, TM1637_SEG_MAP, TM1637_BRIGHTNESS_MAX,
)


class DigitalClockV2:
    """数字时钟V2 — TM1637四位数码管时钟

    功能:
    - 时:分 显示（带冒号闪烁）
    - 闹钟设置和触发
    - 秒表功能
    - 倒计时功能
    - 多种显示格式
    """

    # 显示模式
    MODE_CLOCK = 0      # 时钟模式
    MODE_STOPWATCH = 1  # 秒表模式
    MODE_COUNTDOWN = 2  # 倒计时模式
    MODE_ALARM = 3      # 闹钟设置模式

    def __init__(self):
        self.display = TM1637(num_digits=4)
        self.mode = self.MODE_CLOCK

        # 时钟时间 (时, 分, 秒)
        self.hour = 0
        self.minute = 0
        self.second = 0
        self.colon_on = True  # 冒号状态

        # 闹钟
        self.alarm_hour = 0
        self.alarm_minute = 0
        self.alarm_enabled = False
        self.alarm_triggered = False

        # 秒表 (十分之一秒精度)
        self.stopwatch_running = False
        self.stopwatch_time = 0  # 单位: 0.1秒

        # 倒计时 (秒)
        self.countdown_seconds = 0
        self.countdown_running = False

    def init(self):
        """初始化时钟"""
        self.display.init()
        self.mode = self.MODE_CLOCK

    def set_time(self, hour, minute, second=0):
        """设置时间"""
        self.hour = hour % 24
        self.minute = minute % 60
        self.second = second % 60

    def tick(self):
        """时钟走一秒，返回是否整点"""
        self.second += 1
        if self.second >= 60:
            self.second = 0
            self.minute += 1
            if self.minute >= 60:
                self.minute = 0
                self.hour = (self.hour + 1) % 24
                return True  # 整点
        return False

    def format_hhmm(self):
        """格式化为 HHMM 数字（4位）"""
        return self.hour * 100 + self.minute

    def display_time(self):
        """在数码管上显示时间 HH:MM"""
        if not self.display.initialized:
            return False
        num = self.format_hhmm()
        self.display.display_number(num, pad_with_zero=True)
        return True

    def display_stopwatch(self):
        """显示秒表 MMSS.ss → 显示 MMSS"""
        if not self.display.initialized:
            return False
        # 转换为分秒
        total_sec = self.stopwatch_time // 10
        mins = (total_sec // 60) % 100
        secs = total_sec % 60
        num = mins * 100 + secs
        self.display.display_number(num, pad_with_zero=True)
        return True

    def display_countdown(self):
        """显示倒计时"""
        if not self.display.initialized:
            return False
        mins = (self.countdown_seconds // 60) % 100
        secs = self.countdown_seconds % 60
        num = mins * 100 + secs
        self.display.display_number(num, pad_with_zero=True)
        return True

    def set_alarm(self, hour, minute):
        """设置闹钟"""
        self.alarm_hour = hour % 24
        self.alarm_minute = minute % 60
        self.alarm_enabled = True
        self.alarm_triggered = False

    def check_alarm(self):
        """检查闹钟是否触发，返回True表示触发"""
        if not self.alarm_enabled:
            return False
        if self.alarm_triggered:
            return False
        if self.hour == self.alarm_hour and self.minute == self.alarm_minute:
            if self.second == 0:
                self.alarm_triggered = True
                return True
        return False

    def dismiss_alarm(self):
        """解除闹钟"""
        self.alarm_triggered = False
        self.alarm_enabled = False

    def start_stopwatch(self):
        """启动秒表"""
        self.stopwatch_running = True
        self.mode = self.MODE_STOPWATCH

    def stop_stopwatch(self):
        """停止秒表"""
        self.stopwatch_running = False

    def reset_stopwatch(self):
        """重置秒表"""
        self.stopwatch_time = 0
        self.stopwatch_running = False

    def tick_stopwatch(self):
        """秒表走一格（0.1秒）"""
        if self.stopwatch_running:
            self.stopwatch_time += 1

    def start_countdown(self, seconds):
        """启动倒计时"""
        self.countdown_seconds = seconds
        self.countdown_running = True
        self.mode = self.MODE_COUNTDOWN

    def tick_countdown(self):
        """倒计时走一秒，返回True表示倒计时结束"""
        if not self.countdown_running:
            return False
        self.countdown_seconds -= 1
        if self.countdown_seconds <= 0:
            self.countdown_seconds = 0
            self.countdown_running = False
            return True
        return False


class TestTM1637Init(unittest.TestCase):
    """TM1637初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        clock = DigitalClockV2()
        clock.init()
        self.assertTrue(clock.display.initialized)

    def test_default_brightness(self):
        """默认亮度为7"""
        clock = DigitalClockV2()
        clock.init()
        self.assertEqual(clock.display.brightness, 7)

    def test_display_on(self):
        """显示默认开启"""
        clock = DigitalClockV2()
        clock.init()
        self.assertTrue(clock.display.display_on)


class TestNumberDisplay(unittest.TestCase):
    """数字显示测试"""

    def setUp(self):
        self.clock = DigitalClockV2()
        self.clock.init()

    def test_display_zero(self):
        """显示0000"""
        self.clock.display.display_number(0, pad_with_zero=True)
        # 检查缓冲区: 0→0x3F
        for i in range(4):
            self.assertEqual(self.clock.display.get_digit(i), TM1637_SEG_MAP['0'])

    def test_display_number(self):
        """显示1234"""
        self.clock.display.display_number(1234)
        self.assertEqual(self.clock.display.get_digit(0), TM1637_SEG_MAP['1'])
        self.assertEqual(self.clock.display.get_digit(1), TM1637_SEG_MAP['2'])
        self.assertEqual(self.clock.display.get_digit(2), TM1637_SEG_MAP['3'])
        self.assertEqual(self.clock.display.get_digit(3), TM1637_SEG_MAP['4'])

    def test_display_negative(self):
        """显示负数"""
        self.clock.display.display_number(-5)
        # 第一位应为横杠
        self.assertEqual(self.clock.display.get_digit(0), TM1637_SEG_MAP['-'])

    def test_display_overflow(self):
        """超出范围显示----"""
        self.clock.display.display_number(10000)
        for i in range(4):
            self.assertEqual(self.clock.display.get_digit(i), TM1637_SEG_MAP['-'])

    def test_display_string(self):
        """显示字符串"""
        self.clock.display.display_string("HELP")
        self.assertEqual(self.clock.display.get_digit(0), TM1637_SEG_MAP['H'])
        self.assertEqual(self.clock.display.get_digit(1), TM1637_SEG_MAP['E'])
        self.assertEqual(self.clock.display.get_digit(2), TM1637_SEG_MAP['L'])
        self.assertEqual(self.clock.display.get_digit(3), TM1637_SEG_MAP['P'])


class TestBrightnessControl(unittest.TestCase):
    """亮度控制测试"""

    def setUp(self):
        self.clock = DigitalClockV2()
        self.clock.init()

    def test_set_brightness(self):
        """设置亮度"""
        for level in range(8):
            self.clock.display.set_brightness(level)
            self.assertEqual(self.clock.display.brightness, level)

    def test_brightness_clamp_high(self):
        """亮度超上限限幅"""
        self.clock.display.set_brightness(10)
        self.assertEqual(self.clock.display.brightness, TM1637_BRIGHTNESS_MAX)

    def test_brightness_clamp_low(self):
        """亮度低于0限幅"""
        self.clock.display.set_brightness(-1)
        self.assertEqual(self.clock.display.brightness, 0)


class TestClockTime(unittest.TestCase):
    """时钟时间测试"""

    def setUp(self):
        self.clock = DigitalClockV2()
        self.clock.init()

    def test_set_time(self):
        """设置时间"""
        self.clock.set_time(12, 30, 45)
        self.assertEqual(self.clock.hour, 12)
        self.assertEqual(self.clock.minute, 30)
        self.assertEqual(self.clock.second, 45)

    def test_time_rollover(self):
        """时间溢出回绕"""
        self.clock.set_time(25, 70, 80)
        self.assertEqual(self.clock.hour, 1)   # 25 % 24
        self.assertEqual(self.clock.minute, 10) # 70 % 60
        self.assertEqual(self.clock.second, 20) # 80 % 60

    def test_tick_second(self):
        """秒进位"""
        self.clock.set_time(12, 30, 58)
        self.clock.tick()
        self.assertEqual(self.clock.second, 59)
        self.clock.tick()
        self.assertEqual(self.clock.second, 0)
        self.assertEqual(self.clock.minute, 31)

    def test_tick_minute_rollover(self):
        """分进位到时"""
        self.clock.set_time(12, 59, 59)
        result = self.clock.tick()
        self.assertTrue(result)  # 整点
        self.assertEqual(self.clock.hour, 13)
        self.assertEqual(self.clock.minute, 0)

    def test_tick_hour_rollover(self):
        """时进位到0"""
        self.clock.set_time(23, 59, 59)
        self.clock.tick()
        self.assertEqual(self.clock.hour, 0)

    def test_format_hhmm(self):
        """格式化显示"""
        self.clock.set_time(9, 5)
        self.assertEqual(self.clock.format_hhmm(), 905)

    def test_display_time(self):
        """显示时间到数码管"""
        self.clock.set_time(12, 34)
        self.assertTrue(self.clock.display_time())


class TestAlarm(unittest.TestCase):
    """闹钟测试"""

    def setUp(self):
        self.clock = DigitalClockV2()
        self.clock.init()

    def test_set_alarm(self):
        """设置闹钟"""
        self.clock.set_alarm(7, 30)
        self.assertEqual(self.clock.alarm_hour, 7)
        self.assertEqual(self.clock.alarm_minute, 30)
        self.assertTrue(self.clock.alarm_enabled)

    def test_alarm_trigger(self):
        """闹钟触发"""
        self.clock.set_alarm(8, 0)
        self.clock.set_time(7, 59, 59)
        self.clock.tick()  # → 8:00:00
        self.assertTrue(self.clock.check_alarm())
        self.assertTrue(self.clock.alarm_triggered)

    def test_alarm_not_trigger_wrong_time(self):
        """非闹钟时间不触发"""
        self.clock.set_alarm(8, 0)
        self.clock.set_time(7, 30, 0)
        self.assertFalse(self.clock.check_alarm())

    def test_alarm_once_only(self):
        """闹钟只触发一次"""
        self.clock.set_alarm(8, 0)
        self.clock.set_time(8, 0, 0)
        self.assertTrue(self.clock.check_alarm())
        # 再次检查不应重复触发
        self.assertFalse(self.clock.check_alarm())

    def test_dismiss_alarm(self):
        """解除闹钟"""
        self.clock.set_alarm(8, 0)
        self.clock.dismiss_alarm()
        self.assertFalse(self.clock.alarm_enabled)
        self.assertFalse(self.clock.alarm_triggered)

    def test_alarm_disabled(self):
        """闹钟禁用时不触发"""
        self.clock.set_alarm(8, 0)
        self.clock.dismiss_alarm()
        self.clock.set_time(8, 0, 0)
        self.assertFalse(self.clock.check_alarm())


class TestStopwatch(unittest.TestCase):
    """秒表测试"""

    def setUp(self):
        self.clock = DigitalClockV2()
        self.clock.init()

    def test_start_stopwatch(self):
        """启动秒表"""
        self.clock.start_stopwatch()
        self.assertTrue(self.clock.stopwatch_running)
        self.assertEqual(self.clock.mode, DigitalClockV2.MODE_STOPWATCH)

    def test_stopwatch_tick(self):
        """秒表计时"""
        self.clock.start_stopwatch()
        for _ in range(10):
            self.clock.tick_stopwatch()
        self.assertEqual(self.clock.stopwatch_time, 10)  # 1.0秒

    def test_stop_stopwatch(self):
        """停止秒表"""
        self.clock.start_stopwatch()
        self.clock.tick_stopwatch()
        self.clock.stop_stopwatch()
        self.assertFalse(self.clock.stopwatch_running)
        # 值应保持
        self.assertEqual(self.clock.stopwatch_time, 1)

    def test_reset_stopwatch(self):
        """重置秒表"""
        self.clock.start_stopwatch()
        for _ in range(50):
            self.clock.tick_stopwatch()
        self.clock.reset_stopwatch()
        self.assertEqual(self.clock.stopwatch_time, 0)
        self.assertFalse(self.clock.stopwatch_running)

    def test_stopwatch_not_running(self):
        """秒表未启动时不计时"""
        self.clock.tick_stopwatch()
        self.assertEqual(self.clock.stopwatch_time, 0)


class TestCountdown(unittest.TestCase):
    """倒计时测试"""

    def setUp(self):
        self.clock = DigitalClockV2()
        self.clock.init()

    def test_start_countdown(self):
        """启动倒计时"""
        self.clock.start_countdown(60)
        self.assertEqual(self.clock.countdown_seconds, 60)
        self.assertTrue(self.clock.countdown_running)

    def test_countdown_tick(self):
        """倒计时走秒"""
        self.clock.start_countdown(5)
        for _ in range(4):
            result = self.clock.tick_countdown()
            self.assertFalse(result)
        self.assertEqual(self.clock.countdown_seconds, 1)

    def test_countdown_finish(self):
        """倒计时结束"""
        self.clock.start_countdown(3)
        self.clock.tick_countdown()  # 2
        self.clock.tick_countdown()  # 1
        result = self.clock.tick_countdown()  # 0
        self.assertTrue(result)
        self.assertFalse(self.clock.countdown_running)
        self.assertEqual(self.clock.countdown_seconds, 0)

    def test_countdown_not_running(self):
        """倒计时未启动"""
        result = self.clock.tick_countdown()
        self.assertFalse(result)

    def test_display_countdown(self):
        """显示倒计时"""
        self.clock.start_countdown(125)  # 2分05秒
        self.assertTrue(self.clock.display_countdown())


if __name__ == '__main__':
    unittest.main()
