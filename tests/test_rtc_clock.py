# -*- coding: utf-8 -*-
"""
test_rtc_clock_v2.py - RTC时钟测试 V2
=====================================
测试内容：
  1. 时钟系统初始化
  2. PLL配置与频率设置
  3. 外设分频器配置
  4. 定时器周期计算
  5. UART波特率分频
  6. I2C/SPI频率验证
  7. 时钟源切换
  8. 闰年/日期计算
  9. 闹钟与倒计时
  10. 秒表与数据记录

使用 wrappers.py 封装的 ClockGen、TaskScheduler、SystemDiag、PIDController
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from wrappers import ClockGen, TaskScheduler, SystemDiag

# ---- 测试辅助函数 ----
def assert_close(actual, expected, tolerance=0.01, msg=""):
    """断言浮点数接近"""
    if abs(actual - expected) > tolerance:
        raise AssertionError(f"{msg}: 期望 {expected}±{tolerance}, 实际 {actual}")

def assert_in_range(value, min_val, max_val, msg=""):
    """断言值在范围内"""
    if value < min_val or value > max_val:
        raise AssertionError(f"{msg}: {value} 不在 [{min_val}, {max_val}] 范围内")

def run_test(test_func, test_name=""):
    """运行单个测试函数"""
    try:
        test_func()
        print(f"  [通过] {test_name}")
        return True
    except AssertionError as e:
        print(f"  [失败] {test_name}: {e}")
        return False
    except Exception as e:
        print(f"  [错误] {test_name}: {type(e).__name__}: {e}")
        return False


class RTCClockSystem:
    """RTC时钟系统 - 集成ClockGen + TaskScheduler + PIDController"""

    def __init__(self):
        self.clock = ClockGen()
        self.clock.init()
        self.scheduler = TaskScheduler()
        self.scheduler.init()
        self.diag = SystemDiag()
        # RTC模拟状态
        self.year, self.month, self.day = 2024, 1, 1
        self.hour, self.minute, self.second = 0, 0, 0
        self.tick_count = 0
        # 闹钟
        self.alarm_enabled = False
        self.alarm_hour, self.alarm_minute = 0, 0
        self.alarm_triggered = False
        # 倒计时
        self.countdown_running = False
        self.countdown_remaining = 0
        self.countdown_finished = False
        # 秒表
        self.stopwatch_running = False
        self.stopwatch_elapsed_ms = 0

    def is_leap_year(self, year=None):
        y = year or self.year
        return (y % 4 == 0 and y % 100 != 0) or (y % 400 == 0)

    def get_days_in_month(self, month=None, year=None):
        m, y = month or self.month, year or self.year
        days = [0,31,28,31,30,31,30,31,31,30,31,30,31]
        return 29 if (m == 2 and self.is_leap_year(y)) else days[m]

    def tick(self, ticks=1):
        for _ in range(ticks):
            self.tick_count += 1
            self.second += 1
            if self.second >= 60:
                self.second = 0; self.minute += 1
                if self.minute >= 60:
                    self.minute = 0; self.hour += 1
                    if self.hour >= 24:
                        self.hour = 0; self.day += 1
                        if self.day > self.get_days_in_month():
                            self.day = 1; self.month += 1
                            if self.month > 12:
                                self.month = 1; self.year += 1
            if self.alarm_enabled and not self.alarm_triggered:
                if self.hour == self.alarm_hour and self.minute == self.alarm_minute and self.second == 0:
                    self.alarm_triggered = True
            if self.countdown_running:
                self.countdown_remaining -= 1
                if self.countdown_remaining <= 0:
                    self.countdown_remaining = 0
                    self.countdown_running = False
                    self.countdown_finished = True
            if self.stopwatch_running:
                self.stopwatch_elapsed_ms += 1000

    def set_alarm(self, h, m):
        self.alarm_hour, self.alarm_minute = h, m
        self.alarm_enabled = True; self.alarm_triggered = False

    def clear_alarm(self):
        self.alarm_enabled = False; self.alarm_triggered = False

    def start_countdown(self, s):
        self.countdown_remaining = s
        self.countdown_running = True; self.countdown_finished = False

    def start_stopwatch(self):
        self.stopwatch_running = True; self.stopwatch_elapsed_ms = 0

    def stop_stopwatch(self):
        self.stopwatch_running = False

    def get_time_str(self):
        return f"{self.hour:02d}:{self.minute:02d}:{self.second:02d}"


def test_init():
    """RTC时钟系统初始化"""
    rtc = RTCClockSystem()
    assert rtc.year == 2024 and rtc.hour == 0 and rtc.second == 0
    assert not rtc.alarm_enabled

def test_clock_gen_init():
    """ClockGen初始化"""
    cg = ClockGen(); cg.init()
    assert cg.get_sysclk_freq() > 0

def test_pll_config():
    """PLL配置"""
    cg = ClockGen(); cg.init()
    cg.enable_pll(16)
    freq = cg.get_sysclk_freq()
    assert freq > 0

def test_sysclk_source():
    """时钟源切换"""
    cg = ClockGen(); cg.init()
    for src in [0, 1]:
        cg.set_sysclk_source(src)
        assert cg.get_sysclk_freq() > 0

def test_peripheral_divider():
    """外设分频"""
    cg = ClockGen(); cg.init()
    for div in [1, 2, 4]:
        cg.set_peripheral_divider(div)
        assert cg.get_timer_freq() > 0

def test_timer_period():
    """定时器周期计算"""
    cg = ClockGen(); cg.init()
    p = cg.calc_timer_period(1000)
    assert p > 0

def test_uart_baud():
    """UART波特率分频"""
    cg = ClockGen(); cg.init()
    d = cg.calc_uart_baud_div(115200)
    assert d > 0

def test_i2c_spi_freq():
    """I2C/SPI频率"""
    cg = ClockGen(); cg.init()
    assert cg.get_i2c_freq() > 0
    assert cg.get_spi_freq() > 0

def test_clock_source_name():
    """时钟源名称"""
    cg = ClockGen(); cg.init()
    name = cg.get_clock_source_name()
    assert isinstance(name, str) and len(name) > 0

def test_leap_year():
    """闰年判断"""
    rtc = RTCClockSystem()
    assert rtc.is_leap_year(2024) is True
    assert rtc.is_leap_year(2000) is True
    assert rtc.is_leap_year(1900) is False
    assert rtc.is_leap_year(2023) is False

def test_days_in_month():
    """每月天数"""
    rtc = RTCClockSystem()
    assert rtc.get_days_in_month(1, 2023) == 31
    assert rtc.get_days_in_month(2, 2023) == 28
    assert rtc.get_days_in_month(2, 2024) == 29

def test_tick():
    """Tick推进"""
    rtc = RTCClockSystem()
    rtc.tick(1); assert rtc.second == 1
    rtc.tick(59); assert rtc.minute == 1

def test_tick_carry():
    """进位测试"""
    rtc = RTCClockSystem()
    rtc.hour, rtc.minute, rtc.second = 23, 59, 55
    rtc.tick(5)
    assert rtc.hour == 0 and rtc.day == 2

def test_alarm():
    """闹钟触发"""
    rtc = RTCClockSystem()
    rtc.hour, rtc.minute, rtc.second = 7, 59, 59
    rtc.set_alarm(8, 0)
    rtc.tick(1)
    assert rtc.alarm_triggered

def test_clear_alarm():
    """清除闹钟"""
    rtc = RTCClockSystem()
    rtc.set_alarm(8, 0)
    rtc.clear_alarm()
    assert not rtc.alarm_enabled

def test_countdown():
    """倒计时"""
    rtc = RTCClockSystem()
    rtc.start_countdown(10)
    rtc.tick(7); assert rtc.countdown_remaining == 3
    rtc.tick(3)
    assert rtc.countdown_finished

def test_stopwatch():
    """秒表"""
    rtc = RTCClockSystem()
    rtc.start_stopwatch()
    rtc.tick(5); assert rtc.stopwatch_elapsed_ms == 5000
    rtc.stop_stopwatch(); assert not rtc.stopwatch_running

def test_time_str():
    """时间字符串"""
    rtc = RTCClockSystem()
    rtc.hour, rtc.minute, rtc.second = 14, 30, 45
    assert rtc.get_time_str() == "14:30:45"

def test_scheduler():
    """任务调度器"""
    s = TaskScheduler(); s.init()
    assert s.get_task_count() >= 0

def test_pid_clock():
    """PID控制器（用于时钟校准）"""
    from wrappers import PIDController
    pid = PIDController()
    pid.set_kp(1.0); pid.set_ki(0.1); pid.set_kd(0.01)
    out = pid.calc(1000, 900)
    assert pid.get_output() != 0


def main():
    print("=" * 60)
    print("  RTC时钟系统测试 V2")
    print("=" * 60)
    tests = [
        (test_init, "初始化"), (test_clock_gen_init, "ClockGen初始化"),
        (test_pll_config, "PLL配置"), (test_sysclk_source, "时钟源切换"),
        (test_peripheral_divider, "外设分频"), (test_timer_period, "定时器周期"),
        (test_uart_baud, "UART波特率"), (test_i2c_spi_freq, "I2C/SPI频率"),
        (test_clock_source_name, "时钟源名称"), (test_leap_year, "闰年判断"),
        (test_days_in_month, "每月天数"), (test_tick, "Tick推进"),
        (test_tick_carry, "进位测试"), (test_alarm, "闹钟触发"),
        (test_clear_alarm, "清除闹钟"), (test_countdown, "倒计时"),
        (test_stopwatch, "秒表"), (test_time_str, "时间字符串"),
        (test_scheduler, "任务调度器"), (test_pid_clock, "PID时钟校准"),
    ]
    passed = failed = 0
    for func, name in tests:
        if run_test(func, name): passed += 1
        else: failed += 1
    print("-" * 60)
    print(f"  结果: {passed} 通过, {failed} 失败, 共 {passed + failed} 项")
    print("=" * 60)
    return failed == 0

if __name__ == "__main__":
    sys.exit(0 if main() else 1)
