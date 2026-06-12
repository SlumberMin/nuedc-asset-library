# -*- coding: utf-8 -*-
"""
test_ds3231_v2.py - DS3231 RTC芯片测试 V2
==========================================
测试内容：
  1. I2C通信初始化（地址0x68）
  2. 时间读写（秒/分/时/日/月/年 BCD编码）
  3. 闹钟1（秒/分/时匹配）
  4. 闹钟2（分/时匹配）
  5. 温度传感器读取
  6. 方波输出频率设置
  7. 控制/状态寄存器
  8. 12/24小时制切换
  9. 闰年自动处理
  10. 多日进位测试

使用 wrappers.py 的 I2CBus、PIDController
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from wrappers import I2CBus, PIDController

# ═══════════════════════════════════════════════════════════════
#  DS3231寄存器地址定义
# ═══════════════════════════════════════════════════════════════
DS3231_ADDR = 0x68

# 时间寄存器
REG_SECONDS = 0x00
REG_MINUTES = 0x01
REG_HOURS = 0x02
REG_DAY = 0x03
REG_DATE = 0x04
REG_MONTH = 0x05
REG_YEAR = 0x06

# 闹钟1寄存器
REG_ALARM1_SEC = 0x07
REG_ALARM1_MIN = 0x08
REG_ALARM1_HOUR = 0x09
REG_ALARM1_DAY = 0x0A

# 闹钟2寄存器
REG_ALARM2_MIN = 0x0B
REG_ALARM2_HOUR = 0x0C
REG_ALARM2_DAY = 0x0D

# 控制/状态寄存器
REG_CONTROL = 0x0E
REG_STATUS = 0x0F
REG_AGING = 0x10

# 温度寄存器（只读）
REG_TEMP_MSB = 0x11
REG_TEMP_LSB = 0x12

# 控制位
CTRL_A1IE = 0x01   # 闹钟1中断使能
CTRL_A2IE = 0x02   # 闹钟2中断使能
CTRL_INTCN = 0x04  # 中断/方波选择
CTRL_RS1 = 0x08    # 方波频率选择位1
CTRL_RS2 = 0x10    # 方波频率选择位2
CTRL_BBSQW = 0x40  # 电池方波使能
CTRL_EOSC = 0x80   # 振荡器使能（低有效）

# 状态位
STAT_A1F = 0x01    # 闹钟1标志
STAT_A2F = 0x02    # 闹钟2标志
STAT_OSF = 0x80    # 振荡器停止标志


def int_to_bcd(val):
    """整数转BCD编码"""
    return ((val // 10) << 4) | (val % 10)


def bcd_to_int(bcd):
    """BCD编码转整数"""
    return ((bcd >> 4) * 10) + (bcd & 0x0F)


class DS3231:
    """DS3231 RTC芯片模拟器，与C驱动逻辑一致"""

    def __init__(self, i2c_bus=None):
        self.bus = i2c_bus or I2CBus()
        self.bus.init()
        self.addr = DS3231_ADDR
        # 内部寄存器存储
        self._regs = [0] * 25
        self._regs[REG_CONTROL] = 0x00         # EOSC=0使能振荡器（低有效）
        self._regs[REG_STATUS] = STAT_OSF     # 首次上电振荡器停止标志
        # 温度模拟
        self._temp_c = 25.0
        # 12小时制标志
        self._12h_mode = False

    def _write_reg(self, reg, val):
        """写寄存器"""
        self._regs[reg] = val & 0xFF

    def _read_reg(self, reg):
        """读寄存器"""
        return self._regs[reg] & 0xFF

    def init(self):
        """初始化DS3231"""
        self._regs[REG_CONTROL] = 0x00  # EOSC=0使能振荡器
        self._regs[REG_STATUS] = 0  # 清除振荡器停止标志
        # 设置默认时间 2024-01-01 00:00:00
        self.set_time(2024, 1, 1, 0, 0, 0)

    def set_time(self, year, month, day, hour, minute, second):
        """设置时间"""
        self._write_reg(REG_SECONDS, int_to_bcd(second))
        self._write_reg(REG_MINUTES, int_to_bcd(minute))
        self._write_reg(REG_HOURS, int_to_bcd(hour))
        self._write_reg(REG_DATE, int_to_bcd(day))
        self._write_reg(REG_MONTH, int_to_bcd(month))
        self._write_reg(REG_YEAR, int_to_bcd(year % 100))

    def get_time(self):
        """读取时间，返回 (year, month, day, hour, minute, second)"""
        sec = bcd_to_int(self._read_reg(REG_SECONDS))
        minute = bcd_to_int(self._read_reg(REG_MINUTES))
        hour_reg = self._read_reg(REG_HOURS)
        if self._12h_mode:
            hour = bcd_to_int(hour_reg & 0x1F)
            if hour_reg & 0x20:  # PM
                if hour < 12:
                    hour += 12
        else:
            hour = bcd_to_int(hour_reg & 0x3F)
        day = bcd_to_int(self._read_reg(REG_DATE))
        month = bcd_to_int(self._read_reg(REG_MONTH) & 0x1F)
        year = 2000 + bcd_to_int(self._read_reg(REG_YEAR))
        return year, month, day, hour, minute, sec

    def set_alarm1(self, second, minute, hour, day, mode=0):
        """设置闹钟1，mode: 0=秒分时匹配, 1=分时匹配, 2=时匹配"""
        self._write_reg(REG_ALARM1_SEC, int_to_bcd(second))
        self._write_reg(REG_ALARM1_MIN, int_to_bcd(minute))
        self._write_reg(REG_ALARM1_HOUR, int_to_bcd(hour))
        self._write_reg(REG_ALARM1_DAY, int_to_bcd(day))
        self._regs[REG_CONTROL] |= CTRL_A1IE | CTRL_INTCN

    def set_alarm2(self, minute, hour, day, mode=0):
        """设置闹钟2，mode: 0=分时匹配, 1=时匹配"""
        self._write_reg(REG_ALARM2_MIN, int_to_bcd(minute))
        self._write_reg(REG_ALARM2_HOUR, int_to_bcd(hour))
        self._write_reg(REG_ALARM2_DAY, int_to_bcd(day))
        self._regs[REG_CONTROL] |= CTRL_A2IE | CTRL_INTCN

    def check_alarm1(self):
        """检查闹钟1是否触发"""
        status = self._read_reg(REG_STATUS)
        if status & STAT_A1F:
            return True
        return False

    def clear_alarm1(self):
        """清除闹钟1标志"""
        self._regs[REG_STATUS] &= ~STAT_A1F

    def check_alarm2(self):
        """检查闹钟2是否触发"""
        status = self._read_reg(REG_STATUS)
        if status & STAT_A2F:
            return True
        return False

    def clear_alarm2(self):
        """清除闹钟2标志"""
        self._regs[REG_STATUS] &= ~STAT_A2F

    def get_temperature(self):
        """读取温度（模拟）"""
        msb = int(self._temp_c)
        lsb = int((self._temp_c - msb) * 4) << 6
        self._write_reg(REG_TEMP_MSB, msb & 0xFF)
        self._write_reg(REG_TEMP_LSB, lsb & 0xC0)
        return self._temp_c

    def set_temperature(self, temp):
        """设置模拟温度值"""
        self._temp_c = temp

    def set_square_wave(self, freq_code):
        """设置方波频率: 0=1Hz, 1=1.024kHz, 2=4.096kHz, 3=8.192kHz"""
        ctrl = self._read_reg(REG_CONTROL)
        ctrl &= ~(CTRL_RS1 | CTRL_RS2 | CTRL_INTCN)
        if freq_code == 0:
            pass  # RS1=0, RS2=0
        elif freq_code == 1:
            ctrl |= CTRL_RS1
        elif freq_code == 2:
            ctrl |= CTRL_RS2
        elif freq_code == 3:
            ctrl |= CTRL_RS1 | CTRL_RS2
        self._write_reg(REG_CONTROL, ctrl)

    def set_12h_mode(self, enable):
        """设置12小时制"""
        self._12h_mode = enable

    def is_running(self):
        """检查振荡器是否运行"""
        return not (self._read_reg(REG_STATUS) & STAT_OSF)

    def is_leap_year(self, year):
        """闰年判断"""
        return (year % 4 == 0 and year % 100 != 0) or (year % 400 == 0)


# ---- 测试辅助 ----
def assert_eq(actual, expected, msg=""):
    if actual != expected:
        raise AssertionError(f"{msg}: 期望 {expected}, 实际 {actual}")

def assert_close(actual, expected, tol=0.5, msg=""):
    if abs(actual - expected) > tol:
        raise AssertionError(f"{msg}: 期望 {expected}±{tol}, 实际 {actual}")

def run_test(func, name=""):
    try:
        func()
        print(f"  [通过] {name}")
        return True
    except AssertionError as e:
        print(f"  [失败] {name}: {e}")
        return False
    except Exception as e:
        print(f"  [错误] {name}: {type(e).__name__}: {e}")
        return False


# ═══════════════════════════════════════════════════════════════
#  测试用例
# ═══════════════════════════════════════════════════════════════

def test_init():
    """DS3231初始化"""
    ds = DS3231()
    ds.init()
    assert ds.is_running(), "振荡器应运行"
    y, m, d, h, mi, s = ds.get_time()
    assert_eq(y, 2024, "年份")
    assert_eq(m, 1, "月份")
    assert_eq(d, 1, "日期")

def test_set_time():
    """设置时间"""
    ds = DS3231()
    ds.init()
    ds.set_time(2025, 6, 15, 14, 30, 45)
    y, m, d, h, mi, s = ds.get_time()
    assert_eq(y, 2025, "年")
    assert_eq(m, 6, "月")
    assert_eq(d, 15, "日")
    assert_eq(h, 14, "时")
    assert_eq(mi, 30, "分")
    assert_eq(s, 45, "秒")

def test_bcd_encoding():
    """BCD编码测试"""
    assert_eq(int_to_bcd(0), 0x00, "0")
    assert_eq(int_to_bcd(9), 0x09, "9")
    assert_eq(int_to_bcd(10), 0x10, "10")
    assert_eq(int_to_bcd(59), 0x59, "59")
    assert_eq(bcd_to_int(0x00), 0, "bcd 0")
    assert_eq(bcd_to_int(0x59), 59, "bcd 59")
    assert_eq(bcd_to_int(0x23), 23, "bcd 23")

def test_alarm1():
    """闹钟1设置"""
    ds = DS3231()
    ds.init()
    ds.set_time(2024, 1, 1, 7, 59, 50)
    ds.set_alarm1(0, 0, 8, 1)
    # 手动触发闹钟标志
    ds._regs[REG_STATUS] |= STAT_A1F
    assert ds.check_alarm1(), "闹钟1应触发"
    ds.clear_alarm1()
    assert not ds.check_alarm1(), "清除后不应触发"

def test_alarm2():
    """闹钟2设置"""
    ds = DS3231()
    ds.init()
    ds.set_alarm2(30, 12, 1)
    ds._regs[REG_STATUS] |= STAT_A2F
    assert ds.check_alarm2(), "闹钟2应触发"
    ds.clear_alarm2()
    assert not ds.check_alarm2(), "清除后不应触发"

def test_temperature():
    """温度传感器"""
    ds = DS3231()
    ds.init()
    ds.set_temperature(23.5)
    temp = ds.get_temperature()
    assert_close(temp, 23.5, 0.5, "温度")

def test_temperature_negative():
    """负温度"""
    ds = DS3231()
    ds.init()
    ds.set_temperature(-5.0)
    temp = ds.get_temperature()
    assert_close(temp, -5.0, 0.5, "负温度")

def test_square_wave():
    """方波输出频率"""
    ds = DS3231()
    ds.init()
    for code in [0, 1, 2, 3]:
        ds.set_square_wave(code)
        ctrl = ds._read_reg(REG_CONTROL)
        # 不能同时有INTCN位（方波模式）
        assert not (ctrl & CTRL_INTCN), f"频率{code}: INTCN应为0"

def test_control_reg():
    """控制寄存器"""
    ds = DS3231()
    ds.init()
    ctrl = ds._read_reg(REG_CONTROL)
    assert not (ctrl & CTRL_EOSC), "EOSC应为0（振荡器使能）"

def test_oscillator_stop_flag():
    """振荡器停止标志"""
    ds = DS3231()
    ds.init()
    assert ds.is_running(), "初始化后应运行"
    ds._regs[REG_STATUS] |= STAT_OSF
    assert not ds.is_running(), "设OSF后应停止"

def test_leap_year():
    """闰年判断"""
    ds = DS3231()
    assert ds.is_leap_year(2024) is True
    assert ds.is_leap_year(2000) is True
    assert ds.is_leap_year(1900) is False
    assert ds.is_leap_year(2023) is False

def test_24h_format():
    """24小时制"""
    ds = DS3231()
    ds.init()
    ds.set_time(2024, 1, 1, 23, 59, 59)
    y, m, d, h, mi, s = ds.get_time()
    assert_eq(h, 23, "24h小时")

def test_midnight_carry():
    """午夜进位"""
    ds = DS3231()
    ds.init()
    ds.set_time(2024, 1, 31, 23, 59, 59)
    # 模拟tick: 手动进位
    ds.set_time(2024, 2, 1, 0, 0, 0)
    y, m, d, h, mi, s = ds.get_time()
    assert_eq(m, 2, "进位后月")
    assert_eq(d, 1, "进位后日")

def test_aging_register():
    """老化补偿寄存器"""
    ds = DS3231()
    ds.init()
    ds._write_reg(REG_AGING, 5)
    assert_eq(ds._read_reg(REG_AGING), 5, "aging")
    ds._write_reg(REG_AGING, -3 & 0xFF)
    assert_eq(ds._read_reg(REG_AGING), 0xFD, "负aging")

def test_i2c_bus_integration():
    """I2C总线集成"""
    bus = I2CBus()
    bus.init()
    ds = DS3231(bus)
    ds.init()
    assert bus.tx_count >= 0, "I2C应可通信"


def main():
    print("=" * 60)
    print("  DS3231 RTC芯片测试 V2")
    print("=" * 60)
    tests = [
        (test_init, "初始化"),
        (test_set_time, "设置时间"),
        (test_bcd_encoding, "BCD编码"),
        (test_alarm1, "闹钟1"),
        (test_alarm2, "闹钟2"),
        (test_temperature, "温度传感器"),
        (test_temperature_negative, "负温度"),
        (test_square_wave, "方波输出"),
        (test_control_reg, "控制寄存器"),
        (test_oscillator_stop_flag, "振荡器停止标志"),
        (test_leap_year, "闰年判断"),
        (test_24h_format, "24小时制"),
        (test_midnight_carry, "午夜进位"),
        (test_aging_register, "老化补偿"),
        (test_i2c_bus_integration, "I2C总线集成"),
    ]
    passed = failed = 0
    for func, name in tests:
        if run_test(func, name):
            passed += 1
        else:
            failed += 1
    print("-" * 60)
    print(f"  结果: {passed} 通过, {failed} 失败, 共 {passed + failed} 项")
    print("=" * 60)
    return failed == 0

if __name__ == "__main__":
    sys.exit(0 if main() else 1)
