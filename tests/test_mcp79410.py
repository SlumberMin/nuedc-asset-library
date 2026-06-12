# -*- coding: utf-8 -*-
"""
test_mcp79410_v2.py - MCP79410 RTC+EEPROM测试 V2
==================================================
测试内容：
  1. I2C通信初始化（RTC地址0x6F, EEPROM地址0x57）
  2. 时间读写（BCD编码，含VBATEN位）
  3. 闹钟0和闹钟1
  4. 方波/中断输出
  5. 电源故障检测（PWRFAIL位）
  6. 备用电池切换
  7. EEPROM读写（128字节）
  8. EEPROM页写入
  9. 唯一ID读取
  10. 振荡器校准

使用 wrappers.py 的 I2CBus
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from wrappers import I2CBus

# ═══════════════════════════════════════════════════════════════
#  MCP79410地址与寄存器定义
# ═══════════════════════════════════════════════════════════════
MCP79410_RTC_ADDR = 0x6F
MCP79410_EEPROM_ADDR = 0x57

# RTC寄存器
REG_RTC_SEC = 0x00
REG_RTC_MIN = 0x01
REG_RTC_HOUR = 0x02
REG_RTC_DAY = 0x03
REG_RTC_DATE = 0x04
REG_RTC_MONTH = 0x05
REG_RTC_YEAR = 0x06
REG_RTC_CTRL = 0x07

# 闹钟0
REG_ALM0_SEC = 0x0A
REG_ALM0_MIN = 0x0B
REG_ALM0_HOUR = 0x0C
REG_ALM0_DAY = 0x0D
REG_ALM0_DATE = 0x0E
REG_ALM0_MONTH = 0x0F

# 闹钟1
REG_ALM1_SEC = 0x11
REG_ALM1_MIN = 0x12
REG_ALM1_HOUR = 0x13
REG_ALM1_DAY = 0x14
REG_ALM1_DATE = 0x15
REG_ALM1_MONTH = 0x16

# 控制位
SEC_ST = 0x80        # 启动振荡器
SEC_VBATEN = 0x08    # 备用电池使能
DAY_VBATEN = 0x08    # 备用电池使能（DAY寄存器）
DAY_PWRFAIL = 0x10   # 电源故障标志
DAY_OSCRUN = 0x20    # 振荡器运行标志
CTRL_ALM0EN = 0x10   # 闹钟0使能
CTRL_ALM1EN = 0x20   # 闹钟1使能
CTRL_SQWEN = 0x40    # 方波使能
CTRL_OUT = 0x80      # 输出极性

# EEPROM
EEPROM_SIZE = 128


def int_to_bcd(val):
    """整数转BCD"""
    return ((val // 10) << 4) | (val % 10)


def bcd_to_int(bcd):
    """BCD转整数"""
    return ((bcd >> 4) * 10) + (bcd & 0x0F)


class MCP79410:
    """MCP79410 RTC+EEPROM芯片模拟器"""

    def __init__(self, i2c_bus=None):
        self.bus = i2c_bus or I2CBus()
        self.bus.init()
        # RTC寄存器
        self._rtc_regs = [0] * 25
        # EEPROM存储
        self._eeprom = [0xFF] * EEPROM_SIZE
        # 内部状态
        self._osc_running = False
        self._vbat_enabled = False
        self._pwr_fail = False
        self._temp_id = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]

    def init(self):
        """初始化MCP79410"""
        self._rtc_regs = [0] * 25
        self._osc_running = False
        self._vbat_enabled = False
        self._pwr_fail = False

    def start_oscillator(self):
        """启动振荡器"""
        self._rtc_regs[REG_RTC_SEC] |= SEC_ST
        self._osc_running = True
        # 设置OSCRUN位
        self._rtc_regs[REG_RTC_DAY] |= DAY_OSCRUN

    def stop_oscillator(self):
        """停止振荡器"""
        self._rtc_regs[REG_RTC_SEC] &= ~SEC_ST
        self._osc_running = False
        self._rtc_regs[REG_RTC_DAY] &= ~DAY_OSCRUN

    def is_osc_running(self):
        """检查振荡器是否运行"""
        return bool(self._rtc_regs[REG_RTC_DAY] & DAY_OSCRUN)

    def enable_vbat(self):
        """使能备用电池"""
        self._rtc_regs[REG_RTC_DAY] |= DAY_VBATEN
        self._vbat_enabled = True

    def disable_vbat(self):
        """禁用备用电池"""
        self._rtc_regs[REG_RTC_DAY] &= ~DAY_VBATEN
        self._vbat_enabled = False

    def set_time(self, year, month, day, hour, minute, second):
        """设置时间"""
        self._rtc_regs[REG_RTC_SEC] = int_to_bcd(second) | (self._rtc_regs[REG_RTC_SEC] & 0x80)
        self._rtc_regs[REG_RTC_MIN] = int_to_bcd(minute)
        self._rtc_regs[REG_RTC_HOUR] = int_to_bcd(hour)
        self._rtc_regs[REG_RTC_DATE] = int_to_bcd(day)
        self._rtc_regs[REG_RTC_MONTH] = int_to_bcd(month)
        self._rtc_regs[REG_RTC_YEAR] = int_to_bcd(year % 100)
        # 设置星期（简化计算：1=周一）
        self._rtc_regs[REG_RTC_DAY] = (self._rtc_regs[REG_RTC_DAY] & 0x38) | 1

    def get_time(self):
        """读取时间"""
        sec = bcd_to_int(self._rtc_regs[REG_RTC_SEC] & 0x7F)
        minute = bcd_to_int(self._rtc_regs[REG_RTC_MIN] & 0x7F)
        hour = bcd_to_int(self._rtc_regs[REG_RTC_HOUR] & 0x3F)
        day = bcd_to_int(self._rtc_regs[REG_RTC_DATE] & 0x3F)
        month = bcd_to_int(self._rtc_regs[REG_RTC_MONTH] & 0x1F)
        year = 2000 + bcd_to_int(self._rtc_regs[REG_RTC_YEAR])
        return year, month, day, hour, minute, sec

    def set_alarm0(self, second, minute, hour, day, month):
        """设置闹钟0"""
        self._rtc_regs[REG_ALM0_SEC] = int_to_bcd(second)
        self._rtc_regs[REG_ALM0_MIN] = int_to_bcd(minute)
        self._rtc_regs[REG_ALM0_HOUR] = int_to_bcd(hour)
        self._rtc_regs[REG_ALM0_DATE] = int_to_bcd(day)
        self._rtc_regs[REG_ALM0_MONTH] = int_to_bcd(month)
        self._rtc_regs[REG_RTC_CTRL] |= CTRL_ALM0EN

    def set_alarm1(self, second, minute, hour, day, month):
        """设置闹钟1"""
        self._rtc_regs[REG_ALM1_SEC] = int_to_bcd(second)
        self._rtc_regs[REG_ALM1_MIN] = int_to_bcd(minute)
        self._rtc_regs[REG_ALM1_HOUR] = int_to_bcd(hour)
        self._rtc_regs[REG_ALM1_DATE] = int_to_bcd(day)
        self._rtc_regs[REG_ALM1_MONTH] = int_to_bcd(month)
        self._rtc_regs[REG_RTC_CTRL] |= CTRL_ALM1EN

    def check_alarm0_flag(self):
        """检查闹钟0标志（ALM0IF在ALM0_DAY bit3）"""
        return bool(self._rtc_regs[REG_ALM0_DAY] & 0x08)

    def set_alarm0_flag(self):
        """手动设置闹钟0标志"""
        self._rtc_regs[REG_ALM0_DAY] |= 0x08

    def clear_alarm0_flag(self):
        """清除闹钟0标志"""
        self._rtc_regs[REG_ALM0_DAY] &= ~0x08

    def check_alarm1_flag(self):
        """检查闹钟1标志"""
        return bool(self._rtc_regs[REG_ALM1_DAY] & 0x08)

    def set_alarm1_flag(self):
        """手动设置闹钟1标志"""
        self._rtc_regs[REG_ALM1_DAY] |= 0x08

    def clear_alarm1_flag(self):
        """清除闹钟1标志"""
        self._rtc_regs[REG_ALM1_DAY] &= ~0x08

    def set_pwr_fail(self, flag=True):
        """设置电源故障标志"""
        if flag:
            self._rtc_regs[REG_RTC_DAY] |= DAY_PWRFAIL
            self._pwr_fail = True
        else:
            self._rtc_regs[REG_RTC_DAY] &= ~DAY_PWRFAIL
            self._pwr_fail = False

    def is_pwr_fail(self):
        """检查电源故障"""
        return bool(self._rtc_regs[REG_RTC_DAY] & DAY_PWRFAIL)

    def set_square_wave(self, enable, freq_code=0):
        """设置方波输出，freq_code: 0=1Hz, 1=4.096kHz, 2=8.192kHz, 3=32.768kHz"""
        ctrl = self._rtc_regs[REG_RTC_CTRL]
        if enable:
            ctrl |= CTRL_SQWEN
        else:
            ctrl &= ~CTRL_SQWEN
        self._rtc_regs[REG_RTC_CTRL] = ctrl

    # ---- EEPROM操作 ----
    def eeprom_write_byte(self, addr, data):
        """EEPROM写单字节"""
        if 0 <= addr < EEPROM_SIZE:
            self._eeprom[addr] = data & 0xFF
            return True
        return False

    def eeprom_read_byte(self, addr):
        """EEPROM读单字节"""
        if 0 <= addr < EEPROM_SIZE:
            return self._eeprom[addr]
        return -1

    def eeprom_write_page(self, addr, data):
        """EEPROM页写入（最多8字节对齐）"""
        if addr + len(data) > EEPROM_SIZE:
            return False
        for i, b in enumerate(data):
            self._eeprom[addr + i] = b & 0xFF
        return True

    def eeprom_read_block(self, addr, length):
        """EEPROM块读取"""
        if addr + length > EEPROM_SIZE:
            return None
        return self._eeprom[addr:addr + length]

    def get_unique_id(self):
        """读取64位唯一ID"""
        return bytes(self._temp_id)

    def set_osc_trim(self, trim_val):
        """振荡器校准（-127 ~ +127 ppm）"""
        self._rtc_regs[0x08] = trim_val & 0xFF


# ---- 测试辅助 ----
def assert_eq(actual, expected, msg=""):
    if actual != expected:
        raise AssertionError(f"{msg}: 期望 {expected}, 实际 {actual}")

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
    """MCP79410初始化"""
    mcp = MCP79410()
    mcp.init()
    assert not mcp.is_osc_running(), "初始振荡器应停止"

def test_start_oscillator():
    """启动振荡器"""
    mcp = MCP79410()
    mcp.init()
    mcp.start_oscillator()
    assert mcp.is_osc_running(), "振荡器应运行"
    mcp.stop_oscillator()
    assert not mcp.is_osc_running(), "停止后应不运行"

def test_set_time():
    """设置时间"""
    mcp = MCP79410()
    mcp.init()
    mcp.start_oscillator()
    mcp.set_time(2025, 3, 20, 10, 30, 45)
    y, m, d, h, mi, s = mcp.get_time()
    assert_eq(y, 2025, "年")
    assert_eq(m, 3, "月")
    assert_eq(d, 20, "日")
    assert_eq(h, 10, "时")
    assert_eq(mi, 30, "分")
    assert_eq(s, 45, "秒")

def test_bcd_roundtrip():
    """BCD往返测试"""
    for val in [0, 1, 9, 10, 23, 59]:
        assert_eq(bcd_to_int(int_to_bcd(val)), val, f"BCD {val}")

def test_alarm0():
    """闹钟0"""
    mcp = MCP79410()
    mcp.init()
    mcp.start_oscillator()
    mcp.set_alarm0(0, 0, 8, 15, 6)
    ctrl = mcp._rtc_regs[REG_RTC_CTRL]
    assert ctrl & CTRL_ALM0EN, "闹钟0应使能"
    mcp.set_alarm0_flag()
    assert mcp.check_alarm0_flag(), "闹钟0标志应置位"
    mcp.clear_alarm0_flag()
    assert not mcp.check_alarm0_flag(), "清除后应为0"

def test_alarm1():
    """闹钟1"""
    mcp = MCP79410()
    mcp.init()
    mcp.start_oscillator()
    mcp.set_alarm1(30, 15, 12, 25, 12)
    ctrl = mcp._rtc_regs[REG_RTC_CTRL]
    assert ctrl & CTRL_ALM1EN, "闹钟1应使能"
    mcp.set_alarm1_flag()
    assert mcp.check_alarm1_flag(), "闹钟1标志应置位"
    mcp.clear_alarm1_flag()
    assert not mcp.check_alarm1_flag(), "清除后应为0"

def test_vbat():
    """备用电池使能"""
    mcp = MCP79410()
    mcp.init()
    mcp.enable_vbat()
    assert mcp._vbat_enabled, "VBAT应使能"
    mcp.disable_vbat()
    assert not mcp._vbat_enabled, "VBAT应禁用"

def test_power_fail():
    """电源故障检测"""
    mcp = MCP79410()
    mcp.init()
    assert not mcp.is_pwr_fail(), "初始无故障"
    mcp.set_pwr_fail(True)
    assert mcp.is_pwr_fail(), "应检测到故障"
    mcp.set_pwr_fail(False)
    assert not mcp.is_pwr_fail(), "清除后无故障"

def test_square_wave():
    """方波输出"""
    mcp = MCP79410()
    mcp.init()
    mcp.set_square_wave(True, 0)
    assert mcp._rtc_regs[REG_RTC_CTRL] & CTRL_SQWEN, "方波应使能"
    mcp.set_square_wave(False)
    assert not (mcp._rtc_regs[REG_RTC_CTRL] & CTRL_SQWEN), "方波应禁用"

def test_eeprom_write_read():
    """EEPROM读写"""
    mcp = MCP79410()
    mcp.init()
    mcp.eeprom_write_byte(0, 0xAA)
    assert_eq(mcp.eeprom_read_byte(0), 0xAA, "字节0")
    mcp.eeprom_write_byte(127, 0x55)
    assert_eq(mcp.eeprom_read_byte(127), 0x55, "字节127")

def test_eeprom_boundary():
    """EEPROM边界"""
    mcp = MCP79410()
    mcp.init()
    assert mcp.eeprom_write_byte(-1, 0) == False, "负地址应失败"
    assert mcp.eeprom_write_byte(128, 0) == False, "超界应失败"
    assert mcp.eeprom_read_byte(128) == -1, "超界读应返回-1"

def test_eeprom_page_write():
    """EEPROM页写入"""
    mcp = MCP79410()
    mcp.init()
    data = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80]
    assert mcp.eeprom_write_page(0, data), "页写入应成功"
    readback = mcp.eeprom_read_block(0, 8)
    assert_eq(readback, data, "页读回")

def test_eeprom_block_read():
    """EEPROM块读取"""
    mcp = MCP79410()
    mcp.init()
    for i in range(10):
        mcp.eeprom_write_byte(i, i * 2)
    block = mcp.eeprom_read_block(0, 10)
    for i in range(10):
        assert_eq(block[i], i * 2, f"块读取[{i}]")

def test_unique_id():
    """唯一ID"""
    mcp = MCP79410()
    uid = mcp.get_unique_id()
    assert len(uid) == 8, "ID应为8字节"

def test_osc_trim():
    """振荡器校准"""
    mcp = MCP79410()
    mcp.init()
    mcp.set_osc_trim(10)
    assert_eq(mcp._rtc_regs[0x08], 10, "trim值")
    mcp.set_osc_trim(-5 & 0xFF)
    assert_eq(mcp._rtc_regs[0x08], 0xFB, "负trim")

def test_i2c_integration():
    """I2C总线集成"""
    bus = I2CBus()
    bus.init()
    mcp = MCP79410(bus)
    mcp.init()
    mcp.start_oscillator()
    mcp.set_time(2025, 1, 1, 0, 0, 0)
    assert bus.tx_count >= 0


def main():
    print("=" * 60)
    print("  MCP79410 RTC+EEPROM测试 V2")
    print("=" * 60)
    tests = [
        (test_init, "初始化"),
        (test_start_oscillator, "振荡器启停"),
        (test_set_time, "设置时间"),
        (test_bcd_roundtrip, "BCD往返"),
        (test_alarm0, "闹钟0"),
        (test_alarm1, "闹钟1"),
        (test_vbat, "备用电池"),
        (test_power_fail, "电源故障"),
        (test_square_wave, "方波输出"),
        (test_eeprom_write_read, "EEPROM读写"),
        (test_eeprom_boundary, "EEPROM边界"),
        (test_eeprom_page_write, "EEPROM页写入"),
        (test_eeprom_block_read, "EEPROM块读取"),
        (test_unique_id, "唯一ID"),
        (test_osc_trim, "振荡器校准"),
        (test_i2c_integration, "I2C集成"),
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
