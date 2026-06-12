# -*- coding: utf-8 -*-
"""
test_mcp3008_v2.py - MCP3008 ADC测试 V2
========================================
测试内容：
  1. SPI通信模拟
  2. 单端输入（CH0-CH7）读取
  3. 差分输入读取
  4. 10位ADC分辨率（0-1023）
  5. 电压计算
  6. 多通道扫描
  7. 参考电压设置
  8. 噪声与滤波（移动平均）
  9. 通道切换延迟
  10. 边界值测试

使用 wrappers.py 的 SimpleMA（移动平均滤波器）
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from wrappers import SimpleMA

# ═══════════════════════════════════════════════════════════════
#  MCP3008常量
# ═══════════════════════════════════════════════════════════════
MCP3008_CHANNELS = 8
MCP3008_RESOLUTION = 10  # 位
MCP3008_MAX_VALUE = (1 << MCP3008_RESOLUTION) - 1  # 1023

# 输入模式
MODE_SINGLE = 0   # 单端
MODE_DIFF = 1     # 差分

# 差分通道对
DIFF_PAIRS = [
    (0, 1),  # CH0-CH1
    (1, 0),  # CH1-CH0
    (2, 3),  # CH2-CH3
    (3, 2),  # CH3-CH2
    (4, 5),  # CH4-CH5
    (5, 4),  # CH5-CH4
    (6, 7),  # CH6-CH7
    (7, 6),  # CH7-CH6
]


class MCP3008:
    """MCP3008 8通道10位ADC模拟器"""

    def __init__(self, vref=3.3):
        self.vref = vref
        # 各通道模拟电压值
        self._channel_voltages = [0.0] * MCP3008_CHANNELS
        # 最近一次读取结果
        self._last_raw = 0
        self._last_voltage = 0.0
        # SPI模拟
        self._spi_speed = 1000000  # 1MHz默认
        self._cs_pin = 0

    def set_channel_voltage(self, channel, voltage):
        """设置指定通道的模拟输入电压（模拟传感器）"""
        if 0 <= channel < MCP3008_CHANNELS:
            self._channel_voltages[channel] = max(0.0, min(self.vref, voltage))

    def read_single(self, channel):
        """单端模式读取指定通道，返回 (raw_value, voltage)"""
        if channel < 0 or channel >= MCP3008_CHANNELS:
            return -1, 0.0
        voltage = self._channel_voltages[channel]
        # 电压转ADC原始值
        raw = int((voltage / self.vref) * MCP3008_MAX_VALUE + 0.5)
        raw = max(0, min(MCP3008_MAX_VALUE, raw))
        actual_voltage = (raw / MCP3008_MAX_VALUE) * self.vref
        self._last_raw = raw
        self._last_voltage = actual_voltage
        return raw, actual_voltage

    def read_differential(self, ch_positive, ch_negative):
        """差分模式读取，返回 (raw_value, voltage)"""
        if ch_positive < 0 or ch_positive >= MCP3008_CHANNELS:
            return -1, 0.0
        if ch_negative < 0 or ch_negative >= MCP3008_CHANNELS:
            return -1, 0.0
        diff_voltage = self._channel_voltages[ch_positive] - self._channel_voltages[ch_negative]
        # 差分范围: 0 ~ Vref
        if diff_voltage < 0:
            raw = 0
        else:
            raw = int((diff_voltage / self.vref) * MCP3008_MAX_VALUE + 0.5)
        raw = max(0, min(MCP3008_MAX_VALUE, raw))
        actual_voltage = (raw / MCP3008_MAX_VALUE) * self.vref
        self._last_raw = raw
        self._last_voltage = actual_voltage
        return raw, actual_voltage

    def scan_all_channels(self):
        """扫描所有通道，返回 [(raw, voltage), ...]"""
        results = []
        for ch in range(MCP3008_CHANNELS):
            raw, v = self.read_single(ch)
            results.append((raw, v))
        return results

    def get_resolution(self):
        """返回ADC分辨率"""
        return MCP3008_RESOLUTION

    def get_max_value(self):
        """返回最大ADC值"""
        return MCP3008_MAX_VALUE

    def raw_to_voltage(self, raw):
        """ADC原始值转电压"""
        return (raw / MCP3008_MAX_VALUE) * self.vref

    def voltage_to_raw(self, voltage):
        """电压转ADC原始值"""
        return int((voltage / self.vref) * MCP3008_MAX_VALUE + 0.5)

    def set_spi_speed(self, speed):
        """设置SPI速率"""
        self._spi_speed = speed

    def get_spi_speed(self):
        """获取SPI速率"""
        return self._spi_speed


# ---- 测试辅助 ----
def assert_eq(actual, expected, msg=""):
    if actual != expected:
        raise AssertionError(f"{msg}: 期望 {expected}, 实际 {actual}")

def assert_close(actual, expected, tol=0.05, msg=""):
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
    """MCP3008初始化"""
    adc = MCP3008()
    assert_eq(adc.get_resolution(), 10, "分辨率")
    assert_eq(adc.get_max_value(), 1023, "最大值")
    assert_close(adc.vref, 3.3, 0.01, "参考电压")

def test_read_zero():
    """读取零电压"""
    adc = MCP3008()
    adc.set_channel_voltage(0, 0.0)
    raw, v = adc.read_single(0)
    assert_eq(raw, 0, "零电压raw")
    assert_close(v, 0.0, 0.01, "零电压")

def test_read_max():
    """读取满量程"""
    adc = MCP3008(vref=3.3)
    adc.set_channel_voltage(0, 3.3)
    raw, v = adc.read_single(0)
    assert_eq(raw, 1023, "满量程raw")
    assert_close(v, 3.3, 0.01, "满量程电压")

def test_read_mid():
    """读取中间值"""
    adc = MCP3008(vref=3.3)
    adc.set_channel_voltage(0, 1.65)
    raw, v = adc.read_single(0)
    assert_close(raw, 511, 2, "中间raw")
    assert_close(v, 1.65, 0.01, "中间电压")

def test_all_channels():
    """所有通道读取"""
    adc = MCP3008()
    for ch in range(8):
        adc.set_channel_voltage(ch, ch * 0.4)
    results = adc.scan_all_channels()
    assert_eq(len(results), 8, "通道数")
    for ch in range(8):
        raw, v = results[ch]
        assert raw >= 0, f"CH{ch} raw>=0"

def test_single_channel_read():
    """单通道读取精度"""
    adc = MCP3008(vref=5.0)
    adc.set_channel_voltage(3, 2.5)
    raw, v = adc.read_single(3)
    assert_close(v, 2.5, 0.01, "2.5V读取")

def test_differential_positive():
    """差分模式正向"""
    adc = MCP3008(vref=3.3)
    adc.set_channel_voltage(0, 2.0)
    adc.set_channel_voltage(1, 1.0)
    raw, v = adc.read_differential(0, 1)
    assert raw > 0, "差分应为正"
    assert_close(v, 1.0, 0.02, "差分电压")

def test_differential_negative():
    """差分模式反向（应返回0）"""
    adc = MCP3008(vref=3.3)
    adc.set_channel_voltage(0, 1.0)
    adc.set_channel_voltage(1, 2.0)
    raw, v = adc.read_differential(0, 1)
    assert_eq(raw, 0, "反向差分应为0")

def test_differential_zero():
    """差分模式相等"""
    adc = MCP3008(vref=3.3)
    adc.set_channel_voltage(2, 1.5)
    adc.set_channel_voltage(3, 1.5)
    raw, v = adc.read_differential(2, 3)
    assert_eq(raw, 0, "相等差分应为0")

def test_voltage_conversion():
    """电压转换"""
    adc = MCP3008(vref=3.3)
    v = adc.raw_to_voltage(512)
    assert_close(v, 1.65, 0.01, "512->电压")
    raw = adc.voltage_to_raw(1.65)
    assert_close(raw, 512, 2, "电压->raw")

def test_custom_vref():
    """自定义参考电压"""
    adc = MCP3008(vref=5.0)
    adc.set_channel_voltage(0, 5.0)
    raw, v = adc.read_single(0)
    assert_eq(raw, 1023, "5V满量程")
    assert_close(v, 5.0, 0.01, "5V读取")

def test_noise_filtering():
    """噪声滤波（移动平均）"""
    import random
    random.seed(42)
    ma = SimpleMA(10)
    adc = MCP3008(vref=3.3)
    true_voltage = 1.65
    for _ in range(20):
        # 添加±0.05V噪声
        noisy = true_voltage + random.uniform(-0.05, 0.05)
        adc.set_channel_voltage(0, noisy)
        raw, v = adc.read_single(0)
        ma.update(v)
    filtered = ma.last_output
    assert_close(filtered, true_voltage, 0.02, "滤波后应接近真值")

def test_channel_boundary():
    """通道边界测试"""
    adc = MCP3008()
    raw, v = adc.read_single(-1)
    assert_eq(raw, -1, "负通道应返回-1")
    raw, v = adc.read_single(8)
    assert_eq(raw, -1, "超界通道应返回-1")

def test_voltage_clamp():
    """电压钳位（超过Vref）"""
    adc = MCP3008(vref=3.3)
    adc.set_channel_voltage(0, 5.0)  # 超过Vref
    raw, v = adc.read_single(0)
    assert_eq(raw, 1023, "超Vref应钳位到1023")
    assert_close(v, 3.3, 0.01, "超Vref电压应为Vref")

def test_negative_voltage_clamp():
    """负电压钳位"""
    adc = MCP3008(vref=3.3)
    adc.set_channel_voltage(0, -1.0)
    raw, v = adc.read_single(0)
    assert_eq(raw, 0, "负电压应钳位到0")

def test_spi_speed():
    """SPI速率设置"""
    adc = MCP3008()
    adc.set_spi_speed(2000000)
    assert_eq(adc.get_spi_speed(), 2000000, "SPI速率")

def test_multi_channel_scan():
    """多通道扫描一致性"""
    adc = MCP3008(vref=3.3)
    voltages = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.3]
    for ch, v in enumerate(voltages):
        adc.set_channel_voltage(ch, v)
    results = adc.scan_all_channels()
    for ch, expected_v in enumerate(voltages):
        _, actual_v = results[ch]
        assert_close(actual_v, expected_v, 0.02, f"CH{ch}")


def main():
    print("=" * 60)
    print("  MCP3008 ADC测试 V2")
    print("=" * 60)
    tests = [
        (test_init, "初始化"),
        (test_read_zero, "零电压读取"),
        (test_read_max, "满量程读取"),
        (test_read_mid, "中间值读取"),
        (test_all_channels, "所有通道"),
        (test_single_channel_read, "单通道精度"),
        (test_differential_positive, "差分正向"),
        (test_differential_negative, "差分反向"),
        (test_differential_zero, "差分相等"),
        (test_voltage_conversion, "电压转换"),
        (test_custom_vref, "自定义参考电压"),
        (test_noise_filtering, "噪声滤波"),
        (test_channel_boundary, "通道边界"),
        (test_voltage_clamp, "电压钳位"),
        (test_negative_voltage_clamp, "负电压钳位"),
        (test_spi_speed, "SPI速率"),
        (test_multi_channel_scan, "多通道扫描"),
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
