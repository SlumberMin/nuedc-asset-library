# -*- coding: utf-8 -*-
"""
test_mcp4921_v2.py - MCP4921 DAC测试 V2
========================================
测试内容：
  1. SPI通信模拟
  2. 12位DAC输出（0-4095）
  3. 单通道/双通道输出
  4. 电压输出计算
  5. 增益设置（1x/2x）
  6. 输出关断（SHDN）
  7. 缓冲/非缓冲模式
  8. 波形生成（正弦/三角/锯齿）
  9. PID控制DAC输出
  10. 精度与分辨率测试

使用 wrappers.py 的 PIDController、SimpleMA
"""

import sys
import os
import math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from wrappers import PIDController, SimpleMA

# ═══════════════════════════════════════════════════════════════
#  MCP4921常量
# ═══════════════════════════════════════════════════════════════
MCP4921_RESOLUTION = 12  # 位
MCP4921_MAX_VALUE = (1 << MCP4921_RESOLUTION) - 1  # 4095

# 控制位（16位SPI帧的高4位）
BIT_AB = 0x8000   # 0=DACA, 1=DACB
BIT_BUF = 0x4000  # 0=不缓冲, 1=缓冲
BIT_GA = 0x2000   # 0=2x增益, 1=1x增益
BIT_SHDN = 0x1000 # 0=关断, 1=使能输出


class MCP4921:
    """MCP4921 12位双通道DAC模拟器"""

    def __init__(self, vref=3.3):
        self.vref = vref
        # 两个DAC通道的原始值
        self._dac_a = 0
        self._dac_b = 0
        # 控制状态
        self._gain_a = 1  # 1x或2x
        self._gain_b = 1
        self._shutdown_a = False
        self._shutdown_b = False
        self._buffered_a = False
        self._buffered_b = False
        # SPI
        self._spi_speed = 10000000  # 10MHz默认

    def write_a(self, value):
        """写DAC A通道（0-4095）"""
        value = max(0, min(MCP4921_MAX_VALUE, value))
        self._dac_a = value

    def write_b(self, value):
        """写DAC B通道（0-4095）"""
        value = max(0, min(MCP4921_MAX_VALUE, value))
        self._dac_b = value

    def write_voltage_a(self, voltage):
        """通过电压写DAC A"""
        gain = self._gain_a if not self._shutdown_a else 0
        if gain == 0:
            self._dac_a = 0
            return
        max_v = self.vref * gain
        raw = int((voltage / max_v) * MCP4921_MAX_VALUE + 0.5)
        self.write_a(raw)

    def write_voltage_b(self, voltage):
        """通过电压写DAC B"""
        gain = self._gain_b if not self._shutdown_b else 0
        if gain == 0:
            self._dac_b = 0
            return
        max_v = self.vref * gain
        raw = int((voltage / max_v) * MCP4921_MAX_VALUE + 0.5)
        self.write_b(raw)

    def get_output_a(self):
        """获取DAC A原始值"""
        if self._shutdown_a:
            return 0
        return self._dac_a

    def get_output_b(self):
        """获取DAC B原始值"""
        if self._shutdown_b:
            return 0
        return self._dac_b

    def get_voltage_a(self):
        """获取DAC A输出电压"""
        if self._shutdown_a:
            return 0.0
        return (self._dac_a / MCP4921_MAX_VALUE) * self.vref * self._gain_a

    def get_voltage_b(self):
        """获取DAC B输出电压"""
        if self._shutdown_b:
            return 0.0
        return (self._dac_b / MCP4921_MAX_VALUE) * self.vref * self._gain_b

    def set_gain_a(self, gain):
        """设置A通道增益（1或2）"""
        self._gain_a = gain if gain in (1, 2) else 1

    def set_gain_b(self, gain):
        """设置B通道增益（1或2）"""
        self._gain_b = gain if gain in (1, 2) else 1

    def shutdown_a(self):
        """关断A通道"""
        self._shutdown_a = True

    def shutdown_b(self):
        """关断B通道"""
        self._shutdown_b = True

    def enable_a(self):
        """使能A通道"""
        self._shutdown_a = False

    def enable_b(self):
        """使能B通道"""
        self._shutdown_b = False

    def set_buffered_a(self, buf):
        """设置A通道缓冲模式"""
        self._buffered_a = buf

    def set_buffered_b(self, buf):
        """设置B通道缓冲模式"""
        self._buffered_b = buf

    def get_resolution(self):
        """返回DAC分辨率"""
        return MCP4921_RESOLUTION

    def get_max_value(self):
        """返回最大DAC值"""
        return MCP4921_MAX_VALUE

    def build_spi_word(self, channel, value, buffered=False, gain=1, shutdown=True):
        """构建16位SPI控制字"""
        word = value & 0x0FFF
        if channel == 1:
            word |= BIT_AB
        if buffered:
            word |= BIT_BUF
        if gain == 1:
            word |= BIT_GA
        if shutdown:
            word |= BIT_SHDN
        return word


# ---- 测试辅助 ----
def assert_eq(actual, expected, msg=""):
    if actual != expected:
        raise AssertionError(f"{msg}: 期望 {expected}, 实际 {actual}")

def assert_close(actual, expected, tol=0.02, msg=""):
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
    """MCP4921初始化"""
    dac = MCP4921()
    assert_eq(dac.get_resolution(), 12, "分辨率")
    assert_eq(dac.get_max_value(), 4095, "最大值")
    assert_eq(dac.get_output_a(), 0, "初始A")
    assert_eq(dac.get_output_b(), 0, "初始B")

def test_write_a():
    """写DAC A通道"""
    dac = MCP4921()
    dac.write_a(2048)
    assert_eq(dac.get_output_a(), 2048, "A通道raw")

def test_write_b():
    """写DAC B通道"""
    dac = MCP4921()
    dac.write_b(1000)
    assert_eq(dac.get_output_b(), 1000, "B通道raw")

def test_voltage_output_1x():
    """1x增益电压输出"""
    dac = MCP4921(vref=3.3)
    dac.set_gain_a(1)
    dac.write_a(4095)
    assert_close(dac.get_voltage_a(), 3.3, 0.01, "1x满量程")
    dac.write_a(2048)
    assert_close(dac.get_voltage_a(), 1.65, 0.02, "1x半量程")

def test_voltage_output_2x():
    """2x增益电压输出"""
    dac = MCP4921(vref=3.3)
    dac.set_gain_a(2)
    dac.write_a(4095)
    assert_close(dac.get_voltage_a(), 6.6, 0.02, "2x满量程")

def test_voltage_write():
    """通过电压写入"""
    dac = MCP4921(vref=3.3)
    dac.set_gain_a(1)
    dac.write_voltage_a(1.65)
    assert_close(dac.get_voltage_a(), 1.65, 0.02, "电压写入")

def test_shutdown():
    """输出关断"""
    dac = MCP4921()
    dac.write_a(2048)
    assert_eq(dac.get_output_a(), 2048, "关断前")
    dac.shutdown_a()
    assert_eq(dac.get_output_a(), 0, "关断后raw")
    assert_close(dac.get_voltage_a(), 0.0, 0.01, "关断后电压")
    dac.enable_a()
    assert_eq(dac.get_output_a(), 2048, "使能后")

def test_shutdown_b():
    """B通道关断"""
    dac = MCP4921()
    dac.write_b(3000)
    dac.shutdown_b()
    assert_eq(dac.get_output_b(), 0, "B关断")
    dac.enable_b()
    assert_eq(dac.get_output_b(), 3000, "B使能")

def test_clamp():
    """值钳位"""
    dac = MCP4921()
    dac.write_a(5000)  # 超过4095
    assert_eq(dac.get_output_a(), 4095, "上限钳位")
    dac.write_a(-100)
    assert_eq(dac.get_output_a(), 0, "下限钳位")

def test_gain_switch():
    """增益切换"""
    dac = MCP4921(vref=3.3)
    dac.write_a(4095)
    dac.set_gain_a(1)
    v1 = dac.get_voltage_a()
    dac.set_gain_a(2)
    v2 = dac.get_voltage_a()
    assert_close(v2, v1 * 2, 0.02, "2x应为1x的两倍")

def test_resolution():
    """分辨率测试（1 LSB）"""
    dac = MCP4921(vref=3.3)
    dac.set_gain_a(1)
    lsb = 3.3 / 4095
    dac.write_a(0)
    v0 = dac.get_voltage_a()
    dac.write_a(1)
    v1 = dac.get_voltage_a()
    assert_close(v1 - v0, lsb, 0.0001, "1 LSB")

def test_buffered_mode():
    """缓冲模式"""
    dac = MCP4921()
    dac.set_buffered_a(True)
    assert dac._buffered_a, "缓冲应使能"
    dac.set_buffered_a(False)
    assert not dac._buffered_a, "缓冲应禁用"

def test_spi_word():
    """SPI控制字构建"""
    dac = MCP4921()
    # DAC A, 不缓冲, 1x增益, 使能
    word = dac.build_spi_word(0, 0x0FFF, buffered=False, gain=1, shutdown=True)
    assert_eq(word, 0x3FFF, "A通道控制字")
    # DAC B
    word = dac.build_spi_word(1, 0x0000, buffered=False, gain=2, shutdown=True)
    assert_eq(word, 0x9000, "B通道控制字")

def test_sine_wave():
    """正弦波生成"""
    dac = MCP4921(vref=3.3)
    dac.set_gain_a(1)
    samples = 64
    center = 2048
    amplitude = 2047
    values = []
    for i in range(samples):
        val = int(center + amplitude * math.sin(2 * math.pi * i / samples))
        dac.write_a(val)
        values.append(dac.get_output_a())
    assert min(values) >= 0, "正弦波最小值>=0"
    assert max(values) <= 4095, "正弦波最大值<=4095"
    assert len(values) == samples, "采样数正确"

def test_triangle_wave():
    """三角波生成"""
    dac = MCP4921()
    samples = 100
    values = []
    for i in range(samples):
        if i < samples // 2:
            val = int(4095 * (i / (samples / 2)))
        else:
            val = int(4095 * ((samples - i) / (samples / 2)))
        dac.write_a(val)
        values.append(dac.get_output_a())
    assert min(values) >= 0, "三角波最小值>=0"
    assert max(values) <= 4095, "三角波最大值<=4095"

def test_pid_control():
    """PID控制DAC输出"""
    pid = PIDController(kp=1000.0, ki=50.0, kd=1.0, output_min=0, output_max=4095)
    dac = MCP4921(vref=3.3)
    dac.set_gain_a(1)
    target = 3.0  # 目标电压
    for _ in range(200):
        current_v = dac.get_voltage_a()
        output = pid.calc(target, current_v)
        dac.write_a(int(max(0, min(4095, output))))
    final_v = dac.get_voltage_a()
    assert_close(final_v, target, 0.2, "PID稳态电压")

def test_ma_filter():
    """移动平均滤波"""
    ma = SimpleMA(8)
    for i in range(20):
        ma.update(100 + (i % 3))  # 100, 101, 102循环
    avg = ma.last_output
    assert_close(avg, 101.0, 0.5, "移动平均")

def test_dual_channel():
    """双通道独立输出"""
    dac = MCP4921(vref=3.3)
    dac.set_gain_a(1)
    dac.set_gain_b(1)
    dac.write_a(1000)
    dac.write_b(3000)
    assert_eq(dac.get_output_a(), 1000, "A独立")
    assert_eq(dac.get_output_b(), 3000, "B独立")
    assert_close(dac.get_voltage_a(), 1000 * 3.3 / 4095, 0.02, "A电压")
    assert_close(dac.get_voltage_b(), 3000 * 3.3 / 4095, 0.02, "B电压")


def main():
    print("=" * 60)
    print("  MCP4921 DAC测试 V2")
    print("=" * 60)
    tests = [
        (test_init, "初始化"),
        (test_write_a, "写DAC A"),
        (test_write_b, "写DAC B"),
        (test_voltage_output_1x, "1x增益输出"),
        (test_voltage_output_2x, "2x增益输出"),
        (test_voltage_write, "电压写入"),
        (test_shutdown, "输出关断"),
        (test_shutdown_b, "B通道关断"),
        (test_clamp, "值钳位"),
        (test_gain_switch, "增益切换"),
        (test_resolution, "分辨率"),
        (test_buffered_mode, "缓冲模式"),
        (test_spi_word, "SPI控制字"),
        (test_sine_wave, "正弦波生成"),
        (test_triangle_wave, "三角波生成"),
        (test_pid_control, "PID控制"),
        (test_ma_filter, "移动平均"),
        (test_dual_channel, "双通道"),
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
