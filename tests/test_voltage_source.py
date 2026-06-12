# -*- coding: utf-8 -*-
"""
test_voltage_source_v2.py - 精密电压源测试 V2
==============================================
测试内容：
  1. MCP4725 DAC初始化与输出
  2. DAC电压设置与精度
  3. DAC全量程测试
  4. ADS1115 ADC初始化与读取
  5. ADC通道/增益配置
  6. AD9833 DDS波形发生
  7. MultiADC多通道监控
  8. PIDController稳压
  9. RingBuffer数据缓冲
  10. 电压源系统集成

使用 wrappers.py 封装的 MCP4725、ADS1115、AD9833、MultiADC、PIDController、RingBuffer
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wrappers import MCP4725, ADS1115, AD9833, MultiADC, PIDController, RingBuffer

# ---- 测试辅助函数 ----
def assert_close(actual, expected, tolerance=0.01, msg=""):
    if abs(actual - expected) > tolerance:
        raise AssertionError(f"{msg}: 期望 {expected}±{tolerance}, 实际 {actual}")

def assert_in_range(value, min_val, max_val, msg=""):
    if value < min_val or value > max_val:
        raise AssertionError(f"{msg}: {value} 不在 [{min_val}, {max_val}] 范围内")

def run_test(test_func, test_name=""):
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


class VoltageSource:
    """精密电压源系统"""
    def __init__(self):
        self.dac = MCP4725(); self.dac.init()
        self.adc = ADS1115(); self.adc.init()
        self.dds = AD9833(); self.dds.init()
        self.madc = MultiADC(); self.madc.init()
        self.pid = PIDController()
        self.pid.set_kp(10.0); self.pid.set_ki(1.0); self.pid.set_kd(0.5)
        self.target_mv = 0; self.max_mv = 3300; self.over_voltage = False

    def set_voltage_mv(self, mv):
        if mv > self.max_mv or mv < 0:
            self.over_voltage = True; mv = max(0, min(self.max_mv, mv))
        else:
            self.over_voltage = False
        self.target_mv = mv; self.dac.set_voltage(mv / 1000.0)

def test_dac_init():
    """MCP4725初始化"""
    d = MCP4725(); d.init()
    assert d.get_value() >= 0

def test_dac_voltage():
    """DAC电压设置"""
    d = MCP4725(); d.init()
    d.set_voltage(1.65)
    assert_close(d.get_voltage(), 1.65, tolerance=0.05, msg="DAC 1.65V")

def test_dac_value():
    """DAC值设置"""
    d = MCP4725(); d.init()
    d.set_value(0); assert d.get_value() == 0
    d.set_value(2048)
    assert_close(d.get_voltage(), 1.65, tolerance=0.05, msg="DAC值2048")

def test_dac_full_range():
    """DAC全量程"""
    d = MCP4725(); d.init()
    d.set_value(4095)
    v = d.get_voltage()
    assert v > 3.0, f"满量程应>3.0V: {v}"

def test_dac_power_down():
    """DAC掉电模式"""
    d = MCP4725(); d.init()
    d.set_power_down(1)  # PD_1K
    s = d.read_status()
    assert s is not None
    d.set_power_down(0)  # PD_NONE

def test_dac_eeprom():
    """DAC EEPROM写入"""
    d = MCP4725(); d.init()
    d.set_value(1000); d.write_eeprom()

def test_adc_init():
    """ADS1115初始化"""
    a = ADS1115(); a.init()
    assert a.initialized

def test_adc_read():
    """ADS1115读取"""
    a = ADS1115(); a.init()
    v = a.read_voltage()
    assert isinstance(v, (int, float))

def test_adc_mux():
    """ADS1115通道切换"""
    a = ADS1115(); a.init()
    for mux in range(4):
        a.set_mux(mux)
        v = a.read_voltage()
        assert isinstance(v, (int, float))

def test_adc_pga():
    """ADS1115增益配置"""
    a = ADS1115(); a.init()
    for pga in range(4):
        a.set_pga(pga)
        fsr = a.get_pga_fsr()
        assert fsr > 0

def test_adc_data_rate():
    """ADS1115采样率"""
    a = ADS1115(); a.init()
    for dr in [0, 4, 7]:
        a.set_data_rate(dr)
        v = a.read_voltage()
        assert isinstance(v, (int, float))

def test_adc_simulated():
    """ADS1115模拟值"""
    a = ADS1115(); a.init()
    a.set_simulated_raw(2048)
    v = a.read_voltage()
    assert isinstance(v, (int, float))

def test_multi_adc():
    """MultiADC多通道"""
    m = MultiADC(); m.init()
    for ch in range(4):
        m.set_simulated_raw(ch, 2048)
    for ch in range(4):
        mv = m.read_voltage_mv(ch)
        assert isinstance(mv, (int, float))

def test_multi_adc_oversample():
    """MultiADC过采样"""
    m = MultiADC(); m.init()
    m.set_oversample_count(8)
    m.set_simulated_raw(0, 2000)
    mv = m.read_voltage_mv(0)
    assert isinstance(mv, (int, float))

def test_multi_adc_filtered():
    """MultiADC滤波"""
    m = MultiADC(); m.init()
    m.set_simulated_raw(0, 2000)
    for _ in range(10):
        mv = m.read_filtered_mv(0)
    assert isinstance(mv, (int, float))

def test_dds_init():
    """AD9833 DDS初始化"""
    d = AD9833(); d.init()
    assert d.initialized

def test_dds_frequency():
    """DDS频率设置"""
    d = AD9833(); d.init()
    d.set_frequency(1000)
    assert d.get_frequency() > 0

def test_dds_waveform():
    """DDS波形设置"""
    d = AD9833(); d.init()
    for w in range(3):  # 0=SINE, 1=TRIANGLE, 2=SQUARE
        d.set_waveform(w)
        assert d.get_waveform() == w

def test_pid_stabilize():
    """PID稳压"""
    pid = PIDController()
    pid.set_kp(5.0); pid.set_ki(0.5); pid.set_kd(1.0)
    current = 1000.0
    for _ in range(200):
        pid.calc(1650, current)
        current += pid.get_output() * 0.01
    assert_close(current, 1650, tolerance=50, msg="PID收敛")

def test_ring_buffer():
    """环形缓冲区"""
    buf = RingBuffer(64)
    buf.put_byte(1); buf.put_byte(2)
    assert buf.used() == 2
    assert buf.get_byte() == 1

def test_voltage_source():
    """电压源系统"""
    vs = VoltageSource()
    vs.set_voltage_mv(1650)
    assert vs.target_mv == 1650
    assert not vs.over_voltage

def test_over_voltage():
    """过压保护"""
    vs = VoltageSource()
    vs.set_voltage_mv(4000)
    assert vs.over_voltage


def main():
    print("=" * 60)
    print("  精密电压源系统测试 V2")
    print("=" * 60)
    tests = [
        (test_dac_init, "DAC初始化"), (test_dac_voltage, "DAC电压"),
        (test_dac_value, "DAC值"), (test_dac_full_range, "DAC全量程"),
        (test_dac_power_down, "DAC掉电"), (test_dac_eeprom, "DAC EEPROM"),
        (test_adc_init, "ADC初始化"), (test_adc_read, "ADC读取"),
        (test_adc_mux, "ADC通道"), (test_adc_pga, "ADC增益"),
        (test_adc_data_rate, "ADC采样率"), (test_adc_simulated, "ADC模拟"),
        (test_multi_adc, "MultiADC"), (test_multi_adc_oversample, "过采样"),
        (test_multi_adc_filtered, "滤波"),
        (test_dds_init, "DDS初始化"), (test_dds_frequency, "DDS频率"),
        (test_dds_waveform, "DDS波形"), (test_pid_stabilize, "PID稳压"),
        (test_ring_buffer, "环形缓冲区"), (test_voltage_source, "电压源系统"),
        (test_over_voltage, "过压保护"),
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
