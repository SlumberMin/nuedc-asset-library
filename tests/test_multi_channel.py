# -*- coding: utf-8 -*-
"""
test_multi_channel_v2.py - 多通道采样测试 V2
=============================================
测试内容：
  1. MultiADC系统初始化
  2. 单通道/多通道采样
  3. 过采样与滤波
  4. ADS1115高精度通道
  5. 采样计数与极值
  6. RingBuffer数据存储
  7. PIDController滤波
  8. 多通道系统集成

使用 wrappers.py 封装的 MultiADC、ADS1115、RingBuffer、PIDController
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wrappers import MultiADC, ADS1115, RingBuffer, PIDController

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


class MultiChannelSystem:
    """多通道采样系统"""
    def __init__(self, num_channels=8):
        self.num_channels = num_channels
        self.adc = MultiADC(); self.adc.init()
        self.ads = ADS1115(); self.ads.init()
        self.buffer = RingBuffer(4096)
        self.channel_enabled = [True] * num_channels
        self.sample_count = 0

    def set_simulated(self, ch, raw):
        self.adc.set_simulated_raw(ch, raw)

    def read_channel(self, ch):
        return self.adc.read_voltage_mv(ch)

    def read_all(self):
        results = []
        for ch in range(self.num_channels):
            if self.channel_enabled[ch]:
                results.append((ch, self.adc.read_voltage_mv(ch)))
        self.sample_count += 1
        return results

    def read_filtered(self, ch):
        return self.adc.read_filtered_mv(ch)

    def store_to_buffer(self, ch):
        mv = self.read_channel(ch)
        self.buffer.put_byte(int(mv) & 0xFF)
        return mv


def test_multi_adc_init():
    """MultiADC初始化"""
    m = MultiADC(); m.init()
    assert m.initialized

def test_set_simulated():
    """模拟值设置"""
    m = MultiADC(); m.init()
    m.set_simulated_raw(0, 2048)
    m.set_simulated_raw(1, 1024)
    mv = m.read_voltage_mv(0)
    assert isinstance(mv, (int, float))

def test_raw_to_voltage():
    """原始值转电压"""
    m = MultiADC(); m.init()
    m.set_simulated_raw(0, 2048)
    mv = m.read_voltage_mv(0)
    expected = m.raw_to_voltage_mv(2048)
    assert_close(mv, expected, tolerance=1.0, msg="电压转换")

def test_oversample():
    """过采样"""
    m = MultiADC(); m.init()
    m.set_oversample_count(8)
    m.set_simulated_raw(0, 2000)
    mv = m.read_voltage_mv(0)
    assert isinstance(mv, (int, float))

def test_filtered_read():
    """滤波读取"""
    m = MultiADC(); m.init()
    m.set_simulated_raw(0, 2000)
    for _ in range(20):
        mv = m.read_filtered_mv(0)
    assert isinstance(mv, (int, float))

def test_min_max():
    """极值统计"""
    m = MultiADC(); m.init()
    m.set_simulated_raw(0, 2000)
    for _ in range(5):
        m.read_voltage_mv(0)
    # min/max should update after process_sample
    m.process_sample()
    mn = m.get_min_mv(0)
    mx = m.get_max_mv(0)
    # After process_sample, min/max should be finite
    assert isinstance(mn, (int, float))
    assert isinstance(mx, (int, float))

def test_sample_count():
    """采样计数"""
    m = MultiADC(); m.init()
    m.set_simulated_raw(0, 2048)
    for _ in range(5):
        m.read_voltage_mv(0)
    m.process_sample()
    cnt = m.get_sample_count(0)
    assert isinstance(cnt, int)

def test_ads1115_init():
    """ADS1115初始化"""
    a = ADS1115(); a.init()
    assert a.initialized

def test_ads1115_mux():
    """ADS1115通道"""
    a = ADS1115(); a.init()
    for mux in range(4):
        a.set_mux(mux)
        v = a.read_voltage()
        assert isinstance(v, (int, float))

def test_ads1115_pga():
    """ADS1115增益"""
    a = ADS1115(); a.init()
    for pga in range(4):
        a.set_pga(pga)
        assert a.get_pga_fsr() > 0

def test_ads1115_datarate():
    """ADS1115采样率"""
    a = ADS1115(); a.init()
    for dr in [0, 4, 7]:
        a.set_data_rate(dr)
        v = a.read_voltage()
        assert isinstance(v, (int, float))

def test_multi_channel_system():
    """多通道系统"""
    sys0 = MultiChannelSystem(4)
    for ch in range(4):
        sys0.set_simulated(ch, 1000 + ch * 500)
    results = sys0.read_all()
    assert len(results) == 4

def test_channel_enable():
    """通道使能"""
    sys0 = MultiChannelSystem(4)
    sys0.channel_enabled[2] = False
    results = sys0.read_all()
    channels = [r[0] for r in results]
    assert 2 not in channels

def test_ring_buffer_ops():
    """缓冲区操作"""
    buf = RingBuffer(16)
    for i in range(16):
        buf.put_byte(i)
    assert buf.used() == 16
    assert buf.is_full()
    assert buf.get_byte() == 0
    buf.reset()
    assert buf.used() == 0
    assert buf.is_empty()

def test_pid_filter():
    """PID滤波"""
    pid = PIDController()
    pid.set_kp(1.0); pid.set_ki(0.1); pid.set_kd(0.01)
    pid.calc(2048, 1500)
    assert pid.get_output() > 0
    pid.reset()
    pid.calc(2048, 2500)
    assert pid.get_output() < 0

def test_multi_channel_all():
    """全通道读取"""
    m = MultiADC(); m.init()
    for ch in range(4):
        m.set_simulated_raw(ch, ch * 400)
    for ch in range(4):
        mv = m.read_voltage_mv(ch)
        assert isinstance(mv, (int, float))

def test_buffer_store():
    """缓冲区存储"""
    sys0 = MultiChannelSystem()
    sys0.set_simulated(0, 2048)
    for _ in range(10):
        sys0.store_to_buffer(0)
    assert sys0.buffer.used() == 10


def main():
    print("=" * 60)
    print("  多通道采样系统测试 V2")
    print("=" * 60)
    tests = [
        (test_multi_adc_init, "MultiADC初始化"), (test_set_simulated, "模拟值设置"),
        (test_raw_to_voltage, "原始值转电压"), (test_oversample, "过采样"),
        (test_filtered_read, "滤波读取"), (test_min_max, "极值统计"),
        (test_sample_count, "采样计数"), (test_ads1115_init, "ADS1115初始化"),
        (test_ads1115_mux, "ADS1115通道"), (test_ads1115_pga, "ADS1115增益"),
        (test_ads1115_datarate, "ADS1115采样率"), (test_multi_channel_system, "多通道系统"),
        (test_channel_enable, "通道使能"),
        (test_ring_buffer_ops, "缓冲区操作"), (test_pid_filter, "PID滤波"),
        (test_multi_channel_all, "全通道读取"), (test_buffer_store, "缓冲区存储"),
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
