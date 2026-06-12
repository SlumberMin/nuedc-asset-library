#!/usr/bin/env python3
"""
TFT示波器V2测试 — ST7789显示 + ADC采集 + 波形绘制
覆盖: TFT波形渲染、ADC多通道采集、触发系统、
      时基/幅值调节、波形存储与回放
对应C源文件: 02_mspm0g3507/drivers/st7789.c + 示波器算法

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import ST7789, MultiADC, RingBuffer


# ═══════════════════════════════════════════════════════════════
#  示波器参数常量
# ═══════════════════════════════════════════════════════════════
# 波形显示区域
WAVE_X = 20           # 波形起始X
WAVE_Y = 30           # 波形起始Y
WAVE_W = 280          # 波形宽度
WAVE_H = 180          # 波形高度
WAVE_MID_Y = WAVE_Y + WAVE_H // 2  # 中线Y坐标

# 触发模式
TRIG_NONE = 0         # 自动触发
TRIG_RISING = 1       # 上升沿触发
TRIG_FALLING = 2      # 下降沿触发

# 时基档位 (ms/div)
TIMEBASE_LIST = [1, 2, 5, 10, 20, 50, 100, 200, 500]
TIMEBASE_DIV = 10     # 水平方向10格

# 幅值档位 (mV/div)
AMP_LIST = [50, 100, 200, 500, 1000, 2000, 5000]
AMP_DIV = 8           # 垂直方向8格

# 颜色
COLOR_BG = 0x0000      # 黑色背景
COLOR_GRID = 0x4208    # 灰色网格
COLOR_WAVE1 = 0x07E0   # 绿色波形1
COLOR_WAVE2 = 0xFFE0   # 黄色波形2
COLOR_TRIGGER = 0xF800  # 红色触发线
COLOR_TEXT = 0xFFFF     # 白色文字


class OscilloscopeChannel:
    """示波器通道数据"""

    def __init__(self, ch_id=0):
        self.ch_id = ch_id
        self.enabled = True
        self.offset_mv = 0       # 偏移(mV)
        self.amp_idx = 3         # 幅值档位索引(默认500mV/div)
        self.buffer = []         # 采样缓冲
        self.captured = []       # 捕获的波形
        self.color = COLOR_WAVE1 if ch_id == 0 else COLOR_WAVE2


class TFTScope:
    """TFT示波器V2 — 双通道 + 触发 + 存储

    功能:
    - 双通道ADC采集（12bit）
    - 上升沿/下降沿/自动触发
    - 时基/幅值可调
    - 波形网格绘制
    - 波形存储/回放
    """

    def __init__(self):
        self.tft = ST7789()
        self.adc = MultiADC()
        self.sample_ring = RingBuffer(4096)  # 采样环形缓冲

        # 通道
        self.channels = [OscilloscopeChannel(0), OscilloscopeChannel(1)]

        # 触发
        self.trig_mode = TRIG_RISING
        self.trig_level = 2048   # 触发电平(12bit ADC中值)
        self.trig_channel = 0
        self.trig_pos = 0       # 触发点在波形中的位置
        self.triggered = False

        # 时基
        self.timebase_idx = 4    # 默认20ms/div
        self.running = True

        # 波形存储槽
        self.saved_waveforms = [None, None, None]  # 3个存储槽
        self.current_slot = 0

        # 统计
        self.sample_count = 0
        self.fps = 0

    def init(self):
        """初始化硬件"""
        self.tft.init()
        self.tft.display_on()
        self.tft.set_rotation(1)  # 横屏
        self.adc.init()
        self.sample_ring.reset()
        self._draw_grid()

    def _draw_grid(self):
        """绘制示波器网格"""
        # 水平网格线
        for i in range(AMP_DIV + 1):
            y = WAVE_Y + i * (WAVE_H // AMP_DIV)
            self.tft.draw_hline(WAVE_X, y, WAVE_W, COLOR_GRID)

        # 垂直网格线
        for i in range(TIMEBASE_DIV + 1):
            x = WAVE_X + i * (WAVE_W // TIMEBASE_DIV)
            self.tft.draw_vline(x, WAVE_Y, WAVE_H, COLOR_GRID)

        # 中心十字线（稍亮）
        mid_y = WAVE_Y + WAVE_H // 2
        mid_x = WAVE_X + WAVE_W // 2
        self.tft.draw_hline(WAVE_X, mid_y, WAVE_W, 0x632C)
        self.tft.draw_vline(mid_x, WAVE_Y, WAVE_H, 0x632C)

    def get_amp_mv_per_div(self):
        """获取当前幅值档位(mV/div)"""
        return AMP_LIST[self.channels[0].amp_idx]

    def get_timebase_ms_per_div(self):
        """获取当前时基(ms/div)"""
        return TIMEBASE_LIST[self.timebase_idx]

    def set_trigger(self, mode, level, channel=0):
        """设置触发参数"""
        self.trig_mode = mode
        self.trig_level = max(0, min(4095, level))
        self.trig_channel = channel

    def _check_trigger(self, prev_val, curr_val):
        """检测触发条件"""
        if self.trig_mode == TRIG_NONE:
            return True
        elif self.trig_mode == TRIG_RISING:
            return prev_val < self.trig_level and curr_val >= self.trig_level
        elif self.trig_mode == TRIG_FALLING:
            return prev_val >= self.trig_level and curr_val < self.trig_level
        return False

    def capture(self, samples_ch0, samples_ch1=None):
        """捕获波形数据

        samples_ch0: 通道0的ADC原始值列表
        samples_ch1: 通道1的ADC原始值列表（可选）
        """
        # 存入通道缓冲
        self.channels[0].buffer = list(samples_ch0)
        if samples_ch1:
            self.channels[1].buffer = list(samples_ch1)

        self.sample_count = len(samples_ch0)

        # 寻找触发点
        self.triggered = False
        self.trig_pos = 0

        if self.trig_mode == TRIG_NONE:
            self.triggered = True
            self.trig_pos = 0
        else:
            trig_ch = self.channels[self.trig_channel].buffer
            for i in range(1, len(trig_ch)):
                if self._check_trigger(trig_ch[i - 1], trig_ch[i]):
                    self.trig_pos = i
                    self.triggered = True
                    break

        # 保存捕获数据
        self.channels[0].captured = list(self.channels[0].buffer)
        if samples_ch1:
            self.channels[1].captured = list(self.channels[1].buffer)

    def draw_waveform(self, channel_idx=0):
        """绘制波形到TFT"""
        ch = self.channels[channel_idx]
        if not ch.enabled or not ch.captured:
            return

        data = ch.captured
        n = len(data)
        if n < 2:
            return

        # 计算缩放
        amp_mv = AMP_LIST[ch.amp_idx]
        pixels_per_div_y = WAVE_H // AMP_DIV
        # 12bit ADC → mV: 假设3.3V参考, 4096=3300mV
        mv_per_step = 3300.0 / 4096.0
        pixels_per_mv = pixels_per_div_y / (amp_mv * 1.0)

        # 触发偏移
        trig_offset = max(0, self.trig_pos - WAVE_W // 2)

        # 绘制波形线段
        prev_y = None
        for i in range(min(WAVE_W, n - trig_offset)):
            idx = trig_offset + i
            if idx >= n:
                break
            val_mv = data[idx] * mv_per_step - 1650.0  # 减去中心值
            y = int(WAVE_MID_Y - val_mv * pixels_per_mv + ch.offset_mv * pixels_per_mv / amp_mv)
            y = max(WAVE_Y, min(WAVE_Y + WAVE_H - 1, y))
            x = WAVE_X + i

            if prev_y is not None:
                self.tft.draw_line(x - 1, prev_y, x, y, ch.color)
            prev_y = y

    def draw_trigger_line(self):
        """绘制触发电平线"""
        amp_mv = AMP_LIST[self.channels[0].amp_idx]
        mv_per_step = 3300.0 / 4096.0
        pixels_per_div_y = WAVE_H // AMP_DIV
        pixels_per_mv = pixels_per_div_y / (amp_mv * 1.0)

        trig_mv = self.trig_level * mv_per_step - 1650.0
        y = int(WAVE_MID_Y - trig_mv * pixels_per_mv)
        y = max(WAVE_Y, min(WAVE_Y + WAVE_H - 1, y))
        self.tft.draw_hline(WAVE_X, y, WAVE_W, COLOR_TRIGGER)

    def save_waveform(self, slot):
        """保存波形到存储槽"""
        if 0 <= slot < len(self.saved_waveforms):
            self.saved_waveforms[slot] = {
                'ch0': list(self.channels[0].captured),
                'ch1': list(self.channels[1].captured),
                'timebase': self.timebase_idx,
                'amp': self.channels[0].amp_idx,
            }
            return True
        return False

    def load_waveform(self, slot):
        """从存储槽加载波形"""
        if 0 <= slot < len(self.saved_waveforms) and self.saved_waveforms[slot]:
            data = self.saved_waveforms[slot]
            self.channels[0].captured = data['ch0']
            self.channels[1].captured = data['ch1']
            self.timebase_idx = data['timebase']
            self.channels[0].amp_idx = data['amp']
            return True
        return False

    def measure_frequency(self, channel_idx=0):
        """测量频率（过零检测法）"""
        ch = self.channels[channel_idx]
        if not ch.captured or len(ch.captured) < 10:
            return 0.0

        data = ch.captured
        mid = 2048  # 12bit中值

        # 寻找上升沿过零点
        crossings = []
        for i in range(1, len(data)):
            if data[i - 1] < mid and data[i] >= mid:
                crossings.append(i)

        if len(crossings) < 2:
            return 0.0

        # 计算平均周期
        periods = []
        for i in range(1, len(crossings)):
            periods.append(crossings[i] - crossings[i - 1])

        avg_period = sum(periods) / len(periods)
        timebase_ms = TIMEBASE_LIST[self.timebase_idx]
        samples_per_div = WAVE_W // TIMEBASE_DIV
        ms_per_sample = timebase_ms / samples_per_div
        period_ms = avg_period * ms_per_sample

        if period_ms <= 0:
            return 0.0
        return 1000.0 / period_ms  # Hz

    def measure_vpp(self, channel_idx=0):
        """测量峰峰值(mV)"""
        ch = self.channels[channel_idx]
        if not ch.captured:
            return 0.0

        mv_per_step = 3300.0 / 4096.0
        v_min = min(ch.captured) * mv_per_step
        v_max = max(ch.captured) * mv_per_step
        return v_max - v_min


# ═══════════════════════════════════════════════════════════════
#  测试类
# ═══════════════════════════════════════════════════════════════

class TestScopeInit(unittest.TestCase):
    """示波器初始化测试"""

    def test_init_success(self):
        """硬件初始化成功"""
        scope = TFTScope()
        scope.init()
        self.assertTrue(scope.tft._initialized)
        self.assertTrue(scope.running)

    def test_default_trigger(self):
        """默认触发设置"""
        scope = TFTScope()
        self.assertEqual(scope.trig_mode, TRIG_RISING)
        self.assertEqual(scope.trig_level, 2048)
        self.assertEqual(scope.trig_channel, 0)

    def test_default_channels(self):
        """默认双通道"""
        scope = TFTScope()
        self.assertEqual(len(scope.channels), 2)
        self.assertTrue(scope.channels[0].enabled)
        self.assertTrue(scope.channels[1].enabled)

    def test_default_timebase(self):
        """默认时基"""
        scope = TFTScope()
        self.assertEqual(scope.get_timebase_ms_per_div(), 20)

    def test_default_amp(self):
        """默认幅值"""
        scope = TFTScope()
        self.assertEqual(scope.get_amp_mv_per_div(), 500)


class TestScopeGrid(unittest.TestCase):
    """网格绘制测试"""

    def test_grid_drawn(self):
        """网格已绘制"""
        scope = TFTScope()
        scope.init()
        # 检查网格交点处有像素
        grid_x = WAVE_X
        grid_y = WAVE_Y
        pixel = scope.tft.get_pixel(grid_x, grid_y)
        self.assertNotEqual(pixel, 0)  # 应有网格颜色


class TestScopeCapture(unittest.TestCase):
    """波形捕获测试"""

    def test_capture_sine(self):
        """正弦波捕获"""
        scope = TFTScope()
        scope.init()
        # 生成正弦波 (12bit ADC值)
        samples = []
        for i in range(280):
            val = int(2048 + 1000 * math.sin(2 * math.pi * i / 50))
            samples.append(max(0, min(4095, val)))
        scope.capture(samples)
        self.assertTrue(scope.triggered)
        self.assertGreater(scope.sample_count, 0)

    def test_trigger_rising(self):
        """上升沿触发"""
        scope = TFTScope()
        scope.init()
        scope.set_trigger(TRIG_RISING, 2048, 0)
        # 锯齿波：从低到高
        samples = list(range(1000, 3000, 20))
        scope.capture(samples)
        self.assertTrue(scope.triggered)
        self.assertGreater(scope.trig_pos, 0)

    def test_trigger_falling(self):
        """下降沿触发"""
        scope = TFTScope()
        scope.init()
        scope.set_trigger(TRIG_FALLING, 2048, 0)
        # 从高到低
        samples = list(range(3000, 1000, -20))
        scope.capture(samples)
        self.assertTrue(scope.triggered)

    def test_trigger_none(self):
        """自动触发"""
        scope = TFTScope()
        scope.init()
        scope.set_trigger(TRIG_NONE, 2048, 0)
        samples = [2048] * 100  # 直流
        scope.capture(samples)
        self.assertTrue(scope.triggered)
        self.assertEqual(scope.trig_pos, 0)

    def test_no_trigger(self):
        """无触发条件满足"""
        scope = TFTScope()
        scope.init()
        scope.set_trigger(TRIG_RISING, 4000, 0)  # 高触发电平
        # 低幅值信号
        samples = [2000 + i % 10 for i in range(100)]
        scope.capture(samples)
        self.assertFalse(scope.triggered)

    def test_dual_channel(self):
        """双通道捕获"""
        scope = TFTScope()
        scope.init()
        ch0 = [2048 + 500 * math.sin(2 * math.pi * i / 50) for i in range(200)]
        ch1 = [2048 + 300 * math.cos(2 * math.pi * i / 50) for i in range(200)]
        scope.capture([int(v) for v in ch0], [int(v) for v in ch1])
        self.assertGreater(len(scope.channels[0].captured), 0)
        self.assertGreater(len(scope.channels[1].captured), 0)


class TestScopeMeasure(unittest.TestCase):
    """波形测量测试"""

    def test_measure_vpp_sine(self):
        """正弦波峰峰值"""
        scope = TFTScope()
        scope.init()
        # 幅度1000的正弦波 → 峰峰值=2000步 → 2000*3300/4096 ≈ 1611mV
        samples = [int(2048 + 1000 * math.sin(2 * math.pi * i / 50)) for i in range(200)]
        scope.capture(samples)
        vpp = scope.measure_vpp(0)
        # 应约为2000 * 3300/4096 ≈ 1611mV
        self.assertGreater(vpp, 1500)
        self.assertLess(vpp, 1700)

    def test_measure_frequency(self):
        """频率测量"""
        scope = TFTScope()
        scope.init()
        # 50个采样点为一个周期，280个采样点，timebase_idx=4 (20ms/div)
        # samples_per_div = 280/10 = 28, ms_per_sample = 20/28
        # period = 50 * 20/28 ≈ 35.7ms → freq ≈ 28Hz
        samples = [int(2048 + 1000 * math.sin(2 * math.pi * i / 50)) for i in range(280)]
        scope.capture(samples)
        freq = scope.measure_frequency(0)
        self.assertGreater(freq, 0)


class TestScopeStorage(unittest.TestCase):
    """波形存储测试"""

    def test_save_load(self):
        """保存和加载波形"""
        scope = TFTScope()
        scope.init()
        samples = [2048 + 500 for _ in range(100)]
        scope.capture(samples)
        # 保存到槽0
        self.assertTrue(scope.save_waveform(0))
        # 清除当前数据
        scope.channels[0].captured = []
        # 加载
        self.assertTrue(scope.load_waveform(0))
        self.assertGreater(len(scope.channels[0].captured), 0)

    def test_invalid_slot(self):
        """无效槽位"""
        scope = TFTScope()
        scope.init()
        self.assertFalse(scope.save_waveform(5))
        self.assertFalse(scope.load_waveform(5))

    def test_load_empty_slot(self):
        """加载空槽位"""
        scope = TFTScope()
        scope.init()
        self.assertFalse(scope.load_waveform(1))

    def test_multiple_slots(self):
        """多个存储槽"""
        scope = TFTScope()
        scope.init()
        for slot in range(3):
            samples = [2048 + slot * 100 for _ in range(50)]
            scope.capture(samples)
            self.assertTrue(scope.save_waveform(slot))
        # 验证每个槽的数据不同
        for slot in range(3):
            self.assertTrue(scope.load_waveform(slot))
            avg = sum(scope.channels[0].captured) / len(scope.channels[0].captured)
            self.assertAlmostEqual(avg, 2048 + slot * 100, delta=1)


class TestScopeTimebase(unittest.TestCase):
    """时基调节测试"""

    def test_change_timebase(self):
        """改变时基档位"""
        scope = TFTScope()
        scope.timebase_idx = 0
        self.assertEqual(scope.get_timebase_ms_per_div(), 1)
        scope.timebase_idx = len(TIMEBASE_LIST) - 1
        self.assertEqual(scope.get_timebase_ms_per_div(), 500)

    def test_all_timebases(self):
        """遍历所有时基档位"""
        scope = TFTScope()
        for i in range(len(TIMEBASE_LIST)):
            scope.timebase_idx = i
            self.assertEqual(scope.get_timebase_ms_per_div(), TIMEBASE_LIST[i])


class TestScopeRingBuffer(unittest.TestCase):
    """环形缓冲区测试"""

    def test_write_read(self):
        """写入和读取"""
        buf = RingBuffer(256)
        for i in range(100):
            buf.put_byte(i)
        self.assertEqual(buf.used(), 100)
        data = buf.read(10)
        self.assertEqual(len(data), 10)
        self.assertEqual(data[0], 0)

    def test_overflow(self):
        """缓冲区溢出"""
        buf = RingBuffer(16)
        for i in range(20):
            buf.put_byte(i)
        # 应该只保留最后16个
        self.assertTrue(buf.is_full())

    def test_reset(self):
        """重置"""
        buf = RingBuffer(64)
        for i in range(30):
            buf.put_byte(i)
        buf.reset()
        self.assertTrue(buf.is_empty())
        self.assertEqual(buf.used(), 0)


if __name__ == '__main__':
    unittest.main()
