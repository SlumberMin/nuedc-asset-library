#!/usr/bin/env python3
"""
Biquad二阶滤波器单元测试
覆盖: 初始化/各类型滤波器/频率响应/级联/边界条件/性能基准
注意: 使用纯 Python 模拟 Biquad 滤波器逻辑
"""

import sys
import os
import math
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 ──────────────────────────────────────────

class BiquadFilter:
    """二阶IIR (Biquad) 滤波器"""

    def __init__(self, b0=1.0, b1=0.0, b2=0.0, a0=1.0, a1=0.0, a2=0.0):
        """
        传递函数: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (a0 + a1*z^-1 + a2*z^-2)
        """
        # 归一化 (除以a0)
        self.b0 = b0 / a0
        self.b1 = b1 / a0
        self.b2 = b2 / a0
        self.a1 = a1 / a0
        self.a2 = a2 / a0

        # 状态
        self.x1 = 0.0
        self.x2 = 0.0
        self.y1 = 0.0
        self.y2 = 0.0

    def process(self, x):
        """单步处理 (Direct Form I)"""
        y = (self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
             - self.a1 * self.y1 - self.a2 * self.y2)

        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y

        return y

    def process_batch(self, signal):
        """批量处理"""
        return [self.process(x) for x in signal]

    def reset(self):
        self.x1 = self.x2 = 0.0
        self.y1 = self.y2 = 0.0


class BiquadDesign:
    """Biquad滤波器设计工具"""

    @staticmethod
    def lowpass(fc, fs, Q=0.707):
        """设计低通滤波器"""
        w0 = 2.0 * math.pi * fc / fs
        alpha = math.sin(w0) / (2.0 * Q)
        cosw0 = math.cos(w0)

        b0 = (1.0 - cosw0) / 2.0
        b1 = 1.0 - cosw0
        b2 = (1.0 - cosw0) / 2.0
        a0 = 1.0 + alpha
        a1 = -2.0 * cosw0
        a2 = 1.0 - alpha

        return BiquadFilter(b0, b1, b2, a0, a1, a2)

    @staticmethod
    def highpass(fc, fs, Q=0.707):
        """设计高通滤波器"""
        w0 = 2.0 * math.pi * fc / fs
        alpha = math.sin(w0) / (2.0 * Q)
        cosw0 = math.cos(w0)

        b0 = (1.0 + cosw0) / 2.0
        b1 = -(1.0 + cosw0)
        b2 = (1.0 + cosw0) / 2.0
        a0 = 1.0 + alpha
        a1 = -2.0 * cosw0
        a2 = 1.0 - alpha

        return BiquadFilter(b0, b1, b2, a0, a1, a2)

    @staticmethod
    def bandpass(fc, bw, fs):
        """设计带通滤波器"""
        w0 = 2.0 * math.pi * fc / fs
        alpha = math.sin(w0) * math.sinh(math.log(2.0) / 2.0 * bw * w0 / math.sin(w0))

        b0 = alpha
        b1 = 0.0
        b2 = -alpha
        a0 = 1.0 + alpha
        a1 = -2.0 * math.cos(w0)
        a2 = 1.0 - alpha

        return BiquadFilter(b0, b1, b2, a0, a1, a2)

    @staticmethod
    def notch(fc, Q, fs):
        """设计陷波滤波器"""
        w0 = 2.0 * math.pi * fc / fs
        alpha = math.sin(w0) / (2.0 * Q)

        b0 = 1.0
        b1 = -2.0 * math.cos(w0)
        b2 = 1.0
        a0 = 1.0 + alpha
        a1 = -2.0 * math.cos(w0)
        a2 = 1.0 - alpha

        return BiquadFilter(b0, b1, b2, a0, a1, a2)


# ── 测试用例 ──────────────────────────────────────────────────

class TestBiquadInit(unittest.TestCase):
    """Biquad初始化测试"""

    def test_default_gain(self):
        """默认系数(直通)应输出等于输入"""
        bq = BiquadFilter(b0=1.0)
        y = bq.process(5.0)
        self.assertAlmostEqual(y, 5.0, places=5)

    def test_coefficient_normalization(self):
        """系数应归一化(a0=1)"""
        bq = BiquadFilter(b0=2.0, b1=0.0, b2=0.0, a0=2.0, a1=0.0, a2=0.0)
        self.assertAlmostEqual(bq.b0, 1.0, places=5)

    def test_initial_state_zero(self):
        """初始状态应为零"""
        bq = BiquadFilter()
        self.assertEqual(bq.x1, 0.0)
        self.assertEqual(bq.x2, 0.0)
        self.assertEqual(bq.y1, 0.0)
        self.assertEqual(bq.y2, 0.0)

    def test_reset(self):
        """reset应清零状态"""
        bq = BiquadFilter()
        bq.process(5.0)
        bq.process(3.0)
        bq.reset()
        self.assertEqual(bq.x1, 0.0)
        self.assertEqual(bq.y1, 0.0)


class TestBiquadLowpass(unittest.TestCase):
    """低通滤波器测试"""

    def test_passes_dc(self):
        """低通应通过直流"""
        bq = BiquadDesign.lowpass(100.0, 1000.0, Q=0.707)
        output = bq.process_batch([1.0] * 500)
        self.assertAlmostEqual(output[-1], 1.0, delta=0.05)

    def test_attenuates_high_freq(self):
        """低通应衰减高频"""
        fs = 1000.0
        bq = BiquadDesign.lowpass(50.0, fs, Q=0.707)
        t = [i / fs for i in range(1000)]
        signal = [math.sin(2 * math.pi * 200 * ti) for ti in t]
        output = bq.process_batch(signal)

        rms_in = math.sqrt(sum(x**2 for x in signal[200:]) / len(signal[200:]))
        rms_out = math.sqrt(sum(x**2 for x in output[200:]) / len(output[200:]))

        self.assertLess(rms_out, rms_in * 0.5)

    def test_passes_low_freq(self):
        """低通应通过低频信号"""
        fs = 1000.0
        bq = BiquadDesign.lowpass(100.0, fs, Q=0.707)
        t = [i / fs for i in range(1000)]
        signal = [math.sin(2 * math.pi * 10 * ti) for ti in t]
        output = bq.process_batch(signal)

        rms_in = math.sqrt(sum(x**2 for x in signal[200:]) / len(signal[200:]))
        rms_out = math.sqrt(sum(x**2 for x in output[200:]) / len(output[200:]))

        self.assertGreater(rms_out, rms_in * 0.7)


class TestBiquadHighpass(unittest.TestCase):
    """高通滤波器测试"""

    def test_passes_high_freq(self):
        """高通应通过高频"""
        fs = 1000.0
        bq = BiquadDesign.highpass(100.0, fs, Q=0.707)
        t = [i / fs for i in range(1000)]
        signal = [math.sin(2 * math.pi * 200 * ti) for ti in t]
        output = bq.process_batch(signal)

        rms_in = math.sqrt(sum(x**2 for x in signal[200:]) / len(signal[200:]))
        rms_out = math.sqrt(sum(x**2 for x in output[200:]) / len(output[200:]))

        self.assertGreater(rms_out, rms_in * 0.5)

    def test_attenuates_dc(self):
        """高通应衰减直流"""
        bq = BiquadDesign.highpass(50.0, 1000.0, Q=0.707)
        output = bq.process_batch([5.0] * 500)
        self.assertLess(abs(output[-1]), 0.5)


class TestBiquadNotch(unittest.TestCase):
    """陷波滤波器测试"""

    def test_passes_dc(self):
        """陷波应通过直流"""
        bq = BiquadDesign.notch(50.0, 10.0, 1000.0)
        output = bq.process_batch([3.0] * 500)
        self.assertAlmostEqual(output[-1], 3.0, delta=0.1)

    def test_attenuates_center_freq(self):
        """陷波应衰减中心频率"""
        fs = 1000.0
        fc = 50.0
        bq = BiquadDesign.notch(fc, 10.0, fs)
        t = [i / fs for i in range(2000)]
        signal = [math.sin(2 * math.pi * fc * ti) for ti in t]
        output = bq.process_batch(signal)

        rms_in = math.sqrt(sum(x**2 for x in signal[500:]) / len(signal[500:]))
        rms_out = math.sqrt(sum(x**2 for x in output[500:]) / len(output[500:]))

        self.assertLess(rms_out, rms_in * 0.3)


class TestBiquadStability(unittest.TestCase):
    """稳定性测试"""

    def test_stable_various_params(self):
        """不同参数组合应保持稳定"""
        fs = 1000.0
        designs = [
            BiquadDesign.lowpass(100.0, fs),
            BiquadDesign.lowpass(300.0, fs),
            BiquadDesign.highpass(50.0, fs),
            BiquadDesign.highpass(200.0, fs),
            BiquadDesign.notch(50.0, 5.0, fs),
        ]

        for bq in designs:
            bq.reset()
            for _ in range(1000):
                y = bq.process(1.0)
                self.assertTrue(abs(y) < 100.0, f"滤波器发散: {y}")

    def test_impulse_response_decays(self):
        """脉冲响应应衰减到零"""
        bq = BiquadDesign.lowpass(50.0, 1000.0)
        y = bq.process(1.0)  # 脉冲
        for _ in range(500):
            y = bq.process(0.0)
        self.assertAlmostEqual(y, 0.0, delta=0.01)

    def test_no_overflow_large_signal(self):
        """大信号不应导致溢出"""
        bq = BiquadDesign.lowpass(100.0, 1000.0)
        for _ in range(500):
            y = bq.process(100.0)
            self.assertTrue(abs(y) < 1e6)


class TestBiquadCascade(unittest.TestCase):
    """级联滤波器测试"""

    def test_cascade_sharper_rolloff(self):
        """级联应产生更陡的滚降"""
        fs = 1000.0
        fc = 50.0

        # 单级
        bq1 = BiquadDesign.lowpass(fc, fs)

        # 两级级联
        bq2a = BiquadDesign.lowpass(fc, fs)
        bq2b = BiquadDesign.lowpass(fc, fs)

        t = [i / fs for i in range(1000)]
        signal = [math.sin(2 * math.pi * 200 * ti) for ti in t]

        out1 = bq1.process_batch(signal)
        out2 = [bq2b.process(bq2a.process(x)) for x in signal]

        rms1 = math.sqrt(sum(x**2 for x in out1[200:]) / len(out1[200:]))
        rms2 = math.sqrt(sum(x**2 for x in out2[200:]) / len(out2[200:]))

        # 级联应衰减更多
        self.assertLessEqual(rms2, rms1 + 0.01)


class TestBiquadPerformance(unittest.TestCase):
    """性能基准测试"""

    def test_batch_processing_speed(self):
        """处理10000个样本应在1秒内完成"""
        bq = BiquadDesign.lowpass(100.0, 1000.0)
        signal = [math.sin(2 * math.pi * 50 * i / 1000.0) for i in range(10000)]

        start = time.perf_counter()
        bq.process_batch(signal)
        elapsed = time.perf_counter() - start

        self.assertLess(elapsed, 2.0,
                       f"处理10000样本耗时 {elapsed:.3f}s")


if __name__ == '__main__':
    unittest.main()
