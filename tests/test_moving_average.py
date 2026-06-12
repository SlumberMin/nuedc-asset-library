#!/usr/bin/env python3
"""
滑动平均滤波器单元测试
覆盖: 简单滑动平均(SMA)、加权滑动平均(WMA)、指数滑动平均(EMA)
测试: 初始化、正常滤波、边界条件、异常输入、性能基准
"""

import sys
import os
import unittest
import time
import random

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class SimpleMovingAverage:
    """简单滑动平均(SMA)"""

    def __init__(self, window_size=10):
        if window_size < 1:
            raise ValueError("窗口大小必须>=1")
        self.window_size = window_size
        self.buffer = []
        self.sum = 0.0

    def update(self, value):
        """输入新值，返回当前平均"""
        self.buffer.append(value)
        self.sum += value
        if len(self.buffer) > self.window_size:
            self.sum -= self.buffer.pop(0)
        return self.sum / len(self.buffer)

    def get(self):
        """获取当前平均值"""
        if not self.buffer:
            return 0.0
        return self.sum / len(self.buffer)

    def is_ready(self):
        """缓冲区是否已满"""
        return len(self.buffer) >= self.window_size

    def reset(self):
        """重置"""
        self.buffer.clear()
        self.sum = 0.0


class WeightedMovingAverage:
    """加权滑动平均(WMA)"""

    def __init__(self, window_size=10):
        if window_size < 1:
            raise ValueError("窗口大小必须>=1")
        self.window_size = window_size
        self.buffer = []
        # 权重: 最新的数据权重最大
        self.weights = list(range(1, window_size + 1))
        self.weight_sum = sum(self.weights)

    def update(self, value):
        """输入新值，返回加权平均"""
        self.buffer.append(value)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        # 计算加权平均
        n = len(self.buffer)
        w = self.weights[-n:]  # 取最后n个权重
        w_sum = sum(w)
        result = sum(v * wi for v, wi in zip(self.buffer, w)) / w_sum
        return result

    def get(self):
        """获取当前值"""
        if not self.buffer:
            return 0.0
        n = len(self.buffer)
        w = self.weights[-n:]
        w_sum = sum(w)
        return sum(v * wi for v, wi in zip(self.buffer, w)) / w_sum

    def reset(self):
        """重置"""
        self.buffer.clear()


class ExponentialMovingAverage:
    """指数滑动平均(EMA)"""

    def __init__(self, alpha=0.1):
        if not (0.0 < alpha <= 1.0):
            raise ValueError("alpha必须在(0, 1]之间")
        self.alpha = alpha
        self.value = None
        self.initialized = False

    def update(self, value):
        """输入新值，返回EMA"""
        if not self.initialized:
            self.value = value
            self.initialized = True
        else:
            self.value = self.alpha * value + (1 - self.alpha) * self.value
        return self.value

    def get(self):
        """获取当前值"""
        return self.value if self.value is not None else 0.0

    def reset(self):
        """重置"""
        self.value = None
        self.initialized = False


class TestSimpleMovingAverage(unittest.TestCase):
    """简单滑动平均测试"""

    def test_initialization(self):
        """测试初始化"""
        sma = SimpleMovingAverage(5)
        self.assertEqual(sma.window_size, 5)

    def test_invalid_window_size(self):
        """测试无效窗口大小"""
        with self.assertRaises(ValueError):
            SimpleMovingAverage(0)
        with self.assertRaises(ValueError):
            SimpleMovingAverage(-1)

    def test_single_value(self):
        """测试单值"""
        sma = SimpleMovingAverage(5)
        result = sma.update(10.0)
        self.assertAlmostEqual(result, 10.0)

    def test_constant_input(self):
        """测试常数输入"""
        sma = SimpleMovingAverage(5)
        for _ in range(10):
            result = sma.update(5.0)
        self.assertAlmostEqual(result, 5.0)

    def test_window_filling(self):
        """测试窗口填充"""
        sma = SimpleMovingAverage(3)
        sma.update(1.0)
        self.assertFalse(sma.is_ready())
        sma.update(2.0)
        self.assertFalse(sma.is_ready())
        sma.update(3.0)
        self.assertTrue(sma.is_ready())

    def test_average_calculation(self):
        """测试平均计算"""
        sma = SimpleMovingAverage(3)
        sma.update(1.0)
        sma.update(2.0)
        result = sma.update(3.0)
        self.assertAlmostEqual(result, 2.0)

    def test_sliding_window(self):
        """测试滑动窗口"""
        sma = SimpleMovingAverage(3)
        sma.update(1.0)
        sma.update(2.0)
        sma.update(3.0)  # avg=2.0
        result = sma.update(4.0)  # window=[2,3,4], avg=3.0
        self.assertAlmostEqual(result, 3.0)

    def test_noise_smoothing(self):
        """测试噪声平滑"""
        sma = SimpleMovingAverage(10)
        random.seed(42)
        true_value = 5.0
        for _ in range(200):
            sma.update(true_value + random.gauss(0, 2.0))
        self.assertAlmostEqual(sma.get(), true_value, delta=1.5)

    def test_reset(self):
        """测试重置"""
        sma = SimpleMovingAverage(5)
        sma.update(1.0)
        sma.update(2.0)
        sma.reset()
        self.assertEqual(sma.get(), 0.0)

    def test_get_empty(self):
        """测试空缓冲区get"""
        sma = SimpleMovingAverage(5)
        self.assertEqual(sma.get(), 0.0)

    def test_performance_benchmark(self):
        """性能基准: SMA性能"""
        sma = SimpleMovingAverage(100)
        iterations = 100000
        random.seed(42)
        start = time.perf_counter()
        for i in range(iterations):
            sma.update(random.random())
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 10.0, "SMA应<10μs")


class TestWeightedMovingAverage(unittest.TestCase):
    """加权滑动平均测试"""

    def test_initialization(self):
        """测试初始化"""
        wma = WeightedMovingAverage(5)
        self.assertEqual(wma.window_size, 5)

    def test_recent_value_weighted_more(self):
        """测试近期值权重更大"""
        wma = WeightedMovingAverage(3)
        wma.update(1.0)
        wma.update(1.0)
        result1 = wma.update(10.0)  # [1,1,10] 加权
        wma2 = WeightedMovingAverage(3)
        wma2.update(10.0)
        wma2.update(1.0)
        result2 = wma2.update(1.0)  # [10,1,1] 加权
        self.assertGreater(result1, result2)

    def test_constant_input(self):
        """测试常数输入"""
        wma = WeightedMovingAverage(5)
        for _ in range(10):
            result = wma.update(7.0)
        self.assertAlmostEqual(result, 7.0)

    def test_reset(self):
        """测试重置"""
        wma = WeightedMovingAverage(5)
        wma.update(10.0)
        wma.reset()
        self.assertEqual(wma.get(), 0.0)

    def test_performance_benchmark(self):
        """性能基准: WMA性能"""
        wma = WeightedMovingAverage(50)
        iterations = 50000
        start = time.perf_counter()
        for i in range(iterations):
            wma.update(float(i % 100))
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 20.0, "WMA应<20μs")


class TestExponentialMovingAverage(unittest.TestCase):
    """指数滑动平均测试"""

    def test_initialization(self):
        """测试初始化"""
        ema = ExponentialMovingAverage(0.2)
        self.assertEqual(ema.alpha, 0.2)

    def test_invalid_alpha(self):
        """测试无效alpha"""
        with self.assertRaises(ValueError):
            ExponentialMovingAverage(0.0)
        with self.assertRaises(ValueError):
            ExponentialMovingAverage(1.1)

    def test_first_value(self):
        """测试首次值"""
        ema = ExponentialMovingAverage(0.5)
        result = ema.update(10.0)
        self.assertAlmostEqual(result, 10.0)

    def test_smoothing(self):
        """测试平滑效果"""
        ema = ExponentialMovingAverage(0.1)
        ema.update(0.0)
        result = ema.update(10.0)
        # alpha=0.1: 0.1*10 + 0.9*0 = 1.0
        self.assertAlmostEqual(result, 1.0)

    def test_alpha_one_no_smoothing(self):
        """测试alpha=1无平滑"""
        ema = ExponentialMovingAverage(1.0)
        ema.update(0.0)
        result = ema.update(10.0)
        self.assertAlmostEqual(result, 10.0)

    def test_convergence_to_constant(self):
        """测试收敛到常数"""
        ema = ExponentialMovingAverage(0.1)
        for _ in range(200):
            ema.update(5.0)
        self.assertAlmostEqual(ema.get(), 5.0, delta=0.01)

    def test_reset(self):
        """测试重置"""
        ema = ExponentialMovingAverage(0.1)
        ema.update(10.0)
        ema.reset()
        self.assertFalse(ema.initialized)
        self.assertEqual(ema.get(), 0.0)

    def test_response_speed_comparison(self):
        """测试不同alpha响应速度"""
        ema_slow = ExponentialMovingAverage(0.05)
        ema_fast = ExponentialMovingAverage(0.5)
        ema_slow.update(0.0)
        ema_fast.update(0.0)
        for _ in range(10):
            ema_slow.update(10.0)
            ema_fast.update(10.0)
        self.assertGreater(ema_fast.get(), ema_slow.get())

    def test_performance_benchmark(self):
        """性能基准: EMA性能"""
        ema = ExponentialMovingAverage(0.1)
        iterations = 100000
        start = time.perf_counter()
        for i in range(iterations):
            ema.update(float(i % 100))
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 2.0, "EMA应<2μs")


if __name__ == '__main__':
    unittest.main()
