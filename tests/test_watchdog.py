#!/usr/bin/env python3
"""
看门狗单元测试
覆盖: 初始化、喂狗、超时检测、重置、窗口看门狗、性能基准
"""

import sys
import os
import unittest
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class Watchdog:
    """看门狗简化实现"""

    def __init__(self, timeout_ms=1000, window_min_ms=0, window_max_ms=0):
        self.timeout_ms = timeout_ms
        self.window_min_ms = window_min_ms
        self.window_max_ms = window_max_ms if window_max_ms > 0 else timeout_ms
        self.last_feed_time = 0
        self.is_triggered = False
        self.feed_count = 0
        self.enabled = True
        self._start_time = 0

    def start(self, current_time_ms=None):
        """启动看门狗"""
        if current_time_ms is None:
            current_time_ms = int(time.time() * 1000)
        self._start_time = current_time_ms
        self.last_feed_time = current_time_ms
        self.is_triggered = False
        self.enabled = True

    def feed(self, current_time_ms=None):
        """喂狗"""
        if not self.enabled:
            return False
        if current_time_ms is None:
            current_time_ms = int(time.time() * 1000)
        elapsed = current_time_ms - self.last_feed_time
        # 窗口看门狗检查
        if self.window_min_ms > 0 and elapsed < self.window_min_ms:
            self.is_triggered = True
            return False  # 喂狗太频繁
        self.last_feed_time = current_time_ms
        self.feed_count += 1
        self.is_triggered = False
        return True

    def check(self, current_time_ms=None):
        """检查是否超时"""
        if not self.enabled:
            return True  # 未启用视为正常
        if current_time_ms is None:
            current_time_ms = int(time.time() * 1000)
        elapsed = current_time_ms - self.last_feed_time
        if elapsed > self.timeout_ms:
            self.is_triggered = True
            return False  # 超时
        return True  # 正常

    def reset(self, current_time_ms=None):
        """重置看门狗"""
        if current_time_ms is None:
            current_time_ms = int(time.time() * 1000)
        self.last_feed_time = current_time_ms
        self.is_triggered = False
        self.feed_count = 0
        self.enabled = True

    def stop(self):
        """停止看门狗"""
        self.enabled = False


class TestWatchdog(unittest.TestCase):
    """看门狗基础测试"""

    def test_initialization(self):
        """测试初始化参数"""
        wd = Watchdog(timeout_ms=500)
        self.assertEqual(wd.timeout_ms, 500)
        self.assertFalse(wd.is_triggered)

    def test_default_parameters(self):
        """测试默认参数"""
        wd = Watchdog()
        self.assertEqual(wd.timeout_ms, 1000)
        self.assertEqual(wd.window_min_ms, 0)

    def test_start(self):
        """测试启动"""
        wd = Watchdog(timeout_ms=1000)
        wd.start(current_time_ms=0)
        self.assertTrue(wd.enabled)
        self.assertFalse(wd.is_triggered)

    def test_feed_normal(self):
        """测试正常喂狗"""
        wd = Watchdog(timeout_ms=1000)
        wd.start(current_time_ms=0)
        result = wd.feed(current_time_ms=500)
        self.assertTrue(result)
        self.assertEqual(wd.feed_count, 1)

    def test_no_timeout(self):
        """测试未超时"""
        wd = Watchdog(timeout_ms=1000)
        wd.start(current_time_ms=0)
        self.assertTrue(wd.check(current_time_ms=500))

    def test_timeout(self):
        """测试超时"""
        wd = Watchdog(timeout_ms=1000)
        wd.start(current_time_ms=0)
        result = wd.check(current_time_ms=1500)
        self.assertFalse(result)
        self.assertTrue(wd.is_triggered)

    def test_feed_resets_timeout(self):
        """测试喂狗重置超时"""
        wd = Watchdog(timeout_ms=1000)
        wd.start(current_time_ms=0)
        wd.feed(current_time_ms=800)
        # 1500ms时未超时(800+1000=1800)
        self.assertTrue(wd.check(current_time_ms=1500))
        # 1900ms时超时
        self.assertFalse(wd.check(current_time_ms=1900))

    def test_window_watchdog_too_early(self):
        """测试窗口看门狗-喂狗太早"""
        wd = Watchdog(timeout_ms=1000, window_min_ms=200)
        wd.start(current_time_ms=0)
        # 100ms喂狗太早
        result = wd.feed(current_time_ms=100)
        self.assertFalse(result)
        self.assertTrue(wd.is_triggered)

    def test_window_watchdog_normal(self):
        """测试窗口看门狗-正常喂狗"""
        wd = Watchdog(timeout_ms=1000, window_min_ms=200)
        wd.start(current_time_ms=0)
        result = wd.feed(current_time_ms=300)
        self.assertTrue(result)

    def test_reset(self):
        """测试重置"""
        wd = Watchdog(timeout_ms=1000)
        wd.start(current_time_ms=0)
        wd.check(current_time_ms=1500)  # 触发超时
        wd.reset(current_time_ms=2000)
        self.assertFalse(wd.is_triggered)
        self.assertEqual(wd.feed_count, 0)
        self.assertTrue(wd.check(current_time_ms=2500))

    def test_stop(self):
        """测试停止"""
        wd = Watchdog(timeout_ms=1000)
        wd.start(current_time_ms=0)
        wd.stop()
        self.assertFalse(wd.enabled)
        # 停止后检查应返回True
        self.assertTrue(wd.check(current_time_ms=99999))

    def test_disabled_feed(self):
        """测试禁用时喂狗"""
        wd = Watchdog(timeout_ms=1000)
        wd.start(current_time_ms=0)
        wd.stop()
        result = wd.feed(current_time_ms=500)
        self.assertFalse(result)

    def test_multiple_feeds(self):
        """测试多次喂狗"""
        wd = Watchdog(timeout_ms=1000)
        wd.start(current_time_ms=0)
        for i in range(1, 11):
            wd.feed(current_time_ms=i * 100)
        self.assertEqual(wd.feed_count, 10)

    def test_boundary_exact_timeout(self):
        """测试精确超时边界"""
        wd = Watchdog(timeout_ms=1000)
        wd.start(current_time_ms=0)
        # 恰好超时
        self.assertFalse(wd.check(current_time_ms=1001))
        # 刚好未超时
        self.assertTrue(wd.check(current_time_ms=999))

    def test_zero_timeout(self):
        """测试零超时"""
        wd = Watchdog(timeout_ms=0)
        wd.start(current_time_ms=0)
        self.assertFalse(wd.check(current_time_ms=1))

    def test_performance_benchmark(self):
        """性能基准: 看门狗检查性能"""
        wd = Watchdog(timeout_ms=1000)
        wd.start(current_time_ms=0)
        iterations = 100000
        start = time.perf_counter()
        for i in range(iterations):
            wd.check(current_time_ms=i)
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 10.0, "看门狗检查应<10μs")


if __name__ == '__main__':
    unittest.main()
