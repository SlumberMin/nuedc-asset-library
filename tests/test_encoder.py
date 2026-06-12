#!/usr/bin/env python3
"""
编码器驱动单元测试
覆盖: EncoderReader 初始化、计数、方向、速度计算、角度、距离、清理
注意: 使用 Mock 模拟 GPIO 中断和时间
"""

import sys
import os
import time
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from OrangePi5.utils.gpio_utils import GPIOManager
from OrangePi5.control.encoder_reader import EncoderReader


class MockGPIOForEncoder:
    """模拟GPIO管理器(编码器用)"""
    IN = 1
    PUD_UP = 1
    PUD_DOWN = 0

    def __init__(self):
        self.pins = {}
        self.interrupts = {}

    def setup(self, pin, direction, pull=None):
        self.pins[pin] = {'dir': direction, 'pull': pull, 'state': 0}

    def input(self, pin):
        return self.pins.get(pin, {}).get('state', 0)

    def add_interrupt(self, pin, callback, edge='both'):
        self.interrupts[pin] = callback

    def remove_interrupt(self, pin):
        if pin in self.interrupts:
            del self.interrupts[pin]


def _make_encoder(gpio, pin_a=17, pin_b=None, ppr=360, pull_up=True):
    """绕过__init__直接构造EncoderReader"""
    import threading
    from collections import deque
    enc = EncoderReader.__new__(EncoderReader)
    enc.gpio = gpio
    enc.pin_a = pin_a
    enc.pin_b = pin_b
    enc.ppr = ppr
    enc.quadrature = pin_b is not None
    enc._count = 0
    enc._lock = threading.Lock()
    enc._timestamps = deque(maxlen=1000)
    enc._prev_time = time.monotonic()
    return enc


class TestEncoderInit(unittest.TestCase):
    """编码器初始化测试"""

    def test_single_phase_mode(self):
        """单相模式: pin_b=None"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, pin_a=17, pin_b=None, ppr=360)
        self.assertFalse(enc.quadrature)
        self.assertEqual(enc.ppr, 360)

    def test_quadrature_mode(self):
        """正交模式: pin_b有值"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, pin_a=17, pin_b=27, ppr=360)
        self.assertTrue(enc.quadrature)

    def test_custom_ppr(self):
        """自定义PPR"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, ppr=1024)
        self.assertEqual(enc.ppr, 1024)

    def test_repr(self):
        """__repr__应包含关键信息"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, pin_a=17, pin_b=27, ppr=360)
        s = repr(enc)
        self.assertIn('EncoderReader', s)
        self.assertIn('quadrature', s)


class TestEncoderCount(unittest.TestCase):
    """脉冲计数测试"""

    def test_initial_count_zero(self):
        """初始计数应为0"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio)
        self.assertEqual(enc.get_count(), 0)

    def test_single_phase_increment(self):
        """单相模式: 每次中断计数+1"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, pin_b=None)
        # 模拟中断
        gpio.pins[17] = {'state': 1}
        enc._on_a_edge(17)
        self.assertEqual(enc.get_count(), 1)
        enc._on_a_edge(17)
        self.assertEqual(enc.get_count(), 2)

    def test_quadrature_forward(self):
        """正交模式正转: A==B → count+1"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, pin_a=17, pin_b=27)
        gpio.pins[17] = {'state': 1}
        gpio.pins[27] = {'state': 1}  # A==B
        enc._on_a_edge(17)
        self.assertEqual(enc.get_count(), 1)

    def test_quadrature_reverse(self):
        """正交模式反转: A!=B → count-1"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, pin_a=17, pin_b=27)
        gpio.pins[17] = {'state': 1}
        gpio.pins[27] = {'state': 0}  # A!=B
        enc._on_a_edge(17)
        self.assertEqual(enc.get_count(), -1)

    def test_reset_count(self):
        """重置应清零计数"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio)
        gpio.pins[17] = {'state': 1}
        enc._on_a_edge(17)
        enc._on_a_edge(17)
        enc.reset_count()
        self.assertEqual(enc.get_count(), 0)

    def test_thread_safety(self):
        """多线程并发应安全"""
        import threading
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio)
        gpio.pins[17] = {'state': 1}

        def _pulse(n):
            for _ in range(n):
                enc._on_a_edge(17)

        threads = [threading.Thread(target=_pulse, args=(100,)) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(enc.get_count(), 400)


class TestEncoderAngle(unittest.TestCase):
    """角度计算测试"""

    def test_zero_count_zero_angle(self):
        """零计数应返回零角度"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, ppr=360)
        self.assertAlmostEqual(enc.get_angle(), 0.0)

    def test_one_revolution(self):
        """一圈应返回360°"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, ppr=360)
        for _ in range(360):
            enc._count += 1
        self.assertAlmostEqual(enc.get_angle(), 360.0)

    def test_half_revolution(self):
        """半圈应返回180°"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, ppr=360)
        for _ in range(180):
            enc._count += 1
        self.assertAlmostEqual(enc.get_angle(), 180.0)


class TestEncoderDistance(unittest.TestCase):
    """距离计算测试"""

    def test_zero_distance(self):
        """零计数应返回零距离"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, ppr=360)
        self.assertAlmostEqual(enc.get_distance(), 0.0)

    def test_one_wheel_revolution(self):
        """一圈距离 = π * 直径"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, ppr=360)
        for _ in range(360):
            enc._count += 1
        expected = 3.14159 * 0.065
        self.assertAlmostEqual(enc.get_distance(0.065), expected, places=4)


class TestEncoderSpeed(unittest.TestCase):
    """速度计算测试"""

    def test_zero_speed_when_no_pulses(self):
        """无脉冲时速度应为0"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, ppr=360)
        rpm = enc.get_speed(dt=0.1)
        self.assertAlmostEqual(rpm, 0.0)

    def test_speed_with_recent_pulses(self):
        """有脉冲时应计算出非零速度"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, ppr=360)
        # 注入时间戳
        now = time.monotonic()
        for i in range(36):
            enc._timestamps.append(now - 0.05 + i * 0.001)
        enc._count = 36
        rpm = enc.get_speed(dt=0.1)
        self.assertNotAlmostEqual(rpm, 0.0, places=1)

    def test_speed_zero_dt(self):
        """dt<=0应返回0"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, ppr=360)
        self.assertAlmostEqual(enc.get_speed(dt=0), 0.0)


class TestEncoderCleanup(unittest.TestCase):
    """清理测试"""

    def test_cleanup_removes_interrupt(self):
        """cleanup应移除中断回调"""
        gpio = MockGPIOForEncoder()
        enc = _make_encoder(gpio, pin_a=17)
        gpio.interrupts[17] = enc._on_a_edge
        enc.cleanup()
        self.assertNotIn(17, gpio.interrupts)


if __name__ == '__main__':
    unittest.main()
