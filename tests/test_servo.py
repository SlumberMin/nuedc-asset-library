#!/usr/bin/env python3
"""
舵机驱动单元测试
覆盖: ServoController 初始化、角度映射、限幅、持续模式、扫描测试
注意: 使用 Mock 模拟 GPIO 和 time.sleep
"""

import sys
import os
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from OrangePi5.utils.gpio_utils import GPIOManager
from OrangePi5.control.servo_controller import ServoController


class MockGPIOForServo:
    """模拟GPIO管理器(舵机用)"""
    OUT = 0
    LOW = 0
    HIGH = 1

    def __init__(self):
        self.pins = {}
        self.output_calls = []

    def setup(self, pin, direction):
        self.pins[pin] = {'dir': direction, 'state': 0}

    def output(self, pin, state):
        if pin in self.pins:
            self.pins[pin]['state'] = state
        self.output_calls.append((pin, state))


def _make_servo(gpio, pin=18, min_pulse_us=500, max_pulse_us=2500,
                min_angle=0.0, max_angle=180.0, freq_hz=50):
    """绕过__init__构造ServoController"""
    import threading
    servo = ServoController.__new__(ServoController)
    servo.gpio = gpio
    servo.pin = pin
    servo.min_pulse_us = min_pulse_us
    servo.max_pulse_us = max_pulse_us
    servo.min_angle = min_angle
    servo.max_angle = max_angle
    servo.freq_hz = freq_hz
    servo.period_us = 1_000_000 // freq_hz
    servo._angle = min_angle
    servo._running = False
    servo._thread = None
    servo._lock = threading.Lock()
    return servo


class TestServoInit(unittest.TestCase):
    """舵机初始化测试"""

    def test_default_angle_is_min(self):
        """初始角度应为最小角度"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio)
        self.assertEqual(servo.angle, 0.0)

    def test_period_calculation(self):
        """周期计算: 50Hz → 20000us"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio, freq_hz=50)
        self.assertEqual(servo.period_us, 20000)

    def test_270_degree_servo(self):
        """270°舵机max_angle=270"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio, max_angle=270.0)
        self.assertEqual(servo.max_angle, 270.0)


class TestServoAngleToPulse(unittest.TestCase):
    """角度→脉宽映射测试"""

    def test_min_angle_min_pulse(self):
        """最小角度应对应最小脉宽"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio, min_pulse_us=500, max_pulse_us=2500)
        pulse = servo._angle_to_pulse_us(0)
        self.assertEqual(pulse, 500)

    def test_max_angle_max_pulse(self):
        """最大角度应对应最大脉宽"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio, min_pulse_us=500, max_pulse_us=2500)
        pulse = servo._angle_to_pulse_us(180)
        self.assertEqual(pulse, 2500)

    def test_mid_angle_mid_pulse(self):
        """90°(中间)应对应1500us脉宽"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio, min_pulse_us=500, max_pulse_us=2500)
        pulse = servo._angle_to_pulse_us(90)
        self.assertEqual(pulse, 1500)

    def test_angle_below_min_clamped(self):
        """低于最小角度应被限幅"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio)
        pulse = servo._angle_to_pulse_us(-10)
        self.assertEqual(pulse, 500)

    def test_angle_above_max_clamped(self):
        """高于最大角度应被限幅"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio)
        pulse = servo._angle_to_pulse_us(200)
        self.assertEqual(pulse, 2500)

    def test_270_servo_mid_angle(self):
        """270°舵机: 135° → 1500us"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio, max_angle=270.0)
        pulse = servo._angle_to_pulse_us(135)
        self.assertEqual(pulse, 1500)


class TestServoSetAngle(unittest.TestCase):
    """角度设置测试"""

    @patch('time.sleep', return_value=None)
    def test_set_angle_updates_angle(self, mock_sleep):
        """set_angle应更新当前角度"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio)
        servo.set_angle(90)
        self.assertEqual(servo.angle, 90)

    @patch('time.sleep', return_value=None)
    def test_set_angle_clamps(self, mock_sleep):
        """set_angle应限幅角度"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio)
        servo.set_angle(200)
        self.assertEqual(servo.angle, 180)

    @patch('time.sleep', return_value=None)
    def test_set_angle_sends_pwm(self, mock_sleep):
        """set_angle应发送PWM信号"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio)
        servo.set_angle(90)
        # 应有HIGH和LOW的调用
        high_calls = [c for c in gpio.output_calls if c[1] == MockGPIOForServo.HIGH]
        low_calls = [c for c in gpio.output_calls if c[1] == MockGPIOForServo.LOW]
        self.assertTrue(len(high_calls) > 0)
        self.assertTrue(len(low_calls) > 0)


class TestServoUpdateAngle(unittest.TestCase):
    """持续模式角度更新测试"""

    def test_update_angle_clamps(self):
        """update_angle应限幅"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio)
        servo.update_angle(200)
        self.assertEqual(servo._angle, 180)

    def test_update_angle_negative(self):
        """update_angle应支持负角度"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio, min_angle=-90, max_angle=90)
        servo.update_angle(-45)
        self.assertEqual(servo._angle, -45)


class TestServoContinuousMode(unittest.TestCase):
    """持续PWM模式测试"""

    def test_start_continuous(self):
        """启动持续模式应创建线程"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio)
        with patch('time.sleep', return_value=None):
            servo.start_continuous(angle=90)
            self.assertTrue(servo._running)
            servo.stop()

    def test_start_continuous_with_angle(self):
        """启动时可指定角度"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio)
        with patch('time.sleep', return_value=None):
            servo.start_continuous(angle=45)
            self.assertEqual(servo._angle, 45)
            servo.stop()

    def test_stop_sets_running_false(self):
        """stop应设置_running=False"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio)
        with patch('time.sleep', return_value=None):
            servo.start_continuous()
            servo.stop()
            self.assertFalse(servo._running)

    def test_double_start_ignored(self):
        """重复启动应忽略"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio)
        with patch('time.sleep', return_value=None):
            servo.start_continuous(angle=45)
            servo.start_continuous(angle=90)  # 应忽略
            self.assertEqual(servo._angle, 45)
            servo.stop()


class TestServoCleanup(unittest.TestCase):
    """清理测试"""

    def test_cleanup_sets_low(self):
        """cleanup应将引脚设为LOW"""
        gpio = MockGPIOForServo()
        servo = _make_servo(gpio)
        with patch('time.sleep', return_value=None):
            servo.cleanup()
            last_call = gpio.output_calls[-1]
            self.assertEqual(last_call[1], MockGPIOForServo.LOW)


if __name__ == '__main__':
    unittest.main()
