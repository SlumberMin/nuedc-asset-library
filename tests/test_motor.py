#!/usr/bin/env python3
"""
电机驱动单元测试
覆盖: MotorController / DualMotorController 初始化、速度设置、方向控制、制动
注意: 使用 Mock 模拟 GPIO/PWM 硬件接口
"""

import sys
import os
import unittest
from unittest.mock import MagicMock, patch, PropertyMock

# 将项目根目录加入路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from OrangePi5.utils.gpio_utils import GPIOManager
from OrangePi5.utils.pwm_utils import PWMManager
from OrangePi5.control.motor_controller import MotorController, DualMotorController


class MockGPIO:
    """模拟GPIO管理器"""
    OUT = 0
    HIGH = 1
    LOW = 0

    def __init__(self):
        self.pins = {}
        self.setup_calls = []
        self.output_calls = []

    def setup(self, pin, direction):
        self.pins[pin] = {'dir': direction, 'state': 0}
        self.setup_calls.append((pin, direction))

    def output(self, pin, state):
        if pin in self.pins:
            self.pins[pin]['state'] = state
        self.output_calls.append((pin, state))


class MockPWM:
    """模拟PWM管理器"""
    def __init__(self):
        self.channels = {}
        self.init_calls = []
        self.duty_calls = []
        self.cleanup_calls = []

    def init(self, channel, pin, freq):
        self.channels[channel] = {'pin': pin, 'freq': freq, 'duty': 0}
        self.init_calls.append((channel, pin, freq))

    def set_duty(self, channel, duty):
        if channel in self.channels:
            self.channels[channel]['duty'] = duty
        self.duty_calls.append((channel, duty))

    def cleanup(self, channel):
        if channel in self.channels:
            del self.channels[channel]
        self.cleanup_calls.append(channel)


class TestMotorControllerInit(unittest.TestCase):
    """电机控制器初始化测试"""

    def setUp(self):
        self.gpio = MockGPIO()
        self.pwm = MockPWM()
        # Monkey-patch 类属性
        GPIOManager.OUT = 0
        GPIOManager.HIGH = 1
        GPIOManager.LOW = 0

    def _make_motor(self, **kwargs):
        """创建电机控制器(绕过GPIO/PWM类型检查)"""
        motor = MotorController.__new__(MotorController)
        motor.gpio = self.gpio
        motor.pwm = self.pwm
        motor.in1_pin = kwargs.get('in1_pin', 17)
        motor.in2_pin = kwargs.get('in2_pin', 27)
        motor.pwm_pin = kwargs.get('pwm_pin', 18)
        motor.pwm_channel = kwargs.get('pwm_channel', 0)
        motor.driver = kwargs.get('driver', 'L298N')
        motor.pwm_freq = kwargs.get('pwm_freq', 1000)
        motor.invert = kwargs.get('invert', False)
        motor._speed = 0
        return motor

    def test_motor_speed_property(self):
        """speed属性应返回当前速度"""
        motor = self._make_motor()
        motor._speed = 50
        self.assertEqual(motor.speed, 50)

    def test_set_speed_forward(self):
        """正速度应设置正转方向"""
        motor = self._make_motor()
        motor.set_speed(60)
        self.assertEqual(motor._speed, 60)
        # 应该有方向引脚的调用
        self.assertTrue(len(self.gpio.output_calls) > 0)

    def test_set_speed_reverse(self):
        """负速度应设置反转方向"""
        motor = self._make_motor()
        motor.set_speed(-60)
        self.assertEqual(motor._speed, -60)

    def test_set_speed_zero(self):
        """零速度应停止电机"""
        motor = self._make_motor()
        motor.set_speed(0)
        self.assertEqual(motor._speed, 0)

    def test_speed_clamping_positive(self):
        """速度应被限幅到 [-100, 100]"""
        motor = self._make_motor()
        motor.set_speed(150)
        self.assertEqual(motor._speed, 100)

    def test_speed_clamping_negative(self):
        motor = self._make_motor()
        motor.set_speed(-200)
        self.assertEqual(motor._speed, -100)

    def test_invert_flag(self):
        """反转标志应取反速度"""
        motor = self._make_motor(invert=True)
        motor.set_speed(50)
        # 内部speed取反后为-50，但存储的是反转后的
        self.assertEqual(motor._speed, -50)


class TestMotorControllerDirections(unittest.TestCase):
    """方向控制测试"""

    def setUp(self):
        self.gpio = MockGPIO()
        self.pwm = MockPWM()

    def _make_motor(self, driver='L298N'):
        motor = MotorController.__new__(MotorController)
        motor.gpio = self.gpio
        motor.pwm = self.pwm
        motor.in1_pin = 17
        motor.in2_pin = 27
        motor.pwm_pin = 18
        motor.pwm_channel = 0
        motor.driver = driver
        motor.pwm_freq = 1000
        motor.invert = False
        motor._speed = 0
        return motor

    def test_forward_l298n(self):
        """L298N正转: IN1=HIGH, IN2=LOW"""
        motor = self._make_motor('L298N')
        motor.set_speed(50)
        # 检查方向引脚状态
        calls = {c[0]: c[1] for c in self.gpio.output_calls[-2:]}
        self.assertEqual(calls.get(17), GPIOManager.HIGH)
        self.assertEqual(calls.get(27), GPIOManager.LOW)

    def test_reverse_l298n(self):
        """L298N反转: IN1=LOW, IN2=HIGH"""
        motor = self._make_motor('L298N')
        motor.set_speed(-50)
        calls = {c[0]: c[1] for c in self.gpio.output_calls[-2:]}
        self.assertEqual(calls.get(17), GPIOManager.LOW)
        self.assertEqual(calls.get(27), GPIOManager.HIGH)

    def test_forward_tb6612(self):
        """TB6612正转: IN1=HIGH, IN2=LOW"""
        motor = self._make_motor('TB6612FNG')
        motor.set_speed(50)
        calls = {c[0]: c[1] for c in self.gpio.output_calls[-2:]}
        self.assertEqual(calls.get(17), GPIOManager.HIGH)
        self.assertEqual(calls.get(27), GPIOManager.LOW)

    def test_brake_tb6612(self):
        """TB6612制动: IN1=HIGH, IN2=HIGH"""
        motor = self._make_motor('TB6612FNG')
        motor.brake()
        calls = {c[0]: c[1] for c in self.gpio.output_calls[-2:]}
        self.assertEqual(calls.get(17), GPIOManager.HIGH)
        self.assertEqual(calls.get(27), GPIOManager.HIGH)


class TestMotorControllerStop(unittest.TestCase):
    """停止/制动测试"""

    def setUp(self):
        self.gpio = MockGPIO()
        self.pwm = MockPWM()

    def _make_motor(self):
        motor = MotorController.__new__(MotorController)
        motor.gpio = self.gpio
        motor.pwm = self.pwm
        motor.in1_pin = 17
        motor.in2_pin = 27
        motor.pwm_pin = 18
        motor.pwm_channel = 0
        motor.driver = 'L298N'
        motor.pwm_freq = 1000
        motor.invert = False
        motor._speed = 0
        return motor

    def test_stop_sets_speed_zero(self):
        motor = self._make_motor()
        motor.set_speed(80)
        motor.stop()
        self.assertEqual(motor.speed, 0)

    def test_brake_sets_speed_zero(self):
        motor = self._make_motor()
        motor.set_speed(80)
        motor.brake()
        self.assertEqual(motor.speed, 0)

    def test_stop_sets_pwm_zero(self):
        motor = self._make_motor()
        motor.set_speed(80)
        motor.stop()
        # 最后的PWM调用应该是0
        last_duty = self.pwm.duty_calls[-1][1]
        self.assertEqual(last_duty, 0)


class TestDualMotorController(unittest.TestCase):
    """双电机控制器测试"""

    def setUp(self):
        self.gpio = MockGPIO()
        self.pwm = MockPWM()

    def _make_dual(self):
        left = MotorController.__new__(MotorController)
        left.gpio = self.gpio
        left.pwm = self.pwm
        left.in1_pin = 17
        left.in2_pin = 27
        left.pwm_pin = 18
        left.pwm_channel = 0
        left.driver = 'L298N'
        left.pwm_freq = 1000
        left.invert = False
        left._speed = 0

        right = MotorController.__new__(MotorController)
        right.gpio = self.gpio
        right.pwm = self.pwm
        right.in1_pin = 22
        right.in2_pin = 23
        right.pwm_pin = 24
        right.pwm_channel = 1
        right.driver = 'L298N'
        right.pwm_freq = 1000
        right.invert = False
        right._speed = 0

        return DualMotorController(left, right)

    def test_set_speeds(self):
        dual = self._make_dual()
        dual.set_speeds(50, 75)
        self.assertEqual(dual.left.speed, 50)
        self.assertEqual(dual.right.speed, 75)

    def test_drive_straight(self):
        """纯直线行驶时左右速度应相等"""
        dual = self._make_dual()
        dual.drive(linear=50, angular=0)
        self.assertEqual(dual.left.speed, dual.right.speed)

    def test_drive_turn(self):
        """转弯时左右速度应不同"""
        dual = self._make_dual()
        dual.drive(linear=50, angular=30, wheel_base=0.2)
        self.assertNotEqual(dual.left.speed, dual.right.speed)

    def test_stop_all(self):
        dual = self._make_dual()
        dual.set_speeds(50, 50)
        dual.stop()
        self.assertEqual(dual.left.speed, 0)
        self.assertEqual(dual.right.speed, 0)


class TestMotorPWMDutyMapping(unittest.TestCase):
    """PWM占空比映射测试"""

    def test_duty_proportional_to_speed(self):
        """占空比应与速度绝对值成正比"""
        pwm = MockPWM()
        gpio = MockGPIO()

        motor = MotorController.__new__(MotorController)
        motor.gpio = gpio
        motor.pwm = pwm
        motor.in1_pin = 17
        motor.in2_pin = 27
        motor.pwm_pin = 18
        motor.pwm_channel = 0
        motor.driver = 'L298N'
        motor.pwm_freq = 1000
        motor.invert = False
        motor._speed = 0

        motor.set_speed(50)
        duty_50 = pwm.duty_calls[-1][1]

        motor.set_speed(100)
        duty_100 = pwm.duty_calls[-1][1]

        # 100%速度的占空比应 >= 50%速度的占空比
        self.assertGreaterEqual(duty_100, duty_50)


if __name__ == '__main__':
    unittest.main()
