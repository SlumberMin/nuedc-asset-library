"""
GPIO 电机控制 — 支持 L298N / TB6612FNG 驱动板
适用于 Orange Pi 5 (RK3588S) GPIO + PWM
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.gpio_utils import GPIOManager
from utils.pwm_utils import PWMManager


class MotorController:
    """
    单路直流电机控制器

    支持 L298N 和 TB6612FNG 两种常见驱动板接法。

    Parameters
    ----------
    gpio : GPIOManager
        GPIO 管理器实例
    pwm : PWMManager
        PWM 管理器实例
    in1_pin, in2_pin : int
        方向控制引脚 (BCM 编号)
    pwm_pin : int
        PWM 调速引脚
    pwm_channel : int
        PWM 通道号 (软件 PWM 时为 0)
    driver : str
        'L298N' 或 'TB6612FNG'
    pwm_freq : int
        PWM 频率 (Hz)
    invert : bool
        是否反转方向
    """

    def __init__(
        self,
        gpio: GPIOManager,
        pwm: PWMManager,
        in1_pin: int,
        in2_pin: int,
        pwm_pin: int,
        pwm_channel: int = 0,
        driver: str = "L298N",
        pwm_freq: int = 1000,
        invert: bool = False,
    ):
        self.gpio = gpio
        self.pwm = pwm
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.pwm_pin = pwm_pin
        self.pwm_channel = pwm_channel
        self.driver = driver.upper()
        self.pwm_freq = pwm_freq
        self.invert = invert

        self._speed = 0  # -100 ~ 100

        # 初始化引脚
        self.gpio.setup(self.in1_pin, GPIOManager.OUT)
        self.gpio.setup(self.in2_pin, GPIOManager.OUT)
        self.pwm.init(self.pwm_channel, self.pwm_pin, self.pwm_freq)

        self.stop()

    def set_speed(self, speed: float):
        """
        设置电机速度

        Parameters
        ----------
        speed : float
            -100 ~ 100, 正值正转, 负值反转
        """
        speed = max(-100, min(100, speed))
        if self.invert:
            speed = -speed
        self._speed = speed

        duty = abs(speed)

        if speed > 0:
            self._forward()
        elif speed < 0:
            self._reverse()
        else:
            self._brake()
            duty = 0

        self.pwm.set_duty(self.pwm_channel, duty)

    def _forward(self):
        if self.driver == "TB6612FNG":
            self.gpio.output(self.in1_pin, GPIOManager.HIGH)
            self.gpio.output(self.in2_pin, GPIOManager.LOW)
        else:  # L298N
            self.gpio.output(self.in1_pin, GPIOManager.HIGH)
            self.gpio.output(self.in2_pin, GPIOManager.LOW)

    def _reverse(self):
        if self.driver == "TB6612FNG":
            self.gpio.output(self.in1_pin, GPIOManager.LOW)
            self.gpio.output(self.in2_pin, GPIOManager.HIGH)
        else:
            self.gpio.output(self.in1_pin, GPIOManager.LOW)
            self.gpio.output(self.in2_pin, GPIOManager.HIGH)

    def _brake(self):
        if self.driver == "TB6612FNG":
            self.gpio.output(self.in1_pin, GPIOManager.HIGH)
            self.gpio.output(self.in2_pin, GPIOManager.HIGH)
        else:  # L298N: 两个都 LOW
            self.gpio.output(self.in1_pin, GPIOManager.LOW)
            self.gpio.output(self.in2_pin, GPIOManager.LOW)

    def stop(self):
        """停止电机 (惰性停止, PWM=0)"""
        self._speed = 0
        self.pwm.set_duty(self.pwm_channel, 0)
        self._brake()

    def brake(self):
        """制动 (短接电机两端)"""
        self._speed = 0
        self.pwm.set_duty(self.pwm_channel, 0)
        if self.driver == "TB6612FNG":
            self.gpio.output(self.in1_pin, GPIOManager.HIGH)
            self.gpio.output(self.in2_pin, GPIOManager.HIGH)

    @property
    def speed(self) -> float:
        return self._speed

    def cleanup(self):
        self.stop()
        self.pwm.cleanup(self.pwm_channel)


class DualMotorController:
    """
    双路电机控制器 (差速小车常用)

    Parameters
    ----------
    left_motor, right_motor : MotorController
        左右电机实例
    """

    def __init__(self, left_motor: MotorController, right_motor: MotorController):
        self.left = left_motor
        self.right = right_motor

    def set_speeds(self, left_speed: float, right_speed: float):
        """分别设置左右电机速度"""
        self.left.set_speed(left_speed)
        self.right.set_speed(right_speed)

    def drive(self, linear: float, angular: float, wheel_base: float = 0.2):
        """
        差速驱动模型

        Parameters
        ----------
        linear : float
            线速度 (-100 ~ 100)
        angular : float
            角速度 (-100 ~ 100, 正值左转)
        wheel_base : float
            轮距 (米)
        """
        left = linear - angular * wheel_base / 2
        right = linear + angular * wheel_base / 2

        # 归一化
        max_val = max(abs(left), abs(right), 100)
        if max_val > 100:
            left = left / max_val * 100
            right = right / max_val * 100

        self.set_speeds(left, right)

    def stop(self):
        self.left.stop()
        self.right.stop()

    def brake(self):
        self.left.brake()
        self.right.brake()

    def cleanup(self):
        self.left.cleanup()
        self.right.cleanup()
