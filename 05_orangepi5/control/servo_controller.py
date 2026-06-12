"""
GPIO 舵机控制 — 软件 PWM 实现
适用于 SG90 / MG996R 等常见舵机，Orange Pi 5 (RK3588S)
"""
import time
import threading
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.gpio_utils import GPIOManager


class ServoController:
    """
    软件 PWM 舵机控制器

    标准舵机: 50Hz, 0.5ms~2.5ms 脉宽 → 0°~180°
    180° 舵机: 角度范围 0~180
    270° 舵机: 角度范围 0~270 (调整 max_angle)

    Parameters
    ----------
    gpio : GPIOManager
    pin : int
        BCM 引脚号
    min_pulse_us : int
        最小脉宽 (微秒), 默认 500
    max_pulse_us : int
        最大脉宽 (微秒), 默认 2500
    min_angle : float
        最小角度, 默认 0
    max_angle : float
        最大角度, 默认 180
    freq_hz : int
        PWM 频率, 默认 50Hz
    """

    def __init__(
        self,
        gpio: GPIOManager,
        pin: int,
        min_pulse_us: int = 500,
        max_pulse_us: int = 2500,
        min_angle: float = 0.0,
        max_angle: float = 180.0,
        freq_hz: int = 50,
    ):
        self.gpio = gpio
        self.pin = pin
        self.min_pulse_us = min_pulse_us
        self.max_pulse_us = max_pulse_us
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.freq_hz = freq_hz
        self.period_us = 1_000_000 // freq_hz  # 周期 (微秒)

        self._angle = min_angle
        self._running = False
        self._thread = None
        self._lock = threading.Lock()

        self.gpio.setup(self.pin, GPIOManager.OUT)
        self.gpio.output(self.pin, GPIOManager.LOW)

    def _angle_to_pulse_us(self, angle: float) -> int:
        """角度 → 脉宽 (微秒)"""
        angle = max(self.min_angle, min(self.max_angle, angle))
        ratio = (angle - self.min_angle) / (self.max_angle - self.min_angle)
        return int(self.min_pulse_us + ratio * (self.max_pulse_us - self.min_pulse_us))

    def set_angle(self, angle: float):
        """
        设置舵机角度 (阻塞式，发送约 20 个周期确保到位)

        Parameters
        ----------
        angle : float
            目标角度
        """
        with self._lock:
            self._angle = max(self.min_angle, min(self.max_angle, angle))
            pulse_us = self._angle_to_pulse_us(self._angle)
            rest_us = self.period_us - pulse_us

            # 发送约 20 帧确保舵机到位 (约 0.4s @50Hz)
            for _ in range(20):
                self.gpio.output(self.pin, GPIOManager.HIGH)
                time.sleep(pulse_us / 1_000_000)
                self.gpio.output(self.pin, GPIOManager.LOW)
                time.sleep(rest_us / 1_000_000)

    def start_continuous(self, angle: float = None):
        """启动后台线程持续发送 PWM (用于需要持续维持角度的场景)"""
        if self._running:
            return
        if angle is not None:
            self._angle = angle
        self._running = True
        self._thread = threading.Thread(target=self._pwm_loop, daemon=True)
        self._thread.start()

    def _pwm_loop(self):
        """后台 PWM 循环"""
        while self._running:
            with self._lock:
                pulse_us = self._angle_to_pulse_us(self._angle)
            rest_us = self.period_us - pulse_us

            self.gpio.output(self.pin, GPIOManager.HIGH)
            time.sleep(pulse_us / 1_000_000)
            self.gpio.output(self.pin, GPIOManager.LOW)
            time.sleep(rest_us / 1_000_000)

    def update_angle(self, angle: float):
        """在持续模式下更新角度 (非阻塞)"""
        with self._lock:
            self._angle = max(self.min_angle, min(self.max_angle, angle))

    def stop(self):
        """停止持续 PWM 输出"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        self.gpio.output(self.pin, GPIOManager.LOW)

    @property
    def angle(self) -> float:
        return self._angle

    def sweep(self, start: float = None, end: float = None, step: float = 1.0, delay: float = 0.02):
        """
        扫描测试

        Parameters
        ----------
        start, end : float
            起止角度 (默认使用 min/max)
        step : float
            步进角度
        delay : float
            每步延迟 (秒)
        """
        start = start if start is not None else self.min_angle
        end = end if end is not None else self.max_angle

        # 正向
        angle = start
        while angle <= end:
            self.set_angle(angle)
            angle += step
            time.sleep(delay)

        # 反向
        while angle >= start:
            self.set_angle(angle)
            angle -= step
            time.sleep(delay)

    def cleanup(self):
        self.stop()
        self.gpio.output(self.pin, GPIOManager.LOW)
