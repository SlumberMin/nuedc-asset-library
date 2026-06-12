"""
GPIO 编码器读取 — 中断计数
支持增量式正交编码器 (A/B 相), 适用于 Orange Pi 5 (RK3588S)
"""
import time
import threading
from collections import deque
import math
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.gpio_utils import GPIOManager


class EncoderReader:
    """
    增量式编码器读取器

    使用 GPIO 中断捕获 A 相上升沿/下降沿，可选 B 相判向。
    支持单相 (仅计数) 和正交 (A+B 相判方向) 模式。

    Parameters
    ----------
    gpio : GPIOManager
    pin_a : int
        A 相引脚 (BCM 编号)
    pin_b : int, optional
        B 相引脚 (None 则为单相模式)
    ppr : int
        编码器每转脉冲数 (Pulses Per Revolution)
    pull_up : bool
        是否启用内部上拉
    """

    def __init__(
        self,
        gpio: GPIOManager,
        pin_a: int,
        pin_b: int = None,
        ppr: int = 360,
        pull_up: bool = True,
    ):
        self.gpio = gpio
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ppr = ppr
        self.quadrature = pin_b is not None

        self._count = 0
        self._lock = threading.Lock()
        self._timestamps = deque(maxlen=1000)
        self._prev_time = time.monotonic()

        # 设置引脚
        pull = GPIOManager.PUD_UP if pull_up else GPIOManager.PUD_DOWN
        self.gpio.setup(self.pin_a, GPIOManager.IN, pull)
        if self.quadrature:
            self.gpio.setup(self.pin_b, GPIOManager.IN, pull)

        # 注册中断
        self.gpio.add_interrupt(
            self.pin_a,
            callback=self._on_a_edge,
            edge='both'
        )

    def _on_a_edge(self, channel):
        """A 相边沿中断回调"""
        now = time.monotonic()
        a_state = self.gpio.input(self.pin_a)

        with self._lock:
            if self.quadrature:
                b_state = self.gpio.input(self.pin_b)
                # 正交解码: A 相变化时，根据 B 相状态判断方向
                if a_state == b_state:
                    self._count += 1
                else:
                    self._count -= 1
            else:
                self._count += 1

            self._timestamps.append(now)
            self._prev_time = now

    def get_count(self) -> int:
        """获取当前累计脉冲数"""
        with self._lock:
            return self._count

    def reset_count(self):
        """重置计数"""
        with self._lock:
            self._count = 0
            self._timestamps.clear()

    def get_speed(self, dt: float = 0.1) -> float:
        """
        计算转速 (RPM)

        Parameters
        ----------
        dt : float
            计算窗口 (秒)

        Returns
        -------
        float : RPM (可为负值表示反转)
        """
        now = time.monotonic()
        with self._lock:
            # 统计 dt 时间窗口内的脉冲数
            count_in_window = sum(
                1 for t in self._timestamps if t > now - dt
            )

            # 判断方向: 基于窗口内的增量而非累计计数
            if len(self._timestamps) >= 2:
                recent = [t for t in self._timestamps if t > now - dt]
                if len(recent) >= 2:
                    # 最近窗口内的脉冲方向由计数符号推断
                    direction = 1 if self._count >= 0 else -1
                else:
                    direction = 1 if self._count >= 0 else -1
            else:
                direction = 1

        if dt <= 0:
            return 0.0

        rps = count_in_window / dt / self.ppr  # 转/秒
        rpm = rps * 60 * direction
        return rpm

    def get_angle(self) -> float:
        """获取当前角度 (度)"""
        with self._lock:
            return self._count / self.ppr * 360.0

    def get_distance(self, wheel_diameter_m: float = 0.065) -> float:
        """
        计算行驶距离 (米)

        Parameters
        ----------
        wheel_diameter_m : float
            轮子直径 (米)
        """
        circumference = math.pi * wheel_diameter_m
        with self._lock:
            return self._count / self.ppr * circumference

    def cleanup(self):
        """清理"""
        self.gpio.remove_interrupt(self.pin_a)

    def __repr__(self):
        mode = "quadrature" if self.quadrature else "single"
        return f"EncoderReader(pin_a={self.pin_a}, ppr={self.ppr}, mode={mode})"
