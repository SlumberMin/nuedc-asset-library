"""
GPIO 操作封装 — 支持 WiringPi / lgpio / sysfs 三种后端
自动检测可用后端，适用于 Orange Pi 5 (RK3588S)
"""
import os
import sys
import logging
import time
from typing import Callable, Optional

logger = logging.getLogger(__name__)


class GPIOBackend:
    """GPIO 后端基类"""
    OUT = "out"
    IN = "in"
    HIGH = 1
    LOW = 0
    PUD_UP = "up"
    PUD_DOWN = "down"
    PUD_OFF = "off"

    def setup(self, pin: int, mode: str, pull: str = None):
        raise NotImplementedError

    def output(self, pin: int, value: int):
        raise NotImplementedError

    def input(self, pin: int) -> int:
        raise NotImplementedError

    def cleanup(self, pin: int = None):
        raise NotImplementedError

    def add_interrupt(self, pin: int, callback: Callable, edge: str = 'both'):
        raise NotImplementedError

    def remove_interrupt(self, pin: int):
        raise NotImplementedError


class WiringPiBackend(GPIOBackend):
    """WiringPi 后端 (通过 wiringpi Python 绑定)"""

    def __init__(self):
        import wiringpi
        self.wpi = wiringpi
        wiringpi.wiringPiSetup()  # 使用 WiringPi 编号

    def setup(self, pin: int, mode: str, pull: str = None):
        if mode == self.OUT:
            self.wpi.pinMode(pin, 1)
        else:
            self.wpi.pinMode(pin, 0)
            if pull == self.PUD_UP:
                self.wpi.pullUpDnControl(pin, 2)
            elif pull == self.PUD_DOWN:
                self.wpi.pullUpDnControl(pin, 1)

    def output(self, pin: int, value: int):
        self.wpi.digitalWrite(pin, value)

    def input(self, pin: int) -> int:
        return self.wpi.digitalRead(pin)

    def cleanup(self, pin: int = None):
        pass  # WiringPi 无全局 cleanup

    def add_interrupt(self, pin: int, callback: Callable, edge: str = 'both'):
        mode = {
            'rising': 1, 'falling': 2, 'both': 3
        }.get(edge, 3)
        self.wpi.wiringPiISR(pin, mode, callback)

    def remove_interrupt(self, pin: int):
        pass  # WiringPi 不支持移除中断


class LgpioBackend(GPIOBackend):
    """lgpio 后端 (适用于较新内核, Orange Pi 5 推荐)"""

    def __init__(self):
        import lgpio
        self.lg = lgpio
        self.chip = lgpio.gpiochip_open(0)
        self._handles = {}
        self._callbacks = {}
        self._interrupts = {}

    def setup(self, pin: int, mode: str, pull: str = None):
        if mode == self.OUT:
            self.lg.gpio_claim_output(self.chip, pin, 0)
        else:
            flags = 0
            if pull == self.PUD_UP:
                flags = self.lg.SET_PULL_UP
            elif pull == self.PUD_DOWN:
                flags = self.lg.SET_PULL_DOWN
            self.lg.gpio_claim_input(self.chip, pin, flags)

    def output(self, pin: int, value: int):
        self.lg.gpio_write(self.chip, pin, value)

    def input(self, pin: int) -> int:
        return self.lg.gpio_read(self.chip, pin)

    def cleanup(self, pin: int = None):
        if pin is not None:
            self.lg.gpio_free(self.chip, pin)
        else:
            self.lg.gpiochip_close(self.chip)

    def add_interrupt(self, pin: int, callback: Callable, edge: str = 'both'):
        edge_flag = {
            'rising': self.lg.RISING_EDGE,
            'falling': self.lg.FALLING_EDGE,
            'both': self.lg.BOTH_EDGES,
        }.get(edge, self.lg.BOTH_EDGES)
        self._callbacks[pin] = callback
        self._interrupts[pin] = self.lg.gpio_claim_alert(
            self.chip, pin, edge_flag
        )
        # lgpio 需要轮询事件或使用回调线程
        self.lg.gpio_set_alerts_func(
            self.chip, pin, self._make_handler(pin)
        )

    def _make_handler(self, pin: int):
        def handler(chip, gpio, level, timestamp):
            if pin in self._callbacks:
                self._callbacks[pin](gpio)
        return handler

    def remove_interrupt(self, pin: int):
        if pin in self._interrupts:
            self.lg.gpio_free(self.chip, pin)
            del self._interrupts[pin]
            del self._callbacks[pin]


class SysfsBackend(GPIOBackend):
    """sysfs GPIO 后端 (兼容性最好, 性能最低)"""

    GPIO_PATH = "/sys/class/gpio"

    def __init__(self):
        self._exported = set()

    def _export(self, pin: int):
        if pin not in self._exported:
            path = os.path.join(self.GPIO_PATH, "export")
            if os.path.exists(path):
                with open(path, 'w') as f:
                    f.write(str(pin))
                self._exported.add(pin)
                time.sleep(0.1)  # 等待 sysfs 创建

    def _unexport(self, pin: int):
        path = os.path.join(self.GPIO_PATH, "unexport")
        if os.path.exists(path):
            with open(path, 'w') as f:
                f.write(str(pin))
            self._exported.discard(pin)

    def setup(self, pin: int, mode: str, pull: str = None):
        self._export(pin)
        direction_path = os.path.join(
            self.GPIO_PATH, f"gpio{pin}", "direction"
        )
        with open(direction_path, 'w') as f:
            f.write("out" if mode == self.OUT else "in")

    def output(self, pin: int, value: int):
        value_path = os.path.join(
            self.GPIO_PATH, f"gpio{pin}", "value"
        )
        with open(value_path, 'w') as f:
            f.write(str(value))

    def input(self, pin: int) -> int:
        value_path = os.path.join(
            self.GPIO_PATH, f"gpio{pin}", "value"
        )
        with open(value_path, 'r') as f:
            return int(f.read().strip())

    def cleanup(self, pin: int = None):
        if pin is not None:
            self._unexport(pin)
        else:
            for p in list(self._exported):
                self._unexport(p)

    def add_interrupt(self, pin: int, callback: Callable, edge: str = 'both'):
        """sysfs 中断 (通过 polling /sys/class/gpio/gpioN/value)"""
        edge_path = os.path.join(
            self.GPIO_PATH, f"gpio{pin}", "edge"
        )
        with open(edge_path, 'w') as f:
            f.write(edge)
        # sysfs 中断需要 epoll 或轮询，此处仅存储回调
        self._interrupts = getattr(self, '_interrupts', {})
        self._interrupts[pin] = (callback, edge)

    def remove_interrupt(self, pin: int):
        if hasattr(self, '_interrupts') and pin in self._interrupts:
            edge_path = os.path.join(
                self.GPIO_PATH, f"gpio{pin}", "edge"
            )
            with open(edge_path, 'w') as f:
                f.write("none")
            del self._interrupts[pin]


class GPIOManager:
    """
    GPIO 管理器 — 自动选择后端

    Parameters
    ----------
    backend : str, optional
        'wiringpi', 'lgpio', 'sysfs'。None 则自动检测。
    """

    # 常量转发
    OUT = "out"
    IN = "in"
    HIGH = 1
    LOW = 0
    PUD_UP = "up"
    PUD_DOWN = "down"
    PUD_OFF = "off"

    def __init__(self, backend: str = None):
        self._backend = self._create_backend(backend)
        logger.info(f"GPIO 后端: {type(self._backend).__name__}")

    @staticmethod
    def _create_backend(backend: str = None) -> GPIOBackend:
        if backend:
            backend = backend.lower()

        if backend == 'lgpio' or backend is None:
            try:
                return LgpioBackend()
            except ImportError:
                if backend == 'lgpio':
                    raise

        if backend == 'wiringpi' or backend is None:
            try:
                return WiringPiBackend()
            except ImportError:
                if backend == 'wiringpi':
                    raise

        if backend == 'sysfs' or backend is None:
            return SysfsBackend()

        raise ValueError(f"不支持的 GPIO 后端: {backend}")

    def setup(self, pin: int, mode: str, pull: str = None):
        self._backend.setup(pin, mode, pull)

    def output(self, pin: int, value: int):
        self._backend.output(pin, value)

    def input(self, pin: int) -> int:
        return self._backend.input(pin)

    def cleanup(self, pin: int = None):
        self._backend.cleanup(pin)

    def add_interrupt(self, pin: int, callback: Callable, edge: str = 'both'):
        self._backend.add_interrupt(pin, callback, edge)

    def remove_interrupt(self, pin: int):
        self._backend.remove_interrupt(pin)

    @property
    def backend_name(self) -> str:
        return type(self._backend).__name__
