"""
PWM 操作封装 — 硬件 PWM / 软件 PWM
适用于 Orange Pi 5 (RK3588S), 支持 WiringPi 硬件 PWM 和软件 PWM
"""
import time
import threading
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class HardwarePWM:
    """
    硬件 PWM (Orange Pi 5 的 RK3588S 支持多路硬件 PWM)

    通过 /sys/class/pwm/pwmchipN/ 或 WiringPi 硬件 PWM 接口

    Parameters
    ----------
    chip : int
        PWM 控制器编号 (通常 0)
    channel : int
        PWM 通道号
    freq_hz : int
        频率 (Hz)
    pin : int
        对应的 WiringPi/BCM 引脚 (用于映射)
    """

    PWM_PATH = "/sys/class/pwm"

    def __init__(self, chip: int = 0, channel: int = 0, freq_hz: int = 1000, pin: int = None):
        self.chip = chip
        self.channel = channel
        self.freq_hz = freq_hz
        self.pin = pin
        self._period_ns = int(1_000_000_000 / freq_hz)
        self._duty_ns = 0
        self._enabled = False

    def _pwm_path(self, attr: str = "") -> str:
        return f"{self.PWM_PATH}/pwmchip{self.chip}/pwm{self.channel}/{attr}"

    def init(self):
        """初始化硬件 PWM"""
        # 导出 PWM 通道
        export_path = f"{self.PWM_PATH}/pwmchip{self.chip}/export"
        try:
            with open(export_path, 'w') as f:
                f.write(str(self.channel))
            time.sleep(0.1)
        except (FileNotFoundError, PermissionError) as e:
            logger.warning(f"硬件 PWM 导出失败: {e}")
            raise

        # 设置周期
        self.set_frequency(self.freq_hz)
        self.set_duty(0)
        self.enable()

    def set_frequency(self, freq_hz: int):
        """设置 PWM 频率"""
        self.freq_hz = freq_hz
        self._period_ns = int(1_000_000_000 / freq_hz)
        try:
            with open(self._pwm_path("period"), 'w') as f:
                f.write(str(self._period_ns))
        except (FileNotFoundError, PermissionError) as e:
            logger.warning(f"设置 PWM 频率失败: {e}")

    def set_duty(self, duty_percent: float):
        """
        设置占空比

        Parameters
        ----------
        duty_percent : float
            0 ~ 100
        """
        duty_percent = max(0, min(100, duty_percent))
        self._duty_ns = int(self._period_ns * duty_percent / 100)
        try:
            with open(self._pwm_path("duty_cycle"), 'w') as f:
                f.write(str(self._duty_ns))
        except (FileNotFoundError, PermissionError) as e:
            logger.warning(f"设置 PWM 占空比失败: {e}")

    def enable(self):
        """使能 PWM 输出"""
        try:
            with open(self._pwm_path("enable"), 'w') as f:
                f.write("1")
            self._enabled = True
        except (FileNotFoundError, PermissionError) as e:
            logger.warning(f"使能 PWM 失败: {e}")

    def disable(self):
        """禁用 PWM 输出"""
        try:
            with open(self._pwm_path("enable"), 'w') as f:
                f.write("0")
            self._enabled = False
        except (FileNotFoundError, PermissionError):
            pass

    def cleanup(self):
        self.disable()
        unexport_path = f"{self.PWM_PATH}/pwmchip{self.chip}/unexport"
        try:
            with open(unexport_path, 'w') as f:
                f.write(str(self.channel))
        except (FileNotFoundError, PermissionError):
            pass


class SoftwarePWM:
    """
    软件 PWM 实现 (通过 GPIO 高低电平切换)

    适用于任意 GPIO 引脚，精度受系统调度影响。
    适合舵机控制 (50Hz) 和低速 PWM 场景。

    Parameters
    ----------
    gpio_manager : GPIOManager
        GPIO 管理器实例
    pin : int
        GPIO 引脚号
    freq_hz : int
        PWM 频率 (Hz)
    """

    def __init__(self, gpio_manager, pin: int, freq_hz: int = 1000):
        self.gpio = gpio_manager
        self.pin = pin
        self.freq_hz = freq_hz
        self._duty = 0.0  # 0 ~ 100
        self._running = False
        self._thread = None

        self.gpio.setup(self.pin, self.gpio.OUT)
        self.gpio.output(self.pin, self.gpio.LOW)

    def start(self, duty_percent: float = 0):
        """启动软件 PWM"""
        if self._running:
            return
        self._duty = max(0, min(100, duty_percent))
        self._running = True
        self._thread = threading.Thread(target=self._pwm_loop, daemon=True)
        self._thread.start()

    def _pwm_loop(self):
        """PWM 主循环"""
        period = 1.0 / self.freq_hz
        while self._running:
            if self._duty <= 0:
                self.gpio.output(self.pin, self.gpio.LOW)
                time.sleep(period)
            elif self._duty >= 100:
                self.gpio.output(self.pin, self.gpio.HIGH)
                time.sleep(period)
            else:
                high_time = period * self._duty / 100
                low_time = period - high_time
                self.gpio.output(self.pin, self.gpio.HIGH)
                time.sleep(high_time)
                self.gpio.output(self.pin, self.gpio.LOW)
                time.sleep(low_time)

    def set_duty(self, duty_percent: float):
        """设置占空比 (0 ~ 100)"""
        self._duty = max(0, min(100, duty_percent))

    def stop(self):
        """停止软件 PWM"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        self.gpio.output(self.pin, self.gpio.LOW)

    def cleanup(self):
        self.stop()


class PWMManager:
    """
    PWM 管理器 — 统一管理硬件/软件 PWM

    Parameters
    ----------
    gpio_manager : GPIOManager
        GPIO 管理器 (软件 PWM 需要)
    prefer_hw : bool
        优先使用硬件 PWM
    """

    def __init__(self, gpio_manager=None, prefer_hw: bool = True):
        self.gpio = gpio_manager
        self.prefer_hw = prefer_hw
        self._channels = {}  # channel_id -> PWM instance

    def init(self, channel: int, pin: int, freq_hz: int = 1000,
             hw_chip: int = 0, hw_channel: int = None):
        """
        初始化 PWM 通道

        Parameters
        ----------
        channel : int
            逻辑通道号 (用户自定义)
        pin : int
            GPIO 引脚号
        freq_hz : int
            频率
        hw_chip : int
            硬件 PWM 控制器编号
        hw_channel : int
            硬件 PWM 通道号 (None 则用 channel)
        """
        pwm = None

        if self.prefer_hw:
            try:
                hw_ch = hw_channel if hw_channel is not None else channel
                pwm = HardwarePWM(chip=hw_chip, channel=hw_ch, freq_hz=freq_hz, pin=pin)
                pwm.init()
                logger.info(f"PWM 通道 {channel}: 硬件 PWM (chip={hw_chip}, ch={hw_ch})")
            except Exception as e:
                logger.warning(f"硬件 PWM 不可用, 回退到软件 PWM: {e}")
                pwm = None

        if pwm is None:
            if self.gpio is None:
                raise RuntimeError("软件 PWM 需要 GPIOManager 实例")
            pwm = SoftwarePWM(self.gpio, pin, freq_hz)
            pwm.start(0)
            logger.info(f"PWM 通道 {channel}: 软件 PWM (pin={pin})")

        self._channels[channel] = pwm

    def set_duty(self, channel: int, duty_percent: float):
        """设置指定通道的占空比"""
        if channel in self._channels:
            self._channels[channel].set_duty(duty_percent)
        else:
            raise ValueError(f"PWM 通道 {channel} 未初始化")

    def set_frequency(self, channel: int, freq_hz: int):
        """设置指定通道的频率 (仅硬件 PWM 有效)"""
        if channel in self._channels:
            pwm = self._channels[channel]
            if isinstance(pwm, HardwarePWM):
                pwm.set_frequency(freq_hz)

    def cleanup(self, channel: int = None):
        """清理 PWM"""
        if channel is not None:
            if channel in self._channels:
                self._channels[channel].cleanup()
                del self._channels[channel]
        else:
            for ch in list(self._channels.values()):
                ch.cleanup()
            self._channels.clear()
