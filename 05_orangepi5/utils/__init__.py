"""Orange Pi 5 工具模块"""
from .gpio_utils import GPIOManager
from .pwm_utils import PWMManager
from .serial_comm import SerialProtocol

__all__ = ['GPIOManager', 'PWMManager', 'SerialProtocol']
