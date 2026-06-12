#!/usr/bin/env python3
"""
IO扩展器仿真 (IO Expander Simulation)
=======================================
仿真内容:
  - I2C通信协议仿真 (地址+寄存器读写)
  - GPIO输入/输出/中断模式
  - 多设备级联 (8个设备, 128个IO)
  - 中断逻辑 (边沿检测+优先级)
  - 扫描采样与去抖动
  - PCF8574/MCP23017兼容行为

依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Callable
from enum import IntEnum
from collections import deque


class PinMode(IntEnum):
    INPUT = 0
    OUTPUT = 1
    INPUT_PULLUP = 2
    INTERRUPT_RISING = 3
    INTERRUPT_FALLING = 4
    INTERRUPT_BOTH = 5


class InterruptFlag(IntEnum):
    NONE = 0
    RISING = 1
    FALLING = 2
    BOTH = 3


@dataclass
class IOExpanderConfig:
    """IO扩展器配置"""
    name: str = "MCP23017"
    base_address: int = 0x20    # I2C基础地址
    n_pins: int = 16            # IO引脚数
    has_interrupt: bool = True
    has_pullup: bool = True
    scan_rate_hz: float = 1000  # 扫描频率
    debounce_ms: float = 5      # 去抖动时间
    i2c_speed: float = 400e3    # I2C速度 400kHz


@dataclass
class PinState:
    """单个引脚状态"""
    mode: PinMode = PinMode.INPUT
    output_value: bool = False
    input_value: bool = False
    pullup_enabled: bool = False
    interrupt_flag: InterruptFlag = InterruptFlag.NONE
    debounce_counter: int = 0
    last_stable: bool = False
    transition_time: float = 0


class IOExpanderDevice:
    """单个IO扩展器设备"""

    def __init__(self, config: IOExpanderConfig, address_offset: int = 0):
        self.config = config
        self.address = config.base_address + address_offset
        self.pins = [PinState() for _ in range(config.n_pins)]
        self.registers: Dict[str, int] = {
            'IODIR': 0xFFFF,      # 方向寄存器 (1=输入)
            'IPOL': 0x0000,       # 输入极性
            'GPINTEN': 0x0000,    # 中断使能
            'DEFVAL': 0x0000,     # 默认值
            'INTCON': 0x0000,     # 中断控制
            'IOCON': 0x0000,      # 配置
            'GPPU': 0x0000,       # 上拉使能
            'INTF': 0x0000,       # 中断标志
            'INTCAP': 0x0000,     # 中断捕获
            'GPIO': 0x0000,       # GPIO值
            'OLAT': 0x0000,       # 输出锁存
        }
        self.interrupt_pending = False
        self.transaction_log: List[dict] = []

    def i2c_write(self, register: str, value: int) -> bool:
        """模拟I2C写寄存器"""
        if register not in self.registers:
            return False
        self.registers[register] = value & 0xFFFF
        self.transaction_log.append({
            'type': 'write', 'addr': hex(self.address),
            'reg': register, 'value': hex(value), 'time': 0
        })

        # 更新引脚状态
        if register == 'IODIR':
            for i in range(self.config.n_pins):
                self.pins[i].mode = PinMode.INPUT if (value >> i) & 1 else PinMode.OUTPUT
        elif register == 'GPPU':
            for i in range(self.config.n_pins):
                self.pins[i].pullup_enabled = bool((value >> i) & 1)
        elif register == 'GPIO' or register == 'OLAT':
            for i in range(self.config.n_pins):
                if self.pins[i].mode == PinMode.OUTPUT:
                    self.pins[i].output_value = bool((value >> i) & 1)
        elif register == 'GPINTEN':
            for i in range(self.config.n_pins):
                if (value >> i) & 1:
                    self.pins[i].mode = PinMode.INTERRUPT_BOTH

        return True

    def i2c_read(self, register: str) -> Optional[int]:
        """模拟I2C读寄存器"""
        if register not in self.registers:
            return None

        if register == 'GPIO':
            val = 0
            for i in range(self.config.n_pins):
                if self.pins[i].input_value or (self.pins[i].pullup_enabled and self.pins[i].mode == PinMode.INPUT):
                    val |= (1 << i)
            self.registers['GPIO'] = val
        elif register == 'INTCAP':
            val = self.registers['INTCAP']
            self.interrupt_pending = False
            self.registers['INTF'] = 0
            return val

        self.transaction_log.append({
            'type': 'read', 'addr': hex(self.address),
            'reg': register, 'value': hex(self.registers[register])
        })
        return self.registers[register]

    def set_pin_input(self, pin: int, value: bool, timestamp: float = 0):
        """设置引脚输入值 (外部信号)"""
        if 0 <= pin < self.config.n_pins:
            old = self.pins[pin].input_value
            self.pins[pin].input_value = value

            # 去抖动
            if old != value:
                self.pins[pin].debounce_counter = int(
                    self.config.debounce_ms * self.config.scan_rate_hz / 1000)
                self.pins[pin].transition_time = timestamp

            # 中断检测
            if self.pins[pin].mode >= PinMode.INTERRUPT_RISING:
                if old != value:
                    if value and self.pins[pin].mode in (PinMode.INTERRUPT_RISING, PinMode.INTERRUPT_BOTH):
                        self.pins[pin].interrupt_flag = InterruptFlag.RISING
                        self._trigger_interrupt(pin, InterruptFlag.RISING, value, timestamp)
                    elif not value and self.pins[pin].mode in (PinMode.INTERRUPT_FALLING, PinMode.INTERRUPT_BOTH):
                        self.pins[pin].interrupt_flag = InterruptFlag.FALLING
                        self._trigger_interrupt(pin, InterruptFlag.FALLING, value, timestamp)

    def _trigger_interrupt(self, pin: int, flag: InterruptFlag, value: bool, timestamp: float):
        """触发中断"""
        self.interrupt_pending = True
        self.registers['INTF'] |= (1 << pin)
        self.registers['INTCAP'] = self.registers['GPIO']

    def update(self, dt: float):
        """更新去抖动计数器"""
        for pin in self.pins:
            if pin.debounce_counter > 0:
                pin.debounce_counter -= 1
                if pin.debounce_counter == 0:
                    pin.last_stable = pin.input_value


class IOExpanderBus:
    """IO扩展器总线管理 (多设备级联)"""

    def __init__(self):
        self.devices: List[IOExpanderDevice] = []
        self.interrupt_line = False
        self.interrupt_priority: List[int] = []

    def add_device(self, config: IOExpanderConfig, address_offset: int = 0) -> IOExpanderDevice:
        """添加设备到总线"""
        dev = IOExpanderDevice(config, address_offset)
        self.devices.append(dev)
        return dev

    def scan_bus(self) -> List[int]:
        """扫描I2C总线, 返回在线设备地址"""
        return [dev.address for dev in self.devices]

    def get_interrupt_sources(self) -> List[int]:
        """获取中断源设备列表"""
        sources = []
        for i, dev in enumerate(self.devices):
            if dev.interrupt_pending:
                sources.append(i)
        return sources

    def cascade_read_all(self) -> Dict[int, int]:
        """级联读取所有设备GPIO"""
        result = {}
        for dev in self.devices:
            result[dev.address] = dev.i2c_read('GPIO')
        return result

    def cascade_write_all(self, pin_values: Dict[int, int]):
        """级联写入所有设备"""
        for dev in self.devices:
            if dev.address in pin_values:
                dev.i2c_write('GPIO', pin_values[dev.address])

    def get_total_io_count(self) -> int:
        """获取总IO数"""
        return sum(d.config.n_pins for d in self.devices)


def simulate_keypad(bus: IOExpanderBus, device_idx: int = 0) -> np.ndarray:
    """仿真4x4矩阵键盘扫描"""
    dev = bus.devices[device_idx]
    key_matrix = np.zeros((4, 4), dtype=bool)

    # 行扫描 (P0-P3输出)
    for row in range(4):
        # 设置当前行为低, 其余高
        row_val = ~(1 << row) & 0x0F
        col_val = 0x0F0  # P4-P7输入上拉

        # 模拟按键 (随机)
        if np.random.random() < 0.1:
            pressed_col = np.random.randint(0, 4)
            key_matrix[row, pressed_col] = True

    return key_matrix


def run_demo():
    """运行完整仿真演示"""
    print("=" * 70)
    print("IO扩展器仿真")
    print("=" * 70)

    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    # ── 1. 单设备GPIO仿真 ────────────────────────────────
    print("\n[1] 单设备GPIO仿真...")
    config = IOExpanderConfig()
    bus = IOExpanderBus()
    dev = bus.add_device(config, address_offset=0)

    # 配置: P0-P7输出, P8-P15输入
    dev.i2c_write('IODIR', 0xFF00)
    dev.i2c_write('GPPU', 0xFF00)  # 输入上拉

    # 仿真: 输出LED控制 + 输入按钮
    n_steps = 500
    time_axis = np.arange(n_steps) / config.scan_rate_hz * 1000  # ms
    output_values = np.zeros((n_steps, 8))
    input_values = np.zeros((n_steps, 8))
    gpio_snapshot = np.zeros(n_steps)

    # 输出模式: LED流水灯
    for t in range(n_steps):
        led_pattern = (1 << (t % 8))
        dev.i2c_write('GPIO', led_pattern)
        for i in range(8):
            output_values[t, i] = dev.pins[i].output_value

        # 输入: 模拟按钮按下 (P8在t=100-150, P9在t=200-280)
        for i in range(8):
            press = False
            if i == 0 and 100 <= t <= 150:
                press = True
            elif i == 1 and 200 <= t <= 280:
                press = True
            elif i == 2 and 300 <= t <= 320:
                press = True
            dev.set_pin_input(8 + i, press, t / config.scan_rate_hz)
            input_values[t, i] = dev.pins[8 + i].input_value

        dev.update(1.0 / config.scan_rate_hz)
        gpio_snapshot[t] = dev.i2c_read('GPIO')

    ax = axes[0, 0]
    for i in range(4):
        ax.plot(time_axis, output_values[:, i] + i * 1.5, label=f'LED{i}')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('输出状态 (偏移显示)')
    ax.set_title('LED流水灯输出')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ── 2. 中断仿真 ──────────────────────────────────────
    print("[2] 中断仿真...")
    config_int = IOExpanderConfig(name="MCP23017_INT")
    bus_int = IOExpanderBus()
    dev_int = bus_int.add_device(config_int)

    # 配置P0-P7为中断输入 (双边沿)
    dev_int.i2c_write('IODIR', 0x00FF)
    dev_int.i2c_write('GPINTEN', 0x00FF)

    n_int = 400
    time_int = np.arange(n_int) / config_int.scan_rate_hz * 1000
    interrupt_events = []
    pin_states_int = np.zeros((n_int, 8))
    int_flag_log = np.zeros(n_int)

    for t in range(n_int):
        # 模拟外部信号
        sig_freq = [5, 10, 20, 50, 5, 10, 15, 25]
        for i in range(8):
            val = bool(np.sin(2 * np.pi * sig_freq[i] * t / config_int.scan_rate_hz) > 0.3)
            dev_int.set_pin_input(i, val, t / config_int.scan_rate_hz)
            pin_states_int[t, i] = int(val)

        # 检测中断
        if dev_int.interrupt_pending:
            int_flag_log[t] = 1
            sources = bus_int.get_interrupt_sources()
            interrupt_events.append({
                'time': t / config_int.scan_rate_hz * 1000,
                'flags': dev_int.registers['INTF'],
                'captured': dev_int.registers['INTCAP']
            })
            # 读INTCAP清除中断
            dev_int.i2c_read('INTCAP')

    ax = axes[0, 1]
    ax.imshow(pin_states_int.T, aspect='auto', cmap='binary',
              extent=[0, time_int[-1], 0, 8], origin='lower')
    for evt in interrupt_events[:20]:
        ax.axvline(evt['time'], color='r', alpha=0.3, linewidth=0.5)
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('引脚编号')
    ax.set_title(f'输入信号与中断 (共{len(interrupt_events)}次中断)')
    ax.grid(True, alpha=0.3)

    # ── 3. 去抖动仿真 ────────────────────────────────────
    print("[3] 去抖动仿真...")
    n_debounce = 600
    config_db = IOExpanderConfig(debounce_ms=10)
    dev_db = IOExpanderDevice(config_db)

    time_db = np.arange(n_debounce) / config_db.scan_rate_hz * 1000
    raw_signal = np.zeros(n_debounce)
    debounced = np.zeros(n_debounce)
    stable = np.zeros(n_debounce)

    for t in range(n_debounce):
        # 真实信号: 按钮在t=100按下
        real_state = t >= 100
        # 弹跳: 在按下后10ms内随机跳变
        if 100 <= t <= 110:
            raw_val = bool(np.random.random() > 0.5)
        elif t > 110:
            raw_val = real_state
        else:
            raw_val = False

        raw_signal[t] = int(raw_val)
        dev_db.set_pin_input(0, raw_val, t / config_db.scan_rate_hz)
        dev_db.update(1.0 / config_db.scan_rate_hz)
        debounced[t] = int(dev_db.pins[0].input_value)
        stable[t] = int(dev_db.pins[0].last_stable)

    ax = axes[0, 2]
    ax.plot(time_db, raw_signal, 'r-', alpha=0.5, label='原始信号 (含弹跳)')
    ax.plot(time_db, stable + 1.3, 'b-', linewidth=2, label='去抖动后 (+1.3)')
    ax.plot(time_db, debounced + 2.6, 'g-', linewidth=2, label='即时值 (+2.6)')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('电平')
    ax.set_title('按钮去抖动仿真')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ── 4. 多设备级联 ────────────────────────────────────
    print("[4] 多设备级联仿真...")
    cascade_bus = IOExpanderBus()
    n_devices = 8
    for i in range(n_devices):
        cascade_bus.add_device(IOExpanderConfig(), address_offset=i)

    # 模拟8个设备各16个IO = 128个IO
    n_cascade = 200
    all_gpio = np.zeros((n_cascade, n_devices))

    for t in range(n_cascade):
        for dev_idx, dev in enumerate(cascade_bus.devices):
            # 模拟不同模式的输入
            pattern = 0
            for p in range(16):
                if dev_idx % 2 == 0:
                    val = bool((t + dev_idx * 10 + p * 5) % 20 < 10)
                else:
                    val = bool(np.sin(2 * np.pi * (t * 0.05 + dev_idx + p * 0.1)) > 0)
                dev.set_pin_input(p, val)
            dev.update(1.0 / config.scan_rate_hz)
            all_gpio[t, dev_idx] = dev.i2c_read('GPIO') or 0

    ax = axes[1, 0]
    for i in range(n_devices):
        norm = all_gpio[:, i] / (2 ** 16 - 1) if np.max(all_gpio[:, i]) > 0 else all_gpio[:, i]
        ax.plot(norm + i * 1.2, label=f'Dev {i} @0x{0x20 + i:02X}')
    ax.set_xlabel('采样点')
    ax.set_ylabel('GPIO值 (归一化+偏移)')
    ax.set_title(f'多设备级联 ({n_devices}个设备, {cascade_bus.get_total_io_count()}个IO)')
    ax.legend(fontsize=6, ncol=2)
    ax.grid(True, alpha=0.3)

    # ── 5. I2C总线时序仿真 ───────────────────────────────
    print("[5] I2C总线时序仿真...")
    # 模拟一次I2C读操作的SCL/SDA波形
    bit_rate = config.i2c_speed
    sample_rate = bit_rate * 20  # 过采样
    n_bits = 9  # 8数据 + 1 ACK

    # 地址字节: 0x20写 = 0b01000000
    address_byte = [0, 1, 0, 0, 0, 0, 0, 0, 0]  # 7bit地址 + R/W + ACK

    scl = []
    sda = []
    for bit in address_byte:
        # SCL
        scl.extend([0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        # SDA (数据在SCL低时变化, SCL高时采样)
        sda.extend([bit] * 20)

    # 添加START和STOP条件
    # START: SDA下降沿 while SCL高
    start_scl = [1] * 10 + [1] * 10
    start_sda = [1] * 10 + [0] * 10
    # STOP: SDA上升沿 while SCL高
    stop_scl = [1] * 10 + [1] * 10
    stop_sda = [0] * 10 + [1] * 10

    total_scl = start_scl + scl + stop_scl
    total_sda = start_sda + sda + stop_sda
    t_i2c = np.arange(len(total_scl)) / sample_rate * 1e6  # us

    ax = axes[1, 1]
    ax.plot(t_i2c, np.array(total_scl) * 1.5 + 2, 'b-', linewidth=1, label='SCL')
    ax.plot(t_i2c, np.array(total_sda) * 1.5, 'r-', linewidth=1, label='SDA')
    ax.set_xlabel('时间 (μs)')
    ax.set_ylabel('电平 (V)')
    ax.set_title(f'I2C总线时序 (@{bit_rate / 1e3:.0f}kHz)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-0.5, 4.5)

    # ── 6. 带宽与延迟分析 ────────────────────────────────
    print("[6] 带宽与延迟分析...")
    io_counts = [8, 16, 32, 64, 96, 128]
    latencies = []
    throughputs = []

    for n_io in io_counts:
        n_dev = max(1, n_io // 16)
        # I2C延迟: 每设备需要(1地址字节+2数据字节)×9bit/速率
        per_dev_time = 3 * 9 / config.i2c_speed  # 秒
        total_time = per_dev_time * n_dev
        latencies.append(total_time * 1000)  # ms
        throughputs.append(n_io / total_time)  # IO/s

    ax = axes[1, 2]
    ax2 = ax.twinx()
    ax.bar(range(len(io_counts)), latencies, color='steelblue', alpha=0.7, label='延迟')
    ax2.plot(range(len(io_counts)), [t / 1e6 for t in throughputs], 'ro-', label='吞吐率')
    ax.set_xticks(range(len(io_counts)))
    ax.set_xticklabels([str(n) for n in io_counts])
    ax.set_xlabel('IO总数')
    ax.set_ylabel('读取延迟 (ms)')
    ax2.set_ylabel('吞吐率 (MIO/s)')
    ax.set_title('级联IO数量 vs 性能')
    ax.legend(loc='upper left')
    ax2.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('io_expander_simulation.png', dpi=150, bbox_inches='tight')
    plt.show()

    print(f"\n仿真完成!")
    print(f"  级联设备: {n_devices}个, 总IO: {cascade_bus.get_total_io_count()}")
    print(f"  中断事件: {len(interrupt_events)}次")
    print("  图表已保存为 io_expander_simulation.png")


if __name__ == "__main__":
    run_demo()
