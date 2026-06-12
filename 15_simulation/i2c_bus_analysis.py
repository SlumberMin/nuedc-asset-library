"""
I2C总线分析仿真 - 时序/波形/多主机/仲裁/错误注入
nuedc-asset-library V3
"""
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from enum import Enum
from collections import deque
import random
import time

class I2CSpeedMode(Enum):
    STANDARD = 100000      # 100kHz
    FAST = 400000          # 400kHz
    FAST_PLUS = 1000000    # 1MHz
    HIGH_SPEED = 3400000   # 3.4MHz

class I2CState(Enum):
    IDLE = "idle"
    START = "start"
    ADDRESS = "address"
    ACK_ADDR = "ack_address"
    DATA = "data"
    ACK_DATA = "ack_data"
    NACK = "nack"
    STOP = "stop"
    ARBITRATION_LOST = "arb_lost"
    BUS_ERROR = "bus_error"

class ErrorType(Enum):
    NONE = "none"
    NACK = "nack"
    ARBITRATION_LOST = "arbitration_lost"
    BUS_BUSY = "bus_busy"
    CLOCK_STRETCH = "clock_stretch"
    NOISE = "noise"
    SDA_STUCK = "sda_stuck"
    SCL_STUCK = "scl_stuck"


@dataclass
class I2CDevice:
    """I2C设备"""
    name: str
    address: int           # 7位地址
    is_master: bool = False
    clock_stretch_us: float = 0.0  # 时钟拉伸时间
    response_nack: bool = False    # 是否NACK
    data_to_send: bytes = b''
    received_data: bytes = b''

    @property
    def address_7bit(self) -> int:
        return self.address & 0x7F

    @property
    def address_byte_write(self) -> int:
        return (self.address << 1) | 0

    @property
    def address_byte_read(self) -> int:
        return (self.address << 1) | 1


@dataclass
class I2CTransaction:
    """I2C事务"""
    master: str
    slave_addr: int
    is_read: bool
    data: bytes = b''
    timestamp: float = 0.0
    result: I2CState = I2CState.IDLE
    error: ErrorType = ErrorType.NONE
    duration_us: float = 0.0
    clock_stretch_total_us: float = 0.0


@dataclass
class I2CBusConfig:
    speed_mode: I2CSpeedMode = I2CSpeedMode.FAST
    pullup_ohm: float = 4700.0       # 上拉电阻
    bus_capacitance_pf: float = 100.0  # 总线电容
    voltage: float = 3.3
    sda_rise_time_ns: float = 0.0     # 自动计算
    scl_rise_time_ns: float = 0.0

    def __post_init__(self):
        # RC上升时间估算: t_r = 2.2 * R * C
        self.sda_rise_time_ns = 2.2 * self.pullup_ohm * self.bus_capacitance_pf * 1e-3
        self.scl_rise_time_ns = self.sda_rise_time_ns

    @property
    def clock_period_ns(self) -> float:
        return 1e9 / self.speed_mode.value

    @property
    def bit_time_us(self) -> float:
        return 1e6 / self.speed_mode.value

    @property
    def max_rise_time_ns(self) -> float:
        """I2C规范最大上升时间"""
        limits = {
            I2CSpeedMode.STANDARD: 1000,
            I2CSpeedMode.FAST: 300,
            I2CSpeedMode.FAST_PLUS: 120,
        }
        return limits.get(self.speed_mode, 300)


class I2CBusAnalyzer:
    """I2C总线分析仿真器"""

    def __init__(self, config: I2CBusConfig = None):
        self.config = config or I2CBusConfig()
        self.devices: Dict[str, I2CDevice] = {}
        self.sda = True  # 高电平 = 释放
        self.scl = True
        self.bus_state = I2CState.IDLE
        self.time_us: float = 0.0
        self.transactions: List[I2CTransaction] = []
        self.waveform_sda: List[Tuple[float, bool]] = []
        self.waveform_scl: List[Tuple[float, bool]] = []
        self.error_log: List[dict] = []
        self.arbitration_events: List[dict] = []
        self.error_injection: Dict[str, float] = {}  # error_type -> probability

    def add_device(self, device: I2CDevice):
        self.devices[device.name] = device

    def inject_error(self, error_type: ErrorType, probability: float):
        """注入错误"""
        self.error_injection[error_type.value] = probability

    def _check_error_injection(self) -> Optional[ErrorType]:
        for err_name, prob in self.error_injection.items():
            if random.random() < prob:
                return ErrorType(err_name)
        return None

    def _record_waveform(self, us: float):
        self.waveform_sda.append((us, self.sda))
        self.waveform_scl.append((us, self.scl))

    def _generate_start(self):
        """产生起始条件: SDA下降沿 while SCL高"""
        self.sda = True; self.scl = True
        self._record_waveform(self.time_us)
        self.time_us += self.config.bit_time_us * 0.5
        self.sda = False  # SDA下降
        self._record_waveform(self.time_us)
        self.time_us += self.config.bit_time_us * 0.5
        self.scl = False
        self._record_waveform(self.time_us)
        self.bus_state = I2CState.START

    def _generate_stop(self):
        """产生停止条件: SDA上升沿 while SCL高"""
        self.scl = False; self.sda = False
        self._record_waveform(self.time_us)
        self.time_us += self.config.bit_time_us * 0.5
        self.scl = True
        self._record_waveform(self.time_us)
        self.time_us += self.config.bit_time_us * 0.25
        self.sda = True  # SDA上升
        self._record_waveform(self.time_us)
        self.time_us += self.config.bit_time_us * 0.25
        self.bus_state = I2CState.STOP

    def _write_byte(self, byte_val: int) -> bool:
        """写一个字节，返回ACK"""
        for bit_idx in range(7, -1, -1):
            bit = (byte_val >> bit_idx) & 1
            self.scl = False
            self.sda = bool(not bit)  # I2C: 低=0, 高=1(释放)
            self._record_waveform(self.time_us)
            self.time_us += self.config.bit_time_us * 0.5
            self.scl = True
            self._record_waveform(self.time_us)
            self.time_us += self.config.bit_time_us * 0.5
        # ACK bit (slave pulls SDA low)
        self.scl = False
        self.sda = True  # 释放SDA等待ACK
        self._record_waveform(self.time_us)
        self.time_us += self.config.bit_time_us * 0.5
        self.scl = True
        ack = True  # 模拟ACK
        self._record_waveform(self.time_us)
        self.time_us += self.config.bit_time_us * 0.5
        return ack

    def _read_byte(self, ack: bool = True) -> int:
        """读一个字节"""
        byte_val = 0
        for bit_idx in range(7, -1, -1):
            self.scl = False
            self.sda = True  # 释放
            self._record_waveform(self.time_us)
            self.time_us += self.config.bit_time_us * 0.5
            self.scl = True
            bit = random.randint(0, 1)  # 模拟数据
            byte_val |= (bit << bit_idx)
            self._record_waveform(self.time_us)
            self.time_us += self.config.bit_time_us * 0.5
        # ACK/NACK
        self.scl = False
        self.sda = not ack  # ACK=低, NACK=高
        self._record_waveform(self.time_us)
        self.time_us += self.config.bit_time_us * 0.5
        self.scl = True
        self._record_waveform(self.time_us)
        self.time_us += self.config.bit_time_us * 0.5
        return byte_val

    def _arbitrate_masters(self, masters: List[str]) -> Optional[str]:
        """多主机仲裁仿真"""
        if len(masters) <= 1:
            return masters[0] if masters else None

        # 模拟仲裁：每个主机发送地址，SDA上"线与"逻辑
        # 拥有更多低电平位（地址值更小）的主机获胜
        winner = min(masters, key=lambda m: self.devices[m].address)
        losers = [m for m in masters if m != winner]

        for loser in losers:
            self.arbitration_events.append({
                "time_us": self.time_us,
                "winner": winner,
                "loser": loser,
                "reason": "arbitration_lost",
            })

        return winner

    def execute_transaction(self, master_name: str, slave_addr: int,
                           is_read: bool, data: bytes = b'') -> I2CTransaction:
        """执行一次I2C事务"""
        start_us = self.time_us
        txn = I2CTransaction(
            master=master_name,
            slave_addr=slave_addr,
            is_read=is_read,
            data=data,
            timestamp=start_us,
        )

        # 错误注入检查
        err = self._check_error_injection()
        if err == ErrorType.BUS_BUSY:
            txn.result = I2CState.BUS_ERROR
            txn.error = err
            txn.duration_us = self.time_us - start_us
            self.error_log.append({"time": start_us, "error": err.value, "txn": master_name})
            self.transactions.append(txn)
            return txn

        # START
        self._generate_start()
        txn.result = I2CState.START

        # Address phase
        addr_byte = (slave_addr << 1) | (1 if is_read else 0)
        ack = self._write_byte(addr_byte)

        if err == ErrorType.NACK or not ack:
            txn.result = I2CState.NACK
            txn.error = ErrorType.NACK
            self._generate_stop()
            txn.duration_us = self.time_us - start_us
            self.error_log.append({"time": start_us, "error": "nack_address", "addr": hex(slave_addr)})
            self.transactions.append(txn)
            return txn

        # Clock stretch simulation
        slave = None
        for d in self.devices.values():
            if d.address == slave_addr:
                slave = d
                break
        if slave and slave.clock_stretch_us > 0:
            self.time_us += slave.clock_stretch_us
            txn.clock_stretch_total_us += slave.clock_stretch_us

        # Data phase
        if is_read:
            # Read: slave sends data
            for i in range(len(data) if data else 1):
                byte_val = self._read_byte(ack=(i < (len(data) if data else 1) - 1))
        else:
            # Write: master sends data
            for i, byte_val in enumerate(data):
                ack = self._write_byte(byte_val)
                if err == ErrorType.NACK:
                    txn.result = I2CState.NACK
                    txn.error = err
                    break

        # STOP
        self._generate_stop()

        txn.result = I2CState.STOP if txn.error == ErrorType.NONE else txn.result
        txn.duration_us = self.time_us - start_us
        self.transactions.append(txn)
        return txn

    def analyze_timing(self) -> dict:
        """时序分析"""
        if not self.transactions:
            return {}

        durations = [t.duration_us for t in self.transactions]
        stretch = [t.clock_stretch_total_us for t in self.transactions]

        return {
            "total_transactions": len(self.transactions),
            "total_time_us": self.time_us,
            "avg_duration_us": np.mean(durations),
            "max_duration_us": max(durations),
            "min_duration_us": min(durations),
            "total_clock_stretch_us": sum(stretch),
            "speed_mode": self.config.speed_mode.name,
            "effective_speed_hz": sum(len(t.data) for t in self.transactions) * 8 / (self.time_us / 1e6) if self.time_us > 0 else 0,
        }

    def analyze_errors(self) -> dict:
        """错误分析"""
        error_counts = {}
        for e in self.error_log:
            err_type = e["error"]
            error_counts[err_type] = error_counts.get(err_type, 0) + 1

        result_states = {}
        for t in self.transactions:
            state = t.result.value
            result_states[state] = result_states.get(state, 0) + 1

        return {
            "total_errors": len(self.error_log),
            "error_rate_pct": len(self.error_log) / len(self.transactions) * 100 if self.transactions else 0,
            "error_breakdown": error_counts,
            "result_breakdown": result_states,
            "arbitration_events": len(self.arbitration_events),
        }

    def analyze_arbitration(self) -> dict:
        """多主机仲裁分析"""
        masters = [d for d in self.devices.values() if d.is_master]
        if len(masters) < 2:
            return {"info": "仅1个主机，无仲裁"}

        # 模拟仲裁
        results = {}
        for master in masters:
            wins = 0
            losses = 0
            for other in masters:
                if master.name != other.name:
                    if master.address < other.address:
                        wins += 1
                    else:
                        losses += 1
            results[master.name] = {"address": hex(master.address), "wins": wins, "losses": losses}

        return {
            "num_masters": len(masters),
            "arbitration_results": results,
            "arbitration_events": len(self.arbitration_events),
        }

    def generate_waveform_text(self, max_bits: int = 50) -> str:
        """生成文本波形图"""
        if not self.waveform_sda:
            return "无波形数据"

        lines = ["时间(us)  SCL  SDA  状态", "-" * 40]
        for i, ((t, sda), (_, scl)) in enumerate(
            zip(self.waveform_sda[:max_bits], self.waveform_scl[:max_bits])
        ):
            scl_ch = "‾" if scl else "_"
            sda_ch = "‾" if sda else "_"
            lines.append(f"{t:8.1f}  {scl_ch}    {sda_ch}")
        return "\n".join(lines)

    def electrical_analysis(self) -> dict:
        """电气特性分析"""
        cfg = self.config
        # 电流估算
        iol = cfg.voltage / cfg.pullup_ohm * 1000  # mA

        # RC时间常数
        tau_ns = cfg.pullup_ohm * cfg.bus_capacitance_pf * 1e-3  # ns

        # 最大总线电容
        max_cap_pf = {
            I2CSpeedMode.STANDARD: 400,
            I2CSpeedMode.FAST: 400,
            I2CSpeedMode.FAST_PLUS: 550,
        }

        return {
            "pullup_resistor_ohm": cfg.pullup_ohm,
            "bus_capacitance_pf": cfg.bus_capacitance_pf,
            "voltage_v": cfg.voltage,
            "low_level_current_mA": iol,
            "rc_time_constant_ns": tau_ns,
            "rise_time_ns": cfg.sda_rise_time_ns,
            "max_rise_time_ns": cfg.max_rise_time_ns,
            "rise_time_ok": cfg.sda_rise_time_ns < cfg.max_rise_time_ns,
            "max_bus_capacitance_pf": max_cap_pf.get(cfg.speed_mode, 400),
            "capacitance_ok": cfg.bus_capacitance_pf <= max_cap_pf.get(cfg.speed_mode, 400),
        }


# ── 演示 ──
def demo():
    print("=" * 60)
    print("I2C总线分析仿真 - Demo")
    print("=" * 60)

    # 1. 基本通信
    analyzer = I2CBusAnalyzer(I2CBusConfig(I2CSpeedMode.FAST))
    analyzer.add_device(I2CDevice("Master1", 0x00, is_master=True))
    analyzer.add_device(I2CDevice("EEPROM", 0x50))
    analyzer.add_device(I2CDevice("OLED", 0x3C))
    analyzer.add_device(I2CDevice("IMU", 0x68))

    print("\n[基本通信]")
    transactions = [
        ("Master1", 0x50, False, b'\x00\x10\x48\x65\x6C\x6C\x6F'),  # EEPROM写
        ("Master1", 0x50, True, b'\x00\x00' + bytes(5)),              # EEPROM读
        ("Master1", 0x3C, False, b'\x00\xAF'),                         # OLED命令
        ("Master1", 0x68, True, bytes(14)),                            # IMU读
    ]
    for master, addr, read, data in transactions:
        txn = analyzer.execute_transaction(master, addr, read, data)
        print(f"  {master}->0x{addr:02X} {'R' if read else 'W'} {len(data)}B: "
              f"{txn.result.value}, {txn.duration_us:.1f}us")

    timing = analyzer.analyze_timing()
    print(f"\n  总事务: {timing['total_transactions']}")
    print(f"  总时间: {timing['total_time_us']:.1f}us")
    print(f"  平均时延: {timing['avg_duration_us']:.1f}us")

    # 2. 多主机仲裁
    print("\n[多主机仲裁]")
    arb_analyzer = I2CBusAnalyzer(I2CBusConfig(I2CSpeedMode.FAST))
    arb_analyzer.add_device(I2CDevice("MCU_A", 0x01, is_master=True))
    arb_analyzer.add_device(I2CDevice("MCU_B", 0x02, is_master=True))
    arb_analyzer.add_device(I2CDevice("Sensor", 0x48))

    arb_result = arb_analyzer.analyze_arbitration()
    print(f"  主机数: {arb_result['num_masters']}")
    for name, info in arb_result['arbitration_results'].items():
        print(f"  {name} (addr={info['address']}): 赢{info['wins']}次, 输{info['losses']}次")

    # 3. 错误注入
    print("\n[错误注入]")
    err_analyzer = I2CBusAnalyzer(I2CBusConfig(I2CSpeedMode.FAST))
    err_analyzer.add_device(I2CDevice("Master", 0x00, is_master=True))
    err_analyzer.add_device(I2CDevice("Slave", 0x50, response_nack=True))
    err_analyzer.inject_error(ErrorType.NACK, 0.3)

    for _ in range(30):
        err_analyzer.execute_transaction("Master", 0x50, False, b'\x00\x01\x02')

    err_result = err_analyzer.analyze_errors()
    print(f"  总错误: {err_result['total_errors']}")
    print(f"  错误率: {err_result['error_rate_pct']:.1f}%")
    print(f"  错误分布: {err_result['error_breakdown']}")
    print(f"  结果分布: {err_result['result_breakdown']}")

    # 4. 电气特性
    print("\n[电气特性]")
    for mode in I2CSpeedMode:
        elec = I2CBusAnalyzer(I2CBusConfig(mode)).electrical_analysis()
        print(f"  {mode.name}: 上升时间={elec['rise_time_ns']:.0f}ns "
              f"({'OK' if elec['rise_time_ok'] else 'FAIL'}), "
              f"电流={elec['low_level_current_mA']:.2f}mA")

    # 5. 波形
    print("\n[波形片段]")
    print(analyzer.generate_waveform_text(20))

    # 6. 时钟拉伸
    print("\n[时钟拉伸]")
    stretch_analyzer = I2CBusAnalyzer(I2CBusConfig(I2CSpeedMode.FAST))
    stretch_analyzer.add_device(I2CDevice("Master", 0x00, is_master=True))
    stretch_analyzer.add_device(I2CDevice("SlowSlave", 0x50, clock_stretch_us=100))
    txn = stretch_analyzer.execute_transaction("Master", 0x50, False, b'\x00\x01')
    print(f"  时钟拉伸: {txn.clock_stretch_total_us:.0f}us")
    print(f"  总时间: {txn.duration_us:.1f}us")

    print("\n✅ I2C总线分析仿真完成")


if __name__ == "__main__":
    demo()
