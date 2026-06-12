"""
SPI总线仲裁仿真 - 多设备/优先级/DMA/时序分析
nuedc-asset-library V3
"""
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from enum import Enum
from collections import deque
import time

class SPIMode(Enum):
    MODE0 = (0, 0)  # CPOL=0, CPHA=0
    MODE1 = (0, 1)  # CPOL=0, CPHA=1
    MODE2 = (1, 0)  # CPOL=1, CPHA=0
    MODE3 = (1, 1)  # CPOL=1, CPHA=1


class DevicePriority(Enum):
    CRITICAL = 0    # 最高
    HIGH = 1
    NORMAL = 2
    LOW = 3


@dataclass
class SPIDevice:
    """SPI设备模型"""
    device_id: str
    cs_pin: int
    spi_mode: SPIMode = SPIMode.MODE0
    max_clock_mhz: float = 10.0
    priority: DevicePriority = DevicePriority.NORMAL
    bits_per_word: int = 8
    dma_enabled: bool = False
    cs_setup_ns: float = 100.0   # CS建立时间
    cs_hold_ns: float = 100.0    # CS保持时间
    cs_inactive_ns: float = 200.0  # CS非活动最小时间

    @property
    def clock_period_ns(self) -> float:
        return 1000.0 / self.max_clock_mhz

    @property
    def bit_time_ns(self) -> float:
        return self.clock_period_ns


@dataclass
class SPITransaction:
    """SPI传输事务"""
    device_id: str
    tx_data: bytes
    rx_length: int = 0
    priority: DevicePriority = DevicePriority.NORMAL
    timestamp: float = 0.0
    dma: bool = False
    completed: bool = False
    start_time_ns: float = 0.0
    end_time_ns: float = 0.0


@dataclass
class BusState:
    """总线状态"""
    sck: bool = False
    mosi: bool = False
    miso: bool = False
    active_cs: Optional[int] = None
    is_busy: bool = False


class SPIBusArbitrator:
    """SPI总线仲裁器"""

    def __init__(self, devices: List[SPIDevice] = None):
        self.devices: Dict[str, SPIDevice] = {}
        if devices:
            for d in devices:
                self.devices[d.device_id] = d
        self.bus_state = BusState()
        self.transaction_queue: deque = deque()
        self.current_transaction: Optional[SPITransaction] = None
        self.time_ns: float = 0.0
        self.completed: List[SPITransaction] = []
        self.bus_utilization_ns: float = 0.0
        self.conflict_count: int = 0

    def add_device(self, device: SPIDevice):
        self.devices[device.device_id] = device

    def submit_transaction(self, txn: SPITransaction):
        txn.timestamp = self.time_ns
        self.transaction_queue.append(txn)

    def _calculate_transfer_time(self, txn: SPITransaction) -> float:
        """计算传输时间(ns)"""
        device = self.devices.get(txn.device_id)
        if not device:
            return 0

        data_bits = len(txn.tx_data) * 8
        rx_bits = txn.rx_length * 8
        total_bits = max(data_bits, rx_bits)

        transfer_ns = total_bits * device.clock_period_ns
        overhead_ns = device.cs_setup_ns + device.cs_hold_ns + device.cs_inactive_ns

        if txn.dma:
            # DMA模式减少CPU开销，但有DMA配置时间
            overhead_ns += 200  # DMA setup
        else:
            # CPU模式有中断开销
            overhead_ns += total_bits * 0.1  # 每bit CPU开销

        return transfer_ns + overhead_ns

    def _arbitrate(self) -> Optional[SPITransaction]:
        """仲裁：选择下一个传输"""
        if not self.transaction_queue:
            return None

        # 按优先级排序，同优先级按时间排序
        sorted_queue = sorted(
            self.transaction_queue,
            key=lambda t: (t.priority.value, t.timestamp)
        )

        winner = sorted_queue[0]
        self.transaction_queue.remove(winner)

        # 检测冲突
        if len(sorted_queue) > 1:
            same_priority = sum(1 for t in sorted_queue
                               if t.priority == winner.priority)
            if same_priority > 1:
                self.conflict_count += 1

        return winner

    def simulate_tick(self) -> bool:
        """仿真一个时间步"""
        if self.current_transaction is None or self.current_transaction.completed:
            txn = self._arbitrate()
            if txn is None:
                return False
            self.current_transaction = txn
            txn.start_time_ns = self.time_ns

            device = self.devices[txn.device_id]
            transfer_ns = self._calculate_transfer_time(txn)
            txn.end_time_ns = self.time_ns + transfer_ns
            txn.completed = True

            self.bus_state.active_cs = device.cs_pin
            self.bus_state.is_busy = True

            self.completed.append(txn)
            self.bus_utilization_ns += transfer_ns
            self.time_ns = txn.end_time_ns
        return True

    def run_simulation(self) -> dict:
        """运行完整仿真"""
        start = time.perf_counter()
        while self.simulate_tick():
            pass
        elapsed = time.perf_counter() - start

        return self._analyze(elapsed)

    def _analyze(self, elapsed_s: float) -> dict:
        if not self.completed:
            return {"transactions": 0}

        times = [t.end_time_ns - t.start_time_ns for t in self.completed]
        priorities = {}
        for t in self.completed:
            p = t.priority.value
            priorities.setdefault(p, []).append(t.end_time_ns - t.start_time_ns)

        return {
            "total_transactions": len(self.completed),
            "total_time_ns": self.time_ns,
            "total_time_ms": self.time_ns / 1e6,
            "avg_transfer_ns": np.mean(times),
            "max_transfer_ns": max(times),
            "min_transfer_ns": min(times),
            "bus_utilization_pct": (self.bus_utilization_ns / self.time_ns * 100) if self.time_ns > 0 else 0,
            "conflict_count": self.conflict_count,
            "priority_breakdown": {
                f"priority_{p}": {
                    "count": len(ts),
                    "avg_ns": np.mean(ts),
                    "max_ns": max(ts),
                }
                for p, ts in priorities.items()
            },
            "dma_vs_cpu": {
                "dma_count": sum(1 for t in self.completed if t.dma),
                "cpu_count": sum(1 for t in self.completed if not t.dma),
            },
        }

    def generate_waveform(self, txn_idx: int = 0, samples_per_bit: int = 4) -> dict:
        """生成SPI波形数据"""
        if txn_idx >= len(self.completed):
            return {}

        txn = self.completed[txn_idx]
        device = self.devices[txn.device_id]
        tx_bytes = txn.tx_data

        cpol = device.spi_mode.value[0]
        cpha = device.spi_mode.value[1]

        sck_wave = []
        mosi_wave = []
        cs_wave = []

        # CS低电平选中
        cs_wave.extend([1] * samples_per_bit)  # CS setup
        cs_wave.extend([0] * len(tx_bytes) * 8 * samples_per_bit)

        for byte_val in tx_bytes:
            for bit_idx in range(8):
                bit = (byte_val >> (7 - bit_idx)) & 1
                if cpha == 0:
                    sck_wave.extend([cpol] * samples_per_bit)
                    mosi_wave.extend([bit] * samples_per_bit)
                    sck_wave.extend([1 - cpol] * samples_per_bit)
                    mosi_wave.extend([bit] * samples_per_bit)
                else:
                    sck_wave.extend([1 - cpol] * samples_per_bit)
                    mosi_wave.extend([bit] * samples_per_bit)
                    sck_wave.extend([cpol] * samples_per_bit)
                    mosi_wave.extend([bit] * samples_per_bit)

        cs_wave.extend([1] * samples_per_bit * 2)  # CS release

        return {
            "sck": sck_wave,
            "mosi": mosi_wave,
            "cs": cs_wave,
            "device": txn.device_id,
            "spi_mode": device.spi_mode.name,
            "data_hex": tx_bytes.hex(),
        }


class DMAController:
    """DMA控制器模型"""

    def __init__(self, channels: int = 4, bus_width: int = 32, clock_mhz: float = 100):
        self.channels = channels
        self.bus_width = bus_width
        self.clock_mhz = clock_mhz
        self.active_channels: Dict[int, dict] = {}

    def configure_transfer(self, channel: int, src_addr: int, dst_addr: int,
                          size_bytes: int, priority: int = 0) -> dict:
        """配置DMA传输"""
        # SPI外设通常用外设到内存或内存到外设
        transfer_time_ns = size_bytes / (self.bus_width / 8) / (self.clock_mhz * 1e6) * 1e9

        self.active_channels[channel] = {
            "src": src_addr,
            "dst": dst_addr,
            "size": size_bytes,
            "priority": priority,
            "transfer_time_ns": transfer_time_ns,
        }
        return self.active_channels[channel]

    def compare_with_cpu(self, data_size: int) -> dict:
        """DMA vs CPU传输对比"""
        dma_ns = data_size / (self.bus_width / 8) / (self.clock_mhz * 1e6) * 1e9
        cpu_ns = data_size * 8 * 0.5  # 假设每bit 0.5ns CPU开销
        return {
            "data_size": data_size,
            "dma_time_ns": dma_ns,
            "cpu_time_ns": cpu_ns,
            "speedup": cpu_ns / dma_ns if dma_ns > 0 else 0,
            "cpu_overhead_saved_pct": (1 - dma_ns / cpu_ns) * 100 if cpu_ns > 0 else 0,
        }


# ── 演示 ──
def demo():
    print("=" * 60)
    print("SPI总线仲裁仿真 - Demo")
    print("=" * 60)

    # 1. 多设备仲裁
    devices = [
        SPIDevice("FLASH", cs_pin=0, max_clock_mhz=20, priority=DevicePriority.HIGH, dma_enabled=True),
        SPIDevice("ADC", cs_pin=1, max_clock_mhz=10, priority=DevicePriority.CRITICAL),
        SPIDevice("DAC", cs_pin=2, max_clock_mhz=5, priority=DevicePriority.NORMAL),
        SPIDevice("LCD", cs_pin=3, max_clock_mhz=40, priority=DevicePriority.LOW, dma_enabled=True),
        SPIDevice("SENSOR", cs_pin=4, max_clock_mhz=1, priority=DevicePriority.NORMAL),
    ]

    arbitrator = SPIBusArbitrator(devices)

    # 提交事务
    test_data = [
        ("FLASH", b'\x03\x00\x00\x00' + bytes(256), 256, DevicePriority.HIGH, True),
        ("ADC", b'\x01\x80', 2, DevicePriority.CRITICAL, False),
        ("DAC", b'\x02\xFF\x80', 0, DevicePriority.NORMAL, False),
        ("LCD", b'\x2C' + bytes(320*2), 0, DevicePriority.LOW, True),
        ("SENSOR", b'\x9F', 3, DevicePriority.NORMAL, False),
    ]

    for dev_id, tx, rx_len, pri, dma in test_data:
        arbitrator.submit_transaction(
            SPITransaction(dev_id, tx, rx_len, pri, dma=dma)
        )

    # 多轮仿真
    for _ in range(20):
        for dev_id, tx, rx_len, pri, dma in test_data:
            arbitrator.submit_transaction(
                SPITransaction(dev_id, tx, rx_len, pri, dma=dma)
            )

    result = arbitrator.run_simulation()

    print(f"\n[仲裁结果]")
    print(f"  总事务: {result['total_transactions']}")
    print(f"  总时间: {result['total_time_ms']:.3f} ms")
    print(f"  平均传输: {result['avg_transfer_ns']:.0f} ns")
    print(f"  总线利用率: {result['bus_utilization_pct']:.1f}%")
    print(f"  仲裁冲突: {result['conflict_count']}")

    print(f"\n[优先级分析]")
    for p, info in result['priority_breakdown'].items():
        print(f"  优先级{p}: {info['count']}次, 平均{info['avg_ns']:.0f}ns")

    # 2. 波形生成
    print(f"\n[波形生成]")
    waveform = arbitrator.generate_waveform(0)
    if waveform:
        print(f"  设备: {waveform['device']}")
        print(f"  SPI模式: {waveform['spi_mode']}")
        print(f"  数据: {waveform['data_hex'][:20]}...")
        print(f"  SCK采样点: {len(waveform['sck'])}")

    # 3. DMA对比
    print(f"\n[DMA vs CPU对比]")
    dma = DMAController()
    for size in [64, 256, 1024, 4096]:
        cmp = dma.compare_with_cpu(size)
        print(f"  {size}B: DMA={cmp['dma_time_ns']:.0f}ns, CPU={cmp['cpu_time_ns']:.0f}ns, "
              f"加速{cmp['speedup']:.1f}x")

    # 4. 时序分析
    print(f"\n[时序参数]")
    for dev in devices:
        print(f"  {dev.device_id}: {dev.max_clock_mhz}MHz, "
              f"CS_setup={dev.cs_setup_ns}ns, CS_hold={dev.cs_hold_ns}ns, "
              f"周期={dev.clock_period_ns:.0f}ns")

    print("\n✅ SPI总线仲裁仿真完成")


if __name__ == "__main__":
    demo()
