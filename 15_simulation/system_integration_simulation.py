#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
系统集成仿真 - 多模块协同/接口测试/性能分析
模拟嵌入式系统多模块集成、接口测试与整体性能分析
"""

import time
import random
import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Dict, Optional, Callable, Any, Tuple
from collections import deque


class InterfaceType(Enum):
    SPI = auto()
    I2C = auto()
    UART = auto()
    GPIO = auto()
    ADC = auto()
    DAC = auto()
    PWM = auto()
    DMA = auto()
    CAN = auto()
    USB = auto()


class ModuleState(Enum):
    IDLE = auto()
    INITIALIZING = auto()
    READY = auto()
    RUNNING = auto()
    ERROR = auto()
    SUSPENDED = auto()


class TestResult(Enum):
    PASS = auto()
    FAIL = auto()
    SKIP = auto()
    TIMEOUT = auto()
    ERROR = auto()


class MessageType(Enum):
    COMMAND = auto()
    DATA = auto()
    STATUS = auto()
    ERROR = auto()
    HEARTBEAT = auto()
    ACK = auto()


@dataclass
class InterfaceSpec:
    """接口规格"""
    name: str
    interface_type: InterfaceType
    clock_hz: int = 0
    data_bits: int = 8
    max_bandwidth_bps: int = 0
    latency_us: float = 0
    error_rate: float = 0.0


@dataclass
class Message:
    """模块间消息"""
    msg_id: int
    msg_type: MessageType
    source: str
    target: str
    payload: Any = None
    timestamp: float = 0
    priority: int = 0


@dataclass
class TestCase:
    """测试用例"""
    test_id: str
    name: str
    description: str
    module: str
    test_fn: str
    result: TestResult = TestResult.SKIP
    duration_ms: float = 0
    details: str = ""


@dataclass
class PerformanceMetrics:
    """性能指标"""
    cpu_usage: float = 0
    memory_usage: float = 0
    throughput: float = 0       # 事务/秒
    latency_avg_ms: float = 0
    latency_max_ms: float = 0
    error_rate: float = 0
    power_mw: float = 0
    bandwidth_utilization: float = 0


class SystemModule:
    """系统模块"""

    def __init__(self, name: str, interfaces: List[InterfaceSpec] = None):
        self.name = name
        self.state = ModuleState.IDLE
        self.interfaces: Dict[str, InterfaceSpec] = {}
        self.input_buffer: deque = deque(maxlen=1000)
        self.output_buffer: deque = deque(maxlen=1000)
        self.error_count = 0
        self.process_count = 0
        self.init_time_ms = 0
        self.process_time_ms = 0
        self.dependencies: List[str] = []
        self.config: Dict = {}

        for iface in (interfaces or []):
            self.interfaces[iface.name] = iface

    def initialize(self) -> bool:
        self.state = ModuleState.INITIALIZING
        self.init_time_ms = random.uniform(10, 500)
        if random.random() < 0.95:
            self.state = ModuleState.READY
            return True
        self.state = ModuleState.ERROR
        self.error_count += 1
        return False

    def start(self) -> bool:
        if self.state in (ModuleState.READY, ModuleState.IDLE):
            self.state = ModuleState.RUNNING
            return True
        return False

    def stop(self):
        self.state = ModuleState.IDLE

    def send(self, message: Message) -> bool:
        self.output_buffer.append(message)
        return True

    def receive(self) -> Optional[Message]:
        if self.input_buffer:
            return self.input_buffer.popleft()
        return None

    def process(self, dt: float) -> dict:
        if self.state != ModuleState.RUNNING:
            return {"status": "not_running"}

        t0 = time.time()
        processed = 0
        errors = 0

        while self.input_buffer:
            msg = self.input_buffer.popleft()
            if random.random() < 0.02:  # 2% 错误率
                errors += 1
                self.error_count += 1
            else:
                processed += 1
            self.process_count += 1

        self.process_time_ms = (time.time() - t0) * 1000
        return {"processed": processed, "errors": errors, "time_ms": self.process_time_ms}


class MessageBus:
    """消息总线"""

    def __init__(self):
        self.modules: Dict[str, SystemModule] = {}
        self.message_log: List[Message] = []
        self.msg_counter = 0
        self.total_transmitted = 0
        self.total_errors = 0

    def register_module(self, module: SystemModule):
        self.modules[module.name] = module

    def send_message(self, source: str, target: str, msg_type: MessageType,
                     payload: Any = None, priority: int = 0) -> bool:
        self.msg_counter += 1
        msg = Message(self.msg_counter, msg_type, source, target, payload, time.time(), priority)
        self.message_log.append(msg)
        self.total_transmitted += 1

        target_mod = self.modules.get(target)
        if target_mod and target_mod.state == ModuleState.RUNNING:
            target_mod.input_buffer.append(msg)
            return True
        self.total_errors += 1
        return False

    def broadcast(self, source: str, msg_type: MessageType, payload: Any = None):
        for name, mod in self.modules.items():
            if name != source:
                self.send_message(source, name, msg_type, payload)

    def get_stats(self) -> dict:
        return {
            "modules": len(self.modules),
            "total_messages": self.total_transmitted,
            "errors": self.total_errors,
            "log_size": len(self.message_log),
        }


class InterfaceTester:
    """接口测试器"""

    def __init__(self):
        self.test_cases: List[TestCase] = []
        self.results: List[TestCase] = []

    def add_test(self, test_id: str, name: str, desc: str, module: str, test_fn: str):
        self.test_cases.append(TestCase(test_id, name, desc, module, test_fn))

    def run_spi_test(self, module: SystemModule, iface: InterfaceSpec) -> TestCase:
        tc = TestCase("SPI-001", "SPI通信测试", "验证SPI数据传输", module.name, "spi_test")
        t0 = time.time()
        errors = 0
        for _ in range(100):
            if random.random() < iface.error_rate:
                errors += 1
        tc.duration_ms = (time.time() - t0) * 1000 + random.uniform(1, 10)
        tc.result = TestResult.PASS if errors == 0 else TestResult.FAIL
        tc.details = f"传输100帧, 错误={errors}, 延迟={iface.latency_us}us"
        return tc

    def run_i2c_test(self, module: SystemModule, iface: InterfaceSpec) -> TestCase:
        tc = TestCase("I2C-001", "I2C通信测试", "验证I2C读写", module.name, "i2c_test")
        t0 = time.time()
        ack_failures = 0
        for addr in range(0x48, 0x50):
            if random.random() < iface.error_rate:
                ack_failures += 1
        tc.duration_ms = (time.time() - t0) * 1000 + random.uniform(0.5, 5)
        tc.result = TestResult.PASS if ack_failures == 0 else TestResult.FAIL
        tc.details = f"扫描地址0x48-0x4F, NACK={ack_failures}"
        return tc

    def run_uart_test(self, module: SystemModule, iface: InterfaceSpec) -> TestCase:
        tc = TestCase("UART-001", "UART通信测试", "验证UART收发", module.name, "uart_test")
        sent = 1000
        errors = sum(1 for _ in range(sent) if random.random() < iface.error_rate)
        ber = errors / max(sent, 1)
        tc.duration_ms = sent * iface.data_bits / max(iface.max_bandwidth_bps, 1) * 1000 + random.uniform(1, 5)
        tc.result = TestResult.PASS if ber < 0.01 else TestResult.FAIL
        tc.details = f"发送{sent}字节, 误码率={ber:.4f}"
        return tc

    def run_gpio_test(self, module: SystemModule, iface: InterfaceSpec) -> TestCase:
        tc = TestCase("GPIO-001", "GPIO测试", "验证GPIO输入输出", module.name, "gpio_test")
        tc.duration_ms = random.uniform(0.1, 2)
        tc.result = TestResult.PASS
        tc.details = "8路GPIO输出翻转正常, 输入读取一致"
        return tc

    def run_adc_test(self, module: SystemModule, iface: InterfaceSpec) -> TestCase:
        tc = TestCase("ADC-001", "ADC精度测试", "验证ADC采样精度", module.name, "adc_test")
        # 模拟ADC精度测试
        inl = random.uniform(0.5, 2.0)  # INL (LSB)
        dnl = random.uniform(0.3, 1.5)  # DNL (LSB)
        tc.duration_ms = random.uniform(5, 20)
        tc.result = TestResult.PASS if inl < 2.0 and dnl < 1.0 else TestResult.FAIL
        tc.details = f"INL={inl:.2f}LSB, DNL={dnl:.2f}LSB, 12bit"
        return tc

    def run_all_interface_tests(self, module: SystemModule) -> List[TestCase]:
        results = []
        for name, iface in module.interfaces.items():
            if iface.interface_type == InterfaceType.SPI:
                results.append(self.run_spi_test(module, iface))
            elif iface.interface_type == InterfaceType.I2C:
                results.append(self.run_i2c_test(module, iface))
            elif iface.interface_type == InterfaceType.UART:
                results.append(self.run_uart_test(module, iface))
            elif iface.interface_type == InterfaceType.GPIO:
                results.append(self.run_gpio_test(module, iface))
            elif iface.interface_type == InterfaceType.ADC:
                results.append(self.run_adc_test(module, iface))
        self.results.extend(results)
        return results


class PerformanceProfiler:
    """性能分析器"""

    def __init__(self):
        self.metrics_history: List[Tuple[float, PerformanceMetrics]] = []
        self.module_metrics: Dict[str, List[dict]] = {}

    def measure(self, modules: Dict[str, SystemModule], bus: MessageBus) -> PerformanceMetrics:
        m = PerformanceMetrics()
        active = [mod for mod in modules.values() if mod.state == ModuleState.RUNNING]
        if not active:
            return m

        m.cpu_usage = min(100, sum(15 + random.gauss(0, 3) for _ in active))
        m.memory_usage = sum(random.uniform(10, 50) for _ in active)
        m.throughput = sum(mod.process_count for mod in active) / max(sum(mod.process_time_ms for mod in active) / 1000, 0.001)
        m.latency_avg_ms = sum(mod.process_time_ms for mod in active) / len(active)
        m.latency_max_ms = max(mod.process_time_ms for mod in active)
        total_processed = sum(mod.process_count for mod in active)
        total_errors = sum(mod.error_count for mod in active)
        m.error_rate = total_errors / max(total_processed, 1)
        m.power_mw = 50 + m.cpu_usage * 2 + len(active) * 10
        m.bandwidth_utilization = bus.total_transmitted / max(10000, 1) * 100

        self.metrics_history.append((time.time(), m))
        return m

    def get_summary(self) -> dict:
        if not self.metrics_history:
            return {}
        cpu = [m.cpu_usage for _, m in self.metrics_history]
        mem = [m.memory_usage for _, m in self.metrics_history]
        lat = [m.latency_avg_ms for _, m in self.metrics_history]
        return {
            "samples": len(self.metrics_history),
            "cpu_avg": sum(cpu) / len(cpu),
            "cpu_max": max(cpu),
            "memory_avg": sum(mem) / len(mem),
            "latency_avg": sum(lat) / len(lat),
            "latency_max": max(lat),
        }


class SystemIntegrationSimulator:
    """系统集成仿真器"""

    def __init__(self):
        self.modules: Dict[str, SystemModule] = {}
        self.bus = MessageBus()
        self.interface_tester = InterfaceTester()
        self.profiler = PerformanceProfiler()
        self.integration_log: List[str] = []

    def _log(self, msg: str):
        self.integration_log.append(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def create_system(self):
        """创建完整嵌入式系统"""
        # MCU主控
        mcu = SystemModule("MCU主控", [
            InterfaceSpec("SPI_ADC", InterfaceType.SPI, 10000000, 16, 10000000, 1, 0.001),
            InterfaceSpec("I2C_Sensor", InterfaceType.I2C, 400000, 8, 400000, 5, 0.005),
            InterfaceSpec("UART_Debug", InterfaceType.UART, 115200, 8, 115200, 0.1, 0.0001),
            InterfaceSpec("GPIO_LED", InterfaceType.GPIO, 0, 1, 0, 0.001, 0),
        ])

        # ADC采集模块
        adc_mod = SystemModule("ADC采集", [
            InterfaceSpec("SPI_MCU", InterfaceType.SPI, 10000000, 16, 10000000, 1, 0.001),
            InterfaceSpec("ADC_CH", InterfaceType.ADC, 0, 12, 0, 5, 0.002),
        ])
        adc_mod.dependencies = ["MCU主控"]

        # 传感器模块
        sensor = SystemModule("传感器模块", [
            InterfaceSpec("I2C_MCU", InterfaceType.I2C, 400000, 8, 400000, 5, 0.005),
        ])
        sensor.dependencies = ["MCU主控"]

        # 通信模块
        comm = SystemModule("通信模块", [
            InterfaceSpec("UART_MCU", InterfaceType.UART, 115200, 8, 115200, 0.1, 0.0001),
            InterfaceSpec("CAN_Bus", InterfaceType.CAN, 500000, 8, 500000, 0.5, 0.001),
        ])
        comm.dependencies = ["MCU主控"]

        # 电源管理
        power = SystemModule("电源管理", [
            InterfaceSpec("I2C_MCU", InterfaceType.I2C, 400000, 8, 400000, 5, 0.005),
            InterfaceSpec("DAC_Output", InterfaceType.DAC, 0, 12, 0, 2, 0.003),
        ])
        power.dependencies = ["MCU主控"]

        # 显示模块
        display = SystemModule("显示模块", [
            InterfaceSpec("SPI_MCU", InterfaceType.SPI, 20000000, 8, 20000000, 0.5, 0.001),
        ])
        display.dependencies = ["MCU主控"]

        for mod in [mcu, adc_mod, sensor, comm, power, display]:
            self.modules[mod.name] = mod
            self.bus.register_module(mod)

    def initialize_system(self) -> Dict[str, bool]:
        """按依赖顺序初始化系统"""
        results = {}
        init_order = ["MCU主控", "电源管理", "ADC采集", "传感器模块", "通信模块", "显示模块"]
        for name in init_order:
            mod = self.modules.get(name)
            if mod:
                ok = mod.initialize()
                results[name] = ok
                self._log(f"初始化 {name}: {'成功' if ok else '失败'} ({mod.init_time_ms:.1f}ms)")
        return results

    def start_all(self) -> Dict[str, bool]:
        results = {}
        for name, mod in self.modules.items():
            ok = mod.start()
            results[name] = ok
            self._log(f"启动 {name}: {'成功' if ok else '失败'}")
        return results

    def run_communication_test(self) -> dict:
        """测试模块间通信"""
        results = {}
        test_messages = [
            ("MCU主控", "ADC采集", MessageType.COMMAND, "read_channels"),
            ("ADC采集", "MCU主控", MessageType.DATA, [3.3, 1.2, 2.5, 0.8]),
            ("MCU主控", "传感器模块", MessageType.COMMAND, "read_temp"),
            ("传感器模块", "MCU主控", MessageType.DATA, 45.3),
            ("MCU主控", "通信模块", MessageType.COMMAND, "send_status"),
            ("MCU主控", "显示模块", MessageType.DATA, {"voltage": 3.3, "temp": 45}),
            ("MCU主控", "电源管理", MessageType.COMMAND, "set_voltage", ),
        ]

        for source, target, msg_type, payload in test_messages:
            ok = self.bus.send_message(source, target, msg_type, payload)
            key = f"{source}->{target}"
            results[key] = ok
            self._log(f"消息 {key}: {'成功' if ok else '失败'}")

        # 处理消息
        for mod in self.modules.values():
            result = mod.process(0.01)
            if result.get("processed", 0) > 0:
                self._log(f"{mod.name} 处理: {result}")

        return results

    def run_interface_tests(self) -> Dict[str, List[TestCase]]:
        """运行所有接口测试"""
        results = {}
        for name, mod in self.modules.items():
            test_results = self.interface_tester.run_all_interface_tests(mod)
            results[name] = test_results
            for tc in test_results:
                self._log(f"测试 [{tc.test_id}] {tc.name}: {tc.result.name} ({tc.duration_ms:.2f}ms)")
        return results

    def run_performance_analysis(self, cycles: int = 10) -> PerformanceMetrics:
        """运行性能分析"""
        for _ in range(cycles):
            # 生成负载
            for mod in self.modules.values():
                if mod.state == ModuleState.RUNNING:
                    for _ in range(random.randint(10, 50)):
                        self.bus.send_message(mod.name, "MCU主控", MessageType.DATA,
                                             random.uniform(0, 5))
                    mod.process(0.01)

            metrics = self.profiler.measure(self.modules, self.bus)

        self._log(f"性能分析完成: {cycles} 个周期")
        return metrics

    def run_stress_test(self, duration: float = 5.0) -> dict:
        """压力测试"""
        start = time.time()
        msg_count = 0
        error_count = 0

        while time.time() - start < duration:
            for mod in self.modules.values():
                if mod.state == ModuleState.RUNNING:
                    for _ in range(100):
                        target = random.choice(list(self.modules.keys()))
                        ok = self.bus.send_message(mod.name, target, MessageType.DATA,
                                                   random.uniform(0, 5))
                        msg_count += 1
                        if not ok:
                            error_count += 1
                    mod.process(0.001)

        actual_duration = time.time() - start
        return {
            "duration": actual_duration,
            "messages": msg_count,
            "errors": error_count,
            "throughput": msg_count / actual_duration,
            "error_rate": error_count / max(msg_count, 1),
        }

    def get_system_status(self) -> dict:
        return {
            "modules": {name: mod.state.name for name, mod in self.modules.items()},
            "bus": self.bus.get_stats(),
            "log_entries": len(self.integration_log),
        }


def run_system_integration_simulation():
    """运行系统集成仿真"""
    print("=" * 60)
    print("系统集成仿真 - 多模块协同/接口测试/性能分析")
    print("=" * 60)

    sim = SystemIntegrationSimulator()

    # 1. 创建系统
    print("\n--- 1. 系统构建 ---")
    sim.create_system()
    for name, mod in sim.modules.items():
        ifaces = ", ".join(f"{n}({i.interface_type.name})" for n, i in mod.interfaces.items())
        deps = ", ".join(mod.dependencies) if mod.dependencies else "无"
        print(f"  {name}: 接口=[{ifaces}], 依赖=[{deps}]")

    # 2. 系统初始化
    print("\n--- 2. 系统初始化 ---")
    init_results = sim.initialize_system()
    for name, ok in init_results.items():
        mod = sim.modules[name]
        print(f"  {name}: {'✓' if ok else '✗'} ({mod.init_time_ms:.1f}ms)")

    # 3. 启动所有模块
    print("\n--- 3. 模块启动 ---")
    start_results = sim.start_all()
    for name, ok in start_results.items():
        print(f"  {name}: {'✓' if ok else '✗'} -> {sim.modules[name].state.name}")

    # 4. 模块间通信测试
    print("\n--- 4. 模块间通信 ---")
    comm_results = sim.run_communication_test()
    for path, ok in comm_results.items():
        print(f"  {path}: {'✓' if ok else '✗'}")
    bus_stats = sim.bus.get_stats()
    print(f"  总消息: {bus_stats['total_messages']}, 错误: {bus_stats['errors']}")

    # 5. 接口测试
    print("\n--- 5. 接口测试 ---")
    test_results = sim.run_interface_tests()
    total_pass = 0
    total_fail = 0
    for module, tests in test_results.items():
        passed = sum(1 for t in tests if t.result == TestResult.PASS)
        failed = sum(1 for t in tests if t.result == TestResult.FAIL)
        total_pass += passed
        total_fail += failed
        status = "✓" if failed == 0 else "✗"
        print(f"  {status} {module}: {passed}通过/{failed}失败")
        for tc in tests:
            print(f"    [{tc.test_id}] {tc.name}: {tc.result.name} - {tc.details}")
    print(f"  总计: {total_pass}通过, {total_fail}失败")

    # 6. 性能分析
    print("\n--- 6. 性能分析 ---")
    metrics = sim.run_performance_analysis(10)
    print(f"  CPU使用率: {metrics.cpu_usage:.1f}%")
    print(f"  内存使用: {metrics.memory_usage:.1f}MB")
    print(f"  吞吐量: {metrics.throughput:.1f} 事务/秒")
    print(f"  平均延迟: {metrics.latency_avg_ms:.2f}ms")
    print(f"  最大延迟: {metrics.latency_max_ms:.2f}ms")
    print(f"  错误率: {metrics.error_rate:.4f}")
    print(f"  功耗: {metrics.power_mw:.1f}mW")

    prof_summary = sim.profiler.get_summary()
    if prof_summary:
        print(f"\n  性能摘要 ({prof_summary['samples']} 样本):")
        print(f"    CPU: 平均={prof_summary['cpu_avg']:.1f}%, 最大={prof_summary['cpu_max']:.1f}%")
        print(f"    延迟: 平均={prof_summary['latency_avg']:.2f}ms, 最大={prof_summary['latency_max']:.2f}ms")

    # 7. 压力测试
    print("\n--- 7. 压力测试 ---")
    stress = sim.run_stress_test(2.0)
    print(f"  持续时间: {stress['duration']:.2f}s")
    print(f"  消息总数: {stress['messages']}")
    print(f"  错误数: {stress['errors']}")
    print(f"  吞吐量: {stress['throughput']:.0f} msg/s")
    print(f"  错误率: {stress['error_rate']:.4f}")

    # 8. 广播测试
    print("\n--- 8. 广播测试 ---")
    before = sim.bus.total_transmitted
    sim.bus.broadcast("MCU主控", MessageType.STATUS, "系统正常")
    after = sim.bus.total_transmitted
    print(f"  广播消息: {after - before} 条 (发往 {len(sim.modules) - 1} 个模块)")

    # 9. 系统状态
    print("\n--- 9. 系统状态 ---")
    status = sim.get_system_status()
    for name, state in status["modules"].items():
        mod = sim.modules[name]
        print(f"  {name}: {state} (处理={mod.process_count}, 错误={mod.error_count})")

    # 10. 集成日志
    print("\n--- 10. 集成日志 (最后10条) ---")
    for entry in sim.integration_log[-10:]:
        print(f"  {entry}")

    print("\n" + "=" * 60)
    print("系统集成仿真完成")
    print("=" * 60)


if __name__ == "__main__":
    run_system_integration_simulation()
