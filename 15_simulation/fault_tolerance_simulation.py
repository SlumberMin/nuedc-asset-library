#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
容错系统仿真 - 冗余/切换/恢复/诊断
模拟嵌入式系统的故障检测、冗余切换、自动恢复与诊断
"""

import time
import random
import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Dict, Optional, Callable, Set, Tuple
from collections import deque


class ComponentState(Enum):
    ACTIVE = auto()
    STANDBY = auto()
    DEGRADED = auto()
    FAILED = auto()
    RECOVERING = auto()
    MAINTENANCE = auto()


class FaultType(Enum):
    NONE = auto()
    TIMEOUT = auto()
    OVERCURRENT = auto()
    OVERVOLTAGE = auto()
    OVERTEMPERATURE = auto()
    COMM_LOSS = auto()
    SENSOR_DRIFT = auto()
    MEMORY_CORRUPTION = auto()
    WATCHDOG_RESET = auto()
    POWER_SUPPLY = auto()


class RecoveryStrategy(Enum):
    RESTART = auto()
    FAILOVER = auto()
    GRACEFUL_DEGRADE = auto()
    MANUAL = auto()
    WATCHDOG_RESET = auto()
    REBOOT = auto()


class DiagnosticLevel(Enum):
    INFO = auto()
    WARNING = auto()
    ERROR = auto()
    CRITICAL = auto()
    FATAL = auto()


@dataclass
class FaultEvent:
    """故障事件"""
    fault_id: int
    fault_type: FaultType
    component: str
    timestamp: float
    severity: DiagnosticLevel
    description: str
    resolved: bool = False
    resolution_time: float = 0


@dataclass
class DiagnosticMessage:
    """诊断消息"""
    timestamp: float
    level: DiagnosticLevel
    source: str
    message: str
    fault_id: Optional[int] = None


@dataclass
class HealthMetrics:
    """健康指标"""
    uptime: float = 0
    mtbf: float = 0          # 平均无故障时间
    mttr: float = 0          # 平均恢复时间
    availability: float = 1.0
    fault_count: int = 0
    recovery_count: int = 0
    failover_count: int = 0


@dataclass
class RedundancyGroup:
    """冗余组"""
    name: str
    components: List[str]
    active_index: int = 0
    strategy: str = "active-standby"  # active-standby, active-active, N+1


class Component:
    """系统组件"""

    def __init__(self, name: str, mtbf_hours: float = 1000, recovery_time: float = 1.0):
        self.name = name
        self.state = ComponentState.STANDBY
        self.mtbf_hours = mtbf_hours
        self.recovery_time = recovery_time
        self.uptime = 0.0
        self.downtime = 0.0
        self.fault_history: List[FaultEvent] = []
        self.current_fault: Optional[FaultType] = None
        self.health_score: float = 100.0
        self.temperature: float = 25.0
        self.voltage: float = 3.3
        self.current_draw: float = 0.1
        self.error_counter: int = 0
        self.watchdog_counter: int = 0
        self.last_heartbeat: float = time.time()

    def activate(self):
        self.state = ComponentState.ACTIVE

    def standby(self):
        self.state = ComponentState.STANDBY

    def inject_fault(self, fault_type: FaultType, severity: DiagnosticLevel) -> FaultEvent:
        self.current_fault = fault_type
        self.state = ComponentState.FAILED
        self.health_score = max(0, self.health_score - 30)
        self.error_counter += 1
        event = FaultEvent(
            fault_id=len(self.fault_history), fault_type=fault_type,
            component=self.name, timestamp=time.time(),
            severity=severity, description=f"{fault_type.name} on {self.name}"
        )
        self.fault_history.append(event)
        return event

    def recover(self) -> bool:
        if self.state != ComponentState.FAILED:
            return False
        self.state = ComponentState.RECOVERING
        if random.random() < 0.9:
            self.state = ComponentState.ACTIVE
            self.current_fault = None
            self.health_score = min(100, self.health_score + 20)
            if self.fault_history:
                self.fault_history[-1].resolved = True
                self.fault_history[-1].resolution_time = self.recovery_time
            return True
        return False

    def update_health(self, dt: float):
        if self.state == ComponentState.ACTIVE:
            self.uptime += dt
            base_health = 100.0
            if self.temperature > 60:
                base_health -= (self.temperature - 60) * 0.5
            if self.voltage < 3.0 or self.voltage > 3.6:
                base_health -= 10
            self.health_score = max(0, min(100, base_health + random.gauss(0, 1)))
        elif self.state == ComponentState.FAILED:
            self.downtime += dt

    def send_heartbeat(self):
        self.last_heartbeat = time.time()
        self.watchdog_counter += 1

    def check_watchdog(self, timeout: float = 5.0) -> bool:
        return (time.time() - self.last_heartbeat) < timeout

    def get_metrics(self) -> dict:
        total = self.uptime + self.downtime
        return {
            "name": self.name, "state": self.state.name,
            "health": round(self.health_score, 1),
            "uptime": round(self.uptime, 2), "downtime": round(self.downtime, 2),
            "availability": round(self.uptime / max(total, 0.001), 4),
            "faults": len(self.fault_history), "current_fault": self.current_fault.name if self.current_fault else "NONE",
            "temperature": round(self.temperature, 1), "voltage": round(self.voltage, 3),
        }


class FaultDetector:
    """故障检测器"""

    def __init__(self):
        self.monitors: Dict[str, Callable] = {}
        self.detection_history: List[Tuple[str, FaultType, float]] = []

    def register_monitor(self, name: str, check_fn: Callable):
        self.monitors[name] = check_fn

    def check_all(self, components: Dict[str, Component]) -> List[FaultEvent]:
        faults = []
        for name, check_fn in self.monitors.items():
            result = check_fn(components)
            if result:
                faults.extend(result)
                for f in result:
                    self.detection_history.append((name, f.fault_type, f.timestamp))
        return faults

    def detect_timeout(self, component: Component, threshold: float = 5.0) -> Optional[FaultEvent]:
        if not component.check_watchdog(threshold):
            return component.inject_fault(FaultType.TIMEOUT, DiagnosticLevel.ERROR)
        return None

    def detect_overtemperature(self, component: Component, threshold: float = 75.0) -> Optional[FaultEvent]:
        if component.temperature > threshold:
            return component.inject_fault(FaultType.OVERTEMPERATURE, DiagnosticLevel.WARNING)
        return None

    def detect_overcurrent(self, component: Component, threshold: float = 2.0) -> Optional[FaultEvent]:
        if component.current_draw > threshold:
            return component.inject_fault(FaultType.OVERCURRENT, DiagnosticLevel.ERROR)
        return None


class FailoverManager:
    """故障切换管理器"""

    def __init__(self):
        self.redundancy_groups: Dict[str, RedundancyGroup] = {}
        self.failover_log: List[dict] = []
        self.switch_time_ms: float = 50  # 切换时间

    def create_group(self, name: str, components: List[str], strategy: str = "active-standby"):
        self.redundancy_groups[name] = RedundancyGroup(name, components, 0, strategy)

    def perform_failover(self, group_name: str, components: Dict[str, Component]) -> Optional[str]:
        group = self.redundancy_groups.get(group_name)
        if not group:
            return None

        old_active = group.components[group.active_index]
        for i, comp_name in enumerate(group.components):
            if i != group.active_index:
                comp = components.get(comp_name)
                if comp and comp.state == ComponentState.STANDBY:
                    group.active_index = i
                    comp.activate()
                    components[old_active].standby()
                    entry = {
                        "time": time.time(), "group": group_name,
                        "from": old_active, "to": comp_name,
                        "switch_ms": self.switch_time_ms
                    }
                    self.failover_log.append(entry)
                    return comp_name
        return None

    def get_active(self, group_name: str) -> Optional[str]:
        group = self.redundancy_groups.get(group_name)
        if group:
            return group.components[group.active_index]
        return None


class RecoveryEngine:
    """恢复引擎"""

    def __init__(self):
        self.strategies: Dict[FaultType, RecoveryStrategy] = {
            FaultType.TIMEOUT: RecoveryStrategy.RESTART,
            FaultType.OVERCURRENT: RecoveryStrategy.FAILOVER,
            FaultType.OVERVOLTAGE: RecoveryStrategy.GRACEFUL_DEGRADE,
            FaultType.OVERTEMPERATURE: RecoveryStrategy.GRACEFUL_DEGRADE,
            FaultType.COMM_LOSS: RecoveryStrategy.RESTART,
            FaultType.SENSOR_DRIFT: RecoveryStrategy.RESTART,
            FaultType.MEMORY_CORRUPTION: RecoveryStrategy.REBOOT,
            FaultType.WATCHDOG_RESET: RecoveryStrategy.WATCHDOG_RESET,
            FaultType.POWER_SUPPLY: RecoveryStrategy.FAILOVER,
        }
        self.recovery_log: List[dict] = []
        self.max_retries = 3

    def recover(self, component: Component, fault_type: FaultType,
                failover_mgr: FailoverManager = None,
                components: Dict[str, Component] = None) -> bool:
        strategy = self.strategies.get(fault_type, RecoveryStrategy.RESTART)

        for attempt in range(self.max_retries):
            success = False

            if strategy == RecoveryStrategy.RESTART:
                success = component.recover()
            elif strategy == RecoveryStrategy.FAILOVER and failover_mgr:
                for gname in failover_mgr.redundancy_groups:
                    group = failover_mgr.redundancy_groups[gname]
                    if component.name in group.components:
                        result = failover_mgr.perform_failover(gname, components)
                        success = result is not None
                        break
            elif strategy == RecoveryStrategy.GRACEFUL_DEGRADE:
                component.state = ComponentState.DEGRADED
                component.current_fault = None
                component.health_score = max(30, component.health_score)
                success = True
            elif strategy == RecoveryStrategy.WATCHDOG_RESET:
                component.watchdog_counter = 0
                component.last_heartbeat = time.time()
                success = component.recover()

            entry = {
                "time": time.time(), "component": component.name,
                "strategy": strategy.name, "attempt": attempt + 1,
                "success": success
            }
            self.recovery_log.append(entry)

            if success:
                return True

        return False


class DiagnosticEngine:
    """诊断引擎"""

    def __init__(self):
        self.messages: List[DiagnosticMessage] = []
        self.fault_codes: Dict[str, int] = {}

    def log(self, level: DiagnosticLevel, source: str, message: str, fault_id: int = None):
        msg = DiagnosticMessage(time.time(), level, source, message, fault_id)
        self.messages.append(msg)

    def diagnose_component(self, comp: Component) -> List[str]:
        findings = []
        if comp.health_score < 50:
            findings.append(f"健康度低: {comp.health_score:.1f}/100")
        if comp.temperature > 60:
            findings.append(f"温度偏高: {comp.temperature:.1f}°C")
        if comp.voltage < 3.0 or comp.voltage > 3.6:
            findings.append(f"电压异常: {comp.voltage:.3f}V")
        if comp.error_counter > 10:
            findings.append(f"错误计数高: {comp.error_counter}")
        if comp.current_fault:
            findings.append(f"当前故障: {comp.current_fault.name}")
        if not findings:
            findings.append("组件正常")
        return findings

    def generate_report(self, components: Dict[str, Component]) -> dict:
        report = {"timestamp": time.time(), "components": {}, "summary": {}}
        total_faults = 0
        for name, comp in components.items():
            report["components"][name] = {
                "state": comp.state.name,
                "health": comp.health_score,
                "findings": self.diagnose_component(comp)
            }
            total_faults += len(comp.fault_history)
        report["summary"] = {
            "total_components": len(components),
            "total_faults": total_faults,
            "messages": len(self.messages),
            "warnings": sum(1 for m in self.messages if m.level == DiagnosticLevel.WARNING),
            "errors": sum(1 for m in self.messages if m.level == DiagnosticLevel.ERROR),
        }
        return report


class FaultToleranceSimulator:
    """容错系统仿真器"""

    def __init__(self):
        self.components: Dict[str, Component] = {}
        self.detector = FaultDetector()
        self.failover_mgr = FailoverManager()
        self.recovery_engine = RecoveryEngine()
        self.diagnostics = DiagnosticEngine()
        self.sim_time = 0.0
        self.scenario_log: List[str] = []

    def add_component(self, name: str, mtbf: float = 1000, recovery_time: float = 1.0):
        self.components[name] = Component(name, mtbf, recovery_time)

    def setup_redundancy(self, group_name: str, comp_names: List[str], strategy: str = "active-standby"):
        for name in comp_names:
            if name not in self.components:
                self.add_component(name)
        self.failover_mgr.create_group(group_name, comp_names, strategy)
        # 激活第一个，其余备用
        self.components[comp_names[0]].activate()
        for name in comp_names[1:]:
            self.components[name].standby()

    def inject_and_recover(self, comp_name: str, fault_type: FaultType) -> dict:
        comp = self.components.get(comp_name)
        if not comp:
            return {"error": f"组件 '{comp_name}' 不存在"}

        # 注入故障
        fault = comp.inject_fault(fault_type, DiagnosticLevel.ERROR)
        self.diagnostics.log(DiagnosticLevel.ERROR, comp_name,
                            f"故障注入: {fault_type.name}", fault.fault_id)
        self.scenario_log.append(f"[t={self.sim_time:.2f}] {comp_name}: {fault_type.name} 故障")

        # 尝试恢复
        success = self.recovery_engine.recover(
            comp, fault_type, self.failover_mgr, self.components)

        if success:
            self.diagnostics.log(DiagnosticLevel.INFO, comp_name, "恢复成功")
            self.scenario_log.append(f"[t={self.sim_time:.2f}] {comp_name}: 恢复成功 ({comp.state.name})")
        else:
            self.diagnostics.log(DiagnosticLevel.CRITICAL, comp_name, "恢复失败")
            self.scenario_log.append(f"[t={self.sim_time:.2f}] {comp_name}: 恢复失败")

        return {
            "fault": fault_type.name, "component": comp_name,
            "recovered": success, "new_state": comp.state.name,
            "health": comp.health_score
        }

    def run_scenario(self, name: str, steps: List[dict]) -> List[dict]:
        results = []
        self.scenario_log.append(f"\n=== 场景: {name} ===")
        for i, step in enumerate(steps):
            action = step.get("action")
            if action == "inject":
                r = self.inject_and_recover(step["component"], FaultType[step["fault"]])
                results.append(r)
            elif action == "activate":
                comp = self.components.get(step["component"])
                if comp:
                    comp.activate()
                    self.scenario_log.append(f"[t={self.sim_time:.2f}] {step['component']}: 手动激活")
            elif action == "failover":
                result = self.failover_mgr.perform_failover(step["group"], self.components)
                self.scenario_log.append(f"[t={self.sim_time:.2f}] 切换: {result}")
                results.append({"failover": result})
            elif action == "update":
                dt = step.get("dt", 1.0)
                self.sim_time += dt
                for comp in self.components.values():
                    comp.update_health(dt)
                    if comp.state == ComponentState.ACTIVE:
                        comp.send_heartbeat()
            elif action == "temp":
                comp = self.components.get(step["component"])
                if comp:
                    comp.temperature = step["value"]
            elif action == "voltage":
                comp = self.components.get(step["component"])
                if comp:
                    comp.voltage = step["value"]
        return results

    def get_system_status(self) -> dict:
        states = {}
        for comp in self.components.values():
            states[comp.state.name] = states.get(comp.state.name, 0) + 1
        return {
            "sim_time": self.sim_time, "components": len(self.components),
            "states": states, "recovery_log": len(self.recovery_engine.recovery_log),
            "failover_log": len(self.failover_mgr.failover_log),
        }


def run_fault_tolerance_simulation():
    """运行容错系统仿真"""
    print("=" * 60)
    print("容错系统仿真 - 冗余/切换/恢复/诊断")
    print("=" * 60)

    sim = FaultToleranceSimulator()

    # 1. 系统构建
    print("\n--- 1. 系统构建 ---")
    sim.add_component("主控MCU", 5000, 2.0)
    sim.add_component("备用MCU", 5000, 2.0)
    sim.add_component("电源A", 3000, 1.5)
    sim.add_component("电源B", 3000, 1.5)
    sim.add_component("传感器1", 8000, 0.5)
    sim.add_component("传感器2", 8000, 0.5)
    sim.add_component("通信模块", 4000, 1.0)

    sim.setup_redundancy("MCU组", ["主控MCU", "备用MCU"], "active-standby")
    sim.setup_redundancy("电源组", ["电源A", "电源B"], "active-standby")
    sim.setup_redundancy("传感器组", ["传感器1", "传感器2"], "active-active")

    for name, comp in sim.components.items():
        print(f"  {name}: {comp.state.name}")

    for gname, group in sim.failover_mgr.redundancy_groups.items():
        active = sim.failover_mgr.get_active(gname)
        print(f"  冗余组 '{gname}': 活跃={active}, 策略={group.strategy}")

    # 2. 单组件故障与恢复
    print("\n--- 2. 单组件故障与恢复 ---")
    sim.run_scenario("单点故障", [
        {"action": "update", "dt": 1.0},
        {"action": "inject", "component": "通信模块", "fault": "COMM_LOSS"},
        {"action": "update", "dt": 2.0},
    ])
    for name, comp in sim.components.items():
        m = comp.get_metrics()
        if m["faults"] > 0:
            print(f"  {name}: 状态={m['state']}, 健康={m['health']}, 故障数={m['faults']}")

    # 3. MCU冗余切换
    print("\n--- 3. MCU冗余切换仿真 ---")
    sim.run_scenario("MCU故障切换", [
        {"action": "inject", "component": "主控MCU", "fault": "WATCHDOG_RESET"},
        {"action": "update", "dt": 1.0},
    ])
    mcu_active = sim.failover_mgr.get_active("MCU组")
    print(f"  MCU组当前活跃: {mcu_active}")

    # 4. 电源冗余切换
    print("\n--- 4. 电源故障仿真 ---")
    sim.run_scenario("电源过流", [
        {"action": "inject", "component": "电源A", "fault": "OVERCURRENT"},
        {"action": "update", "dt": 1.0},
    ])
    psu_active = sim.failover_mgr.get_active("电源组")
    print(f"  电源组当前活跃: {psu_active}")

    # 5. 传感器漂移与降级
    print("\n--- 5. 传感器漂移仿真 ---")
    sim.run_scenario("传感器漂移", [
        {"action": "inject", "component": "传感器1", "fault": "SENSOR_DRIFT"},
        {"action": "update", "dt": 0.5},
    ])
    s1 = sim.components["传感器1"]
    print(f"  传感器1: 状态={s1.state.name}, 健康={s1.health_score:.1f}")

    # 6. 温度触发故障
    print("\n--- 6. 过温故障仿真 ---")
    sim.run_scenario("过温", [
        {"action": "temp", "component": "传感器2", "value": 85.0},
        {"action": "inject", "component": "传感器2", "fault": "OVERTEMPERATURE"},
        {"action": "update", "dt": 1.0},
    ])
    s2 = sim.components["传感器2"]
    print(f"  传感器2: 状态={s2.state.name}, 温度={s2.temperature}°C")

    # 7. 多重故障场景
    print("\n--- 7. 多重故障仿真 ---")
    sim.run_scenario("级联故障", [
        {"action": "inject", "component": "备用MCU", "fault": "MEMORY_CORRUPTION"},
        {"action": "inject", "component": "电源B", "fault": "POWER_SUPPLY"},
        {"action": "inject", "component": "通信模块", "fault": "OVERVOLTAGE"},
        {"action": "update", "dt": 2.0},
    ])
    for name, comp in sim.components.items():
        m = comp.get_metrics()
        print(f"  {name}: 状态={m['state']}, 健康={m['health']}, 当前故障={m['current_fault']}")

    # 8. 系统诊断报告
    print("\n--- 8. 诊断报告 ---")
    report = sim.diagnostics.generate_report(sim.components)
    print(f"  组件总数: {report['summary']['total_components']}")
    print(f"  总故障数: {report['summary']['total_faults']}")
    print(f"  警告数: {report['summary']['warnings']}")
    print(f"  错误数: {report['summary']['errors']}")
    for name, info in report["components"].items():
        findings = "; ".join(info["findings"])
        print(f"  [{name}] {info['state']} (健康={info['health']:.1f}): {findings}")

    # 9. 恢复日志
    print("\n--- 9. 恢复日志 ---")
    for entry in sim.recovery_engine.recovery_log[-8:]:
        status = "✓" if entry["success"] else "✗"
        print(f"  {status} {entry['component']}: {entry['strategy']} (尝试{entry['attempt']})")

    # 10. 切换日志
    print("\n--- 10. 故障切换日志 ---")
    for entry in sim.failover_mgr.failover_log:
        print(f"  {entry['group']}: {entry['from']} -> {entry['to']} ({entry['switch_ms']}ms)")

    # 11. 可用性统计
    print("\n--- 11. 可用性统计 ---")
    for name, comp in sim.components.items():
        m = comp.get_metrics()
        print(f"  {name}: 可用性={m['availability']:.2%}, 运行={m['uptime']:.1f}s, 停机={m['downtime']:.1f}s")

    # 12. 场景日志
    print("\n--- 12. 仿真场景日志 ---")
    for log in sim.scenario_log[-12:]:
        print(f"  {log}")

    print("\n" + "=" * 60)
    print("容错系统仿真完成")
    print("=" * 60)


if __name__ == "__main__":
    run_fault_tolerance_simulation()
