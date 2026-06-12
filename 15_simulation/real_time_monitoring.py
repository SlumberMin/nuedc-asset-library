#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时监控仿真 - 数据采集/阈值报警/趋势分析/仪表盘
模拟嵌入式系统的实时数据监控、报警与趋势分析
"""

import time
import random
import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Dict, Optional, Tuple, Callable
from collections import deque
from datetime import datetime


class AlarmLevel(Enum):
    NORMAL = auto()
    ADVISORY = auto()
    CAUTION = auto()
    WARNING = auto()
    CRITICAL = auto()
    EMERGENCY = auto()


class TrendDirection(Enum):
    RISING = auto()
    FALLING = auto()
    STABLE = auto()
    OSCILLATING = auto()


class GaugeType(Enum):
    LINEAR = auto()
    RADIAL = auto()
    BAR = auto()
    DIGITAL = auto()
    LED = auto()


@dataclass
class Threshold:
    """阈值配置"""
    name: str
    low_critical: float = float('-inf')
    low_warning: float = float('-inf')
    low_caution: float = float('-inf')
    high_caution: float = float('inf')
    high_warning: float = float('inf')
    high_critical: float = float('inf')
    hysteresis: float = 0.0


@dataclass
class Alarm:
    """报警事件"""
    alarm_id: int
    channel: str
    level: AlarmLevel
    timestamp: float
    value: float
    threshold_name: str
    message: str
    acknowledged: bool = False
    cleared: bool = False
    clear_time: float = 0


@dataclass
class TrendData:
    """趋势数据"""
    channel: str
    direction: TrendDirection
    slope: float  # 变化率/秒
    r_squared: float  # 线性拟合R²
    predicted_value: float
    time_to_threshold: float = float('inf')  # 预计多久达到阈值


@dataclass
class GaugeWidget:
    """仪表盘控件"""
    id: str
    gauge_type: GaugeType
    channel: str
    label: str
    unit: str
    min_val: float
    max_val: float
    current_value: float = 0
    color_zones: List[Tuple[float, float, str]] = field(default_factory=list)


@dataclass
class DashboardLayout:
    """仪表盘布局"""
    title: str
    gauges: List[GaugeWidget]
    width: int = 1024
    height: int = 768
    grid_cols: int = 3
    grid_rows: int = 2
    refresh_ms: int = 100


class DataAcquisition:
    """数据采集模块"""

    def __init__(self, sample_rate_hz: float = 100):
        self.sample_rate = sample_rate_hz
        self.channels: Dict[str, dict] = {}
        self.buffers: Dict[str, deque] = {}
        self.timestamps: Dict[str, deque] = {}
        self.total_acquired = 0
        self.missed_samples = 0

    def add_channel(self, name: str, unit: str, signal_gen: Callable[[float], float] = None):
        self.channels[name] = {
            "unit": unit, "enabled": True,
            "signal_gen": signal_gen or (lambda t: random.gauss(0, 1))
        }
        self.buffers[name] = deque(maxlen=10000)
        self.timestamps[name] = deque(maxlen=10000)

    def acquire(self, duration_sec: float) -> Dict[str, List[float]]:
        results = {}
        t_start = time.time()
        n_samples = int(duration_sec * self.sample_rate)
        dt = 1.0 / self.sample_rate

        for i in range(n_samples):
            t = i * dt
            for name, ch in self.channels.items():
                if ch["enabled"]:
                    value = ch["signal_gen"](t + t_start)
                    self.buffers[name].append(value)
                    self.timestamps[name].append(t_start + t)
                    results.setdefault(name, []).append(value)
                    self.total_acquired += 1
        return results

    def get_latest(self, channel: str, n: int = 100) -> List[float]:
        buf = self.buffers.get(channel, deque())
        return list(buf)[-n:]

    def get_latest_value(self, channel: str) -> Optional[float]:
        buf = self.buffers.get(channel, deque())
        return buf[-1] if buf else None

    def get_stats(self) -> dict:
        return {
            "channels": len(self.channels),
            "total_acquired": self.total_acquired,
            "missed": self.missed_samples,
            "sample_rate": self.sample_rate,
        }


class AlarmManager:
    """报警管理器"""

    def __init__(self):
        self.thresholds: Dict[str, Threshold] = {}
        self.active_alarms: Dict[str, Alarm] = {}
        self.alarm_history: List[Alarm] = []
        self.alarm_counter = 0
        self.suppression_time: Dict[str, float] = {}  # 报警抑制

    def set_threshold(self, channel: str, threshold: Threshold):
        self.thresholds[channel] = threshold

    def check_value(self, channel: str, value: float) -> Optional[Alarm]:
        th = self.thresholds.get(channel)
        if not th:
            return None

        level = AlarmLevel.NORMAL
        trigger_name = ""

        if value <= th.low_critical:
            level, trigger_name = AlarmLevel.CRITICAL, "low_critical"
        elif value <= th.low_warning:
            level, trigger_name = AlarmLevel.WARNING, "low_warning"
        elif value <= th.low_caution:
            level, trigger_name = AlarmLevel.CAUTION, "low_caution"
        elif value >= th.high_critical:
            level, trigger_name = AlarmLevel.CRITICAL, "high_critical"
        elif value >= th.high_warning:
            level, trigger_name = AlarmLevel.WARNING, "high_warning"
        elif value >= th.high_caution:
            level, trigger_name = AlarmLevel.CAUTION, "high_caution"

        if level == AlarmLevel.NORMAL:
            # 清除已有报警
            if channel in self.active_alarms:
                alarm = self.active_alarms[channel]
                alarm.cleared = True
                alarm.clear_time = time.time()
                del self.active_alarms[channel]
            return None

        # 检查抑制
        key = f"{channel}:{trigger_name}"
        last = self.suppression_time.get(key, 0)
        if time.time() - last < 5.0:  # 5秒抑制
            return None

        self.alarm_counter += 1
        alarm = Alarm(
            alarm_id=self.alarm_counter, channel=channel, level=level,
            timestamp=time.time(), value=value, threshold_name=trigger_name,
            message=f"{channel} = {value:.3f} 触发 {trigger_name} ({level.name})"
        )
        self.active_alarms[channel] = alarm
        self.alarm_history.append(alarm)
        self.suppression_time[key] = time.time()
        return alarm

    def acknowledge(self, alarm_id: int) -> bool:
        for alarm in self.alarm_history:
            if alarm.alarm_id == alarm_id:
                alarm.acknowledged = True
                return True
        return False

    def get_active_count(self) -> Dict[str, int]:
        counts = {}
        for alarm in self.active_alarms.values():
            counts[alarm.level.name] = counts.get(alarm.level.name, 0) + 1
        return counts


class TrendAnalyzer:
    """趋势分析器"""

    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        self.trends: Dict[str, TrendData] = {}

    def analyze(self, channel: str, values: List[float], sample_rate: float) -> TrendData:
        if len(values) < 10:
            return TrendData(channel, TrendDirection.STABLE, 0, 0, values[-1] if values else 0)

        n = min(len(values), self.window_size)
        recent = values[-n:]
        x = list(range(n))
        x_mean = sum(x) / n
        y_mean = sum(recent) / n

        ss_xy = sum((x[i] - x_mean) * (recent[i] - y_mean) for i in range(n))
        ss_xx = sum((x[i] - x_mean) ** 2 for i in range(n))
        ss_yy = sum((recent[i] - y_mean) ** 2 for i in range(n))

        if ss_xx == 0:
            slope = 0
        else:
            slope = ss_xy / ss_xx

        r_squared = (ss_xy ** 2) / (ss_xx * ss_yy) if ss_xx * ss_yy > 0 else 0
        slope_per_sec = slope * sample_rate

        if abs(slope_per_sec) < 0.001:
            direction = TrendDirection.STABLE
        elif slope_per_sec > 0:
            direction = TrendDirection.RISING
        else:
            direction = TrendDirection.FALLING

        # 检测振荡
        sign_changes = sum(1 for i in range(1, len(recent))
                          if (recent[i] - recent[i-1]) * (recent[i-1] - (recent[i-2] if i >= 2 else recent[i-1])) < 0)
        if sign_changes > n * 0.4:
            direction = TrendDirection.OSCILLATING

        predicted = recent[-1] + slope_per_sec * 60  # 预测1分钟后

        trend = TrendData(channel, direction, round(slope_per_sec, 6),
                         round(r_squared, 4), round(predicted, 4))
        self.trends[channel] = trend
        return trend

    def predict_time_to_threshold(self, channel: str, current: float,
                                   threshold: float, sample_rate: float) -> float:
        trend = self.trends.get(channel)
        if not trend or abs(trend.slope) < 1e-10:
            return float('inf')
        remaining = threshold - current
        if (remaining > 0 and trend.slope <= 0) or (remaining < 0 and trend.slope >= 0):
            return float('inf')
        return abs(remaining / trend.slope)


class DashboardRenderer:
    """仪表盘渲染器"""

    def __init__(self):
        self.gauges: Dict[str, GaugeWidget] = {}
        self.layout: Optional[DashboardLayout] = None
        self.render_count = 0

    def create_gauge(self, id: str, gauge_type: GaugeType, channel: str,
                     label: str, unit: str, min_val: float, max_val: float,
                     color_zones: List[Tuple[float, float, str]] = None) -> GaugeWidget:
        gauge = GaugeWidget(id, gauge_type, channel, label, unit, min_val, max_val,
                           color_zones=color_zones or [])
        self.gauges[id] = gauge
        return gauge

    def update_value(self, gauge_id: str, value: float):
        if gauge_id in self.gauges:
            self.gauges[gauge_id].current_value = value

    def render(self) -> dict:
        self.render_count += 1
        widgets = []
        for gid, g in self.gauges.items():
            pct = (g.current_value - g.min_val) / max(g.max_val - g.min_val, 0.001) * 100
            zone = "normal"
            for z_min, z_max, z_color in g.color_zones:
                if z_min <= g.current_value <= z_max:
                    zone = z_color
                    break
            widgets.append({
                "id": gid, "type": g.gauge_type.name,
                "label": g.label, "value": g.current_value,
                "unit": g.unit, "percent": round(pct, 1),
                "zone": zone, "min": g.min_val, "max": g.max_val
            })
        return {"frame": self.render_count, "widgets": widgets, "timestamp": time.time()}


class RealTimeMonitoringSimulator:
    """实时监控仿真器"""

    def __init__(self, sample_rate: float = 100):
        self.acquisition = DataAcquisition(sample_rate)
        self.alarm_mgr = AlarmManager()
        self.trend_analyzer = TrendAnalyzer()
        self.dashboard = DashboardRenderer()
        self.sample_rate = sample_rate
        self.sim_log: List[str] = []

    def setup_channels(self):
        """配置监控通道与信号源"""
        channels = [
            ("voltage", "V", lambda t: 3.3 + 0.2 * math.sin(0.5 * t) + random.gauss(0, 0.05)),
            ("current", "A", lambda t: 1.0 + 0.1 * math.sin(0.3 * t) + 0.02 * t + random.gauss(0, 0.03)),
            ("temperature", "°C", lambda t: 45 + 10 * math.sin(0.1 * t) + 0.5 * t + random.gauss(0, 0.5)),
            ("pressure", "kPa", lambda t: 101.3 + 2 * math.sin(0.2 * t) + random.gauss(0, 0.3)),
            ("power", "W", lambda t: 3.3 * (1.0 + 0.1 * math.sin(0.3 * t)) + random.gauss(0, 0.1)),
        ]
        for name, unit, gen in channels:
            self.acquisition.add_channel(name, unit, gen)

    def setup_thresholds(self):
        """配置报警阈值"""
        configs = {
            "voltage": Threshold("电压", 2.8, 3.0, 3.1, 3.5, 3.6, 3.8, 0.05),
            "current": Threshold("电流", -0.5, 0.1, 0.3, 1.5, 1.8, 2.0, 0.05),
            "temperature": Threshold("温度", -10, 0, 15, 60, 75, 85, 1.0),
            "pressure": Threshold("压力", 90, 95, 98, 105, 108, 115, 0.5),
            "power": Threshold("功率", 0, 1, 2, 5, 6, 8, 0.2),
        }
        for ch, th in configs.items():
            self.alarm_mgr.set_threshold(ch, th)

    def setup_dashboard(self):
        """配置仪表盘"""
        self.dashboard.create_gauge("volt_gauge", GaugeType.RADIAL, "voltage",
                                    "电压", "V", 0, 5,
                                    [(0, 2.8, "red"), (2.8, 3.0, "yellow"),
                                     (3.0, 3.6, "green"), (3.6, 3.8, "yellow"), (3.8, 5, "red")])
        self.dashboard.create_gauge("curr_gauge", GaugeType.RADIAL, "current",
                                    "电流", "A", 0, 3,
                                    [(0, 0.1, "red"), (0.1, 0.3, "yellow"),
                                     (0.3, 1.8, "green"), (1.8, 3, "red")])
        self.dashboard.create_gauge("temp_gauge", GaugeType.RADIAL, "temperature",
                                    "温度", "°C", 0, 100,
                                    [(0, 15, "blue"), (15, 60, "green"),
                                     (60, 75, "yellow"), (75, 100, "red")])
        self.dashboard.create_gauge("press_gauge", GaugeType.BAR, "pressure",
                                    "压力", "kPa", 85, 120,
                                    [(85, 95, "yellow"), (95, 108, "green"), (108, 120, "red")])
        self.dashboard.create_gauge("power_gauge", GaugeType.BAR, "power",
                                    "功率", "W", 0, 10,
                                    [(0, 1, "red"), (1, 6, "green"), (6, 10, "red")])

    def run_monitoring_cycle(self, duration: float = 1.0) -> dict:
        """运行一个监控周期"""
        # 采集
        data = self.acquisition.acquire(duration)

        # 报警检查
        alarms = []
        for channel, values in data.items():
            if values:
                alarm = self.alarm_mgr.check_value(channel, values[-1])
                if alarm:
                    alarms.append(alarm)

        # 趋势分析
        trends = {}
        for channel in self.acquisition.channels:
            latest = self.acquisition.get_latest(channel, 200)
            if latest:
                trends[channel] = self.trend_analyzer.analyze(channel, latest, self.sample_rate)

        # 仪表盘更新
        for gid, gauge in self.dashboard.gauges.items():
            val = self.acquisition.get_latest_value(gauge.channel)
            if val is not None:
                self.dashboard.update_value(gid, val)

        frame = self.dashboard.render()

        return {
            "acquired": {ch: len(v) for ch, v in data.items()},
            "alarms": alarms,
            "trends": trends,
            "dashboard_frame": frame,
        }

    def get_latest_values(self) -> Dict[str, float]:
        values = {}
        for ch in self.acquisition.channels:
            val = self.acquisition.get_latest_value(ch)
            if val is not None:
                values[ch] = round(val, 4)
        return values


def run_real_time_monitoring_simulation():
    """运行实时监控仿真"""
    print("=" * 60)
    print("实时监控仿真 - 数据采集/阈值报警/趋势分析/仪表盘")
    print("=" * 60)

    sim = RealTimeMonitoringSimulator(sample_rate=100)
    sim.setup_channels()
    sim.setup_thresholds()
    sim.setup_dashboard()

    # 1. 通道配置
    print("\n--- 1. 通道配置 ---")
    for name, ch in sim.acquisition.channels.items():
        print(f"  {name}: 单位={ch['unit']}, 采样率={sim.sample_rate}Hz")

    # 2. 报警阈值
    print("\n--- 2. 报警阈值 ---")
    for ch, th in sim.alarm_mgr.thresholds.items():
        print(f"  {ch}: 低危={th.low_critical}, 低警={th.low_warning}, "
              f"低注意={th.low_caution}, 高注意={th.high_caution}, "
              f"高警={th.high_warning}, 高危={th.high_critical}")

    # 3. 多周期监控
    print("\n--- 3. 多周期监控仿真 ---")
    all_alarms = []
    for cycle in range(5):
        result = sim.run_monitoring_cycle(1.0)
        vals = sim.get_latest_values()
        vals_str = ", ".join(f"{k}={v}" for k, v in vals.items())
        print(f"  周期 {cycle+1}: {vals_str}")
        all_alarms.extend(result["alarms"])

    # 4. 趋势分析
    print("\n--- 4. 趋势分析 ---")
    for channel in sim.acquisition.channels:
        latest = sim.acquisition.get_latest(channel, 500)
        if latest:
            trend = sim.trend_analyzer.analyze(channel, latest, sim.sample_rate)
            print(f"  {channel}: 方向={trend.direction.name}, "
                  f"斜率={trend.slope:.6f}/s, R²={trend.r_squared:.4f}, "
                  f"预测值={trend.predicted_value:.4f}")

    # 5. 报警统计
    print("\n--- 5. 报警统计 ---")
    print(f"  历史报警总数: {len(sim.alarm_mgr.alarm_history)}")
    print(f"  当前活跃报警: {len(sim.alarm_mgr.active_alarms)}")
    active_counts = sim.alarm_mgr.get_active_count()
    if active_counts:
        for level, count in active_counts.items():
            print(f"    {level}: {count}")

    if all_alarms:
        print(f"\n  最近报警 (本周期):")
        for a in all_alarms[-5:]:
            print(f"    [{a.level.name}] {a.message}")

    # 6. 仪表盘渲染
    print("\n--- 6. 仪表盘渲染 ---")
    frame = sim.dashboard.render()
    print(f"  帧号: {frame['frame']}, 控件数: {len(frame['widgets'])}")
    for w in frame["widgets"]:
        print(f"  [{w['type']:8s}] {w['label']:6s}: {w['value']:.3f} {w['unit']} "
              f"({w['percent']:.1f}%) 区域={w['zone']}")

    # 7. 阈值预警测试
    print("\n--- 7. 阈值预警仿真 ---")
    # 模拟温度持续升高
    print("  模拟温度升高场景:")
    test_temps = [50, 55, 60, 63, 66, 70, 73, 76, 78, 80, 83, 86]
    for temp in test_temps:
        alarm = sim.alarm_mgr.check_value("temperature", temp)
        if alarm:
            print(f"    T={temp}°C -> [{alarm.level.name}] {alarm.threshold_name}")
        else:
            print(f"    T={temp}°C -> NORMAL")

    # 8. 报警确认
    print("\n--- 8. 报警确认 ---")
    if sim.alarm_mgr.alarm_history:
        first_alarm = sim.alarm_mgr.alarm_history[0]
        sim.alarm_mgr.acknowledge(first_alarm.alarm_id)
        print(f"  确认报警 #{first_alarm.alarm_id}: {first_alarm.message}")

    # 9. 采集统计
    print("\n--- 9. 采集统计 ---")
    stats = sim.acquisition.get_stats()
    print(f"  通道数: {stats['channels']}")
    print(f"  总采集数: {stats['total_acquired']}")
    print(f"  丢失样本: {stats['missed']}")
    print(f"  采样率: {stats['sample_rate']}Hz")

    # 10. 仪表盘配置
    print("\n--- 10. 仪表盘配置 ---")
    for gid, g in sim.dashboard.gauges.items():
        zones = len(g.color_zones)
        print(f"  {gid}: 类型={g.gauge_type.name}, 通道={g.channel}, "
              f"范围=[{g.min_val}, {g.max_val}], 颜色区={zones}")

    print("\n" + "=" * 60)
    print("实时监控仿真完成")
    print("=" * 60)


if __name__ == "__main__":
    run_real_time_monitoring_simulation()
