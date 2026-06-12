#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
环境监测仿真 - 温湿度+气压+空气质量融合+预警
==============================================
功能:
  1. 多传感器仿真 (DHT22/BMP280/MQ135/PM2.5/光照)
  2. 卡尔曼滤波 + 多传感器数据融合
  3. 环境质量综合评估 (AQI计算)
  4. 异常检测 + 多级预警
  5. 趋势预测 (指数平滑)
  6. 传感器故障模拟与诊断

依赖: numpy (必需), matplotlib (可选)
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
from enum import Enum
import time


# ============================================================
# 1. 传感器模型
# ============================================================

class SensorType(Enum):
    TEMPERATURE = "temperature"
    HUMIDITY = "humidity"
    PRESSURE = "pressure"
    CO2 = "co2"
    PM25 = "pm25"
    PM10 = "pm10"
    VOC = "voc"
    LIGHT = "light"
    NOISE = "noise"


@dataclass
class SensorConfig:
    """传感器配置"""
    sensor_type: SensorType
    unit: str
    range_min: float
    range_max: float
    accuracy: float  # 精度 (%)
    resolution: float
    response_time: float  # 秒
    noise_std: float
    drift_rate: float = 0.001  # 漂移率/小时


@dataclass
class Reading:
    """传感器读数"""
    sensor_type: SensorType
    value: float
    timestamp: float
    quality: float = 1.0  # 0-1 数据质量
    is_fault: bool = False


class SensorModel:
    """传感器仿真模型"""

    SENSOR_CONFIGS = {
        SensorType.TEMPERATURE: SensorConfig(
            SensorType.TEMPERATURE, "°C", -40, 80, 0.5, 0.1, 2.0, 0.1),
        SensorType.HUMIDITY: SensorConfig(
            SensorType.HUMIDITY, "%RH", 0, 100, 2.0, 0.1, 5.0, 0.3),
        SensorType.PRESSURE: SensorConfig(
            SensorType.PRESSURE, "hPa", 300, 1100, 0.1, 0.01, 1.0, 0.05),
        SensorType.CO2: SensorConfig(
            SensorType.CO2, "ppm", 0, 5000, 3.0, 1.0, 10.0, 5.0),
        SensorType.PM25: SensorConfig(
            SensorType.PM25, "μg/m³", 0, 500, 10.0, 0.1, 30.0, 2.0),
        SensorType.PM10: SensorConfig(
            SensorType.PM10, "μg/m³", 0, 1000, 10.0, 0.1, 30.0, 3.0),
        SensorType.VOC: SensorConfig(
            SensorType.VOC, "ppb", 0, 1000, 5.0, 1.0, 5.0, 3.0),
        SensorType.LIGHT: SensorConfig(
            SensorType.LIGHT, "lux", 0, 100000, 5.0, 1.0, 0.1, 10.0),
        SensorType.NOISE: SensorConfig(
            SensorType.NOISE, "dB", 30, 130, 1.5, 0.1, 0.1, 0.5),
    }

    def __init__(self, sensor_type: SensorType, fault_rate: float = 0.001):
        self.config = self.SENSOR_CONFIGS[sensor_type]
        self.sensor_type = sensor_type
        self.fault_rate = fault_rate
        self.total_drift = 0.0
        self.fault_state = None  # None, "stuck", "spike", "drift"

    def read(self, true_value: float, timestamp: float) -> Reading:
        """读取传感器值 (含噪声和故障)"""
        cfg = self.config
        value = true_value

        # 漂移
        self.total_drift += cfg.drift_rate * np.random.randn() / 3600
        value += self.total_drift

        # 噪声
        value += cfg.noise_std * np.random.randn()

        # 随机故障
        is_fault = False
        if np.random.random() < self.fault_rate:
            self.fault_state = np.random.choice(["stuck", "spike", "drift"])

        if self.fault_state == "stuck":
            value = true_value  # 卡住不变
            is_fault = True
        elif self.fault_state == "spike":
            value += np.random.choice([-1, 1]) * cfg.range_max * 0.5
            is_fault = True
            if np.random.random() > 0.1:  # 90%概率恢复
                self.fault_state = None
        elif self.fault_state == "drift":
            value += 50 * cfg.noise_std
            is_fault = True
            if np.random.random() > 0.05:
                self.fault_state = None

        # 限幅
        value = np.clip(value, cfg.range_min, cfg.range_max)

        # 数据质量
        quality = 1.0
        if is_fault:
            quality = 0.1
        elif abs(value - true_value) > 3 * cfg.noise_std:
            quality = 0.7

        return Reading(self.sensor_type, value, timestamp, quality, is_fault)


# ============================================================
# 2. 环境模型 (物理仿真)
# ============================================================

class EnvironmentModel:
    """环境物理模型 - 生成真实值"""

    def __init__(self, seed: Optional[int] = None):
        self.rng = np.random.default_rng(seed)
        self.time_offset = 0.0

    def get_true_values(self, t: float) -> Dict[SensorType, float]:
        """在时刻t的环境真实值"""
        hour = (t / 3600 + self.time_offset) % 24  # 一天中的小时

        # 温度: 日变化 + 随机
        temp_base = 22 + 5 * np.sin(2 * np.pi * (hour - 6) / 24)
        temp_noise = self.rng.normal(0, 0.3)
        temperature = temp_base + temp_noise

        # 湿度: 反温度趋势
        hum_base = 60 - 15 * np.sin(2 * np.pi * (hour - 6) / 24)
        humidity = np.clip(hum_base + self.rng.normal(0, 1), 20, 95)

        # 气压: 缓慢变化
        pressure = 1013.25 + 5 * np.sin(2 * np.pi * t / 86400) + self.rng.normal(0, 0.2)

        # CO2: 白天低/夜间高 + 室内效应
        co2_base = 400 + 200 * np.sin(2 * np.pi * (hour - 2) / 24)
        co2 = np.clip(co2_base + self.rng.normal(0, 10), 350, 2000)

        # PM2.5: 早晚高峰
        pm25_base = 35 + 20 * np.sin(2 * np.pi * (hour - 8) / 12)
        pm25 = np.clip(pm25_base + self.rng.exponential(5), 5, 300)

        # PM10: PM2.5的比例
        pm10 = pm25 * 1.5 + self.rng.normal(0, 5)

        # VOC
        voc = 100 + 50 * np.sin(2 * np.pi * hour / 24) + self.rng.normal(0, 10)

        # 光照: 夜间为0
        if 6 < hour < 18:
            light = 50000 * np.sin(np.pi * (hour - 6) / 12) + self.rng.normal(0, 1000)
        else:
            light = 100 + self.rng.normal(0, 20)

        # 噪声
        if 7 < hour < 22:
            noise = 55 + 10 * np.sin(2 * np.pi * (hour - 8) / 14) + self.rng.normal(0, 2)
        else:
            noise = 35 + self.rng.normal(0, 1)

        return {
            SensorType.TEMPERATURE: temperature,
            SensorType.HUMIDITY: humidity,
            SensorType.PRESSURE: pressure,
            SensorType.CO2: co2,
            SensorType.PM25: pm25,
            SensorType.PM10: pm10,
            SensorType.VOC: voc,
            SensorType.LIGHT: max(0, light),
            SensorType.NOISE: np.clip(noise, 30, 130),
        }

    def simulate_event(self, t: float, event: str, duration: float = 3600) -> Dict[SensorType, float]:
        """模拟环境事件"""
        values = self.get_true_values(t)

        if event == "fire":
            values[SensorType.TEMPERATURE] += 30 * min(t / duration, 1)
            values[SensorType.PM25] += 200 * min(t / duration, 1)
            values[SensorType.CO2] += 1000 * min(t / duration, 1)
            values[SensorType.VOC] += 500 * min(t / duration, 1)
        elif event == "flood":
            values[SensorType.HUMIDITY] = 95 + self.rng.normal(0, 1)
            values[SensorType.TEMPERATURE] -= 5
        elif event == "gas_leak":
            values[SensorType.VOC] += 400 * min(t / duration, 1)
            values[SensorType.CO2] += 500
        elif event == "heatwave":
            values[SensorType.TEMPERATURE] += 15
            values[SensorType.HUMIDITY] -= 20

        return values


# ============================================================
# 3. 卡尔曼滤波器
# ============================================================

class KalmanFilter1D:
    """一维卡尔曼滤波器"""

    def __init__(self, process_noise: float = 0.01, measurement_noise: float = 1.0,
                 initial_estimate: float = 0.0, initial_error: float = 100.0):
        self.x = initial_estimate  # 状态估计
        self.P = initial_error     # 估计误差
        self.Q = process_noise     # 过程噪声
        self.R = measurement_noise # 测量噪声
        self.K = 0                 # 卡尔曼增益

    def update(self, measurement: float) -> float:
        """更新估计"""
        # 预测
        x_pred = self.x
        P_pred = self.P + self.Q

        # 更新
        self.K = P_pred / (P_pred + self.R)
        self.x = x_pred + self.K * (measurement - x_pred)
        self.P = (1 - self.K) * P_pred

        return self.x

    @property
    def state(self) -> float:
        return self.x

    @property
    def uncertainty(self) -> float:
        return self.P


class MultiSensorKalmanFusion:
    """多传感器卡尔曼融合"""

    def __init__(self):
        self.filters: Dict[SensorType, KalmanFilter1D] = {}
        self.history: Dict[SensorType, List[float]] = {}

    def add_sensor(self, sensor_type: SensorType, config: SensorConfig):
        """添加传感器滤波器"""
        self.filters[sensor_type] = KalmanFilter1D(
            process_noise=0.01,
            measurement_noise=config.noise_std**2,
            initial_estimate=(config.range_min + config.range_max) / 2,
        )
        self.history[sensor_type] = []

    def fuse(self, readings: List[Reading]) -> Dict[SensorType, float]:
        """融合多个传感器读数"""
        results = {}
        for reading in readings:
            if reading.sensor_type in self.filters:
                filt = self.filters[reading.sensor_type]

                # 根据数据质量调整测量噪声
                original_R = filt.R
                if reading.quality < 0.5:
                    filt.R = original_R / reading.quality
                elif reading.is_fault:
                    filt.R = original_R * 10

                estimate = filt.update(reading.value)
                filt.R = original_R  # 恢复

                results[reading.sensor_type] = estimate
                self.history[reading.sensor_type].append(estimate)

        return results


# ============================================================
# 4. 空气质量评估 (AQI)
# ============================================================

class AQICalculator:
    """空气质量指数计算器 (参照国标HJ 633-2012)"""

    # AQI分级
    LEVELS = [
        (0, 50, "优", "green", "空气质量令人满意，基本无空气污染"),
        (51, 100, "良", "yellow", "空气质量可接受，某些污染物可能对少数人有影响"),
        (101, 150, "轻度污染", "orange", "敏感人群出现症状"),
        (151, 200, "中度污染", "red", "进一步加剧症状"),
        (201, 300, "重度污染", "purple", "健康警告，所有人受影响"),
        (301, 500, "严重污染", "maroon", "健康警告：所有人可能受到严重影响"),
    ]

    # 各污染物IAQI分指数计算 (简化)
    BREAKPOINTS = {
        "pm25":   [(0, 35, 0, 50), (35, 75, 50, 100), (75, 115, 100, 150),
                   (115, 150, 150, 200), (150, 250, 200, 300), (250, 500, 300, 500)],
        "pm10":   [(0, 50, 0, 50), (50, 150, 50, 100), (150, 250, 100, 150),
                   (250, 350, 150, 200), (350, 420, 200, 300), (420, 600, 300, 500)],
        "co2":    [(0, 400, 0, 25), (400, 1000, 25, 50), (1000, 2000, 50, 100),
                   (2000, 5000, 100, 200)],
        "voc":    [(0, 200, 0, 50), (200, 400, 50, 100), (400, 600, 100, 150),
                   (600, 1000, 150, 200)],
    }

    @classmethod
    def calc_iaqi(cls, pollutant: str, concentration: float) -> int:
        """计算单项空气质量指数"""
        breakpoints = cls.BREAKPOINTS.get(pollutant, [])
        for c_low, c_high, i_low, i_high in breakpoints:
            if c_low <= concentration <= c_high:
                return int((i_high - i_low) / (c_high - c_low) * (concentration - c_low) + i_low)
        if breakpoints and concentration > breakpoints[-1][1]:
            return 500
        return 0

    @classmethod
    def calc_aqi(cls, values: Dict[str, float]) -> Dict:
        """计算综合AQI"""
        iaqi = {}
        for pollutant in ["pm25", "pm10", "co2", "voc"]:
            if pollutant in values:
                iaqi[pollutant] = cls.calc_iaqi(pollutant, values[pollutant])

        if not iaqi:
            return {"aqi": 0, "level": "未知", "primary": "无"}

        aqi = max(iaqi.values())
        primary = max(iaqi, key=iaqi.get)

        level_info = cls.LEVELS[0]
        for low, high, level, color, desc in cls.LEVELS:
            if low <= aqi <= high:
                level_info = (low, high, level, color, desc)
                break

        return {
            "aqi": aqi,
            "level": level_info[2],
            "color": level_info[3],
            "description": level_info[4],
            "primary_pollutant": primary,
            "iaqi": iaqi,
        }


# ============================================================
# 5. 预警系统
# ============================================================

class AlertLevel(Enum):
    NORMAL = 0
    NOTICE = 1
    WARNING = 2
    DANGER = 3
    EMERGENCY = 4


@dataclass
class Alert:
    level: AlertLevel
    sensor_type: SensorType
    message: str
    value: float
    threshold: float
    timestamp: float


class AlertSystem:
    """多级预警系统"""

    def __init__(self):
        # 各传感器预警阈值
        self.thresholds = {
            SensorType.TEMPERATURE: {
                AlertLevel.NOTICE: (0, 35),
                AlertLevel.WARNING: (-10, 40),
                AlertLevel.DANGER: (-20, 45),
                AlertLevel.EMERGENCY: (-30, 50),
            },
            SensorType.HUMIDITY: {
                AlertLevel.NOTICE: (20, 80),
                AlertLevel.WARNING: (10, 90),
                AlertLevel.DANGER: (5, 95),
            },
            SensorType.CO2: {
                AlertLevel.NOTICE: (0, 1000),
                AlertLevel.WARNING: (0, 2000),
                AlertLevel.DANGER: (0, 5000),
            },
            SensorType.PM25: {
                AlertLevel.NOTICE: (0, 75),
                AlertLevel.WARNING: (0, 115),
                AlertLevel.DANGER: (0, 250),
                AlertLevel.EMERGENCY: (0, 500),
            },
        }

        self.alert_history: List[Alert] = []
        self.active_alerts: Dict[Tuple[SensorType, AlertLevel], Alert] = {}

    def check(self, sensor_type: SensorType, value: float,
              timestamp: float) -> Optional[Alert]:
        """检查是否触发预警"""
        if sensor_type not in self.thresholds:
            return None

        for level in [AlertLevel.EMERGENCY, AlertLevel.DANGER,
                      AlertLevel.WARNING, AlertLevel.NOTICE]:
            if level in self.thresholds[sensor_type]:
                low, high = self.thresholds[sensor_type][level]
                if value < low or value > high:
                    key = (sensor_type, level)
                    if key not in self.active_alerts:
                        direction = "超过上限" if value > high else "低于下限"
                        alert = Alert(
                            level=level,
                            sensor_type=sensor_type,
                            message=f"{sensor_type.value} {direction}: {value:.1f}",
                            value=value,
                            threshold=high if value > high else low,
                            timestamp=timestamp,
                        )
                        self.active_alerts[key] = alert
                        self.alert_history.append(alert)
                        return alert
                    return None
        return None

    def get_active_summary(self) -> str:
        """获取当前活跃预警摘要"""
        if not self.active_alerts:
            return "无活跃预警"
        lines = []
        for (st, level), alert in self.active_alerts.items():
            lines.append(f"[{level.name}] {alert.message}")
        return "\n".join(lines)


# ============================================================
# 6. 趋势预测
# ============================================================

class TrendPredictor:
    """指数平滑趋势预测"""

    def __init__(self, alpha: float = 0.3, beta: float = 0.1):
        self.alpha = alpha  # 水平平滑系数
        self.beta = beta    # 趋势平滑系数
        self.level = None
        self.trend = 0.0
        self.history: List[float] = []

    def update(self, value: float) -> float:
        """更新并返回下一步预测"""
        self.history.append(value)

        if self.level is None:
            self.level = value
            self.trend = 0.0
            return value

        prev_level = self.level
        self.level = self.alpha * value + (1 - self.alpha) * (self.level + self.trend)
        self.trend = self.beta * (self.level - prev_level) + (1 - self.beta) * self.trend

        return self.level + self.trend

    def predict(self, steps: int = 1) -> List[float]:
        """预测未来N步"""
        if self.level is None:
            return []
        return [self.level + self.trend * i for i in range(1, steps + 1)]

    def detect_anomaly(self, value: float, threshold: float = 3.0) -> bool:
        """异常检测 (基于残差)"""
        if len(self.history) < 5:
            return False
        residual = value - self.level
        std = np.std(self.history[-20:]) if len(self.history) >= 20 else np.std(self.history)
        return abs(residual) > threshold * std


# ============================================================
# 7. 综合监测系统
# ============================================================

class EnvironmentalMonitor:
    """环境监测综合系统"""

    def __init__(self, sample_interval: float = 60.0):
        self.interval = sample_interval  # 采样间隔(秒)
        self.env_model = EnvironmentModel(seed=42)
        self.sensors: Dict[SensorType, SensorModel] = {}
        self.fusion = MultiSensorKalmanFusion()
        self.alert_sys = AlertSystem()
        self.predictors: Dict[SensorType, TrendPredictor] = {}

        # 初始化传感器
        for st in SensorType:
            self.sensors[st] = SensorModel(st)
            self.fusion.add_sensor(st, SensorModel.SENSOR_CONFIGS[st])
            self.predictors[st] = TrendPredictor()

        # 数据记录
        self.readings_log: Dict[SensorType, List[Tuple[float, float, float]]] = {
            st: [] for st in SensorType
        }

    def step(self, t: float, event: Optional[str] = None) -> Dict:
        """执行一个监测步骤"""
        # 获取真实值
        if event:
            true_values = self.env_model.simulate_event(t, event)
        else:
            true_values = self.env_model.get_true_values(t)

        # 传感器读数
        readings = []
        raw_values = {}
        for st, sensor in self.sensors.items():
            reading = sensor.read(true_values[st], t)
            readings.append(reading)
            raw_values[st] = reading.value

        # 卡尔曼融合
        fused = self.fusion.fuse(readings)

        # 预测与异常检测
        anomalies = {}
        predictions = {}
        for st in SensorType:
            if st in fused:
                pred = self.predictors[st].update(fused[st])
                predictions[st] = self.predictors[st].predict(steps=5)
                anomalies[st] = self.predictors[st].detect_anomaly(fused[st])

        # AQI计算
        aqi_values = {}
        for key, st in [("pm25", SensorType.PM25), ("pm10", SensorType.PM10),
                         ("co2", SensorType.CO2), ("voc", SensorType.VOC)]:
            if st in fused:
                aqi_values[key] = fused[st]
        aqi_result = AQICalculator.calc_aqi(aqi_values)

        # 预警检查
        alerts = []
        for st in SensorType:
            if st in fused:
                alert = self.alert_sys.check(st, fused[st], t)
                if alert:
                    alerts.append(alert)

        # 记录数据
        for st in SensorType:
            if st in fused:
                self.readings_log[st].append((t, true_values[st], fused[st]))

        return {
            "timestamp": t,
            "true_values": {st: v for st, v in true_values.items()},
            "raw_readings": raw_values,
            "fused_values": fused,
            "predictions": predictions,
            "anomalies": anomalies,
            "aqi": aqi_result,
            "alerts": alerts,
            "faulty_sensors": [r.sensor_type for r in readings if r.is_fault],
        }

    def run_simulation(self, duration_hours: float = 24.0,
                       event: Optional[str] = None,
                       event_time: Optional[float] = None) -> List[Dict]:
        """运行完整仿真"""
        duration_sec = duration_hours * 3600
        t = 0.0
        results = []

        print(f"开始环境监测仿真 ({duration_hours}小时)...")
        while t < duration_sec:
            current_event = None
            if event and event_time and abs(t - event_time) < 3600:
                current_event = event

            result = self.step(t, current_event)
            results.append(result)

            if result["alerts"]:
                for alert in result["alerts"]:
                    print(f"  ⚠ [{alert.level.name}] t={t/3600:.1f}h: {alert.message}")

            t += self.interval

        # 统计
        total_faults = sum(len(r["faulty_sensors"]) for r in results)
        total_alerts = sum(len(r["alerts"]) for r in results)
        print(f"\n仿真完成: {len(results)}个采样点, {total_faults}次传感器故障, {total_alerts}次预警")

        return results


# ============================================================
# 8. 可视化
# ============================================================

def plot_monitoring_results(results: List[Dict], save_path: Optional[str] = None):
    """绘制监测结果"""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib未安装, 跳过绘图")
        return

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("环境监测仿真结果", fontsize=14)

    times = [r["timestamp"] / 3600 for r in results]

    # 温度
    ax = axes[0, 0]
    true_temp = [r["true_values"][SensorType.TEMPERATURE] for r in results]
    fused_temp = [r["fused_values"].get(SensorType.TEMPERATURE, np.nan) for r in results]
    ax.plot(times, true_temp, 'b-', alpha=0.3, label="真实值")
    ax.plot(times, fused_temp, 'r-', label="融合值")
    ax.set_title("温度 (°C)")
    ax.legend()

    # 湿度
    ax = axes[0, 1]
    true_hum = [r["true_values"][SensorType.HUMIDITY] for r in results]
    fused_hum = [r["fused_values"].get(SensorType.HUMIDITY, np.nan) for r in results]
    ax.plot(times, true_hum, 'b-', alpha=0.3)
    ax.plot(times, fused_hum, 'r-')
    ax.set_title("湿度 (%RH)")

    # PM2.5
    ax = axes[1, 0]
    true_pm = [r["true_values"][SensorType.PM25] for r in results]
    fused_pm = [r["fused_values"].get(SensorType.PM25, np.nan) for r in results]
    ax.plot(times, true_pm, 'b-', alpha=0.3)
    ax.plot(times, fused_pm, 'r-')
    ax.axhline(75, color='orange', linestyle='--', label="预警线")
    ax.axhline(115, color='red', linestyle='--', label="危险线")
    ax.set_title("PM2.5 (μg/m³)")
    ax.legend()

    # CO2
    ax = axes[1, 1]
    true_co2 = [r["true_values"][SensorType.CO2] for r in results]
    fused_co2 = [r["fused_values"].get(SensorType.CO2, np.nan) for r in results]
    ax.plot(times, true_co2, 'b-', alpha=0.3)
    ax.plot(times, fused_co2, 'r-')
    ax.set_title("CO2 (ppm)")

    # AQI
    ax = axes[2, 0]
    aqi_values = [r["aqi"]["aqi"] for r in results]
    ax.fill_between(times, 0, aqi_values, alpha=0.5)
    ax.set_title("AQI指数")
    ax.set_xlabel("时间 (h)")

    # 预警事件
    ax = axes[2, 1]
    alert_times = [r["timestamp"]/3600 for r in results if r["alerts"]]
    alert_levels = [r["alerts"][0].level.value for r in results if r["alerts"]]
    if alert_times:
        ax.scatter(alert_times, alert_levels, c='red', s=50, zorder=5)
    ax.set_title("预警事件")
    ax.set_xlabel("时间 (h)")
    ax.set_yticks([0, 1, 2, 3, 4])
    ax.set_yticklabels(["正常", "注意", "警告", "危险", "紧急"])

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"[OK] 图像已保存: {save_path}")
    plt.show()


# ============================================================
# 9. 主程序
# ============================================================

def main():
    print("=" * 60)
    print("  环境监测仿真 - 多传感器融合 + 预警系统")
    print("=" * 60)

    # 创建监测系统
    monitor = EnvironmentalMonitor(sample_interval=60.0)

    # 1. 正常运行24小时
    print("\n[1] 正常环境24小时监测...")
    results_normal = monitor.run_simulation(duration_hours=24.0)

    # 2. 火灾事件仿真
    print("\n[2] 火灾事件仿真...")
    monitor2 = EnvironmentalMonitor(sample_interval=30.0)
    results_fire = monitor2.run_simulation(
        duration_hours=6.0, event="fire", event_time=7200)

    # 3. 单独功能测试
    print("\n[3] AQI计算测试...")
    test_values = {"pm25": 85, "pm10": 120, "co2": 800, "voc": 300}
    aqi = AQICalculator.calc_aqi(test_values)
    print(f"  AQI: {aqi['aqi']} ({aqi['level']})")
    print(f"  主要污染物: {aqi['primary_pollutant']}")
    print(f"  各分指数: {aqi['iaqi']}")

    # 4. 预测测试
    print("\n[4] 趋势预测测试...")
    predictor = TrendPredictor(alpha=0.3, beta=0.1)
    for i in range(50):
        predictor.update(20 + 5 * np.sin(i * 0.2) + np.random.randn() * 0.5)
    future = predictor.predict(steps=10)
    print(f"  未来10步预测: {[f'{v:.1f}' for v in future]}")

    # 5. 传感器故障诊断
    print("\n[5] 传感器故障统计...")
    total_readings = sum(len(monitor.readings_log[st]) for st in SensorType)
    print(f"  总采样点: {total_readings}")

    # 输出最后一条数据
    print("\n[6] 最后一条融合数据:")
    last = results_normal[-1]
    for st in [SensorType.TEMPERATURE, SensorType.HUMIDITY, SensorType.PM25, SensorType.CO2]:
        v = last["fused_values"].get(st, "N/A")
        unit = SensorModel.SENSOR_CONFIGS[st].unit
        print(f"  {st.value:15s}: {v:>8.1f} {unit}")
    print(f"  AQI: {last['aqi']['aqi']} ({last['aqi']['level']})")

    print("\n" + "=" * 60)
    print("  仿真完成!")
    print("=" * 60)

    try:
        plot_monitoring_results(results_normal)
    except Exception as e:
        print(f"[INFO] 绘图跳过: {e}")

    return results_normal


if __name__ == "__main__":
    main()
