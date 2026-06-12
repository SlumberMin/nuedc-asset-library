#!/usr/bin/env python3
"""
传感器模拟器 - 模拟各种传感器数据输出
=============================================
功能：
  - 支持温度、湿度、气压、光照、距离、电压、电流、加速度等传感器
  - 可配置噪声、漂移、采样率
  - 输出 CSV/JSON 或实时流式数据
  - 支持故障注入（断线、饱和、噪声突增）

用法：
  python sensor_simulator.py --sensor temperature --samples 1000 --noise 0.5
  python sensor_simulator.py --sensor all --format csv --output data.csv
  python sensor_simulator.py --sensor ultrasonic --fault stuck --samples 500
"""

import argparse
import json
import math
import random
import sys
import time
import csv
import io
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Callable, Optional


# ============================================================
# 传感器模型定义
# ============================================================

@dataclass
class SensorConfig:
    """传感器配置参数"""
    name: str                # 传感器名称
    unit: str                # 单位
    min_val: float           # 最小值
    max_val: float           # 最大值
    typical_val: float       # 典型值
    noise_std: float         # 噪声标准差
    drift_rate: float = 0.0  # 漂移率 (单位/秒)
    resolution: float = 0.01 # 分辨率
    response_time: float = 0.1  # 响应时间(秒)


# 预定义传感器库
SENSOR_LIBRARY: Dict[str, SensorConfig] = {
    "temperature": SensorConfig(
        name="温度传感器(NTC/DS18B20)", unit="°C",
        min_val=-40, max_val=125, typical_val=25,
        noise_std=0.2, drift_rate=0.001, resolution=0.0625
    ),
    "humidity": SensorConfig(
        name="湿度传感器(DHT22)", unit="%RH",
        min_val=0, max_val=100, typical_val=50,
        noise_std=1.0, drift_rate=0.01, resolution=0.1
    ),
    "pressure": SensorConfig(
        name="气压传感器(BMP280)", unit="hPa",
        min_val=300, max_val=1100, typical_val=1013.25,
        noise_std=0.5, drift_rate=0.005, resolution=0.01
    ),
    "light": SensorConfig(
        name="光照传感器(BH1750)", unit="lux",
        min_val=0, max_val=65535, typical_val=500,
        noise_std=10.0, drift_rate=0.0, resolution=1.0
    ),
    "ultrasonic": SensorConfig(
        name="超声波传感器(HC-SR04)", unit="cm",
        min_val=2, max_val=400, typical_val=50,
        noise_std=1.5, drift_rate=0.0, resolution=0.1
    ),
    "voltage": SensorConfig(
        name="电压传感器(ADC)", unit="V",
        min_val=0, max_val=3.3, typical_val=1.65,
        noise_std=0.005, drift_rate=0.0001, resolution=0.001
    ),
    "current": SensorConfig(
        name="电流传感器(ACS712)", unit="A",
        min_val=-5, max_val=5, typical_val=0.5,
        noise_std=0.02, drift_rate=0.001, resolution=0.001
    ),
    "accelerometer": SensorConfig(
        name="加速度传感器(MPU6050)", unit="g",
        min_val=-16, max_val=16, typical_val=0,
        noise_std=0.05, drift_rate=0.0, resolution=0.001
    ),
    "gyroscope": SensorConfig(
        name="陀螺仪(MPU6050)", unit="°/s",
        min_val=-2000, max_val=2000, typical_val=0,
        noise_std=0.5, drift_rate=0.01, resolution=0.01
    ),
    "magnetic": SensorConfig(
        name="磁力传感器(QMC5883L)", unit="μT",
        min_val=-800, max_val=800, typical_val=0,
        noise_std=2.0, drift_rate=0.0, resolution=0.1
    ),
    "ir_temp": SensorConfig(
        name="红外测温(MLX90614)", unit="°C",
        min_val=-70, max_val=380, typical_val=25,
        noise_std=0.3, drift_rate=0.002, resolution=0.02
    ),
    "loadcell": SensorConfig(
        name="称重传感器(HX711)", unit="g",
        min_val=0, max_val=5000, typical_val=500,
        noise_std=2.0, drift_rate=0.01, resolution=0.1
    ),
}


# ============================================================
# 信号生成器
# ============================================================

class SignalGenerator:
    """信号生成器 - 生成各种测试信号"""

    @staticmethod
    def constant(value: float, n: int) -> List[float]:
        """恒定值信号"""
        return [value] * n

    @staticmethod
    def ramp(start: float, end: float, n: int) -> List[float]:
        """斜坡信号"""
        if n <= 1:
            return [start]
        step = (end - start) / (n - 1)
        return [start + i * step for i in range(n)]

    @staticmethod
    def sine(amplitude: float, frequency: float, offset: float,
             sample_rate: float, n: int) -> List[float]:
        """正弦信号"""
        values = []
        for i in range(n):
            t = i / sample_rate
            values.append(offset + amplitude * math.sin(2 * math.pi * frequency * t))
        return values

    @staticmethod
    def step(low: float, high: float, step_at: int, n: int) -> List[float]:
        """阶跃信号"""
        return [low if i < step_at else high for i in range(n)]

    @staticmethod
    def square(amplitude: float, frequency: float, offset: float,
               sample_rate: float, n: int) -> List[float]:
        """方波信号"""
        values = []
        for i in range(n):
            t = i / sample_rate
            period = 1.0 / frequency
            phase = (t % period) / period
            values.append(offset + amplitude if phase < 0.5 else offset - amplitude)
        return values

    @staticmethod
    def noise_walk(start: float, step_size: float, n: int) -> List[float]:
        """随机游走信号"""
        values = [start]
        for _ in range(n - 1):
            values.append(values[-1] + random.gauss(0, step_size))
        return values

    @staticmethod
    def impulse(amplitude: float, position: int, n: int) -> List[float]:
        """脉冲信号"""
        values = [0.0] * n
        if 0 <= position < n:
            values[position] = amplitude
        return values


# ============================================================
# 传感器模拟器核心
# ============================================================

class SensorSimulator:
    """传感器模拟器 - 生成带噪声和漂移的传感器数据"""

    def __init__(self, config: SensorConfig, sample_rate: float = 100.0):
        self.config = config
        self.sample_rate = sample_rate
        self.elapsed = 0.0
        self.fault_mode = None   # 故障模式
        self.fault_params = {}

    def set_fault(self, fault_type: str, **params):
        """设置故障模式
        fault_type: stuck(卡死), saturated(饱和), noisy(噪声突增),
                    offset(偏移), drift(漂移), intermittent(间歇)
        """
        self.fault_mode = fault_type
        self.fault_params = params

    def _apply_fault(self, value: float, index: int) -> float:
        """应用故障模式"""
        if self.fault_mode is None:
            return value

        if self.fault_mode == "stuck":
            # 卡死在某个值
            stuck_val = self.fault_params.get("value", self.config.typical_val)
            fault_start = self.fault_params.get("start", 0)
            return stuck_val if index >= fault_start else value

        elif self.fault_mode == "saturated":
            # 饱和在最大值
            return self.config.max_val

        elif self.fault_mode == "noisy":
            # 噪声突增
            fault_start = self.fault_params.get("start", 0)
            if index >= fault_start:
                noise_mult = self.fault_params.get("multiplier", 10)
                return value + random.gauss(0, self.config.noise_std * noise_mult)
            return value

        elif self.fault_mode == "offset":
            # 固定偏移
            offset = self.fault_params.get("offset", self.config.typical_val * 0.1)
            fault_start = self.fault_params.get("start", 0)
            return value + offset if index >= fault_start else value

        elif self.fault_mode == "drift":
            # 异常漂移
            fault_start = self.fault_params.get("start", 0)
            drift_speed = self.fault_params.get("speed", 0.1)
            if index >= fault_start:
                return value + drift_speed * (index - fault_start)
            return value

        elif self.fault_mode == "intermittent":
            # 间歇性故障
            prob = self.fault_params.get("probability", 0.1)
            if random.random() < prob:
                return float('nan')
            return value

        return value

    def generate(self, base_values: List[float]) -> List[dict]:
        """基于基值序列生成带噪声的传感器数据"""
        results = []
        for i, base in enumerate(base_values):
            # 添加高斯噪声
            noisy = base + random.gauss(0, self.config.noise_std)

            # 添加漂移
            t = i / self.sample_rate
            noisy += self.config.drift_rate * t

            # 应用故障
            noisy = self._apply_fault(noisy, i)

            # 量化
            if self.config.resolution > 0:
                noisy = round(noisy / self.config.resolution) * self.config.resolution

            # 限幅
            if not math.isnan(noisy):
                noisy = max(self.config.min_val, min(self.config.max_val, noisy))

            results.append({
                "index": i,
                "time": round(t, 6),
                "value": round(noisy, 6),
                "unit": self.config.unit,
                "raw_adc": self._to_adc(noisy),
            })
        return results

    def _to_adc(self, value: float) -> int:
        """将物理值映射到12位ADC值"""
        if math.isnan(value):
            return 0
        normalized = (value - self.config.min_val) / (self.config.max_val - self.config.min_val)
        return max(0, min(4095, int(normalized * 4095)))

    def generate_realtime(self, base_values: List[float], callback: Callable):
        """实时生成数据（带延时模拟）"""
        for i, base in enumerate(base_values):
            result = self.generate([base])[0]
            callback(result)
            time.sleep(1.0 / self.sample_rate)


# ============================================================
# CLI 接口
# ============================================================

def list_sensors():
    """列出所有支持的传感器"""
    print("\n  支持的传感器类型:")
    print("  " + "=" * 60)
    for key, cfg in SENSOR_LIBRARY.items():
        print(f"  {key:15s} | {cfg.name:25s} | {cfg.unit:6s} | "
              f"范围: [{cfg.min_val}, {cfg.max_val}]")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="传感器模拟器 - 模拟各种传感器数据输出",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s --sensor temperature --samples 100 --signal sine
  %(prog)s --sensor all --format csv --output sensors.csv
  %(prog)s --sensor ultrasonic --fault stuck --samples 200
  %(prog)s --list
        """
    )

    parser.add_argument("--list", action="store_true", help="列出所有支持的传感器")
    parser.add_argument("--sensor", "-s", type=str, default="temperature",
                        help="传感器类型 (默认: temperature, 或 'all')")
    parser.add_argument("--samples", "-n", type=int, default=100,
                        help="采样点数 (默认: 100)")
    parser.add_argument("--rate", "-r", type=float, default=100.0,
                        help="采样率 Hz (默认: 100)")
    parser.add_argument("--noise", type=float, default=None,
                        help="自定义噪声标准差")
    parser.add_argument("--signal", type=str, default="constant",
                        choices=["constant", "ramp", "sine", "step",
                                 "square", "noise_walk", "impulse"],
                        help="基信号类型 (默认: constant)")
    parser.add_argument("--amplitude", type=float, default=None,
                        help="信号幅度")
    parser.add_argument("--frequency", type=float, default=1.0,
                        help="信号频率 Hz (默认: 1.0)")
    parser.add_argument("--format", "-f", type=str, default="table",
                        choices=["table", "csv", "json"],
                        help="输出格式 (默认: table)")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="输出文件路径")
    parser.add_argument("--fault", type=str, default=None,
                        choices=["stuck", "saturated", "noisy", "offset",
                                 "drift", "intermittent"],
                        help="故障注入模式")
    parser.add_argument("--fault-start", type=int, default=0,
                        help="故障起始样本索引")
    parser.add_argument("--realtime", action="store_true",
                        help="实时输出模式")
    parser.add_argument("--seed", type=int, default=None,
                        help="随机种子 (可重复)")

    args = parser.parse_args()

    if args.list:
        list_sensors()
        return

    if args.seed is not None:
        random.seed(args.seed)

    # 确定要模拟的传感器列表
    if args.sensor == "all":
        sensors = list(SENSOR_LIBRARY.keys())
    else:
        sensors = [s.strip() for s in args.sensor.split(",")]
        for s in sensors:
            if s not in SENSOR_LIBRARY:
                print(f"错误: 未知传感器 '{s}'，使用 --list 查看支持列表")
                sys.exit(1)

    all_results = {}

    for sensor_name in sensors:
        cfg = SENSOR_LIBRARY[sensor_name]

        # 创建模拟器
        sim = SensorSimulator(cfg, sample_rate=args.rate)

        # 自定义噪声
        if args.noise is not None:
            cfg.noise_std = args.noise

        # 设置故障
        if args.fault:
            sim.set_fault(args.fault, start=args.fault_start,
                         value=cfg.typical_val, multiplier=10,
                         offset=cfg.typical_val * 0.1, speed=0.1,
                         probability=0.1)

        # 生成基信号
        sg = SignalGenerator()
        amp = args.amplitude if args.amplitude else (cfg.max_val - cfg.min_val) * 0.1

        if args.signal == "constant":
            base_values = sg.constant(cfg.typical_val, args.samples)
        elif args.signal == "ramp":
            base_values = sg.ramp(cfg.min_val * 0.3, cfg.max_val * 0.7, args.samples)
        elif args.signal == "sine":
            base_values = sg.sine(amp, args.frequency, cfg.typical_val,
                                  args.rate, args.samples)
        elif args.signal == "step":
            base_values = sg.step(cfg.typical_val - amp, cfg.typical_val + amp,
                                  args.samples // 2, args.samples)
        elif args.signal == "square":
            base_values = sg.square(amp, args.frequency, cfg.typical_val,
                                    args.rate, args.samples)
        elif args.signal == "noise_walk":
            base_values = sg.noise_walk(cfg.typical_val, cfg.noise_std * 2, args.samples)
        elif args.signal == "impulse":
            base_values = sg.impulse(amp, args.samples // 2, args.samples)
        else:
            base_values = sg.constant(cfg.typical_val, args.samples)

        # 生成数据
        if args.realtime:
            print(f"\n[{cfg.name}] 实时数据流 (Ctrl+C 停止):")
            try:
                sim.generate_realtime(base_values, lambda d: print(
                    f"  t={d['time']:.3f}s  {d['value']:.4f} {d['unit']}  "
                    f"ADC={d['raw_adc']}"
                ))
            except KeyboardInterrupt:
                print("\n已停止")
        else:
            results = sim.generate(base_values)
            all_results[sensor_name] = results

    # 输出结果
    output_lines = []

    if args.format == "table":
        for name, results in all_results.items():
            cfg = SENSOR_LIBRARY[name]
            output_lines.append(f"\n  {cfg.name} ({cfg.unit})")
            output_lines.append(f"  {'=' * 55}")
            output_lines.append(f"  {'Index':>6s} {'Time(s)':>10s} {'Value':>12s} {'ADC':>6s}")
            output_lines.append(f"  {'-' * 55}")
            # 最多显示50行
            display = results[:50]
            for d in display:
                output_lines.append(
                    f"  {d['index']:6d} {d['time']:10.4f} {d['value']:12.4f} {d['raw_adc']:6d}"
                )
            if len(results) > 50:
                output_lines.append(f"  ... (共 {len(results)} 个样本)")
            # 统计信息
            vals = [d['value'] for d in results if not math.isnan(d['value'])]
            if vals:
                output_lines.append(f"  统计: min={min(vals):.4f}  max={max(vals):.4f}  "
                                   f"mean={sum(vals)/len(vals):.4f}  "
                                   f"std={_std(vals):.6f}")

    elif args.format == "csv":
        for name, results in all_results.items():
            cfg = SENSOR_LIBRARY[name]
            output_lines.append(f"# 传感器: {cfg.name}")
            output_lines.append(f"# 单位: {cfg.unit}")
            output_lines.append("index,time,value,unit,raw_adc")
            for d in results:
                output_lines.append(
                    f"{d['index']},{d['time']},{d['value']},{d['unit']},{d['raw_adc']}"
                )

    elif args.format == "json":
        output_lines.append(json.dumps(all_results, indent=2, ensure_ascii=False))

    output_text = "\n".join(output_lines)

    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(output_text)
        print(f"已保存到: {args.output}")
    else:
        print(output_text)


def _std(values: List[float]) -> float:
    """计算标准差"""
    if len(values) < 2:
        return 0.0
    mean = sum(values) / len(values)
    variance = sum((x - mean) ** 2 for x in values) / (len(values) - 1)
    return math.sqrt(variance)


if __name__ == "__main__":
    main()
