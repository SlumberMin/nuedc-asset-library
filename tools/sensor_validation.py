#!/usr/bin/env python3
"""
传感器验证工具 - Sensor Validation Tool
==========================================
功能:
  - 精度评估 (Accuracy / Error)
  - 重复性测试 (Repeatability)
  - 线性度分析 (Linearity)
  - 温度特性 (Temperature Drift / TCS)
  - 噪声分析 (Noise Density / SNR)
  - 响应时间测量
  - 综合验证报告
用法:
  python sensor_validation.py accuracy --ref 100.0 --readings 99.8 100.1 100.3 99.9
  python sensor_validation.py repeatability --readings 1.02 1.01 1.03 1.00 1.02 1.01
  python sensor_validation.py linearity --csv cal_data.csv
  python sensor_validation.py temperature --csv temp_drift.csv
  python sensor_validation.py noise --csv noise_data.csv
  python sensor_validation.py report --config validation_config.json
"""

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional, Tuple


# ── 数据结构 ──────────────────────────────────────────────────────────────────

@dataclass
class ValidationConfig:
    """验证配置"""
    sensor_name: str = "未命名传感器"
    sensor_type: str = "generic"        # generic, temperature, pressure, imu, current, voltage
    unit: str = ""
    nominal_range_min: float = 0.0
    nominal_range_max: float = 100.0
    nominal_accuracy: float = 1.0       # ±%FS 或 ±绝对值
    nominal_resolution: float = 0.1
    operating_temp_min: float = -20.0
    operating_temp_max: float = 85.0


@dataclass
class AccuracyResult:
    """精度评估结果"""
    reference_value: float = 0.0
    mean_reading: float = 0.0
    std_deviation: float = 0.0
    absolute_error: float = 0.0
    relative_error_pct: float = 0.0
    max_error: float = 0.0
    rmse: float = 0.0
    within_spec: bool = False


@dataclass
class RepeatabilityResult:
    """重复性测试结果"""
    n_readings: int = 0
    mean: float = 0.0
    std_deviation: float = 0.0
    cv_pct: float = 0.0             # 变异系数
    range_val: float = 0.0          # 极差
    readings: List[float] = field(default_factory=list)
    outliers: List[float] = field(default_factory=list)


@dataclass
class LinearityResult:
    """线性度分析结果"""
    slope: float = 0.0
    intercept: float = 0.0
    r_squared: float = 0.0
    max_deviation: float = 0.0      # 最大线性偏差
    linearity_pct_fs: float = 0.0   # 线性度 %FS
    fit_points: List[Dict] = field(default_factory=list)


@dataclass
class TemperatureResult:
    """温度特性结果"""
    tcs: float = 0.0                # 温度灵敏度系数 (%/℃)
    offset_drift: float = 0.0       # 零漂 (单位/℃)
    sensitivity_drift: float = 0.0  # 灵敏度漂移
    temp_coefficients: Dict = field(default_factory=dict)
    compensable: bool = False


@dataclass
class NoiseResult:
    """噪声分析结果"""
    peak_to_peak: float = 0.0
    rms_noise: float = 0.0
    snr_db: float = 0.0
    noise_density: float = 0.0      # 噪声密度 (单位/√Hz)
    enob: float = 0.0               # 有效位数
    signal_bandwidth: float = 0.0


# ── 统计计算核心 ──────────────────────────────────────────────────────────────

class Statistics:
    """统计计算工具类"""

    @staticmethod
    def mean(data: List[float]) -> float:
        if not data:
            return 0.0
        return sum(data) / len(data)

    @staticmethod
    def variance(data: List[float]) -> float:
        if len(data) < 2:
            return 0.0
        m = Statistics.mean(data)
        return sum((x - m) ** 2 for x in data) / (len(data) - 1)

    @staticmethod
    def std_dev(data: List[float]) -> float:
        return math.sqrt(Statistics.variance(data))

    @staticmethod
    def rmse(errors: List[float]) -> float:
        if not errors:
            return 0.0
        return math.sqrt(sum(e ** 2 for e in errors) / len(errors))

    @staticmethod
    def linear_regression(x: List[float], y: List[float]) -> Tuple[float, float, float]:
        """
        最小二乘线性回归
        返回: (slope, intercept, r_squared)
        """
        n = len(x)
        if n < 2:
            return 0.0, 0.0, 0.0

        sum_x = sum(x)
        sum_y = sum(y)
        sum_xy = sum(a * b for a, b in zip(x, y))
        sum_x2 = sum(a * a for a in x)
        sum_y2 = sum(b * b for b in y)

        denom = n * sum_x2 - sum_x ** 2
        if abs(denom) < 1e-15:
            return 0.0, Statistics.mean(y), 0.0

        slope = (n * sum_xy - sum_x * sum_y) / denom
        intercept = (sum_y - slope * sum_x) / n

        # R²
        ss_res = sum((y[i] - (slope * x[i] + intercept)) ** 2 for i in range(n))
        y_mean = Statistics.mean(y)
        ss_tot = sum((y[i] - y_mean) ** 2 for i in range(n))
        r_squared = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

        return slope, intercept, r_squared

    @staticmethod
    def detect_outliers_iqr(data: List[float], factor: float = 1.5) -> Tuple[List[float], List[float]]:
        """
        IQR法检测异常值
        返回: (正常数据, 异常值)
        """
        sorted_d = sorted(data)
        n = len(sorted_d)
        q1 = sorted_d[n // 4]
        q3 = sorted_d[3 * n // 4]
        iqr = q3 - q1
        lower = q1 - factor * iqr
        upper = q3 + factor * iqr

        normal = [x for x in data if lower <= x <= upper]
        outliers = [x for x in data if x < lower or x > upper]
        return normal, outliers

    @staticmethod
    def snr_db(signal_power: float, noise_power: float) -> float:
        """信噪比 dB"""
        if noise_power <= 0 or signal_power <= 0:
            return float('inf')
        return 10.0 * math.log10(signal_power / noise_power)

    @staticmethod
    def enob(snr: float) -> float:
        """有效位数 ENOB = (SNR - 1.76) / 6.02"""
        return (snr - 1.76) / 6.02


# ── 传感器验证器 ──────────────────────────────────────────────────────────────

class SensorValidator:
    """传感器验证主类"""

    def __init__(self, config: ValidationConfig = None):
        self.config = config or ValidationConfig()
        self.stats = Statistics()

    def validate_accuracy(self, reference: float, readings: List[float],
                          spec_pct: float = None) -> AccuracyResult:
        """
        精度验证
        """
        result = AccuracyResult(reference_value=reference)
        if not readings:
            return result

        result.mean_reading = self.stats.mean(readings)
        result.std_deviation = self.stats.std_dev(readings)
        errors = [r - reference for r in readings]
        result.absolute_error = self.stats.mean(errors)
        result.max_error = max(abs(e) for e in errors)
        result.rmse = self.stats.rmse(errors)

        if abs(reference) > 1e-12:
            result.relative_error_pct = abs(result.absolute_error / reference) * 100.0

        spec = spec_pct or self.config.nominal_accuracy
        fs_range = self.config.nominal_range_max - self.config.nominal_range_min
        if spec_pct and fs_range > 0:
            spec_abs = spec / 100.0 * fs_range
        else:
            spec_abs = spec
        result.within_spec = result.max_error <= spec_abs

        return result

    def validate_repeatability(self, readings: List[float],
                               n_measurements: int = 1) -> RepeatabilityResult:
        """
        重复性验证
        """
        result = RepeatabilityResult(readings=readings[:])
        normal, outliers = self.stats.detect_outliers_iqr(readings)
        result.outliers = outliers

        if not normal:
            normal = readings

        result.n_readings = len(normal)
        result.mean = self.stats.mean(normal)
        result.std_deviation = self.stats.std_dev(normal)
        result.range_val = max(normal) - min(normal)

        if abs(result.mean) > 1e-12:
            result.cv_pct = result.std_deviation / abs(result.mean) * 100.0

        return result

    def validate_linearity(self, reference: List[float],
                           measured: List[float]) -> LinearityResult:
        """
        线性度验证 (最佳拟合直线法)
        """
        result = LinearityResult()

        slope, intercept, r2 = self.stats.linear_regression(reference, measured)
        result.slope = slope
        result.intercept = intercept
        result.r_squared = r2

        # 计算各点偏差
        deviations = []
        for ref, meas in zip(reference, measured):
            expected = slope * ref + intercept
            dev = meas - expected
            deviations.append(abs(dev))
            result.fit_points.append({
                'reference': ref, 'measured': meas,
                'expected': round(expected, 6), 'deviation': round(dev, 6)
            })

        result.max_deviation = max(deviations) if deviations else 0.0

        # %FS
        fs = max(reference) - min(reference) if reference else 1.0
        result.linearity_pct_fs = result.max_deviation / fs * 100.0 if fs > 0 else 0.0

        return result

    def validate_temperature(self, temp_readings: Dict[float, List[float]],
                             reference_value: float = None) -> TemperatureResult:
        """
        温度特性验证
        temp_readings: {温度: [读数列表]}
        """
        result = TemperatureResult()

        if len(temp_readings) < 2:
            return result

        temps = sorted(temp_readings.keys())
        means = [self.stats.mean(temp_readings[t]) for t in temps]

        # 零漂分析 (以第一个温度为基准)
        base_temp = temps[0]
        base_mean = means[0]

        result.temp_coefficients = {}
        for t, m in zip(temps, means):
            result.temp_coefficients[t] = {
                'mean': round(m, 6),
                'offset_from_base': round(m - base_mean, 6)
            }

        # 温度灵敏度系数 TCS (%/℃)
        if abs(base_mean) > 1e-12:
            delta_t = temps[-1] - temps[0]
            if delta_t > 0:
                delta_output = means[-1] - means[0]
                result.tcs = (delta_output / base_mean) / delta_t * 100.0
                result.offset_drift = delta_output / delta_t

        # 判断是否可补偿 (线性相关时)
        slope, _, r2 = self.stats.linear_regression(temps, means)
        result.sensitivity_drift = slope
        result.compensable = r2 > 0.9  # R²>0.9 认为可线性补偿

        return result

    def validate_noise(self, readings: List[float], sampling_rate: float = 1.0,
                       signal_value: float = None) -> NoiseResult:
        """
        噪声分析
        """
        result = NoiseResult()

        if not readings:
            return result

        mean_val = self.stats.mean(readings)
        noise = [r - mean_val for r in readings]

        result.peak_to_peak = max(readings) - min(readings)
        result.rms_noise = self.stats.std_dev(readings)

        signal = signal_value if signal_value is not None else abs(mean_val)
        if result.rms_noise > 0 and signal > 0:
            noise_power = result.rms_noise ** 2
            signal_power = signal ** 2
            result.snr_db = self.stats.snr_db(signal_power, noise_power)
            result.enob = self.stats.enob(result.snr_db)

        # 噪噪密度 (单位/√Hz)
        if sampling_rate > 0:
            result.noise_density = result.rms_noise / math.sqrt(sampling_rate / 2.0)
            result.signal_bandwidth = sampling_rate / 2.0

        return result


# ── 可视化 ────────────────────────────────────────────────────────────────────

def print_accuracy_report(result: AccuracyResult, config: ValidationConfig):
    """打印精度报告"""
    print("=" * 55)
    print("  精度验证报告")
    print("=" * 55)
    print(f"  参考值:       {result.reference_value}")
    print(f"  平均读数:     {result.mean_reading:.6f}")
    print(f"  标准差:       {result.std_deviation:.6f}")
    print(f"  绝对误差:     {result.absolute_error:+.6f}")
    print(f"  相对误差:     {result.relative_error_pct:.3f}%")
    print(f"  最大误差:     {result.max_error:.6f}")
    print(f"  RMSE:         {result.rmse:.6f}")
    status = "✓ 合格" if result.within_spec else "✗ 超差"
    print(f"  是否达标:     {status}")


def print_repeatability_report(result: RepeatabilityResult):
    """打印重复性报告"""
    print("=" * 55)
    print("  重复性验证报告")
    print("=" * 55)
    print(f"  测量次数:     {result.n_readings}")
    print(f"  平均值:       {result.mean:.6f}")
    print(f"  标准差:       {result.std_deviation:.6f}")
    print(f"  变异系数(CV): {result.cv_pct:.3f}%")
    print(f"  极差:         {result.range_val:.6f}")
    if result.outliers:
        print(f"  异常值:       {result.outliers}")
    quality = "优秀" if result.cv_pct < 0.5 else "良好" if result.cv_pct < 2 else "一般" if result.cv_pct < 5 else "较差"
    print(f"  重复性评级:   {quality}")


def print_linearity_report(result: LinearityResult):
    """打印线性度报告"""
    print("=" * 55)
    print("  线性度验证报告")
    print("=" * 55)
    print(f"  斜率:         {result.slope:.6f}")
    print(f"  截距:         {result.intercept:.6f}")
    print(f"  R²:           {result.r_squared:.6f}")
    print(f"  最大偏差:     {result.max_deviation:.6f}")
    print(f"  线性度(%FS):  {result.linearity_pct_fs:.3f}%")
    quality = "优秀" if result.r_squared > 0.999 else "良好" if result.r_squared > 0.99 else "一般" if result.r_squared > 0.95 else "较差"
    print(f"  线性度评级:   {quality}")

    # 显示各点
    print("\n  测量点分析:")
    print(f"  {'参考值':>10} {'测量值':>10} {'拟合值':>10} {'偏差':>10}")
    for p in result.fit_points:
        print(f"  {p['reference']:>10.4f} {p['measured']:>10.4f} {p['expected']:>10.4f} {p['deviation']:>+10.6f}")


def print_temperature_report(result: TemperatureResult, config: ValidationConfig):
    """打印温度特性报告"""
    print("=" * 55)
    print("  温度特性验证报告")
    print("=" * 55)
    print(f"  温度灵敏度系数(TCS): {result.tcs:.4f} %/℃")
    print(f"  零漂速率:            {result.offset_drift:.6f} 单位/℃")
    print(f"  灵敏度漂移:          {result.sensitivity_drift:.6f} 单位/℃")
    print(f"  可线性补偿:          {'是' if result.compensable else '否'}")

    if result.temp_coefficients:
        print("\n  温度-读数对照:")
        for t, info in sorted(result.temp_coefficients.items()):
            print(f"    {t:>6.1f}℃: 平均={info['mean']:.6f}  偏移={info['offset_from_base']:+.6f}")


def print_noise_report(result: NoiseResult):
    """打印噪声分析报告"""
    print("=" * 55)
    print("  噪声分析报告")
    print("=" * 55)
    print(f"  峰峰值:       {result.peak_to_peak:.6f}")
    print(f"  RMS噪声:      {result.rms_noise:.6f}")
    print(f"  信噪比(SNR):  {result.snr_db:.2f} dB")
    print(f"  噪声密度:     {result.noise_density:.6f} 单位/√Hz")
    print(f"  有效位(ENOB): {result.enob:.2f} bit")


# ── CLI 命令 ──────────────────────────────────────────────────────────────────

def cmd_accuracy(args):
    """精度验证"""
    config = ValidationConfig(
        nominal_accuracy=args.spec if args.spec else 1.0,
        nominal_range_max=args.range_max if args.range_max else args.ref * 1.1
    )
    validator = SensorValidator(config)

    if args.csv:
        readings = []
        with open(args.csv, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            for row in reader:
                try:
                    readings.append(float(row[0]))
                except (ValueError, IndexError):
                    continue
    else:
        readings = [float(x) for x in args.readings]

    result = validator.validate_accuracy(args.ref, readings)
    print_accuracy_report(result, config)
    return result


def cmd_repeatability(args):
    """重复性验证"""
    validator = SensorValidator()

    if args.csv:
        readings = []
        with open(args.csv, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            for row in reader:
                try:
                    readings.append(float(row[0]))
                except (ValueError, IndexError):
                    continue
    else:
        readings = [float(x) for x in args.readings]

    result = validator.validate_repeatability(readings)
    print_repeatability_report(result)
    return result


def cmd_linearity(args):
    """线性度验证"""
    validator = SensorValidator()

    if args.csv:
        ref_list, meas_list = [], []
        with open(args.csv, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                ref_list.append(float(row.get('reference', row.get('ref', row.get('参考', 0)))))
                meas_list.append(float(row.get('measured', row.get('meas', row.get('测量', 0)))))
    else:
        ref_list = [float(x) for x in args.ref_values]
        meas_list = [float(x) for x in args.meas_values]

    result = validator.validate_linearity(ref_list, meas_list)
    print_linearity_report(result)
    return result


def cmd_temperature(args):
    """温度特性验证"""
    validator = SensorValidator()
    temp_data = {}

    if args.csv:
        with open(args.csv, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                temp = float(row.get('temperature', row.get('temp', row.get('温度', 0))))
                val = float(row.get('value', row.get('reading', row.get('读数', 0))))
                if temp not in temp_data:
                    temp_data[temp] = []
                temp_data[temp].append(val)
    else:
        # JSON格式输入
        temp_data = json.loads(args.data)

    result = validator.validate_temperature(temp_data)
    config = ValidationConfig()
    print_temperature_report(result, config)
    return result


def cmd_noise(args):
    """噪声分析"""
    validator = SensorValidator()

    if args.csv:
        readings = []
        with open(args.csv, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            for row in reader:
                try:
                    readings.append(float(row[0]))
                except (ValueError, IndexError):
                    continue
    else:
        readings = [float(x) for x in args.readings]

    result = validator.validate_noise(readings, args.rate, args.signal)
    print_noise_report(result)
    return result


def cmd_report(args):
    """综合验证报告"""
    config = ValidationConfig()

    if args.config:
        with open(args.config, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
        config.sensor_name = cfg.get('name', config.sensor_name)
        config.sensor_type = cfg.get('type', config.sensor_type)
        config.nominal_accuracy = cfg.get('accuracy', config.nominal_accuracy)
        config.nominal_range_max = cfg.get('range_max', config.nominal_range_max)

    validator = SensorValidator(config)

    print("=" * 60)
    print(f"  传感器综合验证报告: {config.sensor_name}")
    print("=" * 60)
    print(f"  传感器类型:  {config.sensor_type}")
    print(f"  标称精度:    ±{config.nominal_accuracy}")
    print(f"  量程:        {config.nominal_range_min} ~ {config.nominal_range_max}")
    print(f"  报告时间:    {time.strftime('%Y-%m-%d %H:%M:%S')}")

    results = {}

    # 精度
    if args.accuracy_csv:
        readings = []
        with open(args.accuracy_csv, 'r', encoding='utf-8') as f:
            for row in csv.reader(f):
                try:
                    readings.append(float(row[0]))
                except:
                    continue
        if readings:
            acc = validator.validate_accuracy(args.ref_val or 0, readings)
            print_accuracy_report(acc, config)
            results['accuracy'] = asdict(acc)

    # 重复性
    if args.repeat_csv:
        readings = []
        with open(args.repeat_csv, 'r', encoding='utf-8') as f:
            for row in csv.reader(f):
                try:
                    readings.append(float(row[0]))
                except:
                    continue
        if readings:
            rep = validator.validate_repeatability(readings)
            print_repeatability_report(rep)
            results['repeatability'] = asdict(rep)

    # 保存结果
    out_file = f"sensor_{config.sensor_name.replace(' ', '_')}_report.json"
    with open(out_file, 'w', encoding='utf-8') as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    print(f"\n  报告已保存: {out_file}")


def main():
    parser = argparse.ArgumentParser(
        description='传感器验证工具 - 电赛资产库',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s accuracy --ref 100.0 --readings 99.8 100.1 100.3 99.9 --spec 0.5
  %(prog)s repeatability --readings 1.02 1.01 1.03 1.00 1.02 1.01
  %(prog)s linearity --csv cal_data.csv
  %(prog)s temperature --csv temp_drift.csv
  %(prog)s noise --csv noise_data.csv --rate 1000
  %(prog)s report --config sensor_config.json --accuracy-csv acc.csv --repeat-csv rep.csv
        """
    )
    sub = parser.add_subparsers(dest='command')

    # 精度
    p_acc = sub.add_parser('accuracy', help='精度验证')
    p_acc.add_argument('--ref', type=float, required=True, help='参考值')
    p_acc.add_argument('--readings', type=float, nargs='+', help='测量读数列表')
    p_acc.add_argument('--csv', type=str, help='读数CSV文件')
    p_acc.add_argument('--spec', type=float, help='精度规格 (绝对值)')
    p_acc.add_argument('--range-max', type=float, help='满量程')

    # 重复性
    p_rep = sub.add_parser('repeatability', help='重复性验证')
    p_rep.add_argument('--readings', type=float, nargs='+', help='测量读数列表')
    p_rep.add_argument('--csv', type=str, help='读数CSV文件')

    # 线性度
    p_lin = sub.add_parser('linearity', help='线性度验证')
    p_lin.add_argument('--csv', type=str, help='校准数据CSV (列: reference, measured)')
    p_lin.add_argument('--ref-values', type=float, nargs='+', help='参考值列表')
    p_lin.add_argument('--meas-values', type=float, nargs='+', help='测量值列表')

    # 温度
    p_temp = sub.add_parser('temperature', help='温度特性验证')
    p_temp.add_argument('--csv', type=str, help='温度数据CSV')
    p_temp.add_argument('--data', type=str, help='JSON格式温度数据')

    # 噪声
    p_noise = sub.add_parser('noise', help='噪声分析')
    p_noise.add_argument('--csv', type=str, help='噪声数据CSV')
    p_noise.add_argument('--readings', type=float, nargs='+', help='采样数据')
    p_noise.add_argument('--rate', type=float, default=1.0, help='采样率 (Hz)')
    p_noise.add_argument('--signal', type=float, help='信号幅值')

    # 综合报告
    p_rpt = sub.add_parser('report', help='综合验证报告')
    p_rpt.add_argument('--config', type=str, help='验证配置JSON')
    p_rpt.add_argument('--accuracy-csv', type=str, help='精度数据CSV')
    p_rpt.add_argument('--repeat-csv', type=str, help='重复性数据CSV')
    p_rpt.add_argument('--ref-val', type=float, help='精度测试参考值')

    args = parser.parse_args()
    if not args.command:
        parser.print_help()
        return

    dispatch = {
        'accuracy': cmd_accuracy, 'repeatability': cmd_repeatability,
        'linearity': cmd_linearity, 'temperature': cmd_temperature,
        'noise': cmd_noise, 'report': cmd_report
    }
    dispatch[args.command](args)


if __name__ == '__main__':
    main()
