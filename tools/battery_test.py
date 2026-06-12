#!/usr/bin/env python3
"""
电池测试工具 - Battery Test Tool
==================================
功能:
  - 放电曲线分析
  - 容量测试 (mAh / Wh)
  - 内阻测量 (DC法 / AC法)
  - 循环寿命评估
  - SoC估算
  - 电池等效电路建模 (Rint / Thevenin / RC)
用法:
  python battery_test.py capacity --csv discharge.csv --nominal 2200
  python battery_test.py resistance --v-oc 4.2 --v-load 3.9 --current 2.0
  python battery_test.py soc --voltage 3.7 --chemistry li-ion
  python battery_test.py cycle --csv cycle_data.csv
  python battery_test.py discharge --csv data.csv --plot
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
class DischargePoint:
    """放电曲线数据点"""
    time_s: float = 0.0
    voltage: float = 0.0
    current: float = 0.0
    capacity_mah: float = 0.0
    energy_mwh: float = 0.0
    temperature: float = 25.0
    soc: float = 100.0


@dataclass
class BatterySpec:
    """电池规格"""
    chemistry: str = "Li-ion"     # 化学体系
    nominal_voltage: float = 3.7  # 标称电压
    charge_voltage: float = 4.2   # 充电截止电压
    cutoff_voltage: float = 3.0   # 放电截止电压
    nominal_capacity: float = 2200  # 标称容量 mAh
    max_current: float = 5.0      # 最大放电电流 A
    max_charge_current: float = 2.0


@dataclass
class CycleResult:
    """循环测试结果"""
    cycle_number: int = 0
    charge_capacity: float = 0.0   # mAh
    discharge_capacity: float = 0.0  # mAh
    coulombic_efficiency: float = 0.0  # %
    energy_efficiency: float = 0.0    # %
    internal_resistance: float = 0.0  # Ω
    end_voltage: float = 0.0


@dataclass
class BatteryTestResult:
    """完整电池测试结果"""
    spec: BatterySpec = field(default_factory=BatterySpec)
    measured_capacity: float = 0.0
    internal_resistance: float = 0.0
    coulomb_efficiency: float = 0.0
    cycle_results: List[CycleResult] = field(default_factory=list)
    discharge_curve: List[DischargePoint] = field(default_factory=list)
    test_time: str = ""
    notes: str = ""


# ── 电池化学体系数据库 ──────────────────────────────────────────────────────────

CHEMISTRY_DB = {
    'li-ion': {
        'name': '锂离子 (LiCoO2)',
        'nominal_v': 3.7, 'charge_v': 4.2, 'cutoff_v': 3.0,
        'soc_curve': {4.20: 100, 4.03: 90, 3.86: 80, 3.83: 70, 3.79: 60,
                      3.75: 50, 3.71: 40, 3.67: 30, 3.63: 20, 3.55: 10, 3.00: 0}
    },
    'lipo': {
        'name': '锂聚合物 (LiPo)',
        'nominal_v': 3.7, 'charge_v': 4.2, 'cutoff_v': 3.0,
        'soc_curve': {4.20: 100, 4.15: 95, 4.05: 85, 3.95: 75, 3.85: 65,
                      3.75: 55, 3.70: 50, 3.65: 40, 3.55: 25, 3.40: 10, 3.00: 0}
    },
    'lifepo4': {
        'name': '磷酸铁锂 (LiFePO4)',
        'nominal_v': 3.2, 'charge_v': 3.65, 'cutoff_v': 2.5,
        'soc_curve': {3.65: 100, 3.40: 95, 3.35: 80, 3.33: 70, 3.32: 60,
                      3.30: 50, 3.28: 40, 3.25: 30, 3.20: 20, 3.10: 10, 2.50: 0}
    },
    'nimh': {
        'name': '镍氢 (NiMH)',
        'nominal_v': 1.2, 'charge_v': 1.5, 'cutoff_v': 1.0,
        'soc_curve': {1.42: 100, 1.38: 80, 1.33: 60, 1.30: 50, 1.26: 40,
                      1.22: 25, 1.15: 10, 1.00: 0}
    },
    'lead-acid': {
        'name': '铅酸',
        'nominal_v': 2.0, 'charge_v': 2.45, 'cutoff_v': 1.75,
        'soc_curve': {2.45: 100, 2.20: 80, 2.10: 60, 2.05: 50, 2.00: 40,
                      1.95: 20, 1.75: 0}
    }
}


# ── 核心计算 ──────────────────────────────────────────────────────────────────

class BatteryCalculator:
    """电池参数计算"""

    @staticmethod
    def calc_capacity(points: List[DischargePoint]) -> Tuple[float, float]:
        """
        梯形积分法计算放电容量和能量
        返回: (容量mAh, 能量mWh)
        """
        if len(points) < 2:
            return 0.0, 0.0

        total_mah = 0.0
        total_mwh = 0.0
        for i in range(1, len(points)):
            dt_h = (points[i].time_s - points[i-1].time_s) / 3600.0
            avg_i = (points[i].current + points[i-1].current) / 2.0
            avg_v = (points[i].voltage + points[i-1].voltage) / 2.0
            total_mah += avg_i * dt_h * 1000.0
            total_mwh += avg_i * avg_v * dt_h * 1000.0

        return round(total_mah, 2), round(total_mwh, 2)

    @staticmethod
    def calc_internal_resistance(v_oc: float, v_load: float, current: float) -> float:
        """
        DC法测量内阻: Rint = (Voc - Vload) / I
        """
        if current <= 0:
            raise ValueError("电流必须大于0")
        return abs(v_oc - v_load) / current

    @staticmethod
    def calc_internal_resistance_pulse(
        v_before: float, v_during: float, v_after: float,
        current: float
    ) -> dict:
        """
        脉冲法测量内阻（区分欧姆内阻和极化内阻）
        R_ohmic = (V_before - V_during) / I   (瞬时响应)
        R_polarization = (V_during - V_after) / I  (弛豫过程，负值表示恢复)
        """
        if current <= 0:
            raise ValueError("电流必须大于0")
        r_ohmic = abs(v_before - v_during) / current
        r_total = abs(v_before - v_after) / current
        r_polarization = r_total - r_ohmic
        return {
            'r_ohmic': round(r_ohmic, 6),
            'r_polarization': round(max(r_polarization, 0), 6),
            'r_total': round(r_total, 6)
        }

    @staticmethod
    def estimate_soc(voltage: float, chemistry: str = 'li-ion') -> float:
        """
        基于OCV-SOC查找表估算荷电状态
        使用线性插值
        """
        if chemistry not in CHEMISTRY_DB:
            raise ValueError(f"未知化学体系: {chemistry}, 可选: {list(CHEMISTRY_DB.keys())}")

        soc_curve = CHEMISTRY_DB[chemistry]['soc_curve']
        voltages = sorted(soc_curve.keys(), reverse=True)

        # 查找插值区间
        if voltage >= voltages[0]:
            return 100.0
        if voltage <= voltages[-1]:
            return 0.0

        for i in range(len(voltages) - 1):
            v_high = voltages[i]
            v_low = voltages[i+1]
            if v_high >= voltage >= v_low:
                soc_high = soc_curve[v_high]
                soc_low = soc_curve[v_low]
                # 线性插值
                ratio = (voltage - v_low) / (v_high - v_low)
                return round(soc_low + ratio * (soc_high - soc_low), 1)

        return 0.0

    @staticmethod
    def calc_coulomb_efficiency(charge_mah: float, discharge_mah: float) -> float:
        """库仑效率 = 放电容量 / 充电容量 × 100%"""
        if charge_mah <= 0:
            return 0.0
        return round(discharge_mah / charge_mah * 100.0, 2)

    @staticmethod
    def calc_energy_density(capacity_mah: float, nominal_v: float, weight_g: float = 0,
                            volume_ml: float = 0) -> dict:
        """
        计算能量密度
        """
        energy_wh = capacity_mah * nominal_v / 1000.0
        result = {'energy_wh': round(energy_wh, 3)}
        if weight_g > 0:
            result['gravimetric_wh_kg'] = round(energy_wh / weight_g * 1000, 1)
        if volume_ml > 0:
            result['volumetric_wh_l'] = round(energy_wh / volume_ml * 1000, 1)
        return result

    @staticmethod
    def calc_peukert(capacity_20h: float, rated_current: float, actual_current: float,
                     k: float = 1.1) -> float:
        """
        佩克特定律修正: C = C_20h × (I_20h / I)^k
        典型k值: 铅酸1.1-1.3, 锂电接近1.0
        """
        if actual_current <= 0:
            return 0.0
        i_20h = capacity_20h / 20.0
        adjusted = capacity_20h * (i_20h / actual_current) ** (k - 1)
        return round(adjusted, 2)

    @staticmethod
    def calc_cycle_degradation(cycles: List[CycleResult]) -> dict:
        """
        循环寿命分析: 容量衰减率、预测寿命
        """
        if len(cycles) < 2:
            return {'error': '数据点不足'}

        caps = [c.discharge_capacity for c in cycles]
        nums = [c.cycle_number for c in cycles]

        # 线性回归: capacity = a * cycle + b
        n = len(caps)
        sum_x = sum(nums)
        sum_y = sum(caps)
        sum_xy = sum(x * y for x, y in zip(nums, caps))
        sum_x2 = sum(x * x for x in nums)

        denom = n * sum_x2 - sum_x ** 2
        if abs(denom) < 1e-12:
            slope = 0
            intercept = caps[0]
        else:
            slope = (n * sum_xy - sum_x * sum_y) / denom
            intercept = (sum_y - slope * sum_x) / n

        # 预测80%容量寿命
        initial_cap = caps[0] if caps else 1
        end_cap = initial_cap * 0.8
        if slope < 0:
            predicted_life = int((end_cap - intercept) / slope)
        else:
            predicted_life = -1  # 不衰减

        # 每周期衰减率
        decay_per_cycle = -slope / initial_cap * 100 if initial_cap > 0 else 0

        return {
            'initial_capacity': round(initial_cap, 2),
            'final_capacity': round(caps[-1], 2),
            'total_cycles': len(cycles),
            'capacity_retention_pct': round(caps[-1] / initial_cap * 100, 2) if initial_cap > 0 else 0,
            'decay_per_cycle_pct': round(decay_per_cycle, 4),
            'predicted_80pct_life': predicted_life,
            'linear_slope': round(slope, 4)
        }


# ── 数据加载 ──────────────────────────────────────────────────────────────────

def load_discharge_csv(filepath: str) -> List[DischargePoint]:
    """从CSV加载放电数据"""
    points = []
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # 尝试多种列名
            t = float(row.get('time_s', row.get('time', row.get('时间', 0))))
            v = float(row.get('voltage', row.get('V', row.get('电压', 0))))
            i = float(row.get('current', row.get('I', row.get('电流', 0))))
            temp = float(row.get('temperature', row.get('temp', row.get('温度', 25))))
            points.append(DischargePoint(time_s=t, voltage=v, current=i, temperature=temp))
    return points


def load_cycle_csv(filepath: str) -> List[CycleResult]:
    """从CSV加载循环测试数据"""
    results = []
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            results.append(CycleResult(
                cycle_number=int(row.get('cycle', row.get('cycle_number', row.get('循环', 0)))),
                charge_capacity=float(row.get('charge_mah', row.get('充电容量', 0))),
                discharge_capacity=float(row.get('discharge_mah', row.get('放电容量', 0))),
                internal_resistance=float(row.get('resistance', row.get('内阻', 0)))
            ))
    return sorted(results, key=lambda x: x.cycle_number)


# ── 可视化 ────────────────────────────────────────────────────────────────────

def plot_discharge_curve(points: List[DischargePoint], output: str = "discharge_curve.png"):
    """绘制放电曲线"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("  [提示] 需要 matplotlib: pip install matplotlib")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    times = [p.time_s / 60 for p in points]  # 转分钟
    voltages = [p.voltage for p in points]
    currents = [p.current for p in points]
    socs = [p.soc for p in points]

    ax1.plot(times, voltages, 'b-', linewidth=2, label='电压')
    ax1.set_ylabel('电压 (V)')
    ax1.set_title('电池放电曲线')
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    ax2.plot(times, currents, 'r-', linewidth=2, label='电流')
    ax2_twin = ax2.twinx()
    ax2_twin.plot(times, socs, 'g--', linewidth=1.5, label='SoC')
    ax2.set_xlabel('时间 (分钟)')
    ax2.set_ylabel('电流 (A)')
    ax2_twin.set_ylabel('SoC (%)')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper left')
    ax2_twin.legend(loc='upper right')

    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)
    print(f"  放电曲线已保存: {output}")


def plot_cycle_degradation(cycles: List[CycleResult], output: str = "cycle_life.png"):
    """绘制循环寿命曲线"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("  [提示] 需要 matplotlib: pip install matplotlib")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    nums = [c.cycle_number for c in cycles]
    caps = [c.discharge_capacity for c in cycles]
    resistances = [c.internal_resistance for c in cycles]
    coulomb = [c.coulombic_efficiency for c in cycles]

    ax1.plot(nums, caps, 'bo-', markersize=4, label='放电容量')
    ax1.set_ylabel('容量 (mAh)')
    ax1.set_title('循环寿命测试')
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    if any(r > 0 for r in resistances):
        ax1_twin = ax1.twinx()
        ax1_twin.plot(nums, resistances, 'rs-', markersize=4, label='内阻')
        ax1_twin.set_ylabel('内阻 (Ω)')
        ax1_twin.legend(loc='upper right')

    ax2.plot(nums, coulomb, 'g^-', markersize=4, label='库仑效率')
    ax2.set_xlabel('循环次数')
    ax2.set_ylabel('效率 (%)')
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)
    print(f"  循环寿命曲线已保存: {output}")


# ── CLI 命令 ──────────────────────────────────────────────────────────────────

def cmd_capacity(args):
    """容量测试"""
    calc = BatteryCalculator()

    if args.csv:
        points = load_discharge_csv(args.csv)
        print(f"  加载 {len(points)} 个数据点")
        cap_mah, cap_mwh = calc.calc_capacity(points)

        # 计算各段容量
        print("\n" + "=" * 55)
        print("  放电容量测试结果")
        print("=" * 55)
        print(f"  数据点数:    {len(points)}")
        print(f"  放电时长:    {points[-1].time_s:.0f} 秒 ({points[-1].time_s/60:.1f} 分钟)")
        print(f"  起始电压:    {points[0].voltage:.3f} V")
        print(f"  终止电压:    {points[-1].voltage:.3f} V")
        print(f"  ► 放电容量:  {cap_mah:.2f} mAh")
        print(f"  ► 放电能量:  {cap_mwh:.2f} mWh")

        if args.nominal:
            pct = cap_mah / args.nominal * 100
            print(f"  ► 标称容量:  {args.nominal} mAh")
            print(f"  ► 达标率:    {pct:.1f}%")

        if args.weight:
            density = calc.calc_energy_density(cap_mah, points[0].voltage, weight_g=args.weight)
            print(f"  ► 能量密度:  {density.get('gravimetric_wh_kg', 'N/A')} Wh/kg")

        # CSV导出
        out_csv = args.csv.replace('.csv', '_capacity_result.csv')
        with open(out_csv, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['指标', '值', '单位'])
            writer.writerow(['放电容量', cap_mah, 'mAh'])
            writer.writerow(['放电能量', cap_mwh, 'mWh'])
            writer.writerow(['放电时长', points[-1].time_s, 's'])
        print(f"\n  结果已保存: {out_csv}")

        if args.plot:
            plot_discharge_curve(points)
    else:
        print("  [错误] 请提供放电数据CSV文件: --csv <file>")


def cmd_resistance(args):
    """内阻测量"""
    calc = BatteryCalculator()

    print("=" * 55)
    print("  电池内阻测量")
    print("=" * 55)

    if args.pulse:
        # 脉冲法
        result = calc.calc_internal_resistance_pulse(
            args.v_oc, args.v_load, args.v_after, args.current
        )
        print(f"  测量方法:    脉冲法")
        print(f"  脉冲前电压:  {args.v_oc:.4f} V")
        print(f"  脉冲中电压:  {args.v_load:.4f} V")
        print(f"  脉冲后电压:  {args.v_after:.4f} V")
        print(f"  脉冲电流:    {args.current:.3f} A")
        print(f"  ► 欧姆内阻:  {result['r_ohmic']*1000:.2f} mΩ")
        print(f"  ► 极化内阻:  {result['r_polarization']*1000:.2f} mΩ")
        print(f"  ► 总内阻:    {result['r_total']*1000:.2f} mΩ")
    else:
        # DC法
        rint = calc.calc_internal_resistance(args.v_oc, args.v_load, args.current)
        print(f"  测量方法:    DC法 (欧姆定律)")
        print(f"  开路电压:    {args.v_oc:.4f} V")
        print(f"  负载电压:    {args.v_load:.4f} V")
        print(f"  放电电流:    {args.current:.3f} A")
        print(f"  ► 内阻:      {rint:.6f} Ω ({rint*1000:.2f} mΩ)")

    if args.chemistry:
        soc = calc.estimate_soc(args.v_oc, args.chemistry)
        print(f"  ► 当前SoC:   {soc:.1f}%")


def cmd_soc(args):
    """SoC估算"""
    calc = BatteryCalculator()

    soc = calc.estimate_soc(args.voltage, args.chemistry)

    info = CHEMISTRY_DB[args.chemistry]
    print("=" * 55)
    print("  电池SoC估算")
    print("=" * 55)
    print(f"  化学体系:    {info['name']}")
    print(f"  测量电压:    {args.voltage:.3f} V")
    print(f"  ► SoC估算:   {soc:.1f}%")

    # 显示查找表
    print("\n  OCV-SOC查找表:")
    curve = info['soc_curve']
    for v in sorted(curve.keys(), reverse=True):
        marker = " ◄" if abs(v - args.voltage) < 0.05 else ""
        print(f"    {v:.2f}V → {curve[v]:>3d}%{marker}")


def cmd_cycle(args):
    """循环寿命分析"""
    calc = BatteryCalculator()

    if not args.csv:
        print("  [错误] 请提供循环测试数据CSV: --csv <file>")
        return

    cycles = load_cycle_csv(args.csv)
    print(f"  加载 {len(cycles)} 个循环数据")

    # 填充库仑效率
    for c in cycles:
        if c.charge_capacity > 0:
            c.coulombic_efficiency = calc.calc_coulomb_efficiency(
                c.charge_capacity, c.discharge_capacity)

    # 衰减分析
    analysis = calc.calc_cycle_degradation(cycles)

    print("\n" + "=" * 55)
    print("  循环寿命分析")
    print("=" * 55)
    for k, v in analysis.items():
        label = {
            'initial_capacity': '初始容量 (mAh)',
            'final_capacity': '终止容量 (mAh)',
            'total_cycles': '测试循环数',
            'capacity_retention_pct': '容量保持率 (%)',
            'decay_per_cycle_pct': '每循环衰减率 (%)',
            'predicted_80pct_life': '预测80%容量寿命 (次)',
            'linear_slope': '线性斜率'
        }.get(k, k)
        print(f"  {label}: {v}")

    if args.plot:
        plot_cycle_degradation(cycles)

    # 导出
    out_json = args.csv.replace('.csv', '_cycle_analysis.json')
    with open(out_json, 'w', encoding='utf-8') as f:
        json.dump(analysis, f, ensure_ascii=False, indent=2)
    print(f"\n  分析结果已保存: {out_json}")


def cmd_peukert(args):
    """佩克特定律修正"""
    calc = BatteryCalculator()

    adjusted = calc.calc_peukert(args.capacity, args.rated_current, args.actual_current, args.k)

    print("=" * 55)
    print("  佩克特定律修正")
    print("=" * 55)
    print(f"  标称容量 (20h率): {args.capacity} mAh")
    print(f"  标称电流:          {args.rated_current} A")
    print(f"  实际放电电流:      {args.actual_current} A")
    print(f"  佩克特指数 k:      {args.k}")
    print(f"  ► 修正后容量:     {adjusted} mAh")
    print(f"  ► 容量损失:        {args.capacity - adjusted:.2f} mAh ({(1 - adjusted/args.capacity)*100:.1f}%)")


def main():
    parser = argparse.ArgumentParser(
        description='电池测试工具 - 电赛资产库',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s capacity --csv discharge.csv --nominal 2200 --plot
  %(prog)s resistance --v-oc 4.2 --v-load 3.9 --current 2.0
  %(prog)s resistance --pulse --v-oc 4.18 --v-load 3.92 --v-after 4.05 --current 3.0
  %(prog)s soc --voltage 3.7 --chemistry li-ion
  %(prog)s cycle --csv cycle_data.csv --plot
  %(prog)s peukert --capacity 2200 --rated-current 0.44 --actual-current 5.0
        """
    )
    sub = parser.add_subparsers(dest='command')

    # 容量测试
    p_cap = sub.add_parser('capacity', help='放电容量测试')
    p_cap.add_argument('--csv', type=str, required=True, help='放电数据CSV')
    p_cap.add_argument('--nominal', type=float, help='标称容量 (mAh)')
    p_cap.add_argument('--weight', type=float, help='电池重量 (g)')
    p_cap.add_argument('--plot', action='store_true', help='绘制放电曲线')

    # 内阻
    p_r = sub.add_parser('resistance', help='内阻测量')
    p_r.add_argument('--v-oc', type=float, required=True, help='开路电压 (V)')
    p_r.add_argument('--v-load', type=float, required=True, help='负载电压 (V)')
    p_r.add_argument('--current', type=float, required=True, help='放电电流 (A)')
    p_r.add_argument('--pulse', action='store_true', help='使用脉冲法')
    p_r.add_argument('--v-after', type=float, help='脉冲后电压 (脉冲法)')
    p_r.add_argument('--chemistry', type=str, help='电池化学体系')

    # SoC
    p_soc = sub.add_parser('soc', help='SoC估算')
    p_soc.add_argument('--voltage', type=float, required=True, help='测量电压 (V)')
    p_soc.add_argument('--chemistry', type=str, default='li-ion',
                       choices=list(CHEMISTRY_DB.keys()), help='电池化学体系')

    # 循环寿命
    p_cyc = sub.add_parser('cycle', help='循环寿命分析')
    p_cyc.add_argument('--csv', type=str, required=True, help='循环数据CSV')
    p_cyc.add_argument('--plot', action='store_true', help='绘制寿命曲线')

    # 佩克特
    p_peu = sub.add_parser('peukert', help='佩克特定律修正')
    p_peu.add_argument('--capacity', type=float, required=True, help='标称容量 (mAh)')
    p_peu.add_argument('--rated-current', type=float, required=True, help='标称电流 (A)')
    p_peu.add_argument('--actual-current', type=float, required=True, help='实际电流 (A)')
    p_peu.add_argument('--k', type=float, default=1.1, help='佩克特指数')

    args = parser.parse_args()
    if not args.command:
        parser.print_help()
        return

    {'capacity': cmd_capacity, 'resistance': cmd_resistance, 'soc': cmd_soc,
     'cycle': cmd_cycle, 'peukert': cmd_peukert}[args.command](args)


if __name__ == '__main__':
    main()
