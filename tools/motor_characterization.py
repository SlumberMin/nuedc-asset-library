#!/usr/bin/env python3
"""
电机特性测试工具 - Motor Characterization Tool
================================================
功能:
  - KV值测量（RPM/V）
  - Kt值测量（Nm/A）
  - 电枢电阻 Rm 测量
  - 电感测量
  - 效率MAP图生成
  - 特性曲线绘制
用法:
  python motor_characterization.py kv --voltage 12 --rpm 1000
  python motor_characterization.py kt --torque 0.5 --current 3.2
  python motor_characterization.py resistance --v-drop 0.64 --current 2.0
  python motor_characterization.py efficiency --csv motor_data.csv
  python motor_characterization.py map --csv motor_data.csv --output map.png
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
class MotorParams:
    """电机参数汇总"""
    name: str = "未命名电机"
    kv: float = 0.0          # KV值 RPM/V
    kt: float = 0.0          # 转矩常数 Nm/A
    km: float = 0.0          # 电机常数 Nm/sqrt(W)
    rm: float = 0.0          # 电枢电阻 Ω
    lm: float = 0.0          # 电枢电感 mH
    no_load_current: float = 0.0   # 空载电流 A
    no_load_rpm: float = 0.0       # 空载转速
    stall_torque: float = 0.0      # 堵转转矩 Nm
    stall_current: float = 0.0     # 堵转电流 A
    max_efficiency: float = 0.0    # 最大效率 %
    max_power: float = 0.0         # 最大功率 W


@dataclass
class EfficiencyPoint:
    """效率MAP上的一个工作点"""
    rpm: float = 0.0
    torque: float = 0.0      # Nm
    current: float = 0.0     # A
    voltage: float = 0.0     # V
    power_in: float = 0.0    # W
    power_out: float = 0.0   # W
    efficiency: float = 0.0  # %
    temp: float = 25.0       # ℃


@dataclass
class MotorTestData:
    """完整测试数据集"""
    params: MotorParams = field(default_factory=MotorParams)
    efficiency_map: List[EfficiencyPoint] = field(default_factory=list)
    test_timestamp: str = ""
    test_notes: str = ""


# ── 核心计算 ──────────────────────────────────────────────────────────────────

class MotorCalculator:
    """电机参数计算器"""

    @staticmethod
    def calc_kv(rpm: float, voltage: float) -> float:
        """
        计算KV值 (RPM/V)
        参数:
            rpm: 无负载转速
            voltage: 施加电压
        返回:
            KV值
        """
        if voltage <= 0:
            raise ValueError("电压必须大于0")
        return rpm / voltage

    @staticmethod
    def calc_kt(torque_nm: float, current_a: float) -> float:
        """
        计算Kt转矩常数 (Nm/A)
        注意: 理论上 Kt = 1 / (KV * 2π/60)
        参数:
            torque_nm: 测量转矩
            current_a: 测量电流
        返回:
            Kt值
        """
        if current_a <= 0:
            raise ValueError("电流必须大于0")
        return torque_nm / current_a

    @staticmethod
    def calc_kv_kt_relation(kv: float = None, kt: float = None) -> dict:
        """
        KV-Kt互算 (理想BLDC: Kt = 60 / (2π * KV))
        """
        result = {}
        if kv is not None and kv > 0:
            result['kt_from_kv'] = 60.0 / (2.0 * math.pi * kv)
            result['kv'] = kv
        if kt is not None and kt > 0:
            result['kv_from_kt'] = 60.0 / (2.0 * math.pi * kt)
            result['kt'] = kt
        return result

    @staticmethod
    def calc_resistance(voltage_drop: float, current: float) -> float:
        """
        通过电压降和电流计算电枢电阻 (欧姆定律)
        """
        if current <= 0:
            raise ValueError("电流必须大于0")
        return voltage_drop / current

    @staticmethod
    def calc_resistance_two_point(v1: float, i1: float, v2: float, i2: float) -> float:
        """
        两点法测电阻: R = ΔV / ΔI (消除反电动势影响)
        """
        dv = v1 - v2
        di = i1 - i2
        if abs(di) < 1e-9:
            raise ValueError("两次电流差太小，无法计算")
        return abs(dv / di)

    @staticmethod
    def calc_inductance(frequency: float, impedance: float, resistance: float) -> float:
        """
        根据阻抗三角形计算电感
        Z² = R² + (2πfL)² → L = sqrt(Z²-R²) / (2πf)
        参数:
            frequency: 测试频率 Hz
            impedance: 测量阻抗 Ω
            resistance: 已知电阻 Ω
        返回:
            电感值 mH
        """
        if impedance < resistance:
            raise ValueError("阻抗不能小于电阻")
        xl_sq = impedance ** 2 - resistance ** 2
        xl = math.sqrt(xl_sq)
        lm_h = xl / (2.0 * math.pi * frequency)
        return lm_h * 1000  # 转mH

    @staticmethod
    def calc_efficiency(power_out: float, power_in: float) -> float:
        """
        计算效率百分比
        """
        if power_in <= 0:
            return 0.0
        return (power_out / power_in) * 100.0

    @staticmethod
    def calc_motor_constant(torque_nm: float, power_loss: float) -> float:
        """
        计算电机常数 Km = T / sqrt(P_loss)  单位 Nm/√W
        """
        if power_loss <= 0:
            return float('inf')
        return torque_nm / math.sqrt(power_loss)

    @staticmethod
    def calc_mechanical_power(torque_nm: float, rpm: float) -> float:
        """机械功率 P = T × ω = T × 2π×n/60"""
        omega = 2.0 * math.pi * rpm / 60.0
        return torque_nm * omega

    @staticmethod
    def calc_electrical_power(voltage: float, current: float) -> float:
        """电功率 P = V × I"""
        return voltage * current

    @staticmethod
    def calc_back_emf_constant(kv: float) -> float:
        """反电动势常数 Ke = 1/(KV × 2π/60)  V/(rad/s)"""
        return 60.0 / (2.0 * math.pi * kv)

    @staticmethod
    def generate_efficiency_map(
        motor: MotorParams,
        voltage_range: List[float],
        rpm_range: List[float]
    ) -> List[EfficiencyPoint]:
        """
        生成效率MAP数据
        基于电机等效电路模型: V = Ke*ω + I*Rm
        """
        points = []
        kt = motor.kt if motor.kt > 0 else (60.0 / (2.0 * math.pi * motor.kv)) if motor.kv > 0 else 1.0
        ke = 60.0 / (2.0 * math.pi * motor.kv) if motor.kv > 0 else kt
        rm = motor.rm if motor.rm > 0 else 0.1

        for v in voltage_range:
            for rpm in rpm_range:
                omega = 2.0 * math.pi * rpm / 60.0
                back_emf = ke * omega
                if back_emf >= v:
                    continue  # 反电动势超过供电电压，不可能
                current = (v - back_emf) / rm
                if current < 0:
                    continue
                torque = kt * current
                p_in = v * current
                p_out = torque * omega
                eff = (p_out / p_in * 100.0) if p_in > 0 else 0.0
                eff = min(eff, 100.0)

                points.append(EfficiencyPoint(
                    rpm=rpm,
                    torque=round(torque, 4),
                    current=round(current, 3),
                    voltage=v,
                    power_in=round(p_in, 2),
                    power_out=round(p_out, 2),
                    efficiency=round(eff, 2)
                ))
        return points


# ── 可视化 ────────────────────────────────────────────────────────────────────

class MotorVisualizer:
    """电机测试结果可视化"""

    @staticmethod
    def draw_efficiency_map_text(points: List[EfficiencyPoint], width: int = 60) -> str:
        """终端文本效率MAP（ASCII热力图）"""
        if not points:
            return "无数据"

        # 按转矩和转速分组
        rpms = sorted(set(p.rpm for p in points))
        torques = sorted(set(p.torque for p in points))

        if len(rpms) < 2 or len(torques) < 2:
            return "数据点不足以生成MAP"

        # 构建矩阵
        eff_map = {}
        for p in points:
            eff_map[(p.rpm, p.torque)] = p.efficiency

        # ASCII字符表示效率等级
        chars = " .:-=+*#%@"
        max_eff = max(p.efficiency for p in points)
        min_eff = min(p.efficiency for p in points)

        lines = []
        lines.append("=" * width)
        lines.append("  电机效率MAP (ASCII表示)")
        lines.append("  坐标: 转速(RPM) x 转矩(Nm)")
        lines.append("  效率: ' '=<10% ... '@'>90%")
        lines.append("=" * width)

        # 选取合理范围的rpm和torque（最多20x20显示）
        step_r = max(1, len(rpms) // 20)
        step_t = max(1, len(torques) // 20)
        rpms_d = rpms[::step_r][:20]
        torques_d = torques[::step_t][:20]

        # 列标题（转矩）
        header = "RPM\\T(Nm) |"
        for t in torques_d:
            header += f"{t:5.2f}"[-4:]
        lines.append(header)
        lines.append("-" * len(header))

        for r in rpms_d:
            row = f"{r:>9.0f} |"
            for t in torques_d:
                eff = eff_map.get((r, t), 0)
                idx = int((eff - min_eff) / (max_eff - min_eff + 0.01) * (len(chars) - 1))
                idx = max(0, min(idx, len(chars) - 1))
                row += f" {chars[idx]} "
            lines.append(row)

        lines.append("-" * len(header))
        lines.append(f"  最高效率: {max_eff:.1f}%  最低效率: {min_eff:.1f}%")
        return "\n".join(lines)

    @staticmethod
    def export_csv(points: List[EfficiencyPoint], filepath: str):
        """导出效率MAP为CSV"""
        with open(filepath, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['RPM', 'Torque_Nm', 'Current_A', 'Voltage_V',
                             'PowerIn_W', 'PowerOut_W', 'Efficiency_%'])
            for p in points:
                writer.writerow([p.rpm, p.torque, p.current, p.voltage,
                                 p.power_in, p.power_out, p.efficiency])
        print(f"  已导出 {len(points)} 个数据点到: {filepath}")

    @staticmethod
    def try_plot_efficiency_map(points: List[EfficiencyPoint], output: str = "efficiency_map.png"):
        """尝试用matplotlib绘制效率MAP等高线图"""
        try:
            import numpy as np
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            print("  [提示] 安装 matplotlib 和 numpy 可生成图表: pip install matplotlib numpy")
            return None

        rpms = [p.rpm for p in points]
        torques = [p.torque for p in points]
        effs = [p.efficiency for p in points]

        # 插值网格
        rpm_arr = np.array(rpms)
        tq_arr = np.array(torques)
        eff_arr = np.array(effs)

        rpm_grid = np.linspace(rpm_arr.min(), rpm_arr.max(), 50)
        tq_grid = np.linspace(tq_arr.min(), tq_arr.max(), 50)
        RPM, TQ = np.meshgrid(rpm_grid, tq_grid)

        from scipy.interpolate import griddata
        EFF = griddata((rpm_arr, tq_arr), eff_arr, (RPM, TQ), method='cubic')

        fig, ax = plt.subplots(figsize=(10, 8))
        cs = ax.contourf(RPM, TQ, EFF, levels=15, cmap='RdYlGn')
        ax.contour(RPM, TQ, EFF, levels=15, colors='k', linewidths=0.5)
        plt.colorbar(cs, ax=ax, label='效率 (%)')
        ax.set_xlabel('转速 (RPM)')
        ax.set_ylabel('转矩 (Nm)')
        ax.set_title('电机效率MAP')
        fig.tight_layout()
        fig.savefig(output, dpi=150)
        plt.close(fig)
        print(f"  效率MAP已保存: {output}")
        return output


# ── CLI ───────────────────────────────────────────────────────────────────────

def cmd_kv(args):
    """KV值测量计算"""
    calc = MotorCalculator()
    kv = calc.calc_kv(args.rpm, args.voltage)
    rel = calc.calc_kv_kt_relation(kv=kv)

    print("=" * 50)
    print("  KV值测量结果")
    print("=" * 50)
    print(f"  施加电压:    {args.voltage:.2f} V")
    print(f"  测量转速:    {args.rpm:.0f} RPM")
    print(f"  ► KV值:     {kv:.2f} RPM/V")
    print(f"  ► 反算Kt:   {rel['kt_from_kv']:.6f} Nm/A")
    print(f"  ► 反电动势常数: {calc.calc_back_emf_constant(kv):.6f} V/(rad/s)")
    return kv


def cmd_kt(args):
    """Kt值测量计算"""
    calc = MotorCalculator()
    kt = calc.calc_kt(args.torque, args.current)
    rel = calc.calc_kv_kt_relation(kt=kt)

    print("=" * 50)
    print("  Kt转矩常数测量结果")
    print("=" * 50)
    print(f"  测量转矩:    {args.torque:.4f} Nm")
    print(f"  测量电流:    {args.current:.3f} A")
    print(f"  ► Kt值:     {kt:.6f} Nm/A")
    print(f"  ► 反算KV:   {rel['kv_from_kt']:.2f} RPM/V")
    return kt


def cmd_resistance(args):
    """电枢电阻测量"""
    calc = MotorCalculator()
    if args.v2 is not None and args.i2 is not None:
        rm = calc.calc_resistance_two_point(args.v_drop, args.current, args.v2, args.i2)
        method = "两点法（消除反电动势）"
    else:
        rm = calc.calc_resistance(args.v_drop, args.current)
        method = "欧姆定律法"

    print("=" * 50)
    print("  电枢电阻测量结果")
    print("=" * 50)
    print(f"  测量方法:    {method}")
    if args.v2 is not None:
        print(f"  测点1:      V={args.v_drop:.4f}V  I={args.current:.4f}A")
        print(f"  测点2:      V={args.v2:.4f}V  I={args.i2:.4f}A")
    else:
        print(f"  电压降:     {args.v_drop:.4f} V")
        print(f"  测量电流:   {args.current:.4f} A")
    print(f"  ► 电阻:     {rm:.6f} Ω ({rm*1000:.3f} mΩ)")
    return rm


def cmd_inductance(args):
    """电感测量"""
    calc = MotorCalculator()
    lm = calc.calc_inductance(args.freq, args.impedance, args.resistance)

    print("=" * 50)
    print("  电感测量结果")
    print("=" * 50)
    print(f"  测试频率:    {args.freq:.0f} Hz")
    print(f"  测量阻抗:    {args.impedance:.4f} Ω")
    print(f"  已知电阻:    {args.resistance:.4f} Ω")
    print(f"  ► 电感:     {lm:.4f} mH ({lm/1000:.6f} H)")
    return lm


def cmd_efficiency(args):
    """效率计算"""
    calc = MotorCalculator()
    p_in = calc.calc_electrical_power(args.voltage, args.current)
    p_out = calc.calc_mechanical_power(args.torque, args.rpm)
    eff = calc.calc_efficiency(p_out, p_in)
    p_loss = p_in - p_out

    print("=" * 50)
    print("  单点效率计算结果")
    print("=" * 50)
    print(f"  电压:       {args.voltage:.2f} V")
    print(f"  电流:       {args.current:.3f} A")
    print(f"  转矩:       {args.torque:.4f} Nm")
    print(f"  转速:       {args.rpm:.0f} RPM")
    print(f"  ► 输入功率:  {p_in:.2f} W")
    print(f"  ► 输出功率:  {p_out:.2f} W")
    print(f"  ► 损耗:     {p_loss:.2f} W")
    print(f"  ► 效率:     {eff:.2f} %")
    if p_loss > 0:
        km = calc.calc_motor_constant(args.torque, p_loss)
        print(f"  ► 电机常数:  {km:.4f} Nm/√W")
    return eff


def cmd_map(args):
    """生成效率MAP"""
    calc = MotorCalculator()
    motor = MotorParams()

    if args.csv:
        # 从CSV读取数据
        points = []
        with open(args.csv, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                points.append(EfficiencyPoint(
                    rpm=float(row.get('RPM', row.get('rpm', 0))),
                    torque=float(row.get('Torque_Nm', row.get('torque', 0))),
                    current=float(row.get('Current_A', row.get('current', 0))),
                    voltage=float(row.get('Voltage_V', row.get('voltage', 0))),
                    power_in=float(row.get('PowerIn_W', row.get('power_in', 0))),
                    power_out=float(row.get('PowerOut_W', row.get('power_out', 0))),
                    efficiency=float(row.get('Efficiency_%', row.get('efficiency', 0)))
                ))
        print(f"  从CSV加载 {len(points)} 个数据点")
    else:
        # 使用模型参数生成
        motor.kv = args.kv
        motor.kt = args.kt if args.kt > 0 else (60.0 / (2 * math.pi * args.kv))
        motor.rm = args.rm

        v_range = [args.voltage * i / 10 for i in range(2, 11)]
        r_range = [args.rpm_max * i / 20 for i in range(1, 21)]

        points = calc.generate_efficiency_map(motor, v_range, r_range)
        print(f"  模型生成 {len(points)} 个数据点")

    if not points:
        print("  [错误] 没有有效数据点")
        return

    # 文本MAP
    viz = MotorVisualizer()
    text_map = viz.draw_efficiency_map_text(points)
    print(text_map)

    # CSV导出
    csv_path = args.output.replace('.png', '.csv') if args.output else "efficiency_map.csv"
    viz.export_csv(points, csv_path)

    # 图表绘制
    if args.output:
        viz.try_plot_efficiency_map(points, args.output)

    return points


def cmd_full(args):
    """综合测试汇总"""
    calc = MotorCalculator()
    params = MotorParams(name=args.name)

    print("=" * 60)
    print(f"  电机综合特性测试: {args.name}")
    print("=" * 60)

    # KV
    if args.kv_rpm and args.kv_v:
        params.kv = calc.calc_kv(args.kv_rpm, args.kv_v)
        print(f"\n  [KV测量] {args.kv_rpm} RPM / {args.kv_v} V = {params.kv:.2f} RPM/V")

    # Kt
    if args.kt_torque and args.kt_current:
        params.kt = calc.calc_kt(args.kt_torque, args.kt_current)
        print(f"  [Kt测量] {args.kt_torque} Nm / {args.kt_current} A = {params.kt:.6f} Nm/A")

    # KV-Kt一致性检查
    if params.kv > 0 and params.kt > 0:
        expected_kt = 60.0 / (2 * math.pi * params.kv)
        error_pct = abs(params.kt - expected_kt) / expected_kt * 100
        status = "✓ 一致" if error_pct < 5 else "⚠ 偏差较大"
        print(f"  [一致性] KV→Kt={expected_kt:.6f}  实测={params.kt:.6f}  偏差={error_pct:.1f}% {status}")

    # 电阻
    if args.rm_v and args.rm_i:
        params.rm = calc.calc_resistance(args.rm_v, args.rm_i)
        print(f"  [电阻测量] {params.rm:.4f} Ω")

    # 电感
    if args.lm_freq and args.lm_z and args.lm_r:
        params.lm = calc.calc_inductance(args.lm_freq, args.lm_z, args.lm_r)
        print(f"  [电感测量] {params.lm:.4f} mH")

    # 汇总
    if params.kv > 0:
        params.km = calc.calc_motor_constant(
            params.kt if params.kt > 0 else 1.0,
            params.rm if params.rm > 0 else 0.1
        )

    print("\n  ── 参数汇总 ──")
    for k, v in asdict(params).items():
        if v and v != 0 and v != "" and v != "未命名电机":
            print(f"    {k}: {v}")

    # 保存JSON
    out = f"motor_{args.name.replace(' ', '_')}_result.json"
    data = asdict(params)
    data['test_time'] = time.strftime('%Y-%m-%d %H:%M:%S')
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    print(f"\n  结果已保存: {out}")
    return params


def main():
    parser = argparse.ArgumentParser(
        description='电机特性测试工具 - 电赛资产库',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s kv --voltage 12 --rpm 3000
  %(prog)s kt --torque 0.5 --current 3.2
  %(prog)s resistance --v-drop 0.64 --current 2.0
  %(prog)s inductance --freq 1000 --impedance 10.5 --resistance 0.5
  %(prog)s efficiency --voltage 12 --current 3.0 --torque 0.3 --rpm 2000
  %(prog)s map --kv 800 --rm 0.1 --voltage 12 --rpm-max 8000
  %(prog)s full --name "2845" --kv-rpm 3000 --kv-v 12 --kt-torque 0.5 --kt-current 3.2
        """
    )
    sub = parser.add_subparsers(dest='command', help='子命令')

    # KV
    p_kv = sub.add_parser('kv', help='KV值测量')
    p_kv.add_argument('--voltage', type=float, required=True, help='施加电压 (V)')
    p_kv.add_argument('--rpm', type=float, required=True, help='无负载转速 (RPM)')

    # Kt
    p_kt = sub.add_parser('kt', help='Kt转矩常数测量')
    p_kt.add_argument('--torque', type=float, required=True, help='测量转矩 (Nm)')
    p_kt.add_argument('--current', type=float, required=True, help='测量电流 (A)')

    # 电阻
    p_r = sub.add_parser('resistance', help='电枢电阻测量')
    p_r.add_argument('--v-drop', type=float, required=True, help='电压降 (V)')
    p_r.add_argument('--current', type=float, required=True, help='测量电流 (A)')
    p_r.add_argument('--v2', type=float, help='第二测点电压 (两点法)')
    p_r.add_argument('--i2', type=float, help='第二测点电流 (两点法)')

    # 电感
    p_l = sub.add_parser('inductance', help='电感测量')
    p_l.add_argument('--freq', type=float, required=True, help='测试频率 (Hz)')
    p_l.add_argument('--impedance', type=float, required=True, help='测量阻抗 (Ω)')
    p_l.add_argument('--resistance', type=float, required=True, help='已知电阻 (Ω)')

    # 效率
    p_e = sub.add_parser('efficiency', help='单点效率计算')
    p_e.add_argument('--voltage', type=float, required=True, help='电压 (V)')
    p_e.add_argument('--current', type=float, required=True, help='电流 (A)')
    p_e.add_argument('--torque', type=float, required=True, help='转矩 (Nm)')
    p_e.add_argument('--rpm', type=float, required=True, help='转速 (RPM)')

    # MAP
    p_m = sub.add_parser('map', help='效率MAP生成')
    p_m.add_argument('--csv', type=str, help='输入CSV文件路径')
    p_m.add_argument('--kv', type=float, default=800, help='KV值')
    p_m.add_argument('--kt', type=float, default=0, help='Kt值')
    p_m.add_argument('--rm', type=float, default=0.1, help='电枢电阻 (Ω)')
    p_m.add_argument('--voltage', type=float, default=12, help='额定电压')
    p_m.add_argument('--rpm-max', type=float, default=8000, help='最大转速')
    p_m.add_argument('--output', type=str, default='efficiency_map.png', help='输出图表路径')

    # 综合
    p_f = sub.add_parser('full', help='综合特性测试')
    p_f.add_argument('--name', type=str, default='motor', help='电机名称')
    p_f.add_argument('--kv-rpm', type=float, help='KV测量-转速')
    p_f.add_argument('--kv-v', type=float, help='KV测量-电压')
    p_f.add_argument('--kt-torque', type=float, help='Kt测量-转矩')
    p_f.add_argument('--kt-current', type=float, help='Kt测量-电流')
    p_f.add_argument('--rm-v', type=float, help='电阻测量-电压降')
    p_f.add_argument('--rm-i', type=float, help='电阻测量-电流')
    p_f.add_argument('--lm-freq', type=float, help='电感测量-频率')
    p_f.add_argument('--lm-z', type=float, help='电感测量-阻抗')
    p_f.add_argument('--lm-r', type=float, help='电感测量-电阻')

    args = parser.parse_args()
    if not args.command:
        parser.print_help()
        return

    dispatch = {
        'kv': cmd_kv, 'kt': cmd_kt, 'resistance': cmd_resistance,
        'inductance': cmd_inductance, 'efficiency': cmd_efficiency,
        'map': cmd_map, 'full': cmd_full
    }
    dispatch[args.command](args)


if __name__ == '__main__':
    main()
