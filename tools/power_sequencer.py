#!/usr/bin/env python3
"""
电源时序设计工具 - 上电顺序/时序图/保护电路设计
用于电赛电源系统设计与验证
"""
import argparse
import json
import sys


# ── 上电时序分析 ──────────────────────────────────────────────
def analyze_power_sequence(rails):
    """
    分析电源上电顺序是否合理
    rails: [{'name': str, 'voltage_v': float, 'delay_ms': float,
             'rise_ms': float, 'depends_on': [str], 'max_inrush_a': float}, ...]
    """
    # 建立依赖图并检查
    rail_map = {r['name']: r for r in rails}
    issues = []
    sequence = []

    for rail in sorted(rails, key=lambda r: r['delay_ms']):
        r = rail
        # 检查依赖是否已上电
        for dep in r.get('depends_on', []):
            if dep not in rail_map:
                issues.append(f"错误: {r['name']} 依赖的 {dep} 不存在")
            else:
                dep_rail = rail_map[dep]
                dep_ready = dep_rail['delay_ms'] + dep_rail.get('rise_ms', 1)
                if r['delay_ms'] < dep_ready:
                    issues.append(
                        f"警告: {r['name']} 在 {r['delay_ms']}ms 上电, "
                        f"但依赖 {dep} 在 {dep_ready:.1f}ms 才就绪"
                    )

        # 检查电压差（防止后级先于前级上电）
        sequence.append({
            'name': r['name'],
            'voltage_v': r['voltage_v'],
            'delay_ms': r['delay_ms'],
            'rise_ms': r.get('rise_ms', 1.0),
            'ready_ms': r['delay_ms'] + r.get('rise_ms', 1.0),
        })

    # 生成ASCII时序图
    timeline = generate_ascii_timeline(sequence)

    return {
        'sequence': sequence,
        'issues': issues,
        'timeline': timeline,
        'total_time_ms': max(s['ready_ms'] for s in sequence) if sequence else 0,
    }


def generate_ascii_timeline(sequence):
    """生成ASCII时序图"""
    if not sequence:
        return ""

    max_time = max(s['ready_ms'] for s in sequence)
    scale = 50 / max_time if max_time > 0 else 1  # 50字符宽度
    lines = []
    lines.append(f"{'电源轨':<12} {'电压':>6}  时序图 (每格={max_time/50:.1f}ms)")
    lines.append("-" * 70)

    for s in sequence:
        start_pos = int(s['delay_ms'] * scale)
        end_pos = int(s['ready_ms'] * scale)
        bar = ' ' * start_pos + '▓' * max(1, end_pos - start_pos) + '→'
        lines.append(f"{s['name']:<10} {s['voltage_v']:>5.1f}V  |{bar}")

    lines.append("-" * 70)
    # 时间轴
    axis = " " * 12 + "      "
    for i in range(0, int(max_time) + 1, max(1, int(max_time / 10))):
        pos = int(i * scale)
        axis_label = f"{i}ms"
        while len(axis) < len("电源轨      电压  ") + pos:
            axis += " "
        axis = axis[:len("电源轨      电压  ") + pos] + axis_label
    lines.append(axis)
    return "\n".join(lines)


# ── 下电时序分析 ──────────────────────────────────────────────
def analyze_power_down_sequence(rails):
    """分析下电顺序（反向依赖）"""
    issues = []
    for rail in rails:
        # 下电时，被依赖的电源应最后关闭
        for dep in rail.get('depends_on', []):
            issues.append(f"提示: {dep} 应在 {rail['name']} 之后关闭")

    # 建议下电顺序：按delay_ms降序
    shutdown_order = sorted(rails, key=lambda r: -r['delay_ms'])
    return {
        'shutdown_order': [r['name'] for r in shutdown_order],
        'issues': issues,
    }


# ── 保护电路参数计算 ──────────────────────────────────────────
def calculate_protection(voltage_v, max_current_a, load_capacitance_uf=100):
    """
    计算保护电路参数
    - 软启动时间
    - 浪涌电流限制
    - TVS选型
    - 滤波电容
    """
    # 软启动电阻（限制浪涌电流到max_current的50%）
    soft_start_target = max_current_a * 0.5
    soft_start_resistor = voltage_v / soft_start_target if soft_start_target > 0 else 0

    # 软启动时间 (T = R * C)
    soft_start_time_ms = soft_start_resistor * load_capacitance_uf * 1e-3

    # 浪涌能量 (E = 0.5 * C * V^2)
    inrush_energy_j = 0.5 * load_capacitance_uf * 1e-6 * voltage_v ** 2

    # TVS钳位电压（取1.2倍工作电压）
    tvs_clamp_v = voltage_v * 1.2
    # TVS功率（假设10/1000us脉冲）
    tvs_power_w = inrush_energy_j / 0.001  # 简化估算

    # 输入滤波电容推荐
    filter_cap_uf = max_current_a * 100  # 经验值: 100uF/A

    # 保险丝额定电流（取1.5倍工作电流）
    fuse_rating_a = max_current_a * 1.5

    return {
        'voltage_v': voltage_v,
        'max_current_a': max_current_a,
        'soft_start_resistor_ohm': round(soft_start_resistor, 2),
        'soft_start_time_ms': round(soft_start_time_ms, 2),
        'inrush_energy_mJ': round(inrush_energy_j * 1000, 2),
        'tvs_clamp_voltage_v': round(tvs_clamp_v, 1),
        'tvs_min_power_w': round(tvs_power_w, 2),
        'input_filter_cap_uF': round(filter_cap_uf, 0),
        'fuse_rating_A': round(fuse_rating_a, 2),
    }


# ── DC-DC效率估算 ────────────────────────────────────────────
def estimate_dcdc_efficiency(vin_v, vout_v, iout_a, topology='buck'):
    """
    估算DC-DC转换效率和功耗
    topology: buck/boost/buck-boost/ldo
    """
    if topology == 'ldo':
        efficiency = vout_v / vin_v if vin_v > 0 else 0
        power_loss = (vin_v - vout_v) * iout_a
    elif topology == 'buck':
        # 典型buck效率 85-95%
        base_eff = 0.92
        # 低压差时效率更高
        ratio = vout_v / vin_v if vin_v > 0 else 0
        efficiency = base_eff * ratio / (ratio + 0.05 * (1 - ratio))
        power_loss = vout_v * iout_a * (1 - efficiency) / efficiency
    elif topology == 'boost':
        base_eff = 0.88
        ratio = vin_v / vout_v if vout_v > 0 else 0
        efficiency = base_eff * ratio / (ratio + 0.08 * (1 - ratio))
        power_loss = vout_v * iout_a * (1 - efficiency) / efficiency
    else:
        efficiency = 0.85
        power_loss = vout_v * iout_a * (1 - efficiency) / efficiency

    pin = vout_v * iout_a / efficiency if efficiency > 0 else 0
    return {
        'topology': topology,
        'vin_v': vin_v,
        'vout_v': vout_v,
        'iout_a': iout_a,
        'efficiency_percent': round(efficiency * 100, 1),
        'input_power_W': round(pin, 2),
        'output_power_W': round(vout_v * iout_a, 2),
        'power_loss_W': round(power_loss, 2),
        'thermal_resistance_needed': round(power_loss / 40, 2) if power_loss > 0 else 0,  # 40°C温升
    }


# ── CLI ──────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='电源时序设计工具')
    sub = parser.add_subparsers(dest='cmd')

    # 上电时序
    p_seq = sub.add_parser('sequence', help='分析上电时序')
    p_seq.add_argument('--json', required=True, help='电源轨配置JSON')
    p_seq.add_argument('--out', help='输出文件')

    # 保护电路
    p_prot = sub.add_parser('protect', help='保护电路参数计算')
    p_prot.add_argument('--voltage', type=float, required=True, help='工作电压V')
    p_prot.add_argument('--current', type=float, required=True, help='最大电流A')
    p_prot.add_argument('--cap', type=float, default=100, help='负载电容uF')

    # DC-DC效率
    p_eff = sub.add_parser('efficiency', help='DC-DC效率估算')
    p_eff.add_argument('--vin', type=float, required=True, help='输入电压V')
    p_eff.add_argument('--vout', type=float, required=True, help='输出电压V')
    p_eff.add_argument('--iout', type=float, required=True, help='输出电流A')
    p_eff.add_argument('--topo', default='buck', choices=['buck', 'boost', 'buck-boost', 'ldo'])

    args = parser.parse_args()
    if not args.cmd:
        parser.print_help()
        return

    if args.cmd == 'sequence':
        rails = json.loads(args.json)
        result = analyze_power_sequence(rails)
        print("\n=== 上电时序分析 ===\n")
        print(result['timeline'])
        if result['issues']:
            print("\n问题:")
            for issue in result['issues']:
                print(f"  ⚠ {issue}")
        print(f"\n总上电时间: {result['total_time_ms']:.1f}ms")
        if args.out:
            with open(args.out, 'w', encoding='utf-8') as f:
                json.dump(result, f, indent=2, ensure_ascii=False)

    elif args.cmd == 'protect':
        result = calculate_protection(args.voltage, args.current, args.cap)
        print(json.dumps(result, indent=2, ensure_ascii=False))

    elif args.cmd == 'efficiency':
        result = estimate_dcdc_efficiency(args.vin, args.vout, args.iout, args.topo)
        print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
