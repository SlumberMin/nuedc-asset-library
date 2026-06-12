#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
热仿真器 - PCB热阻/散热计算/温度预测
======================================
功能：
  - 元器件结温计算（热阻模型）
  - PCB铜皮散热面积估算
  - 散热器选型计算
  - 多元器件热耦合分析
  - 自然对流/强制对流散热评估
  - 温度降额分析
"""

import argparse
import json
import math
import sys
from datetime import datetime

# ============================================================
# 常量与参考数据
# ============================================================

# 常见封装热阻参考 (°C/W) - Junction to Ambient
PACKAGE_THERMAL = {
    # 封装名: (θJA自然对流, θJA强制风冷, θJC, 尺寸mm)
    "SOT-23":     {"theta_ja": 200, "theta_ja_fan": 120, "theta_jc": 80, "size": (2.9, 1.6, 1.1)},
    "SOT-223":    {"theta_ja": 60,  "theta_ja_fan": 40,  "theta_jc": 15, "size": (6.5, 3.5, 1.6)},
    "SOT-89":     {"theta_ja": 100, "theta_ja_fan": 65,  "theta_jc": 30, "size": (4.6, 2.6, 1.5)},
    "SOIC-8":     {"theta_ja": 120, "theta_ja_fan": 80,  "theta_jc": 40, "size": (4.9, 3.9, 1.5)},
    "SOIC-16":    {"theta_ja": 85,  "theta_ja_fan": 60,  "theta_jc": 30, "size": (9.9, 3.9, 1.5)},
    "TSSOP-16":   {"theta_ja": 100, "theta_ja_fan": 70,  "theta_jc": 35, "size": (5.0, 4.4, 1.1)},
    "QFP-48":     {"theta_ja": 50,  "theta_ja_fan": 35,  "theta_jc": 12, "size": (7.0, 7.0, 1.0)},
    "QFP-64":     {"theta_ja": 45,  "theta_ja_fan": 30,  "theta_jc": 10, "size": (10.0, 10.0, 1.0)},
    "QFP-100":    {"theta_ja": 38,  "theta_ja_fan": 25,  "theta_jc": 8,  "size": (14.0, 14.0, 1.0)},
    "QFP-144":    {"theta_ja": 32,  "theta_ja_fan": 22,  "theta_jc": 6,  "size": (20.0, 20.0, 1.0)},
    "LQFP-48":    {"theta_ja": 55,  "theta_ja_fan": 38,  "theta_jc": 14, "size": (7.0, 7.0, 1.4)},
    "LQFP-64":    {"theta_ja": 48,  "theta_ja_fan": 32,  "theta_jc": 11, "size": (10.0, 10.0, 1.4)},
    "LQFP-100":   {"theta_ja": 40,  "theta_ja_fan": 28,  "theta_jc": 9,  "size": (14.0, 14.0, 1.4)},
    "BGA-256":    {"theta_ja": 25,  "theta_ja_fan": 18,  "theta_jc": 5,  "size": (17.0, 17.0, 1.8)},
    "QFN-16":     {"theta_ja": 45,  "theta_ja_fan": 30,  "theta_jc": 10, "size": (4.0, 4.0, 0.9)},
    "QFN-32":     {"theta_ja": 35,  "theta_ja_fan": 24,  "theta_jc": 8,  "size": (5.0, 5.0, 0.9)},
    "QFN-48":     {"theta_ja": 28,  "theta_ja_fan": 20,  "theta_jc": 6,  "size": (7.0, 7.0, 0.9)},
    "D2PAK":      {"theta_ja": 40,  "theta_ja_fan": 25,  "theta_jc": 2,  "size": (10.0, 9.0, 4.5)},
    "TO-220":     {"theta_ja": 60,  "theta_ja_fan": 40,  "theta_jc": 3,  "size": (10.0, 15.0, 4.5)},
    "TO-247":     {"theta_ja": 40,  "theta_ja_fan": 28,  "theta_jc": 2,  "size": (15.0, 20.0, 5.0)},
    "TO-252":     {"theta_ja": 50,  "theta_ja_fan": 35,  "theta_jc": 5,  "size": (6.5, 6.0, 2.3)},
    "0402":       {"theta_ja": 300, "theta_ja_fan": 200, "theta_jc": 100, "size": (1.0, 0.5, 0.5)},
    "0603":       {"theta_ja": 250, "theta_ja_fan": 170, "theta_jc": 80, "size": (1.6, 0.8, 0.5)},
    "0805":       {"theta_ja": 200, "theta_ja_fan": 140, "theta_jc": 60, "size": (2.0, 1.25, 0.5)},
    "1206":       {"theta_ja": 170, "theta_ja_fan": 120, "theta_jc": 50, "size": (3.2, 1.6, 0.5)},
    "1210":       {"theta_ja": 150, "theta_ja_fan": 100, "theta_jc": 40, "size": (3.2, 2.5, 0.5)},
    "2512":       {"theta_ja": 120, "theta_ja_fan": 85,  "theta_jc": 30, "size": (6.3, 3.2, 0.5)},
}

# PCB材料参数
PCB_MATERIALS = {
    "FR-4": {
        "thermal_conductivity": 0.25,  # W/(m·K)
        "density": 1850,               # kg/m³
        "specific_heat": 880,          # J/(kg·K)
        "cte_xy": 14,                  # ppm/°C (XY方向)
        "cte_z": 70,                   # ppm/°C (Z方向)
    },
    "铝基板": {
        "thermal_conductivity": 1.5,
        "density": 2700,
        "specific_heat": 900,
        "cte_xy": 23,
        "cte_z": 23,
    },
    "铜基板": {
        "thermal_conductivity": 3.0,
        "density": 8900,
        "specific_heat": 385,
        "cte_xy": 17,
        "cte_z": 17,
    },
    "陶瓷基板": {
        "thermal_conductivity": 25.0,
        "density": 3800,
        "specific_heat": 800,
        "cte_xy": 6,
        "cte_z": 6,
    },
}

# 散热器参考数据
HEATSINK_SAMPLES = {
    "无散热器":    {"theta_hs": 0,    "size": (0, 0, 0)},
    "小型贴片":    {"theta_hs": 30,   "size": (10, 10, 5)},
    "TO-220标准":  {"theta_hs": 15,   "size": (20, 15, 10)},
    "中型散热器":  {"theta_hs": 8,    "size": (30, 30, 10)},
    "大型散热器":  {"theta_hs": 4,    "size": (50, 50, 15)},
    "翅片散热器S": {"theta_hs": 5,    "size": (25, 25, 12)},
    "翅片散热器M": {"theta_hs": 3,    "size": (40, 40, 20)},
    "翅片散热器L": {"theta_hs": 1.5,  "size": (60, 60, 25)},
}


class ThermalComponent:
    """热分析元器件"""

    def __init__(self, name, package, power_mw, max_junction_temp=125,
                 heatsink=None, airflow="natural"):
        self.name = name
        self.package = package
        self.power_mw = power_mw           # 功耗(mW)
        self.max_tj = max_junction_temp    # 最大结温(°C)
        self.heatsink = heatsink           # 散热器名称
        self.airflow = airflow             # "natural" 或 "forced"

    @property
    def power_w(self):
        return self.power_mw / 1000

    def to_dict(self):
        return {
            "name": self.name,
            "package": self.package,
            "power_mw": self.power_mw,
            "max_junction_temp": self.max_tj,
            "heatsink": self.heatsink,
            "airflow": self.airflow,
        }


def calculate_junction_temp(component, ambient_temp=25.0):
    """
    计算结温
    Tj = Ta + P × (θJC + θCS + θSA)
    θJA = θJC + θCS + θSA (无散热器时θCS+θSA=θJA-θJC)
    """
    pkg = PACKAGE_THERMAL.get(component.package)
    if not pkg:
        return {"error": f"未知封装: {component.package}"}

    # 选择自然/强制对流热阻
    if component.airflow == "forced":
        theta_ja = pkg['theta_ja_fan']
    else:
        theta_ja = pkg['theta_ja']

    theta_jc = pkg['theta_jc']
    theta_cs = 0.5  # 芯片到散热器热界面热阻(典型值)

    # 散热器热阻
    hs_info = HEATSINK_SAMPLES.get(component.heatsink or "无散热器", {"theta_hs": 0})
    theta_sa = hs_info['theta_hs'] if component.heatsink else (theta_ja - theta_jc)

    # 结温计算
    if component.heatsink:
        theta_total = theta_jc + theta_cs + theta_sa
    else:
        theta_total = theta_ja

    tj = ambient_temp + component.power_w * theta_total
    margin = component.max_tj - tj
    margin_pct = margin / component.max_tj * 100

    # 降额温度（推荐最大工作结温为最大额定值的80%）
    derating_temp = component.max_tj * 0.8
    meets_derating = tj <= derating_temp

    return {
        "name": component.name,
        "package": component.package,
        "power_mw": component.power_mw,
        "ambient_temp": ambient_temp,
        "junction_temp": round(tj, 2),
        "max_junction_temp": component.max_tj,
        "margin": round(margin, 2),
        "margin_pct": round(margin_pct, 1),
        "theta_ja": theta_ja,
        "theta_jc": theta_jc,
        "theta_cs": theta_cs,
        "theta_sa": theta_sa,
        "theta_total": round(theta_total, 1),
        "heatsink": component.heatsink or "无",
        "airflow": component.airflow,
        "derating_temp": round(derating_temp, 1),
        "meets_derating": meets_derating,
        "status": "安全" if tj < component.max_tj else "过热",
    }


def calculate_max_power(package, ambient_temp=25.0, max_tj=125.0, airflow="natural"):
    """
    计算给定封装的最大允许功耗
    Pmax = (Tj_max - Ta) / θJA
    """
    pkg = PACKAGE_THERMAL.get(package)
    if not pkg:
        return None

    theta_ja = pkg['theta_ja_fan'] if airflow == "forced" else pkg['theta_ja']
    p_max = (max_tj - ambient_temp) / theta_ja  # W
    return round(p_max * 1000, 2)  # mW


def estimate_pcb_copper_area(power_w, temp_rise=20, copper_thickness=35, board_layers=2):
    """
    估算PCB铜皮散热面积
    参数:
      power_w: 功耗(W)
      temp_rise: 允许温升(°C)
      copper_thickness: 铜厚(μm)，默认35μm(1oz)
      board_layers: 散热铜层数
    返回: 所需铜皮面积 (cm²)
    """
    # 简化经验公式: A = P / (k × ΔT)
    # k ≈ 0.025 ~ 0.04 W/(cm²·°C) 对于水平PCB自然对流
    k = 0.03  # 经验散热系数 W/(cm²·°C)

    # 铜厚修正系数
    thickness_factor = copper_thickness / 35.0

    # 多层修正
    layer_factor = 1.0 + 0.5 * (board_layers - 1)

    area = power_w / (k * temp_rise) * thickness_factor / layer_factor

    return round(area, 2)


def calculate_heatsink_required(power_w, theta_jc, theta_ja, ambient=25, max_tj=125):
    """
    计算所需散热器热阻
    θSA = (Tj_max - Ta) / P - θJC - θCS
    """
    theta_cs = 0.5  # 界面热阻
    theta_sa_required = (max_tj - ambient) / power_w - theta_jc - theta_cs
    return round(theta_sa_required, 2)


def estimate_natural_convection(pcb_area_cm2, power_total_w, ambient_temp=25):
    """
    估算自然对流散热能力
    基于经验公式:
    Q = h × A × ΔT
    h ≈ 10 W/(m²·K) 垂直PCB自然对流
    h ≈ 6 W/(m²·K) 水平PCB自然对流
    """
    area_m2 = pcb_area_cm2 / 10000

    # 垂直放置
    h_vert = 10  # W/(m²·K)
    # 水平放置(热面朝上)
    h_horiz_up = 6
    # 水平放置(热面朝下)
    h_horiz_down = 3

    delta_t_vert = power_total_w / (h_vert * area_m2) if area_m2 > 0 else float('inf')
    delta_t_horiz_up = power_total_w / (h_horiz_up * area_m2) if area_m2 > 0 else float('inf')
    delta_t_horiz_down = power_total_w / (h_horiz_down * area_m2) if area_m2 > 0 else float('inf')

    return {
        "pcb_area_cm2": pcb_area_cm2,
        "total_power_w": power_total_w,
        "vertical_temp_rise": round(delta_t_vert, 1),
        "vertical_final": round(ambient_temp + delta_t_vert, 1),
        "horizontal_up_temp_rise": round(delta_t_horiz_up, 1),
        "horizontal_up_final": round(ambient_temp + delta_t_horiz_up, 1),
        "horizontal_down_temp_rise": round(delta_t_horiz_down, 1),
        "horizontal_down_final": round(ambient_temp + delta_t_horiz_down, 1),
    }


def analyze_thermal_coupling(components, ambient_temp=25.0):
    """
    分析多元器件热耦合效应
    相邻器件会互相加热，简单模型：每相邻器件增加2-5°C
    """
    results = []
    n = len(components)

    for i, comp in enumerate(components):
        result = calculate_junction_temp(comp, ambient_temp)
        if "error" in result:
            results.append(result)
            continue

        # 简单热耦合估算：相邻器件数量 × 耦合系数
        coupling_temp = 0
        for j, other in enumerate(components):
            if i == j:
                continue
            # 简化：假设都在PCB上，距离较近
            coupling_factor = 0.02  # 2%热量传导
            coupling_temp += other.power_w * coupling_factor * result['theta_total']

        result['coupling_temp_rise'] = round(coupling_temp, 2)
        result['junction_temp_with_coupling'] = round(result['junction_temp'] + coupling_temp, 2)
        result['status_coupled'] = "安全" if result['junction_temp_with_coupling'] < comp.max_tj else "过热"
        results.append(result)

    return results


def load_components_from_json(filepath):
    """从JSON文件加载元器件列表"""
    with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)
    components = []
    for item in data.get('components', data if isinstance(data, list) else []):
        components.append(ThermalComponent(
            name=item.get('name', ''),
            package=item.get('package', 'SOIC-8'),
            power_mw=item.get('power_mw', 0),
            max_junction_temp=item.get('max_junction_temp', 125),
            heatsink=item.get('heatsink'),
            airflow=item.get('airflow', 'natural'),
        ))
    return components


def generate_sample_config(output_path):
    """生成示例热分析配置"""
    config = {
        "pcb": {
            "area_cm2": 50,
            "layers": 4,
            "copper_thickness_um": 35,
            "material": "FR-4",
        },
        "ambient_temp": 40,  # 最高环境温度
        "components": [
            {"name": "MCU(STM32F407)", "package": "LQFP-100", "power_mw": 500,
             "max_junction_temp": 125, "airflow": "natural"},
            {"name": "电机驱动(L298N)", "package": "Multiwatt15", "power_mw": 3000,
             "max_junction_temp": 150, "heatsink": "中型散热器", "airflow": "natural"},
            {"name": "LDO(AMS1117-3.3)", "package": "SOT-223", "power_mw": 800,
             "max_junction_temp": 125, "airflow": "natural"},
            {"name": "WiFi模块(ESP32)", "package": "QFN-48", "power_mw": 400,
             "max_junction_temp": 125, "airflow": "natural"},
            {"name": "功率MOS管", "package": "TO-252", "power_mw": 1500,
             "max_junction_temp": 175, "heatsink": "小型贴片", "airflow": "natural"},
            {"name": "电流采样电阻", "package": "2512", "power_mw": 500,
             "max_junction_temp": 155, "airflow": "natural"},
        ],
    }
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(config, f, ensure_ascii=False, indent=2)
    print(f"[OK] 示例配置已生成: {output_path}")


def format_report(results, pcb_info=None, ambient_temp=25.0):
    """格式化热分析报告"""
    lines = []
    lines.append("=" * 70)
    lines.append("              PCB热仿真分析报告")
    lines.append(f"              生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"              环境温度: {ambient_temp}°C")
    lines.append("=" * 70)

    if pcb_info:
        mat = PCB_MATERIALS.get(pcb_info.get('material', 'FR-4'), {})
        lines.append(f"\n■ PCB参数")
        lines.append(f"  板材: {pcb_info.get('material', 'FR-4')}")
        lines.append(f"  面积: {pcb_info.get('area_cm2', 0)}cm²")
        lines.append(f"  层数: {pcb_info.get('layers', 2)}层")
        lines.append(f"  铜厚: {pcb_info.get('copper_thickness_um', 35)}μm")
        if mat:
            lines.append(f"  导热系数: {mat['thermal_conductivity']}W/(m·K)")

    # 元器件热分析
    lines.append(f"\n■ 元器件热分析")
    lines.append(f"  {'名称':<22s} {'封装':<12s} {'功耗':>8s} {'结温':>8s} {'限值':>8s} "
                f"{'余量':>8s} {'散热器':<12s} {'状态':>6s}")
    lines.append(f"  {'-'*88}")

    total_power = 0
    for r in results:
        if "error" in r:
            lines.append(f"  {r['name']:<22s} [错误: {r['error']}]")
            continue

        total_power += r['power_mw']
        status_icon = "✓" if r['status'] == "安全" else "✗"
        lines.append(f"  {r['name']:<22s} {r['package']:<12s} {r['power_mw']:>6.0f}mW "
                    f"{r['junction_temp']:>6.1f}°C {r['max_junction_temp']:>6.0f}°C "
                    f"{r['margin']:>6.1f}°C {r['heatsink']:<12s} {status_icon} {r['status']}")

    lines.append(f"  {'-'*88}")
    lines.append(f"  {'系统总功耗':<22s} {'':12s} {total_power:>6.0f}mW")

    # 降额分析
    lines.append(f"\n■ 降额分析 (推荐结温 ≤ 额定值的80%)")
    for r in results:
        if "error" in r:
            continue
        icon = "✓" if r['meets_derating'] else "✗"
        lines.append(f"  {icon} {r['name']}: {r['junction_temp']:.1f}°C "
                    f"{'≤' if r['meets_derating'] else '>'} {r['derating_temp']:.0f}°C (降额限)")

    # 热耦合分析
    has_coupling = any('coupling_temp_rise' in r for r in results)
    if has_coupling:
        lines.append(f"\n■ 热耦合分析")
        for r in results:
            if 'coupling_temp_rise' in r and r['coupling_temp_rise'] > 0:
                icon = "✓" if r['status_coupled'] == "安全" else "✗"
                lines.append(f"  {icon} {r['name']}: 耦合升温 +{r['coupling_temp_rise']:.1f}°C, "
                            f"综合结温 {r['junction_temp_with_coupling']:.1f}°C")

    # PCB散热能力估算
    if pcb_info:
        area = pcb_info.get('area_cm2', 50)
        power_total_w = total_power / 1000
        convection = estimate_natural_convection(area, power_total_w, ambient_temp)

        lines.append(f"\n■ PCB散热能力估算")
        lines.append(f"  PCB面积: {area}cm²")
        lines.append(f"  总功耗: {power_total_w*1000:.0f}mW ({power_total_w:.3f}W)")
        lines.append(f"  垂直放置: 温升 {convection['vertical_temp_rise']}°C, "
                    f"表面温度 {convection['vertical_final']}°C")
        lines.append(f"  水平(朝上): 温升 {convection['horizontal_up_temp_rise']}°C, "
                    f"表面温度 {convection['horizontal_up_final']}°C")
        lines.append(f"  水平(朝下): 温升 {convection['horizontal_down_temp_rise']}°C, "
                    f"表面温度 {convection['horizontal_down_final']}°C")

    # 铜皮面积估算
    total_power_w = total_power / 1000
    for delta_t in [10, 20, 30]:
        area_needed = estimate_pcb_copper_area(total_power_w, delta_t)
        lines.append(f"\n  温升{delta_t}°C所需铜皮面积: {area_needed}cm²")

    # 优化建议
    lines.append(f"\n■ 热设计建议")
    hotspots = [r for r in results if "error" not in r and r.get('margin_pct', 100) < 30]
    if hotspots:
        lines.append(f"  [关键] 以下器件热余量不足30%:")
        for r in hotspots:
            lines.append(f"    - {r['name']}: 余量 {r['margin_pct']:.1f}%，"
                        f"建议增加散热器或降低功耗")

    lines.append(f"  [通用] 增加铜皮面积，使用热过孔连接内层铜")
    lines.append(f"  [通用] 高功耗器件分散布局，避免热集中")
    lines.append(f"  [通用] 电解电容远离热源，温度每升高10°C寿命减半")
    lines.append(f"  [通用] 考虑使用铝基板或增加散热铜层")
    lines.append(f"  [通用] 连续工作时建议留30%以上温度余量")

    lines.append("\n" + "=" * 70)
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(
        description='热仿真器 - PCB热阻/散热计算/温度预测',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 生成示例配置
  python thermal_simulator.py --init --output thermal_config.json

  # 从配置文件分析
  python thermal_simulator.py --config thermal_config.json

  # 快速计算单个元器件结温
  python thermal_simulator.py --quick --package LQFP-100 --power 500 --ambient 40

  # 计算最大允许功耗
  python thermal_simulator.py --max-power --package TO-220 --max-tj 150

  # 估算铜皮面积
  python thermal_simulator.py --copper-area --power 2.5 --temp-rise 20

  # 列出支持的封装
  python thermal_simulator.py --list-packages
        """
    )

    # 基础模式
    parser.add_argument('--config', '-c', help='热分析配置文件 (JSON)')
    parser.add_argument('--output', '-o', help='输出文件路径', default='thermal_report.txt')
    parser.add_argument('--init', action='store_true', help='生成示例配置文件')
    parser.add_argument('--ambient', type=float, default=25.0, help='环境温度(°C) (默认: 25)')
    parser.add_argument('--coupling', action='store_true', help='启用热耦合分析')

    # 快速计算
    parser.add_argument('--quick', action='store_true', help='快速计算单个器件结温')
    parser.add_argument('--package', help='封装类型')
    parser.add_argument('--power', type=float, help='功耗(mW)')
    parser.add_argument('--max-tj', type=float, default=125, help='最大结温(°C)')
    parser.add_argument('--heatsink', help='散热器型号')

    # 最大功耗计算
    parser.add_argument('--max-power', action='store_true', help='计算最大允许功耗')

    # 铜皮面积估算
    parser.add_argument('--copper-area', action='store_true', help='估算铜皮面积')
    parser.add_argument('--temp-rise', type=float, default=20, help='允许温升(°C)')

    # 列出封装
    parser.add_argument('--list-packages', action='store_true', help='列出支持的封装')
    parser.add_argument('--list-heatsinks', action='store_true', help='列出散热器参考')

    args = parser.parse_args()

    # 列出封装
    if args.list_packages:
        print("\n支持的封装热阻参考:")
        print(f"  {'封装':<15s} {'θJA(自冷)':>10s} {'θJA(风冷)':>10s} {'θJC':>8s} {'最大功耗@25°C':>14s}")
        print(f"  {'-'*60}")
        for name, data in PACKAGE_THERMAL.items():
            pmax = (125 - 25) / data['theta_ja'] * 1000
            print(f"  {name:<15s} {data['theta_ja']:>8.0f}°C/W {data['theta_ja_fan']:>8.0f}°C/W "
                  f"{data['theta_jc']:>6.0f}°C/W {pmax:>10.1f}mW")
        return

    # 列出散热器
    if args.list_heatsinks:
        print("\n散热器参考数据:")
        print(f"  {'型号':<15s} {'热阻(°C/W)':>12s} {'尺寸(mm)':>15s}")
        print(f"  {'-'*45}")
        for name, data in HEATSINK_SAMPLES.items():
            size = f"{data['size'][0]}×{data['size'][1]}×{data['size'][2]}"
            print(f"  {name:<15s} {data['theta_hs']:>10.1f}   {size:>15s}")
        return

    # 初始化
    if args.init:
        output = args.output if args.output != 'thermal_report.txt' else 'thermal_config.json'
        generate_sample_config(output)
        return

    # 快速计算
    if args.quick and args.package and args.power:
        comp = ThermalComponent(
            name="快速分析",
            package=args.package,
            power_mw=args.power,
            max_junction_temp=args.max_tj,
            heatsink=args.heatsink,
        )
        result = calculate_junction_temp(comp, args.ambient)
        if "error" in result:
            print(f"[错误] {result['error']}")
            sys.exit(1)

        print(f"\n■ 快速热分析: {args.package}")
        print(f"  功耗: {args.power}mW ({args.power/1000:.3f}W)")
        print(f"  环境温度: {args.ambient}°C")
        print(f"  总热阻: {result['theta_total']}°C/W")
        print(f"  结温: {result['junction_temp']}°C")
        print(f"  最大结温: {args.max_tj}°C")
        print(f"  温度余量: {result['margin']}°C ({result['margin_pct']}%)")
        print(f"  散热器: {result['heatsink']}")
        print(f"  状态: {result['status']}")

        if result['meets_derating']:
            print(f"  降额: ✓ 满足 (结温 ≤ {result['derating_temp']}°C)")
        else:
            print(f"  降额: ✗ 不满足 (结温 > {result['derating_temp']}°C)")
        return

    # 最大功耗计算
    if args.max_power and args.package:
        pmax = calculate_max_power(args.package, args.ambient, args.max_tj)
        if pmax:
            print(f"\n■ 最大允许功耗: {args.package}")
            print(f"  环境温度: {args.ambient}°C")
            print(f"  最大结温: {args.max_tj}°C")
            print(f"  最大功耗: {pmax}mW ({pmax/1000:.3f}W)")
        else:
            print(f"[错误] 未知封装: {args.package}")
        return

    # 铜皮面积估算
    if args.copper_area and args.power:
        area = estimate_pcb_copper_area(args.power / 1000, args.temp_raise)
        print(f"\n■ 铜皮面积估算")
        print(f"  功耗: {args.power}mW ({args.power/1000:.3f}W)")
        print(f"  允许温升: {args.temp_raise}°C")
        print(f"  所需铜皮面积: {area}cm²")
        return

    # 配置文件分析模式
    if args.config:
        with open(args.config, 'r', encoding='utf-8') as f:
            config = json.load(f)

        pcb_info = config.get('pcb', {})
        components = []
        for item in config.get('components', []):
            components.append(ThermalComponent(
                name=item['name'],
                package=item.get('package', 'SOIC-8'),
                power_mw=item.get('power_mw', 0),
                max_junction_temp=item.get('max_junction_temp', 125),
                heatsink=item.get('heatsink'),
                airflow=item.get('airflow', 'natural'),
            ))

        if not components:
            print("[错误] 配置文件中未找到元器件")
            sys.exit(1)

        # 热分析
        if args.coupling:
            results = analyze_thermal_coupling(components, args.ambient)
        else:
            results = [calculate_junction_temp(c, args.ambient) for c in components]

        # 生成报告
        report = format_report(results, pcb_info, args.ambient)
        print(report)

        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(report)
        print(f"\n[OK] 报告已保存: {args.output}")
        return

    parser.print_help()


if __name__ == '__main__':
    main()
