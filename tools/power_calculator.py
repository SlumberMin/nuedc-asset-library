#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功率计算器 - 电池容量/续航时间/功耗预算
========================================
功能：
  - 电池容量与续航时间计算
  - 系统功耗预算分析
  - 多工作模式功耗建模（活跃/待机/睡眠）
  - 充电时间估算
  - 功耗优化建议
"""

import argparse
import json
import sys
from datetime import datetime

# ============================================================
# 常量定义
# ============================================================

# 电池类型特性
BATTERY_TYPES = {
    "li-ion": {
        "name": "锂离子电池",
        "nominal_voltage": 3.7,      # 标称电压(V)
        "max_voltage": 4.2,          # 满充电压(V)
        "min_voltage": 3.0,          # 放电截止电压(V)
        "discharge_efficiency": 0.90, # 放电效率
        "self_discharge_rate": 0.003, # 自放电率(每月)
        "cycle_life": 500,           # 循环寿命
    },
    "li-po": {
        "name": "锂聚合物电池",
        "nominal_voltage": 3.7,
        "max_voltage": 4.2,
        "min_voltage": 3.0,
        "discharge_efficiency": 0.92,
        "self_discharge_rate": 0.005,
        "cycle_life": 300,
    },
    "li-fe": {
        "name": "磷酸铁锂电池",
        "nominal_voltage": 3.2,
        "max_voltage": 3.65,
        "min_voltage": 2.5,
        "discharge_efficiency": 0.95,
        "self_discharge_rate": 0.002,
        "cycle_life": 2000,
    },
    "nimh": {
        "name": "镍氢电池",
        "nominal_voltage": 1.2,
        "max_voltage": 1.4,
        "min_voltage": 0.9,
        "discharge_efficiency": 0.80,
        "self_discharge_rate": 0.03,
        "cycle_life": 500,
    },
    "lead-acid": {
        "name": "铅酸电池",
        "nominal_voltage": 2.0,
        "max_voltage": 2.4,
        "min_voltage": 1.75,
        "discharge_efficiency": 0.85,
        "self_discharge_rate": 0.05,
        "cycle_life": 300,
    },
}

# 常见模块功耗参考值 (mA @ 3.3V)
MODULE_POWER_REF = {
    "STM32F407_168MHz": {"active": 100, "sleep": 0.5, "standby": 0.002},
    "STM32F103_72MHz":  {"active": 50,  "sleep": 0.3, "standby": 0.001},
    "STM32L476_80MHz":  {"active": 30,  "sleep": 0.03, "standby": 0.0004},
    "ESP32_WiFi":       {"active": 240, "sleep": 0.15, "standby": 0.005},
    "ESP32_BT":         {"active": 130, "sleep": 0.15, "standby": 0.005},
    "NRF24L01_TX":      {"active": 11,  "sleep": 0.009, "standby": 0.009},
    "OLED_128x64":      {"active": 20,  "sleep": 0,    "standby": 0},
    "TFT_2.4inch":      {"active": 40,  "sleep": 0,    "standby": 0},
    "GPS_NEO6M":        {"active": 40,  "sleep": 0.05, "standby": 0.05},
    "IMU_MPU6050":      {"active": 3.5, "sleep": 0.005, "standby": 0.005},
    "ADC_12bit":        {"active": 1,   "sleep": 0,    "standby": 0},
    "LDO_static":       {"active": 0.05, "sleep": 0.05, "standby": 0.05},
    "DCDC_static":      {"active": 0.02, "sleep": 0.02, "standby": 0.02},
}


class PowerModule:
    """功耗模块"""

    def __init__(self, name, active_ma=0, sleep_ma=0, standby_ma=0,
                 voltage=3.3, duty_cycle=1.0, notes=""):
        self.name = name
        self.active_ma = active_ma     # 活跃模式电流(mA)
        self.sleep_ma = sleep_ma       # 睡眠模式电流(mA)
        self.standby_ma = standby_ma   # 待机模式电流(mA)
        self.voltage = voltage         # 工作电压(V)
        self.duty_cycle = duty_cycle   # 活跃占空比 (0~1)
        self.notes = notes

    @property
    def avg_current_ma(self):
        """平均电流(mA) = 活跃电流*占空比 + 睡眠电流*(1-占空比)"""
        return self.active_ma * self.duty_cycle + self.sleep_ma * (1 - self.duty_cycle)

    @property
    def avg_power_mw(self):
        """平均功耗(mW)"""
        return self.avg_current_ma * self.voltage

    def to_dict(self):
        return {
            "name": self.name,
            "active_ma": self.active_ma,
            "sleep_ma": self.sleep_ma,
            "standby_ma": self.standby_ma,
            "voltage": self.voltage,
            "duty_cycle": self.duty_cycle,
            "avg_current_ma": round(self.avg_current_ma, 3),
            "avg_power_mw": round(self.avg_power_mw, 3),
            "notes": self.notes,
        }


def load_modules_from_json(filepath):
    """从JSON文件加载功耗模块配置"""
    with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)
    modules = []
    for item in data.get('modules', data if isinstance(data, list) else []):
        modules.append(PowerModule(
            name=item.get('name', ''),
            active_ma=item.get('active_ma', 0),
            sleep_ma=item.get('sleep_ma', 0),
            standby_ma=item.get('standby_ma', 0),
            voltage=item.get('voltage', 3.3),
            duty_cycle=item.get('duty_cycle', 1.0),
            notes=item.get('notes', ''),
        ))
    return modules


def calculate_battery_stats(capacity_mah, voltage, battery_type="li-ion"):
    """
    计算电池基础参数
    返回: 电池信息字典
    """
    bt = BATTERY_TYPES.get(battery_type, BATTERY_TYPES["li-ion"])
    usable_capacity = capacity_mah * bt["discharge_efficiency"]
    energy_wh = capacity_mah * bt["nominal_voltage"] / 1000
    usable_energy_wh = usable_capacity * bt["nominal_voltage"] / 1000

    return {
        "type": battery_type,
        "name": bt["name"],
        "capacity_mah": capacity_mah,
        "nominal_voltage": bt["nominal_voltage"],
        "max_voltage": bt["max_voltage"],
        "min_voltage": bt["min_voltage"],
        "discharge_efficiency": bt["discharge_efficiency"],
        "usable_capacity_mah": round(usable_capacity, 2),
        "energy_wh": round(energy_wh, 2),
        "usable_energy_wh": round(usable_energy_wh, 2),
        "cycle_life": bt["cycle_life"],
    }


def calculate_runtime(capacity_mah, avg_current_ma, battery_type="li-ion",
                      num_cells=1, safety_margin=0.1):
    """
    计算续航时间
    参数:
      capacity_mah: 电池容量(mAh)
      avg_current_ma: 平均电流(mA)
      battery_type: 电池类型
      num_cells: 电池串联数
      safety_margin: 安全余量(0~1)
    返回: 续航时间字典
    """
    bt = BATTERY_TYPES.get(battery_type, BATTERY_TYPES["li-ion"])

    # 考虑安全余量后的有效容量
    effective_capacity = capacity_mah * (1 - safety_margin) * bt["discharge_efficiency"]
    effective_capacity_total = effective_capacity * num_cells

    # 续航时间计算
    if avg_current_ma <= 0:
        runtime_hours = float('inf')
    else:
        runtime_hours = effective_capacity_total / avg_current_ma

    # 转换为天/时/分
    days = int(runtime_hours // 24)
    hours = int(runtime_hours % 24)
    minutes = int((runtime_hours * 60) % 60)

    return {
        "effective_capacity_mah": round(effective_capacity_total, 2),
        "avg_current_ma": round(avg_current_ma, 3),
        "runtime_hours": round(runtime_hours, 2),
        "runtime_days": round(runtime_hours / 24, 2),
        "display": f"{days}天{hours}时{minutes}分",
        "safety_margin": safety_margin,
        "num_cells": num_cells,
    }


def calculate_power_budget(modules, system_voltage=3.3):
    """
    计算功耗预算
    返回: 功耗预算分析
    """
    total_active_ma = 0
    total_sleep_ma = 0
    total_standby_ma = 0
    module_details = []

    for mod in modules:
        total_active_ma += mod.active_ma
        total_sleep_ma += mod.sleep_ma
        total_standby_ma += mod.standby_ma
        module_details.append(mod.to_dict())

    # 按平均功耗排序（降序）
    module_details.sort(key=lambda x: x['avg_power_mw'], reverse=True)

    return {
        "system_voltage": system_voltage,
        "total_active_ma": round(total_active_ma, 3),
        "total_sleep_ma": round(total_sleep_ma, 3),
        "total_standby_ma": round(total_standby_ma, 3),
        "total_active_mw": round(total_active_ma * system_voltage, 3),
        "total_sleep_mw": round(total_sleep_ma * system_voltage, 3),
        "total_standby_mw": round(total_standby_ma * system_voltage, 3),
        "modules": module_details,
    }


def calculate_charging_time(capacity_mah, charge_current_ma, efficiency=0.85):
    """
    计算充电时间
    参数:
      capacity_mah: 电池容量(mAh)
      charge_current_ma: 充电电流(mA)
      efficiency: 充电效率
    返回: 充电时间信息
    """
    # CC-CV充电模型: CC阶段约70%容量，CV阶段约30%
    cc_capacity = capacity_mah * 0.7
    cv_capacity = capacity_mah * 0.3

    cc_hours = cc_capacity / (charge_current_ma * efficiency)
    cv_hours = cv_capacity / (charge_current_ma * efficiency * 0.5)  # CV阶段电流减半

    total_hours = cc_hours + cv_hours
    days = int(total_hours // 24)
    hours = int(total_hours % 24)
    minutes = int((total_hours * 60) % 60)

    return {
        "capacity_mah": capacity_mah,
        "charge_current_ma": charge_current_ma,
        "efficiency": efficiency,
        "cc_phase_hours": round(cc_hours, 2),
        "cv_phase_hours": round(cv_hours, 2),
        "total_hours": round(total_hours, 2),
        "display": f"{days}天{hours}时{minutes}分",
    }


def estimate_solar_requirements(avg_power_mw, sun_hours=4.0, charge_efficiency=0.75):
    """
    估算太阳能板需求
    参数:
      avg_power_mw: 平均功耗(mW)
      sun_hours: 日均有效日照时长(小时)
      charge_efficiency: 充电效率
    """
    daily_energy_mwh = avg_power_mw * 24
    required_panel_mw = daily_energy_mwh / (sun_hours * charge_efficiency)
    required_panel_w = required_panel_mw / 1000

    return {
        "avg_power_mw": avg_power_mw,
        "daily_energy_mwh": round(daily_energy_mwh, 2),
        "sun_hours": sun_hours,
        "required_panel_mw": round(required_panel_mw, 2),
        "required_panel_w": round(required_panel_w, 3),
    }


def generate_optimization_tips(modules, budget):
    """生成功耗优化建议"""
    tips = []
    sorted_modules = sorted(modules, key=lambda m: m.avg_power_mw, reverse=True)

    # 找出功耗最大的模块
    if sorted_modules:
        top = sorted_modules[0]
        if top.avg_power_mw > budget['total_active_mw'] * 0.3:
            tips.append(f"[高优先] {top.name} 占总功耗 {top.avg_power_mw/budget['total_active_mw']*100:.1f}%，"
                       f"建议优化其占空比或使用低功耗方案")

    # 占空比优化
    for mod in sorted_modules:
        if mod.duty_cycle > 0.8 and mod.active_ma > 10:
            tips.append(f"[占空比] {top.name} 占空比 {mod.duty_cycle*100:.0f}% 较高，"
                       f"考虑间歇工作降低至 {mod.duty_cycle*50:.0f}%")

    # 睡眠模式
    sleep_total = sum(m.sleep_ma for m in modules)
    if sleep_total > 1:
        tips.append(f"[睡眠] 系统睡眠电流 {sleep_total:.2f}mA，"
                   f"考虑使用MOS管切断不必要模块供电")

    # 电压域优化
    for mod in modules:
        if mod.voltage > 3.3 and mod.active_ma > 20:
            tips.append(f"[电压] {mod.name} 工作电压 {mod.voltage}V，"
                       f"考虑使用DC-DC降压减少损耗")

    # 通用建议
    tips.append("[通用] 使用MCU的低功耗模式(Stop/Standby)替代忙等待")
    tips.append("[通用] 传感器采样使用中断驱动而非轮询")
    tips.append("[通用] 通信模块使用休眠唤醒机制")

    return tips


def format_report(battery_stats, runtime, budget, charging=None, solar=None, tips=None):
    """格式化功耗分析报告"""
    lines = []
    lines.append("=" * 60)
    lines.append("          电赛功耗分析报告")
    lines.append(f"          生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("=" * 60)

    # 电池信息
    lines.append("\n■ 电池参数")
    lines.append(f"  类型: {battery_stats['name']}")
    lines.append(f"  容量: {battery_stats['capacity_mah']}mAh ({battery_stats['energy_wh']}Wh)")
    lines.append(f"  有效容量: {battery_stats['usable_capacity_mah']}mAh")
    lines.append(f"  标称电压: {battery_stats['nominal_voltage']}V")
    lines.append(f"  充放范围: {battery_stats['min_voltage']}V ~ {battery_stats['max_voltage']}V")

    # 续航时间
    lines.append(f"\n■ 续航预估 (安全余量: {runtime['safety_margin']*100:.0f}%)")
    lines.append(f"  平均电流: {runtime['avg_current_ma']}mA")
    lines.append(f"  有效容量: {runtime['effective_capacity_mah']}mAh")
    lines.append(f"  续航时间: {runtime['display']} ({runtime['runtime_hours']}小时)")

    # 功耗预算
    lines.append(f"\n■ 功耗预算 (@ {budget['system_voltage']}V)")
    lines.append(f"  {'模块':<20s} {'活跃(mA)':>10s} {'睡眠(mA)':>10s} {'均值(mW)':>10s} {'占比':>8s}")
    lines.append(f"  {'-'*60}")
    for mod in budget['modules']:
        pct = mod['avg_power_mw'] / budget['total_active_mw'] * 100 if budget['total_active_mw'] > 0 else 0
        lines.append(f"  {mod['name']:<20s} {mod['active_ma']:>10.2f} {mod['sleep_ma']:>10.3f} "
                    f"{mod['avg_power_mw']:>10.2f} {pct:>7.1f}%")
    lines.append(f"  {'-'*60}")
    lines.append(f"  {'合计':<20s} {budget['total_active_ma']:>10.2f} {budget['total_sleep_ma']:>10.3f} "
                f"{budget['total_active_mw']:>10.2f} {'100%':>8s}")
    lines.append(f"  睡眠模式功耗: {budget['total_sleep_mw']:.3f}mW")
    lines.append(f"  待机模式功耗: {budget['total_standby_mw']:.3f}mW")

    # 充电时间
    if charging:
        lines.append(f"\n■ 充电估算")
        lines.append(f"  充电电流: {charging['charge_current_ma']}mA")
        lines.append(f"  CC阶段: {charging['cc_phase_hours']}h")
        lines.append(f"  CV阶段: {charging['cv_phase_hours']}h")
        lines.append(f"  总充电时间: {charging['display']} ({charging['total_hours']}h)")

    # 太阳能
    if solar:
        lines.append(f"\n■ 太阳能需求")
        lines.append(f"  日均日照: {solar['sun_hours']}h")
        lines.append(f"  日耗能: {solar['daily_energy_mwh']}mWh")
        lines.append(f"  需要太阳能板: ≥{solar['required_panel_w']}W ({solar['required_panel_mw']}mW)")

    # 优化建议
    if tips:
        lines.append(f"\n■ 优化建议")
        for tip in tips:
            lines.append(f"  {tip}")

    lines.append("\n" + "=" * 60)
    return '\n'.join(lines)


def create_sample_config(output_path):
    """生成示例功耗配置文件"""
    config = {
        "project": "示例电赛项目",
        "system_voltage": 3.3,
        "battery": {
            "type": "li-po",
            "capacity_mah": 2000,
            "cells": 1,
        },
        "modules": [
            {"name": "STM32F407主控", "active_ma": 80, "sleep_ma": 0.5, "standby_ma": 0.002,
             "voltage": 3.3, "duty_cycle": 0.6, "notes": "168MHz满速运行"},
            {"name": "OLED显示屏", "active_ma": 20, "sleep_ma": 0, "standby_ma": 0,
             "voltage": 3.3, "duty_cycle": 0.3, "notes": "间歇刷新"},
            {"name": "WiFi模块", "active_ma": 170, "sleep_ma": 0.5, "standby_ma": 0.01,
             "voltage": 3.3, "duty_cycle": 0.1, "notes": "仅发送数据时唤醒"},
            {"name": "传感器组", "active_ma": 15, "sleep_ma": 0.01, "standby_ma": 0.001,
             "voltage": 3.3, "duty_cycle": 0.05, "notes": "1秒采样一次"},
            {"name": "电机驱动", "active_ma": 500, "sleep_ma": 0.1, "standby_ma": 0.1,
             "voltage": 12.0, "duty_cycle": 0.4, "notes": "竞速电机"},
            {"name": "LDO静态", "active_ma": 0.05, "sleep_ma": 0.05, "standby_ma": 0.05,
             "voltage": 3.3, "duty_cycle": 1.0, "notes": "稳压器静态电流"},
        ],
    }
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(config, f, ensure_ascii=False, indent=2)
    print(f"[OK] 示例配置已生成: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='功率计算器 - 电池容量/续航时间/功耗预算分析',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 生成示例配置
  python power_calculator.py --init --output power_config.json

  # 从配置文件计算
  python power_calculator.py --config power_config.json

  # 快速计算续航时间
  python power_calculator.py --capacity 2000 --current 150 --battery li-po

  # 计算充电时间
  python power_calculator.py --charge --capacity 2000 --charge-current 500

  # 列出支持的电池类型
  python power_calculator.py --list-batteries

  # 查看参考功耗
  python power_calculator.py --list-modules
        """
    )

    # 基础参数
    parser.add_argument('--config', '-c', help='功耗配置文件 (JSON)')
    parser.add_argument('--output', '-o', help='输出文件路径', default='power_report.txt')
    parser.add_argument('--init', action='store_true', help='生成示例配置文件')
    parser.add_argument('--list-batteries', action='store_true', help='列出支持的电池类型')
    parser.add_argument('--list-modules', action='store_true', help='列出参考模块功耗')

    # 快速计算参数
    parser.add_argument('--capacity', type=float, help='电池容量(mAh)')
    parser.add_argument('--current', type=float, help='平均电流(mA)')
    parser.add_argument('--battery', default='li-ion', choices=list(BATTERY_TYPES.keys()),
                        help='电池类型 (默认: li-ion)')
    parser.add_argument('--cells', type=int, default=1, help='串联电池数 (默认: 1)')
    parser.add_argument('--margin', type=float, default=0.1, help='安全余量 (默认: 0.1)')

    # 充电计算
    parser.add_argument('--charge', action='store_true', help='计算充电时间')
    parser.add_argument('--charge-current', type=float, help='充电电流(mA)')
    parser.add_argument('--charge-eff', type=float, default=0.85, help='充电效率 (默认: 0.85)')

    # 太阳能估算
    parser.add_argument('--solar', action='store_true', help='估算太阳能板需求')
    parser.add_argument('--sun-hours', type=float, default=4.0, help='日均有效日照(小时)')

    args = parser.parse_args()

    # 列出电池类型
    if args.list_batteries:
        print("\n支持的电池类型:")
        print(f"  {'类型':<12s} {'名称':<15s} {'标称电压':>8s} {'放电范围':<15s} {'效率':>6s} {'循环':>6s}")
        print(f"  {'-'*65}")
        for key, bt in BATTERY_TYPES.items():
            print(f"  {key:<12s} {bt['name']:<15s} {bt['nominal_voltage']:>7.1f}V "
                  f"{bt['min_voltage']}-{bt['max_voltage']}V "
                  f"{bt['discharge_efficiency']*100:>5.0f}% {bt['cycle_life']:>5d}")
        return

    # 列出参考模块
    if args.list_modules:
        print("\n常见模块功耗参考值 (@3.3V):")
        print(f"  {'模块':<25s} {'活跃(mA)':>10s} {'睡眠(mA)':>10s} {'待机(mA)':>10s}")
        print(f"  {'-'*57}")
        for name, power in MODULE_POWER_REF.items():
            print(f"  {name:<25s} {power['active']:>10.3f} {power['sleep']:>10.3f} "
                  f"{power['standby']:>10.4f}")
        return

    # 初始化模式
    if args.init:
        output = args.output if args.output != 'power_report.txt' else 'power_config.json'
        create_sample_config(output)
        return

    # 快速计算模式
    if args.capacity and args.current:
        battery_stats = calculate_battery_stats(args.capacity, 0, args.battery)
        runtime = calculate_runtime(args.capacity, args.current, args.battery,
                                     args.cells, args.margin)

        print(f"\n■ 快速续航估算")
        print(f"  电池: {battery_stats['name']} {args.capacity}mAh")
        print(f"  平均电流: {args.current}mA")
        print(f"  续航时间: {runtime['display']} ({runtime['runtime_hours']}小时)")

        if args.charge and args.charge_current:
            charging = calculate_charging_time(args.capacity, args.charge_current, args.charge_eff)
            print(f"\n■ 充电时间")
            print(f"  充电电流: {args.charge_current}mA")
            print(f"  总充电时间: {charging['display']} ({charging['total_hours']}h)")

        if args.solar:
            avg_power = args.current * 3.3
            solar = estimate_solar_requirements(avg_power, args.sun_hours)
            print(f"\n■ 太阳能需求")
            print(f"  需要太阳能板: ≥{solar['required_panel_w']}W")

        return

    # 配置文件模式
    if args.config:
        config_data = json.load(open(args.config, 'r', encoding='utf-8'))
        system_voltage = config_data.get('system_voltage', 3.3)
        battery_cfg = config_data.get('battery', {})

        # 加载模块
        modules = []
        for item in config_data.get('modules', []):
            modules.append(PowerModule(
                name=item['name'],
                active_ma=item.get('active_ma', 0),
                sleep_ma=item.get('sleep_ma', 0),
                standby_ma=item.get('standby_ma', 0),
                voltage=item.get('voltage', 3.3),
                duty_cycle=item.get('duty_cycle', 1.0),
                notes=item.get('notes', ''),
            ))

        if not modules:
            print("[错误] 配置文件中未找到功耗模块")
            sys.exit(1)

        # 计算
        battery_stats = calculate_battery_stats(
            battery_cfg.get('capacity_mah', 2000),
            battery_cfg.get('voltage', 3.7),
            battery_cfg.get('type', 'li-ion'),
        )

        budget = calculate_power_budget(modules, system_voltage)
        avg_current = budget['total_active_ma'] * 0.5 + budget['total_sleep_ma'] * 0.5  # 简化估算

        # 从模块实际计算平均电流
        total_avg_ma = sum(m.avg_current_ma for m in modules)

        runtime = calculate_runtime(
            battery_cfg.get('capacity_mah', 2000),
            total_avg_ma,
            battery_cfg.get('type', 'li-ion'),
            battery_cfg.get('cells', 1),
            args.margin,
        )

        charging = None
        if args.charge_current:
            charging = calculate_charging_time(
                battery_cfg.get('capacity_mah', 2000),
                args.charge_current, args.charge_eff)

        solar = None
        if args.solar:
            solar = estimate_solar_requirements(budget['total_active_mw'], args.sun_hours)

        tips = generate_optimization_tips(modules, budget)

        # 生成报告
        report = format_report(battery_stats, runtime, budget, charging, solar, tips)
        print(report)

        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(report)
        print(f"\n[OK] 报告已保存: {args.output}")
        return

    # 无参数时显示帮助
    parser.print_help()


if __name__ == '__main__':
    main()
