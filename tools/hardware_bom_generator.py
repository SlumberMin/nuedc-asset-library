#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================
物料清单生成器 (BOM Generator) - 根据系统方案自动生成 BOM 表
============================================================
功能：
  - 从 JSON 方案文件或命令行参数定义元器件清单
  - 自动分类（电阻、电容、IC、连接器、模块等）
  - 计算总成本、统计各类器件数量
  - 输出 CSV / Excel / Markdown 格式 BOM 表
  - 内置常用电赛元器件数据库

用法：
  python hardware_bom_generator.py --design design.json -o bom.csv
  python hardware_bom_generator.py --design design.json --format excel -o bom.xlsx
  python hardware_bom_generator.py --interactive
  python hardware_bom_generator.py --preset stm32_minimal -o bom.csv
============================================================
"""

import argparse
import csv
import json
import os
import sys
from datetime import datetime
from collections import defaultdict

# ── 常用电赛元器件数据库 ──────────────────────────────────
# 格式: {型号: {category, package, value, price, supplier, note}}
COMMON_COMPONENTS = {
    # ── 电阻 ──
    "10R":    {"category": "电阻", "package": "0603", "value": "10Ω",   "price": 0.01, "supplier": "立创", "note": "限流"},
    "100R":   {"category": "电阻", "package": "0603", "value": "100Ω",  "price": 0.01, "supplier": "立创", "note": ""},
    "330R":   {"category": "电阻", "package": "0603", "value": "330Ω",  "price": 0.01, "supplier": "立创", "note": "LED限流"},
    "1K":     {"category": "电阻", "package": "0603", "value": "1kΩ",   "price": 0.01, "supplier": "立创", "note": ""},
    "4.7K":   {"category": "电阻", "package": "0603", "value": "4.7kΩ", "price": 0.01, "supplier": "立创", "note": "I2C上拉"},
    "10K":    {"category": "电阻", "package": "0603", "value": "10kΩ",  "price": 0.01, "supplier": "立创", "note": "上拉/下拉"},
    "47K":    {"category": "电阻", "package": "0603", "value": "47kΩ",  "price": 0.01, "supplier": "立创", "note": ""},
    "100K":   {"category": "电阻", "package": "0603", "value": "100kΩ", "price": 0.01, "supplier": "立创", "note": ""},
    # ── 电容 ──
    "10pF":   {"category": "电容", "package": "0603", "value": "10pF",   "price": 0.02, "supplier": "立创", "note": "晶振匹配"},
    "22pF":   {"category": "电容", "package": "0603", "value": "22pF",   "price": 0.02, "supplier": "立创", "note": "晶振匹配"},
    "100nF":  {"category": "电容", "package": "0603", "value": "100nF",  "price": 0.02, "supplier": "立创", "note": "去耦电容"},
    "1uF":    {"category": "电容", "package": "0805", "value": "1μF",    "price": 0.03, "supplier": "立创", "note": "滤波"},
    "10uF":   {"category": "电容", "package": "0805", "value": "10μF",   "price": 0.05, "supplier": "立创", "note": "电源滤波"},
    "100uF":  {"category": "电容", "package": "直插",  "value": "100μF",  "price": 0.10, "supplier": "立创", "note": "电解电容"},
    "470uF":  {"category": "电容", "package": "直插",  "value": "470μF",  "price": 0.20, "supplier": "立创", "note": "电解电容"},
    # ── 二极管/LED ──
    "LED_R":  {"category": "LED", "package": "0603", "value": "红色LED",  "price": 0.03, "supplier": "立创", "note": "指示灯"},
    "LED_G":  {"category": "LED", "package": "0603", "value": "绿色LED",  "price": 0.03, "supplier": "立创", "note": "指示灯"},
    "1N4148": {"category": "二极管", "package": "SOD-323", "value": "开关二极管", "price": 0.03, "supplier": "立创", "note": ""},
    "SS34":   {"category": "二极管", "package": "SMA", "value": "肖特基二极管", "price": 0.10, "supplier": "立创", "note": "电源"},
    # ── IC ──
    "STM32F103C8T6": {"category": "MCU", "package": "LQFP48", "value": "STM32F103", "price": 8.00, "supplier": "立创", "note": "主控"},
    "STM32F407VET6": {"category": "MCU", "package": "LQFP100", "value": "STM32F407", "price": 25.00, "supplier": "立创", "note": "高性能主控"},
    "STM32G431":     {"category": "MCU", "package": "LQFP48", "value": "STM32G431", "price": 15.00, "supplier": "立创", "note": "电机控制"},
    "ESP32-S3":      {"category": "MCU", "package": "模组", "value": "ESP32-S3", "price": 18.00, "supplier": "立创", "note": "WiFi+BLE"},
    "LM358":   {"category": "运放", "package": "SOP8", "value": "双运放", "price": 0.50, "supplier": "立创", "note": "信号调理"},
    "OPA2134": {"category": "运放", "package": "SOP8", "value": "高精度运放", "price": 5.00, "supplier": "立创", "note": "音频/精密"},
    "NE5532":  {"category": "运放", "package": "SOP8", "value": "低噪声运放", "price": 1.00, "supplier": "立创", "note": "音频"},
    "LM311":   {"category": "比较器", "package": "SOP8", "value": "比较器", "price": 0.50, "supplier": "立创", "note": ""},
    "LM7805":  {"category": "电源", "package": "TO-220", "value": "5V稳压", "price": 0.50, "supplier": "立创", "note": "线性稳压"},
    "AMS1117-3.3": {"category": "电源", "package": "SOT-223", "value": "3.3V LDO", "price": 0.30, "supplier": "立创", "note": ""},
    "MP1584":  {"category": "电源", "package": "SOP8", "value": "DC-DC降压", "price": 2.00, "supplier": "立创", "note": "开关电源"},
    "TPS5430": {"category": "电源", "package": "SOP8", "value": "DC-DC降压", "price": 3.50, "supplier": "立创", "note": "大电流"},
    "IR2104":  {"category": "驱动", "package": "SOP8", "value": "半桥驱动", "price": 2.00, "supplier": "立创", "note": "电机驱动"},
    "L298N":   {"category": "驱动", "package": "Multiwatt15", "value": "双H桥", "price": 5.00, "supplier": "立创", "note": "电机驱动"},
    "ULN2003": {"category": "驱动", "package": "SOP16", "value": "达林顿阵列", "price": 0.80, "supplier": "立创", "note": "步进电机"},
    "74HC595": {"category": "逻辑", "package": "SOP16", "value": "移位寄存器", "price": 0.30, "supplier": "立创", "note": "IO扩展"},
    "CD4051":  {"category": "逻辑", "package": "SOP16", "value": "8通道MUX", "price": 0.50, "supplier": "立创", "note": "模拟开关"},
    # ── 传感器/模块 ──
    "OLED_0.96":   {"category": "显示", "package": "模块", "value": "0.96寸OLED", "price": 8.00, "supplier": "淘宝", "note": "SSD1306 I2C"},
    "LCD1602":     {"category": "显示", "package": "模块", "value": "1602液晶", "price": 6.00, "supplier": "淘宝", "note": ""},
    "TFT_2.4":     {"category": "显示", "package": "模块", "value": "2.4寸TFT", "price": 15.00, "supplier": "淘宝", "note": "ILI9341"},
    "ADS1115":     {"category": "ADC", "package": "模块", "value": "16位ADC", "price": 8.00, "supplier": "淘宝", "note": "I2C"},
    "MCP4725":     {"category": "DAC", "package": "模块", "value": "12位DAC", "price": 6.00, "supplier": "淘宝", "note": "I2C"},
    "DDS_AD9833":  {"category": "信号源", "package": "模块", "value": "DDS模块", "price": 12.00, "supplier": "淘宝", "note": "信号发生"},
    "HX711":       {"category": "ADC", "package": "模块", "value": "称重ADC", "price": 3.00, "supplier": "淘宝", "note": "电子秤"},
    # ── 连接器/接插件 ──
    "USB_TypeC":   {"category": "连接器", "package": "SMD", "value": "Type-C母座", "price": 0.50, "supplier": "立创", "note": ""},
    "DC_Jack":     {"category": "连接器", "package": "直插", "value": "DC电源座", "price": 0.30, "supplier": "立创", "note": ""},
    "Header_1x20": {"category": "连接器", "package": "直插", "value": "1x20排针", "price": 0.20, "supplier": "立创", "note": ""},
    "Header_2x20": {"category": "连接器", "package": "直插", "value": "2x20排母", "price": 0.40, "supplier": "立创", "note": "树莓派接口"},
    "BNC":         {"category": "连接器", "package": "直插", "value": "BNC母座", "price": 1.00, "supplier": "淘宝", "note": "测试接口"},
    "SMA":         {"category": "连接器", "package": "SMD", "value": "SMA母座", "price": 1.50, "supplier": "淘宝", "note": "射频接口"},
    # ── 晶振 ──
    "8MHz":    {"category": "晶振", "package": "HC-49S", "value": "8MHz晶振", "price": 0.30, "supplier": "立创", "note": "STM32主晶振"},
    "32.768K": {"category": "晶振", "package": "2012", "value": "32.768kHz", "price": 0.30, "supplier": "立创", "note": "RTC"},
    # ── 其他 ──
    "8MHz_SMD":    {"category": "晶振", "package": "3215", "value": "8MHz贴片晶振", "price": 0.50, "supplier": "立创", "note": ""},
    "Button":      {"category": "开关", "package": "直插", "value": "轻触按键", "price": 0.05, "supplier": "立创", "note": ""},
    "Buzzer":      {"category": "蜂鸣器", "package": "直插", "value": "有源蜂鸣器", "price": 0.50, "supplier": "立创", "note": ""},
    "Relay_5V":    {"category": "继电器", "package": "直插", "value": "5V继电器", "price": 1.50, "supplier": "淘宝", "note": ""},
    "PCB":         {"category": "PCB", "package": "-", "value": "PCB打样", "price": 20.00, "supplier": "嘉立创", "note": "5片"},
}

# ── 预设方案 ──────────────────────────────────────────────
PRESETS = {
    "stm32_minimal": {
        "name": "STM32最小系统",
        "description": "STM32F103C8T6 最小系统板",
        "components": [
            {"part": "STM32F103C8T6", "qty": 1, "ref": "U1"},
            {"part": "8MHz", "qty": 1, "ref": "Y1"},
            {"part": "32.768K", "qty": 1, "ref": "Y2"},
            {"part": "10pF", "qty": 2, "ref": "C1,C2", "note": "主晶振匹配"},
            {"part": "22pF", "qty": 2, "ref": "C3,C4", "note": "RTC晶振匹配"},
            {"part": "100nF", "qty": 8, "ref": "C5-C12", "note": "去耦电容"},
            {"part": "10uF", "qty": 2, "ref": "C13,C14", "note": "电源滤波"},
            {"part": "10K", "qty": 3, "ref": "R1-R3", "note": "上拉/下拉"},
            {"part": "AMS1117-3.3", "qty": 1, "ref": "U2"},
            {"part": "LED_G", "qty": 1, "ref": "D1"},
            {"part": "330R", "qty": 1, "ref": "R4", "note": "LED限流"},
            {"part": "Button", "qty": 2, "ref": "SW1,SW2", "note": "复位+用户"},
            {"part": "USB_TypeC", "qty": 1, "ref": "J1"},
            {"part": "Header_1x20", "qty": 2, "ref": "P1,P2"},
            {"part": "PCB", "qty": 1, "ref": "PCB1"},
        ]
    },
    "opamp_signal_chain": {
        "name": "运放信号调理链",
        "description": "常用信号调理电路：放大+滤波+比较",
        "components": [
            {"part": "LM358", "qty": 2, "ref": "U1,U2"},
            {"part": "LM311", "qty": 1, "ref": "U3"},
            {"part": "100nF", "qty": 4, "ref": "C1-C4"},
            {"part": "10K", "qty": 6, "ref": "R1-R6"},
            {"part": "100K", "qty": 4, "ref": "R7-R10"},
            {"part": "1K", "qty": 2, "ref": "R11,R12"},
            {"part": "Header_1x20", "qty": 1, "ref": "P1"},
        ]
    },
    "motor_driver": {
        "name": "电机驱动方案",
        "description": "基于IR2104的半桥电机驱动",
        "components": [
            {"part": "IR2104", "qty": 2, "ref": "U1,U2"},
            {"part": "SS34", "qty": 4, "ref": "D1-D4"},
            {"part": "100nF", "qty": 4, "ref": "C1-C4"},
            {"part": "10uF", "qty": 2, "ref": "C5,C6"},
            {"part": "10R", "qty": 2, "ref": "R1,R2"},
            {"part": "10K", "qty": 4, "ref": "R3-R6"},
            {"part": "Header_1x20", "qty": 1, "ref": "P1"},
        ]
    },
}


def classify_component(part: str) -> str:
    """根据器件型号返回分类。"""
    info = COMMON_COMPONENTS.get(part, {})
    return info.get('category', '其他')


def load_design(filepath: str) -> dict:
    """
    加载设计文件（JSON）。
    格式：
    {
      "name": "设计方案名称",
      "description": "描述",
      "components": [
        {"part": "STM32F103C8T6", "qty": 1, "ref": "U1", "note": "主控"},
        {"part": "100nF", "qty": 10, "ref": "C1-C10", "note": "去耦"},
        ...
      ]
    }
    """
    with open(filepath, 'r', encoding='utf-8') as f:
        return json.load(f)


def build_bom(design: dict) -> list[dict]:
    """
    根据设计方案构建完整 BOM。
    自动从内置数据库补充封装、价格等信息。
    """
    bom_rows = []
    for item in design.get('components', []):
        part = item.get('part', '')
        db_info = COMMON_COMPONENTS.get(part, {})

        row = {
            '序号': len(bom_rows) + 1,
            '位号': item.get('ref', ''),
            '型号/规格': part,
            '值': db_info.get('value', item.get('value', '')),
            '封装': db_info.get('package', item.get('package', '')),
            '数量': item.get('qty', 1),
            '单价(元)': db_info.get('price', item.get('price', 0)),
            '小计(元)': 0,
            '分类': db_info.get('category', item.get('category', '其他')),
            '供应商': db_info.get('supplier', item.get('supplier', '')),
            '备注': item.get('note', db_info.get('note', '')),
        }
        row['小计(元)'] = round(row['数量'] * row['单价(元)'], 2)
        bom_rows.append(row)

    return bom_rows


def print_bom_summary(bom: list[dict], design_name: str = ""):
    """打印 BOM 摘要到终端。"""
    print("\n" + "=" * 70)
    print(f"  物料清单 (BOM) {'- ' + design_name if design_name else ''}")
    print(f"  生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)

    # 按分类汇总
    categories = defaultdict(lambda: {'count': 0, 'cost': 0.0, 'items': 0})
    total_cost = 0
    total_count = 0

    for row in bom:
        cat = row['分类']
        categories[cat]['count'] += row['数量']
        categories[cat]['cost'] += row['小计(元)']
        categories[cat]['items'] += 1
        total_cost += row['小计(元)']
        total_count += row['数量']

    # 打印表格
    print(f"\n{'序号':>4} {'位号':<12} {'型号':<20} {'封装':<12} {'数量':>4} {'单价':>8} {'小计':>8} {'分类':<8}")
    print("-" * 90)
    for row in bom:
        print(f"{row['序号']:>4} {row['位号']:<12} {row['型号/规格']:<20} {row['封装']:<12} "
              f"{row['数量']:>4} {row['单价(元)']:>8.2f} {row['小计(元)']:>8.2f} {row['分类']:<8}")

    print("-" * 90)
    print(f"{'合计':>4} {'':12} {'':20} {'':12} {total_count:>4} {'':>8} {total_cost:>8.2f}")

    # 分类统计
    print(f"\n【分类统计】")
    for cat, info in sorted(categories.items()):
        print(f"  {cat:<10}: {info['items']} 种器件, {info['count']} 个, 小计 {info['cost']:.2f} 元")

    print(f"\n  总计: {len(bom)} 种物料, {total_count} 个器件, 预估成本 {total_cost:.2f} 元")
    print("=" * 70)


def export_csv(bom: list[dict], filepath: str):
    """导出 CSV 格式 BOM。"""
    if not bom:
        return
    headers = list(bom[0].keys())
    with open(filepath, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        writer.writerows(bom)
    print(f"[OK] CSV BOM 已保存: {filepath}")


def export_markdown(bom: list[dict], filepath: str):
    """导出 Markdown 格式 BOM。"""
    if not bom:
        return
    headers = list(bom[0].keys())
    lines = []
    lines.append("# 物料清单 (BOM)\n")
    lines.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    lines.append("| " + " | ".join(headers) + " |")
    lines.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for row in bom:
        lines.append("| " + " | ".join(str(row[h]) for h in headers) + " |")
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write("\n".join(lines))
    print(f"[OK] Markdown BOM 已保存: {filepath}")


def export_json(bom: list[dict], filepath: str):
    """导出 JSON 格式 BOM。"""
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(bom, f, ensure_ascii=False, indent=2)
    print(f"[OK] JSON BOM 已保存: {filepath}")


def interactive_mode():
    """交互模式：手动添加元器件。"""
    print("\n" + "=" * 50)
    print("  物料清单生成器 - 交互模式")
    print("=" * 50)
    print("输入器件信息，输入 'done' 结束\n")

    components = []
    while True:
        print(f"--- 器件 #{len(components) + 1} ---")
        part = input("型号/名称 (或 'done' 结束): ").strip()
        if part.lower() == 'done':
            break

        if part in COMMON_COMPONENTS:
            info = COMMON_COMPONENTS[part]
            print(f"  [已知器件] {info['value']} | 封装: {info['package']} | 单价: {info['price']}元")

        ref = input("位号 (如 R1,C2,U3): ").strip()
        try:
            qty = int(input("数量 [1]: ").strip() or "1")
        except ValueError:
            qty = 1
        note = input("备注: ").strip()

        components.append({"part": part, "qty": qty, "ref": ref, "note": note})
        print()

    if not components:
        print("[WARN] 未添加任何器件")
        return None

    return {"name": "手动输入", "description": "交互模式创建", "components": components}


def list_presets():
    """列出所有预设方案。"""
    print("\n可用预设方案:")
    print("-" * 50)
    for key, preset in PRESETS.items():
        print(f"  {key:<25} - {preset['name']}: {preset['description']}")
    print()


def list_components():
    """列出内置元器件数据库。"""
    print("\n内置元器件数据库:")
    print("-" * 80)
    print(f"{'型号':<20} {'分类':<8} {'封装':<12} {'值':<16} {'单价':>8}")
    print("-" * 80)
    for part, info in sorted(COMMON_COMPONENTS.items()):
        print(f"{part:<20} {info['category']:<8} {info['package']:<12} {info['value']:<16} {info['price']:>8.2f}")
    print(f"\n共 {len(COMMON_COMPONENTS)} 种器件\n")


def main():
    parser = argparse.ArgumentParser(
        description='物料清单生成器 - 自动生成 BOM 表',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python hardware_bom_generator.py --design design.json -o bom.csv
  python hardware_bom_generator.py --preset stm32_minimal -o bom.csv
  python hardware_bom_generator.py --preset stm32_minimal --format markdown -o bom.md
  python hardware_bom_generator.py --interactive -o bom.csv
  python hardware_bom_generator.py --list-presets
  python hardware_bom_generator.py --list-components
        """
    )
    parser.add_argument('--design', '-d', help='设计方案 JSON 文件')
    parser.add_argument('--preset', '-p', help='使用预设方案')
    parser.add_argument('--interactive', action='store_true', help='交互模式手动添加器件')
    parser.add_argument('--output', '-o', default='bom.csv', help='输出文件路径 (默认: bom.csv)')
    parser.add_argument('--format', '-f', choices=['csv', 'json', 'markdown'], default=None,
                        help='输出格式 (默认根据扩展名自动判断)')
    parser.add_argument('--list-presets', action='store_true', help='列出所有预设方案')
    parser.add_argument('--list-components', action='store_true', help='列出内置元器件数据库')

    args = parser.parse_args()

    # 列出功能
    if args.list_presets:
        list_presets()
        return
    if args.list_components:
        list_components()
        return

    # 获取设计方案
    design = None
    if args.interactive:
        design = interactive_mode()
    elif args.preset:
        if args.preset not in PRESETS:
            print(f"[ERROR] 未知预设: {args.preset}")
            list_presets()
            sys.exit(1)
        design = PRESETS[args.preset]
        print(f"[INFO] 使用预设方案: {design['name']}")
    elif args.design:
        if not os.path.isfile(args.design):
            print(f"[ERROR] 设计文件不存在: {args.design}")
            sys.exit(1)
        design = load_design(args.design)
    else:
        print("[ERROR] 请指定 --design, --preset 或 --interactive")
        parser.print_help()
        sys.exit(1)

    if not design:
        print("[ERROR] 无设计方案数据")
        sys.exit(1)

    # 生成 BOM
    bom = build_bom(design)
    if not bom:
        print("[ERROR] BOM 为空")
        sys.exit(1)

    # 打印摘要
    print_bom_summary(bom, design.get('name', ''))

    # 确定输出格式
    fmt = args.format
    if fmt is None:
        ext = os.path.splitext(args.output)[1].lower()
        fmt = {'.csv': 'csv', '.json': 'json', '.md': 'markdown', '.markdown': 'markdown'}.get(ext, 'csv')

    # 导出
    if fmt == 'csv':
        export_csv(bom, args.output)
    elif fmt == 'json':
        export_json(bom, args.output)
    elif fmt == 'markdown':
        export_markdown(bom, args.output)


if __name__ == '__main__':
    main()
