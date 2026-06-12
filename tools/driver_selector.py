#!/usr/bin/env python3
"""驱动选型工具 - 根据负载类型和参数推荐驱动IC

用法:
    python driver_selector.py --load-type motor --motor-type dc --voltage 12 --current 2
    python driver_selector.py --load-type led --current 0.5 --string 3
    python driver_selector.py --list-types
    python driver_selector.py --interactive
"""

import argparse
import json
import sys

# ============================================================
# 驱动IC数据库
# ============================================================
DRIVER_DATABASE = [
    # ======== 电机驱动 ========
    {
        "型号": "L298N", "类别": "电机驱动", "子类": "直流电机",
        "厂商": "ST",
        "描述": "双H桥驱动，可驱动2个直流电机或1个步进电机",
        "供电电压V": [5, 46], "逻辑电压V": [5, 5],
        "持续电流A": 2.0, "峰值电流A": 3.0,
        "PWM频率kHz": 25, "接口": "GPIO+PWM",
        "封装": "Multiwatt15", "单价约": 5.0, "供货": "充足",
        "特点": "经典双H桥，入门首选，压降较大(约2V)"
    },
    {
        "型号": "L293D", "类别": "电机驱动", "子类": "直流电机",
        "厂商": "TI",
        "描述": "四通道半H桥，可驱动2个直流电机",
        "供电电压V": [4.5, 36], "逻辑电压V": [5, 5],
        "持续电流A": 0.6, "峰值电流A": 1.2,
        "PWM频率kHz": 5, "接口": "GPIO+PWM",
        "封装": "DIP16/SOIC16", "单价约": 3.0, "供货": "充足",
        "特点": "小电流电机，DIP封装易焊接"
    },
    {
        "型号": "TB6612FNG", "类别": "电机驱动", "子类": "直流电机",
        "厂商": "东芝",
        "描述": "双H桥驱动，MOS管方案效率高",
        "供电电压V": [2.5, 13.5], "逻辑电压V": [2.7, 5.5],
        "持续电流A": 1.2, "峰值电流A": 3.2,
        "PWM频率kHz": 100, "接口": "GPIO+PWM",
        "封装": "SSOP24", "单价约": 4.0, "供货": "充足",
        "特点": "效率高，压降小(0.5V)，推荐替代L298N"
    },
    {
        "型号": "DRV8833", "类别": "电机驱动", "子类": "直流电机",
        "厂商": "TI",
        "描述": "双H桥低电压驱动",
        "供电电压V": [2.7, 10.8], "逻辑电压V": [2.7, 5.5],
        "持续电流A": 1.5, "峰值电流A": 2.0,
        "PWM频率kHz": 250, "接口": "PWM",
        "封装": "HTSSOP16", "单价约": 6.0, "供货": "充足",
        "特点": "低电压小电机，带过流保护"
    },
    {
        "型号": "BTN7960B", "类别": "电机驱动", "子类": "大功率直流电机",
        "厂商": "Infineon",
        "描述": "单路大电流半桥驱动",
        "供电电压V": [5.5, 40], "逻辑电压V": [5, 5],
        "持续电流A": 43.0, "峰值电流A": 70.0,
        "PWM频率kHz": 25, "接口": "PWM+方向",
        "封装": "TO-220-7", "单价约": 12.0, "供货": "充足",
        "特点": "大电流，电赛常用大电机驱动"
    },
    {
        "型号": "VNH5019A", "类别": "电机驱动", "子类": "大功率直流电机",
        "厂商": "ST",
        "描述": "全桥大电流驱动，集成保护",
        "供电电压V": [5.5, 36], "逻辑电压V": [3.3, 5],
        "持续电流A": 30.0, "峰值电流A": 120.0,
        "PWM频率kHz": 20, "接口": "PWM+方向",
        "封装": "SSOP36", "单价约": 25.0, "供货": "一般",
        "特点": "全集成大电流，带诊断保护"
    },
    # ======== 步进电机驱动 ========
    {
        "型号": "A4988", "类别": "步进电机驱动", "子类": "步进电机",
        "厂商": "Allegro",
        "描述": "微步进步进电机驱动，最大16细分",
        "供电电压V": [8, 35], "逻辑电压V": [3.0, 5.5],
        "持续电流A": 1.0, "峰值电流A": 2.0,
        "PWM频率kHz": 0, "接口": "STEP/DIR/EN",
        "封装": "QFN28", "单价约": 5.0, "供货": "充足",
        "特点": "3D打印机标配，16细分，接线简单"
    },
    {
        "型号": "DRV8825", "类别": "步进电机驱动", "子类": "步进电机",
        "厂商": "TI",
        "描述": "微步进步进电机驱动，最大32细分",
        "供电电压V": [8.2, 45], "逻辑电压V": [2.5, 5.5],
        "持续电流A": 1.5, "峰值电流A": 2.5,
        "PWM频率kHz": 0, "接口": "STEP/DIR/EN",
        "封装": "HTSSOP28", "单价约": 8.0, "供货": "充足",
        "特点": "32细分，比A4988电流更大"
    },
    {
        "型号": "TMC2209", "类别": "步进电机驱动", "子类": "步进电机",
        "厂商": "Trinamic",
        "描述": "静音步进驱动，带StallGuard",
        "供电电压V": [4.75, 29], "逻辑电压V": [3.0, 5.5],
        "持续电流A": 1.4, "峰值电流A": 2.0,
        "PWM频率kHz": 0, "接口": "STEP/DIR/UART",
        "封装": "QFN28", "单价约": 15.0, "供货": "充足",
        "特点": "静音驱动，SpreadCycle，失速检测"
    },
    {
        "型号": "ULN2003A", "类别": "步进电机驱动", "子类": "步进电机",
        "厂商": "TI",
        "描述": "达林顿阵列驱动器，7路",
        "供电电压V": [5, 50], "逻辑电压V": [3.3, 5],
        "持续电流A": 0.5, "峰值电流A": 0.6,
        "PWM频率kHz": 0, "接口": "GPIO",
        "封装": "DIP16/SOIC16", "单价约": 1.5, "供货": "充足",
        "特点": "超低成本，适合28BYJ-48等小步进"
    },
    # ======== 无刷电机驱动 ========
    {
        "型号": "EG2131/EG2132", "类别": "无刷驱动", "子类": "BLDC",
        "厂商": "屹晶微",
        "描述": "三相MOS驱动半桥",
        "供电电压V": [10, 600], "逻辑电压V": [3.3, 5],
        "持续电流A": 10.0, "峰值电流A": 30.0,
        "PWM频率kHz": 200, "接口": "3相PWM+EN",
        "封装": "SOP16", "单价约": 5.0, "供货": "充足",
        "特点": "高压MOS驱动，需外接MOS管"
    },
    {
        "型号": "MP6540", "类别": "无刷驱动", "子类": "BLDC",
        "厂商": "MPS",
        "描述": "三相BLDC栅极驱动器",
        "供电电压V": [5, 36], "逻辑电压V": [3.3, 5],
        "持续电流A": 0.0, "峰值电流A": 0.0,
        "PWM频率kHz": 500, "接口": "3相PWM",
        "封装": "QFN20", "单价约": 12.0, "供货": "一般",
        "特点": "BLDC栅极驱动，需外接MOS"
    },
    {
        "型号": "L6235", "类别": "无刷驱动", "子类": "BLDC",
        "厂商": "ST",
        "描述": "三相BLDC全集成驱动",
        "供电电压V": [7, 52], "逻辑电压V": [3.3, 5],
        "持续电流A": 4.0, "峰值电流A": 5.6,
        "PWM频率kHz": 100, "接口": "HALL+PWM",
        "封装": "PowerSO36", "单价约": 25.0, "供货": "一般",
        "特点": "全集成BLDC驱动，内置MOS"
    },
    # ======== 舵机驱动 ========
    {
        "型号": "PCA9685", "类别": "舵机驱动", "子类": "舵机/PWM",
        "厂商": "NXP",
        "描述": "16路12位PWM驱动器，I2C接口",
        "供电电压V": [2.3, 5.5], "逻辑电压V": [2.3, 5.5],
        "持续电流A": 0.025, "峰值电流A": 0.04,
        "PWM频率kHz": 1.0, "接口": "I2C",
        "封装": "TSSOP28", "单价约": 8.0, "供货": "充足",
        "特点": "16路舵机控制，可级联62个(992路)"
    },
    # ======== LED驱动 ========
    {
        "型号": "TLC5940", "类别": "LED驱动", "子类": "LED",
        "厂商": "TI",
        "描述": "16通道12位PWM恒流LED驱动",
        "供电电压V": [3.0, 5.5], "逻辑电压V": [3.0, 5.5],
        "持续电流A": 0.12, "峰值电流A": 0.12,
        "PWM频率kHz": 0, "接口": "SPI",
        "封装": "TSSOP28/DIP28", "单价约": 10.0, "供货": "充足",
        "特点": "16路恒流，灰度控制，级联"
    },
    {
        "型号": "WS2812B", "类别": "LED驱动", "子类": "RGB LED",
        "厂商": "Worldsemi",
        "描述": "内置IC智能RGB LED",
        "供电电压V": [3.5, 5.3], "逻辑电压V": [3.5, 5.3],
        "持续电流A": 0.06, "峰值电流A": 0.06,
        "PWM频率kHz": 0, "接口": "单线协议",
        "封装": "5050", "单价约": 0.5, "供货": "充足",
        "特点": "可编程RGB灯珠，级联控制，炫彩效果"
    },
    {
        "型号": "AP3032", "类别": "LED驱动", "子类": "LED恒流",
        "厂商": "BCD",
        "描述": "白光LED恒流驱动升压芯片",
        "供电电压V": [2.7, 16], "逻辑电压V": [0, 0],
        "持续电流A": 0.02, "峰值电流A": 0.02,
        "PWM频率kHz": 1200, "接口": "PWM调光",
        "封装": "SOT23-5", "单价约": 1.0, "供货": "充足",
        "特点": "背光/照明LED恒流驱动"
    },
    {
        "型号": "PT4115", "类别": "LED驱动", "子类": "LED恒流",
        "厂商": "华润微",
        "描述": "降压型LED恒流驱动",
        "供电电压V": [6, 30], "逻辑电压V": [0, 0],
        "持续电流A": 1.2, "峰值电流A": 1.2,
        "PWM频率kHz": 200, "接口": "DIM调光",
        "封装": "SOT89-5/SOT23-5", "单价约": 1.0, "供货": "充足",
        "特点": "大功率LED驱动，效率高达97%"
    },
    # ======== 继电器驱动 ========
    {
        "型号": "ULN2803A", "类别": "继电器驱动", "子类": "继电器/负载",
        "厂商": "TI",
        "描述": "8路达林顿阵列，50V/500mA",
        "供电电压V": [5, 50], "逻辑电压V": [3.3, 5],
        "持续电流A": 0.5, "峰值电流A": 0.6,
        "PWM频率kHz": 0, "接口": "GPIO",
        "封装": "DIP18/SOIC18", "单价约": 2.0, "供货": "充足",
        "特点": "8路继电器/感性负载驱动，内置续流二极管"
    },
    # ======== H桥专用 ========
    {
        "型号": "BTS7960", "类别": "电机驱动", "子类": "大功率直流电机",
        "厂商": "Infineon",
        "描述": "43A大电流半桥驱动模块",
        "供电电压V": [5.5, 27], "逻辑电压V": [5, 5],
        "持续电流A": 43.0, "峰值电流A": 60.0,
        "PWM频率kHz": 25, "接口": "PWM+方向",
        "封装": "TO-220-7", "单价约": 15.0, "供货": "充足",
        "特点": "电赛大电机首选，需成对使用组成H桥"
    },
    # ======== MOS管驱动 ========
    {
        "型号": "IR2104", "类别": "MOS驱动", "子类": "MOS半桥",
        "厂商": "Infineon",
        "描述": "半桥MOS管栅极驱动器",
        "供电电压V": [10, 20], "逻辑电压V": [3.3, 5],
        "持续电流A": 0.0, "峰值电流A": 0.0,
        "PWM频率kHz": 200, "接口": "PWM+SD",
        "封装": "SOIC8/DIP8", "单价约": 3.0, "供货": "充足",
        "特点": "半桥驱动，可驱动NMOS，用于DC-DC/电机"
    },
    {
        "型号": "IR2184", "类别": "MOS驱动", "子类": "MOS半桥",
        "厂商": "Infineon",
        "描述": "大电流半桥MOS驱动器",
        "供电电压V": [10, 20], "逻辑电压V": [3.3, 5],
        "持续电流A": 0.0, "峰值电流A": 0.0,
        "PWM频率kHz": 200, "接口": "IN/SD",
        "封装": "SOIC8/DIP8", "单价约": 4.0, "供货": "充足",
        "特点": "4A峰值驱动电流，适合大功率MOS"
    },
    # ======== 音频功放 ========
    {
        "型号": "LM386", "类别": "音频驱动", "子类": "音频功放",
        "厂商": "TI",
        "描述": "低电压音频功放，0.5W",
        "供电电压V": [4, 12], "逻辑电压V": [0, 0],
        "持续电流A": 0.0, "峰值电流A": 0.0,
        "PWM频率kHz": 0, "接口": "模拟音频",
        "封装": "DIP8/SOIC8", "单价约": 1.5, "供货": "充足",
        "特点": "经典小功放，接线简单"
    },
    {
        "型号": "PAM8403", "类别": "音频驱动", "子类": "音频功放",
        "厂商": "Diodes",
        "描述": "3W双通道D类音频功放",
        "供电电压V": [2.5, 5.5], "逻辑电压V": [0, 0],
        "持续电流A": 0.0, "峰值电流A": 0.0,
        "PWM频率kHz": 0, "接口": "模拟音频",
        "封装": "SOP16", "单价约": 1.5, "供货": "充足",
        "特点": "D类高效，3W×2，体积小"
    },
]


def find_drivers(requirements: dict) -> list:
    """根据需求筛选驱动IC"""
    results = []
    for drv in DRIVER_DATABASE:
        score = 0
        reasons = []
        penalties = []

        # 硬性筛选
        if requirements.get("load_type"):
            lt = requirements["load_type"].lower()
            if lt not in drv["类别"].lower() and lt not in drv["子类"].lower():
                continue

        if requirements.get("motor_type"):
            mt = requirements["motor_type"].lower()
            if mt not in drv["子类"].lower() and mt not in drv["类别"].lower():
                continue

        # 电压检查
        if requirements.get("voltage"):
            v = requirements["voltage"]
            if not (drv["供电电压V"][0] <= v <= drv["供电电压V"][1]):
                continue

        # 持续电流检查（要求驱动能提供所需电流）
        if requirements.get("current"):
            i = requirements["current"]
            if drv["持续电流A"] > 0 and drv["持续电流A"] < i * 0.8:
                continue  # 电流不足

        # 软性评分
        # 电流余量
        if requirements.get("current") and drv["持续电流A"] > 0:
            ratio = drv["持续电流A"] / requirements["current"]
            if ratio >= 2:
                score += 15
                reasons.append(f"电流余量充足({drv['持续电流A']}A)")
            elif ratio >= 1.2:
                score += 8

        # 价格加分
        if drv["单价约"] <= 3:
            score += 15
            reasons.append(f"低成本(¥{drv['单价约']})")
        elif drv["单价约"] <= 10:
            score += 8
        elif drv["单价约"] <= 25:
            score += 3

        # 供货加分
        if drv["供货"] == "充足":
            score += 10
            reasons.append("供货充足")
        else:
            penalties.append("供货紧张")

        # PWM频率加分
        if requirements.get("pwm_freq") and drv["PWM频率kHz"] > 0:
            if drv["PWM频率kHz"] >= requirements["pwm_freq"]:
                score += 10
                reasons.append(f"PWM频率{drv['PWM频率kHz']}kHz满足要求")

        # 接口匹配
        if requirements.get("interface"):
            iface = requirements["interface"].upper()
            if iface in drv["接口"].upper():
                score += 10
                reasons.append(f"接口匹配({drv['接口']})")

        # 关键词
        if requirements.get("keyword"):
            kw = requirements["keyword"]
            if kw in drv["特点"] or kw in drv["描述"]:
                score += 15
                reasons.append(f"关键词匹配")

        results.append({
            "型号": drv["型号"],
            "类别": drv["类别"],
            "子类": drv["子类"],
            "厂商": drv["厂商"],
            "描述": drv["描述"],
            "供电电压": f"{drv['供电电压V'][0]}~{drv['供电电压V'][1]}V",
            "持续电流": f"{drv['持续电流A']}A" if drv['持续电流A'] > 0 else "N/A",
            "接口": drv["接口"],
            "单价约": f"¥{drv['单价约']}",
            "供货": drv["供货"],
            "特点": drv["特点"],
            "匹配分": score,
            "优势": reasons,
            "不足": penalties,
        })

    results.sort(key=lambda x: x["匹配分"], reverse=True)
    return results


def print_results(results: list, top_n: int = 5):
    if not results:
        print("\n❌ 未找到满足条件的驱动IC。")
        return

    print(f"\n{'='*70}")
    print(f"  驱动选型推荐结果（共 {len(results)} 款，显示前 {min(top_n, len(results))} 款）")
    print(f"{'='*70}")

    for i, d in enumerate(results[:top_n], 1):
        print(f"\n  ┌─ 第 {i} 名 ─ 匹配分: {d['匹配分']}分")
        print(f"  │ 型号: {d['型号']}  类别: {d['类别']}/{d['子类']}  厂商: {d['厂商']}")
        print(f"  │ 描述: {d['描述']}")
        print(f"  │ 供电: {d['供电电压']}  持续电流: {d['持续电流']}")
        print(f"  │ 接口: {d['接口']}  单价: {d['单价约']}  供货: {d['供货']}")
        print(f"  │ 特点: {d['特点']}")
        if d["优势"]:
            print(f"  │ ✅ {'; '.join(d['优势'])}")
        if d["不足"]:
            print(f"  │ ⚠️ {'; '.join(d['不足'])}")
        print(f"  └{'─'*60}")
    print()


def interactive_mode():
    print("\n" + "="*60)
    print("  🔧 驱动IC交互式选型工具")
    print("="*60)
    print("  （直接回车跳过）\n")

    req = {}
    try:
        lt = input("  负载类型 (电机/步进电机/无刷电机/舵机/LED/继电器/音频/MOS驱动): ").strip()
        if lt: req["load_type"] = lt

        mt = input("  电机类型 (直流/步进/BLDC/大功率): ").strip()
        if mt: req["motor_type"] = mt

        v = input("  供电电压 (V): ").strip()
        if v: req["voltage"] = float(v)

        i = input("  所需持续电流 (A): ").strip()
        if i: req["current"] = float(i)

        iface = input("  接口偏好 (GPIO/PWM/I2C/SPI): ").strip()
        if iface: req["interface"] = iface

        kw = input("  关键词: ").strip()
        if kw: req["keyword"] = kw

        print(f"\n  正在筛选...")
        results = find_drivers(req)
        print_results(results)
    except (ValueError, KeyboardInterrupt):
        print("\n  已取消。")


def list_types():
    types = {}
    for d in DRIVER_DATABASE:
        key = f"{d['类别']}/{d['子类']}"
        if key not in types:
            types[key] = 0
        types[key] += 1

    print(f"\n{'='*60}")
    print(f"  📦 可选驱动类型（共{len(DRIVER_DATABASE)}款）")
    print(f"{'='*60}")
    for t, cnt in sorted(types.items()):
        print(f"  • {t:30s}  共 {cnt} 款")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="驱动选型工具 - 根据负载推荐驱动IC",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例:
  python driver_selector.py --load-type 电机 --voltage 12 --current 2
  python driver_selector.py --load-type LED --current 0.5
  python driver_selector.py --list-types
  python driver_selector.py --interactive
"""
    )

    parser.add_argument("--load-type", help="负载类型 (电机/步进/LED/继电器/舵机/音频)")
    parser.add_argument("--motor-type", help="电机类型 (直流/步进/BLDC/大功率)")
    parser.add_argument("--voltage", type=float, help="供电电压 (V)")
    parser.add_argument("--current", type=float, help="所需持续电流 (A)")
    parser.add_argument("--pwm-freq", type=float, help="PWM频率需求 (kHz)")
    parser.add_argument("--interface", help="接口偏好 (GPIO/PWM/I2C/SPI)")
    parser.add_argument("--keyword", help="关键词")
    parser.add_argument("--top", type=int, default=5, help="显示前N个结果")
    parser.add_argument("--list-types", action="store_true", help="列出所有驱动类型")
    parser.add_argument("--interactive", action="store_true", help="交互式选型")
    parser.add_argument("--json", action="store_true", help="JSON输出")

    args = parser.parse_args()

    if args.list_types:
        list_types()
        return
    if args.interactive:
        interactive_mode()
        return

    req = {}
    if args.load_type: req["load_type"] = args.load_type
    if args.motor_type: req["motor_type"] = args.motor_type
    if args.voltage: req["voltage"] = args.voltage
    if args.current: req["current"] = args.current
    if args.pwm_freq: req["pwm_freq"] = args.pwm_freq
    if args.interface: req["interface"] = args.interface
    if args.keyword: req["keyword"] = args.keyword

    if not req:
        parser.print_help()
        print("\n💡 请至少指定一个条件，或使用 --interactive")
        return

    results = find_drivers(req)
    if args.json:
        print(json.dumps(results[:args.top], ensure_ascii=False, indent=2))
    else:
        print_results(results, top_n=args.top)


if __name__ == "__main__":
    main()
