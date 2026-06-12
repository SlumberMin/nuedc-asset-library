#!/usr/bin/env python3
"""PCB阻抗计算器 - 微带线/带状线/差分阻抗计算

用法:
    # 微带线阻抗
    python pcb_impedance_calculator.py microstrip --w 0.2 --h 0.8 --er 4.4 --t 0.035 --target-z 50

    # 带状线阻抗
    python pcb_impedance_calculator.py stripline --w 0.15 --h 0.4 --er 4.4 --t 0.035 --b 1.0 --target-z 50

    # 差分微带线阻抗
    python pcb_impedance_calculator.py diff_microstrip --w 0.15 --s 0.15 --h 0.8 --er 4.4 --t 0.035 --target-z 100

    # 差分带状线阻抗
    python pcb_impedance_calculator.py diff_stripline --w 0.15 --s 0.15 --h 0.4 --er 4.4 --t 0.035 --b 1.0 --target-z 100

    # 反算线宽（给定目标阻抗）
    python pcb_impedance_calculator.py microstrip --h 0.8 --er 4.4 --t 0.035 --target-z 50 --solve-w

    # 交互模式
    python pcb_impedance_calculator.py interactive

单位：mm（长度）, GHz（频率）, Ω（阻抗）
"""

import argparse
import math
import sys


def microstrip_impedance(w: float, h: float, er: float, t: float = 0.035, f_ghz: float = 1.0) -> dict:
    """
    微带线特性阻抗计算 (IPC-2141 / Hammerstad模型)

    参数:
        w:    线宽 (mm)
        h:    介质厚度 (mm)
        er:   介电常数 (FR4≈4.4)
        t:    铜箔厚度 (mm), 默认1oz=0.035mm
        f_ghz: 频率 (GHz), 用于有效介电常数修正

    返回:
        dict: Z0, er_eff, delay_ps_mm, capacitance_pf_mm, inductance_nh_mm
    """
    # 宽厚比
    w_h = w / h

    # 有效宽度修正（考虑铜箔厚度）
    if t > 0:
        delta_w = (t / math.pi) * (1 + math.log(4 * math.pi * w / t)) if w / t > math.pi * 2 else (t / math.pi) * (1 + math.log(2 * h / t))
        w_eff = w + delta_w
    else:
        w_eff = w

    w_eff_h = w_eff / h

    # 特性阻抗计算
    if w_eff_h <= 1:
        z0 = (60 / math.sqrt(er)) * math.log(8 * h / w_eff + w_eff / (4 * h))
    else:
        z0 = (120 * math.pi / math.sqrt(er)) / (w_eff_h + 1.393 + 0.667 * math.log(w_eff_h + 1.444))

    # 有效介电常数
    if w_eff_h <= 1:
        er_eff = (er + 1) / 2 + (er - 1) / 2 * ((1 / math.sqrt(1 + 12 / w_eff_h)) + 0.04 * (1 - w_eff_h) ** 2)
    else:
        er_eff = (er + 1) / 2 + (er - 1) / 2 / math.sqrt(1 + 12 / w_eff_h)

    # 频率修正（Dispersion）
    if f_ghz > 0:
        f_n = f_ghz * h / 0.0254  # 归一化频率(GHz·mil)
        er_eff_f = er - (er - er_eff) / (1 + 0.004 * f_n ** 2)
        z0_f = z0 * (er_eff / er_eff_f) ** 0.5 * (er_eff_f - 1) / (er_eff - 1) if er_eff != 1 else z0
    else:
        er_eff_f = er_eff
        z0_f = z0

    # 传播延迟 (ps/mm)
    delay = 33.33 * math.sqrt(er_eff_f)

    # 单位长度电容 (pF/mm)
    c_per_mm = delay / z0_f * 1000  # ps/mm → pF/mm

    # 单位长度电感 (nH/mm)
    l_per_mm = z0_f * delay / 1000  # Ω·ps/mm → nH/mm

    return {
        "Z0 (Ω)": round(z0_f, 2),
        "有效介电常数": round(er_eff_f, 3),
        "延迟 (ps/mm)": round(delay, 2),
        "电容 (pF/mm)": round(c_per_mm, 3),
        "电感 (nH/mm)": round(l_per_mm, 3),
        "有效线宽 (mm)": round(w_eff, 4),
    }


def stripline_impedance(w: float, h: float, er: float, t: float = 0.035, b: float = None) -> dict:
    """
    带状线特性阻抗计算 (对称带状线)

    参数:
        w:  线宽 (mm)
        h:  中心导体到参考平面的距离 (mm)
        er: 介电常数
        t:  铜箔厚度 (mm)
        b:  总介质厚度 (mm), 默认 2*h

    返回:
        dict: Z0, er_eff, delay_ps_mm 等
    """
    if b is None:
        b = 2 * h

    # 有效宽度修正
    t_b = t / b
    w_b = w / b

    if t_b > 0:
        if w_b < 0.35:
            delta = t_b / math.pi * (1 + math.log(4 * math.pi * w_b / t_b))
        else:
            delta = t_b / math.pi * (1 + math.log(2 * b / t_b))
        w_eff = w + delta * b
    else:
        w_eff = w

    w_eff_b = w_eff / b

    # 阻抗计算
    if w_eff_b < 0.35:
        z0 = (60 / math.sqrt(er)) * math.log(4 * b / (math.pi * w_eff))
    else:
        z0 = (94.15 / math.sqrt(er)) / (w_eff_b + 0.441 * b / b)

    # 更精确的公式
    # 使用Wadell公式
    k = math.exp(-math.pi * w_eff / b) if w_eff / b > 0 else 0
    if k > 0 and k < 1:
        kp = math.sqrt(1 - k * k)
        # 近似椭圆积分比
        from math import log as ln
        if k < 0.7:
            K_k = math.pi / 2 * (1 + (k / 2) ** 2 + (3 * k ** 2 / 16) ** 2)
            K_kp = ln(2 * (1 + math.sqrt(kp)) / (1 - math.sqrt(kp)))
        else:
            K_k = ln(2 * (1 + math.sqrt(k)) / (1 - math.sqrt(k)))
            K_kp = math.pi / 2 * (1 + (kp / 2) ** 2)

        z0_wadell = (30 * math.pi / math.sqrt(er)) * K_k / K_kp if K_kp > 0 else z0
        z0 = z0_wadell

    er_eff = er
    delay = 33.33 * math.sqrt(er)

    c_per_mm = delay / z0 * 1000
    l_per_mm = z0 * delay / 1000

    return {
        "Z0 (Ω)": round(z0, 2),
        "有效介电常数": round(er_eff, 3),
        "延迟 (ps/mm)": round(delay, 2),
        "电容 (pF/mm)": round(c_per_mm, 3),
        "电感 (nH/mm)": round(l_per_mm, 3),
    }


def diff_microstrip_impedance(w: float, s: float, h: float, er: float, t: float = 0.035) -> dict:
    """
    差分微带线阻抗计算

    参数:
        w: 线宽 (mm)
        s: 线间距 (mm)
        h: 介质厚度 (mm)
        er: 介电常数
        t: 铜箔厚度 (mm)

    返回:
        dict: Zdiff, Z0_single, 耦合系数等
    """
    # 单端阻抗
    single = microstrip_impedance(w, h, er, t)
    z0 = single["Z0 (Ω)"]

    # 耦合系数
    d = s + w  # 中心距
    # 使用IPC近似公式
    k = math.exp(-2 * math.pi * s / h) if s > 0 else 0.99

    # 有效介电常数
    er_eff = single["有效介电常数"]

    # 差分阻抗
    z_diff = 2 * z0 * (1 - 0.48 * math.exp(-0.96 * s / h))

    # 奇模阻抗
    z_odd = z_diff / 2

    # 偶模阻抗
    z_even = z0 * 2 * (1 + 0.48 * math.exp(-0.96 * s / h))

    # 耦合系数
    coupling = (z_even - z_odd) / (z_even + z_odd)

    return {
        "Zdiff (Ω)": round(z_diff, 2),
        "Z0_single (Ω)": round(z0, 2),
        "Zodd (Ω)": round(z_odd, 2),
        "Zeven (Ω)": round(z_even, 2),
        "耦合系数": round(coupling, 3),
        "有效介电常数": round(er_eff, 3),
    }


def diff_stripline_impedance(w: float, s: float, h: float, er: float, t: float = 0.035, b: float = None) -> dict:
    """
    差分带状线阻抗计算

    参数:
        w: 线宽 (mm)
        s: 线间距 (mm)
        h: 中心到参考面距离 (mm)
        er: 介电常数
        t: 铜箔厚度 (mm)
        b: 总介质厚度 (mm)
    """
    if b is None:
        b = 2 * h

    single = stripline_impedance(w, h, er, t, b)
    z0 = single["Z0 (Ω)"]

    z_diff = 2 * z0 * (1 - 0.347 * math.exp(-2.9 * s / b))
    z_odd = z_diff / 2
    z_even = z0 * 2 * (1 + 0.347 * math.exp(-2.9 * s / b))
    coupling = (z_even - z_odd) / (z_even + z_odd)

    return {
        "Zdiff (Ω)": round(z_diff, 2),
        "Z0_single (Ω)": round(z0, 2),
        "Zodd (Ω)": round(z_odd, 2),
        "Zeven (Ω)": round(z_even, 2),
        "耦合系数": round(coupling, 3),
    }


def solve_width_microstrip(target_z: float, h: float, er: float, t: float = 0.035) -> dict:
    """反算微带线宽度（给定目标阻抗）"""
    # 二分法求解
    w_low, w_high = 0.01, 20.0
    for _ in range(100):
        w_mid = (w_low + w_high) / 2
        result = microstrip_impedance(w_mid, h, er, t)
        z = result["Z0 (Ω)"]
        if z > target_z:
            w_low = w_mid
        else:
            w_high = w_mid
        if abs(z - target_z) < 0.01:
            break

    result = microstrip_impedance(w_mid, h, er, t)
    result["线宽 (mm)"] = round(w_mid, 4)
    return result


def solve_width_stripline(target_z: float, h: float, er: float, t: float = 0.035, b: float = None) -> dict:
    """反算带状线宽度"""
    if b is None:
        b = 2 * h

    w_low, w_high = 0.01, 20.0
    for _ in range(100):
        w_mid = (w_low + w_high) / 2
        result = stripline_impedance(w_mid, h, er, t, b)
        z = result["Z0 (Ω)"]
        if z > target_z:
            w_low = w_mid
        else:
            w_high = w_mid
        if abs(z - target_z) < 0.01:
            break

    result = stripline_impedance(w_mid, h, er, t, b)
    result["线宽 (mm)"] = round(w_mid, 4)
    return result


def print_result(title: str, result: dict):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")
    for k, v in result.items():
        print(f"  {k:25s} = {v}")
    print()


def interactive_mode():
    print("\n" + "="*60)
    print("  🔧 PCB阻抗计算器（交互模式）")
    print("="*60)

    print("\n  传输线类型:")
    print("  1. 微带线 (Microstrip)")
    print("  2. 带状线 (Stripline)")
    print("  3. 差分微带线")
    print("  4. 差分带状线")

    try:
        choice = input("\n  请选择 [1-4]: ").strip()

        if choice == "1":
            print("\n  --- 微带线参数 ---")
            h = float(input("  介质厚度 h (mm): "))
            er = float(input("  介电常数 er (FR4≈4.4): "))
            t = float(input("  铜箔厚度 t (mm, 1oz≈0.035): ") or "0.035")
            mode = input("  正算(输入线宽) / 反算(输入目标阻抗) [w/z]: ").strip().lower()

            if mode == 'z':
                target_z = float(input("  目标阻抗 (Ω): "))
                result = solve_width_microstrip(target_z, h, er, t)
                print_result(f"微带线反算（目标Z0={target_z}Ω）", result)
            else:
                w = float(input("  线宽 w (mm): "))
                f = float(input("  频率 (GHz, 0=忽略色散): ") or "0")
                result = microstrip_impedance(w, h, er, t, f)
                print_result(f"微带线阻抗计算 (w={w}mm, h={h}mm)", result)

        elif choice == "2":
            print("\n  --- 带状线参数 ---")
            h = float(input("  中心到参考面距离 h (mm): "))
            b = float(input("  总介质厚度 b (mm, 默认2h): ") or str(2 * h))
            er = float(input("  介电常数 er: "))
            t = float(input("  铜箔厚度 t (mm): ") or "0.035")
            mode = input("  正算(w) / 反算(z) [w/z]: ").strip().lower()

            if mode == 'z':
                target_z = float(input("  目标阻抗 (Ω): "))
                result = solve_width_stripline(target_z, h, er, t, b)
                print_result(f"带状线反算（目标Z0={target_z}Ω）", result)
            else:
                w = float(input("  线宽 w (mm): "))
                result = stripline_impedance(w, h, er, t, b)
                print_result(f"带状线阻抗计算 (w={w}mm)", result)

        elif choice == "3":
            print("\n  --- 差分微带线参数 ---")
            w = float(input("  线宽 w (mm): "))
            s = float(input("  线间距 s (mm): "))
            h = float(input("  介质厚度 h (mm): "))
            er = float(input("  介电常数 er: "))
            t = float(input("  铜箔厚度 t (mm): ") or "0.035")
            result = diff_microstrip_impedance(w, s, h, er, t)
            print_result(f"差分微带线阻抗 (w={w}mm, s={s}mm, h={h}mm)", result)

        elif choice == "4":
            print("\n  --- 差分带状线参数 ---")
            w = float(input("  线宽 w (mm): "))
            s = float(input("  线间距 s (mm): "))
            h = float(input("  中心到参考面距离 h (mm): "))
            b = float(input("  总介质厚度 b (mm, 默认2h): ") or str(2 * h))
            er = float(input("  介电常数 er: "))
            t = float(input("  铜箔厚度 t (mm): ") or "0.035")
            result = diff_stripline_impedance(w, s, h, er, t, b)
            print_result(f"差分带状线阻抗 (w={w}mm, s={s}mm)", result)

        else:
            print("  无效选择")

    except (ValueError, KeyboardInterrupt):
        print("\n  已取消。")


def main():
    parser = argparse.ArgumentParser(
        description="PCB阻抗计算器 - 微带线/带状线/差分阻抗",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
常用参数参考:
  FR4: er=4.4, tanδ=0.02
  1oz铜箔: t=0.035mm
  2oz铜箔: t=0.070mm

常见阻抗标准:
  单端50Ω: 射频/信号线
  差分100Ω: USB/以太网/LVDS
  差分90Ω: USB2.0

示例:
  # 计算50Ω微带线阻抗
  python pcb_impedance_calculator.py microstrip --w 0.2 --h 0.8 --er 4.4

  # 反算50Ω微带线线宽
  python pcb_impedance_calculator.py microstrip --h 0.8 --er 4.4 --target-z 50 --solve-w

  # 计算差分100Ω微带线
  python pcb_impedance_calculator.py diff_microstrip --w 0.15 --s 0.15 --h 0.8 --er 4.4 --target-z 100
"""
    )

    subparsers = parser.add_subparsers(dest="mode", help="传输线类型")

    # 微带线
    ms = subparsers.add_parser("microstrip", help="微带线阻抗计算")
    ms.add_argument("--w", type=float, help="线宽 (mm)")
    ms.add_argument("--h", type=float, required=True, help="介质厚度 (mm)")
    ms.add_argument("--er", type=float, default=4.4, help="介电常数 (默认4.4)")
    ms.add_argument("--t", type=float, default=0.035, help="铜箔厚度 (mm, 默认0.035)")
    ms.add_argument("--f", type=float, default=0, help="频率 (GHz, 0=忽略色散)")
    ms.add_argument("--target-z", type=float, help="目标阻抗 (用于反算)")
    ms.add_argument("--solve-w", action="store_true", help="反算线宽模式")

    # 带状线
    sl = subparsers.add_parser("stripline", help="带状线阻抗计算")
    sl.add_argument("--w", type=float, help="线宽 (mm)")
    sl.add_argument("--h", type=float, required=True, help="中心到参考面距离 (mm)")
    sl.add_argument("--er", type=float, default=4.4, help="介电常数")
    sl.add_argument("--t", type=float, default=0.035, help="铜箔厚度 (mm)")
    sl.add_argument("--b", type=float, help="总介质厚度 (mm, 默认2h)")
    sl.add_argument("--target-z", type=float, help="目标阻抗")
    sl.add_argument("--solve-w", action="store_true", help="反算线宽模式")

    # 差分微带线
    dm = subparsers.add_parser("diff_microstrip", help="差分微带线阻抗计算")
    dm.add_argument("--w", type=float, required=True, help="线宽 (mm)")
    dm.add_argument("--s", type=float, required=True, help="线间距 (mm)")
    dm.add_argument("--h", type=float, required=True, help="介质厚度 (mm)")
    dm.add_argument("--er", type=float, default=4.4, help="介电常数")
    dm.add_argument("--t", type=float, default=0.035, help="铜箔厚度 (mm)")
    dm.add_argument("--target-z", type=float, help="目标差分阻抗")

    # 差分带状线
    ds = subparsers.add_parser("diff_stripline", help="差分带状线阻抗计算")
    ds.add_argument("--w", type=float, required=True, help="线宽 (mm)")
    ds.add_argument("--s", type=float, required=True, help="线间距 (mm)")
    ds.add_argument("--h", type=float, required=True, help="中心到参考面距离 (mm)")
    ds.add_argument("--er", type=float, default=4.4, help="介电常数")
    ds.add_argument("--t", type=float, default=0.035, help="铜箔厚度 (mm)")
    ds.add_argument("--b", type=float, help="总介质厚度 (mm)")

    # 交互模式
    subparsers.add_parser("interactive", help="交互式计算模式")

    args = parser.parse_args()

    if args.mode == "interactive" or args.mode is None:
        interactive_mode()
        return

    if args.mode == "microstrip":
        if args.solve_w:
            if not args.target_z:
                print("❌ 反算需要指定 --target-z")
                return
            result = solve_width_microstrip(args.target_z, args.h, args.er, args.t)
            print_result(f"微带线反算 (目标Z0={args.target_z}Ω)", result)
        elif args.w:
            result = microstrip_impedance(args.w, args.h, args.er, args.t, args.f)
            print_result(f"微带线阻抗 (w={args.w}mm, h={args.h}mm)", result)
        else:
            print("❌ 请指定 --w 或 --solve-w --target-z")

    elif args.mode == "stripline":
        b = args.b or 2 * args.h
        if args.solve_w:
            if not args.target_z:
                print("❌ 反算需要指定 --target-z")
                return
            result = solve_width_stripline(args.target_z, args.h, args.er, args.t, b)
            print_result(f"带状线反算 (目标Z0={args.target_z}Ω)", result)
        elif args.w:
            result = stripline_impedance(args.w, args.h, args.er, args.t, b)
            print_result(f"带状线阻抗 (w={args.w}mm)", result)
        else:
            print("❌ 请指定 --w 或 --solve-w --target-z")

    elif args.mode == "diff_microstrip":
        result = diff_microstrip_impedance(args.w, args.s, args.h, args.er, args.t)
        print_result(f"差分微带线 (w={args.w}mm, s={args.s}mm, h={args.h}mm)", result)

    elif args.mode == "diff_stripline":
        b = args.b or 2 * args.h
        result = diff_stripline_impedance(args.w, args.s, args.h, args.er, args.t, b)
        print_result(f"差分带状线 (w={args.w}mm, s={args.s}mm)", result)


if __name__ == "__main__":
    main()
