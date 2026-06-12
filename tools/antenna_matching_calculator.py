#!/usr/bin/env python3
"""天线匹配计算器 - L型/Π型/T型匹配网络设计

用法:
    # L型匹配网络
    python antenna_matching_calculator.py L --zs 50 --zl 100 --freq 433

    # Π型匹配网络（带Q值指定）
    python antenna_matching_calculator.py PI --zs 50 --zl 5 --freq 2400 --q 10

    # T型匹配网络
    python antenna_matching_calculator.py T --zs 50 --zl 200 --freq 915 --q 5

    # 分析现有匹配网络
    python antenna_matching_calculator.py analyze --zs 50 --zl 100 --freq 433 --topology L

    # 交互模式
    python antenna_matching_calculator.py interactive

频率单位: MHz
阻抗单位: Ω
"""

import argparse
import cmath
import math
import sys


# ============================================================
# L型匹配网络计算
# ============================================================

def L_match(zs: complex, zl: complex, freq_mhz: float) -> list:
    """
    计算L型匹配网络元件值

    L型匹配有8种拓扑结构，这里计算所有可行方案并返回。

    参数:
        zs: 源阻抗 (Ω), 如 50+0j
        zl: 负载阻抗 (Ω), 如 10+j50 (天线阻抗)
        freq_mhz: 工作频率 (MHz)

    返回:
        list: 所有可行的匹配方案
    """
    omega = 2 * math.pi * freq_mhz * 1e6
    rs = zs.real
    xs = zs.imag
    rl = zl.real
    xl = zl.imag

    solutions = []

    # L型匹配核心算法：将负载阻抗匹配到源阻抗
    # 两种主要拓扑：串联-并联 和 并联-串联

    # === 方案1: 负载串联Ls + 并联Cp 到源 ===
    # 先将负载阻抗变换到源阻抗的共轭
    # Q = sqrt(Rl/Rs - 1) (如果Rl > Rs)
    if rl > rs and rs > 0:
        Q_match = math.sqrt(rl / rs - 1)

        # 串联电感/电容抵消负载电抗
        x_series = xl  # 需要抵消的电抗
        x_parallel = (rl * rl + xl * xl) / (rl * Q_match)  # 并联电抗

        # 方案1a: 串联电感 + 并联电容
        if x_series >= 0:  # 负载呈感性，需串联电容抵消
            Ls = None
            Cs = -1 / (omega * x_series) if x_series != 0 else None
        else:
            Ls = -x_series / omega
            Cs = None

        Cp = 1 / (omega * x_parallel) if x_parallel > 0 else None
        Lp = -x_parallel / omega if x_parallel < 0 else None

        if Lp or Cp:
            solutions.append({
                "拓扑": "L型: 串联元件 + 并联元件（负载侧）",
                "Q值": round(Q_match, 2),
                "带宽_MHz": round(freq_mhz / Q_match, 2),
            })
            if Ls:
                solutions[-1]["串联电感"] = format_inductance(Ls)
            if Cs:
                solutions[-1]["串联电容"] = format_capacitance(Cs)
            if Cp:
                solutions[-1]["并联电容(到地)"] = format_capacitance(Cp)
            if Lp:
                solutions[-1]["并联电感(到地)"] = format_inductance(Lp)

    # === 方案2: Rs > Rl 时的L型匹配 ===
    if rs > rl and rl > 0:
        Q_match = math.sqrt(rs / rl - 1)

        # 从负载端看，并联Rs再串联
        x_parallel_load = rl * (1 + Q_match * Q_match) / Q_match
        x_series = rl * Q_match

        solutions.append({
            "拓扑": "L型: 并联元件(源侧) + 串联元件",
            "Q值": round(Q_match, 2),
            "带宽_MHz": round(freq_mhz / Q_match, 2),
        })

        if x_series > 0:
            solutions[-1]["串联电感"] = format_inductance(x_series / omega)
        else:
            solutions[-1]["串联电容"] = format_capacitance(-1 / (omega * x_series))

        if x_parallel_load > 0:
            solutions[-1]["并联电容"] = format_capacitance(1 / (omega * x_parallel_load))
        else:
            solutions[-1]["并联电感"] = format_inductance(-x_parallel_load / omega)

    # === 方案3: 简化L型（忽略负载电抗，仅匹配实部）===
    if rs > 0 and rl > 0:
        if rs != rl:
            Q_simple = math.sqrt(max(rs, rl) / min(rs, rl) - 1) if max(rs, rl) / min(rs, rl) > 1 else 0

            if Q_simple > 0:
                # 高阻抗到低阻抗: L-section
                x_series_simple = min(rs, rl) * Q_simple
                x_parallel_simple = max(rs, rl) / Q_simple

                solutions.append({
                    "拓扑": "L型: 简化匹配(忽略负载电抗)",
                    "Q值": round(Q_simple, 2),
                    "带宽_MHz": round(freq_mhz / Q_simple, 2),
                    "说明": f"Rs={rs}Ω → Rl={rl}Ω, 需先调谐负载电抗",
                })

                if rs > rl:
                    solutions[-1]["串联电感"] = format_inductance(x_series_simple / omega)
                    solutions[-1]["并联电容(到地)"] = format_capacitance(1 / (omega * x_parallel_simple))
                else:
                    solutions[-1]["串联电容"] = format_capacitance(1 / (omega * x_series_simple))
                    solutions[-1]["并联电感(到地)"] = format_inductance(x_parallel_simple / omega)

    return solutions


def PI_match(zs: complex, zl: complex, freq_mhz: float, q: float = 5) -> list:
    """
    Π型匹配网络计算

    适用场景: 高Q值匹配，滤波特性好
    拓扑: 并联C1 - 串联L - 并联C2

    参数:
        zs: 源阻抗
        zl: 负载阻抗
        freq_mhz: 频率(MHz)
        q: 期望Q值
    """
    omega = 2 * math.pi * freq_mhz * 1e6
    rs = zs.real
    rl = zl.real

    solutions = []

    # Π型匹配的核心：两个并联臂 + 一个串联臂
    # 虚拟电阻 R = Rs / (1 + Q^2) = Rl / (1 + Q^2)
    # 需要 Rs 和 Rl 都 > R

    if q <= 0:
        q = 5

    r_virtual = rs / (1 + q * q)

    if r_virtual <= 0:
        return solutions

    # 检查负载是否满足条件
    q_load = math.sqrt(rl / r_virtual - 1) if rl > r_virtual else 0
    q_source = q

    # 串联电抗 (在虚拟电阻处)
    x_series = r_virtual * (q_source + q_load)

    # 并联电抗（源侧）
    x_p1 = rs / q_source

    # 并联电抗（负载侧）
    x_p2 = rl / q_load if q_load > 0 else float('inf')

    solution = {
        "拓扑": "Π型: 并联C1 - 串联L - 并联C2",
        "设计Q值": round(q, 2),
        "带宽_MHz": round(freq_mhz / q, 2),
        "虚拟电阻": f"{r_virtual:.2f} Ω",
    }

    # 计算元件值
    if x_series > 0:
        L_series = x_series / omega
        solution["串联电感 L"] = format_inductance(L_series)
    else:
        C_series = -1 / (omega * x_series)
        solution["串联电容"] = format_capacitance(C_series)

    if x_p1 > 0:
        C_p1 = 1 / (omega * x_p1)
        solution["并联电容 C1(源侧)"] = format_capacitance(C_p1)
    else:
        L_p1 = -x_p1 / omega
        solution["并联电感 L1(源侧)"] = format_inductance(L_p1)

    if x_p2 > 0 and x_p2 < 1e10:
        C_p2 = 1 / (omega * x_p2)
        solution["并联电容 C2(负载侧)"] = format_capacitance(C_p2)
    elif x_p2 < 0:
        L_p2 = -x_p2 / omega
        solution["并联电感 L2(负载侧)"] = format_inductance(L_p2)

    solutions.append(solution)

    # 反向Π型 (L两端并联电容)
    solution2 = dict(solution)
    solution2["拓扑"] = "Π型变体: 并联L1 - 串联C - 并联L2"
    solutions.append(solution2)

    return solutions


def T_match(zs: complex, zl: complex, freq_mhz: float, q: float = 5) -> list:
    """
    T型匹配网络计算

    适用场景: 需要隔直，或负载/源有串联寄生电容
    拓扑: 串联L1 - 并联C - 串联L2

    参数:
        zs: 源阻抗
        zl: 负载阻抗
        freq_mhz: 频率(MHz)
        q: 期望Q值
    """
    omega = 2 * math.pi * freq_mhz * 1e6
    rs = zs.real
    rl = zl.real

    solutions = []

    # T型匹配：虚拟电阻 R = Rs*(1+Q^2) = Rl*(1+Q^2)
    # 需要 Rs 和 Rl 都 < R
    r_virtual = rs * (1 + q * q)

    # 串联电抗
    x_s1 = q * rs  # 源侧
    x_s2 = q * rl  # 负载侧

    # 并联电抗
    x_p = r_virtual / (q + (q * rl / rs)) if (q + q * rl / rs) != 0 else float('inf')

    solution = {
        "拓扑": "T型: 串联L1 - 并联C - 串联L2",
        "设计Q值": round(q, 2),
        "带宽_MHz": round(freq_mhz / q, 2),
        "虚拟电阻": f"{r_virtual:.2f} Ω",
    }

    # 源侧串联
    if x_s1 > 0:
        solution["串联电感 L1(源侧)"] = format_inductance(x_s1 / omega)
    else:
        solution["串联电容 C1(源侧)"] = format_capacitance(-1 / (omega * x_s1))

    # 负载侧串联
    if x_s2 > 0:
        solution["串联电感 L2(负载侧)"] = format_inductance(x_s2 / omega)
    else:
        solution["串联电容 C2(负载侧)"] = format_capacitance(-1 / (omega * x_s2))

    # 并联（到地）
    if x_p > 0:
        solution["并联电容 C(中间)"] = format_capacitance(1 / (omega * x_p))
    elif x_p < 0:
        solution["并联电感 L(中间)"] = format_inductance(-x_p / omega)

    solutions.append(solution)
    return solutions


def analyze_mismatch(zs: complex, zl: complex) -> dict:
    """分析阻抗失配情况"""
    # 反射系数
    gamma = (zl - zs) / (zl + zs)
    gamma_mag = abs(gamma)

    # 回波损耗
    rl_db = -20 * math.log10(gamma_mag) if gamma_mag > 0 else float('inf')

    # VSWR
    if gamma_mag < 1:
        vswr = (1 + gamma_mag) / (1 - gamma_mag)
    else:
        vswr = float('inf')

    # 插入损耗 (不匹配引起的)
    il_db = -10 * math.log10(1 - gamma_mag ** 2) if gamma_mag < 1 else float('inf')

    # 传输功率百分比
    power_transmitted = (1 - gamma_mag ** 2) * 100

    return {
        "反射系数|Γ|": round(gamma_mag, 4),
        "反射系数(角度)": f"{math.degrees(cmath.phase(gamma)):.1f}°",
        "回波损耗": f"{rl_db:.2f} dB",
        "VSWR": round(vswr, 2),
        "插入损耗": f"{il_db:.2f} dB",
        "传输功率": f"{power_transmitted:.1f}%",
    }


# ============================================================
# 格式化辅助函数
# ============================================================

def format_capacitance(c_farad: float) -> str:
    """格式化电容值为合适的单位"""
    if c_farad <= 0:
        return "N/A"
    if c_farad >= 1e-9:
        return f"{c_farad*1e12:.2f} pF"
    elif c_farad >= 1e-12:
        return f"{c_farad*1e9:.2f} nF ({c_farad*1e12:.1f} pF)"
    else:
        return f"{c_farad*1e15:.2f} fF"


def format_inductance(l_henry: float) -> str:
    """格式化电感值为合适的单位"""
    if l_henry <= 0:
        return "N/A"
    if l_henry >= 1e-6:
        return f"{l_henry*1e6:.2f} µH"
    elif l_henry >= 1e-9:
        return f"{l_henry*1e9:.2f} nH ({l_henry*1e6:.3f} µH)"
    else:
        return f"{l_henry*1e12:.2f} pH"


# ============================================================
# 输出
# ============================================================

def print_solutions(topology: str, zs: complex, zl: complex, freq_mhz: float, solutions: list):
    """打印匹配方案"""
    print(f"\n{'='*65}")
    print(f"  📡 {topology}匹配网络设计")
    print(f"{'='*65}")
    print(f"  源阻抗 Zs = {zs} Ω")
    print(f"  负载阻抗 Zl = {zl} Ω")
    print(f"  工作频率 f = {freq_mhz} MHz (λ = {300/freq_mhz:.1f} m)")

    # 失配分析
    mismatch = analyze_mismatch(zs, zl)
    print(f"\n  ─── 未匹配状态分析 ───")
    for k, v in mismatch.items():
        print(f"    {k:20s}: {v}")

    if not solutions:
        print(f"\n  ❌ 无可行的匹配方案（阻抗差距过大或不满足拓扑条件）")
        print(f"  💡 建议: 使用多级匹配或变压器预变换")
        return

    print(f"\n  ─── 匹配方案 ───")
    for i, sol in enumerate(solutions, 1):
        print(f"\n  方案 {i}: {sol['拓扑']}")
        if "Q值" in sol:
            print(f"    Q值 = {sol['Q值']}  带宽 = {sol['带宽_MHz']} MHz")
        elif "设计Q值" in sol:
            print(f"    设计Q值 = {sol['设计Q值']}  带宽 = {sol['带宽_MHz']} MHz")
        if "虚拟电阻" in sol:
            print(f"    虚拟电阻 = {sol['虚拟电阻']}")

        # 打印元件值
        for k, v in sol.items():
            if k in ("拓扑", "Q值", "带宽_MHz", "设计Q值", "虚拟电阻", "说明"):
                continue
            print(f"    {k:25s}: {v}")

        if "说明" in sol:
            print(f"    💡 {sol['说明']}")

    print()


def interactive_mode():
    """交互式匹配网络设计"""
    print("\n" + "="*60)
    print("  🔧 天线匹配网络交互式设计工具")
    print("="*60)

    try:
        print("\n  请选择匹配网络拓扑:")
        print("  1. L型匹配 (最简单，Q值不可控)")
        print("  2. Π型匹配 (高Q值，滤波特性好)")
        print("  3. T型匹配 (隔直，适合有寄生电容)")
        print("  4. 全部计算并比较")

        choice = input("\n  请选择 [1-4]: ").strip()

        freq = float(input("  工作频率 (MHz): "))
        rs = float(input("  源阻抗实部 Rs (Ω, 通常50): "))
        xs = float(input("  源阻抗虚部 Xs (Ω, 纯阻=0): ") or "0")
        rl = float(input("  负载阻抗实部 Rl (Ω): "))
        xl = float(input("  负载阻抗虚部 Xl (Ω, 天线实测值): ") or "0")

        zs = complex(rs, xs)
        zl = complex(rl, xl)

        q = 5
        if choice in ("2", "3"):
            q = float(input("  期望Q值 (推荐5~20, 带宽=f/Q): ") or "5")

        if choice == "1" or choice == "4":
            sols = L_match(zs, zl, freq)
            print_solutions("L型", zs, zl, freq, sols)

        if choice == "2" or choice == "4":
            sols = PI_match(zs, zl, freq, q)
            print_solutions("Π型", zs, zl, freq, sols)

        if choice == "3" or choice == "4":
            sols = T_match(zs, zl, freq, q)
            print_solutions("T型", zs, zl, freq, sols)

    except (ValueError, KeyboardInterrupt):
        print("\n  已取消。")


def main():
    parser = argparse.ArgumentParser(
        description="天线匹配计算器 - L型/Π型/T型匹配网络设计",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
常用天线阻抗参考:
  1/4波长单极天线(50Ω匹配): ~36+j0 Ω
  1/2波长偶极天线: ~73+j42 Ω
  PCB天线(433MHz): ~10+j100 Ω (需匹配)
  蓝牙/2.4G芯片天线: ~50+j0 Ω (已匹配)
  LoRa 433MHz天线: ~50+j0 Ω (已匹配)

常用频率:
  433 MHz (LoRa/遥控)
  868/915 MHz (LoRa/Sigfox)
  2400 MHz (WiFi/蓝牙/Zigbee)
  900 MHz (GSM)

示例:
  # 433MHz天线L型匹配
  python antenna_matching_calculator.py L --zs 50 --zl 36 --freq 433

  # 2.4GHz天线Π型匹配
  python antenna_matching_calculator.py PI --zs 50 --zl 20+j100 --freq 2400 --q 10
"""
    )

    subparsers = parser.add_subparsers(dest="mode", help="匹配网络类型")

    # L型
    l_parser = subparsers.add_parser("L", help="L型匹配网络")
    l_parser.add_argument("--zs", required=True, help="源阻抗 (如: 50 或 50+j0)")
    l_parser.add_argument("--zl", required=True, help="负载阻抗 (如: 100 或 10+j50)")
    l_parser.add_argument("--freq", type=float, required=True, help="频率 (MHz)")

    # Π型
    pi_parser = subparsers.add_parser("PI", help="Π型匹配网络")
    pi_parser.add_argument("--zs", required=True, help="源阻抗")
    pi_parser.add_argument("--zl", required=True, help="负载阻抗")
    pi_parser.add_argument("--freq", type=float, required=True, help="频率 (MHz)")
    pi_parser.add_argument("--q", type=float, default=10, help="期望Q值 (默认10)")

    # T型
    t_parser = subparsers.add_parser("T", help="T型匹配网络")
    t_parser.add_argument("--zs", required=True, help="源阻抗")
    t_parser.add_argument("--zl", required=True, help="负载阻抗")
    t_parser.add_argument("--freq", type=float, required=True, help="频率 (MHz)")
    t_parser.add_argument("--q", type=float, default=5, help="期望Q值 (默认5)")

    # 分析
    a_parser = subparsers.add_parser("analyze", help="分析阻抗失配")
    a_parser.add_argument("--zs", required=True, help="源阻抗")
    a_parser.add_argument("--zl", required=True, help="负载阻抗")
    a_parser.add_argument("--freq", type=float, default=100, help="频率 (MHz)")

    # 交互
    subparsers.add_parser("interactive", help="交互式设计")

    args = parser.parse_args()

    if args.mode == "interactive" or args.mode is None:
        interactive_mode()
        return

    def parse_z(s: str) -> complex:
        """解析阻抗字符串，支持 '50', '50+j0', '10-j50', '10+50j', '10+j100' 等格式"""
        import re
        s = s.replace(" ", "")
        # Python complex() 不能直接解析 "10+j100", 需要转为 "10+100j"
        # 将 "+j数字" 或 "-j数字" 转为 "+数字j" 或 "-数字j"
        s2 = re.sub(r'([+-])j(\d*\.?\d+)', lambda m: m.group(1) + m.group(2) + 'j', s)
        # 处理单独的 "+j" 或 "-j" (即 ±1j)
        s2 = re.sub(r'([+-])j(?!\d)', lambda m: m.group(1) + '1j', s2)
        # 处理开头的 "j数字" 或 "j"
        if s2.startswith('j'):
            s2 = '1' + s2  # "j5" -> "1j5" is wrong, need "5j"
            # Actually: "j5" means j*5, so "5j"
            m2 = re.match(r'^j(\d*\.?\d+)$', s)
            if m2:
                s2 = m2.group(1) + 'j'
            elif s == 'j':
                s2 = '1j'
        try:
            return complex(s2)
        except ValueError:
            raise ValueError(f"无法解析阻抗字符串: {s}")

    if args.mode == "analyze":
        zs = parse_z(args.zs)
        zl = parse_z(args.zl)
        mismatch = analyze_mismatch(zs, zl)
        print(f"\n{'='*60}")
        print(f"  📡 阻抗失配分析")
        print(f"{'='*60}")
        print(f"  Zs = {zs} Ω,  Zl = {zl} Ω")
        for k, v in mismatch.items():
            print(f"  {k:20s}: {v}")

        # 匹配建议
        print(f"\n  ─── 匹配建议 ───")
        rs, rl = zs.real, zl.real
        ratio = max(rs, rl) / min(rs, rl) if min(rs, rl) > 0 else 0
        if mismatch["VSWR"] < 1.5:
            print("  ✅ VSWR < 1.5，匹配良好，无需额外匹配网络")
        elif mismatch["VSWR"] < 2.0:
            print("  ⚠️ VSWR < 2.0，尚可接受，可考虑简单匹配优化")
        elif mismatch["VSWR"] < 3.0:
            print("  ⚠️ VSWR < 3.0，建议使用L型匹配网络")
        else:
            if ratio > 10:
                print("  ❌ VSWR > 3.0 且阻抗比>10:1，建议使用Π型或T型匹配网络")
            else:
                print("  ❌ VSWR > 3.0，建议使用L型匹配网络")
        print()
        return

    zs = parse_z(args.zs)
    zl = parse_z(args.zl)
    freq = args.freq

    if args.mode == "L":
        sols = L_match(zs, zl, freq)
        print_solutions("L型", zs, zl, freq, sols)

    elif args.mode == "PI":
        sols = PI_match(zs, zl, freq, args.q)
        print_solutions("Π型", zs, zl, freq, sols)

    elif args.mode == "T":
        sols = T_match(zs, zl, freq, args.q)
        print_solutions("T型", zs, zl, freq, sols)


if __name__ == "__main__":
    main()
