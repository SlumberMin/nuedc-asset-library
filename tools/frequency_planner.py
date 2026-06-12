#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
频率规划器 - PLL/DDS/时钟树配置计算
====================================
功能：
  - STM32 PLL配置计算（自动求解PLLM/N/P/Q参数）
  - DDS频率控制字计算（AD9833/AD9850/AD9910等）
  - 时钟树分频器配置
  - 频率精度/误差分析
  - 时钟源切换与HSE/HSI/LSE/LSI配置
"""

import argparse
import json
import math
import sys
from datetime import datetime

# ============================================================
# 常量定义
# ============================================================

# DDS芯片参数
DDS_CHIPS = {
    "AD9833": {
        "name": "AD9833",
        "mclk_max": 25e6,        # 最大主时钟(Hz)
        "phase_bits": 12,        # 相位寄存器位数
        "freq_bits": 28,         # 频率寄存器位数
        "dac_bits": 10,          # DAC位数
        "output_max": 12.5e6,    # 最大输出频率(Hz)
        "desc": "低功耗DDS，0-12.5MHz，正弦/三角/方波",
    },
    "AD9850": {
        "name": "AD9850",
        "mclk_max": 125e6,
        "phase_bits": 5,
        "freq_bits": 32,
        "dac_bits": 10,
        "output_max": 62.5e6,
        "desc": "经典DDS，0-62.5MHz",
    },
    "AD9851": {
        "name": "AD9851",
        "mclk_max": 180e6,
        "phase_bits": 5,
        "freq_bits": 32,
        "dac_bits": 10,
        "output_max": 70e6,
        "desc": "高速DDS，含6倍频器",
    },
    "AD9910": {
        "name": "AD9910",
        "mclk_max": 1e9,
        "phase_bits": 16,
        "freq_bits": 32,
        "dac_bits": 14,
        "output_max": 400e6,
        "desc": "高性能DDS，1GSPS，14-bit DAC",
    },
    "AD9959": {
        "name": "AD9959",
        "mclk_max": 500e6,
        "phase_bits": 14,
        "freq_bits": 32,
        "dac_bits": 10,
        "output_max": 200e6,
        "desc": "4通道同步DDS",
    },
}

# STM32系列时钟参数
STM32_CLOCKS = {
    "STM32F407": {
        "hsi": 16e6,
        "hse_typical": 8e6,
        "lse": 32768,
        "lsi": 32000,
        "vco_in_min": 1e6,
        "vco_in_max": 2e6,
        "vco_out_min": 100e6,
        "vco_out_max": 432e6,
        "pll_m_range": (2, 63),
        "pll_n_range": (50, 432),
        "pll_p_values": [2, 4, 6, 8],
        "pll_q_range": (2, 15),
        "sysclk_max": 168e6,
        "apb1_max": 42e6,
        "apb2_max": 84e6,
        "flash_wait_3v3": {  # Flash等待周期 @ 3.3V
            30: 0, 60: 1, 90: 2, 120: 3, 150: 4, 168: 5,
        },
    },
    "STM32F103": {
        "hsi": 8e6,
        "hse_typical": 8e6,
        "lse": 32768,
        "lsi": 40000,
        "vco_in_min": 1e6,
        "vco_in_max": 24e6,
        "vco_out_min": 25e6,
        "vco_out_max": 72e6,
        "pll_m_range": (1, 16),    # 预分频
        "pll_n_range": (2, 16),    # 倍频
        "pll_p_values": [1, 2, 4, 8],  # 不直接支持,但用于AHB分频
        "pll_q_range": None,
        "sysclk_max": 72e6,
        "apb1_max": 36e6,
        "apb2_max": 72e6,
        "flash_wait_3v3": {
            24: 0, 48: 1, 72: 2,
        },
    },
    "STM32L476": {
        "hsi": 16e6,
        "hse_typical": 8e6,
        "lse": 32768,
        "lsi": 32000,
        "msi_range": {  # MSI频率范围
            0: 100e3, 1: 200e3, 2: 400e3, 3: 800e3,
            4: 1e6, 5: 2e6, 6: 4e6, 7: 8e6,
            8: 16e6, 9: 24e6, 10: 32e6, 11: 48e6,
        },
        "vco_in_min": 2.66e6,
        "vco_in_max": 8e6,
        "vco_out_min": 96e6,
        "vco_out_max": 344e6,
        "pll_m_range": (1, 8),
        "pll_n_range": (8, 86),
        "pll_p_values": [2, 4, 6, 8],
        "pll_q_range": (2, 8),
        "pll_r_range": (2, 8),
        "sysclk_max": 80e6,
        "apb1_max": 80e6,
        "apb2_max": 80e6,
        "flash_wait_3v3": {
            16: 0, 32: 1, 48: 2, 64: 3, 80: 4,
        },
    },
    "STM32H743": {
        "hsi": 64e6,
        "hse_typical": 25e6,
        "lse": 32768,
        "lsi": 32000,
        "vco_in_min": 1e6,
        "vco_in_max": 16e6,
        "vco_out_min": 150e6,
        "vco_out_max": 960e6,
        "pll_m_range": (1, 63),
        "pll_n_range": (4, 512),
        "pll_p_values": [2, 4, 6, 8],
        "pll_q_range": (1, 128),
        "pll_r_range": (1, 128),
        "sysclk_max": 480e6,
        "apb1_max": 120e6,
        "apb2_max": 120e6,
        "flash_wait_3v3": {
            70: 0, 140: 2, 185: 3, 210: 4, 225: 5, 240: 7,
        },
    },
}


def format_freq(hz):
    """格式化频率显示"""
    if hz >= 1e9:
        return f"{hz/1e9:.4f}GHz"
    elif hz >= 1e6:
        return f"{hz/1e6:.4f}MHz"
    elif hz >= 1e3:
        return f"{hz/1e3:.4f}kHz"
    else:
        return f"{hz:.1f}Hz"


# ============================================================
# PLL计算
# ============================================================

def calculate_stm32_pll(stm32_series, target_sysclk, hse_freq=None):
    """
    计算STM32 PLL参数
    参数:
      stm32_series: STM32系列型号
      target_sysclk: 目标系统时钟(Hz)
      hse_freq: 外部晶振频率(Hz)，为None则使用典型值
    返回: PLL配置方案列表
    """
    params = STM32_CLOCKS.get(stm32_series)
    if not params:
        print(f"[错误] 不支持的STM32系列: {stm32_series}")
        print(f"  支持: {', '.join(STM32_CLOCKS.keys())}")
        return []

    hse = hse_freq if hse_freq else params['hse_typical']

    if target_sysclk > params['sysclk_max']:
        print(f"[警告] 目标频率 {format_freq(target_sysclk)} 超过 {stm32_series} 最大系统时钟 "
              f"{format_freq(params['sysclk_max'])}")

    solutions = []
    m_min, m_max = params['pll_m_range']
    n_min, n_max = params['pll_n_range']

    # 遍历PLLM
    for m in range(m_min, m_max + 1):
        vco_in = hse / m

        # 检查VCO输入范围
        if vco_in < params['vco_in_min'] or vco_in > params['vco_in_max']:
            continue

        # 遍历PLLN
        for n in range(n_min, n_max + 1):
            vco_out = vco_in * n

            # 检查VCO输出范围
            if vco_out < params['vco_out_min'] or vco_out > params['vco_out_max']:
                continue

            # 遍历PLLP
            for p in params['pll_p_values']:
                sysclk = vco_out / p

                # 检查是否接近目标频率 (误差<1%)
                if abs(sysclk - target_sysclk) / target_sysclk < 0.01:
                    # 计算PLLQ (用于USB时钟48MHz)
                    pll_q = None
                    if params.get('pll_q_range'):
                        q_min, q_max = params['pll_q_range']
                        for q in range(q_min, q_max + 1):
                            q_freq = vco_out / q
                            if abs(q_freq - 48e6) < 48e6 * 0.005:  # 48MHz ±0.5%
                                pll_q = q
                                break

                    # 计算AHB/APB分频
                    ahb_prescaler = 1
                    apb1_prescaler = 1
                    apb2_prescaler = 1

                    if sysclk > params['apb1_max']:
                        for div in [2, 4, 8, 16]:
                            if sysclk / div <= params['apb1_max']:
                                apb1_prescaler = div
                                break

                    if sysclk > params['apb2_max']:
                        for div in [2, 4, 8, 16]:
                            if sysclk / div <= params['apb2_max']:
                                apb2_prescaler = div
                                break

                    # Flash等待周期
                    flash_ws = 0
                    for freq_limit, ws in sorted(params['flash_wait_3v3'].items()):
                        if sysclk / 1e6 <= freq_limit:
                            flash_ws = ws
                            break
                    else:
                        flash_ws = max(params['flash_wait_3v3'].values())

                    error = abs(sysclk - target_sysclk)
                    error_pct = error / target_sysclk * 100

                    solution = {
                        "stm32": stm32_series,
                        "hse": hse,
                        "pll_m": m,
                        "pll_n": n,
                        "pll_p": p,
                        "pll_q": pll_q,
                        "vco_in": vco_in,
                        "vco_out": vco_out,
                        "sysclk": sysclk,
                        "ahb_freq": sysclk / ahb_prescaler,
                        "apb1_freq": sysclk / apb1_prescaler,
                        "apb2_freq": sysclk / apb2_prescaler,
                        "ahb_prescaler": ahb_prescaler,
                        "apb1_prescaler": apb1_prescaler,
                        "apb2_prescaler": apb2_prescaler,
                        "flash_wait_states": flash_ws,
                        "error_hz": error,
                        "error_pct": error_pct,
                        "usb_48m_ok": pll_q is not None,
                    }
                    solutions.append(solution)

    # 按误差排序
    solutions.sort(key=lambda s: (s['error_pct'], s['pll_m']))

    return solutions


def generate_pll_register_config(solution, source="HSE"):
    """
    生成PLL寄存器配置代码
    """
    s = solution
    lines = []
    lines.append(f"/* PLL配置 - {s['stm32']} @ {format_freq(s['sysclk'])} */")
    lines.append(f"/* 源: {source} ({format_freq(s['hse'])}) */")
    lines.append(f"/* VCO输入: {format_freq(s['vco_in'])}, VCO输出: {format_freq(s['vco_out'])} */")
    lines.append("")

    if "F4" in s['stm32'] or "H7" in s['stm32']:
        lines.append("/* 使能HSE */")
        lines.append("RCC->CR |= RCC_CR_HSEON;")
        lines.append("while(!(RCC->CR & RCC_CR_HSERDY));")
        lines.append("")
        lines.append("/* 配置PLL */")
        lines.append(f"RCC->PLLCFGR = 0;")
        lines.append(f"RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_{source};  // PLL源: {source}")
        lines.append(f"RCC->PLLCFGR |= ({s['pll_m']} << RCC_PLLCFGR_PLLM_Pos);  // PLLM={s['pll_m']}")
        lines.append(f"RCC->PLLCFGR |= ({s['pll_n']} << RCC_PLLCFGR_PLLN_Pos);  // PLLN={s['pll_n']}")
        lines.append(f"RCC->PLLCFGR |= ({(s['pll_p']//2 - 1)} << RCC_PLLCFGR_PLLP_Pos);  // PLLP={s['pll_p']}")

        if s['pll_q']:
            lines.append(f"RCC->PLLCFGR |= ({s['pll_q']} << RCC_PLLCFGR_PLLQ_Pos);  // PLLQ={s['pll_q']} (48MHz)")
    elif "F1" in s['stm32']:
        lines.append("/* F1系列PLL配置 */")
        lines.append(f"RCC->CFGR |= RCC_CFGR_PLLSRC;  // PLL源: HSE")
        lines.append(f"RCC->CFGR |= RCC_CFGR_PLLMULL{s['pll_n']}X;  // 倍频x{s['pll_n']}")

    lines.append("")
    lines.append("/* Flash等待周期 */")
    lines.append(f"FLASH->ACR |= FLASH_ACR_LATENCY{s['flash_wait_states']}WS;")
    lines.append("FLASH->ACR |= FLASH_ACR_PRFTEN;  // 使能预取")
    lines.append("FLASH->ACR |= FLASH_ACR_ICEN;    // 使能指令缓存")
    lines.append("FLASH->ACR |= FLASH_ACR_DCEN;    // 使能数据缓存")
    lines.append("")
    lines.append("/* AHB/APB分频 */")
    ahb_map = {1: 0, 2: 8, 4: 9, 8: 10, 16: 11, 64: 14, 128: 15, 256: 16, 512: 17}
    apb_map = {1: 0, 2: 4, 4: 5, 8: 6, 16: 7}

    lines.append(f"RCC->CFGR |= RCC_CFGR_HPRE_DIV{s['ahb_prescaler']};  // AHB={format_freq(s['ahb_freq'])}")
    if s['apb1_prescaler'] > 1:
        lines.append(f"RCC->CFGR |= RCC_CFGR_PPRE1_DIV{s['apb1_prescaler']};  // APB1={format_freq(s['apb1_freq'])}")
    if s['apb2_prescaler'] > 1:
        lines.append(f"RCC->CFGR |= RCC_CFGR_PPRE2_DIV{s['apb2_prescaler']};  // APB2={format_freq(s['apb2_freq'])}")
    lines.append("")
    lines.append("/* 启动PLL */")
    lines.append("RCC->CR |= RCC_CR_PLLON;")
    lines.append("while(!(RCC->CR & RCC_CR_PLLRDY));")
    lines.append("")
    lines.append("/* 切换系统时钟到PLL */")
    lines.append("RCC->CFGR |= RCC_CFGR_SW_PLL;")
    lines.append("while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);")

    return '\n'.join(lines)


# ============================================================
# DDS计算
# ============================================================

def calculate_dds_freq_word(chip, target_freq, mclk=None):
    """
    计算DDS频率控制字
    参数:
      chip: DDS芯片型号
      target_freq: 目标输出频率(Hz)
      mclk: 主时钟频率(Hz)，为None则使用最大值
    返回: 配置字典
    """
    dds = DDS_CHIPS.get(chip)
    if not dds:
        print(f"[错误] 不支持的DDS芯片: {chip}")
        print(f"  支持: {', '.join(DDS_CHIPS.keys())}")
        return None

    mclk_used = mclk if mclk else dds['mclk_max']

    if target_freq > dds['output_max']:
        print(f"[警告] 目标频率 {format_freq(target_freq)} 超过 {chip} 最大输出 {format_freq(dds['output_max'])}")

    if mclk_used > dds['mclk_max']:
        print(f"[警告] 主时钟 {format_freq(mclk_used)} 超过 {chip} 最大值 {format_freq(dds['mclk_max'])}")

    # 频率控制字 = target_freq * 2^N / MCLK
    freq_resolution = mclk_used / (2 ** dds['freq_bits'])
    freq_word = round(target_freq * (2 ** dds['freq_bits']) / mclk_used)

    # 实际输出频率
    actual_freq = freq_word * mclk_used / (2 ** dds['freq_bits'])
    error = abs(actual_freq - target_freq)
    error_pct = error / target_freq * 100 if target_freq > 0 else 0

    # 频率分辨率
    tuning_resolution = mclk_used / (2 ** dds['freq_bits'])

    # Nyquist限制
    nyquist_limit = mclk_used / 2
    practical_limit = mclk_used / 2.5  # 实际建议限制

    # 相位字计算(360°映射到N位)
    phase_word_90 = round(90 * (2 ** dds['phase_bits']) / 360)
    phase_word_180 = round(180 * (2 ** dds['phase_bits']) / 360)

    result = {
        "chip": chip,
        "mclk": mclk_used,
        "target_freq": target_freq,
        "actual_freq": actual_freq,
        "freq_word": freq_word,
        "freq_word_hex": f"0x{freq_word:0{(dds['freq_bits']+3)//4}X}",
        "freq_resolution": tuning_resolution,
        "error_hz": error,
        "error_pct": error_pct,
        "nyquist_limit": nyquist_limit,
        "practical_limit": practical_limit,
        "phase_word_90": phase_word_90,
        "phase_word_180": phase_word_180,
        "dac_bits": dds['dac_bits'],
        "sfdr_estimate": 6.02 * dds['dac_bits'] + 1.76,  # 理论SFDR(dBc)
    }

    return result


def generate_dds_init_code(chip, result):
    """生成DDS初始化代码"""
    lines = []
    dds = DDS_CHIPS.get(chip)

    if chip == "AD9833":
        lines.append("/* AD9833 初始化与频率设置 */")
        lines.append(f"/* 目标频率: {format_freq(result['target_freq'])} */")
        lines.append(f"/* 实际频率: {format_freq(result['actual_freq'])} */")
        lines.append(f"/* 频率字: {result['freq_word_hex']} */")
        lines.append("")
        lines.append("// AD9833 控制字: B28=1, FSELECT=0, PSELECT=0, RESET=1")
        lines.append("#define AD9833_CTRL_B28      (1 << 13)")
        lines.append("#define AD9833_CTRL_RESET    (1 << 8)")
        lines.append("")
        lines.append("void AD9833_SetFreq(uint32_t freq_word) {")
        lines.append("    // 控制字: B28=1, RESET=1")
        lines.append("    SPI_Write16(AD9833_CTRL_B28 | AD9833_CTRL_RESET);")
        lines.append(f"    // FREQ0 LSB: {result['freq_word'] & 0x3FFF:#06x}")
        lines.append(f"    SPI_Write16(0x4000 | (freq_word & 0x3FFF));")
        lines.append(f"    // FREQ0 MSB: {(result['freq_word'] >> 14) & 0x3FFF:#06x}")
        lines.append(f"    SPI_Write16(0x4000 | ((freq_word >> 14) & 0x3FFF));")
        lines.append("    // 取消RESET，开始输出")
        lines.append("    SPI_Write16(AD9833_CTRL_B28);")
        lines.append("}")

    elif chip == "AD9850":
        lines.append("/* AD9850 初始化与频率设置 */")
        lines.append(f"/* 目标频率: {format_freq(result['target_freq'])} */")
        lines.append(f"/* 实际频率: {format_freq(result['actual_freq'])} */")
        lines.append(f"/* 频率字: {result['freq_word_hex']} */")
        lines.append("")
        lines.append("void AD9850_SetFreq(uint32_t freq_word, uint8_t phase) {")
        lines.append("    // W0: 相位 + 控制位")
        lines.append(f"    uint8_t w0 = (phase << 3) | 0x00;  // 相位字, power-down=0")
        lines.append(f"    // W1-W4: 频率字 (LSB first)")
        lines.append(f"    uint8_t w1 = (freq_word >> 0) & 0xFF;")
        lines.append(f"    uint8_t w2 = (freq_word >> 8) & 0xFF;")
        lines.append(f"    uint8_t w3 = (freq_word >> 16) & 0xFF;")
        lines.append(f"    uint8_t w4 = (freq_word >> 24) & 0xFF;")
        lines.append("    // 串行写入 (先W0)")
        lines.append("    AD9850_Write(w0); AD9850_Write(w1);")
        lines.append("    AD9850_Write(w2); AD9850_Write(w3); AD9850_Write(w4);")
        lines.append("    // FQ_UP脉冲更新频率")
        lines.append("    AD9850_FQUP();")
        lines.append("}")

    return '\n'.join(lines)


# ============================================================
# 时钟树分析
# ============================================================

def analyze_clock_tree(stm32_series, sysclk, peripherals=None):
    """
    分析时钟树配置
    参数:
      stm32_series: STM32系列
      sysclk: 系统时钟频率
      peripherals: 外设列表 [(外设名, 期望频率)]
    """
    params = STM32_CLOCKS.get(stm32_series)
    if not params:
        return None

    result = {
        "stm32": stm32_series,
        "sysclk": sysclk,
        "bus_clocks": {},
        "peripheral_clocks": {},
    }

    # 计算总线时钟
    if sysclk <= params['apb1_max']:
        apb1_div = 1
    else:
        for div in [2, 4, 8, 16]:
            if sysclk / div <= params['apb1_max']:
                apb1_div = div
                break

    if sysclk <= params['apb2_max']:
        apb2_div = 1
    else:
        for div in [2, 4, 8, 16]:
            if sysclk / div <= params['apb2_max']:
                apb2_div = div
                break

    apb1_freq = sysclk / apb1_div
    apb2_freq = sysclk / apb2_div

    result['bus_clocks'] = {
        'AHB': sysclk,
        'APB1': apb1_freq,
        'APB2': apb2_freq,
        'APB1_timer': apb1_freq * 2 if apb1_div > 1 else apb1_freq,
        'APB2_timer': apb2_freq * 2 if apb2_div > 1 else apb2_freq,
    }

    # 外设时钟分析
    if peripherals:
        for name, target_freq in peripherals:
            periph_info = {"target": target_freq, "achievable": False, "notes": ""}

            if "UART" in name.upper() or "USART" in name.upper():
                # UART挂在APB1或APB2
                bus = "APB1" if "1" in name or "2" in name else "APB2"
                if "3" in name or "4" in name or "5" in name:
                    bus = "APB1"
                elif "1" in name and "1" not in name[1:]:
                    bus = "APB2"

                bus_freq = result['bus_clocks'].get(bus, apb1_freq)
                # UART波特率计算
                brr = round(bus_freq / target_freq)
                actual_baud = bus_freq / brr
                error = abs(actual_baud - target_freq) / target_freq * 100
                periph_info["bus"] = bus
                periph_info["brr"] = brr
                periph_info["actual_freq"] = actual_baud
                periph_info["error_pct"] = error
                periph_info["achievable"] = error < 2.0
                if error > 2.0:
                    periph_info["notes"] = f"波特率误差 {error:.2f}% 过大"

            elif "TIM" in name.upper():
                bus = "APB2" if "1" in name or "8" in name else "APB1"
                timer_clk = result['bus_clocks']['APB2_timer'] if bus == "APB2" else result['bus_clocks']['APB1_timer']
                psc_arr = find_psc_arr(timer_clk, target_freq)
                periph_info["bus"] = bus
                periph_info["timer_clk"] = timer_clk
                periph_info["psc"] = psc_arr[0]
                periph_info["arr"] = psc_arr[1]
                periph_info["actual_freq"] = timer_clk / ((psc_arr[0] + 1) * (psc_arr[1] + 1))
                periph_info["achievable"] = True

            elif "SPI" in name.upper():
                bus = "APB2" if "1" in name else "APB1"
                bus_freq = result['bus_clocks'][bus]
                div = 2
                while bus_freq / div > target_freq and div <= 256:
                    div *= 2
                periph_info["bus"] = bus
                periph_info["prescaler"] = div
                periph_info["actual_freq"] = bus_freq / div
                periph_info["achievable"] = True

            result['peripheral_clocks'][name] = periph_info

    return result


def find_psc_arr(timer_clk, target_freq, max_val=65536):
    """寻找最优PSC+ARR组合"""
    best_psc = 0
    best_arr = 1
    best_error = float('inf')

    for psc in range(0, 65536):
        arr = timer_clk / ((psc + 1) * target_freq) - 1
        if arr < 0 or arr > 65535:
            continue
        arr_int = round(arr)
        actual = timer_clk / ((psc + 1) * (arr_int + 1))
        error = abs(actual - target_freq)
        if error < best_error:
            best_error = error
            best_psc = psc
            best_arr = arr_int
        if error == 0:
            break

    return (best_psc, best_arr)


# ============================================================
# 格式化输出
# ============================================================

def format_pll_report(solutions, top_n=5):
    """格式化PLL计算报告"""
    lines = []
    lines.append("=" * 65)
    lines.append("             PLL频率规划报告")
    lines.append("=" * 65)

    if not solutions:
        lines.append("\n[未找到可行的PLL配置方案]")
        return '\n'.join(lines)

    for i, s in enumerate(solutions[:top_n]):
        lines.append(f"\n方案 {i+1}: {'★' if i == 0 else ' '}")
        lines.append(f"  输入源: HSE={format_freq(s['hse'])}")
        lines.append(f"  PLLM={s['pll_m']:3d}  PLLN={s['pll_n']:3d}  PLLP={s['pll_p']}  "
                     f"{'PLLQ='+str(s['pll_q'])+' (48MHz USB)' if s['pll_q'] else 'PLLQ=N/A'}")
        lines.append(f"  VCO输入: {format_freq(s['vco_in'])}")
        lines.append(f"  VCO输出: {format_freq(s['vco_out'])}")
        lines.append(f"  SYSCLK:  {format_freq(s['sysclk'])} (误差: {format_freq(s['error_hz'])}, {s['error_pct']:.4f}%)")
        lines.append(f"  AHB:     {format_freq(s['ahb_freq'])} (分频: /{s['ahb_prescaler']})")
        lines.append(f"  APB1:    {format_freq(s['apb1_freq'])} (分频: /{s['apb1_prescaler']})")
        lines.append(f"  APB2:    {format_freq(s['apb2_freq'])} (分频: /{s['apb2_prescaler']})")
        lines.append(f"  Flash:   {s['flash_wait_states']}个等待周期")

    lines.append("\n" + "=" * 65)
    return '\n'.join(lines)


def format_dds_report(chip, result):
    """格式化DDS计算报告"""
    lines = []
    dds = DDS_CHIPS.get(chip)
    lines.append("=" * 65)
    lines.append(f"             DDS频率规划报告 - {chip}")
    lines.append(f"             {dds['desc']}")
    lines.append("=" * 65)

    lines.append(f"\n■ 基本参数")
    lines.append(f"  主时钟(MCLK):     {format_freq(result['mclk'])}")
    lines.append(f"  频率寄存器位数:   {dds['freq_bits']}bit")
    lines.append(f"  DAC位数:          {dds['dac_bits']}bit")
    lines.append(f"  频率分辨率:       {format_freq(result['freq_resolution'])}")

    lines.append(f"\n■ 频率设置")
    lines.append(f"  目标频率:         {format_freq(result['target_freq'])}")
    lines.append(f"  实际频率:         {format_freq(result['actual_freq'])}")
    lines.append(f"  频率控制字:       {result['freq_word']} ({result['freq_word_hex']})")
    lines.append(f"  频率误差:         {format_freq(result['error_hz'])} ({result['error_pct']:.6f}%)")

    lines.append(f"\n■ 限制条件")
    lines.append(f"  Nyquist限制:      {format_freq(result['nyquist_limit'])}")
    lines.append(f"  建议最大输出:     {format_freq(result['practical_limit'])}")
    lines.append(f"  理论SFDR:         ~{result['sfdr_estimate']:.1f}dBc")

    lines.append(f"\n■ 相位控制字")
    lines.append(f"  90°相移:          {result['phase_word_90']} (0x{result['phase_word_90']:04X})")
    lines.append(f"  180°相移:         {result['phase_word_180']} (0x{result['phase_word_180']:04X})")

    lines.append("\n" + "=" * 65)
    return '\n'.join(lines)


def format_clock_tree_report(tree):
    """格式化时钟树报告"""
    lines = []
    lines.append("=" * 65)
    lines.append(f"             时钟树分析报告 - {tree['stm32']}")
    lines.append("=" * 65)

    lines.append(f"\n■ 系统时钟: {format_freq(tree['sysclk'])}")
    lines.append(f"\n■ 总线时钟:")
    for bus, freq in tree['bus_clocks'].items():
        lines.append(f"  {bus:<15s}: {format_freq(freq)}")

    if tree['peripheral_clocks']:
        lines.append(f"\n■ 外设时钟:")
        for name, info in tree['peripheral_clocks'].items():
            status = "✓" if info.get('achievable') else "✗"
            lines.append(f"  {status} {name}:")
            for k, v in info.items():
                if k not in ('achievable',) and v is not None:
                    if isinstance(v, float):
                        lines.append(f"      {k}: {format_freq(v)}" if v > 1000 else f"      {k}: {v}")
                    else:
                        lines.append(f"      {k}: {v}")

    lines.append("\n" + "=" * 65)
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(
        description='频率规划器 - PLL/DDS/时钟树配置计算',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # STM32F407 PLL计算 (目标168MHz)
  python frequency_planner.py pll --stm32 STM32F407 --target 168e6

  # STM32F103 PLL计算 (目标72MHz, 12MHz晶振)
  python frequency_planner.py pll --stm32 STM32F103 --target 72e6 --hse 12e6

  # AD9833 DDS计算 (1kHz输出, 25MHz时钟)
  python frequency_planner.py dds --chip AD9833 --target 1000 --mclk 25e6

  # AD9850 DDS计算 (10MHz输出)
  python frequency_planner.py dds --chip AD9850 --target 10e6

  # 时钟树分析
  python frequency_planner.py clock --stm32 STM32F407 --sysclk 168e6 \\
      --periph UART1:115200 TIM2:1000 SPI1:10e6

  # 生成PLL寄存器配置代码
  python frequency_planner.py pll --stm32 STM32F407 --target 168e6 --codegen

  # 列出支持的芯片
  python frequency_planner.py list
        """
    )

    subparsers = parser.add_subparsers(dest='command', help='子命令')

    # PLL计算
    pll_parser = subparsers.add_parser('pll', help='PLL参数计算')
    pll_parser.add_argument('--stm32', default='STM32F407',
                           choices=list(STM32_CLOCKS.keys()), help='STM32系列')
    pll_parser.add_argument('--target', type=float, required=True, help='目标系统时钟(Hz)')
    pll_parser.add_argument('--hse', type=float, help='外部晶振频率(Hz)')
    pll_parser.add_argument('--top', type=int, default=5, help='显示前N个方案')
    pll_parser.add_argument('--codegen', action='store_true', help='生成寄存器配置代码')
    pll_parser.add_argument('--output', '-o', help='输出文件路径')

    # DDS计算
    dds_parser = subparsers.add_parser('dds', help='DDS频率控制字计算')
    dds_parser.add_argument('--chip', default='AD9833',
                           choices=list(DDS_CHIPS.keys()), help='DDS芯片型号')
    dds_parser.add_argument('--target', type=float, required=True, help='目标输出频率(Hz)')
    dds_parser.add_argument('--mclk', type=float, help='主时钟频率(Hz)')
    dds_parser.add_argument('--codegen', action='store_true', help='生成初始化代码')
    dds_parser.add_argument('--output', '-o', help='输出文件路径')

    # 时钟树分析
    clock_parser = subparsers.add_parser('clock', help='时钟树分析')
    clock_parser.add_argument('--stm32', default='STM32F407',
                             choices=list(STM32_CLOCKS.keys()), help='STM32系列')
    clock_parser.add_argument('--sysclk', type=float, required=True, help='系统时钟(Hz)')
    clock_parser.add_argument('--periph', nargs='*', help='外设频率 (格式: NAME:FREQ)')
    clock_parser.add_argument('--output', '-o', help='输出文件路径')

    # 列出芯片
    list_parser = subparsers.add_parser('list', help='列出支持的芯片')

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    # 列出芯片
    if args.command == 'list':
        print("\n支持的STM32系列:")
        for name, params in STM32_CLOCKS.items():
            print(f"  {name:<12s}: SYSCLK最大 {format_freq(params['sysclk_max'])}, "
                  f"HSE典型 {format_freq(params['hse_typical'])}")
        print("\n支持的DDS芯片:")
        for name, dds in DDS_CHIPS.items():
            print(f"  {name:<10s}: MCLK最大 {format_freq(dds['mclk_max'])}, "
                  f"输出最大 {format_freq(dds['output_max'])}, {dds['desc']}")
        return

    # PLL计算
    if args.command == 'pll':
        solutions = calculate_stm32_pll(args.stm32, args.target, args.hse)
        report = format_pll_report(solutions, args.top)
        print(report)

        if args.codegen and solutions:
            code = generate_pll_register_config(solutions[0], "HSE" if args.hse else "HSI")
            print(f"\n{code}")

            if args.output:
                with open(args.output, 'w', encoding='utf-8') as f:
                    f.write(code)
                print(f"\n[OK] 代码已保存: {args.output}")

    # DDS计算
    elif args.command == 'dds':
        result = calculate_dds_freq_word(args.chip, args.target, args.mclk)
        if result:
            report = format_dds_report(args.chip, result)
            print(report)

            if args.codegen:
                code = generate_dds_init_code(args.chip, result)
                print(f"\n{code}")

                if args.output:
                    with open(args.output, 'w', encoding='utf-8') as f:
                        f.write(code)
                    print(f"\n[OK] 代码已保存: {args.output}")

    # 时钟树分析
    elif args.command == 'clock':
        peripherals = []
        if args.periph:
            for p in args.periph:
                parts = p.split(':')
                if len(parts) == 2:
                    peripherals.append((parts[0], float(parts[1])))

        tree = analyze_clock_tree(args.stm32, args.sysclk, peripherals)
        if tree:
            report = format_clock_tree_report(tree)
            print(report)


if __name__ == '__main__':
    main()
