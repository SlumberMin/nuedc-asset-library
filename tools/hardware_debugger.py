#!/usr/bin/env python3
"""
硬件调试工具 - GPIO/ADC/PWM/I2C/SPI在线调试
=============================================
功能：
  - GPIO引脚状态查看和控制
  - ADC采样和数据分析
  - PWM波形配置和输出
  - I2C设备扫描和寄存器读写
  - SPI数据传输调试
  - 引脚复用表查询
  - 电气参数计算（分压/电流/功耗）

用法：
  python hardware_debugger.py gpio --pin PA0 --mode output --set high
  python hardware_debugger.py adc --pin PA1 --bits 12 --ref 3.3
  python hardware_debugger.py pwm --pin PA8 --freq 1000 --duty 50
  python hardware_debugger.py i2c-scan --bus 1
  python hardware_debugger.py calc --type voltage-divider --r1 10000 --r2 10000 --vin 5.0
  python hardware_debugger.py pinout --chip STM32F103
"""

import argparse
import math
import sys
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field


# ============================================================
# MCU引脚数据库
# ============================================================

@dataclass
class PinInfo:
    """引脚信息"""
    name: str
    port: str          # GPIO端口 (A-K)
    pin_num: int       # 引脚号
    functions: List[str] = field(default_factory=list)   # 复用功能
    adc_channels: List[int] = field(default_factory=list) # ADC通道
    pwm_timers: List[str] = field(default_factory=list)   # PWM定时器
    voltage: float = 3.3  # 工作电压
    max_current_ma: float = 25.0  # 最大电流
    has_pullup: bool = False
    has_pulldown: bool = False


# STM32F103 引脚数据库（简化版，涵盖常用引脚）
STM32F103_PINS = {
    "PA0": PinInfo("PA0", "A", 0,
                   ["ADC1_CH0", "TIM2_CH1", "USART2_CTS", "WKUP"],
                   [0], ["TIM2_CH1"]),
    "PA1": PinInfo("PA1", "A", 1,
                   ["ADC1_CH1", "TIM2_CH2", "USART2_RTS"],
                   [1], ["TIM2_CH2"]),
    "PA2": PinInfo("PA2", "A", 2,
                   ["ADC1_CH2", "TIM2_CH3", "USART2_TX"],
                   [2], ["TIM2_CH3"]),
    "PA3": PinInfo("PA3", "A", 3,
                   ["ADC1_CH3", "TIM2_CH4", "USART2_RX"],
                   [3], ["TIM2_CH4"]),
    "PA4": PinInfo("PA4", "A", 4,
                   ["ADC1_CH4", "SPI1_NSS", "DAC_OUT1"],
                   [4], []),
    "PA5": PinInfo("PA5", "A", 5,
                   ["ADC1_CH5", "SPI1_SCK"],
                   [5], []),
    "PA6": PinInfo("PA6", "A", 6,
                   ["ADC1_CH6", "SPI1_MISO", "TIM3_CH1"],
                   [6], ["TIM3_CH1"]),
    "PA7": PinInfo("PA7", "A", 7,
                   ["ADC1_CH7", "SPI1_MOSI", "TIM3_CH2"],
                   [7], ["TIM3_CH2"]),
    "PA8": PinInfo("PA8", "A", 8,
                   ["TIM1_CH1", "USART1_CK"],
                   [], ["TIM1_CH1"]),
    "PA9": PinInfo("PA9", "A", 9,
                   ["USART1_TX", "TIM1_CH2"],
                   [], ["TIM1_CH2"]),
    "PA10": PinInfo("PA10", "A", 10,
                    ["USART1_RX", "TIM1_CH3"],
                    [], ["TIM1_CH3"]),
    "PA11": PinInfo("PA11", "A", 11,
                    ["USB_DM", "CAN_RX", "TIM1_CH4"],
                    [], ["TIM1_CH4"]),
    "PA12": PinInfo("PA12", "A", 12,
                    ["USB_DP", "CAN_TX"],
                    [], []),
    "PA13": PinInfo("PA13", "A", 13, ["SWDIO", "JTMS"]),
    "PA14": PinInfo("PA14", "A", 14, ["SWCLK", "JTCK"]),
    "PA15": PinInfo("PA15", "A", 15,
                    ["SPI3_NSS", "TIM2_CH1"],
                    [], ["TIM2_CH1_ETR"]),

    "PB0": PinInfo("PB0", "B", 0,
                   ["ADC1_CH8", "TIM3_CH3"],
                   [8], ["TIM3_CH3"]),
    "PB1": PinInfo("PB1", "B", 1,
                   ["ADC1_CH9", "TIM3_CH4"],
                   [9], ["TIM3_CH4"]),
    "PB3": PinInfo("PB3", "B", 3,
                   ["SPI3_SCK", "TIM2_CH2", "JTDO"]),
    "PB4": PinInfo("PB4", "B", 4,
                   ["SPI3_MISO", "NJTRST"]),
    "PB5": PinInfo("PB5", "B", 5,
                   ["SPI3_MOSI", "I2C1_SMBA"]),
    "PB6": PinInfo("PB6", "B", 6,
                   ["I2C1_SCL", "TIM4_CH1", "USART1_TX"],
                   [], ["TIM4_CH1"]),
    "PB7": PinInfo("PB7", "B", 7,
                   ["I2C1_SDA", "TIM4_CH2", "USART1_RX"],
                   [], ["TIM4_CH2"]),
    "PB8": PinInfo("PB8", "B", 8,
                   ["TIM4_CH3", "I2C1_SCL", "CAN_RX"],
                   [], ["TIM4_CH3"]),
    "PB9": PinInfo("PB9", "B", 9,
                   ["TIM4_CH4", "I2C1_SDA", "CAN_TX"],
                   [], ["TIM4_CH4"]),
    "PB10": PinInfo("PB10", "B", 10,
                    ["I2C2_SCL", "USART3_TX", "TIM2_CH3"],
                    [], ["TIM2_CH3"]),
    "PB11": PinInfo("PB11", "B", 11,
                    ["I2C2_SDA", "USART3_RX", "TIM2_CH4"],
                    [], ["TIM2_CH4"]),
    "PB12": PinInfo("PB12", "B", 12, ["SPI2_NSS", "USART3_CK"]),
    "PB13": PinInfo("PB13", "B", 13, ["SPI2_SCK", "USART3_CTS"]),
    "PB14": PinInfo("PB14", "B", 14, ["SPI2_MISO", "USART3_RTS"]),
    "PB15": PinInfo("PB15", "B", 15, ["SPI2_MOSI"]),

    "PC0": PinInfo("PC0", "C", 0, ["ADC1_CH10"], [10]),
    "PC1": PinInfo("PC1", "C", 1, ["ADC1_CH11"], [11]),
    "PC2": PinInfo("PC2", "C", 2, ["ADC1_CH12"], [12]),
    "PC3": PinInfo("PC3", "C", 3, ["ADC1_CH13"], [13]),
    "PC4": PinInfo("PC4", "C", 4, ["ADC1_CH14"], [14]),
    "PC5": PinInfo("PC5", "C", 5, ["ADC1_CH15"], [15]),
    "PC13": PinInfo("PC13", "C", 13, ["TAMPER", "LED"]),
    "PC14": PinInfo("PC14", "C", 14, ["OSC32_IN"]),
    "PC15": PinInfo("PC15", "C", 15, ["OSC32_OUT"]),
}

# STM32F407 额外引脚
STM32F407_EXTRA = {
    "PA0_F4": PinInfo("PA0", "A", 0, ["ADC123_IN0", "TIM2_CH1_ETR", "USART2_CTS"], [0]),
    "PE0": PinInfo("PE0", "E", 0, ["TIM4_ETR"]),
    "PE1": PinInfo("PE1", "E", 1),
}

# ESP32 引脚（简化）
ESP32_PINS = {
    "GPIO0": PinInfo("GPIO0", "0", 0, ["ADC2_CH1", "BOOT"], [1]),
    "GPIO1": PinInfo("GPIO1", "0", 1, ["TXD0"]),
    "GPIO2": PinInfo("GPIO2", "0", 2, ["ADC2_CH2", "HSPI_WP"], [2]),
    "GPIO3": PinInfo("GPIO3", "0", 3, ["RXD0"]),
    "GPIO4": PinInfo("GPIO4", "0", 4, ["ADC2_CH0", "HSPI_HD"], [0]),
    "GPIO5": PinInfo("GPIO5", "0", 5, ["VSPI_CS0"]),
    "GPIO12": PinInfo("GPIO12", "0", 12, ["ADC2_CH5", "HSPI_MISO", "MTDI"], [5]),
    "GPIO13": PinInfo("GPIO13", "0", 13, ["ADC2_CH4", "HSPI_MOSI", "MTCK"], [4]),
    "GPIO14": PinInfo("GPIO14", "0", 14, ["ADC2_CH6", "HSPI_CLK", "MTMS"], [6]),
    "GPIO15": PinInfo("GPIO15", "0", 15, ["ADC2_CH3", "HSPI_CS0", "MTDO"], [3]),
    "GPIO25": PinInfo("GPIO25", "0", 25, ["ADC2_CH8", "DAC1"], [8]),
    "GPIO26": PinInfo("GPIO26", "0", 26, ["ADC2_CH9", "DAC2"], [9]),
    "GPIO27": PinInfo("GPIO27", "0", 27, ["ADC2_CH7", "HSPI_DATA2"], [7]),
    "GPIO32": PinInfo("GPIO32", "0", 32, ["ADC1_CH4"], [4]),
    "GPIO33": PinInfo("GPIO33", "0", 33, ["ADC1_CH5"], [5]),
    "GPIO34": PinInfo("GPIO34", "0", 34, ["ADC1_CH6"], [6]),
    "GPIO35": PinInfo("GPIO35", "0", 35, ["ADC1_CH7"], [7]),
    "GPIO36": PinInfo("GPIO36", "0", 36, ["ADC1_CH0", "SVP"], [0]),
    "GPIO39": PinInfo("GPIO39", "0", 39, ["ADC1_CH3", "SVN"], [3]),
}


# ============================================================
# GPIO 调试器
# ============================================================

class GPIODebugger:
    """GPIO调试器"""

    # GPIO模式
    MODES = {
        "input":       "输入模式（浮空）",
        "input_pu":    "输入模式（上拉）",
        "input_pd":    "输入模式（下拉）",
        "output_pp":   "推挽输出",
        "output_od":   "开漏输出",
        "af_pp":       "复用推挽",
        "af_od":       "复用开漏",
        "analog":      "模拟模式",
    }

    @staticmethod
    def analyze_pin(pin_name: str, chip: str = "STM32F103") -> dict:
        """分析引脚功能"""
        pin_db = STM32F103_PINS if chip.startswith("STM32") else ESP32_PINS
        pin = pin_db.get(pin_name)

        if pin is None:
            return {"error": f"引脚 {pin_name} 未在 {chip} 数据库中找到"}

        return {
            "引脚": pin.name,
            "端口": pin.port,
            "引脚号": pin.pin_num,
            "复用功能": ", ".join(pin.functions) if pin.functions else "无",
            "ADC通道": ", ".join(f"CH{ch}" for ch in pin.adc_channels) if pin.adc_channels else "无",
            "PWM定时器": ", ".join(pin.pwm_timers) if pin.pwm_timers else "无",
            "工作电压": f"{pin.voltage}V",
            "最大电流": f"{pin.max_current_ma}mA",
        }

    @staticmethod
    def check_conflict(pin_name: str, function1: str, function2: str,
                       chip: str = "STM32F103") -> dict:
        """检查引脚功能冲突"""
        pin_db = STM32F103_PINS if chip.startswith("STM32") else ESP32_PINS
        pin = pin_db.get(pin_name)

        if pin is None:
            return {"error": f"引脚 {pin_name} 未找到"}

        has_f1 = any(function1.upper() in f.upper() for f in pin.functions)
        has_f2 = any(function2.upper() in f.upper() for f in pin.functions)

        if has_f1 and has_f2:
            return {
                "conflict": True,
                "message": f"⚠ {pin_name} 同时支持 {function1} 和 {function2}，"
                          f"不可同时使用！",
                "suggestion": "请将其中一个功能分配到其他引脚",
            }
        else:
            return {
                "conflict": False,
                "message": f"✓ {pin_name} 的 {function1} 和 {function2} 无冲突",
            }

    @staticmethod
    def suggest_pins(function: str, chip: str = "STM32F103") -> List[str]:
        """根据功能推荐引脚"""
        pin_db = STM32F103_PINS if chip.startswith("STM32") else ESP32_PINS
        matches = []
        for name, pin in pin_db.items():
            if any(function.upper() in f.upper() for f in pin.functions):
                matches.append(name)
        return matches


# ============================================================
# ADC 调试器
# ============================================================

class ADCDebugger:
    """ADC调试器"""

    @staticmethod
    def calculate(adc_value: int, bits: int = 12, vref: float = 3.3,
                  gain: float = 1.0, offset: float = 0.0) -> dict:
        """ADC值转换计算"""
        max_val = (1 << bits) - 1
        voltage_raw = adc_value * vref / max_val
        voltage_actual = (voltage_raw - offset) * gain
        percentage = adc_value / max_val * 100
        resolution = vref / max_val

        return {
            "ADC值": adc_value,
            "位数": bits,
            "满量程": max_val,
            "参考电压": f"{vref}V",
            "原始电压": f"{voltage_raw:.6f}V",
            "实际电压": f"{voltage_actual:.6f}V",
            "百分比": f"{percentage:.2f}%",
            "分辨率": f"{resolution*1000:.3f}mV",
            "有效位数": f"{bits} bit",
        }

    @staticmethod
    def sampling_analysis(signal_freq: float, sample_rate: float) -> dict:
        """采样参数分析"""
        nyquist = sample_rate / 2
        aliasing = signal_freq > nyquist
        oversampling_ratio = sample_rate / signal_freq

        result = {
            "信号频率": f"{signal_freq} Hz",
            "采样率": f"{sample_rate} Hz",
            "奈奎斯特频率": f"{nyquist} Hz",
            "混叠": "⚠ 会混叠！" if aliasing else "✓ 无混叠",
            "过采样率": f"{oversampling_ratio:.1f}x",
        }

        if aliasing:
            alias_freq = abs(sample_rate - signal_freq)
            result["混叠频率"] = f"{alias_freq:.1f} Hz"
            result["建议采样率"] = f"> {signal_freq * 2:.0f} Hz (至少2倍)"

        # 推荐过采样
        if oversampling_ratio < 10:
            result["建议"] = f"建议采样率 ≥ {signal_freq * 10:.0f} Hz (10倍信号频率)"

        return result

    @staticmethod
    def noise_analysis(values: List[float], vref: float = 3.3,
                       bits: int = 12) -> dict:
        """ADC噪声分析"""
        n = len(values)
        if n < 2:
            return {"error": "至少需要2个采样值"}

        mean = sum(values) / n
        variance = sum((x - mean) ** 2 for x in values) / (n - 1)
        std = math.sqrt(variance)
        max_val = max(values)
        min_val = min(values)
        pk_pk = max_val - min_val
        resolution = vref / ((1 << bits) - 1)
        enob = math.log2(vref / (std * math.sqrt(12))) if std > 0 else bits
        snr = 20 * math.log10(mean / std) if std > 0 and mean > 0 else float('inf')
        sinad = 20 * math.log10(mean / std) if std > 0 and mean > 0 else float('inf')

        return {
            "样本数": n,
            "均值": f"{mean:.6f} V",
            "标准差": f"{std:.6f} V",
            "峰峰值": f"{pk_pk:.6f} V",
            "最大值": f"{max_val:.6f} V",
            "最小值": f"{min_val:.6f} V",
            "LSB分辨率": f"{resolution*1000:.3f} mV",
            "噪声LSB": f"{std/resolution:.2f} LSB",
            "ENOB": f"{min(enob, bits):.1f} bit",
            "SNR": f"{snr:.1f} dB" if snr != float('inf') else "∞",
            "建议": "噪声较大，建议增加硬件滤波或软件滑动平均" if std > resolution * 2 else "噪声水平正常",
        }

    @staticmethod
    def oversampling_benefit(target_bits: int, current_bits: int) -> dict:
        """过采样增益计算"""
        extra_bits = target_bits - current_bits
        if extra_bits <= 0:
            return {"message": f"当前 {current_bits}bit 已满足 {target_bits}bit 需求"}
        oversample_ratio = 4 ** extra_bits
        return {
            "当前分辨率": f"{current_bits} bit",
            "目标分辨率": f"{target_bits} bit",
            "需要额外位数": extra_bits,
            "过采样倍数": oversample_ratio,
            "实际采样率要求": f"信号带宽 × {oversample_ratio}",
        }


# ============================================================
# PWM 调试器
# ============================================================

class PWMDebugger:
    """PWM调试器"""

    @staticmethod
    def calculate(timer_clock: float, prescaler: int, period: int) -> dict:
        """PWM频率计算"""
        freq = timer_clock / (prescaler * period)
        resolution_bits = math.log2(period) if period > 0 else 0
        return {
            "定时器时钟": f"{timer_clock/1e6:.1f} MHz",
            "预分频器": prescaler,
            "自动重载值": period,
            "PWM频率": f"{freq:.2f} Hz",
            "PWM周期": f"{1000/freq:.4f} ms" if freq > 0 else "∞",
            "分辨率": f"{resolution_bits:.1f} bit ({period}级)",
        }

    @staticmethod
    def duty_cycle(pulse: int, period: int) -> dict:
        """占空比计算"""
        if period <= 0:
            return {"error": "周期值必须大于0"}
        duty = pulse / period * 100
        return {
            "脉冲值": pulse,
            "周期值": period,
            "占空比": f"{duty:.2f}%",
            "高电平时间": f"周期×{pulse}/{period}",
        }

    @staticmethod
    def led_brightness(duty: float, led_vf: float = 2.0,
                       supply: float = 5.0, resistor: float = 220) -> dict:
        """LED亮度估算"""
        current_ma = (supply - led_vf) / resistor * 1000
        avg_current = current_ma * duty / 100
        power_mw = led_vf * avg_current

        return {
            "占空比": f"{duty:.1f}%",
            "LED正向压降": f"{led_vf}V",
            "峰值电流": f"{current_ma:.1f}mA",
            "平均电流": f"{avg_current:.2f}mA",
            "功耗": f"{power_mw:.2f}mW",
            "亮度感知": "人眼感知与占空比近似线性（>100Hz时）",
        }

    @staticmethod
    def motor_speed(voltage: float, duty: float, motor_kv: float = 1000) -> dict:
        """电机速度估算"""
        effective_voltage = voltage * duty / 100
        rpm = effective_voltage * motor_kv

        return {
            "供电电压": f"{voltage}V",
            "占空比": f"{duty:.1f}%",
            "等效电压": f"{effective_voltage:.2f}V",
            "估算转速": f"{rpm:.0f} RPM",
            "KV值": f"{motor_kv} RPM/V",
        }


# ============================================================
# 电气参数计算器
# ============================================================

class ElectricalCalculator:
    """电气参数计算器"""

    @staticmethod
    def voltage_divider(r1: float, r2: float, vin: float) -> dict:
        """分压器计算"""
        vout = vin * r2 / (r1 + r2)
        current = vin / (r1 + r2) * 1000  # mA
        power_r1 = (vin - vout) ** 2 / r1 * 1000  # mW
        power_r2 = vout ** 2 / r2 * 1000  # mW

        return {
            "R1": f"{r1} Ω",
            "R2": f"{r2} Ω",
            "输入电压": f"{vin} V",
            "输出电压": f"{vout:.4f} V",
            "分压比": f"{r2/(r1+r2):.4f}",
            "电流": f"{current:.3f} mA",
            "R1功耗": f"{power_r1:.3f} mW",
            "R2功耗": f"{power_r2:.3f} mW",
        }

    @staticmethod
    def led_resistor(vsupply: float, vf: float, if_ma: float) -> dict:
        """LED限流电阻计算"""
        r = (vsupply - vf) / (if_ma / 1000)
        power = (vsupply - vf) * (if_ma / 1000) * 1000  # mW

        # 选择标准电阻
        standard = [10, 22, 33, 47, 68, 100, 150, 220, 330, 470,
                    680, 1000, 1500, 2200, 3300, 4700, 10000]
        r_standard = min(standard, key=lambda x: abs(x - r))
        actual_if = (vsupply - vf) / r_standard * 1000

        return {
            "供电电压": f"{vsupply} V",
            "LED正向压降": f"{vf} V",
            "目标电流": f"{if_ma} mA",
            "计算电阻": f"{r:.1f} Ω",
            "标准电阻": f"{r_standard} Ω",
            "实际电流": f"{actual_if:.2f} mA",
            "电阻功耗": f"{power:.2f} mW",
            "建议功率": f"≥ {math.ceil(power/100)*100} mW (推荐1/4W)",
        }

    @staticmethod
    def power_analysis(voltage: float, current_ma: float) -> dict:
        """功耗分析"""
        power_mw = voltage * current_ma
        power_w = power_mw / 1000

        return {
            "电压": f"{voltage} V",
            "电流": f"{current_ma} mA",
            "功耗": f"{power_mw:.2f} mW ({power_w:.4f} W)",
            "热阻估算": f"{250/power_w:.1f} °C/W (若结温升25°C)",
        }

    @staticmethod
    def capacitor_charge(capacitance_uf: float, resistance_ohm: float,
                         voltage: float, time_s: float) -> dict:
        """RC充电计算"""
        tau = resistance_ohm * capacitance_uf * 1e-6  # 时间常数
        v_charge = voltage * (1 - math.exp(-time_s / tau))
        percent = (1 - math.exp(-time_s / tau)) * 100

        return {
            "电容": f"{capacitance_uf} μF",
            "电阻": f"{resistance_ohm} Ω",
            "时间常数τ": f"{tau*1000:.3f} ms",
            "充电时间": f"{time_s*1000:.2f} ms",
            "充电电压": f"{v_charge:.4f} V",
            "充电百分比": f"{percent:.2f}%",
            "充满时间(5τ)": f"{5*tau*1000:.2f} ms",
        }

    @staticmethod
    def opamp_gain(rf: float, rin: float, topology: str = "inverting") -> dict:
        """运放增益计算"""
        if topology == "inverting":
            gain = -rf / rin
            gain_db = 20 * math.log10(abs(gain))
            return {
                "拓扑": "反相放大器",
                "Rf": f"{rf} Ω",
                "Rin": f"{rin} Ω",
                "增益": f"{gain:.2f} ({gain_db:.1f} dB)",
                "输入阻抗": f"{rin} Ω",
                "相位": "反相 (180°)",
            }
        elif topology == "non_inverting":
            gain = 1 + rf / rin
            gain_db = 20 * math.log10(gain)
            return {
                "拓扑": "同相放大器",
                "Rf": f"{rf} Ω",
                "Rin": f"{rin} Ω",
                "增益": f"{gain:.2f} ({gain_db:.1f} dB)",
                "输入阻抗": "极高 (>MΩ)",
                "相位": "同相 (0°)",
            }
        else:
            return {"error": f"未知拓扑: {topology}"}


# ============================================================
# CLI 接口
# ============================================================

def cmd_gpio(args):
    """GPIO调试"""
    print(f"\n  GPIO 调试")
    print("  " + "=" * 40)

    info = GPIODebugger.analyze_pin(args.pin, args.chip)
    for k, v in info.items():
        print(f"  {k}: {v}")

    if args.mode:
        mode_desc = GPIODebugger.MODES.get(args.mode, args.mode)
        print(f"\n  设置模式: {mode_desc}")

    if args.set:
        state = "HIGH (3.3V)" if args.set.lower() == "high" else "LOW (0V)"
        print(f"  输出状态: {state}")

    if args.function:
        matches = GPIODebugger.suggest_pins(args.function, args.chip)
        if matches:
            print(f"\n  支持 {args.function} 的引脚:")
            for m in matches:
                print(f"    {m}")
        else:
            print(f"  未找到支持 {args.function} 的引脚")


def cmd_adc(args):
    """ADC调试"""
    print(f"\n  ADC 调试")
    print("  " + "=" * 40)

    result = ADCDebugger.calculate(args.value, args.bits, args.ref, args.gain, args.offset)
    for k, v in result.items():
        print(f"  {k}: {v}")


def cmd_pwm(args):
    """PWM调试"""
    print(f"\n  PWM 调试")
    print("  " + "=" * 40)

    # 计算PWM参数
    result = PWMDebugger.calculate(args.timer_clock, args.prescaler, args.period)
    for k, v in result.items():
        print(f"  {k}: {v}")

    # 占空比
    if args.duty is not None:
        pulse = int(args.period * args.duty / 100)
        dc_result = PWMDebugger.duty_cycle(pulse, args.period)
        print(f"\n  占空比设置:")
        for k, v in dc_result.items():
            print(f"  {k}: {v}")


def cmd_i2c_scan(args):
    """I2C扫描"""
    print(f"\n  I2C 设备扫描 (模拟)")
    print("  " + "=" * 40)
    print(f"  总线: I2C{args.bus}")
    print(f"  速率: {args.speed} Hz")
    print()

    # 模拟扫描结果
    known = {
        0x23: "BH1750 光照传感器",
        0x27: "PCF8574 LCD模块",
        0x3C: "SSD1306 OLED",
        0x48: "ADS1115 ADC",
        0x50: "AT24C256 EEPROM",
        0x68: "MPU6050/DS3231",
        0x76: "BMP280",
    }

    found = 0
    print(f"  地址(7bit)  地址(R/W)   设备")
    print(f"  {'-' * 45}")
    for addr, name in sorted(known.items()):
        print(f"  0x{addr:02X}        "
              f"0x{addr*2:02X}/0x{addr*2+1:02X}   {name}")
        found += 1

    print(f"\n  扫描完成，发现 {found} 个设备")
    print(f"  注意: 这是模拟数据，实际使用需要连接硬件")


def cmd_calc(args):
    """电气计算"""
    print(f"\n  电气参数计算: {args.type}")
    print("  " + "=" * 40)

    if args.type == "voltage-divider":
        result = ElectricalCalculator.voltage_divider(args.r1, args.r2, args.vin)
    elif args.type == "led-resistor":
        result = ElectricalCalculator.led_resistor(args.vin, args.vf, args.current)
    elif args.type == "capacitor":
        result = ElectricalCalculator.capacitor_charge(args.cap, args.r1, args.vin, args.time)
    elif args.type == "opamp":
        result = ElectricalCalculator.opamp_gain(args.r1, args.r2, args.topology)
    elif args.type == "adc":
        bits = int(args.vf) if args.vf else 12  # 复用参数
        result = ADCDebugger.calculate(int(args.vin), bits, args.r1)
    else:
        print(f"  未知计算类型: {args.type}")
        return

    for k, v in result.items():
        print(f"  {k}: {v}")


def cmd_pinout(args):
    """引脚表查询"""
    print(f"\n  {args.chip} 引脚复用表")
    print("  " + "=" * 60)

    pin_db = STM32F103_PINS if args.chip.startswith("STM32") else ESP32_PINS

    if args.filter:
        filtered = {k: v for k, v in pin_db.items()
                    if args.filter.upper() in k.upper() or
                    any(args.filter.upper() in f.upper() for f in v.functions)}
        pin_db = filtered

    print(f"  {'引脚':>6s} {'ADC':>8s} {'PWM':>12s} {'功能':}")
    print(f"  {'-' * 60}")

    for name, pin in sorted(pin_db.items()):
        adc = ", ".join(f"CH{ch}" for ch in pin.adc_channels) if pin.adc_channels else "-"
        pwm = ", ".join(pin.pwm_timers) if pin.pwm_timers else "-"
        funcs = ", ".join(pin.functions[:3])
        if len(pin.functions) > 3:
            funcs += "..."
        print(f"  {name:>6s} {adc:>8s} {pwm:>12s}  {funcs}")


def main():
    parser = argparse.ArgumentParser(
        description="硬件调试工具 - GPIO/ADC/PWM/I2C/SPI在线调试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s gpio --pin PA0 --mode output --set high
  %(prog)s adc --value 2048 --bits 12 --ref 3.3
  %(prog)s pwm --timer-clock 72000000 --prescaler 72 --period 1000 --duty 50
  %(prog)s calc --type voltage-divider --r1 10000 --r2 10000 --vin 5.0
  %(prog)s pinout --chip STM32F103 --filter ADC
        """
    )

    subparsers = parser.add_subparsers(dest="command", help="功能")

    # GPIO
    gpio_p = subparsers.add_parser("gpio", help="GPIO调试")
    gpio_p.add_argument("--pin", required=True, help="引脚名 (如 PA0)")
    gpio_p.add_argument("--chip", default="STM32F103", help="芯片型号")
    gpio_p.add_argument("--mode", choices=list(GPIODebugger.MODES.keys()), help="GPIO模式")
    gpio_p.add_argument("--set", choices=["high", "low"], help="输出电平")
    gpio_p.add_argument("--function", help="查询功能引脚")

    # ADC
    adc_p = subparsers.add_parser("adc", help="ADC调试")
    adc_p.add_argument("--value", type=int, required=True, help="ADC值")
    adc_p.add_argument("--bits", type=int, default=12, help="ADC位数")
    adc_p.add_argument("--ref", type=float, default=3.3, help="参考电压")
    adc_p.add_argument("--gain", type=float, default=1.0, help="增益")
    adc_p.add_argument("--offset", type=float, default=0.0, help="偏移")

    # PWM
    pwm_p = subparsers.add_parser("pwm", help="PWM调试")
    pwm_p.add_argument("--timer-clock", type=float, default=72e6, help="定时器时钟(Hz)")
    pwm_p.add_argument("--prescaler", type=int, default=72, help="预分频器")
    pwm_p.add_argument("--period", type=int, default=1000, help="自动重载值")
    pwm_p.add_argument("--duty", type=float, help="占空比(%)")
    pwm_p.add_argument("--pin", help="引脚名")

    # I2C扫描
    i2c_p = subparsers.add_parser("i2c-scan", help="I2C设备扫描")
    i2c_p.add_argument("--bus", type=int, default=1, help="I2C总线号")
    i2c_p.add_argument("--speed", type=int, default=100000, help="I2C速率")

    # 电气计算
    calc_p = subparsers.add_parser("calc", help="电气参数计算")
    calc_p.add_argument("--type", required=True,
                        choices=["voltage-divider", "led-resistor", "capacitor",
                                 "opamp", "adc"],
                        help="计算类型")
    calc_p.add_argument("--r1", type=float, default=10000, help="电阻R1(Ω)")
    calc_p.add_argument("--r2", type=float, default=10000, help="电阻R2(Ω)")
    calc_p.add_argument("--vin", type=float, default=5.0, help="输入电压")
    calc_p.add_argument("--vf", type=float, default=2.0, help="LED正向压降")
    calc_p.add_argument("--current", type=float, default=20, help="目标电流(mA)")
    calc_p.add_argument("--cap", type=float, default=100, help="电容(μF)")
    calc_p.add_argument("--time", type=float, default=0.1, help="时间(s)")
    calc_p.add_argument("--topology", default="inverting",
                        choices=["inverting", "non_inverting"], help="运放拓扑")

    # 引脚表
    pin_p = subparsers.add_parser("pinout", help="引脚复用表查询")
    pin_p.add_argument("--chip", default="STM32F103", help="芯片型号")
    pin_p.add_argument("--filter", help="过滤关键字")

    args = parser.parse_args()

    if args.command == "gpio":
        cmd_gpio(args)
    elif args.command == "adc":
        cmd_adc(args)
    elif args.command == "pwm":
        cmd_pwm(args)
    elif args.command == "i2c-scan":
        cmd_i2c_scan(args)
    elif args.command == "calc":
        cmd_calc(args)
    elif args.command == "pinout":
        cmd_pinout(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
