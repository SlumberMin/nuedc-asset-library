#!/usr/bin/env python3
"""MCU选型工具 - 根据需求推荐最佳微控制器

用法:
    python mcu_selector.py --cpu-bit 32 --flash-min 256 --ram-min 64 --adc --dac --pwm --uart 2 --spi 1 --i2c 1 --can 1
    python mcu_selector.py --family STM32 --gpio-min 30 --voltage 3.3
    python mcu_selector.py --list-families
    python mcu_selector.py --interactive
"""

import argparse
import json
import sys

# ============================================================
# MCU数据库（含常用电赛MCU型号及其关键参数）
# ============================================================
MCU_DATABASE = [
    # ---- STM32 系列 ----
    {
        "型号": "STM32F103C8T6", "厂商": "ST", "系列": "STM32F1",
        "位宽": 32, "内核": "Cortex-M3", "主频MHz": 72,
        "FlashKB": 64, "RAMKB": 20,
        "GPIO": 37, "ADC通道": 10, "DAC": False,
        "PWM": 4, "UART": 3, "SPI": 2, "I2C": 2, "CAN": 1,
        "USB": True, "电压V": [2.0, 3.6],
        "封装": "LQFP48", "单价约": 5.0, "供货": "充足",
        "特点": "经典款，生态成熟，电赛首选"
    },
    {
        "型号": "STM32F103RCT6", "厂商": "ST", "系列": "STM32F1",
        "位宽": 32, "内核": "Cortex-M3", "主频MHz": 72,
        "FlashKB": 256, "RAMKB": 48,
        "GPIO": 51, "ADC通道": 16, "DAC": 2,
        "PWM": 8, "UART": 5, "SPI": 3, "I2C": 2, "CAN": 1,
        "USB": True, "电压V": [2.0, 3.6],
        "封装": "LQFP64", "单价约": 12.0, "供货": "充足",
        "特点": "资源丰富，适合复杂项目"
    },
    {
        "型号": "STM32F407VET6", "厂商": "ST", "系列": "STM32F4",
        "位宽": 32, "内核": "Cortex-M4F", "主频MHz": 168,
        "FlashKB": 512, "RAMKB": 192,
        "GPIO": 82, "ADC通道": 16, "DAC": 2,
        "PWM": 12, "UART": 6, "SPI": 3, "I2C": 3, "CAN": 2,
        "USB": True, "电压V": [1.8, 3.6],
        "封装": "LQFP100", "单价约": 25.0, "供货": "充足",
        "特点": "高性能，DSP/FPU，适合信号处理"
    },
    {
        "型号": "STM32F407ZET6", "厂商": "ST", "系列": "STM32F4",
        "位宽": 32, "内核": "Cortex-M4F", "主频MHz": 168,
        "FlashKB": 512, "RAMKB": 192,
        "GPIO": 114, "ADC通道": 24, "DAC": 2,
        "PWM": 14, "UART": 6, "SPI": 3, "I2C": 3, "CAN": 2,
        "USB": True, "电压V": [1.8, 3.6],
        "封装": "LQFP144", "单价约": 30.0, "供货": "充足",
        "特点": "引脚最多，适合引脚密集设计"
    },
    {
        "型号": "STM32G431CBT6", "厂商": "ST", "系列": "STM32G4",
        "位宽": 32, "内核": "Cortex-M4F", "主频MHz": 170,
        "FlashKB": 128, "RAMKB": 32,
        "GPIO": 30, "ADC通道": 19, "DAC": 4,
        "PWM": 12, "UART": 3, "SPI": 3, "I2C": 3, "CAN": 1,
        "USB": True, "电压V": [1.71, 3.6],
        "封装": "LQFP48", "单价约": 15.0, "供货": "充足",
        "特点": "模拟外设强，适合电源/电机控制"
    },
    {
        "型号": "STM32H743VIT6", "厂商": "ST", "系列": "STM32H7",
        "位宽": 32, "内核": "Cortex-M7", "主频MHz": 480,
        "FlashKB": 2048, "RAMKB": 1024,
        "GPIO": 82, "ADC通道": 16, "DAC": 2,
        "PWM": 16, "UART": 8, "SPI": 6, "I2C": 4, "CAN": 2,
        "USB": True, "电压V": [1.62, 3.6],
        "封装": "LQFP100", "单价约": 60.0, "供货": "一般",
        "特点": "顶级性能，适合图像处理/AI推理"
    },
    # ---- STM32 低功耗 ----
    {
        "型号": "STM32L431RCT6", "厂商": "ST", "系列": "STM32L4",
        "位宽": 32, "内核": "Cortex-M4F", "主频MHz": 80,
        "FlashKB": 256, "RAMKB": 64,
        "GPIO": 50, "ADC通道": 16, "DAC": 2,
        "PWM": 8, "UART": 3, "SPI": 3, "I2C": 3, "CAN": 1,
        "USB": True, "电压V": [1.71, 3.6],
        "封装": "LQFP64", "单价约": 18.0, "供货": "充足",
        "特点": "超低功耗，适合电池供电"
    },
    # ---- GD32 系列（国产替代）----
    {
        "型号": "GD32F103C8T6", "厂商": "兆易创新", "系列": "GD32F1",
        "位宽": 32, "内核": "Cortex-M3", "主频MHz": 108,
        "FlashKB": 64, "RAMKB": 20,
        "GPIO": 37, "ADC通道": 10, "DAC": 0,
        "PWM": 4, "UART": 3, "SPI": 2, "I2C": 2, "CAN": 1,
        "USB": True, "电压V": [2.6, 3.6],
        "封装": "LQFP48", "单价约": 4.0, "供货": "充足",
        "特点": "STM32国产替代，性价比高"
    },
    {
        "型号": "GD32F303CCT6", "厂商": "兆易创新", "系列": "GD32F3",
        "位宽": 32, "内核": "Cortex-M4F", "主频MHz": 120,
        "FlashKB": 256, "RAMKB": 48,
        "GPIO": 37, "ADC通道": 10, "DAC": 2,
        "PWM": 8, "UART": 3, "SPI": 2, "I2C": 2, "CAN": 1,
        "USB": True, "电压V": [2.6, 3.6],
        "封装": "LQFP48", "单价约": 8.0, "供货": "充足",
        "特点": "国产高性能，M4内核带FPU"
    },
    # ---- AT32 系列（雅特力）----
    {
        "型号": "AT32F403ACGT7", "厂商": "雅特力", "系列": "AT32F4",
        "位宽": 32, "内核": "Cortex-M4", "主频MHz": 240,
        "FlashKB": 256, "RAMKB": 96,
        "GPIO": 37, "ADC通道": 10, "DAC": 2,
        "PWM": 8, "UART": 4, "SPI": 2, "I2C": 2, "CAN": 2,
        "USB": True, "电压V": [2.4, 3.6],
        "封装": "LQFP48", "单价约": 10.0, "供货": "充足",
        "特点": "高主频国产MCU，性能强劲"
    },
    # ---- CH32 系列（沁恒）----
    {
        "型号": "CH32V307VCT6", "厂商": "沁恒", "系列": "CH32V",
        "位宽": 32, "内核": "RISC-V", "主频MHz": 144,
        "FlashKB": 256, "RAMKB": 64,
        "GPIO": 51, "ADC通道": 16, "DAC": 2,
        "PWM": 8, "UART": 5, "SPI": 3, "I2C": 2, "CAN": 2,
        "USB": True, "电压V": [2.7, 5.5],
        "封装": "LQFP100", "单价约": 12.0, "供货": "充足",
        "特点": "RISC-V架构，宽电压，以太网MAC"
    },
    # ---- ESP32 系列 ----
    {
        "型号": "ESP32-S3-WROOM-1", "厂商": "乐鑫", "系列": "ESP32S3",
        "位宽": 32, "内核": "Xtensa LX7", "主频MHz": 240,
        "FlashKB": 8192, "RAMKB": 512,
        "GPIO": 45, "ADC通道": 20, "DAC": 0,
        "PWM": 8, "UART": 3, "SPI": 4, "I2C": 2, "CAN": 2,
        "USB": True, "电压V": [3.0, 3.6],
        "封装": "模组", "单价约": 18.0, "供货": "充足",
        "特点": "WiFi+蓝牙，AI加速，适合物联网"
    },
    {
        "型号": "ESP32-C3-32S", "厂商": "乐鑫", "系列": "ESP32C3",
        "位宽": 32, "内核": "RISC-V", "主频MHz": 160,
        "FlashKB": 4096, "RAMKB": 400,
        "GPIO": 22, "ADC通道": 6, "DAC": 0,
        "PWM": 6, "UART": 2, "SPI": 3, "I2C": 1, "CAN": 1,
        "USB": True, "电压V": [3.0, 3.6],
        "封装": "模组", "单价约": 10.0, "供货": "充足",
        "特点": "低成本WiFi+BLE5，RISC-V"
    },
    # ---- MSP430 系列（TI）----
    {
        "型号": "MSP430F5529", "厂商": "TI", "系列": "MSP430",
        "位宽": 16, "内核": "MSP430", "主频MHz": 25,
        "FlashKB": 128, "RAMKB": 10,
        "GPIO": 47, "ADC通道": 12, "DAC": 0,
        "PWM": 4, "UART": 2, "SPI": 2, "I2C": 2, "CAN": 0,
        "USB": True, "电压V": [1.8, 3.6],
        "封装": "LQFP80", "单价约": 20.0, "供货": "一般",
        "特点": "超低功耗，适合计量/传感器节点"
    },
    # ---- 51系列 ----
    {
        "型号": "STC8H8K64U", "厂商": "STC", "系列": "STC8",
        "位宽": 8, "内核": "8051增强", "主频MHz": 36,
        "FlashKB": 64, "RAMKB": 8,
        "GPIO": 44, "ADC通道": 15, "DAC": 0,
        "PWM": 8, "UART": 4, "SPI": 2, "I2C": 1, "CAN": 0,
        "USB": True, "电压V": [1.9, 5.5],
        "封装": "LQFP44", "单价约": 3.5, "供货": "充足",
        "特点": "传统51增强版，宽电压，适合入门"
    },
    # ---- Arduino 兼容 ----
    {
        "型号": "ATmega328P", "厂商": "Microchip", "系列": "AVR",
        "位宽": 8, "内核": "AVR", "主频MHz": 20,
        "FlashKB": 32, "RAMKB": 2,
        "GPIO": 23, "ADC通道": 8, "DAC": 0,
        "PWM": 6, "UART": 1, "SPI": 1, "I2C": 1, "CAN": 0,
        "USB": False, "电压V": [1.8, 5.5],
        "封装": "TQFP32", "单价约": 15.0, "供货": "充足",
        "特点": "Arduino UNO核心，社区生态最大"
    },
    # ---- RP2040 ----
    {
        "型号": "RP2040", "厂商": "树莓派", "系列": "RP2",
        "位宽": 32, "内核": "Cortex-M0+", "主频MHz": 133,
        "FlashKB": 0, "RAMKB": 264,
        "GPIO": 30, "ADC通道": 4, "DAC": 0,
        "PWM": 8, "UART": 2, "SPI": 2, "I2C": 2, "CAN": 0,
        "USB": True, "电压V": [1.8, 5.5],
        "封装": "QFN56", "单价约": 5.0, "供货": "充足",
        "特点": "双核M0+，PIO状态机，社区活跃"
    },
]

# ============================================================
# MCU选型推荐引擎
# ============================================================

def find_best_mcu(requirements: dict) -> list:
    """根据需求筛选并排序MCU，返回按匹配度排序的列表"""
    results = []
    for mcu in MCU_DATABASE:
        score = 0
        reasons = []
        penalties = []

        # --- 硬性筛选（不满足直接排除）---
        if requirements.get("family") and requirements["family"].upper() not in mcu["系列"].upper() and requirements["family"].upper() not in mcu["型号"].upper():
            continue

        if requirements.get("cpu_bit") and mcu["位宽"] < requirements["cpu_bit"]:
            continue

        if requirements.get("flash_min") and mcu["FlashKB"] < requirements["flash_min"]:
            continue

        if requirements.get("ram_min") and mcu["RAMKB"] < requirements["ram_min"]:
            continue

        if requirements.get("gpio_min") and mcu["GPIO"] < requirements["gpio_min"]:
            continue

        if requirements.get("adc") and mcu["ADC通道"] < 1:
            continue

        if requirements.get("dac") and not mcu["DAC"]:
            continue

        if requirements.get("uart_min") and mcu["UART"] < requirements["uart_min"]:
            continue

        if requirements.get("spi_min") and mcu["SPI"] < requirements["spi_min"]:
            continue

        if requirements.get("i2c_min") and mcu["I2C"] < requirements["i2c_min"]:
            continue

        if requirements.get("can") and mcu["CAN"] < 1:
            continue

        if requirements.get("usb") and not mcu["USB"]:
            continue

        # 电压范围检查
        if requirements.get("voltage"):
            v = requirements["voltage"]
            if not (mcu["电压V"][0] <= v <= mcu["电压V"][1]):
                continue

        # --- 软性评分 ---
        if requirements.get("frequency_min"):
            if mcu["主频MHz"] >= requirements["frequency_min"]:
                score += 20
                reasons.append(f"主频{mcu['主频MHz']}MHz满足要求")
            else:
                penalties.append(f"主频{mcu['主频MHz']}MHz低于要求{requirements['frequency_min']}MHz")
                score -= 30

        # 位宽加分
        if mcu["位宽"] == 32:
            score += 15
            reasons.append("32位内核，性能充足")
        elif mcu["位宽"] == 16:
            score += 5

        # Flash余量加分
        if requirements.get("flash_min"):
            ratio = mcu["FlashKB"] / requirements["flash_min"]
            if ratio >= 2:
                score += 10
                reasons.append(f"Flash充裕({mcu['FlashKB']}KB)")
            elif ratio >= 1.5:
                score += 5

        # RAM余量加分
        if requirements.get("ram_min"):
            ratio = mcu["RAMKB"] / requirements["ram_min"]
            if ratio >= 2:
                score += 10
                reasons.append(f"RAM充裕({mcu['RAMKB']}KB)")

        # ADC通道加分
        if requirements.get("adc_channels_min") and mcu["ADC通道"] >= requirements["adc_channels_min"]:
            score += 10
            reasons.append(f"ADC通道充足({mcu['ADC通道']}通道)")

        # PWM加分
        if requirements.get("pwm_min") and mcu["PWM"] >= requirements["pwm_min"]:
            score += 8
            reasons.append(f"PWM通道{mcu['PWM']}个")

        # DAC加分
        if mcu["DAC"]:
            score += 5
            if isinstance(mcu["DAC"], int) and mcu["DAC"] > 1:
                score += 3
                reasons.append(f"DAC通道{mcu['DAC']}个")

        # CAN总线加分
        if mcu["CAN"] > 0:
            score += 5
            reasons.append(f"CAN {mcu['CAN']}个")

        # USB加分
        if mcu["USB"]:
            score += 5

        # 供货充足加分
        if mcu["供货"] == "充足":
            score += 10
            reasons.append("供货充足")
        else:
            penalties.append("供货紧张")
            score -= 10

        # 性价比加分（价格越低越好）
        if mcu["单价约"] <= 5:
            score += 15
            reasons.append(f"高性价比(约¥{mcu['单价约']})")
        elif mcu["单价约"] <= 15:
            score += 8
            reasons.append(f"性价比良好(约¥{mcu['单价约']})")
        elif mcu["单价约"] <= 30:
            score += 3

        # 特点匹配加分
        if requirements.get("keyword"):
            kw = requirements["keyword"]
            if kw in mcu["特点"]:
                score += 15
                reasons.append(f"特点匹配: {mcu['特点']}")

        results.append({
            "型号": mcu["型号"],
            "厂商": mcu["厂商"],
            "系列": mcu["系列"],
            "内核": mcu["内核"],
            "主频MHz": mcu["主频MHz"],
            "FlashKB": mcu["FlashKB"],
            "RAMKB": mcu["RAMKB"],
            "GPIO": mcu["GPIO"],
            "ADC通道": mcu["ADC通道"],
            "DAC": mcu["DAC"],
            "PWM": mcu["PWM"],
            "UART": mcu["UART"],
            "SPI": mcu["SPI"],
            "I2C": mcu["I2C"],
            "CAN": mcu["CAN"],
            "USB": "有" if mcu["USB"] else "无",
            "封装": mcu["封装"],
            "单价约": f"¥{mcu['单价约']}",
            "供货": mcu["供货"],
            "特点": mcu["特点"],
            "匹配分": score,
            "优势": reasons,
            "不足": penalties,
        })

    # 按匹配分降序排列
    results.sort(key=lambda x: x["匹配分"], reverse=True)
    return results


def print_results(results: list, top_n: int = 5):
    """格式化打印推荐结果"""
    if not results:
        print("\n❌ 未找到满足所有硬性要求的MCU，请放宽条件重试。")
        return

    print(f"\n{'='*70}")
    print(f"  MCU选型推荐结果（共找到 {len(results)} 款，显示前 {min(top_n, len(results))} 款）")
    print(f"{'='*70}")

    for i, mcu in enumerate(results[:top_n], 1):
        print(f"\n  ┌─ 第 {i} 名 ─ 匹配分: {mcu['匹配分']}分")
        print(f"  │ 型号: {mcu['型号']}  厂商: {mcu['厂商']}  系列: {mcu['系列']}")
        print(f"  │ 内核: {mcu['内核']}  主频: {mcu['主频MHz']}MHz")
        print(f"  │ Flash: {mcu['FlashKB']}KB  RAM: {mcu['RAMKB']}KB  GPIO: {mcu['GPIO']}")
        print(f"  │ ADC: {mcu['ADC通道']}ch  DAC: {mcu['DAC']}  PWM: {mcu['PWM']}")
        print(f"  │ UART: {mcu['UART']}  SPI: {mcu['SPI']}  I2C: {mcu['I2C']}  CAN: {mcu['CAN']}")
        print(f"  │ USB: {mcu['USB']}  封装: {mcu['封装']}")
        print(f"  │ 单价: {mcu['单价约']}  供货: {mcu['供货']}")
        print(f"  │ 特点: {mcu['特点']}")
        if mcu["优势"]:
            print(f"  │ ✅ {'; '.join(mcu['优势'])}")
        if mcu["不足"]:
            print(f"  │ ⚠️ {'; '.join(mcu['不足'])}")
        print(f"  └{'─'*60}")

    print()


def interactive_mode():
    """交互式选型模式"""
    print("\n" + "="*60)
    print("  🔧 MCU交互式选型工具")
    print("="*60)
    print("  （直接回车跳过该项，表示不作限制）\n")

    req = {}

    try:
        # 系列筛选
        families = ["STM32F1", "STM32F4", "STM32G4", "STM32H7", "STM32L4",
                     "GD32F1", "GD32F3", "AT32", "CH32V", "ESP32", "MSP430", "STC8", "AVR", "RP2"]
        print(f"  可选系列: {', '.join(families)}")
        fam = input("  目标系列: ").strip()
        if fam:
            req["family"] = fam

        # 位宽
        bit = input("  最低位宽 (8/16/32): ").strip()
        if bit:
            req["cpu_bit"] = int(bit)

        # Flash
        flash = input("  最小Flash (KB): ").strip()
        if flash:
            req["flash_min"] = int(flash)

        # RAM
        ram = input("  最小RAM (KB): ").strip()
        if ram:
            req["ram_min"] = int(ram)

        # GPIO
        gpio = input("  最少GPIO数量: ").strip()
        if gpio:
            req["gpio_min"] = int(gpio)

        # 外设需求
        adc = input("  需要ADC? (y/n): ").strip().lower()
        if adc == 'y':
            req["adc"] = True
        adc_ch = input("  最少ADC通道数: ").strip()
        if adc_ch:
            req["adc_channels_min"] = int(adc_ch)

        dac = input("  需要DAC? (y/n): ").strip().lower()
        if dac == 'y':
            req["dac"] = True

        pwm = input("  最少PWM通道数: ").strip()
        if pwm:
            req["pwm_min"] = int(pwm)

        uart = input("  最少UART数量: ").strip()
        if uart:
            req["uart_min"] = int(uart)

        spi = input("  最少SPI数量: ").strip()
        if spi:
            req["spi_min"] = int(spi)

        i2c = input("  最少I2C数量: ").strip()
        if i2c:
            req["i2c_min"] = int(i2c)

        can = input("  需要CAN? (y/n): ").strip().lower()
        if can == 'y':
            req["can"] = True

        usb = input("  需要USB? (y/n): ").strip().lower()
        if usb == 'y':
            req["usb"] = True

        freq = input("  最低主频 (MHz): ").strip()
        if freq:
            req["frequency_min"] = int(freq)

        vcc = input("  工作电压 (V): ").strip()
        if vcc:
            req["voltage"] = float(vcc)

        print(f"\n  正在根据您的需求筛选...")
        results = find_best_mcu(req)
        print_results(results)

    except (ValueError, KeyboardInterrupt):
        print("\n  输入有误或已取消。")


def list_families():
    """列出所有可选MCU系列"""
    families = {}
    for mcu in MCU_DATABASE:
        key = mcu["系列"]
        if key not in families:
            families[key] = {"厂商": mcu["厂商"], "型号数": 0, "示例": mcu["型号"]}
        families[key]["型号数"] += 1

    print(f"\n{'='*60}")
    print(f"  📦 可选MCU系列（共{len(families)}个系列，{len(MCU_DATABASE)}款型号）")
    print(f"{'='*60}")
    for fam, info in families.items():
        print(f"  • {fam:15s}  厂商: {info['厂商']:10s}  型号数: {info['型号数']:2d}  示例: {info['示例']}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="MCU选型工具 - 根据需求推荐最佳微控制器",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例:
  # 需要32位MCU，Flash>=256KB，带ADC+DAC+CAN
  python mcu_selector.py --cpu-bit 32 --flash-min 256 --dac --can

  # STM32系列，GPIO>=50，3.3V供电
  python mcu_selector.py --family STM32 --gpio-min 50 --voltage 3.3

  # 交互式选型
  python mcu_selector.py --interactive

  # 列出所有可选系列
  python mcu_selector.py --list-families
"""
    )

    parser.add_argument("--family", help="目标系列/厂商 (如: STM32F1, GD32, ESP32, CH32)")
    parser.add_argument("--cpu-bit", type=int, choices=[8, 16, 32], help="最低位宽")
    parser.add_argument("--flash-min", type=int, help="最小Flash (KB)")
    parser.add_argument("--ram-min", type=int, help="最小RAM (KB)")
    parser.add_argument("--gpio-min", type=int, help="最少GPIO数量")
    parser.add_argument("--frequency-min", type=int, help="最低主频 (MHz)")
    parser.add_argument("--voltage", type=float, help="工作电压 (V)")
    parser.add_argument("--adc", action="store_true", help="需要ADC")
    parser.add_argument("--adc-channels-min", type=int, help="最少ADC通道数")
    parser.add_argument("--dac", action="store_true", help="需要DAC")
    parser.add_argument("--pwm-min", type=int, help="最少PWM通道数")
    parser.add_argument("--uart-min", type=int, help="最少UART数量")
    parser.add_argument("--spi-min", type=int, help="最少SPI数量")
    parser.add_argument("--i2c-min", type=int, help="最少I2C数量")
    parser.add_argument("--can", action="store_true", help="需要CAN总线")
    parser.add_argument("--usb", action="store_true", help="需要USB")
    parser.add_argument("--keyword", help="关键词匹配 (如: 低功耗, WiFi, 信号处理)")
    parser.add_argument("--top", type=int, default=5, help="显示前N个结果 (默认5)")
    parser.add_argument("--list-families", action="store_true", help="列出所有可选MCU系列")
    parser.add_argument("--interactive", action="store_true", help="交互式选型模式")
    parser.add_argument("--json", action="store_true", help="输出JSON格式")

    args = parser.parse_args()

    if args.list_families:
        list_families()
        return

    if args.interactive:
        interactive_mode()
        return

    # 构建需求字典
    req = {}
    if args.family: req["family"] = args.family
    if args.cpu_bit: req["cpu_bit"] = args.cpu_bit
    if args.flash_min: req["flash_min"] = args.flash_min
    if args.ram_min: req["ram_min"] = args.ram_min
    if args.gpio_min: req["gpio_min"] = args.gpio_min
    if args.frequency_min: req["frequency_min"] = args.frequency_min
    if args.voltage: req["voltage"] = args.voltage
    if args.adc: req["adc"] = True
    if args.adc_channels_min: req["adc_channels_min"] = args.adc_channels_min
    if args.dac: req["dac"] = True
    if args.pwm_min: req["pwm_min"] = args.pwm_min
    if args.uart_min: req["uart_min"] = args.uart_min
    if args.spi_min: req["spi_min"] = args.spi_min
    if args.i2c_min: req["i2c_min"] = args.i2c_min
    if args.can: req["can"] = True
    if args.usb: req["usb"] = True
    if args.keyword: req["keyword"] = args.keyword

    if not req:
        parser.print_help()
        print("\n💡 提示: 请至少指定一个筛选条件，或使用 --interactive 进入交互模式")
        return

    results = find_best_mcu(req)

    if args.json:
        print(json.dumps(results[:args.top], ensure_ascii=False, indent=2))
    else:
        print_results(results, top_n=args.top)


if __name__ == "__main__":
    main()
