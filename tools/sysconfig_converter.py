#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SysConfig配置转换器 - 从引脚表自动生成TI MSPM0/MSP430的.syscfg配置文件
用法: python sysconfig_converter.py --pin-table pins.csv --output board.syscfg
"""

import argparse
import csv
import json
import os
from datetime import datetime


# SysConfig 模块映射：外设 -> syscfg模块名
MODULE_MAP = {
    "gpio": "gpio",
    "i2c": "i2c",
    "spi": "spi",
    "uart": "uart",
    "adc": "adc12",
    "pwm": "timer",
    "timer": "timer",
    "dma": "dma",
}


def parse_pin_table(csv_path: str) -> list:
    """
    解析引脚表CSV文件
    期望列: name, pin, module, direction, pull, label
    示例: LED_RED, P1.0, gpio, output, none, 红色LED
    """
    pins = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pin = {
                "name": row.get("name", "").strip(),
                "pin": row.get("pin", "").strip(),
                "module": row.get("module", "gpio").strip().lower(),
                "direction": row.get("direction", "output").strip().lower(),
                "pull": row.get("pull", "none").strip().lower(),
                "label": row.get("label", "").strip(),
            }
            if pin["name"] and pin["pin"]:
                pins.append(pin)
    return pins


def generate_syscfg(pins: list, board: str = "LP_MSPM0G3507") -> dict:
    """
    生成 .syscfg JSON结构
    """
    syscfg = {
        "meta": {
            "version": "1.0",
            "tool": "sysconfig_converter.py",
            "board": board,
            "date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        },
        "modules": {},
    }

    # 按模块分组
    module_pins = {}
    for pin in pins:
        mod = pin["module"]
        if mod not in module_pins:
            module_pins[mod] = []
        module_pins[mod].append(pin)

    # GPIO配置
    if "gpio" in module_pins:
        gpio_config = []
        for p in module_pins["gpio"]:
            entry = {
                "name": p["name"],
                "pin": p["pin"],
                "direction": p["direction"],
                "initialState": 0,
            }
            if p["pull"] != "none":
                entry["pull"] = p["pull"]
            if p["label"]:
                entry["comment"] = p["label"]
            gpio_config.append(entry)
        syscfg["modules"]["gpio"] = {"instances": gpio_config}

    # I2C配置
    if "i2c" in module_pins:
        i2c_pins = module_pins["i2c"]
        sda = next((p for p in i2c_pins if "sda" in p["name"].lower()), None)
        scl = next((p for p in i2c_pins if "scl" in p["name"].lower()), None)
        if sda and scl:
            syscfg["modules"]["i2c"] = {
                "instances": [{
                    "name": "I2C_0",
                    "sda": sda["pin"],
                    "scl": scl["pin"],
                    "bitRate": 400000,
                    "comment": "I2C总线",
                }]
            }

    # SPI配置
    if "spi" in module_pins:
        spi_pins = module_pins["spi"]
        syscfg["modules"]["spi"] = {
            "instances": [{
                "name": "SPI_0",
                "clk": next((p["pin"] for p in spi_pins if "clk" in p["name"].lower()), ""),
                "mosi": next((p["pin"] for p in spi_pins if "mosi" in p["name"].lower()), ""),
                "miso": next((p["pin"] for p in spi_pins if "miso" in p["name"].lower()), ""),
                "cs": next((p["pin"] for p in spi_pins if "cs" in p["name"].lower()), ""),
                "bitRate": 1000000,
                "comment": "SPI总线",
            }]
        }

    # UART配置
    if "uart" in module_pins:
        uart_pins = module_pins["uart"]
        syscfg["modules"]["uart"] = {
            "instances": [{
                "name": "UART_0",
                "tx": next((p["pin"] for p in uart_pins if "tx" in p["name"].lower()), ""),
                "rx": next((p["pin"] for p in uart_pins if "rx" in p["name"].lower()), ""),
                "baudRate": 115200,
                "comment": "串口",
            }]
        }

    # ADC配置
    if "adc" in module_pins:
        adc_instances = []
        for p in module_pins["adc"]:
            adc_instances.append({
                "name": p["name"],
                "channel": p["pin"],
                "comment": p.get("label", ""),
            })
        syscfg["modules"]["adc12"] = {"instances": adc_instances}

    return syscfg


def generate_syscfg_text(syscfg: dict) -> str:
    """生成 .syscfg 文件格式（TI SysConfig兼容文本格式）"""
    lines = [
        "// 自动生成的 SysConfig 配置文件",
        f"// 生成工具: sysconfig_converter.py",
        f"// 日期: {syscfg['meta']['date']}",
        f"// 开发板: {syscfg['meta']['board']}",
        "",
    ]

    modules = syscfg.get("modules", {})

    # GPIO
    if "gpio" in modules:
        lines.append("// ========== GPIO 配置 ==========")
        for inst in modules["gpio"]["instances"]:
            comment = f"  // {inst['comment']}" if inst.get("comment") else ""
            lines.append(f"// {inst['name']}: {inst['pin']} [{inst['direction']}]{comment}")
        lines.append("")

    # I2C
    if "i2c" in modules:
        lines.append("// ========== I2C 配置 ==========")
        for inst in modules["i2c"]["instances"]:
            lines.append(f"// {inst['name']}: SDA={inst['sda']}, SCL={inst['scl']}, 速率={inst['bitRate']}Hz")
        lines.append("")

    # SPI
    if "spi" in modules:
        lines.append("// ========== SPI 配置 ==========")
        for inst in modules["spi"]["instances"]:
            lines.append(f"// {inst['name']}: CLK={inst['clk']}, MOSI={inst['mosi']}, MISO={inst['miso']}, CS={inst['cs']}")
        lines.append("")

    # UART
    if "uart" in modules:
        lines.append("// ========== UART 配置 ==========")
        for inst in modules["uart"]["instances"]:
            lines.append(f"// {inst['name']}: TX={inst['tx']}, RX={inst['rx']}, 波特率={inst['baudRate']}")
        lines.append("")

    # ADC
    if "adc12" in modules:
        lines.append("// ========== ADC 配置 ==========")
        for inst in modules["adc12"]["instances"]:
            lines.append(f"// {inst['name']}: 通道={inst['channel']}")
        lines.append("")

    lines.append("// JSON 数据 (供工具链解析):")
    lines.append("/*")
    lines.append(json.dumps(syscfg, indent=2, ensure_ascii=False))
    lines.append("*/")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="SysConfig配置转换器 - 从引脚表生成.syscfg配置")
    parser.add_argument("--pin-table", required=True, help="引脚表CSV文件路径")
    parser.add_argument("--output", default="board.syscfg", help="输出.syscfg文件路径")
    parser.add_argument("--board", default="LP_MSPM0G3507", help="目标开发板型号")
    parser.add_argument("--json", action="store_true", help="同时输出JSON格式")
    parser.add_argument("--generate-csv-template", help="生成CSV引脚表模板文件")
    args = parser.parse_args()

    # 生成CSV模板
    if args.generate_csv_template:
        template = "name,pin,module,direction,pull,label\n"
        template += "LED_RED,P1.0,gpio,output,none,红色LED\n"
        template += "LED_GREEN,P1.1,gpio,output,none,绿色LED\n"
        template += "BUTTON_1,P1.2,gpio,input,pullup,按键1\n"
        template += "I2C_SDA,P1.3,i2c,,none,I2C数据线\n"
        template += "I2C_SCL,P1.4,i2c,,none,I2C时钟线\n"
        template += "SPI_CLK,P1.5,spi,,none,SPI时钟\n"
        template += "SPI_MOSI,P1.6,spi,,none,SPI主出\n"
        template += "SPI_MISO,P1.7,spi,,none,SPI主入\n"
        with open(args.generate_csv_template, "w", encoding="utf-8", newline="") as f:
            f.write(template)
        print(f"[✓] CSV模板已生成: {args.generate_csv_template}")
        return

    # 解析引脚表
    pins = parse_pin_table(args.pin_table)
    if not pins:
        print("[✗] 引脚表为空或格式错误")
        return
    print(f"[i] 解析到 {len(pins)} 个引脚配置")

    # 生成syscfg
    syscfg = generate_syscfg(pins, args.board)

    # 输出文件
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8", newline="\n") as f:
        f.write(generate_syscfg_text(syscfg))

    print(f"[✓] SysConfig配置已生成: {args.output}")

    # 可选JSON输出
    if args.json:
        json_path = args.output.rsplit(".", 1)[0] + ".json"
        with open(json_path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(syscfg, f, indent=2, ensure_ascii=False)
        print(f"[✓] JSON配置已生成: {json_path}")

    # 打印摘要
    mods = list(syscfg.get("modules", {}).keys())
    print(f"[i] 包含模块: {', '.join(mods) if mods else '无'}")


if __name__ == "__main__":
    main()
