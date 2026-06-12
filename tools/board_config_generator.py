#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
板级配置生成器 - 从引脚表生成SysConfig/CubeMX配置
=================================================
功能：
  - 从CSV/JSON引脚表读取引脚分配
  - 检测引脚冲突
  - 生成TI SysConfig .cfg文件
  - 生成STM32CubeMX .ioc文件
  - 输出引脚映射报告
"""

import argparse
import csv
import json
import os
import sys
from collections import defaultdict
from datetime import datetime

# ============================================================
# 常量定义
# ============================================================

# 常见MCU引脚复用映射 (简化版)
STM32_AF_MAP = {
    "GPIO": 0, "TIM1_CH1": 1, "TIM1_CH2": 1, "TIM2_CH1": 1,
    "TIM2_CH2": 1, "TIM3_CH1": 2, "TIM3_CH2": 2, "TIM4_CH1": 2,
    "USART1_TX": 7, "USART1_RX": 7, "USART2_TX": 7, "USART2_RX": 7,
    "SPI1_MOSI": 5, "SPI1_MISO": 5, "SPI1_SCK": 5, "SPI1_NSS": 5,
    "SPI2_MOSI": 5, "SPI2_MISO": 5, "I2C1_SCL": 4, "I2C1_SDA": 4,
    "I2C2_SCL": 4, "I2C2_SDA": 4, "ADC1_IN0": 0, "ADC1_IN1": 0,
    "USB_DM": 10, "USB_DP": 10, "CAN1_TX": 9, "CAN1_RX": 9,
}

# SysConfig支持的外设类型
SYSCONFIG_PERIPHERALS = {
    "GPIO", "UART", "SPI", "I2C", "ADC", "DAC", "PWM", "Timer",
    "DMA", "CAN", "USB", "SDIO", "QSPI", "Ethernet",
}

# 引脚模式映射
PIN_MODE_MAP = {
    "input": ("GPIO_INPUT", "PullUp"),
    "output": ("GPIO_OUTPUT", "PushPull"),
    "analog": ("GPIO_ANALOG", "None"),
    "af_pp": ("GPIO_ALT", "PushPull"),
    "af_od": ("GPIO_ALT", "OpenDrain"),
}


class PinConfig:
    """引脚配置数据类"""

    def __init__(self, pin_name, port, pin_num, function, mode="af_pp",
                 speed="HIGH", pull="NOPULL", label="", group="default"):
        self.pin_name = pin_name       # 用户定义的引脚名
        self.port = port               # 端口 (PA, PB, PC...)
        self.pin_num = int(pin_num)    # 引脚号
        self.function = function       # 功能 (USART1_TX, GPIO, ADC1_IN0...)
        self.mode = mode               # 模式 (input, output, analog, af_pp, af_od)
        self.speed = speed             # 速度 (LOW, MEDIUM, HIGH, VERY_HIGH)
        self.pull = pull               # 上拉/下拉 (NOPULL, PULLUP, PULLDOWN)
        self.label = label or pin_name # 显示标签
        self.group = group             # 分组

    @property
    def full_pin(self):
        """完整引脚标识，如 PA0"""
        return f"{self.port}{self.pin_num}"

    def __repr__(self):
        return f"Pin({self.full_pin} -> {self.function} [{self.mode}])"


def parse_pin_csv(filepath):
    """
    从CSV文件解析引脚配置
    CSV格式: pin_name,port,pin_num,function,mode,speed,pull,label,group
    """
    pins = []
    with open(filepath, 'r', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        for row in reader:
            pin = PinConfig(
                pin_name=row.get('pin_name', '').strip(),
                port=row.get('port', '').strip().upper(),
                pin_num=row.get('pin_num', '0').strip(),
                function=row.get('function', 'GPIO').strip(),
                mode=row.get('mode', 'af_pp').strip().lower(),
                speed=row.get('speed', 'HIGH').strip().upper(),
                pull=row.get('pull', 'NOPULL').strip().upper(),
                label=row.get('label', '').strip(),
                group=row.get('group', 'default').strip(),
            )
            pins.append(pin)
    return pins


def parse_pin_json(filepath):
    """从JSON文件解析引脚配置"""
    with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)
    if isinstance(data, dict):
        data = data.get('pins', data.get('pin_config', []))
    pins = []
    for item in data:
        pin = PinConfig(
            pin_name=item.get('pin_name', ''),
            port=item.get('port', '').upper(),
            pin_num=item.get('pin_num', 0),
            function=item.get('function', 'GPIO'),
            mode=item.get('mode', 'af_pp').lower(),
            speed=item.get('speed', 'HIGH').upper(),
            pull=item.get('pull', 'NOPULL').upper(),
            label=item.get('label', ''),
            group=item.get('group', 'default'),
        )
        pins.append(pin)
    return pins


def check_pin_conflicts(pins):
    """
    检测引脚冲突
    返回冲突列表 [(pin_full, [conflicting_functions])]
    """
    pin_map = defaultdict(list)
    for pin in pins:
        pin_map[pin.full_pin].append(pin.function)

    conflicts = []
    for pin_id, funcs in pin_map.items():
        if len(funcs) > 1:
            conflicts.append((pin_id, funcs))
    return conflicts


def check_resource_conflicts(pins):
    """
    检测外设资源冲突（同一外设通道被多次分配）
    """
    resource_map = defaultdict(list)
    for pin in pins:
        func = pin.function
        # 提取外设通道名 (如 TIM2_CH1)
        if '_' in func and not func.startswith('GPIO'):
            resource_map[func].append(pin.full_pin)

    conflicts = []
    for resource, pin_list in resource_map.items():
        if len(pin_list) > 1:
            conflicts.append((resource, pin_list))
    return conflicts


def generate_sysconfig(pins, mcu="MSP432P401R", output_path=None):
    """
    生成TI SysConfig .cfg配置文件
    """
    lines = []
    lines.append(f"// SysConfig配置文件 - 自动生成")
    lines.append(f"// MCU: {mcu}")
    lines.append(f"// 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"// 引脚数量: {len(pins)}")
    lines.append("")
    lines.append("/* ======== 外设配置 ======== */")
    lines.append("")

    # 按外设分组
    groups = defaultdict(list)
    for pin in pins:
        func = pin.function
        if func.startswith("UART") or func.startswith("USART"):
            groups["UART"].append(pin)
        elif func.startswith("SPI"):
            groups["SPI"].append(pin)
        elif func.startswith("I2C"):
            groups["I2C"].append(pin)
        elif func.startswith("ADC") or func.startswith("DAC"):
            groups["ADC"].append(pin)
        elif func.startswith("TIM"):
            groups["Timer"].append(pin)
        elif func.startswith("CAN"):
            groups["CAN"].append(pin)
        elif func.startswith("USB"):
            groups["USB"].append(pin)
        else:
            groups["GPIO"].append(pin)

    # 生成GPIO配置
    if "GPIO" in groups:
        lines.append("// ======== GPIO配置 ========")
        for pin in groups["GPIO"]:
            mode_info = PIN_MODE_MAP.get(pin.mode, ("GPIO_OUTPUT", "PushPull"))
            cfg_type, drive = mode_info
            lines.append(f"const uint8_t {pin.label}_PIN = GPIO_PIN{pin.pin_num};")
            lines.append(f"// {pin.port}{pin.pin_num}: {cfg_type}, {drive}, {pin.pull}")
            lines.append(f"GPIO_setConfig({pin.port}{pin.pin_num}, "
                         f"GPIO_CFG_{cfg_type} | GPIO_CFG_{drive} | GPIO_CFG_{pin.pull});")
            lines.append("")

    # 生成UART配置
    if "UART" in groups:
        lines.append("// ======== UART配置 ========")
        uart_modules = defaultdict(list)
        for pin in groups["UART"]:
            module = pin.function.split('_')[0]  # e.g., USART1
            uart_modules[module].append(pin)
        for module, upins in uart_modules.items():
            lines.append(f"// {module} 配置")
            lines.append(f"const uint32_t {module}_BAUDRATE = 115200;")
            tx_pin = next((p for p in upins if 'TX' in p.function), None)
            rx_pin = next((p for p in upins if 'RX' in p.function), None)
            if tx_pin:
                lines.append(f"// TX: {tx_pin.full_pin}")
            if rx_pin:
                lines.append(f"// RX: {rx_pin.full_pin}")
            lines.append("")

    # 生成SPI配置
    if "SPI" in groups:
        lines.append("// ======== SPI配置 ========")
        spi_modules = defaultdict(list)
        for pin in groups["SPI"]:
            module = pin.function.split('_')[0]
            spi_modules[module].append(pin)
        for module, spins in spi_modules.items():
            lines.append(f"// {module} 配置")
            lines.append(f"const uint32_t {module}_CLOCK_FREQ = 1000000; // 1MHz")
            for sp in spins:
                role = sp.function.split('_')[-1]
                lines.append(f"// {role}: {sp.full_pin}")
            lines.append("")

    # 生成I2C配置
    if "I2C" in groups:
        lines.append("// ======== I2C配置 ========")
        i2c_modules = defaultdict(list)
        for pin in groups["I2C"]:
            module = pin.function.split('_')[0]
            i2c_modules[module].append(pin)
        for module, ipins in i2c_modules.items():
            lines.append(f"// {module} 配置")
            lines.append(f"const uint32_t {module}_BITRATE = 400000; // 400kHz")
            for ip in ipins:
                role = ip.function.split('_')[-1]
                lines.append(f"// {role}: {ip.full_pin}")
            lines.append("")

    # 生成Timer/PWM配置
    if "Timer" in groups:
        lines.append("// ======== Timer/PWM配置 ========")
        for pin in groups["Timer"]:
            lines.append(f"// {pin.function}: {pin.full_pin}")
            lines.append(f"// 预分频/周期需根据具体需求设置")
            lines.append("")

    # 生成ADC配置
    if "ADC" in groups:
        lines.append("// ======== ADC配置 ========")
        for pin in groups["ADC"]:
            lines.append(f"// {pin.function}: {pin.full_pin} ({pin.mode})")
            lines.append("")

    # 引脚映射总结
    lines.append("/* ======== 引脚映射总结 ======== */")
    lines.append("// 引脚名          | 端口   | 功能        | 模式")
    lines.append("// " + "-" * 55)
    for pin in sorted(pins, key=lambda p: (p.port, p.pin_num)):
        lines.append(f"// {pin.label:<16s} | {pin.full_pin:<6s} | {pin.function:<12s} | {pin.mode}")

    content = '\n'.join(lines)

    if output_path:
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"[OK] SysConfig配置已生成: {output_path}")

    return content


def generate_cubemx_ioc(pins, mcu="STM32F407VGT6", output_path=None):
    """
    生成STM32CubeMX .ioc配置文件
    """
    lines = []
    lines.append(f"# STM32CubeMX IOC配置文件 - 自动生成")
    lines.append(f"# MCU: {mcu}")
    lines.append(f"# 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"# 引脚数量: {len(pins)}")
    lines.append("")
    lines.append(f"ProjectManager.CoupleFile=true")
    lines.append(f"ProjectManager.ProjectBuildStruct=0")
    lines.append(f"ProjectManager.ProjectName=generated_project")
    lines.append(f"ProjectManager.TargetMCU={mcu}")
    lines.append("")

    # 引脚配置
    for pin in sorted(pins, key=lambda p: (p.port, p.pin_num)):
        pin_id = f"P{pin.port[1]}{pin.pin_num}"  # PA0 -> P<port><num>
        full_id = f"{pin.port}{pin.pin_num}"

        # 模式映射
        mode_map = {
            "input": "GPIO_Input",
            "output": "GPIO_Output",
            "analog": "ADC1_IN",
            "af_pp": "GPIO_Output",
            "af_od": "GPIO_Output",
        }

        if pin.function.startswith("USART") or pin.function.startswith("UART"):
            module = pin.function.split('_')[0]
            role = pin.function.split('_')[-1]
            lines.append(f"{full_id}.GPIOParameters={pin.label}")
            lines.append(f"{full_id}.GPIO_Label={pin.label}")
            lines.append(f"{full_id}.Locked=true")
            lines.append(f"{full_id}.Mode={module}_{role}")
            lines.append(f"{full_id}.Signal={module}_{role}")
            if 'TX' in pin.function:
                lines.append(f"{full_id}.GPIO_PuPd=GPIO_NOPULL")
            else:
                lines.append(f"{full_id}.GPIO_PuPd=GPIO_PULLUP")

        elif pin.function.startswith("SPI"):
            module = pin.function.split('_')[0]
            role = pin.function.split('_')[-1]
            lines.append(f"{full_id}.GPIOParameters={pin.label}")
            lines.append(f"{full_id}.GPIO_Label={pin.label}")
            lines.append(f"{full_id}.Locked=true")
            lines.append(f"{full_id}.Mode={module}")
            lines.append(f"{full_id}.Signal={module}_{role}")

        elif pin.function.startswith("I2C"):
            module = pin.function.split('_')[0]
            role = pin.function.split('_')[-1]
            lines.append(f"{full_id}.GPIOParameters={pin.label}")
            lines.append(f"{full_id}.GPIO_Label={pin.label}")
            lines.append(f"{full_id}.Locked=true")
            lines.append(f"{full_id}.Mode=I2C")
            lines.append(f"{full_id}.Signal={module}_{role}")
            lines.append(f"{full_id}.GPIO_PuPd=GPIO_PULLUP")
            lines.append(f"{full_id}.GPIO_Speed=GPIO_SPEED_FREQ_HIGH")

        elif pin.function.startswith("TIM"):
            module = pin.function.split('_')[0]
            channel = pin.function.split('_')[-1] if '_' in pin.function else "CH1"
            lines.append(f"{full_id}.GPIOParameters={pin.label}")
            lines.append(f"{full_id}.GPIO_Label={pin.label}")
            lines.append(f"{full_id}.Locked=true")
            lines.append(f"{full_id}.Mode={module}")
            lines.append(f"{full_id}.Signal={module}_{channel}")

        elif "ADC" in pin.function:
            lines.append(f"{full_id}.GPIOParameters={pin.label}")
            lines.append(f"{full_id}.GPIO_Label={pin.label}")
            lines.append(f"{full_id}.Locked=true")
            lines.append(f"{full_id}.Mode=ADC1_IN")
            lines.append(f"{full_id}.Signal=ADC1_IN")

        elif pin.function.startswith("CAN"):
            module = pin.function.split('_')[0]
            role = pin.function.split('_')[-1]
            lines.append(f"{full_id}.GPIOParameters={pin.label}")
            lines.append(f"{full_id}.GPIO_Label={pin.label}")
            lines.append(f"{full_id}.Locked=true")
            lines.append(f"{full_id}.Mode={module}")
            lines.append(f"{full_id}.Signal={module}_{role}")

        else:  # GPIO
            gpio_mode = "GPIO_Output" if pin.mode in ("output", "af_pp", "af_od") else "GPIO_Input"
            lines.append(f"{full_id}.GPIOParameters={pin.label}")
            lines.append(f"{full_id}.GPIO_Label={pin.label}")
            lines.append(f"{full_id}.Locked=true")
            lines.append(f"{full_id}.Mode={gpio_mode}")
            lines.append(f"{full_id}.Signal=GPIO_Output" if gpio_mode == "GPIO_Output"
                         else f"{full_id}.Signal=GPIO_Input")
            if pin.pull == "PULLUP":
                lines.append(f"{full_id}.GPIO_PuPd=GPIO_PULLUP")
            elif pin.pull == "PULLDOWN":
                lines.append(f"{full_id}.GPIO_PuPd=GPIO_PULLDOWN")

        lines.append("")

    # 时钟配置提示
    lines.append("# ======== 时钟配置提示 ========")
    lines.append("# RCC.HSE=8000000  (外部高速晶振)")
    lines.append("# RCC.PLLM=8")
    lines.append("# RCC.PLLN=336")
    lines.append("# RCC.PLLP=2")
    lines.append("# SYSCLK=168MHz")

    content = '\n'.join(lines)

    if output_path:
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"[OK] CubeMX IOC配置已生成: {output_path}")

    return content


def generate_pin_report(pins, output_path=None):
    """
    生成引脚分配报告（Markdown格式）
    """
    lines = []
    lines.append("# 板级引脚配置报告")
    lines.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"引脚总数: {len(pins)}")
    lines.append("")

    # 引脚冲突检查
    conflicts = check_pin_conflicts(pins)
    resource_conflicts = check_resource_conflicts(pins)

    if conflicts or resource_conflicts:
        lines.append("## ⚠️ 发现冲突")
        for pin_id, funcs in conflicts:
            lines.append(f"- **引脚冲突** {pin_id}: {', '.join(funcs)}")
        for resource, pin_list in resource_conflicts:
            lines.append(f"- **资源冲突** {resource}: 被分配到 {', '.join(pin_list)}")
        lines.append("")

    # 按分组统计
    groups = defaultdict(list)
    for pin in pins:
        groups[pin.group].append(pin)

    for group_name, group_pins in sorted(groups.items()):
        lines.append(f"## 分组: {group_name} ({len(group_pins)}个引脚)")
        lines.append("")
        lines.append("| 引脚名 | 端口 | 功能 | 模式 | 速度 | 上拉/下拉 |")
        lines.append("|--------|------|------|------|------|-----------|")
        for pin in sorted(group_pins, key=lambda p: (p.port, p.pin_num)):
            lines.append(f"| {pin.label} | {pin.full_pin} | {pin.function} | "
                        f"{pin.mode} | {pin.speed} | {pin.pull} |")
        lines.append("")

    # 端口使用率统计
    lines.append("## 端口使用率")
    port_usage = defaultdict(int)
    for pin in pins:
        port_usage[pin.port] += 1
    for port, count in sorted(port_usage.items()):
        lines.append(f"- {port}: {count}个引脚")
    lines.append("")

    # 外设统计
    periph_count = defaultdict(int)
    for pin in pins:
        func = pin.function
        if func.startswith("GPIO"):
            periph_count["GPIO"] += 1
        else:
            periph = func.split('_')[0]
            periph_count[periph] += 1
    lines.append("## 外设使用统计")
    for periph, count in sorted(periph_count.items()):
        lines.append(f"- {periph}: {count}个引脚")

    content = '\n'.join(lines)

    if output_path:
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"[OK] 引脚报告已生成: {output_path}")

    return content


def generate_sample_csv(output_path):
    """生成示例引脚CSV文件"""
    lines = [
        "pin_name,port,pin_num,function,mode,speed,pull,label,group",
        "LED1,PA,5,GPIO,output,HIGH,NOPULL,LED_GREEN,led",
        "LED2,PA,6,GPIO,output,HIGH,NOPULL,LED_RED,led",
        "KEY1,PA,0,input,input,LOW,PULLUP,BTN_LEFT,button",
        "KEY2,PA,1,input,input,LOW,PULLUP,BTN_RIGHT,button",
        "UART_TX,PA,9,USART1_TX,af_pp,VERY_HIGH,NOPULL,DEBUG_TX,debug",
        "UART_RX,PA,10,USART1_RX,af_pp,VERY_HIGH,PULLUP,DEBUG_RX,debug",
        "SPI_SCK,PB,13,SPI2_SCK,af_pp,VERY_HIGH,NOPULL,FLASH_SCK,flash",
        "SPI_MOSI,PB,15,SPI2_MOSI,af_pp,VERY_HIGH,NOPULL,FLASH_MOSI,flash",
        "SPI_MISO,PB,14,SPI2_MISO,af_pp,VERY_HIGH,NOPULL,FLASH_MISO,flash",
        "SPI_CS,PB,12,GPIO,output,HIGH,NOPULL,FLASH_CS,flash",
        "I2C_SCL,PB,6,I2C1_SCL,af_od,HIGH,PULLUP,SENSOR_SCL,sensor",
        "I2C_SDA,PB,7,I2C1_SDA,af_od,HIGH,PULLUP,SENSOR_SDA,sensor",
        "ADC_BAT,PA,2,ADC1_IN2,analog,LOW,NOPULL,BAT_VOLT,adc",
        "ADC_CUR,PA,3,ADC1_IN3,analog,LOW,NOPULL,CURRENT,adc",
        "PWM_MOTOR,PB,0,TIM3_CH3,af_pp,HIGH,NOPULL,MOTOR_PWM,motor",
        "PWM_SERVO,PB,1,TIM3_CH4,af_pp,HIGH,NOPULL,SERVO_PWM,motor",
    ]
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
    print(f"[OK] 示例引脚表已生成: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='板级配置生成器 - 从引脚表生成SysConfig/CubeMX配置',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 生成示例引脚CSV
  python board_config_generator.py --init --output pins.csv

  # 从CSV生成配置
  python board_config_generator.py --input pins.csv --mcu STM32F407VGT6 --format all

  # 仅生成SysConfig配置
  python board_config_generator.py --input pins.csv --mcu MSP432P401R --format sysconfig

  # 从JSON生成CubeMX配置
  python board_config_generator.py --input pins.json --mcu STM32F103C8T6 --format cubemx
        """
    )
    parser.add_argument('--input', '-i', help='引脚配置文件路径 (CSV或JSON)')
    parser.add_argument('--output', '-o', help='输出文件路径前缀', default='board_config')
    parser.add_argument('--format', '-f', choices=['sysconfig', 'cubemx', 'report', 'all'],
                        default='all', help='输出格式 (默认: all)')
    parser.add_argument('--mcu', '-m', default='STM32F407VGT6', help='MCU型号 (默认: STM32F407VGT6)')
    parser.add_argument('--init', action='store_true', help='生成示例引脚CSV文件')
    parser.add_argument('--check-only', action='store_true', help='仅检查冲突，不生成配置')

    args = parser.parse_args()

    # 初始化模式
    if args.init:
        output = args.output if args.output != 'board_config' else 'sample_pins.csv'
        generate_sample_csv(output)
        return

    # 必须提供输入文件
    if not args.input:
        parser.error("请提供引脚配置文件 (--input) 或使用 --init 生成示例文件")

    # 解析引脚表
    ext = os.path.splitext(args.input)[1].lower()
    if ext == '.csv':
        pins = parse_pin_csv(args.input)
    elif ext == '.json':
        pins = parse_pin_json(args.input)
    else:
        print(f"[错误] 不支持的文件格式: {ext}，请使用 .csv 或 .json")
        sys.exit(1)

    print(f"[信息] 已加载 {len(pins)} 个引脚配置")

    # 冲突检查
    conflicts = check_pin_conflicts(pins)
    resource_conflicts = check_resource_conflicts(pins)

    if conflicts:
        print(f"\n[警告] 发现 {len(conflicts)} 个引脚冲突:")
        for pin_id, funcs in conflicts:
            print(f"  {pin_id}: {', '.join(funcs)}")

    if resource_conflicts:
        print(f"\n[警告] 发现 {len(resource_conflicts)} 个资源冲突:")
        for resource, pin_list in resource_conflicts:
            print(f"  {resource}: {', '.join(pin_list)}")

    if not conflicts and not resource_conflicts:
        print("[OK] 未发现引脚冲突")

    if args.check_only:
        return

    # 生成配置文件
    base = os.path.splitext(args.output)[0]

    if args.format in ('sysconfig', 'all'):
        generate_sysconfig(pins, args.mcu, f"{base}_sysconfig.cfg")

    if args.format in ('cubemx', 'all'):
        generate_cubemx_ioc(pins, args.mcu, f"{base}_cubemx.ioc")

    if args.format in ('report', 'all'):
        generate_pin_report(pins, f"{base}_report.md")

    print("\n[完成] 配置文件生成完毕")


if __name__ == '__main__':
    main()
