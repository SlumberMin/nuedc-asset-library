#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
驱动代码生成器 - 从传感器数据手册/描述自动生成驱动模板
用法: python driver_generator.py --name BMP280 --bus i2c --addr 0x76 --output ./drivers/
"""

import argparse
import os
import json
from datetime import datetime


# 常见传感器寄存器映射模板
SENSOR_TEMPLATES = {
    "bmp280": {"bus": "i2c", "addr": "0x76", "regs": [
        ("CHIPID", "0xD0", "r", "芯片ID"),
        ("CTRL_MEAS", "0xF4", "rw", "测量控制"),
        ("CONFIG", "0xF5", "rw", "配置"),
        ("PRESS_MSB", "0xF7", "r", "气压高位"),
        ("TEMP_MSB", "0xFA", "r", "温度高位"),
    ]},
    "mpu6050": {"bus": "i2c", "addr": "0x68", "regs": [
        ("WHO_AM_I", "0x75", "r", "设备ID"),
        ("PWR_MGMT_1", "0x6B", "rw", "电源管理1"),
        ("ACCEL_XOUT_H", "0x3B", "r", "加速度X高位"),
        ("GYRO_XOUT_H", "0x43", "r", "陀螺仪X高位"),
    ]},
    "ssd1306": {"bus": "i2c", "addr": "0x3C", "regs": [
        ("COMMAND", "0x00", "w", "命令模式"),
        ("DATA", "0x40", "w", "数据模式"),
    ]},
}


def generate_header(name: str, bus: str, addr: str, regs: list) -> str:
    """生成驱动头文件"""
    guard = f"{name.upper()}_H"
    lines = [
        f"/**",
        f" * @file {name.lower()}.h",
        f" * @brief {name.upper()} 传感器驱动头文件",
        f" * @note  由 driver_generator.py 自动生成",
        f" * @date  {datetime.now().strftime('%Y-%m-%d')}",
        f" */",
        f"#ifndef {guard}",
        f"#define {guard}",
        f"",
        f'#include <stdint.h>',
        f'#include <stdbool.h>',
        f"",
        f"/* 设备地址 */",
        f"#define {name.upper()}_ADDR  ({addr})",
        f"",
        f"/* 寄存器定义 */",
    ]
    for reg_name, reg_addr, access, desc in regs:
        lines.append(f"#define {name.upper()}_{reg_name:<20s}  ({reg_addr})  /* {desc} */")

    lines += [
        "",
        f"/* 错误码 */",
        f"typedef enum {{",
        f"    {name.upper()}_OK = 0,",
        f"    {name.upper()}_ERR_PARAM,",
        f"    {name.upper()}_ERR_BUS,",
        f"    {name.upper()}_ERR_TIMEOUT,",
        f"}} {name.lower()}_err_t;",
        "",
        f"/* 设备句柄 */",
        f"typedef struct {{",
        f"    uint8_t addr;       /* 设备地址 */",
        f"    void   *bus_handle; /* 总线句柄 */",
        f"}} {name.lower()}_dev_t;",
        "",
        f"/* @brief 初始化设备 */",
        f"int {name.lower()}_init({name.lower()}_dev_t *dev, void *bus_handle);",
        "",
        f"/* @brief 读取寄存器 */",
        f"int {name.lower()}_read_reg({name.lower()}_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);",
        "",
        f"/* @brief 写入寄存器 */",
        f"int {name.lower()}_write_reg({name.lower()}_dev_t *dev, uint8_t reg, const uint8_t *buf, uint16_t len);",
        "",
        f"/* @brief 设备自检 */",
        f"int {name.lower()}_self_test({name.lower()}_dev_t *dev);",
        "",
        f"#endif /* {guard} */",
    ]
    return "\n".join(lines)


def generate_source(name: str, bus: str, addr: str, regs: list) -> str:
    """生成驱动源文件"""
    lines = [
        f"/**",
        f" * @file {name.lower()}.c",
        f" * @brief {name.upper()} 传感器驱动实现",
        f" * @note  由 driver_generator.py 自动生成",
        f" * @date  {datetime.now().strftime('%Y-%m-%d')}",
        f" */",
        f"",
        f'#include "{name.lower()}.h"',
        f"",
        f"",
        f"int {name.lower()}_init({name.lower()}_dev_t *dev, void *bus_handle) {{",
        f"    if (!dev || !bus_handle) return {name.upper()}_ERR_PARAM;",
        f"    dev->bus_handle = bus_handle;",
        f"    dev->addr = {name.upper()}_ADDR;",
        f"",
        f"    /* 自检 */",
        f"    int ret = {name.lower()}_self_test(dev);",
        f"    if (ret != {name.upper()}_OK) return ret;",
        f"",
        f"    /* TODO: 用户自定义初始化序列 */",
        f"",
        f"    return {name.upper()}_OK;",
        f"}}",
        "",
        f"int {name.lower()}_read_reg({name.lower()}_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len) {{",
        f"    if (!dev || !buf) return {name.upper()}_ERR_PARAM;",
        f"    /* TODO: 调用实际{bus.upper()}总线读取 */",
        f"    /* 示例: {bus}_read(dev->bus_handle, dev->addr, reg, buf, len); */",
        f"    return {name.upper()}_OK;",
        f"}}",
        "",
        f"int {name.lower()}_write_reg({name.lower()}_dev_t *dev, uint8_t reg, const uint8_t *buf, uint16_t len) {{",
        f"    if (!dev || !buf) return {name.upper()}_ERR_PARAM;",
        f"    /* TODO: 调用实际{bus.upper()}总线写入 */",
        f"    return {name.upper()}_OK;",
        f"}}",
        "",
        f"int {name.lower()}_self_test({name.lower()}_dev_t *dev) {{",
        f"    uint8_t id = 0;",
        f"    int ret = {name.lower()}_read_reg(dev, {name.upper()}_{regs[0][0]}, &id, 1);",
        f"    if (ret != {name.upper()}_OK) return ret;",
        f"    /* TODO: 验证ID值 */",
        f"    return {name.upper()}_OK;",
        f"}}",
    ]
    return "\n".join(lines)


def generate_from_custom(name: str, bus: str, addr: str, reg_json: str) -> tuple:
    """从JSON文件加载自定义寄存器表"""
    with open(reg_json, "r", encoding="utf-8") as f:
        data = json.load(f)
    regs = [(r["name"], r["addr"], r.get("access", "rw"), r.get("desc", "")) for r in data]
    return regs


def main():
    parser = argparse.ArgumentParser(description="驱动代码生成器 - 自动生成传感器驱动模板")
    parser.add_argument("--name", required=True, help="传感器名称 (如 BMP280, MPU6050)")
    parser.add_argument("--bus", default="i2c", choices=["i2c", "spi", "uart"], help="总线类型")
    parser.add_argument("--addr", default="0x00", help="设备地址 (如 0x76)")
    parser.add_argument("--regs-json", help="自定义寄存器表JSON文件路径")
    parser.add_argument("--output", default=".", help="输出目录")
    parser.add_argument("--template", choices=list(SENSOR_TEMPLATES.keys()), help="使用内置模板")
    args = parser.parse_args()

    name = args.name.lower()

    # 确定寄存器表
    if args.template and args.template in SENSOR_TEMPLATES:
        tmpl = SENSOR_TEMPLATES[args.template]
        regs = tmpl["regs"]
        bus = tmpl["bus"]
        addr = tmpl["addr"]
    elif args.regs_json:
        regs = generate_from_custom(name, args.bus, args.addr, args.regs_json)
        bus = args.bus
        addr = args.addr
    else:
        # 默认空寄存器表，用户自行填充
        regs = [
            ("CHIP_ID", "0x00", "r", "芯片ID (请修改)"),
            ("CTRL_REG", "0x01", "rw", "控制寄存器 (请修改)"),
        ]
        bus = args.bus
        addr = args.addr

    # 生成文件
    os.makedirs(args.output, exist_ok=True)
    h_path = os.path.join(args.output, f"{name}.h")
    c_path = os.path.join(args.output, f"{name}.c")

    with open(h_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(generate_header(name, bus, addr, regs))
    with open(c_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(generate_source(name, bus, addr, regs))

    print(f"[✓] 驱动模板已生成:")
    print(f"    头文件: {h_path}")
    print(f"    源文件: {c_path}")
    print(f"    总线: {bus.upper()}, 地址: {addr}, 寄存器数: {len(regs)}")


if __name__ == "__main__":
    main()
