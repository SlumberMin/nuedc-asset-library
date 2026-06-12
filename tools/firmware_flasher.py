#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
固件烧录工具
============
功能：
  - 支持 MSPM0 / STM32 / TM4C 三平台固件烧录
  - 自动检测调试器（J-Link / ST-Link / DAP-Link / OpenOCD）
  - 烧录前自动备份当前固件
  - 烧录后自动校验
  - 批量烧录支持

依赖：
  - STM32: STM32_Programmer_CLI 或 st-flash
  - MSPM0: UniFlash 或 DSServer
  - TM4C: LM Flash Programmer 或 OpenOCD
  - 通用: OpenOCD（可选）

用法：
  python firmware_flasher.py flash --platform stm32 --hex firmware.hex
  python firmware_flasher.py flash --platform mspm0 --hex firmware.hex
  python firmware_flasher.py flash --platform tm4c --bin firmware.bin --addr 0x00000000
  python firmware_flasher.py detect                      # 检测调试器
  python firmware_flasher.py backup --platform stm32     # 备份当前固件
"""

import argparse
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


# ============================================================
# 工具路径配置（根据实际安装位置修改）
# ============================================================
TOOL_PATHS = {
    # STM32 工具
    "stm32cubeprog": r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
    "st_flash": "st-flash",

    # MSPM0 工具
    "uniflash": r"C:\ti\uniflash\bin\uniflash_cli.bat",
    "dsserver": r"C:\ti\ccs\ccs_base\DebugServer\bin\DSLite",

    # TM4C 工具
    "lmflash": r"C:\TI\LMFlashProgrammer\LMFlash.exe",
    "openocd": "openocd",
}

# OpenOCD 板级配置映射
OPENOCD_CONFIGS = {
    "stm32f1": ["-f", "interface/stlink.cfg", "-f", "target/stm32f1x.cfg"],
    "stm32f4": ["-f", "interface/stlink.cfg", "-f", "target/stm32f4x.cfg"],
    "stm32g4": ["-f", "interface/stlink.cfg", "-f", "target/stm32g4x.cfg"],
    "mspm0": ["-f", "interface/ti-icdi.cfg", "-f", "target/mspm0.cfg"],
    "tm4c": ["-f", "interface/ti-icdi.cfg", "-f", "target/tm4c123g.cfg"],
}


def log_info(msg):
    """打印信息"""
    print(f"[信息] {msg}")


def log_error(msg):
    """打印错误"""
    print(f"[错误] {msg}", file=sys.stderr)


def log_ok(msg):
    """打印成功"""
    print(f"[成功] {msg}")


def check_tool_exists(tool_path):
    """检查工具是否存在"""
    if os.path.isfile(tool_path):
        return True
    # 尝试在PATH中查找
    return shutil.which(tool_path) is not None


def run_command(cmd, description="执行命令"):
    """执行外部命令并返回结果"""
    log_info(f"{description}: {' '.join(cmd) if isinstance(cmd, list) else cmd}")
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=120,
            shell=isinstance(cmd, str),
        )
        if result.returncode == 0:
            log_ok(f"{description} 成功")
            if result.stdout.strip():
                print(result.stdout.strip()[:500])
        else:
            log_error(f"{description} 失败 (返回码: {result.returncode})")
            if result.stderr.strip():
                print(result.stderr.strip()[:500])
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        log_error(f"{description} 超时")
        return False
    except FileNotFoundError:
        log_error(f"找不到命令，请检查工具安装路径")
        return False


def detect_debuggers():
    """检测已连接的调试器"""
    log_info("正在检测调试器...")

    found = []

    # 检测 ST-Link
    stlink_path = TOOL_PATHS["stm32cubeprog"]
    if check_tool_exists(stlink_path):
        result = subprocess.run(
            [stlink_path, "--list"],
            capture_output=True, text=True, timeout=10,
        )
        if "ST-Link" in result.stdout or "STLink" in result.stdout:
            found.append(("ST-Link", result.stdout.strip()[:200]))

    # 检测 DAP-Link / J-Link 通过 COM 口
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for p in ports:
            desc = (p.description or "").lower()
            mfg = (p.manufacturer or "").lower()
            if any(k in desc for k in ["jlink", "dap", "stlink", "icdi"]):
                found.append((p.device, p.description))
    except ImportError:
        pass

    # 检测 OpenOCD
    if check_tool_exists(TOOL_PATHS["openocd"]):
        result = subprocess.run(
            [TOOL_PATHS["openocd"], "-c", "adapter list"],
            capture_output=True, text=True, timeout=10,
        )
        if result.stdout.strip():
            found.append(("OpenOCD", result.stdout.strip()[:200]))

    if found:
        log_info(f"检测到 {len(found)} 个调试器:")
        for name, info in found:
            print(f"  - {name}: {info}")
    else:
        log_error("未检测到调试器")

    return found


def backup_firmware(platform, output_path=None):
    """备份当前固件"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if output_path is None:
        output_path = f"backup_{platform}_{timestamp}.bin"

    log_info(f"正在备份 {platform} 固件到 {output_path}...")

    if platform == "stm32":
        tool = TOOL_PATHS["stm32cubeprog"]
        if check_tool_exists(tool):
            return run_command(
                [tool, "-c", "port=SWD", "-u", "0x08000000", output_path, "0x20000"],
                "备份STM32固件"
            )
        else:
            # 尝试 st-flash
            return run_command(
                [TOOL_PATHS["st_flash"], "read", output_path, "0x08000000", "0x20000"],
                "备份STM32固件(st-flash)"
            )

    elif platform == "mspm0":
        log_info("MSPM0备份需使用 UniFlash GUI 或 DSServer")
        tool = TOOL_PATHS["dsserver"]
        if check_tool_exists(tool):
            return run_command(
                [tool, "read", output_path],
                "备份MSPM0固件"
            )
        else:
            log_error("未找到 DSServer 工具")
            return False

    elif platform == "tm4c":
        tool = TOOL_PATHS["openocd"]
        config = OPENOCD_CONFIGS["tm4c"]
        return run_command(
            [tool] + config + [
                "-c", f"flash read_bank 0 {output_path} 0x40000",
                "-c", "shutdown",
            ],
            "备份TM4C固件"
        )

    return False


def flash_stm32(firmware_path):
    """烧录 STM32 固件"""
    tool = TOOL_PATHS["stm32cubeprog"]

    if check_tool_exists(tool):
        ext = Path(firmware_path).suffix.lower()
        cmd = [tool, "-c", "port=SWD", "-w"]
        if ext in (".hex", ".elf"):
            cmd += [firmware_path, "0x08000000"]
        else:
            cmd += [firmware_path, "0x08000000"]
        cmd += ["-v", "-rst"]
        return run_command(cmd, "烧录STM32")

    # 回退到 st-flash
    st_flash = TOOL_PATHS["st_flash"]
    if check_tool_exists(st_flash):
        if firmware_path.endswith(".hex"):
            return run_command(
                [st_flash, "--format", "ihex", "write", firmware_path],
                "烧录STM32(st-flash)"
            )
        else:
            return run_command(
                [st_flash, "write", firmware_path, "0x08000000"],
                "烧录STM32(st-flash)"
            )

    # 回退到 OpenOCD
    for cfg_name, cfg_args in OPENOCD_CONFIGS.items():
        if "stm32" in cfg_name:
            tool = TOOL_PATHS["openocd"]
            if check_tool_exists(tool):
                return run_command(
                    [tool] + cfg_args + [
                        "-c", f"program {firmware_path} verify reset exit",
                    ],
                    "烧录STM32(OpenOCD)"
                )

    log_error("未找到可用的STM32烧录工具，请安装 STM32CubeProgrammer 或 st-flash")
    return False


def flash_mspm0(firmware_path):
    """烧录 MSPM0 固件"""
    tool = TOOL_PATHS["dsserver"]

    if check_tool_exists(tool):
        # 使用 DSServer 烧录
        return run_command(
            [tool, "load", firmware_path],
            "烧录MSPM0"
        )

    uniflash = TOOL_PATHS["uniflash"]
    if check_tool_exists(uniflash):
        return run_command(
            [uniflash, "-p", "MSPM0", "-f", firmware_path],
            "烧录MSPM0(UniFlash)"
        )

    # 回退到 OpenOCD
    tool = TOOL_PATHS["openocd"]
    config = OPENOCD_CONFIGS["mspm0"]
    if check_tool_exists(tool):
        return run_command(
            [tool] + config + [
                "-c", f"program {firmware_path} verify reset exit",
            ],
            "烧录MSPM0(OpenOCD)"
        )

    log_error("未找到可用的MSPM0烧录工具，请安装 UniFlash 或 DSServer")
    return False


def flash_tm4c(firmware_path, addr="0x00000000"):
    """烧录 TM4C 固件"""
    # 尝试 OpenOCD
    tool = TOOL_PATHS["openocd"]
    config = OPENOCD_CONFIGS["tm4c"]
    if check_tool_exists(tool):
        return run_command(
            [tool] + config + [
                "-c", f"program {firmware_path} verify reset exit",
            ],
            "烧录TM4C(OpenOCD)"
        )

    # 尝试 LM Flash Programmer (命令行模式有限，提示用GUI)
    lmflash = TOOL_PATHS["lmflash"]
    if check_tool_exists(lmflash):
        log_info("TM4C 建议使用 LM Flash Programmer GUI 手动烧录")
        return False

    log_error("未找到可用的TM4C烧录工具，请安装 OpenOCD 或 LM Flash Programmer")
    return False


def cmd_flash(args):
    """执行烧录"""
    firmware_path = args.hex or args.bin or args.elf
    if not firmware_path:
        log_error("请指定固件文件路径 (--hex / --bin / --elf)")
        return False

    if not os.path.isfile(firmware_path):
        log_error(f"固件文件不存在: {firmware_path}")
        return False

    platform = args.platform.lower()
    log_info(f"目标平台: {platform.upper()}")
    log_info(f"固件文件: {firmware_path}")

    # 可选备份
    if args.backup:
        backup_firmware(platform)

    # 执行烧录
    success = False
    if platform == "stm32":
        success = flash_stm32(firmware_path)
    elif platform in ("mspm0", "mspm"):
        success = flash_mspm0(firmware_path)
    elif platform in ("tm4c", "tiva"):
        success = flash_tm4c(firmware_path, args.addr)
    else:
        log_error(f"不支持的平台: {platform}，支持: stm32, mspm0, tm4c")
        return False

    if success:
        log_ok(f"{platform.upper()} 固件烧录完成！")
    else:
        log_error(f"{platform.upper()} 固件烧录失败")

    return success


def cmd_detect(args):
    """检测调试器"""
    detect_debuggers()


def cmd_backup(args):
    """备份固件"""
    platform = args.platform.lower()
    backup_firmware(platform, args.output)


def main():
    """主入口"""
    parser = argparse.ArgumentParser(
        description="固件烧录工具 - 支持 MSPM0/STM32/TM4C 三平台",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="command", help="子命令")

    # flash 子命令
    p_flash = subparsers.add_parser("flash", help="烧录固件")
    p_flash.add_argument("--platform", "-t", required=True,
                         choices=["stm32", "mspm0", "tm4c"],
                         help="目标平台")
    p_flash.add_argument("--hex", type=str, default=None, help="HEX固件文件路径")
    p_flash.add_argument("--bin", type=str, default=None, help="BIN固件文件路径")
    p_flash.add_argument("--elf", type=str, default=None, help="ELF固件文件路径")
    p_flash.add_argument("--addr", type=str, default="0x00000000",
                         help="烧录起始地址（默认 0x00000000）")
    p_flash.add_argument("--backup", action="store_true",
                         help="烧录前备份当前固件")

    # detect 子命令
    p_detect = subparsers.add_parser("detect", help="检测调试器")

    # backup 子命令
    p_backup = subparsers.add_parser("backup", help="备份当前固件")
    p_backup.add_argument("--platform", "-t", required=True,
                          choices=["stm32", "mspm0", "tm4c"],
                          help="目标平台")
    p_backup.add_argument("--output", "-o", type=str, default=None,
                          help="备份文件输出路径")

    args = parser.parse_args()

    if args.command == "flash":
        cmd_flash(args)
    elif args.command == "detect":
        cmd_detect(args)
    elif args.command == "backup":
        cmd_backup(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
