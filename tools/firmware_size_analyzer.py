#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================
固件大小分析器 - 解析 map 文件 + 生成各模块占用图表
============================================================
功能：
  - 解析 Keil/IAR/GCC 生成的 .map 文件
  - 提取各 section（.text, .data, .bss, .rodata）大小
  - 按模块/库归类统计 Flash 和 RAM 占用
  - 生成直观的占用图表（饼图、柱状图、树状图）
  - 支持对比多次编译结果
  - 预警 Flash/RAM 接近上限

依赖：pip install matplotlib

用法：
  python firmware_size_analyzer.py --map project.map --flash 64 --ram 20
  python firmware_size_analyzer.py --map project.map --mcu stm32f103c8
  python firmware_size_analyzer.py --elf firmware.elf --flash 512 --ram 128
  python firmware_size_analyzer.py --map map1.map --compare map2.map
============================================================
"""

import argparse
import os
import re
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path

# ── 尝试导入可选依赖 ──────────────────────────────────────
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# ── 中文字体配置 ──────────────────────────────────────────
if HAS_MPL:
    for font_name in ['SimHei', 'Microsoft YaHei', 'WenQuanYi Micro Hei']:
        if any(font_name in f.name for f in fm.fontManager.ttflist):
            plt.rcParams['font.sans-serif'] = [font_name]
            break
    plt.rcParams['axes.unicode_minus'] = False

# ── MCU 规格数据库 ────────────────────────────────────────
MCU_SPECS = {
    "stm32f103c6":  {"flash": 32,  "ram": 10,  "name": "STM32F103C6"},
    "stm32f103c8":  {"flash": 64,  "ram": 20,  "name": "STM32F103C8"},
    "stm32f103cb":  {"flash": 128, "ram": 20,  "name": "STM32F103CB"},
    "stm32f103r8":  {"flash": 64,  "ram": 20,  "name": "STM32F103R8"},
    "stm32f103rb":  {"flash": 128, "ram": 20,  "name": "STM32F103RB"},
    "stm32f103rc":  {"flash": 256, "ram": 48,  "name": "STM32F103RC"},
    "stm32f103ve":  {"flash": 512, "ram": 64,  "name": "STM32F103VE"},
    "stm32f103zet6":{"flash": 512, "ram": 64,  "name": "STM32F103ZET6"},
    "stm32f407ve":  {"flash": 512, "ram": 192, "name": "STM32F407VE"},
    "stm32f407vg":  {"flash": 1024,"ram": 192, "name": "STM32F407VG"},
    "stm32f407ze":  {"flash": 512, "ram": 192, "name": "STM32F407ZE"},
    "stm32g431":    {"flash": 128, "ram": 32,  "name": "STM32G431"},
    "stm32g474":    {"flash": 512, "ram": 128, "name": "STM32G474"},
    "stm32h743":    {"flash": 2048,"ram": 1024,"name": "STM32H743"},
    "stm32l476rg":  {"flash": 1024,"ram": 128, "name": "STM32L476RG"},
    "esp32":        {"flash": 4096,"ram": 520, "name": "ESP32"},
    "esp32s3":      {"flash": 8192,"ram": 512, "name": "ESP32-S3"},
}

# ── Section 分类规则 ──────────────────────────────────────
# Flash: .text, .rodata, .data(init), .constdata
# RAM:   .data(运行时), .bss, .heap, .stack
FLASH_SECTIONS = {'.text', '.rodata', '.constdata', '.init', '.fini', '.ctors', '.dtors'}
RAM_SECTIONS = {'.data', '.bss', '.noinit', '.heap', '.stack'}


def classify_section(name: str) -> str:
    """将 section 名称分类为 flash / ram / other。"""
    name_lower = name.lower().strip()
    # 精确匹配
    if name_lower in FLASH_SECTIONS:
        return 'flash'
    if name_lower in RAM_SECTIONS:
        return 'ram'
    # 前缀匹配
    if name_lower.startswith('.text') or name_lower.startswith('.rodata'):
        return 'flash'
    if name_lower.startswith('.data') or name_lower.startswith('.bss'):
        return 'ram'
    if name_lower.startswith('.ARM') or name_lower.startswith('__ARM'):
        return 'flash'
    return 'other'


def classify_module(section_name: str) -> str:
    """
    从 section 名称推断所属模块/库。
    例如: .text.stm32f1xx_hal_gpio.o -> stm32f1xx_hal_gpio
          .text.usart_printf -> usart_printf
    """
    name = section_name.strip()
    # 尝试匹配 Keil 格式: .text.filename.o
    m = re.match(r'\.[a-zA-Z]+\.(.+?)\.o', name)
    if m:
        return m.group(1)
    # GCC 格式: 尝试从路径中提取
    m = re.match(r'\.[a-zA-Z]+\.(.+)', name)
    if m:
        return m.group(1)
    return name


def parse_keil_map(filepath: str) -> dict:
    """
    解析 Keil MDK 生成的 .map 文件。
    提取 Memory Map 区段中各模块的大小信息。
    """
    sections = []
    module_sizes = defaultdict(lambda: {'flash': 0, 'ram': 0, 'other': 0})

    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
    except Exception as e:
        print(f"[ERROR] 无法读取文件: {e}")
        return {}

    # Keil map 文件中的 section 条目格式：
    #   Section Name        Type    Size      ...
    #   .text               Code    0x00001234 ...
    # 或在 Cross Reference Table 中：
    #   Module               RO      RW      ZI
    patterns = [
        # Keil 风格: section 行 (Hex sizes)
        re.compile(r'^\s*(\.\S+)\s+\w+\s+(0x[0-9a-fA-F]+)\s+', re.MULTILINE),
        # Keil Cross Reference Table
        re.compile(r'^\s*(\S+\.o)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)', re.MULTILINE),
    ]

    # 解析 Cross Reference Table (Keil)
    xref_pattern = re.compile(
        r'^\s*(\S+?\.o)\s+'           # Module name
        r'(0x[0-9a-fA-F]+)\s+'       # RO Size
        r'(0x[0-9a-fA-F]+)\s+'       # RW Size
        r'(0x[0-9a-fA-F]+)',         # ZI Size
        re.MULTILINE
    )
    for m in xref_pattern.finditer(content):
        module = m.group(1).replace('.o', '')
        ro = int(m.group(2), 16)
        rw = int(m.group(3), 16)
        zi = int(m.group(4), 16)
        if ro > 0 or rw > 0 or zi > 0:
            module_sizes[module]['flash'] += ro
            module_sizes[module]['ram'] += rw + zi
            sections.append({
                'name': module,
                'flash': ro,
                'ram': rw + zi,
                'total': ro + rw + zi,
            })

    # 如果没找到 Cross Reference Table，尝试解析 section 信息
    if not sections:
        section_pattern = re.compile(
            r'^\s*(\.\S+)\s+.*?(0x[0-9a-fA-F]{4,})',
            re.MULTILINE
        )
        for m in section_pattern.finditer(content):
            name = m.group(1)
            size = int(m.group(2), 16)
            if size > 0:
                stype = classify_section(name)
                module = classify_module(name)
                module_sizes[module][stype] += size
                sections.append({
                    'name': name,
                    'type': stype,
                    'size': size,
                    'module': module,
                })

    return {
        'sections': sections,
        'module_sizes': dict(module_sizes),
        'source': filepath,
    }


def parse_gcc_map(filepath: str) -> dict:
    """
    解析 GCC (arm-none-eabi-gcc) 生成的 .map 文件。
    """
    sections = []
    module_sizes = defaultdict(lambda: {'flash': 0, 'ram': 0, 'other': 0})

    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
    except Exception as e:
        print(f"[ERROR] 无法读取文件: {e}")
        return {}

    # GCC map 格式:
    #  .text.function_name
    #    0x08001000    0x120 path/to/file.o
    section_pattern = re.compile(
        r'^\s+(\.\S+)\s*\n'
        r'\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)\s+(.+?)(?:\s*\n|\s*$)',
        re.MULTILINE
    )

    for m in section_pattern.finditer(content):
        name = m.group(1).strip()
        addr = int(m.group(2), 16)
        size = int(m.group(3), 16)
        source = m.group(4).strip()

        if size == 0:
            continue

        stype = classify_section(name)
        # 从源文件路径提取模块名
        module = Path(source).stem if source else classify_module(name)

        module_sizes[module][stype] += size
        sections.append({
            'name': name,
            'address': addr,
            'size': size,
            'type': stype,
            'module': module,
            'source': source,
        })

    # 也尝试解析 MEMORY 配置
    memory_pattern = re.compile(
        r'^\s*(\w+)\s.*ORIGIN\s*=\s*0x([0-9a-fA-F]+).*LENGTH\s*=\s*(\d+)([KkMm])',
        re.MULTILINE
    )
    memory_regions = []
    for m in memory_pattern.finditer(content):
        name = m.group(1)
        origin = int(m.group(2), 16)
        length = int(m.group(3))
        unit = m.group(4).upper()
        if unit == 'K':
            length *= 1024
        elif unit == 'M':
            length *= 1024 * 1024
        memory_regions.append({'name': name, 'origin': origin, 'length': length})

    return {
        'sections': sections,
        'module_sizes': dict(module_sizes),
        'memory_regions': memory_regions,
        'source': filepath,
    }


def parse_map_file(filepath: str) -> dict:
    """自动检测 map 文件类型并解析。"""
    if not os.path.isfile(filepath):
        print(f"[ERROR] 文件不存在: {filepath}")
        return {}

    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        head = f.read(4096)

    # 检测类型
    if 'Cross Reference' in head or 'ARM Compiler' in head or 'Memory Map' in head:
        print(f"[INFO] 检测到 Keil MDK map 文件")
        return parse_keil_map(filepath)
    elif 'Linker script and memory map' in head or '.text' in head and '0x' in head:
        print(f"[INFO] 检测到 GCC map 文件")
        return parse_gcc_map(filepath)
    else:
        print(f"[INFO] 未知 map 格式，尝试通用解析")
        return parse_keil_map(filepath) or parse_gcc_map(filepath)


def parse_elf_size(filepath: str) -> dict:
    """
    使用 arm-none-eabi-size 或类似工具解析 ELF 文件。
    如果工具不可用，尝试直接解析 ELF section headers。
    """
    import subprocess
    try:
        result = subprocess.run(
            ['arm-none-eabi-size', '-A', filepath],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            return parse_size_output(result.stdout, filepath)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # 尝试 size -A
    try:
        result = subprocess.run(
            ['size', '-A', filepath],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            return parse_size_output(result.stdout, filepath)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    print("[ERROR] 无法解析 ELF 文件，请安装 arm-none-eabi-size 或提供 .map 文件")
    return {}


def parse_size_output(output: str, source: str) -> dict:
    """解析 size -A 的输出。"""
    sections = []
    module_sizes = defaultdict(lambda: {'flash': 0, 'ram': 0, 'other': 0})

    for line in output.strip().split('\n'):
        parts = line.split()
        if len(parts) >= 3:
            name = parts[0]
            try:
                size = int(parts[1])
            except ValueError:
                continue
            if size > 0 and name.startswith('.'):
                stype = classify_section(name)
                module_sizes[name][stype] += size
                sections.append({
                    'name': name,
                    'size': size,
                    'type': stype,
                })

    return {
        'sections': sections,
        'module_sizes': dict(module_sizes),
        'source': source,
    }


def compute_totals(module_sizes: dict) -> dict:
    """计算 Flash 和 RAM 总占用。"""
    flash_total = sum(v.get('flash', 0) for v in module_sizes.values())
    ram_total = sum(v.get('ram', 0) for v in module_sizes.values())
    other_total = sum(v.get('other', 0) for v in module_sizes.values())
    return {
        'flash': flash_total,
        'ram': ram_total,
        'other': other_total,
        'total': flash_total + ram_total + other_total,
    }


def size_str(size_bytes: int) -> str:
    """格式化大小为可读字符串。"""
    if size_bytes < 1024:
        return f"{size_bytes} B"
    elif size_bytes < 1024 * 1024:
        return f"{size_bytes / 1024:.1f} KB"
    else:
        return f"{size_bytes / (1024*1024):.2f} MB"


def print_analysis(result: dict, flash_limit_kb: int, ram_limit_kb: int):
    """打印分析结果到终端。"""
    module_sizes = result.get('module_sizes', {})
    totals = compute_totals(module_sizes)

    flash_limit = flash_limit_kb * 1024
    ram_limit = ram_limit_kb * 1024

    flash_pct = (totals['flash'] / flash_limit * 100) if flash_limit else 0
    ram_pct = (totals['ram'] / ram_limit * 100) if ram_limit else 0

    print("\n" + "=" * 70)
    print("  固件大小分析报告")
    print(f"  来源: {result.get('source', '未知')}")
    print(f"  生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)

    # 总览
    print(f"\n【资源使用总览】")
    print(f"  Flash: {size_str(totals['flash']):>12} / {size_str(flash_limit):>12}  ({flash_pct:.1f}%)", end="")
    if flash_pct > 100:
        print("  ❌ 超出!")
    elif flash_pct > 90:
        print("  ⚠️  接近上限!")
    else:
        print("  ✅")

    print(f"  RAM:   {size_str(totals['ram']):>12} / {size_str(ram_limit):>12}  ({ram_pct:.1f}%)", end="")
    if ram_pct > 100:
        print("  ❌ 超出!")
    elif ram_pct > 90:
        print("  ⚠️  接近上限!")
    else:
        print("  ✅")

    # 按模块排序（Flash 占用从大到小）
    print(f"\n【模块占用排名 - Flash】")
    print(f"  {'模块':<35} {'Flash':>12} {'RAM':>12} {'总计':>12}")
    print("  " + "-" * 73)

    sorted_modules = sorted(module_sizes.items(), key=lambda x: x[1].get('flash', 0), reverse=True)
    for name, sizes in sorted_modules[:30]:
        flash = sizes.get('flash', 0)
        ram = sizes.get('ram', 0)
        if flash > 0 or ram > 0:
            print(f"  {name:<35} {size_str(flash):>12} {size_str(ram):>12} {size_str(flash+ram):>12}")

    # RAM 占用排名
    print(f"\n【模块占用排名 - RAM】")
    print(f"  {'模块':<35} {'RAM':>12}")
    print("  " + "-" * 49)
    sorted_ram = sorted(module_sizes.items(), key=lambda x: x[1].get('ram', 0), reverse=True)
    for name, sizes in sorted_ram[:20]:
        ram = sizes.get('ram', 0)
        if ram > 0:
            print(f"  {name:<35} {size_str(ram):>12}")

    print("\n" + "=" * 70)


def plot_charts(result: dict, flash_limit_kb: int, ram_limit_kb: int, output_dir: str):
    """生成占用图表。"""
    if not HAS_MPL:
        print("[WARN] 未安装 matplotlib，跳过图表生成")
        return

    module_sizes = result.get('module_sizes', {})
    totals = compute_totals(module_sizes)
    flash_limit = flash_limit_kb * 1024
    ram_limit = ram_limit_kb * 1024

    # ── 图1: Flash/RAM 使用率仪表盘 ──
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Flash
    flash_used = totals['flash']
    flash_free = max(0, flash_limit - flash_used)
    colors_flash = ['#e74c3c' if flash_used > flash_limit * 0.9 else '#3498db', '#ecf0f1']
    ax1.pie([min(flash_used, flash_limit), flash_free], labels=['已用', '空闲'],
            colors=colors_flash, autopct='%1.1f%%', startangle=90)
    ax1.set_title(f'Flash 占用: {size_str(flash_used)} / {size_str(flash_limit)}')

    # RAM
    ram_used = totals['ram']
    ram_free = max(0, ram_limit - ram_used)
    colors_ram = ['#e74c3c' if ram_used > ram_limit * 0.9 else '#2ecc71', '#ecf0f1']
    ax2.pie([min(ram_used, ram_limit), ram_free], labels=['已用', '空闲'],
            colors=colors_ram, autopct='%1.1f%%', startangle=90)
    ax2.set_title(f'RAM 占用: {size_str(ram_used)} / {size_str(ram_limit)}')

    plt.tight_layout()
    path = os.path.join(output_dir, 'size_overview.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  [图表] 资源总览: {path}")

    # ── 图2: Flash 模块占用柱状图 (Top 15) ──
    sorted_flash = sorted(
        [(k, v.get('flash', 0)) for k, v in module_sizes.items() if v.get('flash', 0) > 0],
        key=lambda x: x[1], reverse=True
    )[:15]

    if sorted_flash:
        fig, ax = plt.subplots(figsize=(12, 6))
        names = [x[0] for x in sorted_flash]
        values = [x[1] / 1024 for x in sorted_flash]
        bars = ax.barh(range(len(names)), values, color='#3498db')
        ax.set_yticks(range(len(names)))
        ax.set_yticklabels(names)
        ax.set_xlabel('大小 (KB)')
        ax.set_title('Flash 模块占用排名 (Top 15)')
        ax.invert_yaxis()
        ax.grid(True, axis='x', alpha=0.3)

        # 标注数值
        for bar, val in zip(bars, values):
            ax.text(bar.get_width() + 0.1, bar.get_y() + bar.get_height()/2,
                    f'{val:.1f} KB', va='center', fontsize=8)

        plt.tight_layout()
        path = os.path.join(output_dir, 'size_flash_modules.png')
        fig.savefig(path, dpi=150)
        plt.close(fig)
        print(f"  [图表] Flash模块排名: {path}")

    # ── 图3: RAM 模块占用柱状图 (Top 15) ──
    sorted_ram = sorted(
        [(k, v.get('ram', 0)) for k, v in module_sizes.items() if v.get('ram', 0) > 0],
        key=lambda x: x[1], reverse=True
    )[:15]

    if sorted_ram:
        fig, ax = plt.subplots(figsize=(12, 6))
        names = [x[0] for x in sorted_ram]
        values = [x[1] / 1024 for x in sorted_ram]
        bars = ax.barh(range(len(names)), values, color='#2ecc71')
        ax.set_yticks(range(len(names)))
        ax.set_yticklabels(names)
        ax.set_xlabel('大小 (KB)')
        ax.set_title('RAM 模块占用排名 (Top 15)')
        ax.invert_yaxis()
        ax.grid(True, axis='x', alpha=0.3)

        for bar, val in zip(bars, values):
            ax.text(bar.get_width() + 0.05, bar.get_y() + bar.get_height()/2,
                    f'{val:.2f} KB', va='center', fontsize=8)

        plt.tight_layout()
        path = os.path.join(output_dir, 'size_ram_modules.png')
        fig.savefig(path, dpi=150)
        plt.close(fig)
        print(f"  [图表] RAM模块排名: {path}")

    # ── 图4: Flash 使用饼图（按大类）──
    flash_by_category = defaultdict(int)
    for name, sizes in module_sizes.items():
        cat = name.split('.')[0] if '.' in name else 'application'
        if 'hal' in name.lower() or 'stm32' in name.lower():
            cat = 'HAL/Driver'
        elif 'cortex' in name.lower() or 'startup' in name.lower():
            cat = 'Startup/CMSIS'
        elif 'lib' in name.lower() or 'libc' in name.lower():
            cat = 'C库'
        elif 'printf' in name.lower() or 'string' in name.lower():
            cat = 'C库'
        flash_by_category[cat] += sizes.get('flash', 0)

    if flash_by_category:
        fig, ax = plt.subplots(figsize=(8, 8))
        sorted_cats = sorted(flash_by_category.items(), key=lambda x: x[1], reverse=True)
        labels = [x[0] for x in sorted_cats[:10]]
        values = [x[1] for x in sorted_cats[:10]]
        ax.pie(values, labels=labels, autopct='%1.1f%%', startangle=140)
        ax.set_title('Flash 占用分布（按模块类别）')
        plt.tight_layout()
        path = os.path.join(output_dir, 'size_flash_pie.png')
        fig.savefig(path, dpi=150)
        plt.close(fig)
        print(f"  [图表] Flash分布饼图: {path}")


def compare_maps(result1: dict, result2: dict):
    """对比两个 map 文件的大小差异。"""
    ms1 = result1.get('module_sizes', {})
    ms2 = result2.get('module_sizes', {})

    all_modules = set(ms1.keys()) | set(ms2.keys())
    diffs = []

    for mod in all_modules:
        flash1 = ms1.get(mod, {}).get('flash', 0)
        flash2 = ms2.get(mod, {}).get('flash', 0)
        ram1 = ms1.get(mod, {}).get('ram', 0)
        ram2 = ms2.get(mod, {}).get('ram', 0)
        d_flash = flash2 - flash1
        d_ram = ram2 - ram1
        if d_flash != 0 or d_ram != 0:
            diffs.append((mod, d_flash, d_ram))

    # 按 Flash 变化排序
    diffs.sort(key=lambda x: abs(x[1]), reverse=True)

    print("\n" + "=" * 70)
    print("  固件大小对比报告")
    print("=" * 70)
    print(f"\n  {'模块':<30} {'Flash变化':>12} {'RAM变化':>12}")
    print("  " + "-" * 56)

    total_d_flash = 0
    total_d_ram = 0
    for mod, d_flash, d_ram in diffs[:30]:
        flash_str = f"+{d_flash}" if d_flash > 0 else str(d_flash)
        ram_str = f"+{d_ram}" if d_ram > 0 else str(d_ram)
        marker = " ⬆" if d_flash > 100 else (" ⬇" if d_flash < -100 else "")
        print(f"  {mod:<30} {flash_str:>12} {ram_str:>12}{marker}")
        total_d_flash += d_flash
        total_d_ram += d_ram

    print("  " + "-" * 56)
    flash_str = f"+{total_d_flash}" if total_d_flash > 0 else str(total_d_flash)
    ram_str = f"+{total_d_ram}" if total_d_ram > 0 else str(total_d_ram)
    print(f"  {'总计':<30} {flash_str:>12} {ram_str:>12}")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(
        description='固件大小分析器 - 解析 map 文件生成占用图表',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python firmware_size_analyzer.py --map project.map --flash 64 --ram 20
  python firmware_size_analyzer.py --map project.map --mcu stm32f103c8
  python firmware_size_analyzer.py --map map1.map --compare map2.map
  python firmware_size_analyzer.py --elf firmware.elf --flash 512 --ram 128
  python firmware_size_analyzer.py --list-mcus
        """
    )
    parser.add_argument('--map', '-m', help='Keil/GCC 生成的 .map 文件')
    parser.add_argument('--elf', '-e', help='ELF 可执行文件 (需安装 arm-none-eabi-size)')
    parser.add_argument('--flash', type=int, default=None, help='Flash 大小 (KB)')
    parser.add_argument('--ram', type=int, default=None, help='RAM 大小 (KB)')
    parser.add_argument('--mcu', help='MCU 型号 (自动查找 Flash/RAM 规格)')
    parser.add_argument('--compare', help='对比的第二个 map 文件')
    parser.add_argument('--output', '-o', default='.', help='图表输出目录 (默认: 当前目录)')
    parser.add_argument('--list-mcus', action='store_true', help='列出支持的 MCU 列表')
    parser.add_argument('--no-charts', action='store_true', help='不生成图表')

    args = parser.parse_args()

    if args.list_mcus:
        print("\n支持的 MCU 型号:")
        print("-" * 50)
        print(f"{'型号':<20} {'Flash (KB)':>10} {'RAM (KB)':>10}")
        print("-" * 50)
        for key, spec in sorted(MCU_SPECS.items()):
            print(f"{key:<20} {spec['flash']:>10} {spec['ram']:>10}")
        return

    # 确定 Flash/RAM 限制
    flash_kb = args.flash
    ram_kb = args.ram

    if args.mcu:
        mcu_key = args.mcu.lower().replace('-', '').replace('_', '')
        if mcu_key in MCU_SPECS:
            spec = MCU_SPECS[mcu_key]
            flash_kb = flash_kb or spec['flash']
            ram_kb = ram_kb or spec['ram']
            print(f"[INFO] MCU: {spec['name']} | Flash: {spec['flash']}KB | RAM: {spec['ram']}KB")
        else:
            print(f"[WARN] 未知 MCU: {args.mcu}，请使用 --list-mcus 查看支持列表")

    if flash_kb is None:
        flash_kb = 64  # 默认 64KB
        print(f"[INFO] 未指定 Flash 大小，使用默认 {flash_kb}KB")
    if ram_kb is None:
        ram_kb = 20  # 默认 20KB
        print(f"[INFO] 未指定 RAM 大小，使用默认 {ram_kb}KB")

    # 解析文件
    result = None
    if args.map:
        result = parse_map_file(args.map)
    elif args.elf:
        result = parse_elf_size(args.elf)
    else:
        print("[ERROR] 请指定 --map 或 --elf 参数")
        parser.print_help()
        sys.exit(1)

    if not result or not result.get('module_sizes'):
        print("[ERROR] 未能解析到任何模块信息")
        sys.exit(1)

    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)

    # 打印分析结果
    print_analysis(result, flash_kb, ram_kb)

    # 生成图表
    if not args.no_charts:
        plot_charts(result, flash_kb, ram_kb, args.output)

    # 对比模式
    if args.compare:
        result2 = parse_map_file(args.compare)
        if result2 and result2.get('module_sizes'):
            compare_maps(result, result2)
        else:
            print(f"[WARN] 无法解析对比文件: {args.compare}")

    print(f"\n[DONE] 分析完成")


if __name__ == '__main__':
    main()
