#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
内存使用分析器 - 电赛资产库工具
===================================
功能：分析每个驱动的RAM/Flash占用，帮助评估嵌入式系统资源使用情况
用法：
  python memory_usage_analyzer.py                       # 分析资产库全部代码
  python memory_usage_analyzer.py --path ./drivers      # 指定分析目录
  python memory_usage_analyzer.py --map firmware.map    # 分析map文件
  python memory_usage_analyzer.py --output report.md    # 输出到文件
  python memory_usage_analyzer.py --mcu stm32f103       # 指定MCU型号

原理：
  - 静态分析：通过代码中的变量声明、数组定义、字符串常量估算RAM/Flash占用
  - Map文件分析：解析编译器生成的.map文件获取精确的内存占用数据
  - 代码行数统计：按模块统计代码量作为参考

依赖：无额外依赖（纯Python实现）
"""

import argparse
import json
import os
import re
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path


# ============================================================
# 常见MCU内存配置
# ============================================================

MCU_MEMORY_MAP = {
    'stm32f103c8': {'flash': 64, 'ram': 20, 'name': 'STM32F103C8T6'},
    'stm32f103cb': {'flash': 128, 'ram': 20, 'name': 'STM32F103CBT6'},
    'stm32f103rc': {'flash': 256, 'ram': 48, 'name': 'STM32F103RCT6'},
    'stm32f103ze': {'flash': 512, 'ram': 64, 'name': 'STM32F103ZET6'},
    'stm32f407vg': {'flash': 1024, 'ram': 192, 'name': 'STM32F407VGT6'},
    'mspm0g3507': {'flash': 256, 'ram': 32, 'name': 'MSPM0G3507'},
    'mspm0l1306': {'flash': 64, 'ram': 4, 'name': 'MSPM0L1306'},
    'tm4c123gh6pm': {'flash': 256, 'ram': 32, 'name': 'TM4C123GH6PM'},
}

# C语言类型大小估算（字节）
C_TYPE_SIZES = {
    'char': 1, 'signed char': 1, 'unsigned char': 1, 'uint8_t': 1, 'int8_t': 1,
    'short': 2, 'unsigned short': 2, 'uint16_t': 2, 'int16_t': 2,
    'int': 4, 'unsigned int': 4, 'uint32_t': 4, 'int32_t': 4, 'long': 4,
    'float': 4, 'double': 8,
    'long long': 8, 'uint64_t': 8, 'int64_t': 8,
    'size_t': 4, 'bool': 1, '_Bool': 1,
}


# ============================================================
# 静态内存分析器
# ============================================================

class StaticMemoryAnalyzer:
    """
    静态内存占用分析器
    
    通过正则分析C代码中的：
    - 全局/静态变量声明 -> RAM (.bss/.data)
    - const常量 -> Flash (.rodata)
    - 字符串字面量 -> Flash (.rodata)
    - 函数定义 -> Flash (.text)
    - 栈使用估算 -> Stack
    """

    def __init__(self):
        """初始化分析正则"""
        
        # 全局/静态变量（非const）
        self.re_global_var = re.compile(
            r'^(?!.*\bconst\b)'                    # 非const
            r'(?:static\s+)?'                       # 可选static
            r'((?:volatile\s+)?[\w]+[\w\s\*]*?)\s+' # 类型
            r'(\w+)'                                 # 变量名
            r'(?:\[(\d+)\])?'                        # 可选数组大小
            r'\s*(?:=\s*[^;]+)?\s*;',               # 可选初始化
            re.MULTILINE
        )
        
        # const常量
        self.re_const_var = re.compile(
            r'(?:static\s+)?const\s+([\w]+[\w\s\*]*?)\s+(\w+)(?:\[(\d+)\])?\s*(?:=\s*[^;]+)?;',
            re.MULTILINE
        )
        
        # 字符串字面量
        self.re_string = re.compile(r'"((?:[^"\\]|\\.)*)"')
        
        # 函数定义（估算代码大小）
        self.re_function = re.compile(
            r'^[\w][\w\s\*]+?\s+(\w+)\s*\([^)]*\)\s*\{',
            re.MULTILINE
        )
        
        # 数组定义（全局）
        self.re_array = re.compile(
            r'(?:static\s+)?(?:const\s+)?([\w]+)\s+(\w+)\[(\d+)\]',
            re.MULTILINE
        )
        
        # volatile变量（ISR共享，特殊标记）
        self.re_volatile = re.compile(
            r'volatile\s+([\w]+[\w\s\*]*?)\s+(\w+)',
            re.MULTILINE
        )

    def analyze_file(self, filepath):
        """
        分析单个文件的内存使用
        
        返回:
            dict: {
                'filename': str,
                'ram_static': int,      # 静态RAM占用(字节)
                'ram_bss': int,         # BSS段(未初始化)
                'ram_data': int,        # Data段(已初始化)
                'flash_code': int,      # 代码区占用估算
                'flash_rodata': int,    # 只读数据区
                'flash_strings': int,   # 字符串字面量
                'variables': list,      # 变量列表
                'arrays': list,         # 数组列表
                'functions': list,      # 函数列表
                'volatile_vars': list,  # volatile变量
                'lines_total': int,     # 总行数
                'lines_code': int,      # 代码行数
                'lines_comment': int,   # 注释行数
            }
        """
        result = {
            'filename': os.path.basename(filepath),
            'filepath': str(filepath),
            'ram_static': 0,
            'ram_bss': 0,
            'ram_data': 0,
            'flash_code': 0,
            'flash_rodata': 0,
            'flash_strings': 0,
            'variables': [],
            'arrays': [],
            'functions': [],
            'volatile_vars': [],
            'lines_total': 0,
            'lines_code': 0,
            'lines_comment': 0,
        }
        
        try:
            content = None
            for encoding in ['utf-8', 'gbk', 'latin-1']:
                try:
                    with open(filepath, 'r', encoding=encoding) as f:
                        content = f.read()
                    break
                except UnicodeDecodeError:
                    continue
            
            if content is None:
                return result
            
            # 统计行数
            lines = content.split('\n')
            result['lines_total'] = len(lines)
            
            in_block_comment = False
            for line in lines:
                stripped = line.strip()
                if in_block_comment:
                    result['lines_comment'] += 1
                    if '*/' in stripped:
                        in_block_comment = False
                    continue
                if '/*' in stripped:
                    result['lines_comment'] += 1
                    if '*/' not in stripped:
                        in_block_comment = True
                    continue
                if stripped.startswith('//'):
                    result['lines_comment'] += 1
                    continue
                if stripped and not stripped.startswith('#'):
                    result['lines_code'] += 1
            
            # 移除注释用于分析
            clean = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
            clean = re.sub(r'//.*?$', '', clean, flags=re.MULTILINE)
            
            # ---- 分析全局/静态变量 (RAM) ----
            for m in self.re_global_var.finditer(clean):
                type_str = m.group(1).strip()
                var_name = m.group(2).strip()
                array_size = m.group(3)
                
                # 跳过函数声明
                if '(' in type_str or var_name in ('if', 'while', 'for', 'switch'):
                    continue
                
                size = self._estimate_type_size(type_str, array_size)
                
                var_info = {
                    'name': var_name,
                    'type': type_str,
                    'size': size,
                    'is_array': array_size is not None,
                    'array_size': int(array_size) if array_size else 0
                }
                result['variables'].append(var_info)
                result['ram_data'] += size
                result['ram_static'] += size
            
            # ---- 分析const常量 (Flash .rodata) ----
            for m in self.re_const_var.finditer(clean):
                type_str = m.group(1).strip()
                var_name = m.group(2).strip()
                array_size = m.group(3)
                
                size = self._estimate_type_size(type_str, array_size)
                result['flash_rodata'] += size
            
            # ---- 分析数组 ----
            for m in self.re_array.finditer(clean):
                type_str = m.group(1).strip()
                name = m.group(2).strip()
                count = int(m.group(3))
                elem_size = self._estimate_type_size(type_str)
                total = elem_size * count
                
                result['arrays'].append({
                    'name': name,
                    'type': type_str,
                    'count': count,
                    'total_size': total
                })
            
            # ---- 分析字符串字面量 (Flash .rodata) ----
            string_bytes = 0
            for m in self.re_string.finditer(clean):
                s = m.group(1)
                # 估算编码大小（中文字符约占3字节UTF-8）
                encoded_size = len(s.encode('utf-8', errors='replace')) + 1  # +1 for \0
                string_bytes += encoded_size
            result['flash_strings'] = string_bytes
            result['flash_rodata'] += string_bytes
            
            # ---- 分析函数定义 (Flash .text) ----
            for m in self.re_function.finditer(clean):
                func_name = m.group(1)
                # 估算函数体大小（粗略：每个语句约10-20字节机器码）
                # 简单用函数名后的代码块行数来估算
                result['functions'].append({'name': func_name})
            
            # Flash代码区估算：每行代码约16字节机器码（ARM Thumb2平均）
            result['flash_code'] = result['lines_code'] * 16
            
            # ---- volatile变量 (ISR共享) ----
            for m in self.re_volatile.finditer(clean):
                result['volatile_vars'].append({
                    'type': m.group(1).strip(),
                    'name': m.group(2).strip()
                })
            
            return result
            
        except Exception as e:
            print(f"  [警告] 分析 {filepath} 失败: {e}", file=sys.stderr)
            return result

    def _estimate_type_size(self, type_str, array_size=None):
        """
        估算C类型占用大小
        
        参数:
            type_str: 类型字符串
            array_size: 数组大小（如果有的话）
        """
        # 清理类型字符串
        type_clean = type_str.strip().rstrip('*')
        
        # 基本类型查找
        size = C_TYPE_SIZES.get(type_clean, 4)  # 默认4字节
        
        # 指针类型
        if '*' in type_str:
            size = 4  # 32位MCU指针
        
        # 数组
        if array_size:
            size *= int(array_size)
        
        return size


# ============================================================
# Map文件解析器
# ============================================================

class MapFileParser:
    """
    编译器Map文件解析器
    
    支持解析：
    - GCC (arm-none-eabi-gcc) 生成的 .map 文件
    - IAR 生成的 .map 文件
    - Keil MDK 生成的 .map 文件
    """

    def parse_gcc_map(self, filepath):
        """解析GCC map文件"""
        sections = defaultdict(lambda: {'size': 0, 'symbols': []})
        
        try:
            with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
                content = f.read()
            
            # GCC map文件中的内存配置
            mem_config = re.findall(
                r'(\w+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)',
                content
            )
            
            # 解析符号表
            # 格式: .text  0x08000000  0x1234  path/to/file.o
            symbol_pattern = re.compile(
                r'\s+(\.\w+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(.+)',
                re.MULTILINE
            )
            
            for m in symbol_pattern.finditer(content):
                section = m.group(1)
                addr = int(m.group(2), 16)
                size = int(m.group(3), 16)
                source = m.group(4).strip()
                
                sections[section]['size'] += size
                sections[section]['symbols'].append({
                    'addr': hex(addr),
                    'size': size,
                    'source': source
                })
            
            return dict(sections)
            
        except Exception as e:
            print(f"解析map文件失败: {e}", file=sys.stderr)
            return {}

    def parse_keil_map(self, filepath):
        """解析Keil MDK map文件"""
        sections = defaultdict(lambda: {'size': 0, 'symbols': []})
        
        try:
            with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
                content = f.read()
            
            # Keil格式: Section Name  Size  Type
            section_pattern = re.compile(
                r'^\s+(\w+)\s+(0x[0-9a-fA-F]+)\s+(\w+)',
                re.MULTILINE
            )
            
            for m in section_pattern.finditer(content):
                name = m.group(1)
                size = int(m.group(2), 16)
                sections[name]['size'] += size
            
            return dict(sections)
            
        except Exception as e:
            print(f"解析Keil map文件失败: {e}", file=sys.stderr)
            return {}


# ============================================================
# 报告生成器
# ============================================================

def format_size(size_bytes):
    """格式化大小显示"""
    if size_bytes < 1024:
        return f"{size_bytes} B"
    elif size_bytes < 1024 * 1024:
        return f"{size_bytes / 1024:.1f} KB"
    else:
        return f"{size_bytes / (1024 * 1024):.2f} MB"


def generate_report(analyses, mcu=None):
    """
    生成内存分析报告
    
    参数:
        analyses: 各文件分析结果列表
        mcu: MCU型号（可选，用于显示总内存和使用率）
    """
    lines = []
    
    lines.append('# 内存使用分析报告')
    lines.append('')
    lines.append(f'> 生成时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
    lines.append(f'> 分析文件数: {len(analyses)}')
    lines.append('')
    
    # MCU信息
    if mcu and mcu.lower() in MCU_MEMORY_MAP:
        info = MCU_MEMORY_MAP[mcu.lower()]
        lines.append(f'## 目标MCU: {info["name"]}')
        lines.append('')
        lines.append(f'- Flash总量: {info["flash"]} KB')
        lines.append(f'- RAM总量: {info["ram"]} KB')
        lines.append('')
    
    # 汇总统计
    total_ram = sum(a['ram_static'] for a in analyses)
    total_flash_code = sum(a['flash_code'] for a in analyses)
    total_flash_rodata = sum(a['flash_rodata'] for a in analyses)
    total_flash = total_flash_code + total_flash_rodata
    total_lines = sum(a['lines_code'] for a in analyses)
    
    lines.append('## 总体占用')
    lines.append('')
    lines.append('| 资源 | 估算占用 | 说明 |')
    lines.append('|------|---------|------|')
    lines.append(f'| RAM (静态) | {format_size(total_ram)} | 全局/静态变量 |')
    lines.append(f'| Flash (.text) | {format_size(total_flash_code)} | 代码区 |')
    lines.append(f'| Flash (.rodata) | {format_size(total_flash_rodata)} | 常量/字符串 |')
    lines.append(f'| Flash 总计 | {format_size(total_flash)} | .text + .rodata |')
    lines.append(f'| 代码行数 | {total_lines} | 有效代码行 |')
    lines.append('')
    
    # 如果有MCU信息，计算使用率
    if mcu and mcu.lower() in MCU_MEMORY_MAP:
        info = MCU_MEMORY_MAP[mcu.lower()]
        flash_pct = (total_flash / (info['flash'] * 1024)) * 100
        ram_pct = (total_ram / (info['ram'] * 1024)) * 100
        
        flash_bar = '█' * int(flash_pct / 5) + '░' * (20 - int(flash_pct / 5))
        ram_bar = '█' * int(ram_pct / 5) + '░' * (20 - int(ram_pct / 5))
        
        lines.append('### 资源使用率')
        lines.append('')
        lines.append(f'```')
        lines.append(f'Flash: [{flash_bar}] {flash_pct:.1f}%')
        lines.append(f'RAM:   [{ram_bar}] {ram_pct:.1f}%')
        lines.append(f'```')
        lines.append('')
        
        if flash_pct > 80:
            lines.append('> ⚠️ **警告**: Flash使用率超过80%，可能需要优化代码大小')
        if ram_pct > 80:
            lines.append('> ⚠️ **警告**: RAM使用率超过80%，可能需要优化内存使用')
        lines.append('')
    
    # 各模块详情
    lines.append('## 各模块内存占用')
    lines.append('')
    lines.append('| 文件 | RAM (B) | Flash 代码 (B) | Flash 数据 (B) | 代码行 |')
    lines.append('|------|---------|---------------|---------------|--------|')
    
    # 按RAM占用排序
    sorted_analyses = sorted(analyses, key=lambda a: a['ram_static'], reverse=True)
    
    for a in sorted_analyses:
        lines.append(
            f"| {a['filename']} | {a['ram_static']} | {a['flash_code']} | "
            f"{a['flash_rodata']} | {a['lines_code']} |"
        )
    
    lines.append(f"| **总计** | **{total_ram}** | **{total_flash_code}** | "
                 f"**{total_flash_rodata}** | **{total_lines}** |")
    lines.append('')
    
    # 大型数组警告
    large_arrays = []
    for a in analyses:
        for arr in a.get('arrays', []):
            if arr['total_size'] > 512:
                large_arrays.append((a['filename'], arr))
    
    if large_arrays:
        lines.append('## ⚠️ 大型数组警告 (>512B)')
        lines.append('')
        lines.append('| 文件 | 数组名 | 类型 | 元素数 | 总大小 |')
        lines.append('|------|--------|------|--------|--------|')
        for fname, arr in large_arrays:
            lines.append(
                f"| {fname} | {arr['name']} | {arr['type']} | "
                f"{arr['count']} | {format_size(arr['total_size'])} |"
            )
        lines.append('')
    
    # volatile变量列表
    volatile_vars = []
    for a in analyses:
        for v in a.get('volatile_vars', []):
            volatile_vars.append((a['filename'], v))
    
    if volatile_vars:
        lines.append('## volatile变量（ISR共享）')
        lines.append('')
        lines.append('| 文件 | 类型 | 变量名 |')
        lines.append('|------|------|--------|')
        for fname, v in volatile_vars:
            lines.append(f"| {fname} | `{v['type']}` | `{v['name']}` |")
        lines.append('')
    
    return '\n'.join(lines)


# ============================================================
# 主程序入口
# ============================================================

def main():
    """命令行入口函数"""
    parser = argparse.ArgumentParser(
        description='内存使用分析器 - 分析每个驱动的RAM/Flash占用',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                                    # 分析资产库全部代码
  %(prog)s --path ./drivers --mcu stm32f103c8 # 指定目录和MCU
  %(prog)s --map firmware.map                  # 分析map文件
  %(prog)s --output mem_report.md             # 保存报告
  %(prog)s --json                             # 输出JSON格式
        """
    )
    
    parser.add_argument('--path', '-p', type=str, default=None,
                        help='分析目录路径（默认：资产库根目录）')
    parser.add_argument('--map', '-m', type=str, default=None,
                        help='编译器map文件路径（精确分析）')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='输出报告文件路径')
    parser.add_argument('--mcu', type=str, default=None,
                        help=f'目标MCU型号（可选: {", ".join(MCU_MEMORY_MAP.keys())}）')
    parser.add_argument('--json', '-j', action='store_true',
                        help='输出JSON格式')
    parser.add_argument('--sort', '-s', type=str, 
                        choices=['ram', 'flash', 'lines', 'name'],
                        default='ram', help='排序方式（默认：ram）')
    
    args = parser.parse_args()
    
    # Map文件分析模式
    if args.map:
        print(f"分析map文件: {args.map}")
        map_parser = MapFileParser()
        
        if 'iar' in args.map.lower():
            sections = map_parser.parse_gcc_map(args.map)  # 通用解析
        else:
            sections = map_parser.parse_gcc_map(args.map)
        
        if args.json:
            print(json.dumps(sections, indent=2, default=str))
        else:
            print("\n=== Map文件分析结果 ===\n")
            for section, data in sorted(sections.items()):
                print(f"  {section}: {format_size(data['size'])} ({len(data.get('symbols', []))} 个符号)")
        return
    
    # 静态分析模式
    scan_dir = args.path
    if scan_dir is None:
        scan_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    print(f"正在扫描目录: {scan_dir}")
    
    # 收集所有C/H文件
    target_files = []
    for root, dirs, files in os.walk(scan_dir):
        dirs[:] = [d for d in dirs if not d.startswith('.')]
        for f in files:
            if f.endswith(('.c', '.h')):
                target_files.append(os.path.join(root, f))
    
    target_files.sort()
    print(f"找到 {len(target_files)} 个C/H文件")
    
    # 分析每个文件
    analyzer = StaticMemoryAnalyzer()
    analyses = []
    
    for filepath in target_files:
        result = analyzer.analyze_file(filepath)
        analyses.append(result)
    
    # 排序
    sort_keys = {
        'ram': lambda a: a['ram_static'],
        'flash': lambda a: a['flash_code'] + a['flash_rodata'],
        'lines': lambda a: a['lines_code'],
        'name': lambda a: a['filename'],
    }
    analyses.sort(key=sort_keys[args.sort], reverse=(args.sort != 'name'))
    
    # JSON输出
    if args.json:
        output = json.dumps(analyses, ensure_ascii=False, indent=2)
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as f:
                f.write(output)
            print(f"JSON已保存至: {args.output}")
        else:
            print(output)
        return
    
    # 生成报告
    report = generate_report(analyses, mcu=args.mcu)
    
    # 输出
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(report)
        print(f"报告已保存至: {args.output}")
    else:
        print('\n' + report)


if __name__ == '__main__':
    main()
