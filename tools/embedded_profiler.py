#!/usr/bin/env python3
"""
嵌入式性能分析工具 - nuedc-asset-library
功能：代码执行时间估算、内存使用分析、中断延迟计算、代码复杂度分析
作者：电赛自动迭代引擎 V3
"""

import argparse
import json
import math
import os
import re
import sys


# ─── MCU周期/时钟计算 ─────────────────────────────────────────────

# 常见MCU架构的周期数估算
MCU_ARCH = {
    'stm32f1': {'core': 'Cortex-M3', 'freq_mhz': 72, 'flash_wait': 2,
                'cycles_per_instr': 1.0, 'dsp_mul': 1, 'div': 2-12},
    'stm32f4': {'core': 'Cortex-M4F', 'freq_mhz': 168, 'flash_wait': 5,
                'cycles_per_instr': 1.0, 'dsp_mul': 1, 'div': 2-12},
    'stm32h7': {'core': 'Cortex-M7', 'freq_mhz': 480, 'flash_wait': 4,
                'cycles_per_instr': 1.0, 'dsp_mul': 1, 'div': 2-12},
    'stm32g4': {'core': 'Cortex-M4F', 'freq_mhz': 170, 'flash_wait': 4,
                'cycles_per_instr': 1.0, 'dsp_mul': 1, 'div': 2-12},
    'esp32':   {'core': 'Xtensa LX6', 'freq_mhz': 240, 'flash_wait': 0,
                'cycles_per_instr': 1.0, 'dsp_mul': 3, 'div': 9-39},
    'rp2040':  {'core': 'Cortex-M0+', 'freq_mhz': 133, 'flash_wait': 1,
                'cycles_per_instr': 1.0, 'dsp_mul': 32, 'div': 2-32},
    'ti_c2000': {'core': 'C28x', 'freq_mhz': 200, 'flash_wait': 3,
                 'cycles_per_instr': 1.0, 'dsp_mul': 1, 'div': 2-20},
}


def calc_cycles(operation, arch='stm32f4'):
    """估算单条C操作在指定架构上的周期数"""
    mcu = MCU_ARCH.get(arch, MCU_ARCH['stm32f4'])
    ops = {
        'add': 1, 'sub': 1, 'mul': mcu['dsp_mul'],
        'div': mcu['div'], 'fadd': 1, 'fsub': 1,
        'fmul': 1, 'fdiv': 14, 'fsqrt': 14,
        'load': 2, 'store': 2, 'branch': 1 + (mcu['flash_wait'] if mcu['flash_wait'] > 0 else 0),
        'nop': 1, 'cmp': 1, 'and': 1, 'or': 1, 'xor': 1,
    }
    return ops.get(operation, 1)


# ─── C代码执行时间估算 ────────────────────────────────────────────

# 简单C语句的周期映射
C_STMT_CYCLES = {
    '赋值': 2, '加减': 2, '乘法': 3, '除法': 12,
    '浮点加': 3, '浮点乘': 3, '浮点除': 14,
    '数组访问': 3, '指针解引用': 3, '函数调用': 8,
    '条件分支': 3, '循环开销': 4, '中断进出': 12,
}

# 常见嵌入式算法的周期数估算（基于STM32F4 @ 168MHz）
ALGORITHM_BENCHMARKS = {
    '1024点FFT': {'cycles': 12000, 'ram_bytes': 8192, 'flash_bytes': 4096},
    '256点FFT': {'cycles': 2500, 'ram_bytes': 2048, 'flash_bytes': 4096},
    'PID控制器': {'cycles': 150, 'ram_bytes': 48, 'flash_bytes': 512},
    '卡尔曼滤波': {'cycles': 800, 'ram_bytes': 128, 'flash_bytes': 1024},
    'FIR滤波(64阶)': {'cycles': 400, 'ram_bytes': 512, 'flash_bytes': 512},
    'IIR双二阶': {'cycles': 80, 'ram_bytes': 32, 'flash_bytes': 256},
    'ADC DMA读取': {'cycles': 20, 'ram_bytes': 0, 'flash_bytes': 128},
    'SPI传输(字节)': {'cycles': 50, 'ram_bytes': 0, 'flash_bytes': 64},
    'UART发送(字节)': {'cycles': 100, 'ram_bytes': 0, 'flash_bytes': 128},
    'GPIO翻转': {'cycles': 4, 'ram_bytes': 0, 'flash_bytes': 16},
    'PWM更新': {'cycles': 10, 'ram_bytes': 0, 'flash_bytes': 64},
    '定时器中断': {'cycles': 50, 'ram_bytes': 64, 'flash_bytes': 128},
    '浮点sin()': {'cycles': 100, 'ram_bytes': 0, 'flash_bytes': 256},
    '浮点sqrt()': {'cycles': 30, 'ram_bytes': 0, 'flash_bytes': 128},
    '矩阵乘(4x4)': {'cycles': 500, 'ram_bytes': 192, 'flash_bytes': 512},
}


def estimate_exec_time(code_lines, arch='stm32f4', freq_mhz=None):
    """
    估算C代码片段的执行时间
    基于语句级别的周期数统计
    """
    mcu = MCU_ARCH.get(arch, MCU_ARCH['stm32f4'])
    if freq_mhz is None:
        freq_mhz = mcu['freq_mhz']

    total_cycles = 0
    details = []
    loop_depth = 0
    loop_counts = []

    for i, line in enumerate(code_lines):
        line = line.strip()
        if not line or line.startswith('//') or line.startswith('/*') or line.startswith('*'):
            continue

        cycles = 0
        desc = ''

        # 浮点运算
        if re.search(r'float|double', line) and ('=' in line):
            if '*' in line:
                cycles = 3
                desc = '浮点乘法赋值'
            elif '/' in line:
                cycles = 14
                desc = '浮点除法赋值'
            elif '+' in line or '-' in line:
                cycles = 3
                desc = '浮点加减赋值'
            else:
                cycles = 2
                desc = '浮点赋值'
        # 整数运算
        elif re.search(r'\bint\b|uint\d+|int\d+', line):
            if '*' in line:
                cycles = 3
                desc = '整数乘法'
            elif '/' in line or '%' in line:
                cycles = 12
                desc = '整数除法'
            else:
                cycles = 2
                desc = '整数运算'
        # 数组访问
        elif re.search(r'\w+\[', line):
            cycles = 4
            desc = '数组访问'
        # 函数调用
        elif re.search(r'\w+\s*\(', line):
            cycles = 8
            desc = '函数调用'
        # 循环
        elif re.search(r'\bfor\b|\bwhile\b', line):
            loop_depth += 1
            loop_counts.append(0)
            cycles = 4
            desc = '循环开始'
        elif line == '}':
            if loop_depth > 0:
                loop_depth -= 1
                loop_counts.pop()
            cycles = 2
        # 条件
        elif re.search(r'\bif\b|\bswitch\b', line):
            cycles = 3
            desc = '条件分支'
        # 指针
        elif '*' in line and ('=' in line):
            cycles = 3
            desc = '指针操作'
        else:
            cycles = 2
            desc = '基本语句'

        # 应用循环深度倍数
        effective_cycles = cycles * max(1, loop_depth * 10)  # 粗略估计
        total_cycles += effective_cycles

        if desc:
            details.append({"line": i + 1, "code": line[:80], "cycles": effective_cycles, "desc": desc})

    exec_time_us = total_cycles / freq_mhz
    return {
        "total_cycles": total_cycles,
        "freq_mhz": freq_mhz,
        "exec_time_us": exec_time_us,
        "exec_time_ms": exec_time_us / 1000,
        "details": details[:50]
    }


# ─── 内存使用分析 ────────────────────────────────────────────────

def analyze_memory(code_lines, arch='stm32f4'):
    """
    分析C代码的内存使用
    估算栈使用、堆使用、静态分配
    """
    stack_bytes = 0
    static_bytes = 0
    heap_bytes = 0
    variables = []

    for i, line in enumerate(code_lines):
        line = line.strip()

        # 静态/全局变量
        if re.search(r'^\s*(static|const)\s+', line):
            size = _estimate_var_size(line)
            static_bytes += size
            variables.append({"type": "static", "line": i+1, "size": size, "code": line[:60]})
        # 局部变量（栈）
        elif re.search(r'^\s*(int|uint\d+_t|int\d+_t|float|double|char|short|long)\s+', line):
            size = _estimate_var_size(line)
            stack_bytes += size
            variables.append({"type": "stack", "line": i+1, "size": size, "code": line[:60]})
        # 数组
        elif re.search(r'\w+\s*\[\s*(\d+)\s*\]', line):
            m = re.search(r'\w+\s*\[\s*(\d+)\s*\]', line)
            arr_size = int(m.group(1))
            elem_size = _estimate_var_size(line)
            total = arr_size * elem_size
            if re.search(r'^\s*(static|const)', line):
                static_bytes += total
            else:
                stack_bytes += total
            variables.append({"type": "array", "line": i+1, "size": total, "code": line[:60]})
        # malloc
        elif 'malloc' in line or 'calloc' in line:
            m = re.search(r'malloc\s*\(\s*(\d+)', line)
            if m:
                heap_bytes += int(m.group(1))
            else:
                heap_bytes += 256  # 未知大小
            variables.append({"type": "heap", "line": i+1, "size": 256, "code": line[:60]})

    return {
        "stack_bytes": stack_bytes,
        "static_bytes": static_bytes,
        "heap_bytes": heap_bytes,
        "total_bytes": stack_bytes + static_bytes + heap_bytes,
        "variables": variables[:30]
    }


def _estimate_var_size(line):
    """估算变量大小(字节)"""
    if re.search(r'\bfloat\b', line): return 4
    if re.search(r'\bdouble\b', line): return 8
    if re.search(r'\bint64_t\b|\buint64_t\b|\blong\s+long\b', line): return 8
    if re.search(r'\bint32_t\b|\buint32_t\b|\bint\b|\blong\b', line): return 4
    if re.search(r'\bint16_t\b|\buint16_t\b|\bshort\b', line): return 2
    if re.search(r'\bint8_t\b|\buint8_t\b|\bchar\b', line): return 1
    return 4  # 默认


# ─── 中断延迟分析 ────────────────────────────────────────────────

def calc_interrupt_latency(arch='stm32f4', handler_cycles=100, preempt=True):
    """
    计算中断响应延迟
    arch: MCU架构
    handler_cycles: 中断处理函数周期数
    preempt: 是否可抢占（影响最大延迟）
    """
    mcu = MCU_ARCH.get(arch, MCU_ARCH['stm32f4'])
    freq_mhz = mcu['freq_mhz']

    # 固定延迟：压栈 + 流水线排空 + 取向量
    fixed_latency_cycles = {
        'stm32f1': 12, 'stm32f4': 12, 'stm32h7': 12,
        'stm32g4': 12, 'esp32': 20, 'rp2040': 16, 'ti_c2000': 8,
    }
    fixed = fixed_latency_cycles.get(arch, 12)

    # 最坏情况：正在执行多周期指令（如除法）
    worst_case_extra = mcu['div'] if not preempt else 0

    # 中断延迟
    min_latency_cycles = fixed
    max_latency_cycles = fixed + worst_case_extra
    handler_time_cycles = handler_cycles
    total_cycles = max_latency_cycles + handler_cycles

    return {
        "arch": arch,
        "freq_mhz": freq_mhz,
        "fixed_overhead_cycles": fixed,
        "min_latency_ns": min_latency_cycles / freq_mhz * 1000,
        "max_latency_ns": max_latency_cycles / freq_mhz * 1000,
        "handler_cycles": handler_cycles,
        "handler_time_us": handler_cycles / freq_mhz,
        "total_response_us": total_cycles / freq_mhz,
        "max_interrupt_rate_khz": freq_mhz * 1000 / total_cycles,
    }


# ─── 代码复杂度分析 ───────────────────────────────────────────────

def analyze_complexity(code_lines):
    """
    圈复杂度 (Cyclomatic Complexity) 分析
    """
    complexity = 1  # 基本路径
    branches = []
    max_depth = 0
    current_depth = 0

    for i, line in enumerate(code_lines):
        stripped = line.strip()
        # 分支语句
        if re.search(r'\bif\b|\belse\s+if\b', stripped):
            complexity += 1
            current_depth += 1
            max_depth = max(max_depth, current_depth)
            branches.append({"line": i+1, "type": "if", "code": stripped[:60]})
        elif re.search(r'\belse\b', stripped):
            pass  # else不算
        elif re.search(r'\bcase\b', stripped):
            complexity += 1
            branches.append({"line": i+1, "type": "case", "code": stripped[:60]})
        elif re.search(r'\bfor\b|\bwhile\b', stripped):
            complexity += 1
            current_depth += 1
            max_depth = max(max_depth, current_depth)
            branches.append({"line": i+1, "type": "loop", "code": stripped[:60]})
        elif re.search(r'\b\?\b', stripped):
            complexity += 1
            branches.append({"line": i+1, "type": "ternary", "code": stripped[:60]})

        if stripped == '}' and current_depth > 0:
            current_depth -= 1

    # 风险等级
    if complexity <= 10:
        risk = "低风险"
    elif complexity <= 20:
        risk = "中等风险"
    elif complexity <= 50:
        risk = "高风险"
    else:
        risk = "极高风险"

    return {
        "cyclomatic_complexity": complexity,
        "max_nesting_depth": max_depth,
        "branches": branches[:30],
        "risk_level": risk,
        "total_lines": len(code_lines)
    }


# ─── 采样率与实时性分析 ───────────────────────────────────────────

def realtime_analysis(sample_rate_hz, processing_cycles, arch='stm32f4', margin=0.2):
    """
    实时性可行性分析
    sample_rate_hz: 采样率
    processing_cycles: 每次处理的周期数
    margin: 安全裕度(0~1)
    """
    mcu = MCU_ARCH.get(arch, MCU_ARCH['stm32f4'])
    freq = mcu['freq_mhz'] * 1e6

    available_cycles = freq / sample_rate_hz
    utilization = processing_cycles / available_cycles
    is_feasible = utilization < (1.0 - margin)
    max_sample_rate = freq / processing_cycles * (1 - margin)

    return {
        "sample_rate_hz": sample_rate_hz,
        "available_cycles": available_cycles,
        "processing_cycles": processing_cycles,
        "cpu_utilization": utilization,
        "cpu_utilization_pct": utilization * 100,
        "is_feasible": is_feasible,
        "max_sample_rate_hz": max_sample_rate,
        "safety_margin_pct": (1 - utilization) * 100,
        "recommendation": "可行" if is_feasible else "不可行，需优化或降低采样率"
    }


# ─── CLI ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='嵌入式性能分析工具 - 电赛资产库')
    sub = parser.add_subparsers(dest='command')

    # 执行时间估算
    p_exec = sub.add_parser('exec', help='代码执行时间估算')
    p_exec.add_argument('--input', '-i', help='C代码文件')
    p_exec.add_argument('--code', help='直接输入代码片段')
    p_exec.add_argument('--arch', default='stm32f4',
                       choices=list(MCU_ARCH.keys()), help='目标架构')
    p_exec.add_argument('--freq', type=float, default=None, help='主频(MHz)')

    # 内存分析
    p_mem = sub.add_parser('memory', help='内存使用分析')
    p_mem.add_argument('--input', '-i', required=True, help='C代码文件')
    p_mem.add_argument('--arch', default='stm32f4')

    # 中断延迟
    p_int = sub.add_parser('interrupt', help='中断延迟计算')
    p_int.add_argument('--arch', default='stm32f4')
    p_int.add_argument('--handler-cycles', type=int, default=100, help='ISR周期数')
    p_int.add_argument('--preempt', action='store_true', default=True)

    # 复杂度分析
    p_cc = sub.add_parser('complexity', help='代码复杂度分析')
    p_cc.add_argument('--input', '-i', required=True, help='C代码文件')

    # 实时性分析
    p_rt = sub.add_parser('realtime', help='实时性可行性分析')
    p_rt.add_argument('--sample-rate', type=float, required=True, help='采样率(Hz)')
    p_rt.add_argument('--cycles', type=int, required=True, help='处理周期数')
    p_rt.add_argument('--arch', default='stm32f4')
    p_rt.add_argument('--margin', type=float, default=0.2, help='安全裕度(0~1)')

    # 基准测试
    p_bench = sub.add_parser('benchmarks', help='常见算法基准测试数据')
    p_bench.add_argument('--arch', default='stm32f4')

    # MCU信息
    p_mcu = sub.add_parser('mcu', help='MCU架构信息')
    p_mcu.add_argument('--arch', default=None, help='指定架构(空则列出所有)')

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        return

    if args.command == 'mcu':
        if args.arch:
            mcu = MCU_ARCH.get(args.arch)
            if mcu:
                print(f'\n{args.arch}: {mcu}')
            else:
                print(f'未知架构: {args.arch}')
        else:
            print('\n支持的MCU架构:')
            for name, info in MCU_ARCH.items():
                print(f'  {name:<12} {info["core"]:<16} {info["freq_mhz"]}MHz')

    elif args.command == 'exec':
        if args.input:
            with open(args.input, 'r', encoding='utf-8') as f:
                lines = f.readlines()
        elif args.code:
            lines = args.code.split('\\n')
        else:
            print('错误：需要 --input 或 --code')
            return
        result = estimate_exec_time(lines, args.arch, args.freq)
        print(f'\n执行时间估算 ({args.arch}):')
        print(f'  总周期数: {result["total_cycles"]}')
        print(f'  主频: {result["freq_mhz"]}MHz')
        print(f'  执行时间: {result["exec_time_us"]:.2f}μs ({result["exec_time_ms"]:.4f}ms)')
        if result["details"]:
            print(f'\n  语句分析 (前15条):')
            print(f'  {"行":<6} {"周期":<8} {"描述":<16} {"代码":<50}')
            print('  ' + '-' * 80)
            for d in result["details"][:15]:
                print(f'  {d["line"]:<6} {d["cycles"]:<8} {d["desc"]:<16} {d["code"]:<50}')

    elif args.command == 'memory':
        with open(args.input, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        result = analyze_memory(lines, args.arch)
        print(f'\n内存使用分析 ({args.arch}):')
        print(f'  栈使用: {result["stack_bytes"]} bytes')
        print(f'  静态分配: {result["static_bytes"]} bytes')
        print(f'  堆分配: {result["heap_bytes"]} bytes')
        print(f'  总计: {result["total_bytes"]} bytes')
        if result["variables"]:
            print(f'\n  变量列表:')
            print(f'  {"类型":<10} {"行":<6} {"大小(B)":<10} {"代码":<50}')
            print('  ' + '-' * 76)
            for v in result["variables"][:15]:
                print(f'  {v["type"]:<10} {v["line"]:<6} {v["size"]:<10} {v["code"]:<50}')

    elif args.command == 'interrupt':
        result = calc_interrupt_latency(args.arch, args.handler_cycles, args.preempt)
        print(f'\n中断延迟分析 ({args.arch} @ {result["freq_mhz"]}MHz):')
        print(f'  固定开销: {result["fixed_overhead_cycles"]} cycles')
        print(f'  最小延迟: {result["min_latency_ns"]:.1f}ns')
        print(f'  最大延迟: {result["max_latency_ns"]:.1f}ns')
        print(f'  ISR周期: {result["handler_cycles"]} cycles')
        print(f'  ISR时间: {result["handler_time_us"]:.2f}μs')
        print(f'  总响应: {result["total_response_us"]:.2f}μs')
        print(f'  最大中断率: {result["max_interrupt_rate_khz"]:.1f}kHz')

    elif args.command == 'complexity':
        with open(args.input, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        result = analyze_complexity(lines)
        print(f'\n代码复杂度分析:')
        print(f'  总行数: {result["total_lines"]}')
        print(f'  圈复杂度: {result["cyclomatic_complexity"]}')
        print(f'  最大嵌套深度: {result["max_nesting_depth"]}')
        print(f'  风险等级: {result["risk_level"]}')
        if result["branches"]:
            print(f'\n  分支点 (前10):')
            for b in result["branches"][:10]:
                print(f'    L{b["line"]}: [{b["type"]}] {b["code"]}')

    elif args.command == 'realtime':
        result = realtime_analysis(args.sample_rate, args.cycles, args.arch, args.margin)
        print(f'\n实时性分析 ({args.arch}):')
        print(f'  采样率: {result["sample_rate_hz"]:.0f}Hz')
        print(f'  可用周期: {result["available_cycles"]:.0f}')
        print(f'  处理周期: {result["processing_cycles"]}')
        print(f'  CPU利用率: {result["cpu_utilization_pct"]:.1f}%')
        print(f'  安全裕度: {result["safety_margin_pct"]:.1f}%')
        print(f'  最大采样率: {result["max_sample_rate_hz"]:.0f}Hz')
        print(f'  判定: {result["recommendation"]}')

    elif args.command == 'benchmarks':
        freq = MCU_ARCH.get(args.arch, MCU_ARCH['stm32f4'])['freq_mhz']
        print(f'\n常见算法基准 ({args.arch} @ {freq}MHz):')
        print(f'{"算法":<22} {"周期":<10} {"时间(μs)":<12} {"RAM(B)":<10} {"Flash(B)":<10}')
        print('-' * 64)
        for name, data in ALGORITHM_BENCHMARKS.items():
            t_us = data['cycles'] / freq
            print(f'{name:<22} {data["cycles"]:<10} {t_us:<12.2f} {data["ram_bytes"]:<10} {data["flash_bytes"]:<10}')


if __name__ == '__main__':
    main()
