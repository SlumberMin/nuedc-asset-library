#!/usr/bin/env python3
"""
时序分析工具 - 中断延迟/任务响应/最坏执行时间分析
用于电赛嵌入式系统时序约束评估
"""
import argparse
import json
import sys


# ── 中断延迟分析 ──────────────────────────────────────────────
def analyze_interrupt_latency(tasks):
    """
    分析中断响应延迟
    tasks: [{'name': str, 'priority': int, 'wcet_us': float, 'period_us': float}, ...]
    返回每个任务的最坏中断延迟
    """
    # 按优先级排序（数字越小优先级越高）
    sorted_tasks = sorted(tasks, key=lambda t: t['priority'])
    results = []
    for i, task in enumerate(sorted_tasks):
        # 最坏延迟 = 所有更高优先级任务的WCET之和 + 硬件延迟
        hw_latency = task.get('hw_latency_us', 1.0)  # 硬件中断响应默认1us
        preemption_time = sum(t['wcet_us'] for t in sorted_tasks[:i])
        total_latency = hw_latency + preemption_time
        results.append({
            'name': task['name'],
            'priority': task['priority'],
            'hw_latency_us': hw_latency,
            'preemption_us': preemption_time,
            'worst_isr_latency_us': round(total_latency, 2),
        })
    return results


# ── 任务调度可调度性分析 (RMS) ────────────────────────────────
def rms_schedulability(tasks):
    """
    速率单调调度(RMS)可调度性分析
    使用充分条件: U = Σ(Ci/Ti) <= n(2^(1/n) - 1)
    """
    n = len(tasks)
    if n == 0:
        return {'schedulable': True, 'utilization': 0, 'bound': 0}

    utilization = sum(t['wcet_us'] / t['period_us'] for t in tasks)
    # Liu & Layland bound
    bound = n * (2 ** (1.0 / n) - 1)
    schedulable = utilization <= bound

    # 逐任务响应时间分析（更精确）
    response_times = []
    for task in sorted(tasks, key=lambda t: t['period_us']):
        r = task['wcet_us']  # 初始响应时间 = WCET
        for _ in range(20):  # 迭代求解
            interference = 0
            for other in tasks:
                if other['period_us'] < task['period_us']:
                    interference += (r / other['period_us']) * other['wcet_us']
                elif other['period_us'] == task['period_us'] and other['name'] != task['name']:
                    interference += (r / other['period_us']) * other['wcet_us']
            r_new = task['wcet_us'] + interference
            if abs(r_new - r) < 0.01:
                break
            r = r_new
        deadline_met = r <= task.get('deadline_us', task['period_us'])
        response_times.append({
            'name': task['name'],
            'period_us': task['period_us'],
            'wcet_us': task['wcet_us'],
            'response_time_us': round(r, 2),
            'deadline_met': deadline_met,
        })

    return {
        'schedulable': schedulable,
        'utilization': round(utilization, 4),
        'bound': round(bound, 4),
        'response_times': response_times,
    }


# ── 最坏执行时间(WCET)估算 ──────────────────────────────────
def estimate_wcet(source_lines, arch='cortex-m4', clock_mhz=168):
    """
    根据代码行数和架构估算WCET
    source_lines: 源代码行数
    arch: 目标架构
    clock_mhz: 主频(MHz)
    """
    # 每条指令平均周期数（经验值）
    arch_ipc = {
        'cortex-m0': 0.85,
        'cortex-m3': 0.95,
        'cortex-m4': 1.0,
        'cortex-m7': 1.2,
        'stm32f1': 0.9,
        'stm32f4': 1.0,
        'stm32h7': 1.2,
        'msp430': 0.7,
        'dspic33': 0.8,
    }
    ipc = arch_ipc.get(arch, 1.0)
    # 每行代码平均约3-5条指令
    instructions_per_line = 4
    total_instructions = source_lines * instructions_per_line
    cycles = total_instructions / ipc
    wcet_us = cycles / clock_mhz
    return {
        'arch': arch,
        'clock_mhz': clock_mhz,
        'source_lines': source_lines,
        'estimated_instructions': total_instructions,
        'estimated_cycles': int(cycles),
        'wcet_us': round(wcet_us, 2),
        'wcet_ms': round(wcet_us / 1000, 4),
    }


# ── 定时器配置计算 ──────────────────────────────────────────
def timer_calculation(target_freq_hz, clock_mhz=168, prescaler=1):
    """
    计算定时器重装值
    target_freq_hz: 目标频率
    clock_mhz: 时钟频率(MHz)
    prescaler: 预分频系数
    """
    timer_clock = clock_mhz * 1e6 / prescaler
    period = timer_clock / target_freq_hz
    reload = int(period) - 1
    actual_freq = timer_clock / (reload + 1)
    error_pct = abs(actual_freq - target_freq_hz) / target_freq_hz * 100
    return {
        'target_freq_hz': target_freq_hz,
        'timer_clock_hz': timer_clock,
        'prescaler': prescaler,
        'reload_value': reload,
        'actual_freq_hz': round(actual_freq, 4),
        'error_percent': round(error_pct, 6),
    }


# ── CLI ──────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='时序分析工具 - 嵌入式系统时序评估')
    sub = parser.add_subparsers(dest='cmd', help='子命令')

    # 中断延迟分析
    p_isr = sub.add_parser('isr', help='中断延迟分析')
    p_isr.add_argument('--json', required=True, help='任务列表JSON')
    p_isr.add_argument('--out', help='输出JSON文件')

    # RMS可调度性
    p_rms = sub.add_parser('rms', help='RMS调度可调度性分析')
    p_rms.add_argument('--json', required=True, help='任务列表JSON')
    p_rms.add_argument('--out', help='输出JSON文件')

    # WCET估算
    p_wcet = sub.add_parser('wcet', help='最坏执行时间估算')
    p_wcet.add_argument('--lines', type=int, required=True, help='源代码行数')
    p_wcet.add_argument('--arch', default='cortex-m4', help='目标架构')
    p_wcet.add_argument('--clock', type=float, default=168, help='主频MHz')

    # 定时器计算
    p_tim = sub.add_parser('timer', help='定时器配置计算')
    p_tim.add_argument('--freq', type=float, required=True, help='目标频率Hz')
    p_tim.add_argument('--clock', type=float, default=168, help='时钟MHz')
    p_tim.add_argument('--psc', type=int, default=1, help='预分频')

    args = parser.parse_args()
    if not args.cmd:
        parser.print_help()
        return

    if args.cmd == 'isr':
        tasks = json.loads(args.json)
        result = analyze_interrupt_latency(tasks)
        print(json.dumps(result, indent=2, ensure_ascii=False))
        if args.out:
            with open(args.out, 'w', encoding='utf-8') as f:
                json.dump(result, f, indent=2, ensure_ascii=False)

    elif args.cmd == 'rms':
        tasks = json.loads(args.json)
        result = rms_schedulability(tasks)
        print(json.dumps(result, indent=2, ensure_ascii=False))
        if args.out:
            with open(args.out, 'w', encoding='utf-8') as f:
                json.dump(result, f, indent=2, ensure_ascii=False)

    elif args.cmd == 'wcet':
        result = estimate_wcet(args.lines, args.arch, args.clock)
        print(json.dumps(result, indent=2, ensure_ascii=False))

    elif args.cmd == 'timer':
        result = timer_calculation(args.freq, args.clock, args.psc)
        print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
