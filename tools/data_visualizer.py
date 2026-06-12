#!/usr/bin/env python3
"""
数据可视化工具 - 串口数据实时绘图 + CSV导入 + 多通道显示
用于电赛数据采集和调试
"""
import argparse
import csv
import json
import os
import sys
import time


# ── CSV数据导入与解析 ─────────────────────────────────────────

def load_csv(filepath, delimiter=',', has_header=True):
    """
    加载CSV文件，返回列数据字典
    """
    data = {}
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.reader(f, delimiter=delimiter)
        headers = None
        if has_header:
            headers = next(reader)
            headers = [h.strip() for h in headers]
        rows = list(reader)

    if not rows:
        return {}

    if headers is None:
        headers = [f'col_{i}' for i in range(len(rows[0]))]

    for h in headers:
        data[h] = []

    for row in rows:
        for i, h in enumerate(headers):
            if i < len(row):
                try:
                    data[h].append(float(row[i]))
                except ValueError:
                    data[h].append(row[i].strip())
            else:
                data[h].append(None)

    return data


# ── 统计分析 ──────────────────────────────────────────────────

def compute_stats(values):
    """计算基本统计量"""
    nums = [v for v in values if isinstance(v, (int, float)) and v is not None]
    if not nums:
        return {'count': 0}

    n = len(nums)
    mean = sum(nums) / n
    variance = sum((x - mean) ** 2 for x in nums) / n
    std = variance ** 0.5
    sorted_nums = sorted(nums)
    median = sorted_nums[n // 2] if n % 2 else (sorted_nums[n//2 - 1] + sorted_nums[n//2]) / 2

    return {
        'count': n,
        'min': min(nums),
        'max': max(nums),
        'mean': round(mean, 4),
        'median': round(median, 4),
        'std': round(std, 4),
        'range': round(max(nums) - min(nums), 4),
        'rms': round((sum(x**2 for x in nums) / n) ** 0.5, 4),
    }


# ── ASCII波形绘图 ────────────────────────────────────────────

def ascii_plot(values, title='波形', width=60, height=20):
    """
    终端ASCII波形图
    """
    if not values:
        return "无数据"

    nums = [v for v in values if isinstance(v, (int, float)) and v is not None]
    if not nums:
        return "无数值数据"

    v_min = min(nums)
    v_max = max(nums)
    v_range = v_max - v_min if v_max != v_min else 1.0

    # 限制显示宽度
    if len(nums) > width:
        # 降采样
        step = len(nums) / width
        display = [nums[int(i * step)] for i in range(width)]
    else:
        display = nums[:]

    lines = []
    lines.append(f"┌{'─' * (width + 2)}┐ {title}")
    lines.append(f"│ {v_max:>10.3f} {'│':>{width - 10}}")

    for row in range(height - 2, -1, -1):
        threshold = v_min + v_range * row / (height - 1)
        line = "│ "
        for v in display:
            if v >= threshold + v_range / (height - 1):
                line += "█"
            elif v >= threshold:
                line += "▄"
            else:
                line += " "
        line += "│"
        if row == height // 2:
            mid_val = (v_max + v_min) / 2
            line += f" {mid_val:.3f}"
        lines.append(line)

    lines.append(f"│ {v_min:>10.3f} {'│':>{width - 10}}")
    lines.append(f"└{'─' * (width + 2)}┘")
    lines.append(f"  0{' ' * (width - 5)}{len(nums) - 1}")

    return "\n".join(lines)


def multi_channel_plot(channels_data, channel_names=None, width=60, height=12):
    """
    多通道ASCII波形图（上下排列）
    """
    lines = []
    for i, data in enumerate(channels_data):
        name = channel_names[i] if channel_names and i < len(channel_names) else f'CH{i+1}'
        lines.append(ascii_plot(data, title=name, width=width, height=height))
        lines.append("")
    return "\n".join(lines)


# ── 数据滤波 ──────────────────────────────────────────────────

def moving_average(values, window=5):
    """滑动平均滤波"""
    if len(values) < window:
        return values[:]
    result = []
    for i in range(len(values)):
        start = max(0, i - window // 2)
        end = min(len(values), i + window // 2 + 1)
        result.append(sum(values[start:end]) / (end - start))
    return result


def median_filter(values, window=5):
    """中值滤波"""
    if len(values) < window:
        return values[:]
    result = []
    for i in range(len(values)):
        start = max(0, i - window // 2)
        end = min(len(values), i + window // 2 + 1)
        result.append(sorted(values[start:end])[len(values[start:end]) // 2])
    return result


# ── 串口数据模拟读取 ──────────────────────────────────────────

def parse_serial_line(line, fmt='csv'):
    """
    解析一行串口数据
    fmt: csv(逗号分隔) / json / space(空格分隔) / tab
    """
    line = line.strip()
    if not line:
        return None

    if fmt == 'json':
        try:
            return json.loads(line)
        except json.JSONDecodeError:
            return None
    elif fmt == 'csv':
        parts = line.split(',')
    elif fmt == 'tab':
        parts = line.split('\t')
    else:
        parts = line.split()

    result = []
    for p in parts:
        try:
            result.append(float(p.strip()))
        except ValueError:
            result.append(p.strip())
    return result


def read_serial_data(port, baudrate=115200, duration=10, fmt='csv'):
    """
    读取串口数据（需要pyserial）
    如果pyserial不可用，返回空列表
    """
    try:
        import serial
    except ImportError:
        print("错误: 需要安装 pyserial: pip install pyserial")
        return []

    data_lines = []
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"已打开串口 {port} @ {baudrate}bps, 采集 {duration}秒...")
        start = time.time()
        while time.time() - start < duration:
            line = ser.readline().decode('utf-8', errors='ignore')
            if line:
                parsed = parse_serial_line(line, fmt)
                if parsed is not None:
                    data_lines.append(parsed)
                    print(f"  [{len(data_lines)}] {parsed}")
        ser.close()
    except Exception as e:
        print(f"串口错误: {e}")

    return data_lines


# ── FFT频谱分析 ───────────────────────────────────────────────

def simple_fft(values, sample_rate):
    """
    简易DFT频谱分析（不依赖numpy）
    仅用于小数据量分析
    """
    n = len(values)
    if n < 4:
        return [], []

    # 只计算前N/2个频率分量
    freq_bins = min(n // 2, 512)
    magnitudes = []
    frequencies = []

    for k in range(freq_bins):
        real = 0
        imag = 0
        for i in range(n):
            angle = 2 * math.pi * k * i / n
            real += values[i] * math.cos(angle)
            imag -= values[i] * math.sin(angle)
        mag = (real ** 2 + imag ** 2) ** 0.5 / n
        freq = k * sample_rate / n
        magnitudes.append(mag)
        frequencies.append(freq)

    return frequencies, magnitudes


import math


def find_dominant_freq(values, sample_rate):
    """找主频率"""
    freqs, mags = simple_fft(values, sample_rate)
    if not freqs:
        return 0
    # 跳过DC分量
    peak_idx = max(range(1, len(mags)), key=lambda i: mags[i])
    return freqs[peak_idx]


# ── 数据导出 ──────────────────────────────────────────────────

def export_csv(data_dict, filepath):
    """导出数据到CSV"""
    keys = list(data_dict.keys())
    n = max(len(v) for v in data_dict.values())
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(keys)
        for i in range(n):
            row = [data_dict[k][i] if i < len(data_dict[k]) else '' for k in keys]
            writer.writerow(row)
    print(f"已导出: {filepath} ({n}行)")


# ── CLI ──────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='数据可视化工具 - 串口/CSV数据分析')
    sub = parser.add_subparsers(dest='cmd')

    # CSV分析
    p_csv = sub.add_parser('csv', help='CSV文件分析与可视化')
    p_csv.add_argument('file', help='CSV文件路径')
    p_csv.add_argument('--column', '-c', help='指定列名(逗号分隔)')
    p_csv.add_argument('--delimiter', '-d', default=',', help='分隔符')
    p_csv.add_argument('--stats', action='store_true', help='显示统计信息')
    p_csv.add_argument('--plot', action='store_true', help='ASCII波形图')
    p_csv.add_argument('--filter', choices=['moving_avg', 'median'], help='滤波方法')
    p_csv.add_argument('--window', type=int, default=5, help='滤波窗口')
    p_csv.add_argument('--fft', action='store_true', help='频谱分析')
    p_csv.add_argument('--sample-rate', type=float, default=1000, help='采样率Hz(用于FFT)')
    p_csv.add_argument('--out', help='导出文件')

    # 串口监控
    p_ser = sub.add_parser('serial', help='串口数据采集与绘图')
    p_ser.add_argument('--port', required=True, help='串口号(如COM3)')
    p_ser.add_argument('--baud', type=int, default=115200, help='波特率')
    p_ser.add_argument('--duration', type=int, default=10, help='采集时间秒')
    p_ser.add_argument('--format', default='csv', choices=['csv', 'json', 'space', 'tab'])
    p_ser.add_argument('--plot', action='store_true', help='实时绘图')
    p_ser.add_argument('--out', help='保存到文件')

    # JSON数据
    p_json = sub.add_parser('json', help='JSON数据分析')
    p_json.add_argument('file', help='JSON文件')
    p_json.add_argument('--keys', '-k', help='数据键(逗号分隔)')
    p_json.add_argument('--plot', action='store_true')
    p_json.add_argument('--stats', action='store_true')

    args = parser.parse_args()
    if not args.cmd:
        parser.print_help()
        return

    if args.cmd == 'csv':
        data = load_csv(args.file, args.delimiter)
        if not data:
            print("无数据")
            return

        # 选择列
        columns = args.column.split(',') if args.column else list(data.keys())
        columns = [c.strip() for c in columns]

        print(f"文件: {args.file}")
        print(f"列: {', '.join(data.keys())}")
        print(f"行数: {max(len(v) for v in data.values())}")

        for col in columns:
            if col not in data:
                print(f"列 '{col}' 不存在")
                continue

            values = data[col]
            nums = [v for v in values if isinstance(v, (int, float))]

            if args.filter and nums:
                if args.filter == 'moving_avg':
                    nums = moving_average(nums, args.window)
                elif args.filter == 'median':
                    nums = median_filter(nums, args.window)

            if args.stats:
                s = compute_stats(nums)
                print(f"\n=== {col} 统计 ===")
                for k, v in s.items():
                    print(f"  {k}: {v}")

            if args.plot:
                print(f"\n{ascii_plot(nums, title=col)}")

            if args.fft and nums:
                peak_freq = find_dominant_freq(nums, args.sample_rate)
                print(f"\n  主频率: {peak_freq:.2f} Hz")

        if args.out:
            filtered_data = {}
            for col in columns:
                if col in data:
                    vals = data[col]
                    if args.filter:
                        nums = [v for v in vals if isinstance(v, (int, float))]
                        if args.filter == 'moving_avg':
                            filtered_data[col] = moving_average(nums, args.window)
                        elif args.filter == 'median':
                            filtered_data[col] = median_filter(nums, args.window)
                    else:
                        filtered_data[col] = vals
            export_csv(filtered_data, args.out)

    elif args.cmd == 'serial':
        raw = read_serial_data(args.port, args.baud, args.duration, args.format)
        if raw and args.plot:
            # 尝试绘图（取数值列）
            for i in range(len(raw[0]) if raw else 0):
                col_vals = [row[i] for row in raw if isinstance(row, list) and i < len(row) and isinstance(row[i], (int, float))]
                if col_vals:
                    print(f"\n通道 {i+1}:")
                    print(ascii_plot(col_vals, title=f'CH{i+1}'))
        if args.out and raw:
            with open(args.out, 'w') as f:
                json.dump(raw, f, indent=2)
            print(f"已保存: {args.out}")

    elif args.cmd == 'json':
        with open(args.file, 'r', encoding='utf-8') as f:
            data = json.load(f)

        keys = args.keys.split(',') if args.keys else []
        if not keys and isinstance(data, dict):
            keys = [k for k in data.keys() if isinstance(data[k], list)]

        for key in keys:
            key = key.strip()
            if key in data and isinstance(data[key], list):
                values = [v for v in data[key] if isinstance(v, (int, float))]
                if args.stats:
                    print(f"\n=== {key} ===")
                    print(json.dumps(compute_stats(values), indent=2, ensure_ascii=False))
                if args.plot:
                    print(ascii_plot(values, title=key))


if __name__ == '__main__':
    main()
