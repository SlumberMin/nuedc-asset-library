#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================
测试结果分析器 - 解析 CSV/JSON 测试数据 + 生成统计图表
============================================================
功能：
  - 解析 CSV 和 JSON 格式的测试数据
  - 计算统计指标（均值、方差、最大/最小、通过率等）
  - 生成多种统计图表（折线图、柱状图、箱线图、直方图）
  - 自动判定测试项是否达标
  - 输出分析报告（文本 + 图表）

依赖：pip install matplotlib pandas numpy

用法：
  python test_result_analyzer.py --input results.csv --output ./analysis/
  python test_result_analyzer.py --input results.json --spec spec.json --output ./analysis/
  python test_result_analyzer.py --input results.csv --charts line bar box hist
============================================================
"""

import argparse
import csv
import json
import os
import sys
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

try:
    import numpy as np
    HAS_NP = True
except ImportError:
    HAS_NP = False

# ── 中文字体配置 ──────────────────────────────────────────
if HAS_MPL:
    # 尝试设置中文字体
    for font_name in ['SimHei', 'Microsoft YaHei', 'WenQuanYi Micro Hei', 'Noto Sans CJK SC']:
        if any(font_name in f.name for f in fm.fontManager.ttflist):
            plt.rcParams['font.sans-serif'] = [font_name]
            break
    plt.rcParams['axes.unicode_minus'] = False


def load_csv(filepath: str) -> list[dict]:
    """加载 CSV 测试数据。"""
    data = []
    with open(filepath, 'r', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append(dict(row))
    print(f"[INFO] CSV 加载: {len(data)} 条记录, 字段: {list(data[0].keys()) if data else '无'}")
    return data


def load_json(filepath: str) -> list[dict]:
    """加载 JSON 测试数据。"""
    with open(filepath, 'r', encoding='utf-8') as f:
        raw = json.load(f)
    if isinstance(raw, list):
        return raw
    elif isinstance(raw, dict):
        for key in ['tests', 'data', 'results', 'records']:
            if key in raw and isinstance(raw[key], list):
                return raw[key]
        return [raw]
    return [raw]


def load_data(filepath: str) -> list[dict]:
    """根据扩展名自动加载。"""
    ext = Path(filepath).suffix.lower()
    if ext == '.csv':
        return load_csv(filepath)
    elif ext == '.json':
        return load_json(filepath)
    else:
        print(f"[WARN] 未知格式 {ext}，尝试 CSV")
        return load_csv(filepath)


def load_spec(filepath: str) -> dict:
    """
    加载指标规格文件（JSON）。
    格式示例:
    {
      "电压": {"min": 4.9, "max": 5.1, "unit": "V"},
      "频率": {"min": 999, "max": 1001, "unit": "Hz"}
    }
    """
    with open(filepath, 'r', encoding='utf-8') as f:
        return json.load(f)


def extract_numeric_columns(data: list[dict]) -> dict[str, list[float]]:
    """
    从数据中提取所有数值型列，返回 {列名: [数值列表]}。
    跳过无法转换为浮点数的列。
    """
    if not data:
        return {}

    columns = {}
    headers = list(data[0].keys())

    for h in headers:
        values = []
        for row in data:
            try:
                values.append(float(row.get(h, 0)))
            except (ValueError, TypeError):
                break
        else:
            # 所有行都能成功转换
            if values:
                columns[h] = values

    return columns


def compute_statistics(columns: dict[str, list[float]]) -> dict[str, dict]:
    """
    计算每个数值列的统计指标：
      count, mean, std, min, max, median, p5, p95
    """
    stats = {}
    for name, vals in columns.items():
        n = len(vals)
        mean = sum(vals) / n if n else 0
        variance = sum((x - mean) ** 2 for x in vals) / n if n else 0
        std = variance ** 0.5
        sorted_vals = sorted(vals)
        median = sorted_vals[n // 2] if n else 0
        p5 = sorted_vals[int(n * 0.05)] if n > 1 else sorted_vals[0]
        p95 = sorted_vals[int(n * 0.95)] if n > 1 else sorted_vals[0]

        stats[name] = {
            'count': n,
            'mean': mean,
            'std': std,
            'min': min(vals) if vals else 0,
            'max': max(vals) if vals else 0,
            'median': median,
            'p5': p5,
            'p95': p95,
        }
    return stats


def check_spec(stats: dict[str, dict], spec: dict) -> dict[str, dict]:
    """
    对照规格文件判定每项指标是否达标。
    返回 {指标名: {pass: bool, detail: str, ...}}
    """
    results = {}
    for name, s in stats.items():
        if name not in spec:
            continue
        sp = spec[name]
        vmin = sp.get('min', float('-inf'))
        vmax = sp.get('max', float('inf'))
        unit = sp.get('unit', '')
        passed = vmin <= s['mean'] <= vmax

        results[name] = {
            'pass': passed,
            'mean': s['mean'],
            'spec_min': vmin,
            'spec_max': vmax,
            'unit': unit,
            'detail': f"{'PASS' if passed else 'FAIL'}: {s['mean']:.4g}{unit} (要求 {vmin}~{vmax}{unit})"
        }
    return results


def plot_line_chart(columns: dict[str, list[float]], output_dir: str, title: str = "测试数据趋势"):
    """生成折线图。"""
    if not HAS_MPL or not columns:
        return
    fig, ax = plt.subplots(figsize=(10, 5))
    for name, vals in columns.items():
        ax.plot(range(len(vals)), vals, marker='.', markersize=3, label=name)
    ax.set_xlabel('测试序号')
    ax.set_ylabel('数值')
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    path = os.path.join(output_dir, 'chart_line.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  [图表] 折线图: {path}")


def plot_bar_chart(stats: dict[str, dict], output_dir: str, title: str = "指标均值对比"):
    """生成柱状图（均值 + 误差棒）。"""
    if not HAS_MPL or not stats:
        return
    names = list(stats.keys())
    means = [stats[n]['mean'] for n in names]
    stds = [stats[n]['std'] for n in names]

    fig, ax = plt.subplots(figsize=(10, 5))
    x = range(len(names))
    ax.bar(x, means, yerr=stds, capsize=5, color='steelblue', alpha=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=45, ha='right')
    ax.set_ylabel('均值')
    ax.set_title(title)
    ax.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(output_dir, 'chart_bar.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  [图表] 柱状图: {path}")


def plot_box_chart(columns: dict[str, list[float]], output_dir: str, title: str = "数据分布箱线图"):
    """生成箱线图。"""
    if not HAS_MPL or not columns:
        return
    fig, ax = plt.subplots(figsize=(10, 5))
    data_list = list(columns.values())
    labels = list(columns.keys())
    bp = ax.boxplot(data_list, labels=labels, patch_artist=True)
    colors = plt.cm.Set3([i / len(data_list) for i in range(len(data_list))])
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
    ax.set_ylabel('数值')
    ax.set_title(title)
    ax.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(output_dir, 'chart_box.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  [图表] 箱线图: {path}")


def plot_hist_chart(columns: dict[str, list[float]], output_dir: str, title: str = "数据分布直方图"):
    """生成直方图（每个数值列一个子图）。"""
    if not HAS_MPL or not columns:
        return
    n = len(columns)
    cols = min(n, 3)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 3.5 * rows))
    if n == 1:
        axes = [axes]
    elif rows == 1:
        axes = list(axes)
    else:
        axes = [axes[i][j] for i in range(rows) for j in range(cols)]

    for idx, (name, vals) in enumerate(columns.items()):
        ax = axes[idx]
        ax.hist(vals, bins=20, color='steelblue', alpha=0.7, edgecolor='white')
        ax.set_title(name)
        ax.set_xlabel('数值')
        ax.set_ylabel('频次')

    # 隐藏多余子图
    for idx in range(n, len(axes)):
        axes[idx].set_visible(False)

    fig.suptitle(title, fontsize=14)
    plt.tight_layout()
    path = os.path.join(output_dir, 'chart_hist.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  [图表] 直方图: {path}")


def generate_text_report(stats: dict[str, dict], spec_results: dict[str, dict]) -> str:
    """生成文本格式的分析报告。"""
    lines = []
    lines.append("=" * 60)
    lines.append("测试结果分析报告")
    lines.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("=" * 60)
    lines.append("")

    # 统计摘要
    lines.append("【统计摘要】")
    lines.append(f"{'指标':<15} {'数量':>6} {'均值':>12} {'标准差':>12} {'最小值':>12} {'最大值':>12} {'中位数':>12}")
    lines.append("-" * 90)
    for name, s in stats.items():
        lines.append(f"{name:<15} {s['count']:>6} {s['mean']:>12.4f} {s['std']:>12.4f} {s['min']:>12.4f} {s['max']:>12.4f} {s['median']:>12.4f}")
    lines.append("")

    # 达标判定
    if spec_results:
        lines.append("【达标判定】")
        pass_count = sum(1 for r in spec_results.values() if r['pass'])
        fail_count = len(spec_results) - pass_count
        lines.append(f"  总计: {len(spec_results)} 项 | 通过: {pass_count} 项 | 未通过: {fail_count} 项")
        lines.append("")
        for name, r in spec_results.items():
            status = "✅" if r['pass'] else "❌"
            lines.append(f"  {status} {name}: {r['detail']}")
        lines.append("")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description='测试结果分析器 - 解析测试数据并生成统计图表',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python test_result_analyzer.py -i results.csv -o ./analysis/
  python test_result_analyzer.py -i results.json --spec spec.json -o ./analysis/
  python test_result_analyzer.py -i results.csv --charts line bar box hist
  python test_result_analyzer.py -i results.csv --report-only
        """
    )
    parser.add_argument('--input', '-i', required=True, help='测试数据文件 (CSV 或 JSON)')
    parser.add_argument('--output', '-o', default='./analysis', help='输出目录 (默认: ./analysis)')
    parser.add_argument('--spec', '-s', default=None, help='指标规格文件 (JSON)，用于达标判定')
    parser.add_argument('--charts', nargs='+', default=['line', 'bar', 'box', 'hist'],
                        choices=['line', 'bar', 'box', 'hist'],
                        help='要生成的图表类型 (默认: 全部)')
    parser.add_argument('--report-only', action='store_true', help='仅输出文本报告，不生成图表')
    parser.add_argument('--title', default='测试数据分析', help='图表标题')

    args = parser.parse_args()

    # 检查输入
    if not os.path.isfile(args.input):
        print(f"[ERROR] 输入文件不存在: {args.input}")
        sys.exit(1)

    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)

    # 加载数据
    data = load_data(args.input)
    if not data:
        print("[ERROR] 未加载到任何数据")
        sys.exit(1)

    # 提取数值列
    columns = extract_numeric_columns(data)
    if not columns:
        print("[WARN] 未找到数值型列，尝试将所有列转为数值...")
        # 尝试将所有列转数值
        for h in data[0].keys():
            vals = []
            for row in data:
                try:
                    vals.append(float(row.get(h, 0)))
                except (ValueError, TypeError):
                    vals.append(0)
            columns[h] = vals

    print(f"[INFO] 识别到 {len(columns)} 个数值字段: {list(columns.keys())}")

    # 计算统计
    stats = compute_statistics(columns)

    # 达标判定
    spec_results = {}
    if args.spec:
        if os.path.isfile(args.spec):
            spec = load_spec(args.spec)
            spec_results = check_spec(stats, spec)
            print(f"[INFO] 加载规格文件，判定 {len(spec_results)} 项指标")
        else:
            print(f"[WARN] 规格文件不存在: {args.spec}")

    # 生成文本报告
    report_text = generate_text_report(stats, spec_results)
    report_path = os.path.join(args.output, 'analysis_report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text)
    print(f"[OK] 文本报告: {report_path}")
    print(report_text)

    # 生成图表
    if not args.report_only and HAS_MPL:
        chart_funcs = {
            'line': lambda: plot_line_chart(columns, args.output, args.title),
            'bar': lambda: plot_bar_chart(stats, args.output, f"{args.title} - 均值对比"),
            'box': lambda: plot_box_chart(columns, args.output, f"{args.title} - 分布"),
            'hist': lambda: plot_hist_chart(columns, args.output, f"{args.title} - 直方图"),
        }
        for chart_type in args.charts:
            chart_funcs[chart_type]()
    elif not HAS_MPL:
        print("[WARN] 未安装 matplotlib，跳过图表生成 (pip install matplotlib)")

    # 保存 JSON 格式的统计数据
    stats_json_path = os.path.join(args.output, 'statistics.json')
    with open(stats_json_path, 'w', encoding='utf-8') as f:
        json.dump({'statistics': stats, 'spec_check': spec_results}, f, ensure_ascii=False, indent=2)
    print(f"[OK] JSON 统计: {stats_json_path}")

    print(f"\n[DONE] 分析完成，结果保存在: {args.output}")


if __name__ == '__main__':
    main()
