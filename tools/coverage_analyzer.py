#!/usr/bin/env python3
"""测试覆盖率分析器 - 分析测试文件对驱动/模块的覆盖情况"""

import argparse
import os
import re
from collections import defaultdict


def find_files(root, patterns):
    """递归查找匹配的文件"""
    result = []
    for dirpath, _, filenames in os.walk(root):
        for f in filenames:
            if any(f.endswith(p) for p in patterns):
                result.append(os.path.join(dirpath, f))
    return result


def extract_functions(filepath):
    """从源文件提取函数/方法定义"""
    funcs = set()
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                # C 函数定义
                m = re.match(r'^\s*(?:[\w\s\*]+)\s+(\w+)\s*\(', line)
                if m and not line.strip().startswith('//') and not line.strip().startswith('#'):
                    funcs.add(m.group(1))
                # Python 函数定义
                m = re.match(r'^\s*def\s+(\w+)\s*\(', line)
                if m:
                    funcs.add(m.group(1))
    except Exception:
        pass
    return funcs


def extract_tested_symbols(filepath):
    """从测试文件提取被调用/测试的符号"""
    symbols = set()
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
            # 匹配函数调用
            for m in re.finditer(r'\b(\w+)\s*\(', content):
                symbols.add(m.group(1))
            # 匹配断言中引用的符号
            for m in re.finditer(r'(?:assert|check|test|verify)\w*\s*\(\s*(\w+)', content):
                symbols.add(m.group(1))
    except Exception:
        pass
    return symbols


def analyze_coverage(src_root, test_root=None):
    """分析覆盖率"""
    if test_root is None:
        test_root = src_root

    # 查找源文件和测试文件
    src_files = find_files(src_root, ['.c', '.h', '.py'])
    test_files = find_files(test_root, ['_test.py', '_test.c', 'test_', 'Test'])

    # 提取源文件中的函数
    all_functions = {}  # {filepath: set(funcnames)}
    for f in src_files:
        funcs = extract_functions(f)
        if funcs:
            all_functions[f] = funcs

    # 提取测试文件中覆盖的符号
    tested_symbols = set()
    test_coverage = {}  # {testfile: set(symbols)}
    for f in test_files:
        syms = extract_tested_symbols(f)
        tested_symbols |= syms
        test_coverage[f] = syms

    # 计算覆盖率
    total = 0
    covered = 0
    uncovered = {}
    for filepath, funcs in all_functions.items():
        file_covered = funcs & tested_symbols
        file_uncovered = funcs - tested_symbols
        total += len(funcs)
        covered += len(file_covered)
        if file_uncovered:
            uncovered[filepath] = file_uncovered

    return {
        'total': total,
        'covered': covered,
        'coverage_pct': (covered / total * 100) if total else 0,
        'uncovered': uncovered,
        'src_files': len(src_files),
        'test_files': len(test_files),
        'test_coverage': test_coverage,
    }


def main():
    parser = argparse.ArgumentParser(description='测试覆盖率分析器')
    parser.add_argument('source', help='源代码目录')
    parser.add_argument('--test-dir', '-t', help='测试代码目录（默认同源目录）')
    parser.add_argument('--detailed', '-d', action='store_true', help='显示详细信息')
    parser.add_argument('--output', '-o', help='输出报告文件')
    args = parser.parse_args()

    result = analyze_coverage(args.source, args.test_dir)

    lines = []
    lines.append('=' * 60)
    lines.append('  测试覆盖率分析报告')
    lines.append('=' * 60)
    lines.append(f'  源文件数:   {result["src_files"]}')
    lines.append(f'  测试文件数: {result["test_files"]}')
    lines.append(f'  总函数数:   {result["total"]}')
    lines.append(f'  已覆盖:     {result["covered"]}')
    lines.append(f'  覆盖率:     {result["coverage_pct"]:.1f}%')
    lines.append('')

    if result['uncovered']:
        lines.append('未覆盖的函数:')
        lines.append('-' * 40)
        for filepath, funcs in result['uncovered'].items():
            relpath = os.path.relpath(filepath, args.source)
            for f in sorted(funcs):
                lines.append(f'  {relpath} :: {f}')

    if args.detailed:
        lines.append('')
        lines.append('测试文件覆盖详情:')
        lines.append('-' * 40)
        for tf, syms in result['test_coverage'].items():
            relpath = os.path.relpath(tf, args.test_dir or args.source)
            lines.append(f'  {relpath}: {len(syms)} 个符号')

    report = '\n'.join(lines)
    print(report)

    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(report)
        print(f'\n报告已保存到: {args.output}')


if __name__ == '__main__':
    main()
