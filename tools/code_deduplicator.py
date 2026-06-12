#!/usr/bin/env python3
"""代码去重工具 - 检测项目中的重复代码段"""

import argparse
import hashlib
import os
from collections import defaultdict


# 需要扫描的文件扩展名
CODE_EXTS = {'.c', '.h', '.py', '.cpp', '.hpp', '.java'}


def normalize_line(line):
    """规范化一行代码：去除空白和注释"""
    stripped = line.strip()
    # 跳过空行和纯注释
    if not stripped or stripped.startswith('//') or stripped.startswith('#') or stripped.startswith('/*'):
        return None
    # 去除多余空白
    return ' '.join(stripped.split())


def extract_blocks(filepath, min_lines=5):
    """提取文件中的代码块（连续非空行）"""
    blocks = []
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()

        current_block = []
        block_start = 0
        for i, line in enumerate(lines):
            normalized = normalize_line(line)
            if normalized:
                if not current_block:
                    block_start = i + 1
                current_block.append(normalized)
            else:
                if len(current_block) >= min_lines:
                    blocks.append((block_start, current_block[:]))
                current_block = []
        if len(current_block) >= min_lines:
            blocks.append((block_start, current_block[:]))
    except Exception:
        pass
    return blocks


def hash_block(lines):
    """计算代码块的哈希"""
    content = '\n'.join(lines)
    return hashlib.md5(content.encode()).hexdigest()


def find_duplicates(root, min_lines=5, min_similarity=0.8):
    """查找重复代码段"""
    # 收集所有代码块
    all_blocks = []  # (filepath, start_line, lines, hash)
    for dirpath, _, filenames in os.walk(root):
        for f in filenames:
            if any(f.endswith(ext) for ext in CODE_EXTS):
                fp = os.path.join(dirpath, f)
                for start, lines in extract_blocks(fp, min_lines):
                    h = hash_block(lines)
                    all_blocks.append((fp, start, lines, h))

    # 按哈希分组找完全重复
    hash_groups = defaultdict(list)
    for fp, start, lines, h in all_blocks:
        hash_groups[h].append((fp, start, lines))

    duplicates = []
    for h, items in hash_groups.items():
        if len(items) > 1:
            duplicates.append(items)

    # 按相似度查找部分重复（滑动窗口比较）
    # 简化实现：比较行集的Jaccard相似度
    if min_similarity < 1.0:
        n = len(all_blocks)
        for i in range(n):
            for j in range(i + 1, n):
                fp1, s1, l1, h1 = all_blocks[i]
                fp2, s2, l2, h2 = all_blocks[j]
                if h1 == h2:
                    continue  # 已经被精确匹配捕获
                if fp1 == fp2 and abs(s1 - s2) < min_lines:
                    continue  # 相邻块跳过
                set1 = set(l1)
                set2 = set(l2)
                if not set1 or not set2:
                    continue
                jaccard = len(set1 & set2) / len(set1 | set2)
                if jaccard >= min_similarity:
                    duplicates.append([
                        (fp1, s1, l1),
                        (fp2, s2, l2),
                        ('similarity', jaccard)
                    ])

    return duplicates


def main():
    parser = argparse.ArgumentParser(description='代码去重工具')
    parser.add_argument('root', help='项目根目录')
    parser.add_argument('--min-lines', '-n', type=int, default=5,
                        help='最小重复行数（默认5）')
    parser.add_argument('--similarity', '-s', type=float, default=0.8,
                        help='最小相似度阈值（0-1，默认0.8）')
    parser.add_argument('--output', '-o', help='输出报告文件')
    args = parser.parse_args()

    duplicates = find_duplicates(args.root, args.min_lines, args.similarity)

    lines = []
    lines.append('=' * 60)
    lines.append('  代码重复检测报告')
    lines.append('=' * 60)
    lines.append(f'  最小重复行数: {args.min_lines}')
    lines.append(f'  相似度阈值:   {args.similarity}')
    lines.append(f'  发现重复组数: {len(duplicates)}')
    lines.append('')

    total_dup_lines = 0
    for i, group in enumerate(duplicates, 1):
        lines.append(f'--- 重复组 #{i} ---')
        # 检查是否有相似度标记
        similarity_info = None
        items = group
        if isinstance(group[-1], tuple) and group[-1][0] == 'similarity':
            similarity_info = group[-1][1]
            items = group[:-1]

        for fp, start, code_lines in items:
            relpath = os.path.relpath(fp, args.root)
            lines.append(f'  {relpath}:{start} ({len(code_lines)} 行)')
            # 显示前3行预览
            for cl in code_lines[:3]:
                lines.append(f'    | {cl}')
            if len(code_lines) > 3:
                lines.append(f'    | ... ({len(code_lines) - 3} more)')

        if similarity_info:
            lines.append(f'  相似度: {similarity_info:.0%}')

        total_dup_lines += sum(len(item[2]) for item in items)
        lines.append('')

    if total_dup_lines:
        lines.append(f'重复代码总行数: {total_dup_lines}')
        lines.append(f'建议：将重复代码提取为公共函数/宏')

    report = '\n'.join(lines)
    print(report)

    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(report)
        print(f'\n报告已保存到: {args.output}')


if __name__ == '__main__':
    main()
