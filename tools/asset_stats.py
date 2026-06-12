#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
资产库统计工具 - 统计文件数/代码行数/测试覆盖率/模块分布
用法: python asset_stats.py --root . --output stats.md --format md
"""

import argparse
import os
from datetime import datetime
from collections import defaultdict


# 代码文件扩展名
CODE_EXTS = {".c", ".h", ".py", ".java", ".cpp", ".hpp", ".js", ".ts"}
DOC_EXTS = {".md", ".txt", ".rst", ".pdf"}
CONFIG_EXTS = {".json", ".yaml", ".yml", ".toml", ".cfg", ".ini", ".syscfg"}
TEST_EXTS = {".c", ".h", ".py"}  # 测试文件通过前缀判断


def count_lines(filepath: str) -> dict:
    """统计单个文件的行数（总行/代码行/注释行/空行）"""
    stats = {"total": 0, "code": 0, "comment": 0, "blank": 0}
    try:
        with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
            in_block_comment = False
            for line in f:
                stats["total"] += 1
                stripped = line.strip()

                if not stripped:
                    stats["blank"] += 1
                    continue

                # 块注释
                if "/*" in stripped:
                    in_block_comment = True
                if in_block_comment:
                    stats["comment"] += 1
                    if "*/" in stripped:
                        in_block_comment = False
                    continue

                # 行注释
                if stripped.startswith("//") or stripped.startswith("#") and not stripped.startswith("#!"):
                    stats["comment"] += 1
                    continue

                stats["code"] += 1
    except Exception:
        pass
    return stats


def is_test_file(filepath: str) -> bool:
    """判断是否为测试文件"""
    name = os.path.basename(filepath).lower()
    path = filepath.lower()
    return ("test" in name or "test" in path or name.startswith("test_") or
            "spec" in name or "_test." in name)


def is_doc_file(filepath: str) -> bool:
    """判断是否为文档文件"""
    return os.path.splitext(filepath)[1].lower() in DOC_EXTS


def scan_repository(root: str) -> dict:
    """
    扫描整个资产库，返回统计数据
    """
    stats = {
        "total_files": 0,
        "code_files": 0,
        "doc_files": 0,
        "config_files": 0,
        "other_files": 0,
        "total_lines": 0,
        "code_lines": 0,
        "comment_lines": 0,
        "blank_lines": 0,
        "test_files": 0,
        "test_lines": 0,
        "by_extension": defaultdict(lambda: {"files": 0, "lines": 0, "code": 0}),
        "by_module": defaultdict(lambda: {"files": 0, "lines": 0, "code": 0, "tests": 0}),
        "by_directory": defaultdict(lambda: {"files": 0, "lines": 0}),
    }

    ignore_dirs = {".git", ".vscode", "__pycache__", "node_modules", "build", "output", ".idea"}

    for dirpath, dirnames, filenames in os.walk(root):
        # 跳过忽略目录
        dirnames[:] = [d for d in dirnames if d not in ignore_dirs]

        for fname in filenames:
            filepath = os.path.join(dirpath, fname)
            ext = os.path.splitext(fname)[1].lower()
            rel_path = os.path.relpath(filepath, root)

            stats["total_files"] += 1

            # 分类
            if ext in CODE_EXTS:
                stats["code_files"] += 1
                # 计算行数
                ls = count_lines(filepath)
                stats["total_lines"] += ls["total"]
                stats["code_lines"] += ls["code"]
                stats["comment_lines"] += ls["comment"]
                stats["blank_lines"] += ls["blank"]

                stats["by_extension"][ext]["files"] += 1
                stats["by_extension"][ext]["lines"] += ls["total"]
                stats["by_extension"][ext]["code"] += ls["code"]

                # 测试文件统计
                if is_test_file(filepath):
                    stats["test_files"] += 1
                    stats["test_lines"] += ls["code"]

                # 按模块(第一级目录)统计
                parts = rel_path.replace("\\", "/").split("/")
                module = parts[0] if len(parts) > 1 else "(root)"
                stats["by_module"][module]["files"] += 1
                stats["by_module"][module]["lines"] += ls["total"]
                stats["by_module"][module]["code"] += ls["code"]
                if is_test_file(filepath):
                    stats["by_module"][module]["tests"] += 1

            elif is_doc_file(filepath):
                stats["doc_files"] += 1
            elif ext in CONFIG_EXTS:
                stats["config_files"] += 1
            else:
                stats["other_files"] += 1

            # 按目录统计
            top_dir = os.path.relpath(dirpath, root).split(os.sep)[0] if dirpath != root else "(root)"
            stats["by_directory"][top_dir]["files"] += 1

    return stats


def format_report(stats: dict, fmt: str = "md") -> str:
    """格式化统计报告"""
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    if fmt == "md":
        return _format_md(stats, now)
    elif fmt == "txt":
        return _format_txt(stats, now)
    else:
        return _format_txt(stats, now)


def _format_md(stats: dict, now: str) -> str:
    """Markdown格式报告"""
    test_cov = (stats["test_files"] / stats["code_files"] * 100) if stats["code_files"] > 0 else 0

    lines = [
        f"# 📊 nuedc-asset-library统计报告",
        f"",
        f"> 生成时间: {now}",
        f"> 由 asset_stats.py 自动生成",
        f"",
        f"## 📁 总体概览",
        f"",
        f"| 指标 | 数值 |",
        f"|------|------|",
        f"| 文件总数 | {stats['total_files']} |",
        f"| 代码文件 | {stats['code_files']} |",
        f"| 文档文件 | {stats['doc_files']} |",
        f"| 配置文件 | {stats['config_files']} |",
        f"| 其他文件 | {stats['other_files']} |",
        f"",
        f"## 📝 代码行数",
        f"",
        f"| 指标 | 行数 |",
        f"|------|------|",
        f"| 总行数 | {stats['total_lines']:,} |",
        f"| 代码行 | {stats['code_lines']:,} |",
        f"| 注释行 | {stats['comment_lines']:,} |",
        f"| 空行   | {stats['blank_lines']:,} |",
        f"| 注释率 | {stats['comment_lines']/max(stats['total_lines'],1)*100:.1f}% |",
        f"",
        f"## 🧪 测试覆盖",
        f"",
        f"| 指标 | 数值 |",
        f"|------|------|",
        f"| 测试文件数 | {stats['test_files']} |",
        f"| 测试代码行 | {stats['test_lines']:,} |",
        f"| 测试文件比例 | {test_cov:.1f}% |",
        f"",
        f"## 📦 按文件类型分布",
        f"",
        f"| 扩展名 | 文件数 | 总行数 | 代码行 |",
        f"|--------|--------|--------|--------|",
    ]
    for ext, data in sorted(stats["by_extension"].items(), key=lambda x: -x[1]["code"]):
        lines.append(f"| `{ext}` | {data['files']} | {data['lines']:,} | {data['code']:,} |")

    lines += [
        f"",
        f"## 📂 按模块分布",
        f"",
        f"| 模块 | 文件数 | 总行数 | 代码行 | 测试文件 |",
        f"|------|--------|--------|--------|----------|",
    ]
    for mod, data in sorted(stats["by_module"].items(), key=lambda x: -x[1]["code"]):
        lines.append(f"| `{mod}` | {data['files']} | {data['lines']:,} | {data['code']:,} | {data['tests']} |")

    lines += [
        f"",
        f"---",
        f"*报告由 asset_stats.py 自动生成*",
    ]
    return "\n".join(lines)


def _format_txt(stats: dict, now: str) -> str:
    """纯文本格式报告"""
    test_cov = (stats["test_files"] / stats["code_files"] * 100) if stats["code_files"] > 0 else 0

    lines = [
        "=" * 50,
        "nuedc-asset-library统计报告",
        f"生成时间: {now}",
        "=" * 50,
        "",
        "【总体概览】",
        f"  文件总数: {stats['total_files']}",
        f"  代码文件: {stats['code_files']}",
        f"  文档文件: {stats['doc_files']}",
        f"  配置文件: {stats['config_files']}",
        "",
        "【代码行数】",
        f"  总行数:   {stats['total_lines']:,}",
        f"  代码行:   {stats['code_lines']:,}",
        f"  注释行:   {stats['comment_lines']:,}",
        f"  空行:     {stats['blank_lines']:,}",
        "",
        "【测试覆盖】",
        f"  测试文件: {stats['test_files']}",
        f"  测试代码: {stats['test_lines']:,} 行",
        f"  覆盖比例: {test_cov:.1f}%",
        "",
        "【模块分布】",
    ]
    for mod, data in sorted(stats["by_module"].items(), key=lambda x: -x[1]["code"]):
        lines.append(f"  {mod:<20s} 文件:{data['files']:>4d}  代码:{data['code']:>6d}行  测试:{data['tests']}")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="资产库统计工具 - 统计文件/代码/测试/模块分布")
    parser.add_argument("--root", default=".", help="资产库根目录")
    parser.add_argument("--output", help="输出统计报告文件路径")
    parser.add_argument("--format", choices=["md", "txt"], default="md", help="输出格式")
    parser.add_argument("--summary", action="store_true", help="仅输出摘要到终端")
    args = parser.parse_args()

    if not os.path.isdir(args.root):
        print(f"[✗] 目录不存在: {args.root}")
        return

    print(f"[i] 正在扫描: {os.path.abspath(args.root)}")
    stats = scan_repository(args.root)

    # 生成报告
    report = format_report(stats, args.format)

    if args.summary:
        test_cov = (stats["test_files"] / stats["code_files"] * 100) if stats["code_files"] > 0 else 0
        print(f"\n=== 资产库摘要 ===")
        print(f"  文件: {stats['total_files']} | 代码文件: {stats['code_files']} | 文档: {stats['doc_files']}")
        print(f"  代码行: {stats['code_lines']:,} | 注释行: {stats['comment_lines']:,}")
        print(f"  测试文件: {stats['test_files']} | 测试覆盖: {test_cov:.1f}%")
        print(f"  模块数: {len(stats['by_module'])}")
        return

    # 输出到终端
    print(report)

    # 写入文件
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)) or ".", exist_ok=True)
        with open(args.output, "w", encoding="utf-8", newline="\n") as f:
            f.write(report)
        print(f"\n[✓] 报告已保存: {args.output}")


if __name__ == "__main__":
    main()
