#!/usr/bin/env python3
"""
代码复杂度分析器 - 圈复杂度、认知复杂度、函数长度分析
用法: python code_complexity.py <文件或目录> [--threshold 10] [--format text|json]
"""
import argparse
import ast
import sys
import os
import json
from pathlib import Path
from dataclasses import dataclass, field
from typing import List


@dataclass
class FunctionMetrics:
    """函数度量数据"""
    name: str
    file: str
    line_start: int
    line_end: int
    cyclomatic: int = 1  # 圈复杂度
    cognitive: int = 0   # 认知复杂度
    loc: int = 0         # 行数
    params: int = 0      # 参数数量
    nesting_depth: int = 0


class ComplexityVisitor(ast.NodeVisitor):
    """AST访问器，计算复杂度指标"""

    def __init__(self, filename: str):
        self.filename = filename
        self.results: List[FunctionMetrics] = []
        self._nesting = 0

    def _count_branches(self, node) -> int:
        """计算圈复杂度（每个分支+1）"""
        count = 0
        for child in ast.walk(node):
            if isinstance(child, (ast.If, ast.While, ast.For, ast.AsyncFor)):
                count += 1
            elif isinstance(child, ast.ExceptHandler):
                count += 1
            elif isinstance(child, (ast.With, ast.AsyncWith)):
                count += 1
            elif isinstance(child, ast.BoolOp):
                count += len(child.values) - 1
            elif isinstance(child, ast.comprehension):
                count += 1
                if child.ifs:
                    count += len(child.ifs)
        return count

    def _count_cognitive(self, node, depth=0) -> int:
        """计算认知复杂度（考虑嵌套权重）"""
        count = 0
        for child in ast.iter_child_nodes(node):
            # 跳过嵌套函数/类，不递归进入
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                count += depth + 1
                continue
            if isinstance(child, (ast.If, ast.While, ast.For, ast.AsyncFor)):
                count += 1 + depth
                count += self._count_cognitive(child, depth + 1)
            elif isinstance(child, ast.ExceptHandler):
                count += 1 + depth
                count += self._count_cognitive(child, depth + 1)
            elif isinstance(child, ast.BoolOp):
                count += len(child.values) - 1
            elif isinstance(child, (ast.Break, ast.Continue)):
                count += 1
            else:
                count += self._count_cognitive(child, depth)
        return count

    def _visit_function(self, node):
        """处理函数定义"""
        # 计算行数
        loc = node.end_lineno - node.lineno + 1 if node.end_lineno else 0

        # 参数数量
        args = node.args
        params = len(args.args) + len(args.posonlyargs) + len(args.kwonlyargs)
        if args.vararg: params += 1
        if args.kwarg: params += 1

        # 圈复杂度
        cyclomatic = 1 + self._count_branches(node)

        # 认知复杂度
        cognitive = self._count_cognitive(node)

        metrics = FunctionMetrics(
            name=node.name,
            file=self.filename,
            line_start=node.lineno,
            line_end=node.end_lineno or node.lineno,
            cyclomatic=cyclomatic,
            cognitive=cognitive,
            loc=loc,
            params=params,
            nesting_depth=self._nesting,
        )
        self.results.append(metrics)

    def visit_FunctionDef(self, node):
        self._visit_function(node)
        self._nesting += 1
        self.generic_visit(node)
        self._nesting -= 1

    def visit_AsyncFunctionDef(self, node):
        self._visit_function(node)
        self._nesting += 1
        self.generic_visit(node)
        self._nesting -= 1


def analyze_file(filepath: str) -> List[FunctionMetrics]:
    """分析单个文件"""
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            source = f.read()
        tree = ast.parse(source)
        visitor = ComplexityVisitor(filepath)
        visitor.visit(tree)
        return visitor.results
    except (SyntaxError, UnicodeDecodeError) as e:
        print(f"⚠️  无法解析 {filepath}: {e}", file=sys.stderr)
        return []


def get_rating(value: int, thresholds=(5, 10, 20)) -> str:
    """获取评级"""
    if value <= thresholds[0]:
        return "A(优)"
    elif value <= thresholds[1]:
        return "B(良)"
    elif value <= thresholds[2]:
        return "C(中)"
    else:
        return "D(差)"


def main():
    parser = argparse.ArgumentParser(description="Python代码复杂度分析器")
    parser.add_argument("path", help="Python文件或目录")
    parser.add_argument("--threshold", "-t", type=int, default=10, help="复杂度警告阈值 (默认10)")
    parser.add_argument("--format", "-f", choices=["text", "json"], default="text", help="输出格式")
    parser.add_argument("--sort", "-s", choices=["cyclomatic", "cognitive", "loc", "name"],
                       default="cyclomatic", help="排序方式")
    parser.add_argument("--top", type=int, default=0, help="只显示前N个最复杂的函数")
    args = parser.parse_args()

    path = Path(args.path)
    all_metrics = []

    if path.is_file():
        all_metrics.extend(analyze_file(str(path)))
    elif path.is_dir():
        for py_file in sorted(path.rglob("*.py")):
            all_metrics.extend(analyze_file(str(py_file)))
    else:
        print(f"❌ 路径不存在: {path}")
        sys.exit(1)

    if not all_metrics:
        print("⚠️  未找到任何函数定义")
        sys.exit(0)

    # 排序
    reverse = args.sort != "name"
    all_metrics.sort(key=lambda m: getattr(m, args.sort), reverse=reverse)

    if args.top > 0:
        all_metrics = all_metrics[:args.top]

    # JSON输出
    if args.format == "json":
        data = [
            {
                "name": m.name, "file": m.file,
                "line": f"{m.line_start}-{m.line_end}",
                "cyclomatic": m.cyclomatic, "cognitive": m.cognitive,
                "loc": m.loc, "params": m.params,
                "rating": get_rating(m.cyclomatic),
            }
            for m in all_metrics
        ]
        print(json.dumps(data, ensure_ascii=False, indent=2))
        return

    # 文本输出
    print("=" * 90)
    print(f"{'函数':<30} {'文件':<25} {'圈复杂':>6} {'认知':>6} {'行数':>5} {'参数':>4} {'评级':>6}")
    print("=" * 90)

    warnings = 0
    for m in all_metrics:
        rating = get_rating(m.cyclomatic)
        fname = os.path.basename(m.file)
        if m.cyclomatic > args.threshold:
            warnings += 1
        print(f"{m.name:<30} {fname:<25} {m.cyclomatic:>6} {m.cognitive:>6} {m.loc:>5} {m.params:>4} {rating:>6}")

    print("=" * 90)

    # 统计
    cyc_values = [m.cyclomatic for m in all_metrics]
    cog_values = [m.cognitive for m in all_metrics]
    loc_values = [m.loc for m in all_metrics]

    print(f"\n📊 统计:")
    print(f"   函数总数: {len(all_metrics)}")
    print(f"   圈复杂度 - 平均: {sum(cyc_values)/len(cyc_values):.1f}  最大: {max(cyc_values)}  最小: {min(cyc_values)}")
    print(f"   认知复杂度 - 平均: {sum(cog_values)/len(cog_values):.1f}  最大: {max(cog_values)}")
    print(f"   函数长度 - 平均: {sum(loc_values)/len(loc_values):.1f}行  最长: {max(loc_values)}行")

    if warnings:
        print(f"\n⚠️  {warnings} 个函数复杂度超过阈值 {args.threshold}，建议重构!")
    else:
        print(f"\n✅ 所有函数复杂度均在阈值 {args.threshold} 以内")


if __name__ == "__main__":
    main()
