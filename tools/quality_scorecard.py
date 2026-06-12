#!/usr/bin/env python3
"""代码质量评分卡 - nuedc-asset-library V2审计工具"""

import os
import re
import ast
import sys
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Dict, Tuple

REPO_ROOT = Path(r"./nuedc-asset-library")


@dataclass
class FileScore:
    filepath: str
    scores: Dict[str, float] = field(default_factory=dict)
    total: float = 0.0

    def calc_total(self):
        if self.scores:
            self.total = sum(self.scores.values()) / len(self.scores)
        return self.total


# ── C 语言分析 ──────────────────────────────────────────────

def analyze_c_file(path: Path) -> FileScore:
    fs = FileScore(filepath=str(path.relative_to(REPO_ROOT)))
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except Exception:
        fs.total = 0
        return fs

    lines = text.splitlines()
    total_lines = len(lines)
    if total_lines == 0:
        fs.total = 0
        return fs

    # 1. 注释覆盖率
    comment_lines = 0
    in_block = False
    for line in lines:
        stripped = line.strip()
        if in_block:
            comment_lines += 1
            if "*/" in stripped:
                in_block = False
            continue
        if stripped.startswith("//") or stripped.startswith("/*"):
            comment_lines += 1
            if "/*" in stripped and "*/" not in stripped:
                in_block = True
    fs.scores["注释覆盖率"] = min(100, (comment_lines / total_lines) * 500)  # 20% lines = 100分

    # 2. 函数文档率
    func_pattern = re.compile(
        r'^[a-zA-Z_]\w[\w\s\*]*\s+([a-zA-Z_]\w*)\s*\([^)]*\)\s*\{', re.MULTILINE
    )
    functions = func_pattern.findall(text)
    # 排除关键字
    keywords = {"if", "else", "for", "while", "switch", "do", "return", "sizeof", "main"}
    functions = [f for f in functions if f not in keywords]

    documented = 0
    func_starts = [(m.start(), m.group(1)) for m in func_pattern.finditer(text) if m.group(1) not in keywords]
    for fstart, fname in func_starts:
        # 检查函数前是否有注释块
        prefix = text[:fstart].rstrip()
        if prefix.endswith("*/") or prefix.rstrip().endswith("*/"):
            documented += 1
        elif fstart > 1:
            prev_line = text[:fstart].splitlines()
            if len(prev_line) >= 2 and prev_line[-2].strip().startswith("//"):
                documented += 1

    total_funcs = max(len(func_starts), 1)
    fs.scores["函数文档率"] = min(100, (documented / total_funcs) * 100)

    # 3. 错误处理率
    error_handling = 0
    for fstart, fname in func_starts:
        # 找到函数体
        brace_count = 0
        body_start = text.find("{", fstart)
        if body_start == -1:
            continue
        i = body_start
        while i < len(text):
            if text[i] == "{":
                brace_count += 1
            elif text[i] == "}":
                brace_count -= 1
                if brace_count == 0:
                    break
            i += 1
        body = text[body_start:i+1]
        if any(kw in body for kw in ["if (", "if(", "return -", "return ERR", "goto ", "errno", "perror", "fprintf(stderr"]):
            error_handling += 1
    fs.scores["错误处理率"] = min(100, (error_handling / total_funcs) * 100)

    # 4. 命名一致性
    identifiers = re.findall(r'\b([a-zA-Z_]\w{3,})\b', text)
    camel = sum(1 for i in identifiers if re.search(r'[a-z][A-Z]', i) and "_" not in i)
    snake = sum(1 for i in identifiers if "_" in i and re.search(r'[a-z]', i))
    total_named = camel + snake
    if total_named > 0:
        dominant = max(camel, snake)
        consistency = dominant / total_named
    else:
        consistency = 1.0
    fs.scores["命名一致性"] = min(100, consistency * 100)

    fs.calc_total()
    return fs


# ── Python 分析 ─────────────────────────────────────────────

def _has_type_annotation(node) -> bool:
    """检查函数参数和返回值是否有类型注解"""
    args = node.args
    annotated = 0
    total = 0
    for arg in args.args + args.kwonlyargs:
        if arg.arg == "self":
            continue
        total += 1
        if arg.annotation:
            annotated += 1
    if node.returns:
        annotated += 1
    total += 1  # 返回值
    return (annotated / total) >= 0.5 if total > 0 else True


def analyze_py_file(path: Path) -> FileScore:
    fs = FileScore(filepath=str(path.relative_to(REPO_ROOT)))
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except Exception:
        fs.total = 0
        return fs

    if not text.strip():
        fs.total = 0
        return fs

    try:
        tree = ast.parse(text)
    except SyntaxError:
        fs.scores = {"docstring覆盖率": 0, "类型注解率": 0, "异常处理率": 0, "__main__守卫率": 0}
        fs.calc_total()
        return fs

    # 1. docstring覆盖率
    func_count = 0
    func_with_doc = 0
    class_count = 0
    class_with_doc = 0
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            func_count += 1
            if (node.body and isinstance(node.body[0], ast.Expr)
                    and isinstance(node.body[0].value, (ast.Constant,))
                    and isinstance(node.body[0].value.value, str)):
                func_with_doc += 1
        elif isinstance(node, ast.ClassDef):
            class_count += 1
            if (node.body and isinstance(node.body[0], ast.Expr)
                    and isinstance(node.body[0].value, ast.Constant)
                    and isinstance(node.body[0].value.value, str)):
                class_with_doc += 1

    total_items = func_count + class_count
    doc_items = func_with_doc + class_with_doc
    fs.scores["docstring覆盖率"] = min(100, (doc_items / max(total_items, 1)) * 100)

    # 2. 类型注解率
    annotated_funcs = 0
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if _has_type_annotation(node):
                annotated_funcs += 1
    fs.scores["类型注解率"] = min(100, (annotated_funcs / max(func_count, 1)) * 100)

    # 3. 异常处理率
    funcs_with_try = 0
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            for child in ast.walk(node):
                if isinstance(child, ast.Try):
                    funcs_with_try += 1
                    break
    fs.scores["异常处理率"] = min(100, (funcs_with_try / max(func_count, 1)) * 100)

    # 4. __main__守卫率
    has_main_guard = 'if __name__' in text and '__main__' in text
    # 只对有顶层可执行代码的文件检查
    has_top_level_calls = any(
        isinstance(node, (ast.Expr, ast.Assign)) and not isinstance(getattr(node, 'value', None), ast.Constant)
        for node in ast.iter_child_nodes(tree)
    ) if func_count == 0 else True

    # 如果文件只是定义（函数+类），不需要main守卫也算100
    if func_count > 0 or class_count > 0:
        # 有定义的文件：检查是否有main guard
        if has_main_guard:
            fs.scores["__main__守卫率"] = 100
        else:
            # 如果没有顶层调用代码，也算通过
            top_calls = [n for n in ast.iter_child_nodes(tree) if isinstance(n, ast.Expr) and not isinstance(n.value, ast.Constant)]
            if not top_calls:
                fs.scores["__main__守卫率"] = 80  # 有定义但没guard，扣少量分
            else:
                fs.scores["__main__守卫率"] = 0
    else:
        fs.scores["__main__守卫率"] = 100 if has_main_guard else 50

    fs.calc_total()
    return fs


# ── 主逻辑 ──────────────────────────────────────────────────

def scan_and_score(root: Path):
    c_files = list(root.rglob("*.c"))
    py_files = list(root.rglob("*.py"))
    # 排除 __pycache__
    py_files = [f for f in py_files if "__pycache__" not in str(f)]

    all_scores: List[FileScore] = []

    print("=" * 80)
    print("📊 nuedc-asset-library - 代码质量评分卡 V2")
    print("=" * 80)

    # C 文件
    if c_files:
        print(f"\n🔧 C语言文件 ({len(c_files)}个)")
        print("-" * 80)
        c_totals = {}
        for f in sorted(c_files):
            fs = analyze_c_file(f)
            all_scores.append(fs)
            dims = " | ".join(f"{k}:{v:.0f}" for k, v in fs.scores.items())
            grade = "🟢" if fs.total >= 80 else "🟡" if fs.total >= 60 else "🔴"
            print(f"  {grade} {fs.filepath:<55} 总分:{fs.total:5.1f}  [{dims}]")
            for k, v in fs.scores.items():
                c_totals.setdefault(k, []).append(v)

        print(f"\n  📈 C语言整体统计:")
        for k, vals in c_totals.items():
            avg = sum(vals) / len(vals)
            print(f"     {k}: {avg:.1f}")
        overall_c = sum(fs.total for fs in all_scores[-len(c_files):]) / len(c_files)
        print(f"     整体平均分: {overall_c:.1f}")

    # Python 文件
    if py_files:
        print(f"\n🐍 Python文件 ({len(py_files)}个)")
        print("-" * 80)
        py_totals = {}
        py_start = len(all_scores)
        for f in sorted(py_files):
            fs = analyze_py_file(f)
            all_scores.append(fs)
            dims = " | ".join(f"{k}:{v:.0f}" for k, v in fs.scores.items())
            grade = "🟢" if fs.total >= 80 else "🟡" if fs.total >= 60 else "🔴"
            print(f"  {grade} {fs.filepath:<55} 总分:{fs.total:5.1f}  [{dims}]")
            for k, v in fs.scores.items():
                py_totals.setdefault(k, []).append(v)

        print(f"\n  📈 Python整体统计:")
        for k, vals in py_totals.items():
            avg = sum(vals) / len(vals)
            print(f"     {k}: {avg:.1f}")
        py_scores = all_scores[py_start:]
        overall_py = sum(fs.total for fs in py_scores) / len(py_scores)
        print(f"     整体平均分: {overall_py:.1f}")

    # 总结
    print(f"\n{'=' * 80}")
    print(f"📋 总结")
    print(f"   C文件数: {len(c_files)}, Python文件数: {len(py_files)}")
    if all_scores:
        avg_all = sum(fs.total for fs in all_scores) / len(all_scores)
        print(f"   全库平均分: {avg_all:.1f}/100")
        high = sum(1 for fs in all_scores if fs.total >= 80)
        mid = sum(1 for fs in all_scores if 60 <= fs.total < 80)
        low = sum(1 for fs in all_scores if fs.total < 60)
        print(f"   🟢优秀(≥80): {high}  🟡良好(60-79): {mid}  🔴需改进(<60): {low}")
    print("=" * 80)


if __name__ == "__main__":
    root = REPO_ROOT
    if len(sys.argv) > 1:
        root = Path(sys.argv[1])
    if not root.exists():
        print(f"❌ 路径不存在: {root}")
        sys.exit(1)
    scan_and_score(root)
