#!/usr/bin/env python3
"""依赖图生成器 - 生成模块间的依赖关系图"""

import argparse
import os
import re
from collections import defaultdict


# 支持的语言和对应的导入模式
IMPORT_PATTERNS = {
    '.c': [
        re.compile(r'#include\s+[<"]([^>"]+)[>"]'),
    ],
    '.h': [
        re.compile(r'#include\s+[<"]([^>"]+)[>"]'),
    ],
    '.py': [
        re.compile(r'^\s*(?:from\s+([\w.]+)\s+import|import\s+([\w.]+))', re.MULTILINE),
    ],
}


def find_source_files(root, exts=None):
    """查找源文件"""
    if exts is None:
        exts = set(IMPORT_PATTERNS.keys())
    files = {}
    for dirpath, _, filenames in os.walk(root):
        for f in filenames:
            ext = os.path.splitext(f)[1]
            if ext in exts:
                fp = os.path.join(dirpath, f)
                # 使用相对于root的路径作为模块名
                rel = os.path.relpath(fp, root).replace('\\', '/')
                files[rel] = fp
    return files


def extract_dependencies(filepath, ext):
    """提取文件的依赖"""
    deps = set()
    patterns = IMPORT_PATTERNS.get(ext, [])
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        for pat in patterns:
            for m in pat.finditer(content):
                # 取第一个或第二个分组
                dep = m.group(1) if m.group(1) else m.group(2)
                if dep:
                    deps.add(dep)
    except Exception:
        pass
    return deps


def resolve_dep_to_module(dep, all_modules, current_module):
    """将导入的依赖解析为项目内的模块"""
    # 简单匹配：检查依赖名是否是某个模块的子串
    dep_normalized = dep.replace('.', '/')
    for mod in all_modules:
        mod_name = os.path.splitext(mod)[0].replace('/', '.')
        if dep == mod_name or dep in mod or mod_name.endswith(dep):
            return mod
        # C头文件直接匹配文件名
        if os.path.basename(mod) == dep:
            return mod
    return None


def build_dependency_graph(root):
    """构建依赖图"""
    modules = find_source_files(root)
    graph = defaultdict(set)  # {module: set(dependencies)}

    for module, filepath in modules.items():
        ext = os.path.splitext(filepath)[1]
        raw_deps = extract_dependencies(filepath, ext)

        for dep in raw_deps:
            resolved = resolve_dep_to_module(dep, modules, module)
            if resolved and resolved != module:
                graph[module].add(resolved)

    return dict(graph), modules


def detect_cycles(graph):
    """检测循环依赖"""
    cycles = []
    visited = set()
    path = []

    def dfs(node):
        if node in path:
            cycle = path[path.index(node):] + [node]
            cycles.append(cycle)
            return
        if node in visited:
            return
        visited.add(node)
        path.append(node)
        for dep in graph.get(node, []):
            dfs(dep)
        path.pop()

    for node in graph:
        visited.clear()
        path.clear()
        dfs(node)

    return cycles


def generate_dot(graph, modules):
    """生成DOT格式的图"""
    lines = ['digraph dependencies {']
    lines.append('  rankdir=LR;')
    lines.append('  node [shape=box];')

    # 按目录分组
    dir_modules = defaultdict(list)
    for mod in modules:
        d = os.path.dirname(mod) or 'root'
        dir_modules[d].append(mod)

    for d, mods in dir_modules.items():
        lines.append(f'  subgraph cluster_{d.replace("/", "_")} {{')
        lines.append(f'    label="{d}";')
        for m in mods:
            label = os.path.basename(m)
            lines.append(f'    "{m}" [label="{label}"];')
        lines.append('  }')

    # 边
    for src, deps in graph.items():
        for dst in deps:
            lines.append(f'  "{src}" -> "{dst}";')

    lines.append('}')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description='依赖图生成器')
    parser.add_argument('root', help='项目根目录')
    parser.add_argument('--format', '-f', choices=['text', 'dot', 'mermaid'],
                        default='text', help='输出格式')
    parser.add_argument('--cycles', '-c', action='store_true',
                        help='检测循环依赖')
    parser.add_argument('--output', '-o', help='输出文件')
    args = parser.parse_args()

    graph, modules = build_dependency_graph(args.root)

    if args.format == 'text':
        lines = []
        lines.append('=' * 60)
        lines.append('  模块依赖关系图')
        lines.append('=' * 60)
        lines.append(f'  模块总数: {len(modules)}')
        lines.append(f'  依赖边数: {sum(len(d) for d in graph.values())}')
        lines.append('')

        for mod in sorted(graph.keys()):
            deps = sorted(graph[mod])
            if deps:
                lines.append(f'{mod}:')
                for d in deps:
                    lines.append(f'  -> {d}')

        if args.cycles:
            cycles = detect_cycles(graph)
            lines.append('')
            if cycles:
                lines.append(f'⚠ 发现 {len(cycles)} 个循环依赖:')
                for c in cycles:
                    lines.append(f'  {" -> ".join(c)}')
            else:
                lines.append('✓ 无循环依赖')

        report = '\n'.join(lines)
        print(report)

    elif args.format == 'dot':
        dot = generate_dot(graph, modules)
        print(dot)
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as f:
                f.write(dot)
            print(f'DOT文件已保存，可用 Graphviz 渲染: dot -Tpng {args.output} -o graph.png')

    elif args.format == 'mermaid':
        lines = ['graph LR']
        for src, deps in graph.items():
            src_id = src.replace('.', '_').replace('/', '_')
            for dst in deps:
                dst_id = dst.replace('.', '_').replace('/', '_')
                src_label = os.path.basename(src)
                dst_label = os.path.basename(dst)
                lines.append(f'  {src_id}["{src_label}"] --> {dst_id}["{dst_label}"]')
        mermaid = '\n'.join(lines)
        print(mermaid)

    if args.output and args.format != 'dot':
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(report if args.format == 'text' else mermaid)
        print(f'\n报告已保存到: {args.output}')


if __name__ == '__main__':
    main()
