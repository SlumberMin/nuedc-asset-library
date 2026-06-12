#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文档自动更新器 - 从代码注释/Doxygen标记自动更新README和API文档
用法: python doc_updater.py --source src/ --readme README.md --api-doc API.md
"""

import argparse
import os
import re
from datetime import datetime


def scan_source_files(source_dir: str, extensions=(".c", ".h")) -> list:
    """扫描目录下所有源文件"""
    files = []
    for root, _, filenames in os.walk(source_dir):
        for fname in filenames:
            if any(fname.endswith(ext) for ext in extensions):
                files.append(os.path.join(root, fname))
    return sorted(files)


def extract_comments(filepath: str) -> list:
    """
    从源文件提取Doxygen风格注释
    返回: [(type, name, brief, params, returns, note)]
    """
    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        content = f.read()

    results = []
    filename = os.path.basename(filepath)

    # 匹配 /** ... */ 块注释
    comment_pattern = re.compile(
        r'/\*\*(.*?)\*/\s*(?:'
        r'(?:static\s+)?(?:inline\s+)?(?:const\s+)?'
        r'(\w[\w\s\*]+?)\s+(\*?\w+)\s*\(([^)]*)\)\s*[{;]'
        r')',
        re.DOTALL
    )

    for m in comment_pattern.finditer(content):
        comment_block = m.group(1)
        # func_name = m.group(3)

        # 提取 @brief
        brief_match = re.search(r'@brief\s+(.+?)(?:\n|$)', comment_block)
        brief = brief_match.group(1).strip() if brief_match else ""

        # 提取 @param
        params = re.findall(r'@param\s+(?:\[(?:in|out|in,out)\]\s+)?(\w+)\s+(.+?)(?:\n|$)', comment_block)

        # 提取 @return / @returns
        ret_match = re.search(r'@returns?\s+(.+?)(?:\n|$)', comment_block)
        returns = ret_match.group(1).strip() if ret_match else ""

        # 提取 @note
        note_match = re.search(r'@note\s+(.+?)(?:\n|$)', comment_block)
        note = note_match.group(1).strip() if note_match else ""

        # 提取 @file
        file_match = re.search(r'@file\s+(.+?)(?:\n|$)', comment_block)
        file_desc = file_match.group(1).strip() if file_match else ""

        func_name = m.group(3)
        results.append({
            "file": filename,
            "type": "function",
            "name": func_name,
            "brief": brief,
            "params": params,
            "returns": returns,
            "note": note,
        })

    # 提取 #define 描述
    define_pattern = re.compile(r'/\*\*\s*(.*?)\s*\*/\s*#define\s+(\w+)', re.DOTALL)
    for m in define_pattern.finditer(content):
        desc = m.group(1).replace("\n", " ").strip()
        name = m.group(2)
        results.append({
            "file": filename,
            "type": "macro",
            "name": name,
            "brief": desc,
            "params": [],
            "returns": "",
            "note": "",
        })

    return results


def generate_api_doc(all_comments: list, title: str = "API参考文档") -> str:
    """生成API文档Markdown"""
    lines = [
        f"# {title}",
        "",
        f"> 自动生成于 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"> 由 doc_updater.py 生成，请勿手动编辑",
        "",
    ]

    # 按文件分组
    by_file = {}
    for c in all_comments:
        by_file.setdefault(c["file"], []).append(c)

    for fname, items in sorted(by_file.items()):
        lines.append(f"## 文件: `{fname}`")
        lines.append("")

        for item in items:
            if item["type"] == "function":
                # 函数签名
                param_str = ", ".join(f"{p[0]}" for p in item["params"]) if item["params"] else "void"
                lines.append(f"### `{item['name']}({param_str})`")
                lines.append("")

                if item["brief"]:
                    lines.append(f"**描述**: {item['brief']}")
                    lines.append("")

                if item["params"]:
                    lines.append("**参数**:")
                    lines.append("")
                    lines.append("| 参数 | 说明 |")
                    lines.append("|------|------|")
                    for pname, pdesc in item["params"]:
                        lines.append(f"| `{pname}` | {pdesc} |")
                    lines.append("")

                if item["returns"]:
                    lines.append(f"**返回值**: {item['returns']}")
                    lines.append("")

                if item["note"]:
                    lines.append(f"> **注意**: {item['note']}")
                    lines.append("")

            elif item["type"] == "macro":
                lines.append(f"### `{item['name']}`")
                if item["brief"]:
                    lines.append(f"> {item['brief']}")
                lines.append("")

        lines.append("---")
        lines.append("")

    return "\n".join(lines)


def update_readme(readme_path: str, api_doc_path: str, module_list: list) -> str:
    """更新README文件中的文档引用部分"""
    if os.path.isfile(readme_path):
        with open(readme_path, "r", encoding="utf-8") as f:
            readme = f.read()
    else:
        readme = f"# nuedc-asset-library\n\n"

    # 查找或创建 API 文档部分
    marker_start = "<!-- API_DOC_START -->"
    marker_end = "<!-- API_DOC_END -->"

    api_section = f"""{marker_start}
## API 文档

> 自动生成于 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}，请勿手动编辑

详细API参考: [{os.path.basename(api_doc_path)}]({api_doc_path})

### 模块列表

| 模块 | 文件 | 说明 |
|------|------|------|
"""
    for mod in module_list:
        api_section += f"| {mod['name']} | `{mod['file']}` | {mod['brief']} |\n"

    api_section += f"\n{marker_end}"

    # 替换或追加
    if marker_start in readme and marker_end in readme:
        pattern = re.escape(marker_start) + r'.*?' + re.escape(marker_end)
        readme = re.sub(pattern, api_section, readme, flags=re.DOTALL)
    else:
        readme = readme.rstrip() + "\n\n" + api_section + "\n"

    return readme


def main():
    parser = argparse.ArgumentParser(description="文档自动更新器 - 从代码注释生成API文档")
    parser.add_argument("--source", required=True, help="源代码目录")
    parser.add_argument("--readme", default="README.md", help="README文件路径")
    parser.add_argument("--api-doc", default="API.md", help="输出API文档路径")
    parser.add_argument("--dry-run", action="store_true", help="预览模式，不写入文件")
    args = parser.parse_args()

    if not os.path.isdir(args.source):
        print(f"[✗] 目录不存在: {args.source}")
        return

    # 扫描源文件
    files = scan_source_files(args.source)
    print(f"[i] 扫描到 {len(files)} 个源文件")

    # 提取注释
    all_comments = []
    for f in files:
        comments = extract_comments(f)
        if comments:
            print(f"    {os.path.basename(f)}: {len(comments)} 个文档条目")
            all_comments.extend(comments)

    print(f"[i] 共提取 {len(all_comments)} 个文档条目")

    if not all_comments:
        print("[!] 未找到Doxygen注释，请确保代码中有 /** ... */ 注释")
        return

    # 生成API文档
    api_content = generate_api_doc(all_comments)

    # 模块列表
    seen_files = {}
    for c in all_comments:
        if c["file"] not in seen_files:
            seen_files[c["file"]] = {"name": os.path.splitext(c["file"])[0], "file": c["file"], "brief": c["brief"]}

    # 更新README
    readme_content = update_readme(args.readme, args.api_doc, list(seen_files.values()))

    if args.dry_run:
        print("\n===== API文档预览 =====")
        print(api_content[:2000])
        print("..." if len(api_content) > 2000 else "")
        print("\n[i] 干运行模式，未写入文件")
        return

    # 写入文件
    os.makedirs(os.path.dirname(os.path.abspath(args.api_doc)) or ".", exist_ok=True)
    with open(args.api_doc, "w", encoding="utf-8", newline="\n") as f:
        f.write(api_content)

    with open(args.readme, "w", encoding="utf-8", newline="\n") as f:
        f.write(readme_content)

    print(f"[✓] API文档已更新: {args.api_doc}")
    print(f"[✓] README已更新: {args.readme}")


if __name__ == "__main__":
    main()
