#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
变更日志生成器
==============
功能：
  - 从 git log 自动生成 CHANGELOG.md
  - 遵循 Conventional Commits 规范自动分类
  - 支持自定义分类映射
  - 按版本标签分组（tag-based）
  - 支持过滤和日期范围

依赖：需要 git 命令行工具

用法：
  python changelog_generator.py                       # 从所有提交生成
  python changelog_generator.py --since 2025-01-01    # 只看特定日期后的
  python changelog_generator.py --tags                # 按 git tag 分版本
  python changelog_generator.py -o CHANGELOG.md       # 输出到文件
  python changelog_generator.py --repo /path/to/repo  # 指定仓库路径
"""

import argparse
import re
import subprocess
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="变更日志生成器 - 从 git log 自动生成 CHANGELOG.md",
    )
    parser.add_argument("--repo", "-r", type=str, default=".",
                        help="Git 仓库路径（默认当前目录）")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="输出文件路径（默认打印到终端）")
    parser.add_argument("--since", type=str, default=None,
                        help="起始日期 (YYYY-MM-DD)")
    parser.add_argument("--until", type=str, default=None,
                        help="截止日期 (YYYY-MM-DD)")
    parser.add_argument("--tags", action="store_true",
                        help="按 git tag 分版本显示")
    parser.add_argument("--max-count", type=int, default=500,
                        help="最大提交数（默认 500）")
    parser.add_argument("--no-merges", action="store_true",
                        help="忽略合并提交")
    parser.add_argument("--title", type=str, default="变更日志",
                        help="日志标题")
    parser.add_argument("--lang", type=str, default="zh",
                        choices=["zh", "en"], help="语言（默认中文）")
    return parser.parse_args()


# ============================================================
# Conventional Commits 解析
# ============================================================

# 提交类型 -> 中文显示名
COMMIT_TYPES_ZH = {
    "feat": "✨ 新功能",
    "fix": "🐛 Bug 修复",
    "docs": "📝 文档",
    "style": "💄 代码格式",
    "refactor": "♻️ 重构",
    "perf": "⚡ 性能优化",
    "test": "✅ 测试",
    "build": "📦 构建",
    "ci": "🔧 CI/CD",
    "chore": "🔨 杂项",
    "revert": "⏪ 回滚",
}

COMMIT_TYPES_EN = {
    "feat": "Features",
    "fix": "Bug Fixes",
    "docs": "Documentation",
    "style": "Styles",
    "refactor": "Refactoring",
    "perf": "Performance",
    "test": "Tests",
    "build": "Build",
    "ci": "CI/CD",
    "chore": "Chores",
    "revert": "Reverts",
}

# 简单关键词映射（非 conventional commits 时的回退分类）
KEYWORD_FALLBACK = {
    "feat": ["add", "新增", "添加", "支持", "feature", "implement", "新增"],
    "fix": ["fix", "修复", "bug", "error", "crash", "异常", "错误", "patch"],
    "docs": ["doc", "readme", "文档", "说明", "注释", "comment"],
    "refactor": ["refactor", "重构", "重写", "restructure", "clean"],
    "test": ["test", "测试", "spec", "assert", "pytest"],
    "perf": ["perf", "优化", "performance", "speed", "faster", "加速"],
    "build": ["build", "cmake", "makefile", "编译", "构建", "依赖"],
    "style": ["style", "format", "格式", "lint", "缩进"],
    "ci": ["ci", "github action", "pipeline", "workflow", "deploy"],
    "chore": ["chore", "杂项", "bump", "version", "版本", "清理"],
}

CC_PATTERN = re.compile(
    r"^(?P<type>\w+)"
    r"(?:\((?P<scope>[^)]+)\))?"
    r"(?P<breaking>!)?"
    r":\s*(?P<description>.+)"
)


def parse_commit_message(message):
    """
    解析提交信息，返回 (type, scope, description, is_breaking)
    """
    lines = message.strip().split("\n")
    first_line = lines[0].strip()

    # 尝试 Conventional Commits 格式
    m = CC_PATTERN.match(first_line)
    if m:
        return (
            m.group("type").lower(),
            m.group("scope"),
            m.group("description"),
            bool(m.group("breaking")),
        )

    # 回退：关键词匹配
    lower_msg = first_line.lower()
    for ctype, keywords in KEYWORD_FALLBACK.items():
        for kw in keywords:
            if kw in lower_msg:
                return (ctype, None, first_line, False)

    return ("chore", None, first_line, False)


def run_git(repo_path, args):
    """执行 git 命令"""
    cmd = ["git"] + args
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=30,
            cwd=repo_path,
        )
        if result.returncode != 0:
            return None
        return result.stdout.strip()
    except (subprocess.SubprocessError, FileNotFoundError):
        return None


def get_git_tags(repo_path):
    """获取所有 tag 及其日期"""
    output = run_git(repo_path, ["tag", "--sort=-creatordate", "--format=%(refname:short)|%(creatordate:short)"])
    if not output:
        return []

    tags = []
    for line in output.split("\n"):
        parts = line.strip().split("|", 1)
        if len(parts) == 2:
            tags.append({"name": parts[0], "date": parts[1]})
    return tags


def get_commits(repo_path, since=None, until=None, max_count=500, no_merges=False):
    """获取 git 提交列表"""
    # 使用特殊分隔符便于解析
    sep = "---COMMIT_SEP---"
    format_str = f"%H%n%an%n%ai%n%s%n%b{sep}"

    args = ["log", f"--pretty=format:{format_str}"]
    if since:
        args.append(f"--since={since}")
    if until:
        args.append(f"--until={until}")
    if max_count:
        args.append(f"--max-count={max_count}")
    if no_merges:
        args.append("--no-merges")

    output = run_git(repo_path, args)
    if not output:
        return []

    commits = []
    for entry in output.split(sep):
        entry = entry.strip()
        if not entry:
            continue
        lines = entry.split("\n")
        if len(lines) >= 4:
            body = "\n".join(lines[4:]) if len(lines) > 4 else ""
            commits.append({
                "hash": lines[0][:8],
                "author": lines[1],
                "date": lines[2][:10],
                "subject": lines[3],
                "body": body,
            })

    return commits


def get_commits_between_tags(repo_path, tag1, tag2=None, no_merges=False):
    """获取两个 tag 之间的提交"""
    range_str = f"{tag1}..{tag2}" if tag2 else tag1
    sep = "---COMMIT_SEP---"
    format_str = f"%H%n%an%n%ai%n%s%n%b{sep}"

    args = ["log", f"--pretty=format:{format_str}", range_str]
    if no_merges:
        args.append("--no-merges")

    output = run_git(repo_path, args)
    if not output:
        return []

    commits = []
    for entry in output.split(sep):
        entry = entry.strip()
        if not entry:
            continue
        lines = entry.split("\n")
        if len(lines) >= 4:
            commits.append({
                "hash": lines[0][:8],
                "author": lines[1],
                "date": lines[2][:10],
                "subject": lines[3],
            })

    return commits


def group_commits_by_type(commits, lang="zh"):
    """按提交类型分组"""
    type_names = COMMIT_TYPES_ZH if lang == "zh" else COMMIT_TYPES_EN
    groups = defaultdict(list)
    breaking = []

    for c in commits:
        ctype, scope, desc, is_breaking = parse_commit_message(c["subject"])

        if is_breaking:
            breaking.append(c)

        label = type_names.get(ctype, f"📌 {ctype}" if lang == "zh" else ctype)
        scope_str = f"**{scope}**: " if scope else ""
        entry = f"- {scope_str}{desc} (`{c['hash']}`)"
        groups[label].append(entry)

    return groups, breaking


def generate_changelog_text(commits, title, lang="zh"):
    """生成纯文本 CHANGELOG（不分版本）"""
    groups, breaking = group_commits_by_type(commits, lang)

    lines = [f"# {title}", ""]
    lines.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"共 {len(commits)} 个提交")
    lines.append("")

    if breaking:
        lines.append("## ⚠️ Breaking Changes" if lang == "en" else "## ⚠️ 破坏性变更")
        for c in breaking:
            _, scope, desc, _ = parse_commit_message(c["subject"])
            scope_str = f"**{scope}**: " if scope else ""
            lines.append(f"- {scope_str}{desc} (`{c['hash']}`)")
        lines.append("")

    # 按重要性排序
    priority_order = ["✨ 新功能", "🐛 Bug 修复", "Features", "Bug Fixes"]
    sorted_groups = sorted(
        groups.items(),
        key=lambda x: priority_order.index(x[0]) if x[0] in priority_order else 99,
    )

    for group_name, entries in sorted_groups:
        lines.append(f"## {group_name}")
        lines.extend(entries)
        lines.append("")

    return "\n".join(lines)


def generate_changelog_by_tags(repo_path, commits, title, lang, no_merges):
    """按 git tag 分版本生成 CHANGELOG"""
    tags = get_git_tags(repo_path)

    lines = [f"# {title}", ""]
    lines.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("")

    if not tags:
        lines.append("（未找到 git tag，回退为全部提交列表）\n")
        lines.append(generate_changelog_text(commits, "", lang))
        return "\n".join(lines)

    type_names = COMMIT_TYPES_ZH if lang == "zh" else COMMIT_TYPES_EN

    for i, tag in enumerate(tags):
        tag_name = tag["name"]
        tag_date = tag["date"]

        # 获取该 tag 到下一个 tag 之间的提交
        next_tag = tags[i + 1]["name"] if i + 1 < len(tags) else None
        tag_commits = get_commits_between_tags(repo_path, tag_name, next_tag, no_merges)

        if not tag_commits:
            continue

        lines.append(f"## [{tag_name}] - {tag_date}")
        lines.append("")

        groups, breaking = group_commits_by_type(tag_commits, lang)

        if breaking:
            lines.append("### ⚠️ 破坏性变更" if lang == "zh" else "### Breaking Changes")
            for c in breaking:
                _, scope, desc, _ = parse_commit_message(c["subject"])
                scope_str = f"**{scope}**: " if scope else ""
                lines.append(f"- {scope_str}{desc} (`{c['hash']}`)")
            lines.append("")

        for group_name, entries in groups.items():
            lines.append(f"### {group_name}")
            lines.extend(entries)
            lines.append("")

    # 未包含在 tag 中的提交
    if tags:
        untagged = get_commits_between_tags(repo_path, tags[0]["name"], None, no_merges)
        if untagged:
            lines.append("## [未发布]" if lang == "zh" else "## [Unreleased]")
            lines.append("")
            groups, _ = group_commits_by_type(untagged, lang)
            for group_name, entries in groups.items():
                lines.append(f"### {group_name}")
                lines.extend(entries)
                lines.append("")

    return "\n".join(lines)


def main():
    """主入口"""
    args = parse_args()

    repo_path = os.path.abspath(args.repo)
    if not os.path.isdir(os.path.join(repo_path, ".git")):
        print(f"[错误] {repo_path} 不是 Git 仓库")
        sys.exit(1)

    print(f"[信息] 分析仓库: {repo_path}")

    # 获取提交
    commits = get_commits(
        repo_path,
        since=args.since,
        until=args.until,
        max_count=args.max_count,
        no_merges=args.no_merges,
    )

    if not commits:
        print("[信息] 没有找到提交记录")
        sys.exit(0)

    print(f"[信息] 共 {len(commits)} 个提交")

    # 生成 CHANGELOG
    if args.tags:
        changelog = generate_changelog_by_tags(
            repo_path, commits, args.title, args.lang, args.no_merges,
        )
    else:
        changelog = generate_changelog_text(commits, args.title, args.lang)

    # 输出
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(changelog)
        print(f"[成功] CHANGELOG 已生成: {output_path.resolve()}")
    else:
        print("\n" + changelog)


if __name__ == "__main__":
    main()
