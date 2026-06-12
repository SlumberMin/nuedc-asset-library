#!/usr/bin/env python3
"""发布说明生成器 - 从git log自动生成结构化发布说明"""

import argparse
import os
import re
import subprocess
from collections import defaultdict


# 提交类型分类（Conventional Commits风格）
COMMIT_TYPES = {
    'feat': '✨ 新功能',
    'fix': '🐛 修复',
    'docs': '📝 文档',
    'style': '💄 格式',
    'refactor': '♻️ 重构',
    'perf': '⚡ 性能',
    'test': '✅ 测试',
    'build': '📦 构建',
    'ci': '🔧 CI',
    'chore': '🔨 杂项',
    'fixbug': '🐛 修复',
    'add': '✨ 新功能',
    'update': '♻️ 更新',
    'remove': '🗑️ 移除',
}


def run_git(args_list, cwd=None):
    """执行git命令"""
    try:
        result = subprocess.run(
            ['git'] + args_list,
            capture_output=True, text=True,
            cwd=cwd or os.getcwd(),
            encoding='utf-8', errors='replace'
        )
        return result.stdout.strip() if result.returncode == 0 else ''
    except Exception:
        return ''


def parse_commit(line):
    """解析单行git log输出"""
    # 格式: hash|author|date|message
    parts = line.split('|', 3)
    if len(parts) < 4:
        return None
    return {
        'hash': parts[0].strip(),
        'author': parts[1].strip(),
        'date': parts[2].strip(),
        'message': parts[3].strip(),
    }


def classify_commit(message):
    """根据提交消息分类"""
    msg_lower = message.lower()
    # 检查前缀类型
    for prefix, category in COMMIT_TYPES.items():
        if msg_lower.startswith(prefix + ':') or msg_lower.startswith(prefix + '('):
            return category
    # 关键词匹配
    if any(w in msg_lower for w in ['fix', 'bug', '修复', '修正']):
        return '🐛 修复'
    if any(w in msg_lower for w in ['feat', 'add', '新增', '添加', '功能']):
        return '✨ 新功能'
    if any(w in msg_lower for w in ['doc', 'readme', '文档']):
        return '📝 文档'
    if any(w in msg_lower for w in ['test', '测试']):
        return '✅ 测试'
    if any(w in msg_lower for w in ['refactor', '重构', '优化']):
        return '♻️ 重构'
    return '🔨 其他'


def get_commits(repo_path, since=None, until=None, tag=None):
    """获取git提交记录"""
    args = ['log', '--format=%h|%an|%ad|%s', '--date=short']
    if tag:
        args.append(tag)
    if since:
        args.append(f'--since={since}')
    if until:
        args.append(f'--until={until}')
    output = run_git(args, cwd=repo_path)
    if not output:
        return []
    commits = []
    for line in output.split('\n'):
        c = parse_commit(line)
        if c:
            commits.append(c)
    return commits


def get_tags(repo_path):
    """获取git标签"""
    output = run_git(['tag', '--sort=-version:refname'], cwd=repo_path)
    return output.split('\n') if output else []


def get_diff_stats(repo_path, from_ref, to_ref='HEAD'):
    """获取两个ref之间的变更统计"""
    output = run_git(['diff', '--stat', from_ref, to_ref], cwd=repo_path)
    return output


def generate_notes(commits, version=None, repo_path=None):
    """生成发布说明"""
    # 按类型分组
    grouped = defaultdict(list)
    for c in commits:
        category = classify_commit(c['message'])
        grouped[category].append(c)

    lines = []
    lines.append('# 📋 发布说明')
    if version:
        lines.append(f'## 版本: {version}')
    lines.append('')

    # 统计摘要
    total = len(commits)
    authors = set(c['author'] for c in commits)
    lines.append(f'共 {total} 个提交，来自 {len(authors)} 位贡献者')
    lines.append('')

    # 按类别列出
    # 排序优先级
    priority = ['✨ 新功能', '🐛 修复', '⚡ 性能', '♻️ 重构', '📝 文档',
                '✅ 测试', '📦 构建', '🔧 CI', '🔨 杂项', '🔨 其他']
    for cat in priority:
        items = grouped.get(cat, [])
        if not items:
            continue
        lines.append(f'### {cat} ({len(items)})')
        for c in items:
            hash_short = c['hash'][:7]
            msg = c['message']
            # 去除conventional commit前缀
            msg = re.sub(r'^\w+(\([^)]*\))?!?:\s*', '', msg)
            lines.append(f'- {msg} (`{hash_short}`)')
        lines.append('')

    # 贡献者
    lines.append('### 👥 贡献者')
    for author in sorted(authors):
        count = sum(1 for c in commits if c['author'] == author)
        lines.append(f'- {author} ({count} 次提交)')
    lines.append('')

    # 变更统计（如果有repo_path）
    if repo_path and len(commits) >= 2:
        first = commits[-1]['hash']
        last = commits[0]['hash']
        stats = get_diff_stats(repo_path, first, last)
        if stats:
            lines.append('### 📊 变更统计')
            lines.append('```')
            lines.append(stats)
            lines.append('```')

    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description='发布说明生成器')
    parser.add_argument('repo', nargs='?', default='.', help='Git仓库路径')
    parser.add_argument('--version', '-v', help='版本号')
    parser.add_argument('--since', '-s', help='起始日期 (YYYY-MM-DD)')
    parser.add_argument('--until', '-u', help='截止日期 (YYYY-MM-DD)')
    parser.add_argument('--from-tag', '-f', help='从指定标签开始')
    parser.add_argument('--to-tag', '-t', help='到指定标签结束')
    parser.add_argument('--output', '-o', help='输出文件')
    args = parser.parse_args()

    repo_path = os.path.abspath(args.repo)

    # 确定版本号
    version = args.version
    if not version:
        tags = get_tags(repo_path)
        version = tags[0] if tags else None

    # 获取提交
    since = args.since
    if args.from_tag and not since:
        # 从标签的日期开始
        date_output = run_git(['log', '-1', '--format=%ad', '--date=short', args.from_tag],
                              cwd=repo_path)
        if date_output:
            since = date_output

    commits = get_commits(repo_path, since=since, until=args.until)
    if not commits:
        print('未找到提交记录')
        return

    notes = generate_notes(commits, version, repo_path)
    print(notes)

    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(notes)
        print(f'\n发布说明已保存到: {args.output}')


if __name__ == '__main__':
    main()
