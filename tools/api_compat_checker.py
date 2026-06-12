#!/usr/bin/env python3
"""API兼容性检查器 - 检查三平台(STM32/ESP32/MSP432)API一致性"""

import argparse
import os
import re
from collections import defaultdict


# 支持的平台
PLATFORMS = ['stm32', 'esp32', 'msp432']


def find_platform_dirs(root):
    """查找各平台目录"""
    dirs = {}
    for p in PLATFORMS:
        for dirpath, dirnames, _ in os.walk(root):
            if p in dirpath.lower():
                dirs[p] = dirpath
                break
        # 也检查直接子目录
        candidate = os.path.join(root, p)
        if os.path.isdir(candidate):
            dirs[p] = candidate
    return dirs


def extract_api_signatures(filepath):
    """提取文件中的API函数签名"""
    apis = {}
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        # 匹配统一接口函数声明
        for m in re.finditer(
            r'(?:void|int|uint\w+|float|double|bool|char|status_t|error_t)\s+'
            r'(\w+)\s*\(([^)]*)\)',
            content
        ):
            name = m.group(1)
            params = m.group(2).strip()
            apis[name] = params
    except Exception:
        pass
    return apis


def extract_header_apis(root):
    """从头文件提取统一API定义"""
    unified = {}
    for dirpath, _, filenames in os.walk(root):
        for f in filenames:
            if f.endswith('.h') and 'unified' in f.lower():
                fp = os.path.join(dirpath, f)
                unified.update(extract_api_signatures(fp))
    return unified


def check_compatibility(root):
    """检查三平台API兼容性"""
    # 提取统一API
    unified_apis = extract_header_apis(root)

    # 提取各平台实现
    platform_apis = {}
    for p in PLATFORMS:
        apis = {}
        for dirpath, _, filenames in os.walk(root):
            if p in dirpath.lower():
                for f in filenames:
                    if f.endswith(('.c', '.h')):
                        fp = os.path.join(dirpath, f)
                        apis.update(extract_api_signatures(fp))
        platform_apis[p] = apis

    # 比较
    results = {
        'unified_apis': unified_apis,
        'platform_apis': platform_apis,
        'missing': {},      # 平台缺失的API
        'mismatch': {},     # 签名不匹配
        'extra': {},        # 平台多出的API
    }

    if not unified_apis:
        # 没有统一头文件，比较平台间的交集
        all_names = set()
        for apis in platform_apis.values():
            all_names |= set(apis.keys())
        common = set.intersection(*(set(a.keys()) for a in platform_apis.values() if a))
        results['common'] = common
        results['all'] = all_names
        return results

    for p in PLATFORMS:
        apis = platform_apis.get(p, {})
        # 检查缺失
        missing = set(unified_apis.keys()) - set(apis.keys())
        if missing:
            results['missing'][p] = missing
        # 检查签名不匹配
        mismatches = {}
        for name in set(unified_apis.keys()) & set(apis.keys()):
            if unified_apis[name] != apis[name]:
                mismatches[name] = (unified_apis[name], apis[name])
        if mismatches:
            results['mismatch'][p] = mismatches
        # 检查多余
        extra = set(apis.keys()) - set(unified_apis.keys())
        if extra:
            results['extra'][p] = extra

    return results


def main():
    parser = argparse.ArgumentParser(description='API兼容性检查器')
    parser.add_argument('root', help='项目根目录')
    parser.add_argument('--output', '-o', help='输出报告文件')
    parser.add_argument('--platforms', '-p', nargs='+', default=PLATFORMS,
                        help='要检查的平台')
    args = parser.parse_args()

    global PLATFORMS
    PLATFORMS = args.platforms

    result = check_compatibility(args.root)

    lines = []
    lines.append('=' * 60)
    lines.append('  API兼容性检查报告')
    lines.append('=' * 60)

    if result.get('unified_apis'):
        lines.append(f'\n统一API数量: {len(result["unified_apis"])}')

        for p in PLATFORMS:
            lines.append(f'\n--- {p.upper()} ---')
            missing = result['missing'].get(p, set())
            if missing:
                lines.append(f'  缺失API ({len(missing)}):')
                for name in sorted(missing):
                    lines.append(f'    - {name}')
            mismatch = result['mismatch'].get(p, {})
            if mismatch:
                lines.append(f'  签名不匹配 ({len(mismatch)}):')
                for name, (expected, actual) in sorted(mismatch.items()):
                    lines.append(f'    {name}: 期望({expected}) 实际({actual})')
            extra = result['extra'].get(p, set())
            if extra:
                lines.append(f'  额外API ({len(extra)}):')
                for name in sorted(extra):
                    lines.append(f'    + {name}')
            if not missing and not mismatch and not extra:
                lines.append('  ✓ 完全兼容')
    else:
        lines.append('\n未找到统一API头文件，进行平台间交叉比较')
        lines.append(f'\n平台公共API: {len(result.get("common", []))}')
        lines.append(f'所有API总数: {len(result.get("all", []))}')

    report = '\n'.join(lines)
    print(report)

    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(report)
        print(f'\n报告已保存到: {args.output}')


if __name__ == '__main__':
    main()
