#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试覆盖率报告生成器 - 电赛资产库工具
===================================
功能：分析测试文件对生产代码的覆盖情况，生成覆盖率报告
用法：
  python test_coverage_report.py                         # 分析全部测试
  python test_coverage_report.py --tests-dir ./tests     # 指定测试目录
  python test_coverage_report.py --output report.md      # 输出到文件
  python test_coverage_report.py --detailed               # 详细分析

分析维度：
  1. 文件覆盖：哪些驱动有对应的测试文件
  2. 函数覆盖：哪些函数被测试调用
  3. 错误模式覆盖：错误经验库中的模式是否被测试覆盖
  4. 边界条件覆盖：是否测试了边界值和异常输入

背景：错误经验库 #9 记录了测试不import生产代码的问题
      本工具帮助评估测试的真实覆盖程度

依赖：无额外依赖（纯Python实现）
"""

import argparse
import os
import re
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path


# ============================================================
# 错误经验库模式（用于检查测试是否覆盖了已知错误模式）
# ============================================================

KNOWN_ERROR_PATTERNS = [
    {
        'id': 1,
        'name': '除零风险',
        'test_keywords': ['divid', 'zero', 'divisor', '除零', 'small_value', 'edge_case'],
        'code_keywords': ['/', 'divide', 'divisor'],
        'description': '测试应验证除数为零或极小值的情况'
    },
    {
        'id': 5,
        'name': 'ISR共享变量volatile',
        'test_keywords': ['volatile', 'interrupt', 'isr', 'race', '并发'],
        'code_keywords': ['volatile', 'interrupt', 'ISR'],
        'description': '测试应验证中断安全'
    },
    {
        'id': 6,
        'name': 'I2C忙等待超时',
        'test_keywords': ['timeout', 'i2c', 'busy', '超时', 'stuck'],
        'code_keywords': ['while', 'i2c', 'busy'],
        'description': '测试应验证I2C超时机制'
    },
    {
        'id': 7,
        'name': '数组/缓冲区溢出',
        'test_keywords': ['overflow', 'buffer', 'boundary', 'max_len', '溢出', '边界'],
        'code_keywords': ['buffer', 'array', 'index'],
        'description': '测试应验证缓冲区边界'
    },
    {
        'id': 11,
        'name': '角度/弧度单位混淆',
        'test_keywords': ['angle', 'radian', 'degree', 'convert', '单位', '弧度'],
        'code_keywords': ['angle', 'degree', 'radian', 'PI'],
        'description': '测试应验证角度/弧度转换正确性'
    },
    {
        'id': 12,
        'name': '矩阵维度错误',
        'test_keywords': ['matrix', 'dimension', 'shape', 'transpose', '矩阵'],
        'code_keywords': ['matrix', 'mat_', 'multiply'],
        'description': '测试应验证矩阵维度匹配'
    },
    {
        'id': 16,
        'name': '参数未校验导致除零',
        'test_keywords': ['param', 'valid', 'check', 'protect', 'min_val', '参数校验'],
        'code_keywords': ['max_vel', 'max_accel', 'max_jerk'],
        'description': '测试应验证参数边界保护'
    },
    {
        'id': 18,
        'name': '控制仿真反馈符号',
        'test_keywords': ['feedback', 'sign', 'negative', 'positive', 'diverge', '收敛'],
        'code_keywords': ['val +=', 'val -=', 'output'],
        'description': '测试应验证控制回路反馈方向正确'
    },
    {
        'id': 20,
        'name': '公式实现错误',
        'test_keywords': ['formula', 'cohen', 'coon', 'tuning', '公式', '整定'],
        'code_keywords': ['Kp', 'Ti', 'Td', 'tuning'],
        'description': '测试应验证算法公式计算结果的正确性'
    },
]


# ============================================================
# 测试文件分析器
# ============================================================

class TestAnalyzer:
    """
    测试覆盖率分析器
    
    分析测试文件与生产代码之间的对应关系
    """

    def __init__(self):
        """初始化分析器"""
        
        # 测试函数识别
        self.re_test_func = re.compile(
            r'def\s+(test_\w+)\s*\(',
            re.MULTILINE
        )
        
        # 测试类识别
        self.re_test_class = re.compile(
            r'class\s+(Test\w+)',
            re.MULTILINE
        )
        
        # import语句分析
        self.re_import = re.compile(
            r'(?:from\s+([\w.]+)\s+)?import\s+([\w,\s]+)',
            re.MULTILINE
        )
        
        # assert语句
        self.re_assert = re.compile(
            r'(?:self\.)?assert\w*\s*\(',
            re.MULTILINE
        )
        
        # 测试对函数的调用
        self.re_func_call = re.compile(
            r'(\w+)\s*\(',
            re.MULTILINE
        )

    def find_test_files(self, tests_dir):
        """查找所有测试文件"""
        test_files = []
        for root, dirs, files in os.walk(tests_dir):
            dirs[:] = [d for d in dirs if not d.startswith('.')]
            for f in files:
                if f.startswith('test_') and f.endswith('.py'):
                    test_files.append(os.path.join(root, f))
        return sorted(test_files)

    def find_source_files(self, source_dir):
        """查找所有生产代码文件"""
        source_files = []
        for root, dirs, files in os.walk(source_dir):
            dirs[:] = [d for d in dirs if not d.startswith('.') 
                       and d != 'tests' and d != 'tools']
            for f in files:
                if f.endswith(('.c', '.py')) and not f.startswith('test_'):
                    source_files.append(os.path.join(root, f))
        return sorted(source_files)

    def analyze_test_file(self, filepath):
        """
        分析单个测试文件
        
        返回:
            dict: {
                'filename': str,
                'test_functions': list[str],
                'test_classes': list[str],
                'imports': list[str],
                'assert_count': int,
                'functions_called': set[str],
                'covers_patterns': list[int],  # 覆盖的错误模式ID
                'lines': int,
            }
        """
        result = {
            'filename': os.path.basename(filepath),
            'filepath': str(filepath),
            'test_functions': [],
            'test_classes': [],
            'imports': [],
            'assert_count': 0,
            'functions_called': set(),
            'covers_patterns': [],
            'lines': 0,
            'uses_wrappers': False,  # 是否使用了wrappers.py
            'self_contained': False,  # 是否自行重写了算法（不import生产代码）
        }
        
        try:
            content = None
            for encoding in ['utf-8', 'gbk', 'latin-1']:
                try:
                    with open(filepath, 'r', encoding=encoding) as f:
                        content = f.read()
                    break
                except UnicodeDecodeError:
                    continue
            
            if content is None:
                return result
            
            result['lines'] = len(content.split('\n'))
            
            # 提取测试函数
            for m in self.re_test_func.finditer(content):
                result['test_functions'].append(m.group(1))
            
            # 提取测试类
            for m in self.re_test_class.finditer(content):
                result['test_classes'].append(m.group(1))
            
            # 分析import
            for m in self.re_import.finditer(content):
                module = m.group(1) or ''
                names = [n.strip() for n in m.group(2).split(',')]
                result['imports'].extend(names)
                
                if 'wrappers' in module:
                    result['uses_wrappers'] = True
            
            # 统计assert
            result['assert_count'] = len(self.re_assert.findall(content))
            
            # 提取调用的函数名
            for m in self.re_func_call.finditer(content):
                name = m.group(1)
                # 过滤掉Python内置和常见库函数
                if name not in ('def', 'class', 'if', 'for', 'while', 'return',
                               'print', 'len', 'range', 'int', 'float', 'str',
                               'list', 'dict', 'set', 'tuple', 'type', 'isinstance',
                               'True', 'False', 'None', 'self', 'super', '__init__',
                               'assert', 'abs', 'max', 'min', 'round', 'enumerate',
                               'zip', 'map', 'filter', 'sorted', 'any', 'all',
                               'open', 'format', 'hasattr', 'getattr', 'setattr'):
                    result['functions_called'].add(name)
            
            # 检查是否覆盖已知错误模式
            content_lower = content.lower()
            for pattern in KNOWN_ERROR_PATTERNS:
                for keyword in pattern['test_keywords']:
                    if keyword.lower() in content_lower:
                        result['covers_patterns'].append(pattern['id'])
                        break
            
            # 检查是否自行重写了算法（不import生产代码的标志）
            # 如果测试文件中定义了与算法相关的类/函数，则可能是自行重写
            algo_classes = re.findall(
                r'class\s+(\w*(?:PID|Controller|Filter|Observer|SMC|ADRC|Kalman)\w*)',
                content, re.IGNORECASE
            )
            algo_defs = re.findall(
                r'def\s+(\w*(?:compute|update|calculate|predict|correct|init)\w*)',
                content, re.IGNORECASE
            )
            
            if algo_classes and not result['uses_wrappers']:
                # 检查是否有import生产代码的路径
                has_prod_import = any(
                    imp for imp in result['imports']
                    if any(kw in imp.lower() for kw in ['drivers', 'lib', 'algorithm', 'control'])
                )
                if not has_prod_import:
                    result['self_contained'] = True
            
            result['functions_called'] = list(result['functions_called'])
            return result
            
        except Exception as e:
            print(f"  [警告] 分析 {filepath} 失败: {e}", file=sys.stderr)
            return result

    def find_untested_sources(self, test_analyses, source_files):
        """
        查找没有对应测试的源文件
        
        参数:
            test_analyses: 测试文件分析结果
            source_files: 源文件列表
        """
        # 从测试文件名推断被测试的模块
        tested_modules = set()
        for ta in test_analyses:
            # test_pid.py -> pid
            name = ta['filename']
            if name.startswith('test_'):
                module = name[5:].replace('.py', '').replace('.c', '')
                tested_modules.add(module)
            
            # 也从import中提取
            for imp in ta.get('imports', []):
                parts = imp.split('.')
                if parts:
                    tested_modules.add(parts[-1])
        
        # 查找未覆盖的源文件
        untested = []
        for sf in source_files:
            basename = os.path.basename(sf)
            module = os.path.splitext(basename)[0]
            
            if module not in tested_modules:
                untested.append(sf)
        
        return untested


# ============================================================
# 报告生成器
# ============================================================

def generate_report(test_analyses, untested_sources, source_files, detailed=False):
    """
    生成测试覆盖率报告
    """
    lines = []
    
    lines.append('# 测试覆盖率报告')
    lines.append('')
    lines.append(f'> 生成时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
    lines.append(f'> 测试文件数: {len(test_analyses)}')
    lines.append(f'> 源文件数: {len(source_files)}')
    lines.append('')
    
    # 总体覆盖率
    total_tests = sum(len(ta['test_functions']) for ta in test_analyses)
    total_asserts = sum(ta['assert_count'] for ta in test_analyses)
    file_coverage = (len(source_files) - len(untested_sources)) / max(len(source_files), 1) * 100
    
    lines.append('## 总体统计')
    lines.append('')
    lines.append('| 指标 | 数值 |')
    lines.append('|------|------|')
    lines.append(f'| 测试文件数 | {len(test_analyses)} |')
    lines.append(f'| 测试函数总数 | {total_tests} |')
    lines.append(f'| 断言总数 | {total_asserts} |')
    lines.append(f'| 每测试平均断言 | {total_asserts / max(total_tests, 1):.1f} |')
    lines.append(f'| 文件覆盖率 | {file_coverage:.1f}% ({len(source_files) - len(untested_sources)}/{len(source_files)}) |')
    lines.append('')
    
    # 覆盖率可视化
    covered = len(source_files) - len(untested_sources)
    bar_len = 30
    filled = int(covered / max(len(source_files), 1) * bar_len)
    bar = '█' * filled + '░' * (bar_len - filled)
    lines.append(f'```')
    lines.append(f'文件覆盖率: [{bar}] {file_coverage:.1f}%')
    lines.append(f'```')
    lines.append('')
    
    # 各测试文件详情
    lines.append('## 测试文件详情')
    lines.append('')
    lines.append('| 测试文件 | 测试数 | 断言数 | 覆盖模式 | 状态 |')
    lines.append('|----------|--------|--------|----------|------|')
    
    for ta in sorted(test_analyses, key=lambda x: len(x['test_functions']), reverse=True):
        n_tests = len(ta['test_functions'])
        n_asserts = ta['assert_count']
        n_patterns = len(ta['covers_patterns'])
        
        # 状态标记
        if ta['self_contained']:
            status = '⚠️ 自包含'
        elif ta['uses_wrappers']:
            status = '✅ wrappers'
        else:
            status = '📋'
        
        lines.append(
            f"| {ta['filename']} | {n_tests} | {n_asserts} | {n_patterns} | {status} |"
        )
    lines.append('')
    
    # 错误模式覆盖情况
    lines.append('## 错误模式覆盖情况')
    lines.append('')
    lines.append('基于错误经验库检查测试是否覆盖了已知错误模式：')
    lines.append('')
    
    all_covered_patterns = set()
    for ta in test_analyses:
        all_covered_patterns.update(ta['covers_patterns'])
    
    lines.append('| ID | 错误模式 | 是否有测试覆盖 | 说明 |')
    lines.append('|----|---------|--------------| ------|')
    
    for pattern in KNOWN_ERROR_PATTERNS:
        covered = pattern['id'] in all_covered_patterns
        mark = '✅' if covered else '❌'
        # 找出覆盖该模式的测试文件
        covering_tests = [
            ta['filename'] for ta in test_analyses 
            if pattern['id'] in ta['covers_patterns']
        ]
        test_names = ', '.join(covering_tests[:3])
        if len(covering_tests) > 3:
            test_names += f' +{len(covering_tests)-3}'
        
        lines.append(
            f"| {pattern['id']} | {pattern['name']} | {mark} "
            f"| {test_names if covered else pattern['description']} |"
        )
    lines.append('')
    
    # 未覆盖的模式
    uncovered = [p for p in KNOWN_ERROR_PATTERNS if p['id'] not in all_covered_patterns]
    if uncovered:
        lines.append(f'### ❌ {len(uncovered)} 个错误模式未被测试覆盖')
        lines.append('')
        for p in uncovered:
            lines.append(f'- **#{p["id"]} {p["name"]}**: {p["description"]}')
        lines.append('')
    
    # 未测试的源文件
    if untested_sources:
        lines.append(f'## 未覆盖的源文件 ({len(untested_sources)}个)')
        lines.append('')
        for sf in untested_sources:
            rel_path = os.path.basename(sf)
            lines.append(f'- `{rel_path}`')
        lines.append('')
    
    # 详细模式 - 列出每个测试文件的测试函数
    if detailed:
        lines.append('## 测试函数详情')
        lines.append('')
        
        for ta in sorted(test_analyses, key=lambda x: x['filename']):
            lines.append(f'### {ta["filename"]}')
            lines.append('')
            
            if ta['test_classes']:
                lines.append(f'测试类: {", ".join(ta["test_classes"])}')
                lines.append('')
            
            if ta['test_functions']:
                lines.append('| 测试函数 |')
                lines.append('|----------|')
                for func in ta['test_functions']:
                    lines.append(f'| `{func}()` |')
                lines.append('')
            
            if ta['imports']:
                lines.append(f'导入: {", ".join(ta["imports"][:10])}')
                lines.append('')
    
    # 建议
    lines.append('## 改进建议')
    lines.append('')
    
    suggestions = []
    
    if uncovered:
        suggestions.append(f'1. 为 {len(uncovered)} 个未覆盖的错误模式编写测试用例')
    
    if untested_sources:
        suggestions.append(f'2. 为 {len(untested_sources)} 个未覆盖的源文件编写测试')
    
    self_contained = [ta for ta in test_analyses if ta['self_contained']]
    if self_contained:
        suggestions.append(
            f'3. {len(self_contained)} 个测试文件自行重写了算法逻辑，建议改为import '
            f'wrappers.py中的生产代码封装（错误经验库 #9）'
        )
    
    low_assert = [ta for ta in test_analyses 
                  if ta['assert_count'] < 3 and len(ta['test_functions']) > 0]
    if low_assert:
        suggestions.append(
            f'4. {len(low_assert)} 个测试文件断言数过少，建议增加边界条件测试'
        )
    
    if not suggestions:
        suggestions.append('测试覆盖情况良好，继续保持！')
    
    for s in suggestions:
        lines.append(s)
    lines.append('')
    
    return '\n'.join(lines)


# ============================================================
# 主程序入口
# ============================================================

def main():
    """命令行入口函数"""
    parser = argparse.ArgumentParser(
        description='测试覆盖率报告生成器 - 分析测试对生产代码的覆盖情况',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                                    # 分析全部测试
  %(prog)s --tests-dir ./tests --detailed     # 指定测试目录，详细模式
  %(prog)s --output coverage_report.md        # 保存报告
  %(prog)s --json                             # 输出JSON格式
        """
    )
    
    parser.add_argument('--tests-dir', '-t', type=str, default=None,
                        help='测试文件目录（默认：资产库根目录下的tests/）')
    parser.add_argument('--source-dir', '-s', type=str, default=None,
                        help='生产代码目录（默认：资产库根目录）')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='输出报告文件路径')
    parser.add_argument('--detailed', '-d', action='store_true',
                        help='显示详细信息')
    parser.add_argument('--json', '-j', action='store_true',
                        help='输出JSON格式')
    
    args = parser.parse_args()
    
    # 确定路径
    asset_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    tests_dir = args.tests_dir
    if tests_dir is None:
        # 尝试常见的测试目录位置
        candidates = [
            os.path.join(asset_root, 'tests'),
            os.path.join(asset_root, 'test'),
            os.path.join(asset_root, '05_测试集'),
            asset_root,  # 全局搜索
        ]
        for c in candidates:
            if os.path.isdir(c):
                tests_dir = c
                break
    
    source_dir = args.source_dir or asset_root
    
    print(f"测试目录: {tests_dir}")
    print(f"源码目录: {source_dir}")
    
    # 查找文件
    analyzer = TestAnalyzer()
    
    test_files = analyzer.find_test_files(tests_dir)
    source_files = analyzer.find_source_files(source_dir)
    
    print(f"找到 {len(test_files)} 个测试文件")
    print(f"找到 {len(source_files)} 个源文件")
    
    # 分析测试文件
    test_analyses = []
    for tf in test_files:
        result = analyzer.analyze_test_file(tf)
        test_analyses.append(result)
        print(f"  分析: {result['filename']} ({len(result['test_functions'])} 个测试, "
              f"{result['assert_count']} 个断言)")
    
    # 查找未测试的源文件
    untested = analyzer.find_untested_sources(test_analyses, source_files)
    
    # JSON输出
    if args.json:
        output_data = {
            'summary': {
                'test_files': len(test_analyses),
                'source_files': len(source_files),
                'total_tests': sum(len(ta['test_functions']) for ta in test_analyses),
                'total_asserts': sum(ta['assert_count'] for ta in test_analyses),
                'untested_sources': len(untested)
            },
            'test_analyses': test_analyses,
            'untested_sources': [os.path.basename(f) for f in untested]
        }
        output = json.dumps(output_data, ensure_ascii=False, indent=2, default=list)
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as f:
                f.write(output)
            print(f"JSON已保存至: {args.output}")
        else:
            print(output)
        return
    
    # 生成报告
    report = generate_report(test_analyses, untested, source_files, detailed=args.detailed)
    
    # 输出
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(report)
        print(f"报告已保存至: {args.output}")
    else:
        print('\n' + report)
    
    # 总结
    total_patterns = len(KNOWN_ERROR_PATTERNS)
    covered = len(set(
        pid for ta in test_analyses for pid in ta['covers_patterns']
    ))
    print(f"\n错误模式覆盖: {covered}/{total_patterns}")
    print(f"未测试源文件: {len(untested)}")


if __name__ == '__main__':
    main()
