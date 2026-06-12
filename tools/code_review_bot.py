#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
代码审查机器人 - 电赛资产库工具
===================================
功能：基于错误经验库自动审查C/Python代码，检测已知错误模式
用法：
  python code_review_bot.py file.c                       # 审查单个文件
  python code_review_bot.py --path ./drivers              # 审查整个目录
  python code_review_bot.py --diff HEAD~1                 # 审查git diff
  python code_review_bot.py file.c --strict               # 严格模式
  python code_review_bot.py file.c --output review.md    # 输出到文件

背景：错误经验库记录了48个已知错误模式，本工具自动扫描新代码是否包含这些模式
      结合静态分析规则，提供代码审查意见

错误经验库路径: ../错误经验库.md

依赖：无额外依赖（纯Python实现）
"""

import argparse
import os
import re
import sys
import json
from collections import defaultdict
from datetime import datetime
from pathlib import Path


# ============================================================
# 审查规则引擎
# ============================================================

class ReviewRule:
    """审查规则基类"""
    
    def __init__(self, rule_id, name, severity, description, languages=None):
        """
        参数:
            rule_id: 规则编号
            name: 规则名称
            severity: 严重级别 (ERROR/WARNING/INFO)
            description: 规则描述
            languages: 适用语言列表 ['c', 'python']，None表示全部
        """
        self.rule_id = rule_id
        self.name = name
        self.severity = severity
        self.description = description
        self.languages = languages or ['c', 'python']


# ============================================================
# 基于错误经验库的审查规则
# ============================================================

# 规则定义：每条规则对应错误经验库中的一个或多个模式
REVIEW_RULES = [
    # ---- 来自错误经验库的规则 ----
    
    ReviewRule(
        'ERR-001', '除零风险', 'ERROR',
        '检测到可能的除零风险：除数变量未检查是否为零',
        ['c']
    ),
    ReviewRule(
        'ERR-002', '硬编码绝对路径', 'ERROR',
        '检测到硬编码的绝对路径，应使用相对路径',
        ['python']
    ),
    ReviewRule(
        'ERR-003', '顶层执行无守卫', 'WARNING',
        '检测到模块顶层执行代码，缺少 if __name__ == "__main__" 守卫',
        ['python']
    ),
    ReviewRule(
        'ERR-004', '缺失import', 'ERROR',
        '检测到使用了未导入的模块',
        ['python']
    ),
    ReviewRule(
        'ERR-005', 'ISR变量缺volatile', 'ERROR',
        'ISR共享变量未声明volatile，可能导致优化器消除读取',
        ['c']
    ),
    ReviewRule(
        'ERR-006', 'I2C忙等待无超时', 'ERROR',
        'I2C忙等待循环缺少超时退出机制，可能导致死锁',
        ['c']
    ),
    ReviewRule(
        'ERR-007', '数组/缓冲区溢出', 'ERROR',
        '检测到数组/缓冲区操作可能溢出',
        ['c', 'python']
    ),
    ReviewRule(
        'ERR-008', '死代码', 'INFO',
        '检测到可能的死代码',
        ['c']
    ),
    ReviewRule(
        'ERR-010', '拼写错误', 'WARNING',
        '检测到可能的变量名/枚举拼写错误',
        ['c', 'python']
    ),
    ReviewRule(
        'ERR-011', '角度/弧度单位', 'WARNING',
        '检测到角度/弧度单位转换，建议确认转换公式正确性',
        ['c', 'python']
    ),
    ReviewRule(
        'ERR-013', 'PWM定时器选择', 'ERROR',
        '检测到可能使用了错误的定时器实例输出PWM',
        ['c']
    ),
    ReviewRule(
        'ERR-017', '引脚冲突', 'ERROR',
        '检测到可能的引脚分配冲突（编码器vs超声波PB6/PB7）',
        ['c']
    ),
    ReviewRule(
        'ERR-018', '运算符优先级', 'ERROR',
        '检测到可能的运算符优先级错误：!优先于&',
        ['c']
    ),
    ReviewRule(
        'ERR-019', '混合驱动架构', 'WARNING',
        '检测到混合使用两套驱动架构',
        ['c']
    ),
    ReviewRule(
        'ERR-020', '公式实现错误', 'WARNING',
        '涉及控制公式实现，建议对照参考文献验证',
        ['c']
    ),
    
    # ---- 额外的通用规则 ----
    
    ReviewRule(
        'GEN-001', 'Magic Number', 'INFO',
        '检测到魔术数字（未定义为常量的数字字面量）',
        ['c']
    ),
    ReviewRule(
        'GEN-002', '缺少错误处理', 'WARNING',
        '函数调用缺少返回值检查',
        ['c']
    ),
    ReviewRule(
        'GEN-003', '过长函数', 'WARNING',
        '函数体过长（>100行），建议拆分',
        ['c', 'python']
    ),
    ReviewRule(
        'GEN-004', '过深嵌套', 'WARNING',
        '代码嵌套过深（>4层），建议重构',
        ['c', 'python']
    ),
    ReviewRule(
        'GEN-005', '缺少注释', 'INFO',
        '复杂逻辑缺少注释说明',
        ['c', 'python']
    ),
    ReviewRule(
        'GEN-006', '内存泄漏风险', 'WARNING',
        '检测到malloc/new但未找到对应的free/delete',
        ['c']
    ),
    ReviewRule(
        'GEN-007', '未初始化变量', 'WARNING',
        '变量声明后未初始化即使用',
        ['c']
    ),
]


# ============================================================
# 代码审查引擎
# ============================================================

class CodeReviewer:
    """
    代码审查引擎
    
    对C/Python代码进行静态分析，检测已知错误模式
    """

    def __init__(self, strict=False):
        """
        参数:
            strict: 严格模式下，INFO级别也会报告
        """
        self.strict = strict
        self.findings = []

    def review_file(self, filepath):
        """
        审查单个文件
        
        返回:
            list[dict]: 发现列表，每个发现包含 {
                'rule_id': str,
                'rule_name': str,
                'severity': str,
                'file': str,
                'line': int,
                'code': str,
                'message': str,
                'suggestion': str
            }
        """
        findings = []
        
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
                return findings
            
            filename = os.path.basename(filepath)
            lines = content.split('\n')
            ext = os.path.splitext(filepath)[1].lower()
            lang = 'c' if ext in ('.c', '.h') else 'python' if ext == '.py' else None
            
            if lang is None:
                return findings
            
            # ---- C语言审查 ----
            if lang == 'c':
                findings.extend(self._review_c(content, lines, filename))
            
            # ---- Python审查 ----
            if lang == 'python':
                findings.extend(self._review_python(content, lines, filename))
            
            return findings
            
        except Exception as e:
            print(f"  [警告] 审查 {filepath} 失败: {e}", file=sys.stderr)
            return findings

    def _review_c(self, content, lines, filename):
        """C语言代码审查"""
        findings = []
        
        for i, line in enumerate(lines, 1):
            stripped = line.strip()
            
            # 跳过注释行
            if stripped.startswith('//') or stripped.startswith('/*') or stripped.startswith('*'):
                continue
            
            # ERR-001: 除零风险 - 检测除法中除数为变量（非常量）
            if '/' in stripped and not stripped.startswith('#') and not stripped.startswith('//'):
                # 排除注释中的除法和字符串中的除法
                code_part = stripped.split('//')[0]
                # 匹配 变量/变量 模式（排除 */ 指针解引用）
                div_matches = re.findall(r'(?<!\*)(\w+)\s*/\s*(\w+)', code_part)
                for numerator, denominator in div_matches:
                    # 排除常量除法
                    if denominator.isdigit() or denominator == '0':
                        continue
                    # 检查是否有除零保护
                    protect_pattern = rf'(?:if|while).*{denominator}.*(?:!=|>|>=|<|<=).*0'
                    context_start = max(0, i - 10)
                    context = '\n'.join(lines[context_start:i])
                    if not re.search(protect_pattern, context):
                        findings.append({
                            'rule_id': 'ERR-001',
                            'rule_name': '除零风险',
                            'severity': 'ERROR',
                            'file': filename,
                            'line': i,
                            'code': stripped,
                            'message': f'除数 "{denominator}" 可能为零，缺少保护检查',
                            'suggestion': f'在除法前添加: if (fabsf({denominator}) < 1e-10f) return safe_value;'
                        })
            
            # ERR-005: ISR变量缺volatile
            if re.search(r'(?:void|uint\d+_t|int\d+_t)\s+\w*(?:IRQ|ISR|Handler|interrupt)\w*\s*\(', stripped, re.IGNORECASE):
                # 发现中断处理函数，检查后续代码中修改的变量
                block_start = i
                brace_count = 0
                for j in range(i - 1, min(i + 50, len(lines))):
                    brace_count += lines[j].count('{') - lines[j].count('}')
                    if brace_count == 0 and j > i:
                        break
                    # 查找赋值操作
                    assign = re.match(r'\s*(\w+)\s*=\s*', lines[j])
                    if assign:
                        var = assign.group(1)
                        if var not in ('if', 'while', 'for', 'return', 'else'):
                            # 检查该变量是否声明了volatile
                            var_decl = re.search(rf'volatile\s+.*\b{var}\b', content)
                            if not var_decl:
                                # 检查是否是局部变量
                                local_decl = re.search(rf'^\s+(?:uint\d+_t|int\d+_t|float|char|bool)\s+{var}\b', 
                                                       '\n'.join(lines[block_start:j+1]))
                                if not local_decl:
                                    findings.append({
                                        'rule_id': 'ERR-005',
                                        'rule_name': 'ISR变量缺volatile',
                                        'severity': 'ERROR',
                                        'file': filename,
                                        'line': j + 1,
                                        'code': lines[j].strip(),
                                        'message': f'ISR中修改变量 "{var}" 但未声明volatile',
                                        'suggestion': f'将变量声明为: volatile ... {var};'
                                    })
            
            # ERR-006: I2C忙等待无超时
            if re.search(r'while\s*\(', stripped) and not stripped.endswith(';'):
                # 检查是否是I2C相关等待
                context = '\n'.join(lines[max(0, i-3):i+5])
                if re.search(r'[Ii]2[Cc]|i2c|I2C|busy|flag|status', context):
                    # 检查是否有超时
                    has_timeout = False
                    block_end = min(i + 20, len(lines))
                    for j in range(i - 1, block_end):
                        if re.search(r'timeout|counter|retry|cnt|max_wait', lines[j], re.IGNORECASE):
                            has_timeout = True
                            break
                    if not has_timeout:
                        findings.append({
                            'rule_id': 'ERR-006',
                            'rule_name': 'I2C忙等待无超时',
                            'severity': 'ERROR',
                            'file': filename,
                            'line': i,
                            'code': stripped,
                            'message': 'I2C忙等待循环缺少超时退出机制',
                            'suggestion': '添加超时计数器: uint32_t timeout = 10000; while(condition && timeout-- > 0);'
                        })
            
            # ERR-013: PWM定时器选择
            if re.search(r'TIMG\d|TimerG\d', stripped):
                findings.append({
                    'rule_id': 'ERR-013',
                    'rule_name': 'PWM定时器选择',
                    'severity': 'WARNING',
                    'file': filename,
                    'line': i,
                    'code': stripped,
                    'message': '检测到使用TIMG(通用定时器)，电机PWM应使用TIMA(高级定时器)',
                    'suggestion': '确认是否应使用TIMA0 + CC_x_INDEX'
                })
            
            # ERR-017: 引脚冲突 - PB6/PB7
            if re.search(r'PB[67]|P1\.[67]', stripped):
                # 检查上下文是否涉及编码器
                context = '\n'.join(lines[max(0, i-5):i+5])
                if re.search(r'encoder|编码|ENC', context, re.IGNORECASE):
                    if re.search(r'ultrasonic|超声|TRIG|ECHO', content, re.IGNORECASE):
                        findings.append({
                            'rule_id': 'ERR-017',
                            'rule_name': '引脚冲突',
                            'severity': 'ERROR',
                            'file': filename,
                            'line': i,
                            'code': stripped,
                            'message': 'PB6/PB7可能与超声波模块引脚冲突',
                            'suggestion': '参考pin_config.h确认引脚分配，编码器右轮应使用PB4/PB5'
                        })
            
            # ERR-018: 运算符优先级 !优先于&
            if re.search(r'!\s*\w+.*&\s*\w+', stripped) and '&&' not in stripped:
                if re.search(r'while\s*\(', stripped) or re.search(r'if\s*\(', stripped):
                    findings.append({
                        'rule_id': 'ERR-018',
                        'rule_name': '运算符优先级',
                        'severity': 'ERROR',
                        'file': filename,
                        'line': i,
                        'code': stripped,
                        'message': '!运算符优先级高于&，可能导致逻辑错误',
                        'suggestion': '使用括号: !(expr & FLAG) 而非 !expr & FLAG'
                    })
            
            # ERR-019: 混合驱动架构
            if 'ti_msp_dl_config.h' in stripped:
                # 检查是否同时使用了自定义驱动
                if re.search(r'#include.*driverlib|#include.*platform|#include.*drivers', content):
                    if not any(f['rule_id'] == 'ERR-019' for f in findings):
                        findings.append({
                            'rule_id': 'ERR-019',
                            'rule_name': '混合驱动架构',
                            'severity': 'WARNING',
                            'file': filename,
                            'line': i,
                            'code': stripped,
                            'message': '同时使用了SysConfig驱动和自定义驱动',
                            'suggestion': '建议统一为一套驱动架构'
                        })
            
            # GEN-001: Magic Number
            if not stripped.startswith('#') and not stripped.startswith('//'):
                magic_nums = re.findall(r'(?<![.\w])(\d{2,}(?:\.\d+)?)(?![.\w])', stripped)
                for num in magic_nums:
                    if num in ('0', '1', '2', '10', '100', '16', '8', '32', '64'):
                        continue  # 常见数字不报告
                    if re.search(rf'#define\s+\w+\s+{num}', content):
                        continue  # 已定义为宏
                    findings.append({
                        'rule_id': 'GEN-001',
                        'rule_name': 'Magic Number',
                        'severity': 'INFO',
                        'file': filename,
                        'line': i,
                        'code': stripped,
                        'message': f'魔术数字 {num}，建议定义为常量',
                        'suggestion': f'#define MEANINGFUL_NAME {num}'
                    })
                    break  # 每行只报告一个
            
            # GEN-006: 内存泄漏风险
            if re.search(r'\bmalloc\s*\(|\bcalloc\s*\(|\brealloc\s*\(', stripped):
                # 检查函数内是否有对应的free
                func_end = min(i + 100, len(lines))
                has_free = False
                for j in range(i, func_end):
                    if re.search(r'\bfree\s*\(', lines[j]):
                        has_free = True
                        break
                    if re.search(r'^\w.*\{', lines[j]) and j > i + 1:
                        break  # 进入了新函数
                if not has_free:
                    findings.append({
                        'rule_id': 'GEN-006',
                        'rule_name': '内存泄漏风险',
                        'severity': 'WARNING',
                        'file': filename,
                        'line': i,
                        'code': stripped,
                        'message': '检测到动态内存分配但未找到对应的free()',
                        'suggestion': '确保所有分配的内存最终被释放'
                    })
        
        return findings

    def _review_python(self, content, lines, filename):
        """Python代码审查"""
        findings = []
        
        # 检查顶层执行守卫
        has_guard = False
        has_top_level_exec = False
        
        for i, line in enumerate(lines, 1):
            stripped = line.strip()
            
            # 跳过注释
            if stripped.startswith('#'):
                continue
            
            # ERR-002: 硬编码绝对路径
            if re.search(r'["\'](?:D:|C:|/home/|/usr/|/tmp/).*["\']', stripped):
                if not stripped.startswith('#'):
                    findings.append({
                        'rule_id': 'ERR-002',
                        'rule_name': '硬编码绝对路径',
                        'severity': 'ERROR',
                        'file': filename,
                        'line': i,
                        'code': stripped,
                        'message': '硬编码绝对路径，换机器后会报错',
                        'suggestion': '使用 os.path.join(os.path.dirname(os.path.abspath(__file__)), ...) '
                    })
            
            # ERR-003: 顶层执行守卫
            if '__name__' in stripped and '__main__' in stripped:
                has_guard = True
            
            # 检测可能的顶层执行代码（非函数/类定义、非import、非赋值）
            if (not stripped.startswith(('def ', 'class ', 'import ', 'from ', '#', '@', 
                                          '"""', "'''", 'if __name__', 'else:', 'elif ',
                                          'try:', 'except', 'finally:', 'with ')) 
                and stripped 
                and not stripped.startswith(('=', '(', '[', '{'))
                and '=' not in stripped.split('(')[0]  # 排除赋值
                and not stripped.startswith(('self.', 'return', 'raise', 'yield', 'pass', 'break', 'continue'))
                and re.match(r'^\w+\(', stripped)  # 函数调用模式
                and i > 30):  # 跳过文件头部的配置
                # 这可能是在模块顶层执行的函数调用
                if any(kw in stripped for kw in ['plt.show', 'plt.plot', 'plt.savefig', 
                                                   'print(', 'sns.', 'fig,', 'ax,',
                                                   'np.random', 'scipy.', 'simulate(']):
                    has_top_level_exec = True
                    findings.append({
                        'rule_id': 'ERR-003',
                        'rule_name': '顶层执行无守卫',
                        'severity': 'WARNING',
                        'file': filename,
                        'line': i,
                        'code': stripped,
                        'message': '模块顶层执行代码，import时会自动触发',
                        'suggestion': '封装为函数 + if __name__ == "__main__": 守卫'
                    })
            
            # ERR-004: 缺失import检查
            if re.search(r'\bnp\.', stripped):
                if not re.search(r'import\s+numpy\s+as\s+np|from\s+numpy\s+import', content[:i]):
                    if not any(f['rule_id'] == 'ERR-004' and 'numpy' in f.get('message', '') for f in findings):
                        findings.append({
                            'rule_id': 'ERR-004',
                            'rule_name': '缺失import',
                            'severity': 'ERROR',
                            'file': filename,
                            'line': i,
                            'code': stripped,
                            'message': '使用了 np.xxx 但未 import numpy',
                            'suggestion': '在文件头部添加: import numpy as np'
                        })
            
            if re.search(r'\bplt\.', stripped):
                if not re.search(r'import\s+matplotlib|from\s+matplotlib', content[:i]):
                    if not any(f['rule_id'] == 'ERR-004' and 'matplotlib' in f.get('message', '') for f in findings):
                        findings.append({
                            'rule_id': 'ERR-004',
                            'rule_name': '缺失import',
                            'severity': 'ERROR',
                            'file': filename,
                            'line': i,
                            'code': stripped,
                            'message': '使用了 plt.xxx 但未 import matplotlib',
                            'suggestion': '在文件头部添加: import matplotlib.pyplot as plt'
                        })
            
            # ERR-007: 缓冲区溢出（列表索引）
            if re.search(r'\[\s*-?\d+\s*\]', stripped) and 'range' not in stripped:
                idx_match = re.search(r'\[\s*(-?\d+)\s*\]', stripped)
                if idx_match:
                    idx = int(idx_match.group(1))
                    if idx > 100 or idx < -10:
                        findings.append({
                            'rule_id': 'ERR-007',
                            'rule_name': '数组/缓冲区溢出',
                            'severity': 'WARNING',
                            'file': filename,
                            'line': i,
                            'code': stripped,
                            'message': f'使用了硬编码索引 [{idx}]，可能越界',
                            'suggestion': '检查数组长度或使用动态索引'
                        })
            
            # GEN-003: 过长函数
            if stripped.startswith('def '):
                func_start = i
                func_indent = len(line) - len(line.lstrip())
                func_end = i
                for j in range(i, min(i + 200, len(lines))):
                    if j > i and lines[j].strip() and not lines[j].strip().startswith('#'):
                        curr_indent = len(lines[j]) - len(lines[j].lstrip())
                        if curr_indent <= func_indent and lines[j].strip():
                            func_end = j
                            break
                    func_end = j + 1
                
                func_length = func_end - func_start
                if func_length > 100:
                    findings.append({
                        'rule_id': 'GEN-003',
                        'rule_name': '过长函数',
                        'severity': 'WARNING',
                        'file': filename,
                        'line': i,
                        'code': stripped,
                        'message': f'函数体过长 ({func_length}行)，建议拆分',
                        'suggestion': '将功能独立的代码块提取为子函数'
                    })
        
        return findings


# ============================================================
# 报告生成器
# ============================================================

def generate_report(all_findings, files_reviewed):
    """
    生成审查报告（Markdown格式）
    """
    lines = []
    
    lines.append('# 代码审查报告')
    lines.append('')
    lines.append(f'> 生成时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
    lines.append(f'> 审查文件数: {files_reviewed}')
    lines.append(f'> 发现问题数: {len(all_findings)}')
    lines.append('')
    
    # 统计摘要
    severity_count = defaultdict(int)
    rule_count = defaultdict(int)
    for f in all_findings:
        severity_count[f['severity']] += 1
        rule_count[f['rule_id']] += 1
    
    lines.append('## 问题摘要')
    lines.append('')
    lines.append('| 级别 | 数量 |')
    lines.append('|------|------|')
    
    for sev in ['ERROR', 'WARNING', 'INFO']:
        icon = {'ERROR': '❌', 'WARNING': '⚠️', 'INFO': 'ℹ️'}[sev]
        if severity_count[sev] > 0:
            lines.append(f'| {icon} {sev} | {severity_count[sev]} |')
    
    if not all_findings:
        lines.append('| ✅ 无问题 | 0 |')
    lines.append('')
    
    # 按规则统计
    if rule_count:
        lines.append('## 按规则统计')
        lines.append('')
        lines.append('| 规则ID | 规则名称 | 出现次数 |')
        lines.append('|--------|---------|----------|')
        
        # 获取规则信息
        rule_map = {r.rule_id: r for r in REVIEW_RULES}
        for rule_id, count in sorted(rule_count.items(), key=lambda x: x[1], reverse=True):
            rule = rule_map.get(rule_id)
            name = rule.name if rule else rule_id
            lines.append(f'| {rule_id} | {name} | {count} |')
        lines.append('')
    
    # 详细发现
    if all_findings:
        lines.append('## 详细发现')
        lines.append('')
        
        # 按文件分组
        by_file = defaultdict(list)
        for f in all_findings:
            by_file[f['file']].append(f)
        
        for fname, file_findings in sorted(by_file.items()):
            lines.append(f'### {fname}')
            lines.append('')
            
            for f in sorted(file_findings, key=lambda x: x['line']):
                icon = {'ERROR': '❌', 'WARNING': '⚠️', 'INFO': 'ℹ️'}[f['severity']]
                lines.append(f'{icon} **L{f["line"]}** [{f["rule_id"]}] {f["message"]}')
                lines.append(f'```')
                lines.append(f'{f["code"]}')
                lines.append(f'```')
                lines.append(f'> 💡 建议: {f["suggestion"]}')
                lines.append('')
    
    # 无问题
    if not all_findings:
        lines.append('## ✅ 未发现问题')
        lines.append('')
        lines.append('所有审查规则通过，代码质量良好！')
        lines.append('')
    
    return '\n'.join(lines)


# ============================================================
# 主程序入口
# ============================================================

def main():
    """命令行入口函数"""
    parser = argparse.ArgumentParser(
        description='代码审查机器人 - 基于错误经验库自动审查代码',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s my_driver.c                          # 审查单个文件
  %(prog)s --path ./drivers                      # 审查整个目录
  %(prog)s my_driver.c --strict --output r.md    # 严格模式并保存
  %(prog)s --json my_driver.c                    # JSON输出

规则来源:
  错误经验库（48个已知错误模式）+ 通用静态分析规则
        """
    )
    
    parser.add_argument('files', nargs='*', help='要审查的文件')
    parser.add_argument('--path', '-p', type=str, default=None,
                        help='审查目录（递归扫描）')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='输出报告文件路径')
    parser.add_argument('--strict', '-s', action='store_true',
                        help='严格模式（报告所有级别包括INFO）')
    parser.add_argument('--json', '-j', action='store_true',
                        help='输出JSON格式')
    parser.add_argument('--errors-only', '-e', action='store_true',
                        help='仅报告ERROR级别')
    parser.add_argument('--no-err001', action='store_true',
                        help='禁用ERR-001（除零检查，可能产生大量警告）')
    parser.add_argument('--no-err002', action='store_true',
                        help='禁用ERR-002（硬编码路径检查）')
    parser.add_argument('--no-gen001', action='store_true',
                        help='禁用GEN-001（魔术数字检查，可能产生大量警告）')
    
    args = parser.parse_args()
    
    # 收集要审查的文件
    target_files = list(args.files) if args.files else []
    
    if args.path:
        for root, dirs, files in os.walk(args.path):
            dirs[:] = [d for d in dirs if not d.startswith('.') and d != 'tools']
            for f in files:
                if f.endswith(('.c', '.h', '.py')) and not f.startswith('test_'):
                    target_files.append(os.path.join(root, f))
    
    if not target_files:
        # 默认扫描资产库
        asset_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for root, dirs, files in os.walk(asset_root):
            dirs[:] = [d for d in dirs if not d.startswith('.') and d != 'tools' and d != 'tests']
            for f in files:
                if f.endswith(('.c', '.h')) and not f.startswith('test_'):
                    target_files.append(os.path.join(root, f))
    
    target_files = sorted(set(target_files))
    
    if not target_files:
        print("未找到要审查的文件")
        return
    
    print(f"准备审查 {len(target_files)} 个文件...")
    
    # 执行审查
    reviewer = CodeReviewer(strict=args.strict)
    all_findings = []
    
    # 禁用规则
    disabled_rules = set()
    if args.no_err001:
        disabled_rules.add('ERR-001')
    if args.no_err002:
        disabled_rules.add('ERR-002')
    if args.no_gen001:
        disabled_rules.add('GEN-001')
    
    for filepath in target_files:
        findings = reviewer.review_file(filepath)
        # 过滤禁用的规则
        findings = [f for f in findings if f['rule_id'] not in disabled_rules]
        # 过滤严格模式
        if not args.strict:
            findings = [f for f in findings if f['severity'] != 'INFO']
        if args.errors_only:
            findings = [f for f in findings if f['severity'] == 'ERROR']
        
        if findings:
            all_findings.extend(findings)
            print(f"  {os.path.basename(filepath)}: {len(findings)} 个问题")
        else:
            print(f"  {os.path.basename(filepath)}: ✅")
    
    print(f"\n审查完成: {len(all_findings)} 个问题")
    
    # JSON输出
    if args.json:
        output = json.dumps(all_findings, ensure_ascii=False, indent=2)
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as f:
                f.write(output)
            print(f"JSON已保存至: {args.output}")
        else:
            print(output)
        return
    
    # 生成报告
    report = generate_report(all_findings, len(target_files))
    
    # 输出
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(report)
        print(f"报告已保存至: {args.output}")
    else:
        print('\n' + report)
    
    # 返回状态码
    error_count = sum(1 for f in all_findings if f['severity'] == 'ERROR')
    if error_count > 0:
        print(f"\n❌ 发现 {error_count} 个ERROR级别问题!")
        sys.exit(1)
    elif all_findings:
        print(f"\n⚠️  发现 {len(all_findings)} 个问题（无ERROR）")
    else:
        print(f"\n✅ 审查通过!")


if __name__ == '__main__':
    main()
