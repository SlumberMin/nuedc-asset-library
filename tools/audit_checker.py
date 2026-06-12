#!/usr/bin/env python3
"""
电赛资产库 V2 深度审计 — 错误模式自动检测脚本
==============================================
基于错误经验库中的39个已知错误模式，自动扫描全部 .c / .py 文件。

用法:
    python audit_checker.py [扫描目录]              # 检测模式
    python audit_checker.py [扫描目录] --fix         # 检测 + 自动修复
    python audit_checker.py [扫描目录] --report out.json  # 指定输出文件

输出: JSON格式检测报告
"""

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import List, Optional

# ═══════════════════════════════════════════════════════════════════════
# 数据结构
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class Finding:
    """单条检测发现"""
    rule_id: str            # 对应错误经验库编号, 如 "E01"
    rule_name: str          # 规则名称
    severity: str           # critical / warning / info
    file: str               # 文件路径
    line: int               # 行号 (0=全局)
    message: str            # 描述
    auto_fixable: bool = False
    fixed: bool = False     # 是否已自动修复

@dataclass
class AuditReport:
    """审计报告"""
    scan_time: str = ""
    scan_root: str = ""
    total_c_files: int = 0
    total_py_files: int = 0
    total_findings: int = 0
    critical: int = 0
    warning: int = 0
    info: int = 0
    auto_fixed: int = 0
    findings: List[dict] = field(default_factory=list)

# ═══════════════════════════════════════════════════════════════════════
# C 文件检测规则 (E01 ~ E22)
# ═══════════════════════════════════════════════════════════════════════

class CChecker:
    """C 语言错误模式检测器"""

    def __init__(self, filepath: str, content: str, lines: list, fix: bool = False):
        self.filepath = filepath
        self.content = content
        self.lines = lines
        self.fix = fix
        self.findings: List[Finding] = []

    def run_all(self) -> List[Finding]:
        self.check_division_by_zero()          # E01, E16, E23~E26, E34
        self.check_isr_missing_volatile()      # E05, E33
        self.check_i2c_busy_no_timeout()       # E06, E31, E32
        self.check_dead_code()                 # E08
        self.check_pin_conflict()              # E17(pin)
        self.check_operator_precedence()       # E18(op)
        self.check_buffer_overflow()           # E07, E35
        self.check_static_multi_instance()     # E22
        self.check_hal_max_delay()             # E31
        self.check_pwm_timer_type()            # E13
        return self.findings

    # ─── E01/E16/E23~E26/E34: 除零风险 ────────────────────────────────
    def check_division_by_zero(self):
        """检测除法运算中除数可能为0的情况"""
        div_pattern = re.compile(
            r'(?<!/)\b(\w+)\s*/\s*(\w+)'  # x / y
        )
        # 已知安全的常量除数
        safe_constants = {'2', '3', '4', '5', '8', '10', '16', '100', '180',
                          '360', '1000', '1024', '32768', '65536', '256',
                          '0.5f', '0.5', '2.0f', '2.0', '3.14159f', 'M_PI'}
        # 已知需要保护的函数参数
        risky_params = {'b', 'b0', 'Q', 'dt', 'Ts', 'sv_range', 'range',
                        'max_vel', 'max_accel', 'max_jerk', 'K', 'Ti', 'Td',
                        'divisor', 'angle_range', 'tau'}

        for i, line in enumerate(self.lines, 1):
            stripped = line.strip()
            if stripped.startswith('//') or stripped.startswith('/*'):
                continue
            # 跳过 /* */ 块注释中间的行
            if stripped.startswith('*') and not stripped.startswith('*/'):
                continue
            for m in div_pattern.finditer(line):
                divisor = m.group(2)
                # 跳过常量和字面量
                if divisor in safe_constants:
                    continue
                if re.match(r'^[\d.]+f?$', divisor):
                    continue
                # 检查是否已有保护
                ctx_start = max(0, i - 15)
                context = '\n'.join(self.lines[ctx_start:i-1])
                has_guard = (
                    f'fabsf({divisor})' in context or
                    f'if ({divisor}' in context or
                    f'if({divisor}' in context or
                    f'{divisor} < 1e-' in context or
                    f'{divisor} <= 0' in context or
                    f'{divisor} > 0' in context or
                    f'{divisor} > 1e-' in context or
                    f'{divisor} == 0' in context or
                    f'return;' in context  # 函数内有提前返回(可能含保护)
                )
                if not has_guard:
                    severity = 'critical' if divisor in risky_params else 'warning'
                    self.findings.append(Finding(
                        rule_id='E01', rule_name='除零风险',
                        severity=severity,
                        file=self.filepath, line=i,
                        message=f'除数 `{divisor}` 可能为0，未检测到保护代码',
                        auto_fixable=False
                    ))

    # ─── E05/E33: ISR共享变量缺volatile ────────────────────────────────
    def check_isr_missing_volatile(self):
        """检测ISR共享变量是否缺少volatile修饰"""
        isr_keywords = ['IRQHandler', 'ISR', '_Handler', 'interrupt', '__interrupt']
        # 找到所有ISR函数
        isr_funcs = set()
        func_re = re.compile(r'void\s+(\w*(?:IRQ|ISR|Handler)\w*)\s*\(')
        for m in func_re.finditer(self.content):
            isr_funcs.add(m.group(1))

        if not isr_funcs:
            return

        # 找ISR中赋值的变量 和 ISR内声明的局部变量
        isr_vars = set()
        isr_local_vars = set()  # ISR内部声明的局部变量
        in_isr = False
        brace_depth = 0
        for line in self.lines:
            for func_name in isr_funcs:
                if func_name in line and ('void' in line or '__interrupt' in line):
                    in_isr = True
                    brace_depth = 0
            if in_isr:
                brace_depth += line.count('{') - line.count('}')
                if brace_depth <= 0 and '{' not in line and '}' in line:
                    in_isr = False
                    continue
                # 捕获ISR内声明的局部变量 (uint8_t data; 等)
                local_decl = re.search(r'(?:int|float|uint\d+_t|int\d+_t|char|short|long|double)\s+\*?\s*(\w+)\s*[;=\[]', line)
                if local_decl:
                    isr_local_vars.add(local_decl.group(1))
                # 捕获赋值: var = ...  或 var++ 或 ++var
                # 跳过结构体成员赋值 (obj->field = ... 或 obj.field = ...)
                if '->' in line or ('.' in line and re.search(r'\w+\.\w+\s*=', line)):
                    continue
                assigns = re.findall(r'\b([a-zA-Z_]\w*)\s*(?:=|\+\+|--|<<=|>>=|\+=|-=|\*=|/=|&=|\|=|\^=)', line)
                for v in assigns:
                    if v not in ('if', 'while', 'for', 'return', 'volatile', 'static'):
                        isr_vars.add(v)

        # 检查这些变量在非ISR上下文中声明时是否带volatile
        for var in isr_vars:
            # 跳过ISR内部声明的局部变量(如 uint8_t data 在ISR内)
            if var in isr_local_vars:
                continue
            # 跳过结构体成员访问中的字段名(如 sched->tick_count 中的 tick_count)
            # 这些字段可能在 .h 文件的结构体中已声明为 volatile
            is_struct_member = False
            for line in self.lines:
                if re.search(rf'->\s*{re.escape(var)}\b', line) or re.search(rf'\.\s*{re.escape(var)}\b', line):
                    is_struct_member = True
                    break
            if is_struct_member:
                # 检查对应 .h 文件中是否有 volatile 声明
                header_file = self.filepath.replace('.c', '.h')
                if os.path.exists(header_file):
                    with open(header_file, 'r', encoding='utf-8', errors='replace') as hf:
                        hcontent = hf.read()
                    if re.search(rf'\bvolatile\b.*\b{re.escape(var)}\b', hcontent):
                        continue  # .h 中已有 volatile，跳过
                # 也检查同目录下的所有 .h 文件
                dir_path = os.path.dirname(self.filepath)
                if os.path.isdir(dir_path):
                    for hf_name in os.listdir(dir_path):
                        if hf_name.endswith('.h'):
                            hf_path = os.path.join(dir_path, hf_name)
                            try:
                                with open(hf_path, 'r', encoding='utf-8', errors='replace') as hf:
                                    hcontent = hf.read()
                                if re.search(rf'\\bvolatile\\b.*\\b{re.escape(var)}\\b', hcontent):
                                    is_struct_member = False  # volatile已存在于.h中
                                    break
                            except:
                                pass
                if not is_struct_member:
                    continue  # .h中已有volatile声明，跳过此变量
            decl_re = re.compile(
                rf'(?<!\bvolatile\b\s)(?:extern\s+|static\s+)?(?:\w+\s+)+\b{re.escape(var)}\b'
            )
            for i, line in enumerate(self.lines, 1):
                if re.search(rf'\bvolatile\b.*\b{re.escape(var)}\b', line):
                    break  # 已有volatile
                if re.search(rf'(?:int|float|uint\d+_t|int\d+_t|char|short|long|double|volatile)\s+.*\b{re.escape(var)}\b', line):
                    if 'volatile' not in line and var in line and (';' in line or ',' in line):
                        # 跳过函数参数声明 (行中有 ( 在变量之前)
                        paren_pos = line.find('(')
                        var_pos = line.find(var)
                        if paren_pos >= 0 and var_pos > paren_pos:
                            continue
                        # 跳过局部变量声明 (在ISR函数内部声明的变量)
                        # 简化: 只报告声明行
                        self.findings.append(Finding(
                            rule_id='E05', rule_name='ISR共享变量缺volatile',
                            severity='critical',
                            file=self.filepath, line=i,
                            message=f'变量 `{var}` 在ISR({", ".join(isr_funcs)})中被修改，但声明缺少volatile修饰',
                            auto_fixable=False
                        ))
                        break

    # ─── E06/E31/E32: I2C忙等待无超时 ──────────────────────────────────
    def check_i2c_busy_no_timeout(self):
        """检测I2C忙等待循环缺少超时机制"""
        patterns = [
            # MSPM0: while (I2C忙标志) 无超时
            (re.compile(r'while\s*\(\s*(?:DL_I2C_\w+|I2C\w*Busy|I2CMasterBusy)\s*\('), 'I2C忙等待'),
            # STM32 HAL: while (HAL_I2C_*) 无超时
            (re.compile(r'while\s*\(\s*HAL_I2C_\w+\s*\('), 'HAL_I2C忙等待'),
            # generic: while(检查标志) 在I2C上下文
            (re.compile(r'while\s*\(\s*!(?:DL_I2C_\w+|I2C_\w+)\s*\('), 'I2C标志等待'),
        ]
        for i, line in enumerate(self.lines, 1):
            stripped = line.strip()
            if stripped.startswith('//'):
                continue
            for pat, desc in patterns:
                if pat.search(line):
                    # 检查附近是否有timeout
                    ctx_start = max(0, i - 5)
                    ctx_end = min(len(self.lines), i + 10)
                    context = '\n'.join(self.lines[ctx_start:ctx_end])
                    if 'timeout' not in context.lower() and 'TIMEOUT' not in context and 'retry' not in context.lower():
                        self.findings.append(Finding(
                            rule_id='E06', rule_name='I2C忙等待无超时',
                            severity='critical',
                            file=self.filepath, line=i,
                            message=f'{desc}: while循环无超时退出机制，I2C总线异常时MCU将永久阻塞',
                            auto_fixable=False
                        ))

    # ─── E08: 死代码 ──────────────────────────────────────────────────
    def check_dead_code(self):
        """检测简单的死代码模式"""
        # 模式1: 赋值后立即被覆盖  x = a; x = b;
        assign_re = re.compile(r'^\s*(\w+)\s*=\s*(.+?);')
        prev_var = None
        prev_line = 0
        for i, line in enumerate(self.lines, 1):
            m = assign_re.match(line)
            if m:
                var, val = m.group(1), m.group(2)
                if prev_var == var and not val.startswith(var):
                    self.findings.append(Finding(
                        rule_id='E08', rule_name='死代码',
                        severity='warning',
                        file=self.filepath, line=prev_line,
                        message=f'变量 `{var}` 在第{prev_line}行赋值后，第{i}行立即被覆盖，前一次赋值无效',
                        auto_fixable=False
                    ))
                prev_var = var
                prev_line = i
            elif line.strip() and not line.strip().startswith('//'):
                prev_var = None

        # 模式2: #if 0 ... #endif (注释掉的代码块)
        in_if0 = False
        if0_start = 0
        for i, line in enumerate(self.lines, 1):
            if re.match(r'\s*#if\s+0\b', line):
                in_if0 = True
                if0_start = i
            elif in_if0 and re.match(r'\s*#endif', line):
                self.findings.append(Finding(
                    rule_id='E08', rule_name='死代码',
                    severity='info',
                    file=self.filepath, line=if0_start,
                    message=f'#if 0 死代码块 (第{if0_start}~{i}行)，建议删除',
                    auto_fixable=False
                ))
                in_if0 = False

    # ─── E17: 引脚冲突 ────────────────────────────────────────────────
    def check_pin_conflict(self):
        """检测已知的引脚冲突: 编码器PB6/PB7 vs 超声波PB6/PB7"""
        pb6_uses = []
        pb7_uses = []
        for i, line in enumerate(self.lines, 1):
            stripped = line.strip()
            if stripped.startswith('//'):
                continue
            if re.search(r'PB[_]?6|GPIO_PIN_6.*PORTB|DL_GPIO_PIN_6', line, re.IGNORECASE):
                use_type = 'ultrasonic' if any(k in line.lower() for k in ['trig', 'echo', 'ultrasonic', 'hcsr']) else 'encoder' if any(k in line.lower() for k in ['enc', 'encoder']) else 'other'
                pb6_uses.append((i, use_type, stripped[:80]))
            if re.search(r'PB[_]?7|GPIO_PIN_7.*PORTB|DL_GPIO_PIN_7', line, re.IGNORECASE):
                use_type = 'ultrasonic' if any(k in line.lower() for k in ['trig', 'echo', 'ultrasonic', 'hcsr']) else 'encoder' if any(k in line.lower() for k in ['enc', 'encoder']) else 'other'
                pb7_uses.append((i, use_type, stripped[:80]))

        # 如果同一文件中PB6/PB7同时用于encoder和ultrasonic
        for pin_name, uses in [('PB6', pb6_uses), ('PB7', pb7_uses)]:
            types = set(u[1] for u in uses)
            if 'encoder' in types and 'ultrasonic' in types:
                self.findings.append(Finding(
                    rule_id='E17', rule_name='引脚冲突',
                    severity='critical',
                    file=self.filepath, line=uses[0][0],
                    message=f'{pin_name} 同时被编码器和超声波使用，存在硬件冲突',
                    auto_fixable=False
                ))

    # ─── E18: 运算符优先级 !优先于& ───────────────────────────────────
    def check_operator_precedence(self):
        """检测 !x & flag 模式 (应为 !(x & flag))"""
        pat = re.compile(r'!\s*\w+\s*\)\s*&\s*\w+')
        for i, line in enumerate(self.lines, 1):
            if pat.search(line) and '!(' not in line:
                self.findings.append(Finding(
                    rule_id='E18', rule_name='运算符优先级错误',
                    severity='critical',
                    file=self.filepath, line=i,
                    message='`!x & flag` 中 `!` 优先于 `&`，可能需要 `!(x & flag)`',
                    auto_fixable=False
                ))

    # ─── E07/E35: 缓冲区溢出 ──────────────────────────────────────────
    def check_buffer_overflow(self):
        """检测小缓冲区用于int转换"""
        pat = re.compile(r'char\s+(\w+)\s*\[\s*(\d+)\s*\]')
        for i, line in enumerate(self.lines, 1):
            m = pat.search(line)
            if m:
                buf_name, size = m.group(1), int(m.group(2))
                if size < 16 and ('num' in buf_name.lower() or 'str' in buf_name.lower() or 'buf' in buf_name.lower()):
                    # 检查附近是否有sprintf/snprintf
                    ctx = '\n'.join(self.lines[max(0,i-1):min(len(self.lines),i+15)])
                    if 'sprintf' in ctx and 'snprintf' not in ctx:
                        self.findings.append(Finding(
                            rule_id='E07', rule_name='缓冲区溢出风险',
                            severity='warning',
                            file=self.filepath, line=i,
                            message=f'`{buf_name}[{size}]` 用于sprintf转换，int32_t最多需12字节+null，缓冲区偏小',
                            auto_fixable=False
                        ))

    # ─── E22: static变量多实例冲突 ────────────────────────────────────
    def check_static_multi_instance(self):
        """检测函数内static局部变量(多实例冲突风险)"""
        pat = re.compile(r'^\s+static\s+(?:volatile\s+)?(?:\w+\s+)+(\w+)\s*[=;]')
        for i, line in enumerate(self.lines, 1):
            m = pat.match(line)
            if m:
                var = m.group(1)
                self.findings.append(Finding(
                    rule_id='E22', rule_name='static变量多实例冲突',
                    severity='warning',
                    file=self.filepath, line=i,
                    message=f'函数内`static`变量`{var}`：多实例共用同一状态，使用时需注意单实例限制',
                    auto_fixable=False
                ))

    # ─── E31: HAL_MAX_DELAY ───────────────────────────────────────────
    def check_hal_max_delay(self):
        """检测HAL_MAX_DELAY无限等待"""
        for i, line in enumerate(self.lines, 1):
            if 'HAL_MAX_DELAY' in line:
                self.findings.append(Finding(
                    rule_id='E31', rule_name='HAL_MAX_DELAY无限等待',
                    severity='critical',
                    file=self.filepath, line=i,
                    message='使用HAL_MAX_DELAY(0xFFFFFFFF)作为超时参数，I2C异常时将永久阻塞',
                    auto_fixable=False
                ))

    # ─── E13: PWM定时器类型 ───────────────────────────────────────────
    def check_pwm_timer_type(self):
        """检测电机PWM是否使用TIMG0(应为TIMA0)"""
        for i, line in enumerate(self.lines, 1):
            stripped = line.strip()
            if stripped.startswith('//'):
                continue
            if re.search(r'TIMG0.*(?:PWM|CC_.*INDEX|motor)', line, re.IGNORECASE):
                self.findings.append(Finding(
                    rule_id='E13', rule_name='PWM定时器类型错误',
                    severity='warning',
                    file=self.filepath, line=i,
                    message='电机PWM应使用TIMA0(高级定时器)而非TIMG0(通用定时器)',
                    auto_fixable=False
                ))


# ═══════════════════════════════════════════════════════════════════════
# Python 文件检测规则 (E02 ~ E04, E27~E30, E36~E39)
# ═══════════════════════════════════════════════════════════════════════

class PyChecker:
    """Python 错误模式检测器"""

    def __init__(self, filepath: str, content: str, lines: list, fix: bool = False):
        self.filepath = filepath
        self.content = content
        self.lines = lines
        self.fix = fix
        self.findings: List[Finding] = []
        self._modified = False  # 标记是否被修改

    def run_all(self) -> List[Finding]:
        self.check_hardcoded_paths()           # E02
        self.check_top_level_no_guard()        # E03
        self.check_missing_import()            # E04, E15
        self.check_plt_show_no_agg()           # E38, E39
        self.check_np_trapz()                  # E37
        self.check_bare_except()               # E29
        self.check_npz_getattr()               # E27
        self.check_serial_close_in_loop()      # E28
        self.check_duplicate_import_in_func()  # E36
        self.check_conditional_import_usage()  # E30
        return self.findings

    # ─── E02: 硬编码绝对路径 ──────────────────────────────────────────
    def check_hardcoded_paths(self):
        """检测硬编码的绝对路径"""
        patterns = [
            re.compile(r'''(?:['"])[A-Za-z]:\\[^\s'"]*'''),   # Windows: "D:\..."
            re.compile(r'''(?:['"])[A-Za-z]:/[^\s'"]*'''),    # Windows: "D:/..."
            re.compile(r'''(?:['"])/home/\w+/[^\s'"]*'''),    # Linux: "/home/user/..."
            re.compile(r'''(?:['"])/Users/\w+/[^\s'"]*'''),   # macOS: "/Users/user/..."
        ]
        for i, line in enumerate(self.lines, 1):
            stripped = line.strip()
            if stripped.startswith('#'):
                continue
            for pat in patterns:
                if pat.search(line):
                    self.findings.append(Finding(
                        rule_id='E02', rule_name='硬编码绝对路径',
                        severity='warning',
                        file=self.filepath, line=i,
                        message=f'硬编码绝对路径，应使用相对路径或os.path.dirname(__file__)构造',
                        auto_fixable=False
                    ))
                    break  # 一行只报一次

    # ─── E03: 顶层执行无守卫 ─────────────────────────────────────────
    def check_top_level_no_guard(self):
        """检测模块顶层是否有未守卫的执行代码"""
        has_guard = False
        has_simulation_code = False
        sim_line = 0

        for i, line in enumerate(self.lines, 1):
            if "__name__" in line and "__main__" in line:
                has_guard = True
            stripped = line.strip()
            # 检测顶层仿真代码
            if not has_guard and i > 5:
                if any(kw in stripped for kw in ['plt.show()', 'plt.plot(', 'plt.figure(',
                                                   'plt.savefig(', 'plt.subplot(',
                                                   'simulator.run(', '.simulate(']):
                    if not stripped.startswith('#') and not stripped.startswith('def '):
                        has_simulation_code = True
                        if sim_line == 0:
                            sim_line = i

        if has_simulation_code and not has_guard:
            self.findings.append(Finding(
                rule_id='E03', rule_name='顶层执行无守卫',
                severity='warning',
                file=self.filepath, line=sim_line,
                message='仿真代码在模块顶层执行，import即触发。应封装为函数+`if __name__=="__main__"`守卫',
                auto_fixable=False
            ))

    # ─── E04/E15: 缺失import ──────────────────────────────────────────
    def check_missing_import(self):
        """检测使用了np/plt但未import"""
        # 跳过审计工具自身(它们以字符串形式引用np/plt模式)
        if os.path.basename(self.filepath) in ('audit_checker.py', 'batch_fix_critical.py'):
            return
        # 排除字符串字面量和注释中的 np./plt. 引用
        # 移除所有字符串内容后再检测
        content_no_strings = re.sub(r'"[^"]*"', '""', self.content)
        content_no_strings = re.sub(r"'[^']*'", "''", content_no_strings)
        # 移除注释
        content_no_strings = re.sub(r'//.*$', '', content_no_strings, flags=re.MULTILINE)
        content_no_strings = re.sub(r'/\*.*?\*/', '', content_no_strings, flags=re.DOTALL)
        uses_np = bool(re.search(r'\bnp\.\w+', content_no_strings))
        uses_plt = bool(re.search(r'\bplt\.\w+', content_no_strings))

        imports_np = bool(re.search(r'^\s*(?:import\s+numpy|from\s+numpy)', self.content, re.MULTILINE))
        imports_plt = bool(re.search(r'^\s*(?:import\s+matplotlib|from\s+matplotlib)', self.content, re.MULTILINE))

        if uses_np and not imports_np:
            self.findings.append(Finding(
                rule_id='E04', rule_name='缺失import',
                severity='critical',
                file=self.filepath, line=1,
                message='使用了 `np.xxx` 但未 `import numpy as np`',
                auto_fixable=True
            ))
        if uses_plt and not imports_plt:
            self.findings.append(Finding(
                rule_id='E04', rule_name='缺失import',
                severity='critical',
                file=self.filepath, line=1,
                message='使用了 `plt.xxx` 但未 `import matplotlib.pyplot as plt`',
                auto_fixable=True
            ))

    # ─── E38/E39: plt.show() + Agg后端 ────────────────────────────────
    def check_plt_show_no_agg(self):
        """检测plt.show()在Agg模式下的问题，以及matplotlib.use()位置"""
        has_agg = bool(re.search(r"matplotlib\.use\(['\"]Agg['\"]\)", self.content))
        has_plt_show = bool(re.search(r'plt\.show\(\)', self.content))
        has_import_plt = bool(re.search(r'^\s*import\s+matplotlib\.pyplot\s+as\s+plt', self.content, re.MULTILINE))
        has_use = bool(re.search(r'matplotlib\.use\(', self.content))

        if has_agg and has_plt_show:
            for i, line in enumerate(self.lines, 1):
                if 'plt.show()' in line and not line.strip().startswith('#'):
                    self.findings.append(Finding(
                        rule_id='E38', rule_name='plt.show()无Agg后端',
                        severity='warning',
                        file=self.filepath, line=i,
                        message='Agg无头模式下调用plt.show()可能阻塞或产生警告，应仅使用plt.savefig()',
                        auto_fixable=True
                    ))

        # E39: matplotlib.use()必须在import pyplot之前
        if has_use and has_import_plt:
            use_line = 0
            import_line = 0
            for i, line in enumerate(self.lines, 1):
                if 'matplotlib.use(' in line and not line.strip().startswith('#'):
                    use_line = i
                if re.match(r'\s*import\s+matplotlib\.pyplot\s+as\s+plt', line):
                    import_line = i
            if use_line > import_line > 0:
                self.findings.append(Finding(
                    rule_id='E39', rule_name='matplotlib.use()位置错误',
                    severity='critical',
                    file=self.filepath, line=use_line,
                    message=f'matplotlib.use("Agg")在第{use_line}行，但import pyplot在第{import_line}行，后端已初始化，use()无效',
                    auto_fixable=True
                ))

    # ─── E37: np.trapz等旧API ─────────────────────────────────────────
    def check_np_trapz(self):
        """检测numpy已移除的API (使用正则词边界匹配，避免误报np.float32等合法类型)"""
        # 使用正则确保精确匹配: np.float 不匹配 np.float32, np.float_, np.floating
        deprecated_patterns = [
            (re.compile(r'\bnp\.trapz\b'), 'np.trapz', 'np.trapezoid'),
            (re.compile(r'\bnp\.float(?![0-9_e])'), 'np.float', 'float (builtin)'),
            (re.compile(r'\bnp\.int(?![0-9_cep])'), 'np.int', 'int (builtin)'),
            (re.compile(r'\bnp\.bool(?!_)'), 'np.bool', 'bool (builtin)'),
            (re.compile(r'\bnp\.complex(?![0-9_])'), 'np.complex', 'complex (builtin)'),
            (re.compile(r'\bnp\.object(?!_)'), 'np.object', 'object (builtin)'),
            (re.compile(r'\bnp\.str(?!_)'), 'np.str', 'str (builtin)'),
        ]
        for i, line in enumerate(self.lines, 1):
            stripped = line.strip()
            if stripped.startswith('#'):
                continue
            # 移除字符串字面量后再检测
            line_no_str = re.sub(r'"[^"]*"', '""', line)
            line_no_str = re.sub(r"'[^']*'", "''", line_no_str)
            for pat, old_api, new_api in deprecated_patterns:
                if pat.search(line_no_str) and not stripped.startswith('#'):
                    # 排除兼容shim模式
                    is_shim = (
                        f'{old_api} = ' in line or
                        f'{old_api}=' in line or
                        f'= {old_api}' in line or
                        f'= {old_api} ' in line or
                        f'hasattr(np' in line or
                        'trapezoid' in line
                    )
                    if not is_shim:
                        self.findings.append(Finding(
                            rule_id='E37', rule_name='numpy旧API',
                            severity='critical' if 'trapz' in old_api else 'warning',
                            file=self.filepath, line=i,
                            message=f'`{old_api}` 在numpy 2.x中已移除，替代为`{new_api}`',
                            auto_fixable=True
                        ))

    # ─── E29: bare except ─────────────────────────────────────────────
    def check_bare_except(self):
        """检测bare except: (会吞掉KeyboardInterrupt等)"""
        for i, line in enumerate(self.lines, 1):
            if re.match(r'\s*except\s*:', line):
                self.findings.append(Finding(
                    rule_id='E29', rule_name='bare except',
                    severity='warning',
                    file=self.filepath, line=i,
                    message='bare `except:` 会吞掉KeyboardInterrupt、SystemExit等关键异常，应指定具体异常类型',
                    auto_fixable=False
                ))

    # ─── E27: npz getattr读取失败 ─────────────────────────────────────
    def check_npz_getattr(self):
        """检测npz加载后用getattr读取数据"""
        if re.search(r'getattr\s*\(\s*self\s*,', self.content):
            # 检查附近是否有np.load
            if re.search(r'np\.load\s*\(', self.content):
                for i, line in enumerate(self.lines, 1):
                    if 'getattr(self,' in line or 'getattr(self, ' in line:
                        self.findings.append(Finding(
                            rule_id='E27', rule_name='npz getattr读取失败',
                            severity='critical',
                            file=self.filepath, line=i,
                            message='npz加载的数据在data字典中，getattr(self, ...)永远返回None',
                            auto_fixable=False
                        ))

    # ─── E28: 串口close()在循环内 ─────────────────────────────────────
    def check_serial_close_in_loop(self):
        """检测串口/文件close()是否在循环体内"""
        in_loop = 0
        for i, line in enumerate(self.lines, 1):
            if re.search(r'\b(?:while|for)\b', line):
                in_loop += 1
            if re.search(r'\b(?:ser|serial|port)\.close\(\)', line):
                if in_loop > 0:
                    self.findings.append(Finding(
                        rule_id='E28', rule_name='串口close()在循环内',
                        severity='critical',
                        file=self.filepath, line=i,
                        message='串口.close()在循环内部，后续迭代将无法读取数据',
                        auto_fixable=False
                    ))
            if 'break' in line or (in_loop > 0 and re.search(r'^\s{0,4}(?:return|continue)', line)):
                pass  # 不减少计数，简化处理

    # ─── E36: 函数内重复import ────────────────────────────────────────
    def check_duplicate_import_in_func(self):
        """检测函数内重复import已顶层导入的模块"""
        top_imports = set()
        for line in self.lines:
            m = re.match(r'\s*import\s+(\w+)', line)
            if m:
                top_imports.add(m.group(1))
            m = re.match(r'\s*from\s+(\w+)', line)
            if m:
                top_imports.add(m.group(1))

        # 检查函数内的import
        in_func = 0
        for i, line in enumerate(self.lines, 1):
            if re.match(r'\s*def\s+\w+', line):
                in_func += 1
            if in_func > 0:
                m = re.match(r'\s*import\s+(\w+)', line)
                if m and m.group(1) in top_imports:
                    self.findings.append(Finding(
                        rule_id='E36', rule_name='函数内重复import',
                        severity='warning',
                        file=self.filepath, line=i,
                        message=f'函数内`import {m.group(1)}`与顶层重复，可能导致UnboundLocalError',
                        auto_fixable=False
                    ))

    # ─── E30: 条件导入的库在共享函数中直接使用 ──────────────────────────
    def check_conditional_import_usage(self):
        """检测条件导入(HAS_NUMPY等守卫)的库在非守卫函数中直接使用"""
        # 查找 HAS_XXX 守卫模式
        has_guards = re.findall(r'HAS_(\w+)', self.content)
        if not has_guards:
            return

        # 查找在守卫外使用 np.xxx 等的代码
        guard_name = 'HAS_' + has_guards[0]
        in_guard = False
        guard_indent = 0
        for i, line in enumerate(self.lines, 1):
            if guard_name in line:
                in_guard = True
                guard_indent = len(line) - len(line.lstrip())
                continue
            if in_guard:
                indent = len(line) - len(line.lstrip())
                if indent <= guard_indent and line.strip():
                    in_guard = False

            # 在守卫外使用np
            if not in_guard and re.search(r'\bnp\.\w+', line) and not line.strip().startswith('#'):
                self.findings.append(Finding(
                    rule_id='E30', rule_name='条件导入库未守卫使用',
                    severity='warning',
                    file=self.filepath, line=i,
                    message='条件导入的numpy在守卫外直接使用，未导入时将NameError',
                    auto_fixable=False
                ))
                break  # 只报一次


# ═══════════════════════════════════════════════════════════════════════
# 自动修复引擎
# ═══════════════════════════════════════════════════════════════════════

class AutoFixer:
    """自动修复器 — 仅修复安全的、确定性的模式"""

    @staticmethod
    def fix_py(filepath: str, findings: List[Finding]) -> int:
        """修复Python文件中可自动修复的问题，返回修复数量"""
        fixed_count = 0
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        original = content

        # E04: 添加缺失的import numpy
        if any(f.rule_id == 'E04' and 'numpy' in f.message and not f.fixed for f in findings):
            if re.search(r'\bnp\.\w+', content) and not re.search(r'^\s*(?:import\s+numpy|from\s+numpy)', content, re.MULTILINE):
                content = 'import numpy as np\n' + content
                for f in findings:
                    if f.rule_id == 'E04' and 'numpy' in f.message:
                        f.fixed = True
                        fixed_count += 1

        # E04: 添加缺失的import matplotlib
        if any(f.rule_id == 'E04' and 'matplotlib' in f.message and not f.fixed for f in findings):
            if re.search(r'\bplt\.\w+', content) and not re.search(r'^\s*(?:import\s+matplotlib|from\s+matplotlib)', content, re.MULTILINE):
                content = 'import matplotlib\nmatplotlib.use(\'Agg\')\nimport matplotlib.pyplot as plt\n' + content
                for f in findings:
                    if f.rule_id == 'E04' and 'matplotlib' in f.message:
                        f.fixed = True
                        fixed_count += 1

        # E37: np.trapz → np.trapezoid
        if any(f.rule_id == 'E37' and 'trapz' in f.message and not f.fixed for f in findings):
            # 添加兼容shim
            shim = '\n# numpy 2.x 兼容\nif not hasattr(np, \'trapz\'):\n    np.trapz = np.trapezoid\n'
            if 'np.trapz' in content and 'np.trapz = np.trapezoid' not in content:
                # 在import numpy之后插入shim
                content = re.sub(
                    r'(import\s+numpy\s+as\s+np)',
                    r'\1' + shim,
                    content, count=1
                )
                for f in findings:
                    if f.rule_id == 'E37' and 'trapz' in f.message:
                        f.fixed = True
                        fixed_count += 1

        # E38: 删除plt.show() (在Agg模式下)
        if any(f.rule_id == 'E38' and not f.fixed for f in findings):
            if re.search(r"matplotlib\.use\(['\"]Agg['\"]\)", content):
                content = re.sub(r'\n\s*plt\.show\(\)\s*\n', '\n', content)
                for f in findings:
                    if f.rule_id == 'E38':
                        f.fixed = True
                        fixed_count += 1

        # E39: 把matplotlib.use('Agg')移到import pyplot之前
        if any(f.rule_id == 'E39' and not f.fixed for f in findings):
            lines = content.split('\n')
            use_line_idx = None
            import_plt_idx = None
            import_mpl_idx = None
            use_line_text = None
            for idx, line in enumerate(lines):
                if 'matplotlib.use(' in line and not line.strip().startswith('#'):
                    use_line_idx = idx
                    use_line_text = line
                if re.match(r'\s*import\s+matplotlib\.pyplot\s+as\s+plt', line):
                    import_plt_idx = idx
                if re.match(r'\s*import\s+matplotlib\s*$', line):
                    import_mpl_idx = idx
            if use_line_idx is not None and import_plt_idx is not None and use_line_idx > import_plt_idx:
                # 删除原位置的use行
                del lines[use_line_idx]
                # 在import matplotlib之后、import pyplot之前插入
                insert_idx = import_mpl_idx + 1 if import_mpl_idx is not None else import_plt_idx
                lines.insert(insert_idx, use_line_text)
                content = '\n'.join(lines)
                for f in findings:
                    if f.rule_id == 'E39':
                        f.fixed = True
                        fixed_count += 1

        if content != original:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(content)

        return fixed_count


# ═══════════════════════════════════════════════════════════════════════
# 主扫描引擎
# ═══════════════════════════════════════════════════════════════════════

def scan_directory(root: str, fix: bool = False) -> AuditReport:
    """扫描目录，返回审计报告"""
    report = AuditReport()
    report.scan_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    report.scan_root = os.path.abspath(root)
    all_findings: List[Finding] = []

    # 跳过的目录
    skip_dirs = {'.git', '__pycache__', 'node_modules', '.vs', 'build', 'output', 'templates'}

    c_count = 0
    py_count = 0

    for dirpath, dirnames, filenames in os.walk(root):
        # 跳过特殊目录
        dirnames[:] = [d for d in dirnames if d not in skip_dirs]

        for fname in filenames:
            fpath = os.path.join(dirpath, fname)
            rel_path = os.path.relpath(fpath, root)

            try:
                if fname.endswith('.c') or fname.endswith('.h'):
                    c_count += 1
                    with open(fpath, 'r', encoding='utf-8', errors='ignore') as f:
                        content = f.read()
                    lines = content.split('\n')
                    checker = CChecker(rel_path, content, lines, fix=fix)
                    all_findings.extend(checker.run_all())

                elif fname.endswith('.py'):
                    py_count += 1
                    with open(fpath, 'r', encoding='utf-8', errors='ignore') as f:
                        content = f.read()
                    lines = content.split('\n')
                    checker = PyChecker(rel_path, content, lines, fix=fix)
                    findings = checker.run_all()
                    all_findings.extend(findings)

                    # 自动修复
                    if fix and findings:
                        fixable = [f for f in findings if f.auto_fixable]
                        if fixable:
                            fixed = AutoFixer.fix_py(fpath, findings)
                            report.auto_fixed += fixed

            except Exception as e:
                print(f"  [WARN] 无法读取 {rel_path}: {e}", file=sys.stderr)

    report.total_c_files = c_count
    report.total_py_files = py_count
    report.total_findings = len(all_findings)
    report.critical = sum(1 for f in all_findings if f.severity == 'critical')
    report.warning = sum(1 for f in all_findings if f.severity == 'warning')
    report.info = sum(1 for f in all_findings if f.severity == 'info')
    report.findings = [asdict(f) for f in all_findings]

    return report


def print_summary(report: AuditReport):
    """打印审计摘要"""
    print("=" * 60)
    print("电赛资产库 V2 深度审计 — 自动检测报告")
    print("=" * 60)
    print(f"扫描时间: {report.scan_time}")
    print(f"扫描目录: {report.scan_root}")
    print(f"C/H文件:  {report.total_c_files} 个")
    print(f"Python:   {report.total_py_files} 个")
    print("-" * 60)
    print(f"发现总数: {report.total_findings}")
    print(f"  严重:   {report.critical}")
    print(f"  警告:   {report.warning}")
    print(f"  信息:   {report.info}")
    if report.auto_fixed:
        print(f"自动修复: {report.auto_fixed} 处")
    print("-" * 60)

    # 按规则汇总
    from collections import Counter
    rule_counts = Counter(f['rule_id'] for f in report.findings)
    rule_names = {f['rule_id']: f['rule_name'] for f in report.findings}
    print("\n规则命中统计:")
    for rule_id, count in sorted(rule_counts.items()):
        print(f"  {rule_id} [{rule_names[rule_id]}]: {count} 处")
    print()


def main():
    parser = argparse.ArgumentParser(
        description='电赛资产库 V2 深度审计 — 错误模式自动检测脚本'
    )
    parser.add_argument('scan_dir', nargs='?', default='.',
                        help='扫描目录 (默认: 当前目录)')
    parser.add_argument('--fix', action='store_true',
                        help='自动修复安全的、确定性的问题')
    parser.add_argument('--report', '-o', default='audit_report.json',
                        help='输出报告文件 (默认: audit_report.json)')
    parser.add_argument('--quiet', '-q', action='store_true',
                        help='安静模式，仅输出JSON')
    args = parser.parse_args()

    scan_root = os.path.abspath(args.scan_dir)
    if not os.path.isdir(scan_root):
        print(f"错误: 目录不存在: {scan_root}", file=sys.stderr)
        sys.exit(1)

    report = scan_directory(scan_root, fix=args.fix)

    if not args.quiet:
        print_summary(report)

    # 写入JSON报告
    report_path = os.path.join(scan_root, args.report) if not os.path.isabs(args.report) else args.report
    os.makedirs(os.path.dirname(report_path) or '.', exist_ok=True)
    with open(report_path, 'w', encoding='utf-8') as f:
        json.dump(asdict(report), f, ensure_ascii=False, indent=2)

    if not args.quiet:
        print(f"报告已写入: {report_path}")

    # 返回码: 有critical则非0
    sys.exit(1 if report.critical > 0 else 0)


if __name__ == '__main__':
    main()
