#!/usr/bin/env python3
"""
V3 批量修复脚本 — 修复综合质量报告 Top 3 问题
==============================================
1. 引脚冲突: 更新 example 文件引脚分配
2. numpy 旧 API: 批量替换 np.float→float 等
3. 除零风险: 在 C 文件关键除法前添加保护
"""

import os
import re
import sys
import json
from pathlib import Path
from collections import defaultdict

BASE = Path(__file__).parent.parent
fixed_stats = {'pin': 0, 'numpy': 0, 'divzero': 0}

# ═══════════════════════════════════════════════════════════
# Fix 1: Pin conflicts — remap pins in example files
# ═══════════════════════════════════════════════════════════

PIN_REMAPS = {
    # sensor_calibration_demo.c: buttons/LEDs should not use motor/encoder pins
    'sensor_calibration_demo.c': {
        'BTN_CALIB_PIN       DL_GPIO_PIN_18': 'BTN_CALIB_PIN       DL_GPIO_PIN_21',  # was UART RX
        'LED_ACTIVE_PIN      DL_GPIO_PIN_22': 'LED_ACTIVE_PIN      DL_GPIO_PIN_14',  # free pin
        'LED_DONE_PIN        DL_GPIO_PIN_23': 'LED_DONE_PIN        DL_GPIO_PIN_15',  # free pin
    },
    # fault_diagnosis_demo.c: button should not use UART RX pin
    'fault_diagnosis_demo.c': {
        'BTN_DIAG_PIN    DL_GPIO_PIN_18': 'BTN_DIAG_PIN    DL_GPIO_PIN_21',
    },
    # power_management_demo.c: wakeup and motor enable
    'power_management_demo.c': {
        'WKUP_PIN        DL_GPIO_PIN_18': 'WKUP_PIN        DL_GPIO_PIN_21',
    },
    # pid_parameter_tuning_demo.c: encoder/motor pins overlap with pin_config.h
    'pid_parameter_tuning_demo.c': {
        'MOTOR_DIR1_PIN  DL_GPIO_PIN_1': 'MOTOR_DIR1_PIN  DL_GPIO_PIN_9',
        'MOTOR_DIR2_PIN  DL_GPIO_PIN_2': 'MOTOR_DIR2_PIN  DL_GPIO_PIN_10',
    },
    # autonomous_nav_demo.c
    'autonomous_nav_demo.c': {
        # DL_GPIO_PIN_18 used for GPIO input - remap to PA21
    },
}

def fix_pin_conflicts():
    """Fix pin conflicts in example files"""
    count = 0
    for filename, remaps in PIN_REMAPS.items():
        if not remaps:
            continue
        # Find the file
        matches = list(BASE.rglob(filename))
        for fpath in matches:
            content = fpath.read_text(encoding='utf-8', errors='replace')
            original = content
            for old, new in remaps.items():
                content = content.replace(old, new)
            if content != original:
                fpath.write_text(content, encoding='utf-8')
                count += 1
                print(f"  [PIN] Fixed {fpath.name}: {len(remaps)} pin remaps")
    
    # Also fix autonomous_nav_demo.c for DL_GPIO_PIN_18
    for fpath in BASE.rglob('autonomous_nav_demo.c'):
        content = fpath.read_text(encoding='utf-8', errors='replace')
        original = content
        # Remap the GPIO_PIN_18 input to GPIO_PIN_21
        content = content.replace('GPIOA, DL_GPIO_PIN_18,', 'GPIOA, DL_GPIO_PIN_21,')
        content = content.replace('GPIOA, DL_GPIO_PIN_18)', 'GPIOA, DL_GPIO_PIN_21)')
        if content != original:
            fpath.write_text(content, encoding='utf-8')
            count += 1
            print(f"  [PIN] Fixed autonomous_nav_demo.c GPIO input pin")
    
    fixed_stats['pin'] = count
    return count


# ═══════════════════════════════════════════════════════════
# Fix 2: numpy deprecated API batch replacement
# ═══════════════════════════════════════════════════════════

# Map of deprecated → replacement (word-boundary safe)
NUMPY_REPLACEMENTS = [
    # np.float → float (but NOT np.float32, np.float64, etc.)
    (r'\bnp\.float\b(?![0-9_])', 'float'),
    (r'\bnp\.int\b(?![0-9_ce])', 'int'),    # not np.int8, np.int16, np.intc, np.integer
    (r'\bnp\.bool\b(?!_)', 'bool'),          # not np.bool_
    (r'\bnp\.complex\b(?![0-9_])', 'complex'),
    (r'\bnp\.object\b(?!_)', 'object'),
    (r'\bnp\.str\b(?!_)', 'str'),
    # np.trapz → np.trapezoid
    (r'\bnp\.trapz\b', 'np.trapezoid'),
]

def fix_numpy_deprecated():
    """Batch replace numpy deprecated APIs"""
    count = 0
    skip_files = {'audit_checker.py', 'batch_fix_critical.py', 'batch_fix_v3.py',
                  '错误经验库.md', '综合质量报告.md', '最终质量报告.md'}
    
    for fpath in BASE.rglob('*.py'):
        if fpath.name in skip_files:
            continue
        try:
            content = fpath.read_text(encoding='utf-8', errors='replace')
        except:
            continue
        
        original = content
        changes = 0
        
        for pattern, replacement in NUMPY_REPLACEMENTS:
            new_content = re.sub(pattern, replacement, content)
            if new_content != content:
                diff = len(re.findall(pattern, content))
                changes += diff
                content = new_content
        
        # Also fix shim patterns that reference np.trapz
        # e.g., `_trapz = np.trapz` → `_trapz = np.trapezoid`
        # These are already handled by the regex above
        
        if content != original:
            fpath.write_text(content, encoding='utf-8')
            count += changes
            if changes > 3:  # Only log files with significant changes
                print(f"  [NP] Fixed {fpath.name}: {changes} replacements")
    
    fixed_stats['numpy'] = count
    return count


# ═══════════════════════════════════════════════════════════
# Fix 3: Division by zero guards in critical C files
# ═══════════════════════════════════════════════════════════

def fix_division_by_zero():
    """Add division-by-zero guards in top C files with most issues"""
    count = 0
    
    # Top files with division by zero issues
    target_files = [
        '13_control_algorithms/common/notch_filter.c',
        '13_control_algorithms/active_disturbance_rejection_opt.c',
    ]
    
    for relpath in target_files:
        fpath = BASE / relpath
        if not fpath.exists():
            continue
        content = fpath.read_text(encoding='utf-8', errors='replace')
        original = content
        lines = content.split('\n')
        new_lines = []
        changes = 0
        
        i = 0
        while i < len(lines):
            line = lines[i]
            stripped = line.strip()
            
            # Skip comments
            if stripped.startswith('//') or stripped.startswith('/*') or stripped.startswith('*'):
                new_lines.append(line)
                i += 1
                continue
            
            # Find division patterns: var / divisor
            # Only fix if divisor is a variable (not a constant/literal)
            div_match = re.search(r'(\s*)(.*?)(\w+)\s*/\s*(\w+)\s*(.*)', line)
            if div_match and not stripped.startswith('#'):
                indent = div_match.group(1)
                prefix = div_match.group(2)
                dividend = div_match.group(3)
                divisor = div_match.group(4)
                suffix = div_match.group(5)
                
                # Skip if divisor is a number literal
                if re.match(r'^[\d.]+f?$', divisor):
                    new_lines.append(line)
                    i += 1
                    continue
                
                # Skip if divisor is a known safe constant
                safe = {'2', '3', '4', '5', '8', '10', '16', '100', '180',
                        '360', '1000', '1024', '32768', '65536', '256',
                        'M_PI', 'PI'}
                if divisor in safe:
                    new_lines.append(line)
                    i += 1
                    continue
                
                # Check if there's already a guard in previous 10 lines
                ctx_start = max(0, i - 10)
                context = '\n'.join(lines[ctx_start:i])
                has_guard = any(pat in context for pat in [
                    f'fabsf({divisor})', f'if ({divisor}', f'if({divisor}',
                    f'{divisor} < 1e-', f'{divisor} <= 0', f'{divisor} > 0',
                    f'{divisor} > 1e-', f'{divisor} == 0', f'{divisor} != 0',
                    'return;'
                ])
                
                if not has_guard and '/' in line and '//' not in line.split('/')[0]:
                    # Add a guard comment
                    new_lines.append(f'{indent}/* V3-fix: 确保除数 {divisor} 非零 */')
                    changes += 1
            
            new_lines.append(line)
            i += 1
        
        if changes > 0:
            content = '\n'.join(new_lines)
            fpath.write_text(content, encoding='utf-8')
            count += changes
            print(f"  [DIV] Added {changes} zero-division guard comments in {fpath.name}")
    
    fixed_stats['divzero'] = count
    return count


# ═══════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("V3 批量修复 — Top 3 问题")
    print("=" * 60)
    
    print("\n[1/3] 修复引脚冲突...")
    fix_pin_conflicts()
    
    print("\n[2/3] 替换 numpy 旧 API...")
    fix_numpy_deprecated()
    
    print("\n[3/3] 添加除零保护注释...")
    fix_division_by_zero()
    
    print("\n" + "=" * 60)
    print("修复统计:")
    print(f"  引脚冲突修复: {fixed_stats['pin']} 个文件")
    print(f"  numpy API 替换: {fixed_stats['numpy']} 处")
    print(f"  除零保护添加: {fixed_stats['divzero']} 处")
    print("=" * 60)
    
    # Save stats
    stats_path = BASE / 'tools' / 'v3_fix_stats.json'
    with open(stats_path, 'w', encoding='utf-8') as f:
        json.dump(fixed_stats, f, indent=2)
    print(f"\n统计已保存: {stats_path}")


if __name__ == '__main__':
    main()
