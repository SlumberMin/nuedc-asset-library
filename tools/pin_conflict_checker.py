#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
引脚冲突检查器 - 电赛资产库工具
===================================
功能：扫描所有驱动的引脚分配配置，检测不同外设之间的引脚冲突
用法：
  python pin_conflict_checker.py                      # 扫描全部驱动
  python pin_conflict_checker.py --path ./drivers     # 指定扫描目录
  python pin_conflict_checker.py --verbose             # 显示详细信息
  python pin_conflict_checker.py --output report.md   # 输出到文件
  python pin_conflict_checker.py --mcu mspm0g3507     # 指定MCU型号

背景：错误经验库 #17 记录了编码器右轮引脚与超声波引脚完全重叠(PB6/PB7)的冲突问题
      本工具旨在自动检测此类冲突，避免硬件连接错误

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
# 已知MCU引脚映射表
# ============================================================

# MSPM0G3507引脚复用表（常见外设映射）
MSPM0G3507_PIN_MAP = {
    # GPIO端口定义
    'PA0': {'port': 'GPIOA', 'pin': 0},
    'PA1': {'port': 'GPIOA', 'pin': 1},
    'PA2': {'port': 'GPIOA', 'pin': 2},
    'PA3': {'port': 'GPIOA', 'pin': 3},
    'PA4': {'port': 'GPIOA', 'pin': 4},
    'PA5': {'port': 'GPIOA', 'pin': 5},
    'PA6': {'port': 'GPIOA', 'pin': 6},
    'PA7': {'port': 'GPIOA', 'pin': 7},
    'PA8': {'port': 'GPIOA', 'pin': 8},
    'PA9': {'port': 'GPIOA', 'pin': 9},
    'PA10': {'port': 'GPIOA', 'pin': 10},
    'PA11': {'port': 'GPIOA', 'pin': 11},
    'PA12': {'port': 'GPIOA', 'pin': 12},
    'PA13': {'port': 'GPIOA', 'pin': 13},
    'PA14': {'port': 'GPIOA', 'pin': 14},
    'PA15': {'port': 'GPIOA', 'pin': 15},
    'PA16': {'port': 'GPIOA', 'pin': 16},
    'PA17': {'port': 'GPIOA', 'pin': 17},
    'PA18': {'port': 'GPIOA', 'pin': 18},
    'PA19': {'port': 'GPIOA', 'pin': 19},
    'PA20': {'port': 'GPIOA', 'pin': 20},
    'PA21': {'port': 'GPIOA', 'pin': 21},
    'PA22': {'port': 'GPIOA', 'pin': 22},
    'PA23': {'port': 'GPIOA', 'pin': 23},
    'PA24': {'port': 'GPIOA', 'pin': 24},
    'PA25': {'port': 'GPIOA', 'pin': 25},
    'PA26': {'port': 'GPIOA', 'pin': 26},
    'PA27': {'port': 'GPIOA', 'pin': 27},
    'PB0': {'port': 'GPIOB', 'pin': 0},
    'PB1': {'port': 'GPIOB', 'pin': 1},
    'PB2': {'port': 'GPIOB', 'pin': 2},
    'PB3': {'port': 'GPIOB', 'pin': 3},
    'PB4': {'port': 'GPIOB', 'pin': 4},
    'PB5': {'port': 'GPIOB', 'pin': 5},
    'PB6': {'port': 'GPIOB', 'pin': 6},
    'PB7': {'port': 'GPIOB', 'pin': 7},
    'PB8': {'port': 'GPIOB', 'pin': 8},
    'PB9': {'port': 'GPIOB', 'pin': 9},
    'PB10': {'port': 'GPIOB', 'pin': 10},
    'PB11': {'port': 'GPIOB', 'pin': 11},
    'PB12': {'port': 'GPIOB', 'pin': 12},
    'PB13': {'port': 'GPIOB', 'pin': 13},
    'PB14': {'port': 'GPIOB', 'pin': 14},
    'PB15': {'port': 'GPIOB', 'pin': 15},
}

# 常见引脚用途模式
PIN_ROLE_PATTERNS = {
    'encoder': ['ENC', 'ENCODER', 'A相', 'B相', 'ABZ'],
    'motor': ['MOTOR', 'PWM', 'IN1', 'IN2', 'ENA', 'ENB'],
    'ultrasonic': ['TRIG', 'ECHO', 'ULTRA'],
    'oled': ['SCL', 'SDA', 'OLED'],
    'uart': ['TX', 'RX', 'UART', 'USART', 'TXD', 'RXD'],
    'adc': ['ADC', 'AD_'],
    'spi': ['MOSI', 'MISO', 'SCK', 'CS', 'SPI'],
    'led': ['LED', 'D1', 'D2'],
    'key': ['KEY', 'BUTTON', 'SW'],
    'sensor': ['SENSOR', 'GYRO', 'ACCEL', 'MPU'],
}


# ============================================================
# 引脚分配解析器
# ============================================================

class PinConfigParser:
    """
    引脚配置解析器
    
    从C头文件/源文件中提取引脚分配信息
    支持多种常见格式：
    - #define MOTOR_IN1_PORT GPIO_PORT_A
    - #define MOTOR_IN1_PIN  12
    - GPIO_setPin(PA12)
    - 引脚配置表注释
    """

    def __init__(self):
        """初始化解析正则"""
        
        # 格式1: #define XXX_PORT GPIO_PORT_X 或 GPIOX
        self.re_port = re.compile(
            r'#define\s+(\w+?)_(?:PORT|GPIO)\s+(?:GPIO_)?(?:PORT_)?([AB])',
            re.IGNORECASE
        )
        
        # 格式2: #define XXX_PIN  NN
        self.re_pin_num = re.compile(
            r'#define\s+(\w+?)_PIN\s+(\d+)',
            re.IGNORECASE
        )
        
        # 格式3: #define XXX PXX  (如 #define LED1 PA5)
        self.re_pin_def = re.compile(
            r'#define\s+(\w+)\s+P([AB])(\d+)',
            re.IGNORECASE
        )
        
        # 格式4: DL_GPIO_PIN_xx (MSPM0 SysConfig风格)
        self.re_dl_pin = re.compile(
            r'#define\s+(\w+)\s+DL_GPIO_PIN_(\d+)',
            re.IGNORECASE
        )
        
        # 格式5: 引脚配置表注释 /* PA12 - Motor_IN1 */
        self.re_pin_comment = re.compile(
            r'P([AB])(\d+)\s*[-–]\s*(\w[\w\s]*)',
            re.IGNORECASE
        )
        
        # 格式6: 引脚映射表行 (如 "PA12", "PB6", "GPIO_PORT_A, 12")
        self.re_gpio_call = re.compile(
            r'GPIO_setPin\w*\s*\(\s*(?:GPIO_PORT_([AB])\s*,\s*(\d+)|P([AB])(\d+))\s*\)',
            re.IGNORECASE
        )
        
        # 格式7: 直接的引脚编号定义  #define XXX_PA12  ...
        self.re_embedded_pin = re.compile(
            r'#define\s+\w*_?(P[AB]\d+)\w*\s',
            re.IGNORECASE
        )

    def parse_file(self, filepath):
        """
        解析单个文件的引脚配置
        
        返回:
            list[dict]: 引脚分配列表，每个元素包含 {
                'pin': 'PA12',
                'usage': 'MOTOR_IN1',
                'module': 'motor',
                'file': 'pin_config.h',
                'line': 42
            }
        """
        assignments = []
        
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
                return assignments
            
            filename = os.path.basename(filepath)
            module_name = self._guess_module(filename, content)
            lines = content.split('\n')
            
            # 收集 PORT+PIN 配对
            port_defs = {}  # prefix -> port_letter
            pin_defs = {}   # prefix -> pin_number
            
            for i, line in enumerate(lines):
                # 跳过注释行
                stripped = line.strip()
                if stripped.startswith('//') or stripped.startswith('/*'):
                    # 但检查引脚注释
                    for m in self.re_pin_comment.finditer(stripped):
                        port = m.group(1).upper()
                        num = int(m.group(2))
                        usage = m.group(3).strip()
                        assignments.append({
                            'pin': f'P{port}{num}',
                            'usage': usage,
                            'module': module_name,
                            'file': filename,
                            'line': i + 1
                        })
                    continue
                
                # 格式1: PORT定义
                for m in self.re_port.finditer(line):
                    prefix = m.group(1)
                    port = m.group(2).upper()
                    port_defs[prefix] = port
                
                # 格式2: PIN编号定义
                for m in self.re_pin_num.finditer(line):
                    prefix = m.group(1)
                    num = int(m.group(2))
                    pin_defs[prefix] = num
                
                # 格式3: 直接引脚定义 PXX
                for m in self.re_pin_def.finditer(line):
                    usage = m.group(1)
                    port = m.group(2).upper()
                    num = int(m.group(3))
                    assignments.append({
                        'pin': f'P{port}{num}',
                        'usage': usage,
                        'module': module_name,
                        'file': filename,
                        'line': i + 1
                    })
                
                # 格式4: DL_GPIO_PIN
                for m in self.re_dl_pin.finditer(line):
                    usage = m.group(1)
                    pin_num = int(m.group(2))
                    # DL_GPIO_PIN通常映射到特定端口
                    assignments.append({
                        'pin': f'DL_GPIO_PIN_{pin_num}',
                        'usage': usage,
                        'module': module_name,
                        'file': filename,
                        'line': i + 1,
                        'ambiguous': True  # 标记为需人工确认
                    })
                
                # 格式6: GPIO_setPin调用
                for m in self.re_gpio_call.finditer(line):
                    if m.group(1):
                        port = m.group(1).upper()
                        num = int(m.group(2))
                    else:
                        port = m.group(3).upper()
                        num = int(m.group(4))
                    assignments.append({
                        'pin': f'P{port}{num}',
                        'usage': f'GPIO_{port}{num}',
                        'module': module_name,
                        'file': filename,
                        'line': i + 1
                    })
            
            # 合并 PORT+PIN 配对
            for prefix in port_defs:
                if prefix in pin_defs:
                    port = port_defs[prefix]
                    num = pin_defs[prefix]
                    # 检查是否已添加
                    pin_str = f'P{port}{num}'
                    already = any(a['pin'] == pin_str and a['usage'] == prefix for a in assignments)
                    if not already:
                        assignments.append({
                            'pin': pin_str,
                            'usage': prefix,
                            'module': module_name,
                            'file': filename,
                            'line': 0
                        })
            
            return assignments
            
        except Exception as e:
            print(f"  [警告] 解析 {filepath} 失败: {e}", file=sys.stderr)
            return assignments

    def _guess_module(self, filename, content):
        """根据文件名和内容猜测所属模块"""
        name_lower = filename.lower()
        content_lower = content[:2000].lower() if content else ''
        
        module_keywords = {
            'encoder': ['encoder', '编码器'],
            'motor': ['motor', '电机', 'hbridge'],
            'ultrasonic': ['ultrasonic', '超声波', 'hcsr04'],
            'oled': ['oled', 'ssd1306'],
            'uart': ['uart', 'serial', '串口'],
            'adc': ['adc', 'analog'],
            'spi': ['spi'],
            'i2c': ['i2c', 'oled', 'mpu', 'tcs'],
            'led': ['led'],
            'key': ['key', 'button'],
            'sensor': ['sensor', 'mpu', 'jy901', 'gyro'],
            'pwm': ['pwm', 'timer'],
            'bluetooth': ['bluetooth', 'ble', '蓝牙'],
            'gray': ['gray', '灰度'],
        }
        
        for module, keywords in module_keywords.items():
            for kw in keywords:
                if kw in name_lower or kw in content_lower:
                    return module
        
        return os.path.splitext(filename)[0]


# ============================================================
# 冲突检测引擎
# ============================================================

class ConflictDetector:
    """
    引脚冲突检测器
    
    检测逻辑：
    1. 精确冲突：同一引脚(PXn)被两个不同模块使用
    2. 端口冲突：同一端口的不同用途可能干扰（如I2C和GPIO同时使用同一端口）
    3. 已知冲突模式：基于错误经验库的已知冲突
    """

    def __init__(self):
        """初始化冲突检测器"""
        
        # 已知冲突模式（来自错误经验库）
        self.known_conflicts = [
            {
                'modules': ['encoder', 'ultrasonic'],
                'pins': ['PB6', 'PB7'],
                'description': '编码器右轮PB6/PB7与超声波Trig/Echo冲突',
                'reference': '错误经验库 #17'
            },
            {
                'modules': ['gray', 'encoder'],
                'pins': ['PB0', 'PB1', 'PB2', 'PB3', 'PB4', 'PB5'],
                'description': '灰度传感器与编码器共用PB0~PB5',
                'reference': 'pin_config.h标准分配'
            },
            {
                'modules': ['gray', 'ultrasonic'],
                'pins': ['PB6', 'PB7'],
                'description': '灰度传感器与超声波共用PB6~PB7',
                'reference': 'pin_config.h标准分配'
            }
        ]

    def detect(self, all_assignments):
        """
        检测引脚冲突
        
        参数:
            all_assignments: 所有文件的引脚分配列表
            
        返回:
            dict: {
                'conflicts': [...],      # 确定冲突
                'warnings': [...],       # 潜在冲突/警告
                'pin_usage': {...}       # 引脚使用汇总
            }
        """
        result = {
            'conflicts': [],
            'warnings': [],
            'pin_usage': defaultdict(list)
        }
        
        # 分离驱动/库文件与独立示例文件
        # 独立示例文件互不冲突（不会同时运行），只需检查驱动层冲突
        driver_keywords = {'pin_config', 'driver', 'config', 'hal', 'bsp'}
        example_keywords = {'demo', 'example', 'test', 'sample'}
        
        def is_example(assignment):
            fname = assignment.get('file', '').lower()
            return any(kw in fname for kw in example_keywords)
        
        # 按引脚分组（仅驱动层文件）
        driver_assignments = [a for a in all_assignments if not is_example(a)]
        example_assignments = [a for a in all_assignments if is_example(a)]
        
        for assignment in driver_assignments:
            pin = assignment['pin']
            result['pin_usage'][pin].append(assignment)
        
        # 检测驱动层精确冲突（真实硬件冲突）
        for pin, usages in result['pin_usage'].items():
            if len(usages) > 1:
                modules = set(u['module'] for u in usages)
                if len(modules) > 1:
                    conflict = {
                        'type': 'exact',
                        'pin': pin,
                        'severity': 'ERROR',
                        'modules': list(modules),
                        'usages': usages,
                        'description': f"引脚 {pin} 被 {len(modules)} 个模块同时使用: {', '.join(modules)}"
                    }
                    result['conflicts'].append(conflict)
                elif len(usages) > 2:
                    result['warnings'].append({
                        'type': 'duplicate',
                        'pin': pin,
                        'severity': 'WARNING',
                        'module': usages[0]['module'],
                        'description': f"引脚 {pin} 在模块 {usages[0]['module']} 中重复定义 {len(usages)} 次"
                    })
        
        # 检测示例文件间的冲突（降级为WARNING，因为示例不会同时运行）
        example_pin_usage = defaultdict(list)
        for assignment in example_assignments:
            example_pin_usage[assignment['pin']].append(assignment)
        for pin, usages in example_pin_usage.items():
            modules = set(u['module'] for u in usages)
            if len(modules) > 1:
                result['warnings'].append({
                    'type': 'example_overlap',
                    'pin': pin,
                    'severity': 'WARNING',
                    'modules': list(modules),
                    'description': f"示例文件引脚 {pin} 被 {len(modules)} 个模块使用: {', '.join(modules)} (独立示例，互不影响)"
                })
        
        # 检测已知冲突模式
        for known in self.known_conflicts:
            involved_modules = set()
            for assignment in all_assignments:
                if assignment['pin'] in known['pins']:
                    if assignment['module'] in known['modules']:
                        involved_modules.add(assignment['module'])
            
            if len(involved_modules) > 1:
                result['warnings'].append({
                    'type': 'known_pattern',
                    'pins': known['pins'],
                    'severity': 'WARNING',
                    'modules': list(involved_modules),
                    'description': known['description'],
                    'reference': known['reference']
                })
        
        # 检测DL_GPIO_PIN模糊引脚
        ambiguous = [a for a in all_assignments if a.get('ambiguous')]
        if ambiguous:
            result['warnings'].append({
                'type': 'ambiguous',
                'severity': 'INFO',
                'count': len(ambiguous),
                'description': f"发现 {len(ambiguous)} 个DL_GPIO_PIN定义，需人工确认端口映射"
            })
        
        return result


# ============================================================
# 报告生成器
# ============================================================

def generate_report(check_result, all_assignments, verbose=False):
    """
    生成冲突检查报告（Markdown格式）
    
    参数:
        check_result: ConflictDetector.detect()的返回值
        all_assignments: 所有引脚分配列表
        verbose: 是否显示详细信息
    """
    lines = []
    
    lines.append('# 引脚冲突检查报告')
    lines.append('')
    lines.append(f'> 生成时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
    lines.append(f'> 扫描文件数: {len(set(a["file"] for a in all_assignments))}')
    lines.append(f'> 引脚分配总数: {len(all_assignments)}')
    lines.append('')
    
    # 摘要
    conflicts = check_result['conflicts']
    warnings = check_result['warnings']
    
    if not conflicts and not warnings:
        lines.append('## ✅ 未检测到引脚冲突')
        lines.append('')
        lines.append('所有引脚分配互不冲突。')
    else:
        if conflicts:
            lines.append(f'## ❌ 检测到 {len(conflicts)} 个引脚冲突')
            lines.append('')
            for i, c in enumerate(conflicts, 1):
                lines.append(f'### 冲突 {i}: {c["pin"]}')
                lines.append('')
                lines.append(f'**严重程度:** {c["severity"]}')
                lines.append(f'**描述:** {c["description"]}')
                lines.append('')
                lines.append('| 文件 | 模块 | 用途 | 行号 |')
                lines.append('|------|------|------|------|')
                for u in c['usages']:
                    lines.append(f'| {u["file"]} | {u["module"]} | {u["usage"]} | {u["line"]} |')
                lines.append('')
        
        if warnings:
            lines.append(f'## ⚠️ {len(warnings)} 条警告')
            lines.append('')
            for w in warnings:
                lines.append(f'- **{w["type"].upper()}**: {w["description"]}')
                if w.get('reference'):
                    lines.append(f'  - 参考: {w["reference"]}')
            lines.append('')
    
    # 引脚使用汇总表
    if verbose:
        lines.append('## 引脚使用汇总')
        lines.append('')
        lines.append('| 引脚 | 使用模块数 | 模块列表 |')
        lines.append('|------|-----------|----------|')
        
        for pin in sorted(check_result['pin_usage'].keys()):
            usages = check_result['pin_usage'][pin]
            modules = set(u['module'] for u in usages)
            conflict_mark = ' ❌' if len(modules) > 1 else ''
            lines.append(f'| {pin} | {len(usages)} | {", ".join(modules)}{conflict_mark} |')
        lines.append('')
    
    return '\n'.join(lines)


# ============================================================
# 主程序入口
# ============================================================

def main():
    """命令行入口函数"""
    parser = argparse.ArgumentParser(
        description='引脚冲突检查器 - 扫描所有驱动的引脚分配，检测冲突',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                                    # 扫描资产库全部驱动
  %(prog)s --path ./drivers --verbose         # 指定目录，显示详细信息
  %(prog)s --output pin_report.md             # 保存报告到文件
  %(prog)s --json                             # 输出JSON格式
        """
    )
    
    parser.add_argument('--path', '-p', type=str, default=None,
                        help='扫描目录路径（默认：资产库根目录）')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='输出报告文件路径')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='显示详细信息')
    parser.add_argument('--json', '-j', action='store_true',
                        help='输出JSON格式')
    parser.add_argument('--mcu', type=str, default='mspm0g3507',
                        help='MCU型号（默认：mspm0g3507）')
    
    args = parser.parse_args()
    
    # 确定扫描路径
    scan_dir = args.path
    if scan_dir is None:
        scan_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # JSON模式下进度信息输出到stderr
    log = (lambda msg: print(msg, file=sys.stderr)) if args.json else print
    
    log(f"正在扫描目录: {scan_dir}")
    
    # 收集所有相关文件
    target_files = []
    for root, dirs, files in os.walk(scan_dir):
        # 跳过隐藏目录
        dirs[:] = [d for d in dirs if not d.startswith('.')]
        for f in files:
            if f.endswith(('.h', '.c')):
                target_files.append(os.path.join(root, f))
    
    target_files.sort()
    log(f"找到 {len(target_files)} 个C/H文件")
    
    # 解析所有文件的引脚配置
    parser_obj = PinConfigParser()
    all_assignments = []
    
    for filepath in target_files:
        assignments = parser_obj.parse_file(filepath)
        if assignments:
            all_assignments.extend(assignments)
            log(f"  {os.path.basename(filepath)}: {len(assignments)} 个引脚分配")
    
    log(f"\n共提取 {len(all_assignments)} 个引脚分配")
    
    # 检测冲突
    detector = ConflictDetector()
    result = detector.detect(all_assignments)
    
    # JSON输出
    if args.json:
        import json
        output_data = {
            'summary': {
                'files_scanned': len(target_files),
                'total_assignments': len(all_assignments),
                'conflicts': len(result['conflicts']),
                'warnings': len(result['warnings'])
            },
            'conflicts': result['conflicts'],
            'warnings': result['warnings'],
            'pin_usage': {k: v for k, v in result['pin_usage'].items()}
        }
        output = json.dumps(output_data, ensure_ascii=False, indent=2, default=str)
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as f:
                f.write(output)
            print(f"JSON已保存至: {args.output}")
        else:
            print(output)
        return
    
    # 生成报告
    report = generate_report(result, all_assignments, verbose=args.verbose)
    
    # 输出
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(report)
        print(f"报告已保存至: {args.output}")
    else:
        print('\n' + report)
    
    # 返回状态码
    if result['conflicts']:
        print(f"\n⚠️  发现 {len(result['conflicts'])} 个冲突，请检查!")
        sys.exit(1)
    else:
        print(f"\n✅ 无冲突。{len(result['warnings'])} 条警告。")


if __name__ == '__main__':
    main()
