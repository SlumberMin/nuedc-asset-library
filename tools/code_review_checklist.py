#!/usr/bin/env python3
"""
代码审查清单工具 - 基于错误经验库的自动检查
用于电赛嵌入式C代码质量检查
"""
import argparse
import json
import os
import re
import sys


# ── 检查规则定义 ──────────────────────────────────────────────

# 每条规则: (ID, 类别, 严重度, 描述, 正则模式, 建议)
CHECK_RULES = [
    # === 内存与指针 ===
    ('MEM001', '内存', '严重', '未检查malloc返回值',
     r'malloc\s*\([^)]+\)\s*;',
     'malloc后必须检查返回值是否为NULL'),
    ('MEM002', '内存', '严重', '潜在内存泄漏(缺少free)',
     r'malloc\s*\(',
     '确保每个malloc都有对应的free'),
    ('MEM003', '内存', '严重', '悬空指针风险(释放后使用)',
     r'free\s*\([^)]+\)',
     'free后应将指针设为NULL'),
    ('MEM004', '内存', '警告', '栈上大数组',
     r'(?:int|char|uint8_t|uint16_t|uint32_t|float|double)\s+\w+\s*\[\s*(\d{4,})\s*\]',
     '嵌入式系统栈空间有限，大数组应使用static或全局变量'),

    # === 中断安全 ===
    ('ISR001', '中断', '严重', '中断中调用延时函数',
     r'(?:__interrupt|ISR|void\s+\w+_Handler)\s*\([^)]*\)\s*\{[^}]*(?:delay|HAL_Delay|_delay_ms|sleep)',
     '中断服务程序中禁止使用阻塞延时'),
    ('ISR002', '中断', '严重', '中断中使用printf/串口阻塞发送',
     r'(?:__interrupt|ISR|void\s+\w+_Handler)\s*\([^)]*\)\s*\{[^}]*(?:printf|puts|UART_Send)',
     '中断中禁止阻塞IO操作'),
    ('ISR003', '中断', '警告', '中断中使用浮点运算',
     r'(?:__interrupt|ISR|void\s+\w+_Handler)\s*\([^)]*\)\s*\{[^}]*\bfloat\b',
     '中断中避免浮点运算（上下文切换开销大）'),
    ('ISR004', '中断', '警告', '中断变量缺少volatile',
     r'(?:(?:extern|static)\s+)?(?:uint\d+_t|int|char|bool)\s+(\w+)\s*=',
     '跨中断/主程序共享的变量需声明volatile'),

    # === 嵌入式常见错误 ===
    ('EMB001', '嵌入式', '严重', '位操作优先级错误',
     r'if\s*\([^)]*[^&|]\s*[&|]\s+[^&|][^)]*\)',
     '位运算&/|优先级低于比较运算符，必须加括号: (val & MASK) == FLAG'),
    ('EMB002', '嵌入式', '严重', '整数溢出风险',
     r'uint8_t\s+\w+\s*=\s*\d{3,}|uint16_t\s+\w+\s*=\s*\d{5,}',
     '确认赋值不超出类型范围(uint8_t: 0-255, uint16_t: 0-65535)'),
    ('EMB003', '嵌入式', '警告', '除法未检查除数为零',
     r'/\s*(?:\w+)(?!\s*!=\s*0)(?!\s*>\s*0)',
     '除法前必须检查除数不为零'),
    ('EMB004', '嵌入式', '警告', '数组越界风险(硬编码索引)',
     r'\w+\s*\[\s*\d+\s*\]',
     '硬编码数组索引需确认不越界'),
    ('EMB005', '嵌入式', '严重', 'ADC/DMA缓冲区对齐问题',
     r'(?:DMA|ADC|dac)\w*',
     'DMA传输缓冲区需4字节对齐(__attribute__((aligned(4))))'),
    ('EMB006', '嵌入式', '警告', '未使用的变量',
     r'(?:int|uint\d+_t|float|char)\s+(\w+)\s*=.*;\s*$',
     '未使用的变量浪费RAM，应删除或加(void)标记'),

    # === 硬件配置 ===
    ('HW001', '硬件', '严重', 'GPIO未使能时钟就配置',
     r'GPIO_Init|HAL_GPIO_Init',
     '使用GPIO前必须先使能对应端口时钟(RCC->AHB1ENR等)'),
    ('HW002', '硬件', '警告', '未检查外设初始化返回值',
     r'HAL_\w+_Init\s*\(',
     'HAL库函数应检查返回值是否为HAL_OK'),
    ('HW003', '硬件', '严重', '中断优先级未配置',
     r'HAL_NVIC_EnableIRQ|NVIC_EnableIRQ',
     '使能中断前必须配置优先级(NVIC_SetPriority)'),
    ('HW004', '硬件', '警告', '看门狗未配置',
     r'HAL_IWDG|IWDG',
     '比赛代码建议启用看门狗防死机'),

    # === 定时与精度 ===
    ('TIM001', '定时', '警告', '浮点数精度比较',
     r'==\s*\d+\.\d+|!=\s*\d+\.\d+',
     '浮点数禁止直接==比较，应使用 fabs(a-b) < epsilon'),
    ('TIM002', '定时', '警告', '延时函数精度问题',
     r'delay_ms|delay_us|HAL_Delay|_delay_ms',
     'HAL_Delay依赖SysTick中断，多任务环境可能不准'),

    # === 代码规范 ===
    ('STD001', '规范', '提示', '魔法数字(硬编码常量)',
     r'(?<![a-zA-Z_0-9])(?:0x[0-9a-fA-F]{4,}|\d{4,})(?![a-zA-Z_0-9])',
     '应定义为有意义的宏或常量'),
    ('STD002', '规范', '提示', '函数过长(>100行)',
     r'(?:void|int|uint\d+_t|static)\s+\w+\s*\([^)]*\)\s*\{',
     '函数应控制在100行以内，过大应拆分'),
    ('STD003', '规范', '提示', '缺少函数注释',
     r'^(?:void|int|uint\d+_t|static)\s+\w+\s*\([^)]*\)\s*\{',
     '函数应有Doxygen格式注释说明功能、参数、返回值'),
    ('STD004', '规范', '提示', 'TODO/FIXME标记',
     r'//\s*(?:TODO|FIXME|HACK|XXX)\b',
     '比赛前应处理所有TODO/FIXME'),

    # === 电源与功耗 ===
    ('PWR001', '电源', '警告', '未配置低功耗模式',
     r'(?:PWR|STOP|STANDBY|SLEEP)',
     '电池供电系统应配置合适的低功耗模式'),
]


# ── 代码解析与检查 ────────────────────────────────────────────

def check_file(filepath, rules=None):
    """
    对单个文件执行所有检查规则
    返回问题列表
    """
    if rules is None:
        rules = CHECK_RULES

    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
            lines = content.split('\n')
    except Exception as e:
        return [{'file': filepath, 'rule': 'FILE001', 'severity': '错误',
                 'message': f'无法读取文件: {e}', 'line': 0}]

    issues = []
    # 标记被注释的行
    in_block_comment = False
    comment_lines = set()

    for i, line in enumerate(lines):
        stripped = line.strip()
        if in_block_comment:
            comment_lines.add(i)
            if '*/' in stripped:
                in_block_comment = False
            continue
        if stripped.startswith('/*'):
            comment_lines.add(i)
            if '*/' not in stripped:
                in_block_comment = True
            continue
        if stripped.startswith('//'):
            comment_lines.add(i)
            continue

    for rule_id, category, severity, description, pattern, suggestion in rules:
        try:
            regex = re.compile(pattern, re.IGNORECASE | re.MULTILINE | re.DOTALL)
        except re.error:
            continue

        # 按行检查
        for i, line in enumerate(lines):
            if i in comment_lines:
                continue
            if regex.search(line):
                issues.append({
                    'file': os.path.basename(filepath),
                    'rule': rule_id,
                    'category': category,
                    'severity': severity,
                    'message': description,
                    'line': i + 1,
                    'code': line.strip()[:80],
                    'suggestion': suggestion,
                })

        # 按文件整体检查（用于跨行匹配如中断函数检查）
        if rule_id.startswith(('ISR', 'HW')):
            for m in regex.finditer(content):
                # 找到匹配所在的行号
                line_no = content[:m.start()].count('\n') + 1
                if line_no not in comment_lines:
                    # 避免重复（行级检查已覆盖的跳过）
                    already = any(iss['rule'] == rule_id and iss['line'] == line_no for iss in issues)
                    if not already:
                        issues.append({
                            'file': os.path.basename(filepath),
                            'rule': rule_id,
                            'category': category,
                            'severity': severity,
                            'message': description,
                            'line': line_no,
                            'code': lines[line_no - 1].strip()[:80] if line_no <= len(lines) else '',
                            'suggestion': suggestion,
                        })

    return issues


def check_directory(dirpath, extensions=('.c', '.h'), rules=None):
    """检查整个目录"""
    all_issues = []
    file_count = 0

    for root, dirs, files in os.walk(dirpath):
        for fname in files:
            if any(fname.endswith(ext) for ext in extensions):
                filepath = os.path.join(root, fname)
                issues = check_file(filepath, rules)
                all_issues.extend(issues)
                file_count += 1

    return all_issues, file_count


# ── 报告生成 ──────────────────────────────────────────────────

def generate_report(issues, file_count, output_format='text'):
    """生成审查报告"""
    if output_format == 'json':
        return json.dumps({
            'total_files': file_count,
            'total_issues': len(issues),
            'by_severity': {
                '严重': len([i for i in issues if i['severity'] == '严重']),
                '警告': len([i for i in issues if i['severity'] == '警告']),
                '提示': len([i for i in issues if i['severity'] == '提示']),
            },
            'issues': issues
        }, indent=2, ensure_ascii=False)

    # 文本格式
    lines = []
    lines.append("=" * 70)
    lines.append("  电赛代码审查报告")
    lines.append("=" * 70)
    lines.append(f"  检查文件数: {file_count}")
    lines.append(f"  发现问题数: {len(issues)}")

    critical = [i for i in issues if i['severity'] == '严重']
    warning = [i for i in issues if i['severity'] == '警告']
    info = [i for i in issues if i['severity'] == '提示']

    lines.append(f"    严重: {len(critical)}  警告: {len(warning)}  提示: {len(info)}")
    lines.append("=" * 70)

    severity_icons = {'严重': '🔴', '警告': '🟡', '提示': '🔵', '错误': '❌'}

    # 按严重度排序
    severity_order = {'严重': 0, '警告': 1, '提示': 2, '错误': 3}
    sorted_issues = sorted(issues, key=lambda x: (severity_order.get(x['severity'], 9), x['file'], x['line']))

    current_file = None
    for issue in sorted_issues:
        if issue['file'] != current_file:
            current_file = issue['file']
            lines.append(f"\n📄 {current_file}")
            lines.append("-" * 50)

        icon = severity_icons.get(issue['severity'], '❓')
        lines.append(f"  {icon} [{issue['rule']}] 第{issue['line']}行 - {issue['message']}")
        lines.append(f"     代码: {issue.get('code', '')[:60]}")
        lines.append(f"     建议: {issue.get('suggestion', '')}")

    # 评分
    score = max(0, 100 - len(critical) * 10 - len(warning) * 3 - len(info) * 1)
    lines.append(f"\n{'=' * 70}")
    lines.append(f"  代码质量评分: {score}/100")
    if score >= 90:
        lines.append("  评价: ✅ 优秀 - 代码质量很高")
    elif score >= 70:
        lines.append("  评价: ⚠️ 良好 - 建议修复严重问题")
    elif score >= 50:
        lines.append("  评价: ⚡ 一般 - 需要较多改进")
    else:
        lines.append("  评价: ❌ 较差 - 建议全面重构")
    lines.append("=" * 70)

    return "\n".join(lines)


# ── 自定义规则加载 ────────────────────────────────────────────

def load_custom_rules(filepath):
    """从JSON文件加载自定义检查规则"""
    if not os.path.exists(filepath):
        return []
    with open(filepath, 'r', encoding='utf-8') as f:
        custom = json.load(f)
    rules = []
    for r in custom:
        rules.append((r['id'], r['category'], r['severity'], r['description'],
                       r['pattern'], r.get('suggestion', '')))
    return rules


# ── 经验库知识 ────────────────────────────────────────────────

LESSONS_LEARNED = """
=== 电赛常见错误经验库 ===

1. ADC采样精度问题
   - ADC时钟不能超过规定频率(如STM32F4: 36MHz)
   - 采样时间要足够，建议239.5 cycles
   - 参考电压要稳定，加100nF滤波电容

2. DMA传输问题
   - DMA缓冲区必须对齐(__attribute__((aligned(4))))
   - 循环模式下注意半传输/全传输中断
   - DMA传输完成后再读取数据

3. 定时器精度
   - 高精度定时用硬件定时器，不用软件延时
   - PWM频率 = 定时器时钟 / (预分频 * 重装值)
   - 注意预分频和重装值的取值范围

4. 通信协议
   - UART: 波特率误差<2%，推荐115200/9600
   - SPI: 注意CPOL/CPHA配置匹配
   - I2C: 上拉电阻4.7kΩ，总线速率400kHz

5. 滤波算法
   - 一阶RC滤波: y = α*x + (1-α)*y_prev
   - 滑动平均: 取最近N个值平均
   - 中值滤波: 去除脉冲噪声效果好

6. 电源设计
   - 每个VCC引脚加100nF去耦电容
   - 大电流路径用粗走线/铜皮
   - 模拟/数字地单点连接

7. 调试技巧
   - 先确认时钟配置正确
   - 用GPIO翻转+示波器测执行时间
   - 串口打印要加\\r\\n(Windows终端)

8. 比赛策略
   - 基础功能优先，扩展功能其次
   - 预留调试接口(UART/SWD)
   - 看门狗防死机
"""


# ── CLI ──────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='代码审查清单工具 - 基于错误经验库的自动检查',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='示例:\n'
               '  python code_review_checklist.py scan ./src/\n'
               '  python code_review_checklist.py scan main.c --format json\n'
               '  python code_review_checklist.py lessons\n'
               '  python code_review_checklist.py rules\n'
    )
    sub = parser.add_subparsers(dest='cmd')

    # 扫描
    p_scan = sub.add_parser('scan', help='扫描代码')
    p_scan.add_argument('path', help='文件或目录路径')
    p_scan.add_argument('--format', '-f', default='text', choices=['text', 'json'])
    p_scan.add_argument('--out', '-o', help='输出文件')
    p_scan.add_argument('--ext', default='.c,.h', help='文件扩展名(逗号分隔)')
    p_scan.add_argument('--custom-rules', help='自定义规则JSON文件')
    p_scan.add_argument('--exclude-severity', help='排除的严重度(逗号分隔)')

    # 查看规则
    p_rules = sub.add_parser('rules', help='列出所有检查规则')
    p_rules.add_argument('--category', help='按类别过滤')

    # 经验库
    p_lessons = sub.add_parser('lessons', help='查看错误经验库')

    # 生成清单模板
    p_template = sub.add_parser('template', help='生成自定义规则模板')

    args = parser.parse_args()
    if not args.cmd:
        parser.print_help()
        return

    if args.cmd == 'scan':
        # 加载规则
        rules = list(CHECK_RULES)
        if args.custom_rules:
            rules.extend(load_custom_rules(args.custom_rules))

        # 排除严重度
        if args.exclude_severity:
            exclude = set(args.exclude_severity.split(','))
            rules = [r for r in rules if r[2] not in exclude]

        # 扫描
        if os.path.isfile(args.path):
            issues = check_file(args.path, rules)
            file_count = 1
        elif os.path.isdir(args.path):
            extensions = tuple(args.ext.split(','))
            issues, file_count = check_directory(args.path, extensions, rules)
        else:
            print(f"路径不存在: {args.path}")
            return

        # 生成报告
        report = generate_report(issues, file_count, args.format)
        print(report)

        if args.out:
            with open(args.out, 'w', encoding='utf-8') as f:
                f.write(report)
            print(f"\n报告已保存: {args.out}")

    elif args.cmd == 'rules':
        print(f"\n{'=' * 70}")
        print(f"  检查规则列表 (共{len(CHECK_RULES)}条)")
        print(f"{'=' * 70}")

        categories = {}
        for rule in CHECK_RULES:
            cat = rule[1]
            if args.category and cat != args.category:
                continue
            if cat not in categories:
                categories[cat] = []
            categories[cat].append(rule)

        for cat, rules in categories.items():
            print(f"\n📂 {cat} ({len(rules)}条)")
            for rule_id, _, severity, desc, _, suggestion in rules:
                icon = {'严重': '🔴', '警告': '🟡', '提示': '🔵'}.get(severity, '❓')
                print(f"  {icon} [{rule_id}] {desc}")
                print(f"     → {suggestion}")

    elif args.cmd == 'lessons':
        print(LESSONS_LEARNED)

    elif args.cmd == 'template':
        template = [
            {
                "id": "CUSTOM001",
                "category": "自定义",
                "severity": "警告",
                "description": "自定义检查描述",
                "pattern": r"正则表达式",
                "suggestion": "修复建议"
            }
        ]
        print(json.dumps(template, indent=2, ensure_ascii=False))
        print("\n将以上模板保存为JSON文件，使用 --custom-rules 参数加载")


if __name__ == '__main__':
    main()
