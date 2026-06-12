#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试自动化框架 - 批量运行测试+生成报告+邮件通知
================================================
功能：
  - 批量执行测试脚本（Python/Shell/固件测试）
  - 测试结果收集与汇总
  - HTML/Markdown测试报告生成
  - 邮件通知（SMTP）
  - 测试套件管理
  - 测试历史记录与趋势分析
  - 超时/重试机制
"""

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

# ============================================================
# 测试用例定义
# ============================================================

# 测试状态枚举
STATUS_PASS = "PASS"
STATUS_FAIL = "FAIL"
STATUS_SKIP = "SKIP"
STATUS_ERROR = "ERROR"
STATUS_TIMEOUT = "TIMEOUT"

# 状态图标
STATUS_ICONS = {
    STATUS_PASS: "✓",
    STATUS_FAIL: "✗",
    STATUS_SKIP: "○",
    STATUS_ERROR: "!",
    STATUS_TIMEOUT: "⏱",
}


class TestCase:
    """测试用例"""

    def __init__(self, name, command, description="", timeout=60, retries=0,
                 category="default", tags=None, expected_returncode=0,
                 working_dir=None, env=None):
        self.name = name
        self.command = command            # 测试命令
        self.description = description    # 描述
        self.timeout = timeout            # 超时时间(秒)
        self.retries = retries            # 重试次数
        self.category = category          # 分类
        self.tags = tags or []            # 标签
        self.expected_rc = expected_returncode
        self.working_dir = working_dir
        self.env = env or {}

    def to_dict(self):
        return {
            "name": self.name,
            "command": self.command,
            "description": self.description,
            "timeout": self.timeout,
            "retries": self.retries,
            "category": self.category,
            "tags": self.tags,
            "expected_returncode": self.expected_rc,
            "working_dir": self.working_dir,
            "env": self.env,
        }


class TestResult:
    """测试结果"""

    def __init__(self, test_case):
        self.test = test_case
        self.status = STATUS_SKIP
        self.returncode = None
        self.stdout = ""
        self.stderr = ""
        self.duration = 0.0       # 耗时(秒)
        self.attempt = 0          # 第几次尝试
        self.timestamp = None
        self.error_message = ""

    def to_dict(self):
        return {
            "name": self.test.name,
            "category": self.test.category,
            "status": self.status,
            "returncode": self.returncode,
            "duration": round(self.duration, 3),
            "attempt": self.attempt,
            "timestamp": self.timestamp,
            "stdout": self.stdout[:5000],  # 截断过长输出
            "stderr": self.stderr[:5000],
            "error_message": self.error_message,
        }


class TestSuite:
    """测试套件"""

    def __init__(self, name, description=""):
        self.name = name
        self.description = description
        self.tests = []
        self.setup_cmd = None       # 套件初始化命令
        self.teardown_cmd = None    # 套件清理命令

    def add_test(self, test_case):
        self.tests.append(test_case)

    def load_from_json(self, filepath):
        """从JSON文件加载测试用例"""
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)

        self.name = data.get('suite_name', self.name)
        self.description = data.get('description', self.description)
        self.setup_cmd = data.get('setup')
        self.teardown_cmd = data.get('teardown')

        for item in data.get('tests', []):
            test = TestCase(
                name=item.get('name', ''),
                command=item.get('command', ''),
                description=item.get('description', ''),
                timeout=item.get('timeout', 60),
                retries=item.get('retries', 0),
                category=item.get('category', 'default'),
                tags=item.get('tags', []),
                expected_returncode=item.get('expected_returncode', 0),
                working_dir=item.get('working_dir'),
                env=item.get('env', {}),
            )
            self.tests.append(test)

    def load_from_csv(self, filepath):
        """从CSV文件加载测试用例"""
        with open(filepath, 'r', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            for row in reader:
                test = TestCase(
                    name=row.get('name', ''),
                    command=row.get('command', ''),
                    description=row.get('description', ''),
                    timeout=int(row.get('timeout', 60)),
                    retries=int(row.get('retries', 0)),
                    category=row.get('category', 'default'),
                    tags=row.get('tags', '').split(';') if row.get('tags') else [],
                    expected_returncode=int(row.get('expected_returncode', 0)),
                )
                self.tests.append(test)

    def to_json(self, filepath):
        """导出测试套件为JSON"""
        data = {
            "suite_name": self.name,
            "description": self.description,
            "setup": self.setup_cmd,
            "teardown": self.teardown_cmd,
            "tests": [t.to_dict() for t in self.tests],
        }
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)


def run_single_test(test_case):
    """
    执行单个测试用例
    返回: TestResult
    """
    result = TestResult(test_case)
    result.timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    max_attempts = test_case.retries + 1

    for attempt in range(1, max_attempts + 1):
        result.attempt = attempt
        start_time = time.time()

        try:
            # 准备环境变量
            env = os.environ.copy()
            env.update(test_case.env)

            # 执行命令
            proc = subprocess.run(
                test_case.command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=test_case.timeout,
                cwd=test_case.working_dir,
                env=env,
            )

            result.duration = time.time() - start_time
            result.returncode = proc.returncode
            result.stdout = proc.stdout
            result.stderr = proc.stderr

            # 判断结果
            if proc.returncode == test_case.expected_rc:
                result.status = STATUS_PASS
                result.error_message = ""
                break  # 通过，不再重试
            else:
                result.status = STATUS_FAIL
                result.error_message = f"返回码: {proc.returncode}, 期望: {test_case.expected_rc}"
                if attempt < max_attempts:
                    print(f"    [重试] 第{attempt}次失败，{max_attempts - attempt}次重试机会...")
                    time.sleep(1)

        except subprocess.TimeoutExpired:
            result.duration = time.time() - start_time
            result.status = STATUS_TIMEOUT
            result.error_message = f"超时({test_case.timeout}秒)"
            if attempt < max_attempts:
                print(f"    [重试] 超时，第{attempt}次重试...")
                time.sleep(1)

        except Exception as e:
            result.duration = time.time() - start_time
            result.status = STATUS_ERROR
            result.error_message = str(e)
            break  # 错误不重试

    return result


def run_suite(suite, parallel=False, max_workers=4, verbose=False):
    """
    运行测试套件
    返回: (结果列表, 统计字典)
    """
    results = []
    total = len(suite.tests)
    start_time = time.time()

    # 运行setup
    if suite.setup_cmd:
        print(f"[Setup] 执行: {suite.setup_cmd}")
        try:
            subprocess.run(suite.setup_cmd, shell=True, timeout=120, check=True)
        except Exception as e:
            print(f"[Setup] 失败: {e}")
            print("[中止] 套件初始化失败")
            return [], {}

    print(f"\n{'='*60}")
    print(f"  测试套件: {suite.name}")
    print(f"  用例数量: {total}")
    print(f"  开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'='*60}\n")

    for i, test in enumerate(suite.tests, 1):
        icon = STATUS_ICONS.get(STATUS_SKIP, "?")
        print(f"[{i}/{total}] {test.name}")
        if verbose and test.description:
            print(f"    描述: {test.description}")
        print(f"    命令: {test.command}")

        result = run_single_test(test)
        results.append(result)

        icon = STATUS_ICONS.get(result.status, "?")
        print(f"    {icon} {result.status} ({result.duration:.2f}s)")
        if result.status != STATUS_PASS and result.error_message:
            print(f"    错误: {result.error_message}")
        if verbose and result.stdout:
            for line in result.stdout.strip().split('\n')[:5]:
                print(f"    | {line}")
        print()

    # 运行teardown
    if suite.teardown_cmd:
        print(f"[Teardown] 执行: {suite.teardown_cmd}")
        try:
            subprocess.run(suite.teardown_cmd, shell=True, timeout=120)
        except Exception as e:
            print(f"[Teardown] 失败: {e}")

    total_time = time.time() - start_time

    # 统计
    stats = {
        "total": total,
        "pass": sum(1 for r in results if r.status == STATUS_PASS),
        "fail": sum(1 for r in results if r.status == STATUS_FAIL),
        "error": sum(1 for r in results if r.status == STATUS_ERROR),
        "timeout": sum(1 for r in results if r.status == STATUS_TIMEOUT),
        "skip": sum(1 for r in results if r.status == STATUS_SKIP),
        "total_time": round(total_time, 2),
        "pass_rate": 0,
    }
    if total > 0:
        stats["pass_rate"] = round(stats["pass"] / total * 100, 1)

    print(f"{'='*60}")
    print(f"  测试完成!")
    print(f"  通过: {stats['pass']}/{total} ({stats['pass_rate']}%)")
    print(f"  失败: {stats['fail']}  错误: {stats['error']}  超时: {stats['timeout']}")
    print(f"  总耗时: {stats['total_time']}秒")
    print(f"{'='*60}")

    return results, stats


def generate_text_report(suite_name, results, stats):
    """生成文本报告"""
    lines = []
    lines.append("=" * 70)
    lines.append(f"  测试报告: {suite_name}")
    lines.append(f"  生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("=" * 70)

    lines.append(f"\n■ 统计概要")
    lines.append(f"  总用例数: {stats['total']}")
    lines.append(f"  通过: {stats['pass']} ({stats['pass_rate']}%)")
    lines.append(f"  失败: {stats['fail']}")
    lines.append(f"  错误: {stats['error']}")
    lines.append(f"  超时: {stats['timeout']}")
    lines.append(f"  跳过: {stats['skip']}")
    lines.append(f"  总耗时: {stats['total_time']}秒")

    # 按分类汇总
    categories = {}
    for r in results:
        cat = r.test.category
        if cat not in categories:
            categories[cat] = {"pass": 0, "fail": 0, "total": 0}
        categories[cat]["total"] += 1
        if r.status == STATUS_PASS:
            categories[cat]["pass"] += 1
        else:
            categories[cat]["fail"] += 1

    lines.append(f"\n■ 分类汇总")
    lines.append(f"  {'分类':<20s} {'总数':>6s} {'通过':>6s} {'失败':>6s} {'通过率':>8s}")
    lines.append(f"  {'-'*48}")
    for cat, cstats in sorted(categories.items()):
        rate = cstats['pass'] / cstats['total'] * 100 if cstats['total'] > 0 else 0
        lines.append(f"  {cat:<20s} {cstats['total']:>6d} {cstats['pass']:>6d} "
                    f"{cstats['fail']:>6d} {rate:>7.1f}%")

    lines.append(f"\n■ 详细结果")
    lines.append(f"  {'#':>3s} {'状态':>4s} {'名称':<30s} {'耗时':>8s} {'尝试':>4s} {'分类':<15s}")
    lines.append(f"  {'-'*66}")
    for i, r in enumerate(results, 1):
        icon = STATUS_ICONS.get(r.status, "?")
        lines.append(f"  {i:>3d}  {icon} {r.test.name:<30s} {r.duration:>6.2f}s "
                    f"{r.attempt:>4d} {r.test.category:<15s}")

    # 失败详情
    failed = [r for r in results if r.status != STATUS_PASS and r.status != STATUS_SKIP]
    if failed:
        lines.append(f"\n■ 失败/错误详情")
        for r in failed:
            lines.append(f"\n  [{r.status}] {r.test.name}")
            lines.append(f"  命令: {r.test.command}")
            lines.append(f"  错误: {r.error_message}")
            if r.stderr:
                for line in r.stderr.strip().split('\n')[:10]:
                    lines.append(f"  stderr: {line}")

    lines.append("\n" + "=" * 70)
    return '\n'.join(lines)


def generate_html_report(suite_name, results, stats, output_path):
    """生成HTML报告"""
    html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>测试报告 - {suite_name}</title>
    <style>
        body {{ font-family: 'Microsoft YaHei', Arial, sans-serif; margin: 20px; background: #f5f5f5; }}
        .container {{ max-width: 1200px; margin: 0 auto; }}
        .header {{ background: #2c3e50; color: white; padding: 20px; border-radius: 8px; }}
        .summary {{ display: flex; gap: 20px; margin: 20px 0; }}
        .stat-card {{ background: white; padding: 20px; border-radius: 8px; flex: 1; text-align: center;
                     box-shadow: 0 2px 4px rgba(0,0,0,0.1); }}
        .stat-card h3 {{ margin: 0; color: #666; font-size: 14px; }}
        .stat-card .value {{ font-size: 36px; font-weight: bold; margin: 10px 0; }}
        .pass {{ color: #27ae60; }}
        .fail {{ color: #e74c3c; }}
        .error {{ color: #e67e22; }}
        .timeout {{ color: #9b59b6; }}
        .skip {{ color: #95a5a6; }}
        table {{ width: 100%; border-collapse: collapse; background: white;
                border-radius: 8px; overflow: hidden; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }}
        th {{ background: #34495e; color: white; padding: 12px; text-align: left; }}
        td {{ padding: 10px 12px; border-bottom: 1px solid #eee; }}
        tr:hover {{ background: #f8f9fa; }}
        .status-pass {{ color: #27ae60; font-weight: bold; }}
        .status-fail {{ color: #e74c3c; font-weight: bold; }}
        .status-error {{ color: #e67e22; font-weight: bold; }}
        .status-timeout {{ color: #9b59b6; font-weight: bold; }}
        .details {{ background: white; padding: 20px; border-radius: 8px; margin: 20px 0;
                   box-shadow: 0 2px 4px rgba(0,0,0,0.1); }}
        .fail-detail {{ background: #ffeaa7; padding: 10px; margin: 10px 0; border-radius: 4px;
                       border-left: 4px solid #e74c3c; }}
        pre {{ background: #2d3436; color: #dfe6e9; padding: 15px; border-radius: 4px;
              overflow-x: auto; font-size: 13px; }}
        .footer {{ text-align: center; color: #999; margin-top: 20px; font-size: 12px; }}
    </style>
</head>
<body>
<div class="container">
    <div class="header">
        <h1>📋 测试报告</h1>
        <p>套件: {suite_name} | 时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
    </div>

    <div class="summary">
        <div class="stat-card">
            <h3>总用例</h3>
            <div class="value">{stats['total']}</div>
        </div>
        <div class="stat-card">
            <h3>通过</h3>
            <div class="value pass">{stats['pass']}</div>
        </div>
        <div class="stat-card">
            <h3>失败</h3>
            <div class="value fail">{stats['fail']}</div>
        </div>
        <div class="stat-card">
            <h3>通过率</h3>
            <div class="value {'pass' if stats['pass_rate'] >= 80 else 'fail'}">{stats['pass_rate']}%</div>
        </div>
        <div class="stat-card">
            <h3>总耗时</h3>
            <div class="value">{stats['total_time']}s</div>
        </div>
    </div>

    <div class="details">
        <h2>测试结果</h2>
        <table>
            <thead>
                <tr>
                    <th>#</th><th>状态</th><th>名称</th><th>分类</th>
                    <th>耗时</th><th>尝试</th>
                </tr>
            </thead>
            <tbody>
"""
    for i, r in enumerate(results, 1):
        status_class = f"status-{r.status.lower()}"
        icon = STATUS_ICONS.get(r.status, "?")
        html += f"""                <tr>
                    <td>{i}</td>
                    <td class="{status_class}">{icon} {r.status}</td>
                    <td>{r.test.name}</td>
                    <td>{r.test.category}</td>
                    <td>{r.duration:.2f}s</td>
                    <td>{r.attempt}</td>
                </tr>
"""

    html += """            </tbody>
        </table>
    </div>
"""

    # 失败详情
    failed = [r for r in results if r.status not in (STATUS_PASS, STATUS_SKIP)]
    if failed:
        html += '    <div class="details">\n        <h2>失败详情</h2>\n'
        for r in failed:
            html += f"""        <div class="fail-detail">
            <strong>[{r.status}] {r.test.name}</strong><br>
            命令: <code>{r.test.command}</code><br>
            错误: {r.error_message}
"""
            if r.stderr:
                html += f"            <pre>{r.stderr[:2000]}</pre>\n"
            html += "        </div>\n"
        html += "    </div>\n"

    html += f"""
    <div class="footer">
        nuedc-asset-library - 测试自动化框架 | 自动生成
    </div>
</div>
</body>
</html>"""

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html)
    return html


def send_email_notification(smtp_server, smtp_port, sender, password, recipients,
                           subject, body, html_body=None):
    """
    发送邮件通知
    """
    import smtplib
    from email.mime.text import MIMEText
    from email.mime.multipart import MIMEMultipart

    msg = MIMEMultipart('alternative')
    msg['Subject'] = subject
    msg['From'] = sender
    msg['To'] = ', '.join(recipients)

    # 纯文本
    msg.attach(MIMEText(body, 'plain', 'utf-8'))

    # HTML
    if html_body:
        msg.attach(MIMEText(html_body, 'html', 'utf-8'))

    try:
        with smtplib.SMTP_SSL(smtp_server, smtp_port) as server:
            server.login(sender, password)
            server.sendmail(sender, recipients, msg.as_string())
        print(f"[OK] 邮件已发送至: {', '.join(recipients)}")
        return True
    except Exception as e:
        print(f"[错误] 邮件发送失败: {e}")
        return False


def save_test_history(results, stats, history_file):
    """保存测试历史记录"""
    record = {
        "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        "stats": stats,
        "results": [r.to_dict() for r in results],
    }

    history = []
    if os.path.exists(history_file):
        with open(history_file, 'r', encoding='utf-8') as f:
            try:
                history = json.load(f)
            except json.JSONDecodeError:
                history = []

    history.append(record)
    # 只保留最近100次
    history = history[-100:]

    with open(history_file, 'w', encoding='utf-8') as f:
        json.dump(history, f, ensure_ascii=False, indent=2)

    return record


def generate_trend_report(history_file):
    """生成趋势分析报告"""
    if not os.path.exists(history_file):
        print("[提示] 无历史记录")
        return ""

    with open(history_file, 'r', encoding='utf-8') as f:
        history = json.load(f)

    if not history:
        return ""

    lines = []
    lines.append("=" * 60)
    lines.append("  测试趋势分析")
    lines.append("=" * 60)
    lines.append(f"\n  历史记录数: {len(history)}")

    # 最近10次
    recent = history[-10:]
    lines.append(f"\n  {'时间':<20s} {'通过率':>8s} {'总用例':>6s} {'失败':>6s} {'耗时':>8s}")
    lines.append(f"  {'-'*50}")
    for h in recent:
        s = h.get('stats', {})
        lines.append(f"  {h['timestamp']:<20s} {s.get('pass_rate', 0):>7.1f}% "
                    f"{s.get('total', 0):>6d} {s.get('fail', 0):>6d} "
                    f"{s.get('total_time', 0):>6.1f}s")

    # 趋势
    rates = [h['stats'].get('pass_rate', 0) for h in recent]
    if len(rates) >= 2:
        trend = rates[-1] - rates[0]
        if trend > 0:
            lines.append(f"\n  趋势: ↑ 通过率提升 {trend:.1f}%")
        elif trend < 0:
            lines.append(f"\n  趋势: ↓ 通过率下降 {abs(trend):.1f}%")
        else:
            lines.append(f"\n  趋势: → 通过率稳定")

    lines.append("\n" + "=" * 60)
    return '\n'.join(lines)


def create_sample_suite(output_path):
    """创建示例测试套件"""
    suite = {
        "suite_name": "电赛基础功能测试",
        "description": "电赛项目的单元测试和集成测试",
        "tests": [
            {
                "name": "编译检查",
                "command": "echo '编译成功' && exit 0",
                "description": "检查项目是否能正常编译",
                "timeout": 120,
                "category": "build",
            },
            {
                "name": "ADC采样测试",
                "command": "python -c \"import random; v=random.uniform(0,3.3); print(f'ADC={v:.3f}V'); exit(0 if 0<v<3.3 else 1)\"",
                "description": "测试ADC采样功能",
                "timeout": 30,
                "category": "hardware",
            },
            {
                "name": "UART通信测试",
                "command": "echo 'UART OK: 115200 baud' && exit 0",
                "description": "测试串口通信",
                "timeout": 10,
                "category": "hardware",
                "tags": ["serial", "uart"],
            },
            {
                "name": "PID控制精度测试",
                "command": "python -c \"error=abs(100-98.5); print(f'误差={error}%'); exit(0 if error<5 else 1)\"",
                "description": "测试PID控制精度在5%以内",
                "timeout": 60,
                "retries": 1,
                "category": "algorithm",
            },
            {
                "name": "电池电压检测",
                "command": "python -c \"v=7.8; print(f'电池电压={v}V'); exit(0 if 6.0<v<8.4 else 1)\"",
                "description": "检测电池电压是否在正常范围",
                "timeout": 10,
                "category": "power",
            },
            {
                "name": "传感器数据完整性",
                "command": "python -c \"data=[1,2,3,4,5]; assert len(data)==5; print(f'传感器数据: {data}')\"",
                "description": "检查传感器数据完整性",
                "timeout": 10,
                "category": "sensor",
            },
        ],
    }
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(suite, f, ensure_ascii=False, indent=2)
    print(f"[OK] 示例测试套件已生成: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='测试自动化框架 - 批量运行测试+生成报告+邮件通知',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 生成示例测试套件
  python test_automation.py --init --output test_suite.json

  # 运行测试套件
  python test_automation.py --suite test_suite.json

  # 运行测试+生成HTML报告+保存历史
  python test_automation.py --suite test_suite.json --html --history

  # 运行测试+邮件通知
  python test_automation.py --suite test_suite.json --html \\
      --email-smtp smtp.qq.com --email-port 465 \\
      --email-sender user@qq.com --email-password xxx \\
      --email-to admin@example.com

  # 查看测试趋势
  python test_automation.py --trend

  # 只运行特定分类
  python test_automation.py --suite test_suite.json --filter-category hardware

  # 只运行含特定标签
  python test_automation.py --suite test_suite.json --filter-tag uart
        """
    )

    parser.add_argument('--suite', '-s', help='测试套件文件 (JSON/CSV)')
    parser.add_argument('--output', '-o', help='输出文件路径前缀', default='test_report')
    parser.add_argument('--init', action='store_true', help='生成示例测试套件')
    parser.add_argument('--html', action='store_true', help='生成HTML报告')
    parser.add_argument('--history', action='store_true', help='保存测试历史')
    parser.add_argument('--trend', action='store_true', help='查看测试趋势')
    parser.add_argument('--verbose', '-v', action='store_true', help='详细输出')

    # 过滤器
    parser.add_argument('--filter-category', help='只运行指定分类')
    parser.add_argument('--filter-tag', help='只运行含指定标签的用例')

    # 邮件参数
    parser.add_argument('--email-smtp', help='SMTP服务器')
    parser.add_argument('--email-port', type=int, default=465, help='SMTP端口')
    parser.add_argument('--email-sender', help='发件人邮箱')
    parser.add_argument('--email-password', help='邮箱密码/授权码')
    parser.add_argument('--email-to', nargs='*', help='收件人邮箱列表')

    # 执行控制
    parser.add_argument('--timeout', type=int, help='全局超时覆盖(秒)')
    parser.add_argument('--dry-run', action='store_true', help='仅列出测试，不执行')

    args = parser.parse_args()

    # 初始化
    if args.init:
        output = args.output if args.output != 'test_report' else 'test_suite.json'
        create_sample_suite(output)
        return

    # 趋势分析
    if args.trend:
        history_file = f"{args.output}_history.json"
        report = generate_trend_report(history_file)
        if report:
            print(report)
        return

    if not args.suite:
        parser.print_help()
        return

    # 加载测试套件
    suite = TestSuite("测试套件")
    ext = os.path.splitext(args.suite)[1].lower()
    if ext == '.json':
        suite.load_from_json(args.suite)
    elif ext == '.csv':
        suite.load_from_csv(args.suite)
    else:
        print(f"[错误] 不支持的文件格式: {ext}")
        sys.exit(1)

    # 过滤
    if args.filter_category:
        suite.tests = [t for t in suite.tests if t.category == args.filter_category]
    if args.filter_tag:
        suite.tests = [t for t in suite.tests if args.filter_tag in t.tags]

    # 全局超时覆盖
    if args.timeout:
        for t in suite.tests:
            t.timeout = args.timeout

    if not suite.tests:
        print("[警告] 没有可运行的测试用例")
        sys.exit(0)

    # Dry run
    if args.dry_run:
        print(f"\n测试套件: {suite.name} ({len(suite.tests)}个用例)")
        print(f"{'#':>3s} {'名称':<30s} {'命令':<40s} {'分类':<15s} {'超时':>6s}")
        print("-" * 96)
        for i, t in enumerate(suite.tests, 1):
            print(f"{i:>3d} {t.name:<30s} {t.command:<40s} {t.category:<15s} {t.timeout:>4d}s")
        return

    # 运行测试
    results, stats = run_suite(suite, verbose=args.verbose)

    if not results:
        print("[提示] 无测试结果")
        return

    # 生成文本报告
    text_report = generate_text_report(suite.name, results, stats)
    text_path = f"{args.output}.txt"
    with open(text_path, 'w', encoding='utf-8') as f:
        f.write(text_report)
    print(f"\n[OK] 文本报告: {text_path}")

    # 生成HTML报告
    html_content = None
    if args.html:
        html_path = f"{args.output}.html"
        html_content = generate_html_report(suite.name, results, stats, html_path)
        print(f"[OK] HTML报告: {html_path}")

    # 保存历史
    history_file = f"{args.output}_history.json"
    if args.history:
        save_test_history(results, stats, history_file)
        print(f"[OK] 历史记录已保存: {history_file}")

    # 邮件通知
    if args.email_smtp and args.email_sender and args.email_to:
        # 邮件正文
        subject = f"[测试报告] {suite.name} - 通过率 {stats['pass_rate']}%"
        body = text_report

        send_email_notification(
            smtp_server=args.email_smtp,
            smtp_port=args.email_port,
            sender=args.email_sender,
            password=args.email_password,
            recipients=args.email_to,
            subject=subject,
            body=body,
            html_body=html_content,
        )

    # 返回码：有失败则非0
    if stats['fail'] > 0 or stats['error'] > 0:
        sys.exit(1)


if __name__ == '__main__':
    main()
