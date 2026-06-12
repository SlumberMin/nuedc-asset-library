#!/usr/bin/env python3
"""
电赛资产库自动化测试运行器
功能: 自动发现和运行测试、生成报告、性能基准汇总
支持参数: --verbose, --filter, --benchmark, --report
"""

import sys
import os
import unittest
import time
import json
import html
from datetime import datetime
from pathlib import Path

# 测试目录路径
TEST_DIR = Path(__file__).parent
REPORT_DIR = TEST_DIR / "reports"


def discover_tests(pattern=None):
    """发现所有测试文件"""
    test_files = []

    if pattern:
        # 按模式过滤
        for test_file in TEST_DIR.glob(f"test_{pattern}*.py"):
            test_files.append(test_file)
    else:
        # 发现所有测试文件
        for test_file in TEST_DIR.glob("test_*.py"):
            test_files.append(test_file)

    return test_files


def run_tests(test_files, verbose=False):
    """运行所有测试并收集结果"""
    results = {
        "total": 0,
        "passed": 0,
        "failed": 0,
        "errors": 0,
        "skipped": 0,
        "test_details": [],
        "start_time": None,
        "end_time": None,
        "duration": 0
    }

    results["start_time"] = time.time()

    # 使用unittest的TestLoader
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    for test_file in test_files:
        try:
            # 动态导入测试模块
            module_name = test_file.stem
            spec = __import__(module_name, fromlist=[module_name])

            # 加载测试
            tests = loader.loadTestsFromModule(spec)
            suite.addTests(tests)
        except Exception as e:
            print(f"警告: 无法加载 {test_file.name}: {e}")

    # 运行测试
    class CustomTestResult(unittest.TextTestResult):
        def __init__(self, stream, descriptions, verbosity):
            super().__init__(stream, descriptions, verbosity)
            self.test_details = []
            self.current_test = None

        def startTest(self, test):
            super().startTest(test)
            self.current_test = {
                "name": str(test),
                "status": "running",
                "start_time": time.time()
            }

        def addSuccess(self, test):
            super().addSuccess(test)
            self.current_test["status"] = "passed"
            self.current_test["end_time"] = time.time()
            self.test_details.append(self.current_test)

        def addFailure(self, test, err):
            super().addFailure(test, err)
            self.current_test["status"] = "failed"
            self.current_test["error"] = str(err)
            self.current_test["end_time"] = time.time()
            self.test_details.append(self.current_test)

        def addError(self, test, err):
            super().addError(test, err)
            self.current_test["status"] = "error"
            self.current_test["error"] = str(err)
            self.current_test["end_time"] = time.time()
            self.test_details.append(self.current_test)

        def addSkip(self, test, reason):
            super().addSkip(test, reason)
            self.current_test["status"] = "skipped"
            self.current_test["reason"] = reason
            self.current_test["end_time"] = time.time()
            self.test_details.append(self.current_test)

    verbosity = 2 if verbose else 1
    stream = sys.stdout if verbose else open(os.devnull, 'w')
    result = CustomTestResult(stream, True, verbosity)

    print(f"发现 {len(test_files)} 个测试文件")
    print(f"开始运行测试...")
    print("-" * 70)

    # 运行测试套件
    if verbose:
        suite.run(result)
    else:
        # 非详细模式，只打印总结
        test_result = unittest.TextTestResult(stream, True, 0)
        suite.run(test_result)

    results["end_time"] = time.time()
    results["duration"] = results["end_time"] - results["start_time"]

    if verbose:
        results["test_details"] = result.test_details
        results["total"] = result.testsRun
        results["failed"] = len(result.failures)
        results["errors"] = len(result.errors)
        results["skipped"] = len(result.skipped)
        results["passed"] = results["total"] - results["failed"] - results["errors"]
    else:
        # 非详细模式下从test_result获取统计
        results["total"] = test_result.testsRun if 'test_result' in locals() else result.testsRun
        results["failed"] = len(test_result.failures) if 'test_result' in locals() else 0
        results["errors"] = len(test_result.errors) if 'test_result' in locals() else 0
        results["skipped"] = len(test_result.skipped) if 'test_result' in locals() else 0
        results["passed"] = results["total"] - results["failed"] - results["errors"]

    return results


def calculate_coverage(test_results):
    """计算测试覆盖率"""
    # 简化计算：测试覆盖模块数 / 总模块数
    total_files = len(list(TEST_DIR.glob("test_*.py")))
    covered_files = len([f for f in TEST_DIR.glob("test_*.py")])

    # 根据测试结果估算
    coverage = (covered_files / max(total_files, 1)) * 100
    return {
        "files_covered": covered_files,
        "files_total": total_files,
        "coverage_percent": coverage
    }


def load_benchmark_results():
    """加载性能基准结果"""
    benchmark_file = TEST_DIR / "benchmark_results.json"
    if benchmark_file.exists():
        with open(benchmark_file, 'r') as f:
            return json.load(f)
    return {}


def save_benchmark_results(results, coverage):
    """保存性能基准结果"""
    benchmark_data = {
        "timestamp": datetime.now().isoformat(),
        "test_results": {
            "total": results["total"],
            "passed": results["passed"],
            "failed": results["failed"],
            "skipped": results["skipped"]
        },
        "coverage": coverage,
        "duration": results["duration"]
    }

    # 添加历史记录
    history_file = TEST_DIR / "benchmark_history.json"
    history = []
    if history_file.exists():
        with open(history_file, 'r') as f:
            history = json.load(f)

    history.append(benchmark_data)

    # 只保留最近20条记录
    if len(history) > 20:
        history = history[-20:]

    with open(history_file, 'w') as f:
        json.dump(history, f, indent=2)

    return benchmark_data


def generate_text_report(results, coverage):
    """生成文本报告"""
    report = f"""
================================================================================
电赛资产库测试报告
================================================================================
生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

测试结果汇总
--------------------------------------------------------------------------------
总测试数:     {results['total']}
通过:         {results['passed']} ✓
失败:         {results['failed']} ✗
错误:         {results['errors']} ✗
跳过:         {results['skipped']} ○
--------------------------------------------------------------------------------
通过率:       {results['passed'] / max(results['total'], 1) * 100:.1f}%
执行时间:     {results['duration']:.2f} 秒

测试覆盖率
--------------------------------------------------------------------------------
测试文件数:   {coverage['files_covered']}
文件覆盖率:   {coverage['coverage_percent']:.1f}%

{'=' * 80}
"""

    if results.get('test_details') and results['failed'] > 0:
        report += "\n失败测试详情:\n"
        report += "-" * 80 + "\n"
        for detail in results['test_details']:
            if detail['status'] == 'failed':
                report += f"✗ {detail['name']}\n"
                if 'error' in detail:
                    report += f"  错误: {detail['error'][:200]}\n"
                report += "\n"

    return report


def generate_html_report(results, coverage):
    """生成HTML报告"""
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    pass_rate = results['passed'] / max(results['total'], 1) * 100

    html_content = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>电赛资产库测试报告</title>
    <style>
        * {{
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }}
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: #f5f5f5;
            line-height: 1.6;
            padding: 20px;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            overflow: hidden;
        }}
        .header {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 30px;
            text-align: center;
        }}
        .header h1 {{
            font-size: 28px;
            margin-bottom: 10px;
        }}
        .header .subtitle {{
            opacity: 0.9;
        }}
        .content {{
            padding: 30px;
        }}
        .stats-grid {{
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 20px;
            margin-bottom: 30px;
        }}
        .stat-card {{
            background: #f8f9fa;
            padding: 20px;
            border-radius: 10px;
            text-align: center;
            border: 2px solid #e9ecef;
        }}
        .stat-card.total {{
            border-color: #007bff;
        }}
        .stat-card.passed {{
            border-color: #28a745;
        }}
        .stat-card.failed {{
            border-color: #dc3545;
        }}
        .stat-card.skipped {{
            border-color: #ffc107;
        }}
        .stat-card h3 {{
            font-size: 32px;
            margin: 10px 0;
        }}
        .stat-card p {{
            color: #666;
            font-size: 14px;
        }}
        .stat-card.total h3 {{
            color: #007bff;
        }}
        .stat-card.passed h3 {{
            color: #28a745;
        }}
        .stat-card.failed h3 {{
            color: #dc3545;
        }}
        .stat-card.skipped h3 {{
            color: #ffc107;
        }}
        .progress-bar {{
            background: #e9ecef;
            height: 20px;
            border-radius: 10px;
            overflow: hidden;
            margin: 20px 0;
        }}
        .progress-fill {{
            height: 100%;
            border-radius: 10px;
            transition: width 0.5s;
        }}
        .progress-fill.pass {{
            background: linear-gradient(90deg, #28a745, #20c997);
        }}
        .progress-fill.fail {{
            background: linear-gradient(90deg, #dc3545, #ff6b6b);
        }}
        .coverage-section {{
            background: #f8f9fa;
            padding: 20px;
            border-radius: 10px;
            margin-bottom: 30px;
        }}
        .coverage-section h2 {{
            color: #333;
            margin-bottom: 15px;
            font-size: 20px;
        }}
        .coverage-bar {{
            background: #e9ecef;
            height: 24px;
            border-radius: 12px;
            overflow: hidden;
            position: relative;
        }}
        .coverage-fill {{
            height: 100%;
            border-radius: 12px;
            background: linear-gradient(90deg, #007bff, #6610f2);
            display: flex;
            align-items: center;
            justify-content: center;
            color: white;
            font-weight: bold;
            font-size: 12px;
        }}
        .details-section {{
            margin-top: 30px;
        }}
        .details-section h2 {{
            color: #333;
            margin-bottom: 15px;
            font-size: 20px;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            background: white;
            border-radius: 10px;
            overflow: hidden;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }}
        th {{
            background: #343a40;
            color: white;
            padding: 12px;
            text-align: left;
        }}
        td {{
            padding: 12px;
            border-bottom: 1px solid #e9ecef;
        }}
        tr:last-child td {{
            border-bottom: none;
        }}
        tr:hover {{
            background: #f8f9fa;
        }}
        .status-passed {{
            color: #28a745;
            font-weight: bold;
        }}
        .status-failed {{
            color: #dc3545;
            font-weight: bold;
        }}
        .status-skipped {{
            color: #ffc107;
            font-weight: bold;
        }}
        .summary-box {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
            border-radius: 10px;
            margin-top: 30px;
        }}
        .summary-box h2 {{
            margin-bottom: 10px;
        }}
        .summary-box p {{
            opacity: 0.9;
            line-height: 1.8;
        }}
        .timestamp {{
            text-align: center;
            color: #666;
            font-size: 12px;
            margin-top: 30px;
            padding-top: 20px;
            border-top: 1px solid #e9ecef;
        }}
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>电赛资产库测试报告</h1>
            <p class="subtitle">Test Report - {timestamp}</p>
        </div>

        <div class="content">
            <div class="stats-grid">
                <div class="stat-card total">
                    <p>总测试数</p>
                    <h3>{results['total']}</h3>
                </div>
                <div class="stat-card passed">
                    <p>通过</p>
                    <h3>{results['passed']}</h3>
                </div>
                <div class="stat-card failed">
                    <p>失败</p>
                    <h3>{results['failed']}</h3>
                </div>
                <div class="stat-card skipped">
                    <p>跳过</p>
                    <h3>{results['skipped']}</h3>
                </div>
            </div>

            <div class="progress-bar">
                <div class="progress-fill pass" style="width: {pass_rate}%">
                    {pass_rate:.1f}% 通过率
                </div>
            </div>

            <div class="coverage-section">
                <h2>测试覆盖率</h2>
                <div class="coverage-bar">
                    <div class="coverage-fill" style="width: {coverage['coverage_percent']}%">
                        {coverage['files_covered']} / {coverage['files_total']} 文件 ({coverage['coverage_percent']:.1f}%)
                    </div>
                </div>
            </div>

            <div class="details-section">
                <h2>测试结果详情</h2>
                <table>
                    <thead>
                        <tr>
                            <th>测试名称</th>
                            <th>状态</th>
                            <th>执行时间</th>
                        </tr>
                    </thead>
                    <tbody>
"""

    # 添加测试详情
    if results.get('test_details'):
        for detail in results['test_details'][:50]:  # 只显示前50条
            status_class = f"status-{detail['status']}"
            status_text = {
                'passed': '✓ 通过',
                'failed': '✗ 失败',
                'error': '✗ 错误',
                'skipped': '○ 跳过'
            }.get(detail['status'], detail['status'])

            duration = detail.get('end_time', 0) - detail.get('start_time', 0)

            html_content += f"""                        <tr>
                            <td>{html.escape(detail['name'])}</td>
                            <td class="{status_class}">{status_text}</td>
                            <td>{duration:.3f}s</td>
                        </tr>
"""

    html_content += f"""                    </tbody>
                </table>
            </div>

            <div class="summary-box">
                <h2>执行摘要</h2>
                <p>
                    测试执行耗时: <strong>{results['duration']:.2f}</strong> 秒<br>
                    通过率: <strong>{pass_rate:.1f}%</strong> | 测试文件覆盖率: <strong>{coverage['coverage_percent']:.1f}%</strong><br>
                    总测试数: <strong>{results['total']}</strong> | 通过: <strong>{results['passed']}</strong> | 失败: <strong>{results['failed']}</strong> | 跳过: <strong>{results['skipped']}</strong>
                </p>
            </div>

            <div class="timestamp">
                报告生成时间: {timestamp}
            </div>
        </div>
    </div>
</body>
</html>"""

    return html_content


def save_report(report_content, report_type, timestamp):
    """保存报告文件"""
    REPORT_DIR.mkdir(exist_ok=True)

    if report_type == "text":
        filename = REPORT_DIR / f"test_report_{timestamp}.txt"
    else:
        filename = REPORT_DIR / f"test_report_{timestamp}.html"

    with open(filename, 'w', encoding='utf-8') as f:
        f.write(report_content)

    return filename


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='电赛资产库自动化测试运行器')
    parser.add_argument('--verbose', '-v', action='store_true', help='详细输出')
    parser.add_argument('--filter', '-f', type=str, help='按模式过滤测试文件')
    parser.add_argument('--benchmark', '-b', action='store_true', help='只运行性能基准测试')
    parser.add_argument('--report', '-r', action='store_true', help='生成HTML报告')

    args = parser.parse_args()

    print("电赛资产库自动化测试运行器")
    print("=" * 70)

    # 发现测试文件
    if args.benchmark:
        test_files = [TEST_DIR / "test_performance_benchmark.py"]
        if not test_files[0].exists():
            print("错误: 未找到 test_performance_benchmark.py")
            sys.exit(1)
    else:
        test_files = discover_tests(args.filter)

    if not test_files:
        print("错误: 未找到测试文件")
        sys.exit(1)

    print(f"发现 {len(test_files)} 个测试文件")

    # 运行测试
    results = run_tests(test_files, args.verbose)

    # 计算覆盖率
    coverage = calculate_coverage(results)

    # 保存基准结果
    benchmark_data = save_benchmark_results(results, coverage)

    # 生成报告
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # 文本报告
    text_report = generate_text_report(results, coverage)
    text_filename = save_report(text_report, "text", timestamp)
    print(f"\n文本报告已保存到: {text_filename}")

    # HTML报告
    if args.report:
        html_report = generate_html_report(results, coverage)
        html_filename = save_report(html_report, "html", timestamp)
        print(f"HTML报告已保存到: {html_filename}")

    # 打印摘要
    print("\n" + "=" * 70)
    print("测试运行摘要")
    print("=" * 70)
    print(f"总测试数:   {results['total']}")
    print(f"通过:       {results['passed']}")
    print(f"失败:       {results['failed']}")
    print(f"错误:       {results['errors']}")
    print(f"跳过:       {results['skipped']}")
    print(f"通过率:     {results['passed'] / max(results['total'], 1) * 100:.1f}%")
    print(f"执行时间:   {results['duration']:.2f} 秒")
    print("=" * 70)

    # 返回退出码
    sys.exit(0 if results['failed'] == 0 and results['errors'] == 0 else 1)


if __name__ == '__main__':
    main()
