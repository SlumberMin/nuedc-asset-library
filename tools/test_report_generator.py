#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试报告生成器
==============
功能：
  - 从 pytest JSON 输出生成精美的 HTML 测试报告
  - 支持 pytest-html 报告的二次美化
  - 显示通过/失败/跳过统计
  - 包含耗时分析和失败详情
  - 可嵌入代码覆盖率数据

依赖：无额外依赖（纯Python标准库）

用法：
  python test_report_generator.py --input results.json -o report.html
  python test_report_generator.py --input results.json --coverage coverage.json
  python test_report_generator.py --pytest-dir . --run-pytest            # 自动运行pytest并生成报告
"""

import argparse
import json
import os
import sys
from datetime import datetime
from pathlib import Path


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="测试报告生成器 - 从pytest输出生成HTML报告",
    )
    parser.add_argument("--input", "-i", type=str, default=None,
                        help="pytest JSON结果文件路径")
    parser.add_argument("--output", "-o", type=str, default="test_report.html",
                        help="输出HTML报告路径（默认 test_report.html）")
    parser.add_argument("--coverage", type=str, default=None,
                        help="coverage JSON文件路径")
    parser.add_argument("--title", type=str, default="电赛项目测试报告",
                        help="报告标题")
    parser.add_argument("--run-pytest", action="store_true",
                        help="自动运行 pytest 并收集结果")
    parser.add_argument("--pytest-dir", type=str, default=".",
                        help="pytest运行目录（默认当前目录）")
    parser.add_argument("--pytest-args", type=str, default="",
                        help="额外的pytest参数")
    return parser.parse_args()


# HTML 报告模板
HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{title}</title>
<style>
  * {{ margin: 0; padding: 0; box-sizing: border-box; }}
  body {{
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Microsoft YaHei', sans-serif;
    background: #f0f2f5; color: #333; line-height: 1.6;
  }}
  .container {{ max-width: 1100px; margin: 0 auto; padding: 24px; }}
  .header {{
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    color: white; padding: 32px; border-radius: 12px; margin-bottom: 24px;
    box-shadow: 0 4px 16px rgba(0,0,0,0.15);
  }}
  .header h1 {{ font-size: 28px; margin-bottom: 8px; }}
  .header .meta {{ opacity: 0.85; font-size: 14px; }}

  .stats-grid {{
    display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
    gap: 16px; margin-bottom: 24px;
  }}
  .stat-card {{
    background: white; border-radius: 10px; padding: 20px; text-align: center;
    box-shadow: 0 2px 8px rgba(0,0,0,0.08); transition: transform 0.2s;
  }}
  .stat-card:hover {{ transform: translateY(-2px); }}
  .stat-card .number {{ font-size: 36px; font-weight: 700; }}
  .stat-card .label {{ font-size: 14px; color: #666; margin-top: 4px; }}
  .stat-card.passed .number {{ color: #52c41a; }}
  .stat-card.failed .number {{ color: #ff4d4f; }}
  .stat-card.skipped .number {{ color: #faad14; }}
  .stat-card.error .number {{ color: #ff7a45; }}
  .stat-card.total .number {{ color: #1890ff; }}
  .stat-card.duration .number {{ color: #722ed1; font-size: 24px; }}

  .section {{
    background: white; border-radius: 10px; padding: 24px; margin-bottom: 24px;
    box-shadow: 0 2px 8px rgba(0,0,0,0.08);
  }}
  .section h2 {{
    font-size: 18px; margin-bottom: 16px; padding-bottom: 8px;
    border-bottom: 2px solid #f0f0f0;
  }}

  /* 进度条 */
  .progress-bar {{
    width: 100%; height: 24px; background: #f0f0f0; border-radius: 12px;
    overflow: hidden; margin: 16px 0; display: flex;
  }}
  .progress-bar .pass {{ background: #52c41a; transition: width 0.6s; }}
  .progress-bar .fail {{ background: #ff4d4f; transition: width 0.6s; }}
  .progress-bar .skip {{ background: #faad14; transition: width 0.6s; }}

  /* 测试结果表格 */
  table {{ width: 100%; border-collapse: collapse; }}
  th {{
    background: #fafafa; padding: 12px 16px; text-align: left;
    font-weight: 600; font-size: 13px; color: #666;
    border-bottom: 2px solid #e8e8e8;
  }}
  td {{
    padding: 10px 16px; border-bottom: 1px solid #f0f0f0;
    font-size: 14px; vertical-align: top;
  }}
  tr:hover {{ background: #fafafa; }}

  .badge {{
    display: inline-block; padding: 2px 10px; border-radius: 10px;
    font-size: 12px; font-weight: 600;
  }}
  .badge.passed {{ background: #f6ffed; color: #52c41a; border: 1px solid #b7eb8f; }}
  .badge.failed {{ background: #fff2f0; color: #ff4d4f; border: 1px solid #ffccc7; }}
  .badge.skipped {{ background: #fffbe6; color: #faad14; border: 1px solid #ffe58f; }}
  .badge.error {{ background: #fff7e6; color: #ff7a45; border: 1px solid #ffbb96; }}

  .failure-detail {{
    background: #fff2f0; border: 1px solid #ffccc7; border-radius: 6px;
    padding: 12px; margin: 8px 0; font-family: 'Cascadia Code', monospace;
    font-size: 12px; white-space: pre-wrap; word-break: break-all;
    max-height: 200px; overflow-y: auto;
  }}

  .coverage-bar {{
    width: 100%; height: 8px; background: #f0f0f0; border-radius: 4px;
    overflow: hidden; margin-top: 4px;
  }}
  .coverage-bar .fill {{ background: #1890ff; height: 100%; border-radius: 4px; }}

  .footer {{
    text-align: center; padding: 16px; color: #999; font-size: 12px;
  }}
</style>
</head>
<body>
<div class="container">
  <div class="header">
    <h1>📊 {title}</h1>
    <div class="meta">
      生成时间: {generated_at} &nbsp;|&nbsp;
      测试耗时: {total_duration:.2f}s
    </div>
  </div>

  <!-- 统计卡片 -->
  <div class="stats-grid">
    <div class="stat-card total">
      <div class="number">{total}</div><div class="label">总测试数</div>
    </div>
    <div class="stat-card passed">
      <div class="number">{passed}</div><div class="label">通过</div>
    </div>
    <div class="stat-card failed">
      <div class="number">{failed}</div><div class="label">失败</div>
    </div>
    <div class="stat-card skipped">
      <div class="number">{skipped}</div><div class="label">跳过</div>
    </div>
    <div class="stat-card duration">
      <div class="number">{total_duration:.2f}s</div><div class="label">总耗时</div>
    </div>
  </div>

  <!-- 通过率进度条 -->
  <div class="section">
    <h2>📈 通过率</h2>
    <div class="progress-bar">
      <div class="pass" style="width:{pass_rate:.1f}%"></div>
      <div class="fail" style="width:{fail_rate:.1f}%"></div>
      <div class="skip" style="width:{skip_rate:.1f}%"></div>
    </div>
    <div style="text-align:center;font-size:18px;font-weight:700;color:{pass_color}">
      通过率: {pass_rate:.1f}%
    </div>
  </div>

  <!-- 覆盖率（可选） -->
  {coverage_section}

  <!-- 测试详情 -->
  <div class="section">
    <h2>📋 测试详情</h2>
    <table>
      <thead>
        <tr><th>#</th><th>测试用例</th><th>状态</th><th>耗时</th></tr>
      </thead>
      <tbody>
        {test_rows}
      </tbody>
    </table>
  </div>

  <!-- 失败详情 -->
  {failure_section}

  <div class="footer">
    nuedc-asset-library - 测试报告生成器 | 自动生成于 {generated_at}
  </div>
</div>
</body>
</html>"""


def run_pytest_and_collect(pytest_dir, pytest_args):
    """运行 pytest 并收集 JSON 结果"""
    import subprocess
    result_json = os.path.join(pytest_dir, ".pytest_results.json")

    cmd = [
        sys.executable, "-m", "pytest",
        pytest_dir,
        f"--json-report", f"--json-report-file={result_json}",
        "-v",
    ]
    if pytest_args:
        cmd.extend(pytest_args.split())

    try:
        subprocess.run(cmd, capture_output=False, timeout=300)
    except FileNotFoundError:
        # pytest-json-report 可能未安装，尝试用文本输出解析
        cmd_fallback = [
            sys.executable, "-m", "pytest",
            pytest_dir, "-v", "--tb=short",
        ]
        if pytest_args:
            cmd_fallback.extend(pytest_args.split())
        result = subprocess.run(cmd_fallback, capture_output=True, text=True, timeout=300)
        return parse_pytest_text(result.stdout, result.returncode)

    if os.path.exists(result_json):
        with open(result_json, "r", encoding="utf-8") as f:
            data = json.load(f)
        return normalize_pytest_json(data)

    return None


def parse_pytest_text(text, return_code):
    """解析 pytest 文本输出"""
    import re
    tests = []
    duration = 0.0

    # 匹配测试行: test_file.py::test_name PASSED/FAILED/SKIPPED [时间]
    pattern = re.compile(r"(\S+::\S+)\s+(PASSED|FAILED|SKIPPED|ERROR)(?:\s+\[([\d.]+)s\])?")
    for m in pattern.finditer(text):
        name, status, dur = m.groups()
        tests.append({
            "nodeid": name,
            "outcome": status.lower(),
            "duration": float(dur) if dur else 0.0,
            "longrepr": "",
        })

    # 匹配总耗时
    dur_match = re.search(r"in\s+([\d.]+)s", text)
    if dur_match:
        duration = float(dur_match.group(1))

    return {
        "tests": tests,
        "duration": duration,
        "summary": {
            "passed": sum(1 for t in tests if t["outcome"] == "passed"),
            "failed": sum(1 for t in tests if t["outcome"] == "failed"),
            "skipped": sum(1 for t in tests if t["outcome"] == "skipped"),
            "error": sum(1 for t in tests if t["outcome"] == "error"),
        },
    }


def normalize_pytest_json(data):
    """标准化 pytest-json-report 输出"""
    tests = []
    for t in data.get("tests", []):
        tests.append({
            "nodeid": t.get("nodeid", ""),
            "outcome": t.get("outcome", "unknown"),
            "duration": t.get("duration", 0.0),
            "longrepr": str(t.get("call", {}).get("longrepr", "")),
        })

    summary = data.get("summary", {})
    return {
        "tests": tests,
        "duration": data.get("duration", 0.0),
        "summary": {
            "passed": summary.get("passed", 0),
            "failed": summary.get("failed", 0),
            "skipped": summary.get("skipped", 0),
            "error": summary.get("error", 0),
        },
    }


def load_pytest_json(filepath):
    """加载 pytest JSON 结果文件"""
    with open(filepath, "r", encoding="utf-8") as f:
        data = json.load(f)

    # 判断格式
    if "tests" in data:
        return normalize_pytest_json(data)
    elif "report" in data:
        return normalize_pytest_json(data["report"])
    else:
        # 尝试当作简单格式
        return data


def generate_html(data, title, coverage_data=None):
    """生成 HTML 报告"""
    tests = data.get("tests", [])
    summary = data.get("summary", {})
    duration = data.get("duration", 0.0)

    total = len(tests)
    passed = summary.get("passed", sum(1 for t in tests if t["outcome"] == "passed"))
    failed = summary.get("failed", sum(1 for t in tests if t["outcome"] == "failed"))
    skipped = summary.get("skipped", sum(1 for t in tests if t["outcome"] == "skipped"))
    errors = summary.get("error", sum(1 for t in tests if t["outcome"] == "error"))

    pass_rate = (passed / total * 100) if total > 0 else 0
    fail_rate = (failed / total * 100) if total > 0 else 0
    skip_rate = (skipped / total * 100) if total > 0 else 0

    pass_color = "#52c41a" if pass_rate >= 80 else "#faad14" if pass_rate >= 60 else "#ff4d4f"

    # 测试详情行
    test_rows = []
    for i, t in enumerate(tests, 1):
        outcome = t.get("outcome", "unknown")
        badge_class = outcome if outcome in ("passed", "failed", "skipped", "error") else "error"
        status_label = {"passed": "通过", "failed": "失败", "skipped": "跳过", "error": "错误"}.get(outcome, outcome)
        test_rows.append(
            f'<tr><td>{i}</td><td><code>{t["nodeid"]}</code></td>'
            f'<td><span class="badge {badge_class}">{status_label}</span></td>'
            f'<td>{t.get("duration", 0):.3f}s</td></tr>'
        )

    # 失败详情
    failures = [t for t in tests if t.get("outcome") == "failed" and t.get("longrepr")]
    if failures:
        failure_items = ""
        for t in failures:
            failure_items += (
                f'<h3 style="margin:12px 0 8px;font-size:14px;color:#ff4d4f">'
                f'❌ {t["nodeid"]}</h3>'
                f'<div class="failure-detail">{t["longrepr"][:2000]}</div>'
            )
        failure_section = f'<div class="section"><h2>❌ 失败详情</h2>{failure_items}</div>'
    else:
        failure_section = ""

    # 覆盖率
    coverage_section = ""
    if coverage_data:
        cov_pct = coverage_data.get("total_coverage", 0)
        cov_files = coverage_data.get("files", [])
        file_rows = ""
        for f_info in cov_files[:20]:  # 最多显示20个文件
            pct = f_info.get("coverage", 0)
            color = "#52c41a" if pct >= 80 else "#faad14" if pct >= 60 else "#ff4d4f"
            file_rows += (
                f'<tr><td><code>{f_info.get("filename","")}</code></td>'
                f'<td>{pct:.1f}%</td>'
                f'<td><div class="coverage-bar"><div class="fill" style="width:{pct}%;background:{color}"></div></div></td></tr>'
            )
        coverage_section = f"""
    <div class="section">
      <h2>📊 代码覆盖率</h2>
      <div style="text-align:center;font-size:24px;font-weight:700;color:#1890ff;margin:12px 0">
        总覆盖率: {cov_pct:.1f}%
      </div>
      <table>
        <thead><tr><th>文件</th><th>覆盖率</th><th>进度</th></tr></thead>
        <tbody>{file_rows}</tbody>
      </table>
    </div>"""

    return HTML_TEMPLATE.format(
        title=title,
        generated_at=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        total_duration=duration,
        total=total,
        passed=passed,
        failed=failed + errors,
        skipped=skipped,
        pass_rate=pass_rate,
        fail_rate=fail_rate,
        skip_rate=skip_rate,
        pass_color=pass_color,
        test_rows="\n        ".join(test_rows),
        failure_section=failure_section,
        coverage_section=coverage_section,
    )


def load_coverage_data(filepath):
    """加载 coverage JSON 数据"""
    if not filepath or not os.path.exists(filepath):
        return None

    with open(filepath, "r", encoding="utf-8") as f:
        data = json.load(f)

    # 标准 coverage.py JSON 格式
    if "totals" in data:
        total_cov = data["totals"].get("percent_covered", 0)
        files = []
        for fname, fdata in data.get("files", {}).items():
            files.append({
                "filename": fname,
                "coverage": fdata.get("summary", {}).get("percent_covered", 0),
            })
        return {"total_coverage": total_cov, "files": files}

    return data


def main():
    """主入口"""
    args = parse_args()

    # 加载数据
    if args.run_pytest:
        log_info(f"运行 pytest 目录: {args.pytest_dir}")
        data = run_pytest_and_collect(args.pytest_dir, args.pytest_args)
        if data is None:
            log_error("pytest 运行失败或无结果")
            sys.exit(1)
    elif args.input:
        data = load_pytest_json(args.input)
    else:
        log_error("请指定 --input 文件或使用 --run-pytest")
        sys.exit(1)

    # 加载覆盖率数据
    coverage_data = load_coverage_data(args.coverage)

    # 生成报告
    html = generate_html(data, args.title, coverage_data)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html)

    log_ok(f"测试报告已生成: {output_path.resolve()}")

    # 打印摘要
    summary = data.get("summary", {})
    total = len(data.get("tests", []))
    print(f"\n{'='*40}")
    print(f"  总测试数: {total}")
    print(f"  通过: {summary.get('passed', 0)}")
    print(f"  失败: {summary.get('failed', 0)}")
    print(f"  跳过: {summary.get('skipped', 0)}")
    print(f"  耗时: {data.get('duration', 0):.2f}s")
    print(f"{'='*40}")


def log_info(msg):
    print(f"[信息] {msg}")

def log_error(msg):
    print(f"[错误] {msg}", file=sys.stderr)

def log_ok(msg):
    print(f"[成功] {msg}")


if __name__ == "__main__":
    main()
