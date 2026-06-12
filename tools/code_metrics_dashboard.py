#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
代码指标仪表盘
==============
功能：
  - Web界面展示代码质量评分、圈复杂度、代码行数
  - 集成 pytest 覆盖率数据
  - 交互式图表（Chart.js）
  - 文件级/目录级指标视图
  - 趋势分析（如有历史数据）

依赖：
  - 标准库即可运行（内置HTTP服务器）
  - 可选: radon（复杂度分析）、coverage（覆盖率）

用法：
  python code_metrics_dashboard.py --path ./src              # 分析src目录
  python code_metrics_dashboard.py --path . --port 8080      # 指定端口
  python code_metrics_dashboard.py --path . --coverage coverage.json
"""

import argparse
import http.server
import json
import os
import re
import subprocess
import sys
import threading
import webbrowser
from datetime import datetime
from pathlib import Path


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description="代码指标仪表盘 - Web界面展示代码质量")
    parser.add_argument("--path", "-p", type=str, default=".",
                        help="待分析的代码目录（默认当前目录）")
    parser.add_argument("--port", type=int, default=8088,
                        help="Web服务器端口（默认 8088）")
    parser.add_argument("--coverage", type=str, default=None,
                        help="coverage JSON文件路径")
    parser.add_argument("--open", action="store_true",
                        help="自动打开浏览器")
    parser.add_argument("--report", type=str, default=None,
                        help="生成静态HTML报告到指定路径（不启动服务器）")
    return parser.parse_args()


# ============================================================
# 代码指标收集
# ============================================================

def count_lines(filepath):
    """统计文件行数（总行数、代码行、注释行、空行）"""
    total = 0
    code = 0
    comment = 0
    blank = 0

    in_block_comment = False
    ext = Path(filepath).suffix.lower()

    try:
        with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                total += 1
                stripped = line.strip()

                if not stripped:
                    blank += 1
                    continue

                # 块注释处理
                if in_block_comment:
                    comment += 1
                    if "*/" in stripped or '"""' in stripped or "'''" in stripped:
                        in_block_comment = False
                    continue

                if ext in (".c", ".h", ".cpp", ".hpp", ".java", ".js", ".ts"):
                    if stripped.startswith("//") or stripped.startswith("/*"):
                        comment += 1
                        if "/*" in stripped and "*/" not in stripped:
                            in_block_comment = True
                        continue
                elif ext in (".py",):
                    if stripped.startswith("#"):
                        comment += 1
                        continue
                    if stripped.startswith('"""') or stripped.startswith("'''"):
                        comment += 1
                        if stripped.count('"""') == 1 or stripped.count("'''") == 1:
                            in_block_comment = True
                        continue

                code += 1
    except Exception:
        pass

    return {"total": total, "code": code, "comment": comment, "blank": blank}


def analyze_complexity_radon(filepath):
    """使用 radon 分析圈复杂度"""
    try:
        result = subprocess.run(
            [sys.executable, "-m", "radon", "cc", filepath, "-j", "-s"],
            capture_output=True, text=True, timeout=30,
        )
        if result.returncode == 0 and result.stdout.strip():
            data = json.loads(result.stdout)
            complexities = []
            for fname, blocks in data.items():
                for block in blocks:
                    complexities.append({
                        "name": block.get("name", ""),
                        "type": block.get("type", ""),
                        "complexity": block.get("complexity", 0),
                        "rank": block.get("rank", "?"),
                        "lineno": block.get("lineno", 0),
                    })
            return complexities
    except (subprocess.SubprocessError, json.JSONDecodeError, FileNotFoundError):
        pass
    return []


def analyze_complexity_simple(filepath):
    """简单圈复杂度估算（不依赖radon）"""
    try:
        with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
            content = f.read()
    except Exception:
        return []

    ext = Path(filepath).suffix.lower()

    if ext == ".py":
        patterns = [
            r"\bif\b", r"\belif\b", r"\bfor\b", r"\bwhile\b",
            r"\band\b", r"\bor\b", r"\bexcept\b", r"\bwith\b",
        ]
    elif ext in (".c", ".h", ".cpp", ".java", ".js"):
        patterns = [
            r"\bif\s*\(", r"\belse\s+if\b", r"\bfor\s*\(",
            r"\bwhile\s*\(", r"&&", r"\|\|", r"\bcase\b", r"\bcatch\b",
        ]
    else:
        return []

    count = 1  # 基础复杂度
    for pat in patterns:
        count += len(re.findall(pat, content))

    return [{"name": os.path.basename(filepath), "complexity": count, "rank": _cc_rank(count)}]


def _cc_rank(cc):
    """圈复杂度等级"""
    if cc <= 5:
        return "A"
    elif cc <= 10:
        return "B"
    elif cc <= 20:
        return "C"
    elif cc <= 30:
        return "D"
    else:
        return "F"


def scan_directory(base_path):
    """扫描目录，收集所有代码文件的指标"""
    code_exts = {".c", ".h", ".cpp", ".hpp", ".py", ".java", ".js", ".ts", ".m"}
    files = []
    base = Path(base_path)

    for root, dirs, filenames in os.walk(base):
        # 跳过常见非代码目录
        dirs[:] = [d for d in dirs if d not in (
            ".git", "node_modules", "__pycache__", ".venv", "venv",
            "build", "dist", ".idea", ".vscode",
        )]
        for fname in filenames:
            ext = Path(fname).suffix.lower()
            if ext in code_exts:
                fpath = os.path.join(root, fname)
                rel_path = os.path.relpath(fpath, base)

                lines = count_lines(fpath)

                # 尝试 radon，否则简单估算
                complexity = analyze_complexity_radon(fpath)
                if not complexity:
                    complexity = analyze_complexity_simple(fpath)

                avg_cc = 0
                if complexity:
                    avg_cc = sum(c["complexity"] for c in complexity) / len(complexity)

                files.append({
                    "path": rel_path,
                    "ext": ext,
                    "lines": lines,
                    "complexity": complexity,
                    "avg_cc": round(avg_cc, 1),
                })

    return files


def load_coverage_data(filepath):
    """加载覆盖率数据"""
    if not filepath or not os.path.exists(filepath):
        return None
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        if "totals" in data:
            return {
                "total_percent": data["totals"].get("percent_covered", 0),
                "files": {
                    k: v.get("summary", {}).get("percent_covered", 0)
                    for k, v in data.get("files", {}).items()
                },
            }
    except Exception:
        pass
    return None


def compute_quality_score(files_data, coverage_data=None):
    """计算综合质量评分 (0-100)"""
    if not files_data:
        return 0

    score = 100.0

    # 圈复杂度扣分
    all_cc = [f["avg_cc"] for f in files_data if f["avg_cc"] > 0]
    if all_cc:
        avg_cc = sum(all_cc) / len(all_cc)
        if avg_cc > 10:
            score -= min(30, (avg_cc - 10) * 3)

    # 注释率加分
    total_code = sum(f["lines"]["code"] for f in files_data)
    total_comment = sum(f["lines"]["comment"] for f in files_data)
    if total_code > 0:
        comment_ratio = total_comment / total_code
        if comment_ratio < 0.1:
            score -= 10
        elif comment_ratio > 0.3:
            score += 5

    # 覆盖率
    if coverage_data:
        cov = coverage_data.get("total_percent", 0)
        if cov < 50:
            score -= 20
        elif cov < 80:
            score -= 10

    return max(0, min(100, round(score)))


# ============================================================
# HTML 模板
# ============================================================

DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>代码指标仪表盘</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
  * {{ margin: 0; padding: 0; box-sizing: border-box; }}
  body {{
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Microsoft YaHei', sans-serif;
    background: #f0f2f5; color: #333;
  }}
  .navbar {{
    background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
    color: white; padding: 16px 32px; display: flex; align-items: center; gap: 16px;
  }}
  .navbar h1 {{ font-size: 20px; }}
  .navbar .time {{ margin-left: auto; opacity: 0.7; font-size: 13px; }}

  .container {{ max-width: 1200px; margin: 0 auto; padding: 24px; }}

  .cards {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 16px; margin-bottom: 24px; }}
  .card {{
    background: white; border-radius: 10px; padding: 24px; text-align: center;
    box-shadow: 0 2px 8px rgba(0,0,0,0.08); position: relative; overflow: hidden;
  }}
  .card::before {{
    content: ''; position: absolute; top: 0; left: 0; right: 0; height: 4px;
  }}
  .card.quality::before {{ background: linear-gradient(90deg, #52c41a, #73d13d); }}
  .card.complexity::before {{ background: linear-gradient(90deg, #faad14, #ffc53d); }}
  .card.coverage::before {{ background: linear-gradient(90deg, #1890ff, #40a9ff); }}
  .card.files::before {{ background: linear-gradient(90deg, #722ed1, #9254de); }}
  .card .number {{ font-size: 40px; font-weight: 700; }}
  .card .label {{ font-size: 13px; color: #999; margin-top: 4px; }}
  .card.quality .number {{ color: #52c41a; }}
  .card.complexity .number {{ color: #faad14; }}
  .card.coverage .number {{ color: #1890ff; }}
  .card.files .number {{ color: #722ed1; }}

  .panel {{
    background: white; border-radius: 10px; padding: 24px; margin-bottom: 24px;
    box-shadow: 0 2px 8px rgba(0,0,0,0.08);
  }}
  .panel h2 {{ font-size: 16px; margin-bottom: 16px; padding-bottom: 8px; border-bottom: 2px solid #f0f0f0; }}

  .chart-row {{ display: grid; grid-template-columns: 1fr 1fr; gap: 24px; margin-bottom: 24px; }}
  @media (max-width: 768px) {{ .chart-row {{ grid-template-columns: 1fr; }} }}

  table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
  th {{ background: #fafafa; padding: 10px 12px; text-align: left; font-weight: 600; border-bottom: 2px solid #e8e8e8; }}
  td {{ padding: 8px 12px; border-bottom: 1px solid #f0f0f0; }}
  tr:hover {{ background: #f9f9ff; }}

  .rank {{ display: inline-block; width: 24px; height: 24px; line-height: 24px; text-align: center;
           border-radius: 4px; font-weight: 700; font-size: 12px; color: white; }}
  .rank.A {{ background: #52c41a; }}
  .rank.B {{ background: #73d13d; }}
  .rank.C {{ background: #faad14; }}
  .rank.D {{ background: #ff7a45; }}
  .rank.F {{ background: #ff4d4f; }}

  .footer {{ text-align: center; padding: 16px; color: #999; font-size: 12px; }}
</style>
</head>
<body>
<div class="navbar">
  <h1>📊 代码指标仪表盘</h1>
  <div class="time">分析时间: {generated_at}</div>
</div>
<div class="container">
  <!-- 顶部指标卡片 -->
  <div class="cards">
    <div class="card quality">
      <div class="number">{quality_score}</div>
      <div class="label">质量评分 (0-100)</div>
    </div>
    <div class="card complexity">
      <div class="number">{avg_complexity}</div>
      <div class="label">平均圈复杂度</div>
    </div>
    <div class="card coverage">
      <div class="number">{coverage_pct}%</div>
      <div class="label">代码覆盖率</div>
    </div>
    <div class="card files">
      <div class="number">{file_count}</div>
      <div class="label">代码文件数</div>
    </div>
  </div>

  <!-- 图表区域 -->
  <div class="chart-row">
    <div class="panel">
      <h2>📁 文件类型分布</h2>
      <canvas id="extChart" height="200"></canvas>
    </div>
    <div class="panel">
      <h2>📊 代码行数分布 (Top 10)</h2>
      <canvas id="locChart" height="200"></canvas>
    </div>
  </div>

  <div class="chart-row">
    <div class="panel">
      <h2>🔄 圈复杂度分布</h2>
      <canvas id="ccChart" height="200"></canvas>
    </div>
    <div class="panel">
      <h2>📝 代码 / 注释 / 空行占比</h2>
      <canvas id="lineChart" height="200"></canvas>
    </div>
  </div>

  <!-- 文件详情表 -->
  <div class="panel">
    <h2>📋 文件详情 ({file_count} 个文件)</h2>
    <table>
      <thead>
        <tr><th>文件</th><th>代码行</th><th>注释行</th><th>空行</th><th>圈复杂度</th><th>等级</th></tr>
      </thead>
      <tbody>
        {file_rows}
      </tbody>
    </table>
  </div>

  <div class="footer">
    nuedc-asset-library - 代码指标仪表盘 | 自动生成于 {generated_at}
  </div>
</div>

<script>
// 文件类型分布饼图
new Chart(document.getElementById('extChart'), {{
  type: 'doughnut',
  data: {{
    labels: {ext_labels},
    datasets: [{{ data: {ext_counts}, backgroundColor: ['#52c41a','#1890ff','#faad14','#722ed1','#ff4d4f','#ff7a45','#36cfc9','#ffc53d'] }}]
  }},
  options: {{ responsive: true, plugins: {{ legend: {{ position: 'bottom' }} }} }}
}});

// 代码行数 Top 10 条形图
new Chart(document.getElementById('locChart'), {{
  type: 'bar',
  data: {{
    labels: {loc_labels},
    datasets: [
      {{ label: '代码', data: {loc_code}, backgroundColor: '#1890ff' }},
      {{ label: '注释', data: {loc_comment}, backgroundColor: '#52c41a' }},
    ]
  }},
  options: {{
    responsive: true, indexAxis: 'y',
    scales: {{ x: {{ stacked: true }}, y: {{ stacked: true }} }},
    plugins: {{ legend: {{ position: 'bottom' }} }}
  }}
}});

// 圈复杂度分布
new Chart(document.getElementById('ccChart'), {{
  type: 'bar',
  data: {{
    labels: ['A (1-5)', 'B (6-10)', 'C (11-20)', 'D (21-30)', 'F (>30)'],
    datasets: [{{ label: '文件数', data: {cc_dist}, backgroundColor: ['#52c41a','#73d13d','#faad14','#ff7a45','#ff4d4f'] }}]
  }},
  options: {{ responsive: true, plugins: {{ legend: {{ display: false }} }} }}
}});

// 代码/注释/空行饼图
new Chart(document.getElementById('lineChart'), {{
  type: 'pie',
  data: {{
    labels: ['代码', '注释', '空行'],
    datasets: [{{ data: [{total_code}, {total_comment}, {total_blank}], backgroundColor: ['#1890ff','#52c41a','#d9d9d9'] }}]
  }},
  options: {{ responsive: true, plugins: {{ legend: {{ position: 'bottom' }} }} }}
}});
</script>
</body>
</html>"""


def generate_dashboard_html(files_data, coverage_data=None, base_path="."):
    """生成仪表盘 HTML"""
    # 统计汇总
    total_code = sum(f["lines"]["code"] for f in files_data)
    total_comment = sum(f["lines"]["comment"] for f in files_data)
    total_blank = sum(f["lines"]["blank"] for f in files_data)

    # 平均复杂度
    all_cc = [f["avg_cc"] for f in files_data if f["avg_cc"] > 0]
    avg_cc = round(sum(all_cc) / len(all_cc), 1) if all_cc else 0

    # 质量评分
    quality = compute_quality_score(files_data, coverage_data)

    # 覆盖率
    coverage_pct = coverage_data.get("total_percent", 0) if coverage_data else 0

    # 文件类型统计
    ext_count = {}
    for f in files_data:
        ext = f["ext"]
        ext_count[ext] = ext_count.get(ext, 0) + 1
    ext_labels = json.dumps(list(ext_count.keys()))
    ext_counts = json.dumps(list(ext_count.values()))

    # Top 10 代码行数
    sorted_files = sorted(files_data, key=lambda x: x["lines"]["code"], reverse=True)[:10]
    loc_labels = json.dumps([os.path.basename(f["path"]) for f in sorted_files])
    loc_code = json.dumps([f["lines"]["code"] for f in sorted_files])
    loc_comment = json.dumps([f["lines"]["comment"] for f in sorted_files])

    # 圈复杂度分布
    cc_dist = [0, 0, 0, 0, 0]
    for f in files_data:
        cc = f["avg_cc"]
        if cc <= 5: cc_dist[0] += 1
        elif cc <= 10: cc_dist[1] += 1
        elif cc <= 20: cc_dist[2] += 1
        elif cc <= 30: cc_dist[3] += 1
        else: cc_dist[4] += 1

    # 文件详情行
    file_rows = []
    for f in sorted(files_data, key=lambda x: x["lines"]["code"], reverse=True):
        rank = "A"
        if f["complexity"]:
            ranks = [c.get("rank", "A") for c in f["complexity"]]
            rank = min(ranks) if ranks else "A"  # 取最差等级
        file_rows.append(
            f'<tr><td><code>{f["path"]}</code></td>'
            f'<td>{f["lines"]["code"]}</td>'
            f'<td>{f["lines"]["comment"]}</td>'
            f'<td>{f["lines"]["blank"]}</td>'
            f'<td>{f["avg_cc"]}</td>'
            f'<td><span class="rank {rank}">{rank}</span></td></tr>'
        )

    return DASHBOARD_HTML.format(
        generated_at=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        quality_score=quality,
        avg_complexity=avg_cc,
        coverage_pct=f"{coverage_pct:.1f}",
        file_count=len(files_data),
        ext_labels=ext_labels,
        ext_counts=ext_counts,
        loc_labels=loc_labels,
        loc_code=loc_code,
        loc_comment=loc_comment,
        cc_dist=json.dumps(cc_dist),
        total_code=total_code,
        total_comment=total_comment,
        total_blank=total_blank,
        file_rows="\n        ".join(file_rows),
    )


def serve_html(html_content, port):
    """启动本地 HTTP 服务器展示仪表盘"""

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(html_content.encode("utf-8"))

        def log_message(self, format, *args):
            pass  # 静默日志

    server = http.server.HTTPServer(("127.0.0.1", port), Handler)
    url = f"http://127.0.0.1:{port}"
    print(f"[信息] 仪表盘已启动: {url}")
    print("[信息] 按 Ctrl+C 停止服务器")

    # 自动打开浏览器
    threading.Timer(0.5, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[信息] 服务器已停止")


def main():
    """主入口"""
    args = parse_args()

    base_path = os.path.abspath(args.path)
    if not os.path.isdir(base_path):
        print(f"[错误] 目录不存在: {base_path}")
        sys.exit(1)

    print(f"[信息] 分析目录: {base_path}")

    # 收集指标
    files_data = scan_directory(base_path)
    print(f"[信息] 扫描到 {len(files_data)} 个代码文件")

    # 加载覆盖率
    coverage_data = None
    if args.coverage:
        coverage_data = load_coverage_data(args.coverage)
        if coverage_data:
            print(f"[信息] 覆盖率: {coverage_data.get('total_percent', 0):.1f}%")

    # 生成 HTML
    html = generate_dashboard_html(files_data, coverage_data, base_path)

    # 输出模式
    if args.report:
        output = Path(args.report)
        output.parent.mkdir(parents=True, exist_ok=True)
        with open(output, "w", encoding="utf-8") as f:
            f.write(html)
        print(f"[成功] 报告已生成: {output.resolve()}")
    else:
        serve_html(html, args.port)


if __name__ == "__main__":
    main()
