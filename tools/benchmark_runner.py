#!/usr/bin/env python3
"""
基准测试运行器 - 运行所有仿真+测试，生成性能报告
用法: python benchmark_runner.py [项目目录] [--output report.json] [--runs 3] [--verbose]
"""
import argparse
import subprocess
import sys
import os
import time
import json
import glob
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import List, Optional


@dataclass
class BenchmarkResult:
    """单个基准测试结果"""
    name: str
    file: str
    type: str  # "test" | "simulation" | "script"
    status: str  # "pass" | "fail" | "error" | "skip"
    elapsed_sec: float
    returncode: int
    stdout: str = ""
    stderr: str = ""
    runs: int = 1
    avg_sec: float = 0.0
    min_sec: float = 0.0
    max_sec: float = 0.0


def find_tests(directory: str) -> List[dict]:
    """查找所有测试文件"""
    tests = []
    test_patterns = ["test_*.py", "*_test.py", "tests/*.py"]

    for pattern in test_patterns:
        for f in glob.glob(os.path.join(directory, "**", pattern), recursive=True):
            if "__pycache__" in f or ".git" in f:
                continue
            tests.append({"file": f, "type": "test"})
    return tests


def find_simulations(directory: str) -> List[dict]:
    """查找仿真脚本"""
    sims = []
    sim_dirs = ["sim", "simulation", "simulations"]

    for sim_dir in sim_dirs:
        full_dir = os.path.join(directory, sim_dir)
        if os.path.isdir(full_dir):
            for f in Path(full_dir).glob("*.py"):
                if not f.name.startswith("_"):
                    sims.append({"file": str(f), "type": "simulation"})
    return sims


def find_main_scripts(directory: str) -> List[dict]:
    """查找可运行的主脚本"""
    scripts = []
    src_dir = os.path.join(directory, "src")
    if os.path.isdir(src_dir):
        main_file = os.path.join(src_dir, "main.py")
        if os.path.exists(main_file):
            scripts.append({"file": main_file, "type": "script"})
    return scripts


def run_benchmark(file_info: dict, num_runs: int = 3, timeout: int = 60) -> BenchmarkResult:
    """运行单个基准测试"""
    filepath = file_info["file"]
    ftype = file_info["type"]
    name = os.path.basename(filepath)

    times = []
    last_result = None

    for run_idx in range(num_runs):
        start = time.perf_counter()
        try:
            result = subprocess.run(
                [sys.executable, filepath],
                capture_output=True, text=True,
                timeout=timeout,
                cwd=os.path.dirname(filepath) or "."
            )
            elapsed = time.perf_counter() - start
            times.append(elapsed)

            if run_idx == num_runs - 1:
                last_result = result

            if result.returncode != 0 and run_idx == 0:
                # 第一次就失败，不用重试
                return BenchmarkResult(
                    name=name, file=filepath, type=ftype,
                    status="fail" if result.returncode != 0 else "pass",
                    elapsed_sec=elapsed, returncode=result.returncode,
                    stdout=result.stdout[-500:] if result.stdout else "",
                    stderr=result.stderr[-500:] if result.stderr else "",
                    runs=1, avg_sec=elapsed, min_sec=elapsed, max_sec=elapsed,
                )

        except subprocess.TimeoutExpired:
            elapsed = time.perf_counter() - start
            return BenchmarkResult(
                name=name, file=filepath, type=ftype,
                status="error", elapsed_sec=elapsed, returncode=-1,
                stderr=f"超时 ({timeout}秒)",
                runs=1, avg_sec=elapsed, min_sec=elapsed, max_sec=elapsed,
            )
        except Exception as e:
            elapsed = time.perf_counter() - start
            return BenchmarkResult(
                name=name, file=filepath, type=ftype,
                status="error", elapsed_sec=elapsed, returncode=-1,
                stderr=str(e),
                runs=1, avg_sec=elapsed, min_sec=elapsed, max_sec=elapsed,
            )

    avg = sum(times) / len(times)
    return BenchmarkResult(
        name=name, file=filepath, type=ftype,
        status="pass" if last_result.returncode == 0 else "fail",
        elapsed_sec=times[-1], returncode=last_result.returncode,
        stdout=last_result.stdout[-500:] if last_result.stdout else "",
        stderr=last_result.stderr[-500:] if last_result.stderr else "",
        runs=num_runs, avg_sec=avg, min_sec=min(times), max_sec=max(times),
    )


def generate_report(results: List[BenchmarkResult], output_path: Optional[str] = None) -> dict:
    """生成性能报告"""
    passed = [r for r in results if r.status == "pass"]
    failed = [r for r in results if r.status == "fail"]
    errors = [r for r in results if r.status == "error"]

    report = {
        "timestamp": datetime.now().isoformat(),
        "summary": {
            "total": len(results),
            "passed": len(passed),
            "failed": len(failed),
            "errors": len(errors),
            "total_time_sec": sum(r.elapsed_sec for r in results),
            "avg_time_sec": sum(r.avg_sec for r in results) / len(results) if results else 0,
        },
        "results": [asdict(r) for r in results],
    }

    if output_path:
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print(f"\n📄 报告已保存: {output_path}")

    return report


def print_report(results: List[BenchmarkResult]):
    """打印文本报告"""
    print("\n" + "=" * 85)
    print(f"{'名称':<30} {'类型':<10} {'状态':<6} {'平均耗时':>10} {'最短':>10} {'最长':>10} {'次数':>4}")
    print("=" * 85)

    for r in results:
        status_icon = {"pass": "✅", "fail": "❌", "error": "💥", "skip": "⏭️"}[r.status]
        type_label = {"test": "测试", "simulation": "仿真", "script": "脚本"}[r.type]
        avg_str = f"{r.avg_sec*1000:.1f}ms" if r.avg_sec < 1 else f"{r.avg_sec:.3f}s"
        min_str = f"{r.min_sec*1000:.1f}ms" if r.min_sec < 1 else f"{r.min_sec:.3f}s"
        max_str = f"{r.max_sec*1000:.1f}ms" if r.max_sec < 1 else f"{r.max_sec:.3f}s"
        print(f"{r.name:<30} {type_label:<10} {status_icon} {r.status:<4} {avg_str:>10} {min_str:>10} {max_str:>10} {r.runs:>4}")

    print("=" * 85)

    # 汇总
    passed = sum(1 for r in results if r.status == "pass")
    failed = sum(1 for r in results if r.status == "fail")
    errors = sum(1 for r in results if r.status == "error")
    total_time = sum(r.elapsed_sec for r in results)

    print(f"\n📊 汇总:")
    print(f"   总计: {len(results)} | ✅通过: {passed} | ❌失败: {failed} | 💥错误: {errors}")
    print(f"   总耗时: {total_time:.2f}秒")

    if failed + errors > 0:
        print(f"\n⚠️  失败/错误详情:")
        for r in results:
            if r.status in ("fail", "error"):
                print(f"   • {r.name}: {r.stderr[:200] if r.stderr else '无错误信息'}")


def main():
    parser = argparse.ArgumentParser(description="基准测试运行器")
    parser.add_argument("directory", nargs="?", default=".", help="项目目录")
    parser.add_argument("--output", "-o", help="JSON报告输出路径")
    parser.add_argument("--runs", "-r", type=int, default=3, help="每个测试运行次数 (默认3)")
    parser.add_argument("--timeout", type=int, default=60, help="单个测试超时秒数 (默认60)")
    parser.add_argument("--verbose", "-v", action="store_true", help="显示详细输出")
    parser.add_argument("--tests-only", action="store_true", help="只运行测试")
    parser.add_argument("--sims-only", action="store_true", help="只运行仿真")
    args = parser.parse_args()

    directory = os.path.abspath(args.directory)
    print(f"🚀 基准测试运行器")
    print(f"   目录: {directory}")
    print(f"   每项运行 {args.runs} 次, 超时 {args.timeout}秒\n")

    # 发现测试项
    items = []
    if not args.sims_only:
        items.extend(find_tests(directory))
    if not args.tests_only:
        items.extend(find_simulations(directory))
        items.extend(find_main_scripts(directory))

    # 去重
    seen = set()
    unique_items = []
    for item in items:
        if item["file"] not in seen:
            seen.add(item["file"])
            unique_items.append(item)
    items = unique_items

    if not items:
        print("⚠️  未找到任何测试、仿真或主脚本")
        sys.exit(0)

    print(f"📋 发现 {len(items)} 个项目:")
    for item in items:
        rel = os.path.relpath(item["file"], directory)
        print(f"   • [{item['type']}] {rel}")
    print()

    # 运行
    results = []
    for i, item in enumerate(items, 1):
        rel = os.path.relpath(item["file"], directory)
        print(f"[{i}/{len(items)}] 运行 {rel} ...", end=" ", flush=True)
        result = run_benchmark(item, num_runs=args.runs, timeout=args.timeout)
        results.append(result)

        status_icon = {"pass": "✅", "fail": "❌", "error": "💥", "skip": "⏭️"}[result.status]
        avg_str = f"{result.avg_sec*1000:.1f}ms" if result.avg_sec < 1 else f"{result.avg_sec:.3f}s"
        print(f"{status_icon} {avg_str}")

        if args.verbose and result.stderr:
            print(f"   stderr: {result.stderr[:200]}")

    # 报告
    print_report(results)

    if args.output:
        generate_report(results, args.output)
    elif not args.output:
        # 默认保存
        report_path = os.path.join(directory, "benchmark_report.json")
        generate_report(results, report_path)

    # 退出码
    failures = sum(1 for r in results if r.status in ("fail", "error"))
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
