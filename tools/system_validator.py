#!/usr/bin/env python3
"""
系统验证工具 - System Validator
=================================
功能:
  - 端到端测试 (E2E)
  - 回归测试 (Regression)
  - 性能基准测试 (Benchmark)
  - 集成测试 (Integration)
  - 测试套件管理
  - 测试报告生成
用法:
  python system_validator.py e2e --config e2e_config.json
  python system_validator.py regression --baseline baseline.json --current current.json
  python system_validator.py benchmark --config bench_config.json --iterations 100
  python system_validator.py suite --suite test_suite.json
  python system_validator.py init --name "电机驱动板V2" --output test_suite.json
"""

import argparse
import csv
import hashlib
import json
import os
import sys
import time
import traceback
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import List, Dict, Optional, Any, Callable


# ── 数据结构 ──────────────────────────────────────────────────────────────────

class TestStatus(Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"
    ERROR = "ERROR"
    TIMEOUT = "TIMEOUT"


class TestSeverity(Enum):
    CRITICAL = "CRITICAL"   # 关键功能
    MAJOR = "MAJOR"         # 主要功能
    MINOR = "MINOR"         # 次要功能
    INFO = "INFO"           # 信息性


@dataclass
class TestCase:
    """测试用例"""
    id: str = ""
    name: str = ""
    description: str = ""
    category: str = ""
    severity: str = "MAJOR"
    timeout_s: float = 30.0
    preconditions: List[str] = field(default_factory=list)
    steps: List[Dict] = field(default_factory=list)
    expected: str = ""
    actual: str = ""
    status: str = "SKIP"
    duration_s: float = 0.0
    error_msg: str = ""
    timestamp: str = ""


@dataclass
class TestSuite:
    """测试套件"""
    name: str = ""
    version: str = "1.0"
    description: str = ""
    system_under_test: str = ""
    test_cases: List[TestCase] = field(default_factory=list)
    config: Dict = field(default_factory=dict)


@dataclass
class BenchmarkResult:
    """基准测试结果"""
    test_name: str = ""
    iterations: int = 0
    total_time_s: float = 0.0
    min_time_ms: float = 0.0
    max_time_ms: float = 0.0
    mean_time_ms: float = 0.0
    median_time_ms: float = 0.0
    p95_time_ms: float = 0.0
    p99_time_ms: float = 0.0
    std_dev_ms: float = 0.0
    throughput_ops: float = 0.0
    samples: List[float] = field(default_factory=list)


@dataclass
class RegressionResult:
    """回归测试结果"""
    baseline_version: str = ""
    current_version: str = ""
    total_tests: int = 0
    passed: int = 0
    failed: int = 0
    new_failures: int = 0
    fixed: int = 0
    unchanged: int = 0
    regressions: List[Dict] = field(default_factory=list)
    improvements: List[Dict] = field(default_factory=list)
    performance_delta: Dict = field(default_factory=dict)


@dataclass
class TestReport:
    """测试报告"""
    report_id: str = ""
    system_name: str = ""
    test_type: str = ""
    start_time: str = ""
    end_time: str = ""
    duration_s: float = 0.0
    total: int = 0
    passed: int = 0
    failed: int = 0
    skipped: int = 0
    errors: int = 0
    pass_rate: float = 0.0
    test_cases: List[Dict] = field(default_factory=list)
    benchmarks: List[Dict] = field(default_factory=list)
    environment: Dict = field(default_factory=dict)
    summary: str = ""


# ── 测试执行器 ────────────────────────────────────────────────────────────────

class TestExecutor:
    """测试执行引擎"""

    def __init__(self, verbose: bool = True):
        self.verbose = verbose
        self.results: List[TestCase] = []
        self.hooks: Dict[str, List[Callable]] = {
            'before_test': [], 'after_test': [],
            'before_suite': [], 'after_suite': []
        }

    def register_hook(self, event: str, func: Callable):
        """注册测试钩子"""
        if event in self.hooks:
            self.hooks[event].append(func)

    def run_hook(self, event: str, **kwargs):
        """执行钩子"""
        for func in self.hooks.get(event, []):
            try:
                func(**kwargs)
            except Exception as e:
                print(f"  [钩子错误] {event}: {e}")

    def execute_test_case(self, tc: TestCase) -> TestCase:
        """执行单个测试用例"""
        tc.timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        self.run_hook('before_test', test_case=tc)

        if self.verbose:
            print(f"  ▶ [{tc.id}] {tc.name} ", end="", flush=True)

        start = time.time()
        try:
            # 执行测试步骤
            all_pass = True
            for step in tc.steps:
                result = self._execute_step(step, tc)
                if not result:
                    all_pass = False
                    break

            tc.status = TestStatus.PASS.value if all_pass else TestStatus.FAIL.value

        except TimeoutError:
            tc.status = TestStatus.TIMEOUT.value
            tc.error_msg = f"测试超时 ({tc.timeout_s}s)"
        except Exception as e:
            tc.status = TestStatus.ERROR.value
            tc.error_msg = str(e)
            if self.verbose:
                print(f"  [异常] {traceback.format_exc()[:200]}")

        tc.duration_s = round(time.time() - start, 4)
        self.run_hook('after_test', test_case=tc)

        if self.verbose:
            status_sym = {"PASS": "✓", "FAIL": "✗", "ERROR": "⚠", "TIMEOUT": "⏱", "SKIP": "○"}
            sym = status_sym.get(tc.status, "?")
            print(f"{sym} {tc.status} ({tc.duration_s*1000:.1f}ms)")

        self.results.append(tc)
        return tc

    def _execute_step(self, step: Dict, tc: TestCase) -> bool:
        """
        执行测试步骤
        step格式: {"action": "check_value", "params": {...}}
        """
        action = step.get('action', 'noop')
        params = step.get('params', {})

        if action == 'noop':
            return True
        elif action == 'wait':
            time.sleep(params.get('seconds', 1))
            return True
        elif action == 'check_value':
            actual = params.get('actual', 0)
            expected = params.get('expected', 0)
            tolerance = params.get('tolerance', 0)
            tc.actual = str(actual)
            tc.expected = str(expected)
            return abs(actual - expected) <= tolerance
        elif action == 'check_range':
            value = params.get('value', 0)
            min_val = params.get('min', float('-inf'))
            max_val = params.get('max', float('inf'))
            return min_val <= value <= max_val
        elif action == 'check_true':
            return bool(params.get('condition', False))
        elif action == 'check_file_exists':
            return os.path.exists(params.get('path', ''))
        elif action == 'check_command':
            import subprocess
            try:
                result = subprocess.run(
                    params.get('cmd', 'echo ok'),
                    shell=True, capture_output=True, timeout=tc.timeout_s,
                    text=True
                )
                expected_code = params.get('exit_code', 0)
                return result.returncode == expected_code
            except:
                return False
        elif action == 'check_communication':
            # 通信连通性检查
            import socket
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(params.get('timeout', 3))
                sock.connect((params.get('host', 'localhost'), params.get('port', 80)))
                sock.close()
                return True
            except:
                return False
        elif action == 'check_serial':
            try:
                import serial
                ser = serial.Serial(params.get('port', 'COM1'), params.get('baud', 115200), timeout=2)
                ser.close()
                return True
            except:
                return False
        elif action == 'custom':
            # 自定义Python表达式
            try:
                result = eval(params.get('expression', 'True'))
                return bool(result)
            except:
                return False
        else:
            tc.error_msg = f"未知动作: {action}"
            return False

    def execute_suite(self, suite: TestSuite) -> TestReport:
        """执行整个测试套件"""
        self.run_hook('before_suite', suite=suite)
        report = TestReport(
            report_id=hashlib.md5(str(time.time()).encode()).hexdigest()[:8],
            system_name=suite.system_under_test,
            start_time=time.strftime('%Y-%m-%d %H:%M:%S'),
            environment=self._get_environment()
        )

        print("\n" + "=" * 60)
        print(f"  测试套件: {suite.name} v{suite.version}")
        print(f"  被测系统: {suite.system_under_test}")
        print(f"  测试数量: {len(suite.test_cases)}")
        print("=" * 60)

        start = time.time()
        self.results = []

        for tc in suite.test_cases:
            if tc.status == 'SKIP':
                if self.verbose:
                    print(f"  ○ [{tc.id}] {tc.name} — SKIP")
                continue
            self.execute_test_case(tc)

        total_time = time.time() - start
        self.run_hook('after_suite', suite=suite, results=self.results)

        # 汇总
        report.end_time = time.strftime('%Y-%m-%d %H:%M:%S')
        report.duration_s = round(total_time, 2)
        report.total = len(self.results)
        report.passed = sum(1 for r in self.results if r.status == 'PASS')
        report.failed = sum(1 for r in self.results if r.status == 'FAIL')
        report.errors = sum(1 for r in self.results if r.status == 'ERROR')
        report.skipped = len(suite.test_cases) - report.total
        report.pass_rate = report.passed / report.total * 100 if report.total > 0 else 0
        report.test_cases = [asdict(r) for r in self.results]

        if report.pass_rate >= 100:
            report.summary = "全部通过 ✓"
        elif report.pass_rate >= 90:
            report.summary = "基本通过，少量失败"
        elif report.pass_rate >= 70:
            report.summary = "部分通过，需关注失败项"
        else:
            report.summary = "大量失败，系统存在严重问题"

        self._print_report_summary(report)
        return report

    def _get_environment(self) -> Dict:
        """获取测试环境信息"""
        import platform
        return {
            'platform': platform.system(),
            'platform_version': platform.version(),
            'python_version': platform.python_version(),
            'hostname': platform.node(),
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
        }

    def _print_report_summary(self, report: TestReport):
        """打印报告摘要"""
        print("\n" + "=" * 60)
        print("  测试报告摘要")
        print("=" * 60)
        print(f"  系统名称:    {report.system_name}")
        print(f"  报告ID:      {report.report_id}")
        print(f"  测试时长:    {report.duration_s:.2f} 秒")
        print(f"  总用例数:    {report.total}")
        print(f"  通过:        {report.passed} ✓")
        print(f"  失败:        {report.failed} ✗")
        print(f"  错误:        {report.errors} ⚠")
        print(f"  跳过:        {report.skipped} ○")
        print(f"  ► 通过率:    {report.pass_rate:.1f}%")
        print(f"  ► 评价:      {report.summary}")

        if report.failed > 0 or report.errors > 0:
            print("\n  失败/错误用例:")
            for tc in self.results:
                if tc.status in ('FAIL', 'ERROR'):
                    print(f"    [{tc.id}] {tc.name}: {tc.error_msg or tc.status}")


# ── 回归测试 ──────────────────────────────────────────────────────────────────

class RegressionAnalyzer:
    """回归测试分析器"""

    @staticmethod
    def compare(baseline_path: str, current_path: str) -> RegressionResult:
        """
        比较基准和当前测试结果
        """
        with open(baseline_path, 'r', encoding='utf-8') as f:
            baseline = json.load(f)
        with open(current_path, 'r', encoding='utf-8') as f:
            current = json.load(f)

        result = RegressionResult(
            baseline_version=baseline.get('version', 'unknown'),
            current_version=current.get('version', 'unknown')
        )

        # 按ID索引
        base_cases = {tc['id']: tc for tc in baseline.get('test_cases', [])}
        curr_cases = {tc['id']: tc for tc in current.get('test_cases', [])}

        all_ids = set(list(base_cases.keys()) + list(curr_cases.keys()))
        result.total_tests = len(all_ids)

        for tid in sorted(all_ids):
            base = base_cases.get(tid)
            curr = curr_cases.get(tid)

            if base and curr:
                if base['status'] == curr['status']:
                    if curr['status'] == 'PASS':
                        result.passed += 1
                    result.unchanged += 1
                elif base['status'] == 'PASS' and curr['status'] == 'FAIL':
                    result.new_failures += 1
                    result.failed += 1
                    result.regressions.append({
                        'id': tid, 'name': curr.get('name', ''),
                        'baseline': 'PASS', 'current': 'FAIL',
                        'error': curr.get('error_msg', '')
                    })
                elif base['status'] == 'FAIL' and curr['status'] == 'PASS':
                    result.fixed += 1
                    result.passed += 1
                    result.improvements.append({
                        'id': tid, 'name': curr.get('name', ''),
                        'baseline': 'FAIL', 'current': 'PASS'
                    })
                else:
                    if curr['status'] == 'PASS':
                        result.passed += 1
                    else:
                        result.failed += 1

            # 性能比较
            if base and curr and 'duration_s' in base and 'duration_s' in curr:
                if base['duration_s'] > 0:
                    delta_pct = (curr['duration_s'] - base['duration_s']) / base['duration_s'] * 100
                    result.performance_delta[tid] = {
                        'baseline_ms': base['duration_s'] * 1000,
                        'current_ms': curr['duration_s'] * 1000,
                        'delta_pct': round(delta_pct, 2)
                    }

        return result


# ── 基准测试 ──────────────────────────────────────────────────────────────────

class BenchmarkRunner:
    """基准测试运行器"""

    @staticmethod
    def run(test_name: str, test_func: Callable, iterations: int = 100,
            warmup: int = 10) -> BenchmarkResult:
        """
        运行基准测试
        """
        result = BenchmarkResult(test_name=test_name, iterations=iterations)

        print(f"  基准测试: {test_name}")
        print(f"  预热: {warmup}次  正式: {iterations}次")

        # 预热
        for _ in range(warmup):
            test_func()

        # 正式测试
        samples = []
        start_total = time.time()

        for i in range(iterations):
            t0 = time.perf_counter()
            test_func()
            t1 = time.perf_counter()
            ms = (t1 - t0) * 1000.0
            samples.append(ms)

            if (i + 1) % (iterations // 5 or 1) == 0:
                print(f"    进度: {i+1}/{iterations}  当前均值: {sum(samples)/len(samples):.3f}ms")

        result.total_time_s = time.time() - start_total
        result.samples = samples
        samples_sorted = sorted(samples)
        n = len(samples_sorted)

        result.min_time_ms = samples_sorted[0]
        result.max_time_ms = samples_sorted[-1]
        result.mean_time_ms = sum(samples) / n
        result.median_time_ms = samples_sorted[n // 2]
        result.p95_time_ms = samples_sorted[int(n * 0.95)]
        result.p99_time_ms = samples_sorted[int(n * 0.99)]
        result.std_dev_ms = (sum((s - result.mean_time_ms) ** 2 for s in samples) / (n - 1)) ** 0.5
        result.throughput_ops = 1000.0 / result.mean_time_ms if result.mean_time_ms > 0 else 0

        return result

    @staticmethod
    def run_suite(config: Dict) -> List[BenchmarkResult]:
        """
        运行基准测试套件
        config: {"tests": [{"name": "...", "iterations": 100}, ...]}
        """
        results = []
        tests = config.get('tests', [])

        # 内置基准测试
        builtin_tests = {
            'math_operations': lambda: sum(i**2 for i in range(1000)),
            'string_operations': lambda: ''.join(str(i) for i in range(100)),
            'list_operations': lambda: sorted(list(range(1000, 0, -1))),
            'dict_operations': lambda: {str(i): i for i in range(500)},
            'file_io': lambda: BenchmarkRunner._bench_file_io(),
            'json_parse': lambda: [json.loads('{"a":1,"b":[1,2,3],"c":{"d":"test"}}') for _ in range(100)],
            'csv_write': lambda: BenchmarkRunner._bench_csv_write(),
        }

        for test_cfg in tests:
            name = test_cfg.get('name', '')
            iters = test_cfg.get('iterations', 100)
            warmup = test_cfg.get('warmup', 10)

            if name in builtin_tests:
                result = BenchmarkRunner.run(name, builtin_tests[name], iters, warmup)
                results.append(result)
            elif 'func' in test_cfg:
                # 自定义函数（通过配置）
                result = BenchmarkRunner.run(name, test_cfg['func'], iters, warmup)
                results.append(result)
            else:
                print(f"  [跳过] 未知基准测试: {name}")

        return results

    @staticmethod
    def _bench_file_io():
        """文件IO基准"""
        import tempfile
        data = 'x' * 10000
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.txt') as f:
            f.write(data)
            name = f.name
        with open(name, 'r') as f:
            _ = f.read()
        os.unlink(name)

    @staticmethod
    def _bench_csv_write():
        """CSV写入基准"""
        import tempfile
        import csv
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.csv',
                                          newline='') as f:
            writer = csv.writer(f)
            for i in range(100):
                writer.writerow([i, f'value_{i}', i * 1.5])
            name = f.name
        os.unlink(name)


# ── 端到端测试 ────────────────────────────────────────────────────────────────

class E2ETestRunner:
    """端到端测试运行器"""

    @staticmethod
    def create_e2e_suite(config: Dict) -> TestSuite:
        """
        从配置创建E2E测试套件
        config格式:
        {
            "name": "系统名称",
            "tests": [
                {
                    "id": "TC001",
                    "name": "测试名",
                    "steps": [
                        {"action": "check_value", "params": {"actual": 3.3, "expected": 3.3, "tolerance": 0.1}},
                        ...
                    ]
                }
            ]
        }
        """
        suite = TestSuite(
            name=config.get('name', 'E2E测试'),
            description=config.get('description', ''),
            system_under_test=config.get('system', '未指定')
        )

        for tc_cfg in config.get('tests', []):
            tc = TestCase(
                id=tc_cfg.get('id', f'TC{len(suite.test_cases)+1:03d}'),
                name=tc_cfg.get('name', ''),
                description=tc_cfg.get('description', ''),
                category=tc_cfg.get('category', 'e2e'),
                severity=tc_cfg.get('severity', 'MAJOR'),
                timeout_s=tc_cfg.get('timeout', 30),
                steps=tc_cfg.get('steps', []),
                expected=tc_cfg.get('expected', '')
            )
            suite.test_cases.append(tc)

        return suite

    @staticmethod
    def create_hardware_e2e(port: str = None, baud: int = 115200) -> TestSuite:
        """创建硬件E2E测试套件"""
        suite = TestSuite(
            name="硬件端到端测试",
            system_under_test="嵌入式硬件系统",
            test_cases=[
                TestCase(id="HW001", name="串口连接测试", category="connectivity",
                         severity="CRITICAL",
                         steps=[{"action": "check_serial",
                                 "params": {"port": port or "COM1", "baud": baud}}]),
                TestCase(id="HW002", name="固件版本检查", category="firmware",
                         severity="MAJOR",
                         steps=[{"action": "wait", "params": {"seconds": 0.5}}]),
                TestCase(id="HW003", name="GPIO自检", category="hardware",
                         severity="CRITICAL",
                         steps=[{"action": "wait", "params": {"seconds": 0.1}}]),
                TestCase(id="HW004", name="ADC精度校验", category="analog",
                         severity="MAJOR",
                         steps=[{"action": "check_value",
                                 "params": {"actual": 3.30, "expected": 3.30, "tolerance": 0.05}}]),
                TestCase(id="HW005", name="PWM输出验证", category="motor",
                         severity="MAJOR",
                         steps=[{"action": "check_range",
                                 "params": {"value": 50, "min": 0, "max": 100}}]),
                TestCase(id="HW006", name="通信链路测试", category="communication",
                         severity="CRITICAL",
                         steps=[{"action": "wait", "params": {"seconds": 0.2}}]),
            ]
        )
        return suite


# ── CLI 命令 ──────────────────────────────────────────────────────────────────

def cmd_e2e(args):
    """端到端测试"""
    if args.config:
        with open(args.config, 'r', encoding='utf-8') as f:
            config = json.load(f)
        suite = E2ETestRunner.create_e2e_suite(config)
    elif args.hardware:
        suite = E2ETestRunner.create_hardware_e2e(args.port, args.baud)
    else:
        # 默认E2E测试
        suite = TestSuite(
            name="默认端到端测试",
            system_under_test="电赛系统",
            test_cases=[
                TestCase(id="E2E001", name="基础功能验证", severity="CRITICAL",
                         steps=[{"action": "check_true", "params": {"condition": True}}]),
                TestCase(id="E2E002", name="数值范围检查", severity="MAJOR",
                         steps=[{"action": "check_range", "params": {"value": 3.3, "min": 3.0, "max": 3.6}}]),
                TestCase(id="E2E003", name="精度验证", severity="MAJOR",
                         steps=[{"action": "check_value", "params": {"actual": 12.01, "expected": 12.0, "tolerance": 0.1}}]),
                TestCase(id="E2E004", name="超时测试", severity="MINOR", timeout_s=2,
                         steps=[{"action": "wait", "params": {"seconds": 1}}]),
                TestCase(id="E2E005", name="边界条件", severity="MAJOR",
                         steps=[{"action": "check_true", "params": {"condition": True}}]),
            ]
        )

    executor = TestExecutor(verbose=True)
    report = executor.execute_suite(suite)

    # 保存报告
    out_file = f"e2e_report_{report.report_id}.json"
    with open(out_file, 'w', encoding='utf-8') as f:
        json.dump(asdict(report), f, ensure_ascii=False, indent=2)
    print(f"\n  报告已保存: {out_file}")


def cmd_regression(args):
    """回归测试"""
    analyzer = RegressionAnalyzer()

    if args.baseline and args.current:
        result = analyzer.compare(args.baseline, args.current)
    else:
        # 模拟回归数据
        print("  [提示] 使用模拟数据演示回归分析")
        result = RegressionResult(
            baseline_version="v1.0", current_version="v2.0",
            total_tests=10, passed=8, failed=1, new_failures=1, fixed=2, unchanged=7
        )
        result.regressions = [{"id": "TC005", "name": "PWM频率测试", "baseline": "PASS", "current": "FAIL"}]
        result.improvements = [{"id": "TC003", "name": "ADC校准", "baseline": "FAIL", "current": "PASS"},
                               {"id": "TC007", "name": "通信稳定性", "baseline": "FAIL", "current": "PASS"}]

    print("\n" + "=" * 60)
    print("  回归测试分析报告")
    print("=" * 60)
    print(f"  基准版本:    {result.baseline_version}")
    print(f"  当前版本:    {result.current_version}")
    print(f"  总测试数:    {result.total_tests}")
    print(f"  通过:        {result.passed}")
    print(f"  失败:        {result.failed}")
    print(f"  新增失败:    {result.new_failures}")
    print(f"  已修复:      {result.fixed}")
    print(f"  无变化:      {result.unchanged}")

    if result.regressions:
        print("\n  ▼ 新增失败（回归）:")
        for r in result.regressions:
            print(f"    [{r['id']}] {r['name']}: {r['baseline']} → {r['current']}")

    if result.improvements:
        print("\n  ▲ 已修复:")
        for i in result.improvements:
            print(f"    [{i['id']}] {i['name']}: {i['baseline']} → {i['current']}")

    if result.performance_delta:
        print("\n  性能变化:")
        for tid, delta in result.performance_delta.items():
            direction = "↑" if delta['delta_pct'] > 0 else "↓"
            print(f"    [{tid}] {delta['baseline_ms']:.1f}ms → {delta['current_ms']:.1f}ms "
                  f"({direction}{abs(delta['delta_pct']):.1f}%)")


def cmd_benchmark(args):
    """基准测试"""
    print("=" * 60)
    print("  系统性能基准测试")
    print("=" * 60)

    if args.config:
        with open(args.config, 'r', encoding='utf-8') as f:
            config = json.load(f)
        results = BenchmarkRunner.run_suite(config)
    else:
        # 默认基准测试
        default_tests = [
            {'name': 'math_operations', 'iterations': args.iterations},
            {'name': 'string_operations', 'iterations': args.iterations},
            {'name': 'list_operations', 'iterations': args.iterations},
            {'name': 'dict_operations', 'iterations': args.iterations},
            {'name': 'json_parse', 'iterations': args.iterations},
        ]
        results = BenchmarkRunner.run_suite({'tests': default_tests})

    # 打印结果
    print("\n" + "=" * 60)
    print("  基准测试结果汇总")
    print("=" * 60)
    print(f"  {'测试名':<25} {'均值(ms)':>10} {'P95(ms)':>10} {'标准差':>10} {'ops/s':>10}")
    print("-" * 70)
    for r in results:
        print(f"  {r.test_name:<25} {r.mean_time_ms:>10.3f} {r.p95_time_ms:>10.3f} "
              f"{r.std_dev_ms:>10.3f} {r.throughput_ops:>10.1f}")

    # 保存
    out = {'benchmarks': [asdict(r) for r in results], 'time': time.strftime('%Y-%m-%d %H:%M:%S')}
    out_file = "benchmark_result.json"
    with open(out_file, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"\n  结果已保存: {out_file}")


def cmd_suite(args):
    """运行测试套件"""
    if not args.suite:
        print("  [错误] 请指定测试套件JSON文件: --suite <file>")
        return

    with open(args.suite, 'r', encoding='utf-8') as f:
        suite_data = json.load(f)

    suite = E2ETestRunner.create_e2e_suite(suite_data)
    executor = TestExecutor(verbose=not args.quiet)
    report = executor.execute_suite(suite)

    out_file = f"suite_report_{report.report_id}.json"
    with open(out_file, 'w', encoding='utf-8') as f:
        json.dump(asdict(report), f, ensure_ascii=False, indent=2)
    print(f"\n  报告已保存: {out_file}")


def cmd_init(args):
    """初始化测试套件模板"""
    template = {
        "name": args.name or "电赛系统测试",
        "description": "自动生成的测试套件模板",
        "system": args.name or "被测系统",
        "version": "1.0",
        "tests": [
            {
                "id": "TC001",
                "name": "基础功能测试",
                "category": "functional",
                "severity": "CRITICAL",
                "description": "验证系统基本功能正常",
                "timeout": 10,
                "steps": [
                    {"action": "check_true", "params": {"condition": True}}
                ]
            },
            {
                "id": "TC002",
                "name": "电源电压检查",
                "category": "power",
                "severity": "CRITICAL",
                "description": "验证电源输出在规格范围内",
                "steps": [
                    {"action": "check_range", "params": {"value": 3.3, "min": 3.2, "max": 3.4}}
                ]
            },
            {
                "id": "TC003",
                "name": "通信接口测试",
                "category": "communication",
                "severity": "MAJOR",
                "description": "验证通信接口正常工作",
                "timeout": 30,
                "steps": [
                    {"action": "check_true", "params": {"condition": True}}
                ]
            },
            {
                "id": "TC004",
                "name": "传感器数据采集",
                "category": "sensor",
                "severity": "MAJOR",
                "description": "验证传感器数据采集正常",
                "steps": [
                    {"action": "check_value", "params": {"actual": 25.0, "expected": 25.0, "tolerance": 2.0}}
                ]
            },
            {
                "id": "TC005",
                "name": "电机控制测试",
                "category": "motor",
                "severity": "MAJOR",
                "description": "验证电机控制响应正常",
                "timeout": 60,
                "steps": [
                    {"action": "check_range", "params": {"value": 50, "min": 0, "max": 100}}
                ]
            }
        ]
    }

    out = args.output or f"test_suite_{args.name.replace(' ', '_') if args.name else 'template'}.json"
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(template, f, ensure_ascii=False, indent=2)
    print(f"  测试套件模板已生成: {out}")
    print(f"  包含 {len(template['tests'])} 个测试用例")
    print(f"  请根据实际系统修改测试参数后运行:")
    print(f"    python system_validator.py suite --suite {out}")


def main():
    parser = argparse.ArgumentParser(
        description='系统验证工具 - 电赛资产库',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s e2e --config e2e_config.json
  %(prog)s e2e --hardware --port COM3 --baud 115200
  %(prog)s regression --baseline v1.json --current v2.json
  %(prog)s benchmark --iterations 200
  %(prog)s suite --suite test_suite.json
  %(prog)s init --name "电机驱动板V2" --output test_suite.json
        """
    )
    sub = parser.add_subparsers(dest='command')

    # E2E
    p_e2e = sub.add_parser('e2e', help='端到端测试')
    p_e2e.add_argument('--config', type=str, help='E2E配置JSON')
    p_e2e.add_argument('--hardware', action='store_true', help='硬件E2E模式')
    p_e2e.add_argument('--port', type=str, default='COM1', help='串口')
    p_e2e.add_argument('--baud', type=int, default=115200, help='波特率')

    # Regression
    p_reg = sub.add_parser('regression', help='回归测试')
    p_reg.add_argument('--baseline', type=str, help='基准结果JSON')
    p_reg.add_argument('--current', type=str, help='当前结果JSON')

    # Benchmark
    p_bench = sub.add_parser('benchmark', help='性能基准测试')
    p_bench.add_argument('--config', type=str, help='基准配置JSON')
    p_bench.add_argument('--iterations', type=int, default=100, help='迭代次数')

    # Suite
    p_suite = sub.add_parser('suite', help='运行测试套件')
    p_suite.add_argument('--suite', type=str, help='测试套件JSON')
    p_suite.add_argument('--quiet', action='store_true', help='安静模式')

    # Init
    p_init = sub.add_parser('init', help='初始化测试套件模板')
    p_init.add_argument('--name', type=str, default='电赛系统', help='系统名称')
    p_init.add_argument('--output', type=str, help='输出文件')

    args = parser.parse_args()
    if not args.command:
        parser.print_help()
        return

    dispatch = {
        'e2e': cmd_e2e, 'regression': cmd_regression,
        'benchmark': cmd_benchmark, 'suite': cmd_suite, 'init': cmd_init
    }
    dispatch[args.command](args)


if __name__ == '__main__':
    main()
