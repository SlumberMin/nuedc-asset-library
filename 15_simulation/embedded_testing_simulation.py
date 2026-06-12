#!/usr/bin/env python3
"""
嵌入式测试仿真
- 单元测试框架
- 集成测试
- HIL (硬件在环) 测试框架
- 测试覆盖率分析
- 故障注入测试
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Callable, Any, Optional, Dict
from enum import Enum, auto
import time
import traceback

# ============================================================
# 1. 测试框架核心
# ============================================================

class TestStatus(Enum):
    PASS = auto()
    FAIL = auto()
    SKIP = auto()
    ERROR = auto()

@dataclass
class TestResult:
    name: str
    status: TestStatus
    duration_ms: float = 0.0
    message: str = ""
    assertions: int = 0

@dataclass
class TestSuite:
    name: str
    results: List[TestResult] = field(default_factory=list)

    def pass_count(self) -> int:
        return sum(1 for r in self.results if r.status == TestStatus.PASS)

    def fail_count(self) -> int:
        return sum(1 for r in self.results if r.status == TestStatus.FAIL)

    def total(self) -> int:
        return len(self.results)

    def pass_rate(self) -> float:
        return self.pass_count() / self.total() if self.total() > 0 else 0.0


class TestRunner:
    """测试运行器"""

    def __init__(self):
        self.suites: List[TestSuite] = []
        self.current_suite: Optional[TestSuite] = None

    def suite(self, name: str):
        self.current_suite = TestSuite(name)
        self.suites.append(self.current_suite)
        return self

    def run_test(self, name: str, test_func: Callable) -> TestResult:
        """运行单个测试"""
        start = time.time()
        try:
            test_func()
            result = TestResult(name, TestStatus.PASS,
                              duration_ms=(time.time()-start)*1000)
        except AssertionError as e:
            result = TestResult(name, TestStatus.FAIL,
                              duration_ms=(time.time()-start)*1000,
                              message=str(e))
        except Exception as e:
            result = TestResult(name, TestStatus.ERROR,
                              duration_ms=(time.time()-start)*1000,
                              message=traceback.format_exc())

        if self.current_suite:
            self.current_suite.results.append(result)
        return result

    def report(self) -> str:
        """生成测试报告"""
        lines = ["=" * 60, "  测试报告", "=" * 60]
        total_pass, total_fail, total_error = 0, 0, 0

        for suite in self.suites:
            lines.append(f"\n📦 {suite.name}")
            for r in suite.results:
                icon = {"PASS": "✅", "FAIL": "❌", "SKIP": "⏭️", "ERROR": "💥"}
                status = icon.get(r.status.name, "?")
                lines.append(f"  {status} {r.name} ({r.duration_ms:.2f}ms)")
                if r.message:
                    lines.append(f"     → {r.message}")

            total_pass += suite.pass_count()
            total_fail += suite.fail_count()
            total_error += sum(1 for r in suite.results if r.status == TestStatus.ERROR)

        lines.append("\n" + "=" * 60)
        total = total_pass + total_fail + total_error
        lines.append(f"总计: {total} | ✅ {total_pass} | ❌ {total_fail} | 💥 {total_error}")
        lines.append(f"通过率: {total_pass/total*100:.1f}%" if total > 0 else "无测试")
        return "\n".join(lines)


# ============================================================
# 2. 断言工具
# ============================================================

def assert_equal(actual, expected, msg=""):
    if actual != expected:
        raise AssertionError(f"{msg}: 期望 {expected}, 实际 {actual}")

def assert_close(actual, expected, tol=1e-6, msg=""):
    if abs(actual - expected) > tol:
        raise AssertionError(f"{msg}: 期望 {expected}±{tol}, 实际 {actual}")

def assert_true(condition, msg=""):
    if not condition:
        raise AssertionError(f"{msg}: 条件不满足")

def assert_in_range(value, low, high, msg=""):
    if not (low <= value <= high):
        raise AssertionError(f"{msg}: {value} 不在 [{low}, {high}] 范围内")


# ============================================================
# 3. 模拟嵌入式模块
# ============================================================

class PIDController:
    """模拟 PID 控制器"""
    def __init__(self, kp=1.0, ki=0.1, kd=0.01, out_min=-100, out_max=100):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        return np.clip(output, self.out_min, self.out_max)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class FIRFilter:
    """FIR 滤波器"""
    def __init__(self, coeffs: List[float]):
        self.coeffs = np.array(coeffs)
        self.buffer = np.zeros(len(coeffs))

    def process(self, sample: float) -> float:
        self.buffer = np.roll(self.buffer, 1)
        self.buffer[0] = sample
        return np.dot(self.coeffs, self.buffer)

    def reset(self):
        self.buffer[:] = 0


class RingBuffer:
    """环形缓冲区"""
    def __init__(self, size: int):
        self.size = size
        self.buffer = [None] * size
        self.head = 0
        self.count = 0

    def push(self, item) -> bool:
        if self.is_full():
            return False
        self.buffer[self.head] = item
        self.head = (self.head + 1) % self.size
        self.count += 1
        return True

    def pop(self):
        if self.is_empty():
            return None
        tail = (self.head - self.count) % self.size
        item = self.buffer[tail]
        self.count -= 1
        return item

    def is_full(self) -> bool:
        return self.count >= self.size

    def is_empty(self) -> bool:
        return self.count == 0

    def available(self) -> int:
        return self.count


# ============================================================
# 4. 单元测试
# ============================================================

def run_unit_tests(runner: TestRunner):
    """运行单元测试"""
    runner.suite("PID 控制器单元测试")

    def test_pid_output_range():
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, out_min=-50, out_max=50)
        out = pid.update(100, 0, 0.01)
        assert_in_range(out, -50, 50, "PID输出范围")

    def test_pid_convergence():
        pid = PIDController(kp=2.0, ki=0.5, kd=0.1)
        value = 0.0
        for _ in range(1000):
            value += pid.update(10.0, value, 0.001) * 0.001
        assert_close(value, 10.0, tol=0.5, msg="PID收敛")

    def test_pid_reset():
        pid = PIDController()
        pid.update(10, 0, 0.01)
        pid.reset()
        assert_close(pid.integral, 0.0, msg="PID重置")
        assert_close(pid.prev_error, 0.0, msg="PID重置误差")

    runner.run_test("输出范围限制", test_pid_output_range)
    runner.run_test("收敛性", test_pid_convergence)
    runner.run_test("重置功能", test_pid_reset)

    # FIR 滤波器测试
    runner.suite("FIR 滤波器单元测试")

    def test_fir_passthrough():
        """全1系数应通过信号"""
        fir = FIRFilter([1.0, 0.0, 0.0])
        out = fir.process(5.0)
        assert_close(out, 5.0, msg="直通测试")

    def test_fir_averaging():
        """平均滤波"""
        fir = FIRFilter([0.25, 0.25, 0.25, 0.25])
        for v in [1.0, 2.0, 3.0, 4.0]:
            out = fir.process(v)
        assert_close(out, 2.5, tol=0.01, msg="平均滤波")

    runner.run_test("直通特性", test_fir_passthrough)
    runner.run_test("平均滤波", test_fir_averaging)

    # 环形缓冲区测试
    runner.suite("环形缓冲区单元测试")

    def test_ring_buffer_basic():
        rb = RingBuffer(4)
        assert_true(rb.is_empty(), "初始为空")
        rb.push(1); rb.push(2); rb.push(3)
        assert_equal(rb.available(), 3, "容量检查")
        assert_equal(rb.pop(), 1, "FIFO顺序")
        assert_equal(rb.pop(), 2, "FIFO顺序")

    def test_ring_buffer_overflow():
        rb = RingBuffer(3)
        rb.push(1); rb.push(2); rb.push(3)
        assert_true(rb.is_full(), "满缓冲区")
        assert_true(not rb.push(4), "溢出拒绝")

    def test_ring_buffer_wrap():
        rb = RingBuffer(3)
        rb.push(1); rb.push(2); rb.push(3)
        rb.pop(); rb.pop()
        rb.push(4); rb.push(5)
        assert_equal(rb.pop(), 3, "环形回绕")

    runner.run_test("基本操作", test_ring_buffer_basic)
    runner.run_test("溢出保护", test_ring_buffer_overflow)
    runner.run_test("环形回绕", test_ring_buffer_wrap)


# ============================================================
# 5. 集成测试
# ============================================================

def run_integration_tests(runner: TestRunner):
    """集成测试: 多模块协同"""
    runner.suite("集成测试")

    def test_pid_with_filter():
        """PID + 滤波器集成"""
        pid = PIDController(kp=1.0, ki=0.5, kd=0.0)
        fir = FIRFilter([0.1, 0.2, 0.4, 0.2, 0.1])

        value = 0.0
        for _ in range(2000):
            filtered = fir.process(value)
            control = pid.update(10.0, filtered, 0.001)
            value += control * 0.001

        assert_close(value, 10.0, tol=1.0, msg="PID+滤波器集成")

    def test_buffer_pipeline():
        """缓冲区流水线"""
        buf1 = RingBuffer(10)
        buf2 = RingBuffer(10)

        # 生产者 → buf1 → 处理 → buf2 → 消费者
        for i in range(5):
            buf1.push(i * 2)

        while not buf1.is_empty():
            v = buf1.pop()
            buf2.push(v + 1)

        results = []
        while not buf2.is_empty():
            results.append(buf2.pop())

        assert_equal(results, [1, 3, 5, 7, 9], "流水线数据正确")

    runner.run_test("PID+滤波器协同", test_pid_with_filter)
    runner.run_test("缓冲区流水线", test_buffer_pipeline)


# ============================================================
# 6. HIL (硬件在环) 测试框架
# ============================================================

@dataclass
class HILOPlant:
    """HIL 被控对象模型"""
    position: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0
    mass: float = 1.0
    damping: float = 0.1

    def step(self, force: float, dt: float) -> float:
        self.acceleration = (force - self.damping * self.velocity) / self.mass
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt
        return self.position

    def get_sensor_reading(self, noise_std: float = 0.01) -> float:
        """模拟传感器读数 (含噪声)"""
        return self.position + np.random.randn() * noise_std


class HILTestBench:
    """HIL 测试台"""

    def __init__(self):
        self.plant = HILOPlant()
        self.controller = PIDController(kp=10.0, ki=1.0, kd=0.5)
        self.log_time = []
        self.log_position = []
        self.log_setpoint = []
        self.log_control = []

    def run(self, setpoint: float, duration: float = 5.0, dt: float = 0.001):
        """运行 HIL 仿真"""
        steps = int(duration / dt)
        self.controller.reset()

        for i in range(steps):
            t = i * dt
            sensor = self.plant.get_sensor_reading()
            control = self.controller.update(setpoint, sensor, dt)
            self.plant.step(control, dt)

            self.log_time.append(t)
            self.log_position.append(self.plant.position)
            self.log_setpoint.append(setpoint)
            self.log_control.append(control)

    def analyze(self) -> Dict:
        """分析 HIL 结果"""
        pos = np.array(self.log_position)
        sp = np.array(self.log_setpoint)
        t = np.array(self.log_time)

        # 稳态误差
        steady_idx = int(len(pos) * 0.8)
        steady_error = np.mean(np.abs(pos[steady_idx:] - sp[steady_idx:]))

        # 超调量
        overshoot = (np.max(pos) - sp[-1]) / sp[-1] * 100 if sp[-1] != 0 else 0

        # 调节时间 (2% 准则)
        settling_time = t[-1]
        for i in range(len(pos)-1, -1, -1):
            if abs(pos[i] - sp[i]) > 0.02 * abs(sp[i]):
                settling_time = t[min(i+1, len(t)-1)]
                break

        return {
            'steady_error': steady_error,
            'overshoot': overshoot,
            'settling_time': settling_time,
            'rise_time': t[np.where(pos >= 0.9*sp[-1])[0][0]] if sp[-1] > 0 else 0
        }

    def plot(self, title: str = "HIL 测试结果"):
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        ax1.plot(self.log_time, self.log_position, 'b-', label='位置', linewidth=1)
        ax1.plot(self.log_time, self.log_setpoint, 'r--', label='设定值', linewidth=2)
        ax1.set_ylabel('位置')
        ax1.set_title(title)
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        ax2.plot(self.log_time, self.log_control, 'g-', label='控制量', linewidth=1)
        ax2.set_ylabel('控制量')
        ax2.set_xlabel('时间 (s)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('./nuedc-asset-library/15_simulation/hil_test.png', dpi=150)
        plt.show()


def run_hil_tests(runner: TestRunner):
    """HIL 测试"""
    runner.suite("HIL 硬件在环测试")

    def test_hil_step_response():
        bench = HILTestBench()
        bench.run(setpoint=10.0, duration=3.0)
        result = bench.analyze()
        assert_true(result['steady_error'] < 0.1, f"稳态误差: {result['steady_error']:.4f}")
        assert_true(result['overshoot'] < 30, f"超调量: {result['overshoot']:.1f}%")
        bench.plot("HIL 阶跃响应测试")

    def test_hil_disturbance():
        bench = HILTestBench()
        bench.run(setpoint=5.0, duration=5.0)
        result = bench.analyze()
        assert_true(result['settling_time'] < 3.0, f"调节时间: {result['settling_time']:.2f}s")

    runner.run_test("阶跃响应", test_hil_step_response)
    runner.run_test("抗扰性", test_hil_disturbance)


# ============================================================
# 7. 故障注入测试
# ============================================================

def run_fault_injection_tests(runner: TestRunner):
    """故障注入测试"""
    runner.suite("故障注入测试")

    def test_sensor_saturation():
        """传感器饱和故障"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, out_min=-100, out_max=100)
        # 模拟传感器饱和在 5.0
        value = 0.0
        for _ in range(500):
            sensor = min(value, 5.0)  # 饱和
            control = pid.update(10.0, sensor, 0.001)
            value += control * 0.001
        # 不应发散
        assert_true(abs(value) < 1000, f"传感器饱和下系统稳定: {value:.2f}")

    def test_output_clipping():
        """输出限幅保护"""
        pid = PIDController(kp=100.0, ki=0.0, kd=0.0, out_min=-10, out_max=10)
        out = pid.update(1000, 0, 0.01)
        assert_in_range(out, -10, 10, "输出限幅")

    def test_zero_division_protection():
        """除零保护"""
        pid = PIDController()
        out = pid.update(10, 10, 0.0)  # dt=0
        assert_true(np.isfinite(out), "dt=0 不应产生NaN")

    runner.run_test("传感器饱和", test_sensor_saturation)
    runner.run_test("输出限幅", test_output_clipping)
    runner.run_test("除零保护", test_zero_division_protection)


# ============================================================
# 8. 覆盖率分析 (模拟)
# ============================================================

def analyze_coverage() -> Dict:
    """模拟代码覆盖率分析"""
    # 模拟各模块的覆盖率数据
    modules = {
        'PID控制器': {'lines': 50, 'covered': 45, 'branches': 12, 'branch_covered': 10},
        'FIR滤波器': {'lines': 20, 'covered': 20, 'branches': 4, 'branch_covered': 4},
        '环形缓冲区': {'lines': 35, 'covered': 32, 'branches': 8, 'branch_covered': 7},
        'HIL模型':   {'lines': 40, 'covered': 35, 'branches': 10, 'branch_covered': 8},
    }

    total_lines = sum(m['lines'] for m in modules.values())
    total_covered = sum(m['covered'] for m in modules.values())
    total_branches = sum(m['branches'] for m in modules.values())
    total_branch_covered = sum(m['branch_covered'] for m in modules.values())

    return {
        'modules': modules,
        'line_coverage': total_covered / total_lines * 100,
        'branch_coverage': total_branch_covered / total_branches * 100
    }


def plot_coverage(coverage: Dict):
    """绘制覆盖率图"""
    modules = coverage['modules']
    names = list(modules.keys())
    line_cov = [m['covered']/m['lines']*100 for m in modules.values()]
    branch_cov = [m['branch_covered']/m['branches']*100 for m in modules.values()]

    x = np.arange(len(names))
    width = 0.35

    fig, ax = plt.subplots(figsize=(10, 6))
    bars1 = ax.bar(x - width/2, line_cov, width, label='行覆盖率', color='steelblue')
    bars2 = ax.bar(x + width/2, branch_cov, width, label='分支覆盖率', color='coral')

    ax.set_ylabel('覆盖率 (%)')
    ax.set_title(f'代码覆盖率分析 (总体: 行{coverage["line_coverage"]:.1f}% / 分支{coverage["branch_coverage"]:.1f}%)')
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.legend()
    ax.set_ylim(0, 110)
    ax.grid(True, alpha=0.3, axis='y')

    # 添加数值标签
    for bar in bars1:
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height(),
                f'{bar.get_height():.0f}%', ha='center', va='bottom', fontsize=8)
    for bar in bars2:
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height(),
                f'{bar.get_height():.0f}%', ha='center', va='bottom', fontsize=8)

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/test_coverage.png', dpi=150)
    plt.show()


# ============================================================
# 9. 主程序
# ============================================================

if __name__ == '__main__':
    print("=" * 60)
    print("  嵌入式测试仿真")
    print("=" * 60)

    runner = TestRunner()

    # 运行所有测试
    run_unit_tests(runner)
    run_integration_tests(runner)
    run_hil_tests(runner)
    run_fault_injection_tests(runner)

    # 打印报告
    print(runner.report())

    # 覆盖率分析
    print("\n--- 代码覆盖率分析 ---")
    coverage = analyze_coverage()
    for name, data in coverage['modules'].items():
        print(f"  {name}: 行覆盖率 {data['covered']}/{data['lines']} = {data['covered']/data['lines']*100:.1f}%")
    print(f"  总体: 行{coverage['line_coverage']:.1f}% / 分支{coverage['branch_coverage']:.1f}%")

    plot_coverage(coverage)

    print("\n✅ 所有测试仿真完成")
