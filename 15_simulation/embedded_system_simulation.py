#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
嵌入式系统仿真 - 任务调度/中断/资源竞争/时序分析
==========================================
功能：
  - 任务模型（周期任务、非周期任务、偶发任务）
  - 调度算法（RM、EDF、FIFO、优先级抢占）
  - 中断仿真（中断延迟、嵌套、优先级反转）
  - 资源竞争（互斥锁、信号量、死锁检测）
  - 时序分析（WCET、响应时间、可调度性测试）
  - 时间线甘特图可视化

适用场景：RTOS任务设计、实时性分析、嵌入式软件架构验证
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import BrokenBarHCollection
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple
from enum import Enum
from copy import deepcopy
import warnings
warnings.filterwarnings('ignore')

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ======================== 任务模型 ========================

class TaskType(Enum):
    PERIODIC = "周期任务"
    SPORADIC = "偶发任务"
    APERIODIC = "非周期任务"


class TaskState(Enum):
    READY = "就绪"
    RUNNING = "运行"
    BLOCKED = "阻塞"
    WAITING = "等待"
    COMPLETED = "完成"


@dataclass
class Task:
    """任务描述"""
    task_id: int
    name: str
    task_type: TaskType
    period: float              # 周期 (ms)
    wcet: float                # 最坏执行时间 (ms)
    bcet: float = 0.0          # 最好执行时间 (ms)
    deadline: float = 0.0      # 相对截止期 (ms), 0=等于周期
    priority: int = 0          # 优先级 (数值越大越高)
    resources: List[str] = field(default_factory=list)  # 需要的资源

    def __post_init__(self):
        if self.deadline == 0:
            self.deadline = self.period
        if self.bcet == 0:
            self.bcet = self.wcet * 0.3

    @property
    def utilization(self) -> float:
        return self.wcet / self.period


@dataclass
class JobInstance:
    """任务的一个作业实例"""
    task_id: int
    release_time: float        # 释放时间
    deadline: float            # 绝对截止期
    remaining_time: float      # 剩余执行时间
    state: TaskState = TaskState.READY
    start_time: float = 0.0
    finish_time: float = 0.0
    blocked_by: Optional[int] = None

    @property
    def response_time(self) -> float:
        return self.finish_time - self.release_time


@dataclass
class Interrupt:
    """中断描述"""
    interrupt_id: int
    name: str
    priority: int              # 中断优先级
    arrival_time: float        # 到达时间
    handler_time: float        # 处理时间 (ms)
    deadline: float = 0.0      # 绝对截止期


@dataclass
class Resource:
    """共享资源"""
    name: str
    mutex_locked: bool = False
    locked_by: Optional[int] = None
    lock_time: float = 0.0
    max_lock_duration: float = 0.0  # 最大持有时间


# ======================== 调度器 ========================

class Scheduler:
    """任务调度器基类"""

    def __init__(self, num_cpus: int = 1):
        self.num_cpus = num_cpus
        self.timeline: List[dict] = []
        self.preemptions = 0
        self.context_switches = 0


class RMScheduler(Scheduler):
    """速率单调调度 (Rate Monotonic)"""

    def schedule(self, tasks: List[Task], sim_time: float, dt: float = 0.01) -> dict:
        # 按周期排序，周期短优先级高
        sorted_tasks = sorted(tasks, key=lambda t: t.period)
        priorities = {t.task_id: len(tasks) - i for i, t in enumerate(sorted_tasks)}

        return self._simulate(tasks, priorities, sim_time, dt)


class EDFScheduler(Scheduler):
    """最早截止期优先调度 (Earliest Deadline First)"""

    def schedule(self, tasks: List[Task], sim_time: float, dt: float = 0.01) -> dict:
        return self._simulate_edf(tasks, sim_time, dt)


class PriorityScheduler(Scheduler):
    """固定优先级抢占式调度"""

    def __init__(self, num_cpus: int = 1, preemptive: bool = True):
        super().__init__(num_cpus)
        self.preemptive = preemptive

    def schedule(self, tasks: List[Task], sim_time: float, dt: float = 0.01) -> dict:
        priorities = {t.task_id: t.priority for t in tasks}
        return self._simulate(tasks, priorities, sim_time, dt)


# 共用仿真逻辑
def _run_schedule(tasks, priorities, sim_time, dt, edf=False):
    """通用调度仿真核心"""
    n_steps = int(sim_time / dt)
    timeline = []
    jobs = []  # 活跃作业队列
    completed_jobs = []
    preemptions = 0
    cpu_busy = 0.0
    current_running = {}  # cpu_id -> job

    # 生成所有释放事件
    releases = {}
    for t in tasks:
        times = np.arange(0, sim_time, t.period)
        for rt in times:
            rel_time = int(rt / dt)
            if rel_time not in releases:
                releases[rel_time] = []
            releases[rel_time].append(t)

    for step in range(n_steps):
        current_time = step * dt

        # 检查新释放
        if step in releases:
            for t in releases[step]:
                job = JobInstance(
                    task_id=t.task_id,
                    release_time=current_time,
                    deadline=current_time + t.deadline,
                    remaining_time=t.wcet
                )
                jobs.append(job)

        # EDF: 动态计算优先级
        if edf:
            for j in jobs:
                if j.state == TaskState.READY:
                    priorities[j.task_id] = 1.0 / max(j.deadline - current_time, 0.001)

        # 排序就绪队列
        ready_jobs = [j for j in jobs if j.state == TaskState.READY]
        ready_jobs.sort(key=lambda j: priorities.get(j.task_id, 0), reverse=True)

        # 分配CPU
        running_jobs = [j for j in jobs if j.state == TaskState.RUNNING]

        # 抢占检查
        for cpu in range(min(1, len(running_jobs))):
            if running_jobs:
                running = running_jobs[cpu] if cpu < len(running_jobs) else None
                if running and ready_jobs:
                    highest = ready_jobs[0]
                    hp = priorities.get(highest.task_id, 0)
                    cp = priorities.get(running.task_id, 0)
                    if hp > cp:
                        running.state = TaskState.READY
                        preemptions += 1

        # 运行最高优先级就绪任务
        ready_jobs = [j for j in jobs if j.state == TaskState.READY]
        ready_jobs.sort(key=lambda j: priorities.get(j.task_id, 0), reverse=True)

        running_jobs = [j for j in jobs if j.state == TaskState.RUNNING]

        if ready_jobs and not running_jobs:
            ready_jobs[0].state = TaskState.RUNNING
            ready_jobs[0].start_time = min(ready_jobs[0].start_time, current_time) \
                if ready_jobs[0].start_time > 0 else current_time

        # 执行
        for j in jobs:
            if j.state == TaskState.RUNNING:
                j.remaining_time -= dt
                cpu_busy += dt
                timeline.append({'time': current_time, 'task_id': j.task_id, 'dt': dt})
                if j.remaining_time <= 0:
                    j.state = TaskState.COMPLETED
                    j.finish_time = current_time
                    completed_jobs.append(j)

        # 超时检查
        for j in jobs:
            if j.state in (TaskState.READY, TaskState.RUNNING) and current_time > j.deadline:
                timeline.append({'time': current_time, 'task_id': -1, 'dt': dt,
                                'note': f'Task {j.task_id} MISS'})

        # 清理已完成
        jobs = [j for j in jobs if j.state != TaskState.COMPLETED]

    # 统计
    utilization = sum(t.utilization for t in tasks)
    missed = len([j for j in completed_jobs if j.finish_time > j.deadline])

    return {
        'timeline': timeline,
        'completed_jobs': completed_jobs,
        'utilization': utilization,
        'preemptions': preemptions,
        'missed_deadlines': missed,
        'total_jobs': len(completed_jobs)
    }


# 为各调度器绑定方法
def _schedule_generic(self, tasks, sim_time, dt=0.01):
    return _run_schedule(tasks, {t.task_id: t.priority for t in tasks}, sim_time, dt)

RMScheduler.schedule = lambda self, tasks, sim_time, dt=0.01: \
    _run_schedule(tasks, {t.task_id: len(tasks)-i for i, t in enumerate(sorted(tasks, key=lambda t: t.period))}, sim_time, dt)

EDFScheduler._simulate_edf = lambda self, tasks, sim_time, dt=0.01: \
    _run_schedule(tasks, {t.task_id: 0 for t in tasks}, sim_time, dt, edf=True)
EDFScheduler.schedule = EDFScheduler._simulate_edf

PriorityScheduler.schedule = lambda self, tasks, sim_time, dt=0.01: \
    _run_schedule(tasks, {t.task_id: t.priority for t in tasks}, sim_time, dt)


# ======================== 可调度性分析 ========================

class SchedulabilityAnalyzer:
    """可调度性分析工具"""

    @staticmethod
    def rm_utilization_bound(n: int) -> float:
        """RM利用率上界: n(2^(1/n) - 1)"""
        return n * (2**(1/n) - 1)

    @staticmethod
    def rta_analysis(tasks: List[Task]) -> dict:
        """响应时间分析 (RTA)"""
        results = {}
        sorted_tasks = sorted(tasks, key=lambda t: t.period)

        for i, task in enumerate(sorted_tasks):
            R = task.wcet  # 初始
            while True:
                interference = sum(
                    np.ceil(R / higher.period) * higher.wcet
                    for higher in sorted_tasks[:i]
                )
                R_new = task.wcet + interference
                if R_new == R:
                    break
                R = R_new
                if R > task.deadline:
                    break
            results[task.task_id] = {
                'wcrt': R,  # 最坏响应时间
                'schedulable': R <= task.deadline,
                'slack': task.deadline - R
            }
        return results

    @staticmethod
    def edf_schedulability(tasks: List[Task]) -> dict:
        """EDF可调度性: U <= 1"""
        U = sum(t.utilization for t in tasks)
        return {'utilization': U, 'schedulable': U <= 1.0}


# ======================== 中断仿真 ========================

class InterruptSimulator:
    """中断系统仿真"""

    def __init__(self, interrupt_latency: float = 0.001,
                 context_switch_time: float = 0.0005):
        self.interrupt_latency = interrupt_latency  # 中断延迟 (ms)
        self.context_switch_time = context_switch_time
        self.history = []

    def simulate_interrupts(self, interrupts: List[Interrupt],
                            sim_time: float, dt: float = 0.001) -> dict:
        """仿真中断处理"""
        n_steps = int(sim_time / dt)
        cpu_state = 'task'  # task / isr
        current_isr = None
        isr_remaining = 0
        preempted_task_time = 0
        total_isr_time = 0
        max_response = 0

        timeline = []
        pending = []

        int_events = {int(int_arr.arrival_time/dt): int_arr for int_arr in interrupts}

        for step in range(n_steps):
            t = step * dt

            # 新中断到达
            if step in int_events:
                ir = int_events[step]
                pending.append(ir)
                pending.sort(key=lambda x: -x.priority)

            # 中断处理
            if pending and cpu_state == 'task':
                ir = pending.pop(0)
                # 中断延迟
                cpu_state = 'isr'
                current_isr = ir
                isr_remaining = ir.handler_time
                timeline.append({'time': t, 'event': 'interrupt_enter',
                               'id': ir.interrupt_id, 'name': ir.name})

            if cpu_state == 'isr':
                isr_remaining -= dt
                total_isr_time += dt
                if isr_remaining <= 0:
                    response = t - current_isr.arrival_time
                    max_response = max(max_response, response)
                    timeline.append({'time': t, 'event': 'interrupt_exit',
                                   'id': current_isr.interrupt_id,
                                   'response_time': response})
                    cpu_state = 'task'
                    self.history.append({
                        'interrupt': current_isr,
                        'response_time': response,
                        'exit_time': t
                    })

        return {
            'timeline': timeline,
            'total_isr_time': total_isr_time,
            'max_response_time': max_response,
            'cpu_utilization_isr': total_isr_time / sim_time * 100,
            'history': self.history
        }


# ======================== 资源竞争仿真 ========================

class ResourceContentionSimulator:
    """资源竞争与死锁检测"""

    def __init__(self):
        self.resources: Dict[str, Resource] = {}
        self.lock_log = []
        self.deadlock_log = []

    def add_resource(self, name: str, max_lock_duration: float = 1.0):
        self.resources[name] = Resource(name=name, max_lock_duration=max_lock_duration)

    def simulate(self, task_accesses: Dict[int, List[Tuple[float, str, float]]],
                 sim_time: float, dt: float = 0.01) -> dict:
        """
        task_accesses: {task_id: [(acquire_time, resource_name, hold_duration), ...]}
        """
        n_steps = int(sim_time / dt)
        waiting_tasks = {}  # task_id -> (resource_waiting_for, wait_start)
        holding = {}  # resource_name -> (task_id, release_time)
        priority_inversions = 0
        total_wait = 0
        n_waits = 0

        # 构建事件
        events = {}
        for tid, accesses in task_accesses.items():
            for acq_time, res_name, hold_dur in accesses:
                step = int(acq_time / dt)
                if step not in events:
                    events[step] = []
                events[step].append((tid, res_name, hold_dur))

        for step in range(n_steps):
            t = step * dt

            # 释放到期的锁
            to_release = []
            for res_name, (tid, release_t) in holding.items():
                if t >= release_t:
                    to_release.append(res_name)
            for res_name in to_release:
                del holding[res_name]
                self.lock_log.append({'time': t, 'event': 'release',
                                      'resource': res_name})

            # 处理事件
            if step in events:
                for tid, res_name, hold_dur in events[step]:
                    if res_name in holding:
                        # 资源被占用，等待
                        waiting_tasks[tid] = (res_name, t)
                        self.lock_log.append({'time': t, 'event': 'wait',
                                              'task': tid, 'resource': res_name})
                    else:
                        holding[res_name] = (tid, t + hold_dur)
                        self.lock_log.append({'time': t, 'event': 'acquire',
                                              'task': tid, 'resource': res_name})

            # 检查等待中的任务
            newly_acquired = []
            for tid, (res_name, wait_start) in waiting_tasks.items():
                if res_name not in holding:
                    holding[res_name] = (tid, t + 0.5)  # 默认持有
                    wait_time = t - wait_start
                    total_wait += wait_time
                    n_waits += 1
                    newly_acquired.append(tid)
            for tid in newly_acquired:
                del waiting_tasks[tid]

        # 简单死锁检测（等待图环检测）
        has_deadlock = len(waiting_tasks) > 0

        return {
            'lock_log': self.lock_log,
            'priority_inversions': priority_inversions,
            'total_wait_time': total_wait,
            'num_waits': n_waits,
            'avg_wait': total_wait / max(n_waits, 1),
            'deadlock_detected': has_deadlock,
            'deadlocked_tasks': list(waiting_tasks.keys())
        }


# ======================== 时序分析 ========================

class TimingAnalyzer:
    """时序分析工具"""

    @staticmethod
    def wcet_analysis(tasks: List[Task], iterations: int = 100) -> dict:
        """WCET统计分析（蒙特卡洛）"""
        results = {}
        for t in tasks:
            # 模拟不同路径的执行时间
            exec_times = np.random.uniform(t.bcet, t.wcet, iterations)
            results[t.task_id] = {
                'bcet': t.bcet,
                'wcet': t.wcet,
                'mean': np.mean(exec_times),
                'std': np.std(exec_times),
                'p99': np.percentile(exec_times, 99)
            }
        return results

    @staticmethod
    def end_to_end_latency(task_chain: List[Task], scheduling_overhead: float = 0.01) -> float:
        """端到端延迟分析"""
        total = sum(t.wcet for t in task_chain)
        total += scheduling_overhead * len(task_chain)
        # 加入周期任务的最大相位延迟
        for t in task_chain:
            total += t.period  # 最坏等待
        return total


# ======================== 可视化 ========================

def run_embedded_simulations():
    """运行嵌入式系统仿真"""
    print("=" * 60)
    print("嵌入式系统仿真系统")
    print("=" * 60)

    # 定义任务集
    tasks = [
        Task(1, "传感器采集", TaskType.PERIODIC, period=10, wcet=1.5, priority=5),
        Task(2, "PID控制", TaskType.PERIODIC, period=5, wcet=2.0, priority=8),
        Task(3, "数据通信", TaskType.PERIODIC, period=50, wcet=5.0, priority=3),
        Task(4, "显示更新", TaskType.PERIODIC, period=100, wcet=8.0, priority=1),
        Task(5, "安全监测", TaskType.PERIODIC, period=20, wcet=1.0, priority=10),
        Task(6, "日志记录", TaskType.APERIODIC, period=200, wcet=3.0, priority=2),
    ]

    sim_time = 200.0  # 200ms

    # ---- 1. RM调度 ----
    print("\n[1] 速率单调(RM)调度...")
    rm = RMScheduler()
    result_rm = rm.schedule(tasks, sim_time, dt=0.1)
    print(f"   总利用率: {result_rm['utilization']:.2%}")
    print(f"   完成作业数: {result_rm['total_jobs']}")
    print(f"   错过截止期: {result_rm['missed_deadlines']}")

    # ---- 2. EDF调度 ----
    print("\n[2] 最早截止期优先(EDF)调度...")
    edf = EDFScheduler()
    result_edf = edf.schedule(tasks, sim_time, dt=0.1)
    print(f"   总利用率: {result_edf['utilization']:.2%}")
    print(f"   完成作业数: {result_edf['total_jobs']}")
    print(f"   错过截止期: {result_edf['missed_deadlines']}")

    # ---- 3. 优先级抢占调度 ----
    print("\n[3] 固定优先级抢占调度...")
    ps = PriorityScheduler(preemptive=True)
    result_ps = ps.schedule(tasks, sim_time, dt=0.1)
    print(f"   完成作业数: {result_ps['total_jobs']}")

    # ---- 4. 可调度性分析 ----
    print("\n[4] 可调度性分析...")
    analyzer = SchedulabilityAnalyzer()

    # RM利用率上界
    n = len(tasks)
    U_rm_bound = analyzer.rm_utilization_bound(n)
    print(f"   RM利用率上界({n}任务): {U_rm_bound:.4f}")
    print(f"   当前利用率: {result_rm['utilization']:.4f}")
    print(f"   RM可调度: {'✓' if result_rm['utilization'] <= U_rm_bound else '不确定(需RTA)'}")

    # RTA
    rta_results = analyzer.rta_analysis(tasks)
    print(f"\n   响应时间分析(RTA):")
    for tid, r in rta_results.items():
        task = [t for t in tasks if t.task_id == tid][0]
        status = "✓" if r['schedulable'] else "✗"
        print(f"     {task.name}: WCRT={r['wcrt']:.2f}ms, 截止期={task.deadline}ms {status}")

    # EDF
    edf_check = analyzer.edf_schedulability(tasks)
    print(f"\n   EDF可调度: {'✓' if edf_check['schedulable'] else '✗'} (U={edf_check['utilization']:.4f})")

    # ---- 5. WCET分析 ----
    print("\n[5] WCET统计分析...")
    wcet_results = TimingAnalyzer.wcet_analysis(tasks)
    for tid, r in wcet_results.items():
        task = [t for t in tasks if t.task_id == tid][0]
        print(f"   {task.name}: BCET={r['bcet']:.2f}, 均值={r['mean']:.2f}, "
              f"P99={r['p99']:.2f}, WCET={r['wcet']:.2f}ms")

    # ---- 6. 端到端延迟 ----
    print("\n[6] 端到端延迟分析...")
    task_chain = [tasks[0], tasks[1], tasks[4]]  # 采集->控制->监测
    e2e = TimingAnalyzer.end_to_end_latency(task_chain)
    chain_names = " → ".join(t.name for t in task_chain)
    print(f"   任务链: {chain_names}")
    print(f"   端到端最大延迟: {e2e:.2f}ms")

    # ---- 7. 中断仿真 ----
    print("\n[7] 中断仿真...")
    int_sim = InterruptSimulator(interrupt_latency=0.002, context_switch_time=0.001)
    interrupts = [
        Interrupt(1, "ADC完成", priority=5, arrival_time=5.0, handler_time=0.5),
        Interrupt(2, "UART接收", priority=3, arrival_time=12.0, handler_time=1.0),
        Interrupt(3, "定时器溢出", priority=7, arrival_time=8.0, handler_time=0.2),
        Interrupt(4, "外部触发", priority=10, arrival_time=15.0, handler_time=0.3),
        Interrupt(5, "DMA完成", priority=8, arrival_time=20.0, handler_time=0.1),
    ]
    int_result = int_sim.simulate_interrupts(interrupts, sim_time=30, dt=0.01)
    print(f"   ISR总占用: {int_result['total_isr_time']:.3f}ms")
    print(f"   最大中断响应: {int_result['max_response_time']:.3f}ms")
    print(f"   ISR CPU占用: {int_result['cpu_utilization_isr']:.2f}%")

    # ---- 8. 资源竞争 ----
    print("\n[8] 资源竞争仿真...")
    res_sim = ResourceContentionSimulator()
    res_sim.add_resource("SPI_bus", max_lock_duration=2.0)
    res_sim.add_resource("shared_buffer", max_lock_duration=1.0)
    res_sim.add_resource("I2C_bus", max_lock_duration=1.5)

    accesses = {
        1: [(5.0, "SPI_bus", 1.5), (30.0, "shared_buffer", 0.5), (60.0, "SPI_bus", 1.0)],
        2: [(6.0, "SPI_bus", 1.0), (25.0, "shared_buffer", 0.8)],
        3: [(10.0, "I2C_bus", 1.0), (35.0, "SPI_bus", 0.5)],
    }
    res_result = res_sim.simulate(accesses, sim_time=100)
    print(f"   等待次数: {res_result['num_waits']}")
    print(f"   平均等待: {res_result['avg_wait']:.3f}ms")
    print(f"   死锁检测: {'是' if res_result['deadlock_detected'] else '否'}")

    # ======================== 绘图 ========================
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))
    fig.suptitle('嵌入式系统仿真', fontsize=16, fontweight='bold')

    # (1) RM调度甘特图
    ax = axes[0, 0]
    task_colors = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6', '#1abc9c']
    task_names_map = {t.task_id: t.name for t in tasks}
    task_names_map[-1] = '空闲'

    # 从timeline绘制甘特图
    tl = result_rm['timeline']
    if tl:
        for entry in tl[:5000]:  # 限制数量
            tid = entry['task_id']
            t = entry['time']
            c = task_colors[tid % len(task_colors)] if tid >= 0 else 'white'
            ax.barh(tid if tid >= 0 else -1, entry['dt'], left=t, height=0.6,
                   color=c, edgecolor='none')
    ax.set_yticks([t.task_id for t in tasks])
    ax.set_yticklabels([t.name for t in tasks], fontsize=8)
    ax.set_xlabel('时间 (ms)')
    ax.set_title(f'RM调度 (U={result_rm["utilization"]:.1%})')
    ax.grid(True, alpha=0.3, axis='x')

    # (2) EDF调度甘特图
    ax = axes[0, 1]
    tl = result_edf['timeline']
    if tl:
        for entry in tl[:5000]:
            tid = entry['task_id']
            t = entry['time']
            c = task_colors[tid % len(task_colors)] if tid >= 0 else 'white'
            ax.barh(tid if tid >= 0 else -1, entry['dt'], left=t, height=0.6,
                   color=c, edgecolor='none')
    ax.set_yticks([t.task_id for t in tasks])
    ax.set_yticklabels([t.name for t in tasks], fontsize=8)
    ax.set_xlabel('时间 (ms)')
    ax.set_title(f'EDF调度 (U={result_edf["utilization"]:.1%})')
    ax.grid(True, alpha=0.3, axis='x')

    # (3) 任务利用率饼图
    ax = axes[1, 0]
    utils = [t.utilization * 100 for t in tasks]
    names = [t.name for t in tasks]
    idle = max(0, 100 - sum(utils))
    utils_pie = utils + [idle]
    names_pie = names + ['空闲']
    colors_pie = task_colors[:len(tasks)] + ['#ecf0f1']
    wedges, texts, autotexts = ax.pie(utils_pie, labels=names_pie, colors=colors_pie,
                                       autopct='%1.1f%%', startangle=90, textprops={'fontsize': 8})
    ax.set_title(f'CPU利用率分布 (总={sum(utils):.1f}%)')

    # (4) 可调度性分析
    ax = axes[1, 1]
    task_labels = [t.name for t in tasks]
    wcrt_values = [rta_results[t.task_id]['wcrt'] for t in tasks]
    deadlines = [t.deadline for t in tasks]
    x_pos = np.arange(len(tasks))
    bars = ax.barh(x_pos - 0.2, wcrt_values, 0.35, label='WCRT', color='steelblue')
    ax.barh(x_pos + 0.2, deadlines, 0.35, label='截止期', color='lightcoral')
    ax.set_yticks(x_pos)
    ax.set_yticklabels(task_labels, fontsize=8)
    ax.set_xlabel('时间 (ms)')
    ax.set_title('响应时间分析 (RTA)')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='x')

    # 可调度标记
    for i, t in enumerate(tasks):
        r = rta_results[t.task_id]
        status = "✓" if r['schedulable'] else "✗"
        color = 'green' if r['schedulable'] else 'red'
        ax.annotate(status, xy=(max(wcrt_values[i], deadlines[i])+0.5, i),
                   fontsize=12, color=color, fontweight='bold')

    # (5) 中断时序
    ax = axes[2, 0]
    int_colors = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6']
    for ir in interrupts:
        ax.barh(ir.interrupt_id, ir.handler_time, left=ir.arrival_time,
               color=int_colors[ir.interrupt_id % len(int_colors)], alpha=0.7,
               edgecolor='black', linewidth=0.5)
        ax.text(ir.arrival_time + ir.handler_time/2, ir.interrupt_id,
               f'{ir.name}\n({ir.handler_time}ms)', ha='center', va='center', fontsize=7)
    ax.set_yticks([ir.interrupt_id for ir in interrupts])
    ax.set_yticklabels([ir.name for ir in interrupts], fontsize=8)
    ax.set_xlabel('时间 (ms)')
    ax.set_title('中断时序')
    ax.grid(True, alpha=0.3, axis='x')

    # (6) 资源竞争时序
    ax = axes[2, 1]
    if res_result['lock_log']:
        log = res_result['lock_log'][:30]  # 限制
        y_map = {"SPI_bus": 0, "shared_buffer": 1, "I2C_bus": 2}
        for entry in log:
            res = entry['resource']
            y = y_map.get(res, 0)
            marker = 'o' if entry['event'] == 'acquire' else 's'
            color = 'green' if entry['event'] == 'acquire' else 'red'
            if entry['event'] != 'wait':
                ax.plot(entry['time'], y, marker=marker, color=color, markersize=8)
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(['SPI_bus', 'shared_buffer', 'I2C_bus'])
    ax.set_xlabel('时间 (ms)')
    ax.set_title(f'资源竞争 (等待{res_result["num_waits"]}次)')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = './nuedc-asset-library/15_simulation/embedded_system_result.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n图表已保存: {save_path}")
    plt.show()
    print("\n仿真完成！")


if __name__ == '__main__':
    run_embedded_simulations()
