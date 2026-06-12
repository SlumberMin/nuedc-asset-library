#!/usr/bin/env python3
"""
实时调度仿真
- RM (速率单调调度)
- EDF (最早截止期优先调度)
- CBS (常量带宽服务器)
- 可调度性分析
- 响应时间分析
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from heapq import heappush, heappop

# ============================================================
# 1. 任务模型
# ============================================================

@dataclass
class Task:
    id: int
    period: float        # 周期 (ms)
    execution: float     # 最坏执行时间 (ms)
    deadline: float      # 相对截止期 (ms)
    priority: int = 0    # 优先级 (RM 由周期决定)
    color: str = 'blue'

    def utilization(self) -> float:
        return self.execution / self.period


@dataclass
class Job:
    task_id: int
    release_time: float
    deadline: float
    remaining: float
    period: float

    def __lt__(self, other):
        return self.deadline < other.deadline  # EDF 排序


# ============================================================
# 2. 可调度性分析
# ============================================================

def rm_schedulability_test(tasks: List[Task]) -> Tuple[bool, float]:
    """
    RM 可调度性测试
    充分条件: U <= n(2^(1/n) - 1)
    """
    n = len(tasks)
    U = sum(t.utilization() for t in tasks)
    bound = n * (2**(1/n) - 1)
    schedulable = U <= bound
    return schedulable, U

def edf_schedulability_test(tasks: List[Task]) -> Tuple[bool, float]:
    """
    EDF 可调度性测试
    充要条件: U <= 1
    """
    U = sum(t.utilization() for t in tasks)
    return U <= 1.0, U

def response_time_analysis(tasks: List[Task]) -> List[float]:
    """
    响应时间分析 (RM)
    R_i = C_i + sum(ceil(R_i/T_j) * C_j) for all higher priority j
    """
    # 按周期排序 (周期短 = 优先级高)
    sorted_tasks = sorted(tasks, key=lambda t: t.period)
    responses = []

    for i, task in enumerate(sorted_tasks):
        R = task.execution
        for _ in range(100):  # 迭代
            R_new = task.execution
            for j in range(i):
                R_new += np.ceil(R / sorted_tasks[j].period) * sorted_tasks[j].execution
            if abs(R_new - R) < 0.001:
                break
            R = R_new
        responses.append(R)

    return responses


# ============================================================
# 3. RM 调度器
# ============================================================

class RMScheduler:
    """速率单调调度器"""

    def __init__(self, tasks: List[Task]):
        self.tasks = sorted(tasks, key=lambda t: t.period)
        for i, t in enumerate(self.tasks):
            t.priority = len(tasks) - i  # 周期短优先级高

    def schedule(self, hyperperiod: float) -> List[Tuple[float, float, int]]:
        """返回调度结果: [(start, end, task_id), ...]"""
        timeline = []
        ready_queue = []  # (priority, task_index, job)
        current_time = 0.0
        current_task = None
        current_start = 0.0

        # 初始化释放
        next_release = {}
        for i, t in enumerate(self.tasks):
            next_release[i] = 0.0

        while current_time < hyperperiod:
            # 释放新任务
            for i, t in enumerate(self.tasks):
                if abs(current_time - next_release[i]) < 0.001:
                    heappush(ready_queue, (-t.priority, i, t.execution))
                    next_release[i] += t.period

            # 选择最高优先级任务
            if ready_queue:
                neg_pri, idx, remaining = heappop(ready_queue)
                t = self.tasks[idx]

                if current_task != idx:
                    if current_task is not None:
                        timeline.append((current_start, current_time, current_task))
                    current_task = idx
                    current_start = current_time

                # 执行到下一个事件
                next_event = hyperperiod
                for i, t2 in enumerate(self.tasks):
                    nr = next_release[i]
                    if nr > current_time and nr < next_event:
                        next_event = nr

                exec_time = min(remaining, next_event - current_time)
                remaining -= exec_time
                current_time += exec_time

                if remaining > 0.001:
                    heappush(ready_queue, (-t.priority, idx, remaining))
                else:
                    # 任务完成
                    if current_task is not None:
                        timeline.append((current_start, current_time, current_task))
                        current_task = None
            else:
                # 空闲
                if current_task is not None:
                    timeline.append((current_start, current_time, current_task))
                    current_task = None
                next_event = min(next_release.values())
                if next_event > current_time:
                    current_time = next_event

        if current_task is not None:
            timeline.append((current_start, current_time, current_task))

        return timeline


# ============================================================
# 4. EDF 调度器
# ============================================================

class EDFScheduler:
    """最早截止期优先调度器"""

    def __init__(self, tasks: List[Task]):
        self.tasks = tasks

    def schedule(self, hyperperiod: float) -> List[Tuple[float, float, int]]:
        """返回调度结果"""
        timeline = []
        ready_queue = []  # (deadline, task_index, remaining)
        current_time = 0.0
        current_task = None
        current_start = 0.0

        next_release = {i: 0.0 for i in range(len(self.tasks))}

        while current_time < hyperperiod:
            # 释放新任务
            for i, t in enumerate(self.tasks):
                if abs(current_time - next_release[i]) < 0.001:
                    deadline = current_time + t.deadline
                    heappush(ready_queue, (deadline, i, t.execution, t.period))
                    next_release[i] += t.period

            if ready_queue:
                dl, idx, remaining, period = heappop(ready_queue)
                t = self.tasks[idx]

                if current_task != idx:
                    if current_task is not None:
                        timeline.append((current_start, current_time, current_task))
                    current_task = idx
                    current_start = current_time

                next_event = hyperperiod
                for i, t2 in enumerate(self.tasks):
                    nr = next_release[i]
                    if nr > current_time and nr < next_event:
                        next_event = nr

                exec_time = min(remaining, next_event - current_time)
                remaining -= exec_time
                current_time += exec_time

                if remaining > 0.001:
                    heappush(ready_queue, (dl, idx, remaining, period))
                else:
                    if current_task is not None:
                        timeline.append((current_start, current_time, current_task))
                        current_task = None
            else:
                if current_task is not None:
                    timeline.append((current_start, current_time, current_task))
                    current_task = None
                next_event = min(next_release.values())
                if next_event > current_time:
                    current_time = next_event

        if current_task is not None:
            timeline.append((current_start, current_time, current_task))

        return timeline


# ============================================================
# 5. CBS (常量带宽服务器) - 用于软实时/混合任务
# ============================================================

@dataclass
class Server:
    budget: float     # 周期预算
    period: float     # 服务器周期
    remaining: float = 0.0

    def replenish(self):
        self.remaining = self.budget

    def utilization(self) -> float:
        return self.budget / self.period


class CBSScheduler:
    """常量带宽服务器调度器 (简化版)"""

    def __init__(self, periodic_tasks: List[Task], server: Server):
        self.tasks = periodic_tasks
        self.server = server

    def schedule(self, hyperperiod: float) -> List[Tuple[float, float, int]]:
        """混合调度: 周期任务用 RM, 非周期任务用 CBS"""
        # 简化: 将服务器视为一个周期任务
        server_task = Task(
            id=-1,
            period=self.server.period,
            execution=self.server.budget,
            deadline=self.server.period,
            color='gray'
        )
        all_tasks = self.tasks + [server_task]
        rm = RMScheduler(all_tasks)
        return rm.schedule(hyperperiod)


# ============================================================
# 6. 可视化
# ============================================================

def visualize_schedule(timeline: List[Tuple[float, float, int]],
                       tasks: List[Task], title: str, save_name: str):
    """甘特图可视化"""
    fig, ax = plt.subplots(figsize=(14, 4))

    colors = plt.cm.Set3(np.linspace(0, 1, len(tasks)))
    task_colors = {t.id: colors[i] for i, t in enumerate(tasks)}

    for start, end, task_id in timeline:
        color = task_colors.get(task_id, 'gray')
        ax.barh(task_id, end - start, left=start, height=0.6,
                color=color, edgecolor='black', linewidth=0.5)

    # 标记截止期
    for t in tasks:
        for k in range(int(timeline[-1][1] / t.period) + 1):
            dl = k * t.period + t.deadline
            ax.plot(dl, t.id, 'rv', markersize=5)

    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('任务 ID')
    ax.set_title(title)
    ax.set_yticks([t.id for t in tasks])
    ax.set_yticklabels([f'Task {t.id} (T={t.period}, C={t.execution})' for t in tasks])
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(f'./nuedc-asset-library/15_simulation/{save_name}', dpi=150)
    plt.show()


def visualize_utilization(tasks: List[Task]):
    """利用率分析图"""
    utils = [t.utilization() for t in tasks]
    U_total = sum(utils)
    n = len(tasks)
    rm_bound = n * (2**(1/n) - 1)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # 各任务利用率
    ax1.bar(range(n), utils, color='steelblue')
    ax1.set_xlabel('任务 ID')
    ax1.set_ylabel('利用率')
    ax1.set_title('各任务利用率')
    ax1.axhline(1/n, color='r', linestyle='--', label=f'均匀分配={1/n:.3f}')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 总利用率 vs 边界
    ax2.bar(['总利用率', 'RM边界', 'EDF边界'], [U_total, rm_bound, 1.0],
            color=['steelblue', 'orange', 'green'])
    ax2.set_ylabel('利用率')
    ax2.set_title(f'可调度性分析 (U={U_total:.3f})')
    ax2.axhline(1.0, color='red', linestyle='--')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/scheduling_analysis.png', dpi=150)
    plt.show()


# ============================================================
# 7. 仿真场景
# ============================================================

def demo_rm():
    """RM 调度演示"""
    print("\n--- RM (速率单调) 调度 ---")
    tasks = [
        Task(0, period=10, execution=2, deadline=10),
        Task(1, period=15, execution=3, deadline=15),
        Task(2, period=25, execution=5, deadline=25),
    ]

    schedulable, U = rm_schedulability_test(tasks)
    print(f"  利用率: {U:.3f}")
    print(f"  RM可调度: {'✅ 是' if schedulable else '❌ 否'}")

    responses = response_time_analysis(tasks)
    sorted_tasks = sorted(tasks, key=lambda t: t.period)
    for t, r in zip(sorted_tasks, responses):
        print(f"  Task {t.id}: 响应时间={r:.2f}ms (截止期={t.deadline}ms) "
              f"{'✅' if r <= t.deadline else '❌'}")

    hyperperiod = np.lcm.reduce([int(t.period) for t in tasks])
    rm = RMScheduler(tasks)
    timeline = rm.schedule(hyperperiod)
    visualize_schedule(timeline, tasks, f'RM 调度 (U={U:.3f})', 'rm_schedule.png')

def demo_edf():
    """EDF 调度演示"""
    print("\n--- EDF (最早截止期优先) 调度 ---")
    tasks = [
        Task(0, period=10, execution=3, deadline=10),
        Task(1, period=15, execution=4, deadline=15),
        Task(2, period=20, execution=5, deadline=20),
    ]

    schedulable, U = edf_schedulability_test(tasks)
    print(f"  利用率: {U:.3f}")
    print(f"  EDF可调度: {'✅ 是' if schedulable else '❌ 否'}")

    hyperperiod = np.lcm.reduce([int(t.period) for t in tasks])
    edf = EDFScheduler(tasks)
    timeline = edf.schedule(hyperperiod)
    visualize_schedule(timeline, tasks, f'EDF 调度 (U={U:.3f})', 'edf_schedule.png')

def demo_cbs():
    """CBS 调度演示"""
    print("\n--- CBS (常量带宽服务器) 调度 ---")
    tasks = [
        Task(0, period=10, execution=2, deadline=10),
        Task(1, period=20, execution=3, deadline=20),
    ]
    server = Server(budget=4, period=10)

    U_periodic = sum(t.utilization() for t in tasks)
    U_server = server.utilization()
    U_total = U_periodic + U_server
    print(f"  周期任务利用率: {U_periodic:.3f}")
    print(f"  服务器利用率: {U_server:.3f}")
    print(f"  总利用率: {U_total:.3f}")

    hyperperiod = np.lcm.reduce([int(t.period) for t in tasks] + [int(server.period)])
    cbs = CBSScheduler(tasks, server)
    all_tasks = tasks + [Task(-1, server.period, server.budget, server.period)]
    timeline = cbs.schedule(hyperperiod)
    visualize_schedule(timeline, all_tasks, f'CBS 调度 (U={U_total:.3f})', 'cbs_schedule.png')


if __name__ == '__main__':
    print("=" * 60)
    print("  实时调度仿真")
    print("=" * 60)

    demo_rm()
    demo_edf()
    demo_cbs()

    # 综合分析
    tasks = [
        Task(0, period=10, execution=2, deadline=10),
        Task(1, period=15, execution=3, deadline=15),
        Task(2, period=25, execution=5, deadline=25),
    ]
    visualize_utilization(tasks)

    print("\n✅ 所有调度仿真完成")
