#!/usr/bin/env python3
"""
任务调度器单元测试
覆盖: 任务添加/删除、优先级调度、时间触发、溢出处理、性能基准
"""

import sys
import os
import unittest
import time
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class Task:
    """任务实体"""
    def __init__(self, task_id, callback, period_ms=0, priority=0, repeat=True):
        self.task_id = task_id
        self.callback = callback
        self.period_ms = period_ms
        self.priority = priority
        self.repeat = repeat
        self.last_run = 0
        self.enabled = True
        self.run_count = 0


class TaskScheduler:
    """任务调度器简化实现"""

    MAX_TASKS = 32

    def __init__(self):
        self.tasks = {}       # task_id -> Task
        self.tick_count = 0
        self.overflow = False

    def add_task(self, task_id, callback, period_ms=0, priority=0, repeat=True):
        """添加任务"""
        if len(self.tasks) >= self.MAX_TASKS:
            self.overflow = True
            return False
        if task_id in self.tasks:
            return False  # 重复ID
        task = Task(task_id, callback, period_ms, priority, repeat)
        self.tasks[task_id] = task
        return True

    def remove_task(self, task_id):
        """删除任务"""
        if task_id not in self.tasks:
            return False
        del self.tasks[task_id]
        return True

    def enable_task(self, task_id, enabled=True):
        """启用/禁用任务"""
        if task_id not in self.tasks:
            return False
        self.tasks[task_id].enabled = enabled
        return True

    def tick(self, current_time_ms=None):
        """调度一次tick"""
        if current_time_ms is None:
            current_time_ms = int(time.time() * 1000)
        self.tick_count += 1

        # 按优先级排序(数值越小优先级越高)
        sorted_tasks = sorted(self.tasks.values(), key=lambda t: t.priority)
        executed = []

        for task in sorted_tasks:
            if not task.enabled:
                continue
            if task.period_ms > 0:
                if current_time_ms - task.last_run >= task.period_ms:
                    task.callback()
                    task.last_run = current_time_ms
                    task.run_count += 1
                    executed.append(task.task_id)
                    if not task.repeat:
                        task.enabled = False
            else:
                # 无周期任务每次tick都执行
                task.callback()
                task.run_count += 1
                executed.append(task.task_id)

        return executed

    def get_task_count(self):
        """获取任务数量"""
        return len(self.tasks)

    def get_task(self, task_id):
        """获取任务"""
        return self.tasks.get(task_id, None)


class TestTaskScheduler(unittest.TestCase):
    """任务调度器测试"""

    def test_add_task(self):
        """测试添加任务"""
        scheduler = TaskScheduler()
        result = scheduler.add_task("task1", lambda: None, period_ms=100)
        self.assertTrue(result)
        self.assertEqual(scheduler.get_task_count(), 1)

    def test_add_duplicate_task(self):
        """测试添加重复任务ID"""
        scheduler = TaskScheduler()
        scheduler.add_task("task1", lambda: None)
        result = scheduler.add_task("task1", lambda: None)
        self.assertFalse(result)
        self.assertEqual(scheduler.get_task_count(), 1)

    def test_remove_task(self):
        """测试删除任务"""
        scheduler = TaskScheduler()
        scheduler.add_task("task1", lambda: None)
        result = scheduler.remove_task("task1")
        self.assertTrue(result)
        self.assertEqual(scheduler.get_task_count(), 0)

    def test_remove_nonexistent_task(self):
        """测试删除不存在的任务"""
        scheduler = TaskScheduler()
        result = scheduler.remove_task("nonexistent")
        self.assertFalse(result)

    def test_enable_disable_task(self):
        """测试启用/禁用任务"""
        scheduler = TaskScheduler()
        scheduler.add_task("task1", lambda: None, period_ms=0)
        scheduler.enable_task("task1", False)
        executed = scheduler.tick()
        self.assertEqual(len(executed), 0)
        scheduler.enable_task("task1", True)
        executed = scheduler.tick()
        self.assertEqual(len(executed), 1)

    def test_enable_nonexistent_task(self):
        """测试启用不存在的任务"""
        scheduler = TaskScheduler()
        result = scheduler.enable_task("nonexistent")
        self.assertFalse(result)

    def test_priority_scheduling(self):
        """测试优先级调度顺序"""
        scheduler = TaskScheduler()
        order = []
        scheduler.add_task("low", lambda: order.append("low"), priority=10)
        scheduler.add_task("high", lambda: order.append("high"), priority=1)
        scheduler.add_task("mid", lambda: order.append("mid"), priority=5)
        scheduler.tick()
        self.assertEqual(order, ["high", "mid", "low"])

    def test_periodic_task(self):
        """测试周期任务"""
        scheduler = TaskScheduler()
        call_count = [0]
        def cb(): call_count[0] += 1
        scheduler.add_task("periodic", cb, period_ms=100)
        # 使用非零起始时间, last_run初始化为0
        scheduler.tick(current_time_ms=1000)
        self.assertEqual(call_count[0], 1)
        # 50ms后不应执行
        scheduler.tick(current_time_ms=1050)
        self.assertEqual(call_count[0], 1)
        # 100ms后应执行
        scheduler.tick(current_time_ms=1100)
        self.assertEqual(call_count[0], 2)

    def test_one_shot_task(self):
        """测试一次性任务"""
        scheduler = TaskScheduler()
        call_count = [0]
        def cb(): call_count[0] += 1
        scheduler.add_task("once", cb, period_ms=100, repeat=False)
        scheduler.tick(current_time_ms=0)
        scheduler.tick(current_time_ms=100)
        scheduler.tick(current_time_ms=200)
        self.assertEqual(call_count[0], 1)

    def test_max_tasks_overflow(self):
        """测试任务数量溢出"""
        scheduler = TaskScheduler()
        for i in range(TaskScheduler.MAX_TASKS):
            scheduler.add_task(f"task_{i}", lambda: None)
        result = scheduler.add_task("overflow", lambda: None)
        self.assertFalse(result)
        self.assertTrue(scheduler.overflow)

    def test_non_periodic_task(self):
        """测试非周期任务每次tick执行"""
        scheduler = TaskScheduler()
        call_count = [0]
        def cb(): call_count[0] += 1
        scheduler.add_task("every_tick", cb, period_ms=0)
        for _ in range(10):
            scheduler.tick()
        self.assertEqual(call_count[0], 10)

    def test_get_task(self):
        """测试获取任务信息"""
        scheduler = TaskScheduler()
        scheduler.add_task("task1", lambda: None, period_ms=100, priority=5)
        task = scheduler.get_task("task1")
        self.assertIsNotNone(task)
        self.assertEqual(task.period_ms, 100)
        self.assertEqual(task.priority, 5)

    def test_get_nonexistent_task(self):
        """测试获取不存在的任务"""
        scheduler = TaskScheduler()
        task = scheduler.get_task("nonexistent")
        self.assertIsNone(task)

    def test_multiple_tasks_execution(self):
        """测试多任务执行"""
        scheduler = TaskScheduler()
        results = []
        scheduler.add_task("a", lambda: results.append("a"), period_ms=0)
        scheduler.add_task("b", lambda: results.append("b"), period_ms=0)
        scheduler.add_task("c", lambda: results.append("c"), period_ms=0)
        scheduler.tick()
        self.assertEqual(len(results), 3)

    def test_empty_scheduler_tick(self):
        """测试空调度器tick"""
        scheduler = TaskScheduler()
        executed = scheduler.tick()
        self.assertEqual(len(executed), 0)

    def test_performance_benchmark(self):
        """性能基准: 调度器tick性能"""
        scheduler = TaskScheduler()
        for i in range(20):
            scheduler.add_task(f"task_{i}", lambda: None, period_ms=0)
        iterations = 5000
        start = time.perf_counter()
        for _ in range(iterations):
            scheduler.tick()
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 500.0, "调度器tick应<500μs")


if __name__ == '__main__':
    unittest.main()
