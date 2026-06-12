#!/usr/bin/env python3
"""
事件系统单元测试
覆盖: 事件注册/注销、事件触发、优先级处理、一次性监听、性能基准
"""

import sys
import os
import unittest
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class EventSystem:
    """事件系统简化实现"""

    def __init__(self):
        self._listeners = {}  # event_name -> [(priority, callback, once)]
        self._event_log = []

    def on(self, event_name, callback, priority=0, once=False):
        """注册事件监听"""
        if event_name not in self._listeners:
            self._listeners[event_name] = []
        self._listeners[event_name].append((priority, callback, once))
        self._listeners[event_name].sort(key=lambda x: x[0])

    def off(self, event_name, callback=None):
        """注销事件监听"""
        if event_name not in self._listeners:
            return False
        if callback is None:
            del self._listeners[event_name]
            return True
        original_len = len(self._listeners[event_name])
        self._listeners[event_name] = [
            (p, c, o) for p, c, o in self._listeners[event_name] if c != callback
        ]
        if not self._listeners[event_name]:
            del self._listeners[event_name]
        return len(self._listeners.get(event_name, [])) < original_len

    def emit(self, event_name, *args, **kwargs):
        """触发事件"""
        self._event_log.append(event_name)
        results = []
        if event_name not in self._listeners:
            return results
        to_remove = []
        for i, (priority, callback, once) in enumerate(self._listeners[event_name]):
            result = callback(*args, **kwargs)
            results.append(result)
            if once:
                to_remove.append(i)
        # 移除一次性监听
        for i in reversed(to_remove):
            self._listeners[event_name].pop(i)
        if not self._listeners.get(event_name):
            self._listeners.pop(event_name, None)
        return results

    def has_listeners(self, event_name):
        """检查是否有监听器"""
        return event_name in self._listeners and len(self._listeners[event_name]) > 0

    def get_listener_count(self, event_name):
        """获取监听器数量"""
        return len(self._listeners.get(event_name, []))

    def get_event_log(self):
        """获取事件日志"""
        return list(self._event_log)


class TestEventSystem(unittest.TestCase):
    """事件系统测试"""

    def test_on_and_emit(self):
        """测试注册和触发事件"""
        es = EventSystem()
        result = []
        es.on("test", lambda x: result.append(x))
        es.emit("test", 42)
        self.assertEqual(result, [42])

    def test_multiple_listeners(self):
        """测试多个监听器"""
        es = EventSystem()
        results = []
        es.on("test", lambda: results.append("a"))
        es.on("test", lambda: results.append("b"))
        es.emit("test")
        self.assertEqual(results, ["a", "b"])

    def test_priority_order(self):
        """测试优先级顺序"""
        es = EventSystem()
        results = []
        es.on("test", lambda: results.append("low"), priority=10)
        es.on("test", lambda: results.append("high"), priority=1)
        es.on("test", lambda: results.append("mid"), priority=5)
        es.emit("test")
        self.assertEqual(results, ["high", "mid", "low"])

    def test_once_listener(self):
        """测试一次性监听器"""
        es = EventSystem()
        count = [0]
        es.on("test", lambda: count.__setitem__(0, count[0] + 1), once=True)
        es.emit("test")
        es.emit("test")
        self.assertEqual(count[0], 1)
        self.assertFalse(es.has_listeners("test"))

    def test_off_specific_callback(self):
        """测试注销特定回调"""
        es = EventSystem()
        cb = lambda: None
        es.on("test", cb)
        result = es.off("test", cb)
        self.assertTrue(result)
        self.assertFalse(es.has_listeners("test"))

    def test_off_all_listeners(self):
        """测试注销所有监听器"""
        es = EventSystem()
        es.on("test", lambda: None)
        es.on("test", lambda: None)
        result = es.off("test")
        self.assertTrue(result)
        self.assertFalse(es.has_listeners("test"))

    def test_off_nonexistent_event(self):
        """测试注销不存在的事件"""
        es = EventSystem()
        result = es.off("nonexistent")
        self.assertFalse(result)

    def test_emit_no_listeners(self):
        """测试触发无监听器的事件"""
        es = EventSystem()
        results = es.emit("nonexistent")
        self.assertEqual(results, [])

    def test_emit_with_args(self):
        """测试带参数触发"""
        es = EventSystem()
        received = []
        es.on("data", lambda x, y: received.append(x + y))
        es.emit("data", 3, 4)
        self.assertEqual(received, [7])

    def test_emit_with_kwargs(self):
        """测试带关键字参数触发"""
        es = EventSystem()
        received = []
        es.on("data", lambda **kw: received.append(kw))
        es.emit("data", name="test", value=42)
        self.assertEqual(received, [{"name": "test", "value": 42}])

    def test_event_log(self):
        """测试事件日志"""
        es = EventSystem()
        es.emit("a")
        es.emit("b")
        es.emit("a")
        self.assertEqual(es.get_event_log(), ["a", "b", "a"])

    def test_listener_count(self):
        """测试监听器数量"""
        es = EventSystem()
        es.on("test", lambda: None)
        es.on("test", lambda: None)
        self.assertEqual(es.get_listener_count("test"), 2)

    def test_has_listeners(self):
        """测试检查监听器"""
        es = EventSystem()
        self.assertFalse(es.has_listeners("test"))
        es.on("test", lambda: None)
        self.assertTrue(es.has_listeners("test"))

    def test_return_values(self):
        """测试返回值收集"""
        es = EventSystem()
        es.on("calc", lambda: 10)
        es.on("calc", lambda: 20)
        results = es.emit("calc")
        self.assertEqual(results, [10, 20])

    def test_chained_events(self):
        """测试链式事件"""
        es = EventSystem()
        es.on("start", lambda: es.emit("middle"))
        es.on("middle", lambda: es.emit("end"))
        end_called = [False]
        es.on("end", lambda: end_called.__setitem__(0, True))
        es.emit("start")
        self.assertTrue(end_called[0])

    def test_performance_benchmark(self):
        """性能基准: 事件触发性能"""
        es = EventSystem()
        for i in range(10):
            es.on("perf", lambda: None)
        iterations = 10000
        start = time.perf_counter()
        for _ in range(iterations):
            es.emit("perf")
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 200.0, "事件触发应<200μs")


if __name__ == '__main__':
    unittest.main()
