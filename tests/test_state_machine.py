#!/usr/bin/env python3
"""
状态机单元测试
覆盖: 状态定义、状态转移、事件分发、进入/退出回调、层次状态、性能基准

V2修复: import wrappers.py中的生产代码逻辑，而非自行重写
对应C源: 02_mspm0g3507/drivers/state_machine.c
"""

import sys
import os
import unittest
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from tests.wrappers import StateMachine, SMStateDesc, SMEvent, SM_NO_PARENT


class TestStateMachine(unittest.TestCase):
    """状态机测试（与C版本SM_Machine逻辑一致）"""

    def _make_simple_table(self, states, transitions):
        """辅助: 构建简单状态表"""
        table = [SMStateDesc() for _ in range(states)]
        for from_s, event_id, to_s in transitions:
            def make_handler(target):
                return lambda sm, evt: (sm.transition(target), True)[1]
            table[from_s].on_event = make_handler(to_s)
        return table

    def test_initialization(self):
        """测试初始化"""
        table = [SMStateDesc(), SMStateDesc(), SMStateDesc()]
        sm = StateMachine(table, init_state=0)
        self.assertEqual(sm.get_state(), 0)
        self.assertFalse(sm.is_running_state())

    def test_start(self):
        """测试启动状态机"""
        entered = [False]
        def on_enter(sm, evt):
            entered[0] = True
        table = [SMStateDesc(on_enter=on_enter), SMStateDesc()]
        sm = StateMachine(table, init_state=0)
        sm.start()
        self.assertTrue(sm.is_running_state())
        self.assertTrue(entered[0])

    def test_dispatch_event(self):
        """测试事件分发"""
        handled_events = []
        def on_event_0(sm, evt):
            handled_events.append(evt.id)
            return True
        table = [SMStateDesc(on_event=on_event_0), SMStateDesc()]
        sm = StateMachine(table, init_state=0)
        sm.start()
        result = sm.dispatch(SMEvent(eid=1))
        self.assertTrue(result)
        self.assertEqual(handled_events, [1])

    def test_dispatch_returns_false_when_unhandled(self):
        """测试未处理事件返回False"""
        table = [SMStateDesc(), SMStateDesc()]  # 无on_event处理器
        sm = StateMachine(table, init_state=0)
        sm.start()
        result = sm.dispatch(SMEvent(eid=99))
        self.assertFalse(result)

    def test_dispatch_returns_false_when_not_running(self):
        """测试未启动时分发事件返回False"""
        table = [SMStateDesc(), SMStateDesc()]
        sm = StateMachine(table, init_state=0)
        result = sm.dispatch(SMEvent(eid=1))
        self.assertFalse(result)

    def test_transition(self):
        """测试状态转换"""
        enter_called = [False]
        exit_called = [False]
        def on_exit_0(sm, evt):
            exit_called[0] = True
        def on_enter_1(sm, evt):
            enter_called[0] = True
        table = [SMStateDesc(on_exit=on_exit_0), SMStateDesc(on_enter=on_enter_1)]
        sm = StateMachine(table, init_state=0)
        sm.start()
        sm.transition(1)
        self.assertEqual(sm.get_state(), 1)
        self.assertEqual(sm.get_previous_state(), 0)
        self.assertTrue(exit_called[0])
        self.assertTrue(enter_called[0])

    def test_transition_same_state_noop(self):
        """测试同状态转换无操作"""
        enter_count = [0]
        def on_enter(sm, evt):
            enter_count[0] += 1
        table = [SMStateDesc(on_enter=on_enter)]
        sm = StateMachine(table, init_state=0)
        sm.start()
        enter_count[0] = 0
        sm.transition(0)  # 同状态
        self.assertEqual(enter_count[0], 0)

    def test_transition_invalid_state(self):
        """测试无效状态ID被忽略"""
        table = [SMStateDesc(), SMStateDesc()]
        sm = StateMachine(table, init_state=0)
        sm.start()
        sm.transition(99)  # 无效
        self.assertEqual(sm.get_state(), 0)

    def test_return_to_previous(self):
        """测试返回上一个状态"""
        table = [SMStateDesc(), SMStateDesc(), SMStateDesc()]
        sm = StateMachine(table, init_state=0)
        sm.start()
        sm.transition(1)
        sm.transition(2)
        sm.return_to_previous()
        self.assertEqual(sm.get_state(), 1)

    def test_state_ticks(self):
        """测试状态计时器"""
        table = [SMStateDesc(), SMStateDesc()]
        sm = StateMachine(table, init_state=0)
        sm.start()
        self.assertEqual(sm.get_state_ticks(), 0)
        sm.tick()
        sm.tick()
        sm.tick()
        self.assertEqual(sm.get_state_ticks(), 3)

    def test_state_ticks_reset_on_transition(self):
        """测试转换时计时器重置"""
        table = [SMStateDesc(), SMStateDesc()]
        sm = StateMachine(table, init_state=0)
        sm.start()
        sm.tick()
        sm.tick()
        sm.transition(1)
        self.assertEqual(sm.get_state_ticks(), 0)

    def test_stop(self):
        """测试停止状态机"""
        table = [SMStateDesc()]
        sm = StateMachine(table, init_state=0)
        sm.start()
        self.assertTrue(sm.is_running_state())
        sm.stop()
        self.assertFalse(sm.is_running_state())

    def test_parent_state_event_fallback(self):
        """测试事件沿父状态链传播"""
        parent_handled = [False]
        def on_event_parent(sm, evt):
            parent_handled[0] = True
            return True
        table = [
            SMStateDesc(parent=SM_NO_PARENT, on_event=on_event_parent),  # state 0 = parent
            SMStateDesc(parent=0),  # state 1 = child, no on_event
        ]
        sm = StateMachine(table, init_state=1)
        sm.start()
        sm.dispatch(SMEvent(eid=5))
        self.assertTrue(parent_handled[0])

    def test_complex_scenario(self):
        """测试复杂场景：多状态顺序转换"""
        sequence = []
        def on_enter_s0(sm, evt):
            sequence.append("enter_s0")
        def on_enter_s1(sm, evt):
            sequence.append("enter_s1")
        def on_enter_s2(sm, evt):
            sequence.append("enter_s2")
        def on_exit_s0(sm, evt):
            sequence.append("exit_s0")
        def on_exit_s1(sm, evt):
            sequence.append("exit_s1")

        def on_event_s0(sm, evt):
            sm.transition(1)
            return True
        def on_event_s1(sm, evt):
            sm.transition(2)
            return True

        table = [
            SMStateDesc(on_enter=on_enter_s0, on_exit=on_exit_s0, on_event=on_event_s0),
            SMStateDesc(on_enter=on_enter_s1, on_exit=on_exit_s1, on_event=on_event_s1),
            SMStateDesc(on_enter=on_enter_s2),
        ]
        sm = StateMachine(table, init_state=0)
        sm.start()
        self.assertEqual(sequence, ["enter_s0"])

        sm.dispatch(SMEvent(eid=1))
        self.assertEqual(sequence, ["enter_s0", "exit_s0", "enter_s1"])

        sm.dispatch(SMEvent(eid=1))
        self.assertEqual(sequence, ["enter_s0", "exit_s0", "enter_s1", "exit_s1", "enter_s2"])

    def test_performance_benchmark(self):
        """性能基准: 事件分发性能"""
        count = [0]
        def on_event(sm, evt):
            count[0] += 1
            return True
        table = [SMStateDesc(on_event=on_event)]
        sm = StateMachine(table, init_state=0)
        sm.start()
        iterations = 100000
        start = time.perf_counter()
        for _ in range(iterations):
            sm.dispatch(SMEvent(eid=0))
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 50.0, "事件分发应<50μs")


if __name__ == '__main__':
    unittest.main()
