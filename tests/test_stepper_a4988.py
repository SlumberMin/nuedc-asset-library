#!/usr/bin/env python3
"""
A4988步进电机 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、使能/失能、速度设置、细分、绝对/相对移动、停止
对应C源文件: 02_mspm0g3507/drivers/stepper_a4988.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #7:  速度限幅保护
  #5:  使能状态下才能步进
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    StepperA4988,
    STEPPER_DIR_CW, STEPPER_DIR_CCW,
    STEPPER_MAX_SPEED,
)


class TestStepperA4988V2(unittest.TestCase):
    """A4988步进电机V2 — 基于wrappers.py包装层"""

    def setUp(self):
        self.motor = StepperA4988()
        self.motor.init(step_pin=10, dir_pin=11, en_pin=12)

    def test_init_state(self):
        """初始化后状态正确"""
        self.assertTrue(self.motor.initialized)
        self.assertEqual(self.motor.position, 0)
        self.assertEqual(self.motor.target, 0)
        self.assertFalse(self.motor.enabled)
        self.assertEqual(self.motor.step_pin, 10)
        self.assertEqual(self.motor.dir_pin, 11)
        self.assertEqual(self.motor.en_pin, 12)
        self.assertEqual(self.motor.microstep, 1)

    def test_enable_disable(self):
        """使能/失能控制"""
        self.motor.enable()
        self.assertTrue(self.motor.enabled)
        self.motor.disable()
        self.assertFalse(self.motor.enabled)

    def test_set_speed_normal(self):
        """正常速度设置"""
        self.motor.set_speed(500)
        self.assertEqual(self.motor.speed, 500)

    def test_set_speed_clamp_max(self):
        """速度上限限幅"""
        self.motor.set_speed(9999)
        self.assertEqual(self.motor.speed, STEPPER_MAX_SPEED)

    def test_set_speed_clamp_min(self):
        """速度下限限幅"""
        self.motor.set_speed(-100)
        self.assertEqual(self.motor.speed, 0)

    def test_set_microstep_valid(self):
        """设置有效细分值"""
        for ms in [1, 2, 4, 8, 16]:
            self.assertTrue(self.motor.set_microstep(ms))
            self.assertEqual(self.motor.microstep, ms)

    def test_set_microstep_invalid(self):
        """设置无效细分值应失败"""
        self.assertFalse(self.motor.set_microstep(3))
        self.assertFalse(self.motor.set_microstep(0))
        self.assertFalse(self.motor.set_microstep(32))

    def test_move_to_forward(self):
        """正方向移动"""
        self.motor.enable()
        self.motor.move_to(100)
        self.assertEqual(self.motor.target, 100)
        self.assertEqual(self.motor.direction, STEPPER_DIR_CW)

    def test_move_to_backward(self):
        """反方向移动"""
        self.motor.enable()
        self.motor.move_to(-50)
        self.assertEqual(self.motor.target, -50)
        self.assertEqual(self.motor.direction, STEPPER_DIR_CCW)

    def test_step_forward(self):
        """逐步前进"""
        self.motor.enable()
        self.motor.move_to(5)
        steps = []
        while self.motor.step():
            steps.append(self.motor.position)
        self.assertEqual(steps, [1, 2, 3, 4, 5])
        self.assertEqual(self.motor.position, 5)

    def test_step_backward(self):
        """逐步后退"""
        self.motor.enable()
        self.motor.reset_position(5)
        self.motor.move_to(0)
        steps = []
        while self.motor.step():
            steps.append(self.motor.position)
        self.assertEqual(steps, [4, 3, 2, 1, 0])

    def test_step_disabled_no_move(self):
        """未使能时步进返回False"""
        self.motor.move_to(10)
        self.assertFalse(self.motor.step())
        self.assertEqual(self.motor.position, 0)

    def test_step_at_target_no_move(self):
        """到达目标后步进返回False"""
        self.motor.enable()
        self.motor.move_to(0)
        self.assertFalse(self.motor.step())

    def test_run_to_target(self):
        """运行到目标位置"""
        self.motor.enable()
        self.motor.move_to(100)
        count = self.motor.run_to_target()
        self.assertEqual(count, 100)
        self.assertEqual(self.motor.position, 100)
        self.assertTrue(self.motor.is_at_target())

    def test_move_relative(self):
        """相对移动"""
        self.motor.enable()
        self.motor.reset_position(10)
        self.motor.move_relative(20)
        self.assertEqual(self.motor.target, 30)
        self.motor.run_to_target()
        self.assertEqual(self.motor.position, 30)

    def test_stop_at_current(self):
        """停止后目标等于当前位置"""
        self.motor.enable()
        self.motor.move_to(100)
        self.motor.run_to_target()
        self.motor.move_to(0)
        self.motor.step()  # 走一步到position=99
        self.motor.stop()
        self.assertEqual(self.motor.target, self.motor.position)

    def test_reset_position(self):
        """重置位置"""
        self.motor.enable()
        self.motor.move_to(50)
        self.motor.run_to_target()
        self.motor.reset_position(0)
        self.assertEqual(self.motor.position, 0)
        self.assertEqual(self.motor.target, 0)

    def test_is_at_target(self):
        """到达目标判断"""
        self.motor.enable()
        self.motor.move_to(0)
        self.assertTrue(self.motor.is_at_target())
        self.motor.move_to(5)
        self.assertFalse(self.motor.is_at_target())

    def test_large_move(self):
        """大距离移动（1000步）"""
        self.motor.enable()
        self.motor.move_to(1000)
        count = self.motor.run_to_target()
        self.assertEqual(count, 1000)
        self.assertEqual(self.motor.position, 1000)

    def test_bidirectional_move(self):
        """往返运动"""
        self.motor.enable()
        self.motor.move_to(50)
        self.motor.run_to_target()
        self.assertEqual(self.motor.position, 50)
        self.motor.move_to(-30)
        self.motor.run_to_target()
        self.assertEqual(self.motor.position, -30)

    def test_direction_override(self):
        """手动设置方向"""
        self.motor.set_direction(STEPPER_DIR_CCW)
        self.assertEqual(self.motor.direction, STEPPER_DIR_CCW)
        self.motor.set_direction(STEPPER_DIR_CW)
        self.assertEqual(self.motor.direction, STEPPER_DIR_CW)


if __name__ == '__main__':
    unittest.main()
