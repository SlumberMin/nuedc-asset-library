#!/usr/bin/env python3
"""
转台控制 V2 测试 — 基于wrappers.py包装层
覆盖: A4988步进电机 + AS5048A磁编码器 + PID角度控制
模拟场景: 步进电机驱动转台，编码器反馈角度，PID闭环控制
对应C源文件: 02_mspm0g3507/drivers/stepper_a4988.c
              02_mspm0g3507/drivers/as5048a.c
              02_mspm0g3507/drivers/advanced_pid.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    StepperA4988, STEPPER_DIR_CW, STEPPER_DIR_CCW, STEPPER_MAX_SPEED,
    AS5048A, AS5048A_MAX_RAW, AS5048A_DEGREES,
    PIDController,
)


class TestRotaryTableInit(unittest.TestCase):
    """转台初始化测试"""

    def test_stepper_init(self):
        """步进电机初始化成功"""
        motor = StepperA4988()
        self.assertTrue(motor.init())
        self.assertTrue(motor.initialized)
        self.assertEqual(motor.get_position(), 0)

    def test_encoder_init(self):
        """磁编码器初始化成功"""
        encoder = AS5048A()
        self.assertTrue(encoder.init())
        self.assertTrue(encoder.initialized)

    def test_pid_init(self):
        """PID控制器初始化"""
        pid = PIDController(kp=2.0, ki=0.1, kd=0.5)
        self.assertAlmostEqual(pid.kp, 2.0)
        self.assertAlmostEqual(pid.ki, 0.1)
        self.assertAlmostEqual(pid.kd, 0.5)

    def test_full_system_init(self):
        """完整转台系统初始化"""
        motor = StepperA4988()
        encoder = AS5048A()
        pid = PIDController()
        self.assertTrue(motor.init())
        self.assertTrue(encoder.init())
        self.assertTrue(motor.initialized)
        self.assertTrue(encoder.initialized)


class TestStepperMotor(unittest.TestCase):
    """步进电机基本控制测试"""

    def setUp(self):
        self.motor = StepperA4988()
        self.motor.init()
        self.motor.enable()

    def test_move_to_positive(self):
        """正方向移动"""
        self.motor.move_to(100)
        self.assertEqual(self.motor.target, 100)
        self.assertEqual(self.motor.direction, STEPPER_DIR_CW)

    def test_move_to_negative(self):
        """负方向移动"""
        self.motor.move_to(-100)
        self.assertEqual(self.motor.target, -100)
        self.assertEqual(self.motor.direction, STEPPER_DIR_CCW)

    def test_step_execution(self):
        """单步执行"""
        self.motor.move_to(5)
        results = []
        for _ in range(5):
            results.append(self.motor.step())
        # 5步都应成功
        self.assertTrue(all(results))
        self.assertEqual(self.motor.get_position(), 5)

    def test_step_no_move_at_target(self):
        """到达目标后停止步进"""
        self.motor.move_to(3)
        self.motor.run_to_target()
        self.assertFalse(self.motor.step())
        self.assertTrue(self.motor.is_at_target())

    def test_step_disabled_no_move(self):
        """失能状态下不能步进"""
        self.motor.disable()
        self.motor.move_to(10)
        self.assertFalse(self.motor.step())

    def test_run_to_target(self):
        """运行到目标位置"""
        self.motor.move_to(200)
        count = self.motor.run_to_target()
        self.assertEqual(count, 200)
        self.assertEqual(self.motor.get_position(), 200)
        self.assertTrue(self.motor.is_at_target())

    def test_move_relative(self):
        """相对移动"""
        self.motor.move_to(50)
        self.motor.run_to_target()
        self.motor.move_relative(30)
        self.assertEqual(self.motor.target, 80)

    def test_stop(self):
        """停止并更新目标"""
        self.motor.move_to(100)
        # 执行一些步
        for _ in range(30):
            self.motor.step()
        self.motor.stop()
        self.assertEqual(self.motor.target, self.motor.get_position())

    def test_reset_position(self):
        """重置位置"""
        self.motor.move_to(500)
        self.motor.run_to_target()
        self.motor.reset_position(0)
        self.assertEqual(self.motor.get_position(), 0)
        self.assertEqual(self.motor.target, 0)


class TestStepperSpeed(unittest.TestCase):
    """步进电机速度控制测试"""

    def setUp(self):
        self.motor = StepperA4988()
        self.motor.init()

    def test_set_speed_normal(self):
        """正常速度设置"""
        self.motor.set_speed(500)
        self.assertEqual(self.motor.speed, 500)

    def test_set_speed_max(self):
        """最大速度限制"""
        self.motor.set_speed(STEPPER_MAX_SPEED)
        self.assertEqual(self.motor.speed, STEPPER_MAX_SPEED)

    def test_set_speed_over_max(self):
        """超过最大速度被截断"""
        self.motor.set_speed(STEPPER_MAX_SPEED + 500)
        self.assertEqual(self.motor.speed, STEPPER_MAX_SPEED)

    def test_set_speed_negative(self):
        """负速度设为0"""
        self.motor.set_speed(-100)
        self.assertEqual(self.motor.speed, 0)

    def test_set_microstep(self):
        """细分设置"""
        for ms in (1, 2, 4, 8, 16):
            self.assertTrue(self.motor.set_microstep(ms))
            self.assertEqual(self.motor.microstep, ms)

    def test_set_invalid_microstep(self):
        """无效细分失败"""
        self.assertFalse(self.motor.set_microstep(3))
        self.assertFalse(self.motor.set_microstep(0))


class TestAS5048AEncoder(unittest.TestCase):
    """AS5048A磁编码器测试"""

    def setUp(self):
        self.encoder = AS5048A()
        self.encoder.init()

    def test_read_raw(self):
        """读取原始角度值"""
        self.encoder.set_simulated(8192)
        raw = self.encoder.read_raw()
        self.assertEqual(raw, 8192)

    def test_read_angle(self):
        """读取角度值（0-360°）"""
        self.encoder.set_simulated(AS5048A_MAX_RAW // 2)  # 180°附近
        angle = self.encoder.read_angle()
        self.assertAlmostEqual(angle, 180.0, delta=1.0)

    def test_read_angle_zero(self):
        """原始值0对应0°"""
        self.encoder.set_simulated(0)
        self.assertAlmostEqual(self.encoder.read_angle(), 0.0)

    def test_read_angle_full(self):
        """最大原始值对应360°"""
        self.encoder.set_simulated(AS5048A_MAX_RAW)
        angle = self.encoder.read_angle()
        self.assertAlmostEqual(angle, AS5048A_DEGREES, delta=1.0)

    def test_set_zero_offset(self):
        """设置零点偏移"""
        self.encoder.set_simulated(4096)
        self.encoder.set_zero(self.encoder.read_angle())
        # 设置零点后，带偏移读数应为0
        angle = self.encoder.read_angle_with_offset()
        self.assertAlmostEqual(angle, 0.0, delta=1.0)

    def test_agc_and_magnitude(self):
        """读取AGC和磁场强度"""
        self.encoder.set_simulated(8192, agc=128, magnitude=500)
        self.assertEqual(self.encoder.read_agc(), 128)
        self.assertEqual(self.encoder.read_magnitude(), 500)

    def test_not_initialized(self):
        """未初始化返回None"""
        enc2 = AS5048A()
        self.assertIsNone(enc2.read_raw())
        self.assertIsNone(enc2.read_angle())
        self.assertIsNone(enc2.read_agc())


class TestRotaryTablePIDControl(unittest.TestCase):
    """转台PID闭环控制测试"""

    def setUp(self):
        self.motor = StepperA4988()
        self.encoder = AS5048A()
        self.pid = PIDController(kp=2.0, ki=0.05, kd=0.3,
                                 output_min=-500, output_max=500)
        self.motor.init()
        self.motor.enable()
        self.encoder.init()

    def test_pid_converge_to_target(self):
        """PID控制收敛到目标角度"""
        target_angle = 90.0
        # 模拟当前位置对应的编码器原始值
        current_raw = 0
        for _ in range(100):
            # 编码器读数
            self.encoder.set_simulated(current_raw)
            current_angle = self.encoder.read_angle()

            # PID计算
            output = self.pid.calc(target_angle, current_angle)

            # 输出驱动步进电机
            steps = int(abs(output) / 10)
            if output > 0:
                self.motor.move_relative(steps)
            elif output < 0:
                self.motor.move_relative(-steps)
            self.motor.run_to_target()

            # 更新编码器模拟值（每步对应一定角度增量）
            current_raw = int(self.motor.get_position() * AS5048A_MAX_RAW / 2000)
            current_raw = current_raw % (AS5048A_MAX_RAW + 1)

        # 最终角度应接近目标
        final_angle = self.encoder.read_angle()
        self.assertAlmostEqual(final_angle, target_angle, delta=20)

    def test_pid_output_limits(self):
        """PID输出限幅"""
        # 大误差应被限幅
        output = self.pid.calc(360.0, 0.0)
        self.assertLessEqual(output, 500.0)
        self.assertGreaterEqual(output, -500.0)

    def test_pid_reset(self):
        """PID重置"""
        self.pid.calc(100.0, 0.0)
        self.pid.reset()
        self.assertAlmostEqual(self.pid.integral, 0.0)
        self.assertAlmostEqual(self.pid.prev_error, 0.0)

    def test_pid_dead_zone(self):
        """PID死区功能"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, dead_zone=5.0)
        # 误差小于死区时输出应为0
        output = pid.calc(100.0, 98.0)  # 误差=2 < 死区5
        self.assertAlmostEqual(output, 0.0)


if __name__ == '__main__':
    unittest.main()
