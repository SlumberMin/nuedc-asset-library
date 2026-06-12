#!/usr/bin/env python3
"""
平衡控制 V2测试 — 基于wrappers.py包装层
覆盖: PID平衡环、编码器速度环、角度融合、级联控制
对应C源文件: 02_mspm0g3507/examples/balance_car_demo.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    PIDController, KalmanFilter, Encoder,
    ENC_CHANNELS, ENC_PPR_DEFAULT,
)


class BalanceController:
    """平衡车级联控制算法

    与C版本balance_car_demo.c逻辑一致:
    - 外环: 角度PID → 目标速度
    - 内环: 速度PID → 电机PWM
    - 卡尔曼滤波融合角度
    """

    def __init__(self):
        # 角度环 (外环) — 注意: error = target - actual
        # 前倾(actual>0) → error<0 → 需要负kp使PWM为正(前进)
        self.angle_pid = PIDController(kp=-25.0, ki=0.0, kd=-5.0,
                                        output_min=-100, output_max=100)
        # 速度环 (内环)
        self.speed_pid = PIDController(kp=3.0, ki=0.1, kd=0.0,
                                        output_min=-100, output_max=100)
        # 编码器
        self.encoder = Encoder()
        # 状态
        self.angle = 0.0
        self.target_angle = 0.0  # 平衡点
        self.motor_l = 0
        self.motor_r = 0
        self.fallen = False
        self.fall_threshold = 45.0  # 摔倒判定角度

    def init(self):
        """初始化"""
        self.encoder.init()
        self.angle_pid.reset()
        self.speed_pid.reset()
        self.fallen = False
        self.angle = 0.0

    def update(self, acc_angle, gyro_rate, encoder_delta_l, encoder_delta_r, dt=0.01):
        """平衡控制更新

        参数:
            acc_angle: 加速度计计算的倾斜角(度)
            gyro_rate: 陀螺仪角速度(度/秒)
            encoder_delta_l: 左编码器增量
            encoder_delta_r: 右编码器增量
            dt: 控制周期(秒)

        返回: (motor_l, motor_r, angle)
        """
        # 简单互补滤波融合角度
        self.angle = 0.98 * (self.angle + gyro_rate * dt) + 0.02 * acc_angle

        # 摔倒检测
        if abs(self.angle) > self.fall_threshold:
            self.fallen = True
            self.motor_l = 0
            self.motor_r = 0
            return 0, 0, self.angle

        # 外环: 角度PID → 目标速度
        target_speed = self.angle_pid.calc(self.target_angle, self.angle)

        # 内环: 速度PID → PWM
        avg_speed = (encoder_delta_l + encoder_delta_r) / 2.0
        pwm = self.speed_pid.calc(target_speed, avg_speed)

        # 差速(简单直行，不转向)
        self.motor_l = int(pwm)
        self.motor_r = int(pwm)

        return self.motor_l, self.motor_r, self.angle


class TestBalanceInit(unittest.TestCase):
    """初始化测试"""

    def test_init(self):
        """初始化成功"""
        bc = BalanceController()
        bc.init()
        self.assertTrue(bc.encoder.initialized)
        self.assertFalse(bc.fallen)

    def test_default_angle_zero(self):
        """默认目标角度0"""
        bc = BalanceController()
        self.assertAlmostEqual(bc.target_angle, 0.0)


class TestKalmanFusion(unittest.TestCase):
    """卡尔曼滤波器测试"""

    def test_predict(self):
        """卡尔曼预测步"""
        kf = KalmanFilter(dt=0.1, proc_noise=0.01, meas_noise=0.1)
        self.assertAlmostEqual(kf.x[0], 0.0)
        kf.predict()
        # 预测后状态不变(初始速度为0)
        self.assertAlmostEqual(kf.x[0], 0.0, places=3)

    def test_update_1d(self):
        """1D观测更新"""
        kf = KalmanFilter(dt=0.1, proc_noise=0.01, meas_noise=0.1)
        kf.update_1d(5.0)
        # 状态应向测量值靠拢
        self.assertAlmostEqual(kf.x[0], 5.0, delta=1.0)

    def test_stable_convergence(self):
        """持续观测收敛"""
        kf = KalmanFilter(dt=0.1, proc_noise=0.01, meas_noise=0.1)
        for _ in range(100):
            kf.predict()
            kf.update_1d(10.0)
        self.assertAlmostEqual(kf.x[0], 10.0, delta=0.5)

    def test_noise_rejection(self):
        """噪声抑制"""
        import random
        random.seed(42)
        kf = KalmanFilter(dt=0.1, proc_noise=0.01, meas_noise=1.0)
        true_val = 10.0
        for _ in range(200):
            kf.predict()
            noisy = true_val + random.gauss(0, 2.0)
            kf.update_1d(noisy)
        self.assertAlmostEqual(kf.x[0], true_val, delta=1.0)


class TestEncoderSpeed(unittest.TestCase):
    """编码器速度测试"""

    def test_speed_calculation(self):
        """编码器速度计算"""
        enc = Encoder()
        enc.init()
        enc.inject_pulse(0, 100)
        enc.sample_callback()
        self.assertEqual(enc.get_speed(0), 100)

    def test_speed_rpm(self):
        """RPM计算"""
        enc = Encoder()
        enc.init()
        enc.ppr = 13
        enc.inject_pulse(0, 13)  # 1圈 = 13脉冲
        enc.sample_callback()
        rpm = enc.get_speed_rpm(0)
        self.assertGreater(rpm, 0)

    def test_reset(self):
        """编码器重置"""
        enc = Encoder()
        enc.init()
        enc.inject_pulse(0, 500)
        enc.reset(0)
        self.assertEqual(enc.get_count(0), 0)
        self.assertEqual(enc.get_speed(0), 0)

    def test_multi_channel(self):
        """多通道编码器"""
        enc = Encoder()
        enc.init()
        enc.inject_pulse(0, 100)
        enc.inject_pulse(1, -100)
        enc.sample_callback()
        self.assertEqual(enc.get_speed(0), 100)
        self.assertEqual(enc.get_speed(1), -100)


class TestAnglePID(unittest.TestCase):
    """角度PID测试"""

    def test_positive_tilt_forward(self):
        """前倾(+5°) → 正PWM(前进修正)"""
        pid = PIDController(kp=-25.0, ki=0.0, kd=0.0,
                            output_min=-100, output_max=100)
        # error = 0 - 5 = -5, output = -25 * -5 = 125 → clamped to 100
        pwm = pid.calc(0, 5.0)
        self.assertGreater(pwm, 0)

    def test_negative_tilt_backward(self):
        """后倾(-5°) → 负PWM(后退修正)"""
        pid = PIDController(kp=-25.0, ki=0.0, kd=0.0,
                            output_min=-100, output_max=100)
        # error = 0 - (-5) = 5, output = -25 * 5 = -125 → clamped to -100
        pwm = pid.calc(0, -5.0)
        self.assertLess(pwm, 0)

    def test_balanced_zero(self):
        """平衡状态 → PWM接近0"""
        pid = PIDController(kp=-1.0, ki=0.0, kd=0.0,
                            output_min=-100, output_max=100)
        pwm = pid.calc(0, 0.0)
        self.assertAlmostEqual(pwm, 0.0, places=1)


class TestBalanceControl(unittest.TestCase):
    """级联平衡控制测试"""

    def test_upright(self):
        """直立状态 → PWM接近0"""
        bc = BalanceController()
        bc.init()
        motor_l, motor_r, angle = bc.update(
            acc_angle=0.0, gyro_rate=0.0,
            encoder_delta_l=0, encoder_delta_r=0
        )
        self.assertAlmostEqual(angle, 0.0, delta=1.0)

    def test_tilt_forward(self):
        """前倾 → 正PWM"""
        bc = BalanceController()
        bc.init()
        motor_l, motor_r, angle = bc.update(
            acc_angle=5.0, gyro_rate=0.0,
            encoder_delta_l=0, encoder_delta_r=0
        )
        self.assertGreater(motor_l, 0)
        self.assertGreater(motor_r, 0)

    def test_tilt_backward(self):
        """后倾 → 负PWM"""
        bc = BalanceController()
        bc.init()
        motor_l, motor_r, angle = bc.update(
            acc_angle=-5.0, gyro_rate=0.0,
            encoder_delta_l=0, encoder_delta_r=0
        )
        self.assertLess(motor_l, 0)
        self.assertLess(motor_r, 0)

    def test_fall_detection(self):
        """摔倒检测: 超过阈值 → 停机"""
        bc = BalanceController()
        bc.init()
        # 互补滤波收敛慢，直接注入大角度
        bc.angle = 50.0
        motor_l, motor_r, angle = bc.update(
            acc_angle=50.0, gyro_rate=0.0,
            encoder_delta_l=0, encoder_delta_r=0
        )
        self.assertTrue(bc.fallen)
        self.assertEqual(motor_l, 0)
        self.assertEqual(motor_r, 0)

    def test_fall_recovery(self):
        """摔倒后重新初始化恢复"""
        bc = BalanceController()
        bc.init()
        bc.angle = 60.0
        bc.update(acc_angle=60.0, gyro_rate=0.0,
                  encoder_delta_l=0, encoder_delta_r=0)
        self.assertTrue(bc.fallen)
        bc.init()
        self.assertFalse(bc.fallen)

    def test_motor_symmetry(self):
        """左右电机对称"""
        bc = BalanceController()
        bc.init()
        motor_l, motor_r, _ = bc.update(
            acc_angle=3.0, gyro_rate=0.0,
            encoder_delta_l=5, encoder_delta_r=5
        )
        self.assertEqual(motor_l, motor_r)

    def test_encoder_feedback(self):
        """编码器反馈影响PWM"""
        bc = BalanceController()
        bc.init()
        # 前倾+编码器已经前进(反馈速度) → PWM应减小
        m1, _, _ = bc.update(5.0, 0.0, 0, 0)
        bc.init()
        m2, _, _ = bc.update(5.0, 0.0, 50, 50)
        # 有速度反馈时PWM应不同
        self.assertNotEqual(m1, m2)


if __name__ == '__main__':
    unittest.main()
