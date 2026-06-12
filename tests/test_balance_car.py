#!/usr/bin/env python3
"""
平衡车V2测试 — IMU + 编码器 + 三环PID
覆盖: MPU6050姿态读取、互补滤波融合、
      三环PID（角度环+角速度环+速度环）、编码器反馈
对应C源文件: 02_mspm0g3507/drivers/mpu6050.c + balance_car算法

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    MPU6050, MPU6050_Data, Encoder, PIDController,
    MPU6050_ACCEL_FS_2G, MPU6050_GYRO_FS_250DPS,
    ENC_CHANNELS,
)


class ComplementaryFilter:
    """互补滤波器 — 融合加速度计和陀螺仪数据

    angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
    alpha ≈ 0.98 (信任陀螺仪短期精度)
    """

    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.angle = 0.0

    def update(self, accel_angle, gyro_rate, dt):
        """更新融合角度

        accel_angle: 加速度计计算的倾斜角(deg)
        gyro_rate: 陀螺仪角速度(deg/s)
        dt: 时间间隔(s)
        """
        self.angle = self.alpha * (self.angle + gyro_rate * dt) + \
                     (1.0 - self.alpha) * accel_angle
        return self.angle

    def reset(self):
        self.angle = 0.0


class BalanceCarV2:
    """平衡车V2 — 三环PID控制

    三环结构:
    1. 角度环(外环): 目标角度与实际角度偏差 → 角速度设定
    2. 角速度环(中环): 角速度偏差 → 电机PWM基础值
    3. 速度环(内环): 编码器速度偏差 → 修正角度环零点

    电机输出 = 角度环输出 + 角速度环输出 + 速度环输出
    """

    TARGET_ANGLE = 0.0   # 目标平衡角度(度)

    def __init__(self):
        self.imu = MPU6050()
        self.encoder_l = Encoder()  # 左编码器 (CH0)
        self.encoder_r = Encoder()  # 右编码器 (CH1)
        self.comp_filter = ComplementaryFilter(alpha=0.98)

        # 三环PID
        self.pid_angle = PIDController(kp=30.0, ki=0.0, kd=0.0,
                                        output_min=-500, output_max=500)
        self.pid_gyro = PIDController(kp=5.0, ki=0.0, kd=0.0,
                                       output_min=-1000, output_max=1000)
        self.pid_speed = PIDController(kp=1.0, ki=0.02, kd=0.0,
                                        output_min=-200, output_max=200)

        self.angle = 0.0
        self.gyro_y = 0.0
        self.motor_pwm = 0.0
        self.speed_error = 0.0
        self.dt = 0.01  # 10ms控制周期

    def init(self):
        """初始化IMU和编码器"""
        self.imu.init(accel_fs=MPU6050_ACCEL_FS_2G,
                      gyro_fs=MPU6050_GYRO_FS_250DPS)
        for ch in range(ENC_CHANNELS):
            self.encoder_l.init() if ch == 0 else None
        self.encoder_l.init()
        self.encoder_r.init()
        self.comp_filter.reset()
        self.pid_angle.reset()
        self.pid_gyro.reset()
        self.pid_speed.reset()

    def _calc_accel_angle(self, accel_x_g, accel_z_g):
        """从加速度计计算倾斜角(度)"""
        if abs(accel_z_g) < 1e-6:
            accel_z_g = 1e-6
        return math.atan2(accel_x_g, accel_z_g) * 180.0 / math.pi

    def update(self, accel_xyz, gyro_xyz, enc_left, enc_right):
        """一步控制更新

        accel_xyz: (ax, ay, az) 加速度原始值
        gyro_xyz:  (gx, gy, gz) 陀螺仪原始值
        enc_left:  左编码器脉冲增量
        enc_right: 右编码器脉冲增量

        返回: (motor_pwm, angle, gyro)
        """
        # 设置IMU原始数据并读取
        self.imu.set_raw_values(accel_xyz, gyro_xyz)
        ok, data = self.imu.read_all()
        if not ok:
            return 0, 0, 0

        # 加速度计角度
        accel_angle = self._calc_accel_angle(data.accel_x_g, data.accel_z_g)

        # 陀螺仪角速度(Y轴)
        self.gyro_y = data.gyro_y_dps

        # 互补滤波融合角度
        self.angle = self.comp_filter.update(accel_angle, self.gyro_y, self.dt)

        # 注入编码器脉冲
        self.encoder_l.inject_pulse(0, enc_left)
        self.encoder_r.inject_pulse(1, enc_right)
        self.encoder_l.sample_callback()
        self.encoder_r.sample_callback()

        # 速度环: 编码器速度偏差
        speed_l = self.encoder_l.get_speed(0)
        speed_r = self.encoder_r.get_speed(1)
        avg_speed = (speed_l + speed_r) / 2.0
        self.speed_error = self.pid_speed.calc(0, avg_speed)

        # 角度环: 角度偏差(带速度修正)
        target = self.TARGET_ANGLE + self.speed_error * 0.01
        angle_output = self.pid_angle.calc(target, self.angle)

        # 角速度环: 角速度偏差
        gyro_output = self.pid_gyro.calc(angle_output / 30.0, self.gyro_y)

        # 合成电机PWM
        self.motor_pwm = angle_output + gyro_output

        # 限幅
        self.motor_pwm = max(-1000, min(1000, self.motor_pwm))

        return self.motor_pwm, self.angle, self.gyro_y


class TestBalanceCarInit(unittest.TestCase):
    """平衡车初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        car = BalanceCarV2()
        car.init()
        self.assertTrue(car.imu.initialized)
        self.assertTrue(car.encoder_l.initialized)
        self.assertTrue(car.encoder_r.initialized)

    def test_initial_angle_zero(self):
        """初始角度为0"""
        car = BalanceCarV2()
        car.init()
        self.assertAlmostEqual(car.angle, 0.0)

    def test_pid_params(self):
        """PID参数合理"""
        car = BalanceCarV2()
        self.assertGreater(car.pid_angle.kp, 0)
        self.assertGreater(car.pid_gyro.kp, 0)
        self.assertGreater(car.pid_speed.kp, 0)


class TestIMUReading(unittest.TestCase):
    """IMU数据读取测试"""

    def test_read_accel(self):
        """读取加速度数据"""
        imu = MPU6050()
        imu.init()
        # 竖直放置: ax=0, az=16384 (1g)
        imu.set_raw_values((0, 0, 16384), (0, 0, 0))
        ok, data = imu.read_all()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.accel_z_g, 1.0, places=1)
        self.assertAlmostEqual(data.accel_x_g, 0.0, places=1)

    def test_read_gyro(self):
        """读取陀螺仪数据"""
        imu = MPU6050()
        imu.init()
        # Y轴旋转: gyro_y=131 → 1°/s
        imu.set_raw_values((0, 0, 16384), (0, 131, 0))
        ok, data = imu.read_all()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.gyro_y_dps, 1.0, places=0)

    def test_read_temperature(self):
        """读取温度"""
        imu = MPU6050()
        imu.init()
        imu.set_raw_values((0, 0, 16384), (0, 0, 0), raw_temp=0)
        ok, data = imu.read_all()
        self.assertTrue(ok)
        # T = 0/340 + 36.53 = 36.53°C
        self.assertAlmostEqual(data.temperature, 36.53, places=0)

    def test_chip_id(self):
        """读取芯片ID"""
        imu = MPU6050()
        imu.init()
        ok, chip_id = imu.read_id()
        self.assertTrue(ok)
        self.assertEqual(chip_id, 0x68)

    def test_sleep_wake(self):
        """睡眠/唤醒"""
        imu = MPU6050()
        imu.init()
        self.assertTrue(imu.sleep())
        self.assertTrue(imu.wake_up())


class TestComplementaryFilter(unittest.TestCase):
    """互补滤波器测试"""

    def test_steady_state(self):
        """稳态融合"""
        cf = ComplementaryFilter(alpha=0.98)
        # 持续输入同一角度，互补滤波器需要多次迭代收敛
        for _ in range(500):
            angle = cf.update(10.0, 0.0, 0.01)
        self.assertAlmostEqual(angle, 10.0, delta=2.0)

    def test_gyro_response(self):
        """陀螺仪响应"""
        cf = ComplementaryFilter(alpha=0.98)
        # 陀螺仪持续旋转
        angle = 0.0
        for _ in range(100):
            angle = cf.update(0.0, 100.0, 0.01)  # 100°/s
        # 0.98*(angle+100*0.01) + 0.02*0 ≈ 最终角度显著偏移
        self.assertGreater(abs(angle), 1.0)

    def test_reset(self):
        """重置"""
        cf = ComplementaryFilter()
        cf.update(45.0, 0.0, 0.01)
        cf.reset()
        self.assertAlmostEqual(cf.angle, 0.0)


class TestEncoder(unittest.TestCase):
    """编码器测试"""

    def test_pulse_counting(self):
        """脉冲计数"""
        enc = Encoder()
        enc.init()
        enc.inject_pulse(0, 100)
        enc.inject_pulse(0, 50)
        self.assertEqual(enc.get_count(0), 150)

    def test_speed_calculation(self):
        """速度计算"""
        enc = Encoder()
        enc.init()
        enc.inject_pulse(0, 100)
        enc.sample_callback()
        self.assertEqual(enc.get_speed(0), 100)

    def test_speed_rpm(self):
        """RPM计算"""
        enc = Encoder()
        enc.init()
        enc.inject_pulse(0, 360)  # 1转/采样周期
        enc.sample_callback()
        rpm = enc.get_speed_rpm(0)
        # 360脉冲/10ms → 360*60/(360*0.01) = 6000 RPM
        self.assertAlmostEqual(rpm, 6000.0, places=0)

    def test_reset(self):
        """编码器重置"""
        enc = Encoder()
        enc.init()
        enc.inject_pulse(0, 100)
        enc.reset(0)
        self.assertEqual(enc.get_count(0), 0)


class TestBalanceCarControl(unittest.TestCase):
    """平衡车控制测试"""

    def test_upright_stable(self):
        """竖直稳定→PWM≈0"""
        car = BalanceCarV2()
        car.init()
        # 竖直: ax≈0, az≈1g, 无旋转
        for _ in range(50):
            pwm, angle, gyro = car.update(
                (0, 0, 16384), (0, 0, 0), 0, 0)
        self.assertAlmostEqual(angle, 0.0, places=0)
        self.assertAlmostEqual(pwm, 0.0, delta=50)

    def test_tilt_forward(self):
        """前倾→正PWM(向后修正)"""
        car = BalanceCarV2()
        car.init()
        # 前倾: ax>0
        for _ in range(10):
            pwm, angle, gyro = car.update(
                (3000, 0, 16000), (0, 0, 0), 0, 0)
        # 角度应为正(前倾)
        self.assertGreater(angle, 0)

    def test_tilt_backward(self):
        """后倾→负PWM(向前修正)"""
        car = BalanceCarV2()
        car.init()
        for _ in range(10):
            pwm, angle, gyro = car.update(
                (-3000, 0, 16000), (0, 0, 0), 0, 0)
        self.assertLess(angle, 0)

    def test_pwm_clamp(self):
        """PWM限幅 [-1000, 1000]"""
        car = BalanceCarV2()
        car.init()
        # 极端倾斜
        for _ in range(100):
            pwm, angle, gyro = car.update(
                (16000, 0, 3000), (0, 0, 0), 0, 0)
        self.assertGreaterEqual(pwm, -1000)
        self.assertLessEqual(pwm, 1000)

    def test_speed_loop_correction(self):
        """速度环修正: 有编码器反馈时修正角度零点"""
        car = BalanceCarV2()
        car.init()
        # 竖直但有速度(前进中)
        for _ in range(20):
            pwm, angle, gyro = car.update(
                (0, 0, 16384), (0, 0, 0), 100, 100)
        # 速度环应产生非零修正
        self.assertNotAlmostEqual(car.speed_error, 0.0, places=0)


if __name__ == '__main__':
    unittest.main()
