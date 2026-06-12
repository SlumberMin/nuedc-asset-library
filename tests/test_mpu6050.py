#!/usr/bin/env python3
"""
MPU6050 六轴IMU V3 测试 — 姿态解算+校准深度测试
覆盖: V2全部 + 加速度/陀螺仪量程切换、物理单位转换精度、
      重力方向检测、姿态角估算（俯仰/横滚）、零偏校准模拟、
      温度测量、睡眠/唤醒、芯片ID验证
对应C源文件: 02_mspm0g3507/drivers/mpu6050.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #6:  I2C忙等待/超时
  #13: MPU6050是姿态解算核心传感器，精度直接影响控制效果
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    MPU6050, MPU6050_Data,
    MPU6050_I2C_ADDR, MPU6050_I2C_ADDR_LOW, MPU6050_I2C_ADDR_HIGH,
    MPU6050_WHO_AM_I_VALUE,
    MPU6050_ACCEL_FS_2G, MPU6050_ACCEL_FS_4G, MPU6050_ACCEL_FS_8G, MPU6050_ACCEL_FS_16G,
    MPU6050_GYRO_FS_250DPS, MPU6050_GYRO_FS_500DPS, MPU6050_GYRO_FS_1000DPS, MPU6050_GYRO_FS_2000DPS,
)


def estimate_pitch_roll(ax, ay, az):
    """
    从加速度估算俯仰角(pitch)和横滚角(roll)
    输入: ax, ay, az 单位g
    返回: (pitch_deg, roll_deg)
    """
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi
    roll = math.atan2(ay, az) * 180.0 / math.pi
    return pitch, roll


class TestMPU6050V3Init(unittest.TestCase):
    """MPU6050 V3 初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        mpu = MPU6050()
        ok = mpu.init()
        self.assertTrue(ok)
        self.assertTrue(mpu.initialized)

    def test_default_addr(self):
        """默认I2C地址0x68"""
        mpu = MPU6050()
        self.assertEqual(mpu.addr, MPU6050_I2C_ADDR)
        self.assertEqual(mpu.addr, 0x68)

    def test_addr_variants(self):
        """地址变体"""
        self.assertEqual(MPU6050_I2C_ADDR_LOW, 0x68)
        self.assertEqual(MPU6050_I2C_ADDR_HIGH, 0x69)

    def test_default_fs(self):
        """默认量程: ±2g, ±250dps"""
        mpu = MPU6050()
        mpu.init()
        self.assertEqual(mpu._accel_fs, MPU6050_ACCEL_FS_2G)
        self.assertEqual(mpu._gyro_fs, MPU6050_GYRO_FS_250DPS)

    def test_not_sleeping_after_init(self):
        """初始化后不在睡眠模式"""
        mpu = MPU6050()
        mpu.init()
        self.assertFalse(mpu._sleeping)

    def test_read_id(self):
        """芯片ID应为0x68"""
        mpu = MPU6050()
        mpu.init()
        ok, chip_id = mpu.read_id()
        self.assertTrue(ok)
        self.assertEqual(chip_id, MPU6050_WHO_AM_I_VALUE)
        self.assertEqual(chip_id, 0x68)


class TestMPU6050V3ReadAll(unittest.TestCase):
    """MPU6050 V3 数据读取测试"""

    def test_read_not_init(self):
        """未初始化读取应失败"""
        mpu = MPU6050()
        ok, data = mpu.read_all()
        self.assertFalse(ok)
        self.assertIsNone(data)

    def test_read_default_zero(self):
        """默认原始值全零"""
        mpu = MPU6050()
        mpu.init()
        ok, data = mpu.read_all()
        self.assertTrue(ok)
        self.assertEqual(data.accel_x_raw, 0)
        self.assertEqual(data.accel_y_raw, 0)
        self.assertEqual(data.accel_z_raw, 0)
        self.assertEqual(data.gyro_x_raw, 0)
        self.assertEqual(data.gyro_y_raw, 0)
        self.assertEqual(data.gyro_z_raw, 0)

    def test_set_and_read_raw(self):
        """设置并读取原始值"""
        mpu = MPU6050()
        mpu.init()
        mpu.set_raw_values((1000, 2000, 3000), (100, 200, 300), 500)
        ok, data = mpu.read_all()
        self.assertTrue(ok)
        self.assertEqual(data.accel_x_raw, 1000)
        self.assertEqual(data.accel_y_raw, 2000)
        self.assertEqual(data.accel_z_raw, 3000)
        self.assertEqual(data.gyro_x_raw, 100)
        self.assertEqual(data.gyro_y_raw, 200)
        self.assertEqual(data.gyro_z_raw, 300)

    def test_read_temperature(self):
        """温度读取"""
        mpu = MPU6050()
        mpu.init()
        mpu.set_raw_values((0, 0, 0), (0, 0, 0), 0)
        ok, temp = mpu.read_temperature()
        self.assertTrue(ok)
        # T = 0/340 + 36.53 = 36.53
        self.assertAlmostEqual(temp, 36.53, places=2)

    def test_read_temperature_not_init(self):
        """未初始化读温度应失败"""
        mpu = MPU6050()
        ok, temp = mpu.read_temperature()
        self.assertFalse(ok)


class TestMPU6050V3AccelConversion(unittest.TestCase):
    """MPU6050 V3 加速度物理单位转换"""

    def test_accel_2g_sensitivity(self):
        """±2g量程: 灵敏度16384 LSB/g"""
        mpu = MPU6050()
        mpu.init(accel_fs=MPU6050_ACCEL_FS_2G)
        # 模拟1g: raw=16384
        mpu.set_raw_values((0, 0, 16384), (0, 0, 0))
        ok, data = mpu.read_all()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.accel_z_g, 1.0, places=2)

    def test_accel_4g_sensitivity(self):
        """±4g量程: 灵敏度8192 LSB/g"""
        mpu = MPU6050()
        mpu.init(accel_fs=MPU6050_ACCEL_FS_4G)
        mpu.set_raw_values((0, 0, 8192), (0, 0, 0))
        ok, data = mpu.read_all()
        self.assertAlmostEqual(data.accel_z_g, 1.0, places=2)

    def test_accel_8g_sensitivity(self):
        """±8g量程: 灵敏度4096 LSB/g"""
        mpu = MPU6050()
        mpu.init(accel_fs=MPU6050_ACCEL_FS_8G)
        mpu.set_raw_values((0, 0, 4096), (0, 0, 0))
        ok, data = mpu.read_all()
        self.assertAlmostEqual(data.accel_z_g, 1.0, places=2)

    def test_accel_16g_sensitivity(self):
        """±16g量程: 灵敏度2048 LSB/g"""
        mpu = MPU6050()
        mpu.init(accel_fs=MPU6050_ACCEL_FS_16G)
        mpu.set_raw_values((0, 0, 2048), (0, 0, 0))
        ok, data = mpu.read_all()
        self.assertAlmostEqual(data.accel_z_g, 1.0, places=2)

    def test_accel_negative(self):
        """负加速度"""
        mpu = MPU6050()
        mpu.init()
        mpu.set_raw_values((0, 0, -16384), (0, 0, 0))
        ok, data = mpu.read_all()
        self.assertAlmostEqual(data.accel_z_g, -1.0, places=2)


class TestMPU6050V3GyroConversion(unittest.TestCase):
    """MPU6050 V3 陀螺仪物理单位转换"""

    def test_gyro_250dps_sensitivity(self):
        """±250dps量程: 灵敏度131 LSB/(°/s)"""
        mpu = MPU6050()
        mpu.init(gyro_fs=MPU6050_GYRO_FS_250DPS)
        mpu.set_raw_values((0, 0, 0), (0, 0, 131))
        ok, data = mpu.read_all()
        self.assertAlmostEqual(data.gyro_z_dps, 1.0, places=2)

    def test_gyro_500dps_sensitivity(self):
        """±500dps量程: 灵敏度65.5 LSB/(°/s)"""
        mpu = MPU6050()
        mpu.init(gyro_fs=MPU6050_GYRO_FS_500DPS)
        mpu.set_raw_values((0, 0, 0), (0, 0, 66))
        ok, data = mpu.read_all()
        self.assertAlmostEqual(data.gyro_z_dps, 66.0 / 65.5, places=1)

    def test_gyro_1000dps_sensitivity(self):
        """±1000dps量程: 灵敏度32.8 LSB/(°/s)"""
        mpu = MPU6050()
        mpu.init(gyro_fs=MPU6050_GYRO_FS_1000DPS)
        mpu.set_raw_values((0, 0, 0), (32, 0, 0))
        ok, data = mpu.read_all()
        self.assertAlmostEqual(data.gyro_x_dps, 32.0 / 32.8, places=1)

    def test_gyro_2000dps_sensitivity(self):
        """±2000dps量程: 灵敏度16.4 LSB/(°/s)"""
        mpu = MPU6050()
        mpu.init(gyro_fs=MPU6050_GYRO_FS_2000DPS)
        mpu.set_raw_values((0, 0, 0), (0, 0, 16))
        ok, data = mpu.read_all()
        self.assertAlmostEqual(data.gyro_z_dps, 16.0 / 16.4, places=1)


class TestMPU6050V3Attitude(unittest.TestCase):
    """MPU6050 V3 姿态解算测试"""

    def test_pitch_zero_when_level(self):
        """水平放置: 俯仰角≈0°"""
        mpu = MPU6050()
        mpu.init()
        # 水平: ax=0, ay=0, az=1g
        mpu.set_raw_values((0, 0, 16384), (0, 0, 0))
        ok, data = mpu.read_all()
        self.assertTrue(ok)
        pitch, roll = estimate_pitch_roll(data.accel_x_g, data.accel_y_g, data.accel_z_g)
        self.assertAlmostEqual(pitch, 0.0, places=1)
        self.assertAlmostEqual(roll, 0.0, places=1)

    def test_pitch_positive_when_tilted_forward(self):
        """前倾: 俯仰角>0"""
        mpu = MPU6050()
        mpu.init()
        # 前倾45°: ax≈-0.707g, az≈0.707g
        ax_raw = int(-0.707 * 16384)
        az_raw = int(0.707 * 16384)
        mpu.set_raw_values((ax_raw, 0, az_raw), (0, 0, 0))
        ok, data = mpu.read_all()
        pitch, roll = estimate_pitch_roll(data.accel_x_g, data.accel_y_g, data.accel_z_g)
        self.assertGreater(pitch, 30)

    def test_roll_positive_when_tilted_right(self):
        """右倾: 横滚角>0"""
        mpu = MPU6050()
        mpu.init()
        ay_raw = int(0.707 * 16384)
        az_raw = int(0.707 * 16384)
        mpu.set_raw_values((0, ay_raw, az_raw), (0, 0, 0))
        ok, data = mpu.read_all()
        pitch, roll = estimate_pitch_roll(data.accel_x_g, data.accel_y_g, data.accel_z_g)
        self.assertGreater(roll, 30)

    def test_inverted_down(self):
        """倒置（z轴朝下）: az≈-1g"""
        mpu = MPU6050()
        mpu.init()
        mpu.set_raw_values((0, 0, -16384), (0, 0, 0))
        ok, data = mpu.read_all()
        self.assertAlmostEqual(data.accel_z_g, -1.0, places=2)


class TestMPU6050V3Calibration(unittest.TestCase):
    """MPU6050 V3 校准模拟测试"""

    def test_zero_offset_calibration(self):
        """零偏校准: 减去偏移量后应为零"""
        mpu = MPU6050()
        mpu.init()
        # 模拟静止时有偏移
        offset_ax, offset_ay, offset_az = 100, -50, 16484  # 16484≈1g+100偏移
        mpu.set_raw_values((offset_ax, offset_ay, offset_az), (0, 0, 0))
        ok, data = mpu.read_all()
        self.assertTrue(ok)
        # 校准: 减去偏移
        cal_ax = data.accel_x_raw - offset_ax
        cal_ay = data.accel_y_raw - offset_ay
        cal_az = data.accel_z_raw - (offset_az - 16384)  # 减去偏移，保留1g
        self.assertEqual(cal_ax, 0)
        self.assertEqual(cal_ay, 0)

    def test_gyro_zero_calibration(self):
        """陀螺仪零偏校准"""
        mpu = MPU6050()
        mpu.init()
        # 静止时陀螺仪有小偏移
        offset_gx, offset_gy, offset_gz = 5, -3, 8
        mpu.set_raw_values((0, 0, 16384), (offset_gx, offset_gy, offset_gz))
        ok, data = mpu.read_all()
        # 校准后应为零
        cal_gx = data.gyro_x_raw - offset_gx
        cal_gy = data.gyro_y_raw - offset_gy
        cal_gz = data.gyro_z_raw - offset_gz
        self.assertEqual(cal_gx, 0)
        self.assertEqual(cal_gy, 0)
        self.assertEqual(cal_gz, 0)


class TestMPU6050V3Sleep(unittest.TestCase):
    """MPU6050 V3 睡眠/唤醒测试"""

    def test_sleep_success(self):
        """睡眠模式"""
        mpu = MPU6050()
        mpu.init()
        ok = mpu.sleep()
        self.assertTrue(ok)
        self.assertTrue(mpu._sleeping)

    def test_wake_up_success(self):
        """唤醒"""
        mpu = MPU6050()
        mpu.init()
        mpu.sleep()
        ok = mpu.wake_up()
        self.assertTrue(ok)
        self.assertFalse(mpu._sleeping)

    def test_sleep_not_init(self):
        """未初始化睡眠应失败"""
        mpu = MPU6050()
        ok = mpu.sleep()
        self.assertFalse(ok)

    def test_wake_up_not_init(self):
        """未初始化唤醒应失败"""
        mpu = MPU6050()
        ok = mpu.wake_up()
        self.assertFalse(ok)

    def test_sleep_wake_cycle(self):
        """睡眠/唤醒多次循环"""
        mpu = MPU6050()
        mpu.init()
        for _ in range(10):
            mpu.sleep()
            self.assertTrue(mpu._sleeping)
            mpu.wake_up()
            self.assertFalse(mpu._sleeping)


class TestMPU6050V3DataStructure(unittest.TestCase):
    """MPU6050 V3 数据结构测试"""

    def test_data_defaults(self):
        """默认数据全零"""
        data = MPU6050_Data()
        self.assertEqual(data.accel_x_raw, 0)
        self.assertEqual(data.accel_y_g, 0.0)
        self.assertEqual(data.gyro_x_dps, 0.0)
        self.assertEqual(data.temperature, 0.0)


class TestMPU6050V3FullWorkflow(unittest.TestCase):
    """MPU6050 V3 完整工作流程"""

    def test_init_calibrate_read_attitude(self):
        """初始化→校准→读取→姿态解算流程"""
        mpu = MPU6050()
        # 1. 初始化
        self.assertTrue(mpu.init())
        # 2. 模拟水平静止（含偏移）
        mpu.set_raw_values((10, -20, 16400), (3, -2, 1))
        ok, data = mpu.read_all()
        self.assertTrue(ok)
        # 3. 校准后计算姿态
        cal_ax = (data.accel_x_raw - 10) / 16384.0
        cal_ay = (data.accel_y_raw + 20) / 16384.0
        cal_az = (data.accel_z_raw - 16) / 16384.0  # 16400-16384=16偏移
        pitch, roll = estimate_pitch_roll(cal_ax, cal_ay, cal_az)
        # 水平应接近0°
        self.assertAlmostEqual(pitch, 0.0, places=0)
        self.assertAlmostEqual(roll, 0.0, places=0)


if __name__ == '__main__':
    unittest.main()
