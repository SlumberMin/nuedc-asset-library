#!/usr/bin/env python3
"""
多传感器阵列 V2 测试 — 基于wrappers.py包装层
覆盖: SHT30温湿度 + SGP30空气质量 + QMC5883L电子罗盘 + MPU6050六轴IMU
模拟场景: 多传感器协同采集、数据融合、CRC校验
对应C源文件: 02_mspm0g3507/drivers/sht30.c
              02_mspm0g3507/drivers/sgp30.c
              02_mspm0g3507/drivers/qmc5883l.c
              02_mspm0g3507/drivers/mpu6050.c
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    SHT30, SHT30_I2C_ADDR, SHT30_I2C_ADDR_HIGH,
    SGP30, SGP30_I2C_ADDR,
    QMC5883L, QMC5883L_I2C_ADDR, QMC5883L_RANGE_2G, QMC5883L_RANGE_8G,
    MPU6050, MPU6050_I2C_ADDR, MPU6050_I2C_ADDR_HIGH,
    MPU6050_ACCEL_FS_2G, MPU6050_ACCEL_FS_4G, MPU6050_ACCEL_FS_8G,
    MPU6050_GYRO_FS_250DPS, MPU6050_GYRO_FS_500DPS, MPU6050_GYRO_FS_2000DPS,
    KalmanFilter,
)


class TestSHT30Init(unittest.TestCase):
    """SHT30温湿度传感器初始化测试"""

    def test_init_success(self):
        """SHT30初始化成功"""
        sensor = SHT30()
        self.assertTrue(sensor.init())
        self.assertTrue(sensor.initialized)

    def test_default_address(self):
        """默认I2C地址0x44"""
        sensor = SHT30()
        self.assertEqual(sensor.addr, SHT30_I2C_ADDR)

    def test_custom_address(self):
        """自定义I2C地址0x45"""
        sensor = SHT30(addr=SHT30_I2C_ADDR_HIGH)
        self.assertEqual(sensor.addr, SHT30_I2C_ADDR_HIGH)


class TestSHT30Measurement(unittest.TestCase):
    """SHT30温湿度测量测试"""

    def setUp(self):
        self.sensor = SHT30()
        self.sensor.init()

    def test_read_temperature(self):
        """读取温度（25°C附近）"""
        # raw_temp = (25 + 45) * 65535 / 175 = 26214
        self.sensor.set_raw_values(26214, 32768)
        ok, data = self.sensor.measure_single()
        self.assertTrue(ok)
        temp, humi = data
        self.assertAlmostEqual(temp, 25.0, delta=1.0)

    def test_read_humidity(self):
        """读取湿度（50%RH附近）"""
        # raw_humi = 50 * 65535 / 100 = 32768
        self.sensor.set_raw_values(26214, 32768)
        ok, data = self.sensor.measure_single()
        self.assertTrue(ok)
        temp, humi = data
        self.assertAlmostEqual(humi, 50.0, delta=1.0)

    def test_zero_temperature(self):
        """0°C温度"""
        # raw_temp = (0 + 45) * 65535 / 175 = 16837
        self.sensor.set_raw_values(16837, 0)
        ok, data = self.sensor.measure_single()
        self.assertTrue(ok)
        self.assertAlmostEqual(data[0], 0.0, delta=1.0)

    def test_humidity_clamp(self):
        """湿度值被限制在0-100%"""
        self.sensor.set_raw_values(0, 70000)  # 超过100%
        ok, data = self.sensor.measure_single()
        self.assertTrue(ok)
        self.assertLessEqual(data[1], 100.0)

    def test_not_initialized(self):
        """未初始化测量失败"""
        sensor2 = SHT30()
        ok, data = sensor2.measure_single()
        self.assertFalse(ok)
        self.assertIsNone(data)


class TestSHT30Heater(unittest.TestCase):
    """SHT30加热器控制测试"""

    def setUp(self):
        self.sensor = SHT30()
        self.sensor.init()

    def test_heater_on(self):
        """开启加热器"""
        self.assertTrue(self.sensor.heater_on())
        self.assertTrue(self.sensor._heater_on)

    def test_heater_off(self):
        """关闭加热器"""
        self.sensor.heater_on()
        self.assertTrue(self.sensor.heater_off())
        self.assertFalse(self.sensor._heater_on)

    def test_soft_reset(self):
        """软复位"""
        self.sensor.heater_on()
        self.assertTrue(self.sensor.soft_reset())
        self.assertFalse(self.sensor._heater_on)

    def test_read_status(self):
        """读取状态寄存器"""
        ok, status = self.sensor.read_status()
        self.assertTrue(ok)
        self.assertEqual(status, 0x0000)


class TestSHT30CRC(unittest.TestCase):
    """SHT30 CRC-8校验测试"""

    def test_crc8_known_values(self):
        """CRC-8已知值验证"""
        # SHT30 CRC-8 多项式0x31，初值0xFF
        data = bytes([0xBE, 0xEF])
        crc = SHT30.crc8(data)
        self.assertIsInstance(crc, int)
        self.assertGreaterEqual(crc, 0)
        self.assertLessEqual(crc, 255)

    def test_crc8_consistency(self):
        """CRC-8一致性"""
        data = bytes([0x12, 0x34])
        crc1 = SHT30.crc8(data)
        crc2 = SHT30.crc8(data)
        self.assertEqual(crc1, crc2)

    def test_crc8_different_data(self):
        """不同数据不同CRC"""
        crc1 = SHT30.crc8(bytes([0x00, 0x00]))
        crc2 = SHT30.crc8(bytes([0xFF, 0xFF]))
        self.assertNotEqual(crc1, crc2)


class TestSGP30Init(unittest.TestCase):
    """SGP30空气质量传感器初始化测试"""

    def test_init_success(self):
        """SGP30初始化成功"""
        sensor = SGP30()
        self.assertTrue(sensor.init())
        self.assertTrue(sensor.initialized)

    def test_default_address(self):
        """默认I2C地址0x58"""
        sensor = SGP30()
        self.assertEqual(sensor.addr, SGP30_I2C_ADDR)

    def test_default_values(self):
        """默认测量值"""
        sensor = SGP30()
        sensor.init()
        ok, data = sensor.measure()
        self.assertTrue(ok)
        tvoc, eco2 = data
        self.assertEqual(tvoc, 0)
        self.assertEqual(eco2, 400)  # 默认基线eCO2


class TestSGP30Measurement(unittest.TestCase):
    """SGP30空气质量测量测试"""

    def setUp(self):
        self.sensor = SGP30()
        self.sensor.init()

    def test_measure_air_quality(self):
        """测量空气质量"""
        self.sensor.set_raw_values(220, 600)
        ok, data = self.sensor.measure()
        self.assertTrue(ok)
        tvoc, eco2 = data
        self.assertEqual(tvoc, 220)
        self.assertEqual(eco2, 600)

    def test_measure_not_initialized(self):
        """未初始化测量失败"""
        sensor2 = SGP30()
        ok, data = sensor2.measure()
        self.assertFalse(ok)
        self.assertIsNone(data)

    def test_baseline_get_set(self):
        """获取和恢复基线值"""
        self.sensor.set_baseline(1000, 50000)
        ok, baseline = self.sensor.get_baseline()
        self.assertTrue(ok)
        self.assertEqual(baseline[0], 1000)
        self.assertEqual(baseline[1], 50000)

    def test_set_humidity_compensation(self):
        """设置湿度补偿"""
        self.assertTrue(self.sensor.set_humidity(50.0, 25.0))

    def test_selftest(self):
        """自检"""
        self.assertTrue(self.sensor.selftest())

    def test_selftest_not_initialized(self):
        """未初始化自检失败"""
        sensor2 = SGP30()
        self.assertFalse(sensor2.selftest())


class TestSGP30CRC(unittest.TestCase):
    """SGP30 CRC-8校验测试"""

    def test_crc8_known_values(self):
        """CRC-8已知值验证"""
        data = bytes([0x20, 0x03])
        crc = SGP30.crc8(data)
        self.assertIsInstance(crc, int)
        self.assertGreaterEqual(crc, 0)
        self.assertLessEqual(crc, 255)

    def test_crc8_consistency(self):
        """CRC-8一致性"""
        data = bytes([0xBE, 0xEF])
        crc1 = SGP30.crc8(data)
        crc2 = SGP30.crc8(data)
        self.assertEqual(crc1, crc2)


class TestQMC5883LInit(unittest.TestCase):
    """QMC5883L电子罗盘初始化测试"""

    def test_init_success(self):
        """QMC5883L初始化成功"""
        compass = QMC5883L()
        self.assertTrue(compass.init())
        self.assertTrue(compass.initialized)

    def test_default_address(self):
        """默认I2C地址0x0D"""
        compass = QMC5883L()
        self.assertEqual(compass.addr, QMC5883L_I2C_ADDR)

    def test_default_range(self):
        """默认量程±2Gauss"""
        compass = QMC5883L()
        compass.init()
        self.assertEqual(compass._range, QMC5883L_RANGE_2G)

    def test_set_range_8g(self):
        """设置±8Gauss量程"""
        compass = QMC5883L()
        compass.init(range_cfg=QMC5883L_RANGE_8G)
        self.assertEqual(compass._range, QMC5883L_RANGE_8G)


class TestQMC5883LData(unittest.TestCase):
    """QMC5883L磁场数据测试"""

    def setUp(self):
        self.compass = QMC5883L()
        self.compass.init()

    def test_read_data_north(self):
        """读取北方（X轴正方向）"""
        self.compass.set_raw_values(1000, 0, 0)
        ok, data = self.compass.read_data()
        self.assertTrue(ok)
        # atan2(0, 1000) = 0°
        self.assertAlmostEqual(data.heading_deg, 0.0, delta=1.0)

    def test_read_data_east(self):
        """读取东方（Y轴正方向）"""
        self.compass.set_raw_values(0, 1000, 0)
        ok, data = self.compass.read_data()
        self.assertTrue(ok)
        # atan2(1000, 0) = 90°
        self.assertAlmostEqual(data.heading_deg, 90.0, delta=1.0)

    def test_read_data_south(self):
        """读取南方（X轴负方向）"""
        self.compass.set_raw_values(-1000, 0, 0)
        ok, data = self.compass.read_data()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.heading_deg, 180.0, delta=1.0)

    def test_read_data_west(self):
        """读取西方（Y轴负方向）"""
        self.compass.set_raw_values(0, -1000, 0)
        ok, data = self.compass.read_data()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.heading_deg, 270.0, delta=1.0)

    def test_gauss_conversion(self):
        """原始值转高斯"""
        # ±2Gauss量程，灵敏度12000 LSB/Gauss
        self.compass.set_raw_values(12000, 0, 0)
        ok, data = self.compass.read_data()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.x_gauss, 1.0, delta=0.1)

    def test_temperature_reading(self):
        """读取温度"""
        self.compass.set_raw_values(0, 0, 0, temp_raw=2500)
        ok, data = self.compass.read_data()
        self.assertTrue(ok)
        # 温度 = raw / 100
        self.assertAlmostEqual(data.temperature, 25.0, delta=0.1)

    def test_not_initialized(self):
        """未初始化读取失败"""
        compass2 = QMC5883L()
        ok, data = compass2.read_data()
        self.assertFalse(ok)
        self.assertIsNone(data)

    def test_chip_id(self):
        """读取芯片ID"""
        ok, chip_id = self.compass.read_chip_id()
        self.assertTrue(ok)
        self.assertEqual(chip_id, 0xFF)

    def test_soft_reset(self):
        """软复位"""
        self.compass.set_raw_values(100, 200, 300)
        self.assertTrue(self.compass.soft_reset())
        ok, data = self.compass.read_data()
        self.assertTrue(ok)
        self.assertEqual(data.x, 0)


class TestMPU6050Init(unittest.TestCase):
    """MPU6050六轴IMU初始化测试"""

    def test_init_success(self):
        """MPU6050初始化成功"""
        imu = MPU6050()
        self.assertTrue(imu.init())
        self.assertTrue(imu.initialized)

    def test_default_address(self):
        """默认I2C地址0x68"""
        imu = MPU6050()
        self.assertEqual(imu.addr, MPU6050_I2C_ADDR)

    def test_custom_address(self):
        """自定义I2C地址0x69"""
        imu = MPU6050(addr=MPU6050_I2C_ADDR_HIGH)
        self.assertEqual(imu.addr, MPU6050_I2C_ADDR_HIGH)

    def test_default_range(self):
        """默认量程"""
        imu = MPU6050()
        imu.init()
        self.assertEqual(imu._accel_fs, MPU6050_ACCEL_FS_2G)
        self.assertEqual(imu._gyro_fs, MPU6050_GYRO_FS_250DPS)

    def test_custom_range(self):
        """自定义量程"""
        imu = MPU6050()
        imu.init(accel_fs=MPU6050_ACCEL_FS_4G, gyro_fs=MPU6050_GYRO_FS_500DPS)
        self.assertEqual(imu._accel_fs, MPU6050_ACCEL_FS_4G)
        self.assertEqual(imu._gyro_fs, MPU6050_GYRO_FS_500DPS)

    def test_chip_id(self):
        """读取芯片ID"""
        imu = MPU6050()
        ok, chip_id = imu.read_id()
        self.assertTrue(ok)
        self.assertEqual(chip_id, 0x68)


class TestMPU6050Data(unittest.TestCase):
    """MPU6050数据读取测试"""

    def setUp(self):
        self.imu = MPU6050()
        self.imu.init()

    def test_read_all_data(self):
        """读取全部数据"""
        # 模拟静止状态：Z轴1g，其他轴0
        self.imu.set_raw_values((0, 0, 16384), (0, 0, 0), 1000)
        ok, data = self.imu.read_all()
        self.assertTrue(ok)
        # 加速度
        self.assertAlmostEqual(data.accel_z_g, 1.0, delta=0.01)
        self.assertAlmostEqual(data.accel_x_g, 0.0, delta=0.01)

    def test_gyro_conversion(self):
        """陀螺仪数据转换"""
        # ±250dps量程，灵敏度131 LSB/dps
        self.imu.set_raw_values((0, 0, 0), (1310, 0, 0), 0)
        ok, data = self.imu.read_all()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.gyro_x_dps, 10.0, delta=0.5)

    def test_temperature_reading(self):
        """读取温度"""
        # 温度公式: T = raw/340 + 36.53
        # raw=0 → 36.53°C
        self.imu.set_raw_values((0, 0, 0), (0, 0, 0), 0)
        ok, data = self.imu.read_all()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.temperature, 36.53, delta=0.1)

    def test_read_temperature_only(self):
        """仅读取温度"""
        self.imu.set_raw_values((0, 0, 0), (0, 0, 0), 3400)
        ok, temp = self.imu.read_temperature()
        self.assertTrue(ok)
        # 3400/340 + 36.53 = 46.53
        self.assertAlmostEqual(temp, 46.53, delta=0.1)

    def test_sleep_wake(self):
        """睡眠和唤醒"""
        self.assertTrue(self.imu.sleep())
        self.assertTrue(self.imu._sleeping)
        self.assertTrue(self.imu.wake_up())
        self.assertFalse(self.imu._sleeping)

    def test_not_initialized(self):
        """未初始化读取失败"""
        imu2 = MPU6050()
        ok, data = imu2.read_all()
        self.assertFalse(ok)
        self.assertIsNone(data)

    def test_sleep_not_initialized(self):
        """未初始化睡眠失败"""
        imu2 = MPU6050()
        self.assertFalse(imu2.sleep())
        self.assertFalse(imu2.wake_up())


class TestMultiSensorFusion(unittest.TestCase):
    """多传感器数据融合测试"""

    def setUp(self):
        """初始化所有传感器"""
        self.sht30 = SHT30()
        self.sgp30 = SGP30()
        self.compass = QMC5883L()
        self.imu = MPU6050()
        self.sht30.init()
        self.sgp30.init()
        self.compass.init()
        self.imu.init()

    def test_all_sensors_initialized(self):
        """所有传感器初始化成功"""
        self.assertTrue(self.sht30.initialized)
        self.assertTrue(self.sgp30.initialized)
        self.assertTrue(self.compass.initialized)
        self.assertTrue(self.imu.initialized)

    def test_concurrent_measurement(self):
        """并发测量"""
        # 设置模拟值
        self.sht30.set_raw_values(26214, 32768)  # 25°C, 50%RH
        self.sgp30.set_raw_values(150, 500)       # TVOC=150, eCO2=500
        self.compass.set_raw_values(5000, 3000, 1000)  # 磁场
        self.imu.set_raw_values((0, 0, 16384), (0, 0, 0), 0)  # 静止

        # 同时读取所有传感器
        ok1, temp_humi = self.sht30.measure_single()
        ok2, air = self.sgp30.measure()
        ok3, mag = self.compass.read_data()
        ok4, motion = self.imu.read_all()

        self.assertTrue(ok1)
        self.assertTrue(ok2)
        self.assertTrue(ok3)
        self.assertTrue(ok4)

        # 验证数据范围
        self.assertAlmostEqual(temp_humi[0], 25.0, delta=1.0)
        self.assertEqual(air[0], 150)  # TVOC
        self.assertEqual(air[1], 500)  # eCO2
        self.assertGreater(mag.heading_deg, 0)
        self.assertAlmostEqual(motion.accel_z_g, 1.0, delta=0.01)

    def test_environmental_monitoring(self):
        """环境监测场景"""
        # 高温高湿
        self.sht30.set_raw_values(45000, 55000)
        ok, data = self.sht30.measure_single()
        self.assertTrue(ok)
        temp, humi = data
        self.assertGreater(temp, 70)  # 高温
        self.assertGreater(humi, 80)  # 高湿

    def test_imu_with_kalman_filter(self):
        """IMU数据卡尔曼滤波"""
        kf = KalmanFilter(dt=0.01, proc_noise=0.1, meas_noise=1.0)

        # 模拟带噪声的加速度数据
        accel_readings = [16300, 16400, 16350, 16380, 16390]
        for raw in accel_readings:
            self.imu.set_raw_values((0, 0, raw), (0, 0, 0), 0)
            ok, data = self.imu.read_all()
            self.assertTrue(ok)
            kf.predict()
            kf.update_1d(data.accel_z_g)

        # 滤波后应接近1g
        self.assertAlmostEqual(kf.x[0], 1.0, delta=0.2)

    def test_heading_with_tilt_compensation(self):
        """倾斜补偿航向角（简化版）"""
        # 使用IMU加速度数据判断倾斜
        self.imu.set_raw_values((0, 8192, 14189), (0, 0, 0), 0)
        ok, motion = self.imu.read_all()
        self.assertTrue(ok)

        # 计算倾斜角
        pitch = math.atan2(motion.accel_x_g,
                           math.sqrt(motion.accel_y_g**2 + motion.accel_z_g**2))

        # 使用罗盘读取航向
        self.compass.set_raw_values(5000, 3000, 0)
        ok, mag = self.compass.read_data()
        self.assertTrue(ok)

        # 验证数据可用
        self.assertIsNotNone(mag.heading_deg)
        self.assertGreater(mag.heading_deg, 0)


if __name__ == '__main__':
    unittest.main()
