#!/usr/bin/env python3
"""
JY901S 九轴IMU V2测试 — 基于wrappers.py包装层
覆盖: 初始化、帧解析、加速度/角速度/角度/磁场、校验和、异常处理
对应C源文件: 02_mspm0g3507/drivers/jy901s.h

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    JY901S, JY901S_Data,
    JY901S_FRAME_HEAD, JY901S_TYPE_ACC, JY901S_TYPE_GYRO,
    JY901S_TYPE_ANGLE, JY901S_TYPE_MAG, JY901S_FRAME_LEN,
)


class TestJY901SInit(unittest.TestCase):
    """初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        imu = JY901S()
        ok = imu.init()
        self.assertTrue(ok)
        self.assertTrue(imu.initialized)

    def test_default_data_zero(self):
        """初始化后数据应为零"""
        imu = JY901S()
        imu.init()
        data = imu.get_data()
        self.assertAlmostEqual(data.acc_x, 0.0)
        self.assertAlmostEqual(data.roll, 0.0)
        self.assertEqual(data.mag_x, 0)

    def test_get_roll_pitch_yaw(self):
        """角度getter方法"""
        imu = JY901S()
        imu.init()
        self.assertAlmostEqual(imu.get_roll(), 0.0)
        self.assertAlmostEqual(imu.get_pitch(), 0.0)
        self.assertAlmostEqual(imu.get_yaw(), 0.0)


class TestJY901SFrameBuild(unittest.TestCase):
    """帧构建测试"""

    def test_build_frame_acc(self):
        """构建加速度帧: 长度11、帧头正确"""
        frame = JY901S.build_frame(JY901S_TYPE_ACC, [1000, 2000, 3000, 0])
        self.assertEqual(len(frame), JY901S_FRAME_LEN)
        self.assertEqual(frame[0], JY901S_FRAME_HEAD)
        self.assertEqual(frame[1], JY901S_TYPE_ACC)

    def test_build_frame_checksum(self):
        """帧校验和正确"""
        raw = [0x0100, 0x0200, 0x0300, 0x0000]
        frame = JY901S.build_frame(JY901S_TYPE_ANGLE, raw)
        calc_sum = sum(frame[:10]) & 0xFF
        self.assertEqual(frame[10], calc_sum)

    def test_build_frame_negative_values(self):
        """负数原始值构建帧正确"""
        frame = JY901S.build_frame(JY901S_TYPE_GYRO, [-1000, -2000, 0, 0])
        self.assertEqual(len(frame), JY901S_FRAME_LEN)


class TestJY901SAccParse(unittest.TestCase):
    """加速度帧解析测试"""

    def test_parse_acc_zero(self):
        """零加速度解析"""
        imu = JY901S()
        imu.init()
        # 原始值0 → 0g
        frame = JY901S.build_frame(JY901S_TYPE_ACC, [0, 0, 0, 0])
        for b in frame:
            parsed, ftype = imu.feed_byte(b)
        self.assertTrue(parsed)
        self.assertEqual(ftype, JY901S_TYPE_ACC)
        data = imu.get_data()
        self.assertAlmostEqual(data.acc_x, 0.0, places=3)
        self.assertAlmostEqual(data.acc_y, 0.0, places=3)
        self.assertAlmostEqual(data.acc_z, 0.0, places=3)
        self.assertTrue(data.acc_updated)

    def test_parse_acc_1g(self):
        """1g加速度解析: 原始值2048 → 1g"""
        imu = JY901S()
        imu.init()
        raw_1g = int(32768.0 / 16.0)  # = 2048 → 1g
        frame = JY901S.build_frame(JY901S_TYPE_ACC, [raw_1g, 0, 0, 0])
        for b in frame:
            imu.feed_byte(b)
        data = imu.get_data()
        self.assertAlmostEqual(data.acc_x, 1.0, places=2)


class TestJY901SGyroParse(unittest.TestCase):
    """角速度帧解析测试"""

    def test_parse_gyro(self):
        """角速度解析"""
        imu = JY901S()
        imu.init()
        # 原始值16384 → 1000 dps
        raw_1000 = int(32768.0 / 2000.0 * 1000.0)  # = 16384
        frame = JY901S.build_frame(JY901S_TYPE_GYRO, [raw_1000, 0, 0, 0])
        for b in frame:
            imu.feed_byte(b)
        data = imu.get_data()
        self.assertAlmostEqual(data.gyro_x, 1000.0, delta=1.0)
        self.assertTrue(data.gyro_updated)


class TestJY901SAngleParse(unittest.TestCase):
    """角度帧解析测试"""

    def test_parse_angle_zero(self):
        """零角度解析"""
        imu = JY901S()
        imu.init()
        frame = JY901S.build_frame(JY901S_TYPE_ANGLE, [0, 0, 0, 0])
        for b in frame:
            imu.feed_byte(b)
        self.assertAlmostEqual(imu.get_roll(), 0.0, places=2)
        self.assertAlmostEqual(imu.get_pitch(), 0.0, places=2)
        self.assertAlmostEqual(imu.get_yaw(), 0.0, places=2)
        self.assertTrue(imu.is_angle_updated())

    def test_parse_angle_90(self):
        """90度解析: 原始值16384 → 90°"""
        imu = JY901S()
        imu.init()
        raw_90 = int(32768.0 / 180.0 * 90.0)  # = 16384
        frame = JY901S.build_frame(JY901S_TYPE_ANGLE, [raw_90, 0, 0, 0])
        for b in frame:
            imu.feed_byte(b)
        self.assertAlmostEqual(imu.get_roll(), 90.0, delta=0.1)

    def test_parse_angle_negative(self):
        """负角度解析: -90°"""
        imu = JY901S()
        imu.init()
        raw_neg90 = int(-32768.0 / 180.0 * 90.0) & 0xFFFF
        frame = JY901S.build_frame(JY901S_TYPE_ANGLE, [raw_neg90, 0, 0, 0])
        for b in frame:
            imu.feed_byte(b)
        self.assertAlmostEqual(imu.get_roll(), -90.0, delta=0.1)

    def test_angle_flag_clear(self):
        """角度标志清除"""
        imu = JY901S()
        imu.init()
        frame = JY901S.build_frame(JY901S_TYPE_ANGLE, [0, 0, 0, 0])
        for b in frame:
            imu.feed_byte(b)
        self.assertTrue(imu.is_angle_updated())
        imu.clear_angle_flag()
        self.assertFalse(imu.is_angle_updated())


class TestJY901SMagParse(unittest.TestCase):
    """磁场帧解析测试"""

    def test_parse_mag(self):
        """磁场原始值解析"""
        imu = JY901S()
        imu.init()
        frame = JY901S.build_frame(JY901S_TYPE_MAG, [100, -200, 300, 0])
        for b in frame:
            imu.feed_byte(b)
        data = imu.get_data()
        self.assertEqual(data.mag_x, 100)
        self.assertEqual(data.mag_y, -200)
        self.assertEqual(data.mag_z, 300)


class TestJY901SFrameError(unittest.TestCase):
    """帧错误处理测试"""

    def test_bad_checksum(self):
        """错误校验和应被丢弃"""
        imu = JY901S()
        imu.init()
        frame = JY901S.build_frame(JY901S_TYPE_ACC, [0, 0, 0, 0])
        frame[10] ^= 0xFF  # 破坏校验和
        for b in frame:
            imu.feed_byte(b)
        self.assertEqual(imu.get_frame_count(), 0)
        self.assertEqual(imu.get_parse_errors(), 1)

    def test_noise_bytes_ignored(self):
        """随机噪声字节应被忽略"""
        imu = JY901S()
        imu.init()
        # 喂入随机字节
        for b in [0x12, 0x34, 0x56, 0x78]:
            imu.feed_byte(b)
        self.assertEqual(imu.get_frame_count(), 0)

    def test_multiple_frames(self):
        """连续解析多帧"""
        imu = JY901S()
        imu.init()
        # 角度帧
        f1 = JY901S.build_frame(JY901S_TYPE_ANGLE, [1000, 2000, 3000, 0])
        # 加速度帧
        f2 = JY901S.build_frame(JY901S_TYPE_ACC, [500, 600, 700, 0])
        for b in f1 + f2:
            imu.feed_byte(b)
        self.assertEqual(imu.get_frame_count(), 2)

    def test_int16_conversion(self):
        """int16转换: 正/负/零"""
        self.assertEqual(JY901S._bytes_to_int16(0, 0), 0)
        self.assertEqual(JY901S._bytes_to_int16(0x00, 0x40), 0x4000)
        # 0x00FF = 255, 但 0xFF00 = 65280 → -256
        self.assertEqual(JY901S._bytes_to_int16(0xFF, 0xFF), -1)


if __name__ == '__main__':
    unittest.main()
