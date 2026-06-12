#!/usr/bin/env python3
"""
GPS NEO-6M V3 测试 — 多语句解析+坐标转换深度测试
覆盖: V2全部 + GGA/RMC多语句连续解析、NMEA到十进制度转换、
      多talker前缀（GP/GN）、速度/日期/卫星数、新数据标志、
      无效语句处理、缓冲区溢出保护
对应C源文件: 02_mspm0g3507/drivers/gps_neo6m.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #6:  I2C/UART忙等待/超时
  #13: GPS用于定位导航，坐标转换精度至关重要
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    GPS_NEO6M, GPS_Data,
    GPS_NMEA_MAX_LEN, GPS_MAX_SATELLITES,
    GPS_NMEA_NONE, GPS_NMEA_GGA, GPS_NMEA_RMC, GPS_NMEA_GSV, GPS_NMEA_GSA, GPS_NMEA_UNKNOWN,
    GPS_FIX_NONE, GPS_FIX_GPS, GPS_FIX_DGPS,
)

# ── 常用NMEA测试语句 ──
# GGA: 北京天安门附近 39°54'N, 116°23'E
GGA_BEIJING = "GPGGA,082530.00,3954.1234,N,11623.5678,E,1,08,1.0,50.0,M,-5.0,M,,"
# RMC: 有效定位
RMC_VALID = "GPRMC,082530.00,A,3954.1234,N,11623.5678,E,0.5,90.0,120626,,,A"
# RMC: 无效定位
RMC_VOID = "GPRMC,082530.00,V,3954.1234,N,11623.5678,E,0.5,90.0,120626,,,A"
# GN前缀GGA
GN_GGA = "GNGGA,120000.00,3110.5555,N,12126.8888,E,1,12,0.8,100.0,M,0.0,M,,"


def calc_checksum(sentence_body):
    """计算NMEA校验和（'*'之前的部分）"""
    cs = 0
    for ch in sentence_body:
        cs ^= ord(ch)
    return cs


def make_valid_nmea(body):
    """生成带正确校验和的NMEA语句"""
    cs = calc_checksum(body)
    return f"${body}*{cs:02X}\n"


class TestGPSV3Init(unittest.TestCase):
    """GPS V3 初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        gps = GPS_NEO6M()
        ok = gps.init()
        self.assertTrue(ok)
        self.assertTrue(gps.initialized)

    def test_init_resets_data(self):
        """初始化重置数据"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        gps.init()
        data = gps.get_data()
        self.assertEqual(data.latitude, 0.0)
        self.assertEqual(data.longitude, 0.0)

    def test_init_resets_buffer(self):
        """初始化清空缓冲区"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string("garbage data that should be cleared")
        gps.init()
        self.assertEqual(len(gps._buf), 0)

    def test_new_data_false_after_get(self):
        """get_data后新数据标志清除"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        self.assertTrue(gps.has_new_data())
        gps.get_data()
        self.assertFalse(gps.has_new_data())


class TestGPSV3GGAParsing(unittest.TestCase):
    """GPS V3 GGA语句解析测试"""

    def test_gga_beijing(self):
        """解析北京坐标"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        data = gps.get_data()
        self.assertTrue(data.valid_position)
        self.assertTrue(data.valid_time)
        # 纬度: 39°54.1234' = 39 + 54.1234/60 ≈ 39.902057
        self.assertAlmostEqual(data.latitude, 39.0 + 54.1234 / 60.0, places=4)
        # 经度: 116°23.5678' = 116 + 23.5678/60 ≈ 116.392797
        self.assertAlmostEqual(data.longitude, 116.0 + 23.5678 / 60.0, places=4)

    def test_gga_time(self):
        """GGA时间解析"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        data = gps.get_data()
        self.assertEqual(data.hour, 8)
        self.assertEqual(data.minute, 25)
        self.assertEqual(data.second, 30)

    def test_gga_fix_quality(self):
        """GGA定位质量"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        data = gps.get_data()
        self.assertEqual(data.fix_quality, GPS_FIX_GPS)

    def test_gga_satellites(self):
        """GGA卫星数"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        data = gps.get_data()
        self.assertEqual(data.satellites_used, 8)

    def test_gga_hdop(self):
        """GGA水平精度因子"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        data = gps.get_data()
        self.assertAlmostEqual(data.hdop, 1.0, places=1)

    def test_gga_altitude(self):
        """GGA海拔"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        data = gps.get_data()
        self.assertAlmostEqual(data.altitude_m, 50.0, places=1)

    def test_gga_gn_prefix(self):
        """GN前缀GGA（北斗+GPS混合）"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(GN_GGA))
        data = gps.get_data()
        self.assertTrue(data.valid_position)
        # 31°10.5555'N
        self.assertAlmostEqual(data.latitude, 31.0 + 10.5555 / 60.0, places=4)
        # 121°26.8888'E
        self.assertAlmostEqual(data.longitude, 121.0 + 26.8888 / 60.0, places=4)


class TestGPSV3RMCParsing(unittest.TestCase):
    """GPS V3 RMC语句解析测试"""

    def test_rmc_valid(self):
        """RMC有效定位"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(RMC_VALID))
        data = gps.get_data()
        # 速度: 0.5节 → 0.5*1.852=0.926 km/h
        self.assertAlmostEqual(data.speed_knots, 0.5, places=1)
        self.assertAlmostEqual(data.speed_kmh, 0.5 * 1.852, places=2)
        # 航向
        self.assertAlmostEqual(data.course_deg, 90.0, places=1)

    def test_rmc_date(self):
        """RMC日期解析"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(RMC_VALID))
        data = gps.get_data()
        self.assertTrue(data.valid_date)
        self.assertEqual(data.day, 12)
        self.assertEqual(data.month, 6)
        self.assertEqual(data.year, 2026)

    def test_rmc_void_ignored(self):
        """RMC无效定位应被忽略（不更新位置）"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(RMC_VOID))
        data = gps.get_data()
        # 无效定位不应设置valid_position
        # RMC Void不解析速度/航向
        self.assertEqual(data.speed_knots, 0.0)


class TestGPSV3MultiSentence(unittest.TestCase):
    """GPS V3 多语句连续解析测试"""

    def test_gga_then_rmc(self):
        """先GGA后RMC，数据合并"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        gps.feed_string(make_valid_nmea(RMC_VALID))
        data = gps.get_data()
        # GGA数据
        self.assertTrue(data.valid_position)
        self.assertEqual(data.satellites_used, 8)
        # RMC数据
        self.assertAlmostEqual(data.speed_knots, 0.5, places=1)
        self.assertTrue(data.valid_date)

    def test_rmc_then_gga(self):
        """先RMC后GGA"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(RMC_VALID))
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        data = gps.get_data()
        self.assertTrue(data.valid_position)
        self.assertAlmostEqual(data.latitude, 39.0 + 54.1234 / 60.0, places=4)

    def test_multiple_gga_updates(self):
        """多次GGA更新"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        # 第二个GGA：上海 31°10'N, 121°26'E
        gps.feed_string(make_valid_nmea(GN_GGA))
        data = gps.get_data()
        # 应更新为上海坐标
        self.assertAlmostEqual(data.latitude, 31.0 + 10.5555 / 60.0, places=4)


class TestGPSV3CoordinateConversion(unittest.TestCase):
    """GPS V3 坐标转换测试"""

    def test_nmea_to_degrees_equator(self):
        """赤道: 0°0.0000'N → 0.0"""
        deg = GPS_NEO6M.nmea_to_degrees(0.0, 'N')
        self.assertAlmostEqual(deg, 0.0, places=6)

    def test_nmea_to_degrees_north(self):
        """北纬39°54.1234'"""
        deg = GPS_NEO6M.nmea_to_degrees(3954.1234, 'N')
        expected = 39.0 + 54.1234 / 60.0
        self.assertAlmostEqual(deg, expected, places=6)

    def test_nmea_to_degrees_south(self):
        """南纬取负"""
        deg = GPS_NEO6M.nmea_to_degrees(3954.1234, 'S')
        self.assertLess(deg, 0)
        self.assertAlmostEqual(deg, -(39.0 + 54.1234 / 60.0), places=6)

    def test_nmea_to_degrees_east(self):
        """东经116°23.5678'"""
        deg = GPS_NEO6M.nmea_to_degrees(11623.5678, 'E')
        expected = 116.0 + 23.5678 / 60.0
        self.assertAlmostEqual(deg, expected, places=6)

    def test_nmea_to_degrees_west(self):
        """西经取负"""
        deg = GPS_NEO6M.nmea_to_degrees(11623.5678, 'W')
        self.assertLess(deg, 0)

    def test_nmea_to_degrees_pure_degrees(self):
        """纯度数（分=0）"""
        deg = GPS_NEO6M.nmea_to_degrees(4500.0000, 'N')
        self.assertAlmostEqual(deg, 45.0, places=6)

    def test_nmea_to_degrees_high_precision(self):
        """高精度小数"""
        deg = GPS_NEO6M.nmea_to_degrees(3954.12345, 'N')
        expected = 39.0 + 54.12345 / 60.0
        self.assertAlmostEqual(deg, expected, places=6)


class TestGPSV3ByteFeed(unittest.TestCase):
    """GPS V3 逐字节喂入测试"""

    def test_feed_byte_by_byte(self):
        """逐字节喂入完整语句"""
        gps = GPS_NEO6M()
        gps.init()
        sentence = make_valid_nmea(GGA_BEIJING) + "\n"
        for ch in make_valid_nmea(GGA_BEIJING):
            gps.feed_byte(ord(ch))
        data = gps.get_data()
        self.assertTrue(data.valid_position)

    def test_feed_string(self):
        """feed_string等效于逐字节"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        data = gps.get_data()
        self.assertTrue(data.valid_position)

    def test_buffer_overflow_protection(self):
        """超长数据不崩溃"""
        gps = GPS_NEO6M()
        gps.init()
        # 喂入超过GPS_NMEA_MAX_LEN字节的垃圾数据
        garbage = "X" * (GPS_NMEA_MAX_LEN + 100)
        for ch in garbage:
            gps.feed_byte(ord(ch))
        # 不应崩溃
        self.assertTrue(True)


class TestGPSV3InvalidSentences(unittest.TestCase):
    """GPS V3 无效语句处理测试"""

    def test_no_checksum_ignored(self):
        """无校验和的语句被忽略"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string("$GPGGA,invalid*XX\n")
        # 无效语句不应崩溃且不设置有效位置
        data = gps.get_data()
        self.assertFalse(data.valid_time)

    def test_empty_sentence(self):
        """空语句不崩溃"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string("$*00\r\n")
        self.assertTrue(True)

    def test_unknown_talker(self):
        """未知talker类型不崩溃"""
        gps = GPS_NEO6M()
        gps.init()
        gps.feed_string(make_valid_nmea("XXAAA,1,2,3"))
        self.assertTrue(True)


class TestGPSV3DataStructure(unittest.TestCase):
    """GPS V3 数据结构测试"""

    def test_gps_data_defaults(self):
        """默认数据"""
        data = GPS_Data()
        self.assertEqual(data.hour, 0)
        self.assertEqual(data.latitude, 0.0)
        self.assertEqual(data.longitude, 0.0)
        self.assertEqual(data.altitude_m, 0.0)
        self.assertEqual(data.speed_knots, 0.0)
        self.assertEqual(data.speed_kmh, 0.0)
        self.assertEqual(data.fix_quality, GPS_FIX_NONE)
        self.assertFalse(data.valid_position)
        self.assertFalse(data.valid_time)
        self.assertFalse(data.valid_date)
        self.assertEqual(data.satellites_used, 0)
        self.assertEqual(data.satellites, [])


class TestGPSV3Constants(unittest.TestCase):
    """GPS V3 常量一致性验证"""

    def test_nmea_constants(self):
        self.assertEqual(GPS_NMEA_MAX_LEN, 128)
        self.assertEqual(GPS_MAX_SATELLITES, 12)

    def test_nmea_types(self):
        self.assertEqual(GPS_NMEA_NONE, 0)
        self.assertEqual(GPS_NMEA_GGA, 1)
        self.assertEqual(GPS_NMEA_RMC, 2)
        self.assertEqual(GPS_NMEA_GSV, 3)
        self.assertEqual(GPS_NMEA_GSA, 4)
        self.assertEqual(GPS_NMEA_UNKNOWN, 5)

    def test_fix_quality(self):
        self.assertEqual(GPS_FIX_NONE, 0)
        self.assertEqual(GPS_FIX_GPS, 1)
        self.assertEqual(GPS_FIX_DGPS, 2)


class TestGPSV3FullWorkflow(unittest.TestCase):
    """GPS V3 完整工作流程"""

    def test_startup_to_first_fix(self):
        """冷启动→首次定位流程"""
        gps = GPS_NEO6M()
        gps.init()
        # 1. 初始无数据
        data = gps.get_data()
        self.assertFalse(data.valid_position)
        # 2. 收到GGA
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        data = gps.get_data()
        self.assertTrue(data.valid_position)
        self.assertTrue(data.valid_time)
        # 3. 收到RMC补全速度和日期
        gps.feed_string(make_valid_nmea(RMC_VALID))
        data = gps.get_data()
        self.assertTrue(data.valid_date)
        self.assertGreater(data.speed_knots, 0)
        # 4. 验证完整数据
        self.assertEqual(data.satellites_used, 8)
        self.assertAlmostEqual(data.altitude_m, 50.0, places=1)

    def test_position_update(self):
        """位置更新（模拟移动）"""
        gps = GPS_NEO6M()
        gps.init()
        # 位置1: 北京
        gps.feed_string(make_valid_nmea(GGA_BEIJING))
        data = gps.get_data()
        lat1 = data.latitude
        # 位置2: 上海
        gps.feed_string(make_valid_nmea(GN_GGA))
        data = gps.get_data()
        lat2 = data.latitude
        # 坐标应更新
        self.assertNotAlmostEqual(lat1, lat2, places=2)


if __name__ == '__main__':
    unittest.main()
