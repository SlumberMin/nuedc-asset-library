#!/usr/bin/env python3
"""
MAX7219 LED点阵 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、像素设置/清除、整行数据、亮度、关断模式、
      测试模式、级联、边界条件、SPI日志
对应C源文件: 02_mspm0g3507/drivers/max7219.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    MAX7219, MAX7219_ROWS, MAX7219_COLS,
    MAX7219_REG_SHUTDOWN, MAX7219_REG_INTENSITY,
    MAX7219_REG_SCAN_LIMIT, MAX7219_REG_TEST,
    MAX7219_REG_DIGIT0, MAX7219_INTENSITY_MAX,
)


class TestMAX7219Init(unittest.TestCase):
    """MAX7219初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        m = MAX7219()
        self.assertTrue(m.init())
        self.assertTrue(m.initialized)

    def test_init_shutdown_off(self):
        """初始化后退出关断模式"""
        m = MAX7219()
        m.init()
        self.assertFalse(m.shutdown_mode)

    def test_init_intensity_default(self):
        """初始化后亮度为7"""
        m = MAX7219()
        m.init()
        self.assertEqual(m.intensity, 7)

    def test_init_scan_limit_full(self):
        """初始化后扫描全部8行"""
        m = MAX7219()
        m.init()
        self.assertEqual(m.scan_limit, 7)

    def test_init_buffer_cleared(self):
        """初始化后缓冲区全零"""
        m = MAX7219()
        m.init()
        for r in range(MAX7219_ROWS):
            self.assertEqual(m.get_row(r), 0x00)

    def test_default_cascaded_one(self):
        """默认级联数为1"""
        m = MAX7219()
        self.assertEqual(m.num_cascaded, 1)

    def test_cascaded_two(self):
        """级联2个MAX7219"""
        m = MAX7219(2)
        self.assertEqual(m.num_cascaded, 2)
        self.assertEqual(len(m._buf), 2)

    def test_cascaded_zero_clamp(self):
        """级联数为0时自动修正为1"""
        m = MAX7219(0)
        self.assertEqual(m.num_cascaded, 1)


class TestMAX7219Pixel(unittest.TestCase):
    """MAX7219像素操作测试"""

    def setUp(self):
        self.m = MAX7219()
        self.m.init()

    def test_set_pixel_on(self):
        """点亮单个像素"""
        self.assertTrue(self.m.set_pixel(0, 0, True))
        self.assertTrue(self.m.get_pixel(0, 0))

    def test_set_pixel_off(self):
        """熄灭单个像素"""
        self.m.set_pixel(3, 5, True)
        self.assertTrue(self.m.set_pixel(3, 5, False))
        self.assertFalse(self.m.get_pixel(3, 5))

    def test_set_pixel_boundary(self):
        """边界像素: (0,0)和(7,7)"""
        self.assertTrue(self.m.set_pixel(0, 0, True))
        self.assertTrue(self.m.set_pixel(7, 7, True))
        self.assertTrue(self.m.get_pixel(0, 0))
        self.assertTrue(self.m.get_pixel(7, 7))

    def test_set_pixel_out_of_range(self):
        """越界像素设置失败"""
        self.assertFalse(self.m.set_pixel(-1, 0, True))
        self.assertFalse(self.m.set_pixel(8, 0, True))
        self.assertFalse(self.m.set_pixel(0, -1, True))
        self.assertFalse(self.m.set_pixel(0, 8, True))

    def test_get_pixel_out_of_range(self):
        """越界像素获取返回False"""
        self.assertFalse(self.m.get_pixel(-1, 0))
        self.assertFalse(self.m.get_pixel(0, 8))

    def test_set_pixel_not_initialized(self):
        """未初始化时设置像素失败"""
        m2 = MAX7219()
        self.assertFalse(m2.set_pixel(0, 0, True))

    def test_pixel_independence(self):
        """像素独立：修改一个不影响其他"""
        self.m.set_pixel(0, 0, True)
        self.m.set_pixel(0, 1, True)
        self.m.set_pixel(0, 0, False)
        self.assertFalse(self.m.get_pixel(0, 0))
        self.assertTrue(self.m.get_pixel(0, 1))


class TestMAX7219Row(unittest.TestCase):
    """MAX7219整行数据测试"""

    def setUp(self):
        self.m = MAX7219()
        self.m.init()

    def test_set_row(self):
        """设置整行数据"""
        self.assertTrue(self.m.set_row(3, 0xAA))
        self.assertEqual(self.m.get_row(3), 0xAA)

    def test_set_row_boundary(self):
        """边界行设置"""
        self.assertTrue(self.m.set_row(0, 0xFF))
        self.assertTrue(self.m.set_row(7, 0xFF))

    def test_set_row_out_of_range(self):
        """越界行设置失败"""
        self.assertFalse(self.m.set_row(-1, 0x00))
        self.assertFalse(self.m.set_row(8, 0x00))

    def test_set_row_mask_8bit(self):
        """行数据仅保留低8位"""
        self.m.set_row(0, 0x1FF)
        self.assertEqual(self.m.get_row(0), 0xFF)

    def test_get_row_default_zero(self):
        """未设置时行数据为0"""
        self.assertEqual(self.m.get_row(4), 0x00)


class TestMAX7219Clear(unittest.TestCase):
    """MAX7219清除测试"""

    def setUp(self):
        self.m = MAX7219()
        self.m.init()

    def test_clear_all(self):
        """清除全部显示"""
        for r in range(MAX7219_ROWS):
            self.m.set_row(r, 0xFF)
        self.m.clear()
        for r in range(MAX7219_ROWS):
            self.assertEqual(self.m.get_row(r), 0x00)

    def test_clear_single_chip(self):
        """清除指定芯片"""
        m2 = MAX7219(2)
        m2.init()
        m2.set_row(0, 0xFF, chip_index=0)
        m2.set_row(0, 0xFF, chip_index=1)
        m2.clear(chip_index=0)
        self.assertEqual(m2.get_row(0, chip_index=0), 0x00)
        self.assertEqual(m2.get_row(0, chip_index=1), 0xFF)


class TestMAX7219Brightness(unittest.TestCase):
    """MAX7219亮度测试"""

    def setUp(self):
        self.m = MAX7219()
        self.m.init()

    def test_set_intensity_normal(self):
        """设置亮度"""
        self.m.set_intensity(10)
        self.assertEqual(self.m.intensity, 10)

    def test_set_intensity_clamp_max(self):
        """亮度超过15被限幅"""
        self.m.set_intensity(20)
        self.assertEqual(self.m.intensity, MAX7219_INTENSITY_MAX)

    def test_set_intensity_clamp_min(self):
        """亮度低于0被限幅"""
        self.m.set_intensity(-5)
        self.assertEqual(self.m.intensity, 0)

    def test_set_intensity_zero(self):
        """亮度为0"""
        self.m.set_intensity(0)
        self.assertEqual(self.m.intensity, 0)


class TestMAX7219Modes(unittest.TestCase):
    """MAX7219模式控制测试"""

    def setUp(self):
        self.m = MAX7219()
        self.m.init()

    def test_shutdown_on(self):
        """进入关断模式"""
        self.m.set_shutdown(True)
        self.assertTrue(self.m.shutdown_mode)

    def test_shutdown_off(self):
        """退出关断模式"""
        self.m.set_shutdown(True)
        self.m.set_shutdown(False)
        self.assertFalse(self.m.shutdown_mode)

    def test_test_mode_on(self):
        """进入测试模式"""
        self.m.set_test_mode(True)
        self.assertTrue(self.m.test_mode)

    def test_test_mode_off(self):
        """退出测试模式"""
        self.m.set_test_mode(True)
        self.m.set_test_mode(False)
        self.assertFalse(self.m.test_mode)

    def test_set_decode_mode(self):
        """设置解码模式"""
        self.m.set_decode_mode(0xFF)
        self.assertEqual(self.m.decode_mode, 0xFF)

    def test_set_scan_limit(self):
        """设置扫描限制"""
        self.m.set_scan_limit(3)
        self.assertEqual(self.m.scan_limit, 3)

    def test_set_scan_limit_clamp(self):
        """扫描限制超范围被限幅"""
        self.m.set_scan_limit(10)
        self.assertEqual(self.m.scan_limit, 7)


class TestMAX7219Flush(unittest.TestCase):
    """MAX7219刷新测试"""

    def test_flush_success(self):
        """初始化后刷新成功"""
        m = MAX7219()
        m.init()
        self.assertTrue(m.flush())

    def test_flush_not_initialized(self):
        """未初始化刷新失败"""
        m = MAX7219()
        self.assertFalse(m.flush())


class TestMAX7219Cascaded(unittest.TestCase):
    """MAX7219级联测试"""

    def test_chip_index_pixel(self):
        """级联模式下设置不同芯片的像素"""
        m = MAX7219(2)
        m.init()
        self.assertTrue(m.set_pixel(0, 0, True, chip_index=0))
        self.assertTrue(m.set_pixel(0, 0, True, chip_index=1))
        self.assertTrue(m.get_pixel(0, 0, chip_index=0))
        self.assertTrue(m.get_pixel(0, 0, chip_index=1))

    def test_invalid_chip_index(self):
        """无效芯片索引失败"""
        m = MAX7219(2)
        m.init()
        self.assertFalse(m.set_pixel(0, 0, True, chip_index=2))
        self.assertFalse(m.set_pixel(0, 0, True, chip_index=-1))


class TestMAX7219SPILog(unittest.TestCase):
    """MAX7219 SPI日志测试"""

    def test_spi_log_records(self):
        """SPI操作被记录"""
        m = MAX7219()
        m.init()
        m.set_pixel(0, 0, True)
        self.assertGreater(len(m.spi_log), 0)
        # 每次set_pixel产生一条日志
        last = m.spi_log[-1]
        self.assertEqual(last[1], MAX7219_REG_DIGIT0)


class TestMAX7219Constants(unittest.TestCase):
    """MAX7219常量一致性"""

    def test_constants(self):
        """常量值正确"""
        self.assertEqual(MAX7219_ROWS, 8)
        self.assertEqual(MAX7219_COLS, 8)
        self.assertEqual(MAX7219_INTENSITY_MAX, 0x0F)


if __name__ == '__main__':
    unittest.main()
