#!/usr/bin/env python3
"""
WS2812 可寻址LED V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、RGB设置、HSV设置、填充、清除、亮度、边界条件
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import WS2812, WS2812_MAX_LEDS, WS2812_RESET_US


class TestWS2812V2Init(unittest.TestCase):
    """初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        led = WS2812(8)
        ok = led.init()
        self.assertTrue(ok)
        self.assertTrue(led.initialized)

    def test_num_leds(self):
        """LED数量正确"""
        led = WS2812(16)
        self.assertEqual(led.num_leds, 16)

    def test_num_leds_max(self):
        """超过最大值应限制"""
        led = WS2812(1000)
        self.assertEqual(led.num_leds, WS2812_MAX_LEDS)

    def test_all_black_after_init(self):
        """初始化后全黑"""
        led = WS2812(4)
        for i in range(4):
            r, g, b = led.get_pixel(i)
            self.assertEqual(r, 0)
            self.assertEqual(g, 0)
            self.assertEqual(b, 0)


class TestWS2812V2RGB(unittest.TestCase):
    """RGB颜色测试"""

    def test_set_pixel_rgb(self):
        """设置RGB颜色"""
        led = WS2812(4)
        led.init()
        ok = led.set_pixel(0, 255, 128, 64)
        self.assertTrue(ok)
        r, g, b = led.get_pixel(0)
        self.assertEqual(r, 255)
        self.assertEqual(g, 128)
        self.assertEqual(b, 64)

    def test_grb_order(self):
        """内部存储应为GRB顺序"""
        led = WS2812(1)
        led.init()
        led.set_pixel(0, 10, 20, 30)  # R=10, G=20, B=30
        # 内部: [G=20, R=10, B=30]
        self.assertEqual(led._buf[0], [20, 10, 30])

    def test_set_pixel_invalid(self):
        """无效索引应失败"""
        led = WS2812(4)
        led.init()
        self.assertFalse(led.set_pixel(-1, 0, 0, 0))
        self.assertFalse(led.set_pixel(4, 0, 0, 0))

    def test_get_pixel_invalid(self):
        """无效索引返回全0"""
        led = WS2812(4)
        r, g, b = led.get_pixel(-1)
        self.assertEqual((r, g, b), (0, 0, 0))


class TestWS2812V2HSV(unittest.TestCase):
    """HSV颜色测试"""

    def test_hsv_red(self):
        """HSV(0,255,255) → RGB(255,0,0)"""
        led = WS2812(1)
        led.set_pixel_hsv(0, 0, 255, 255)
        r, g, b = led.get_pixel(0)
        self.assertEqual(r, 255)
        self.assertEqual(g, 0)
        self.assertEqual(b, 0)

    def test_hsv_green(self):
        """HSV(120,255,255) → RGB(0,255,0)"""
        led = WS2812(1)
        led.set_pixel_hsv(0, 120, 255, 255)
        r, g, b = led.get_pixel(0)
        self.assertEqual(r, 0)
        self.assertEqual(g, 255)
        self.assertEqual(b, 0)

    def test_hsv_blue(self):
        """HSV(240,255,255) → RGB(0,0,255)"""
        led = WS2812(1)
        led.set_pixel_hsv(0, 240, 255, 255)
        r, g, b = led.get_pixel(0)
        self.assertEqual(r, 0)
        self.assertEqual(g, 0)
        self.assertEqual(b, 255)

    def test_hsv_invalid_idx(self):
        """无效索引应失败"""
        led = WS2812(4)
        self.assertFalse(led.set_pixel_hsv(-1, 0, 255, 255))

    def test_hsv_wrap_360(self):
        """色相超过360应取模"""
        led = WS2812(1)
        led.set_pixel_hsv(0, 360, 255, 255)
        r, g, b = led.get_pixel(0)
        self.assertEqual(r, 255)  # 360° = 0° = 红色


class TestWS2812V2Fill(unittest.TestCase):
    """填充测试"""

    def test_fill_all(self):
        """填充所有LED"""
        led = WS2812(8)
        led.init()
        led.fill(255, 0, 0)
        for i in range(8):
            r, g, b = led.get_pixel(i)
            self.assertEqual(r, 255)
            self.assertEqual(g, 0)
            self.assertEqual(b, 0)

    def test_clear(self):
        """清除所有LED"""
        led = WS2812(8)
        led.init()
        led.fill(255, 255, 255)
        led.clear()
        for i in range(8):
            r, g, b = led.get_pixel(i)
            self.assertEqual((r, g, b), (0, 0, 0))


class TestWS2812V2Brightness(unittest.TestCase):
    """亮度测试"""

    def test_set_brightness(self):
        """设置亮度"""
        led = WS2812(4)
        led.set_brightness(128)
        self.assertEqual(led._brightness, 128)

    def test_brightness_mask(self):
        """亮度应被0xFF掩码"""
        led = WS2812(4)
        led.set_brightness(300)
        self.assertEqual(led._brightness, 300 & 0xFF)


class TestWS2812V2Show(unittest.TestCase):
    """发送测试"""

    def test_show_success(self):
        """初始化后show成功"""
        led = WS2812(8)
        led.init()
        self.assertTrue(led.show())

    def test_show_not_initialized(self):
        """未初始化show应失败"""
        led = WS2812(8)
        self.assertFalse(led.show())


class TestWS2812V2Constants(unittest.TestCase):
    """常量一致性"""

    def test_constants(self):
        self.assertEqual(WS2812_MAX_LEDS, 256)
        self.assertEqual(WS2812_RESET_US, 50)


if __name__ == '__main__':
    unittest.main()
