# -*- coding: utf-8 -*-
"""
test_st7789_v2.py - ST7789 TFT显示测试 V2
============================================
测试内容：
  1. SPI通信初始化与命令序列
  2. 240x320 RGB565帧缓冲
  3. 显示开/关、睡眠/唤醒
  4. 像素写入与读取
  5. RGB888→RGB565颜色转换
  6. 旋转方向(0-3)
  7. 矩形/圆/线绘制
  8. 颜色反转
  9. 窗口设置
  10. 亮度调节

使用 wrappers.py 的 ST7789 类
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    ST7789, ST7789_WIDTH, ST7789_HEIGHT,
    ST7789_SWRESET, ST7789_SLPOUT, ST7789_DISPON, ST7789_DISPOFF,
    ST7789_CASET, ST7789_RASET, ST7789_MADCTL, ST7789_COLMOD,
    ST7789_INVON, ST7789_INVOFF,
    ST7789_MADCTL_RGB, ST7789_MADCTL_MV, ST7789_MADCTL_MX, ST7789_MADCTL_MY,
)


class TestST7789V2(unittest.TestCase):
    """ST7789 TFT显示V2 — 基于wrappers.py包装层"""

    def setUp(self):
        """每个测试前初始化TFT"""
        self.tft = ST7789()
        self.tft.init()

    # ── 初始化测试 ──────────────────────────────────

    def test_init_dimensions(self):
        """初始化后尺寸正确(240x320)"""
        self.assertEqual(self.tft.width, ST7789_WIDTH)
        self.assertEqual(self.tft.height, ST7789_HEIGHT)

    def test_init_display_off(self):
        """初始化后显示默认关闭"""
        self.assertFalse(self.tft.is_on())

    def test_init_not_sleeping(self):
        """初始化后退出睡眠模式"""
        self.assertFalse(self.tft.is_sleeping())

    def test_init_framebuffer_cleared(self):
        """初始化后帧缓冲全0(黑色)"""
        fb = self.tft.get_framebuffer()
        self.assertEqual(len(fb), ST7789_WIDTH * ST7789_HEIGHT * 2)
        self.assertTrue(all(b == 0 for b in fb))

    def test_init_cmd_log_has_reset(self):
        """初始化命令序列包含SWRESET和SLPOUT"""
        cmds = self.tft.get_cmd_log()
        cmd_codes = [c[1] for c in cmds if c[0] == 'cmd']
        self.assertIn(ST7789_SWRESET, cmd_codes)
        self.assertIn(ST7789_SLPOUT, cmd_codes)
        self.assertIn(ST7789_COLMOD, cmd_codes)

    # ── 显示控制 ──────────────────────────────────

    def test_display_on_off(self):
        """显示开关控制"""
        self.tft.display_on()
        self.assertTrue(self.tft.is_on())
        self.tft.display_off()
        self.assertFalse(self.tft.is_on())

    def test_sleep_wake(self):
        """睡眠与唤醒"""
        self.tft.sleep()
        self.assertTrue(self.tft.is_sleeping())
        self.tft.wake()
        self.assertFalse(self.tft.is_sleeping())

    # ── 像素操作 ──────────────────────────────────

    def test_set_get_pixel(self):
        """像素写入与读取"""
        red = self.tft.color565(255, 0, 0)
        self.tft.set_pixel(100, 150, red)
        self.assertEqual(self.tft.get_pixel(100, 150), red)

    def test_pixel_boundary_clipping(self):
        """越界像素不崩溃"""
        white = self.tft.color565(255, 255, 255)
        # 越界写入不应崩溃
        self.tft.set_pixel(-1, 0, white)
        self.tft.set_pixel(0, -1, white)
        self.tft.set_pixel(ST7789_WIDTH, 0, white)
        self.tft.set_pixel(0, ST7789_HEIGHT, white)
        # 越界读取返回0
        self.assertEqual(self.tft.get_pixel(-1, 0), 0)
        self.assertEqual(self.tft.get_pixel(ST7789_WIDTH, 0), 0)

    def test_pixel_corner(self):
        """四角像素"""
        blue = self.tft.color565(0, 0, 255)
        corners = [(0, 0), (ST7789_WIDTH - 1, 0),
                   (0, ST7789_HEIGHT - 1), (ST7789_WIDTH - 1, ST7789_HEIGHT - 1)]
        for x, y in corners:
            self.tft.set_pixel(x, y, blue)
            self.assertEqual(self.tft.get_pixel(x, y), blue)

    # ── 颜色转换 ──────────────────────────────────

    def test_color565_pure_red(self):
        """纯红RGB565"""
        c = ST7789.color565(255, 0, 0)
        self.assertEqual(c, 0xF800)

    def test_color565_pure_green(self):
        """纯绿RGB565"""
        c = ST7789.color565(0, 255, 0)
        self.assertEqual(c, 0x07E0)

    def test_color565_pure_blue(self):
        """纯蓝RGB565"""
        c = ST7789.color565(0, 0, 255)
        self.assertEqual(c, 0x001F)

    def test_color565_white(self):
        """白色RGB565"""
        c = ST7789.color565(255, 255, 255)
        self.assertEqual(c, 0xFFFF)

    def test_color565_black(self):
        """黑色RGB565"""
        c = ST7789.color565(0, 0, 0)
        self.assertEqual(c, 0x0000)

    # ── 清屏与填充 ──────────────────────────────────

    def test_clear(self):
        """清屏"""
        self.tft.fill(0xFFFF)
        self.tft.clear()
        self.assertEqual(self.tft.get_pixel(120, 160), 0)

    def test_fill_solid(self):
        """全屏填充"""
        green = ST7789.color565(0, 255, 0)
        self.tft.fill(green)
        self.assertEqual(self.tft.get_pixel(0, 0), green)
        self.assertEqual(self.tft.get_pixel(120, 160), green)
        self.assertEqual(self.tft.get_pixel(ST7789_WIDTH - 1, ST7789_HEIGHT - 1), green)

    def test_fill_rect(self):
        """矩形填充"""
        yellow = ST7789.color565(255, 255, 0)
        self.tft.fill_rect(10, 10, 50, 30, yellow)
        self.assertEqual(self.tft.get_pixel(10, 10), yellow)
        self.assertEqual(self.tft.get_pixel(59, 39), yellow)
        self.assertEqual(self.tft.get_pixel(9, 10), 0)   # 外部应为黑

    # ── 图形绘制 ──────────────────────────────────

    def test_draw_hline(self):
        """水平线"""
        cyan = ST7789.color565(0, 255, 255)
        self.tft.draw_hline(0, 50, 100, cyan)
        for x in range(100):
            self.assertEqual(self.tft.get_pixel(x, 50), cyan)
        self.assertEqual(self.tft.get_pixel(100, 50), 0)  # 线外

    def test_draw_vline(self):
        """垂直线"""
        magenta = ST7789.color565(255, 0, 255)
        self.tft.draw_vline(50, 0, 100, magenta)
        for y in range(100):
            self.assertEqual(self.tft.get_pixel(50, y), magenta)

    def test_draw_rect(self):
        """矩形边框"""
        white = 0xFFFF
        self.tft.draw_rect(20, 20, 100, 80, white)
        # 四角应有像素
        self.assertEqual(self.tft.get_pixel(20, 20), white)
        self.assertEqual(self.tft.get_pixel(119, 20), white)
        self.assertEqual(self.tft.get_pixel(20, 99), white)
        self.assertEqual(self.tft.get_pixel(119, 99), white)
        # 内部应为空
        self.assertEqual(self.tft.get_pixel(50, 50), 0)

    def test_draw_circle(self):
        """画圆"""
        red = ST7789.color565(255, 0, 0)
        self.tft.draw_circle(120, 160, 50, red)
        # 圆上点应有像素
        self.assertEqual(self.tft.get_pixel(120, 110), red)  # 顶部
        self.assertEqual(self.tft.get_pixel(120, 210), red)  # 底部
        self.assertEqual(self.tft.get_pixel(70, 160), red)   # 左侧
        self.assertEqual(self.tft.get_pixel(170, 160), red)  # 右侧

    def test_draw_line_diagonal(self):
        """对角线"""
        blue = ST7789.color565(0, 0, 255)
        self.tft.draw_line(0, 0, 100, 100, blue)
        # 起点和终点应有像素
        self.assertEqual(self.tft.get_pixel(0, 0), blue)
        self.assertEqual(self.tft.get_pixel(100, 100), blue)
        # 中间点应有像素
        self.assertEqual(self.tft.get_pixel(50, 50), blue)

    # ── 反转 ──────────────────────────────────

    def test_invert_on_off(self):
        """颜色反转开关"""
        self.tft.invert(True)
        self.assertTrue(self.tft.is_inverted())
        self.tft.invert(False)
        self.assertFalse(self.tft.is_inverted())

    def test_invert_affects_pixel(self):
        """反转后像素值取反"""
        self.tft.invert(True)
        red = ST7789.color565(255, 0, 0)  # 0xF800
        self.tft.set_pixel(50, 50, red)
        # 反转后应为 ~0xF800 & 0xFFFF = 0x07FF
        self.assertEqual(self.tft.get_pixel(50, 50), (~red) & 0xFFFF)

    # ── 旋转 ──────────────────────────────────

    def test_rotation_0(self):
        """旋转0: 竖屏(240x320)"""
        self.tft.set_rotation(0)
        self.assertEqual(self.tft.get_rotation(), 0)
        self.assertEqual(self.tft.width, 240)
        self.assertEqual(self.tft.height, 320)

    def test_rotation_1(self):
        """旋转1: 横屏(320x240)"""
        self.tft.set_rotation(1)
        self.assertEqual(self.tft.get_rotation(), 1)
        self.assertEqual(self.tft.width, 320)
        self.assertEqual(self.tft.height, 240)

    def test_rotation_2(self):
        """旋转2: 竖屏翻转(240x320)"""
        self.tft.set_rotation(2)
        self.assertEqual(self.tft.get_rotation(), 2)
        self.assertEqual(self.tft.width, 240)
        self.assertEqual(self.tft.height, 320)

    def test_rotation_3(self):
        """旋转3: 横屏翻转(320x240)"""
        self.tft.set_rotation(3)
        self.assertEqual(self.tft.get_rotation(), 3)
        self.assertEqual(self.tft.width, 320)
        self.assertEqual(self.tft.height, 240)

    def test_rotation_wraps(self):
        """旋转值循环(4→0)"""
        self.tft.set_rotation(4)
        self.assertEqual(self.tft.get_rotation(), 0)

    # ── 窗口设置 ──────────────────────────────────

    def test_set_window(self):
        """设置绘图窗口不崩溃"""
        self.tft.set_window(0, 0, 100, 100)
        cmds = self.tft.get_cmd_log()
        # 应包含CASET和RASET命令
        cmd_codes = [c[1] for c in cmds]
        self.assertIn(ST7789_CASET, cmd_codes)
        self.assertIn(ST7789_RASET, cmd_codes)

    # ── 亮度 ──────────────────────────────────

    def test_brightness(self):
        """亮度调节"""
        self.tft.set_brightness(128)
        self.assertEqual(self.tft.get_brightness(), 128)
        self.tft.set_brightness(0)
        self.assertEqual(self.tft.get_brightness(), 0)
        self.tft.set_brightness(255)
        self.assertEqual(self.tft.get_brightness(), 255)

    def test_brightness_clamp(self):
        """亮度值限幅"""
        self.tft.set_brightness(300)
        self.assertEqual(self.tft.get_brightness(), 255)
        self.tft.set_brightness(-10)
        self.assertEqual(self.tft.get_brightness(), 0)

    # ── 帧缓冲 ──────────────────────────────────

    def test_framebuffer_size(self):
        """帧缓冲大小: 240*320*2 = 153600字节"""
        self.assertEqual(self.tft.get_framebuffer_size(), 240 * 320 * 2)

    def test_framebuffer_readonly(self):
        """帧缓冲返回bytes(只读副本)"""
        fb = self.tft.get_framebuffer()
        self.assertIsInstance(fb, bytes)

    # ── 命令日志 ──────────────────────────────────

    def test_cmd_log_grows(self):
        """命令日志记录操作"""
        initial = len(self.tft.get_cmd_log())
        self.tft.display_on()
        self.tft.invert(True)
        self.assertGreater(len(self.tft.get_cmd_log()), initial)


if __name__ == '__main__':
    unittest.main()
