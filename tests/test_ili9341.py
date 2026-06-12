# -*- coding: utf-8 -*-
"""
test_ili9341_v2.py - ILI9341 TFT显示测试 V2
==============================================
测试内容：
  1. SPI初始化与命令序列
  2. 240x320 RGB565帧缓冲
  3. 显示开/关、睡眠/唤醒
  4. 像素读写
  5. RGB565颜色转换
  6. 旋转方向(0-3, BGR色彩空间)
  7. 绘图API(矩形/圆/线)
  8. 颜色反转
  9. 窗口设置(CASET/RASET)
  10. 亮度与读像素(RAMRD)

使用 wrappers.py 的 ILI9341 类
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    ILI9341, ILI9341_WIDTH, ILI9341_HEIGHT,
    ILI9341_SWRESET, ILI9341_SLPOUT, ILI9341_DISPON, ILI9341_DISPOFF,
    ILI9341_CASET, ILI9341_RASET, ILI9341_MADCTL, ILI9341_COLMOD,
    ILI9341_PWCTR1, ILI9341_VMCTR1, ILI9341_RAMRD,
)


class TestILI9341V2(unittest.TestCase):
    """ILI9341 TFT显示V2 — 基于wrappers.py包装层"""

    def setUp(self):
        """每个测试前初始化"""
        self.tft = ILI9341()
        self.tft.init()

    # ── 初始化 ──────────────────────────────────

    def test_init_dimensions(self):
        """初始化尺寸240x320"""
        self.assertEqual(self.tft.width, ILI9341_WIDTH)
        self.assertEqual(self.tft.height, ILI9341_HEIGHT)

    def test_init_display_off(self):
        """初始化后显示默认关闭"""
        self.assertFalse(self.tft.is_on())

    def test_init_cmd_sequence(self):
        """初始化命令序列包含SWRESET→SLPOUT→电源设置"""
        cmds = self.tft.get_cmd_log()
        cmd_codes = [c[1] for c in cmds if c[0] == 'cmd']
        self.assertIn(ILI9341_SWRESET, cmd_codes)
        self.assertIn(ILI9341_SLPOUT, cmd_codes)
        self.assertIn(ILI9341_PWCTR1, cmd_codes)
        self.assertIn(ILI9341_VMCTR1, cmd_codes)
        self.assertIn(ILI9341_MADCTL, cmd_codes)
        self.assertIn(ILI9341_COLMOD, cmd_codes)

    def test_init_framebuffer_zero(self):
        """初始化后帧缓冲全黑"""
        fb = self.tft.get_framebuffer()
        self.assertTrue(all(b == 0 for b in fb))

    def test_init_not_sleeping(self):
        """初始化后不在睡眠"""
        self.assertFalse(self.tft.is_sleeping())

    # ── 显示控制 ──────────────────────────────────

    def test_display_on_off(self):
        """显示开关"""
        self.tft.display_on()
        self.assertTrue(self.tft.is_on())
        self.tft.display_off()
        self.assertFalse(self.tft.is_on())

    def test_sleep_wake(self):
        """睡眠唤醒"""
        self.tft.sleep()
        self.assertTrue(self.tft.is_sleeping())
        self.tft.wake()
        self.assertFalse(self.tft.is_sleeping())

    # ── 像素操作 ──────────────────────────────────

    def test_pixel_write_read(self):
        """像素写读"""
        red = ILI9341.color565(255, 0, 0)
        self.tft.set_pixel(120, 160, red)
        self.assertEqual(self.tft.get_pixel(120, 160), red)

    def test_pixel_all_colors(self):
        """多种颜色写读"""
        colors = [
            ILI9341.color565(255, 0, 0),     # 红
            ILI9341.color565(0, 255, 0),     # 绿
            ILI9341.color565(0, 0, 255),     # 蓝
            ILI9341.color565(255, 255, 0),   # 黄
            ILI9341.color565(0, 255, 255),   # 青
            ILI9341.color565(255, 0, 255),   # 品红
            ILI9341.color565(255, 255, 255), # 白
            0x0000,                           # 黑
        ]
        for i, c in enumerate(colors):
            self.tft.set_pixel(i * 10, 0, c)
            self.assertEqual(self.tft.get_pixel(i * 10, 0), c)

    def test_pixel_out_of_bounds(self):
        """越界像素安全"""
        self.tft.set_pixel(-1, 0, 0xFFFF)
        self.tft.set_pixel(ILI9341_WIDTH, 0, 0xFFFF)
        self.tft.set_pixel(0, ILI9341_HEIGHT, 0xFFFF)
        self.assertEqual(self.tft.get_pixel(-1, 0), 0)
        self.assertEqual(self.tft.get_pixel(ILI9341_WIDTH, 0), 0)

    def test_read_pixel_api(self):
        """read_pixel接口(模拟RAMRD)"""
        green = ILI9341.color565(0, 255, 0)
        self.tft.set_pixel(50, 50, green)
        self.assertEqual(self.tft.read_pixel(50, 50), green)

    # ── 颜色转换 ──────────────────────────────────

    def test_color565_red(self):
        """RGB565纯红"""
        self.assertEqual(ILI9341.color565(255, 0, 0), 0xF800)

    def test_color565_green(self):
        """RGB565纯绿"""
        self.assertEqual(ILI9341.color565(0, 255, 0), 0x07E0)

    def test_color565_blue(self):
        """RGB565纯蓝"""
        self.assertEqual(ILI9341.color565(0, 0, 255), 0x001F)

    def test_color565_white_black(self):
        """RGB565黑白"""
        self.assertEqual(ILI9341.color565(255, 255, 255), 0xFFFF)
        self.assertEqual(ILI9341.color565(0, 0, 0), 0x0000)

    # ── 填充 ──────────────────────────────────

    def test_clear(self):
        """清屏"""
        self.tft.fill(0xFFFF)
        self.tft.clear()
        self.assertEqual(self.tft.get_pixel(120, 160), 0)

    def test_fill_solid(self):
        """全屏填充"""
        blue = ILI9341.color565(0, 0, 255)
        self.tft.fill(blue)
        for x, y in [(0, 0), (120, 160), (239, 319)]:
            self.assertEqual(self.tft.get_pixel(x, y), blue)

    def test_fill_rect(self):
        """矩形填充"""
        yellow = ILI9341.color565(255, 255, 0)
        self.tft.fill_rect(20, 20, 100, 50, yellow)
        self.assertEqual(self.tft.get_pixel(20, 20), yellow)
        self.assertEqual(self.tft.get_pixel(119, 69), yellow)
        self.assertEqual(self.tft.get_pixel(19, 20), 0)

    # ── 图形绘制 ──────────────────────────────────

    def test_draw_rect_border(self):
        """矩形边框"""
        white = 0xFFFF
        self.tft.draw_rect(10, 10, 80, 60, white)
        self.assertEqual(self.tft.get_pixel(10, 10), white)
        self.assertEqual(self.tft.get_pixel(89, 69), white)
        self.assertEqual(self.tft.get_pixel(50, 40), 0)  # 内部空

    def test_draw_hline(self):
        """水平线"""
        red = 0xF800
        self.tft.draw_hline(0, 100, 200, red)
        for x in range(200):
            self.assertEqual(self.tft.get_pixel(x, 100), red)

    def test_draw_vline(self):
        """垂直线"""
        green = 0x07E0
        self.tft.draw_vline(100, 0, 200, green)
        for y in range(200):
            self.assertEqual(self.tft.get_pixel(100, y), green)

    def test_draw_circle(self):
        """画圆"""
        cyan = ILI9341.color565(0, 255, 255)
        self.tft.draw_circle(120, 160, 40, cyan)
        self.assertEqual(self.tft.get_pixel(120, 120), cyan)  # 顶部
        self.assertEqual(self.tft.get_pixel(120, 200), cyan)  # 底部

    def test_draw_line(self):
        """Bresenham画线"""
        magenta = ILI9341.color565(255, 0, 255)
        self.tft.draw_line(0, 0, 100, 50, magenta)
        self.assertEqual(self.tft.get_pixel(0, 0), magenta)
        self.assertEqual(self.tft.get_pixel(100, 50), magenta)

    # ── 反转 ──────────────────────────────────

    def test_invert_on_off(self):
        """反转开关"""
        self.tft.invert(True)
        self.assertTrue(self.tft.is_inverted())
        self.tft.invert(False)
        self.assertFalse(self.tft.is_inverted())

    def test_invert_affects_pixel(self):
        """反转影响像素值"""
        self.tft.invert(True)
        red = 0xF800
        self.tft.set_pixel(50, 50, red)
        self.assertEqual(self.tft.get_pixel(50, 50), (~red) & 0xFFFF)

    # ── 旋转 ──────────────────────────────────

    def test_rotation_0(self):
        """旋转0: 竖屏"""
        self.tft.set_rotation(0)
        self.assertEqual(self.tft.width, 240)
        self.assertEqual(self.tft.height, 320)

    def test_rotation_1(self):
        """旋转1: 横屏"""
        self.tft.set_rotation(1)
        self.assertEqual(self.tft.width, 320)
        self.assertEqual(self.tft.height, 240)

    def test_rotation_2(self):
        """旋转2: 竖屏翻转"""
        self.tft.set_rotation(2)
        self.assertEqual(self.tft.width, 240)
        self.assertEqual(self.tft.height, 320)

    def test_rotation_3(self):
        """旋转3: 横屏翻转"""
        self.tft.set_rotation(3)
        self.assertEqual(self.tft.width, 320)
        self.assertEqual(self.tft.height, 240)

    def test_rotation_wraps(self):
        """旋转值循环"""
        self.tft.set_rotation(4)
        self.assertEqual(self.tft.get_rotation(), 0)

    # ── 窗口设置 ──────────────────────────────────

    def test_set_window(self):
        """窗口设置生成CASET/RASET命令"""
        self.tft.set_window(10, 10, 100, 100)
        cmds = [c[1] for c in self.tft.get_cmd_log()]
        self.assertIn(ILI9341_CASET, cmds)
        self.assertIn(ILI9341_RASET, cmds)

    # ── 亮度 ──────────────────────────────────

    def test_brightness(self):
        """亮度调节"""
        self.tft.set_brightness(100)
        self.assertEqual(self.tft.get_brightness(), 100)

    def test_brightness_clamp(self):
        """亮度限幅"""
        self.tft.set_brightness(-5)
        self.assertEqual(self.tft.get_brightness(), 0)
        self.tft.set_brightness(300)
        self.assertEqual(self.tft.get_brightness(), 255)

    # ── 帧缓冲 ──────────────────────────────────

    def test_framebuffer_size(self):
        """帧缓冲大小240*320*2"""
        self.assertEqual(self.tft.get_framebuffer_size(), 240 * 320 * 2)

    def test_framebuffer_bytes_type(self):
        """帧缓冲返回bytes"""
        self.assertIsInstance(self.tft.get_framebuffer(), bytes)


if __name__ == '__main__':
    unittest.main()
