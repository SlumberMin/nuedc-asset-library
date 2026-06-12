#!/usr/bin/env python3
"""
LED点阵V2测试 — MAX7219 8×8点阵显示
覆盖: MAX7219初始化、像素操作、整行设置、
      级联配置、亮度控制、测试模式、
      字符/图案显示、缓冲区管理
对应C源文件: 02_mspm0g3507/drivers/max7219.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    MAX7219,
    MAX7219_INTENSITY_MAX, MAX7219_ROWS, MAX7219_COLS,
)


# ═══════════════════════════════════════════════════════════════
#  预定义字符图案（8×8位图）
# ═══════════════════════════════════════════════════════════════

# 笑脸图案
SMILEY = [
    0x3C,  #   ####  
    0x42,  #  #    # 
    0xA5,  # # #  # #
    0x81,  # #      #
    0xA5,  # # #  # #
    0x99,  # #  ##  #
    0x42,  #  #    # 
    0x3C,  #   ####  
]

# 心形图案
HEART = [
    0x00,  #         
    0x66,  #  ##  ## 
    0xFF,  # ########
    0xFF,  # ########
    0xFF,  # ########
    0x7E,  #  ###### 
    0x3C,  #   ####  
    0x18,  #    ##   
]

# 箭头图案
ARROW_UP = [
    0x18,  #    ##   
    0x3C,  #   ####  
    0x7E,  #  ###### 
    0xFF,  # ########
    0x18,  #    ##   
    0x18,  #    ##   
    0x18,  #    ##   
    0x18,  #    ##   
]


class LedMatrixV2:
    """LED点阵V2 — MAX7219 8×8矩阵显示控制

    功能:
    - MAX7219驱动8×8 LED点阵
    - 支持单/多芯片级联
    - 像素级控制和整行/整列操作
    - 预定义图案显示
    - 亮度和显示模式管理
    """

    def __init__(self, num_chips=1):
        self.driver = MAX7219(num_cascaded=num_chips)
        self.num_chips = num_chips

    def init(self):
        """初始化LED点阵"""
        self.driver.init()

    def show_pattern(self, pattern, chip_index=0):
        """显示8字节图案

        参数: pattern - 8个字节的列表，每字节对应一行
        """
        if len(pattern) != MAX7219_ROWS:
            return False
        for row, data in enumerate(pattern):
            self.driver.set_row(row, data, chip_index)
        return True

    def scroll_left(self, chip_index=0):
        """整个显示向左滚动一位"""
        for row in range(MAX7219_ROWS):
            data = self.driver.get_row(row, chip_index)
            # 左移一位，高位丢失
            data = (data << 1) & 0xFF
            self.driver.set_row(row, data, chip_index)

    def scroll_right(self, chip_index=0):
        """整个显示向右滚动一位"""
        for row in range(MAX7219_ROWS):
            data = self.driver.get_row(row, chip_index)
            data = (data >> 1) & 0xFF
            self.driver.set_row(row, data, chip_index)

    def invert(self, chip_index=0):
        """反转显示（取反）"""
        for row in range(MAX7219_ROWS):
            data = self.driver.get_row(row, chip_index)
            self.driver.set_row(row, (~data) & 0xFF, chip_index)

    def draw_border(self, chip_index=0):
        """绘制边框"""
        self.driver.set_row(0, 0xFF, chip_index)   # 顶部
        self.driver.set_row(7, 0xFF, chip_index)   # 底部
        for row in range(1, 7):
            self.driver.set_pixel(row, 0, True, chip_index)
            self.driver.set_pixel(row, 7, True, chip_index)

    def count_leds_on(self, chip_index=0):
        """统计当前点亮的LED数量"""
        count = 0
        for row in range(MAX7219_ROWS):
            data = self.driver.get_row(row, chip_index)
            count += bin(data).count('1')
        return count

    def fill(self, on=True, chip_index=0):
        """全亮或全灭"""
        val = 0xFF if on else 0x00
        for row in range(MAX7219_ROWS):
            self.driver.set_row(row, val, chip_index)


class TestMAX7219Init(unittest.TestCase):
    """MAX7219初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        matrix = LedMatrixV2()
        matrix.init()
        self.assertTrue(matrix.driver.initialized)

    def test_init_not_shutdown(self):
        """初始化后不应处于关断模式"""
        matrix = LedMatrixV2()
        matrix.init()
        self.assertFalse(matrix.driver.shutdown_mode)

    def test_default_intensity(self):
        """默认亮度为7"""
        matrix = LedMatrixV2()
        matrix.init()
        self.assertEqual(matrix.driver.intensity, 7)

    def test_default_scan_limit(self):
        """默认扫描全部8行"""
        matrix = LedMatrixV2()
        matrix.init()
        self.assertEqual(matrix.driver.scan_limit, 7)


class TestPixelOperations(unittest.TestCase):
    """像素操作测试"""

    def setUp(self):
        self.matrix = LedMatrixV2()
        self.matrix.init()

    def test_set_pixel_on(self):
        """点亮单个像素"""
        result = self.matrix.driver.set_pixel(3, 5, True)
        self.assertTrue(result)
        self.assertTrue(self.matrix.driver.get_pixel(3, 5))

    def test_set_pixel_off(self):
        """熄灭单个像素"""
        self.matrix.driver.set_pixel(3, 5, True)
        result = self.matrix.driver.set_pixel(3, 5, False)
        self.assertTrue(result)
        self.assertFalse(self.matrix.driver.get_pixel(3, 5))

    def test_pixel_out_of_range(self):
        """越界像素返回False"""
        self.assertFalse(self.matrix.driver.set_pixel(8, 0, True))
        self.assertFalse(self.matrix.driver.set_pixel(0, 8, True))
        self.assertFalse(self.matrix.driver.set_pixel(-1, 0, True))

    def test_pixel_before_init(self):
        """初始化前设置像素返回False"""
        m = MAX7219()
        self.assertFalse(m.set_pixel(0, 0, True))

    def test_multiple_pixels(self):
        """设置多个像素"""
        positions = [(0, 0), (1, 1), (2, 2), (7, 7)]
        for r, c in positions:
            self.matrix.driver.set_pixel(r, c, True)
        for r, c in positions:
            self.assertTrue(self.matrix.driver.get_pixel(r, c))


class TestRowOperations(unittest.TestCase):
    """行操作测试"""

    def setUp(self):
        self.matrix = LedMatrixV2()
        self.matrix.init()

    def test_set_row(self):
        """设置整行数据"""
        self.assertTrue(self.matrix.driver.set_row(0, 0xAA))
        self.assertEqual(self.matrix.driver.get_row(0), 0xAA)

    def test_set_row_all_patterns(self):
        """设置不同行模式"""
        patterns = [0x00, 0xFF, 0x55, 0xAA, 0x18, 0x7E]
        for i, pat in enumerate(patterns):
            self.matrix.driver.set_row(i, pat)
        for i, pat in enumerate(patterns):
            self.assertEqual(self.matrix.driver.get_row(i), pat)

    def test_row_out_of_range(self):
        """越界行返回False"""
        self.assertFalse(self.matrix.driver.set_row(8, 0xFF))


class TestBrightnessAndMode(unittest.TestCase):
    """亮度和模式测试"""

    def setUp(self):
        self.matrix = LedMatrixV2()
        self.matrix.init()

    def test_brightness_range(self):
        """亮度范围0-15"""
        for level in range(16):
            self.matrix.driver.set_intensity(level)
            self.assertEqual(self.matrix.driver.intensity, level)

    def test_brightness_clamp(self):
        """亮度超限自动限幅"""
        self.matrix.driver.set_intensity(20)
        self.assertEqual(self.matrix.driver.intensity, MAX7219_INTENSITY_MAX)
        self.matrix.driver.set_intensity(-5)
        self.assertEqual(self.matrix.driver.intensity, 0)

    def test_test_mode(self):
        """测试模式"""
        self.matrix.driver.set_test_mode(True)
        self.assertTrue(self.matrix.driver.test_mode)
        self.matrix.driver.set_test_mode(False)
        self.assertFalse(self.matrix.driver.test_mode)

    def test_shutdown(self):
        """关断模式"""
        self.matrix.driver.set_shutdown(True)
        self.assertTrue(self.matrix.driver.shutdown_mode)
        self.matrix.driver.set_shutdown(False)
        self.assertFalse(self.matrix.driver.shutdown_mode)


class TestPatternDisplay(unittest.TestCase):
    """图案显示测试"""

    def setUp(self):
        self.matrix = LedMatrixV2()
        self.matrix.init()

    def test_show_smiley(self):
        """显示笑脸图案"""
        self.assertTrue(self.matrix.show_pattern(SMILEY))
        # 验证第一行: 0x3C = 00111100
        self.assertEqual(self.matrix.driver.get_row(0), 0x3C)
        self.assertEqual(self.matrix.driver.get_row(7), 0x3C)

    def test_show_heart(self):
        """显示心形图案"""
        self.assertTrue(self.matrix.show_pattern(HEART))
        self.assertEqual(self.matrix.driver.get_row(1), 0x66)

    def test_show_arrow(self):
        """显示箭头图案"""
        self.assertTrue(self.matrix.show_pattern(ARROW_UP))
        self.assertEqual(self.matrix.driver.get_row(3), 0xFF)

    def test_invalid_pattern_length(self):
        """无效长度图案返回False"""
        self.assertFalse(self.matrix.show_pattern([0x00] * 5))


class TestMatrixOperations(unittest.TestCase):
    """矩阵高级操作测试"""

    def setUp(self):
        self.matrix = LedMatrixV2()
        self.matrix.init()
        self.matrix.show_pattern(SMILEY)

    def test_clear(self):
        """清除显示"""
        self.matrix.driver.clear()
        for row in range(MAX7219_ROWS):
            self.assertEqual(self.matrix.driver.get_row(row), 0x00)

    def test_fill_on(self):
        """全亮"""
        self.matrix.fill(True)
        for row in range(MAX7219_ROWS):
            self.assertEqual(self.matrix.driver.get_row(row), 0xFF)

    def test_fill_off(self):
        """全灭"""
        self.matrix.fill(False)
        for row in range(MAX7219_ROWS):
            self.assertEqual(self.matrix.driver.get_row(row), 0x00)

    def test_invert(self):
        """反转显示"""
        original = [self.matrix.driver.get_row(r) for r in range(MAX7219_ROWS)]
        self.matrix.invert()
        for row in range(MAX7219_ROWS):
            self.assertEqual(self.matrix.driver.get_row(row), (~original[row]) & 0xFF)

    def test_scroll_left(self):
        """左移一位"""
        self.matrix.driver.clear()
        self.matrix.driver.set_row(0, 0x08)  # 00001000
        self.matrix.scroll_left()
        self.assertEqual(self.matrix.driver.get_row(0), 0x10)  # 00010000

    def test_scroll_right(self):
        """右移一位"""
        self.matrix.driver.clear()
        self.matrix.driver.set_row(0, 0x10)  # 00010000
        self.matrix.scroll_right()
        self.assertEqual(self.matrix.driver.get_row(0), 0x08)  # 00001000

    def test_draw_border(self):
        """绘制边框"""
        self.matrix.driver.clear()
        self.matrix.draw_border()
        # 顶部和底部全亮
        self.assertEqual(self.matrix.driver.get_row(0), 0xFF)
        self.assertEqual(self.matrix.driver.get_row(7), 0xFF)
        # 侧面
        self.assertTrue(self.matrix.driver.get_pixel(3, 0))
        self.assertTrue(self.matrix.driver.get_pixel(3, 7))
        # 中间内部应为空
        self.assertFalse(self.matrix.driver.get_pixel(3, 3))

    def test_count_leds_on(self):
        """统计点亮LED数量"""
        self.matrix.driver.clear()
        self.assertEqual(self.matrix.count_leds_on(), 0)
        self.matrix.driver.set_pixel(0, 0, True)
        self.matrix.driver.set_pixel(1, 1, True)
        self.assertEqual(self.matrix.count_leds_on(), 2)
        # 全亮 = 64
        self.matrix.fill(True)
        self.assertEqual(self.matrix.count_leds_on(), 64)


class TestCascade(unittest.TestCase):
    """级联芯片测试"""

    def test_two_chip_init(self):
        """双芯片级联初始化"""
        matrix = LedMatrixV2(num_chips=2)
        matrix.init()
        self.assertEqual(matrix.driver.num_cascaded, 2)

    def test_independent_display(self):
        """两个芯片独立显示"""
        matrix = LedMatrixV2(num_chips=2)
        matrix.init()
        # 芯片0显示笑脸
        matrix.show_pattern(SMILEY, chip_index=0)
        # 芯片1显示心形
        matrix.show_pattern(HEART, chip_index=1)
        self.assertEqual(matrix.driver.get_row(0, 0), 0x3C)  # 笑脸第0行
        self.assertEqual(matrix.driver.get_row(1, 1), 0x66)  # 心形第1行

    def test_invalid_chip_index(self):
        """无效芯片索引"""
        matrix = LedMatrixV2(num_chips=1)
        matrix.init()
        self.assertFalse(matrix.driver.set_pixel(0, 0, True, chip_index=5))
        self.assertEqual(matrix.driver.get_row(0, chip_index=5), 0)


class TestSPILog(unittest.TestCase):
    """SPI操作日志测试"""

    def test_spi_log_recorded(self):
        """SPI操作被记录"""
        matrix = LedMatrixV2()
        matrix.init()
        matrix.driver.set_pixel(0, 0, True)
        # 应有初始化 + set_pixel的SPI记录
        self.assertGreater(len(matrix.driver.spi_log), 0)

    def test_spi_log_content(self):
        """SPI日志内容正确"""
        matrix = LedMatrixV2()
        matrix.init()
        matrix.driver.clear()  # 清空之前的记录
        initial_count = len(matrix.driver.spi_log)
        matrix.driver.set_pixel(2, 3, True)
        # 应新增一条记录
        self.assertEqual(len(matrix.driver.spi_log), initial_count + 1)
        chip, reg, data = matrix.driver.spi_log[-1]
        self.assertEqual(chip, 0)
        # 寄存器地址应为DIGIT0 + row
        self.assertEqual(reg, 0x01 + 2)  # DIGIT0=0x01, row=2


if __name__ == '__main__':
    unittest.main()
