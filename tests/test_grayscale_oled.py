# -*- coding: utf-8 -*-
"""
test_grayscale_oled_v2.py - 灰度OLED测试 V2
=============================================
测试内容：
  1. Grayscale灰度传感器初始化
  2. 单通道/全通道读取
  3. 白线检测
  4. LCD1602显示
  5. MAX7219 LED矩阵
  6. TM1637数码管
  7. 多显示器协同
  8. 数据缓冲记录

使用 wrappers.py 封装的 Grayscale、LCD1602、MAX7219、TM1637、RingBuffer
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wrappers import Grayscale, LCD1602, MAX7219, TM1637, RingBuffer

# ---- 测试辅助函数 ----
def assert_close(actual, expected, tolerance=0.01, msg=""):
    if abs(actual - expected) > tolerance:
        raise AssertionError(f"{msg}: 期望 {expected}±{tolerance}, 实际 {actual}")

def assert_in_range(value, min_val, max_val, msg=""):
    if value < min_val or value > max_val:
        raise AssertionError(f"{msg}: {value} 不在 [{min_val}, {max_val}] 范围内")

def run_test(test_func, test_name=""):
    try:
        test_func()
        print(f"  [通过] {test_name}")
        return True
    except AssertionError as e:
        print(f"  [失败] {test_name}: {e}")
        return False
    except Exception as e:
        print(f"  [错误] {test_name}: {type(e).__name__}: {e}")
        return False


class GrayscaleDisplaySystem:
    """灰度传感器 + 多显示器系统"""
    def __init__(self):
        self.gs = Grayscale(); self.gs.init()
        self.lcd = LCD1602(); self.lcd.init()
        self.matrix = MAX7219(); self.matrix.init()
        self.digit = TM1637(); self.digit.init()
        self.buffer = RingBuffer(256)
        self.read_count = 0

    def read_grayscale(self, ch=None):
        if ch is not None:
            return self.gs.read(ch)
        return self.gs.read_all()

    def count_white(self):
        return self.gs.count_white()

    def display_lcd(self, row, text):
        self.lcd.set_cursor(row, 0)
        self.lcd.write_string(text)

    def display_matrix_row(self, row, data):
        self.matrix.set_row(row, data)

    def display_digit(self, value):
        self.digit.display_number(value)


def test_grayscale_init():
    """灰度传感器初始化"""
    g = Grayscale(); g.init()
    assert g.initialized

def test_grayscale_read():
    """灰度单通道读取"""
    g = Grayscale(); g.init()
    val = g.read(0)
    assert isinstance(val, (int, float))

def test_grayscale_read_all():
    """灰度全通道读取"""
    g = Grayscale(); g.init()
    vals = g.read_all()
    # read_all returns raw sensor data (int or list depending on implementation)
    assert vals is not None

def test_grayscale_channels():
    """灰度多通道"""
    g = Grayscale(); g.init()
    for ch in range(8):
        val = g.read(ch)
        assert val is not None

def test_grayscale_set_channel():
    """灰度通道设置"""
    g = Grayscale(); g.init()
    g.set_channel(0, 128)
    val = g.read(0)
    assert isinstance(val, (int, float))

def test_grayscale_count_white():
    """白线检测"""
    g = Grayscale(); g.init()
    count = g.count_white()
    assert isinstance(count, int) and count >= 0

def test_lcd_init():
    """LCD1602初始化"""
    lcd = LCD1602(); lcd.init()
    assert lcd.initialized

def test_lcd_write_string():
    """LCD显示字符串"""
    lcd = LCD1602(); lcd.init()
    lcd.set_cursor(0, 0)
    lcd.write_string("Hello")
    lcd.set_cursor(1, 0)
    lcd.write_string("World")

def test_lcd_print_line():
    """LCD行打印"""
    lcd = LCD1602(); lcd.init()
    lcd.print_line(0, "Line0")
    lcd.print_line(1, "Line1")
    assert lcd.get_line(0).startswith("Line0")

def test_lcd_clear():
    """LCD清屏"""
    lcd = LCD1602(); lcd.init()
    lcd.write_string("Test")
    lcd.clear()
    assert lcd.get_line(0).strip() == ""

def test_lcd_display_control():
    """LCD显示控制"""
    lcd = LCD1602(); lcd.init()
    lcd.display_on_off(True)
    lcd.cursor_on_off(True)
    lcd.blink_on_off(True)
    assert lcd.display_on
    assert lcd.cursor_on
    assert lcd.blink_on

def test_led_matrix_init():
    """MAX7219初始化"""
    m = MAX7219(); m.init()
    assert m.initialized

def test_led_matrix_set_row():
    """LED矩阵行设置"""
    m = MAX7219(); m.init()
    for row in range(8):
        m.set_row(row, 0xFF)

def test_led_matrix_intensity():
    """LED矩阵亮度"""
    m = MAX7219(); m.init()
    m.set_intensity(15)

def test_led_matrix_pixel():
    """LED矩阵像素"""
    m = MAX7219(); m.init()
    m.set_pixel(0, 0, True)
    assert m.get_pixel(0, 0) == True
    m.set_pixel(0, 0, False)
    assert m.get_pixel(0, 0) == False

def test_led_matrix_flush():
    """LED矩阵刷新"""
    m = MAX7219(); m.init()
    m.set_row(0, 0xAA)
    m.flush()

def test_digit_init():
    """TM1637初始化"""
    t = TM1637(); t.init()
    assert t.initialized

def test_digit_display_number():
    """数码管数字显示"""
    t = TM1637(); t.init()
    t.display_number(1234)
    t.display_number(0)

def test_digit_display_string():
    """数码管字符串显示"""
    t = TM1637(); t.init()
    t.display_string("ABCD")

def test_digit_brightness():
    """数码管亮度"""
    t = TM1637(); t.init()
    t.set_brightness(7)
    assert t.brightness == 7

def test_digit_colon():
    """数码管冒号"""
    t = TM1637(); t.init()
    t.set_colon(True)
    t.set_colon(False)

def test_system_init():
    """系统初始化"""
    sys0 = GrayscaleDisplaySystem()
    assert sys0.read_count == 0

def test_system_grayscale():
    """系统灰度读取"""
    sys0 = GrayscaleDisplaySystem()
    val = sys0.read_grayscale(0)
    assert val is not None

def test_system_lcd():
    """系统LCD显示"""
    sys0 = GrayscaleDisplaySystem()
    sys0.display_lcd(0, "CH0: 128")
    sys0.display_lcd(1, "CH1: 200")

def test_system_matrix():
    """系统LED矩阵"""
    sys0 = GrayscaleDisplaySystem()
    sys0.display_matrix_row(0, 0xAA)
    sys0.display_matrix_row(7, 0x55)

def test_system_digit():
    """系统数码管"""
    sys0 = GrayscaleDisplaySystem()
    sys0.display_digit(42)

def test_buffer():
    """数据缓冲"""
    sys0 = GrayscaleDisplaySystem()
    for i in range(8):
        sys0.buffer.put_byte(i)
    assert sys0.buffer.used() == 8


def main():
    print("=" * 60)
    print("  灰度OLED显示系统测试 V2")
    print("=" * 60)
    tests = [
        (test_grayscale_init, "灰度初始化"), (test_grayscale_read, "灰度读取"),
        (test_grayscale_read_all, "全通道读取"), (test_grayscale_channels, "多通道"),
        (test_grayscale_set_channel, "通道设置"), (test_grayscale_count_white, "白线检测"),
        (test_lcd_init, "LCD初始化"), (test_lcd_write_string, "LCD显示"),
        (test_lcd_print_line, "LCD行打印"), (test_lcd_clear, "LCD清屏"),
        (test_lcd_display_control, "LCD显示控制"),
        (test_led_matrix_init, "LED矩阵初始化"), (test_led_matrix_set_row, "LED矩阵行设置"),
        (test_led_matrix_intensity, "LED矩阵亮度"), (test_led_matrix_pixel, "LED矩阵像素"),
        (test_led_matrix_flush, "LED矩阵刷新"),
        (test_digit_init, "数码管初始化"), (test_digit_display_number, "数码管数字"),
        (test_digit_display_string, "数码管字符串"), (test_digit_brightness, "数码管亮度"),
        (test_digit_colon, "数码管冒号"),
        (test_system_init, "系统初始化"), (test_system_grayscale, "系统灰度读取"),
        (test_system_lcd, "系统LCD显示"), (test_system_matrix, "系统LED矩阵"),
        (test_system_digit, "系统数码管"), (test_buffer, "数据缓冲"),
    ]
    passed = failed = 0
    for func, name in tests:
        if run_test(func, name): passed += 1
        else: failed += 1
    print("-" * 60)
    print(f"  结果: {passed} 通过, {failed} 失败, 共 {passed + failed} 项")
    print("=" * 60)
    return failed == 0

if __name__ == "__main__":
    sys.exit(0 if main() else 1)
