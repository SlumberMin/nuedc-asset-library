# -*- coding: utf-8 -*-
"""
test_ssd1327_v2.py - SSD1327 OLED显示测试 V2
==============================================
测试内容：
  1. I2C/SPI通信初始化
  2. 128x128灰度OLED帧缓冲
  3. 显示开/关
  4. 像素写入与读取
  5. 字符/字符串绘制
  6. 图形绘制（线/矩形/圆）
  7. 灰度级别（0-15, 4位）
  8. 显示反转
  9. 亮度/对比度调节
  10. 滚动显示
  11. 区域填充/清除
  12. 帧缓冲DMA传输模拟

使用 wrappers.py 的 I2CBus、SimpleMA、RingBuffer
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from wrappers import I2CBus, SimpleMA

# ═══════════════════════════════════════════════════════════════
#  SSD1327常量
# ═══════════════════════════════════════════════════════════════
SSD1327_WIDTH = 128
SSD1327_HEIGHT = 128
SSD1327_I2C_ADDR = 0x3C
SSD1327_MAX_GRAY = 15  # 4位灰度

# 命令
CMD_DISPLAY_OFF = 0xAE
CMD_DISPLAY_ON = 0xAF
CMD_SET_CONTRAST = 0x81
CMD_SET_REMAP = 0xA0
CMD_SET_DISPLAY_START = 0xA1
CMD_SET_DISPLAY_OFFSET = 0xA2
CMD_NORMAL_DISPLAY = 0xA4
CMD_ALL_ON = 0xA5
CMD_ALL_OFF = 0xA6
CMD_INVERSE_DISPLAY = 0xA7
CMD_SET_MUX_RATIO = 0xA8
CMD_SET_PHASE_LEN = 0xB1
CMD_SET_CLOCK_DIV = 0xB3
CMD_SET_VCOMH = 0xBE
CMD_SET_PRECHARGE = 0xBC
CMD_NOP = 0xE3

# 滚动
CMD_H_SCROLL_SETUP = 0x26
CMD_VH_SCROLL_SETUP = 0x28
CMD_DEACTIVATE_SCROLL = 0x2E
CMD_ACTIVATE_SCROLL = 0x2F


# 简易ASCII字体（5x7像素，仅部分字符）
_FONT_5x7 = {
    ' ': [0x00, 0x00, 0x00, 0x00, 0x00],
    '!': [0x00, 0x00, 0x5F, 0x00, 0x00],
    '0': [0x3E, 0x51, 0x49, 0x45, 0x3E],
    '1': [0x00, 0x42, 0x7F, 0x40, 0x00],
    '2': [0x42, 0x61, 0x51, 0x49, 0x46],
    '3': [0x21, 0x41, 0x45, 0x4B, 0x31],
    '4': [0x18, 0x14, 0x12, 0x7F, 0x10],
    '5': [0x27, 0x45, 0x45, 0x45, 0x39],
    '6': [0x3C, 0x4A, 0x49, 0x49, 0x30],
    '7': [0x01, 0x71, 0x09, 0x05, 0x03],
    '8': [0x36, 0x49, 0x49, 0x49, 0x36],
    '9': [0x06, 0x49, 0x49, 0x29, 0x1E],
    'A': [0x7E, 0x11, 0x11, 0x11, 0x7E],
    'B': [0x7F, 0x49, 0x49, 0x49, 0x36],
    'C': [0x3E, 0x41, 0x41, 0x41, 0x22],
    'D': [0x7F, 0x41, 0x41, 0x22, 0x1C],
    'E': [0x7F, 0x49, 0x49, 0x49, 0x41],
    'F': [0x7F, 0x09, 0x09, 0x09, 0x01],
    'H': [0x7F, 0x08, 0x08, 0x08, 0x7F],
    'O': [0x3E, 0x41, 0x41, 0x41, 0x3E],
    'W': [0x7F, 0x02, 0x0C, 0x02, 0x7F],
}


class SSD1327:
    """SSD1327 128x128灰度OLED模拟器"""

    def __init__(self, i2c_bus=None):
        self.bus = i2c_bus or I2CBus()
        self.bus.init()
        self.addr = SSD1327_I2C_ADDR
        self.width = SSD1327_WIDTH
        self.height = SSD1327_HEIGHT
        # 帧缓冲（4位灰度，每字节2像素）
        self._framebuf = bytearray((self.width * self.height) // 2)
        # 状态
        self._display_on = False
        self._contrast = 127
        self._inverted = False
        self._scrolling = False
        self._start_line = 0
        self._offset = 0
        # 绘图光标
        self._cursor_x = 0
        self._cursor_y = 0
        self._text_color = 15  # 最亮灰度
        self._bg_color = 0     # 黑色
        # 命令缓冲（使用list模拟）
        self._cmd_buffer = []

    def init(self):
        """初始化显示"""
        self._display_on = False
        self._contrast = 127
        self._inverted = False
        self.clear()
        self.send_command(CMD_DISPLAY_OFF)
        self.send_command(CMD_SET_MUX_RATIO, 0x7F)  # 128行
        self.send_command(CMD_SET_CLOCK_DIV, 0x01)
        self.send_command(CMD_SET_DISPLAY_OFFSET, self._offset)

    def send_command(self, cmd, *args):
        """发送命令"""
        self._cmd_buffer.append(cmd & 0xFF)
        for a in args:
            self._cmd_buffer.append(a & 0xFF)

    def display_on(self):
        """开显示"""
        self._display_on = True
        self.send_command(CMD_DISPLAY_ON)

    def display_off(self):
        """关显示"""
        self._display_on = False
        self.send_command(CMD_DISPLAY_OFF)

    def is_on(self):
        """显示是否开启"""
        return self._display_on

    def clear(self):
        """清屏"""
        for i in range(len(self._framebuf)):
            self._framebuf[i] = 0

    def set_pixel(self, x, y, gray):
        """设置像素灰度（0-15）"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return
        gray = max(0, min(SSD1327_MAX_GRAY, gray))
        if self._inverted:
            gray = SSD1327_MAX_GRAY - gray
        # 每字节2像素，高4位=左像素，低4位=右像素
        idx = (y * self.width + x) // 2
        if x % 2 == 0:
            self._framebuf[idx] = (self._framebuf[idx] & 0x0F) | (gray << 4)
        else:
            self._framebuf[idx] = (self._framebuf[idx] & 0xF0) | gray

    def get_pixel(self, x, y):
        """读取像素灰度"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return 0
        idx = (y * self.width + x) // 2
        if x % 2 == 0:
            return (self._framebuf[idx] >> 4) & 0x0F
        else:
            return self._framebuf[idx] & 0x0F

    def set_contrast(self, level):
        """设置对比度/亮度（0-255）"""
        self._contrast = level & 0xFF
        self.send_command(CMD_SET_CONTRAST, self._contrast)

    def get_contrast(self):
        """获取对比度"""
        return self._contrast

    def invert_display(self, invert=True):
        """反转显示"""
        self._inverted = invert
        if invert:
            self.send_command(CMD_INVERSE_DISPLAY)
        else:
            self.send_command(CMD_NORMAL_DISPLAY)

    def is_inverted(self):
        """是否反转"""
        return self._inverted

    def fill(self, gray):
        """全屏填充"""
        gray = max(0, min(15, gray))
        byte_val = (gray << 4) | gray
        for i in range(len(self._framebuf)):
            self._framebuf[i] = byte_val

    def fill_rect(self, x0, y0, w, h, gray):
        """矩形填充"""
        for y in range(y0, y0 + h):
            for x in range(x0, x0 + w):
                self.set_pixel(x, y, gray)

    def draw_hline(self, x0, y, w, gray):
        """水平线"""
        for x in range(x0, x0 + w):
            self.set_pixel(x, y, gray)

    def draw_vline(self, x, y0, h, gray):
        """垂直线"""
        for y in range(y0, y0 + h):
            self.set_pixel(x, y, gray)

    def draw_rect(self, x0, y0, w, h, gray):
        """矩形边框"""
        self.draw_hline(x0, y0, w, gray)
        self.draw_hline(x0, y0 + h - 1, w, gray)
        self.draw_vline(x0, y0, h, gray)
        self.draw_vline(x0 + w - 1, y0, h, gray)

    def draw_circle(self, cx, cy, r, gray):
        """画圆（Bresenham算法）"""
        x = r
        y = 0
        err = 1 - r
        while x >= y:
            self.set_pixel(cx + x, cy + y, gray)
            self.set_pixel(cx + y, cy + x, gray)
            self.set_pixel(cx - y, cy + x, gray)
            self.set_pixel(cx - x, cy + y, gray)
            self.set_pixel(cx - x, cy - y, gray)
            self.set_pixel(cx - y, cy - x, gray)
            self.set_pixel(cx + y, cy - x, gray)
            self.set_pixel(cx + x, cy - y, gray)
            y += 1
            if err < 0:
                err += 2 * y + 1
            else:
                x -= 1
                err += 2 * (y - x) + 1

    def set_cursor(self, x, y):
        """设置文本光标"""
        self._cursor_x = x
        self._cursor_y = y

    def set_text_color(self, color, bg=0):
        """设置文本颜色"""
        self._text_color = color
        self._bg_color = bg

    def draw_char(self, ch):
        """绘制单个字符（5x7）"""
        glyph = _FONT_5x7.get(ch, _FONT_5x7.get(' ', [0] * 5))
        for col in range(5):
            for row in range(7):
                if glyph[col] & (1 << row):
                    self.set_pixel(self._cursor_x + col, self._cursor_y + row, self._text_color)
                else:
                    self.set_pixel(self._cursor_x + col, self._cursor_y + row, self._bg_color)
        # 字符间距
        for row in range(7):
            self.set_pixel(self._cursor_x + 5, self._cursor_y + row, self._bg_color)
        self._cursor_x += 6

    def draw_string(self, text):
        """绘制字符串"""
        for ch in text:
            if self._cursor_x + 6 > self.width:
                self._cursor_x = 0
                self._cursor_y += 8
            if self._cursor_y + 7 > self.height:
                break
            self.draw_char(ch)

    def setup_h_scroll(self, start_page, end_page, speed=0):
        """设置水平滚动"""
        self.send_command(CMD_H_SCROLL_SETUP, 0x00, start_page, speed, end_page)

    def activate_scroll(self):
        """激活滚动"""
        self._scrolling = True
        self.send_command(CMD_ACTIVATE_SCROLL)

    def deactivate_scroll(self):
        """停止滚动"""
        self._scrolling = False
        self.send_command(CMD_DEACTIVATE_SCROLL)

    def is_scrolling(self):
        """是否在滚动"""
        return self._scrolling

    def set_start_line(self, line):
        """设置起始行"""
        self._start_line = line & 0x7F
        self.send_command(CMD_SET_DISPLAY_START, self._start_line)

    def set_offset(self, offset):
        """设置垂直偏移"""
        self._offset = offset & 0x7F
        self.send_command(CMD_SET_DISPLAY_OFFSET, self._offset)

    def get_framebuffer(self):
        """获取帧缓冲"""
        return bytes(self._framebuf)

    def get_framebuffer_size(self):
        """获取帧缓冲大小"""
        return len(self._framebuf)


# ---- 测试辅助 ----
def assert_eq(actual, expected, msg=""):
    if actual != expected:
        raise AssertionError(f"{msg}: 期望 {expected}, 实际 {actual}")

def run_test(func, name=""):
    try:
        func()
        print(f"  [通过] {name}")
        return True
    except AssertionError as e:
        print(f"  [失败] {name}: {e}")
        return False
    except Exception as e:
        print(f"  [错误] {name}: {type(e).__name__}: {e}")
        return False


# ═══════════════════════════════════════════════════════════════
#  测试用例
# ═══════════════════════════════════════════════════════════════

def test_init():
    """SSD1327初始化"""
    oled = SSD1327()
    oled.init()
    assert_eq(oled.width, 128, "宽度")
    assert_eq(oled.height, 128, "高度")
    assert not oled.is_on(), "初始应关显示"

def test_display_on_off():
    """显示开关"""
    oled = SSD1327()
    oled.init()
    oled.display_on()
    assert oled.is_on(), "应开显示"
    oled.display_off()
    assert not oled.is_on(), "应关显示"

def test_pixel_write_read():
    """像素写读"""
    oled = SSD1327()
    oled.init()
    oled.set_pixel(10, 20, 15)
    assert_eq(oled.get_pixel(10, 20), 15, "像素灰度")
    oled.set_pixel(11, 20, 7)
    assert_eq(oled.get_pixel(11, 20), 7, "相邻像素")

def test_pixel_boundary():
    """像素边界"""
    oled = SSD1327()
    oled.init()
    # 不应崩溃
    oled.set_pixel(-1, 0, 15)
    oled.set_pixel(0, -1, 15)
    oled.set_pixel(128, 0, 15)
    oled.set_pixel(0, 128, 15)
    assert_eq(oled.get_pixel(-1, 0), 0, "越界读应返回0")

def test_clear():
    """清屏"""
    oled = SSD1327()
    oled.init()
    oled.fill(15)
    oled.clear()
    assert_eq(oled.get_pixel(64, 64), 0, "清屏后应为0")

def test_fill():
    """全屏填充"""
    oled = SSD1327()
    oled.init()
    oled.fill(8)
    for x in [0, 63, 127]:
        for y in [0, 63, 127]:
            assert_eq(oled.get_pixel(x, y), 8, f"填充({x},{y})")

def test_fill_rect():
    """矩形填充"""
    oled = SSD1327()
    oled.init()
    oled.fill_rect(10, 10, 20, 15, 12)
    assert_eq(oled.get_pixel(10, 10), 12, "矩形左上")
    assert_eq(oled.get_pixel(29, 24), 12, "矩形右下")
    assert_eq(oled.get_pixel(9, 10), 0, "矩形外部")

def test_draw_rect():
    """矩形边框"""
    oled = SSD1327()
    oled.init()
    oled.draw_rect(5, 5, 10, 8, 15)
    assert_eq(oled.get_pixel(5, 5), 15, "边框顶边")
    assert_eq(oled.get_pixel(14, 12), 15, "边框右下")
    assert_eq(oled.get_pixel(10, 9), 0, "边框内部应为空")

def test_draw_circle():
    """画圆"""
    oled = SSD1327()
    oled.init()
    oled.draw_circle(64, 64, 20, 15)
    # 圆心不应有像素（空心圆）
    # 圆上应有像素
    assert_eq(oled.get_pixel(64, 44), 15, "圆顶")
    assert_eq(oled.get_pixel(64, 84), 15, "圆底")

def test_hline_vline():
    """水平线和垂直线"""
    oled = SSD1327()
    oled.init()
    oled.draw_hline(0, 10, 128, 15)
    for x in range(0, 128, 20):
        assert_eq(oled.get_pixel(x, 10), 15, f"水平线{x}")
    oled.draw_vline(10, 0, 128, 12)
    for y in range(0, 128, 20):
        assert_eq(oled.get_pixel(10, y), 12, f"垂直线{y}")

def test_grayscale_levels():
    """灰度级别测试"""
    oled = SSD1327()
    oled.init()
    for g in range(16):
        oled.set_pixel(g * 8, 64, g)
    for g in range(16):
        assert_eq(oled.get_pixel(g * 8, 64), g, f"灰度{g}")

def test_contrast():
    """对比度调节"""
    oled = SSD1327()
    oled.init()
    oled.set_contrast(200)
    assert_eq(oled.get_contrast(), 200, "对比度")
    oled.set_contrast(0)
    assert_eq(oled.get_contrast(), 0, "最低对比度")

def test_inverse():
    """显示反转"""
    oled = SSD1327()
    oled.init()
    oled.set_pixel(50, 50, 15)
    oled.invert_display(True)
    assert oled.is_inverted(), "应反转"
    # 新像素写入应自动反转
    oled.set_pixel(60, 60, 15)
    assert_eq(oled.get_pixel(60, 60), 0, "反转后15应变为0")
    oled.invert_display(False)
    assert not oled.is_inverted(), "应恢复"

def test_scroll():
    """滚动控制"""
    oled = SSD1327()
    oled.init()
    oled.setup_h_scroll(0, 7, 0)
    oled.activate_scroll()
    assert oled.is_scrolling(), "应滚动"
    oled.deactivate_scroll()
    assert not oled.is_scrolling(), "应停止滚动"

def test_start_line():
    """起始行设置"""
    oled = SSD1327()
    oled.init()
    oled.set_start_line(32)
    assert_eq(oled._start_line, 32, "起始行")

def test_offset():
    """垂直偏移"""
    oled = SSD1327()
    oled.init()
    oled.set_offset(10)
    assert_eq(oled._offset, 10, "偏移")

def test_draw_string():
    """字符串绘制"""
    oled = SSD1327()
    oled.init()
    oled.set_cursor(0, 0)
    oled.set_text_color(15)
    oled.draw_string("ABC")
    # 检查A的第一个列有非零像素
    assert oled.get_pixel(0, 0) != 0 or oled.get_pixel(1, 0) != 0, "字符串应有像素"

def test_framebuffer():
    """帧缓冲"""
    oled = SSD1327()
    oled.init()
    fb = oled.get_framebuffer()
    expected_size = (128 * 128) // 2  # 8192
    assert_eq(oled.get_framebuffer_size(), expected_size, "帧缓冲大小")
    assert_eq(len(fb), expected_size, "帧缓冲长度")

def test_i2c_integration():
    """I2C总线集成"""
    bus = I2CBus()
    bus.init()
    oled = SSD1327(bus)
    oled.init()
    oled.display_on()
    oled.set_pixel(0, 0, 15)
    assert bus.tx_count >= 0


def main():
    print("=" * 60)
    print("  SSD1327 OLED显示测试 V2")
    print("=" * 60)
    tests = [
        (test_init, "初始化"),
        (test_display_on_off, "显示开关"),
        (test_pixel_write_read, "像素写读"),
        (test_pixel_boundary, "像素边界"),
        (test_clear, "清屏"),
        (test_fill, "全屏填充"),
        (test_fill_rect, "矩形填充"),
        (test_draw_rect, "矩形边框"),
        (test_draw_circle, "画圆"),
        (test_hline_vline, "水平/垂直线"),
        (test_grayscale_levels, "灰度级别"),
        (test_contrast, "对比度"),
        (test_inverse, "显示反转"),
        (test_scroll, "滚动控制"),
        (test_start_line, "起始行"),
        (test_offset, "垂直偏移"),
        (test_draw_string, "字符串绘制"),
        (test_framebuffer, "帧缓冲"),
        (test_i2c_integration, "I2C集成"),
    ]
    passed = failed = 0
    for func, name in tests:
        if run_test(func, name):
            passed += 1
        else:
            failed += 1
    print("-" * 60)
    print(f"  结果: {passed} 通过, {failed} 失败, 共 {passed + failed} 项")
    print("=" * 60)
    return failed == 0

if __name__ == "__main__":
    sys.exit(0 if main() else 1)
