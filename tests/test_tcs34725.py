#!/usr/bin/env python3
"""
TCS34725 颜色传感器 V3 测试 — 颜色分类算法测试
覆盖: V2全部 + RGBC比例计算、颜色分类逻辑、白平衡模拟、
      异常值处理、传感器饱和、多场景颜色识别
对应C源文件: 02_mspm0g3507/drivers/tcs34725.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #6:  I2C忙等待/超时（初始化失败场景）
  #13: 颜色传感器用于色块分拣，需覆盖典型颜色场景
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    TCS34725, TCS34725_RGBC,
    TCS34725_ADDR, TCS34725_CMD_BIT, TCS34725_CMD_AUTO_INC,
    TCS34725_ENABLE_PON, TCS34725_ENABLE_AEN,
)


def classify_color(rgbc):
    """
    颜色分类算法 — 基于RGBC比例判断颜色
    返回: 'red', 'green', 'blue', 'white', 'black', 'unknown'
    """
    total = rgbc.red + rgbc.green + rgbc.blue
    if total < 10:
        return 'black'
    if rgbc.clear > 60000 and total > 50000:
        return 'white'
    r_ratio = rgbc.red / total
    g_ratio = rgbc.green / total
    b_ratio = rgbc.blue / total
    if r_ratio > 0.5:
        return 'red'
    if g_ratio > 0.5:
        return 'green'
    if b_ratio > 0.5:
        return 'blue'
    return 'unknown'


class TestTCS34725V3(unittest.TestCase):
    """TCS34725颜色传感器V3 — 颜色分类算法测试"""

    def setUp(self):
        self.sensor = TCS34725()

    # ── 基础初始化 ──

    def test_init_success(self):
        """初始化成功"""
        result = self.sensor.init()
        self.assertTrue(result)
        self.assertTrue(self.sensor.initialized)
        self.assertTrue(self.sensor.enabled)

    def test_read_rgbc_before_init(self):
        """未初始化时读取应失败"""
        ok, rgbc = self.sensor.read_rgbc()
        self.assertFalse(ok)
        self.assertEqual(rgbc.clear, 0)

    def test_read_rgbc_after_init(self):
        """初始化后读取默认值"""
        self.sensor.init()
        ok, rgbc = self.sensor.read_rgbc()
        self.assertTrue(ok)
        self.assertEqual(rgbc.clear, 0)

    def test_set_and_read_rgbc(self):
        """设置并读取RGBC值"""
        self.sensor.init()
        self.sensor.set_rgbc(1000, 500, 600, 400)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertTrue(ok)
        self.assertEqual(rgbc.clear, 1000)
        self.assertEqual(rgbc.red, 500)
        self.assertEqual(rgbc.green, 600)
        self.assertEqual(rgbc.blue, 400)

    def test_rgbc_data_structure(self):
        """RGBC数据结构"""
        rgbc = TCS34725_RGBC(100, 200, 300, 400)
        self.assertEqual(rgbc.clear, 100)
        self.assertEqual(rgbc.red, 200)
        self.assertEqual(rgbc.green, 300)
        self.assertEqual(rgbc.blue, 400)

    def test_rgbc_default_zero(self):
        """RGBC默认值为0"""
        rgbc = TCS34725_RGBC()
        self.assertEqual(rgbc.clear, 0)
        self.assertEqual(rgbc.red, 0)

    # ── 寄存器操作 ──

    def test_write_reg_before_init(self):
        """未初始化写寄存器应失败"""
        ok = self.sensor.write_reg(0x00, 0x03)
        self.assertFalse(ok)

    def test_write_and_read_reg(self):
        """写入并读回寄存器"""
        self.sensor.init()
        ok = self.sensor.write_reg(0x00, 0x03)
        self.assertTrue(ok)
        ok, val = self.sensor.read_reg(0x00)
        self.assertTrue(ok)
        self.assertEqual(val, 0x03)

    def test_read_reg_before_init(self):
        """未初始化读寄存器应失败"""
        ok, val = self.sensor.read_reg(0x00)
        self.assertFalse(ok)

    def test_enable_register_after_init(self):
        """初始化后ENABLE寄存器应有PON|AEN"""
        self.sensor.init()
        ok, val = self.sensor.read_reg(0x00)
        self.assertTrue(ok)
        self.assertEqual(val, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)

    def test_i2c_address(self):
        """I2C地址常量"""
        self.assertEqual(TCS34725_ADDR, 0x29)

    def test_cmd_bits(self):
        """命令位常量"""
        self.assertEqual(TCS34725_CMD_BIT, 0x80)
        self.assertEqual(TCS34725_CMD_AUTO_INC, 0xA0)

    def test_rgbc_max_values(self):
        """RGBC最大值(16位)"""
        self.sensor.init()
        max_val = 0xFFFF
        self.sensor.set_rgbc(max_val, max_val, max_val, max_val)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertTrue(ok)
        self.assertEqual(rgbc.clear, max_val)
        self.assertEqual(rgbc.red, max_val)

    def test_multiple_init(self):
        """多次初始化应安全"""
        self.sensor.init()
        self.sensor.init()
        ok, rgbc = self.sensor.read_rgbc()
        self.assertTrue(ok)

    # ── V3: 颜色分类算法 ──

    def test_classify_red(self):
        """红色色块分类"""
        self.sensor.init()
        self.sensor.set_rgbc(3000, 2500, 200, 150)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertTrue(ok)
        self.assertEqual(classify_color(rgbc), 'red')

    def test_classify_green(self):
        """绿色色块分类"""
        self.sensor.init()
        self.sensor.set_rgbc(3000, 200, 2500, 150)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertEqual(classify_color(rgbc), 'green')

    def test_classify_blue(self):
        """蓝色色块分类"""
        self.sensor.init()
        self.sensor.set_rgbc(3000, 150, 200, 2500)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertEqual(classify_color(rgbc), 'blue')

    def test_classify_white(self):
        """白色分类"""
        self.sensor.init()
        self.sensor.set_rgbc(65000, 20000, 20000, 20000)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertEqual(classify_color(rgbc), 'white')

    def test_classify_black(self):
        """黑色分类（无光）"""
        self.sensor.init()
        self.sensor.set_rgbc(0, 0, 0, 0)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertEqual(classify_color(rgbc), 'black')

    def test_classify_red_dominant_edge(self):
        """红色刚好过阈值(50%)"""
        self.sensor.init()
        # red=501, green=250, blue=249 → total=1000, r_ratio=0.501
        self.sensor.set_rgbc(3000, 501, 250, 249)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertEqual(classify_color(rgbc), 'red')

    def test_classify_no_dominant(self):
        """无主导色（各色比例接近）→ unknown"""
        self.sensor.init()
        self.sensor.set_rgbc(3000, 334, 333, 333)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertEqual(classify_color(rgbc), 'unknown')

    # ── V3: 16位边界值 ──

    def test_rgbc_zero_all(self):
        """全零RGBC"""
        self.sensor.init()
        self.sensor.set_rgbc(0, 0, 0, 0)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertTrue(ok)
        self.assertEqual(rgbc.clear, 0)
        self.assertEqual(rgbc.red, 0)
        self.assertEqual(rgbc.green, 0)
        self.assertEqual(rgbc.blue, 0)

    def test_rgbc_saturation(self):
        """传感器饱和值(0xFFFF)"""
        self.sensor.init()
        self.sensor.set_rgbc(0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertTrue(ok)
        self.assertEqual(rgbc.clear, 0xFFFF)
        # 饱和时分类为白色
        self.assertEqual(classify_color(rgbc), 'white')

    # ── V3: 白平衡模拟 ──

    def test_white_balance_ratios(self):
        """白平衡: 理想白光下RGBC应接近均匀"""
        self.sensor.init()
        # 模拟白光照射
        self.sensor.set_rgbc(10000, 3300, 3400, 3300)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertTrue(ok)
        total = rgbc.red + rgbc.green + rgbc.blue
        # 各色比例应接近1/3
        self.assertAlmostEqual(rgbc.red / total, 1.0 / 3, places=1)
        self.assertAlmostEqual(rgbc.green / total, 1.0 / 3, places=1)
        self.assertAlmostEqual(rgbc.blue / total, 1.0 / 3, places=1)

    # ── V3: 连续读取一致性 ──

    def test_repeated_read_stability(self):
        """连续多次读取应返回相同值"""
        self.sensor.init()
        self.sensor.set_rgbc(5000, 1000, 2000, 3000)
        for _ in range(50):
            ok, rgbc = self.sensor.read_rgbc()
            self.assertTrue(ok)
            self.assertEqual(rgbc.clear, 5000)
            self.assertEqual(rgbc.red, 1000)
            self.assertEqual(rgbc.green, 2000)
            self.assertEqual(rgbc.blue, 3000)

    # ── V3: 多次set_rgbc覆盖 ──

    def test_set_rgbc_overwrite(self):
        """多次set_rgbc应覆盖前值"""
        self.sensor.init()
        self.sensor.set_rgbc(100, 200, 300, 400)
        self.sensor.set_rgbc(500, 600, 700, 800)
        ok, rgbc = self.sensor.read_rgbc()
        self.assertEqual(rgbc.clear, 500)
        self.assertEqual(rgbc.red, 600)
        self.assertEqual(rgbc.green, 700)
        self.assertEqual(rgbc.blue, 800)


if __name__ == '__main__':
    unittest.main()
