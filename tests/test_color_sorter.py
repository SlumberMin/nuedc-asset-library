#!/usr/bin/env python3
"""
颜色分拣 V2测试 — 基于wrappers.py包装层
覆盖: TCS34725传感器读取、RGB颜色分类、分拣决策
对应C源文件: 02_mspm0g3507/drivers/tcs34725.c + color_sorter算法

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    TCS34725, TCS34725_RGBC, Servo,
    TCS34725_ADDR, TCS34725_ENABLE_PON, TCS34725_ENABLE_AEN,
)


# 颜色分类阈值（与C版本一致）
COLOR_NONE   = 0  # 无物料
COLOR_RED    = 1  # 红色
COLOR_GREEN  = 2  # 绿色
COLOR_BLUE   = 3  # 蓝色
COLOR_WHITE  = 4  # 白色


class ColorSorter:
    """颜色分拣算法 — 基于TCS34725颜色传感器

    与C版本color_sorter.c逻辑一致:
    - 读取RGBC值
    - 计算RGB比例
    - 阈值判断颜色
    - 舵机分拣动作
    """

    # 分拣阈值
    MIN_CLEAR = 50       # 最小有效光照值
    RED_RATIO_THR = 0.40  # 红色占比阈值
    GREEN_RATIO_THR = 0.35
    BLUE_RATIO_THR = 0.35

    def __init__(self):
        self.sensor = TCS34725()
        self.servo = Servo()
        self.last_color = COLOR_NONE
        self.sort_count = {COLOR_RED: 0, COLOR_GREEN: 0, COLOR_BLUE: 0, COLOR_WHITE: 0}

    def init(self):
        """初始化传感器和舵机"""
        self.sensor.init()
        self.servo.init()
        self.sort_count = {COLOR_RED: 0, COLOR_GREEN: 0, COLOR_BLUE: 0, COLOR_WHITE: 0}

    def classify_color(self, rgbc):
        """根据RGBC数据分类颜色

        返回: color_id (int)
        """
        clear = rgbc.clear
        if clear < self.MIN_CLEAR:
            return COLOR_NONE

        total = rgbc.red + rgbc.green + rgbc.blue
        if total < 1:
            return COLOR_NONE

        r_ratio = rgbc.red / total
        g_ratio = rgbc.green / total
        b_ratio = rgbc.blue / total

        # 红色: R分量最高且超过阈值
        if r_ratio > self.RED_RATIO_THR and r_ratio > g_ratio and r_ratio > b_ratio:
            return COLOR_RED

        # 绿色: G分量最高
        if g_ratio > self.GREEN_RATIO_THR and g_ratio > r_ratio and g_ratio > b_ratio:
            return COLOR_GREEN

        # 蓝色: B分量最高
        if b_ratio > self.BLUE_RATIO_THR and b_ratio > r_ratio and b_ratio > g_ratio:
            return COLOR_BLUE

        return COLOR_WHITE

    def sort_action(self, color_id):
        """根据颜色执行分拣动作

        返回: servo_angle (int)
        """
        self.last_color = color_id
        if color_id == COLOR_RED:
            angle = 45
        elif color_id == COLOR_GREEN:
            angle = 90
        elif color_id == COLOR_BLUE:
            angle = 135
        else:
            angle = 0  # 默认位置

        self.servo.set_angle(angle)
        self.sort_count[color_id] = self.sort_count.get(color_id, 0) + 1
        return angle

    def update(self, clear, red, green, blue):
        """完整分拣流程: 读取→分类→分拣

        返回: (color_id, angle)
        """
        self.sensor.set_rgbc(clear, red, green, blue)
        ok, rgbc = self.sensor.read_rgbc()
        if not ok:
            return COLOR_NONE, 0

        color_id = self.classify_color(rgbc)
        angle = self.sort_action(color_id)
        return color_id, angle


class TestColorSorterInit(unittest.TestCase):
    """初始化测试"""

    def test_init(self):
        """初始化成功"""
        cs = ColorSorter()
        cs.init()
        self.assertTrue(cs.sensor.initialized)
        self.assertTrue(cs.servo.running)

    def test_default_sort_count(self):
        """初始化后分拣计数为零"""
        cs = ColorSorter()
        cs.init()
        for count in cs.sort_count.values():
            self.assertEqual(count, 0)


class TestTCS34725Sensor(unittest.TestCase):
    """TCS34725传感器测试"""

    def test_init_enabled(self):
        """初始化后传感器使能"""
        sensor = TCS34725()
        sensor.init()
        self.assertTrue(sensor.enabled)
        ok, _ = sensor.read_reg(0x00)
        self.assertTrue(ok)

    def test_write_read_reg(self):
        """寄存器读写"""
        sensor = TCS34725()
        sensor.init()
        ok = sensor.write_reg(0x01, 0xAB)
        self.assertTrue(ok)
        ok, val = sensor.read_reg(0x01)
        self.assertTrue(ok)
        self.assertEqual(val, 0xAB)

    def test_rgbc_read_write(self):
        """RGBC数据读写"""
        sensor = TCS34725()
        sensor.init()
        sensor.set_rgbc(1000, 400, 300, 200)
        ok, rgbc = sensor.read_rgbc()
        self.assertTrue(ok)
        self.assertEqual(rgbc.clear, 1000)
        self.assertEqual(rgbc.red, 400)
        self.assertEqual(rgbc.green, 300)
        self.assertEqual(rgbc.blue, 200)

    def test_not_initialized(self):
        """未初始化读取失败"""
        sensor = TCS34725()
        ok, _ = sensor.read_rgbc()
        self.assertFalse(ok)

    def test_default_rgbc_zero(self):
        """默认RGBC为零"""
        sensor = TCS34725()
        sensor.init()
        ok, rgbc = sensor.read_rgbc()
        self.assertTrue(ok)
        self.assertEqual(rgbc.clear, 0)


class TestColorClassification(unittest.TestCase):
    """颜色分类测试"""

    def test_red(self):
        """红色分类: R分量最高"""
        cs = ColorSorter()
        cs.init()
        rgbc = TCS34725_RGBC(clear=500, red=400, green=50, blue=50)
        self.assertEqual(cs.classify_color(rgbc), COLOR_RED)

    def test_green(self):
        """绿色分类: G分量最高"""
        cs = ColorSorter()
        cs.init()
        rgbc = TCS34725_RGBC(clear=500, red=50, green=400, blue=50)
        self.assertEqual(cs.classify_color(rgbc), COLOR_GREEN)

    def test_blue(self):
        """蓝色分类: B分量最高"""
        cs = ColorSorter()
        cs.init()
        rgbc = TCS34725_RGBC(clear=500, red=50, green=50, blue=400)
        self.assertEqual(cs.classify_color(rgbc), COLOR_BLUE)

    def test_white(self):
        """白色分类: R/G/B均匀"""
        cs = ColorSorter()
        cs.init()
        rgbc = TCS34725_RGBC(clear=800, red=250, green=250, blue=250)
        self.assertEqual(cs.classify_color(rgbc), COLOR_WHITE)

    def test_none_dark(self):
        """无物料: clear值太低"""
        cs = ColorSorter()
        cs.init()
        rgbc = TCS34725_RGBC(clear=10, red=5, green=3, blue=2)
        self.assertEqual(cs.classify_color(rgbc), COLOR_NONE)

    def test_none_zero_total(self):
        """RGB全零 → 无物料"""
        cs = ColorSorter()
        cs.init()
        rgbc = TCS34725_RGBC(clear=100, red=0, green=0, blue=0)
        self.assertEqual(cs.classify_color(rgbc), COLOR_NONE)


class TestSortAction(unittest.TestCase):
    """分拣动作测试"""

    def test_red_sort(self):
        """红色 → 舵机45°"""
        cs = ColorSorter()
        cs.init()
        angle = cs.sort_action(COLOR_RED)
        self.assertEqual(angle, 45)

    def test_green_sort(self):
        """绿色 → 舵机90°"""
        cs = ColorSorter()
        cs.init()
        angle = cs.sort_action(COLOR_GREEN)
        self.assertEqual(angle, 90)

    def test_blue_sort(self):
        """蓝色 → 舵机135°"""
        cs = ColorSorter()
        cs.init()
        angle = cs.sort_action(COLOR_BLUE)
        self.assertEqual(angle, 135)

    def test_white_sort(self):
        """白色 → 舵机0°(默认)"""
        cs = ColorSorter()
        cs.init()
        angle = cs.sort_action(COLOR_WHITE)
        self.assertEqual(angle, 0)

    def test_sort_count(self):
        """分拣计数递增"""
        cs = ColorSorter()
        cs.init()
        cs.sort_action(COLOR_RED)
        cs.sort_action(COLOR_RED)
        cs.sort_action(COLOR_GREEN)
        self.assertEqual(cs.sort_count[COLOR_RED], 2)
        self.assertEqual(cs.sort_count[COLOR_GREEN], 1)
        self.assertEqual(cs.sort_count[COLOR_BLUE], 0)


class TestColorSorterUpdate(unittest.TestCase):
    """完整更新流程测试"""

    def test_red_update(self):
        """红色物体完整分拣流程"""
        cs = ColorSorter()
        cs.init()
        color_id, angle = cs.update(500, 400, 50, 50)
        self.assertEqual(color_id, COLOR_RED)
        self.assertEqual(angle, 45)

    def test_green_update(self):
        """绿色物体完整分拣流程"""
        cs = ColorSorter()
        cs.init()
        color_id, angle = cs.update(500, 50, 400, 50)
        self.assertEqual(color_id, COLOR_GREEN)
        self.assertEqual(angle, 90)

    def test_dark_no_action(self):
        """无物料 → 不动作"""
        cs = ColorSorter()
        cs.init()
        color_id, angle = cs.update(10, 3, 2, 1)
        self.assertEqual(color_id, COLOR_NONE)


if __name__ == '__main__':
    unittest.main()
