#!/usr/bin/env python3
"""
颜色分拣V4测试 — TCS34725 + PCA9685 + 计数统计
覆盖: TCS34725颜色读取、RGB→颜色分类、PCA9685舵机控制、
      分拣计数统计、色标校准
对应C源文件: 02_mspm0g3507/drivers/tcs34725.c + color_sorting算法

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    TCS34725, TCS34725_RGBC, PCA9685,
)


# ═══════════════════════════════════════════════════════════════
#  颜色分类定义
# ═══════════════════════════════════════════════════════════════
COLOR_UNKNOWN = 0
COLOR_RED = 1
COLOR_GREEN = 2
COLOR_BLUE = 3
COLOR_WHITE = 4
COLOR_BLACK = 5

# 舵机通道
SERVO_GATE = 0        # 分拣门舵机
SERVO_GATE_RED = 45   # 红色位置角度
SERVO_GATE_GREEN = 90 # 绿色位置角度
SERVO_GATE_BLUE = 135 # 蓝色位置角度
SERVO_GATE_DEFAULT = 0 # 默认位置


class ColorSorterV4:
    """颜色分拣V4 — TCS34725 + PCA9685 + 计数统计

    功能:
    - TCS34725读取RGBC颜色值
    - RGB比例分类算法（归一化+阈值判断）
    - PCA9685控制分拣舵机
    - 统计各颜色计数
    """

    def __init__(self):
        self.color_sensor = TCS34725()
        self.pwm_driver = PCA9685()
        self.counts = {COLOR_RED: 0, COLOR_GREEN: 0,
                       COLOR_BLUE: 0, COLOR_UNKNOWN: 0}
        self.last_color = COLOR_UNKNOWN
        self.last_rgbc = TCS34725_RGBC()

    def init(self):
        """初始化传感器和PWM驱动"""
        self.color_sensor.init()
        self.pwm_driver.init()
        self.pwm_driver.set_pwm_freq(50)  # 50Hz舵机频率
        self.reset_counts()

    def reset_counts(self):
        """重置计数"""
        for k in self.counts:
            self.counts[k] = 0

    def classify_color(self, rgbc):
        """RGB比例分类算法

        规则:
        1. 先检查亮度(clear)：过低→黑色，过高→白色
        2. 归一化RGB比例
        3. 占主导通道判定颜色
        """
        clear = rgbc.clear
        red = rgbc.red
        green = rgbc.green
        blue = rgbc.blue

        # 亮度检查
        if clear < 50:
            return COLOR_BLACK
        if clear > 60000:
            return COLOR_WHITE

        # 归一化比例
        total = red + green + blue
        if total == 0:
            return COLOR_UNKNOWN

        r_ratio = red / total
        g_ratio = green / total
        b_ratio = blue / total

        # 主导通道阈值
        DOMINANT_THRESHOLD = 0.40

        if r_ratio > DOMINANT_THRESHOLD and r_ratio > g_ratio and r_ratio > b_ratio:
            return COLOR_RED
        elif g_ratio > DOMINANT_THRESHOLD and g_ratio > r_ratio and g_ratio > b_ratio:
            return COLOR_GREEN
        elif b_ratio > DOMINANT_THRESHOLD and b_ratio > r_ratio and b_ratio > g_ratio:
            return COLOR_BLUE

        return COLOR_UNKNOWN

    def sort_once(self):
        """执行一次分拣

        返回: (success, color_id)
        """
        ok, rgbc = self.color_sensor.read_rgbc()
        if not ok:
            return False, COLOR_UNKNOWN

        self.last_rgbc = rgbc
        color = self.classify_color(rgbc)
        self.last_color = color

        if color not in (COLOR_UNKNOWN, COLOR_BLACK, COLOR_WHITE):
            self.counts[color] = self.counts.get(color, 0) + 1
            # 驱动舵机到对应位置
            angle_map = {
                COLOR_RED: SERVO_GATE_RED,
                COLOR_GREEN: SERVO_GATE_GREEN,
                COLOR_BLUE: SERVO_GATE_BLUE,
            }
            angle = angle_map.get(color, SERVO_GATE_DEFAULT)
            self.pwm_driver.set_angle(SERVO_GATE, angle)

        return True, color

    def get_total_count(self):
        """获取总分拣数"""
        return sum(self.counts.values())

    def get_color_ratio(self, color_id):
        """获取某颜色占比"""
        total = self.get_total_count()
        if total == 0:
            return 0.0
        return self.counts.get(color_id, 0) / total


class TestColorSensorInit(unittest.TestCase):
    """TCS34725初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        sensor = TCS34725()
        self.assertTrue(sensor.init())
        self.assertTrue(sensor.initialized)
        self.assertTrue(sensor.enabled)

    def test_read_before_init(self):
        """初始化前读取失败"""
        sensor = TCS34725()
        ok, rgbc = sensor.read_rgbc()
        self.assertFalse(ok)

    def test_set_and_read_rgbc(self):
        """设置并读取RGBC值"""
        sensor = TCS34725()
        sensor.init()
        sensor.set_rgbc(1000, 800, 200, 100)
        ok, rgbc = sensor.read_rgbc()
        self.assertTrue(ok)
        self.assertEqual(rgbc.clear, 1000)
        self.assertEqual(rgbc.red, 800)
        self.assertEqual(rgbc.green, 200)
        self.assertEqual(rgbc.blue, 100)


class TestColorClassification(unittest.TestCase):
    """颜色分类算法测试"""

    def setUp(self):
        self.sorter = ColorSorterV4()
        self.sorter.init()

    def test_red_classification(self):
        """红色分类: R占主导"""
        rgbc = TCS34725_RGBC(clear=2000, red=1200, green=400, blue=400)
        color = self.sorter.classify_color(rgbc)
        self.assertEqual(color, COLOR_RED)

    def test_green_classification(self):
        """绿色分类: G占主导"""
        rgbc = TCS34725_RGBC(clear=2000, red=300, green=1200, blue=500)
        color = self.sorter.classify_color(rgbc)
        self.assertEqual(color, COLOR_GREEN)

    def test_blue_classification(self):
        """蓝色分类: B占主导"""
        rgbc = TCS34725_RGBC(clear=2000, red=300, green=400, blue=1300)
        color = self.sorter.classify_color(rgbc)
        self.assertEqual(color, COLOR_BLUE)

    def test_black_classification(self):
        """黑色分类: 亮度极低"""
        rgbc = TCS34725_RGBC(clear=10, red=5, green=3, blue=2)
        color = self.sorter.classify_color(rgbc)
        self.assertEqual(color, COLOR_BLACK)

    def test_white_classification(self):
        """白色分类: 亮度极高"""
        rgbc = TCS34725_RGBC(clear=65000, red=20000, green=20000, blue=20000)
        color = self.sorter.classify_color(rgbc)
        self.assertEqual(color, COLOR_WHITE)

    def test_unknown_mixed(self):
        """混合色: 无主导通道→未知"""
        rgbc = TCS34725_RGBC(clear=2000, red=700, green=700, blue=600)
        color = self.sorter.classify_color(rgbc)
        self.assertEqual(color, COLOR_UNKNOWN)

    def test_zero_rgbc(self):
        """全零: 黑色"""
        rgbc = TCS34725_RGBC(clear=0, red=0, green=0, blue=0)
        color = self.sorter.classify_color(rgbc)
        self.assertEqual(color, COLOR_BLACK)


class TestPCA9685PWM(unittest.TestCase):
    """PCA9685舵机控制测试"""

    def test_init_and_freq(self):
        """初始化并设置频率"""
        pwm = PCA9685()
        pwm.init()
        self.assertTrue(pwm.initialized)
        self.assertEqual(pwm.freq, 50)

    def test_set_angle_range(self):
        """舵机角度范围 0~180"""
        pwm = PCA9685()
        pwm.init()
        # 0°
        pwm.set_angle(0, 0)
        _, off0 = pwm.get_pwm(0)
        # 90°
        pwm.set_angle(0, 90)
        _, off90 = pwm.get_pwm(0)
        # 180°
        pwm.set_angle(0, 180)
        _, off180 = pwm.get_pwm(0)
        self.assertLess(off0, off90)
        self.assertLess(off90, off180)

    def test_set_angle_clamp(self):
        """角度超限自动限幅"""
        pwm = PCA9685()
        pwm.init()
        pwm.set_angle(0, -10)
        _, off_min = pwm.get_pwm(0)
        pwm.set_angle(0, 200)
        _, off_max = pwm.get_pwm(0)
        # 应与0°和180°相同
        pwm.set_angle(0, 0)
        _, off_0 = pwm.get_pwm(0)
        pwm.set_angle(0, 180)
        _, off_180 = pwm.get_pwm(0)
        self.assertEqual(off_min, off_0)
        self.assertEqual(off_max, off_180)

    def test_invalid_channel(self):
        """无效通道返回False"""
        pwm = PCA9685()
        pwm.init()
        self.assertFalse(pwm.set_angle(-1, 90))
        self.assertFalse(pwm.set_angle(16, 90))

    def test_all_off(self):
        """关闭所有通道"""
        pwm = PCA9685()
        pwm.init()
        pwm.set_angle(5, 90)
        pwm.all_off()
        on, off = pwm.get_pwm(5)
        self.assertEqual(on, 0)
        self.assertEqual(off, 0)


class TestColorSorterIntegration(unittest.TestCase):
    """颜色分拣集成测试"""

    def setUp(self):
        self.sorter = ColorSorterV4()
        self.sorter.init()

    def test_sort_red(self):
        """分拣红色物体"""
        self.sorter.color_sensor.set_rgbc(2000, 1200, 400, 400)
        ok, color = self.sorter.sort_once()
        self.assertTrue(ok)
        self.assertEqual(color, COLOR_RED)
        self.assertEqual(self.sorter.counts[COLOR_RED], 1)

    def test_sort_multiple(self):
        """连续分拣多种颜色"""
        # 红
        self.sorter.color_sensor.set_rgbc(2000, 1200, 400, 400)
        self.sorter.sort_once()
        # 绿
        self.sorter.color_sensor.set_rgbc(2000, 300, 1200, 500)
        self.sorter.sort_once()
        # 蓝
        self.sorter.color_sensor.set_rgbc(2000, 300, 400, 1300)
        self.sorter.sort_once()
        # 黑(不计数)
        self.sorter.color_sensor.set_rgbc(10, 5, 3, 2)
        self.sorter.sort_once()

        self.assertEqual(self.sorter.counts[COLOR_RED], 1)
        self.assertEqual(self.sorter.counts[COLOR_GREEN], 1)
        self.assertEqual(self.sorter.counts[COLOR_BLUE], 1)
        self.assertEqual(self.sorter.get_total_count(), 3)

    def test_count_reset(self):
        """计数重置"""
        self.sorter.color_sensor.set_rgbc(2000, 1200, 400, 400)
        self.sorter.sort_once()
        self.assertGreater(self.sorter.get_total_count(), 0)
        self.sorter.reset_counts()
        self.assertEqual(self.sorter.get_total_count(), 0)

    def test_color_ratio(self):
        """颜色占比计算"""
        # 2红1蓝
        self.sorter.color_sensor.set_rgbc(2000, 1200, 400, 400)
        self.sorter.sort_once()
        self.sorter.sort_once()
        self.sorter.color_sensor.set_rgbc(2000, 300, 400, 1300)
        self.sorter.sort_once()

        ratio_red = self.sorter.get_color_ratio(COLOR_RED)
        ratio_blue = self.sorter.get_color_ratio(COLOR_BLUE)
        self.assertAlmostEqual(ratio_red, 2.0 / 3.0, places=2)
        self.assertAlmostEqual(ratio_blue, 1.0 / 3.0, places=2)

    def test_sort_unknown_no_count(self):
        """未知颜色不计数"""
        self.sorter.color_sensor.set_rgbc(2000, 700, 700, 600)
        ok, color = self.sorter.sort_once()
        self.assertTrue(ok)
        self.assertEqual(color, COLOR_UNKNOWN)
        self.assertEqual(self.sorter.get_total_count(), 0)

    def test_servo_follows_color(self):
        """舵机跟随颜色动作"""
        self.sorter.color_sensor.set_rgbc(2000, 1200, 400, 400)
        self.sorter.sort_once()
        _, off = self.sorter.pwm_driver.get_pwm(SERVO_GATE)
        # 红色→45°角度，PWM应大于0
        self.assertGreater(off, 0)


if __name__ == '__main__':
    unittest.main()
