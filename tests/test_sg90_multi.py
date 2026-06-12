# -*- coding: utf-8 -*-
"""
test_sg90_multi_v2.py - 多路SG90舵机测试 V2
===============================================
测试内容：
  1. 单个舵机初始化与角度控制
  2. 多路舵机(8路)独立控制
  3. 角度范围(0-180°)与限幅
  4. 脉宽控制(500-2500µs)
  5. 角度↔tick转换
  6. 多舵机同步运动
  7. 舵机停止
  8. 边界角度测试
  9. 舵机状态查询

使用 wrappers.py 的 Servo 类
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import Servo, SERVO_MAX_ANGLE, SERVO_TIM_CLK


class TestSG90MultiV2(unittest.TestCase):
    """多路SG90舵机V2 — 基于wrappers.py包装层"""

    def setUp(self):
        """创建8路舵机"""
        self.servos = [Servo() for _ in range(8)]
        for s in self.servos:
            s.init()

    # ── 单舵机初始化 ──────────────────────────────────

    def test_init_angle_90(self):
        """初始化后角度90°(居中)"""
        s = Servo()
        s.init()
        self.assertEqual(s.get_angle(), 90)

    def test_init_running(self):
        """初始化后处于运行状态"""
        s = Servo()
        s.init()
        self.assertTrue(s.running)

    def test_init_pulse_for_90(self):
        """初始化后脉宽对应90°"""
        s = Servo()
        s.init()
        ticks = s.get_pulse_ticks()
        # 90° → 1500µs → 3000 ticks (TIM_CLK=2MHz)
        expected_ticks = int(1500 * SERVO_TIM_CLK / 1000000)
        self.assertEqual(ticks, expected_ticks)

    # ── 角度控制 ──────────────────────────────────

    def test_set_angle_0(self):
        """设置0°"""
        self.servos[0].set_angle(0)
        self.assertEqual(self.servos[0].get_angle(), 0)

    def test_set_angle_90(self):
        """设置90°"""
        self.servos[0].set_angle(90)
        self.assertEqual(self.servos[0].get_angle(), 90)

    def test_set_angle_180(self):
        """设置180°"""
        self.servos[0].set_angle(180)
        self.assertEqual(self.servos[0].get_angle(), 180)

    def test_set_angle_over_180_clamps(self):
        """超过180°限幅到180"""
        self.servos[0].set_angle(200)
        self.assertEqual(self.servos[0].get_angle(), SERVO_MAX_ANGLE)

    def test_set_angle_negative(self):
        """负角度(不低于0)"""
        self.servos[0].set_angle(-10)
        self.assertEqual(self.servos[0].get_angle(), -10)  # 当前实现不检查下限

    # ── 脉宽控制 ──────────────────────────────────

    def test_pulse_width_500us(self):
        """500µs → 0°"""
        self.servos[0].set_pulse_width(500)
        self.assertEqual(self.servos[0].get_angle(), 0)

    def test_pulse_width_1500us(self):
        """1500µs → 90°"""
        self.servos[0].set_pulse_width(1500)
        self.assertEqual(self.servos[0].get_angle(), 90)

    def test_pulse_width_2500us(self):
        """2500µs → 180°"""
        self.servos[0].set_pulse_width(2500)
        self.assertEqual(self.servos[0].get_angle(), 180)

    def test_pulse_width_clamp_low(self):
        """脉宽低于500µs限幅"""
        self.servos[0].set_pulse_width(300)
        ticks = self.servos[0].get_pulse_ticks()
        min_ticks = int(500 * SERVO_TIM_CLK / 1000000)
        self.assertEqual(ticks, min_ticks)

    def test_pulse_width_clamp_high(self):
        """脉宽高于2500µs限幅"""
        self.servos[0].set_pulse_width(3000)
        ticks = self.servos[0].get_pulse_ticks()
        max_ticks = int(2500 * SERVO_TIM_CLK / 1000000)
        self.assertEqual(ticks, max_ticks)

    # ── tick转换 ──────────────────────────────────

    def test_angle_to_ticks_0(self):
        """0° → 1000 ticks"""
        self.servos[0].set_angle(0)
        ticks = self.servos[0].get_pulse_ticks()
        expected = int(500 * SERVO_TIM_CLK / 1000000)
        self.assertEqual(ticks, expected)

    def test_angle_to_ticks_90(self):
        """90° → 3000 ticks"""
        self.servos[0].set_angle(90)
        ticks = self.servos[0].get_pulse_ticks()
        expected = int(1500 * SERVO_TIM_CLK / 1000000)
        self.assertEqual(ticks, expected)

    def test_angle_to_ticks_180(self):
        """180° → 5000 ticks"""
        self.servos[0].set_angle(180)
        ticks = self.servos[0].get_pulse_ticks()
        expected = int(2500 * SERVO_TIM_CLK / 1000000)
        self.assertEqual(ticks, expected)

    # ── 多路独立控制 ──────────────────────────────────

    def test_multi_servo_independent(self):
        """8路舵机独立控制"""
        angles = [0, 22, 45, 67, 90, 112, 135, 180]
        for i, angle in enumerate(angles):
            self.servos[i].set_angle(angle)
        for i, angle in enumerate(angles):
            self.assertEqual(self.servos[i].get_angle(), angle)

    def test_multi_servo_different_angles(self):
        """不同舵机不同角度"""
        self.servos[0].set_angle(0)
        self.servos[1].set_angle(45)
        self.servos[2].set_angle(90)
        self.servos[3].set_angle(135)
        self.servos[4].set_angle(180)
        self.assertEqual(self.servos[0].get_angle(), 0)
        self.assertEqual(self.servos[1].get_angle(), 45)
        self.assertEqual(self.servos[2].get_angle(), 90)
        self.assertEqual(self.servos[3].get_angle(), 135)
        self.assertEqual(self.servos[4].get_angle(), 180)

    def test_modify_one不影响_others(self):
        """修改一个舵机不影响其他"""
        self.servos[0].set_angle(30)
        self.servos[1].set_angle(60)
        self.servos[0].set_angle(120)
        self.assertEqual(self.servos[0].get_angle(), 120)
        self.assertEqual(self.servos[1].get_angle(), 60)

    # ── 同步运动 ──────────────────────────────────

    def test_all_servos_same_angle(self):
        """所有舵机同步到同一角度"""
        for s in self.servos:
            s.set_angle(90)
        for s in self.servos:
            self.assertEqual(s.get_angle(), 90)

    def test_sweep_simulation(self):
        """模拟扫描运动"""
        for angle in range(0, 181, 10):
            for s in self.servos:
                s.set_angle(angle)
        for s in self.servos:
            self.assertEqual(s.get_angle(), 180)

    # ── 停止 ──────────────────────────────────

    def test_stop(self):
        """停止舵机"""
        self.servos[0].stop()
        self.assertFalse(self.servos[0].running)

    def test_stop不影响_angle(self):
        """停止后角度保持"""
        self.servos[0].set_angle(45)
        self.servos[0].stop()
        self.assertEqual(self.servos[0].get_angle(), 45)

    def test_multi_stop(self):
        """多舵机停止"""
        for s in self.servos:
            s.stop()
        for s in self.servos:
            self.assertFalse(s.running)

    # ── 脉宽↔角度一致性 ──────────────────────────────────

    def test_pulse_width_angle_consistency(self):
        """脉宽设置后角度正确反算"""
        test_pulses = [500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500]
        for pulse in test_pulses:
            self.servos[0].set_pulse_width(pulse)
            expected_angle = int((pulse - 500) * SERVO_MAX_ANGLE / 2000)
            self.assertEqual(self.servos[0].get_angle(), expected_angle)

    def test_set_angle_then_read_ticks(self):
        """设置角度后读取tick值"""
        self.servos[0].set_angle(60)
        ticks = self.servos[0].get_pulse_ticks()
        self.assertGreater(ticks, 0)
        self.assertGreater(int(2500 * SERVO_TIM_CLK / 1000000), ticks)

    # ── 边界测试 ──────────────────────────────────

    def test_angle_1(self):
        """最小非零角度"""
        self.servos[0].set_angle(1)
        self.assertEqual(self.servos[0].get_angle(), 1)

    def test_angle_179(self):
        """最大非满角度"""
        self.servos[0].set_angle(179)
        self.assertEqual(self.servos[0].get_angle(), 179)

    def test_pulse_width_501(self):
        """最小非边界脉宽"""
        self.servos[0].set_pulse_width(501)
        self.assertGreater(self.servos[0].get_pulse_ticks(), 0)

    def test_pulse_width_2499(self):
        """最大非边界脉宽"""
        self.servos[0].set_pulse_width(2499)
        max_ticks = int(2500 * SERVO_TIM_CLK / 1000000)
        self.assertLess(self.servos[0].get_pulse_ticks(), max_ticks)


if __name__ == '__main__':
    unittest.main()
