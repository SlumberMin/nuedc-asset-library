#!/usr/bin/env python3
"""
蜂鸣器 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、开关控制、频率/占空比设置、音符播放、蜂鸣模式
对应C源文件: 02_mspm0g3507/drivers/buzzer.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #7:  频率范围限幅保护
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    Buzzer,
    BUZZER_STATE_OFF, BUZZER_STATE_ON, BUZZER_STATE_BEEP,
    BUZZER_FREQ_MIN, BUZZER_FREQ_MAX,
    BUZZER_NOTES,
)


class TestBuzzerV2(unittest.TestCase):
    """蜂鸣器V2 — 基于wrappers.py包装层"""

    def setUp(self):
        self.buz = Buzzer()
        self.buz.init()

    def test_init_state(self):
        """初始化后状态正确"""
        self.assertTrue(self.buz.initialized)
        self.assertEqual(self.buz.state, BUZZER_STATE_OFF)
        self.assertEqual(self.buz.frequency, 1000)
        self.assertEqual(self.buz.duty, 50)

    def test_on(self):
        """持续鸣响"""
        self.buz.on()
        self.assertEqual(self.buz.state, BUZZER_STATE_ON)
        self.assertTrue(self.buz.is_on())

    def test_off(self):
        """关闭蜂鸣器"""
        self.buz.on()
        self.buz.off()
        self.assertEqual(self.buz.state, BUZZER_STATE_OFF)
        self.assertFalse(self.buz.is_on())

    def test_on_off_toggle(self):
        """开关切换"""
        self.buz.on()
        self.assertTrue(self.buz.is_on())
        self.buz.off()
        self.assertFalse(self.buz.is_on())
        self.buz.on()
        self.assertTrue(self.buz.is_on())

    def test_set_frequency_normal(self):
        """正常频率设置"""
        self.buz.set_frequency(2000)
        self.assertEqual(self.buz.frequency, 2000)

    def test_set_frequency_clamp_min(self):
        """频率下限限幅"""
        self.buz.set_frequency(10)
        self.assertEqual(self.buz.frequency, BUZZER_FREQ_MIN)

    def test_set_frequency_clamp_max(self):
        """频率上限限幅"""
        self.buz.set_frequency(99999)
        self.assertEqual(self.buz.frequency, BUZZER_FREQ_MAX)

    def test_set_frequency_boundary(self):
        """频率边界值"""
        self.buz.set_frequency(BUZZER_FREQ_MIN)
        self.assertEqual(self.buz.frequency, BUZZER_FREQ_MIN)
        self.buz.set_frequency(BUZZER_FREQ_MAX)
        self.assertEqual(self.buz.frequency, BUZZER_FREQ_MAX)

    def test_set_duty_normal(self):
        """正常占空比设置"""
        self.buz.set_duty(75)
        self.assertEqual(self.buz.duty, 75)

    def test_set_duty_clamp_min(self):
        """占空比下限限幅"""
        self.buz.set_duty(-10)
        self.assertEqual(self.buz.duty, 0)

    def test_set_duty_clamp_max(self):
        """占空比上限限幅"""
        self.buz.set_duty(150)
        self.assertEqual(self.buz.duty, 100)

    def test_play_note_valid(self):
        """播放有效音符"""
        self.assertTrue(self.buz.play_note('A4'))
        self.assertEqual(self.buz.frequency, 440)
        self.assertTrue(self.buz.is_on())

    def test_play_note_c5(self):
        """播放C5音符"""
        self.assertTrue(self.buz.play_note('C5'))
        self.assertEqual(self.buz.frequency, 523)

    def test_play_note_invalid(self):
        """播放无效音符"""
        self.assertFalse(self.buz.play_note('X9'))
        self.assertFalse(self.buz.is_on())

    def test_play_all_notes(self):
        """遍历所有音符"""
        for note_name, freq in BUZZER_NOTES.items():
            self.assertTrue(self.buz.play_note(note_name))
            self.assertEqual(self.buz.frequency, freq)
            self.buz.off()

    def test_beep_single(self):
        """单次蜂鸣模式"""
        self.buz.beep(count=1, on_ms=200, off_ms=100)
        self.assertEqual(self.buz.state, BUZZER_STATE_BEEP)
        self.assertEqual(self.buz.total_beeps, 1)
        self.assertEqual(self.buz.beep_on_ms, 200)
        self.assertEqual(self.buz.beep_off_ms, 100)

    def test_beep_tick_complete(self):
        """蜂鸣tick完成后自动关闭"""
        self.buz.beep(count=3, on_ms=100, off_ms=100)
        # 模拟时间流逝: 3个完整周期 = 3 * (100+100) = 600ms
        self.buz.tick(600)
        self.assertEqual(self.buz.state, BUZZER_STATE_OFF)
        self.assertEqual(self.buz.beep_count, 3)

    def test_beep_tick_partial(self):
        """蜂鸣tick部分完成"""
        self.buz.beep(count=5, on_ms=100, off_ms=100)
        # 1.5个周期 = 300ms -> 应完成1个完整周期
        self.buz.tick(300)
        self.assertEqual(self.buz.state, BUZZER_STATE_BEEP)
        self.assertEqual(self.buz.beep_count, 1)

    def test_tick_when_not_beeping(self):
        """非蜂鸣状态下tick无操作"""
        self.buz.off()
        self.buz.tick(1000)
        self.assertEqual(self.buz.state, BUZZER_STATE_OFF)

    def test_get_state(self):
        """状态查询"""
        self.assertEqual(self.buz.get_state(), BUZZER_STATE_OFF)
        self.buz.on()
        self.assertEqual(self.buz.get_state(), BUZZER_STATE_ON)
        self.buz.beep(1)
        self.assertEqual(self.buz.get_state(), BUZZER_STATE_BEEP)

    def test_beep_single_cycle(self):
        """单周期蜂鸣"""
        self.buz.beep(count=1, on_ms=50, off_ms=50)
        self.buz.tick(50)  # 0.5个周期，还未完成
        self.assertEqual(self.buz.beep_count, 0)
        self.buz.tick(50)  # 1个周期，完成
        self.assertEqual(self.buz.beep_count, 1)
        self.assertEqual(self.buz.state, BUZZER_STATE_OFF)

    def test_beep_incremental_tick(self):
        """逐步tick蜂鸣"""
        self.buz.beep(count=2, on_ms=100, off_ms=50)
        # 逐步tick
        for _ in range(150):  # 150ms = 1个周期
            self.buz.tick(1)
        self.assertEqual(self.buz.beep_count, 1)
        self.assertEqual(self.buz.state, BUZZER_STATE_BEEP)
        for _ in range(150):  # 300ms = 2个周期
            self.buz.tick(1)
        self.assertEqual(self.buz.beep_count, 2)
        self.assertEqual(self.buz.state, BUZZER_STATE_OFF)


if __name__ == '__main__':
    unittest.main()
