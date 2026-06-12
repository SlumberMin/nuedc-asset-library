#!/usr/bin/env python3
"""
灰度传感器 V3 测试 — 更全面的边界测试
覆盖: V2全部 + 行走决策模拟、交替模式、连续读取稳定性、白线偏差计算、
      多模式掩码组合、灰度值与方向决策关联
对应C源文件: 02_mspm0g3507/drivers/grayscale.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #7:  数组/缓冲区溢出（通道边界）
  #12: 灰度传感器用于巡线决策，需覆盖典型行走模式
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import Grayscale, GRAY_CH0, GRAY_CH7


class TestGrayscaleV3(unittest.TestCase):
    """灰度传感器V3 — 全面边界测试"""

    def setUp(self):
        self.gs = Grayscale()
        self.gs.init()

    # ── 初始化与基本读写 ──

    def test_init_all_white(self):
        """初始化后全部为白(1)"""
        self.assertTrue(self.gs.initialized)
        for ch in range(8):
            self.assertEqual(self.gs.read(ch), 1)

    def test_read_single_white(self):
        """单路白色读取"""
        self.gs.set_channel(0, 1)
        self.assertEqual(self.gs.read(0), 1)

    def test_read_single_black(self):
        """单路黑色读取"""
        self.gs.set_channel(3, 0)
        self.assertEqual(self.gs.read(3), 0)

    # ── 无效通道边界 ──

    def test_read_invalid_channel_high(self):
        """无效通道(>7)返回0xFF"""
        self.assertEqual(self.gs.read(8), 0xFF)

    def test_read_invalid_channel_negative(self):
        """无效通道(<0)返回0xFF"""
        self.assertEqual(self.gs.read(-1), 0xFF)

    def test_read_extreme_negative(self):
        """极端负通道返回0xFF"""
        self.assertEqual(self.gs.read(-100), 0xFF)

    def test_read_extreme_positive(self):
        """极端大通道返回0xFF"""
        self.assertEqual(self.gs.read(255), 0xFF)

    # ── 掩码读取 ──

    def test_read_all_white(self):
        """全部白色: mask = 0xFF"""
        for ch in range(8):
            self.gs.set_channel(ch, 1)
        self.assertEqual(self.gs.read_all(), 0xFF)

    def test_read_all_black(self):
        """全部黑色: mask = 0x00"""
        for ch in range(8):
            self.gs.set_channel(ch, 0)
        self.assertEqual(self.gs.read_all(), 0x00)

    def test_read_all_pattern(self):
        """特定掩码模式: ch0,2,4,6白"""
        for ch in range(8):
            self.gs.set_channel(ch, ch % 2 == 0)
        mask = self.gs.read_all()
        self.assertEqual(mask, 0b01010101)

    def test_mask_bit_mapping(self):
        """掩码位映射: ch N → bit N"""
        for ch in range(8):
            gs2 = Grayscale()
            gs2.init()
            for c in range(8):
                gs2.set_channel(c, 0)
            gs2.set_channel(ch, 1)
            mask = gs2.read_all()
            self.assertEqual(mask, 1 << ch, f"ch{ch} should map to bit{ch}")

    # ── 白色计数 ──

    def test_count_white_all(self):
        """全部白色计数=8"""
        for ch in range(8):
            self.gs.set_channel(ch, 1)
        self.assertEqual(self.gs.count_white(), 8)

    def test_count_white_none(self):
        """全部黑色计数=0"""
        for ch in range(8):
            self.gs.set_channel(ch, 0)
        self.assertEqual(self.gs.count_white(), 0)

    def test_count_white_partial(self):
        """部分白色计数"""
        for ch in range(8):
            self.gs.set_channel(ch, 1 if ch < 3 else 0)
        self.assertEqual(self.gs.count_white(), 3)

    # ── V3: 典型巡线模式测试 ──

    def test_line_center_pattern(self):
        """巡线居中模式: 中间两路黑，其余白"""
        # 模拟黑线恰好在中间ch3和ch4之间
        for ch in range(8):
            self.gs.set_channel(ch, 0 if ch in (3, 4) else 1)
        mask = self.gs.read_all()
        # ch3=0, ch4=0 → bit3=0, bit4=0 → 0b11100111 = 0xE7
        self.assertEqual(mask, 0b11100111)
        self.assertEqual(self.gs.count_white(), 6)

    def test_line_left_pattern(self):
        """巡线偏左模式: 黑线在左侧"""
        for ch in range(8):
            self.gs.set_channel(ch, 0 if ch in (0, 1, 2) else 1)
        self.assertEqual(self.gs.count_white(), 5)

    def test_line_right_pattern(self):
        """巡线偏右模式: 黑线在右侧"""
        for ch in range(8):
            self.gs.set_channel(ch, 0 if ch in (5, 6, 7) else 1)
        self.assertEqual(self.gs.count_white(), 5)

    def test_line_lost_all_black(self):
        """丢线模式: 全黑（可能是十字路口或脱离赛道）"""
        for ch in range(8):
            self.gs.set_channel(ch, 0)
        self.assertEqual(self.gs.count_white(), 0)
        self.assertEqual(self.gs.read_all(), 0x00)

    def test_line_lost_all_white(self):
        """全白模式: 可能是起跑线或十字路口"""
        for ch in range(8):
            self.gs.set_channel(ch, 1)
        self.assertEqual(self.gs.count_white(), 8)
        self.assertEqual(self.gs.read_all(), 0xFF)

    # ── V3: 偏差计算模拟 ──

    def test_deviation_center(self):
        """偏差计算: 居中时偏差应为0"""
        # 居中: ch3, ch4为黑
        values = [1, 1, 1, 0, 0, 1, 1, 1]
        for ch, v in enumerate(values):
            self.gs.set_channel(ch, v)
        # 加权偏差: Σ(ch_i * black_i) / Σ(black_i) - 3.5
        black_positions = [i for i, v in enumerate(values) if v == 0]
        if black_positions:
            deviation = sum(black_positions) / len(black_positions) - 3.5
        else:
            deviation = 0
        self.assertAlmostEqual(deviation, 0.0, places=1)

    def test_deviation_left(self):
        """偏差计算: 偏左时偏差为负"""
        values = [0, 0, 1, 1, 1, 1, 1, 1]
        for ch, v in enumerate(values):
            self.gs.set_channel(ch, v)
        black_positions = [i for i, v in enumerate(values) if v == 0]
        deviation = sum(black_positions) / len(black_positions) - 3.5
        self.assertLess(deviation, 0)

    def test_deviation_right(self):
        """偏差计算: 偏右时偏差为正"""
        values = [1, 1, 1, 1, 1, 1, 0, 0]
        for ch, v in enumerate(values):
            self.gs.set_channel(ch, v)
        black_positions = [i for i, v in enumerate(values) if v == 0]
        deviation = sum(black_positions) / len(black_positions) - 3.5
        self.assertGreater(deviation, 0)

    # ── V3: set_channel越界安全性 ──

    def test_set_channel_out_of_range_ignored(self):
        """越界set_channel不应崩溃"""
        self.gs.set_channel(-1, 1)
        self.gs.set_channel(10, 1)
        self.gs.set_channel(100, 0)

    def test_set_channel_value_truthy(self):
        """set_channel的value参数使用truthy判断"""
        self.gs.set_channel(0, 2)  # 非0即为白
        self.assertEqual(self.gs.read(0), 1)
        self.gs.set_channel(0, 0)  # 0为黑
        self.assertEqual(self.gs.read(0), 0)

    # ── V3: 连续读取一致性 ──

    def test_repeated_read_stability(self):
        """连续多次读取应稳定"""
        self.gs.set_channel(0, 0)
        self.gs.set_channel(7, 0)
        for _ in range(100):
            self.assertEqual(self.gs.read(0), 0)
            self.assertEqual(self.gs.read(7), 0)
            self.assertEqual(self.gs.read(3), 1)

    # ── V3: 交错读写 ──

    def test_interleaved_read_write(self):
        """交错读写不互相干扰"""
        # 全黑后逐个改白再改回黑，验证读写独立
        for i in range(8):
            self.gs.set_channel(i, 0)
        for i in range(8):
            self.gs.set_channel(i, 1)
            self.assertEqual(self.gs.read(i), 1)
            self.gs.set_channel(i, 0)  # 改回黑
            self.assertEqual(self.gs.read(i), 0)

    # ── V3: 常量验证 ──

    def test_channel_constants(self):
        """通道常量正确"""
        self.assertEqual(GRAY_CH0, 0)
        self.assertEqual(GRAY_CH7, 7)


if __name__ == '__main__':
    unittest.main()
