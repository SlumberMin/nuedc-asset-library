#!/usr/bin/env python3
"""
旋转编码器 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、CW/CCW旋转、按钮点击、位置查询、重置
对应C源文件: 02_mspm0g3507/drivers/rotary_encoder.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #5:  中断注入模拟旋转事件
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    RotaryEncoder,
    ROTARY_DIR_NONE, ROTARY_DIR_CW, ROTARY_DIR_CCW,
)


class TestRotaryEncoderV2(unittest.TestCase):
    """旋转编码器V2 — 基于wrappers.py包装层"""

    def setUp(self):
        self.enc = RotaryEncoder()
        self.enc.init()

    def test_init_state(self):
        """初始化后状态正确"""
        self.assertTrue(self.enc.initialized)
        self.assertEqual(self.enc.position, 0)
        self.assertEqual(self.enc.direction, ROTARY_DIR_NONE)
        self.assertFalse(self.enc.button_pressed)
        self.assertEqual(self.enc.clicks, 0)

    def test_cw_rotation(self):
        """顺时针旋转"""
        for _ in range(10):
            self.enc._inject_rotation(ROTARY_DIR_CW)
        self.assertEqual(self.enc.position, 10)
        self.assertEqual(self.enc.direction, ROTARY_DIR_CW)

    def test_ccw_rotation(self):
        """逆时针旋转"""
        for _ in range(5):
            self.enc._inject_rotation(ROTARY_DIR_CCW)
        self.assertEqual(self.enc.position, -5)
        self.assertEqual(self.enc.direction, ROTARY_DIR_CCW)

    def test_mixed_rotation(self):
        """混合方向旋转"""
        self.enc._inject_rotation(ROTARY_DIR_CW)
        self.enc._inject_rotation(ROTARY_DIR_CW)
        self.enc._inject_rotation(ROTARY_DIR_CW)
        self.enc._inject_rotation(ROTARY_DIR_CCW)
        self.assertEqual(self.enc.position, 2)
        self.assertEqual(self.enc.direction, ROTARY_DIR_CCW)

    def test_get_position(self):
        """位置查询"""
        self.assertEqual(self.enc.get_position(), 0)
        self.enc._inject_rotation(ROTARY_DIR_CW)
        self.enc._inject_rotation(ROTARY_DIR_CW)
        self.assertEqual(self.enc.get_position(), 2)

    def test_get_direction(self):
        """方向查询"""
        self.assertEqual(self.enc.get_direction(), ROTARY_DIR_NONE)
        self.enc._inject_rotation(ROTARY_DIR_CW)
        self.assertEqual(self.enc.get_direction(), ROTARY_DIR_CW)
        self.enc._inject_rotation(ROTARY_DIR_CCW)
        self.assertEqual(self.enc.get_direction(), ROTARY_DIR_CCW)

    def test_button_press(self):
        """按钮按下"""
        self.enc._inject_button_press()
        self.assertTrue(self.enc.is_pressed())
        self.assertEqual(self.enc.get_clicks(), 1)

    def test_button_release(self):
        """按钮释放"""
        self.enc._inject_button_press()
        self.enc._inject_button_release()
        self.assertFalse(self.enc.is_pressed())

    def test_multiple_clicks(self):
        """多次点击计数"""
        for _ in range(5):
            self.enc._inject_button_press()
            self.enc._inject_button_release()
        self.assertEqual(self.enc.get_clicks(), 5)

    def test_reset(self):
        """重置所有状态"""
        self.enc._inject_rotation(ROTARY_DIR_CW)
        self.enc._inject_rotation(ROTARY_DIR_CW)
        self.enc._inject_button_press()
        self.enc.reset()
        self.assertEqual(self.enc.position, 0)
        self.assertEqual(self.enc.direction, ROTARY_DIR_NONE)
        self.assertEqual(self.enc.clicks, 0)

    def test_zero_rotation_no_direction_change(self):
        """无旋转时方向不变"""
        self.assertEqual(self.enc.get_direction(), ROTARY_DIR_NONE)

    def test_position_bidirectional(self):
        """双向旋转后位置正确"""
        # 正转5步
        for _ in range(5):
            self.enc._inject_rotation(ROTARY_DIR_CW)
        # 反转8步
        for _ in range(8):
            self.enc._inject_rotation(ROTARY_DIR_CCW)
        self.assertEqual(self.enc.position, -3)
        self.assertEqual(self.enc.direction, ROTARY_DIR_CCW)

    def test_large_rotation(self):
        """大量旋转"""
        for _ in range(1000):
            self.enc._inject_rotation(ROTARY_DIR_CW)
        self.assertEqual(self.enc.position, 1000)

    def test_button_with_rotation(self):
        """旋转与按钮交替操作"""
        self.enc._inject_rotation(ROTARY_DIR_CW)
        self.enc._inject_button_press()
        self.enc._inject_rotation(ROTARY_DIR_CW)
        self.enc._inject_button_press()
        self.assertEqual(self.enc.position, 2)
        self.assertEqual(self.enc.clicks, 2)


if __name__ == '__main__':
    unittest.main()
