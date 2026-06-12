#!/usr/bin/env python3
"""
PCA9685 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、PWM频率设置、通道PWM、角度映射、边界条件、常量
对应C源文件: 02_mspm0g3507/drivers/pca9685.h

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #1:  除零保护（频率计算）
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    PCA9685, PCA9685_ADDR, PCA9685_OSC_FREQ, PCA9685_PWM_STEPS,
    PCA9685_NUM_CHANNELS,
)


class TestPCA9685V2Init(unittest.TestCase):
    """初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        drv = PCA9685()
        ok = drv.init()
        self.assertTrue(ok)
        self.assertTrue(drv.initialized)

    def test_default_freq(self):
        """初始化后默认50Hz"""
        drv = PCA9685()
        drv.init()
        self.assertEqual(drv.freq, 50)

    def test_default_addr(self):
        """默认I2C地址0x40"""
        drv = PCA9685()
        self.assertEqual(drv.addr, PCA9685_ADDR)

    def test_all_channels_zero(self):
        """初始化后所有通道PWM=0"""
        drv = PCA9685()
        drv.init()
        for ch in range(PCA9685_NUM_CHANNELS):
            on, off = drv.get_pwm(ch)
            self.assertEqual(on, 0)
            self.assertEqual(off, 0)


class TestPCA9685V2Freq(unittest.TestCase):
    """PWM频率设置测试"""

    def test_set_50hz(self):
        """设置50Hz（舵机标准）"""
        drv = PCA9685()
        drv.init()
        ok = drv.set_pwm_freq(50)
        self.assertTrue(ok)
        self.assertEqual(drv.freq, 50)

    def test_set_60hz(self):
        """设置60Hz"""
        drv = PCA9685()
        drv.init()
        ok = drv.set_pwm_freq(60)
        self.assertTrue(ok)

    def test_set_1000hz(self):
        """设置1kHz"""
        drv = PCA9685()
        drv.init()
        ok = drv.set_pwm_freq(1000)
        self.assertTrue(ok)

    def test_freq_too_low(self):
        """过低频率应失败"""
        drv = PCA9685()
        drv.init()
        ok = drv.set_pwm_freq(10)
        self.assertFalse(ok)

    def test_freq_too_high(self):
        """过高频率应失败"""
        drv = PCA9685()
        drv.init()
        ok = drv.set_pwm_freq(2000)
        self.assertFalse(ok)


class TestPCA9685V2PWM(unittest.TestCase):
    """PWM通道测试"""

    def test_set_pwm_channel0(self):
        """设置通道0 PWM"""
        drv = PCA9685()
        drv.init()
        ok = drv.set_pwm(0, 0, 307)
        self.assertTrue(ok)
        on, off = drv.get_pwm(0)
        self.assertEqual(on, 0)
        self.assertEqual(off, 307)

    def test_set_pwm_all_channels(self):
        """设置所有16通道"""
        drv = PCA9685()
        drv.init()
        for ch in range(PCA9685_NUM_CHANNELS):
            ok = drv.set_pwm(ch, 0, ch * 10)
            self.assertTrue(ok)

    def test_set_pwm_invalid_channel(self):
        """无效通道号应失败"""
        drv = PCA9685()
        drv.init()
        self.assertFalse(drv.set_pwm(-1, 0, 100))
        self.assertFalse(drv.set_pwm(16, 0, 100))

    def test_pwm_value_mask(self):
        """PWM值应被12位掩码"""
        drv = PCA9685()
        drv.init()
        drv.set_pwm(0, 0, 0xFFFF)
        on, off = drv.get_pwm(0)
        self.assertEqual(off, 0x0FFF)

    def test_all_off(self):
        """关闭所有通道"""
        drv = PCA9685()
        drv.init()
        for ch in range(PCA9685_NUM_CHANNELS):
            drv.set_pwm(ch, 100, 300)
        drv.all_off()
        for ch in range(PCA9685_NUM_CHANNELS):
            on, off = drv.get_pwm(ch)
            self.assertEqual(on, 0)
            self.assertEqual(off, 0)


class TestPCA9685V2Angle(unittest.TestCase):
    """舵机角度映射测试"""

    def test_angle_0(self):
        """0° → off=102"""
        drv = PCA9685()
        drv.init()
        ok = drv.set_angle(0, 0)
        self.assertTrue(ok)
        _, off = drv.get_pwm(0)
        self.assertEqual(off, 102)

    def test_angle_90(self):
        """90° → off≈307"""
        drv = PCA9685()
        drv.init()
        drv.set_angle(0, 90)
        _, off = drv.get_pwm(0)
        self.assertAlmostEqual(off, 307, delta=1)

    def test_angle_180(self):
        """180° → off=512"""
        drv = PCA9685()
        drv.init()
        drv.set_angle(0, 180)
        _, off = drv.get_pwm(0)
        self.assertEqual(off, 512)

    def test_angle_clamp_low(self):
        """角度<0应钳位到0"""
        drv = PCA9685()
        drv.init()
        drv.set_angle(0, -10)
        _, off = drv.get_pwm(0)
        self.assertEqual(off, 102)

    def test_angle_clamp_high(self):
        """角度>180应钳位到180"""
        drv = PCA9685()
        drv.init()
        drv.set_angle(0, 200)
        _, off = drv.get_pwm(0)
        self.assertEqual(off, 512)

    def test_angle_invalid_channel(self):
        """无效通道设置角度应失败"""
        drv = PCA9685()
        drv.init()
        self.assertFalse(drv.set_angle(-1, 90))
        self.assertFalse(drv.set_angle(16, 90))


class TestPCA9685V2Constants(unittest.TestCase):
    """常量一致性"""

    def test_constants(self):
        self.assertEqual(PCA9685_ADDR, 0x40)
        self.assertEqual(PCA9685_OSC_FREQ, 25000000)
        self.assertEqual(PCA9685_PWM_STEPS, 4096)
        self.assertEqual(PCA9685_NUM_CHANNELS, 16)


if __name__ == '__main__':
    unittest.main()
