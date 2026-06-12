#!/usr/bin/env python3
"""
红外循迹传感器单元测试
覆盖: SensorIR 的位置计算、加权平均、在线检测、十字路口检测、数字/ADC模式
注意: 使用纯 Python 模拟 C 结构体逻辑，测试算法正确性
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class SensorIRSimulator:
    """模拟 SensorIR_t 结构体的 Python 实现"""

    def __init__(self, channel_count=5, weights=None, threshold=2048):
        self.channel_count = channel_count
        self.threshold = threshold
        self.weights = weights or self._default_weights(channel_count)
        self.raw_value = [0] * channel_count
        self.digital = [False] * channel_count
        self.position = 0.0
        self.on_line = False
        self.cross_detected = False

    @staticmethod
    def _default_weights(count):
        """默认对称权重: 5路→[-4,-2,0,2,4]"""
        if count == 5:
            return [-4, -2, 0, 2, 4]
        elif count == 3:
            return [-2, 0, 2]
        elif count == 7:
            return [-3, -2, -1, 0, 1, 2, 3]
        else:
            # 通用: 等间距对称
            half = count // 2
            return [i - half for i in range(count)]

    def update_adc(self, raw_values):
        """ADC模式: 读取原始值，计算数字状态和位置"""
        self.raw_value = list(raw_values[:self.channel_count])
        # 黑线上=低ADC → digital=False; 白地=高ADC → digital=True
        self.digital = [v > self.threshold for v in self.raw_value]

        # 加权平均计算位置
        weight_sum = 0.0
        count_on = 0
        for i in range(self.channel_count):
            if not self.digital[i]:  # 在黑线上(digital=False)
                weight_sum += self.weights[i]
                count_on += 1

        if count_on > 0:
            self.position = weight_sum / count_on
            self.on_line = True
        else:
            self.position = 0.0
            self.on_line = False

        # 十字路口: 所有传感器都在黑线上
        self.cross_detected = all(not d for d in self.digital)

    def update_gpio(self, states):
        """GPIO模式: 直接读取数字状态"""
        self.digital = list(states[:self.channel_count])

        weight_sum = 0.0
        count_on = 0
        for i in range(self.channel_count):
            if not self.digital[i]:
                weight_sum += self.weights[i]
                count_on += 1

        if count_on > 0:
            self.position = weight_sum / count_on
            self.on_line = True
        else:
            self.position = 0.0
            self.on_line = False

        self.cross_detected = all(not d for d in self.digital)


class TestSensorIRInit(unittest.TestCase):
    """初始化测试"""

    def test_default_5ch_weights(self):
        """5路默认权重应为[-4,-2,0,2,4]"""
        ir = SensorIRSimulator(channel_count=5)
        self.assertEqual(ir.weights, [-4, -2, 0, 2, 4])

    def test_3ch_weights(self):
        """3路默认权重应为[-2,0,2]"""
        ir = SensorIRSimulator(channel_count=3)
        self.assertEqual(ir.weights, [-2, 0, 2])

    def test_custom_weights(self):
        """自定义权重"""
        ir = SensorIRSimulator(channel_count=5, weights=[-10, -5, 0, 5, 10])
        self.assertEqual(ir.weights, [-10, -5, 0, 5, 10])

    def test_initial_state(self):
        """初始状态: 不在线上"""
        ir = SensorIRSimulator()
        self.assertFalse(ir.on_line)
        self.assertFalse(ir.cross_detected)


class TestSensorIRPosition(unittest.TestCase):
    """位置计算测试"""

    def test_center_position(self):
        """中间传感器触发: position≈0"""
        ir = SensorIRSimulator(channel_count=5, threshold=500)
        # 只有中间传感器(索引2)在线上(低ADC)
        ir.update_adc([100, 100, 200, 100, 100])
        # digital: [True, True, False, True, True] → 只有index=2在线上
        self.assertAlmostEqual(ir.position, 0.0)

    def test_left_position(self):
        """左侧传感器触发: position<0"""
        ir = SensorIRSimulator(channel_count=5, threshold=500)
        ir.update_adc([200, 200, 100, 100, 100])
        # digital: [False, False, True, True, True]
        # 在线上的: index 0(-4), index 1(-2)
        self.assertLess(ir.position, 0)

    def test_right_position(self):
        """右侧传感器触发: position>0"""
        ir = SensorIRSimulator(channel_count=5, threshold=500)
        ir.update_adc([100, 100, 100, 200, 200])
        # digital: [True, True, True, False, False]
        # 在线上的: index 3(2), index 4(4)
        self.assertGreater(ir.position, 0)

    def test_all_on_line_position(self):
        """全部在线上: position=0"""
        ir = SensorIRSimulator(channel_count=5, threshold=500)
        ir.update_adc([200, 200, 200, 200, 200])
        # 全部在线上 → position = sum(weights)/count = 0
        self.assertAlmostEqual(ir.position, 0.0)

    def test_no_line_position(self):
        """全不在线上: position=0"""
        ir = SensorIRSimulator(channel_count=5, threshold=500)
        ir.update_adc([1000, 1000, 1000, 1000, 1000])
        self.assertAlmostEqual(ir.position, 0.0)
        self.assertFalse(ir.on_line)


class TestSensorIROnLine(unittest.TestCase):
    """在线检测测试"""

    def test_on_line_when_any_low(self):
        """任一传感器在线上时on_line=True"""
        ir = SensorIRSimulator(channel_count=5, threshold=500)
        ir.update_adc([1000, 1000, 200, 1000, 1000])
        self.assertTrue(ir.on_line)

    def test_off_line_when_all_high(self):
        """全白地时on_line=False"""
        ir = SensorIRSimulator(channel_count=5, threshold=500)
        ir.update_adc([1000, 1000, 1000, 1000, 1000])
        self.assertFalse(ir.on_line)


class TestSensorIRCrossDetect(unittest.TestCase):
    """十字路口检测测试"""

    def test_cross_detected_all_low(self):
        """全部传感器检测到黑线 → 十字路口"""
        ir = SensorIRSimulator(channel_count=5, threshold=500)
        ir.update_adc([200, 200, 200, 200, 200])
        self.assertTrue(ir.cross_detected)

    def test_no_cross_partial(self):
        """部分传感器在线上 → 非十字路口"""
        ir = SensorIRSimulator(channel_count=5, threshold=500)
        ir.update_adc([200, 200, 200, 1000, 1000])
        self.assertFalse(ir.cross_detected)

    def test_no_cross_all_white(self):
        """全白地 → 非十字路口"""
        ir = SensorIRSimulator(channel_count=5, threshold=500)
        ir.update_adc([1000, 1000, 1000, 1000, 1000])
        self.assertFalse(ir.cross_detected)


class TestSensorIRGPIOMode(unittest.TestCase):
    """GPIO数字模式测试"""

    def test_gpio_on_line(self):
        """GPIO模式: False=黑线, True=白地"""
        ir = SensorIRSimulator(channel_count=5)
        # 中间传感器在线上
        ir.update_gpio([True, True, False, True, True])
        self.assertTrue(ir.on_line)
        self.assertAlmostEqual(ir.position, 0.0)

    def test_gpio_cross(self):
        """GPIO模式十字路口"""
        ir = SensorIRSimulator(channel_count=5)
        ir.update_gpio([False, False, False, False, False])
        self.assertTrue(ir.cross_detected)


class TestSensorIRThreshold(unittest.TestCase):
    """阈值灵敏度测试"""

    def test_threshold_boundary(self):
        """恰好等于阈值: 应视为白地(>threshold)"""
        ir = SensorIRSimulator(channel_count=5, threshold=500)
        ir.update_adc([500, 500, 500, 500, 500])
        # 500 > 500 is False → 全在线上
        self.assertTrue(ir.cross_detected)

    def test_high_threshold(self):
        """高阈值: 更多被视为黑线"""
        ir = SensorIRSimulator(channel_count=5, threshold=3000)
        ir.update_adc([1000, 1000, 1000, 1000, 1000])
        # 1000 < 3000 → 全部在线上
        self.assertTrue(ir.cross_detected)


if __name__ == '__main__':
    unittest.main()
