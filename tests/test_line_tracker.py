#!/usr/bin/env python3
"""
循迹算法 V2测试 — 基于wrappers.py包装层
覆盖: 灰度传感器读取、线检测、偏差计算、PID循迹逻辑
对应C源文件: 02_mspm0g3507/drivers/grayscale.c + line_tracker算法

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    Grayscale, PIDController,
    GRAY_CH0, GRAY_CH1, GRAY_CH2, GRAY_CH3,
    GRAY_CH4, GRAY_CH5, GRAY_CH6, GRAY_CH7,
)


class LineTracker:
    """循迹算法 — 基于8路灰度传感器

    与C版本line_tracker.c逻辑一致:
    - 加权平均计算线位置偏差
    - PID输出转向量
    """

    # 传感器位置权重 (-3.5 ~ +3.5)
    WEIGHTS = [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]

    def __init__(self, pid_kp=10.0, pid_ki=0.0, pid_kd=2.0):
        self.sensor = Grayscale()
        self.pid = PIDController(kp=pid_kp, ki=pid_ki, kd=pid_kd,
                                  output_min=-100, output_max=100)
        self.error = 0.0
        self.on_line = True

    def init(self):
        """初始化"""
        self.sensor.init()
        self.pid.reset()

    def update(self):
        """更新循迹计算

        返回: (steer: float, on_line: bool)
        steer: 正值向右转，负值向左转
        """
        mask = self.sensor.read_all()

        # 检查是否在线上
        if mask == 0:
            self.on_line = False
            return self.pid.output, False

        self.on_line = True

        # 加权平均计算偏差
        weighted_sum = 0.0
        weight_count = 0
        for i in range(8):
            val = self.sensor.read(i)
            if val == 1:  # 白色(在线上)
                weighted_sum += self.WEIGHTS[i]
                weight_count += 1

        if weight_count > 0:
            self.error = weighted_sum / weight_count
        else:
            self.error = 0.0

        steer = self.pid.calc(0, self.error)
        return steer, True


class TestLineTrackerInit(unittest.TestCase):
    """循迹初始化测试"""

    def test_init(self):
        """初始化成功"""
        tracker = LineTracker()
        tracker.init()
        self.assertTrue(tracker.sensor.initialized)

    def test_default_weights(self):
        """权重对称分布"""
        w = LineTracker.WEIGHTS
        self.assertEqual(len(w), 8)
        self.assertAlmostEqual(w[0] + w[7], 0.0)
        self.assertAlmostEqual(w[3] + w[4], 0.0)


class TestLineTrackerSensor(unittest.TestCase):
    """灰度传感器测试"""

    def test_all_white(self):
        """全白 → 全1"""
        gs = Grayscale()
        gs.init()
        for ch in range(8):
            gs.set_channel(ch, 1)
        self.assertEqual(gs.read_all(), 0xFF)
        self.assertEqual(gs.count_white(), 8)

    def test_all_black(self):
        """全黑 → 全0"""
        gs = Grayscale()
        gs.init()
        for ch in range(8):
            gs.set_channel(ch, 0)
        self.assertEqual(gs.read_all(), 0x00)
        self.assertEqual(gs.count_white(), 0)

    def test_center_line(self):
        """中心线上: 只有CH3/CH4白"""
        gs = Grayscale()
        gs.init()
        for ch in range(8):
            gs.set_channel(ch, 0)
        gs.set_channel(GRAY_CH3, 1)
        gs.set_channel(GRAY_CH4, 1)
        self.assertEqual(gs.read(GRAY_CH3), 1)
        self.assertEqual(gs.read(GRAY_CH4), 1)
        self.assertEqual(gs.read(GRAY_CH0), 0)

    def test_invalid_channel(self):
        """无效通道返回0xFF"""
        gs = Grayscale()
        gs.init()
        self.assertEqual(gs.read(-1), 0xFF)
        self.assertEqual(gs.read(8), 0xFF)


class TestLineTrackerTracking(unittest.TestCase):
    """循迹跟踪测试"""

    def _setup_tracker(self, channel_values):
        """辅助: 创建循迹器并设置传感器值"""
        tracker = LineTracker()
        tracker.init()
        for ch, val in enumerate(channel_values):
            tracker.sensor.set_channel(ch, val)
        return tracker

    def test_center_on_line(self):
        """居中线上 → 偏差接近0"""
        # 只有中间两路在线上
        vals = [0, 0, 0, 1, 1, 0, 0, 0]
        tracker = self._setup_tracker(vals)
        steer, on_line = tracker.update()
        self.assertTrue(on_line)
        # CH3权重-0.5, CH4权重+0.5, 平均=0
        self.assertAlmostEqual(tracker.error, 0.0, places=1)

    def test_drift_left(self):
        """偏左 → 负偏差 → 正转向(向右修正)"""
        vals = [0, 0, 1, 1, 0, 0, 0, 0]
        tracker = self._setup_tracker(vals)
        steer, on_line = tracker.update()
        self.assertTrue(on_line)
        self.assertLess(tracker.error, 0)  # 偏左为负
        self.assertGreater(steer, 0)  # PID输出正(向右修正)

    def test_drift_right(self):
        """偏右 → 正偏差 → 负转向(向左修正)"""
        vals = [0, 0, 0, 0, 1, 1, 0, 0]
        tracker = self._setup_tracker(vals)
        steer, on_line = tracker.update()
        self.assertTrue(on_line)
        self.assertGreater(tracker.error, 0)  # 偏右为正
        self.assertLess(steer, 0)  # PID输出负(向左修正)

    def test_off_line(self):
        """全黑 → 脱线"""
        vals = [0, 0, 0, 0, 0, 0, 0, 0]
        tracker = self._setup_tracker(vals)
        steer, on_line = tracker.update()
        self.assertFalse(on_line)

    def test_all_white(self):
        """全白 → 在线(十字路口)"""
        vals = [1, 1, 1, 1, 1, 1, 1, 1]
        tracker = self._setup_tracker(vals)
        steer, on_line = tracker.update()
        self.assertTrue(on_line)
        # 权重对称，平均=0
        self.assertAlmostEqual(tracker.error, 0.0, places=1)

    def test_extreme_left(self):
        """极左偏: 只有CH0白"""
        vals = [1, 0, 0, 0, 0, 0, 0, 0]
        tracker = self._setup_tracker(vals)
        steer, on_line = tracker.update()
        self.assertTrue(on_line)
        self.assertAlmostEqual(tracker.error, -3.5, places=1)

    def test_extreme_right(self):
        """极右偏: 只有CH7白"""
        vals = [0, 0, 0, 0, 0, 0, 0, 1]
        tracker = self._setup_tracker(vals)
        steer, on_line = tracker.update()
        self.assertTrue(on_line)
        self.assertAlmostEqual(tracker.error, 3.5, places=1)


class TestLineTrackerPID(unittest.TestCase):
    """循迹PID控制测试"""

    def test_pid_output_clamp(self):
        """PID输出限幅 [-100, 100]"""
        pid = PIDController(kp=50.0, ki=0.0, kd=0.0,
                            output_min=-100, output_max=100)
        out = pid.calc(0, -3.5)
        self.assertGreaterEqual(out, -100)
        self.assertLessEqual(out, 100)

    def test_pid_steady_state(self):
        """持续偏差 → 积分累积"""
        pid = PIDController(kp=1.0, ki=1.0, kd=0.0,
                            output_min=-1000, output_max=1000)
        for _ in range(100):
            pid.calc(1.0, 0.0)  # error = 1.0 > 0
        self.assertGreater(pid.integral, 0)

    def test_pid_reset(self):
        """PID重置清零状态"""
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
        pid.calc(10, 0)
        pid.reset()
        self.assertAlmostEqual(pid.integral, 0.0)
        self.assertAlmostEqual(pid.prev_error, 0.0)


if __name__ == '__main__':
    unittest.main()
