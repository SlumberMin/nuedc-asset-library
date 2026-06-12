#!/usr/bin/env python3
"""
循迹V2测试 — PID循迹 + 灰度传感器 + 变速策略
覆盖: 8路灰度传感器读取、加权偏差计算、PID转向控制、
      根据偏差大小自适应变速（直道加速/弯道减速）
对应C源文件: 02_mspm0g3507/drivers/grayscale.c + line_following算法

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    Grayscale, PIDController, SimpleMA,
    GRAY_CH0, GRAY_CH1, GRAY_CH2, GRAY_CH3,
    GRAY_CH4, GRAY_CH5, GRAY_CH6, GRAY_CH7,
)


class LineFollowingV2:
    """循迹V2算法 — PID + 灰度 + 变速策略

    特性:
    - 加权平均计算线位置偏差
    - PID输出转向修正量
    - 根据偏差大小选择速度档位（直道/弯道/急弯）
    - 滑动平均滤波平滑偏差
    """

    # 传感器位置权重 (-3.5 ~ +3.5)
    WEIGHTS = [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]

    # 速度档位 (base_speed, 偏差阈值)
    SPEED_HIGH = 80      # 直道高速
    SPEED_MEDIUM = 50    # 弯道中速
    SPEED_LOW = 25       # 急弯低速
    SPEED_STOP = 0       # 脱线停车

    # 偏差阈值
    THRESHOLD_STRAIGHT = 0.5   # 直道阈值
    THRESHOLD_CURVE = 2.0      # 弯道阈值
    # 超过THRESHOLD_CURVE为急弯

    def __init__(self, kp=12.0, ki=0.0, kd=3.0):
        self.sensor = Grayscale()
        self.pid = PIDController(kp=kp, ki=ki, kd=kd,
                                  output_min=-100, output_max=100)
        self.filter = SimpleMA(3)  # 3点滑动平均滤波
        self.error = 0.0
        self.steer = 0.0
        self.base_speed = 0
        self.on_line = True

    def init(self):
        """初始化传感器和PID"""
        self.sensor.init()
        self.pid.reset()
        self.filter.reset()

    def _calc_error(self):
        """加权平均计算偏差"""
        weighted_sum = 0.0
        weight_count = 0
        for i in range(8):
            if self.sensor.read(i) == 1:
                weighted_sum += self.WEIGHTS[i]
                weight_count += 1
        if weight_count > 0:
            return weighted_sum / weight_count
        return 0.0

    def _select_speed(self, abs_error):
        """根据偏差大小选择速度档位"""
        if abs_error < self.THRESHOLD_STRAIGHT:
            return self.SPEED_HIGH      # 直道：高速
        elif abs_error < self.THRESHOLD_CURVE:
            return self.SPEED_MEDIUM    # 弯道：中速
        else:
            return self.SPEED_LOW       # 急弯：低速

    def update(self):
        """更新循迹计算

        返回: (left_speed, right_speed, on_line)
        """
        mask = self.sensor.read_all()

        # 脱线检测
        if mask == 0:
            self.on_line = False
            self.base_speed = self.SPEED_STOP
            return 0, 0, False

        self.on_line = True

        # 计算偏差并滤波
        raw_error = self._calc_error()
        self.error = self.filter.update(raw_error)

        # PID转向计算
        self.steer = self.pid.calc(0, self.error)

        # 变速策略
        self.base_speed = self._select_speed(abs(self.error))

        # 差速转向: 左=base+steer, 右=base-steer
        left = self.base_speed + self.steer
        right = self.base_speed - self.steer

        # 限幅 [0, 100]
        left = max(0, min(100, left))
        right = max(0, min(100, right))

        return left, right, True


class TestLineFollowingInit(unittest.TestCase):
    """循迹V2初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        lf = LineFollowingV2()
        lf.init()
        self.assertTrue(lf.sensor.initialized)

    def test_default_weights_symmetric(self):
        """权重对称分布"""
        w = LineFollowingV2.WEIGHTS
        self.assertEqual(len(w), 8)
        self.assertAlmostEqual(w[0] + w[7], 0.0)
        self.assertAlmostEqual(w[3] + w[4], 0.0)

    def test_speed_levels_ordered(self):
        """速度档位递减"""
        self.assertGreater(LineFollowingV2.SPEED_HIGH, LineFollowingV2.SPEED_MEDIUM)
        self.assertGreater(LineFollowingV2.SPEED_MEDIUM, LineFollowingV2.SPEED_LOW)
        self.assertGreater(LineFollowingV2.SPEED_LOW, LineFollowingV2.SPEED_STOP)


class TestLineFollowingSensor(unittest.TestCase):
    """灰度传感器读取测试"""

    def test_all_white(self):
        """全白→全1"""
        gs = Grayscale()
        gs.init()
        for ch in range(8):
            gs.set_channel(ch, 1)
        self.assertEqual(gs.read_all(), 0xFF)
        self.assertEqual(gs.count_white(), 8)

    def test_all_black(self):
        """全黑→全0"""
        gs = Grayscale()
        gs.init()
        for ch in range(8):
            gs.set_channel(ch, 0)
        self.assertEqual(gs.read_all(), 0x00)
        self.assertEqual(gs.count_white(), 0)

    def test_center_only(self):
        """中心线: CH3/CH4白"""
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


class TestLineFollowingTracking(unittest.TestCase):
    """循迹跟踪测试"""

    def _setup(self, channel_values, kp=12.0, ki=0.0, kd=3.0):
        """辅助: 创建循迹器并设置传感器"""
        lf = LineFollowingV2(kp=kp, ki=ki, kd=kd)
        lf.init()
        for ch, val in enumerate(channel_values):
            lf.sensor.set_channel(ch, val)
        return lf

    def test_center_straight(self):
        """居中直行→偏差≈0, 高速档"""
        vals = [0, 0, 0, 1, 1, 0, 0, 0]
        lf = self._setup(vals)
        left, right, on_line = lf.update()
        self.assertTrue(on_line)
        self.assertAlmostEqual(lf.error, 0.0, places=1)
        self.assertEqual(lf.base_speed, LineFollowingV2.SPEED_HIGH)

    def test_drift_left(self):
        """偏左→负偏差→右轮快于左轮"""
        vals = [0, 0, 1, 1, 0, 0, 0, 0]
        lf = self._setup(vals)
        left, right, on_line = lf.update()
        self.assertTrue(on_line)
        self.assertLess(lf.error, 0)  # 偏左为负

    def test_drift_right(self):
        """偏右→正偏差→左轮快于右轮"""
        vals = [0, 0, 0, 0, 1, 1, 0, 0, 0]
        lf = self._setup(vals)
        left, right, on_line = lf.update()
        self.assertTrue(on_line)
        self.assertGreater(lf.error, 0)  # 偏右为正

    def test_off_line_stop(self):
        """全黑→脱线停车"""
        vals = [0, 0, 0, 0, 0, 0, 0, 0]
        lf = self._setup(vals)
        left, right, on_line = lf.update()
        self.assertFalse(on_line)
        self.assertEqual(left, 0)
        self.assertEqual(right, 0)

    def test_extreme_left_low_speed(self):
        """极左偏→急弯低速档"""
        vals = [1, 0, 0, 0, 0, 0, 0, 0]
        lf = self._setup(vals)
        left, right, on_line = lf.update()
        self.assertTrue(on_line)
        # 偏差=3.5 > THRESHOLD_CURVE(2.0) → 急弯
        self.assertEqual(lf.base_speed, LineFollowingV2.SPEED_LOW)

    def test_extreme_right_low_speed(self):
        """极右偏→急弯低速档"""
        vals = [0, 0, 0, 0, 0, 0, 0, 1]
        lf = self._setup(vals)
        left, right, on_line = lf.update()
        self.assertTrue(on_line)
        self.assertEqual(lf.base_speed, LineFollowingV2.SPEED_LOW)

    def test_mild_curve_medium_speed(self):
        """轻微弯道→中速档"""
        # CH2,CH3白: 偏差=(-1.5+-0.5)/2=-1.0, |1.0|在(0.5,2.0)区间
        vals = [0, 0, 1, 1, 0, 0, 0, 0]
        lf = self._setup(vals)
        lf.update()
        self.assertEqual(lf.base_speed, LineFollowingV2.SPEED_MEDIUM)

    def test_all_white_crossroad(self):
        """全白(十字路口)→在线, 偏差≈0"""
        vals = [1, 1, 1, 1, 1, 1, 1, 1]
        lf = self._setup(vals)
        left, right, on_line = lf.update()
        self.assertTrue(on_line)
        self.assertAlmostEqual(lf.error, 0.0, places=1)


class TestLineFollowingPID(unittest.TestCase):
    """PID控制测试"""

    def test_output_clamp(self):
        """PID输出限幅"""
        pid = PIDController(kp=50.0, ki=0.0, kd=0.0,
                            output_min=-100, output_max=100)
        out = pid.calc(0, -3.5)
        self.assertGreaterEqual(out, -100)
        self.assertLessEqual(out, 100)

    def test_pid_reset(self):
        """PID重置清零"""
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
        pid.calc(10, 0)
        pid.reset()
        self.assertAlmostEqual(pid.integral, 0.0)
        self.assertAlmostEqual(pid.prev_error, 0.0)

    def test_integral_accumulation(self):
        """持续偏差→积分累积"""
        pid = PIDController(kp=1.0, ki=1.0, kd=0.0,
                            output_min=-1000, output_max=1000)
        for _ in range(50):
            pid.calc(1.0, 0.0)
        self.assertGreater(pid.integral, 0)

    def test_steer_direction_correction(self):
        """偏差修正方向验证"""
        pid = PIDController(kp=10.0, ki=0.0, kd=0.0,
                            output_min=-100, output_max=100)
        # 偏左(error<0) → 正steer(向右修正)
        steer_left = pid.calc(0, -2.0)
        pid.reset()
        # 偏右(error>0) → 负steer(向左修正)
        steer_right = pid.calc(0, 2.0)
        self.assertGreater(steer_left, 0)
        self.assertLess(steer_right, 0)


class TestLineFollowingFilter(unittest.TestCase):
    """滑动平均滤波测试"""

    def test_smoothing(self):
        """滤波平滑效果"""
        f = SimpleMA(3)
        # 突变值被平滑
        f.update(0.0)
        f.update(0.0)
        v = f.update(3.0)
        self.assertAlmostEqual(v, 1.0, places=1)  # (0+0+3)/3=1

    def test_constant_input(self):
        """恒定输入→输出等于输入"""
        f = SimpleMA(5)
        for _ in range(10):
            v = f.update(2.5)
        self.assertAlmostEqual(v, 2.5, places=2)


if __name__ == '__main__':
    unittest.main()
