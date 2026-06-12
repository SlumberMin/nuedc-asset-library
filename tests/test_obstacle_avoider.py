#!/usr/bin/env python3
"""
避障算法 V2测试 — 基于wrappers.py包装层
覆盖: 超声波测距、障碍物检测、避障策略、多传感器融合
对应C源文件: 02_mspm0g3507/drivers/ultrasonic.c + obstacle_avoider算法

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    Ultrasonic, PIDController,
    ULTRASONIC_TIMEOUT_US, ULTRASONIC_US_PER_CM,
    ULTRASONIC_MIN_CM, ULTRASONIC_MAX_CM,
)


class ObstacleAvoider:
    """避障算法 — 基于超声波传感器

    与C版本obstacle_avoider.c逻辑一致:
    - 前方距离 < 安全距离 → 转向
    - 左右距离比较决定转向方向
    - PID控制转向角度
    """

    # 避障参数
    SAFE_DISTANCE_CM = 30.0
    DANGER_DISTANCE_CM = 15.0
    MAX_SPEED = 100

    def __init__(self):
        self.front = Ultrasonic()
        self.left = Ultrasonic()
        self.right = Ultrasonic()
        self.pid = PIDController(kp=2.0, ki=0.0, kd=0.5,
                                  output_min=-80, output_max=80)
        self.state = 'FORWARD'  # FORWARD, TURN_LEFT, TURN_RIGHT, STOP
        self.steer = 0

    def init(self):
        """初始化三个超声波传感器"""
        self.front.init()
        self.left.init()
        self.right.init()
        self.pid.reset()
        self.state = 'FORWARD'

    def update(self, front_cm, left_cm, right_cm):
        """更新避障决策

        返回: (state: str, steer: int, speed: int)
        """
        # 测距
        _, f_dist = self.front.measure(self._cm_to_pulse(front_cm))
        _, l_dist = self.left.measure(self._cm_to_pulse(left_cm))
        _, r_dist = self.right.measure(self._cm_to_pulse(right_cm))

        # 判断避障状态
        if f_dist < self.DANGER_DISTANCE_CM:
            # 紧急: 距离太近，停止
            self.state = 'STOP'
            self.steer = 0
            return self.state, 0, 0

        if f_dist < self.SAFE_DISTANCE_CM:
            # 需要避障: 比较左右
            if l_dist > r_dist:
                self.state = 'TURN_LEFT'
                self.steer = -self.pid.calc(self.SAFE_DISTANCE_CM, f_dist)
            else:
                self.state = 'TURN_RIGHT'
                self.steer = self.pid.calc(self.SAFE_DISTANCE_CM, f_dist)
            speed = self.MAX_SPEED // 2
        else:
            # 安全: 直行
            self.state = 'FORWARD'
            self.steer = 0
            speed = self.MAX_SPEED

        return self.state, int(self.steer), speed

    @staticmethod
    def _cm_to_pulse(cm):
        """距离(cm)转回Echo脉宽(µs)，用于测试"""
        if cm <= 0 or cm > ULTRASONIC_MAX_CM:
            return 0
        return int(cm * ULTRASONIC_US_PER_CM)


class TestObstacleInit(unittest.TestCase):
    """避障初始化测试"""

    def test_init(self):
        """初始化成功"""
        av = ObstacleAvoider()
        av.init()
        self.assertTrue(av.front.initialized)
        self.assertTrue(av.left.initialized)
        self.assertTrue(av.right.initialized)
        self.assertEqual(av.state, 'FORWARD')


class TestUltrasonicSensor(unittest.TestCase):
    """超声波传感器测试"""

    def test_measure_10cm(self):
        """10cm测距"""
        us = Ultrasonic()
        us.init()
        pulse = int(10.0 * ULTRASONIC_US_PER_CM)
        ok, dist = us.measure(pulse)
        self.assertTrue(ok)
        self.assertAlmostEqual(dist, 10.0, delta=0.5)

    def test_measure_100cm(self):
        """100cm测距"""
        us = Ultrasonic()
        us.init()
        pulse = int(100.0 * ULTRASONIC_US_PER_CM)
        ok, dist = us.measure(pulse)
        self.assertTrue(ok)
        self.assertAlmostEqual(dist, 100.0, delta=1.0)

    def test_timeout_zero(self):
        """脉宽0 → 超时"""
        us = Ultrasonic()
        us.init()
        ok, _ = us.measure(0)
        self.assertFalse(ok)
        self.assertEqual(us.timeout_count, 1)

    def test_timeout_exceeded(self):
        """超过最大距离 → 超时"""
        us = Ultrasonic()
        us.init()
        ok, _ = us.measure(ULTRASONIC_TIMEOUT_US + 1)
        self.assertFalse(ok)

    def test_distance_below_min(self):
        """低于最小距离 → 失败"""
        us = Ultrasonic()
        us.init()
        pulse = int((ULTRASONIC_MIN_CM - 1) * ULTRASONIC_US_PER_CM)
        ok, _ = us.measure(pulse)
        self.assertFalse(ok)

    def test_measure_count(self):
        """测量次数统计"""
        us = Ultrasonic()
        us.init()
        us.measure(int(50 * ULTRASONIC_US_PER_CM))
        us.measure(int(60 * ULTRASONIC_US_PER_CM))
        self.assertEqual(us.measure_count, 2)


class TestObstacleForward(unittest.TestCase):
    """直行测试"""

    def test_all_clear(self):
        """前方无障碍 → 直行"""
        av = ObstacleAvoider()
        av.init()
        state, steer, speed = av.update(100, 100, 100)
        self.assertEqual(state, 'FORWARD')
        self.assertEqual(steer, 0)
        self.assertEqual(speed, ObstacleAvoider.MAX_SPEED)

    def test_side_obstacle(self):
        """侧方有障碍但前方无 → 仍直行"""
        av = ObstacleAvoider()
        av.init()
        state, steer, speed = av.update(100, 10, 100)
        self.assertEqual(state, 'FORWARD')


class TestObstacleAvoidance(unittest.TestCase):
    """避障策略测试"""

    def test_front_obstacle_turn_left(self):
        """前方障碍、左侧更宽 → 左转"""
        av = ObstacleAvoider()
        av.init()
        state, steer, speed = av.update(20, 80, 40)
        self.assertEqual(state, 'TURN_LEFT')
        self.assertLess(steer, 0)  # 负值=左转
        self.assertLess(speed, ObstacleAvoider.MAX_SPEED)

    def test_front_obstacle_turn_right(self):
        """前方障碍、右侧更宽 → 右转"""
        av = ObstacleAvoider()
        av.init()
        state, steer, speed = av.update(20, 40, 80)
        self.assertEqual(state, 'TURN_RIGHT')
        self.assertGreater(steer, 0)  # 正值=右转

    def test_danger_stop(self):
        """距离太近 → 停止"""
        av = ObstacleAvoider()
        av.init()
        state, steer, speed = av.update(10, 50, 50)
        self.assertEqual(state, 'STOP')
        self.assertEqual(speed, 0)

    def test_boundary_safe(self):
        """刚好在安全距离边界 → 不避障"""
        av = ObstacleAvoider()
        av.init()
        state, _, _ = av.update(31, 50, 50)
        self.assertEqual(state, 'FORWARD')

    def test_boundary_danger(self):
        """刚好在危险距离边界 → 停止"""
        av = ObstacleAvoider()
        av.init()
        state, _, speed = av.update(14, 50, 50)
        self.assertEqual(state, 'STOP')


class TestObstacleMultiUpdate(unittest.TestCase):
    """多轮更新测试"""

    def test_approach_sequence(self):
        """逐渐接近障碍物的状态变化"""
        av = ObstacleAvoider()
        av.init()

        # 远处: 直行
        s1, _, _ = av.update(100, 50, 50)
        self.assertEqual(s1, 'FORWARD')

        # 中距离: 避障
        s2, _, _ = av.update(25, 50, 80)
        self.assertIn(s2, ['TURN_LEFT', 'TURN_RIGHT'])

        # 很近: 停止
        s3, _, sp3 = av.update(10, 50, 50)
        self.assertEqual(s3, 'STOP')
        self.assertEqual(sp3, 0)

    def test_escape_sequence(self):
        """避障后恢复直行"""
        av = ObstacleAvoider()
        av.init()

        av.update(20, 80, 40)  # 避障
        state, _, _ = av.update(100, 80, 80)  # 障碍物消失
        self.assertEqual(state, 'FORWARD')


if __name__ == '__main__':
    unittest.main()
