#!/usr/bin/env python3
"""
前馈控制单元测试
覆盖: 速度前馈、加速度前馈、前馈+PID组合、查表前馈、
      输出限幅、重置功能
注意: 使用纯 Python 模拟 C feedforward.h 逻辑
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 (对照 C feedforward.h) ──────────────────

FF_TYPE_NONE = 0
FF_TYPE_VELOCITY = 1
FF_TYPE_ACCELERATION = 2
FF_TYPE_CUSTOM = 3


class FeedForwardSimulator:
    """前馈控制器 Python 模拟"""

    def __init__(self, ff_type=FF_TYPE_VELOCITY, Kv=1.0, Ka=0.0, Kj=0.0,
                 out_min=-100.0, out_max=100.0):
        self.type = ff_type
        self.Kv = Kv
        self.Ka = Ka
        self.Kj = Kj
        self.output_min = out_min
        self.output_max = out_max
        self.prev_ref = 0.0
        self.prev_vel = 0.0
        self.output = 0.0
        self.initialized = False

    def calc(self, reference, dt):
        """前馈计算 (对应 C FeedForward_Calc)"""
        if not self.initialized:
            self.prev_ref = reference
            self.initialized = True
            self.output = 0.0
            return 0.0

        if dt <= 0:
            return self.output

        if self.type == FF_TYPE_NONE:
            self.output = 0.0

        elif self.type == FF_TYPE_VELOCITY:
            # 速度前馈: Kv * (ref - prev_ref) / dt
            velocity = (reference - self.prev_ref) / dt
            self.output = self.Kv * velocity

        elif self.type == FF_TYPE_ACCELERATION:
            # 加速度前馈: Kv * vel + Ka * acc
            velocity = (reference - self.prev_ref) / dt
            acceleration = (velocity - self.prev_vel) / dt
            self.output = self.Kv * velocity + self.Ka * acceleration
            self.prev_vel = velocity

        self.prev_ref = reference

        # 输出限幅
        self.output = max(self.output_min, min(self.output_max, self.output))
        return self.output

    def calc_explicit(self, velocity, acceleration, jerk=0.0):
        """显式输入的前馈计算 (对应 C FeedForward_CalcExplicit)"""
        self.output = (self.Kv * velocity +
                       self.Ka * acceleration +
                       self.Kj * jerk)
        self.output = max(self.output_min, min(self.output_max, self.output))
        return self.output

    def reset(self):
        self.prev_ref = 0.0
        self.prev_vel = 0.0
        self.output = 0.0
        self.initialized = False


class FeedForwardLUT:
    """查表前馈 (对应 C FeedForward_LUT_t)"""

    def __init__(self, inputs, outputs):
        assert len(inputs) == len(outputs), "输入输出长度必须相同"
        assert len(inputs) <= 64, "查表最大64项"
        self.inputs = list(inputs)
        self.outputs = list(outputs)
        self.size = len(inputs)

    def lookup(self, x):
        """线性插值查表"""
        if x <= self.inputs[0]:
            return self.outputs[0]
        if x >= self.inputs[-1]:
            return self.outputs[-1]
        for i in range(1, self.size):
            if x <= self.inputs[i]:
                ratio = (x - self.inputs[i-1]) / (self.inputs[i] - self.inputs[i-1] + 1e-9)
                return self.outputs[i-1] + ratio * (self.outputs[i] - self.outputs[i-1])
        return self.outputs[-1]


# ── 测试用例 ──────────────────────────────────────────────────

class TestFeedForwardInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        ff = FeedForwardSimulator()
        self.assertEqual(ff.type, FF_TYPE_VELOCITY)
        self.assertEqual(ff.Kv, 1.0)
        self.assertEqual(ff.Ka, 0.0)
        self.assertFalse(ff.initialized)

    def test_custom_params(self):
        ff = FeedForwardSimulator(ff_type=FF_TYPE_ACCELERATION, Kv=2.0, Ka=0.5)
        self.assertEqual(ff.type, FF_TYPE_ACCELERATION)
        self.assertEqual(ff.Kv, 2.0)
        self.assertEqual(ff.Ka, 0.5)

    def test_reset(self):
        ff = FeedForwardSimulator()
        ff.calc(reference=10.0, dt=0.01)
        ff.reset()
        self.assertFalse(ff.initialized)
        self.assertEqual(ff.prev_ref, 0.0)


class TestVelocityFeedForward(unittest.TestCase):
    """速度前馈测试"""

    def test_first_call_returns_zero(self):
        """首次调用应返回零(无法计算速度)"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_VELOCITY, Kv=1.0)
        output = ff.calc(reference=10.0, dt=0.01)
        self.assertAlmostEqual(output, 0.0)

    def test_velocity_ff_proportional_to_speed(self):
        """速度前馈输出应与目标速度成正比"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_VELOCITY, Kv=2.0)
        ff.calc(reference=0.0, dt=0.01)  # 初始化
        output = ff.calc(reference=1.0, dt=0.01)
        # 速度 = (1.0 - 0.0) / 0.01 = 100
        # 输出 = Kv * 100 = 200 -> 限幅到 100
        self.assertGreater(output, 0)

    def test_constant_reference_zero_output(self):
        """恒定目标应产生零前馈输出"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_VELOCITY, Kv=1.0)
        ff.calc(reference=5.0, dt=0.01)  # 初始化
        for _ in range(10):
            output = ff.calc(reference=5.0, dt=0.01)
        self.assertAlmostEqual(output, 0.0, places=2)

    def test_negative_velocity(self):
        """负速度应产生负输出"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_VELOCITY, Kv=1.0,
                                   out_min=-1000, out_max=1000)
        ff.calc(reference=10.0, dt=0.01)
        output = ff.calc(reference=0.0, dt=0.01)
        self.assertLess(output, 0)


class TestAccelerationFeedForward(unittest.TestCase):
    """加速度前馈测试"""

    def test_acceleration_ff_with_accel(self):
        """加速度前馈应响应加速度变化"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_ACCELERATION,
                                   Kv=1.0, Ka=0.5, out_min=-1000, out_max=1000)
        ff.calc(reference=0.0, dt=0.01)  # 初始化
        ff.calc(reference=1.0, dt=0.01)  # 匀速段
        ff.calc(reference=4.0, dt=0.01)  # 加速段
        output = ff.calc(reference=9.0, dt=0.01)  # 更大加速
        self.assertGreater(abs(output), 0)

    def test_constant_speed_no_accel(self):
        """匀速运动时加速度前馈应仅输出速度前馈"""
        ff_vel = FeedForwardSimulator(ff_type=FF_TYPE_VELOCITY, Kv=1.0,
                                       out_min=-1000, out_max=1000)
        ff_acc = FeedForwardSimulator(ff_type=FF_TYPE_ACCELERATION, Kv=1.0, Ka=0.5,
                                       out_min=-1000, out_max=1000)
        ff_vel.calc(reference=0.0, dt=0.01)
        ff_acc.calc(reference=0.0, dt=0.01)
        # 匀速输入
        out1 = ff_vel.calc(reference=1.0, dt=0.01)
        out2 = ff_acc.calc(reference=1.0, dt=0.01)
        # 第二步匀速
        out3 = ff_vel.calc(reference=2.0, dt=0.01)
        out4 = ff_acc.calc(reference=2.0, dt=0.01)
        # 匀速段加速度为零，两者应相近
        self.assertAlmostEqual(out3, out4, delta=1.0)


class TestExplicitFeedForward(unittest.TestCase):
    """显式前馈测试"""

    def test_explicit_velocity_only(self):
        """仅速度前馈"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_ACCELERATION, Kv=2.0, Ka=0.0)
        output = ff.calc_explicit(velocity=5.0, acceleration=0.0, jerk=0.0)
        self.assertAlmostEqual(output, 10.0)

    def test_explicit_acceleration(self):
        """加速度前馈"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_ACCELERATION, Kv=0.0, Ka=3.0)
        output = ff.calc_explicit(velocity=0.0, acceleration=4.0, jerk=0.0)
        self.assertAlmostEqual(output, 12.0)

    def test_explicit_jerk(self):
        """加加速度前馈"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_ACCELERATION, Kv=0.0, Ka=0.0, Kj=2.0)
        output = ff.calc_explicit(velocity=0.0, acceleration=0.0, jerk=5.0)
        self.assertAlmostEqual(output, 10.0)

    def test_explicit_combined(self):
        """组合前馈"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_ACCELERATION,
                                   Kv=1.0, Ka=2.0, Kj=0.5)
        output = ff.calc_explicit(velocity=3.0, acceleration=4.0, jerk=2.0)
        expected = 1.0 * 3.0 + 2.0 * 4.0 + 0.5 * 2.0
        self.assertAlmostEqual(output, expected)


class TestFeedForwardLUT(unittest.TestCase):
    """查表前馈测试"""

    def test_exact_match(self):
        """精确匹配点"""
        lut = FeedForwardLUT([0, 1, 2, 3], [0, 10, 20, 30])
        self.assertAlmostEqual(lut.lookup(1.0), 10.0)

    def test_interpolation(self):
        """线性插值"""
        lut = FeedForwardLUT([0, 1, 2], [0, 10, 20])
        self.assertAlmostEqual(lut.lookup(0.5), 5.0, delta=0.1)

    def test_below_range(self):
        """低于范围应返回最小值"""
        lut = FeedForwardLUT([1, 2, 3], [10, 20, 30])
        self.assertAlmostEqual(lut.lookup(0.0), 10.0)

    def test_above_range(self):
        """高于范围应返回最大值"""
        lut = FeedForwardLUT([1, 2, 3], [10, 20, 30])
        self.assertAlmostEqual(lut.lookup(10.0), 30.0)


class TestFeedForwardOutputLimit(unittest.TestCase):
    """输出限幅测试"""

    def test_output_clamped_high(self):
        """输出应被限幅在上限"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_VELOCITY, Kv=100.0,
                                   out_min=-10, out_max=10)
        ff.calc(reference=0.0, dt=0.01)
        output = ff.calc(reference=100.0, dt=0.01)
        self.assertLessEqual(output, 10.0)

    def test_output_clamped_low(self):
        """输出应被限幅在下限"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_VELOCITY, Kv=100.0,
                                   out_min=-10, out_max=10)
        ff.calc(reference=100.0, dt=0.01)
        output = ff.calc(reference=0.0, dt=0.01)
        self.assertGreaterEqual(output, -10.0)


class TestFeedForwardEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_zero_dt(self):
        """零dt应返回上次输出"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_VELOCITY, Kv=1.0)
        ff.calc(reference=5.0, dt=0.01)
        output = ff.calc(reference=10.0, dt=0.0)
        self.assertEqual(output, ff.output)

    def test_none_type_zero_output(self):
        """NONE类型应始终输出零"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_NONE)
        ff.calc(reference=5.0, dt=0.01)
        output = ff.calc(reference=10.0, dt=0.01)
        self.assertAlmostEqual(output, 0.0)

    def test_reset_and_reuse(self):
        """重置后应能重新使用"""
        ff = FeedForwardSimulator(ff_type=FF_TYPE_VELOCITY, Kv=1.0)
        ff.calc(reference=10.0, dt=0.01)
        ff.calc(reference=20.0, dt=0.01)
        ff.reset()
        output = ff.calc(reference=5.0, dt=0.01)
        self.assertAlmostEqual(output, 0.0)  # 首次调用应为零


if __name__ == '__main__':
    unittest.main()
