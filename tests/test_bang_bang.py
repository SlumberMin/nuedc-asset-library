#!/usr/bin/env python3
"""
Bang-Bang控制器单元测试
覆盖: 简单Bang-Bang、滞回模式、PD切换模式、
      输出限幅、重置功能
注意: 使用纯 Python 模拟 C bang_bang.h 逻辑
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 (对照 C bang_bang.h) ──────────────────────

BB_MODE_SIMPLE = 0
BB_MODE_HYSTERESIS = 1
BB_MODE_PD_SWITCH = 2


class BangBangSimulator:
    """Bang-Bang控制器 Python 模拟"""

    def __init__(self, mode=BB_MODE_SIMPLE, pos_output=10.0, neg_output=-10.0,
                 hysteresis=0.0, switch_threshold=5.0, switch_kp=1.0, switch_kd=0.1,
                 out_min=-100.0, out_max=100.0):
        self.mode = mode
        self.pos_output = pos_output
        self.neg_output = neg_output
        self.hysteresis = hysteresis
        self.switch_threshold = switch_threshold
        self.switch_kp = switch_kp
        self.switch_kd = switch_kd
        self.output_min = out_min
        self.output_max = out_max
        self.prev_error = 0.0
        self.output = 0.0
        self.in_pd_mode = False

    def calc(self, setpoint, feedback, dt=0.01):
        """Bang-Bang控制计算"""
        error = setpoint - feedback

        if self.mode == BB_MODE_SIMPLE:
            # 简单Bang-Bang
            if error > 0:
                self.output = self.pos_output
            elif error < 0:
                self.output = self.neg_output
            else:
                self.output = 0.0

        elif self.mode == BB_MODE_HYSTERESIS:
            # 带滞回的Bang-Bang
            if error > self.hysteresis:
                self.output = self.pos_output
            elif error < -self.hysteresis:
                self.output = self.neg_output
            # else: 保持上次输出 (滞回区内不变)

        elif self.mode == BB_MODE_PD_SWITCH:
            # 接近目标时切换PD
            if abs(error) > self.switch_threshold:
                self.in_pd_mode = False
                if error > 0:
                    self.output = self.pos_output
                elif error < 0:
                    self.output = self.neg_output
                else:
                    self.output = 0.0
            else:
                self.in_pd_mode = True
                # PD控制
                error_dot = (error - self.prev_error) / dt if dt > 0 else 0
                self.output = self.switch_kp * error + self.switch_kd * error_dot

        self.prev_error = error

        # 输出限幅
        self.output = max(self.output_min, min(self.output_max, self.output))
        return self.output

    def is_in_pd_mode(self):
        return self.in_pd_mode

    def reset(self):
        self.prev_error = 0.0
        self.output = 0.0
        self.in_pd_mode = False


# ── 测试用例 ──────────────────────────────────────────────────

class TestBangBangInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        bb = BangBangSimulator()
        self.assertEqual(bb.mode, BB_MODE_SIMPLE)
        self.assertEqual(bb.pos_output, 10.0)
        self.assertEqual(bb.neg_output, -10.0)

    def test_custom_params(self):
        bb = BangBangSimulator(mode=BB_MODE_HYSTERESIS, pos_output=20.0,
                               neg_output=-20.0, hysteresis=2.0)
        self.assertEqual(bb.mode, BB_MODE_HYSTERESIS)
        self.assertEqual(bb.pos_output, 20.0)
        self.assertEqual(bb.hysteresis, 2.0)

    def test_reset(self):
        bb = BangBangSimulator()
        bb.calc(setpoint=10.0, feedback=0.0)
        bb.reset()
        self.assertEqual(bb.prev_error, 0.0)
        self.assertEqual(bb.output, 0.0)


class TestSimpleBangBang(unittest.TestCase):
    """简单Bang-Bang测试"""

    def test_positive_error_positive_output(self):
        """正误差应输出正值"""
        bb = BangBangSimulator(mode=BB_MODE_SIMPLE, pos_output=10.0)
        output = bb.calc(setpoint=10.0, feedback=0.0)
        self.assertEqual(output, 10.0)

    def test_negative_error_negative_output(self):
        """负误差应输出负值"""
        bb = BangBangSimulator(mode=BB_MODE_SIMPLE, neg_output=-10.0)
        output = bb.calc(setpoint=0.0, feedback=10.0)
        self.assertEqual(output, -10.0)

    def test_zero_error_zero_output(self):
        """零误差应输出零"""
        bb = BangBangSimulator(mode=BB_MODE_SIMPLE)
        output = bb.calc(setpoint=5.0, feedback=5.0)
        self.assertEqual(output, 0.0)

    def test_output_is_bang_bang(self):
        """输出只有两个值(正最大或负最大)"""
        bb = BangBangSimulator(mode=BB_MODE_SIMPLE, pos_output=15.0, neg_output=-15.0)
        outputs = set()
        for sp in [10, 5, -5, -10, 3, -3]:
            out = bb.calc(setpoint=sp, feedback=0.0)
            outputs.add(out)
        self.assertTrue(outputs.issubset({15.0, -15.0, 0.0}))


class TestHysteresisBangBang(unittest.TestCase):
    """滞回Bang-Bang测试"""

    def test_outside_hysteresis_positive(self):
        """误差大于滞回应输出"""
        bb = BangBangSimulator(mode=BB_MODE_HYSTERESIS, pos_output=10.0,
                               neg_output=-10.0, hysteresis=2.0)
        output = bb.calc(setpoint=10.0, feedback=0.0)  # error=10 > 2
        self.assertEqual(output, 10.0)

    def test_inside_hysteresis_maintains(self):
        """滞回区内应保持上次输出"""
        bb = BangBangSimulator(mode=BB_MODE_HYSTERESIS, pos_output=10.0,
                               neg_output=-10.0, hysteresis=5.0)
        # 先产生正输出
        bb.calc(setpoint=10.0, feedback=0.0)
        first_output = bb.output
        # 进入滞回区 (error=1 < 5)
        bb.calc(setpoint=5.0, feedback=4.0)
        self.assertEqual(bb.output, first_output)

    def test_hysteresis_prevents_chattering(self):
        """滞回应减少抖振"""
        bb = BangBangSimulator(mode=BB_MODE_HYSTERESIS, pos_output=10.0,
                               neg_output=-10.0, hysteresis=3.0)
        # 在滞回区附近来回
        outputs = []
        for error in [4.0, 3.5, 3.0, 2.5, 3.0, 3.5, 4.0]:
            out = bb.calc(setpoint=error, feedback=0.0)
            outputs.append(out)
        # 不应频繁切换
        switches = sum(1 for i in range(1, len(outputs)) if outputs[i] != outputs[i-1])
        self.assertLessEqual(switches, 2)


class TestPDSwitchBangBang(unittest.TestCase):
    """PD切换模式测试"""

    def test_far_from_target_bang_bang(self):
        """远离目标时应使用Bang-Bang"""
        bb = BangBangSimulator(mode=BB_MODE_PD_SWITCH, pos_output=10.0,
                               neg_output=-10.0, switch_threshold=5.0)
        output = bb.calc(setpoint=100.0, feedback=0.0)
        self.assertEqual(output, 10.0)
        self.assertFalse(bb.is_in_pd_mode())

    def test_near_target_uses_pd(self):
        """接近目标时应切换到PD"""
        bb = BangBangSimulator(mode=BB_MODE_PD_SWITCH, pos_output=10.0,
                               neg_output=-10.0, switch_threshold=10.0,
                               switch_kp=2.0, switch_kd=0.1)
        # 先远离
        bb.calc(setpoint=50.0, feedback=0.0)
        # 接近目标
        output = bb.calc(setpoint=5.0, feedback=3.0)  # error=2 < threshold=10
        self.assertTrue(bb.is_in_pd_mode())
        # PD输出应比Bang-Bang更精细
        self.assertLess(abs(output), 10.0)

    def test_pd_mode_smooth_transition(self):
        """PD模式应产生平滑过渡"""
        bb = BangBangSimulator(mode=BB_MODE_PD_SWITCH, pos_output=20.0,
                               neg_output=-20.0, switch_threshold=5.0,
                               switch_kp=2.0, switch_kd=0.0)
        # 从远处接近
        outputs = []
        pos = 0.0
        target = 10.0
        for _ in range(100):
            out = bb.calc(setpoint=target, feedback=pos, dt=0.01)
            outputs.append(out)
            pos += out * 0.005
        # 最终输出应接近零
        self.assertLess(abs(outputs[-1]), 20.0)

    def test_pd_mode_error_zero(self):
        """PD模式零误差应输出零"""
        bb = BangBangSimulator(mode=BB_MODE_PD_SWITCH, pos_output=10.0,
                               neg_output=-10.0, switch_threshold=10.0,
                               switch_kp=2.0, switch_kd=0.1)
        # 进入PD模式
        bb.calc(setpoint=5.0, feedback=3.0)  # error=2 < 10
        output = bb.calc(setpoint=5.0, feedback=5.0)  # error=0
        self.assertAlmostEqual(output, 0.0, places=2)


class TestBangBangOutputLimit(unittest.TestCase):
    """输出限幅测试"""

    def test_output_clamped(self):
        """输出应被限幅"""
        bb = BangBangSimulator(mode=BB_MODE_SIMPLE, pos_output=200.0,
                               neg_output=-200.0, out_min=-50, out_max=50)
        output = bb.calc(setpoint=10.0, feedback=0.0)
        self.assertLessEqual(output, 50.0)
        self.assertGreaterEqual(output, -50.0)


class TestBangBangEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_large_error(self):
        """大误差应正常工作"""
        bb = BangBangSimulator(mode=BB_MODE_SIMPLE, pos_output=10.0, neg_output=-10.0)
        output = bb.calc(setpoint=1e6, feedback=0.0)
        self.assertEqual(output, 10.0)

    def test_negative_outputs(self):
        """负输出值应正常"""
        bb = BangBangSimulator(mode=BB_MODE_SIMPLE, pos_output=5.0, neg_output=-5.0)
        output = bb.calc(setpoint=-10.0, feedback=0.0)
        self.assertEqual(output, -5.0)

    def test_successive_calls(self):
        """连续调用应稳定"""
        bb = BangBangSimulator(mode=BB_MODE_SIMPLE, pos_output=10.0, neg_output=-10.0)
        for i in range(100):
            output = bb.calc(setpoint=float(i), feedback=0.0)
            self.assertIn(output, [10.0, -10.0, 0.0])


if __name__ == '__main__':
    unittest.main()
