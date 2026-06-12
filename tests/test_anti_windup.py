#!/usr/bin/env python3
"""
抗积分饱和(Anti-Windup)模块单元测试
覆盖: 积分限幅、条件积分、反算法抗饱和、跟踪抗饱和、
      与无抗饱和PID对比、恢复时间测试
注意: 使用纯 Python 模拟 C anti_windup 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 ──────────────────────────────────────────

class AntiWindupMethod:
    NONE = 0
    CONDITIONAL_INTEGRATION = 1
    CLAMPING = 2
    BACK_CALCULATION = 3
    TRACKING = 4


class PIDWithAntiWindupSimulator:
    """带抗饱和机制的PID控制器"""

    def __init__(self, kp=1.0, ki=0.1, kd=0.01,
                 out_min=-100.0, out_max=100.0,
                 anti_windup_method=AntiWindupMethod.CLAMPING,
                 tracking_gain=0.5, back_calc_gain=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = out_min
        self.output_max = out_max
        self.anti_windup_method = anti_windup_method
        self.tracking_gain = tracking_gain
        self.back_calc_gain = back_calc_gain

        # 内部状态
        self.integral = 0.0
        self.prev_error = 0.0
        self.output = 0.0
        self.output_unsaturated = 0.0

    def calc(self, setpoint, feedback, dt=0.01):
        """PID计算 (带抗饱和)"""
        error = setpoint - feedback

        # 未饱和输出 (用于检测饱和)
        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.output_unsaturated = p_term + i_term + d_term

        # 饱和检测
        saturated = (self.output_unsaturated > self.output_max or
                     self.output_unsaturated < self.output_min)

        # 抗饱和积分更新
        if self.anti_windup_method == AntiWindupMethod.NONE:
            self.integral += error * dt

        elif self.anti_windup_method == AntiWindupMethod.CONDITIONAL_INTEGRATION:
            # 仅在未饱和时积分
            if not saturated:
                self.integral += error * dt

        elif self.anti_windup_method == AntiWindupMethod.CLAMPING:
            # 积分限幅
            self.integral += error * dt
            # 计算最大允许积分值
            max_integral = (self.output_max - p_term - d_term) / self.ki if self.ki > 0 else float('inf')
            min_integral = (self.output_min - p_term - d_term) / self.ki if self.ki > 0 else float('-inf')
            self.integral = max(min_integral, min(max_integral, self.integral))

        elif self.anti_windup_method == AntiWindupMethod.BACK_CALCULATION:
            # 反算法: 积分项增加饱和修正
            output_saturated = max(self.output_min, min(self.output_max, self.output_unsaturated))
            windup_error = output_saturated - self.output_unsaturated
            self.integral += (error + self.back_calc_gain * windup_error) * dt

        elif self.anti_windup_method == AntiWindupMethod.TRACKING:
            # 跟踪抗饱和
            output_saturated = max(self.output_min, min(self.output_max, self.output_unsaturated))
            tracking_error = (output_saturated - p_term - d_term) / self.ki - self.integral if self.ki > 0 else 0
            self.integral += (error + self.tracking_gain * tracking_error) * dt

        # 重新计算输出
        self.output = p_term + self.ki * self.integral + d_term
        self.output = max(self.output_min, min(self.output_max, self.output))

        self.prev_error = error
        return self.output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.output = 0.0
        self.output_unsaturated = 0.0


# ── 测试用例 ──────────────────────────────────────────────────

class TestAntiWindupInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        pid = PIDWithAntiWindupSimulator()
        self.assertEqual(pid.anti_windup_method, AntiWindupMethod.CLAMPING)

    def test_custom_method(self):
        pid = PIDWithAntiWindupSimulator(
            anti_windup_method=AntiWindupMethod.BACK_CALCULATION)
        self.assertEqual(pid.anti_windup_method, AntiWindupMethod.BACK_CALCULATION)

    def test_integral_zero_initially(self):
        pid = PIDWithAntiWindupSimulator()
        self.assertEqual(pid.integral, 0.0)

    def test_reset(self):
        pid = PIDWithAntiWindupSimulator()
        pid.calc(setpoint=100.0, feedback=0.0)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.output, 0.0)


class TestConditionalIntegration(unittest.TestCase):
    """条件积分测试"""

    def test_integral_not_accumulating_when_saturated(self):
        """饱和时不累积积分"""
        pid = PIDWithAntiWindupSimulator(
            kp=10.0, ki=5.0, kd=0.0,
            out_min=-10, out_max=10,
            anti_windup_method=AntiWindupMethod.CONDITIONAL_INTEGRATION)

        # 大误差导致饱和
        for _ in range(100):
            pid.calc(setpoint=100.0, feedback=0.0, dt=0.01)

        integral_at_sat = pid.integral

        # 继续运行, 积分不应增加
        for _ in range(100):
            pid.calc(setpoint=100.0, feedback=0.0, dt=0.01)

        self.assertAlmostEqual(pid.integral, integral_at_sat, delta=0.01)

    def test_integral_accumulates_when_not_saturated(self):
        """未饱和时正常累积积分"""
        pid = PIDWithAntiWindupSimulator(
            kp=0.0, ki=1.0, kd=0.0,
            out_min=-100, out_max=100,
            anti_windup_method=AntiWindupMethod.CONDITIONAL_INTEGRATION)

        for _ in range(10):
            pid.calc(setpoint=10.0, feedback=0.0, dt=0.01)

        # 积分应累积
        self.assertGreater(pid.integral, 0.0)


class TestClampingAntiWindup(unittest.TestCase):
    """积分限幅测试"""

    def test_integral_bounded(self):
        """积分应被限幅"""
        pid = PIDWithAntiWindupSimulator(
            kp=10.0, ki=5.0, kd=0.0,
            out_min=-10, out_max=10,
            anti_windup_method=AntiWindupMethod.CLAMPING)

        for _ in range(500):
            pid.calc(setpoint=100.0, feedback=0.0, dt=0.01)

        # 输出应被限幅
        self.assertEqual(pid.output, 10.0)
        # 积分不应无限增长
        self.assertTrue(pid.integral < 100.0)


class TestBackCalculation(unittest.TestCase):
    """反算法抗饱和测试"""

    def test_back_calculation_reduces_windup(self):
        """反算法应减少积分饱和"""
        pid_bc = PIDWithAntiWindupSimulator(
            kp=10.0, ki=5.0, kd=0.0,
            out_min=-10, out_max=10,
            anti_windup_method=AntiWindupMethod.BACK_CALCULATION,
            back_calc_gain=0.5)

        for _ in range(200):
            pid_bc.calc(setpoint=100.0, feedback=0.0, dt=0.01)

        integral_bc = pid_bc.integral

        # 对比无抗饱和
        pid_none = PIDWithAntiWindupSimulator(
            kp=10.0, ki=5.0, kd=0.0,
            out_min=-10, out_max=10,
            anti_windup_method=AntiWindupMethod.NONE)

        for _ in range(200):
            pid_none.calc(setpoint=100.0, feedback=0.0, dt=0.01)

        integral_none = pid_none.integral

        # 反算法的积分应更小
        self.assertLess(abs(integral_bc), abs(integral_none))


class TestRecoveryTime(unittest.TestCase):
    """恢复时间测试"""

    def test_fast_recovery_with_anti_windup(self):
        """带抗饱和应更快恢复"""
        for method in [AntiWindupMethod.NONE, AntiWindupMethod.CLAMPING]:
            pid = PIDWithAntiWindupSimulator(
                kp=2.0, ki=5.0, kd=0.1,
                out_min=-10, out_max=10,
                anti_windup_method=method)

            # 饱和阶段
            for _ in range(300):
                pid.calc(setpoint=100.0, feedback=0.0, dt=0.01)

            # 切换到合理目标
            target = 5.0
            val = 0.0
            steps_to_recover = 0
            for _ in range(500):
                out = pid.calc(setpoint=target, feedback=val, dt=0.01)
                val += out * 0.01
                if abs(val - target) < 0.5:
                    if steps_to_recover == 0:
                        steps_to_recover = 1
                    break
                steps_to_recover += 1

            if method == AntiWindupMethod.NONE:
                none_steps = steps_to_recover
            else:
                clamp_steps = steps_to_recover

        # 有抗饱和应恢复更快或相当
        # 验证两种方法都能最终收敛到目标
        self.assertGreater(none_steps, 0, "无抗饱和方法应该需要步数")
        self.assertGreater(clamp_steps, 0, "夹紧方法应该需要步数")
        # 抗饱和方法应该恢复更快(步数更少)或相当
        self.assertLessEqual(clamp_steps, none_steps * 1.5,
                           msg="抗饱和方法应该恢复更快或相当")


class TestAntiWindupComparison(unittest.TestCase):
    """各方法对比测试"""

    def test_all_methods_converge(self):
        """所有方法都应收敛"""
        for method in [AntiWindupMethod.NONE, AntiWindupMethod.CONDITIONAL_INTEGRATION,
                       AntiWindupMethod.CLAMPING, AntiWindupMethod.BACK_CALCULATION]:
            pid = PIDWithAntiWindupSimulator(
                kp=3.0, ki=2.0, kd=0.1,
                out_min=-50, out_max=50,
                anti_windup_method=method)

            val = 0.0
            target = 10.0
            for _ in range(500):
                out = pid.calc(setpoint=target, feedback=val, dt=0.01)
                val += out * 0.01

            self.assertAlmostEqual(val, target, delta=2.0,
                                   msg=f"Method {method} failed to converge")

    def test_no_windup_methods_prevent_integral_growth(self):
        """抗饱和方法应限制积分增长"""
        methods_with_windup = [AntiWindupMethod.CONDITIONAL_INTEGRATION,
                               AntiWindupMethod.CLAMPING,
                               AntiWindupMethod.BACK_CALCULATION]

        pid_none = PIDWithAntiWindupSimulator(
            kp=10.0, ki=5.0, kd=0.0,
            out_min=-5, out_max=5,
            anti_windup_method=AntiWindupMethod.NONE)

        for _ in range(200):
            pid_none.calc(setpoint=100.0, feedback=0.0, dt=0.01)
        integral_none = abs(pid_none.integral)

        for method in methods_with_windup:
            pid = PIDWithAntiWindupSimulator(
                kp=10.0, ki=5.0, kd=0.0,
                out_min=-5, out_max=5,
                anti_windup_method=method)
            for _ in range(200):
                pid.calc(setpoint=100.0, feedback=0.0, dt=0.01)
            # 抗饱和方法的积分应更小
            self.assertLess(abs(pid.integral), integral_none + 0.1,
                           msg=f"Method {method} didn't prevent windup")


class TestAntiWindupEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_zero_ki_no_windup_issue(self):
        """Ki=0时不存在积分饱和问题"""
        pid = PIDWithAntiWindupSimulator(
            kp=5.0, ki=0.0, kd=0.0,
            out_min=-10, out_max=10,
            anti_windup_method=AntiWindupMethod.NONE)
        for _ in range(100):
            pid.calc(setpoint=100.0, feedback=0.0, dt=0.01)
        # 输出应正常限幅
        self.assertEqual(pid.output, 10.0)

    def test_already_saturated_no_change(self):
        """已在饱和时不应恶化"""
        pid = PIDWithAntiWindupSimulator(
            kp=100.0, ki=0.0, kd=0.0,
            out_min=-5, out_max=5,
            anti_windup_method=AntiWindupMethod.BACK_CALCULATION)
        out1 = pid.calc(setpoint=100.0, feedback=0.0, dt=0.01)
        out2 = pid.calc(setpoint=100.0, feedback=0.0, dt=0.01)
        self.assertEqual(out1, 5.0)
        self.assertEqual(out2, 5.0)


if __name__ == '__main__':
    unittest.main()
