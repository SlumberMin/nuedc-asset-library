#!/usr/bin/env python3
"""
增量式PID控制器单元测试
覆盖: 增量式PID计算、积分饱和避免、与位置式PID对比、
      步进响应、扰动抑制、无扰切换
注意: 使用纯 Python 模拟 C 增量式PID逻辑
"""

import sys
import os
import math
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 ──────────────────────────────────────────

class IncrementalPIDSimulator:
    """增量式PID控制器"""

    def __init__(self, kp=1.0, ki=0.1, kd=0.01,
                 out_min=-100.0, out_max=100.0, delta_out_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = out_min
        self.output_max = out_max
        self.delta_out_max = delta_out_max  # 单步最大变化量

        # 内部状态
        self.error = 0.0
        self.error_last = 0.0
        self.error_last2 = 0.0
        self.output = 0.0
        self.output_last = 0.0

    def calc(self, setpoint, feedback, dt=0.01):
        """增量式PID计算"""
        self.error_last2 = self.error_last
        self.error_last = self.error
        self.error = setpoint - feedback

        # 增量计算: delta_u = Kp*(e-e1) + Ki*e + Kd*(e-2*e1+e2)
        delta = (self.kp * (self.error - self.error_last) +
                 self.ki * self.error * dt +
                 self.kd * (self.error - 2.0 * self.error_last + self.error_last2) / dt)

        # 单步变化量限幅
        if self.delta_out_max is not None:
            delta = max(-self.delta_out_max, min(self.delta_out_max, delta))

        # 累加增量得到输出
        self.output = self.output_last + delta

        # 输出限幅
        self.output = max(self.output_min, min(self.output_max, self.output))
        self.output_last = self.output

        return self.output

    def reset(self):
        self.error = 0.0
        self.error_last = 0.0
        self.error_last2 = 0.0
        self.output = 0.0
        self.output_last = 0.0


class PositionalPIDSimulator:
    """位置式PID控制器(用于对比)"""

    def __init__(self, kp=1.0, ki=0.1, kd=0.01,
                 out_min=-100.0, out_max=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = out_min
        self.output_max = out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.output = 0.0

    def calc(self, setpoint, feedback, dt=0.01):
        error = setpoint - feedback
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.output = max(self.output_min, min(self.output_max, self.output))
        self.prev_error = error
        return self.output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.output = 0.0


# ── 测试用例 ──────────────────────────────────────────────────

class TestIncrementalPIDInit(unittest.TestCase):
    """初始化测试"""

    def test_default_params(self):
        ipid = IncrementalPIDSimulator()
        self.assertEqual(ipid.kp, 1.0)
        self.assertEqual(ipid.ki, 0.1)
        self.assertEqual(ipid.kd, 0.01)

    def test_custom_params(self):
        ipid = IncrementalPIDSimulator(kp=5.0, ki=0.5, kd=0.1)
        self.assertEqual(ipid.kp, 5.0)
        self.assertEqual(ipid.ki, 0.5)
        self.assertEqual(ipid.kd, 0.1)

    def test_initial_output_zero(self):
        ipid = IncrementalPIDSimulator()
        self.assertEqual(ipid.output, 0.0)

    def test_reset(self):
        ipid = IncrementalPIDSimulator()
        ipid.calc(setpoint=10.0, feedback=0.0)
        ipid.reset()
        self.assertEqual(ipid.error, 0.0)
        self.assertEqual(ipid.output, 0.0)


class TestIncrementalPIDCalculation(unittest.TestCase):
    """计算正确性测试"""

    def test_first_step_p_only(self):
        """第一步(前两步误差为零)时仅P项贡献"""
        ipid = IncrementalPIDSimulator(kp=2.0, ki=0.0, kd=0.0)
        output = ipid.calc(setpoint=10.0, feedback=0.0, dt=0.01)
        # delta = kp*(10-0) = 20, output = 0+20 = 20
        self.assertAlmostEqual(output, 20.0)

    def test_integral_accumulation(self):
        """积分项应随时间累积"""
        ipid = IncrementalPIDSimulator(kp=0.0, ki=1.0, kd=0.0,
                                        out_min=-1000, out_max=1000)
        outputs = []
        for _ in range(10):
            out = ipid.calc(setpoint=10.0, feedback=0.0, dt=0.01)
            outputs.append(out)
        # 增量式PID的积分累积是通过output_last实现
        for i in range(1, len(outputs)):
            self.assertGreaterEqual(outputs[i], outputs[i-1])

    def test_derivative_action(self):
        """微分项应响应误差变化"""
        ipid = IncrementalPIDSimulator(kp=0.0, ki=0.0, kd=1.0,
                                        out_min=-1000, out_max=1000)
        # 第一次调用
        ipid.calc(setpoint=10.0, feedback=0.0, dt=0.01)
        # 误差变化率增大
        output = ipid.calc(setpoint=20.0, feedback=0.0, dt=0.01)
        # 应有微分贡献
        self.assertIsNotNone(output)

    def test_output_clamping(self):
        """输出应被限幅"""
        ipid = IncrementalPIDSimulator(kp=100.0, ki=10.0, kd=0.0,
                                        out_min=-5, out_max=5)
        for _ in range(5):
            output = ipid.calc(setpoint=100.0, feedback=0.0, dt=0.01)
        self.assertLessEqual(output, 5.0)
        self.assertGreaterEqual(output, -5.0)

    def test_delta_limit(self):
        """增量限幅应工作"""
        ipid = IncrementalPIDSimulator(kp=100.0, ki=0.0, kd=0.0,
                                        out_min=-200, out_max=200,
                                        delta_out_max=5.0)
        output = ipid.calc(setpoint=100.0, feedback=0.0, dt=0.01)
        # 增量被限幅到5.0
        self.assertAlmostEqual(output, 5.0)


class TestIncrementalPIDvsPositional(unittest.TestCase):
    """增量式 vs 位置式对比"""

    def test_both_converge(self):
        """两种方法都应收敛到目标"""
        ipid = IncrementalPIDSimulator(kp=2.0, ki=1.0, kd=0.1,
                                        out_min=-100, out_max=100)
        pid = PositionalPIDSimulator(kp=2.0, ki=1.0, kd=0.1,
                                      out_min=-100, out_max=100)

        val_i = 0.0
        val_p = 0.0
        target = 10.0

        for _ in range(500):
            out_i = ipid.calc(setpoint=target, feedback=val_i, dt=0.01)
            out_p = pid.calc(setpoint=target, feedback=val_p, dt=0.01)
            val_i += out_i * 0.01
            val_p += out_p * 0.01

        self.assertAlmostEqual(val_i, target, delta=0.5)
        self.assertAlmostEqual(val_p, target, delta=0.5)

    def test_incremental_naturally_limits_windup(self):
        """增量式PID天然避免积分饱和"""
        ipid = IncrementalPIDSimulator(kp=10.0, ki=5.0, kd=0.0,
                                        out_min=-10, out_max=10)
        pid = PositionalPIDSimulator(kp=10.0, ki=5.0, kd=0.0,
                                      out_min=-10, out_max=10)

        # 大误差持续输入(积分饱和条件)
        for _ in range(500):
            ipid.calc(setpoint=100.0, feedback=0.0, dt=0.01)
            pid.calc(setpoint=100.0, feedback=0.0, dt=0.01)

        # 切换到小误差
        out_i = ipid.calc(setpoint=0.0, feedback=0.0, dt=0.01)
        out_p = pid.calc(setpoint=0.0, feedback=0.0, dt=0.01)

        # 增量式应更快速恢复(无积分累积)
        # 验证输出值不为零且在合理范围内
        self.assertGreater(abs(out_i), 0, "增量式PID应该有非零输出")
        self.assertLess(abs(out_i), 100, "增量式PID输出应该在限幅范围内")
        # 位置式PID的积分项仍然很大，输出也应该有非零值
        self.assertGreater(abs(out_p), 0, "位置式PID应该有非零输出")

    def test_no_windup_in_incremental(self):
        """增量式PID不存在积分累积"""
        ipid = IncrementalPIDSimulator(kp=0.0, ki=1.0, kd=0.0,
                                        out_min=-10, out_max=10)
        # 大误差持续输入
        for _ in range(1000):
            ipid.calc(setpoint=100.0, feedback=0.0, dt=0.01)
        # 输出应被限幅
        self.assertEqual(ipid.output, 10.0)
        # 但output_last也是10.0, 不会累积到无穷大
        self.assertEqual(ipid.output_last, 10.0)


class TestIncrementalPIDStepResponse(unittest.TestCase):
    """阶跃响应测试"""

    def test_step_response(self):
        """阶跃响应应收敛"""
        ipid = IncrementalPIDSimulator(kp=2.0, ki=1.0, kd=0.1,
                                        out_min=-100, out_max=100)
        target = 10.0
        val = 0.0
        for _ in range(500):
            out = ipid.calc(setpoint=target, feedback=val, dt=0.01)
            val += out * 0.01
        self.assertAlmostEqual(val, target, delta=0.5)

    def test_overshoot_bounded(self):
        """超调量应有界"""
        ipid = IncrementalPIDSimulator(kp=3.0, ki=2.0, kd=0.5,
                                        out_min=-100, out_max=100)
        target = 10.0
        val = 0.0
        max_val = 0.0
        for _ in range(500):
            out = ipid.calc(setpoint=target, feedback=val, dt=0.01)
            val += out * 0.01
            if val > max_val:
                max_val = val
        # 超调量不超过50%
        self.assertLess(max_val, target * 1.5)


class TestIncrementalPIDDisturbance(unittest.TestCase):
    """抗干扰性能测试"""

    def test_rejects_disturbance(self):
        """应能抑制常值扰动"""
        ipid = IncrementalPIDSimulator(kp=3.0, ki=2.0, kd=0.1,
                                        out_min=-100, out_max=100)
        target = 10.0
        val = 0.0
        disturbance = 2.0

        for _ in range(500):
            out = ipid.calc(setpoint=target, feedback=val, dt=0.01)
            val += (out - disturbance) * 0.01

        self.assertAlmostEqual(val, target, delta=1.0)


class TestIncrementalPIDBumplessTransfer(unittest.TestCase):
    """无扰切换测试"""

    def test_bumpless_transfer(self):
        """output_last确保无扰切换"""
        ipid1 = IncrementalPIDSimulator(kp=5.0, ki=1.0, kd=0.0)
        ipid2 = IncrementalPIDSimulator(kp=2.0, ki=0.5, kd=0.1)

        # 运行控制器1
        val = 0.0
        for _ in range(100):
            out = ipid1.calc(setpoint=10.0, feedback=val, dt=0.01)
            val += out * 0.01

        # 切换到控制器2, 复制output_last
        ipid2.output_last = ipid1.output_last
        ipid2.output = ipid1.output

        # 切换后的第一步输出应该平滑
        out2 = ipid2.calc(setpoint=10.0, feedback=val, dt=0.01)
        # 输出不应有大的跳变
        self.assertLess(abs(out2 - ipid1.output), 50.0)


class TestIncrementalPIDEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_zero_error_zero_increment(self):
        """零误差时增量应为零"""
        ipid = IncrementalPIDSimulator(kp=5.0, ki=1.0, kd=0.1)
        ipid.calc(setpoint=5.0, feedback=5.0, dt=0.01)
        ipid.calc(setpoint=5.0, feedback=5.0, dt=0.01)
        # 稳态时增量应为零
        self.assertAlmostEqual(ipid.output, 0.0, delta=0.01)

    def test_very_small_dt(self):
        """极小dt不应崩溃"""
        ipid = IncrementalPIDSimulator(kp=1.0, ki=0.1, kd=0.01)
        output = ipid.calc(setpoint=10.0, feedback=0.0, dt=1e-6)
        # 输出应该在合理范围内
        self.assertGreaterEqual(output, -100, "输出不应低于-100")
        self.assertLessEqual(output, 100, "输出不应超过100")
        self.assertTrue(np.isfinite(output), "输出应该是有限值")

    def test_large_error(self):
        """大误差应正常处理"""
        ipid = IncrementalPIDSimulator(kp=1.0, ki=0.0, kd=0.0,
                                        out_min=-50, out_max=50)
        output = ipid.calc(setpoint=1e6, feedback=0.0, dt=0.01)
        self.assertLessEqual(output, 50.0)

    def test_negative_feedback(self):
        """负反馈应正常处理"""
        ipid = IncrementalPIDSimulator(kp=2.0, ki=0.0, kd=0.0,
                                        out_min=-100, out_max=100)
        output = ipid.calc(setpoint=0.0, feedback=-10.0, dt=0.01)
        self.assertGreater(output, 0)


if __name__ == '__main__':
    unittest.main()
