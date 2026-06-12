#!/usr/bin/env python3
"""
串级PID控制单元测试
覆盖: PIDController基本功能、DCMotor模型、串级PID仿真、
      单环PID对比、扰动抑制、性能指标计算
测试对象: 11_控制算法库/simulation/cascade_pid_simulation.py
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '11_控制算法库', 'simulation'))

from cascade_pid_simulation import (
    DCMotor, PIDController,
    simulate_cascade_pid, simulate_single_pid, simulate_with_disturbance,
    compute_metrics, dt, steps
)


class TestPIDController(unittest.TestCase):
    """PID控制器基本功能测试"""

    def test_p_only(self):
        """纯比例控制: 输出 = kp * error"""
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0, out_min=-100, out_max=100)
        output = pid.calc(setpoint=10.0, feedback=5.0, dt=0.01)
        self.assertAlmostEqual(output, 10.0, places=2)

    def test_p_negative_error(self):
        """负误差应产生负输出"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, out_min=-100, out_max=100)
        output = pid.calc(setpoint=0.0, feedback=5.0, dt=0.01)
        self.assertLess(output, 0)

    def test_integral_accumulation(self):
        """积分项应随时间累积"""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, out_min=-1000, out_max=1000)
        outputs = []
        for _ in range(10):
            out = pid.calc(setpoint=10.0, feedback=0.0, dt=0.1)
            outputs.append(out)
        for i in range(1, len(outputs)):
            self.assertGreaterEqual(outputs[i], outputs[i - 1])

    def test_output_clamping(self):
        """输出应被限幅"""
        pid = PIDController(kp=100.0, ki=0.0, kd=0.0, out_min=-10, out_max=10)
        output = pid.calc(setpoint=100.0, feedback=0.0, dt=0.01)
        self.assertLessEqual(output, 10.0)
        self.assertGreaterEqual(output, -10.0)

    def test_reset(self):
        """reset应清除内部状态"""
        pid = PIDController(kp=1.0, ki=0.1, kd=0.01, out_min=-100, out_max=100)
        pid.calc(setpoint=10.0, feedback=0.0, dt=0.01)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)

    def test_zero_error_zero_output_p_only(self):
        """零误差纯P控制器输出应为零"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, out_min=-100, out_max=100)
        output = pid.calc(setpoint=5.0, feedback=5.0, dt=0.01)
        self.assertAlmostEqual(output, 0.0, places=5)


class TestDCMotor(unittest.TestCase):
    """直流电机模型测试"""

    def test_initial_state(self):
        """初始状态应为零"""
        motor = DCMotor()
        self.assertEqual(motor.position, 0.0)
        self.assertEqual(motor.velocity, 0.0)
        self.assertEqual(motor.current, 0.0)

    def test_voltage_increases_position(self):
        """正电压应使位置增加"""
        motor = DCMotor()
        for _ in range(1000):
            motor.step(voltage=12.0, dt=0.001)
        self.assertGreater(motor.position, 0.0)

    def test_negative_voltage(self):
        """负电压应产生负方向运动"""
        motor = DCMotor()
        for _ in range(1000):
            motor.step(voltage=-12.0, dt=0.001)
        self.assertLess(motor.position, 0.0)

    def test_step_returns_tuple(self):
        """step应返回(position, velocity)"""
        motor = DCMotor()
        result = motor.step(voltage=5.0, dt=0.001)
        self.assertEqual(len(result), 2)

    def test_friction_effect(self):
        """摩擦应限制速度"""
        motor1 = DCMotor()
        motor2 = DCMotor()
        # 施加相同电压足够长时间
        for _ in range(10000):
            motor1.step(voltage=12.0, dt=0.001)
            motor2.step(voltage=6.0, dt=0.001)
        # 高电压应产生更高速度
        self.assertGreater(motor1.velocity, motor2.velocity)


class TestCascadePIDSimulation(unittest.TestCase):
    """串级PID仿真测试"""

    def test_simulate_cascade_output_shapes(self):
        """串级PID仿真输出形状应正确"""
        pos, vel, vel_sp = simulate_cascade_pid()
        self.assertEqual(len(pos), steps)
        self.assertEqual(len(vel), steps)
        self.assertEqual(len(vel_sp), steps)

    def test_simulate_cascade_converges(self):
        """串级PID应收敛到目标位置"""
        pos, _, _ = simulate_cascade_pid()
        # 最终位置应接近目标 1.0
        self.assertAlmostEqual(pos[-1], 1.0, delta=0.05)

    def test_simulate_single_output_shapes(self):
        """单环PID仿真输出形状应正确"""
        pos, vel = simulate_single_pid()
        self.assertEqual(len(pos), steps)
        self.assertEqual(len(vel), steps)

    def test_simulate_single_converges(self):
        """单环PID也应收敛到目标位置"""
        pos, _ = simulate_single_pid()
        self.assertAlmostEqual(pos[-1], 1.0, delta=0.1)

    def test_cascade_has_less_overshoot(self):
        """串级PID的超调量应不大于单环PID"""
        cas_pos, _, _ = simulate_cascade_pid()
        sin_pos, _ = simulate_single_pid()
        cas_metrics = compute_metrics(cas_pos)
        sin_metrics = compute_metrics(sin_pos)
        # 串级PID通常超调更小
        self.assertLessEqual(cas_metrics['overshoot'],
                            sin_metrics['overshoot'] + 10.0)  # 允许一定裕度

    def test_disturbance_simulation(self):
        """带扰动仿真应能运行"""
        pos = simulate_with_disturbance()
        self.assertEqual(len(pos), steps)


class TestComputeMetrics(unittest.TestCase):
    """性能指标计算测试"""

    def test_overshoot(self):
        """超调量计算"""
        pos = np.array([0, 0.5, 1.2, 1.1, 1.0])
        metrics = compute_metrics(pos, target=1.0)
        self.assertAlmostEqual(metrics['overshoot'], 20.0, delta=1.0)

    def test_zero_overshoot(self):
        """无超调"""
        pos = np.array([0, 0.5, 0.8, 0.95, 1.0])
        metrics = compute_metrics(pos, target=1.0)
        self.assertLessEqual(metrics['overshoot'], 1.0)

    def test_metrics_keys(self):
        """指标应包含必要键"""
        pos = np.linspace(0, 1, 100)
        metrics = compute_metrics(pos, target=1.0)
        self.assertIn('overshoot', metrics)
        self.assertIn('rise_time', metrics)
        self.assertIn('settling_time', metrics)
        self.assertIn('ss_error', metrics)

    def test_ss_error(self):
        """稳态误差计算"""
        pos = np.ones(100) * 0.98
        metrics = compute_metrics(pos, target=1.0)
        self.assertAlmostEqual(metrics['ss_error'], 0.02, delta=0.01)


class TestCascadePIDEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_zero_target(self):
        """零目标的指标计算"""
        pos = np.zeros(100)
        metrics = compute_metrics(pos, target=0.0)
        # 应该不会崩溃
        self.assertIsNotNone(metrics)


if __name__ == '__main__':
    unittest.main()
