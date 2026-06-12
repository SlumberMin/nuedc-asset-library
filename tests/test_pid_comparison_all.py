#!/usr/bin/env python3
"""
所有PID变种对比单元测试
覆盖: 9种PID变种(经典、增量式、抗积分饱和、不完全微分、微分先行、
      串级、模糊、自适应、Smith预估)的统一仿真对比、
      被控对象模型(二阶+带延迟)、性能指标(上升时间、超调量、稳态误差、ITAE、IAE)、
      综合评分、阶跃响应、方波跟踪
测试对象: 15_simulation/pid_comparison_all.py
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from _15_simulation.pid_comparison_all import (
    Plant2,
    DelayPlant,
    PIDBase,
    ClassicPID,
    IncrementalPID,
    AntiWindupPID,
    IncompleteDiffPID,
    DerivativeFirstPID,
    CascadePID,
    SimpleFuzzyPID,
    AdaptivePID,
    SmithPredictorPID,
    run_sim,
    calc_metrics,
)


# ── 被控对象测试 ──

class TestPlant2(unittest.TestCase):
    """二阶被控对象测试"""

    def test_zero_input_stays_zero(self):
        plant = Plant2(a1=1.2, a0=0.8, b=1.0)
        for _ in range(1000):
            plant.update(0.0)
        self.assertAlmostEqual(plant.x1, 0.0, delta=0.01)

    def test_step_response(self):
        plant = Plant2(a1=1.2, a0=0.8, b=1.0)
        for _ in range(1000):
            plant.update(1.0)
        self.assertGreater(plant.x1, 0.0)

    def test_reset(self):
        plant = Plant2()
        for _ in range(100):
            plant.update(1.0)
        plant.reset()
        self.assertEqual(plant.x1, 0.0)
        self.assertEqual(plant.x2, 0.0)


class TestDelayPlant(unittest.TestCase):
    """带延迟的被控对象测试"""

    def test_delay_effect(self):
        """带延迟的系统响应应比无延迟慢"""
        plant_no_delay = Plant2()
        plant_delay = DelayPlant(delay=0.3)
        for _ in range(500):
            plant_no_delay.update(1.0)
            plant_delay.update(1.0)
        self.assertGreater(plant_no_delay.x1, plant_delay.y)

    def test_reset(self):
        plant = DelayPlant(delay=0.3)
        for _ in range(100):
            plant.update(1.0)
        plant.reset()
        self.assertEqual(plant.y, 0.0)


# ── PIDBase测试 ──

class TestPIDBase(unittest.TestCase):
    """PID基类测试"""

    def test_initialization(self):
        pid = PIDBase(Kp=2.5, Ki=1.2, Kd=0.8, dt=0.01)
        self.assertEqual(pid.Kp, 2.5)
        self.assertEqual(pid.Ki, 1.2)
        self.assertEqual(pid.Kd, 0.8)

    def test_compute(self):
        pid = PIDBase(Kp=1.0, Ki=0.0, Kd=0.0, dt=0.01)
        u = pid.compute(1.0)
        self.assertAlmostEqual(u, 1.0, delta=0.1)

    def test_reset(self):
        pid = PIDBase(Kp=1.0, Ki=1.0, Kd=0.0, dt=0.01)
        pid.compute(1.0)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)


# ── 各PID变种测试 ──

class TestClassicPID(unittest.TestCase):
    """经典PID测试"""

    def test_name(self):
        self.assertEqual(ClassicPID.name, '经典PID')

    def test_runs(self):
        pid = ClassicPID(2.5, 1.2, 0.8, dt=0.01)
        plant = Plant2()
        y_hist, u_hist = run_sim(pid, plant, np.ones(500) * 3.0, np.arange(0, 5, 0.01))
        self.assertEqual(len(y_hist), 500)
        self.assertFalse(np.any(np.isnan(y_hist)))


class TestIncrementalPID(unittest.TestCase):
    """增量式PID测试"""

    def test_name(self):
        self.assertEqual(IncrementalPID.name, '增量式PID')

    def test_runs(self):
        pid = IncrementalPID(2.5, 1.2, 0.8, dt=0.01)
        plant = Plant2()
        y_hist, u_hist = run_sim(pid, plant, np.ones(500) * 3.0, np.arange(0, 5, 0.01))
        self.assertEqual(len(y_hist), 500)
        self.assertFalse(np.any(np.isnan(y_hist)))

    def test_reset_extra_state(self):
        pid = IncrementalPID(2.5, 1.2, 0.8, dt=0.01)
        pid.compute(1.0)
        pid.reset()
        self.assertEqual(pid.prev2_error, 0.0)


class TestAntiWindupPID(unittest.TestCase):
    """抗积分饱和PID测试"""

    def test_name(self):
        self.assertEqual(AntiWindupPID.name, '抗积分饱和PID')

    def test_output_clamped(self):
        """输出应被限幅"""
        pid = AntiWindupPID(2.5, 1.2, 0.8, dt=0.01, u_max=8, u_min=-8)
        for i in range(1000):
            u = pid.compute(10.0)  # 大误差
            self.assertGreaterEqual(u, -8.0)
            self.assertLessEqual(u, 8.0)

    def test_runs(self):
        pid = AntiWindupPID(2.5, 1.2, 0.8, dt=0.01, u_max=8, u_min=-8)
        plant = Plant2()
        y_hist, u_hist = run_sim(pid, plant, np.ones(500) * 3.0, np.arange(0, 5, 0.01))
        self.assertEqual(len(y_hist), 500)


class TestIncompleteDiffPID(unittest.TestCase):
    """不完全微分PID测试"""

    def test_name(self):
        self.assertEqual(IncompleteDiffPID.name, '不完全微分PID')

    def test_runs(self):
        pid = IncompleteDiffPID(2.5, 1.2, 0.8, dt=0.01, alpha=0.15)
        plant = Plant2()
        y_hist, u_hist = run_sim(pid, plant, np.ones(500) * 3.0, np.arange(0, 5, 0.01))
        self.assertEqual(len(y_hist), 500)
        self.assertFalse(np.any(np.isnan(y_hist)))


class TestDerivativeFirstPID(unittest.TestCase):
    """微分先行PID测试"""

    def test_name(self):
        self.assertEqual(DerivativeFirstPID.name, '微分先行PID')

    def test_runs(self):
        pid = DerivativeFirstPID(2.5, 1.2, 0.8, dt=0.01, beta=0.8)
        plant = Plant2()
        y_hist, u_hist = run_sim(pid, plant, np.ones(500) * 3.0, np.arange(0, 5, 0.01))
        self.assertEqual(len(y_hist), 500)


class TestCascadePID(unittest.TestCase):
    """串级PID测试"""

    def test_name(self):
        self.assertEqual(CascadePID.name, '串级PID')

    def test_runs(self):
        pid = CascadePID(2.5, 1.2, 0.8, 4.0, 2.0, 0.3, dt=0.01)
        plant = Plant2()
        y_hist, u_hist = run_sim(pid, plant, np.ones(500) * 3.0, np.arange(0, 5, 0.01))
        self.assertEqual(len(y_hist), 500)

    def test_reset(self):
        pid = CascadePID(2.5, 1.2, 0.8, 4.0, 2.0, 0.3, dt=0.01)
        pid.compute(1.0, y_inner=0.5)
        pid.reset()
        self.assertEqual(pid.outer.integral, 0.0)
        self.assertEqual(pid.inner.integral, 0.0)


class TestSimpleFuzzyPID(unittest.TestCase):
    """简化模糊PID测试"""

    def test_name(self):
        self.assertEqual(SimpleFuzzyPID.name, '模糊PID')

    def test_runs(self):
        pid = SimpleFuzzyPID(2.5, 1.2, 0.8, dt=0.01)
        plant = Plant2()
        y_hist, u_hist = run_sim(pid, plant, np.ones(500) * 3.0, np.arange(0, 5, 0.01))
        self.assertEqual(len(y_hist), 500)

    def test_adaptive_params(self):
        """参数应随误差变化"""
        pid = SimpleFuzzyPID(2.5, 1.2, 0.8, dt=0.01)
        u1 = pid.compute(5.0)  # 大误差
        kp_at_large_error = pid.Kp
        for _ in range(50):
            pid.compute(0.01)  # 小误差
        kp_at_small_error = pid.Kp
        # 大误差时Kp应更大
        self.assertGreater(kp_at_large_error, kp_at_small_error)


class TestAdaptivePID(unittest.TestCase):
    """自适应PID(增益调度)测试"""

    def test_name(self):
        self.assertEqual(AdaptivePID.name, '自适应PID')

    def test_runs(self):
        pid = AdaptivePID(2.5, 1.2, 0.8, dt=0.01)
        plant = Plant2()
        y_hist, u_hist = run_sim(pid, plant, np.ones(500) * 3.0, np.arange(0, 5, 0.01))
        self.assertEqual(len(y_hist), 500)

    def test_gain_scheduling(self):
        """不同误差应导致不同增益"""
        pid = AdaptivePID(2.5, 1.2, 0.8, dt=0.01)
        pid.compute(5.0)  # 大误差(ae>2)
        kp_large = pid.Kp
        pid2 = AdaptivePID(2.5, 1.2, 0.8, dt=0.01)
        pid2.compute(0.1)  # 小误差(ae<0.5)
        kp_small = pid2.Kp
        self.assertGreater(kp_large, kp_small)


class TestSmithPredictorPID(unittest.TestCase):
    """Smith预估PID测试"""

    def test_name(self):
        self.assertEqual(SmithPredictorPID.name, 'Smith预估PID')

    def test_runs(self):
        pid = SmithPredictorPID(2.5, 1.2, 0.8, dt=0.01, delay=0.2)
        plant = DelayPlant(delay=0.2)
        sp = np.ones(500) * 3.0
        t = np.arange(0, 5, 0.01)
        y_hist, u_hist = run_sim(pid, plant, sp, t)
        self.assertEqual(len(y_hist), 500)

    def test_reset(self):
        pid = SmithPredictorPID(2.5, 1.2, 0.8, dt=0.01, delay=0.2)
        pid.compute(1.0, y_actual=0.5, u_prev=0.5)
        pid.reset()
        self.assertEqual(pid.pid.integral, 0.0)


# ── 仿真函数测试 ──

class TestRunSim(unittest.TestCase):
    """run_sim测试"""

    def test_returns_arrays(self):
        plant = Plant2()
        pid = ClassicPID(2.5, 1.2, 0.8, dt=0.01)
        sp = np.ones(500) * 3.0
        t = np.arange(0, 5, 0.01)
        y_hist, u_hist = run_sim(pid, plant, sp, t)
        self.assertEqual(len(y_hist), 500)
        self.assertEqual(len(u_hist), 500)

    def test_all_controllers_run(self):
        """所有9种控制器应能正常运行"""
        t = np.arange(0, 5, 0.01)
        sp = np.ones(len(t)) * 3.0
        dt = 0.01
        controllers = [
            ('经典PID', ClassicPID(2.5, 1.2, 0.8, dt)),
            ('增量式PID', IncrementalPID(2.5, 1.2, 0.8, dt)),
            ('抗积分饱和PID', AntiWindupPID(2.5, 1.2, 0.8, dt, u_max=8, u_min=-8)),
            ('不完全微分PID', IncompleteDiffPID(2.5, 1.2, 0.8, dt, alpha=0.15)),
            ('微分先行PID', DerivativeFirstPID(2.5, 1.2, 0.8, dt, beta=0.8)),
            ('串级PID', CascadePID(2.5, 1.2, 0.8, 4.0, 2.0, 0.3, dt)),
            ('模糊PID', SimpleFuzzyPID(2.5, 1.2, 0.8, dt)),
            ('自适应PID', AdaptivePID(2.5, 1.2, 0.8, dt)),
            ('Smith预估PID', SmithPredictorPID(2.5, 1.2, 0.8, dt, delay=0.2)),
        ]
        for name, ctrl in controllers:
            plant = DelayPlant(delay=0.2) if name == 'Smith预估PID' else Plant2()
            y_hist, u_hist = run_sim(ctrl, plant, sp, t)
            self.assertEqual(len(y_hist), len(t), f"{name} length mismatch")
            self.assertFalse(np.any(np.isnan(y_hist)), f"{name} has NaN")


# ── 性能指标测试 ──

class TestCalcMetrics(unittest.TestCase):
    """性能指标计算测试"""

    def test_perfect_tracking(self):
        """完美跟踪应有零误差指标"""
        t = np.arange(0, 5, 0.01)
        sp = np.ones_like(t) * 3.0
        y = np.ones_like(t) * 3.0
        metrics = calc_metrics(y, sp, t)
        self.assertAlmostEqual(metrics['ss_error'], 0.0)
        self.assertAlmostEqual(metrics['iae'], 0.0)
        self.assertAlmostEqual(metrics['overshoot'], 0.0)

    def test_has_all_keys(self):
        t = np.arange(0, 5, 0.01)
        sp = np.ones_like(t) * 3.0
        y = np.ones_like(t) * 2.0  # 有稳态误差
        metrics = calc_metrics(y, sp, t)
        for key in ['rise_time', 'overshoot', 'ss_error', 'itae', 'iae']:
            self.assertIn(key, metrics)

    def test_overshoot_positive(self):
        """超调量应非负"""
        t = np.arange(0, 5, 0.01)
        sp = np.ones_like(t) * 3.0
        y = np.ones_like(t) * 4.0  # 超调
        metrics = calc_metrics(y, sp, t)
        self.assertGreater(metrics['overshoot'], 0)

    def test_zero_setpoint_overshoot(self):
        """设定值为0时超调量应为0"""
        t = np.arange(0, 5, 0.01)
        sp = np.zeros_like(t)
        y = np.zeros_like(t)
        metrics = calc_metrics(y, sp, t)
        self.assertEqual(metrics['overshoot'], 0)


# ── 综合对比测试 ──

class TestComprehensiveComparison(unittest.TestCase):
    """综合对比测试"""

    def test_all_controllers_converge(self):
        """所有控制器最终输出应接近设定值"""
        dt = 0.01
        t = np.arange(0, 8, dt)
        sp = np.ones_like(t) * 3.0
        controllers = [
            ClassicPID(2.5, 1.2, 0.8, dt),
            IncrementalPID(2.5, 1.2, 0.8, dt),
            AntiWindupPID(2.5, 1.2, 0.8, dt, u_max=8, u_min=-8),
            IncompleteDiffPID(2.5, 1.2, 0.8, dt, alpha=0.15),
            DerivativeFirstPID(2.5, 1.2, 0.8, dt, beta=0.8),
            CascadePID(2.5, 1.2, 0.8, 4.0, 2.0, 0.3, dt),
            SimpleFuzzyPID(2.5, 1.2, 0.8, dt),
            AdaptivePID(2.5, 1.2, 0.8, dt),
            SmithPredictorPID(2.5, 1.2, 0.8, dt, delay=0.2),
        ]
        names = ['经典PID', '增量式PID', '抗积分饱和PID', '不完全微分PID',
                 '微分先行PID', '串级PID', '模糊PID', '自适应PID', 'Smith预估PID']

        for name, ctrl in zip(names, controllers):
            plant = DelayPlant(delay=0.2) if name == 'Smith预估PID' else Plant2()
            y, u = run_sim(ctrl, plant, sp, t)
            # 最终10%的平均输出应接近设定值
            ss_start = int(0.9 * len(t))
            avg_output = np.mean(y[ss_start:])
            self.assertAlmostEqual(avg_output, 3.0, delta=1.5,
                                   msg=f"{name} did not converge (avg={avg_output:.2f})")

    def test_different_controllers_differ(self):
        """不同控制器应产生不同结果"""
        dt = 0.01
        t = np.arange(0, 5, dt)
        sp = np.ones_like(t) * 3.0

        plant1 = Plant2()
        y_classic, _ = run_sim(ClassicPID(2.5, 1.2, 0.8, dt), plant1, sp, t)

        plant2 = Plant2()
        y_fuzzy, _ = run_sim(SimpleFuzzyPID(2.5, 1.2, 0.8, dt), plant2, sp, t)

        # 不应完全相同
        self.assertFalse(np.allclose(y_classic, y_fuzzy, atol=1e-4))


if __name__ == '__main__':
    unittest.main()
