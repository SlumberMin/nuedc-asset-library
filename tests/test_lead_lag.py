#!/usr/bin/env python3
"""
超前-滞后补偿器(Lead-Lag Compensator)测试
覆盖: 频率响应验证、相位裕度改善、稳定性分析、
      Bode图计算、阻尼比控制、频域性能验证
目标: 验证超前补偿器的相位提升、滞后补偿器的增益提升效果
"""

import sys
import os
import math
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class LeadLagCompensator:
    """超前-滞后补偿器实现"""

    def __init__(self, compensator_type="lead"):
        self.compensator_type = compensator_type
        # 超前补偿器参数
        self.zeta = 0.707  # 阻尼比
        self.wn = 10.0     # 自然频率
        # 补偿器零极点
        self.zero = 0.0
        self.pole = 0.0
        # 增益
        self.K = 1.0
        # 状态
        self.x1 = 0.0
        self.x2 = 0.0

    def configure_lead_compensator(self, PM_target, crossover_freq, PM_untuned):
        """配置超前补偿器以达到目标相位裕度"""
        # 计算需要的相位提升
        phase_boost_needed = PM_target - PM_untuned + 10  # 额外10度余量
        phase_boost_needed = min(phase_boost_needed, 60)  # 超前补偿器最大约60度

        # 计算alpha = (1+sin(φ))/(1-sin(φ))
        phi_rad = math.radians(phase_boost_needed)
        self.alpha = (1 + math.sin(phi_rad)) / (1 - math.sin(phi_rad))

        # 零点和极点位置
        self.zero = crossover_freq / math.sqrt(self.alpha)
        self.pole = crossover_freq * math.sqrt(self.alpha)

        # 补偿器增益
        self.K = 1.0 / math.sqrt(self.alpha)

        return {
            "alpha": self.alpha,
            "zero": self.zero,
            "pole": self.pole,
            "phase_boost": phase_boost_needed
        }

    def configure_lag_compensator(self, gain_margin_target, crossover_freq, gain_untuned):
        """配置滞后补偿器以达到目标增益裕度"""
        # 计算需要的增益衰减
        gain_margin_db = 20 * math.log10(gain_untuned) - gain_margin_target

        # 计算beta
        beta = 10 ** (gain_margin_db / 20)
        beta = max(beta, 1.0)  # beta >= 1

        # 零点和极点位置 (滞后补偿器)
        self.zero = crossover_freq / (10 * beta)  # 零点在交叉频率1/10处
        self.pole = self.zero / beta

        # 补偿器增益
        self.K = 1.0

        return {
            "beta": beta,
            "zero": self.zero,
            "pole": self.pole,
            "gain_margin": gain_margin_db
        }

    def frequency_response(self, omega):
        """计算补偿器频率响应 H(jω)"""
        # H(s) = K * (s + z) / (s + p)
        # H(jω) = K * (jω + z) / (jω + p)

        numerator_real = self.K * self.zero
        numerator_imag = self.K * omega
        denominator_real = self.pole
        denominator_imag = omega

        # 复数除法
        mag_num = math.sqrt(numerator_real**2 + numerator_imag**2)
        mag_den = math.sqrt(denominator_real**2 + denominator_imag**2)
        magnitude = mag_num / mag_den

        phase_num = math.atan2(numerator_imag, numerator_real)
        phase_den = math.atan2(denominator_imag, denominator_real)
        phase = math.degrees(phase_num - phase_den)

        return magnitude, phase

    def apply(self, error, dt):
        """应用补偿器计算控制信号"""
        # 超前-滞后补偿器: H(s) = K * (s + z) / (s + p)
        # 微分方程: p*x1_dot = -p*x1 + z*K*u  (u=error)
        #           输出 y = x1 + K*u

        alpha_coeff = self.pole
        x1_dot = (-self.pole * self.x1 + self.zero * self.K * error) / alpha_coeff
        self.x1 += x1_dot * dt

        output = self.x1 + self.K * error
        return output

    def reset(self):
        """重置补偿器状态"""
        self.x1 = 0.0
        self.x2 = 0.0


class TestLeadCompensator(unittest.TestCase):
    """超前补偿器测试"""

    def setUp(self):
        self.compensator = LeadLagCompensator(compensator_type="lead")

    def test_phase_boost_calculation(self):
        """测试相位提升计算"""
        config = self.compensator.configure_lead_compensator(
            PM_target=60,
            crossover_freq=10.0,
            PM_untuned=30
        )

        # 相位提升应该为 60 - 30 + 10 = 40 度
        self.assertAlmostEqual(config["phase_boost"], 40, delta=2)
        self.assertGreater(config["alpha"], 1)  # alpha > 1 for lead
        self.assertGreater(config["pole"], config["zero"])  # pole > zero for lead

    def test_frequency_response_magnitude(self):
        """测试频率响应幅值"""
        self.compensator.configure_lead_compensator(60, 10.0, 30)

        # 在零点频率处，幅值应该上升
        mag_at_zero, _ = self.compensator.frequency_response(self.compensator.zero)

        # 在极点频率处，幅值应该下降
        mag_at_pole, _ = self.compensator.frequency_response(self.compensator.pole)

        self.assertGreater(mag_at_zero, 1.0)
        self.assertLess(mag_at_pole, 1.0)

    def test_phase_at_crossover(self):
        """测试交叉频率处的相位"""
        self.compensator.configure_lead_compensator(60, 10.0, 30)

        _, phase = self.compensator.frequency_response(10.0)

        # 在交叉频率处应该有正相位提升
        self.assertGreater(phase, 0)

    def test_compensator_apply_step_response(self):
        """测试补偿器阶跃响应"""
        dt = 0.001
        time_sim = 2.0
        steps = int(time_sim / dt)

        self.compensator.configure_lead_compensator(60, 10.0, 30)

        outputs = []
        for i in range(steps):
            error = 1.0  # 单位阶跃
            output = self.compensator.apply(error, dt)
            outputs.append(output)

        # 阶跃响应应该达到稳态
        self.assertAlmostEqual(outputs[-1], 1.0, delta=0.1)

    def test_phase_boost_max_limit(self):
        """测试相位提升上限"""
        config = self.compensator.configure_lead_compensator(
            PM_target=120,  # 不现实的目标
            crossover_freq=10.0,
            PM_untuned=10
        )

        # 相位提升不应超过60度
        self.assertLessEqual(config["phase_boost"], 60)

    def test_alpha_range(self):
        """测试alpha参数范围"""
        config = self.compensator.configure_lead_compensator(60, 10.0, 30)

        # alpha应该在1到10之间
        self.assertGreater(config["alpha"], 1)
        self.assertLess(config["alpha"], 10)


class TestLagCompensator(unittest.TestCase):
    """滞后补偿器测试"""

    def setUp(self):
        self.compensator = LeadLagCompensator(compensator_type="lag")

    def test_gain_margin_improvement(self):
        """测试增益裕度改善"""
        config = self.compensator.configure_lag_compensator(
            gain_margin_target=10,  # 目标10dB增益裕度
            crossover_freq=10.0,
            gain_untuned=100  # 未补偿系统增益
        )

        self.assertGreater(config["beta"], 1)
        self.assertGreater(config["gain_margin"], 0)

    def test_frequency_response_attenuation(self):
        """测试高频衰减"""
        self.compensator.configure_lag_compensator(10, 10.0, 100)

        # 在高频处应该有衰减
        mag_high, _ = self.compensator.frequency_response(100.0)
        mag_low, _ = self.compensator.frequency_response(0.1)

        self.assertLess(mag_high, mag_low)

    def test_lag_compensator_low_frequency_gain(self):
        """测试低频增益"""
        self.compensator.configure_lag_compensator(10, 10.0, 100)

        # 低频增益应该接近1 (0dB)
        mag_dc, _ = self.compensator.frequency_response(0.01)
        self.assertAlmostEqual(mag_dc, 1.0, delta=0.1)

    def test_steady_state_error_improvement(self):
        """测试稳态误差改善"""
        dt = 0.001
        time_sim = 5.0
        steps = int(time_sim / dt)

        self.compensator.configure_lag_compensator(10, 10.0, 100)

        errors = []
        for i in range(steps):
            error = 1.0  # 单位阶跃
            output = self.compensator.apply(error, dt)
            errors.append(abs(1.0 - output))

        # 稳态误差应该小于10%
        self.assertLess(errors[-1], 0.1)


class TestFrequencyResponse(unittest.TestCase):
    """频率响应测试"""

    def setUp(self):
        self.compensator = LeadLagCompensator()
        self.compensator.configure_lead_compensator(60, 10.0, 30)

    def test_bode_plot_points(self):
        """测试Bode图计算"""
        frequencies = [0.1, 1.0, 10.0, 100.0]

        for freq in frequencies:
            mag, phase = self.compensator.frequency_response(freq)
            self.assertGreater(mag, 0)
            self.assertGreaterEqual(phase, -180)
            self.assertLessEqual(phase, 180)

    def test_phase_at_zero_frequency(self):
        """测试零频率处的相位"""
        mag_dc, phase_dc = self.compensator.frequency_response(0.001)
        self.assertAlmostEqual(phase_dc, 0, delta=1)

    def test_high_frequency_asymptote(self):
        """测试高频渐近线"""
        # 在高频处，增益应该趋于 K * (zero/pole)
        mag_high, _ = self.compensator.frequency_response(1000.0)
        expected_high = self.compensator.K * self.compensator.zero / self.compensator.pole
        self.assertAlmostEqual(mag_high, expected_high, delta=0.01)

    def test_low_frequency_asymptote(self):
        """测试低频渐近线"""
        # 在低频处，增益应该趋于 K
        mag_low, _ = self.compensator.frequency_response(0.001)
        self.assertAlmostEqual(mag_low, self.compensator.K, delta=0.1)


class TestStabilityImprovement(unittest.TestCase):
    """稳定性改善测试"""

    def test_phase_margin_improvement(self):
        """测试相位裕度改善"""
        # 假设未补偿系统在交叉频率处的相位为-120度
        PM_untuned = 180 - 120  # 60度
        PM_target = 70

        compensator = LeadLagCompensator()
        config = compensator.configure_lead_compensator(PM_target, 10.0, PM_untuned)

        # 相位裕度应该改善
        self.assertGreater(config["phase_boost"], 0)

    def test_bandwidth_extension(self):
        """测试带宽扩展"""
        compensator = LeadLagCompensator()
        config = compensator.configure_lead_compensator(60, 20.0, 30)

        # 交叉频率应该在设计值
        self.assertAlmostEqual(config["zero"] * math.sqrt(config["alpha"]),
                               20.0, delta=1)

    def test_settling_time_improvement(self):
        """测试稳定时间改善"""
        dt = 0.001
        time_sim = 3.0
        steps = int(time_sim / dt)

        compensator = LeadLagCompensator()
        compensator.configure_lead_compensator(60, 10.0, 30)

        outputs = []
        for i in range(steps):
            output = compensator.apply(1.0, dt)
            outputs.append(output)

        # 找到首次达到并保持在90%的时间
        settling_time = None
        for i in range(len(outputs)):
            if all(abs(outputs[j] - 1.0) < 0.1 for j in range(i, len(outputs))):
                settling_time = i * dt
                break

        # 应该在2秒内稳定
        self.assertIsNotNone(settling_time)
        self.assertLess(settling_time, 2.0)


class TestCompensatorReset(unittest.TestCase):
    """补偿器重置测试"""

    def test_reset_clears_state(self):
        """测试重置清除状态"""
        compensator = LeadLagCompensator()
        compensator.configure_lead_compensator(60, 10.0, 30)

        # 先应用一些信号
        for _ in range(100):
            compensator.apply(1.0, 0.01)

        # 重置
        compensator.reset()

        self.assertEqual(compensator.x1, 0.0)
        self.assertEqual(compensator.x2, 0.0)

    def test_reset_returns_to_initial(self):
        """测试重置返回初始状态"""
        compensator = LeadLagCompensator()
        compensator.configure_lead_compensator(60, 10.0, 30)

        # 记录初始响应
        initial_output = compensator.apply(1.0, 0.01)

        # 应用信号
        for _ in range(100):
            compensator.apply(1.0, 0.01)

        # 重置
        compensator.reset()

        # 再次应用应该得到相同初始响应
        reset_output = compensator.apply(1.0, 0.01)
        self.assertAlmostEqual(initial_output, reset_output, delta=0.001)


if __name__ == '__main__':
    unittest.main()
