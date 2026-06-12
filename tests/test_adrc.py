#!/usr/bin/env python3
"""
ADRC自抗扰控制器单元测试
覆盖: TD跟踪微分器、ESO扩张状态观测器、NLSEF非线性误差反馈、完整ADRC计算
注意: 使用纯 Python 模拟 C ADRC 逻辑，测试算法正确性
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ── Python 模拟实现 ──────────────────────────────────────────

def _fal(e, alpha, delta):
    """非线性函数 fal(e, alpha, delta)"""
    if abs(e) > delta:
        return math.copysign(abs(e) ** alpha, e)
    else:
        return e / (delta ** (1 - alpha))


class ADRC_TDSimulator:
    """跟踪微分器(TD) — fhan函数实现"""

    def __init__(self, r=50.0, h=0.01):
        self.r = r
        self.h = h
        self.x1 = 0.0
        self.x2 = 0.0

    def _fhan(self, x1, x2, r, h):
        """最速综合函数"""
        d = r * h * h
        a0 = h * x2
        y = x1 + a0
        a1 = math.sqrt(d * (d + 8.0 * abs(y)))
        a2 = a0 + math.copysign(1.0, y) * (a1 - d) / 2.0
        sy = (math.copysign(1.0, y + d) - math.copysign(1.0, y - d)) / 2.0
        a = (a0 + y - a2) * sy + a2
        sa = (math.copysign(1.0, a + d) - math.copysign(1.0, a - d)) / 2.0
        return -r * (a / d - math.copysign(1.0, a)) * sa - r * math.copysign(1.0, a)

    def update(self, v0):
        """输入目标值v0, 更新跟踪信号x1和微分x2"""
        fh = self._fhan(self.x1 - v0, self.x2, self.r, self.h)
        self.x1 += self.h * self.x2
        self.x2 += self.h * fh
        return self.x1, self.x2

    def reset(self):
        self.x1 = 0.0
        self.x2 = 0.0


class ADRC_ESOSimulator:
    """扩张状态观测器(ESO)"""

    def __init__(self, beta1=100, beta2=300, beta3=1000,
                 alpha1=0.5, alpha2=0.25, delta=0.01, b=1.0, h=0.01):
        self.beta1 = beta1
        self.beta2 = beta2
        self.beta3 = beta3
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.delta = delta
        self.b = b
        self.h = h
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0

    def update(self, y, u):
        """y=测量值, u=控制输入"""
        e = self.z1 - y
        self.z1 += self.h * (self.z2 - self.beta1 * e)
        self.z2 += self.h * (self.z3 - self.beta2 * _fal(e, self.alpha1, self.delta) + self.b * u)
        self.z3 += self.h * (-self.beta3 * _fal(e, self.alpha2, self.delta))
        return self.z1, self.z2, self.z3

    def reset(self):
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0


class ADRC_NLSEFSimulator:
    """非线性状态误差反馈(NLSEF)"""

    def __init__(self, beta0=0.5, beta1=0.1, alpha0=0.75, alpha1=1.5, delta=0.01):
        self.beta0 = beta0
        self.beta1 = beta1
        self.alpha0 = alpha0
        self.alpha1 = alpha1
        self.delta = delta

    def compute(self, e1, e2):
        """e1=位置误差, e2=速度误差"""
        u0 = self.beta0 * _fal(e1, self.alpha0, self.delta) + \
             self.beta1 * _fal(e2, self.alpha1, self.delta)
        return u0


class ADRC_Simulator:
    """完整ADRC控制器"""

    def __init__(self, h=0.01, b=1.0):
        self.h = h
        self.b = b
        self.td = ADRC_TDSimulator(r=50, h=h)
        self.eso = ADRC_ESOSimulator(h=h, b=b)
        self.nlsef = ADRC_NLSEFSimulator()
        self.output = 0.0

    def compute(self, target, measurement):
        # TD: 安排过渡过程
        v1, v2 = self.td.update(target)
        # ESO: 观测状态
        z1, z2, z3 = self.eso.update(measurement, self.output)
        # NLSEF: 误差反馈
        e1 = v1 - z1
        e2 = v2 - z2
        u0 = self.nlsef.compute(e1, e2)
        # 补偿扰动
        self.output = (u0 - z3) / self.b
        return self.output

    def reset(self):
        self.td.reset()
        self.eso.reset()
        self.output = 0.0


# ── 测试用例 ──────────────────────────────────────────────────

class TestADRC_TD(unittest.TestCase):
    """跟踪微分器测试"""

    def test_td_converges_to_target(self):
        """TD输出应逐渐趋近目标值"""
        td = ADRC_TDSimulator(r=50, h=0.001)
        target = 10.0
        for _ in range(2000):
            x1, x2 = td.update(target)
        self.assertAlmostEqual(x1, target, delta=0.5)

    def test_td_smooth_tracking(self):
        """TD应产生平滑的跟踪信号(无阶跃)"""
        td = ADRC_TDSimulator(r=50, h=0.001)
        target = 10.0
        prev_x1 = 0.0
        for _ in range(2000):
            x1, x2 = td.update(target)
            # 每步变化不应太大
            self.assertLess(abs(x1 - prev_x1), 2.0)
            prev_x1 = x1

    def test_td_provides_derivative(self):
        """TD应提供微分信号"""
        td = ADRC_TDSimulator(r=100, h=0.001)
        target = 10.0
        for _ in range(3000):
            x1, x2 = td.update(target)
        # 收敛后微分应趋近0
        self.assertAlmostEqual(x2, 0.0, delta=1.0)

    def test_td_reset(self):
        """reset应清零状态"""
        td = ADRC_TDSimulator()
        td.update(10.0)
        td.reset()
        self.assertEqual(td.x1, 0.0)
        self.assertEqual(td.x2, 0.0)


class TestADRC_ESO(unittest.TestCase):
    """扩张状态观测器测试"""

    def test_eso_tracks_state(self):
        """ESO应能跟踪系统状态"""
        eso = ADRC_ESOSimulator(beta1=100, beta2=300, beta3=500, h=0.001)
        # 模拟恒定输出
        true_y = 5.0
        for _ in range(2000):
            z1, z2, z3 = eso.update(true_y, 0.0)
        self.assertAlmostEqual(z1, true_y, delta=0.5)

    def test_eso_estimates_disturbance(self):
        """ESO应能估计恒定扰动"""
        eso = ADRC_ESOSimulator(beta1=100, beta2=300, beta3=500, h=0.001)
        # 模拟: y = u + d, d=2.0(恒定扰动)
        d = 2.0
        u = 0.0
        for _ in range(5000):
            y = u + d
            z1, z2, z3 = eso.update(y, u)
        # z3应趋近扰动d
        self.assertAlmostEqual(z3, d, delta=1.0)

    def test_eso_reset(self):
        """reset应清零状态"""
        eso = ADRC_ESOSimulator()
        eso.update(5.0, 1.0)
        eso.reset()
        self.assertEqual(eso.z1, 0.0)
        self.assertEqual(eso.z2, 0.0)
        self.assertEqual(eso.z3, 0.0)


class TestADRC_NLSEF(unittest.TestCase):
    """非线性误差反馈测试"""

    def test_zero_error_zero_output(self):
        """零误差应产生零输出"""
        nlsef = ADRC_NLSEFSimulator()
        u = nlsef.compute(0.0, 0.0)
        self.assertAlmostEqual(u, 0.0, places=5)

    def test_positive_error_positive_output(self):
        """正误差应产生正输出"""
        nlsef = ADRC_NLSEFSimulator()
        u = nlsef.compute(5.0, 0.0)
        self.assertGreater(u, 0)

    def test_negative_error_negative_output(self):
        """负误差应产生负输出"""
        nlsef = ADRC_NLSEFSimulator()
        u = nlsef.compute(-5.0, 0.0)
        self.assertLess(u, 0)


class TestADRCComplete(unittest.TestCase):
    """完整ADRC测试"""

    def test_step_response_converges(self):
        """阶跃响应应趋近目标"""
        adrc = ADRC_Simulator(h=0.001, b=1.0)
        target = 10.0
        state = 0.0
        for _ in range(5000):
            output = adrc.compute(target, state)
            state += output * 0.001  # 简单积分
        self.assertGreater(state, 5.0)

    def test_adrc_reset(self):
        """reset应清零所有状态"""
        adrc = ADRC_Simulator()
        adrc.compute(10.0, 0.0)
        adrc.reset()
        self.assertEqual(adrc.td.x1, 0.0)
        self.assertEqual(adrc.eso.z1, 0.0)
        self.assertEqual(adrc.output, 0.0)

    def test_output_bounded(self):
        """输出应有合理范围"""
        adrc = ADRC_Simulator(h=0.01, b=1.0)
        for _ in range(100):
            output = adrc.compute(100.0, 0.0)
            # ADRC输出应该在合理范围内（基于误差和控制器参数）
            self.assertGreaterEqual(output, -1000, "输出不应低于-1000")
            self.assertLessEqual(output, 1000, "输出不应超过1000")
            self.assertTrue(math.isfinite(output), "输出应该是有限值")


class TestFALFunction(unittest.TestCase):
    """fal函数测试"""

    def test_fal_linear_zone(self):
        """线性区内: fal(e, alpha, delta) ≈ e/delta^(1-alpha)"""
        result = _fal(0.001, 0.5, 0.01)
        expected = 0.001 / (0.01 ** 0.5)
        self.assertAlmostEqual(result, expected, places=5)

    def test_fal_nonlinear_zone(self):
        """非线性区: fal(e, alpha, delta) = sign(e)*|e|^alpha"""
        result = _fal(5.0, 0.5, 0.01)
        expected = math.sqrt(5.0)
        self.assertAlmostEqual(result, expected, places=5)

    def test_fal_negative(self):
        """负输入"""
        result = _fal(-5.0, 0.5, 0.01)
        self.assertAlmostEqual(result, -math.sqrt(5.0), places=5)

    def test_fal_zero(self):
        """零输入"""
        result = _fal(0.0, 0.5, 0.01)
        self.assertAlmostEqual(result, 0.0, places=5)


if __name__ == '__main__':
    unittest.main()
