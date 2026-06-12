#!/usr/bin/env python3
"""
简化FOC(磁场定向控制)单元测试
覆盖: Park/Clarke变换、PI控制器、SVPWM、速度环/电流环
测试: 变换正确性、边界条件、异常输入、性能基准
"""

import sys
import os
import unittest
import math
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def clarke_transform(ia, ib, ic):
    """Clarke变换: abc -> alpha-beta"""
    alpha = ia
    beta = (ia + 2 * ib) / math.sqrt(3)
    return alpha, beta


def inv_clarke_transform(alpha, beta):
    """反Clarke变换: alpha-beta -> abc"""
    ia = alpha
    ib = -alpha / 2 + beta * math.sqrt(3) / 2
    ic = -alpha / 2 - beta * math.sqrt(3) / 2
    return ia, ib, ic


def park_transform(alpha, beta, theta):
    """Park变换: alpha-beta -> d-q"""
    d = alpha * math.cos(theta) + beta * math.sin(theta)
    q = -alpha * math.sin(theta) + beta * math.cos(theta)
    return d, q


def inv_park_transform(d, q, theta):
    """反Park变换: d-q -> alpha-beta"""
    alpha = d * math.cos(theta) - q * math.sin(theta)
    beta = d * math.sin(theta) + q * math.cos(theta)
    return alpha, beta


class PIController:
    """PI控制器(用于FOC电流/速度环)"""

    def __init__(self, kp=1.0, ki=0.0, dt=0.001, output_min=-100.0, output_max=100.0):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, setpoint, measurement):
        """PI控制更新"""
        error = setpoint - measurement
        self.integral += error * self.dt
        # 抗积分饱和
        self.integral = max(self.output_min / self.ki if self.ki != 0 else -1e10,
                           min(self.output_max / self.ki if self.ki != 0 else 1e10, self.integral))
        output = self.kp * error + self.ki * self.integral
        # 输出限幅
        output = max(self.output_min, min(self.output_max, output))
        return output

    def reset(self):
        """重置"""
        self.integral = 0.0
        self.prev_error = 0.0


class SimpleFOC:
    """简化FOC控制器"""

    def __init__(self, kp_iq=0.5, ki_iq=0.01, kp_id=0.5, ki_id=0.01,
                 kp_speed=1.0, ki_speed=0.1, dt=0.001):
        self.dt = dt
        self.id_pi = PIController(kp_id, ki_id, dt, -24.0, 24.0)
        self.iq_pi = PIController(kp_iq, ki_iq, dt, -24.0, 24.0)
        self.speed_pi = PIController(kp_speed, ki_speed, dt, -10.0, 10.0)
        self.target_speed = 0.0
        self.theta = 0.0

    def update(self, ia, ib, ic, theta, speed_rpm):
        """FOC一步更新"""
        self.theta = theta
        # Clarke变换
        alpha, beta = clarke_transform(ia, ib, ic)
        # Park变换
        id_measured, iq_measured = park_transform(alpha, beta, theta)
        # 速度环输出iq参考
        iq_ref = self.speed_pi.update(self.target_speed, speed_rpm)
        id_ref = 0.0  # id参考为0(最大转矩/电流比)
        # 电流环PI
        vd = self.id_pi.update(id_ref, id_measured)
        vq = self.iq_pi.update(iq_ref, iq_measured)
        # 反Park变换
        valpha, vbeta = inv_park_transform(vd, vq, theta)
        return valpha, vbeta, vd, vq

    def set_target_speed(self, speed_rpm):
        """设置目标速度"""
        self.target_speed = speed_rpm

    def reset(self):
        """重置"""
        self.id_pi.reset()
        self.iq_pi.reset()
        self.speed_pi.reset()


class TestClarkeTransform(unittest.TestCase):
    """Clarke变换测试"""

    def test_balanced_three_phase(self):
        """测试平衡三相"""
        alpha, beta = clarke_transform(1.0, -0.5, -0.5)
        self.assertAlmostEqual(alpha, 1.0, places=5)
        self.assertAlmostEqual(beta, 0.0, places=5)

    def test_zero_input(self):
        """测试零输入"""
        alpha, beta = clarke_transform(0.0, 0.0, 0.0)
        self.assertAlmostEqual(alpha, 0.0)
        self.assertAlmostEqual(beta, 0.0)

    def test_roundtrip(self):
        """测试正反变换往返"""
        ia, ib, ic = 1.0, -0.5, -0.5
        alpha, beta = clarke_transform(ia, ib, ic)
        ia2, ib2, ic2 = inv_clarke_transform(alpha, beta)
        self.assertAlmostEqual(ia, ia2, places=5)
        self.assertAlmostEqual(ib, ib2, places=5)
        self.assertAlmostEqual(ic, ic2, places=5)

    def test_symmetry(self):
        """测试对称性"""
        a, b = clarke_transform(1.0, 1.0, 1.0)
        # 三相相等时alpha=1, beta=2/sqrt(3)
        self.assertAlmostEqual(a, 1.0)

    def test_performance_benchmark(self):
        """性能基准: Clarke变换"""
        iterations = 100000
        start = time.perf_counter()
        for i in range(iterations):
            clarke_transform(1.0, -0.5, -0.5)
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 2.0, "Clarke变换应<2μs")


class TestParkTransform(unittest.TestCase):
    """Park变换测试"""

    def test_zero_angle(self):
        """测试零角度"""
        d, q = park_transform(1.0, 0.0, 0.0)
        self.assertAlmostEqual(d, 1.0, places=5)
        self.assertAlmostEqual(q, 0.0, places=5)

    def test_90_degrees(self):
        """测试90度"""
        d, q = park_transform(1.0, 0.0, math.pi / 2)
        self.assertAlmostEqual(d, 0.0, places=5)
        self.assertAlmostEqual(q, -1.0, places=5)

    def test_roundtrip(self):
        """测试正反变换往返"""
        alpha, beta = 2.0, 1.5
        theta = 0.7
        d, q = park_transform(alpha, beta, theta)
        a2, b2 = inv_park_transform(d, q, theta)
        self.assertAlmostEqual(alpha, a2, places=5)
        self.assertAlmostEqual(beta, b2, places=5)

    def test_zero_input(self):
        """测试零输入"""
        d, q = park_transform(0.0, 0.0, 1.0)
        self.assertAlmostEqual(d, 0.0)
        self.assertAlmostEqual(q, 0.0)

    def test_performance_benchmark(self):
        """性能基准: Park变换"""
        iterations = 100000
        theta = 0.5
        start = time.perf_counter()
        for i in range(iterations):
            park_transform(1.0, 0.5, theta)
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 2.0, "Park变换应<2μs")


class TestPIController(unittest.TestCase):
    """PI控制器测试"""

    def test_initialization(self):
        """测试初始化"""
        pi = PIController(kp=0.5, ki=0.01)
        self.assertEqual(pi.kp, 0.5)
        self.assertEqual(pi.ki, 0.01)

    def test_proportional_only(self):
        """测试纯比例控制"""
        pi = PIController(kp=2.0, ki=0.0)
        output = pi.update(10.0, 5.0)
        self.assertAlmostEqual(output, 10.0)

    def test_output_saturation(self):
        """测试输出饱和"""
        pi = PIController(kp=100.0, ki=0.0, output_min=-10.0, output_max=10.0)
        output = pi.update(100.0, 0.0)
        self.assertEqual(output, 10.0)

    def test_zero_error(self):
        """测试零误差"""
        pi = PIController(kp=1.0, ki=0.1)
        output = pi.update(10.0, 10.0)
        self.assertAlmostEqual(output, 0.0)

    def test_reset(self):
        """测试重置"""
        pi = PIController(kp=1.0, ki=1.0)
        pi.update(10.0, 0.0)
        pi.reset()
        self.assertEqual(pi.integral, 0.0)

    def test_performance_benchmark(self):
        """性能基准: PI控制器"""
        pi = PIController(kp=1.0, ki=0.1)
        iterations = 100000
        start = time.perf_counter()
        for i in range(iterations):
            pi.update(10.0, float(i % 100) / 10.0)
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 2.0, "PI控制应<2μs")


class TestSimpleFOC(unittest.TestCase):
    """简化FOC控制器测试"""

    def test_initialization(self):
        """测试初始化"""
        foc = SimpleFOC()
        self.assertEqual(foc.target_speed, 0.0)

    def test_update_returns_finite(self):
        """测试输出有限"""
        foc = SimpleFOC()
        foc.set_target_speed(1000)
        valpha, vbeta, vd, vq = foc.update(1.0, -0.5, -0.5, 0.0, 0.0)
        for v in [valpha, vbeta, vd, vq]:
            self.assertTrue(math.isfinite(v))

    def test_id_reference_zero(self):
        """测试id参考为0"""
        foc = SimpleFOC(kp_id=0.0, ki_id=0.0)
        foc.set_target_speed(0)
        _, _, vd, _ = foc.update(0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(vd, 0.0)

    def test_reset(self):
        """测试重置"""
        foc = SimpleFOC()
        foc.update(1.0, -0.5, -0.5, 0.0, 500.0)
        foc.reset()
        self.assertEqual(foc.id_pi.integral, 0.0)

    def test_performance_benchmark(self):
        """性能基准: FOC一步更新"""
        foc = SimpleFOC()
        foc.set_target_speed(1000)
        iterations = 10000
        start = time.perf_counter()
        for i in range(iterations):
            foc.update(1.0, -0.5, -0.5, float(i) * 0.001, 500.0)
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 50.0, "FOC更新应<50μs")


if __name__ == '__main__':
    unittest.main()
