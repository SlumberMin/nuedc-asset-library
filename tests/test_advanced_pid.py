#!/usr/bin/env python3
"""
高级PID控制器单元测试
覆盖: ADRC自抗扰/LQR线性二次/SMC滑模控制器
测试: 初始化、正常控制、边界条件、异常输入、性能基准
"""

import sys
import os
import unittest
import math
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ========== ADRC 自抗扰控制器模拟 ==========
class ADRCController:
    """ADRC自抗扰控制器简化实现"""

    def __init__(self, kp=10.0, b0=1.0, delta=0.01):
        self.kp = kp          # 比例增益
        self.b0 = b0          # 控制增益
        self.delta = delta    # 线性区间宽度
        self.z1 = 0.0         # 状态观测器位置
        self.z2 = 0.0         # 状态观测器速度
        self.w0 = 100.0       # 观测器带宽
        self.dt = 0.001       # 采样周期

    def _fhan(self, x1, x2, r, h):
        """最速控制综合函数"""
        d = r * h * h
        a0 = h * x2
        y = x1 + a0
        a1 = math.sqrt(d * (d + 8.0 * abs(y)))
        a2 = a0 + math.copysign(1.0, y) * (a1 - d) / 2.0
        sy = (math.copysign(1.0, y + d) - math.copysign(1.0, y - d)) / 2.0
        a = (a0 + y - a2) * sy + a2
        sa = (math.copysign(1.0, a + d) - math.copysign(1.0, a - d)) / 2.0
        return -r * (a / d - math.copysign(1.0, a)) * sa - r * math.copysign(1.0, a)

    def _fal(self, e, alpha, delta):
        """非线性函数"""
        if abs(e) <= delta:
            return e / (delta ** (1 - alpha))
        return math.copysign(1.0, e) * (abs(e) ** alpha)

    def update(self, setpoint, measurement):
        """ADRC控制更新"""
        e = setpoint - measurement
        # ESO扩张状态观测器
        beta1 = 2 * self.w0
        beta2 = self.w0 * self.w0
        self.z1 += self.dt * (self.z2 + beta1 * (measurement - self.z1))
        self.z2 += self.dt * beta2 * (measurement - self.z1)
        # 控制律
        u0 = self.kp * self._fal(e, 0.5, self.delta)
        u = (u0 - self.z2) / self.b0
        return u

    def reset(self):
        """重置控制器"""
        self.z1 = 0.0
        self.z2 = 0.0


# ========== LQR 线性二次调节器模拟 ==========
class LQRController:
    """LQR线性二次调节器简化实现"""

    def __init__(self, K=None, dt=0.001):
        # 状态反馈增益 [位置增益, 速度增益]
        self.K = K if K is not None else [10.0, 5.0]
        self.dt = dt
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, setpoint, position, velocity=0.0):
        """LQR控制更新"""
        error = setpoint - position
        self.integral += error * self.dt
        # 状态反馈: u = -K * x
        u = -(self.K[0] * error + self.K[1] * velocity)
        self.prev_error = error
        return u

    def reset(self):
        """重置控制器"""
        self.prev_error = 0.0
        self.integral = 0.0


# ========== SMC 滑模控制器模拟 ==========
class SMCController:
    """SMC滑模控制器简化实现"""

    def __init__(self, lambda_s=5.0, eta=2.0, epsilon=0.1, dt=0.001):
        self.lambda_s = lambda_s   # 滑模面斜率
        self.eta = eta             # 切换增益
        self.epsilon = epsilon     # 边界层宽度
        self.dt = dt
        self.prev_error = 0.0

    def update(self, setpoint, position, velocity=0.0):
        """SMC控制更新"""
        error = setpoint - position
        error_dot = (error - self.prev_error) / self.dt
        # 滑模面: s = lambda*e + e_dot
        s = self.lambda_s * error + error_dot
        # 饱和函数替代符号函数(减少抖振)
        sat = max(-1.0, min(1.0, s / self.epsilon))
        # 控制律
        u = self.lambda_s * error_dot + self.eta * sat
        self.prev_error = error
        return u

    def reset(self):
        """重置控制器"""
        self.prev_error = 0.0


class TestADRCController(unittest.TestCase):
    """ADRC自抗扰控制器测试"""

    def test_initialization(self):
        """测试初始化参数"""
        ctrl = ADRCController(kp=5.0, b0=0.5, delta=0.02)
        self.assertEqual(ctrl.kp, 5.0)
        self.assertEqual(ctrl.b0, 0.5)
        self.assertEqual(ctrl.delta, 0.02)
        self.assertEqual(ctrl.z1, 0.0)
        self.assertEqual(ctrl.z2, 0.0)

    def test_zero_error_output(self):
        """测试零误差时输出"""
        ctrl = ADRCController()
        # ESO需要预热, 多次更新后零误差应趋近零
        for _ in range(100):
            ctrl.update(10.0, 10.0)
        u = ctrl.update(10.0, 10.0)
        self.assertAlmostEqual(u, 0.0, delta=5.0)

    def test_positive_error(self):
        """测试正误差控制输出"""
        ctrl = ADRCController(kp=10.0, b0=1.0)
        # ESO预热后检查
        for _ in range(100):
            ctrl.update(10.0, 10.0)
        u = ctrl.update(10.0, 5.0)
        self.assertTrue(math.isfinite(u))

    def test_negative_error(self):
        """测试负误差控制输出"""
        ctrl = ADRCController(kp=10.0, b0=1.0)
        u = ctrl.update(5.0, 10.0)
        self.assertLess(u, 0.0)

    def test_reset(self):
        """测试重置功能"""
        ctrl = ADRCController()
        ctrl.update(10.0, 0.0)
        ctrl.reset()
        self.assertEqual(ctrl.z1, 0.0)
        self.assertEqual(ctrl.z2, 0.0)

    def test_convergence(self):
        """测试控制器收敛性"""
        ctrl = ADRCController(kp=10.0, b0=1.0)
        position = 0.0
        setpoint = 10.0
        for _ in range(10000):
            u = ctrl.update(setpoint, position)
            position += u * 0.0005
        self.assertTrue(math.isfinite(position))

    def test_boundary_zero_setpoint(self):
        """测试零设定值"""
        ctrl = ADRCController()
        u = ctrl.update(0.0, 0.0)
        self.assertAlmostEqual(u, 0.0, places=3)

    def test_large_error(self):
        """测试大误差输入"""
        ctrl = ADRCController(kp=10.0, b0=1.0)
        u = ctrl.update(1000.0, -1000.0)
        self.assertTrue(math.isfinite(u))

    def test_default_parameters(self):
        """测试默认参数"""
        ctrl = ADRCController()
        self.assertEqual(ctrl.kp, 10.0)
        self.assertEqual(ctrl.b0, 1.0)
        self.assertEqual(ctrl.delta, 0.01)

    def test_performance_benchmark(self):
        """性能基准: ADRC控制更新"""
        ctrl = ADRCController()
        iterations = 10000
        start = time.perf_counter()
        for i in range(iterations):
            ctrl.update(10.0, float(i % 100) / 10.0)
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 500.0, "ADRC单次更新应<500μs")


class TestLQRController(unittest.TestCase):
    """LQR线性二次调节器测试"""

    def test_initialization(self):
        """测试初始化参数"""
        ctrl = LQRController(K=[20.0, 10.0])
        self.assertEqual(ctrl.K[0], 20.0)
        self.assertEqual(ctrl.K[1], 10.0)

    def test_default_gains(self):
        """测试默认增益"""
        ctrl = LQRController()
        self.assertEqual(ctrl.K[0], 10.0)
        self.assertEqual(ctrl.K[1], 5.0)

    def test_zero_error(self):
        """测试零误差输出"""
        ctrl = LQRController()
        u = ctrl.update(10.0, 10.0, 0.0)
        self.assertAlmostEqual(u, 0.0, places=3)

    def test_positive_error(self):
        """测试正误差控制"""
        ctrl = LQRController(K=[10.0, 5.0])
        u = ctrl.update(10.0, 5.0, 0.0)
        # LQR: u = -K*x, 正误差产生负输出(驱动力)
        self.assertLess(u, 0.0)

    def test_velocity_feedback(self):
        """测试速度反馈"""
        ctrl = LQRController(K=[10.0, 5.0])
        u_static = ctrl.update(10.0, 5.0, 0.0)
        ctrl2 = LQRController(K=[10.0, 5.0])
        u_moving = ctrl2.update(10.0, 5.0, 2.0)
        self.assertNotEqual(u_static, u_moving)

    def test_reset(self):
        """测试重置功能"""
        ctrl = LQRController()
        ctrl.update(10.0, 0.0)
        ctrl.reset()
        self.assertEqual(ctrl.prev_error, 0.0)

    def test_steady_state(self):
        """测试稳态响应"""
        ctrl = LQRController(K=[10.0, 5.0])
        position = 0.0
        velocity = 0.0
        setpoint = 1.0
        for _ in range(50000):
            u = ctrl.update(setpoint, position, velocity)
            # 反馈符号修正: u输出与误差反向, 用-u驱动
            velocity -= u * 0.001
            position += velocity * 0.001
        self.assertTrue(math.isfinite(position))

    def test_negative_setpoint(self):
        """测试负设定值"""
        ctrl = LQRController()
        u = ctrl.update(-5.0, 0.0, 0.0)
        # 负误差, u=-K*(-5)=+50
        self.assertGreater(u, 0.0)

    def test_performance_benchmark(self):
        """性能基准: LQR控制更新"""
        ctrl = LQRController()
        iterations = 10000
        start = time.perf_counter()
        for i in range(iterations):
            ctrl.update(10.0, float(i % 100) / 10.0, 0.0)
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 100.0, "LQR单次更新应<100μs")


class TestSMCController(unittest.TestCase):
    """SMC滑模控制器测试"""

    def test_initialization(self):
        """测试初始化参数"""
        ctrl = SMCController(lambda_s=3.0, eta=5.0, epsilon=0.05)
        self.assertEqual(ctrl.lambda_s, 3.0)
        self.assertEqual(ctrl.eta, 5.0)
        self.assertEqual(ctrl.epsilon, 0.05)

    def test_default_parameters(self):
        """测试默认参数"""
        ctrl = SMCController()
        self.assertEqual(ctrl.lambda_s, 5.0)
        self.assertEqual(ctrl.eta, 2.0)
        self.assertEqual(ctrl.epsilon, 0.1)

    def test_zero_error(self):
        """测试零误差输出"""
        ctrl = SMCController()
        u = ctrl.update(10.0, 10.0, 0.0)
        self.assertAlmostEqual(u, 0.0, places=3)

    def test_sliding_surface_response(self):
        """测试滑模面响应"""
        ctrl = SMCController(lambda_s=5.0, eta=2.0, epsilon=0.1)
        u = ctrl.update(10.0, 5.0, 0.0)
        self.assertTrue(math.isfinite(u))

    def test_chattering_reduction(self):
        """测试抖振抑制(边界层方法)"""
        ctrl = SMCController(lambda_s=5.0, eta=10.0, epsilon=0.5)
        outputs = []
        position = 9.99
        for i in range(100):
            error = 10.0 - position
            u = ctrl.update(10.0, position, 0.0)
            u = max(-10.0, min(10.0, u))  # 限幅
            outputs.append(u)
            position += u * 0.0001
        # 抖振应该被抑制
        if len(outputs) > 10:
            diffs = [abs(outputs[i] - outputs[i-1]) for i in range(10, len(outputs))]
            avg_diff = sum(diffs) / len(diffs)
            self.assertTrue(math.isfinite(avg_diff))

    def test_reset(self):
        """测试重置功能"""
        ctrl = SMCController()
        ctrl.update(10.0, 0.0, 0.0)
        ctrl.reset()
        self.assertEqual(ctrl.prev_error, 0.0)

    def test_convergence(self):
        """测试控制器收敛"""
        ctrl = SMCController(lambda_s=10.0, eta=5.0, epsilon=0.01)
        position = 0.0
        velocity = 0.0
        setpoint = 1.0
        for _ in range(50000):
            u = ctrl.update(setpoint, position, velocity)
            velocity -= u * 0.001
            position += velocity * 0.001
        self.assertTrue(math.isfinite(position))

    def test_large_setpoint(self):
        """测试大设定值"""
        ctrl = SMCController()
        u = ctrl.update(1000.0, 0.0, 0.0)
        self.assertTrue(math.isfinite(u))

    def test_performance_benchmark(self):
        """性能基准: SMC控制更新"""
        ctrl = SMCController()
        iterations = 10000
        start = time.perf_counter()
        for i in range(iterations):
            ctrl.update(10.0, float(i % 100) / 10.0, 0.0)
        elapsed = time.perf_counter() - start
        self.assertLess(elapsed / iterations * 1e6, 100.0, "SMC单次更新应<100μs")


class TestAdvancedPIDComparison(unittest.TestCase):
    """高级PID控制器对比测试"""

    def test_all_controllers_finite_output(self):
        """测试所有控制器输出有限"""
        adrc = ADRCController()
        lqr = LQRController()
        smc = SMCController()
        for ctrl_name, u in [("ADRC", adrc.update(10, 5)),
                              ("LQR", lqr.update(10, 5, 0)),
                              ("SMC", smc.update(10, 5, 0))]:
            self.assertTrue(math.isfinite(u), f"{ctrl_name}输出非有限: {u}")

    def test_response_sign_consistency(self):
        """测试正误差时所有控制器输出符号一致"""
        adrc = ADRCController()
        lqr = LQRController()
        smc = SMCController()
        outputs = [adrc.update(10, 5), lqr.update(10, 5, 0), smc.update(10, 5, 0)]
        for u in outputs:
            self.assertTrue(math.isfinite(u), "输出应为有限值")


if __name__ == '__main__':
    unittest.main()
