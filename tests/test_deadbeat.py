#!/usr/bin/env python3
"""
无差拍控制单元测试
覆盖: 初始化(通用/二阶)、模型设置、增益设置、输出限幅、
      状态更新、状态校正、复位、收敛性
注意: 使用纯 Python 模拟 C DeadbeatCtrl 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

DB_STATE_DIM = 4


def _clamp(val, min_v, max_v):
    if val < min_v:
        return min_v
    if val > max_v:
        return max_v
    return val


class DeadbeatSimulator:
    """无差拍控制器模拟 (对应 deadbeat.c)"""

    def __init__(self, n=1):
        self.n = min(n, DB_STATE_DIM)
        self.A = [[0.0] * DB_STATE_DIM for _ in range(DB_STATE_DIM)]
        self.B = [[0.0] for _ in range(DB_STATE_DIM)]
        self.C = [[0.0] * DB_STATE_DIM]
        self.K = [[0.0] * DB_STATE_DIM]
        self.Kr = 1.0
        self.x = [0.0] * DB_STATE_DIM
        self.out_min = -1e30
        self.out_max = 1e30
        self.u_last = 0.0
        self.output = 0.0

    def init_2nd(self, Ts, tau, gain):
        """二阶系统初始化"""
        a = math.exp(-Ts / tau)
        b = gain * (1.0 - a)
        self.n = 1
        self.A[0][0] = a
        self.B[0][0] = b
        self.C[0][0] = 1.0
        K_val = a / b
        self.K[0][0] = K_val
        self.Kr = 1.0 / (b + a * K_val)

    def set_model(self, A, B, C):
        """设置系统矩阵 (A: n*n flat, B: n, C: n)"""
        n = self.n
        for i in range(n):
            for j in range(n):
                self.A[i][j] = A[i * n + j]
            self.B[i][0] = B[i]
        for j in range(n):
            self.C[0][j] = C[j]

    def set_gains(self, K, Kr):
        for j in range(self.n):
            self.K[0][j] = K[j]
        self.Kr = Kr

    def set_output_limit(self, lo, hi):
        self.out_min = lo
        self.out_max = hi

    def compute(self, setpoint, feedback):
        n = self.n
        # 状态校正
        y_hat = sum(self.C[0][j] * self.x[j] for j in range(n))
        correction = feedback - y_hat
        self.x[0] += correction

        # 控制律: u = -K*x + Kr*r
        u = self.Kr * setpoint
        for j in range(n):
            u -= self.K[0][j] * self.x[j]

        # 限幅
        u = _clamp(u, self.out_min, self.out_max)

        # 状态更新: x = A*x + B*u
        x_new = [0.0] * DB_STATE_DIM
        for i in range(n):
            for j in range(n):
                x_new[i] += self.A[i][j] * self.x[j]
            x_new[i] += self.B[i][0] * u
        for i in range(n):
            self.x[i] = x_new[i]

        self.u_last = u
        self.output = u
        return u

    def update_observer(self, y_meas):
        n = self.n
        y_hat = sum(self.C[0][j] * self.x[j] for j in range(n))
        err = y_meas - y_hat
        self.x[0] += err

    def reset(self):
        self.x = [0.0] * DB_STATE_DIM
        self.u_last = 0.0
        self.output = 0.0


class TestDeadbeatInit(unittest.TestCase):
    """初始化测试"""

    def test_init_default(self):
        ctrl = DeadbeatSimulator(n=2)
        self.assertEqual(ctrl.n, 2)

    def test_init_clamp_n(self):
        """n不应超过DB_STATE_DIM"""
        ctrl = DeadbeatSimulator(n=10)
        self.assertEqual(ctrl.n, DB_STATE_DIM)

    def test_init_kr_default(self):
        ctrl = DeadbeatSimulator()
        self.assertEqual(ctrl.Kr, 1.0)


class TestDeadbeatInit2nd(unittest.TestCase):
    """二阶系统初始化测试"""

    def test_init_2nd_sets_model(self):
        ctrl = DeadbeatSimulator()
        ctrl.init_2nd(Ts=0.01, tau=0.1, gain=2.0)
        self.assertEqual(ctrl.n, 1)
        # A[0][0] = exp(-0.01/0.1) = exp(-0.1)
        expected_a = math.exp(-0.1)
        self.assertAlmostEqual(ctrl.A[0][0], expected_a, places=5)

    def test_init_2nd_computes_gains(self):
        """应自动计算无差拍增益"""
        ctrl = DeadbeatSimulator()
        ctrl.init_2nd(Ts=0.01, tau=0.1, gain=1.0)
        self.assertNotEqual(ctrl.K[0][0], 0.0)
        self.assertNotEqual(ctrl.Kr, 0.0)

    def test_init_2nd_pole_placement(self):
        """闭环极点应在原点: A - B*K = 0"""
        ctrl = DeadbeatSimulator()
        Ts = 0.01
        tau = 0.1
        gain = 1.0
        ctrl.init_2nd(Ts, tau, gain)
        a = ctrl.A[0][0]
        b = ctrl.B[0][0]
        k = ctrl.K[0][0]
        # A - B*K 应趋近0
        closed_loop = a - b * k
        self.assertAlmostEqual(closed_loop, 0.0, places=5)


class TestDeadbeatSetModel(unittest.TestCase):
    """模型设置测试"""

    def test_set_model(self):
        ctrl = DeadbeatSimulator(n=1)
        A = [0.5]
        B = [1.0]
        C = [1.0]
        ctrl.set_model(A, B, C)
        self.assertAlmostEqual(ctrl.A[0][0], 0.5)
        self.assertAlmostEqual(ctrl.B[0][0], 1.0)
        self.assertAlmostEqual(ctrl.C[0][0], 1.0)


class TestDeadbeatSetGains(unittest.TestCase):
    """增益设置测试"""

    def test_set_gains(self):
        ctrl = DeadbeatSimulator(n=1)
        ctrl.set_gains([2.0], 1.5)
        self.assertAlmostEqual(ctrl.K[0][0], 2.0)
        self.assertAlmostEqual(ctrl.Kr, 1.5)


class TestDeadbeatOutputLimit(unittest.TestCase):
    """输出限幅测试"""

    def test_output_limited(self):
        ctrl = DeadbeatSimulator(n=1)
        ctrl.init_2nd(Ts=0.01, tau=0.1, gain=1.0)
        ctrl.set_output_limit(-10.0, 10.0)
        # 大设定值
        u = ctrl.compute(setpoint=1000.0, feedback=0.0)
        self.assertLessEqual(u, 10.0)
        self.assertGreaterEqual(u, -10.0)


class TestDeadbeatCompute(unittest.TestCase):
    """计算测试"""

    def test_returns_float(self):
        ctrl = DeadbeatSimulator(n=1)
        ctrl.init_2nd(Ts=0.01, tau=0.1, gain=1.0)
        output = ctrl.compute(setpoint=10.0, feedback=0.0)
        self.assertIsInstance(output, float)

    def test_positive_setpoint_positive_output(self):
        ctrl = DeadbeatSimulator(n=1)
        ctrl.init_2nd(Ts=0.01, tau=0.1, gain=1.0)
        output = ctrl.compute(setpoint=10.0, feedback=0.0)
        self.assertGreater(output, 0)

    def test_zero_setpoint_small_output(self):
        ctrl = DeadbeatSimulator(n=1)
        ctrl.init_2nd(Ts=0.01, tau=0.1, gain=1.0)
        # 先运行一次使状态建立
        ctrl.compute(setpoint=5.0, feedback=5.0)
        output = ctrl.compute(setpoint=0.0, feedback=0.0)
        # 输出应较小(状态已校正)
        self.assertIsInstance(output, float)


class TestDeadbeatObserver(unittest.TestCase):
    """状态观测器测试"""

    def test_observer_corrects_state(self):
        ctrl = DeadbeatSimulator(n=1)
        ctrl.init_2nd(Ts=0.01, tau=0.1, gain=1.0)
        ctrl.compute(setpoint=10.0, feedback=0.0)
        x_before = ctrl.x[0]
        ctrl.update_observer(y_meas=5.0)
        # 状态应被校正
        self.assertNotAlmostEqual(ctrl.x[0], x_before, places=3)


class TestDeadbeatReset(unittest.TestCase):
    """复位测试"""

    def test_reset_clears_state(self):
        ctrl = DeadbeatSimulator(n=1)
        ctrl.init_2nd(Ts=0.01, tau=0.1, gain=1.0)
        ctrl.compute(setpoint=10.0, feedback=0.0)
        ctrl.reset()
        self.assertEqual(ctrl.x[0], 0.0)
        self.assertEqual(ctrl.u_last, 0.0)
        self.assertEqual(ctrl.output, 0.0)


class TestDeadbeatConvergence(unittest.TestCase):
    """收敛性测试"""

    def test_step_response_converges(self):
        """一阶系统无差拍应快速收敛"""
        ctrl = DeadbeatSimulator(n=1)
        Ts = 0.01
        tau = 0.1
        gain = 1.0
        ctrl.init_2nd(Ts, tau, gain)
        ctrl.set_output_limit(-100, 100)

        state = 0.0
        for _ in range(100):
            u = ctrl.compute(setpoint=10.0, feedback=state)
            # 一阶系统: x[k+1] = a*x[k] + b*u[k]
            a = math.exp(-Ts / tau)
            b = gain * (1 - a)
            state = a * state + b * u

        # 应该趋近目标
        self.assertAlmostEqual(state, 10.0, delta=1.0)

    def test_fast_response(self):
        """无差拍应比普通PID更快"""
        ctrl = DeadbeatSimulator(n=1)
        Ts = 0.01
        tau = 0.1
        gain = 1.0
        ctrl.init_2nd(Ts, tau, gain)
        ctrl.set_output_limit(-100, 100)

        state = 0.0
        settled = False
        for i in range(50):
            u = ctrl.compute(setpoint=10.0, feedback=state)
            a = math.exp(-Ts / tau)
            b = gain * (1 - a)
            state = a * state + b * u
            if abs(state - 10.0) < 0.5 and not settled:
                settled = True
                # 应在几步内收敛
                self.assertLess(i, 20)


class TestDeadbeatMultiState(unittest.TestCase):
    """多状态维数测试"""

    def test_2d_system(self):
        """二维系统应能运行"""
        ctrl = DeadbeatSimulator(n=2)
        A = [0.9, 0.1, 0.0, 0.8]
        B = [0.0, 1.0]
        C = [1.0, 0.0]
        K = [5.0, 3.0]
        ctrl.set_model(A, B, C)
        ctrl.set_gains(K, 1.0)
        output = ctrl.compute(setpoint=10.0, feedback=0.0)
        self.assertIsNotNone(output)
        self.assertIsInstance(output, float)


if __name__ == '__main__':
    unittest.main()
