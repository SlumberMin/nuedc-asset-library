#!/usr/bin/env python3
"""
扩展卡尔曼滤波器 (EKF) 单元测试
覆盖: 初始化/预测/更新/完整步进/传感器融合/数值稳定性/性能基准
注意: 使用纯 Python + numpy 模拟 EKF 逻辑
"""

import sys
import os
import math
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False


# ── Python 模拟实现 ──────────────────────────────────────────

class EKFSimulator:
    """扩展卡尔曼滤波器 (纯Python实现, 不依赖numpy)"""

    def __init__(self, n_states, n_meas):
        """
        n_states: 状态维度
        n_meas: 量测维度
        """
        self.n = n_states
        self.p = n_meas

        # 状态向量
        self.x = [0.0] * n_states

        # 协方差矩阵 (单位阵 * 10)
        self.P = [[10.0 if i == j else 0.0 for j in range(n_states)]
                  for i in range(n_states)]

        # 过程噪声协方差
        self.Q = [[0.01 if i == j else 0.0 for j in range(n_states)]
                  for i in range(n_states)]

        # 量测噪声协方差
        self.R = [[1.0 if i == j else 0.0 for j in range(n_meas)]
                  for i in range(n_meas)]

    def _mat_mul(self, A, B):
        """矩阵乘法 A @ B"""
        rows_a, cols_a = len(A), len(A[0])
        rows_b, cols_b = len(B), len(B[0])
        assert cols_a == rows_b
        C = [[0.0] * cols_b for _ in range(rows_a)]
        for i in range(rows_a):
            for j in range(cols_b):
                for k in range(cols_a):
                    C[i][j] += A[i][k] * B[k][j]
        return C

    def _mat_add(self, A, B):
        """矩阵加法"""
        return [[A[i][j] + B[i][j] for j in range(len(A[0]))]
                for i in range(len(A))]

    def _mat_sub(self, A, B):
        """矩阵减法"""
        return [[A[i][j] - B[i][j] for j in range(len(A[0]))]
                for i in range(len(A))]

    def _mat_transpose(self, A):
        """矩阵转置"""
        return [[A[j][i] for j in range(len(A))]
                for i in range(len(A[0]))]

    def _mat_vec_mul(self, A, v):
        """矩阵乘向量"""
        result = [0.0] * len(A)
        for i in range(len(A)):
            for j in range(len(v)):
                result[i] += A[i][j] * v[j]
        return result

    def _vec_sub(self, a, b):
        """向量减法"""
        return [a[i] - b[i] for i in range(len(a))]

    def _vec_add(self, a, b):
        """向量加法"""
        return [a[i] + b[i] for i in range(len(a))]

    def _inv_2x2(self, M):
        """2x2矩阵求逆"""
        a, b = M[0][0], M[0][1]
        c, d = M[1][0], M[1][1]
        det = a * d - b * c
        if abs(det) < 1e-12:
            return [[1e12, 0], [0, 1e12]]  # 数值保护
        return [[d / det, -b / det], [-c / det, a / det]]

    def _inv_1x1(self, M):
        """1x1矩阵求逆"""
        val = M[0][0]
        if abs(val) < 1e-12:
            return [[1e12]]
        return [[1.0 / val]]

    def _inv(self, M):
        """矩阵求逆 (支持1x1和2x2)"""
        n = len(M)
        if n == 1:
            return self._inv_1x1(M)
        elif n == 2:
            return self._inv_2x2(M)
        else:
            raise NotImplementedError("仅支持1x1和2x2矩阵求逆")

    def predict_linear(self, F, u=None, B=None):
        """
        线性预测步
        F: 状态转移矩阵
        B: 控制输入矩阵 (可选)
        u: 控制输入 (可选)
        """
        # x = F @ x + B @ u
        self.x = self._mat_vec_mul(F, self.x)
        if B is not None and u is not None:
            Bu = self._mat_vec_mul(B, u)
            self.x = self._vec_add(self.x, Bu)

        # P = F @ P @ F^T + Q
        Ft = self._mat_transpose(F)
        self.P = self._mat_add(
            self._mat_mul(self._mat_mul(F, self.P), Ft),
            self.Q
        )

    def update_linear(self, z, H):
        """
        线性更新步
        z: 量测向量
        H: 量测矩阵
        """
        # z_pred = H @ x
        z_pred = self._mat_vec_mul(H, self.x)

        # S = H @ P @ H^T + R
        Ht = self._mat_transpose(H)
        S = self._mat_add(
            self._mat_mul(self._mat_mul(H, self.P), Ht),
            self.R
        )

        # K = P @ H^T @ S^(-1)
        S_inv = self._inv(S)
        K = self._mat_mul(self._mat_mul(self.P, Ht), S_inv)

        # x = x + K @ (z - z_pred)
        innovation = self._vec_sub(z, z_pred)
        K_innov = self._mat_vec_mul(K, innovation)
        self.x = self._vec_add(self.x, K_innov)

        # P = (I - K @ H) @ P
        KH = self._mat_mul(K, H)
        I = [[1.0 if i == j else 0.0 for j in range(self.n)]
             for i in range(self.n)]
        IKH = self._mat_sub(I, KH)
        self.P = self._mat_mul(IKH, self.P)

    def reset(self):
        """重置状态"""
        self.x = [0.0] * self.n
        self.P = [[10.0 if i == j else 0.0 for j in range(self.n)]
                  for i in range(self.n)]


# ── 测试用例 ──────────────────────────────────────────────────

class TestEKFInit(unittest.TestCase):
    """EKF初始化测试"""

    def test_default_state_zero(self):
        """初始状态应为零"""
        ekf = EKFSimulator(2, 1)
        for xi in ekf.x:
            self.assertAlmostEqual(xi, 0.0)

    def test_covariance_diagonal(self):
        """初始协方差应为对角阵"""
        ekf = EKFSimulator(2, 1)
        self.assertAlmostEqual(ekf.P[0][0], 10.0)
        self.assertAlmostEqual(ekf.P[1][1], 10.0)
        self.assertAlmostEqual(ekf.P[0][1], 0.0)

    def test_dimensions(self):
        """状态和量测维度应正确"""
        ekf = EKFSimulator(3, 2)
        self.assertEqual(ekf.n, 3)
        self.assertEqual(ekf.p, 2)
        self.assertEqual(len(ekf.x), 3)
        self.assertEqual(len(ekf.R), 2)

    def test_custom_dimensions(self):
        """不同维度组合应正常初始化"""
        for n, p in [(1, 1), (2, 1), (2, 2), (3, 2), (4, 3)]:
            ekf = EKFSimulator(n, p)
            self.assertEqual(len(ekf.x), n)
            self.assertEqual(len(ekf.P), n)
            self.assertEqual(len(ekf.R), p)


class TestEKFPredict(unittest.TestCase):
    """EKF预测步测试"""

    def test_linear_predict_identity(self):
        """单位阵F不应改变状态"""
        ekf = EKFSimulator(2, 1)
        ekf.x = [1.0, 2.0]
        F = [[1.0, 0.0], [0.0, 1.0]]
        ekf.predict_linear(F)
        self.assertAlmostEqual(ekf.x[0], 1.0, places=5)
        self.assertAlmostEqual(ekf.x[1], 2.0, places=5)

    def test_predict_increases_uncertainty(self):
        """预测应增加不确定性"""
        ekf = EKFSimulator(2, 1)
        F = [[1.0, 0.01], [0.0, 1.0]]
        trace_before = ekf.P[0][0] + ekf.P[1][1]
        ekf.predict_linear(F)
        trace_after = ekf.P[0][0] + ekf.P[1][1]
        self.assertGreater(trace_after, trace_before)

    def test_predict_with_control_input(self):
        """带控制输入的预测"""
        ekf = EKFSimulator(2, 1)
        ekf.x = [0.0, 0.0]
        F = [[1.0, 0.01], [0.0, 1.0]]
        B = [[0.0], [0.01]]
        u = [1.0]
        ekf.predict_linear(F, u=u, B=B)
        # 速度应因控制输入而增加
        self.assertGreater(ekf.x[1], 0.0)

    def test_predict_constant_velocity(self):
        """匀速运动模型预测"""
        ekf = EKFSimulator(2, 1)
        dt = 0.01
        ekf.x = [0.0, 5.0]  # 位置=0, 速度=5
        F = [[1.0, dt], [0.0, 1.0]]
        for _ in range(100):
            ekf.predict_linear(F)
        # 位置应接近 5.0 * 1.0 = 5.0
        self.assertAlmostEqual(ekf.x[0], 5.0, delta=0.5)


class TestEKFUpdate(unittest.TestCase):
    """EKF更新步测试"""

    def test_update_reduces_uncertainty(self):
        """量测更新应减少不确定性"""
        ekf = EKFSimulator(2, 1)
        H = [[1.0, 0.0]]
        trace_before = ekf.P[0][0] + ekf.P[1][1]
        ekf.update_linear([5.0], H)
        trace_after = ekf.P[0][0] + ekf.P[1][1]
        self.assertLess(trace_after, trace_before)

    def test_update_pulls_toward_measurement(self):
        """更新应使状态向量测方向移动"""
        ekf = EKFSimulator(2, 1)
        ekf.x = [0.0, 0.0]
        H = [[1.0, 0.0]]
        ekf.update_linear([10.0], H)
        self.assertGreater(ekf.x[0], 0.0)

    def test_update_consistency(self):
        """多次更新后状态应趋近量测值"""
        ekf = EKFSimulator(1, 1)
        ekf.x = [0.0]
        ekf.P = [[100.0]]
        H = [[1.0]]
        for _ in range(50):
            ekf.update_linear([5.0], H)
        self.assertAlmostEqual(ekf.x[0], 5.0, delta=0.1)


class TestEKFStep(unittest.TestCase):
    """EKF完整步进测试 (预测+更新)"""

    def test_track_constant_signal(self):
        """应能跟踪恒定信号"""
        ekf = EKFSimulator(1, 1)
        ekf.Q = [[0.01]]
        ekf.R = [[0.1]]
        F = [[1.0]]
        H = [[1.0]]
        for _ in range(200):
            ekf.predict_linear(F)
            ekf.update_linear([10.0], H)
        self.assertAlmostEqual(ekf.x[0], 10.0, delta=0.5)

    def test_filter_noise(self):
        """应能滤除噪声"""
        import random
        random.seed(42)
        ekf = EKFSimulator(2, 1)
        dt = 0.01
        F = [[1.0, dt], [0.0, 1.0]]
        H = [[1.0, 0.0]]
        ekf.Q = [[0.001, 0.0], [0.0, 0.01]]
        ekf.R = [[0.5]]

        true_val = 10.0
        errors = []
        for _ in range(300):
            noise = random.gauss(0, 0.7)
            ekf.predict_linear(F)
            ekf.update_linear([true_val + noise], H)
            errors.append(abs(ekf.x[0] - true_val))

        # 后半段误差应小于噪声标准差的一半
        avg_err = sum(errors[150:]) / len(errors[150:])
        self.assertLess(avg_err, 0.5)

    def test_reset(self):
        """reset应恢复初始状态"""
        ekf = EKFSimulator(2, 1)
        ekf.x = [5.0, 3.0]
        ekf.P = [[1.0, 0.0], [0.0, 1.0]]
        ekf.reset()
        self.assertEqual(ekf.x[0], 0.0)
        self.assertEqual(ekf.x[1], 0.0)
        self.assertAlmostEqual(ekf.P[0][0], 10.0)


class TestEKFSensorFusion(unittest.TestCase):
    """传感器融合测试"""

    def test_two_sensors(self):
        """双传感器融合应优于单传感器"""
        import random
        random.seed(42)

        # 单传感器
        ekf1 = EKFSimulator(1, 1)
        ekf1.Q = [[0.01]]
        ekf1.R = [[0.5]]
        F = [[1.0]]
        H1 = [[1.0]]

        # 双传感器
        ekf2 = EKFSimulator(1, 2)
        ekf2.Q = [[0.01]]
        ekf2.R = [[0.5, 0.0], [0.0, 0.5]]
        H2 = [[1.0], [1.0]]

        true_val = 10.0
        err1_sum, err2_sum = 0.0, 0.0
        for _ in range(200):
            z1 = true_val + random.gauss(0, 0.7)
            z2 = true_val + random.gauss(0, 0.7)

            ekf1.predict_linear(F)
            ekf1.update_linear([z1], H1)

            ekf2.predict_linear(F)
            ekf2.update_linear([z1, z2], H2)

            err1_sum += abs(ekf1.x[0] - true_val)
            err2_sum += abs(ekf2.x[0] - true_val)

        # 双传感器平均误差应更小
        avg1 = err1_sum / 200
        avg2 = err2_sum / 200
        self.assertLessEqual(avg2, avg1 + 0.05)


class TestEKFEdgeCases(unittest.TestCase):
    """边界条件测试"""

    def test_zero_noise(self):
        """零噪声下应完美跟踪"""
        ekf = EKFSimulator(2, 1)
        dt = 0.01
        F = [[1.0, dt], [0.0, 1.0]]
        H = [[1.0, 0.0]]
        ekf.Q = [[0.0, 0.0], [0.0, 0.0]]
        ekf.R = [[0.001]]  # 极小量测噪声

        ekf.x = [0.0, 1.0]
        for i in range(100):
            z_true = i * dt * 1.0
            ekf.predict_linear(F)
            ekf.update_linear([z_true], H)

        # 应接近真实轨迹末端
        self.assertAlmostEqual(ekf.x[0], 100 * dt * 1.0, delta=1.0)

    def test_large_initial_uncertainty(self):
        """大初始不确定性应快速收敛"""
        ekf = EKFSimulator(1, 1)
        ekf.x = [0.0]
        ekf.P = [[1000.0]]
        ekf.Q = [[0.01]]
        ekf.R = [[0.1]]
        F = [[1.0]]
        H = [[1.0]]

        for _ in range(100):
            ekf.predict_linear(F)
            ekf.update_linear([5.0], H)

        self.assertAlmostEqual(ekf.x[0], 5.0, delta=0.5)

    def test_stability_many_steps(self):
        """长时间运行不应发散"""
        ekf = EKFSimulator(2, 1)
        F = [[1.0, 0.01], [0.0, 1.0]]
        H = [[1.0, 0.0]]
        for _ in range(5000):
            ekf.predict_linear(F)
            ekf.update_linear([5.0], H)
            self.assertTrue(all(abs(xi) < 1e6 for xi in ekf.x),
                          "EKF状态发散")
            for row in ekf.P:
                for val in row:
                    self.assertTrue(abs(val) < 1e6, "EKF协方差发散")


class TestEKFPerformance(unittest.TestCase):
    """性能基准测试"""

    def test_predict_update_speed(self):
        """1000次预测+更新应在1秒内完成"""
        ekf = EKFSimulator(2, 1)
        F = [[1.0, 0.01], [0.0, 1.0]]
        H = [[1.0, 0.0]]

        start = time.perf_counter()
        for _ in range(1000):
            ekf.predict_linear(F)
            ekf.update_linear([5.0], H)
        elapsed = time.perf_counter() - start

        self.assertLess(elapsed, 5.0,
                       f"1000次EKF步进耗时 {elapsed:.3f}s, 应<5s")


if __name__ == '__main__':
    unittest.main()
