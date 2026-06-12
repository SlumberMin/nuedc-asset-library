#!/usr/bin/env python3
"""
解耦控制单元测试
覆盖: 前馈解耦、反馈解耦、混合解耦、耦合系数矩阵、
      滤波平滑、多通道、重置状态
测试对象: 11_控制算法库/common/decoupling.c (C实现的Python仿真验证)
"""

import sys
import os
import unittest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class DecouplingSim:
    """解耦控制器Python仿真 (对应decoupling.c实现)"""

    MAX_CHANNELS = 4

    def __init__(self, n_channels=2, type_='feedforward'):
        self.n = n_channels
        self.type = type_
        self.K = np.zeros((self.MAX_CHANNELS, self.MAX_CHANNELS))
        self.D = np.zeros((self.MAX_CHANNELS, self.MAX_CHANNELS))
        self.Kf = np.zeros((self.MAX_CHANNELS, self.MAX_CHANNELS))
        self.filter_alpha = 0.9
        self.decouple_out = np.zeros(self.MAX_CHANNELS)
        self.prev_out = np.zeros(self.MAX_CHANNELS)

    def set_coupling_matrix(self, K_data):
        n = self.n
        self.K[:n, :n] = np.array(K_data).reshape(n, n)

    def set_feedback_matrix(self, D_data):
        n = self.n
        self.D[:n, :n] = np.array(D_data).reshape(n, n)

    def set_feedforward_matrix(self, Kf_data):
        n = self.n
        self.Kf[:n, :n] = np.array(Kf_data).reshape(n, n)

    def set_filter(self, alpha):
        self.filter_alpha = np.clip(alpha, 0.0, 1.0)

    def feedforward_update(self, ref):
        n = self.n
        r = np.array(ref[:n])
        # 前馈解耦: out_i = -sum_j(Kf[i,j] * r_j), i!=j
        out = np.zeros(n)
        for i in range(n):
            for j in range(n):
                if i != j:
                    out[i] -= self.Kf[i, j] * r[j]
        # 滤波
        self.decouple_out[:n] = (self.filter_alpha * self.prev_out[:n]
                                 + (1 - self.filter_alpha) * out)
        self.prev_out[:n] = self.decouple_out[:n]
        return self.decouple_out[:n].copy()

    def feedback_update(self, feedback):
        n = self.n
        fb = np.array(feedback[:n])
        # 反馈解耦: out_i = -sum_j(D[i,j] * fb_j)
        out = -self.D[:n, :n] @ fb
        self.decouple_out[:n] = (self.filter_alpha * self.prev_out[:n]
                                 + (1 - self.filter_alpha) * out)
        self.prev_out[:n] = self.decouple_out[:n]
        return self.decouple_out[:n].copy()

    def hybrid_update(self, ref, feedback):
        ff = self.feedforward_update(ref)
        fb = self.feedback_update(feedback)
        n = self.n
        result = ff + fb
        self.decouple_out[:n] = result
        return result

    def get_output(self, channel):
        return self.decouple_out[channel]

    def reset(self):
        self.decouple_out[:] = 0
        self.prev_out[:] = 0


class TestDecouplingInit(unittest.TestCase):
    """初始化测试"""

    def test_default_init(self):
        dec = DecouplingSim()
        self.assertEqual(dec.n, 2)
        self.assertEqual(dec.type, 'feedforward')

    def test_custom_channels(self):
        dec = DecouplingSim(n_channels=4)
        self.assertEqual(dec.n, 4)

    def test_initial_output_zero(self):
        dec = DecouplingSim()
        self.assertAlmostEqual(dec.get_output(0), 0.0)
        self.assertAlmostEqual(dec.get_output(1), 0.0)


class TestCouplingMatrix(unittest.TestCase):
    """耦合系数矩阵测试"""

    def test_set_coupling_matrix(self):
        dec = DecouplingSim(n_channels=2)
        K = [[1.0, 0.3], [0.2, 1.0]]
        dec.set_coupling_matrix(K)
        self.assertAlmostEqual(dec.K[0, 0], 1.0)
        self.assertAlmostEqual(dec.K[0, 1], 0.3)
        self.assertAlmostEqual(dec.K[1, 0], 0.2)
        self.assertAlmostEqual(dec.K[1, 1], 1.0)

    def test_set_3channel_matrix(self):
        dec = DecouplingSim(n_channels=3)
        K = [[1, 0.1, 0.2], [0.1, 1, 0.3], [0.2, 0.3, 1]]
        dec.set_coupling_matrix(K)
        self.assertAlmostEqual(dec.K[0, 2], 0.2)
        self.assertAlmostEqual(dec.K[2, 1], 0.3)


class TestFeedforwardDecoupling(unittest.TestCase):
    """前馈解耦测试"""

    def test_no_coupling_zero_output(self):
        """无耦合时解耦输出应为零"""
        dec = DecouplingSim(n_channels=2)
        dec.set_filter(0.0)  # 无滤波
        dec.set_feedforward_matrix([[0, 0], [0, 0]])
        out = dec.feedforward_update([1.0, 1.0])
        np.testing.assert_array_almost_equal(out, [0.0, 0.0])

    def test_coupling_produces_output(self):
        """有耦合时应产生非零补偿"""
        dec = DecouplingSim(n_channels=2)
        dec.set_filter(0.0)
        dec.set_feedforward_matrix([[0, 0.5], [0.3, 0]])
        out = dec.feedforward_update([1.0, 0.0])
        # out[0] = -Kf[0,1]*ref[1] = 0, out[1] = -Kf[1,0]*ref[0] = -0.3
        self.assertAlmostEqual(out[0], 0.0)
        self.assertAlmostEqual(out[1], -0.3, places=5)

    def test_symmetric_coupling(self):
        """对称耦合系数应对称补偿"""
        dec = DecouplingSim(n_channels=2)
        dec.set_filter(0.0)
        dec.set_feedforward_matrix([[0, 0.5], [0.5, 0]])
        out = dec.feedforward_update([1.0, 1.0])
        # out[0] = -0.5*1 = -0.5, out[1] = -0.5*1 = -0.5
        self.assertAlmostEqual(out[0], -0.5)
        self.assertAlmostEqual(out[1], -0.5)


class TestFeedbackDecoupling(unittest.TestCase):
    """反馈解耦测试"""

    def test_no_feedback_gain_zero_output(self):
        dec = DecouplingSim(n_channels=2)
        dec.set_filter(0.0)
        dec.set_feedback_matrix([[0, 0], [0, 0]])
        out = dec.feedback_update([0.5, 0.3])
        np.testing.assert_array_almost_equal(out, [0.0, 0.0])

    def test_feedback_compensation(self):
        dec = DecouplingSim(n_channels=2)
        dec.set_filter(0.0)
        dec.set_feedback_matrix([[0, 0.4], [0.6, 0]])
        out = dec.feedback_update([1.0, 0.5])
        # out = -D @ fb = [-(0*1+0.4*0.5), -(0.6*1+0*0.5)] = [-0.2, -0.6]
        self.assertAlmostEqual(out[0], -0.2, places=5)
        self.assertAlmostEqual(out[1], -0.6, places=5)


class TestHybridDecoupling(unittest.TestCase):
    """混合解耦测试"""

    def test_hybrid_sum(self):
        """混合输出应为前馈+反馈之和"""
        dec = DecouplingSim(n_channels=2)
        dec.set_filter(0.0)
        dec.set_feedforward_matrix([[0, 0.3], [0.2, 0]])
        dec.set_feedback_matrix([[0, 0.1], [0.1, 0]])
        out = dec.hybrid_update([1.0, 1.0], [0.5, 0.5])
        # ff: [-0.3, -0.2], fb: [-0.05, -0.05]
        self.assertAlmostEqual(out[0], -0.35, places=4)
        self.assertAlmostEqual(out[1], -0.25, places=4)


class TestFilterSmoothing(unittest.TestCase):
    """滤波平滑测试"""

    def test_no_filter(self):
        """alpha=0应无滤波"""
        dec = DecouplingSim(n_channels=2)
        dec.set_filter(0.0)
        dec.set_feedforward_matrix([[0, 1.0], [1.0, 0]])
        out = dec.feedforward_update([1.0, 0.0])
        self.assertAlmostEqual(out[1], -1.0, places=5)

    def test_full_filter(self):
        """alpha=1应完全保持上一次输出"""
        dec = DecouplingSim(n_channels=2)
        dec.set_filter(1.0)
        dec.set_feedforward_matrix([[0, 1.0], [1.0, 0]])
        dec.prev_out[0] = 999.0
        dec.prev_out[1] = 888.0
        out = dec.feedforward_update([1.0, 0.0])
        self.assertAlmostEqual(out[0], 999.0)
        self.assertAlmostEqual(out[1], 888.0)

    def test_partial_filter(self):
        """部分滤波应平滑输出"""
        dec = DecouplingSim(n_channels=2)
        dec.set_filter(0.5)
        dec.set_feedforward_matrix([[0, 2.0], [0, 0]])
        dec.prev_out[0] = 0.0
        dec.prev_out[1] = 0.0
        out = dec.feedforward_update([0.0, 1.0])
        # 原始 = [-2.0, 0], 滤波 = 0.5*0 + 0.5*(-2) = -1.0
        self.assertAlmostEqual(out[0], -1.0, places=5)


class TestReset(unittest.TestCase):
    """重置测试"""

    def test_reset_clears_state(self):
        dec = DecouplingSim(n_channels=2)
        dec.decouple_out[0] = 100
        dec.prev_out[1] = 200
        dec.reset()
        self.assertAlmostEqual(dec.get_output(0), 0.0)
        self.assertAlmostEqual(dec.get_output(1), 0.0)


class TestMultiChannel(unittest.TestCase):
    """多通道测试"""

    def test_4channel_feedforward(self):
        dec = DecouplingSim(n_channels=4)
        dec.set_filter(0.0)
        Kf = np.zeros((4, 4))
        Kf[0, 1] = 0.1
        Kf[1, 2] = 0.2
        Kf[2, 3] = 0.3
        Kf[3, 0] = 0.4
        dec.set_feedforward_matrix(Kf)
        ref = [1.0, 1.0, 1.0, 1.0]
        out = dec.feedforward_update(ref)
        self.assertAlmostEqual(out[0], -0.1, places=5)
        self.assertAlmostEqual(out[1], -0.2, places=5)
        self.assertAlmostEqual(out[2], -0.3, places=5)
        self.assertAlmostEqual(out[3], -0.4, places=5)


class TestDecouplingEffectiveness(unittest.TestCase):
    """解耦效果验证"""

    def test_reduces_cross_coupling(self):
        """解耦应减少通道间耦合"""
        # 模拟: 通道0参考变化, 通道1应不受影响(解耦后)
        dec = DecouplingSim(n_channels=2)
        dec.set_filter(0.0)
        dec.set_feedforward_matrix([[0, 0.5], [0.5, 0]])

        # 无解耦时通道1受到通道0的耦合影响
        coupled_output_ch1 = 0.5 * 1.0  # K[1,0] * ref[0]

        # 有解耦时补偿
        out = dec.feedforward_update([1.0, 0.0])
        residual = coupled_output_ch1 + out[1]
        # 残余应小于耦合量
        self.assertLess(abs(residual), abs(coupled_output_ch1))


if __name__ == '__main__':
    unittest.main()
