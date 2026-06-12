#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
视觉伺服单元测试
覆盖: interaction_matrix_point交互矩阵、IBVSSimulator初始化/单步/运行/收敛、
      PBVSSimulator初始化/单步/运行/收敛、错误特征点数、期望位姿自定义
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from _11_控制算法库.simulation.visual_servo_simulation import (
    interaction_matrix_point,
    IBVSSimulator,
    PBVSSimulator,
)


# ==================== 交互矩阵测试 ====================

class TestInteractionMatrixPoint(unittest.TestCase):
    """交互矩阵测试"""

    def test_returns_2x6(self):
        L = interaction_matrix_point(320, 240, 1.0, 500, 500, 320, 240)
        self.assertEqual(L.shape, (2, 6))

    def test_returns_ndarray(self):
        L = interaction_matrix_point(320, 240, 1.0, 500, 500, 320, 240)
        self.assertIsInstance(L, np.ndarray)

    def test_center_point_values(self):
        """光心处的交互矩阵应有特定结构"""
        L = interaction_matrix_point(320, 240, 1.0, 500, 500, 320, 240)
        # u-cx=0, v-cy=0 → L[0,3]=0, L[0,4]=-fx, L[0,5]=0
        self.assertAlmostEqual(L[0, 0], -500.0, places=1)
        self.assertAlmostEqual(L[1, 1], -500.0, places=1)

    def test_different_depths(self):
        """不同深度应改变交互矩阵值"""
        L1 = interaction_matrix_point(320, 240, 0.5, 500, 500, 320, 240)
        L2 = interaction_matrix_point(320, 240, 2.0, 500, 500, 320, 240)
        self.assertFalse(np.allclose(L1, L2))

    def test_off_center_point(self):
        """非光心点应有非零交互矩阵"""
        L = interaction_matrix_point(400, 300, 1.0, 500, 500, 320, 240)
        self.assertGreater(np.sum(np.abs(L)), 0)

    def test_dtype_float(self):
        L = interaction_matrix_point(320, 240, 1.0, 500, 500, 320, 240)
        self.assertTrue(np.issubdtype(L.dtype, np.floating))


# ==================== IBVS仿真器初始化测试 ====================

class TestIBVSSimulatorInit(unittest.TestCase):
    """IBVS仿真器初始化测试"""

    def test_default_params(self):
        sim = IBVSSimulator()
        self.assertEqual(sim.fx, 500)
        self.assertEqual(sim.fy, 500)
        self.assertEqual(sim.cx, 320)
        self.assertEqual(sim.cy, 240)
        self.assertEqual(sim.lambda_gain, 0.5)
        self.assertEqual(sim.Z, 1.0)
        self.assertEqual(sim.n, 4)

    def test_custom_params(self):
        sim = IBVSSimulator(n_features=6, fx=600, fy=600, cx=400, cy=300,
                            lambda_gain=0.8, depth_est=2.0)
        self.assertEqual(sim.n, 6)
        self.assertEqual(sim.fx, 600)
        self.assertEqual(sim.lambda_gain, 0.8)
        self.assertEqual(sim.Z, 2.0)

    def test_desired_features_shape(self):
        sim = IBVSSimulator(n_features=4)
        self.assertEqual(sim.desired.shape, (4, 2))

    def test_current_features_shape(self):
        sim = IBVSSimulator(n_features=4)
        self.assertEqual(sim.current.shape, (4, 2))

    def test_error_hist_empty(self):
        sim = IBVSSimulator()
        self.assertEqual(len(sim.error_hist), 0)

    def test_feature_trails_empty(self):
        sim = IBVSSimulator(n_features=4)
        self.assertEqual(len(sim.feature_trails), 4)
        for trail in sim.feature_trails:
            self.assertEqual(len(trail), 0)


# ==================== IBVS单步测试 ====================

class TestIBVSSimulatorStep(unittest.TestCase):
    """IBVS仿真器单步测试"""

    def test_step_returns_velocity(self):
        sim = IBVSSimulator()
        v = sim.step()
        self.assertEqual(v.shape, (6,))

    def test_error_history_recorded(self):
        sim = IBVSSimulator()
        sim.step()
        self.assertEqual(len(sim.error_hist), 1)

    def test_feature_trails_recorded(self):
        sim = IBVSSimulator(n_features=4)
        sim.step()
        for trail in sim.feature_trails:
            self.assertEqual(len(trail), 1)

    def test_multiple_steps(self):
        sim = IBVSSimulator()
        for _ in range(10):
            sim.step()
        self.assertEqual(len(sim.error_hist), 10)
        for trail in sim.feature_trails:
            self.assertEqual(len(trail), 10)

    def test_error_decreases_over_steps(self):
        """误差应随迭代减小"""
        sim = IBVSSimulator(lambda_gain=0.5)
        for _ in range(50):
            sim.step()
        self.assertLess(sim.error_hist[-1], sim.error_hist[0])


# ==================== IBVS运行与收敛测试 ====================

class TestIBVSSimulatorRun(unittest.TestCase):
    """IBVS仿真器运行测试"""

    def test_run_completes(self):
        sim = IBVSSimulator()
        sim.run(max_iter=200, threshold=1.0)
        self.assertGreater(len(sim.error_hist), 0)

    def test_convergence_within_max_iter(self):
        """在足够迭代次数内应收敛"""
        sim = IBVSSimulator(lambda_gain=0.5)
        sim.run(max_iter=500, threshold=5.0)
        self.assertLess(sim.error_hist[-1], 5.0)

    def test_initial_error_larger_than_final(self):
        sim = IBVSSimulator()
        sim.run(max_iter=200)
        self.assertGreater(sim.error_hist[0], sim.error_hist[-1])


# ==================== PBVS仿真器初始化测试 ====================

class TestPBVSSimulatorInit(unittest.TestCase):
    """PBVS仿真器初始化测试"""

    def test_default_params(self):
        sim = PBVSSimulator()
        self.assertEqual(sim.lambda_pos, 1.0)
        self.assertEqual(sim.lambda_rot, 0.8)

    def test_custom_params(self):
        sim = PBVSSimulator(lambda_pos=2.0, lambda_rot=1.5)
        self.assertEqual(sim.lambda_pos, 2.0)
        self.assertEqual(sim.lambda_rot, 1.5)

    def test_desired_pose(self):
        sim = PBVSSimulator()
        np.testing.assert_array_equal(sim.desired_pose, np.zeros(6))

    def test_current_pose_offset(self):
        sim = PBVSSimulator()
        # 默认应有偏移
        self.assertNotEqual(np.linalg.norm(sim.current_pose), 0.0)

    def test_histories_empty(self):
        sim = PBVSSimulator()
        self.assertEqual(len(sim.error_pos_hist), 0)
        self.assertEqual(len(sim.error_rot_hist), 0)
        self.assertEqual(len(sim.pose_hist), 0)


# ==================== PBVS单步测试 ====================

class TestPBVSSimulatorStep(unittest.TestCase):
    """PBVS仿真器单步测试"""

    def test_step_returns_velocity(self):
        sim = PBVSSimulator()
        v = sim.step()
        self.assertEqual(v.shape, (6,))

    def test_position_error_recorded(self):
        sim = PBVSSimulator()
        sim.step()
        self.assertEqual(len(sim.error_pos_hist), 1)

    def test_rotation_error_recorded(self):
        sim = PBVSSimulator()
        sim.step()
        self.assertEqual(len(sim.error_rot_hist), 1)

    def test_pose_history_recorded(self):
        sim = PBVSSimulator()
        sim.step()
        self.assertEqual(len(sim.pose_hist), 1)

    def test_multiple_steps(self):
        sim = PBVSSimulator()
        for _ in range(10):
            sim.step()
        self.assertEqual(len(sim.error_pos_hist), 10)
        self.assertEqual(len(sim.pose_hist), 10)

    def test_error_decreases(self):
        """位置误差应随迭代减小"""
        sim = PBVSSimulator(lambda_pos=1.0, lambda_rot=0.8)
        for _ in range(200):
            sim.step()
        self.assertLess(sim.error_pos_hist[-1], sim.error_pos_hist[0])


# ==================== PBVS运行与收敛测试 ====================

class TestPBVSSimulatorRun(unittest.TestCase):
    """PBVS仿真器运行测试"""

    def test_run_completes(self):
        sim = PBVSSimulator()
        sim.run(max_iter=300, threshold=0.01)
        self.assertGreater(len(sim.error_pos_hist), 0)

    def test_convergence(self):
        """PBVS应在足够迭代后收敛"""
        sim = PBVSSimulator(lambda_pos=1.0, lambda_rot=0.8)
        sim.run(max_iter=500, threshold=0.01)
        total_err = sim.error_pos_hist[-1] + sim.error_rot_hist[-1]
        self.assertLess(total_err, 0.1)

    def test_initial_error_larger(self):
        sim = PBVSSimulator()
        sim.run(max_iter=300)
        self.assertGreater(sim.error_pos_hist[0], sim.error_pos_hist[-1])

    def test_pose_converges_to_desired(self):
        """最终位姿应趋近期望"""
        sim = PBVSSimulator(lambda_pos=1.0, lambda_rot=0.8)
        sim.run(max_iter=500, threshold=0.001)
        final_err = np.linalg.norm(sim.current_pose - sim.desired_pose)
        self.assertLess(final_err, 0.5)


if __name__ == '__main__':
    unittest.main()
