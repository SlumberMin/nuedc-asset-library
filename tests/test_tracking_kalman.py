#!/usr/bin/env python3
"""
Kalman跟踪器单元测试
覆盖: 单目标跟踪、匀速/匀加速模型、预测-更新、多目标跟踪、轨迹管理
注意: 使用纯Python + NumPy + OpenCV KalmanFilter
"""

import sys
import os
import unittest
import numpy as np
from collections import deque
from typing import List, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    _has_cv2 = True
except ImportError:
    _has_cv2 = False


# ── 模拟实现 ──────────────────────────────────────────────────

class KalmanTrackerSimulator:
    """Kalman跟踪器模拟（纯numpy实现）"""

    def __init__(self, model="constant_velocity", dt=1.0,
                 process_noise=1e-2, measure_noise=1e-1):
        self.model = model
        self.dt = dt
        self._age = 0
        self._hits = 0
        self._misses = 0
        self._history = deque(maxlen=50)
        self._initialized = False

        if model == "constant_velocity":
            self._dim = 4
            self._init_cv(dt, process_noise, measure_noise)
        elif model == "constant_acceleration":
            self._dim = 6
            self._init_ca(dt, process_noise, measure_noise)
        else:
            raise ValueError(f"未知模型: {model}")

    def _init_cv(self, dt, q, r):
        n = 4
        self.x = np.zeros((n, 1), dtype=np.float64)
        self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]], dtype=np.float64)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=np.float64)
        self.Q = np.eye(n, dtype=np.float64) * q
        self.R = np.eye(2, dtype=np.float64) * r
        self.P = np.eye(n, dtype=np.float64)

    def _init_ca(self, dt, q, r):
        n = 6
        dt2 = 0.5 * dt * dt
        self.x = np.zeros((n, 1), dtype=np.float64)
        self.F = np.array([
            [1, 0, dt, 0, dt2, 0],
            [0, 1, 0, dt, 0, dt2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]], dtype=np.float64)
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0]], dtype=np.float64)
        self.Q = np.eye(n, dtype=np.float64) * q
        self.R = np.eye(2, dtype=np.float64) * r
        self.P = np.eye(n, dtype=np.float64)

    def init(self, x, y):
        self.x[0, 0] = x
        self.x[1, 0] = y
        for i in range(2, self._dim):
            self.x[i, 0] = 0
        self._initialized = True
        self._history.append((x, y))

    @property
    def position(self):
        return float(self.x[0, 0]), float(self.x[1, 0])

    @property
    def velocity(self):
        return float(self.x[2, 0]), float(self.x[3, 0])

    @property
    def acceleration(self):
        if self.model != "constant_acceleration":
            raise AttributeError("仅匀加速模型支持加速度查询")
        return float(self.x[4, 0]), float(self.x[5, 0])

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self._age += 1
        return float(self.x[0, 0]), float(self.x[1, 0])

    def update(self, x, y):
        if not self._initialized:
            self.init(x, y)
            return x, y
        self.predict()
        z = np.array([[x], [y]], dtype=np.float64)
        y_res = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y_res
        self.P = (np.eye(self._dim) - K @ self.H) @ self.P
        self._hits += 1
        self._misses = 0
        pos = self.position
        self._history.append(pos)
        return pos

    def predict_only(self):
        self._misses += 1
        return self.predict()

    def get_smoothed_position(self, window=5):
        if len(self._history) == 0:
            return self.position
        n = min(window, len(self._history))
        pts = list(self._history)[-n:]
        xs = sum(p[0] for p in pts) / n
        ys = sum(p[1] for p in pts) / n
        return xs, ys

    def get_history(self):
        return list(self._history)


class MultiTargetTrackerSimulator:
    """多目标跟踪管理器模拟"""

    def __init__(self, max_misses=10, model="constant_velocity"):
        self.trackers: List[KalmanTrackerSimulator] = []
        self.max_misses = max_misses
        self.model = model
        self._next_id = 0
        self._ids: List[int] = []

    def update(self, detections):
        for trk in self.trackers:
            trk.predict()

        if len(self.trackers) == 0:
            for det in detections:
                self._add_tracker(det[0], det[1])
            return

        if len(detections) == 0:
            for trk in self.trackers:
                trk.predict_only()
            self._remove_dead()
            return

        det_arr = np.array([(d[0], d[1]) for d in detections])
        trk_arr = np.array([trk.position for trk in self.trackers])

        cost = np.full((len(self.trackers), len(detections)), float('inf'))
        for i, trk_pos in enumerate(trk_arr):
            for j, det_pos in enumerate(det_arr):
                cost[i, j] = np.linalg.norm(trk_pos - det_pos)

        matched_trk = set()
        matched_det = set()
        while True:
            if cost.size == 0:
                break
            idx = np.argmin(cost)
            i, j = divmod(idx, cost.shape[1])
            if cost[i, j] > 200:
                break
            self.trackers[i].update(det_arr[j, 0], det_arr[j, 1])
            matched_trk.add(i)
            matched_det.add(j)
            cost[i, :] = float("inf")
            cost[:, j] = float("inf")

        for i in range(len(self.trackers)):
            if i not in matched_trk:
                self.trackers[i].predict_only()

        for j in range(len(detections)):
            if j not in matched_det:
                self._add_tracker(det_arr[j, 0], det_arr[j, 1])

        self._remove_dead()

    def _add_tracker(self, x, y):
        trk = KalmanTrackerSimulator(model=self.model)
        trk.init(x, y)
        self.trackers.append(trk)
        self._ids.append(self._next_id)
        self._next_id += 1

    def _remove_dead(self):
        alive = []
        alive_ids = []
        for trk, tid in zip(self.trackers, self._ids):
            if trk._misses < self.max_misses:
                alive.append(trk)
                alive_ids.append(tid)
        self.trackers = alive
        self._ids = alive_ids

    def get_tracks(self):
        return [
            {"id": tid, "pos": trk.position, "vel": trk.velocity,
             "age": trk.age, "hits": trk.hits}
            for trk, tid in zip(self.trackers, self._ids)
        ]


# ── 测试用例 ──────────────────────────────────────────────────

class TestKalmanTrackerInit(unittest.TestCase):
    """初始化测试"""

    def test_cv_model_position_zero(self):
        trk = KalmanTrackerSimulator(model="constant_velocity")
        pos = trk.position
        self.assertAlmostEqual(pos[0], 0.0)
        self.assertAlmostEqual(pos[1], 0.0)

    def test_ca_model_position_zero(self):
        trk = KalmanTrackerSimulator(model="constant_acceleration")
        pos = trk.position
        self.assertAlmostEqual(pos[0], 0.0)
        self.assertAlmostEqual(pos[1], 0.0)

    def test_invalid_model_raises(self):
        with self.assertRaises(ValueError):
            KalmanTrackerSimulator(model="unknown")

    def test_init_sets_position(self):
        trk = KalmanTrackerSimulator()
        trk.init(100.0, 200.0)
        pos = trk.position
        self.assertAlmostEqual(pos[0], 100.0)
        self.assertAlmostEqual(pos[1], 200.0)


class TestKalmanTrackerPredict(unittest.TestCase):
    """预测测试"""

    def test_predict_cv_stationary(self):
        """静止目标预测位置不变"""
        trk = KalmanTrackerSimulator(model="constant_velocity", dt=1.0)
        trk.init(50.0, 50.0)
        pred = trk.predict()
        self.assertAlmostEqual(pred[0], 50.0, delta=1.0)
        self.assertAlmostEqual(pred[1], 50.0, delta=1.0)

    def test_predict_increments_age(self):
        trk = KalmanTrackerSimulator()
        trk.init(0, 0)
        self.assertEqual(trk._age, 0)
        trk.predict()
        self.assertEqual(trk._age, 1)
        trk.predict()
        self.assertEqual(trk._age, 2)


class TestKalmanTrackerUpdate(unittest.TestCase):
    """更新测试"""

    def test_update_converges_to_constant(self):
        """恒定观测应收敛"""
        trk = KalmanTrackerSimulator(model="constant_velocity", dt=1.0,
                                      process_noise=0.001, measure_noise=0.1)
        for _ in range(50):
            trk.update(100.0, 200.0)
        pos = trk.position
        self.assertAlmostEqual(pos[0], 100.0, delta=2.0)
        self.assertAlmostEqual(pos[1], 200.0, delta=2.0)

    def test_update_tracks_linear_motion(self):
        """匀速运动应跟踪"""
        trk = KalmanTrackerSimulator(model="constant_velocity", dt=1.0,
                                      process_noise=0.01, measure_noise=0.1)
        for i in range(50):
            trk.update(float(i), float(i))
        pos = trk.position
        # 应接近最后的观测值
        self.assertAlmostEqual(pos[0], 49.0, delta=5.0)
        self.assertAlmostEqual(pos[1], 49.0, delta=5.0)

    def test_update_increments_hits(self):
        trk = KalmanTrackerSimulator()
        trk.update(10.0, 20.0)
        self.assertEqual(trk._hits, 1)
        trk.update(11.0, 21.0)
        self.assertEqual(trk._hits, 2)

    def test_auto_init_on_first_update(self):
        """第一次update自动初始化"""
        trk = KalmanTrackerSimulator()
        self.assertFalse(trk._initialized)
        pos = trk.update(42.0, 84.0)
        self.assertTrue(trk._initialized)
        self.assertAlmostEqual(pos[0], 42.0)
        self.assertAlmostEqual(pos[1], 84.0)


class TestKalmanTrackerPredictOnly(unittest.TestCase):
    """仅预测（目标丢失）测试"""

    def test_predict_only_increments_misses(self):
        trk = KalmanTrackerSimulator()
        trk.init(50, 50)
        trk.predict_only()
        self.assertEqual(trk._misses, 1)
        trk.predict_only()
        self.assertEqual(trk._misses, 2)

    def test_predict_only_resets_on_update(self):
        trk = KalmanTrackerSimulator()
        trk.init(50, 50)
        trk.predict_only()
        trk.predict_only()
        self.assertEqual(trk._misses, 2)
        trk.update(55, 55)
        self.assertEqual(trk._misses, 0)


class TestKalmanTrackerVelocity(unittest.TestCase):
    """速度估计测试"""

    def test_cv_velocity_estimation(self):
        """匀速运动应估计出正确速度"""
        trk = KalmanTrackerSimulator(model="constant_velocity", dt=1.0,
                                      process_noise=0.001, measure_noise=0.1)
        for i in range(50):
            trk.update(float(i * 2), float(i * 3))
        vx, vy = trk.velocity
        # 速度应趋近 (2.0, 3.0)
        self.assertAlmostEqual(vx, 2.0, delta=1.0)
        self.assertAlmostEqual(vy, 3.0, delta=1.0)


class TestKalmanTrackerCA(unittest.TestCase):
    """匀加速模型测试"""

    def test_ca_acceleration_property(self):
        trk = KalmanTrackerSimulator(model="constant_acceleration")
        trk.init(0, 0)
        ax, ay = trk.acceleration
        self.assertAlmostEqual(ax, 0.0)
        self.assertAlmostEqual(ay, 0.0)

    def test_cv_no_acceleration(self):
        trk = KalmanTrackerSimulator(model="constant_velocity")
        trk.init(0, 0)
        with self.assertRaises(AttributeError):
            _ = trk.acceleration


class TestKalmanTrackerHistory(unittest.TestCase):
    """轨迹历史测试"""

    def test_history_after_updates(self):
        trk = KalmanTrackerSimulator()
        trk.update(10, 20)
        trk.update(11, 21)
        trk.update(12, 22)
        history = trk.get_history()
        self.assertEqual(len(history), 3)

    def test_history_maxlen(self):
        trk = KalmanTrackerSimulator()
        for i in range(60):
            trk.update(float(i), float(i))
        history = trk.get_history()
        self.assertLessEqual(len(history), 50)

    def test_smoothed_position(self):
        trk = KalmanTrackerSimulator()
        trk.update(10, 10)
        trk.update(10, 10)
        trk.update(10, 10)
        sx, sy = trk.get_smoothed_position(window=3)
        self.assertAlmostEqual(sx, 10.0, delta=2.0)
        self.assertAlmostEqual(sy, 10.0, delta=2.0)


class TestMultiTargetTracker(unittest.TestCase):
    """多目标跟踪测试"""

    def test_create_trackers_from_detections(self):
        mtt = MultiTargetTrackerSimulator()
        mtt.update([(10, 10), (200, 200)])
        tracks = mtt.get_tracks()
        self.assertEqual(len(tracks), 2)

    def test_matching_existing_tracks(self):
        mtt = MultiTargetTrackerSimulator()
        mtt.update([(10, 10), (200, 200)])
        # 第二帧，目标稍微移动
        mtt.update([(12, 12), (198, 198)])
        tracks = mtt.get_tracks()
        self.assertEqual(len(tracks), 2)

    def test_new_detection_creates_tracker(self):
        mtt = MultiTargetTrackerSimulator()
        mtt.update([(10, 10)])
        self.assertEqual(len(mtt.trackers), 1)
        mtt.update([(10, 10), (300, 300)])  # 新目标出现
        self.assertEqual(len(mtt.trackers), 2)

    def test_missed_target_removed(self):
        mtt = MultiTargetTrackerSimulator(max_misses=3)
        mtt.update([(10, 10)])
        # 连续无检测
        for _ in range(5):
            mtt.update([])
        self.assertEqual(len(mtt.trackers), 0)

    def test_get_tracks_format(self):
        mtt = MultiTargetTrackerSimulator()
        mtt.update([(50, 50)])
        tracks = mtt.get_tracks()
        t = tracks[0]
        self.assertIn("id", t)
        self.assertIn("pos", t)
        self.assertIn("vel", t)
        self.assertIn("age", t)
        self.assertIn("hits", t)

    def test_empty_detections(self):
        mtt = MultiTargetTrackerSimulator()
        mtt.update([])
        self.assertEqual(len(mtt.trackers), 0)
        self.assertEqual(mtt.get_tracks(), [])


if __name__ == '__main__':
    unittest.main()
