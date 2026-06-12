#!/usr/bin/env python3
"""
多ROI检测器单元测试
覆盖: ROI管理、网格划分、并行检测、坐标转换、结果合并
注意: 使用纯Python + NumPy + OpenCV模拟
"""

import sys
import os
import unittest
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Callable, Dict, Any
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    _has_cv2 = True
except ImportError:
    _has_cv2 = False


# ── 模拟实现 ──────────────────────────────────────────────────

@dataclass
class ROIRegion:
    name: str
    x: int
    y: int
    w: int
    h: int
    color: Tuple[int, int, int] = (0, 255, 0)
    params: Dict[str, Any] = field(default_factory=dict)
    enabled: bool = True


class MultiROIDetectorSimulator:
    """多ROI检测器模拟"""

    def __init__(self, max_workers=4):
        self.rois: List[ROIRegion] = []
        self._detectors: Dict[str, Callable] = {}
        self._executor = ThreadPoolExecutor(max_workers=max_workers)
        self._results: Dict[str, list] = {}

    def add_roi(self, name, x, y, w, h, color=(0, 255, 0), params=None):
        roi = ROIRegion(name=name, x=x, y=y, w=w, h=h,
                        color=color, params=params or {})
        self.rois.append(roi)
        return self

    def add_grid_rois(self, rows, cols, frame_w, frame_h, prefix="roi"):
        rw, rh = frame_w // cols, frame_h // rows
        for r in range(rows):
            for c in range(cols):
                name = f"{prefix}_{r}_{c}"
                self.add_roi(name, c * rw, r * rh, rw, rh)
        return self

    def set_detector(self, roi_name, detector_fn):
        self._detectors[roi_name] = detector_fn
        return self

    def set_default_detector(self, detector_fn):
        for roi in self.rois:
            if roi.name not in self._detectors:
                self._detectors[roi.name] = detector_fn
        return self

    def _detect_single(self, roi: ROIRegion, frame: np.ndarray):
        if not roi.enabled or roi.name not in self._detectors:
            return roi.name, []
        x, y = max(0, roi.x), max(0, roi.y)
        x2 = min(frame.shape[1], roi.x + roi.w)
        y2 = min(frame.shape[0], roi.y + roi.h)
        cropped = frame[y:y2, x:x2]
        if cropped.size == 0:
            return roi.name, []
        detections = self._detectors[roi.name](cropped, roi.params)
        results = []
        for det in (detections or []):
            if isinstance(det, dict):
                det = dict(det)
                if "cx" in det:
                    det["cx"] += x
                    det["cy"] += y
                if "bbox" in det:
                    bx, by, bw, bh = det["bbox"]
                    det["bbox"] = (bx + x, by + y, bw, bh)
            elif isinstance(det, (list, tuple)) and len(det) >= 2:
                det = list(det)
                det[0] += x
                det[1] += y
            results.append(det)
        return roi.name, results

    def detect(self, frame: np.ndarray) -> Dict[str, list]:
        futures = []
        for roi in self.rois:
            fut = self._executor.submit(self._detect_single, roi, frame)
            futures.append(fut)
        self._results = {}
        for fut in futures:
            name, detections = fut.result()
            self._results[name] = detections
        return self._results

    def get_all_detections(self):
        all_dets = []
        for name, dets in self._results.items():
            for d in dets:
                if isinstance(d, dict):
                    d = dict(d)
                    d["roi"] = name
                all_dets.append(d)
        return all_dets

    def __del__(self):
        self._executor.shutdown(wait=False)


def _dummy_detector(cropped, params):
    """简单检测器：返回图像中心"""
    h, w = cropped.shape[:2]
    return [{"cx": w // 2, "cy": h // 2, "area": h * w}]


def _bright_spot_detector(cropped, params):
    """亮点检测器"""
    threshold = params.get("threshold", 200)
    gray = cropped if len(cropped.shape) == 2 else cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    mask = gray > threshold
    if not np.any(mask):
        return []
    ys, xs = np.where(mask)
    return [{"cx": int(np.mean(xs)), "cy": int(np.mean(ys)), "area": int(np.sum(mask))}]


# ── 测试用例 ──────────────────────────────────────────────────

class TestROIManagement(unittest.TestCase):
    """ROI管理测试"""

    def test_add_single_roi(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("test", 0, 0, 100, 100)
        self.assertEqual(len(det.rois), 1)
        self.assertEqual(det.rois[0].name, "test")
        self.assertEqual(det.rois[0].x, 0)
        self.assertEqual(det.rois[0].w, 100)

    def test_add_multiple_rois(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("left", 0, 0, 320, 480)
        det.add_roi("right", 320, 0, 320, 480)
        self.assertEqual(len(det.rois), 2)

    def test_roi_chaining(self):
        det = MultiROIDetectorSimulator()
        ret = det.add_roi("a", 0, 0, 100, 100).add_roi("b", 100, 0, 100, 100)
        self.assertIs(ret, det)

    def test_roi_enabled_default(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("test", 0, 0, 100, 100)
        self.assertTrue(det.rois[0].enabled)

    def test_roi_custom_color(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("test", 0, 0, 100, 100, color=(255, 0, 0))
        self.assertEqual(det.rois[0].color, (255, 0, 0))

    def test_roi_custom_params(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("test", 0, 0, 100, 100, params={"threshold": 150})
        self.assertEqual(det.rois[0].params["threshold"], 150)


class TestGridROIs(unittest.TestCase):
    """网格划分测试"""

    def test_2x2_grid(self):
        det = MultiROIDetectorSimulator()
        det.add_grid_rois(2, 2, 640, 480)
        self.assertEqual(len(det.rois), 4)
        names = [r.name for r in det.rois]
        self.assertIn("roi_0_0", names)
        self.assertIn("roi_0_1", names)
        self.assertIn("roi_1_0", names)
        self.assertIn("roi_1_1", names)

    def test_grid_dimensions(self):
        det = MultiROIDetectorSimulator()
        det.add_grid_rois(2, 2, 640, 480)
        for roi in det.rois:
            self.assertEqual(roi.w, 320)  # 640 / 2
            self.assertEqual(roi.h, 240)  # 480 / 2

    def test_grid_positions(self):
        det = MultiROIDetectorSimulator()
        det.add_grid_rois(2, 3, 300, 200)
        # roi_1_2 应在 (200, 100)
        roi = next(r for r in det.rois if r.name == "roi_1_2")
        self.assertEqual(roi.x, 200)
        self.assertEqual(roi.y, 100)

    def test_grid_prefix(self):
        det = MultiROIDetectorSimulator()
        det.add_grid_rois(1, 2, 200, 100, prefix="cam")
        names = [r.name for r in det.rois]
        self.assertIn("cam_0_0", names)
        self.assertIn("cam_0_1", names)


class TestDetectorAssignment(unittest.TestCase):
    """检测器分配测试"""

    def test_set_detector(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("test", 0, 0, 100, 100)
        det.set_detector("test", _dummy_detector)
        self.assertIn("test", det._detectors)

    def test_set_default_detector(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("a", 0, 0, 100, 100)
        det.add_roi("b", 100, 0, 100, 100)
        det.set_default_detector(_dummy_detector)
        self.assertIn("a", det._detectors)
        self.assertIn("b", det._detectors)

    def test_default_does_not_override_specific(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("a", 0, 0, 100, 100)
        det.set_detector("a", _bright_spot_detector)
        det.set_default_detector(_dummy_detector)
        self.assertIs(det._detectors["a"], _bright_spot_detector)


class TestDetection(unittest.TestCase):
    """检测功能测试"""

    def test_detect_returns_results(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("center", 100, 100, 200, 200)
        det.set_default_detector(_dummy_detector)
        frame = np.zeros((400, 400, 3), dtype=np.uint8)
        results = det.detect(frame)
        self.assertIn("center", results)
        self.assertEqual(len(results["center"]), 1)

    def test_coordinate_translation(self):
        """检测坐标应转换回原图坐标系"""
        det = MultiROIDetectorSimulator()
        det.add_roi("region", 100, 200, 100, 100)
        det.set_default_detector(_dummy_detector)
        frame = np.zeros((400, 400, 3), dtype=np.uint8)
        results = det.detect(frame)
        d = results["region"][0]
        # 检测器返回裁剪图中心 (50, 50)，加上偏移 (100, 200)
        self.assertEqual(d["cx"], 150)
        self.assertEqual(d["cy"], 250)

    def test_disabled_roi_skipped(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("test", 0, 0, 100, 100)
        det.rois[0].enabled = False
        det.set_default_detector(_dummy_detector)
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        results = det.detect(frame)
        self.assertEqual(results["test"], [])

    def test_no_detector_returns_empty(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("test", 0, 0, 100, 100)
        # 不设置检测器
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        results = det.detect(frame)
        self.assertEqual(results["test"], [])

    def test_empty_frame(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("test", 500, 500, 100, 100)
        det.set_default_detector(_dummy_detector)
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        results = det.detect(frame)
        # ROI超出帧范围，裁剪为空
        self.assertEqual(results["test"], [])


class TestResultMerging(unittest.TestCase):
    """结果合并测试"""

    def test_get_all_detections(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("a", 0, 0, 100, 100)
        det.add_roi("b", 100, 0, 100, 100)
        det.set_default_detector(_dummy_detector)
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        det.detect(frame)
        all_dets = det.get_all_detections()
        self.assertEqual(len(all_dets), 2)
        # 每个结果应有roi标记
        roi_names = {d["roi"] for d in all_dets}
        self.assertEqual(roi_names, {"a", "b"})

    def test_empty_detections(self):
        det = MultiROIDetectorSimulator()
        det.add_roi("empty", 0, 0, 100, 100)
        det.rois[0].enabled = False
        det.set_default_detector(_dummy_detector)
        frame = np.zeros((200, 200, 3), dtype=np.uint8)
        det.detect(frame)
        self.assertEqual(det.get_all_detections(), [])


if __name__ == '__main__':
    unittest.main()
