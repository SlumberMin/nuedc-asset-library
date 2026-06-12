#!/usr/bin/env python3
"""
颜色校准GUI单元测试
覆盖: 参数管理、HSV范围、掩码生成、参数保存/加载
注意: 不依赖GUI窗口，使用纯Python + NumPy + OpenCV模拟
"""

import sys
import os
import unittest
import json
import tempfile
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    _has_cv2 = True
except ImportError:
    _has_cv2 = False


# ── 模拟实现（去除GUI依赖）────────────────────────────────────

class ColorCalibrationSimulator:
    """颜色校准核心逻辑模拟（无GUI）"""

    def __init__(self):
        self.params = {}
        self._callbacks = {}

    def add_trackbar(self, name, default=0, max_val=255, callback=None):
        self.params[name] = default
        if callback:
            self._callbacks[name] = callback
        return self

    def add_hsv_range(self, prefix=""):
        p = f"{prefix}_" if prefix else ""
        self.add_trackbar(f"{p}H_min", 0, 179)
        self.add_trackbar(f"{p}H_max", 179, 179)
        self.add_trackbar(f"{p}S_min", 0, 255)
        self.add_trackbar(f"{p}S_max", 255, 255)
        self.add_trackbar(f"{p}V_min", 0, 255)
        self.add_trackbar(f"{p}V_max", 255, 255)
        return self

    def get_hsv_range(self, prefix=""):
        p = f"{prefix}_" if prefix else ""
        lower = np.array([
            self.params[f"{p}H_min"],
            self.params[f"{p}S_min"],
            self.params[f"{p}V_min"]
        ])
        upper = np.array([
            self.params[f"{p}H_max"],
            self.params[f"{p}S_max"],
            self.params[f"{p}V_max"]
        ])
        return lower, upper

    def apply_mask(self, frame, hsv_lower, hsv_upper):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def save_params(self, filepath):
        data = {k: int(v) for k, v in self.params.items()}
        os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    def load_params(self, filepath):
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        for k, v in data.items():
            if k in self.params:
                self.params[k] = int(v)

    def set_param(self, name, value):
        self.params[name] = value


# ── 测试用例 ──────────────────────────────────────────────────

class TestTrackbarParamManagement(unittest.TestCase):
    """参数管理测试"""

    def test_add_single_trackbar(self):
        gui = ColorCalibrationSimulator()
        gui.add_trackbar("test_param", default=100, max_val=255)
        self.assertEqual(gui.params["test_param"], 100)

    def test_add_hsv_range_default(self):
        gui = ColorCalibrationSimulator()
        gui.add_hsv_range()
        self.assertEqual(gui.params["H_min"], 0)
        self.assertEqual(gui.params["H_max"], 179)
        self.assertEqual(gui.params["S_min"], 0)
        self.assertEqual(gui.params["S_max"], 255)
        self.assertEqual(gui.params["V_min"], 0)
        self.assertEqual(gui.params["V_max"], 255)

    def test_add_hsv_range_with_prefix(self):
        gui = ColorCalibrationSimulator()
        gui.add_hsv_range(prefix="red")
        self.assertIn("red_H_min", gui.params)
        self.assertIn("red_H_max", gui.params)
        self.assertIn("red_S_min", gui.params)

    def test_multiple_hsv_ranges(self):
        gui = ColorCalibrationSimulator()
        gui.add_hsv_range(prefix="red")
        gui.add_hsv_range(prefix="blue")
        self.assertEqual(len(gui.params), 12)  # 6 * 2
        self.assertNotEqual(gui.params["red_H_min"], gui.params["blue_H_min"])


class TestHSVRangeRetrieval(unittest.TestCase):
    """HSV范围获取测试"""

    def test_get_default_range(self):
        gui = ColorCalibrationSimulator()
        gui.add_hsv_range()
        lower, upper = gui.get_hsv_range()
        np.testing.assert_array_equal(lower, [0, 0, 0])
        np.testing.assert_array_equal(upper, [179, 255, 255])

    def test_get_custom_range(self):
        gui = ColorCalibrationSimulator()
        gui.add_hsv_range()
        gui.set_param("H_min", 10)
        gui.set_param("H_max", 30)
        gui.set_param("S_min", 50)
        lower, upper = gui.get_hsv_range()
        np.testing.assert_array_equal(lower, [10, 50, 0])
        np.testing.assert_array_equal(upper, [30, 255, 255])

    def test_get_range_with_prefix(self):
        gui = ColorCalibrationSimulator()
        gui.add_hsv_range(prefix="blue")
        gui.set_param("blue_H_min", 100)
        gui.set_param("blue_H_max", 130)
        lower, upper = gui.get_hsv_range(prefix="blue")
        self.assertEqual(lower[0], 100)
        self.assertEqual(upper[0], 130)


@unittest.skipUnless(_has_cv2, "OpenCV not available")
class TestMaskGeneration(unittest.TestCase):
    """掩码生成测试"""

    def _make_color_frame(self, bgr, size=(100, 100)):
        frame = np.full((size[0], size[1], 3), bgr, dtype=np.uint8)
        return frame

    def test_red_mask(self):
        gui = ColorCalibrationSimulator()
        # 红色BGR帧
        red_frame = self._make_color_frame([0, 0, 255])
        lower = np.array([0, 100, 100])
        upper = np.array([10, 255, 255])
        mask = gui.apply_mask(red_frame, lower, upper)
        self.assertEqual(mask.shape, (100, 100))
        self.assertEqual(mask.dtype, np.uint8)

    def test_mask_all_zeros_when_no_match(self):
        gui = ColorCalibrationSimulator()
        # 黑色帧，HSV低范围匹配
        black_frame = self._make_color_frame([0, 0, 0])
        lower = np.array([0, 100, 100])
        upper = np.array([10, 255, 255])
        mask = gui.apply_mask(black_frame, lower, upper)
        self.assertEqual(np.sum(mask), 0)

    def test_mask_full_when_match(self):
        gui = ColorCalibrationSimulator()
        # 全范围匹配
        frame = self._make_color_frame([128, 128, 128])
        lower = np.array([0, 0, 0])
        upper = np.array([179, 255, 255])
        mask = gui.apply_mask(frame, lower, upper)
        self.assertTrue(np.mean(mask) > 200)


class TestParamSaveLoad(unittest.TestCase):
    """参数保存/加载测试"""

    def test_save_and_load(self):
        gui = ColorCalibrationSimulator()
        gui.add_hsv_range()
        gui.set_param("H_min", 10)
        gui.set_param("H_max", 50)
        gui.set_param("S_min", 80)

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False, mode='w') as f:
            path = f.name

        try:
            gui.save_params(path)

            gui2 = ColorCalibrationSimulator()
            gui2.add_hsv_range()
            gui2.load_params(path)

            self.assertEqual(gui2.params["H_min"], 10)
            self.assertEqual(gui2.params["H_max"], 50)
            self.assertEqual(gui2.params["S_min"], 80)
            self.assertEqual(gui2.params["S_max"], 255)  # 未修改的保持默认
        finally:
            os.unlink(path)

    def test_save_creates_directory(self):
        gui = ColorCalibrationSimulator()
        gui.add_trackbar("test", 42)
        tmpdir = tempfile.mkdtemp()
        path = os.path.join(tmpdir, "subdir", "params.json")
        try:
            gui.save_params(path)
            self.assertTrue(os.path.exists(path))
        finally:
            import shutil
            shutil.rmtree(tmpdir)

    def test_load_only_updates_existing_keys(self):
        gui = ColorCalibrationSimulator()
        gui.add_hsv_range()
        # 保存一个额外的key
        tmpfile = tempfile.NamedTemporaryFile(suffix=".json", delete=False, mode='w')
        json.dump({"H_min": 20, "unknown_key": 999}, tmpfile)
        tmpfile.close()
        try:
            gui.load_params(tmpfile.name)
            self.assertEqual(gui.params["H_min"], 20)
            self.assertNotIn("unknown_key", gui.params)
        finally:
            os.unlink(tmpfile.name)


class TestCallbackMechanism(unittest.TestCase):
    """回调机制测试"""

    def test_callback_invoked(self):
        gui = ColorCalibrationSimulator()
        result = {}
        gui.add_trackbar("param1", 0, 255,
                         callback=lambda n, v: result.update({"name": n, "value": v}))
        # 模拟滑动条变化
        if "param1" in gui._callbacks:
            gui._callbacks["param1"]("param1", 42)
        self.assertEqual(result.get("name"), "param1")
        self.assertEqual(result.get("value"), 42)

    def test_chaining(self):
        gui = ColorCalibrationSimulator()
        ret = gui.add_trackbar("a", 0).add_trackbar("b", 1).add_hsv_range()
        self.assertIs(ret, gui)
        self.assertIn("a", gui.params)
        self.assertIn("H_min", gui.params)


if __name__ == '__main__':
    unittest.main()
