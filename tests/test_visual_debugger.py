#!/usr/bin/env python3
"""
视觉调试器单元测试
覆盖: 面板管理、计时器、FPS计数、网格渲染、叠加层
注意: 使用纯Python + NumPy + OpenCV模拟（无GUI窗口）
"""

import sys
import os
import unittest
import time
import numpy as np
from collections import OrderedDict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    _has_cv2 = True
except ImportError:
    _has_cv2 = False


# ── 模拟实现 ──────────────────────────────────────────────────

class VisualDebuggerSimulator:
    """视觉调试器模拟（无GUI显示）"""

    def __init__(self, max_cols=3, cell_size=(320, 240)):
        self._panels = OrderedDict()
        self._max_cols = max_cols
        self._cell_w, self._cell_h = cell_size
        self._timers = {}

    def add_panel(self, name, image=None):
        self._panels[name] = image
        return self

    def update(self, name, image):
        self._panels[name] = image

    def remove_panel(self, name):
        self._panels.pop(name, None)

    def clear(self):
        self._panels.clear()
        self._timers.clear()

    class Timer:
        def __init__(self, debugger, name):
            self._dbg = debugger
            self._name = name
            self._start = 0

        def __enter__(self):
            self._start = time.perf_counter()
            return self

        def __exit__(self, *args):
            elapsed = (time.perf_counter() - self._start) * 1000
            if self._name not in self._dbg._timers:
                self._dbg._timers[self._name] = []
            t = self._dbg._timers[self._name]
            t.append(elapsed)
            if len(t) > 30:
                t.pop(0)

    def timer(self, name):
        return self.Timer(self, name)

    def get_avg_time(self, name):
        if name in self._timers and self._timers[name]:
            return sum(self._timers[name]) / len(self._timers[name])
        return 0.0

    def _resize_to_cell(self, img):
        if img is None:
            return np.zeros((self._cell_h, self._cell_w, 3), dtype=np.uint8)
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return cv2.resize(img, (self._cell_w, self._cell_h))

    def render(self):
        if not self._panels:
            return np.zeros((self._cell_h, self._cell_w, 3), dtype=np.uint8)

        cells = []
        for name, img in self._panels.items():
            cell = self._resize_to_cell(img)
            cells.append(cell)

        n = len(cells)
        cols = min(n, self._max_cols)
        rows = (n + cols - 1) // cols

        while len(cells) < rows * cols:
            cells.append(np.zeros((self._cell_h, self._cell_w, 3), dtype=np.uint8))

        grid_rows = []
        for r in range(rows):
            row_cells = cells[r * cols: (r + 1) * cols]
            grid_rows.append(np.hstack(row_cells))
        return np.vstack(grid_rows)


class FPSCounterSimulator:
    """FPS计数器模拟"""

    def __init__(self, window=30):
        self._timestamps = []
        self._window = window
        self._fps = 0

    def tick(self, timestamp=None):
        now = timestamp if timestamp is not None else time.perf_counter()
        self._timestamps.append(now)
        if len(self._timestamps) > self._window:
            self._timestamps.pop(0)
        if len(self._timestamps) >= 2:
            dt = self._timestamps[-1] - self._timestamps[0]
            self._fps = (len(self._timestamps) - 1) / dt if dt > 0 else 0
        return self._fps

    @property
    def fps(self):
        return self._fps


class StreamOverlaySimulator:
    """叠加层模拟"""

    def __init__(self):
        self._texts = []
        self._circles = []
        self._rects = []
        self._lines = []

    def add_text(self, text, pos, color=(0, 255, 0), scale=0.6, thickness=1):
        self._texts.append((text, pos, color, scale, thickness))
        return self

    def add_circle(self, center, radius=5, color=(0, 255, 0), thickness=-1):
        self._circles.append((center, radius, color, thickness))
        return self

    def add_rect(self, pt1, pt2, color=(0, 255, 0), thickness=2):
        self._rects.append((pt1, pt2, color, thickness))
        return self

    def add_line(self, pt1, pt2, color=(0, 255, 0), thickness=2):
        self._lines.append((pt1, pt2, color, thickness))
        return self

    def clear(self):
        self._texts.clear()
        self._circles.clear()
        self._rects.clear()
        self._lines.clear()

    def draw(self, frame):
        vis = frame.copy()
        for t, pos, c, s, th in self._texts:
            cv2.putText(vis, t, pos, cv2.FONT_HERSHEY_SIMPLEX, s, c, th)
        for center, r, c, th in self._circles:
            cv2.circle(vis, center, r, c, th)
        for p1, p2, c, th in self._rects:
            cv2.rectangle(vis, p1, p2, c, th)
        for p1, p2, c, th in self._lines:
            cv2.line(vis, p1, p2, c, th)
        return vis


# ── 测试用例 ──────────────────────────────────────────────────

class TestVisualDebuggerPanelManagement(unittest.TestCase):
    """面板管理测试"""

    def test_add_panel(self):
        dbg = VisualDebuggerSimulator()
        dbg.add_panel("test", np.zeros((100, 100, 3), dtype=np.uint8))
        self.assertIn("test", dbg._panels)

    def test_update_panel(self):
        dbg = VisualDebuggerSimulator()
        dbg.add_panel("test", np.zeros((100, 100, 3), dtype=np.uint8))
        new_img = np.ones((100, 100, 3), dtype=np.uint8) * 255
        dbg.update("test", new_img)
        np.testing.assert_array_equal(dbg._panels["test"], new_img)

    def test_remove_panel(self):
        dbg = VisualDebuggerSimulator()
        dbg.add_panel("a", None)
        dbg.add_panel("b", None)
        dbg.remove_panel("a")
        self.assertNotIn("a", dbg._panels)
        self.assertIn("b", dbg._panels)

    def test_remove_nonexistent_panel(self):
        dbg = VisualDebuggerSimulator()
        dbg.remove_panel("nonexistent")  # 不应报错

    def test_clear(self):
        dbg = VisualDebuggerSimulator()
        dbg.add_panel("a", None)
        dbg.add_panel("b", None)
        dbg._timers["t1"] = [1.0, 2.0]
        dbg.clear()
        self.assertEqual(len(dbg._panels), 0)
        self.assertEqual(len(dbg._timers), 0)

    def test_chaining(self):
        dbg = VisualDebuggerSimulator()
        ret = dbg.add_panel("a", None).add_panel("b", None)
        self.assertIs(ret, dbg)


class TestVisualDebuggerTimer(unittest.TestCase):
    """计时器测试"""

    def test_timer_measures_time(self):
        dbg = VisualDebuggerSimulator()
        with dbg.timer("test"):
            time.sleep(0.01)
        avg = dbg.get_avg_time("test")
        self.assertGreater(avg, 5.0)  # 至少5ms

    def test_timer_multiple_calls(self):
        dbg = VisualDebuggerSimulator()
        for _ in range(5):
            with dbg.timer("test"):
                pass
        self.assertEqual(len(dbg._timers["test"]), 5)

    def test_timer_max_samples(self):
        dbg = VisualDebuggerSimulator()
        for _ in range(35):
            with dbg.timer("test"):
                pass
        self.assertEqual(len(dbg._timers["test"]), 30)

    def test_avg_time_empty(self):
        dbg = VisualDebuggerSimulator()
        self.assertEqual(dbg.get_avg_time("nonexistent"), 0.0)

    def test_multiple_timer_names(self):
        dbg = VisualDebuggerSimulator()
        with dbg.timer("detect"):
            time.sleep(0.001)
        with dbg.timer("render"):
            time.sleep(0.001)
        self.assertIn("detect", dbg._timers)
        self.assertIn("render", dbg._timers)


@unittest.skipUnless(_has_cv2, "OpenCV not available")
class TestVisualDebuggerRender(unittest.TestCase):
    """渲染测试"""

    def test_render_empty(self):
        dbg = VisualDebuggerSimulator()
        result = dbg.render()
        self.assertEqual(result.shape, (240, 320, 3))

    def test_render_single_panel(self):
        dbg = VisualDebuggerSimulator(max_cols=3)
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        dbg.add_panel("test", img)
        result = dbg.render()
        self.assertEqual(result.shape[0], 240)
        self.assertEqual(result.shape[1], 320)

    def test_render_multiple_panels_grid(self):
        dbg = VisualDebuggerSimulator(max_cols=2, cell_size=(160, 120))
        for i in range(4):
            dbg.add_panel(f"panel_{i}", np.zeros((50, 50, 3), dtype=np.uint8))
        result = dbg.render()
        # 4个面板，2列 -> 2行
        self.assertEqual(result.shape, (240, 320, 3))

    def test_render_gray_image(self):
        dbg = VisualDebuggerSimulator()
        gray = np.zeros((100, 100), dtype=np.uint8)
        dbg.add_panel("gray", gray)
        result = dbg.render()
        self.assertEqual(len(result.shape), 3)
        self.assertEqual(result.shape[2], 3)

    def test_render_none_panel(self):
        dbg = VisualDebuggerSimulator()
        dbg.add_panel("empty", None)
        result = dbg.render()
        # None应渲染为黑色
        self.assertEqual(result.shape, (240, 320, 3))


class TestFPSCounter(unittest.TestCase):
    """FPS计数器测试"""

    def test_initial_fps_zero(self):
        fps = FPSCounterSimulator()
        self.assertEqual(fps.fps, 0)

    def test_fps_after_ticks(self):
        fps = FPSCounterSimulator()
        base = time.perf_counter()
        for i in range(30):
            fps.tick(base + i * 0.033)  # ~30fps
        self.assertAlmostEqual(fps.fps, 30.0, delta=5.0)

    def test_fps_window_limit(self):
        fps = FPSCounterSimulator(window=10)
        base = time.perf_counter()
        for i in range(20):
            fps.tick(base + i * 0.033)
        # 窗口限制为10
        self.assertLessEqual(len(fps._timestamps), 10)

    def test_single_tick_no_fps(self):
        fps = FPSCounterSimulator()
        fps.tick()
        self.assertEqual(fps.fps, 0)


@unittest.skipUnless(_has_cv2, "OpenCV not available")
class TestStreamOverlay(unittest.TestCase):
    """叠加层测试"""

    def test_draw_modifies_frame(self):
        overlay = StreamOverlaySimulator()
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        overlay.add_text("test", (10, 10))
        result = overlay.draw(frame)
        # 原始帧不应被修改
        self.assertEqual(np.sum(frame), 0)
        # 结果应与原始帧不同
        self.assertFalse(np.array_equal(frame, result))

    def test_draw_circle(self):
        overlay = StreamOverlaySimulator()
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        overlay.add_circle((50, 50), radius=10, color=(0, 0, 255), thickness=-1)
        result = overlay.draw(frame)
        # 圆心区域应有红色像素
        self.assertTrue(result[50, 50, 2] > 0)

    def test_draw_rect(self):
        overlay = StreamOverlaySimulator()
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        overlay.add_rect((10, 10), (50, 50), color=(0, 255, 0))
        result = overlay.draw(frame)
        self.assertTrue(np.sum(result) > 0)

    def test_clear(self):
        overlay = StreamOverlaySimulator()
        overlay.add_text("a", (0, 0))
        overlay.add_circle((10, 10))
        overlay.add_rect((0, 0), (5, 5))
        overlay.add_line((0, 0), (10, 10))
        overlay.clear()
        self.assertEqual(len(overlay._texts), 0)
        self.assertEqual(len(overlay._circles), 0)
        self.assertEqual(len(overlay._rects), 0)
        self.assertEqual(len(overlay._lines), 0)

    def test_chaining(self):
        overlay = StreamOverlaySimulator()
        ret = overlay.add_text("a", (0, 0)).add_circle((10, 10)).add_rect((0, 0), (5, 5))
        self.assertIs(ret, overlay)


if __name__ == '__main__':
    unittest.main()
