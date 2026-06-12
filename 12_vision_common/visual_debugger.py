"""
视觉调试工具 - 实时显示图像处理中间结果
支持多窗口拼接显示、性能计时、参数面板
"""
import cv2
import numpy as np
import time
from typing import Dict, Optional, Tuple, Any
from collections import OrderedDict


class VisualDebugger:
    """视觉流水线调试器，实时显示各阶段结果"""

    def __init__(self, max_cols=3, cell_size=(320, 240)):
        self._panels: OrderedDict = OrderedDict()
        self._max_cols = max_cols
        self._cell_w, self._cell_h = cell_size
        self._timers: Dict[str, list] = {}
        self._fps_counter = {}
        self._fps_display = {}

    def add_panel(self, name: str, image=None):
        """添加显示面板"""
        self._panels[name] = image
        return self

    def update(self, name: str, image):
        """更新指定面板的图像"""
        self._panels[name] = image

    def remove_panel(self, name: str):
        self._panels.pop(name, None)

    def clear(self):
        self._panels.clear()
        self._timers.clear()

    class Timer:
        """上下文管理器计时器"""
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

    def timer(self, name: str):
        """用法: with debugger.timer("detect"): ... """
        return self.Timer(self, name)

    def get_avg_time(self, name: str) -> float:
        if name in self._timers and self._timers[name]:
            return sum(self._timers[name]) / len(self._timers[name])
        return 0.0

    def _resize_to_cell(self, img):
        if img is None:
            return np.zeros((self._cell_h, self._cell_w, 3), dtype=np.uint8)
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return cv2.resize(img, (self._cell_w, self._cell_h))

    def render(self) -> np.ndarray:
        """将所有面板拼接成一张大图"""
        if not self._panels:
            return np.zeros((self._cell_h, self._cell_w, 3), dtype=np.uint8)

        cells = []
        for name, img in self._panels.items():
            cell = self._resize_to_cell(img)
            # 叠加名称和计时信息
            cv2.putText(cell, name, (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            if name in self._timers and self._timers[name]:
                avg = self.get_avg_time(name)
                cv2.putText(cell, f"{avg:.1f}ms", (5, 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cells.append(cell)

        n = len(cells)
        cols = min(n, self._max_cols)
        rows = (n + cols - 1) // cols

        # 补齐空位
        while len(cells) < rows * cols:
            cells.append(np.zeros((self._cell_h, self._cell_w, 3), dtype=np.uint8))

        grid_rows = []
        for r in range(rows):
            row_cells = cells[r * cols: (r + 1) * cols]
            grid_rows.append(np.hstack(row_cells))
        return np.vstack(grid_rows)

    def show(self, window_name="Visual Debugger", wait_ms=1):
        """渲染并显示"""
        composite = self.render()
        cv2.imshow(window_name, composite)
        return cv2.waitKey(wait_ms) & 0xFF

    def snapshot(self, filepath):
        """保存当前拼接图"""
        composite = self.render()
        cv2.imwrite(filepath, composite)
        print(f"[Debugger] 快照已保存: {filepath}")


class StreamOverlay:
    """视频流叠加信息层"""

    def __init__(self):
        self._texts = []
        self._circles = []
        self._rects = []
        self._lines = []
        self._arrows = []

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
        self._arrows.clear()

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


class FPSCounter:
    """FPS计数器"""

    def __init__(self, window=30):
        self._timestamps = []
        self._window = window
        self._fps = 0

    def tick(self):
        now = time.perf_counter()
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

    def draw(self, frame, pos=(10, 30), color=(0, 255, 0)):
        cv2.putText(frame, f"FPS: {self._fps:.1f}", pos,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return frame


def quick_debug_pipeline(source=0):
    """快速调试流水线演示"""
    debugger = VisualDebugger()
    fps = FPSCounter()
    cap = cv2.VideoCapture(source)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        fps.tick()

        with debugger.timer("resize"):
            small = cv2.resize(frame, (640, 480))

        with debugger.timer("gray"):
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        with debugger.timer("blur"):
            blur = cv2.GaussianBlur(gray, (5, 5), 0)

        with debugger.timer("edge"):
            edges = cv2.Canny(blur, 50, 150)

        with debugger.timer("hsv"):
            hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        fps.draw(small)
        debugger.update("Original", small)
        debugger.update("Gray", gray)
        debugger.update("Blur", blur)
        debugger.update("Edges", edges)
        debugger.update("HSV-H", hsv[:, :, 0])

        key = debugger.show()
        if key == ord("q") or key == 27:
            break
        elif key == ord("s"):
            debugger.snapshot("debug_snapshot.png")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    quick_debug_pipeline(0)
