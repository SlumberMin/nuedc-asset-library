#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GUI设计仿真 - 菜单/按钮/图表/动画/交互
模拟嵌入式系统GUI组件的设计与交互行为
"""

import time
import random
import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Dict, Optional, Callable, Tuple
from collections import deque


class WidgetType(Enum):
    BUTTON = auto()
    MENU = auto()
    LABEL = auto()
    CHART = auto()
    SLIDER = auto()
    TEXTBOX = auto()
    CHECKBOX = auto()
    PROGRESS_BAR = auto()
    ICON = auto()
    PANEL = auto()


class EventType(Enum):
    CLICK = auto()
    HOVER = auto()
    DRAG = auto()
    KEY_PRESS = auto()
    TIMER = auto()
    SCROLL = auto()
    MENU_SELECT = auto()


class AnimationType(Enum):
    NONE = auto()
    FADE_IN = auto()
    FADE_OUT = auto()
    SLIDE_LEFT = auto()
    SLIDE_RIGHT = auto()
    SCALE_UP = auto()
    BOUNCE = auto()
    BLINK = auto()


@dataclass
class Widget:
    """GUI控件"""
    id: str
    widget_type: WidgetType
    x: int
    y: int
    width: int
    height: int
    label: str = ""
    visible: bool = True
    enabled: bool = True
    z_order: int = 0
    animation: AnimationType = AnimationType.NONE
    animation_progress: float = 0.0
    callback: Optional[str] = None
    children: List[str] = field(default_factory=list)
    style: Dict = field(default_factory=dict)


@dataclass
class GUIEvent:
    """GUI事件"""
    event_type: EventType
    widget_id: str
    timestamp: float
    data: Dict = field(default_factory=dict)


@dataclass
class ChartData:
    """图表数据"""
    series_name: str
    x_values: List[float] = field(default_factory=list)
    y_values: List[float] = field(default_factory=list)
    color: str = "#000000"


class AnimationEngine:
    """动画引擎"""

    def __init__(self, fps: int = 30):
        self.fps = fps
        self.frame_time = 1.0 / fps
        self.active_animations: Dict[str, dict] = {}

    def start_animation(self, widget_id: str, anim_type: AnimationType,
                        duration: float, easing: str = "linear"):
        self.active_animations[widget_id] = {
            "type": anim_type, "duration": duration,
            "elapsed": 0.0, "easing": easing, "done": False
        }

    def update(self, dt: float) -> List[str]:
        completed = []
        for wid, anim in self.active_animations.items():
            if anim["done"]:
                continue
            anim["elapsed"] += dt
            progress = min(anim["elapsed"] / anim["duration"], 1.0)
            progress = self._apply_easing(progress, anim["easing"])
            if anim["elapsed"] >= anim["duration"]:
                anim["done"] = True
                completed.append(wid)
        return completed

    @staticmethod
    def _apply_easing(t: float, easing: str) -> float:
        if easing == "ease_in":
            return t * t
        elif easing == "ease_out":
            return 1 - (1 - t) ** 2
        elif easing == "ease_in_out":
            return 2 * t * t if t < 0.5 else 1 - (-2 * t + 2) ** 2 / 2
        return t

    def get_progress(self, widget_id: str) -> float:
        if widget_id not in self.active_animations:
            return 0.0
        a = self.active_animations[widget_id]
        return min(a["elapsed"] / a["duration"], 1.0)


class EventDispatcher:
    """事件分发器"""

    def __init__(self):
        self.handlers: Dict[str, List[Callable]] = {}
        self.event_queue: deque = deque(maxlen=1000)
        self.event_log: List[GUIEvent] = []

    def register(self, event_name: str, handler: Callable):
        self.handlers.setdefault(event_name, []).append(handler)

    def dispatch(self, event: GUIEvent):
        self.event_log.append(event)
        key = f"{event.event_type.name}:{event.widget_id}"
        for h in self.handlers.get(key, []):
            h(event)
        for h in self.handlers.get(event.event_type.name, []):
            h(event)

    def simulate_click(self, widget_id: str):
        self.dispatch(GUIEvent(EventType.CLICK, widget_id, time.time()))

    def simulate_hover(self, widget_id: str):
        self.dispatch(GUIEvent(EventType.HOVER, widget_id, time.time()))

    def get_recent_events(self, n: int = 10) -> List[GUIEvent]:
        return self.event_log[-n:]


class ChartRenderer:
    """图表渲染器"""

    def __init__(self, width: int = 400, height: int = 300):
        self.width = width
        self.height = height
        self.series: Dict[str, ChartData] = {}
        self.grid_visible = True
        self.x_range = (0, 100)
        self.y_range = (0, 100)

    def add_series(self, name: str, color: str = "#0000FF"):
        self.series[name] = ChartData(series_name=name, color=color)

    def add_point(self, series_name: str, x: float, y: float):
        if series_name in self.series:
            s = self.series[series_name]
            s.x_values.append(x)
            s.y_values.append(y)
            if len(s.x_values) > 1000:
                s.x_values = s.x_values[-500:]
                s.y_values = s.y_values[-500:]

    def render_to_buffer(self) -> dict:
        """模拟渲染图表到帧缓冲区"""
        buf = {
            "width": self.width, "height": self.height,
            "grid": self.grid_visible,
            "series_renders": {}
        }
        for name, s in self.series.items():
            points = []
            for x, y in zip(s.x_values, s.y_values):
                px = (x - self.x_range[0]) / (self.x_range[1] - self.x_range[0]) * self.width
                py = self.height - (y - self.y_range[0]) / (self.y_range[1] - self.y_range[0]) * self.height
                points.append((int(px), int(py)))
            buf["series_renders"][name] = {
                "color": s.color, "points": points, "count": len(points)
            }
        return buf

    def auto_range(self, margin: float = 0.1):
        all_x, all_y = [], []
        for s in self.series.values():
            all_x.extend(s.x_values)
            all_y.extend(s.y_values)
        if all_x and all_y:
            dx = (max(all_x) - min(all_x)) * margin or 1
            dy = (max(all_y) - min(all_y)) * margin or 1
            self.x_range = (min(all_x) - dx, max(all_x) + dx)
            self.y_range = (min(all_y) - dy, max(all_y) + dy)


class MenuSystem:
    """菜单系统"""

    def __init__(self):
        self.menus: Dict[str, List[Dict]] = {}
        self.active_menu: Optional[str] = None
        self.selection_history: List[str] = []

    def add_menu(self, name: str, items: List[Dict]):
        self.menus[name] = items

    def open_menu(self, name: str) -> List[Dict]:
        self.active_menu = name
        self.selection_history.append(name)
        return self.menus.get(name, [])

    def navigate_submenu(self, item_label: str) -> List[Dict]:
        for item in self.menus.get(self.active_menu, []):
            if item["label"] == item_label and "submenu" in item:
                return self.open_menu(item["submenu"])
        return []

    def get_breadcrumb(self) -> List[str]:
        return list(self.selection_history)


class GUISimulator:
    """GUI设计仿真器"""

    def __init__(self, screen_width: int = 800, screen_height: int = 600):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.widgets: Dict[str, Widget] = {}
        self.animation_engine = AnimationEngine(fps=30)
        self.event_dispatcher = EventDispatcher()
        self.chart_renderer = ChartRenderer()
        self.menu_system = MenuSystem()
        self.frame_buffer = [[(0, 0, 0)] * screen_width for _ in range(screen_height)]
        self.fps_history: deque = deque(maxlen=100)
        self.current_theme = "light"
        self.themes = {
            "light": {"bg": (255, 255, 255), "fg": (0, 0, 0), "accent": (0, 120, 215)},
            "dark": {"bg": (30, 30, 30), "fg": (200, 200, 200), "accent": (0, 150, 255)},
        }
        self._setup_default_layout()

    def _setup_default_layout(self):
        self.add_widget(Widget("title_bar", WidgetType.PANEL, 0, 0, 800, 40, label="电赛监控系统"))
        self.add_widget(Widget("menu_btn", WidgetType.BUTTON, 5, 5, 60, 30, label="菜单", callback="open_menu"))
        self.add_widget(Widget("status_label", WidgetType.LABEL, 650, 10, 140, 20, label="状态: 正常"))
        self.add_widget(Widget("main_chart", WidgetType.CHART, 20, 60, 500, 340, label="实时数据"))
        self.add_widget(Widget("side_panel", WidgetType.PANEL, 540, 60, 240, 340, label="控制面板"))
        self.add_widget(Widget("start_btn", WidgetType.BUTTON, 560, 80, 100, 35, label="开始", callback="start_action"))
        self.add_widget(Widget("stop_btn", WidgetType.BUTTON, 670, 80, 100, 35, label="停止", callback="stop_action"))
        self.add_widget(Widget("speed_slider", WidgetType.SLIDER, 560, 140, 200, 25, label="速度"))
        self.add_widget(Widget("progress_bar", WidgetType.PROGRESS_BAR, 20, 420, 760, 20, label="进度"))
        self.add_widget(Widget("status_bar", WidgetType.PANEL, 0, 560, 800, 40, label="就绪"))
        self.menu_system.add_menu("main", [
            {"label": "文件", "submenu": "file"},
            {"label": "视图", "submenu": "view"},
            {"label": "工具", "submenu": "tools"},
            {"label": "帮助", "submenu": "help"},
        ])
        self.menu_system.add_menu("file", [
            {"label": "新建", "action": "new"}, {"label": "打开", "action": "open"},
            {"label": "保存", "action": "save"}, {"label": "退出", "action": "exit"},
        ])
        self.menu_system.add_menu("view", [
            {"label": "主题切换", "action": "toggle_theme"},
            {"label": "全屏", "action": "fullscreen"},
        ])

    def add_widget(self, widget: Widget):
        self.widgets[widget.id] = widget
        self.event_dispatcher.register(f"CLICK:{widget.id}", self._default_click_handler)

    def _default_click_handler(self, event: GUIEvent):
        w = self.widgets.get(event.widget_id)
        if w and w.enabled:
            self.animation_engine.start_animation(w.id, AnimationType.SCALE_UP, 0.2, "ease_out")

    def click_widget(self, widget_id: str):
        self.event_dispatcher.simulate_click(widget_id)

    def get_widget(self, widget_id: str) -> Optional[Widget]:
        return self.widgets.get(widget_id)

    def set_visible(self, widget_id: str, visible: bool):
        if widget_id in self.widgets:
            self.widgets[widget_id].visible = visible

    def update_frame(self, dt: float = 0.033):
        self.animation_engine.update(dt)
        fps = 1.0 / max(dt, 0.001)
        self.fps_history.append(fps)

    def render_frame(self) -> dict:
        theme = self.themes[self.current_theme]
        visible_widgets = sorted(
            [w for w in self.widgets.values() if w.visible],
            key=lambda w: w.z_order
        )
        return {
            "screen": (self.screen_width, self.screen_height),
            "theme": self.current_theme,
            "widgets": [{
                "id": w.id, "type": w.widget_type.name,
                "pos": (w.x, w.y), "size": (w.width, w.height),
                "label": w.label, "enabled": w.enabled,
                "animation": self.animation_engine.get_progress(w.id)
            } for w in visible_widgets],
            "fps": sum(self.fps_history) / max(len(self.fps_history), 1),
        }

    def set_theme(self, theme_name: str):
        if theme_name in self.themes:
            self.current_theme = theme_name

    def simulate_interaction_scenario(self, scenario: str = "basic") -> List[dict]:
        results = []
        if scenario == "basic":
            steps = [
                ("click", "menu_btn"), ("menu_select", "main", "文件"),
                ("click", "start_btn"), ("slider", "speed_slider", 0.75),
                ("click", "stop_btn"),
            ]
        elif scenario == "theme_switch":
            steps = [
                ("click", "menu_btn"), ("menu_select", "main", "视图"),
                ("theme", "dark"), ("theme", "light"),
            ]
        else:
            steps = [("click", "start_btn")]

        for step in steps:
            t = step[0]
            if t == "click":
                self.click_widget(step[1])
                results.append({"action": "click", "widget": step[1], "ok": True})
            elif t == "menu_select":
                items = self.menu_system.open_menu(step[1])
                results.append({"action": "menu_open", "items": len(items)})
            elif t == "slider":
                if step[1] in self.widgets:
                    self.widgets[step[1]].style["value"] = step[2]
                    results.append({"action": "slider", "value": step[2]})
            elif t == "theme":
                self.set_theme(step[1])
                results.append({"action": "theme", "name": step[1]})
            self.update_frame(0.033)
        return results


def run_gui_simulation():
    """运行GUI设计仿真"""
    print("=" * 60)
    print("GUI设计仿真 - 菜单/按钮/图表/动画/交互")
    print("=" * 60)

    sim = GUISimulator(800, 600)
    print(f"\n[初始化] 屏幕: {sim.screen_width}x{sim.screen_height}")
    print(f"[初始化] 控件数: {len(sim.widgets)}")

    # 1. 控件树
    print("\n--- 1. 控件树 ---")
    for w in sim.widgets.values():
        print(f"  [{w.widget_type.name:14s}] {w.id:16s} ({w.x},{w.y}) {w.width}x{w.height}  '{w.label}'")

    # 2. 基础交互场景
    print("\n--- 2. 基础交互仿真 ---")
    results = sim.simulate_interaction_scenario("basic")
    for r in results:
        print(f"  {r}")

    # 3. 图表数据填充与渲染
    print("\n--- 3. 图表仿真 ---")
    cr = sim.chart_renderer
    cr.add_series("电压", "#FF0000")
    cr.add_series("电流", "#0000FF")
    for i in range(200):
        cr.add_point("电压", i * 0.1, 3.3 + 0.5 * math.sin(i * 0.1) + random.gauss(0, 0.05))
        cr.add_point("电流", i * 0.1, 1.0 + 0.3 * math.cos(i * 0.1) + random.gauss(0, 0.03))
    cr.auto_range()
    buf = cr.render_to_buffer()
    for name, info in buf["series_renders"].items():
        print(f"  系列 '{name}': {info['count']} 个数据点, 颜色={info['color']}")
    print(f"  X范围: {cr.x_range}, Y范围: {cr.y_range}")

    # 4. 动画引擎
    print("\n--- 4. 动画仿真 ---")
    ae = sim.animation_engine
    ae.start_animation("start_btn", AnimationType.FADE_IN, 0.5, "ease_out")
    ae.start_animation("main_chart", AnimationType.SCALE_UP, 0.8, "ease_in_out")
    for _ in range(30):
        completed = ae.update(0.033)
    print(f"  活跃动画数: {len(ae.active_animations)}")
    for wid, anim in ae.active_animations.items():
        print(f"  [{wid}] 类型={anim['type'].name} 完成={anim['done']}")

    # 5. 主题切换
    print("\n--- 5. 主题切换仿真 ---")
    results = sim.simulate_interaction_scenario("theme_switch")
    for r in results:
        print(f"  {r}")

    # 6. 渲染帧
    print("\n--- 6. 渲染帧 ---")
    frame = sim.render_frame()
    print(f"  屏幕: {frame['screen']}, 主题: {frame['theme']}")
    print(f"  可见控件: {len(frame['widgets'])}, 平均FPS: {frame['fps']:.1f}")

    # 7. 菜单导航
    print("\n--- 7. 菜单导航 ---")
    items = sim.menu_system.open_menu("main")
    print(f"  主菜单: {[i['label'] for i in items]}")
    sub = sim.menu_system.navigate_submenu("文件")
    print(f"  文件子菜单: {[i['label'] for i in sub]}")
    print(f"  面包屑: {sim.menu_system.get_breadcrumb()}")

    # 8. 性能统计
    print("\n--- 8. 性能统计 ---")
    fps_list = list(sim.fps_history)
    if fps_list:
        print(f"  帧数: {len(fps_list)}")
        print(f"  平均FPS: {sum(fps_list)/len(fps_list):.1f}")
        print(f"  最低FPS: {min(fps_list):.1f}")
        print(f"  最高FPS: {max(fps_list):.1f}")

    print("\n" + "=" * 60)
    print("GUI设计仿真完成")
    print("=" * 60)


if __name__ == "__main__":
    run_gui_simulation()
