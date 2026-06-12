"""
物体计数模块 - 检测 + 跟踪 + 计数线 + 统计
功能：基于背景减除/颜色检测实现物体计数
依赖：opencv-python, numpy
适用：电赛中人流计数、产品计数、车辆计数等场景
"""

import cv2
import numpy as np
from collections import OrderedDict
import time
import math

# ============================================================
# 质心跟踪器（Centroid Tracker）
# ============================================================

class CentroidTracker:
    """
    基于质心的多目标跟踪器
    通过欧氏距离关联相邻帧的目标
    """

    def __init__(self, max_disappeared=30, max_distance=80):
        """
        初始化跟踪器
        Args:
            max_disappeared: 目标消失最大帧数（超过则删除）
            max_distance: 目标关联最大距离
        """
        self.next_id = 0
        self.objects = OrderedDict()      # id -> centroid
        self.disappeared = OrderedDict()  # id -> 消失帧数
        self.tracks = OrderedDict()       # id -> 历史轨迹 [(x,y), ...]
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance

    def register(self, centroid):
        """注册新目标"""
        self.objects[self.next_id] = centroid
        self.disappeared[self.next_id] = 0
        self.tracks[self.next_id] = [centroid]
        self.next_id += 1

    def deregister(self, object_id):
        """删除目标"""
        del self.objects[object_id]
        del self.disappeared[object_id]
        del self.tracks[object_id]

    def update(self, rects):
        """
        更新跟踪状态
        Args:
            rects: 当前帧检测到的目标矩形列表 [(x1,y1,x2,y2), ...]
        Returns:
            objects: OrderedDict {id: (cx, cy)}
        """
        # 无检测结果
        if len(rects) == 0:
            for obj_id in list(self.disappeared.keys()):
                self.disappeared[obj_id] += 1
                if self.disappeared[obj_id] > self.max_disappeared:
                    self.deregister(obj_id)
            return self.objects.copy()

        # 计算检测目标质心
        centroids = []
        for (x1, y1, x2, y2) in rects:
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            centroids.append((cx, cy))
        centroids = np.array(centroids)

        # 无已跟踪目标，全部注册
        if len(self.objects) == 0:
            for c in centroids:
                self.register(tuple(c))
            return self.objects.copy()

        # 计算已有目标与新检测的代价矩阵
        obj_ids = list(self.objects.keys())
        obj_centroids = list(self.objects.values())

        D = np.zeros((len(obj_centroids), len(centroids)))
        for i, oc in enumerate(obj_centroids):
            for j, nc in enumerate(centroids):
                D[i, j] = math.hypot(oc[0] - nc[0], oc[1] - nc[1])

        # 匈牙利匹配（贪心近似）
        rows = D.min(axis=1).argsort()
        cols = D.argmin(axis=1)[rows]

        used_rows, used_cols = set(), set()
        for row, col in zip(rows, cols):
            if row in used_rows or col in used_cols:
                continue
            if D[row, col] > self.max_distance:
                continue

            obj_id = obj_ids[row]
            new_centroid = tuple(centroids[col])
            self.objects[obj_id] = new_centroid
            self.disappeared[obj_id] = 0
            self.tracks[obj_id].append(new_centroid)

            used_rows.add(row)
            used_cols.add(col)

        # 未匹配的已有目标
        for row in set(range(len(obj_centroids))) - used_rows:
            obj_id = obj_ids[row]
            self.disappeared[obj_id] += 1
            if self.disappeared[obj_id] > self.max_disappeared:
                self.deregister(obj_id)

        # 未匹配的新检测
        for col in set(range(len(centroids))) - used_cols:
            self.register(tuple(centroids[col]))

        return self.objects.copy()


# ============================================================
# 计数线管理器
# ============================================================

class CountingLine:
    """
    虚拟计数线
    目标穿过计数线时触发计数
    """

    def __init__(self, pt1, pt2, name="line"):
        """
        初始化计数线
        Args:
            pt1: 起点 (x, y)
            pt2: 终点 (x, y)
            name: 计数线名称
        """
        self.pt1 = pt1
        self.pt2 = pt2
        self.name = name
        self.count_up = 0    # 从下到上穿过
        self.count_down = 0  # 从上到下穿过
        self.crossed_ids = set()  # 已穿过的ID（防重复）

    def _side(self, point):
        """计算点相对于计数线的方向（叉积符号）"""
        x, y = point
        x1, y1 = self.pt1
        x2, y2 = self.pt2
        return (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)

    def check_crossing(self, object_id, prev_pos, curr_pos):
        """
        检测目标是否穿过计数线
        Args:
            object_id: 目标ID
            prev_pos: 上一帧位置
            curr_pos: 当前帧位置
        Returns:
            direction: 'up' / 'down' / None
        """
        if prev_pos is None or curr_pos is None:
            return None

        prev_side = self._side(prev_pos)
        curr_side = self._side(curr_pos)

        # 符号不同表示穿过
        if prev_side * curr_side >= 0:
            return None

        # 判断方向
        if curr_side > 0:
            self.count_up += 1
            direction = 'up'
        else:
            self.count_down += 1
            direction = 'down'

        self.crossed_ids.add(object_id)
        return direction

    def get_total(self):
        """获取总计数"""
        return self.count_up + self.count_down

    def reset(self):
        """重置计数"""
        self.count_up = 0
        self.count_down = 0
        self.crossed_ids.clear()

    def draw(self, frame, color=(0, 255, 0), thickness=2):
        """绘制计数线"""
        cv2.line(frame, self.pt1, self.pt2, color, thickness)
        label = f"{self.name}: {self.get_total()}"
        mid_x = (self.pt1[0] + self.pt2[0]) // 2
        mid_y = (self.pt1[1] + self.pt2[1]) // 2
        cv2.putText(frame, label, (mid_x - 40, mid_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


# ============================================================
# 物体计数器（整合检测+跟踪+计数）
# ============================================================

class ObjectCounter:
    """
    物体计数器
    整合背景减除检测、质心跟踪、计数线计数
    """

    def __init__(self, method='background_subtraction'):
        """
        初始化计数器
        Args:
            method: 检测方法 'background_subtraction' / 'color' / 'contour'
        """
        self.method = method
        self.tracker = CentroidTracker(max_disappeared=30, max_distance=80)
        self.counting_lines = []
        self.prev_positions = {}  # id -> 上一帧位置

        # 背景减除器
        if method == 'background_subtraction':
            self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
                history=500, varThreshold=50, detectShadows=True
            )

        # 颜色检测参数（HSV范围）
        self.color_ranges = {
            'red': [(np.array([0, 80, 80]), np.array([10, 255, 255])),
                    (np.array([170, 80, 80]), np.array([180, 255, 255]))],
            'blue': [(np.array([100, 80, 80]), np.array([130, 255, 255]))],
            'green': [(np.array([35, 80, 80]), np.array([85, 255, 255]))],
            'yellow': [(np.array([20, 80, 80]), np.array([35, 255, 255]))],
        }

        # 形态学核
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))

        # 统计
        self.total_counted = 0
        self.stats = {
            'up': 0, 'down': 0,
            'history': [],  # [(timestamp, count), ...]
        }

    def add_counting_line(self, pt1, pt2, name="line"):
        """添加计数线"""
        line = CountingLine(pt1, pt2, name)
        self.counting_lines.append(line)
        return line

    def set_color_target(self, color_name):
        """设置目标颜色"""
        if color_name in self.color_ranges:
            self.target_color = color_name
        else:
            print(f"不支持的颜色: {color_name}, 可选: {list(self.color_ranges.keys())}")

    def _detect_background_subtraction(self, frame):
        """基于背景减除的目标检测"""
        mask = self.bg_subtractor.apply(frame)

        # 去除阴影（MOG2输出中127表示阴影）
        mask[mask == 127] = 0

        # 形态学操作
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)

        return mask

    def _detect_color(self, frame, color_name='red'):
        """基于颜色的目标检测"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        ranges = self.color_ranges.get(color_name, [])
        for (lower, upper) in ranges:
            mask |= cv2.inRange(hsv, lower, upper)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)
        return mask

    def _detect_contour(self, frame):
        """基于边缘的简单目标检测"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        mask = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, self.kernel_close)
        return mask

    def detect_objects(self, frame, color_name=None):
        """
        检测目标
        Args:
            frame: 输入帧
            color_name: 颜色检测时的目标颜色
        Returns:
            rects: 目标矩形列表 [(x1,y1,x2,y2), ...]
            mask: 分割掩码
        """
        if self.method == 'background_subtraction':
            mask = self._detect_background_subtraction(frame)
        elif self.method == 'color':
            color = color_name or getattr(self, 'target_color', 'red')
            mask = self._detect_color(frame, color)
        else:
            mask = self._detect_contour(frame)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        rects = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:  # 面积过滤
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            # 宽高比过滤（去除细长噪声）
            aspect = w / max(h, 1)
            if aspect < 0.2 or aspect > 5.0:
                continue
            rects.append((x, y, x + w, y + h))

        return rects, mask

    def process_frame(self, frame, color_name=None):
        """
        处理一帧图像
        Args:
            frame: 输入BGR图像
            color_name: 目标颜色（颜色模式）
        Returns:
            vis: 可视化图像
            count_info: 计数信息字典
        """
        # 1. 检测
        rects, mask = self.detect_objects(frame, color_name)

        # 2. 跟踪
        objects = self.tracker.update(rects)

        # 3. 保存上一帧位置
        for obj_id, centroid in objects.items():
            if obj_id in self.tracker.tracks and len(self.tracker.tracks[obj_id]) >= 2:
                prev = self.tracker.tracks[obj_id][-2]
                self.prev_positions[obj_id] = prev
            else:
                self.prev_positions[obj_id] = centroid

        # 4. 计数线检测
        for line in self.counting_lines:
            for obj_id in objects:
                prev = self.prev_positions.get(obj_id)
                curr = objects[obj_id]
                direction = line.check_crossing(obj_id, prev, curr)
                if direction:
                    self.total_counted += 1
                    self.stats[direction] += 1

        # 5. 可视化
        vis = frame.copy()

        # 绘制目标框和轨迹
        for obj_id, (cx, cy) in objects.items():
            # 绘制轨迹
            if obj_id in self.tracker.tracks:
                pts = self.tracker.tracks[obj_id]
                for i in range(1, len(pts)):
                    alpha = i / len(pts)
                    color = (0, int(255 * alpha), int(255 * (1 - alpha)))
                    cv2.line(vis, pts[i-1], pts[i], color, 2)

            # 绘制质心
            cv2.circle(vis, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(vis, f"ID:{obj_id}", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 绘制检测框
        for (x1, y1, x2, y2) in rects:
            cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 0, 0), 2)

        # 绘制计数线
        for line in self.counting_lines:
            line.draw(vis)

        # 绘制统计信息
        h, w = frame.shape[:2]
        info_y = 30
        cv2.putText(vis, f"Tracked: {len(objects)}", (10, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(vis, f"Total counted: {self.total_counted}", (10, info_y + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(vis, f"Up: {self.stats['up']}  Down: {self.stats['down']}",
                    (10, info_y + 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 200), 2)

        count_info = {
            'tracked': len(objects),
            'total_counted': self.total_counted,
            'up': self.stats['up'],
            'down': self.stats['down'],
            'objects': dict(objects),
        }

        return vis, count_info


# ============================================================
# 使用示例
# ============================================================

def demo_camera():
    """摄像头实时物体计数演示"""
    counter = ObjectCounter(method='background_subtraction')

    # 添加水平计数线（画面中央）
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    ret, frame = cap.read()
    if ret:
        h, w = frame.shape[:2]
        counter.add_counting_line((0, h // 2), (w, h // 2), "center")
    cap.release()

    # 重新打开
    cap = cv2.VideoCapture(0)
    print("物体计数演示 - 按 q 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        vis, info = counter.process_frame(frame)
        cv2.imshow('Object Counting', vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def demo_video(video_path):
    """视频文件物体计数演示"""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"无法打开视频: {video_path}")
        return

    ret, frame = cap.read()
    if not ret:
        return

    h, w = frame.shape[:2]
    counter = ObjectCounter(method='background_subtraction')
    counter.add_counting_line((0, h * 2 // 3), (w, h * 2 // 3), "line1")

    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        vis, info = counter.process_frame(frame)
        cv2.imshow('Object Counting', vis)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"最终统计: {info}")


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        demo_video(sys.argv[1])
    else:
        demo_camera()
