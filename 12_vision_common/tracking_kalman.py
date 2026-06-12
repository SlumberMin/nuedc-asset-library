"""
Kalman跟踪器 - 支持匀速模型和匀加速模型
适用于目标跟踪、坐标平滑、运动预测
"""
import numpy as np
import cv2
from typing import Optional, Tuple, List
from collections import deque


class KalmanTracker:
    """基于OpenCV的Kalman滤波跟踪器"""

    # 匀速模型: 状态=[x, y, vx, vy], 观测=[x, y]
    # 匀加速模型: 状态=[x, y, vx, vy, ax, ay], 观测=[x, y]

    def __init__(self, model="constant_velocity", dt=1.0,
                 process_noise=1e-2, measure_noise=1e-1):
        """
        Args:
            model: "constant_velocity" 或 "constant_acceleration"
            dt: 时间步长
            process_noise: 过程噪声协方差系数
            measure_noise: 测量噪声协方差系数
        """
        self.model = model
        self.dt = dt
        self._age = 0
        self._hits = 0
        self._misses = 0
        self._history = deque(maxlen=50)
        self._initialized = False

        if model == "constant_velocity":
            self._init_cv(dt, process_noise, measure_noise)
        elif model == "constant_acceleration":
            self._init_ca(dt, process_noise, measure_noise)
        else:
            raise ValueError(f"未知模型: {model}")

    def _init_cv(self, dt, q, r):
        """匀速模型初始化"""
        self.kf = cv2.KalmanFilter(4, 2)
        # 状态转移矩阵
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        # 观测矩阵
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * q
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * r
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)

    def _init_ca(self, dt, q, r):
        """匀加速模型初始化"""
        self.kf = cv2.KalmanFilter(6, 2)
        dt2 = 0.5 * dt * dt
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0,  dt2, 0],
            [0, 1, 0,  dt, 0,  dt2],
            [0, 0, 1,  0,  dt,  0],
            [0, 0, 0,  1,  0,   dt],
            [0, 0, 0,  0,  1,   0],
            [0, 0, 0,  0,  0,   1]
        ], dtype=np.float32)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0]
        ], dtype=np.float32)
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * q
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * r
        self.kf.errorCovPost = np.eye(6, dtype=np.float32)

    def init(self, x: float, y: float):
        """用初始观测值初始化跟踪器"""
        if self.model == "constant_velocity":
            self.kf.statePost = np.array(
                [[x], [y], [0], [0]], dtype=np.float32)
        else:
            self.kf.statePost = np.array(
                [[x], [y], [0], [0], [0], [0]], dtype=np.float32)
        self._initialized = True
        self._history.append((x, y))

    @property
    def position(self) -> Tuple[float, float]:
        """当前估计位置"""
        s = self.kf.statePost
        return float(s[0, 0]), float(s[1, 0])

    @property
    def velocity(self) -> Tuple[float, float]:
        """当前估计速度"""
        s = self.kf.statePost
        return float(s[2, 0]), float(s[3, 0])

    @property
    def acceleration(self) -> Tuple[float, float]:
        """当前估计加速度(仅匀加速模型)"""
        if self.model != "constant_acceleration":
            raise AttributeError("仅匀加速模型支持加速度查询")
        s = self.kf.statePost
        return float(s[4, 0]), float(s[5, 0])

    @property
    def age(self):
        return self._age

    @property
    def hits(self):
        return self._hits

    def predict(self) -> Tuple[float, float]:
        """预测下一步位置"""
        pred = self.kf.predict()
        self._age += 1
        return float(pred[0, 0]), float(pred[1, 0])

    def update(self, x: float, y: float) -> Tuple[float, float]:
        """用观测值更新，返回滤波后的位置"""
        if not self._initialized:
            self.init(x, y)
            return x, y
        measurement = np.array([[np.float32(x)], [np.float32(y)]])
        self.kf.correct(measurement)
        self._hits += 1
        self._misses = 0
        pos = self.position
        self._history.append(pos)
        return pos

    def predict_only(self) -> Tuple[float, float]:
        """仅预测不更新(目标丢失时调用)"""
        self._misses += 1
        return self.predict()

    def get_smoothed_position(self, window=5) -> Tuple[float, float]:
        """获取历史轨迹的平滑位置"""
        if len(self._history) == 0:
            return self.position
        n = min(window, len(self._history))
        pts = list(self._history)[-n:]
        xs = sum(p[0] for p in pts) / n
        ys = sum(p[1] for p in pts) / n
        return xs, ys

    def get_history(self) -> List[Tuple[float, float]]:
        """获取轨迹历史"""
        return list(self._history)

    def draw(self, frame, color=(0, 255, 0), trail=True):
        """在帧上绘制跟踪结果"""
        cx, cy = self.position
        cx, cy = int(cx), int(cy)
        cv2.circle(frame, (cx, cy), 8, color, -1)
        vx, vy = self.velocity
        cv2.arrowedLine(frame, (cx, cy),
                        (cx + int(vx * 10), cy + int(vy * 10)),
                        color, 2, tipLength=0.3)
        if trail and len(self._history) > 1:
            pts = [(int(p[0]), int(p[1])) for p in self._history]
            for i in range(1, len(pts)):
                alpha = i / len(pts)
                c = tuple(int(v * alpha) for v in color)
                cv2.line(frame, pts[i - 1], pts[i], c, 2)
        return frame


class MultiTargetTracker:
    """多目标Kalman跟踪管理器"""

    def __init__(self, max_misses=10, iou_threshold=0.3,
                 model="constant_velocity"):
        self.trackers: List[KalmanTracker] = []
        self.max_misses = max_misses
        self.iou_threshold = iou_threshold
        self.model = model
        self._next_id = 0
        self._ids: List[int] = []

    def update(self, detections: List[Tuple[float, float]]):
        """
        用新的检测结果更新跟踪器
        detections: [(x, y), ...] 或 [(x, y, w, h), ...]
        """
        # 预测所有已有跟踪器
        for trk in self.trackers:
            trk.predict()

        if len(self.trackers) == 0:
            # 初始化新跟踪器
            for det in detections:
                self._add_tracker(det[0], det[1])
            return

        if len(detections) == 0:
            # 无检测，全部predict_only
            for trk in self.trackers:
                trk.predict_only()
            self._remove_dead()
            return

        # 简单最近邻匹配
        det_arr = np.array([(d[0], d[1]) for d in detections])
        trk_arr = np.array([trk.position for trk in self.trackers])

        cost = np.zeros((len(self.trackers), len(detentions := detections)))
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
            if cost[i, j] > 200:  # 距离阈值
                break
            self.trackers[i].update(det_arr[j, 0], det_arr[j, 1])
            matched_trk.add(i)
            matched_det.add(j)
            cost[i, :] = float("inf")
            cost[:, j] = float("inf")

        # 未匹配的跟踪器
        for i in range(len(self.trackers)):
            if i not in matched_trk:
                self.trackers[i].predict_only()

        # 未匹配的检测 -> 新跟踪器
        for j in range(len(detections)):
            if j not in matched_det:
                self._add_tracker(det_arr[j, 0], det_arr[j, 1])

        self._remove_dead()

    def _add_tracker(self, x, y):
        trk = KalmanTracker(model=self.model)
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
        """获取所有活跃轨迹"""
        return [
            {"id": tid, "pos": trk.position, "vel": trk.velocity,
             "age": trk.age, "hits": trk.hits}
            for trk, tid in zip(self.trackers, self._ids)
        ]

    def draw(self, frame):
        colors = [(0, 255, 0), (0, 0, 255), (255, 0, 0),
                  (0, 255, 255), (255, 0, 255), (255, 255, 0)]
        for trk, tid in zip(self.trackers, self._ids):
            c = colors[tid % len(colors)]
            trk.draw(frame, color=c)
            cx, cy = trk.position
            cv2.putText(frame, f"ID:{tid}", (int(cx) + 10, int(cy)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
        return frame


if __name__ == "__main__":
    # 简单演示: 单目标跟踪
    tracker = KalmanTracker(model="constant_velocity")
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        # 用简单的颜色检测模拟检测结果
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 100, 100]),
                           np.array([10, 255, 255]))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
                tracker.update(cx, cy)
            else:
                tracker.predict_only()
        else:
            tracker.predict_only()

        tracker.draw(frame)
        cv2.putText(frame, f"vel={tracker.velocity}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.imshow("Kalman Track", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
