"""
轨迹预测器 - 线性/抛物线/样条插值预测
适用于弹道预测、运动轨迹补全、目标落点预估
"""
import numpy as np
from typing import List, Tuple, Optional
from collections import deque


class TrajectoryPredictor:
    """轨迹预测器，支持多种预测模型"""

    def __init__(self, max_history=50):
        self._history_x = deque(maxlen=max_history)
        self._history_y = deque(maxlen=max_history)
        self._history_t = deque(maxlen=max_history)
        self._t_counter = 0

    def add_point(self, x: float, y: float, t: Optional[float] = None):
        """添加观测点"""
        self._history_x.append(x)
        self._history_y.append(y)
        self._history_t.append(t if t is not None else self._t_counter)
        self._t_counter += 1

    @property
    def count(self):
        return len(self._history_x)

    def clear(self):
        self._history_x.clear()
        self._history_y.clear()
        self._history_t.clear()
        self._t_counter = 0

    def predict_linear(self, steps=10) -> List[Tuple[float, float]]:
        """
        线性预测: x = a*t + b, y = c*t + d
        适用于匀速直线运动
        """
        n = self.count
        if n < 2:
            return []

        t = np.array(self._history_t, dtype=np.float64)
        x = np.array(self._history_x, dtype=np.float64)
        y = np.array(self._history_y, dtype=np.float64)

        # 最小二乘拟合
        if n == 2:
            vx = (x[1] - x[0]) / (t[1] - t[0] + 1e-9)
            vy = (y[1] - y[0]) / (t[1] - t[0] + 1e-9)
            ax, bx = vx, x[-1] - vx * t[-1]
            ay, by = vy, y[-1] - vy * t[-1]
        else:
            coeffs_x = np.polyfit(t, x, 1)
            coeffs_y = np.polyfit(t, y, 1)
            ax, bx = coeffs_x
            ay, by = coeffs_y

        predictions = []
        t_last = t[-1]
        dt = (t[-1] - t[0]) / max(n - 1, 1)
        for i in range(1, steps + 1):
            t_new = t_last + dt * i
            predictions.append((
                float(ax * t_new + bx),
                float(ay * t_new + by)
            ))
        return predictions

    def predict_parabolic(self, steps=10) -> List[Tuple[float, float]]:
        """
        抛物线预测: x = a*t + b, y = a2*t^2 + b2*t + c2
        适用于重力作用下的抛体运动
        """
        n = self.count
        if n < 3:
            return self.predict_linear(steps)

        t = np.array(self._history_t, dtype=np.float64)
        x = np.array(self._history_x, dtype=np.float64)
        y = np.array(self._history_y, dtype=np.float64)

        # x方向线性拟合, y方向二次拟合
        coeffs_x = np.polyfit(t, x, 1)
        coeffs_y = np.polyfit(t, y, 2)

        predictions = []
        t_last = t[-1]
        dt = (t[-1] - t[0]) / max(n - 1, 1)
        for i in range(1, steps + 1):
            t_new = t_last + dt * i
            px = float(np.polyval(coeffs_x, t_new))
            py = float(np.polyval(coeffs_y, t_new))
            predictions.append((px, py))
        return predictions

    def predict_spline(self, steps=10, smoothing=None) -> List[Tuple[float, float]]:
        """
        样条插值预测（使用numpy多项式拟合代替scipy）
        """
        n = self.count
        if n < 4:
            return self.predict_parabolic(steps)

        t = np.array(self._history_t, dtype=np.float64)
        x = np.array(self._history_x, dtype=np.float64)
        y = np.array(self._history_y, dtype=np.float64)

        degree = min(3, n - 1)
        coeffs_x = np.polyfit(t, x, degree)
        coeffs_y = np.polyfit(t, y, degree)

        predictions = []
        t_last = t[-1]
        dt = (t[-1] - t[0]) / max(n - 1, 1)
        for i in range(1, steps + 1):
            t_new = t_last + dt * i
            px = float(np.polyval(coeffs_x, t_new))
            py = float(np.polyval(coeffs_y, t_new))
            predictions.append((px, py))
        return predictions

    def predict(self, method="parabolic", steps=10) -> List[Tuple[float, float]]:
        """统一预测接口"""
        if method == "linear":
            return self.predict_linear(steps)
        elif method == "parabolic":
            return self.predict_parabolic(steps)
        elif method == "spline":
            return self.predict_spline(steps)
        else:
            raise ValueError(f"未知预测方法: {method}")

    def predict_intercept(self, target_y: float,
                          method="parabolic") -> Optional[float]:
        """预测轨迹与水平线 y=target_y 的交点x坐标"""
        preds = self.predict(method=method, steps=50)
        hist = list(zip(self._history_x, self._history_y))
        all_pts = hist + preds

        for i in range(1, len(all_pts)):
            y0, y1 = all_pts[i - 1][1], all_pts[i][1]
            if (y0 - target_y) * (y1 - target_y) <= 0:
                # 线性插值
                ratio = (target_y - y0) / (y1 - y0 + 1e-9)
                x_target = all_pts[i - 1][0] + ratio * (all_pts[i][0] - all_pts[i - 1][0])
                return float(x_target)
        return None

    def draw(self, frame, method="parabolic", steps=20,
             history_color=(0, 255, 0), pred_color=(0, 0, 255)):
        """在帧上绘制历史轨迹和预测轨迹"""
        # 历史轨迹
        pts = list(zip(self._history_x, self._history_y))
        for i in range(1, len(pts)):
            p1 = (int(pts[i - 1][0]), int(pts[i - 1][1]))
            p2 = (int(pts[i][0]), int(pts[i][1]))
            cv2.line(frame, p1, p2, history_color, 2)

        # 预测轨迹
        preds = self.predict(method=method, steps=steps)
        if preds and pts:
            all_pred = [pts[-1]] + preds
            for i in range(1, len(all_pred)):
                p1 = (int(all_pred[i - 1][0]), int(all_pred[i - 1][1]))
                p2 = (int(all_pred[i][0]), int(all_pred[i][1]))
                cv2.line(frame, p1, p2, pred_color, 2, cv2.LINE_AA)
            # 终点标记
            end = (int(preds[-1][0]), int(preds[-1][1]))
            cv2.circle(frame, end, 6, pred_color, -1)

        return frame


# 需要在draw中使用cv2
import cv2


class MultiPointPredictor:
    """多目标轨迹预测管理"""

    def __init__(self, max_history=50):
        self._predictors = {}

    def add_point(self, obj_id, x, y, t=None):
        if obj_id not in self._predictors:
            self._predictors[obj_id] = TrajectoryPredictor()
        self._predictors[obj_id].add_point(x, y, t)

    def predict(self, obj_id, method="parabolic", steps=10):
        if obj_id in self._predictors:
            return self._predictors[obj_id].predict(method, steps)
        return []

    def get_predictor(self, obj_id) -> Optional[TrajectoryPredictor]:
        return self._predictors.get(obj_id)

    def remove(self, obj_id):
        self._predictors.pop(obj_id, None)

    def draw_all(self, frame, method="parabolic", steps=10):
        colors = [(0, 255, 0), (0, 0, 255), (255, 0, 0),
                  (0, 255, 255), (255, 0, 255)]
        for i, (oid, pred) in enumerate(self._predictors.items()):
            c = colors[i % len(colors)]
            pred.draw(frame, method=method, steps=steps,
                      history_color=c, pred_color=c)
        return frame


if __name__ == "__main__":
    # 演示: 模拟抛体运动
    pred = TrajectoryPredictor()
    for i in range(15):
        t = i * 0.1
        x = 100 + 50 * t
        y = 300 - 100 * t + 0.5 * 9.8 * t * t * 10  # 模拟重力
        pred.add_point(x, y, t)

    linear = pred.predict_linear(10)
    parab = pred.predict_parabolic(10)
    print(f"线性预测终点: {linear[-1] if linear else 'N/A'}")
    print(f"抛物线预测终点: {parab[-1] if parab else 'N/A'}")
    print(f"与y=300交点: {pred.predict_intercept(300)}")
