"""
DeepSORT 目标跟踪 V2
匈牙利匹配 + 级联匹配 + 外观特征（Re-ID）
适用于电赛中的多目标跟踪场景
"""

import cv2
import numpy as np
from collections import defaultdict
from scipy.optimize import linear_sum_assignment


# ============================================================
# 卡尔曼滤波器 - 用于目标状态预测
# ============================================================
class KalmanFilter:
    """简化的卡尔曼滤波器，跟踪 [x, y, w, h, vx, vy, vw, vh]"""

    def __init__(self):
        # 状态维度8，观测维度4
        self.dim_x = 8
        self.dim_z = 4

        # 状态转移矩阵 (匀速模型)
        self.F = np.eye(self.dim_x)
        self.F[:4, 4:] = np.eye(4)

        # 观测矩阵
        self.H = np.eye(self.dim_z, self.dim_x)

        # 过程噪声
        self.Q = np.eye(self.dim_x) * 1.0
        self.Q[4:, 4:] *= 0.01

        # 观测噪声
        self.R = np.eye(self.dim_z) * 10.0

        # 状态协方差
        self.P = np.eye(self.dim_x) * 100.0

        self.x = np.zeros((self.dim_x, 1))

    def predict(self):
        """预测下一时刻状态"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:4].flatten()

    def update(self, z):
        """用观测值更新状态"""
        z = np.array(z).reshape(-1, 1)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(self.dim_x) - K @ self.H) @ self.P

    def get_state(self):
        """返回当前状态 [x, y, w, h]"""
        return self.x[:4].flatten()


# ============================================================
# 外观特征提取器 - 简化版 Re-ID
# ============================================================
class AppearanceFeatureExtractor:
    """
    提取目标的外观特征向量，用于重识别
    使用颜色直方图 + HOG特征的轻量级方案
    """

    def __init__(self, feature_dim=64):
        self.feature_dim = feature_dim
        # HOG特征提取器
        self.hog = cv2.HOGDescriptor(
            (64, 64),   # winSize
            (16, 16),   # blockSize
            (8, 8),     # blockStride
            (8, 8),     # cellSize
            9            # nbins
        )

    def extract(self, image, bbox):
        """
        从图像中提取目标区域的外观特征
        参数:
            image: 原始图像 (BGR)
            bbox: [x1, y1, x2, y2] 边界框
        返回:
            归一化的特征向量 (feature_dim,)
        """
        x1, y1, x2, y2 = [int(v) for v in bbox]
        h, w = image.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)

        if x2 <= x1 or y2 <= y1:
            return np.zeros(self.feature_dim)

        crop = image[y1:y2, x1:x2]

        # 颜色直方图特征 (HSV空间)
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hist_h = cv2.calcHist([hsv], [0], None, [16], [0, 180]).flatten()
        hist_s = cv2.calcHist([hsv], [1], None, [16], [0, 256]).flatten()
        hist_v = cv2.calcHist([hsv], [2], None, [16], [0, 256]).flatten()
        color_feat = np.concatenate([hist_h, hist_s, hist_v])  # 48维

        # HOG特征
        resized = cv2.resize(crop, (64, 64))
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        hog_feat = self.hog.compute(gray).flatten()  # 可变长度

        # 截断/填充到固定长度
        hog_feat = hog_feat[:max(0, self.feature_dim - len(color_feat))]
        if len(hog_feat) < (self.feature_dim - len(color_feat)):
            hog_feat = np.pad(hog_feat, (0, self.feature_dim - len(color_feat) - len(hog_feat)))

        feat = np.concatenate([color_feat, hog_feat])[:self.feature_dim]

        # L2归一化
        norm = np.linalg.norm(feat)
        if norm > 0:
            feat = feat / norm

        return feat

    def compute_distance(self, feat1, feat2):
        """计算两个特征向量之间的余弦距离 (0~2, 越小越相似)"""
        dot = np.dot(feat1, feat2)
        norm1 = np.linalg.norm(feat1)
        norm2 = np.linalg.norm(feat2)
        if norm1 == 0 or norm2 == 0:
            return 1.0
        cosine_sim = dot / (norm1 * norm2)
        return 1.0 - cosine_sim


# ============================================================
# 单目标跟踪轨迹
# ============================================================
class Track:
    """单个目标的跟踪轨迹"""

    def __init__(self, track_id, bbox, feature, class_id=-1):
        self.track_id = track_id
        self.bbox = bbox                    # [x1, y1, x2, y2]
        self.feature = feature              # 外观特征
        self.class_id = class_id            # 类别ID
        self.hits = 1                       # 匹配成功次数
        self.age = 0                        # 轨迹年龄（帧数）
        self.time_since_update = 0          # 距上次更新的帧数
        self.kf = KalmanFilter()            # 卡尔曼滤波器
        self.state = 'tentative'            # tentative / confirmed / deleted

        # 初始化卡尔曼滤波器状态
        cx = (bbox[0] + bbox[2]) / 2
        cy = (bbox[1] + bbox[3]) / 2
        w = bbox[2] - bbox[0]
        h = bbox[3] - bbox[1]
        self.kf.x = np.array([[cx], [cy], [w], [h], [0], [0], [0], [0]], dtype=float)

        # 特征历史（用于级联匹配）
        self.features = [feature]

    def predict(self):
        """预测下一帧位置"""
        self.kf.predict()
        state = self.kf.get_state()
        cx, cy, w, h = state
        self.bbox = [cx - w/2, cy - h/2, cx + w/2, cy + h/2]
        self.age += 1
        self.time_since_update += 1
        return self.bbox

    def update(self, bbox, feature, class_id=-1):
        """用新的检测结果更新轨迹"""
        cx = (bbox[0] + bbox[2]) / 2
        cy = (bbox[1] + bbox[3]) / 2
        w = bbox[2] - bbox[0]
        h = bbox[3] - bbox[1]
        self.kf.update([cx, cy, w, h])

        state = self.kf.get_state()
        cx, cy, w, h = state
        self.bbox = [cx - w/2, cy - h/2, cx + w/2, cy + h/2]

        self.feature = feature
        self.features.append(feature)
        if len(self.features) > 100:
            self.features.pop(0)

        self.hits += 1
        self.time_since_update = 0
        self.class_id = class_id

        # 状态转移
        if self.state == 'tentative' and self.hits >= 3:
            self.state = 'confirmed'

    def mark_missed(self):
        """标记为未匹配"""
        if self.state == 'tentative':
            self.state = 'deleted'
        elif self.time_since_update > 30:
            self.state = 'deleted'

    def is_confirmed(self):
        return self.state == 'confirmed'


# ============================================================
# IoU 计算
# ============================================================
def compute_iou_matrix(boxes1, boxes2):
    """计算两组边界框之间的IoU矩阵"""
    if len(boxes1) == 0 or len(boxes2) == 0:
        return np.zeros((len(boxes1), len(boxes2)))

    boxes1 = np.array(boxes1)
    boxes2 = np.array(boxes2)

    x1 = np.maximum(boxes1[:, 0:1], boxes2[:, 0])
    y1 = np.maximum(boxes1[:, 1:2], boxes2[:, 1])
    x2 = np.minimum(boxes1[:, 2:3], boxes2[:, 2])
    y2 = np.minimum(boxes1[:, 3:4], boxes2[:, 3])

    inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    area1 = (boxes1[:, 2] - boxes1[:, 0]) * (boxes1[:, 3] - boxes1[:, 1])
    area2 = (boxes2[:, 2] - boxes2[:, 0]) * (boxes2[:, 3] - boxes2[:, 1])
    union = area1[:, None] + area2 - inter

    return inter / np.maximum(union, 1e-6)


# ============================================================
# DeepSORT 核心跟踪器
# ============================================================
class DeepSORTTracker:
    """
    DeepSORT 多目标跟踪器
    结合运动信息（IoU + 卡尔曼滤波）和外观特征（Re-ID）
    
    匹配流程:
    1. 级联匹配：按轨迹"丢失时长"分层，优先匹配丢失时间短的轨迹
    2. IoU匹配：对级联未匹配的轨迹和检测做IoU关联
    3. 创建新轨迹：对未匹配的检测创建新轨迹
    """

    def __init__(self, max_age=30, n_init=3, iou_threshold=0.3,
                 appearance_weight=0.5, max_cosine_distance=0.4):
        """
        参数:
            max_age: 轨迹最大丢失帧数，超过则删除
            n_init: 轨迹确认所需的最小匹配次数
            iou_threshold: IoU匹配阈值
            appearance_weight: 外观特征在融合代价中的权重 (0~1)
            max_cosine_distance: 外观特征最大余弦距离
        """
        self.max_age = max_age
        self.n_init = n_init
        self.iou_threshold = iou_threshold
        self.appearance_weight = appearance_weight
        self.max_cosine_distance = max_cosine_distance

        self.tracks = []          # 当前所有轨迹
        self.next_id = 1          # 下一个轨迹ID
        self.frame_count = 0
        self.feature_extractor = AppearanceFeatureExtractor(feature_dim=64)

    def _gating_distance(self, track, detections):
        """
        计算轨迹与各检测之间的融合代价
        代价 = (1 - w) * iou_cost + w * appearance_cost
        """
        track_bbox = np.array(track.bbox).reshape(1, -1)
        det_bboxes = np.array([d['bbox'] for d in detections])

        # IoU代价 (1 - IoU)
        iou_matrix = compute_iou_matrix(track_bbox, det_bboxes)
        iou_cost = 1.0 - iou_matrix[0]  # shape: (N,)

        # 外观代价
        app_cost = np.zeros(len(detections))
        for i, det in enumerate(detections):
            app_cost[i] = self.feature_extractor.compute_distance(
                track.feature, det['feature']
            )

        # 融合代价
        w = self.appearance_weight
        cost = (1 - w) * iou_cost + w * app_cost

        # 外观距离过大时设为无穷
        cost[app_cost > self.max_cosine_distance] = 1e5

        return cost

    def _hungarian_match(self, cost_matrix, threshold):
        """
        匈牙利算法匹配
        返回: (matched_pairs, unmatched_tracks, unmatched_detections)
        """
        if cost_matrix.size == 0:
            return [], list(range(cost_matrix.shape[0])), list(range(cost_matrix.shape[1]))

        row_indices, col_indices = linear_sum_assignment(cost_matrix)

        matched = []
        unmatched_tracks = list(range(cost_matrix.shape[0]))
        unmatched_dets = list(range(cost_matrix.shape[1]))

        for r, c in zip(row_indices, col_indices):
            if cost_matrix[r, c] <= threshold:
                matched.append((r, c))
                unmatched_tracks.remove(r)
                unmatched_dets.remove(c)

        return matched, unmatched_tracks, unmatched_dets

    def update(self, detections, frame=None):
        """
        用新的检测结果更新跟踪器
        
        参数:
            detections: 检测结果列表，每项为 dict:
                {'bbox': [x1,y1,x2,y2], 'conf': float, 'class_id': int}
            frame: 当前帧图像（用于提取外观特征，可选）
        
        返回:
            active_tracks: list of dict, 包含 track_id, bbox, class_id
        """
        self.frame_count += 1

        # 提取检测的外观特征
        for det in detections:
            if frame is not None:
                det['feature'] = self.feature_extractor.extract(frame, det['bbox'])
            else:
                det['feature'] = np.random.randn(64)
                det['feature'] /= np.linalg.norm(det['feature'])

        # ---- 步骤1: 所有轨迹做预测 ----
        for track in self.tracks:
            track.predict()

        # ---- 步骤2: 级联匹配 ----
        # 按丢失时长分组
        confirmed_tracks = [t for t in self.tracks if t.is_confirmed()]
        unconfirmed_tracks = [t for t in self.tracks if not t.is_confirmed()]

        # 按 time_since_update 排序（级联匹配的核心思想）
        matched_track_indices = set()
        matched_det_indices = set()

        for age_level in range(self.max_age + 1):
            # 取当前age级别的轨迹
            age_tracks = [t for t in confirmed_tracks
                          if t.time_since_update == age_level
                          and id(t) not in {id(self.tracks[i]) for i in matched_track_indices}]
            if not age_tracks or not detections:
                continue

            # 找到这些轨迹在self.tracks中的索引
            track_indices = []
            for t in age_tracks:
                for i, st in enumerate(self.tracks):
                    if id(st) == id(t) and i not in matched_track_indices:
                        track_indices.append(i)
                        break

            det_indices = [j for j in range(len(detections)) if j not in matched_det_indices]
            if not track_indices or not det_indices:
                continue

            # 计算代价矩阵
            cost = np.zeros((len(track_indices), len(det_indices)))
            for i, ti in enumerate(track_indices):
                cost[i] = self._gating_distance(self.tracks[ti],
                                                 [detections[j] for j in det_indices])

            matched, unmatched_t, unmatched_d = self._hungarian_match(
                cost, threshold=self.iou_threshold + self.max_cosine_distance)

            for m_t, m_d in matched:
                ti = track_indices[m_t]
                di = det_indices[m_d]
                self.tracks[ti].update(
                    detections[di]['bbox'],
                    detections[di]['feature'],
                    detections[di].get('class_id', -1)
                )
                matched_track_indices.add(ti)
                matched_det_indices.add(di)

        # ---- 步骤3: IoU匹配（对未匹配的轨迹） ----
        remaining_tracks = [i for i in range(len(self.tracks))
                            if i not in matched_track_indices]
        remaining_dets = [j for j in range(len(detections))
                          if j not in matched_det_indices]

        if remaining_tracks and remaining_dets:
            iou_mat = compute_iou_matrix(
                [self.tracks[i].bbox for i in remaining_tracks],
                [detections[j]['bbox'] for j in remaining_dets]
            )
            iou_cost = 1.0 - iou_mat
            matched_iou, unmatch_t, unmatch_d = self._hungarian_match(
                iou_cost, threshold=1 - self.iou_threshold)

            for m_t, m_d in matched_iou:
                ti = remaining_tracks[m_t]
                di = remaining_dets[m_d]
                self.tracks[ti].update(
                    detections[di]['bbox'],
                    detections[di]['feature'],
                    detections[di].get('class_id', -1)
                )
                matched_track_indices.add(ti)
                matched_det_indices.add(di)

        # ---- 步骤4: 标记未匹配轨迹 ----
        for i in range(len(self.tracks)):
            if i not in matched_track_indices:
                self.tracks[i].mark_missed()

        # ---- 步骤5: 为未匹配的检测创建新轨迹 ----
        for j in range(len(detections)):
            if j not in matched_det_indices:
                new_track = Track(
                    self.next_id,
                    detections[j]['bbox'],
                    detections[j]['feature'],
                    detections[j].get('class_id', -1)
                )
                self.tracks.append(new_track)
                self.next_id += 1

        # ---- 步骤6: 删除已失效的轨迹 ----
        self.tracks = [t for t in self.tracks if t.state != 'deleted']

        # 返回活跃轨迹
        active = []
        for t in self.tracks:
            if t.is_confirmed():
                active.append({
                    'track_id': t.track_id,
                    'bbox': [int(v) for v in t.bbox],
                    'class_id': t.class_id,
                    'hits': t.hits,
                    'age': t.age
                })
        return active


# ============================================================
# 可视化绘制
# ============================================================
def draw_tracks(frame, tracks, draw_trail=True):
    """
    在图像上绘制跟踪结果
    参数:
        frame: 图像
        tracks: 跟踪结果列表
        draw_trail: 是否绘制轨迹（预留）
    """
    # 颜色映射
    colors = [
        (0, 255, 0), (255, 0, 0), (0, 0, 255),
        (255, 255, 0), (0, 255, 255), (255, 0, 255),
        (128, 255, 0), (255, 128, 0), (128, 0, 255),
    ]

    for track in tracks:
        tid = track['track_id']
        x1, y1, x2, y2 = track['bbox']
        color = colors[tid % len(colors)]

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"ID:{tid}"
        cv2.putText(frame, label, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    return frame


# ============================================================
# 使用示例
# ============================================================
def demo():
    """
    演示：使用简单的背景差分检测 + DeepSORT跟踪
    """
    # 创建跟踪器
    tracker = DeepSORTTracker(
        max_age=30,
        n_init=3,
        iou_threshold=0.3,
        appearance_weight=0.4,
        max_cosine_distance=0.5
    )

    # 打开摄像头（或替换为视频路径）
    cap = cv2.VideoCapture(0)

    # 背景差分检测器
    bg_subtractor = cv2.createBackgroundSubtractorMOG2(
        history=500, varThreshold=50, detectShadows=True
    )

    print("DeepSORT 多目标跟踪演示")
    print("按 'q' 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # ---- 简单的目标检测（背景差分） ----
        mask = bg_subtractor.apply(frame)
        mask = cv2.medianBlur(mask, 5)
        _, mask = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            detections.append({
                'bbox': [x, y, x + w, y + h],
                'conf': 0.9,
                'class_id': 0
            })

        # ---- DeepSORT 更新 ----
        active_tracks = tracker.update(detections, frame)

        # ---- 可视化 ----
        result = draw_tracks(frame.copy(), active_tracks)
        info = f"Tracks: {len(active_tracks)}  Dets: {len(detections)}"
        cv2.putText(result, info, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("DeepSORT Tracking", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    demo()
