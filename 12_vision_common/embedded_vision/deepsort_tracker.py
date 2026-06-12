# -*- coding: utf-8 -*-
"""
模块2: 实时目标跟踪 - DeepSORT简化版
======================================
简化DeepSORT, 适配嵌入式平台(Orange Pi 5)

DeepSORT核心原理:
  1. 检测: 每帧用检测器获取目标框
  2. 预测: 卡尔曼滤波预测目标下一帧位置
  3. 匹配: 匈牙利算法匹配检测框与跟踪轨迹
  4. 更新: 匹配成功的更新轨迹, 未匹配的创建/删除

简化策略(嵌入式优化):
  - 去掉ReID特征提取(省50%计算量)
  - 只用IoU+运动匹配
  - 轨迹生命周期管理简洁化
  - 卡尔曼滤波用numpy向量化

电赛场景:
  - 智能小车追踪移动目标
  - 机械臂抓取运动物体
  - 多目标计数/测速
"""

import cv2
import numpy as np
from collections import deque
from scipy.optimize import linear_sum_assignment  # 匈牙利算法


class KalmanTracker:
    """
    卡尔曼滤波跟踪器(单个目标)
    
    状态向量: [cx, cy, w, h, vx, vy, vw, vh]
    观测向量: [cx, cy, w, h]
    
    简化版: 线性匀速模型, 忽略加速度
    """
    
    _id_counter = 0
    
    def __init__(self, bbox):
        """
        初始化跟踪器
        
        Args:
            bbox: 初始边界框 [x, y, w, h]
        """
        KalmanTracker._id_counter += 1
        self.track_id = KalmanTracker._id_counter
        
        # 状态: [cx, cy, w, h, vx, vy, vw, vh]
        cx, cy = bbox[0] + bbox[2]/2, bbox[1] + bbox[3]/2
        self.state = np.array([cx, cy, bbox[2], bbox[3], 0, 0, 0, 0], dtype=np.float64)
        
        # 协方差矩阵
        self.P = np.eye(8) * 100.0
        self.P[4:, 4:] *= 10.0  # 速度不确定性更大
        
        # 状态转移矩阵(匀速模型)
        self.F = np.eye(8)
        self.F[0, 4] = 1  # cx += vx
        self.F[1, 5] = 1  # cy += vy
        self.F[2, 6] = 1  # w += vw
        self.F[3, 7] = 1  # h += vh
        
        # 观测矩阵
        self.H = np.zeros((4, 8))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = self.H[3, 3] = 1
        
        # 噪声
        self.Q = np.eye(8) * 1.0
        self.Q[4:, 4:] *= 0.1
        self.R = np.eye(4) * 10.0
        
        # 跟踪状态
        self.hits = 1         # 连续匹配次数
        self.age = 1          # 总帧数
        self.time_since_update = 0  # 未更新帧数
    
    def predict(self):
        """预测下一帧状态"""
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1
        self.time_since_update += 1
        return self.get_bbox()
    
    def update(self, bbox):
        """
        用检测框更新状态
        
        Args:
            bbox: [x, y, w, h]
        """
        cx, cy = bbox[0] + bbox[2]/2, bbox[1] + bbox[3]/2
        z = np.array([cx, cy, bbox[2], bbox[3]])
        
        y = z - self.H @ self.state
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y
        self.P = (np.eye(8) - K @ self.H) @ self.P
        
        self.hits += 1
        self.time_since_update = 0
    
    def get_bbox(self):
        """获取当前边界框 [x, y, w, h]"""
        cx, cy, w, h = self.state[:4]
        return np.array([cx - w/2, cy - h/2, w, h])
    
    def is_confirmed(self, min_hits=3):
        """轨迹是否已确认(连续匹配>=min_hits次)"""
        return self.hits >= min_hits
    
    def is_deleted(self, max_age=30):
        """轨迹是否应删除(连续未更新>=max_age帧)"""
        return self.time_since_update >= max_age


def compute_iou_matrix(boxes_a, boxes_b):
    """
    计算两组边界框的IoU矩阵
    
    Args:
        boxes_a: Nx4 数组 [x, y, w, h]
        boxes_b: Mx4 数组 [x, y, w, h]
    Returns:
        iou: NxM 矩阵
    """
    if len(boxes_a) == 0 or len(boxes_b) == 0:
        return np.zeros((len(boxes_a), len(boxes_b)))
    
    boxes_a = np.array(boxes_a, dtype=np.float64)
    boxes_b = np.array(boxes_b, dtype=np.float64)
    
    # 转为xyxy
    a_x1, a_y1 = boxes_a[:, 0], boxes_a[:, 1]
    a_x2, a_y2 = a_x1 + boxes_a[:, 2], a_y1 + boxes_a[:, 3]
    b_x1, b_y1 = boxes_b[:, 0], boxes_b[:, 1]
    b_x2, b_y2 = b_x1 + boxes_b[:, 2], b_y1 + boxes_b[:, 3]
    
    # 交集
    xx1 = np.maximum(a_x1[:, None], b_x1[None, :])
    yy1 = np.maximum(a_y1[:, None], b_y1[None, :])
    xx2 = np.minimum(a_x2[:, None], b_x2[None, :])
    yy2 = np.minimum(a_y2[:, None], b_y2[None, :])
    
    inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
    area_a = boxes_a[:, 2] * boxes_a[:, 3]
    area_b = boxes_b[:, 2] * boxes_b[:, 3]
    union = area_a[:, None] + area_b[None, :] - inter
    
    return inter / np.maximum(union, 1e-6)


class SimpleDeepSORT:
    """
    简化版DeepSORT多目标跟踪器
    
    简化策略:
      - 去掉ReID外观特征(省50%算力)
      - 只用IoU + 运动预测匹配
      - 卡尔曼滤波预测 + 匈牙利算法匹配
    
    用法:
        tracker = SimpleDeepSORT(iou_threshold=0.3)
        detections = detector.detect(frame)  # [(cls, conf, (x,y,w,h)), ...]
        tracks = tracker.update(detections)
        for tid, cls, bbox in tracks:
            print(f"Track {tid}: {cls} at {bbox}")
    """
    
    def __init__(self, iou_threshold=0.3, max_age=30, min_hits=3):
        """
        Args:
            iou_threshold: IoU匹配阈值
            max_age: 未更新最大帧数
            min_hits: 确认轨迹最小匹配次数
        """
        self.iou_threshold = iou_threshold
        self.max_age = max_age
        self.min_hits = min_hits
        self.trackers = []
    
    def update(self, detections):
        """
        更新跟踪器
        
        Args:
            detections: [(class_name, confidence, (x,y,w,h)), ...]
        Returns:
            tracks: [(track_id, class_name, (x,y,w,h)), ...]
        """
        det_boxes = np.array([d[2] for d in detections], dtype=np.float64)
        det_classes = [d[0] for d in detections]
        
        # Step 1: 所有跟踪器预测
        for trk in self.trackers:
            trk.predict()
        
        # Step 2: 匈牙利匹配
        if len(self.trackers) > 0 and len(det_boxes) > 0:
            trk_boxes = np.array([t.get_bbox() for t in self.trackers])
            iou_matrix = compute_iou_matrix(det_boxes, trk_boxes)
            cost_matrix = 1.0 - iou_matrix  # 代价=1-IoU
            
            row_idx, col_idx = linear_sum_assignment(cost_matrix)
            
            matched_dets, matched_trks = set(), set()
            for r, c in zip(row_idx, col_idx):
                if iou_matrix[r, c] >= self.iou_threshold:
                    self.trackers[c].update(det_boxes[r])
                    matched_dets.add(r)
                    matched_trks.add(c)
            
            # 未匹配的检测 → 新建跟踪器
            for i in range(len(detections)):
                if i not in matched_dets:
                    self.trackers.append(KalmanTracker(det_boxes[i]))
                    # 给新跟踪器记录类别
                    self.trackers[-1].class_name = det_classes[i]
            
            # 未匹配的跟踪器 → 保持(等待删除)
            # (由is_deleted自动管理)
        else:
            # 没有已有跟踪器, 全部新建
            for i, (cls, conf, bbox) in enumerate(detections):
                trk = KalmanTracker(bbox)
                trk.class_name = cls
                self.trackers.append(trk)
        
        # Step 3: 删除过期跟踪器
        self.trackers = [t for t in self.trackers if not t.is_deleted(self.max_age)]
        
        # Step 4: 返回已确认的轨迹
        results = []
        for trk in self.trackers:
            if trk.is_confirmed(self.min_hits):
                bbox = trk.get_bbox().astype(int)
                x, y, w, h = bbox
                cls = getattr(trk, 'class_name', 'unknown')
                results.append((trk.track_id, cls, (int(x), int(y), int(w), int(h))))
        
        return results
    
    def draw_tracks(self, frame, tracks):
        """绘制跟踪轨迹"""
        for tid, cls, (x, y, w, h) in tracks:
            # 根据track_id生成不同颜色
            color_hash = (tid * 67 + 131) % 255
            color = (color_hash, (tid * 31 + 89) % 255, (tid * 43 + 199) % 255)
            
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
            label = f"ID:{tid} {cls}"
            cv2.putText(frame, label, (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return frame


# ===== 电赛应用示例 =====

def demo_target_tracking():
    """
    智能小车目标追踪示例
    
    场景: 检测并持续追踪目标, 输出目标位置与速度
    应用: 小车跟随模式 / 机械臂抓取
    """
    from .platform_utils import CameraThread, FrameCounter, optimize_opencv
    from .object_detector import MobileNetSSDDetector
    
    optimize_opencv()
    cam = CameraThread(src=0, width=640, height=480).start()
    detector = MobileNetSSDDetector(conf_threshold=0.4)
    tracker = SimpleDeepSORT(iou_threshold=0.3, max_age=20, min_hits=2)
    counter = FrameCounter()
    
    print("[目标追踪] 启动... 按q退出")
    while True:
        frame = cam.read()
        if frame is None:
            continue
        
        detections = detector.detect(frame)
        tracks = tracker.update(detections)
        frame = tracker.draw_tracks(frame, tracks)
        
        counter.tick()
        cv2.putText(frame, f"FPS:{counter.fps:.1f} Tracks:{len(tracks)}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        cv2.imshow("Target Tracking", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cam.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo_target_tracking()
