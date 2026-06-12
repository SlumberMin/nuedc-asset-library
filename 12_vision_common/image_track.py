"""
目标跟踪V2模块 - KCF/MOSSE/CSRT/通用Tracker
适用场景: 电赛中运动目标实时跟踪、多目标跟踪
依赖: opencv-python (contrib版推荐), numpy
注意: 部分跟踪器需要 opencv-contrib-python
"""

import cv2
import numpy as np


class ImageTrackV2:
    """目标跟踪工具集V2"""

    # ---- 跟踪器创建 ----

    TRACKER_MAP = {
        'kcf': 'KCF',
        'mosse': 'MOSSE',
        'csrt': 'CSRT',
        'mil': 'MIL',
        'boosting': 'Boosting',
        'tld': 'TLD',
        'medianflow': 'MedianFlow',
        'goturn': 'GOTURN',
    }

    @staticmethod
    def create_tracker(tracker_type='csrt'):
        """
        创建跟踪器实例
        :param tracker_type: 'kcf'|'mosse'|'csrt'|'mil'|'boosting'|'tld'|'medianflow'|'goturn'
        :return: cv2.Tracker 实例
        """
        tracker_type = tracker_type.lower()

        # OpenCV 4.5.1+ 新API
        if hasattr(cv2, 'legacy'):
            tracker_creators = {
                'kcf': cv2.TrackerKCF_create,
                'csrt': cv2.TrackerCSRT_create,
                'mosse': cv2.legacy.TrackerMOSSE_create,
                'mil': cv2.TrackerMIL_create,
                'boosting': cv2.legacy.TrackerBoosting_create,
                'tld': cv2.legacy.TrackerTLD_create,
                'medianflow': cv2.legacy.TrackerMedianFlow_create,
                'goturn': cv2.TrackerGOTURN_create,
            }
        else:
            # 旧版API (OpenCV < 4.5.1)
            tracker_creators = {
                'kcf': cv2.TrackerKCF_create,
                'csrt': cv2.TrackerCSRT_create,
                'mosse': cv2.TrackerMOSSE_create,
                'mil': cv2.TrackerMIL_create,
                'boosting': cv2.TrackerBoosting_create,
                'tld': cv2.TrackerTLD_create,
                'medianflow': cv2.TrackerMedianFlow_create,
                'goturn': cv2.TrackerGOTURN_create,
            }

        creator = tracker_creators.get(tracker_type)
        if creator is None:
            raise ValueError(f"不支持的跟踪器: {tracker_type}, 可选: {list(tracker_creators.keys())}")
        return creator()

    # ---- 单目标跟踪器 ----

    @staticmethod
    def init_tracker(image, bbox, tracker_type='csrt'):
        """
        初始化跟踪器
        :param image: 第一帧图像 (BGR)
        :param bbox: 初始目标框 (x, y, w, h)
        :param tracker_type: 跟踪器类型
        :return: 跟踪器实例 (已初始化)
        """
        tracker = ImageTrackV2.create_tracker(tracker_type)
        tracker.init(image, tuple(bbox))
        return tracker

    @staticmethod
    def update_tracker(tracker, image):
        """
        更新跟踪器 (逐帧调用)
        :param tracker: 已初始化的跟踪器
        :param image: 当前帧 (BGR)
        :return: (success: bool, bbox: (x,y,w,h))
        """
        success, bbox = tracker.update(image)
        if success:
            bbox = tuple(int(v) for v in bbox)
        return success, bbox

    # ---- 跟踪可视化 ----

    @staticmethod
    def draw_tracking(image, bbox, success=True, color=(0, 255, 0), thickness=2):
        """
        绘制跟踪框
        :param image: 输入图像
        :param bbox: (x, y, w, h)
        :param success: 跟踪是否成功
        :return: 绘制后的图像
        """
        vis = image.copy()
        x, y, w, h = [int(v) for v in bbox]
        box_color = color if success else (0, 0, 255)
        cv2.rectangle(vis, (x, y), (x + w, y + h), box_color, thickness)
        status = "Tracking" if success else "Lost"
        cv2.putText(vis, status, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
        return vis

    # ---- 多目标跟踪 ----

    class MultiTracker:
        """多目标跟踪管理器"""

        def __init__(self, tracker_type='csrt'):
            self.tracker_type = tracker_type
            self.trackers = []  # [(tracker, bbox), ...]
            self.labels = []  # 可选标签

        def add(self, image, bbox, label=None):
            """添加一个跟踪目标"""
            tracker = ImageTrackV2.init_tracker(image, bbox, self.tracker_type)
            self.trackers.append(tracker)
            self.labels.append(label if label else f"obj_{len(self.trackers)}")

        def update(self, image):
            """
            更新所有跟踪器
            :return: list of (label, success, bbox)
            """
            results = []
            for i, tracker in enumerate(self.trackers):
                success, bbox = ImageTrackV2.update_tracker(tracker, image)
                results.append((self.labels[i], success, bbox))
            return results

        def draw(self, image, results=None, colors=None):
            """
            绘制所有跟踪框
            :return: 绘制后的图像
            """
            vis = image.copy()
            if results is None:
                results = self.update(image)
            if colors is None:
                colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255),
                          (255, 255, 0), (0, 255, 255), (255, 0, 255)]

            for i, (label, success, bbox) in enumerate(results):
                if bbox is not None:
                    color = colors[i % len(colors)]
                    x, y, w, h = [int(v) for v in bbox]
                    box_color = color if success else (0, 0, 255)
                    cv2.rectangle(vis, (x, y), (x + w, y + h), box_color, 2)
                    cv2.putText(vis, label, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
            return vis

        def remove_lost(self):
            """移除丢失的跟踪目标"""
            # 需要外部调用update后根据结果判断
            pass

        @property
        def count(self):
            return len(self.trackers)

    # ---- KCF 跟踪 ----

    @staticmethod
    def kcf_track(image, bbox):
        """快捷: KCF跟踪器初始化"""
        return ImageTrackV2.init_tracker(image, bbox, 'kcf')

    # ---- MOSSE 跟踪 (最快) ----

    @staticmethod
    def mosse_track(image, bbox):
        """快捷: MOSSE跟踪器初始化 (速度最快, ~700FPS)"""
        return ImageTrackV2.init_tracker(image, bbox, 'mosse')

    # ---- CSRT 跟踪 (最准) ----

    @staticmethod
    def csrt_track(image, bbox):
        """快捷: CSRT跟踪器初始化 (精度最高)"""
        return ImageTrackV2.init_tracker(image, bbox, 'csrt')

    # ---- 跟踪器性能对比 ----

    @staticmethod
    def benchmark(image_sequence, bbox, tracker_types=None):
        """
        对比不同跟踪器在同一序列上的性能
        :param image_sequence: 图像帧列表
        :param bbox: 初始目标框
        :param tracker_types: 跟踪器列表, None则使用全部
        :return: dict {tracker_type: {'fps': float, 'frames_tracked': int}}
        """
        import time

        if tracker_types is None:
            tracker_types = ['kcf', 'mosse', 'csrt']

        results = {}
        for tt in tracker_types:
            try:
                tracker = ImageTrackV2.init_tracker(image_sequence[0], bbox, tt)
                tracked = 0
                t_start = time.time()

                for frame in image_sequence[1:]:
                    success, _ = ImageTrackV2.update_tracker(tracker, frame)
                    if success:
                        tracked += 1

                elapsed = time.time() - t_start
                fps = len(image_sequence) / max(elapsed, 1e-6)
                results[tt] = {
                    'fps': round(fps, 1),
                    'frames_tracked': tracked,
                    'total_frames': len(image_sequence) - 1,
                    'success_rate': round(tracked / max(len(image_sequence) - 1, 1), 3),
                }
            except Exception as e:
                results[tt] = {'error': str(e)}

        return results


# ======================== 快捷函数 ========================

def create_tracker(tracker_type='csrt'):
    return ImageTrackV2.create_tracker(tracker_type)

def init_tracker(image, bbox, tracker_type='csrt'):
    return ImageTrackV2.init_tracker(image, bbox, tracker_type)

def update_tracker(tracker, image):
    return ImageTrackV2.update_tracker(tracker, image)

def create_multi_tracker(tracker_type='csrt'):
    return ImageTrackV2.MultiTracker(tracker_type)
