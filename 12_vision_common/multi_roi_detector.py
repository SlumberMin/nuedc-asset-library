"""
多ROI区域检测器 - 分区域并行处理
支持自定义多个检测区域，各区域独立检测策略
"""
import cv2
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import List, Tuple, Callable, Optional, Dict, Any


@dataclass
class ROIRegion:
    """单个ROI区域定义"""
    name: str
    x: int
    y: int
    w: int
    h: int
    color: Tuple[int, int, int] = (0, 255, 0)
    params: Dict[str, Any] = field(default_factory=dict)
    enabled: bool = True


class MultiROIDetector:
    """多ROI区域并行检测器"""

    def __init__(self, max_workers=4):
        self.rois: List[ROIRegion] = []
        self._detectors: Dict[str, Callable] = {}
        self._executor = ThreadPoolExecutor(max_workers=max_workers)
        self._results: Dict[str, Any] = {}

    def add_roi(self, name, x, y, w, h, color=(0, 255, 0), params=None):
        """添加ROI区域"""
        roi = ROIRegion(name=name, x=x, y=y, w=w, h=h,
                        color=color, params=params or {})
        self.rois.append(roi)
        return self

    def add_grid_rois(self, rows, cols, frame_w, frame_h, prefix="roi"):
        """网格划分ROI区域"""
        rw, rh = frame_w // cols, frame_h // rows
        for r in range(rows):
            for c in range(cols):
                name = f"{prefix}_{r}_{c}"
                self.add_roi(name, c * rw, r * rh, rw, rh)
        return self

    def set_detector(self, roi_name, detector_fn):
        """
        为指定ROI设置检测函数
        detector_fn(cropped_img, roi_params) -> list of detections
        """
        self._detectors[roi_name] = detector_fn
        return self

    def set_default_detector(self, detector_fn):
        """为所有未设置检测器的ROI设置默认检测"""
        for roi in self.rois:
            if roi.name not in self._detectors:
                self._detectors[roi.name] = detector_fn
        return self

    def _detect_single(self, roi: ROIRegion, frame: np.ndarray):
        """单个ROI检测"""
        if not roi.enabled or roi.name not in self._detectors:
            return roi.name, []
        x, y = max(0, roi.x), max(0, roi.y)
        x2 = min(frame.shape[1], roi.x + roi.w)
        y2 = min(frame.shape[0], roi.y + roi.h)
        cropped = frame[y:y2, x:x2]
        if cropped.size == 0:
            return roi.name, []
        detections = self._detectors[roi.name](cropped, roi.params)
        # 将检测坐标转换回原图坐标
        results = []
        for det in (detections or []):
            if isinstance(det, dict):
                det = dict(det)
                if "cx" in det:
                    det["cx"] += x
                    det["cy"] += y
                if "bbox" in det:
                    bx, by, bw, bh = det["bbox"]
                    det["bbox"] = (bx + x, by + y, bw, bh)
            elif isinstance(det, (list, tuple)) and len(det) >= 2:
                det = list(det)
                det[0] += x
                det[1] += y
            results.append(det)
        return roi.name, results

    def detect(self, frame: np.ndarray) -> Dict[str, list]:
        """并行检测所有ROI区域"""
        futures = []
        for roi in self.rois:
            fut = self._executor.submit(self._detect_single, roi, frame)
            futures.append(fut)

        self._results = {}
        for fut in futures:
            name, detections = fut.result()
            self._results[name] = detections
        return self._results

    def draw_rois(self, frame, show_detections=True, show_names=True):
        """在帧上绘制所有ROI和检测结果"""
        vis = frame.copy()
        for roi in self.rois:
            if not roi.enabled:
                continue
            cv2.rectangle(vis, (roi.x, roi.y),
                          (roi.x + roi.w, roi.y + roi.h),
                          roi.color, 2)
            if show_names:
                cv2.putText(vis, roi.name, (roi.x + 4, roi.y + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, roi.color, 1)

            if show_detections and roi.name in self._results:
                for det in self._results[roi.name]:
                    if isinstance(det, dict) and "cx" in det:
                        cx, cy = int(det["cx"]), int(det["cy"])
                        cv2.circle(vis, (cx, cy), 5, roi.color, -1)
        return vis

    def get_all_detections(self):
        """获取所有区域的检测结果合并列表"""
        all_dets = []
        for name, dets in self._results.items():
            for d in dets:
                if isinstance(d, dict):
                    d = dict(d)
                    d["roi"] = name
                all_dets.append(d)
        return all_dets

    def __del__(self):
        self._executor.shutdown(wait=False)


def default_color_detector(cropped, params):
    """默认颜色检测器示例"""
    lower = np.array(params.get("lower", [0, 100, 100]))
    upper = np.array(params.get("upper", [10, 255, 255]))
    hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    results = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < params.get("min_area", 100):
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        results.append({"cx": cx, "cy": cy, "area": area})
    return results


if __name__ == "__main__":
    detector = MultiROIDetector()
    detector.add_roi("left", 0, 0, 320, 480, color=(0, 255, 0),
                     params={"lower": [0, 100, 100], "upper": [10, 255, 255]})
    detector.add_roi("right", 320, 0, 320, 480, color=(0, 0, 255),
                     params={"lower": [100, 100, 100], "upper": [130, 255, 255]})
    detector.set_default_detector(default_color_detector)

    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        results = detector.detect(frame)
        vis = detector.draw_rois(frame)
        cv2.imshow("Multi ROI", vis)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
