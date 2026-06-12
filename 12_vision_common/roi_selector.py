"""
ROI选择器 - 鼠标框选 + 保存/加载
适用于: 区域提取、目标检测标注、裁剪预设
"""

import cv2
import json
import numpy as np
from pathlib import Path


class ROISelector:
    """交互式ROI框选器"""

    def __init__(self, window_name="ROI Selector"):
        self.window_name = window_name
        self.roi = None          # (x, y, w, h)
        self.drawing = False
        self.start_point = None
        self.end_point = None
        self.current_frame = None

    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_point = (x, y)
            self.end_point = (x, y)

        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.end_point = (x, y)

        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.end_point = (x, y)
            x1 = min(self.start_point[0], self.end_point[0])
            y1 = min(self.start_point[1], self.end_point[1])
            x2 = max(self.start_point[0], self.end_point[0])
            y2 = max(self.start_point[1], self.end_point[1])
            w, h = x2 - x1, y2 - y1
            if w > 5 and h > 5:
                self.roi = (x1, y1, w, h)

    def select(self, frame):
        """
        交互式选择ROI
        Args:
            frame: 输入图像
        Returns:
            roi: (x, y, w, h) 或 None(取消)
        """
        self.current_frame = frame.copy()
        self.roi = None
        self.drawing = False
        self.start_point = None
        self.end_point = None

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)

        print("操作: 鼠标拖拽框选 | 'Enter'确认 | 'c'清除 | 'Esc'取消")

        while True:
            display = self.current_frame.copy()

            # 绘制当前框选区域
            if self.start_point and self.end_point:
                x1 = min(self.start_point[0], self.end_point[0])
                y1 = min(self.start_point[1], self.end_point[1])
                x2 = max(self.start_point[0], self.end_point[0])
                y2 = max(self.start_point[1], self.end_point[1])
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # 显示尺寸
                label = f"{x2-x1}x{y2-y1}"
                cv2.putText(display, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 已确认的ROI
            if self.roi and not self.drawing:
                x, y, w, h = self.roi
                cv2.rectangle(display, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(display, f"ROI: ({x},{y}) {w}x{h}", (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(30) & 0xFF

            if key == 27:  # Esc
                self.roi = None
                break
            elif key == 13 or key == 10:  # Enter
                if self.roi:
                    break
            elif key == ord('c'):
                self.roi = None
                self.start_point = None
                self.end_point = None

        cv2.destroyWindow(self.window_name)
        return self.roi

    def select_multiple(self, frame):
        """选择多个ROI"""
        rois = []
        temp_frame = frame.copy()

        while True:
            roi = self.select(temp_frame)
            if roi is None:
                break
            rois.append(roi)
            # 在帧上绘制已选ROI
            x, y, w, h = roi
            cv2.rectangle(temp_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(temp_frame, f"ROI#{len(rois)}", (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            print(f"已选择 {len(rois)} 个ROI, 继续选择或Esc结束")

        return rois


class ROIManager:
    """ROI保存/加载管理器"""

    def __init__(self, save_path="rois.json"):
        self.save_path = Path(save_path)
        self.rois = {}  # name -> {roi, image_shape, ...}

    def add(self, name, roi, image_shape=None, metadata=None):
        """添加一个命名ROI"""
        self.rois[name] = {
            "roi": list(roi),  # [x, y, w, h]
            "image_shape": list(image_shape) if image_shape else None,
            "metadata": metadata or {}
        }

    def get(self, name):
        """获取ROI"""
        if name in self.rois:
            return tuple(self.rois[name]["roi"])
        return None

    def remove(self, name):
        self.rois.pop(name, None)

    def save(self, path=None):
        """保存到JSON文件"""
        path = Path(path) if path else self.save_path
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(self.rois, f, indent=2, ensure_ascii=False)
        print(f"已保存 {len(self.rois)} 个ROI到 {path}")

    def load(self, path=None):
        """从JSON文件加载"""
        path = Path(path) if path else self.save_path
        if not path.exists():
            print(f"文件不存在: {path}")
            return
        with open(path, 'r', encoding='utf-8') as f:
            self.rois = json.load(f)
        print(f"已加载 {len(self.rois)} 个ROI")

    def list_all(self):
        """列出所有ROI"""
        for name, info in self.rois.items():
            roi = info["roi"]
            print(f"  {name}: ({roi[0]},{roi[1]}) {roi[2]}x{roi[3]}")

    def extract_roi(self, frame, name):
        """从图像中提取指定ROI区域"""
        roi = self.get(name)
        if roi is None:
            return None
        x, y, w, h = roi
        return frame[y:y+h, x:x+w]


def select_and_save(image_path, save_path="rois.json"):
    """从图像选择ROI并保存"""
    img = cv2.imread(image_path)
    if img is None:
        print(f"无法读取图像: {image_path}")
        return

    selector = ROISelector()
    manager = ROIManager(save_path)

    # 尝试加载已有的ROI
    manager.load()

    while True:
        roi = selector.select(img)
        if roi is None:
            break
        name = input("输入ROI名称 (留空跳过): ").strip()
        if name:
            manager.add(name, roi, img.shape[:2])
            print(f"已保存ROI: {name} = {roi}")

    manager.save()


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        select_and_save(sys.argv[1])
    else:
        # 从摄像头选择
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret:
                selector = ROISelector()
                roi = selector.select(frame)
                if roi:
                    print(f"选择的ROI: {roi}")
