"""
颜色校准GUI工具 - 滑动条实时预览 + 参数保存/加载
用于快速调试HSV/RGB颜色阈值
"""
import cv2
import numpy as np
import json
import os


class ColorCalibrationGUI:
    """颜色校准GUI，通过滑动条实时调整颜色阈值"""

    def __init__(self, window_name="Color Calibration", width=640, height=480):
        self.window_name = window_name
        self.width = width
        self.height = height
        self.params = {}
        self._trackbars = {}
        self._callbacks = {}
        self._frame = None
        self._mask = None

    def add_trackbar(self, name, default=0, max_val=255, callback=None):
        """添加滑动条参数"""
        self.params[name] = default
        self._trackbars[name] = max_val
        self._callbacks[name] = callback
        return self

    def add_hsv_range(self, prefix=""):
        """添加HSV范围滑动条组"""
        p = f"{prefix}_" if prefix else ""
        self.add_trackbar(f"{p}H_min", 0, 179)
        self.add_trackbar(f"{p}H_max", 179, 179)
        self.add_trackbar(f"{p}S_min", 0, 255)
        self.add_trackbar(f"{p}S_max", 255, 255)
        self.add_trackbar(f"{p}V_min", 0, 255)
        self.add_trackbar(f"{p}V_max", 255, 255)
        return self

    def get_hsv_range(self, prefix=""):
        """获取当前HSV范围"""
        p = f"{prefix}_" if prefix else ""
        lower = np.array([
            self.params[f"{p}H_min"],
            self.params[f"{p}S_min"],
            self.params[f"{p}V_min"]
        ])
        upper = np.array([
            self.params[f"{p}H_max"],
            self.params[f"{p}S_max"],
            self.params[f"{p}V_max"]
        ])
        return lower, upper

    def create_window(self):
        """创建窗口和滑动条"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.width, self.height)
        for name, max_val in self._trackbars.items():
            cv2.createTrackbar(
                name, self.window_name,
                self.params[name], max_val,
                lambda val, n=name: self._on_trackbar(n, val)
            )

    def _on_trackbar(self, name, value):
        self.params[name] = value
        if self._callbacks.get(name):
            self._callbacks[name](name, value)

    def apply_mask(self, frame, hsv_lower, hsv_hsv_upper):
        """对帧应用HSV掩码"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_hsv_upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        self._mask = mask
        return mask

    def save_params(self, filepath):
        """保存参数到JSON文件"""
        data = {k: int(v) for k, v in self.params.items()}
        os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"[ColorCal] 参数已保存到 {filepath}")

    def load_params(self, filepath):
        """从JSON文件加载参数"""
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        for k, v in data.items():
            if k in self.params:
                self.params[k] = int(v)
                cv2.setTrackbarPos(k, self.window_name, int(v))
        print(f"[ColorCal] 参数已加载自 {filepath}")

    def run(self, source=0, save_path=None):
        """
        运行校准主循环
        source: 摄像头索引或视频文件路径
        save_path: 退出时自动保存参数的路径
        """
        self.create_window()
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            print(f"[ColorCal] 无法打开视频源: {source}")
            return

        print("[ColorCal] 按 's' 保存参数, 'q'/ESC 退出")

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            frame = cv2.resize(frame, (self.width, self.height))
            self._frame = frame

            lower, upper = self.get_hsv_range()
            mask = self.apply_mask(frame, lower, upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            # 拼接显示: 原图 | 掩码 | 结果
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            display = np.hstack([frame, mask_bgr, result])
            cv2.imshow(self.window_name, display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            elif key == ord("s") and save_path:
                self.save_params(save_path)

        cap.release()
        cv2.destroyAllWindows()


def quick_calibrate(source=0, save_path="color_params.json"):
    """快速颜色校准入口"""
    gui = ColorCalibrationGUI()
    gui.add_hsv_range()
    if os.path.exists(save_path):
        gui.load_params(save_path)
    gui.run(source=source, save_path=save_path)


if __name__ == "__main__":
    import sys
    src = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    quick_calibrate(source=src)
