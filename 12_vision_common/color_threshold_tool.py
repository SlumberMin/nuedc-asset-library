#!/usr/bin/env python3
"""
自动颜色阈值校准工具
功能：滑动条实时调整HSV阈值，支持保存/加载配置
适用：OpenCV + Orange Pi 5 / USB摄像头
"""

import cv2
import numpy as np
import json
import os

class ColorThresholdTool:
    """颜色阈值校准工具，支持HSV/HLS色彩空间"""

    def __init__(self, camera_id=0, config_dir="configs"):
        self.camera_id = camera_id
        self.config_dir = config_dir
        os.makedirs(config_dir, exist_ok=True)

        # 默认HSV阈值 [H_min, S_min, V_min, H_max, S_max, V_max]
        self.thresholds = {
            'black':  [0, 0, 0, 180, 255, 50],
            'red1':   [0, 100, 100, 10, 255, 255],
            'red2':   [160, 100, 100, 180, 255, 255],
            'green':  [35, 80, 80, 85, 255, 255],
            'blue':   [100, 100, 100, 130, 255, 255],
            'yellow': [20, 100, 100, 35, 255, 255],
            'white':  [0, 0, 200, 180, 30, 255],
        }
        self.current_color = 'red1'
        self.running = True

    def _create_trackbars(self, window_name):
        """创建滑动条"""
        vals = self.thresholds[self.current_color]
        cv2.createTrackbar('H_min', window_name, vals[0], 180, lambda x: None)
        cv2.createTrackbar('S_min', window_name, vals[1], 255, lambda x: None)
        cv2.createTrackbar('V_min', window_name, vals[2], 255, lambda x: None)
        cv2.createTrackbar('H_max', window_name, vals[3], 180, lambda x: None)
        cv2.createTrackbar('S_max', window_name, vals[4], 255, lambda x: None)
        cv2.createTrackbar('V_max', window_name, vals[5], 255, lambda x: None)

    def _get_trackbar_values(self, window_name):
        """获取滑动条当前值"""
        h_min = cv2.getTrackbarPos('H_min', window_name)
        s_min = cv2.getTrackbarPos('S_min', window_name)
        v_min = cv2.getTrackbarPos('V_min', window_name)
        h_max = cv2.getTrackbarPos('H_max', window_name)
        s_max = cv2.getTrackbarPos('S_max', window_name)
        v_max = cv2.getTrackbarPos('V_max', window_name)
        return [h_min, s_min, v_min, h_max, s_max, v_max]

    def _update_trackbar_values(self, window_name, vals):
        """更新滑动条值"""
        cv2.setTrackbarPos('H_min', window_name, vals[0])
        cv2.setTrackbarPos('S_min', window_name, vals[1])
        cv2.setTrackbarPos('V_min', window_name, vals[2])
        cv2.setTrackbarPos('H_max', window_name, vals[3])
        cv2.setTrackbarPos('S_max', window_name, vals[4])
        cv2.setTrackbarPos('V_max', window_name, vals[5])

    def _create_morphology_kernel(self, size=5):
        """创建形态学处理核"""
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def _apply_morphology(self, mask):
        """形态学处理：开运算去噪 + 闭运算填充"""
        kernel = self._create_morphology_kernel(5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def save_threshold(self, name, values, filename="thresholds.json"):
        """保存阈值到JSON文件"""
        filepath = os.path.join(self.config_dir, filename)
        data = {}
        if os.path.exists(filepath):
            with open(filepath, 'r') as f:
                data = json.load(f)
        data[name] = values
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"[保存] {name} = {values}")

    def load_thresholds(self, filename="thresholds.json"):
        """从JSON文件加载阈值"""
        filepath = os.path.join(self.config_dir, filename)
        if os.path.exists(filepath):
            with open(filepath, 'r') as f:
                self.thresholds.update(json.load(f))
            print(f"[加载] 阈值配置: {filepath}")

    def detect_color(self, frame, color_name=None):
        """
        检测指定颜色区域
        返回: mask, contours, frame_with_drawing
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color = color_name or self.current_color

        if color == 'red':
            # 红色跨越H=0/180边界，需要两段合并
            lower1 = np.array([0, 100, 100])
            upper1 = np.array([10, 255, 255])
            lower2 = np.array([160, 100, 100])
            upper2 = np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv, lower1, upper1)
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            vals = self.thresholds.get(color, self.thresholds['red1'])
            lower = np.array(vals[:3])
            upper = np.array(vals[3:])
            mask = cv2.inRange(hsv, lower, upper)

        mask = self._apply_morphology(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 绘制检测结果
        result = frame.copy()
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(result, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cx, cy = x + w//2, y + h//2
                cv2.circle(result, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(result, f"({cx},{cy})", (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return mask, contours, result

    def run(self):
        """启动交互式校准界面"""
        cap = cv2.VideoCapture(self.camera_id)
        if not cap.isOpened():
            print(f"[错误] 无法打开摄像头 {self.camera_id}")
            return

        # 设置分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        win_trackbars = 'Thresholds'
        win_mask = 'Mask'
        win_result = 'Result'

        cv2.namedWindow(win_trackbars, cv2.WINDOW_NORMAL)
        cv2.namedWindow(win_mask, cv2.WINDOW_NORMAL)
        cv2.namedWindow(win_result, cv2.WINDOW_NORMAL)

        self._create_trackbars(win_trackbars)

        color_list = list(self.thresholds.keys())
        color_idx = 0

        print("=" * 50)
        print("颜色阈值校准工具")
        print("=" * 50)
        print("操作说明:")
        print("  滑动条   - 调整HSV阈值")
        print("  n/p      - 下一个/上一个预设颜色")
        print("  s        - 保存当前阈值")
        print("  r        - 重置为默认值")
        print("  q/ESC    - 退出")
        print("=" * 50)

        while self.running:
            ret, frame = cap.read()
            if not ret:
                print("[错误] 无法读取摄像头画面")
                break

            # 获取当前阈值
            vals = self._get_trackbar_values(win_trackbars)
            self.thresholds[self.current_color] = vals

            # 检测颜色
            mask, contours, result = self.detect_color(frame)

            # 在画面上显示当前颜色名称
            cv2.putText(result, f"Color: {self.current_color}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(result, f"HSV: {vals}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(result, f"Contours: {len([c for c in contours if cv2.contourArea(c) > 500])}",
                       (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # 显示直方图辅助（H通道分布）
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h_hist = cv2.calcHist([hsv], [0], None, [180], [0, 180])
            h_hist = h_hist.flatten()
            hist_img = np.zeros((100, 180, 3), dtype=np.uint8)
            max_val = h_hist.max() if h_hist.max() > 0 else 1
            for i in range(180):
                h = int(h_hist[i] / max_val * 90)
                cv2.line(hist_img, (i, 99), (i, 99-h), (i*2, 255, 200), 1)
            hist_img = cv2.cvtColor(hist_img, cv2.COLOR_HSV2BGR)

            cv2.imshow(win_trackbars, hist_img)
            cv2.imshow(win_mask, mask)
            cv2.imshow(win_result, result)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:
                break
            elif key == ord('n'):
                color_idx = (color_idx + 1) % len(color_list)
                self.current_color = color_list[color_idx]
                self._update_trackbar_values(win_trackbars, self.thresholds[self.current_color])
                print(f"切换到: {self.current_color}")
            elif key == ord('p'):
                color_idx = (color_idx - 1) % len(color_list)
                self.current_color = color_list[color_idx]
                self._update_trackbar_values(win_trackbars, self.thresholds[self.current_color])
                print(f"切换到: {self.current_color}")
            elif key == ord('s'):
                self.save_threshold(self.current_color, vals)
                print(f"已保存 {self.current_color} 阈值")
            elif key == ord('r'):
                self._create_trackbars(win_trackbars)
                print(f"已重置 {self.current_color} 为默认值")

        cap.release()
        cv2.destroyAllWindows()

        # 退出时保存所有阈值
        self.save_threshold('__all__', self.thresholds)
        print("[完成] 阈值校准结束")


# ============ 独立运行 ============
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='颜色阈值校准工具')
    parser.add_argument('--camera', type=int, default=0, help='摄像头ID')
    parser.add_argument('--config', type=str, default='configs', help='配置目录')
    parser.add_argument('--load', type=str, default=None, help='加载已有配置文件')
    args = parser.parse_args()

    tool = ColorThresholdTool(camera_id=args.camera, config_dir=args.config)
    if args.load:
        tool.load_thresholds(args.load)
    tool.run()
