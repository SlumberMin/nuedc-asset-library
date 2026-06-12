#!/usr/bin/env python3
"""
电赛视觉调试工具箱
功能：颜色阈值调整 / 形状检测 / 摄像头测试
运行：python 视觉调试工具箱.py [camera|color|shape]
依赖：pip install opencv-python numpy
"""

import cv2
import numpy as np
import sys
import json
import os
from datetime import datetime


# ============================================================
# 1. 摄像头测试模块
# ============================================================

def camera_test(camera_id=0, width=640, height=480):
    """摄像头基础测试 - 显示画面、帧率、分辨率"""
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    if not cap.isOpened():
        print(f"[ERROR] 无法打开摄像头 {camera_id}")
        print("检查：1) 摄像头连接  2) 权限  3) 是否被其他程序占用")
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_set = cap.get(cv2.CAP_PROP_FPS)
    print(f"摄像头 {camera_id} 已打开: {actual_w}x{actual_h} @ {fps_set}fps")

    frame_count = 0
    fps = 0
    t_start = cv2.getTickCount()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] 读取帧失败")
            break

        frame_count += 1
        if frame_count % 30 == 0:
            t_now = cv2.getTickCount()
            fps = 30.0 / ((t_now - t_start) / cv2.getTickFrequency())
            t_start = t_now

        # 绘制十字准星
        h, w = frame.shape[:2]
        cv2.line(frame, (w // 2 - 30, h // 2), (w // 2 + 30, h // 2), (0, 255, 0), 1)
        cv2.line(frame, (w // 2, h // 2 - 30), (w // 2, h // 2 + 30), (0, 255, 0), 1)

        # 显示信息
        info = f"{actual_w}x{actual_h} | FPS:{fps:.1f} | Frame:{frame_count}"
        cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 显示鼠标坐标
        cv2.putText(frame, "Press 's'=snapshot, 'g'=gray, 'q'=quit",
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("Camera Test", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            fname = f"snapshot_{datetime.now().strftime('%H%M%S')}.jpg"
            cv2.imwrite(fname, frame)
            print(f"快照已保存: {fname}")
        elif key == ord('g'):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imshow("Grayscale", gray)

    cap.release()
    cv2.destroyAllWindows()


# ============================================================
# 2. 颜色阈值调试模块
# ============================================================

class ColorThresholdDebugger:
    """交互式颜色阈值调整工具"""

    def __init__(self, camera_id=0):
        self.camera_id = camera_id
        self.current_mode = "HSV"

    def nothing(self, x):
        pass

    def run(self):
        cap = cv2.VideoCapture(self.camera_id)
        if not cap.isOpened():
            print(f"[ERROR] 无法打开摄像头 {self.camera_id}")
            return

        cv2.namedWindow("Controls", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Controls", 400, 300)

        # 默认红色范围 (HSV)
        cv2.createTrackbar("H_min", "Controls", 0, 179, self.nothing)
        cv2.createTrackbar("H_max", "Controls", 10, 179, self.nothing)
        cv2.createTrackbar("S_min", "Controls", 120, 255, self.nothing)
        cv2.createTrackbar("S_max", "Controls", 255, 255, self.nothing)
        cv2.createTrackbar("V_min", "Controls", 70, 255, self.nothing)
        cv2.createTrackbar("V_max", "Controls", 255, 255, self.nothing)

        # 预设按钮
        cv2.createTrackbar("Preset:0=Red1 1=Red2 2=Green 3=Blue 4=Yellow 5=Custom",
                           "Controls", 0, 5, self.nothing)

        presets = {
            0: {"name": "Red1", "h_min": 0, "h_max": 10, "s_min": 120, "s_max": 255, "v_min": 70, "v_max": 255},
            1: {"name": "Red2", "h_min": 156, "h_max": 180, "s_min": 120, "s_max": 255, "v_min": 70, "v_max": 255},
            2: {"name": "Green", "h_min": 35, "h_max": 85, "s_min": 80, "s_max": 255, "v_min": 60, "v_max": 255},
            3: {"name": "Blue", "h_min": 100, "h_max": 125, "s_min": 80, "s_max": 255, "v_min": 60, "v_max": 255},
            4: {"name": "Yellow", "h_min": 15, "h_max": 35, "s_min": 100, "s_max": 255, "v_min": 80, "v_max": 255},
        }

        last_preset = -1
        print("颜色阈值调试器已启动")
        print("操作: 拖动滑块调整阈值 | 's'=保存配置 | 'r'=红色范围叠加 | 'q'=退出")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # 检查预设切换
            preset = cv2.getTrackbarItem("Preset:0=Red1 1=Red2 2=Green 3=Blue 4=Yellow 5=Custom", "Controls")
            if preset != last_preset and preset in presets:
                p = presets[preset]
                cv2.setTrackbarPos("H_min", "Controls", p["h_min"])
                cv2.setTrackbarPos("H_max", "Controls", p["h_max"])
                cv2.setTrackbarPos("S_min", "Controls", p["s_min"])
                cv2.setTrackbarPos("S_max", "Controls", p["s_max"])
                cv2.setTrackbarPos("V_min", "Controls", p["v_min"])
                cv2.setTrackbarPos("V_max", "Controls", p["v_max"])
                last_preset = preset

            # 读取阈值
            h_min = cv2.getTrackbarPos("H_min", "Controls")
            h_max = cv2.getTrackbarPos("H_max", "Controls")
            s_min = cv2.getTrackbarPos("S_min", "Controls")
            s_max = cv2.getTrackbarPos("S_max", "Controls")
            v_min = cv2.getTrackbarPos("V_min", "Controls")
            v_max = cv2.getTrackbarPos("V_max", "Controls")

            # 颜色分割
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv, lower, upper)

            # 形态学处理
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 应用mask
            result = cv2.bitwise_and(frame, frame, mask=mask)

            # 计算面积占比
            pixel_count = cv2.countNonZero(mask)
            total_pixels = mask.shape[0] * mask.shape[1]
            ratio = pixel_count / total_pixels * 100

            # 在frame上显示阈值信息
            info = f"H:[{h_min},{h_max}] S:[{s_min},{s_max}] V:[{v_min},{v_max}] | Area:{ratio:.1f}%"
            cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            # 显示当前颜色样本
            color_sample = np.full((50, 100, 3), [(h_min + h_max) // 2, (s_min + s_max) // 2, (v_min + v_max) // 2], dtype=np.uint8)
            color_sample_bgr = cv2.cvtColor(color_sample, cv2.COLOR_HSV2BGR)
            frame[5:55, frame.shape[1] - 110:frame.shape[1] - 10] = color_sample_bgr

            # 拼接显示
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            top = np.hstack([frame, mask_bgr])
            cv2.imshow("Original | Mask", top)
            cv2.imshow("Result", result)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                config = {
                    "name": "custom",
                    "color_space": "HSV",
                    "lower": [int(h_min), int(s_min), int(v_min)],
                    "upper": [int(h_max), int(s_max), int(v_max)],
                    "area_ratio": round(ratio, 2)
                }
                fname = f"color_threshold_{datetime.now().strftime('%H%M%S')}.json"
                with open(fname, 'w') as f:
                    json.dump(config, f, indent=2, ensure_ascii=False)
                print(f"阈值配置已保存: {fname}")
                print(f"  lower = ({h_min}, {s_min}, {v_min})")
                print(f"  upper = ({h_max}, {s_max}, {v_max})")

        cap.release()
        cv2.destroyAllWindows()


# ============================================================
# 3. 形状检测模块
# ============================================================

class ShapeDetector:
    """形状检测：圆形、三角形、矩形、多边形"""

    def __init__(self, camera_id=0):
        self.camera_id = camera_id

    def detect(self, frame):
        """检测frame中的形状，返回检测结果列表"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=1)
        edges = cv2.erode(edges, kernel, iterations=1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:  # 过滤小区域
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue

            # 多边形近似
            epsilon = 0.02 * perimeter
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            vertices = len(approx)

            # 形状分类
            circularity = 4 * np.pi * area / (perimeter * perimeter)

            if circularity > 0.85:
                shape = "Circle"
                color = (0, 255, 0)
            elif vertices == 3:
                shape = "Triangle"
                color = (255, 0, 0)
            elif vertices == 4:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = w / float(h)
                if 0.85 < aspect_ratio < 1.15:
                    shape = "Square"
                else:
                    shape = "Rectangle"
                color = (0, 0, 255)
            elif vertices == 5:
                shape = "Pentagon"
                color = (255, 255, 0)
            elif vertices == 6:
                shape = "Hexagon"
                color = (255, 0, 255)
            else:
                shape = f"Polygon({vertices})"
                color = (128, 128, 128)

            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

            results.append({
                "shape": shape,
                "center": (cx, cy),
                "area": area,
                "vertices": vertices,
                "circularity": circularity,
                "contour": cnt,
                "color": color,
            })

            # 绘制结果
            cv2.drawContours(frame, [cnt], -1, color, 2)
            cv2.putText(frame, f"{shape} A:{int(area)}", (cx - 40, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.circle(frame, (cx, cy), 3, color, -1)

        return results

    def run(self):
        """实时形状检测"""
        cap = cv2.VideoCapture(self.camera_id)
        if not cap.isOpened():
            print(f"[ERROR] 无法打开摄像头 {self.camera_id}")
            return

        print("形状检测器已启动")
        print("操作: 'c'=Canny边缘视图 | 'd'=阈值调整 | 's'=截图 | 'q'=退出")

        show_canny = False

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            display = frame.copy()
            results = self.detect(display)

            # 统计信息
            shape_count = {}
            for r in results:
                shape_count[r["shape"]] = shape_count.get(r["shape"], 0) + 1

            info_parts = [f"{k}:{v}" for k, v in shape_count.items()]
            info = " | ".join(info_parts) if info_parts else "No shapes detected"
            cv2.putText(display, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Shape Detection", display)

            if show_canny:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                canny = cv2.Canny(blurred, 50, 150)
                cv2.imshow("Canny Edge", canny)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                show_canny = not show_canny
                if not show_canny:
                    cv2.destroyWindow("Canny Edge")
            elif key == ord('s'):
                fname = f"shape_{datetime.now().strftime('%H%M%S')}.jpg"
                cv2.imwrite(fname, display)
                print(f"截图已保存: {fname}, 检测到 {len(results)} 个形状")

        cap.release()
        cv2.destroyAllWindows()


# ============================================================
# 4. 图片静态分析
# ============================================================

def analyze_image(image_path):
    """对图片进行颜色+形状静态分析"""
    img = cv2.imread(image_path)
    if img is None:
        print(f"[ERROR] 无法读取图片: {image_path}")
        return

    print(f"图片尺寸: {img.shape[1]}x{img.shape[0]}")

    # 颜色统计
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h_mean, s_mean, v_mean = cv2.mean(hsv)[:3]
    print(f"平均HSV: H={h_mean:.0f}, S={s_mean:.0f}, V={v_mean:.0f}")

    # 形状检测
    detector = ShapeDetector()
    results = detector.detect(img)
    print(f"检测到 {len(results)} 个形状:")
    for i, r in enumerate(results):
        print(f"  [{i+1}] {r['shape']} @ {r['center']}, 面积={int(r['area'])}, 圆度={r['circularity']:.2f}")

    cv2.imshow("Analysis", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# ============================================================
# 5. 主入口
# ============================================================

def print_usage():
    print("""
╔══════════════════════════════════════════════╗
║        电赛视觉调试工具箱 v1.0              ║
╠══════════════════════════════════════════════╣
║  用法: python 视觉调试工具箱.py <command>    ║
║                                              ║
║  命令:                                       ║
║    camera [id]   - 摄像头测试（默认0）        ║
║    color  [id]   - 颜色阈值调试               ║
║    shape  [id]   - 形状检测                    ║
║    image  <path> - 图片静态分析                ║
║    list          - 列出可用摄像头              ║
║                                              ║
║  示例:                                       ║
║    python 视觉调试工具箱.py color 0           ║
║    python 视觉调试工具箱.py shape             ║
║    python 视觉调试工具箱.py image test.jpg    ║
╚══════════════════════════════════════════════╝
""")


def list_cameras():
    """尝试打开摄像头0~4，列出可用的"""
    print("正在扫描摄像头...")
    available = []
    for i in range(5):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            available.append(i)
            print(f"  摄像头 {i}: 可用 ({w}x{h})")
            cap.release()
        else:
            print(f"  摄像头 {i}: 不可用")
    if not available:
        print("未发现可用摄像头！")
    return available


def main():
    if len(sys.argv) < 2:
        print_usage()
        return

    cmd = sys.argv[1].lower()
    cam_id = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else 0

    if cmd == "camera":
        camera_test(cam_id)
    elif cmd == "color":
        debugger = ColorThresholdDebugger(cam_id)
        debugger.run()
    elif cmd == "shape":
        detector = ShapeDetector(cam_id)
        detector.run()
    elif cmd == "image":
        if len(sys.argv) < 3:
            print("请指定图片路径")
            return
        analyze_image(sys.argv[2])
    elif cmd == "list":
        list_cameras()
    else:
        print(f"未知命令: {cmd}")
        print_usage()


if __name__ == "__main__":
    main()
