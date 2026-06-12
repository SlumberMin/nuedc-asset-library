#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交通标志识别模块 - Traffic Sign Recognition
============================================
针对 Orange Pi 5 优化的交通标志识别算法
包含：颜色分割、形状检测、模板匹配、分类

技术栈：OpenCV + NumPy + 多线程优化
适配：Orange Pi 5 (RK3588S) / Linux ARM64

作者：nuedc-asset-library
"""

import cv2
import numpy as np
import os
import threading
import time
from collections import defaultdict


class TrafficSignRecognizer:
    """
    交通标志识别器

    支持标志类型：
    - 红色圆形：禁止标志（禁止通行、限速等）
    - 蓝色圆形：指示标志（直行、左转等）
    - 黄色三角形：警告标志（注意行人、弯道等）
    - 红色八边形：停车标志（STOP）
    - 红色倒三角：让行标志

    使用示例：
        recognizer = TrafficSignRecognizer()
        results = recognizer.recognize(frame)
        for r in results:
            print(r['type'], r['shape'], r['position'])
    """

    def __init__(self, template_dir=None, min_area=500, max_area=50000):
        """
        初始化交通标志识别器

        参数：
            template_dir: 模板图像目录（可选）
            min_area: 最小检测面积
            max_area: 最大检测面积
        """
        # ==================== 检测参数 ====================
        self.min_area = min_area
        self.max_area = max_area
        self.min_circularity = 0.7  # 圆形度阈值
        self.min_aspect_ratio = 0.5
        self.max_aspect_ratio = 2.0

        # ==================== 颜色阈值（HSV空间）====================
        # 红色（两个范围，因为红色在HSV中跨越0°）
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])

        # 蓝色
        self.blue_lower = np.array([100, 100, 100])
        self.blue_upper = np.array([130, 255, 255])

        # 黄色
        self.yellow_lower = np.array([15, 100, 100])
        self.yellow_upper = np.array([35, 255, 255])

        # 白色
        self.white_lower = np.array([0, 0, 200])
        self.white_upper = np.array([180, 30, 255])

        # ==================== 标志类型映射 ====================
        self.sign_types = {
            ('red', 'circle'): '禁止标志',
            ('red', 'octagon'): '停车标志',
            ('red', 'triangle_down'): '让行标志',
            ('blue', 'circle'): '指示标志',
            ('yellow', 'triangle'): '警告标志',
            ('white', 'rectangle'): '信息标志',
        }

        # ==================== 模板匹配 ====================
        self.templates = {}  # {sign_name: [template_images]}
        self.template_size = (64, 64)  # 统一模板大小

        if template_dir and os.path.exists(template_dir):
            self._load_templates(template_dir)

        # ==================== 多线程 ====================
        self._lock = threading.Lock()

        print("[交通标志识别] 初始化完成")
        print(f"[交通标志识别] 面积范围: {min_area} - {max_area}")

    def _load_templates(self, template_dir):
        """
        加载模板图像

        参数：
            template_dir: 模板目录路径
        """
        for filename in os.listdir(template_dir):
            if filename.endswith(('.png', '.jpg', '.bmp')):
                name = os.path.splitext(filename)[0]
                filepath = os.path.join(template_dir, filename)
                img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    img = cv2.resize(img, self.template_size)
                    if name not in self.templates:
                        self.templates[name] = []
                    self.templates[name].append(img)

        print(f"[交通标志识别] 加载了 {len(self.templates)} 类模板")

    def detect_color_mask(self, hsv, color):
        """
        根据颜色生成掩码

        参数：
            hsv: HSV图像
            color: 颜色名称 ('red', 'blue', 'yellow', 'white')

        返回：
            mask: 二值掩码
        """
        if color == 'red':
            mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
            mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        elif color == 'blue':
            mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        elif color == 'yellow':
            mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        elif color == 'white':
            mask = cv2.inRange(hsv, self.white_lower, self.white_upper)
        else:
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        # 形态学操作：去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        return mask

    def detect_shape(self, contour):
        """
        检测轮廓形状

        参数：
            contour: 轮廓

        返回：
            shape: 形状名称
            vertices: 顶点坐标
        """
        # 计算轮廓周长
        peri = cv2.arcLength(contour, True)

        # 多边形近似
        epsilon = 0.03 * peri
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # 计算面积和边界矩形
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)

        # 计算圆形度
        circularity = 4 * np.pi * area / (peri * peri) if peri > 0 else 0

        # 计算纵横比
        aspect_ratio = float(w) / h if h > 0 else 0

        num_vertices = len(approx)

        # 形状判断
        if num_vertices == 3:
            # 三角形：判断正三角还是倒三角
            if self._is_inverted_triangle(contour):
                return 'triangle_down', approx
            return 'triangle', approx

        elif num_vertices == 4:
            # 四边形：判断矩形还是正方形
            if 0.85 < aspect_ratio < 1.15:
                return 'square', approx
            return 'rectangle', approx

        elif num_vertices == 5:
            return 'pentagon', approx

        elif num_vertices == 6:
            return 'hexagon', approx

        elif num_vertices == 8:
            return 'octagon', approx

        elif num_vertices > 8 and circularity > self.min_circularity:
            return 'circle', approx

        else:
            return 'unknown', approx

    def _is_inverted_triangle(self, contour):
        """
        判断是否为倒三角形

        参数：
            contour: 轮廓

        返回：
            bool: 是否为倒三角
        """
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return False

        cy = int(M['m01'] / M['m00'])
        x, y, w, h = cv2.boundingRect(contour)

        # 质心在下半部分则为倒三角
        return cy > y + h * 0.6

    def template_match(self, roi_gray):
        """
        模板匹配

        参数：
            roi_gray: 灰度ROI图像

        返回：
            best_match: 最佳匹配名称
            best_score: 匹配分数
        """
        if not self.templates:
            return None, 0.0

        roi_resized = cv2.resize(roi_gray, self.template_size)

        best_match = None
        best_score = 0

        for name, templates in self.templates.items():
            for template in templates:
                # 相关系数匹配
                result = cv2.matchTemplate(roi_resized, template, cv2.TM_CCOEFF_NORMED)
                score = result[0][0] if result.size > 0 else 0

                if score > best_score:
                    best_score = score
                    best_match = name

        return best_match, best_score

    def recognize(self, frame, use_template=True):
        """
        识别图像中的交通标志

        参数：
            frame: 输入BGR图像
            use_template: 是否使用模板匹配

        返回：
            results: 识别结果列表 [{
                'type': 标志类型,
                'shape': 形状,
                'color': 颜色,
                'position': (x, y, w, h),
                'confidence': 置信度,
                'template_match': 模板匹配结果
            }]
        """
        results = []
        h, w = frame.shape[:2]

        # 预处理
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 高斯模糊去噪
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

        # 对每种颜色进行检测
        for color in ['red', 'blue', 'yellow']:
            mask = self.detect_color_mask(hsv, color)

            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)

                # 面积过滤
                if area < self.min_area or area > self.max_area:
                    continue

                # 检测形状
                shape, vertices = self.detect_shape(contour)

                if shape == 'unknown':
                    continue

                # 获取边界矩形
                x, y, bw, bh = cv2.boundingRect(contour)

                # 纵横比过滤
                aspect = float(bw) / bh if bh > 0 else 0
                if aspect < self.min_aspect_ratio or aspect > self.max_aspect_ratio:
                    continue

                # 扩展ROI
                pad = 10
                x1 = max(0, x - pad)
                y1 = max(0, y - pad)
                x2 = min(w, x + bw + pad)
                y2 = min(h, y + bh + pad)

                roi_gray = gray[y1:y2, x1:x2]

                # 确定标志类型
                sign_type = self.sign_types.get((color, shape), f'{color}_{shape}')

                # 模板匹配
                template_match = None
                confidence = 0.7  # 基础置信度

                if use_template and roi_gray.size > 0:
                    template_match, match_score = self.template_match(roi_gray)
                    if match_score > 0.5:
                        confidence = 0.5 + 0.5 * match_score

                results.append({
                    'type': sign_type,
                    'shape': shape,
                    'color': color,
                    'position': (x, y, bw, bh),
                    'confidence': confidence,
                    'template_match': template_match,
                    'contour': contour,
                })

        # 非极大值抑制
        results = self._nms(results, iou_thresh=0.3)

        return results

    def _nms(self, results, iou_thresh=0.3):
        """
        非极大值抑制（去除重叠检测）

        参数：
            results: 检测结果列表
            iou_thresh: IoU阈值

        返回：
            filtered: 过滤后的结果
        """
        if len(results) <= 1:
            return results

        # 按置信度排序
        results.sort(key=lambda x: x['confidence'], reverse=True)

        filtered = []
        used = set()

        for i, r1 in enumerate(results):
            if i in used:
                continue

            filtered.append(r1)

            for j, r2 in enumerate(results[i + 1:], i + 1):
                if j in used:
                    continue

                iou = self._compute_iou(r1['position'], r2['position'])
                if iou > iou_thresh:
                    used.add(j)

        return filtered

    def _compute_iou(self, box1, box2):
        """
        计算两个矩形的IoU

        参数：
            box1: (x, y, w, h)
            box2: (x, y, w, h)

        返回：
            iou: 交并比
        """
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2

        # 交集
        xi1 = max(x1, x2)
        yi1 = max(y1, y2)
        xi2 = min(x1 + w1, x2 + w2)
        yi2 = min(y1 + h1, y2 + h2)

        if xi2 <= xi1 or yi2 <= yi1:
            return 0

        intersection = (xi2 - xi1) * (yi2 - yi1)
        union = w1 * h1 + w2 * h2 - intersection

        return intersection / union if union > 0 else 0

    def draw_results(self, frame, results):
        """
        在图像上绘制识别结果

        参数：
            frame: 输入图像
            results: 识别结果列表

        返回：
            annotated: 标注后的图像
        """
        annotated = frame.copy()

        # 颜色映射
        color_map = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255),
            'white': (255, 255, 255),
        }

        for r in results:
            x, y, w, h = r['position']
            color = color_map.get(r['color'], (0, 255, 0))

            # 绘制边界框
            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)

            # 绘制轮廓
            if 'contour' in r:
                cv2.drawContours(annotated, [r['contour']], -1, color, 2)

            # 标签文字
            label = r['type']
            if r['template_match']:
                label = f"{r['type']}: {r['template_match']}"

            conf_text = f" ({r['confidence']:.0%})"
            label += conf_text

            # 文字背景
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(annotated, (x, y - th - 10), (x + tw, y), color, -1)
            cv2.putText(annotated, label, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return annotated


class TrafficSignDetectionPipeline:
    """
    交通标志检测流水线（支持多线程）

    使用示例：
        pipeline = TrafficSignDetectionPipeline(camera_id=0)
        pipeline.start()

        while True:
            result = pipeline.get_result()
            if result is not None:
                cv2.imshow('Traffic Signs', result)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        pipeline.stop()
    """

    def __init__(self, camera_id=0, resolution=(640, 480), template_dir=None):
        """
        初始化流水线

        参数：
            camera_id: 摄像头ID
            resolution: 分辨率
            template_dir: 模板目录
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.recognizer = TrafficSignRecognizer(template_dir=template_dir)

        self._cap = None
        self._frame = None
        self._result = None
        self._running = False
        self._lock = threading.Lock()
        self._threads = []

    def start(self):
        """启动流水线"""
        self._cap = cv2.VideoCapture(self.camera_id)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

        self._running = True

        # 采集线程
        t1 = threading.Thread(target=self._capture_loop, daemon=True)
        t1.start()
        self._threads.append(t1)

        # 处理线程
        t2 = threading.Thread(target=self._process_loop, daemon=True)
        t2.start()
        self._threads.append(t2)

        print("[标志识别流水线] 已启动")

    def stop(self):
        """停止流水线"""
        self._running = False
        for t in self._threads:
            t.join(timeout=2)
        if self._cap:
            self._cap.release()
        print("[标志识别流水线] 已停止")

    def _capture_loop(self):
        """采集循环"""
        while self._running:
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._frame = frame

    def _process_loop(self):
        """处理循环"""
        while self._running:
            frame = None
            with self._lock:
                if self._frame is not None:
                    frame = self._frame.copy()

            if frame is not None:
                results = self.recognizer.recognize(frame)
                annotated = self.recognizer.draw_results(frame, results)
                with self._lock:
                    self._result = annotated

    def get_result(self):
        """获取最新结果"""
        with self._lock:
            return self._result


# ================================================================
#                          使用示例
# ================================================================
if __name__ == '__main__':
    print("=" * 60)
    print("交通标志识别 - Traffic Sign Recognition")
    print("针对 Orange Pi 5 优化")
    print("=" * 60)

    # 创建识别器
    recognizer = TrafficSignRecognizer()

    # 从摄像头读取
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("\n按 'q' 退出")
    print("按 't' 切换模板匹配")
    print("按 'c' 切换颜色检测")

    use_template = True

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        t_start = time.time()
        results = recognizer.recognize(frame, use_template=use_template)
        annotated = recognizer.draw_results(frame, results)

        fps = 1.0 / max(time.time() - t_start, 1e-6)
        cv2.putText(annotated, f'FPS: {fps:.1f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # 显示检测数量
        cv2.putText(annotated, f'Signs: {len(results)}', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow('Traffic Sign Recognition', annotated)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('t'):
            use_template = not use_template
            print(f"模板匹配: {'开启' if use_template else '关闭'}")

    cap.release()
    cv2.destroyAllWindows()
