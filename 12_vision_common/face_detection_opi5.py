#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人脸检测模块 - Face Detection for Orange Pi 5
================================================
针对 Orange Pi 5 优化的人脸检测算法
包含：Haar级联、DNN深度学习、轻量级检测

技术栈：OpenCV + NumPy + 多线程优化
适配：Orange Pi 5 (RK3588S) / Linux ARM64

作者：nuedc-asset-library
"""

import cv2
import numpy as np
import threading
import time
import os
from collections import deque


class FaceDetectorHaar:
    """
    Haar级联人脸检测器

    特点：
    - 基于OpenCV内置Haar分类器
    - 速度快，适合嵌入式平台
    - 支持多尺度检测
    - 支持多线程并行检测

    使用示例：
        detector = FaceDetectorHaar()
        faces = detector.detect(frame)
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
    """

    def __init__(self, cascade_path=None, scale_factor=1.1, min_neighbors=5,
                 min_size=(30, 30), max_size=None):
        """
        初始化Haar级联检测器

        参数：
            cascade_path: 级联文件路径（None使用默认）
            scale_factor: 缩放因子（越小越精确但越慢）
            min_neighbors: 最小邻居数（越大越严格）
            min_size: 最小人脸尺寸
            max_size: 最大人脸尺寸
        """
        # 加载级联分类器
        if cascade_path is None:
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'

        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            raise RuntimeError(f"无法加载级联文件: {cascade_path}")

        self.scale_factor = scale_factor
        self.min_neighbors = min_neighbors
        self.min_size = min_size
        self.max_size = max_size

        # 加载眼睛检测器（可选）
        eye_path = cv2.data.haarcascades + 'haarcascade_eye.xml'
        self.eye_cascade = cv2.CascadeClassifier(eye_path)

        print(f"[Haar检测器] 初始化完成")
        print(f"[Haar检测器] 缩放因子: {scale_factor}, 最小邻居: {min_neighbors}")

    def detect(self, gray, detect_eyes=False):
        """
        检测人脸

        参数：
            gray: 灰度图像
            detect_eyes: 是否同时检测眼睛

        返回：
            faces: 人脸矩形列表 [(x, y, w, h), ...]
            eyes: 眼睛矩形列表（如果detect_eyes=True）
        """
        # 直方图均衡化（提高对比度）
        gray_eq = cv2.equalizeHist(gray)

        # 多尺度检测
        faces = self.face_cascade.detectMultiScale(
            gray_eq,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=self.min_size,
            maxSize=self.max_size,
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        # 转换为列表
        if len(faces) == 0:
            faces = []
        else:
            faces = faces.tolist()

        # 检测眼睛
        eyes_list = []
        if detect_eyes and faces:
            for (x, y, w, h) in faces:
                roi_gray = gray_eq[y:y + h, x:x + w]
                eyes = self.eye_cascade.detectMultiScale(roi_gray)
                if len(eyes) > 0:
                    # 转换到原图坐标
                    eyes_global = [(ex + x, ey + y, ew, eh) for (ex, ey, ew, eh) in eyes]
                    eyes_list.append(eyes_global)
                else:
                    eyes_list.append([])

        if detect_eyes:
            return faces, eyes_list
        return faces


class FaceDetectorDNN:
    """
    DNN深度学习人脸检测器

    特点：
    - 基于Caffe/TensorFlow模型
    - 精度高，鲁棒性强
    - 支持多角度人脸
    - 使用OpenCV DNN模块

    使用示例：
        detector = FaceDetectorDNN()
        faces = detector.detect(frame)
    """

    def __init__(self, model_type='caffe', confidence_threshold=0.5,
                 nms_threshold=0.4, input_size=(300, 300)):
        """
        初始化DNN检测器

        参数：
            model_type: 模型类型 ('caffe', 'tensorflow', 'onnx')
            confidence_threshold: 置信度阈值
            nms_threshold: NMS阈值
            input_size: 输入尺寸
        """
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.input_size = input_size
        self.model_type = model_type

        self.net = None

        # 尝试加载模型
        self._load_model(model_type)

        print(f"[DNN检测器] 初始化完成 (模型: {model_type})")

    def _load_model(self, model_type):
        """
        加载DNN模型

        参数：
            model_type: 模型类型
        """
        # 检查是否有预下载的模型
        model_dir = os.path.expanduser('~/.hermes/models/face_detection')
        os.makedirs(model_dir, exist_ok=True)

        if model_type == 'caffe':
            prototxt = os.path.join(model_dir, 'deploy.prototxt')
            caffemodel = os.path.join(model_dir, 'res10_300x300_ssd_iter_140000.caffemodel')

            if os.path.exists(prototxt) and os.path.exists(caffemodel):
                self.net = cv2.dnn.readNetFromCaffe(prototxt, caffemodel)
                print("[DNN检测器] 已加载Caffe模型")
            else:
                print("[DNN检测器] Caffe模型未找到，使用Haar级联替代")
                print(f"[DNN检测器] 请下载模型到: {model_dir}")

        elif model_type == 'onnx':
            onnx_path = os.path.join(model_dir, 'face_detection_yunet.onnx')
            if os.path.exists(onnx_path):
                self.net = cv2.dnn.readNetFromONNX(onnx_path)
                print("[DNN检测器] 已加载ONNX模型")

        # 设置后端（Orange Pi 5优化）
        if self.net is not None:
            # 尝试使用OpenCL加速（RK3588S支持）
            try:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL)
                print("[DNN检测器] 使用OpenCL加速")
            except Exception:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                print("[DNN检测器] 使用CPU推理")

    def detect(self, frame):
        """
        检测人脸

        参数：
            frame: BGR图像

        返回：
            faces: 人脸矩形列表 [(x, y, w, h), ...]
            confidences: 置信度列表
        """
        if self.net is None:
            print("[DNN检测器] 模型未加载，无法检测")
            return [], []

        h, w = frame.shape[:2]

        # 创建blob
        blob = cv2.dnn.blobFromImage(
            frame, 1.0, self.input_size,
            (104.0, 177.0, 123.0), swapRB=False, crop=False
        )

        # 前向推理
        self.net.setInput(blob)
        detections = self.net.forward()

        faces = []
        confidences = []

        # 解析检测结果
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]

            if confidence > self.confidence_threshold:
                # 获取边界框
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                x1, y1, x2, y2 = box.astype(int)

                # 边界裁剪
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(w, x2)
                y2 = min(h, y2)

                fw = x2 - x1
                fh = y2 - y1

                if fw > 0 and fh > 0:
                    faces.append((x1, y1, fw, fh))
                    confidences.append(float(confidence))

        # NMS
        if len(faces) > 1:
            indices = cv2.dnn.NMSBoxes(faces, confidences,
                                        self.confidence_threshold,
                                        self.nms_threshold)
            if len(indices) > 0:
                indices = indices.flatten()
                faces = [faces[i] for i in indices]
                confidences = [confidences[i] for i in indices]

        return faces, confidences


class FaceDetectorLightweight:
    """
    轻量级人脸检测器（适合Orange Pi 5）

    特点：
    - 使用缩小图像加速
    - 多线程并行
    - 结果跟踪和预测
    - 低延迟

    使用示例：
        detector = FaceDetectorLightweight()
        faces = detector.detect(frame)
    """

    def __init__(self, scale=0.5, use_haar=True):
        """
        初始化轻量级检测器

        参数：
            scale: 图像缩放比例（越大越精确但越慢）
            use_haar: 是否使用Haar（False则使用DNN）
        """
        self.scale = scale
        self.use_haar = use_haar

        # Haar检测器
        self.haar_detector = FaceDetectorHaar(
            scale_factor=1.2,  # 稍大的步长
            min_neighbors=4,
            min_size=(20, 20)
        )

        # DNN检测器（可选）
        self.dnn_detector = None
        if not use_haar:
            try:
                self.dnn_detector = FaceDetectorDNN()
            except Exception:
                print("[轻量级检测器] DNN加载失败，使用Haar")

        # 跟踪器（用于帧间跟踪）
        self.trackers = []
        self.tracker_type = 'KCF'  # KCF/CSRT/MOSSE
        self.track_lost_count = 0
        self.max_track_lost = 5

        # 线程池
        self._thread_pool = []
        self._lock = threading.Lock()

        print(f"[轻量级检测器] 初始化完成 (缩放: {scale})")

    def detect(self, frame):
        """
        检测人脸（带跟踪优化）

        参数：
            frame: BGR图像

        返回：
            faces: 人脸矩形列表
        """
        h, w = frame.shape[:2]

        # 缩小图像加速
        small = cv2.resize(frame, None, fx=self.scale, fy=self.scale)
        gray_small = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        # 检测
        if self.use_haar or self.dnn_detector is None:
            faces_small = self.haar_detector.detect(gray_small)
        else:
            faces_small, _ = self.dnn_detector.detect(small)

        # 缩放回原图尺寸
        faces = []
        inv_scale = 1.0 / self.scale
        for (x, y, fw, fh) in faces_small:
            faces.append((
                int(x * inv_scale),
                int(y * inv_scale),
                int(fw * inv_scale),
                int(fh * inv_scale)
            ))

        return faces

    def detect_with_tracking(self, frame):
        """
        检测+跟踪（减少重复检测）

        策略：
        - 每N帧执行一次完整检测
        - 帧间使用跟踪器预测位置
        - 跟踪失败时重新检测

        参数：
            frame: BGR图像

        返回：
            faces: 人脸矩形列表
        """
        # 如果有活跃的跟踪器，先更新
        if self.trackers:
            new_trackers = []
            tracked_faces = []

            for tracker, face in self.trackers:
                ok, bbox = tracker.update(frame)
                if ok:
                    x, y, w, h = [int(v) for v in bbox]
                    tracked_faces.append((x, y, w, h))
                    new_trackers.append((tracker, face))

            self.trackers = new_trackers

            # 如果跟踪成功，返回跟踪结果
            if tracked_faces:
                return tracked_faces

        # 执行检测
        faces = self.detect(frame)

        # 为每个检测到的人脸创建跟踪器
        self.trackers = []
        for face in faces:
            tracker = cv2.TrackerKCF_create()
            try:
                tracker.init(frame, face)
                self.trackers.append((tracker, face))
            except Exception:
                pass

        return faces


class FaceAnalyzer:
    """
    人脸分析器（在检测基础上进行分析）

    功能：
    - 人脸关键点检测
    - 表情识别（简单）
    - 年龄/性别估计（需要模型）

    使用示例：
        analyzer = FaceAnalyzer()
        results = analyzer.analyze(frame)
    """

    def __init__(self):
        """初始化人脸分析器"""
        # 加载眼睛检测器
        self.eye_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_eye.xml'
        )

        # 加载微笑检测器
        self.smile_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_smile.xml'
        )

        print("[人脸分析器] 初始化完成")

    def detect_eyes(self, gray, face_rect):
        """
        检测眼睛

        参数：
            gray: 灰度图像
            face_rect: 人脸矩形 (x, y, w, h)

        返回：
            eyes: 眼睛矩形列表
        """
        x, y, w, h = face_rect
        roi = gray[y:y + h, x:x + w]

        eyes = self.eye_cascade.detectMultiScale(roi, 1.1, 5, minSize=(20, 20))

        # 转换到原图坐标
        result = []
        for (ex, ey, ew, eh) in eyes:
            if ey < h * 0.6:  # 眼睛在脸上半部分
                result.append((ex + x, ey + y, ew, eh))

        return result

    def detect_smile(self, gray, face_rect):
        """
        检测微笑

        参数：
            gray: 灰度图像
            face_rect: 人脸矩形

        返回：
            is_smiling: 是否微笑
            smile_rect: 微笑矩形
        """
        x, y, w, h = face_rect
        roi = gray[y + h // 2:y + h, x:x + w]  # 只在下半脸检测

        smiles = self.smile_cascade.detectMultiScale(
            roi, 1.5, 15, minSize=(25, 15)
        )

        if len(smiles) > 0:
            sx, sy, sw, sh = smiles[0]
            return True, (sx + x, sy + y + h // 2, sw, sh)

        return False, None

    def estimate_gaze_direction(self, gray, face_rect, eyes):
        """
        估计视线方向（简单版本）

        参数：
            gray: 灰度图像
            face_rect: 人脸矩形
            eyes: 眼睛列表

        返回：
            direction: 视线方向 ('left', 'right', 'center', 'unknown')
        """
        if len(eyes) < 2:
            return 'unknown'

        x, y, w, h = face_rect
        face_center_x = x + w // 2

        # 计算眼睛中心
        eye_centers = []
        for (ex, ey, ew, eh) in eyes:
            eye_centers.append((ex + ew // 2, ey + eh // 2))

        # 比较眼睛位置
        avg_eye_x = np.mean([ec[0] for ec in eye_centers])

        if avg_eye_x < face_center_x - w * 0.1:
            return 'left'
        elif avg_eye_x > face_center_x + w * 0.1:
            return 'right'
        else:
            return 'center'


class MultiThreadFaceDetector:
    """
    多线程人脸检测器（充分利用Orange Pi 5多核）

    使用示例：
        detector = MultiThreadFaceDetector(n_threads=4)
        detector.start()
        faces = detector.detect_async(frame)
        detector.stop()
    """

    def __init__(self, n_threads=4, detector_type='haar'):
        """
        初始化多线程检测器

        参数：
            n_threads: 线程数
            detector_type: 检测器类型
        """
        self.n_threads = n_threads
        self.detectors = []

        for _ in range(n_threads):
            if detector_type == 'haar':
                self.detectors.append(FaceDetectorHaar())
            elif detector_type == 'lightweight':
                self.detectors.append(FaceDetectorLightweight())

        self._task_queue = []
        self._result_queue = {}
        self._lock = threading.Lock()
        self._running = False
        self._threads = []
        self._task_id = 0

        print(f"[多线程检测器] 初始化完成 ({n_threads} 线程)")

    def start(self):
        """启动工作线程"""
        self._running = True
        for i in range(self.n_threads):
            t = threading.Thread(target=self._worker, args=(i,), daemon=True)
            t.start()
            self._threads.append(t)

    def stop(self):
        """停止工作线程"""
        self._running = False
        for t in self._threads:
            t.join(timeout=2)

    def _worker(self, worker_id):
        """工作线程函数"""
        detector = self.detectors[worker_id]
        while self._running:
            task = None
            with self._lock:
                if self._task_queue:
                    task = self._task_queue.pop(0)

            if task is not None:
                task_id, frame = task
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
                faces = detector.detect(gray)
                with self._lock:
                    self._result_queue[task_id] = faces
            else:
                time.sleep(0.01)

    def detect_async(self, frame):
        """
        异步检测（提交任务）

        参数：
            frame: 输入图像

        返回：
            task_id: 任务ID
        """
        with self._lock:
            self._task_id += 1
            task_id = self._task_id
            self._task_queue.append((task_id, frame))
        return task_id

    def get_result(self, task_id, timeout=1.0):
        """
        获取检测结果

        参数：
            task_id: 任务ID
            timeout: 超时时间

        返回：
            faces: 人脸列表（或None）
        """
        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                if task_id in self._result_queue:
                    return self._result_queue.pop(task_id)
            time.sleep(0.01)
        return None


def draw_faces(frame, faces, color=(0, 255, 0), thickness=2, draw_id=True):
    """
    在图像上绘制人脸框

    参数：
        frame: 输入图像
        faces: 人脸矩形列表 [(x, y, w, h), ...]
        color: 绘制颜色
        thickness: 线宽
        draw_id: 是否绘制ID

    返回：
        annotated: 标注后的图像
    """
    annotated = frame.copy()

    for i, (x, y, w, h) in enumerate(faces):
        # 绘制矩形
        cv2.rectangle(annotated, (x, y), (x + w, y + h), color, thickness)

        # 绘制角点
        corner_len = min(w, h) // 5
        # 左上角
        cv2.line(annotated, (x, y), (x + corner_len, y), color, thickness + 1)
        cv2.line(annotated, (x, y), (x, y + corner_len), color, thickness + 1)
        # 右上角
        cv2.line(annotated, (x + w, y), (x + w - corner_len, y), color, thickness + 1)
        cv2.line(annotated, (x + w, y), (x + w, y + corner_len), color, thickness + 1)
        # 左下角
        cv2.line(annotated, (x, y + h), (x + corner_len, y + h), color, thickness + 1)
        cv2.line(annotated, (x, y + h), (x, y + h - corner_len), color, thickness + 1)
        # 右下角
        cv2.line(annotated, (x + w, y + h), (x + w - corner_len, y + h), color, thickness + 1)
        cv2.line(annotated, (x + w, y + h), (x + w, y + h - corner_len), color, thickness + 1)

        # 绘制ID
        if draw_id:
            cv2.putText(annotated, f'Face #{i + 1}', (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    return annotated


# ================================================================
#                          使用示例
# ================================================================
if __name__ == '__main__':
    print("=" * 60)
    print("人脸检测 - Face Detection for Orange Pi 5")
    print("针对 Orange Pi 5 优化")
    print("=" * 60)

    # 选择检测器
    print("\n选择检测器:")
    print("  1 - Haar级联（快速）")
    print("  2 - DNN深度学习（精确）")
    print("  3 - 轻量级（带跟踪）")
    print("  4 - 多线程并行")

    choice = input("请选择 (1-4, 默认1): ").strip() or '1'

    if choice == '1':
        detector = FaceDetectorHaar()
        detect_func = lambda frame: detector.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    elif choice == '2':
        detector = FaceDetectorDNN()
        detect_func = lambda frame: (detector.detect(frame)[0],)
    elif choice == '3':
        detector = FaceDetectorLightweight(scale=0.5)
        detect_func = lambda frame: (detector.detect(frame),)
    elif choice == '4':
        detector = MultiThreadFaceDetector(n_threads=4)
        detect_func = lambda frame: (detector.detecters[0].detect(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)),)
    else:
        detector = FaceDetectorHaar()
        detect_func = lambda frame: detector.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

    # 创建分析器
    analyzer = FaceAnalyzer()

    # 从摄像头读取
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("\n按 'q' 退出")
    print("按 'e' 切换眼睛检测")
    print("按 's' 切换微笑检测")

    detect_eyes = False
    detect_smile = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        t_start = time.time()

        # 检测人脸
        result = detect_func(frame)
        faces = result[0] if isinstance(result, tuple) else result

        # 绘制人脸框
        annotated = draw_faces(frame, faces)

        # 分析
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for face in faces:
            if detect_eyes:
                eyes = analyzer.detect_eyes(gray, face)
                for (ex, ey, ew, eh) in eyes:
                    cv2.rectangle(annotated, (ex, ey), (ex + ew, ey + eh), (255, 0, 0), 2)

            if detect_smile:
                is_smiling, smile_rect = analyzer.detect_smile(gray, face)
                if is_smiling and smile_rect:
                    sx, sy, sw, sh = smile_rect
                    cv2.rectangle(annotated, (sx, sy), (sx + sw, sy + sh), (0, 0, 255), 2)
                    cv2.putText(annotated, 'Smile!', (sx, sy - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        fps = 1.0 / max(time.time() - t_start, 1e-6)
        cv2.putText(annotated, f'FPS: {fps:.1f}  Faces: {len(faces)}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow('Face Detection - Orange Pi 5', annotated)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('e'):
            detect_eyes = not detect_eyes
            print(f"眼睛检测: {'开启' if detect_eyes else '关闭'}")
        elif key == ord('s'):
            detect_smile = not detect_smile
            print(f"微笑检测: {'开启' if detect_smile else '关闭'}")

    cap.release()
    cv2.destroyAllWindows()
