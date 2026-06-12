"""
表情识别模块 - 人脸特征点 + 表情分类 + 情绪分析
功能：基于OpenCV和dlib实现人脸表情识别
依赖：opencv-python, numpy, dlib (可选)
适用：电赛中人机交互、情感计算等场景
"""

import cv2
import numpy as np
from collections import deque

# ============================================================
# 基于几何特征的表情识别（无需深度学习模型）
# ============================================================

class EmotionRecognizer:
    """
    基于人脸几何特征的表情识别器
    通过分析眼睛开合度、嘴巴张开度、眉毛位置等判断情绪
    """

    # 情绪标签
    EMOTIONS = ['neutral', 'happy', 'surprise', 'sad', 'angry']

    def __init__(self, history_len=10):
        """
        初始化表情识别器
        Args:
            history_len: 情绪历史平滑窗口长度
        """
        # 加载Haar级联分类器
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        self.eye_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_eye.xml'
        )
        self.smile_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_smile.xml'
        )

        # 情绪历史缓冲（用于平滑）
        self.emotion_history = deque(maxlen=history_len)

        # 特征阈值（可根据实际场景调整）
        self.thresholds = {
            'eye_open_ratio': 0.25,      # 眼睛张开比率阈值
            'mouth_open_ratio': 0.35,     # 嘴巴张开比率阈值
            'smile_confidence': 0.6,      # 微笑检测置信度
        }

    def detect_face(self, gray):
        """
        检测人脸区域
        Args:
            gray: 灰度图
        Returns:
            faces: 人脸矩形列表 [(x, y, w, h), ...]
        """
        faces = self.face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(80, 80)
        )
        return faces

    def analyze_facial_features(self, gray, face_rect):
        """
        分析人脸区域内的面部特征
        Args:
            gray: 灰度图
            face_rect: 人脸矩形 (x, y, w, h)
        Returns:
            features: 特征字典
        """
        x, y, w, h = face_rect
        roi_gray = gray[y:y+h, x:x+w]

        features = {
            'face_size': (w, h),
            'eye_count': 0,
            'eye_open_ratio': 0.0,
            'smile_detected': False,
            'smile_neighbors': 0,
            'face_brightness': np.mean(roi_gray),
            'upper_face_brightness': 0.0,
            'lower_face_brightness': 0.0,
        }

        # 检测眼睛
        eyes = self.eye_cascade.detectMultiScale(
            roi_gray, scaleFactor=1.1, minNeighbors=5, minSize=(20, 15)
        )
        features['eye_count'] = len(eyes)

        if len(eyes) > 0:
            # 计算眼睛平均开合度
            eye_ratios = []
            for (ex, ey, ew, eh) in eyes:
                ratio = eh / h  # 眼睛高度相对于人脸的比例
                eye_ratios.append(ratio)
            features['eye_open_ratio'] = np.mean(eye_ratios)

        # 检测微笑/嘴巴
        roi_lower = roi_gray[int(h*0.5):h, :]  # 下半部分人脸
        if roi_lower.size > 0:
            features['lower_face_brightness'] = np.mean(roi_lower)

        smiles = self.smile_cascade.detectMultiScale(
            roi_gray, scaleFactor=1.7, minNeighbors=22, minSize=(25, 15)
        )
        if len(smiles) > 0:
            features['smile_detected'] = True
            features['smile_neighbors'] = len(smiles)

        # 上半脸亮度（用于皱眉检测）
        roi_upper = roi_gray[0:int(h*0.4), :]
        if roi_upper.size > 0:
            features['upper_face_brightness'] = np.mean(roi_upper)

        return features

    def classify_emotion(self, features):
        """
        基于特征进行情绪分类
        Args:
            features: 面部特征字典
        Returns:
            emotion: 情绪标签
            confidence: 置信度 [0, 1]
            scores: 各情绪得分
        """
        scores = {e: 0.0 for e in self.EMOTIONS}

        eye_open = features['eye_open_ratio']
        smile = features['smile_detected']
        smile_n = features['smile_neighbors']

        # 高兴：微笑检测 + 眼睛微闭
        if smile:
            scores['happy'] += 0.4 + 0.1 * min(smile_n, 3)
            if eye_open < self.thresholds['eye_open_ratio']:
                scores['happy'] += 0.2

        # 惊讶：眼睛大睁 + 嘴巴张开
        if eye_open > self.thresholds['eye_open_ratio'] * 1.3:
            scores['surprise'] += 0.3
        if not smile and eye_open > self.thresholds['eye_open_ratio']:
            scores['surprise'] += 0.2

        # 悲伤：眼睛半闭
        if eye_open < self.thresholds['eye_open_ratio'] * 0.7 and not smile:
            scores['sad'] += 0.3

        # 愤怒：眉头紧皱（上半脸较暗）+ 无微笑
        upper_b = features['upper_face_brightness']
        lower_b = features['lower_face_brightness']
        if upper_b > 0 and upper_b < lower_b * 0.85 and not smile:
            scores['angry'] += 0.3

        # 中性：默认
        if not smile and eye_open < self.thresholds['eye_open_ratio'] * 1.1:
            scores['neutral'] += 0.2

        # 归一化
        total = sum(scores.values())
        if total > 0:
            for k in scores:
                scores[k] /= total
        else:
            scores['neutral'] = 1.0

        best_emotion = max(scores, key=scores.get)
        confidence = scores[best_emotion]

        return best_emotion, confidence, scores

    def recognize(self, frame):
        """
        对输入帧进行完整的表情识别
        Args:
            frame: BGR彩色图像
        Returns:
            results: 每个人脸的识别结果列表
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.detect_face(gray)
        results = []

        for face_rect in faces:
            features = self.analyze_facial_features(gray, face_rect)
            emotion, confidence, scores = self.classify_emotion(features)

            # 时间平滑
            self.emotion_history.append(emotion)
            smoothed_emotion = max(set(self.emotion_history),
                                   key=self.emotion_history.count)

            results.append({
                'bbox': face_rect,
                'emotion': smoothed_emotion,
                'confidence': confidence,
                'scores': scores,
                'features': features,
            })

        return results

    def draw_results(self, frame, results):
        """
        在图像上绘制识别结果
        Args:
            frame: 原始图像
            results: recognize()的返回结果
        Returns:
            vis: 可视化图像
        """
        vis = frame.copy()

        # 情绪对应颜色
        color_map = {
            'neutral': (200, 200, 200),
            'happy': (0, 255, 255),
            'surprise': (0, 165, 255),
            'sad': (255, 100, 100),
            'angry': (0, 0, 255),
        }

        # 中文情绪标签
        emotion_cn = {
            'neutral': 'ZhongXing', 'happy': 'GaoXing',
            'surprise': 'JingYa', 'sad': 'BeiShang', 'angry': 'FenNu',
        }

        for res in results:
            x, y, w, h = res['bbox']
            emotion = res['emotion']
            conf = res['confidence']
            color = color_map.get(emotion, (255, 255, 255))

            # 绘制人脸框
            cv2.rectangle(vis, (x, y), (x+w, y+h), color, 2)

            # 绘制标签
            label = f"{emotion} ({conf:.1%})"
            cv2.putText(vis, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # 绘制情绪得分条
            bar_x = x + w + 10
            bar_y = y
            for i, (emo, score) in enumerate(res['scores'].items()):
                bar_w = int(score * 60)
                cv2.rectangle(vis, (bar_x, bar_y + i*18),
                              (bar_x + bar_w, bar_y + i*18 + 14),
                              color_map.get(emo, (200,200,200)), -1)
                cv2.putText(vis, emo[:3], (bar_x - 30, bar_y + i*18 + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1)

        return vis


# ============================================================
# 基于LK光流的表情变化检测（轻量级替代方案）
# ============================================================

class ExpressionChangeDetector:
    """
    基于光流的面部表情变化检测
    不依赖分类器，通过检测面部运动来判断表情变化
    """

    def __init__(self, grid_size=20):
        self.grid_size = grid_size
        self.prev_gray = None
        self.prev_points = None

    def detect_change(self, gray, face_rect):
        """
        检测面部区域的表情变化强度
        Args:
            gray: 灰度图
            face_rect: 人脸矩形
        Returns:
            motion_score: 运动强度 [0, 1]
            motion_field: 运动场 (dx, dy) 列表
        """
        x, y, w, h = face_rect
        roi = gray[y:y+h, x:x+w]

        # 在网格点上检测特征
        gs = self.grid_size
        points = []
        for gy in range(gs//2, h, gs):
            for gx in range(gs//2, w, gs):
                points.append([gx, gy])

        if not points:
            return 0.0, []

        points = np.float32(points).reshape(-1, 1, 2)

        motion_score = 0.0
        motion_field = []

        if self.prev_gray is not None and self.prev_points is not None:
            # 光流追踪
            new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, roi, self.prev_points, None
            )
            if new_pts is not None:
                good_old = self.prev_points[status.flatten() == 1]
                good_new = new_pts[status.flatten() == 1]

                if len(good_old) > 0:
                    displacements = np.linalg.norm(good_new - good_old, axis=1)
                    motion_score = min(np.mean(displacements) / 5.0, 1.0)
                    motion_field = list(zip(good_old, good_new, displacements))

        self.prev_gray = roi.copy()
        self.prev_points = points

        return motion_score, motion_field


# ============================================================
# 使用示例
# ============================================================

def demo_camera():
    """摄像头实时表情识别演示"""
    recognizer = EmotionRecognizer(history_len=15)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("表情识别演示 - 按 q 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = recognizer.recognize(frame)
        vis = recognizer.draw_results(frame, results)

        for res in results:
            print(f"  人脸 {res['bbox'][:2]}: {res['emotion']} "
                  f"({res['confidence']:.1%})")

        cv2.imshow('Emotion Recognition', vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def demo_image(image_path):
    """单张图片表情识别演示"""
    recognizer = EmotionRecognizer()
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"无法读取图片: {image_path}")
        return

    results = recognizer.recognize(frame)
    vis = recognizer.draw_results(frame, results)

    for res in results:
        print(f"情绪: {res['emotion']}, 置信度: {res['confidence']:.1%}")
        for emo, score in res['scores'].items():
            print(f"  {emo}: {score:.2f}")

    cv2.imshow('Emotion Recognition', vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        demo_image(sys.argv[1])
    else:
        demo_camera()
