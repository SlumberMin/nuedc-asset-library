"""
轻量级姿态估计模块
基于 OpenPose MobileNet 的人体关键点检测 + 关键点连接绘制
适用于电赛中的人体姿态分析场景
"""

import cv2
import numpy as np
import time


# ============================================================
# 人体关键点定义（COCO 18点格式）
# ============================================================
# 关键点名称索引
KEYPOINT_NAMES = [
    "鼻子", "脖子",           # 0, 1
    "右肩", "右肘", "右腕",   # 2, 3, 4
    "左肩", "左肘", "左腕",   # 5, 6, 7
    "右髋", "右膝", "右踝",   # 8, 9, 10
    "左髋", "左膝", "左踝",   # 11, 12, 13
    "右眼", "左眼",           # 14, 15
    "右耳", "左耳"            # 16, 17
]

# 骨架连接定义 (关键点对)
SKELETON_CONNECTIONS = [
    (1, 0),   # 脖子-鼻子
    (1, 2),   # 脖子-右肩
    (1, 5),   # 脖子-左肩
    (2, 3),   # 右肩-右肘
    (3, 4),   # 右肘-右腕
    (5, 6),   # 左肩-左肘
    (6, 7),   # 左肘-左腕
    (1, 8),   # 脖子-右髋
    (1, 11),  # 脖子-左髋
    (8, 9),   # 右髋-右膝
    (9, 10),  # 右膝-右踝
    (11, 12), # 左髋-左膝
    (12, 13), # 左膝-左踝
    (0, 14),  # 鼻子-右眼
    (0, 15),  # 鼻子-左眼
    (14, 16), # 右眼-右耳
    (15, 17), # 左眼-左耳
]

# 每条骨骼的颜色 (BGR)
SKELETON_COLORS = [
    (255, 0, 0), (255, 85, 0), (255, 170, 0),
    (255, 255, 0), (170, 255, 0), (85, 255, 0),
    (0, 255, 0), (0, 255, 85), (0, 255, 170),
    (0, 255, 255), (0, 170, 255), (0, 85, 255),
    (0, 0, 255), (85, 0, 255), (170, 0, 255),
    (255, 0, 255), (255, 0, 170),
]


# ============================================================
# 轻量级姿态估计器
# ============================================================
class PoseEstimatorLite:
    """
    轻量级人体姿态估计器
    
    方案A: 使用 OpenCV DNN 加载 OpenPose 模型（推荐，精度高）
    方案B: 基于人体检测 + 肤色分割的简化关键点估计（无需模型，速度快）
    """

    def __init__(self, mode='opencv_dnn',
                 proto_path=None, model_path=None,
                 input_size=(368, 368), threshold=0.1):
        """
        参数:
            mode: 'opencv_dnn' 使用OpenPose模型, 'simple' 使用简化方案
            proto_path: OpenPose prototxt 文件路径
            model_path: OpenPose caffemodel 文件路径
            input_size: 网络输入尺寸
            threshold: 关键点置信度阈值
        """
        self.mode = mode
        self.input_size = input_size
        self.threshold = threshold
        self.net = None

        if mode == 'opencv_dnn' and proto_path and model_path:
            try:
                self.net = cv2.dnn.readNetFromCaffe(proto_path, model_path)
                print(f"[姿态估计] 已加载OpenPose模型: {model_path}")
            except Exception as e:
                print(f"[姿态估计] 模型加载失败: {e}，切换为简化模式")
                self.mode = 'simple'
        else:
            self.mode = 'simple'
            print("[姿态估计] 使用简化模式（肤色分割）")

    def estimate_opencv_dnn(self, frame):
        """
        使用OpenCV DNN运行OpenPose推理
        返回: list of keypoints, 每个keypoint为 (x, y, confidence)
        """
        h, w = frame.shape[:2]
        inp_w, inp_h = self.input_size

        # 构建输入blob
        blob = cv2.dnn.blobFromImage(
            frame, 1.0 / 255, (inp_w, inp_h), (0, 0, 0), swapRB=False, crop=False
        )
        self.net.setInput(blob)

        # 前向推理
        output = self.net.forward()  # shape: (1, 44, 46, 46) for 18-keypoint model
        # 前18个通道是关键点热力图，后18个通道是PAF（部分亲和力场）
        num_points = 18
        heatmaps = output[0, :num_points, :, :]

        keypoints = []
        for i in range(num_points):
            heatmap = heatmaps[i]
            # 找到热力图最大值位置
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(heatmap)

            if max_val > self.threshold:
                # 坐标映射回原图尺寸
                x = int(max_loc[0] * w / heatmap.shape[1])
                y = int(max_loc[1] * h / heatmap.shape[0])
                keypoints.append((x, y, float(max_val)))
            else:
                keypoints.append(None)

        return keypoints

    def estimate_simple(self, frame):
        """
        简化方案：基于肤色分割 + 轮廓分析估计关键点位置
        精度较低但无需模型，适合资源受限场景
        """
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 肤色检测 (HSV范围)
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)
        skin_mask = cv2.inRange(hsv, lower_skin, upper_skin)

        # 形态学处理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        skin_mask = cv2.morphologyEx(skin_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        skin_mask = cv2.morphologyEx(skin_mask, cv2.MORPH_OPEN, kernel, iterations=1)

        # 查找肤色轮廓
        contours, _ = cv2.findContours(skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        keypoints = [None] * 18

        if contours:
            # 找最大的肤色区域（假设为人体主体）
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 2000:
                x, y, bw, bh = cv2.boundingRect(largest)
                cx, cy = x + bw // 2, y + bh // 2

                # 粗略估计关键点位置（基于人体比例）
                # 头部区域
                head_y = y + int(bh * 0.08)
                neck_y = y + int(bh * 0.15)
                shoulder_y = y + int(bh * 0.2)
                elbow_y = y + int(bh * 0.35)
                wrist_y = y + int(bh * 0.5)
                hip_y = y + int(bh * 0.55)
                knee_y = y + int(bh * 0.72)
                ankle_y = y + int(bh * 0.92)

                shoulder_w = int(bw * 0.4)

                # 鼻子
                keypoints[0] = (cx, head_y, 0.5)
                # 脖子
                keypoints[1] = (cx, neck_y, 0.5)
                # 右肩
                keypoints[2] = (cx + shoulder_w, shoulder_y, 0.4)
                # 右肘
                keypoints[3] = (cx + shoulder_w + 10, elbow_y, 0.3)
                # 右腕
                keypoints[4] = (cx + shoulder_w + 5, wrist_y, 0.3)
                # 左肩
                keypoints[5] = (cx - shoulder_w, shoulder_y, 0.4)
                # 左肘
                keypoints[6] = (cx - shoulder_w - 10, elbow_y, 0.3)
                # 左腕
                keypoints[7] = (cx - shoulder_w - 5, wrist_y, 0.3)
                # 右髋
                keypoints[8] = (cx + int(bw * 0.15), hip_y, 0.4)
                # 右膝
                keypoints[9] = (cx + int(bw * 0.12), knee_y, 0.3)
                # 右踝
                keypoints[10] = (cx + int(bw * 0.1), ankle_y, 0.3)
                # 左髋
                keypoints[11] = (cx - int(bw * 0.15), hip_y, 0.4)
                # 左膝
                keypoints[12] = (cx - int(bw * 0.12), knee_y, 0.3)
                # 左踝
                keypoints[13] = (cx - int(bw * 0.1), ankle_y, 0.3)

        return keypoints

    def estimate(self, frame):
        """
        姿态估计主入口
        参数:
            frame: BGR图像
        返回:
            keypoints: list of (x, y, conf) 或 None
        """
        if self.mode == 'opencv_dnn' and self.net is not None:
            return self.estimate_opencv_dnn(frame)
        else:
            return self.estimate_simple(frame)

    def detect_gesture(self, keypoints):
        """
        基于关键点的简单手势识别
        返回: 手势描述字符串
        """
        if keypoints is None:
            return "未检测到人体"

        gestures = []

        # 检查是否举手
        l_shoulder = keypoints[5]
        l_wrist = keypoints[7]
        r_shoulder = keypoints[2]
        r_wrist = keypoints[4]

        if l_shoulder and l_wrist:
            if l_wrist[1] < l_shoulder[1]:
                gestures.append("左手举起")
        if r_shoulder and r_wrist:
            if r_wrist[1] < r_shoulder[1]:
                gestures.append("右手举起")

        # 检查是否站立（双脚踝在地面上）
        l_ankle = keypoints[13]
        r_ankle = keypoints[10]
        if l_ankle and r_ankle:
            if abs(l_ankle[1] - r_ankle[1]) < 20:
                gestures.append("站立")

        return " | ".join(gestures) if gestures else "正常姿态"


# ============================================================
# 可视化绘制
# ============================================================
def draw_pose(frame, keypoints, draw_points=True, draw_skeleton=True,
              point_radius=4, line_thickness=2):
    """
    在图像上绘制人体姿态
    参数:
        frame: 图像
        keypoints: 关键点列表
        draw_points: 是否绘制关键点
        draw_skeleton: 是否绘制骨架连接
    """
    result = frame.copy()

    if keypoints is None:
        return result

    # 绘制骨架连线
    if draw_skeleton:
        for idx, (i, j) in enumerate(SKELETON_CONNECTIONS):
            if i < len(keypoints) and j < len(keypoints):
                kp_i = keypoints[i]
                kp_j = keypoints[j]
                if kp_i and kp_j and kp_i[2] > 0.1 and kp_j[2] > 0.1:
                    color = SKELETON_COLORS[idx % len(SKELETON_COLORS)]
                    pt1 = (int(kp_i[0]), int(kp_i[1]))
                    pt2 = (int(kp_j[0]), int(kp_j[1]))
                    cv2.line(result, pt1, pt2, color, line_thickness, cv2.LINE_AA)

    # 绘制关键点
    if draw_points:
        for i, kp in enumerate(keypoints):
            if kp and kp[2] > 0.1:
                x, y = int(kp[0]), int(kp[1])
                color = SKELETON_COLORS[i % len(SKELETON_COLORS)]
                cv2.circle(result, (x, y), point_radius, color, -1, cv2.LINE_AA)
                cv2.circle(result, (x, y), point_radius + 1, (255, 255, 255), 1, cv2.LINE_AA)

    return result


def draw_pose_debug(frame, keypoints):
    """带调试信息的姿态绘制"""
    result = draw_pose(frame, keypoints)
    if keypoints:
        for i, kp in enumerate(keypoints):
            if kp and kp[2] > 0.1:
                cv2.putText(result, str(i), (int(kp[0]) + 5, int(kp[1]) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    return result


# ============================================================
# 使用示例
# ============================================================
def demo_camera():
    """摄像头实时姿态估计演示"""
    estimator = PoseEstimatorLite(mode='simple')

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("轻量级姿态估计演示")
    print("按 'q' 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        t_start = time.time()
        keypoints = estimator.estimate(frame)
        t_cost = time.time() - t_start

        # 绘制结果
        result = draw_pose(frame, keypoints)

        # 显示信息
        fps = 1.0 / max(t_cost, 1e-6)
        gesture = estimator.detect_gesture(keypoints)
        cv2.putText(result, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(result, f"Gesture: {gesture}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Pose Estimation", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def demo_image(image_path):
    """静态图像姿态估计演示"""
    estimator = PoseEstimatorLite(mode='simple')

    frame = cv2.imread(image_path)
    if frame is None:
        print(f"无法读取图像: {image_path}")
        return

    keypoints = estimator.estimate(frame)
    result = draw_pose(frame, keypoints, draw_points=True, draw_skeleton=True)

    gesture = estimator.detect_gesture(keypoints)
    cv2.putText(result, gesture, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    cv2.imshow("Pose Result", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def demo_with_openpose_model(proto_path, model_path, image_path):
    """
    使用OpenPose模型的姿态估计演示
    模型下载:
      prototxt: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/models/pose/coco/pose_deploy_linevec.prototxt
      caffemodel: http://posefs1.perception.cs.cmu.edu/OpenPose/models/pose/coco/pose_iter_440000.caffemodel
    """
    estimator = PoseEstimatorLite(
        mode='opencv_dnn',
        proto_path=proto_path,
        model_path=model_path,
        input_size=(368, 368),
        threshold=0.1
    )

    frame = cv2.imread(image_path)
    if frame is None:
        print(f"无法读取图像: {image_path}")
        return

    keypoints = estimator.estimate(frame)
    result = draw_pose(frame, keypoints)

    gesture = estimator.detect_gesture(keypoints)
    cv2.putText(result, gesture, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    cv2.imshow("OpenPose Result", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1:
        demo_image(sys.argv[1])
    else:
        demo_camera()
