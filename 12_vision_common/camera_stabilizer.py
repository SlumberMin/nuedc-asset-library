"""
相机稳定器 - 光流法 + 仿射变换实现图像稳定
适用于: 视频去抖、运动补偿、跟踪稳定化
"""

import cv2
import numpy as np
from collections import deque


class CameraStabilizer:
    """基于光流法和仿射变换的相机稳定器"""

    def __init__(self, smooth_window=30, crop_ratio=0.05):
        """
        Args:
            smooth_window: 平滑窗口大小(帧数)
            crop_ratio: 裁剪比例(去除黑边)
        """
        self.smooth_window = smooth_window
        self.crop_ratio = crop_ratio
        self.prev_gray = None
        self.transforms = deque(maxlen=smooth_window)
        self.cumulative_transform = np.zeros(2, dtype=np.float64)  # dx, dy

    def stabilize(self, frame):
        """
        稳定一帧图像
        Args:
            frame: 输入BGR图像
        Returns:
            stabilized: 稳定后的图像
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        if self.prev_gray is None:
            self.prev_gray = gray
            return frame

        # 计算光流
        prev_pts = cv2.goodFeaturesToTrack(
            self.prev_gray, maxCorners=200, qualityLevel=0.01, minDistance=30, blockSize=3
        )
        if prev_pts is None:
            self.prev_gray = gray
            return frame

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, prev_pts, None,
            winSize=(15, 15), maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )

        # 过滤有效点
        idx = np.where(status == 1)[0]
        if len(idx) < 10:
            self.prev_gray = gray
            return frame

        prev_pts = prev_pts[idx]
        curr_pts = curr_pts[idx]

        # 估计仿射变换矩阵
        m, _ = cv2.estimateAffinePartial2D(prev_pts, curr_pts)
        if m is None:
            self.prev_gray = gray
            return frame

        # 提取平移和旋转
        dx = m[0, 2]
        dy = m[1, 2]
        da = np.arctan2(m[1, 0], m[0, 0])

        self.transforms.append([dx, dy, da])

        # 累积变换 + 平滑
        smooth_dx = np.mean([t[0] for t in self.transforms])
        smooth_dy = np.mean([t[1] for t in self.transforms])
        smooth_da = np.mean([t[2] for t in self.transforms])

        # 构建平滑后的变换矩阵
        cos_a = np.cos(smooth_da)
        sin_a = np.sin(smooth_da)
        smooth_m = np.array([
            [cos_a, -sin_a, smooth_dx],
            [sin_a,  cos_a, smooth_dy]
        ], dtype=np.float32)

        # 应用变换
        stabilized = cv2.warpAffine(frame, smooth_m, (w, h))

        # 裁剪黑边
        crop_x = int(w * self.crop_ratio)
        crop_y = int(h * self.crop_ratio)
        if crop_x > 0 and crop_y > 0:
            stabilized = stabilized[crop_y:h - crop_y, crop_x:w - crop_x]
            stabilized = cv2.resize(stabilized, (w, h))

        self.prev_gray = gray
        return stabilized

    def reset(self):
        """重置状态"""
        self.prev_gray = None
        self.transforms.clear()


class HomographyStabilizer:
    """基于单应性矩阵的稳定器(更精确但更慢)"""

    def __init__(self, smooth_window=20):
        self.smooth_window = smooth_window
        self.prev_gray = None
        self.h_matrixs = deque(maxlen=smooth_window)

    def stabilize(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            return frame

        # 特征点检测
        orb = cv2.ORB_create(500)
        kp1, des1 = orb.detectAndCompute(self.prev_gray, None)
        kp2, des2 = orb.detectAndCompute(gray, None)

        if des1 is None or des2 is None or len(kp1) < 10 or len(kp2) < 10:
            self.prev_gray = gray
            return frame

        # 特征匹配
        bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        matches = bf.knnMatch(des1, des2, k=2)

        good = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)

        if len(good) < 10:
            self.prev_gray = gray
            return frame

        pts1 = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        pts2 = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        h, _ = cv2.findHomography(pts2, pts1, cv2.RANSAC, 5.0)
        if h is None:
            self.prev_gray = gray
            return frame

        self.h_matrixs.append(h)

        # 平滑(取对数域平均)
        smooth_h = np.mean(list(self.h_matrixs), axis=0)

        h, w = frame.shape[:2]
        stabilized = cv2.warpPerspective(frame, smooth_h, (w, h))

        self.prev_gray = gray
        return stabilized


def demo():
    """摄像头实时稳定演示"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    stabilizer = CameraStabilizer(smooth_window=30)
    print("按 'q' 退出, 按 'r' 重置")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        stabilized = stabilizer.stabilize(frame)

        # 左右对比
        h, w = frame.shape[:2]
        display = np.zeros((h, w * 2, 3), dtype=np.uint8)
        display[:, :w] = frame
        display[:, w:] = stabilized

        cv2.putText(display, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(display, "Stabilized", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Stabilization Demo", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            stabilizer.reset()

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo()
