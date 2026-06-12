#!/usr/bin/env python3
"""
视觉里程计模块 - 特征匹配 + 本质矩阵 + 运动估计
适用于电赛自主导航、里程计、SLAM前端等任务
依赖: numpy, opencv-python
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List, Dict


class VisualOdometry:
    """视觉里程计：从连续图像帧估计相机运动"""

    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: Optional[np.ndarray] = None,
                 feature_method: str = 'orb', match_method: str = 'bf'):
        """
        初始化视觉里程计
        Args:
            camera_matrix: 3x3相机内参矩阵
            dist_coeffs: 畸变系数
            feature_method: 特征检测器 ('orb', 'sift', 'akaze')
            match_method: 匹配方法 ('bf'暴力匹配, 'flann')
        """
        self.K = camera_matrix
        self.dist = dist_coeffs if dist_coeffs is not None else np.zeros(5)
        self.fx = camera_matrix[0, 0]
        self.fy = camera_matrix[1, 1]
        self.cx = camera_matrix[0, 2]
        self.cy = camera_matrix[1, 2]

        # 特征检测器
        if feature_method == 'orb':
            self.detector = cv2.ORB_create(nfeatures=2000)
        elif feature_method == 'sift':
            self.detector = cv2.SIFT_create(nfeatures=2000)
        elif feature_method == 'akaze':
            self.detector = cv2.AKAZE_create()
        else:
            raise ValueError(f"不支持的特征方法: {feature_method}")

        # 特征匹配器
        if match_method == 'bf':
            if feature_method == 'sift':
                self.matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
            else:
                self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        elif match_method == 'flann':
            index_params = dict(algorithm=1, trees=5)
            search_params = dict(checks=50)
            self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

        # 状态
        self.prev_frame = None
        self.prev_kp = None
        self.prev_des = None
        self.trajectory = [np.zeros(3)]  # 位置轨迹
        self.current_R = np.eye(3)  # 当前旋转
        self.current_t = np.zeros(3)  # 当前平移

    def detect_and_compute(self, image: np.ndarray) -> Tuple[list, np.ndarray]:
        """
        检测特征点并计算描述子
        Args:
            image: 灰度或彩色图像
        Returns:
            (关键点列表, 描述子数组)
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        kp, des = self.detector.detectAndCompute(gray, None)
        return kp, des

    def match_features(self, des1: np.ndarray, des2: np.ndarray,
                       ratio_threshold: float = 0.75) -> list:
        """
        特征匹配（Lowe比率测试）
        Args:
            des1: 描述子1
            des2: 描述子2
            ratio_threshold: 比率测试阈值
        Returns:
            优质匹配列表
        """
        if des1 is None or des2 is None:
            return []

        matches = self.matcher.knnMatch(des1, des2, k=2)

        # Lowe比率测试
        good_matches = []
        for m_pair in matches:
            if len(m_pair) == 2:
                m, n = m_pair
                if m.distance < ratio_threshold * n.distance:
                    good_matches.append(m)

        return good_matches

    def estimate_motion(self, kp1: list, kp2: list, matches: list) -> Dict:
        """
        估计两帧之间的运动
        Args:
            kp1: 帧1关键点
            kp2: 帧2关键点
            matches: 匹配结果
        Returns:
            {'R': 旋转矩阵, 't': 平移向量, 'E': 本质矩阵,
             'mask': 内点掩码, 'n_inliers': 内点数}
        """
        if len(matches) < 8:
            return None

        # 提取匹配点坐标
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])

        # 计算本质矩阵（使用RANSAC）
        E, mask = cv2.findEssentialMat(
            pts1, pts2, self.K,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )

        if E is None:
            return None

        # 从本质矩阵恢复R和t
        n_inliers, R, t, pose_mask = cv2.recoverPose(E, pts1, pts2, self.K, mask)

        return {
            'R': R,
            't': t.flatten(),
            'E': E,
            'mask': mask,
            'pose_mask': pose_mask,
            'n_inliers': n_inliers,
            'pts1': pts1,
            'pts2': pts2
        }

    def process_frame(self, frame: np.ndarray) -> Optional[Dict]:
        """
        处理一帧图像，更新里程计状态
        Args:
            frame: 当前帧 (H,W,3) 或 (H,W)
        Returns:
            运动估计结果，首帧返回None
        """
        kp, des = self.detect_and_compute(frame)

        result = None
        if self.prev_des is not None and des is not None:
            # 特征匹配
            matches = self.match_features(self.prev_des, des)

            if len(matches) >= 8:
                # 运动估计
                motion = self.estimate_motion(self.prev_kp, kp, matches)

                if motion is not None:
                    R, t = motion['R'], motion['t']

                    # 累积变换（在相机坐标系中）
                    # 注意：这里假设单位平移，实际尺度需要IMU/轮式里程计提供
                    scale = 1.0  # 单位尺度
                    self.current_t += self.current_R @ (t * scale)
                    self.current_R = R @ self.current_R

                    self.trajectory.append(self.current_t.copy())
                    result = motion
                    result['position'] = self.current_t.copy()
                    result['trajectory_length'] = len(self.trajectory)

        # 更新状态
        self.prev_frame = frame
        self.prev_kp = kp
        self.prev_des = des

        return result

    def triangulate_points(self, R1, t1, R2, t2,
                           pts1: np.ndarray, pts2: np.ndarray) -> np.ndarray:
        """
        三角化匹配点，得到3D坐标
        Args:
            R1, t1: 第一帧的位姿
            R2, t2: 第二帧的位姿
            pts1, pts2: 匹配点坐标 (N,2)
        Returns:
            3D点 (N,3)
        """
        # 投影矩阵
        P1 = self.K @ np.hstack([R1, t1.reshape(-1, 1)])
        P2 = self.K @ np.hstack([R2, t2.reshape(-1, 1)])

        # 三角化
        points_4d = cv2.triangulatePoints(P1, P2, pts1.T, pts2.T)
        points_3d = (points_4d[:3] / points_4d[3]).T

        return points_3d

    def get_trajectory(self) -> np.ndarray:
        """获取完整轨迹"""
        return np.array(self.trajectory)

    def compute_fundamental_matrix(self, pts1: np.ndarray,
                                   pts2: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        计算基础矩阵
        Args:
            pts1, pts2: 匹配点 (N,2)
        Returns:
            (基础矩阵F, 内点掩码)
        """
        F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC, 1.0, 0.99)
        return F, mask

    def compute_epipolar_error(self, pts1: np.ndarray, pts2: np.ndarray,
                               F: np.ndarray) -> np.ndarray:
        """
        计算极线误差（用于评估匹配质量）
        Args:
            pts1, pts2: 匹配点 (N,2)
            F: 基础矩阵
        Returns:
            每个匹配点的极线距离
        """
        # x2^T * F * x1 = 0
        pts1_h = np.hstack([pts1, np.ones((len(pts1), 1))])
        pts2_h = np.hstack([pts2, np.ones((len(pts2), 1))])

        lines1 = (F.T @ pts2_h.T).T
        lines2 = (F @ pts1_h.T).T

        # 点到极线距离
        dist1 = np.abs(np.sum(pts1_h * lines2, axis=1)) / \
                np.sqrt(lines2[:, 0] ** 2 + lines2[:, 1] ** 2)
        dist2 = np.abs(np.sum(pts2_h * lines1, axis=1)) / \
                np.sqrt(lines1[:, 0] ** 2 + lines1[:, 1] ** 2)

        return (dist1 + dist2) / 2

    def draw_matches(self, img1: np.ndarray, kp1: list,
                     img2: np.ndarray, kp2: list,
                     matches: list, max_draw: int = 50) -> np.ndarray:
        """
        绘制特征匹配结果
        """
        draw_matches = matches[:max_draw]
        return cv2.drawMatches(img1, kp1, img2, kp2, draw_matches, None,
                               matchColor=(0, 255, 0), flags=2)


if __name__ == '__main__':
    # ==================== 使用示例 ====================
    print("=== 视觉里程计模块使用示例 ===\n")

    # 1. 相机内参
    K = np.array([[525.0, 0, 319.5],
                  [0, 525.0, 239.5],
                  [0, 0, 1.0]])

    vo = VisualOdometry(K, feature_method='orb')
    print(f"视觉里程计初始化完成，特征检测器: ORB")

    # 2. 模拟图像序列
    print("\n模拟图像序列处理:")
    # 创建有纹理的测试图像
    base = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    cv2.rectangle(base, (100, 100), (300, 300), (255, 0, 0), -1)
    cv2.circle(base, (400, 200), 50, (0, 255, 0), -1)
    cv2.putText(base, "Texture", (150, 250),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # 模拟相机运动（平移+旋转）
    M_translate = np.float32([[1, 0, 10], [0, 1, 5]])
    M_rotate = cv2.getRotationMatrix2D((320, 240), 2, 1.0)

    for i in range(5):
        if i == 0:
            frame = base.copy()
        else:
            frame = cv2.warpAffine(base, M_translate * (i + 1), (640, 480))

        result = vo.process_frame(frame)

        if result is not None:
            pos = result['position']
            print(f"  帧{i}: 位置=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], "
                  f"内点={result['n_inliers']}, 轨迹长度={result['trajectory_length']}")
        else:
            print(f"  帧{i}: 初始化帧（或匹配不足）")

    # 3. 获取轨迹
    traj = vo.get_trajectory()
    print(f"\n完整轨迹: {traj.shape[0]}个位置点")
    print(f"最终位置: {traj[-1]}")

    # 4. 基础矩阵与极线误差
    print("\n基础矩阵计算:")
    img1 = np.random.randint(0, 255, (240, 320), dtype=np.uint8)
    img2 = np.random.randint(0, 255, (240, 320), dtype=np.uint8)
    kp1, des1 = vo.detect_and_compute(img1)
    kp2, des2 = vo.detect_and_compute(img2)
    matches = vo.match_features(des1, des2)

    if len(matches) >= 8:
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])
        F, mask = vo.compute_fundamental_matrix(pts1, pts2)
        errors = vo.compute_epipolar_error(pts1, pts2, F)
        print(f"  基础矩阵形状: {F.shape}")
        print(f"  平均极线误差: {errors.mean():.4f} 像素")
    else:
        print(f"  匹配不足（{len(matches)}个），无法计算基础矩阵")

    print("\n示例完成！")
