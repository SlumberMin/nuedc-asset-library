# -*- coding: utf-8 -*-
"""
模块5: 视觉SLAM简化版
========================
嵌入式平台轻量级Visual Odometry / SLAM

完整SLAM太重(~1GB内存), 此处实现:
  1. 视觉里程计(VO): 仅前后帧匹配估计运动
  2. 简易地图: 关键帧+特征点存储
  3. 回环检测: 基于BoW(可选, 电赛一般不需要)

技术路线:
  方案A: 特征点法(ORB+PnP) → 最稳定★
  方案B: 光流法(Lucas-Kanade) → 更快
  方案C: 混合: 光流追踪 + 关键帧特征匹配

优化:
  1. ORB特征: 每帧最多500个特征点
  2. 关键帧间隔: 每10帧或位移>阈值时插入
  3. 地图点淘汰: 老化机制避免内存溢出
  4. 单应矩阵估计: 比本质矩阵更鲁棒(平面场景)

电赛场景:
  - 自主导航: 估计小车位姿
  - 路径记录: 记录行驶轨迹
  - 位置闭环: 回到起点
"""

import cv2
import numpy as np
from collections import deque


class FeatureExtractor:
    """
    ORB特征提取器
    
    ORB特点:
      - 旋转不变性
      - 速度是SIFT的100倍
      - 免费(无专利问题)
      - 适合嵌入式平台
    """
    
    def __init__(self, max_features=500):
        self.orb = cv2.ORB_create(nfeatures=max_features)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    
    def detect_and_compute(self, gray):
        """
        检测ORB特征并计算描述子
        
        Args:
            gray: 灰度图
        Returns:
            keypoints: 关键点列表
            descriptors: 描述子 (Nx32)
        """
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        return keypoints, descriptors
    
    def match(self, desc1, desc2, ratio=0.75):
        """
        特征匹配(Lowe比率测试)
        
        Args:
            desc1, desc2: 描述子
            ratio: 比率阈值
        Returns:
            good_matches: 优质匹配列表
        """
        if desc1 is None or desc2 is None:
            return []
        if len(desc1) < 2 or len(desc2) < 2:
            return []
        
        matches = self.bf.knnMatch(desc1, desc2, k=2)
        
        good = []
        for m, n in matches:
            if m.distance < ratio * n.distance:
                good.append(m)
        
        return good


class FrameInfo:
    """帧信息存储"""
    def __init__(self, frame_id, gray, keypoints, descriptors, pose=None):
        self.frame_id = frame_id
        self.gray = gray
        self.keypoints = keypoints
        self.descriptors = descriptors
        self.pose = pose if pose is not None else np.eye(4)  # 4x4变换矩阵
        self.points_3d = None  # 三角化后的3D点


class SimpleVisualOdometry:
    """
    简化版视觉里程计
    
    流程:
      1. 提取当前帧ORB特征
      2. 与上一帧特征匹配
      3. 估计相机运动(本质矩阵/单应矩阵)
      4. 累积位姿
    
    注意: 这是单目VO, 尺度不确定
    电赛建议: 用编码器/IMU提供尺度信息
    
    用法:
        vo = SimpleVisualOdometry(camera_matrix)
        while True:
            frame = capture()
            vo.process_frame(frame)
            print(vo.get_position())
    """
    
    def __init__(self, camera_matrix, dist_coeffs=None, 
                 max_features=500, min_matches=15):
        """
        Args:
            camera_matrix: 3x3摄像头内参
            dist_coeffs: 畸变系数
            max_features: 最大特征数
            min_matches: 最小匹配数(低于此数跳过)
        """
        self.K = camera_matrix
        self.dist = dist_coeffs
        self.extractor = FeatureExtractor(max_features)
        self.min_matches = min_matches
        
        self.prev_frame = None
        self.curr_frame = None
        self.frame_id = 0
        self.trajectory = []  # 位姿历史
        self.current_pose = np.eye(4)
        
        # 关键帧管理
        self.keyframes = deque(maxlen=50)  # 最多保留50个关键帧
        self.keyframe_interval = 10  # 每N帧检查一次
        
        # 用于尺度估计(如果有IMU/编码器)
        self.scale = 1.0
    
    def process_frame(self, frame):
        """
        处理一帧图像
        
        Args:
            frame: BGR图像
        Returns:
            success: 是否成功估计运动
            motion: [dx, dy, dz, yaw] 或 None
        """
        self.frame_id += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        
        # 提取特征
        kps, descs = self.extractor.detect_and_compute(gray)
        
        curr = FrameInfo(self.frame_id, gray, kps, descs, self.current_pose.copy())
        
        success = False
        motion = None
        
        if self.prev_frame is not None and descs is not None:
            # 特征匹配
            matches = self.extractor.match(self.prev_frame.descriptors, descs)
            
            if len(matches) >= self.min_matches:
                # 获取匹配点坐标
                pts1 = np.float32([self.prev_frame.keypoints[m.queryIdx].pt for m in matches])
                pts2 = np.float32([self.curr_frame.keypoints[m.trainIdx].pt for m in matches]) if self.curr_frame else None
                
                # 用当前帧的keypoints
                pts2 = np.float32([kps[m.trainIdx].pt for m in matches])
                
                # 估计运动
                R, t, inliers = self._estimate_motion(pts1, pts2)
                
                if R is not None and inliers is not None and np.sum(inliers) > 10:
                    # 构建变换矩阵
                    T = np.eye(4)
                    T[:3, :3] = R
                    T[:3, 3] = t.flatten() * self.scale
                    
                    # 累积位姿
                    self.current_pose = self.current_pose @ np.linalg.inv(T)
                    
                    success = True
                    pos = self.current_pose[:3, 3]
                    yaw = np.degrees(np.arctan2(self.current_pose[1, 0], 
                                                self.current_pose[0, 0]))
                    motion = [pos[0], pos[1], pos[2], yaw]
                    
                    self.trajectory.append(pos.copy())
        
        # 关键帧插入
        if self._is_keyframe():
            self.keyframes.append(curr)
        
        self.prev_frame = self.curr_frame if self.curr_frame else curr
        self.curr_frame = curr
        
        return success, motion
    
    def _estimate_motion(self, pts1, pts2):
        """
        估计两帧间运动
        
        方法: 本质矩阵分解
        返回: R(旋转), t(平移), inliers(内点)
        """
        # 用RANSAC估计本质矩阵
        E, mask = cv2.findEssentialMat(
            pts1, pts2, self.K, method=cv2.RANSAC, 
            prob=0.999, threshold=1.0
        )
        
        if E is None or mask is None:
            return None, None, None
        
        # 分解本质矩阵
        _, R, t, mask2 = cv2.recoverPose(E, pts1, pts2, self.K, mask=mask)
        
        return R, t, mask2 > 0
    
    def _is_keyframe(self):
        """判断是否为关键帧"""
        if len(self.keyframes) == 0:
            return True
        if self.frame_id % self.keyframe_interval == 0:
            return True
        # 位移足够大时也插入关键帧
        if len(self.keyframes) > 0:
            last_pos = self.keyframes[-1].pose[:3, 3]
            curr_pos = self.current_pose[:3, 3]
            if np.linalg.norm(curr_pos - last_pos) > 50:  # 50mm
                return True
        return False
    
    def get_position(self):
        """获取当前位置 [x, y, z] (mm)"""
        return self.current_pose[:3, 3].copy()
    
    def get_yaw(self):
        """获取朝向角(度)"""
        return np.degrees(np.arctan2(
            self.current_pose[1, 0], self.current_pose[0, 0]))
    
    def get_trajectory(self):
        """获取轨迹历史"""
        return np.array(self.trajectory) if self.trajectory else np.zeros((0, 3))
    
    def draw_trajectory(self, size=(500, 500), scale=1.0):
        """绘制俯视轨迹图"""
        canvas = np.ones((size[0], size[1], 3), dtype=np.uint8) * 255
        trajectory = self.get_trajectory()
        
        if len(trajectory) < 2:
            return canvas
        
        # 归一化到画布
        cx, cy = size[1] // 2, size[0] // 2
        points = trajectory[:, :2] * scale + np.array([cx, cy])
        points = points.astype(np.int32)
        
        # 绘制轨迹线
        for i in range(1, len(points)):
            color = (0, int(255 * i / len(points)), 0)
            cv2.line(canvas, tuple(points[i-1]), tuple(points[i]), color, 2)
        
        # 绘制当前位置
        cv2.circle(canvas, tuple(points[-1]), 5, (0, 0, 255), -1)
        
        # 绘制朝向
        yaw = np.radians(self.get_yaw())
        dx, dy = int(20 * np.cos(yaw)), int(20 * np.sin(yaw))
        cv2.arrowedLine(canvas, tuple(points[-1]), 
                       (points[-1][0]+dx, points[-1][1]+dy), (0,0,255), 2)
        
        return canvas


class MonoSLAM(SimpleVisualOdometry):
    """
    单目SLAM(在VO基础上增加地图)
    
    增强:
      1. 维护3D地图点
      2. 相机位姿优化(简化BA)
      3. 轨迹图可视化
    
    电赛应用: 简单的自主导航/路径记录
    """
    
    def __init__(self, camera_matrix, dist_coeffs=None, scale=1.0):
        super().__init__(camera_matrix, dist_coeffs)
        self.map_points = []  # 3D地图点
        self.scale = scale
    
    def triangulate(self, pts1, pts2, pose1, pose2):
        """
        三角化3D点
        
        Args:
            pts1, pts2: 匹配的2D点
            pose1, pose2: 两帧的投影矩阵
        Returns:
            points_3d: Nx3 3D坐标
        """
        P1 = self.K @ pose1[:3, :]
        P2 = self.K @ pose2[:3, :]
        
        points_4d = cv2.triangulatePoints(P1, P2, pts1.T, pts2.T)
        points_3d = points_4d[:3] / points_4d[3]
        
        return points_3d.T
    
    def process_frame(self, frame):
        """处理帧, 同时构建地图"""
        success, motion = super().process_frame(frame)
        
        if success and self.prev_frame is not None and self.curr_frame is not None:
            # 三角化新地图点(每隔几帧)
            if self.frame_id % 3 == 0:
                matches = self.extractor.match(
                    self.prev_frame.descriptors, self.curr_frame.descriptors)
                if len(matches) > 10:
                    pts1 = np.float32([self.prev_frame.keypoints[m.queryIdx].pt 
                                       for m in matches])
                    pts2 = np.float32([self.curr_frame.keypoints[m.trainIdx].pt 
                                       for m in matches])
                    
                    pts3d = self.triangulate(pts1, pts2, 
                                            self.prev_frame.pose, self.current_pose)
                    
                    # 过滤远处点和不可靠点
                    valid = np.abs(pts3d[:, 2]) < 5000  # z < 5m
                    new_points = pts3d[valid]
                    if len(new_points) > 0:
                        self.map_points.extend(new_points.tolist())
            
            # 地图点数量限制
            if len(self.map_points) > 10000:
                self.map_points = self.map_points[-5000:]
        
        return success, motion
    
    def get_map_points(self):
        """获取3D地图点"""
        return np.array(self.map_points) if self.map_points else np.zeros((0, 3))


# ===== 电赛应用示例 =====
def demo_navigation():
    """
    自主导航示例
    
    场景: 小车行驶中实时估计位姿
    输出: 当前位置、朝向、轨迹图
    
    配合编码器使用: 提供真实尺度
    """
    from .platform_utils import CameraThread, FrameCounter, optimize_opencv
    optimize_opencv()
    
    # 摄像头内参(需要标定)
    # 默认值(640x480通用近似)
    K = np.array([
        [600, 0, 320],
        [0, 600, 240],
        [0, 0, 1]
    ], dtype=np.float64)
    
    cam = CameraThread(src=0, width=640, height=480).start()
    slam = MonoSLAM(K, scale=1.0)
    counter = FrameCounter()
    
    print("[导航] Visual SLAM启动... 按q退出")
    while True:
        frame = cam.read()
        if frame is None:
            continue
        
        success, motion = slam.process_frame(frame)
        
        # 显示位姿
        pos = slam.get_position()
        yaw = slam.get_yaw()
        
        # 绘制轨迹
        traj_img = slam.draw_trajectory(size=(400, 400), scale=0.5)
        
        # 显示原始帧
        counter.tick()
        cv2.putText(frame, f"FPS:{counter.fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        cv2.putText(frame, f"Pos:({pos[0]:.0f},{pos[1]:.0f},{pos[2]:.0f})", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1)
        cv2.putText(frame, f"Yaw:{yaw:.1f} deg", (10, 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1)
        
        # 并排显示
        display = np.hstack([frame, traj_img])
        cv2.imshow("SLAM Navigation", display)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # 保存轨迹
    trajectory = slam.get_trajectory()
    if len(trajectory) > 0:
        np.savetxt("trajectory.csv", trajectory, delimiter=",", 
                   header="x,y,z", comments="")
        print(f"[导航] 轨迹已保存, 共{len(trajectory)}点")
    
    cam.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo_navigation()
