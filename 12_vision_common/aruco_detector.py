#!/usr/bin/env python3
"""
ArUco / AprilTag 标记检测
功能：标记检测 + 6DOF位姿估计 + ID识别
适用：OpenCV 4.x + Orange Pi 5
依赖：pip install opencv-contrib-python
"""

import cv2
import numpy as np
import json
import os


class ArucoDetector:
    """ArUco/AprilTag标记检测与位姿估计"""

    # 支持的字典类型
    DICT_TYPES = {
        '4x4_50':     cv2.aruco.DICT_4X4_50,
        '4x4_100':    cv2.aruco.DICT_4X4_100,
        '4x4_250':    cv2.aruco.DICT_4X4_250,
        '4x4_1000':   cv2.aruco.DICT_4X4_1000,
        '5x5_50':     cv2.aruco.DICT_5X5_50,
        '5x5_100':    cv2.aruco.DICT_5X5_100,
        '5x5_250':    cv2.aruco.DICT_5X5_250,
        '5x5_1000':   cv2.aruco.DICT_5X5_1000,
        '6x6_50':     cv2.aruco.DICT_6X6_50,
        '6x6_100':    cv2.aruco.DICT_6X6_100,
        '6x6_250':    cv2.aruco.DICT_6X6_250,
        '6x6_1000':   cv2.aruco.DICT_6X6_1000,
        '7x7_50':     cv2.aruco.DICT_7X7_50,
        '7x7_100':    cv2.aruco.DICT_7X7_100,
        '7x7_250':    cv2.aruco.DICT_7X7_250,
        '7x7_1000':   cv2.aruco.DICT_7X7_1000,
        'apriltag_16h5':  cv2.aruco.DICT_APRILTAG_16h5,
        'apriltag_25h9':  cv2.aruco.DICT_APRILTAG_25h9,
        'apriltag_36h10': cv2.aruco.DICT_APRILTAG_36h10,
        'apriltag_36h11': cv2.aruco.DICT_APRILTAG_36h11,
    }

    def __init__(self, dict_type='5x5_100', marker_length=0.05,
                 camera_matrix=None, dist_coeffs=None):
        """
        参数:
            dict_type: 字典类型
            marker_length: 标记物理边长(米)
            camera_matrix: 3x3相机内参矩阵 (None则不估计位姿)
            dist_coeffs: 畸变系数
        """
        self.marker_length = marker_length

        # 初始化检测器
        dict_id = self.DICT_TYPES.get(dict_type, cv2.aruco.DICT_5X5_100)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        # OpenCV 4.7+ 使用新API
        try:
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dict,
                cv2.aruco.DetectorParameters()
            )
            self.use_new_api = True
        except AttributeError:
            self.use_new_api = False

        # 相机参数
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        if dist_coeffs is None:
            self.dist_coeffs = np.zeros((5, 1))

    def load_camera_params(self, filepath):
        """从文件加载相机参数"""
        if not os.path.exists(filepath):
            print(f"[警告] 相机参数文件不存在: {filepath}")
            return False

        data = np.load(filepath)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']
        print(f"[加载] 相机参数: {filepath}")
        return True

    def _detect_markers(self, gray):
        """检测标记"""
        if self.use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict,
                parameters=cv2.aruco.DetectorParameters()
            )
        return corners, ids, rejected

    def estimate_pose(self, corners):
        """
        6DOF位姿估计
        返回: list of (rvec, tvec, euler_angles)
        """
        if self.camera_matrix is None:
            return []

        poses = []
        half = self.marker_length / 2
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

        for corner in corners:
            success, rvec, tvec = cv2.solvePnP(
                obj_points, corner[0],
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if success:
                # 转换为欧拉角
                rot_mat, _ = cv2.Rodrigues(rvec)
                euler = self._rotation_matrix_to_euler(rot_mat)
                poses.append({
                    'rvec': rvec,
                    'tvec': tvec,
                    'rotation_matrix': rot_mat,
                    'euler': euler,  # (roll, pitch, yaw) in degrees
                    'distance': np.linalg.norm(tvec),
                })
        return poses

    @staticmethod
    def _rotation_matrix_to_euler(R):
        """旋转矩阵转欧拉角 (ZYX顺序)"""
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return np.degrees([x, y, z])

    def detect(self, frame):
        """
        完整检测流程

        返回:
            results: list of dict, 每个包含:
                - id: 标记ID
                - corners: 角点坐标
                - center: 中心坐标
                - pose: 位姿(如果有相机参数)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self._detect_markers(gray)

        results = []
        poses = []
        if ids is not None and len(ids) > 0 and self.camera_matrix is not None:
            poses = self.estimate_pose(corners)

        if ids is not None:
            for i, (corner, marker_id) in enumerate(zip(corners, ids.flatten())):
                pts = corner[0]
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))

                result = {
                    'id': int(marker_id),
                    'corners': pts.tolist(),
                    'center': (cx, cy),
                    'pose': poses[i] if i < len(poses) else None,
                }
                results.append(result)

        return results

    def draw(self, frame, results):
        """绘制检测结果"""
        vis = frame.copy()

        for r in results:
            pts = np.array(r['corners'], dtype=np.int32)

            # 绘制标记边框
            cv2.polylines(vis, [pts], True, (0, 255, 0), 2)

            # 绘制角点
            for pt in pts:
                cv2.circle(vis, (int(pt[0]), int(pt[1])), 4, (0, 0, 255), -1)

            # 绘制中心
            cx, cy = r['center']
            cv2.circle(vis, (cx, cy), 5, (255, 0, 0), -1)

            # 显示ID
            cv2.putText(vis, f"ID:{r['id']}", (cx - 10, cy - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # 显示位姿
            pose = r.get('pose')
            if pose is not None:
                tvec = pose['tvec']
                euler = pose['euler']
                dist = pose['distance']

                # 绘制坐标轴
                if self.camera_matrix is not None:
                    axis_len = self.marker_length * 0.5
                    axis_pts = np.float32([
                        [0, 0, 0],
                        [axis_len, 0, 0],
                        [0, axis_len, 0],
                        [0, 0, axis_len],
                    ])
                    img_pts, _ = cv2.projectPoints(
                        axis_pts, pose['rvec'], pose['tvec'],
                        self.camera_matrix, self.dist_coeffs
                    )
                    img_pts = img_pts.reshape(-1, 2).astype(int)
                    origin = tuple(img_pts[0])
                    cv2.line(vis, origin, tuple(img_pts[1]), (0, 0, 255), 2)  # X红
                    cv2.line(vis, origin, tuple(img_pts[2]), (0, 255, 0), 2)  # Y绿
                    cv2.line(vis, origin, tuple(img_pts[3]), (255, 0, 0), 2)  # Z蓝

                # 显示距离和角度
                info = f"D:{dist:.3f}m R:{euler[0]:.1f} P:{euler[1]:.1f} Y:{euler[2]:.1f}"
                cv2.putText(vis, info, (cx - 30, cy + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv2.putText(vis, f"Markers: {len(results)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        return vis

    @staticmethod
    def generate_marker(marker_id, dict_type='5x5_100', size=200,
                        output_path=None):
        """生成ArUco标记图片"""
        dict_id = ArucoDetector.DICT_TYPES.get(dict_type, cv2.aruco.DICT_5X5_100)
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)

        # 添加白色边框
        border = size // 5
        bordered = np.ones((size + 2*border, size + 2*border), dtype=np.uint8) * 255
        bordered[border:border+size, border:border+size] = marker_img

        if output_path:
            cv2.imwrite(output_path, bordered)
            print(f"[生成] 标记 {marker_id} -> {output_path}")

        return bordered


def run_demo(camera_id=0, dict_type='5x5_100', marker_length=0.05,
             camera_params=None):
    """实时演示"""
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    detector = ArucoDetector(dict_type=dict_type, marker_length=marker_length)

    if camera_params and os.path.exists(camera_params):
        detector.load_camera_params(camera_params)

    print("=" * 50)
    print(f"ArUco标记检测 - 字典: {dict_type}")
    print("q/ESC: 退出 | g: 生成标记图")
    print("=" * 50)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = detector.detect(frame)
        vis = detector.draw(frame, results)

        cv2.imshow('ArUco Detector', vis)

        key = cv2.waitKey(1) & 0xFF
        if key in [ord('q'), 27]:
            break
        elif key == ord('g'):
            # 生成标记到当前目录
            for i in range(5):
                ArucoDetector.generate_marker(
                    i, dict_type, 400,
                    f"aruco_marker_{i}.png"
                )
            print("已生成5个标记图片")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='ArUco/AprilTag标记检测')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--dict', type=str, default='5x5_100')
    parser.add_argument('--marker-length', type=float, default=0.05,
                       help='标记边长(米)')
    parser.add_argument('--camera-params', type=str, default=None,
                       help='相机参数文件(.npz)')
    parser.add_argument('--generate', type=int, default=-1,
                       help='生成指定ID的标记图')
    args = parser.parse_args()

    if args.generate >= 0:
        ArucoDetector.generate_marker(args.generate, args.dict, 400,
                                       f"aruco_marker_{args.generate}.png")
    else:
        run_demo(args.camera, args.dict, args.marker_length, args.camera_params)
