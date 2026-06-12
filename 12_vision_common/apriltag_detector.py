#!/usr/bin/env python3
"""
AprilTag检测器 - 比ArUco更鲁棒的Fiducial标记检测
功能：AprilTag标记检测 + 6DOF位姿估计 + ID识别
适用：OpenCV 4.x + Orange Pi 5
依赖：pip install opencv-contrib-python

AprilTag vs ArUco:
  - AprilTag对光照变化更鲁棒
  - 更低的误检率（False Positive Rate）
  - 支持更大的检测距离
  - 适合户外和复杂光照环境
"""

import cv2
import numpy as np
import math


class AprilTagDetector:
    """AprilTag标记检测器"""

    # 支持的Tag家族
    TAG_FAMILIES = {
        'tag16h5':  cv2.aruco.DICT_APRILTAG_16h5,
        'tag25h9':  cv2.aruco.DICT_APRILTAG_25h9,
        'tag36h10': cv2.aruco.DICT_APRILTAG_36h10,
        'tag36h11': cv2.aruco.DICT_APRILTAG_36h11,
    }

    def __init__(self, family='tag36h11', marker_length=0.05,
                 camera_matrix=None, dist_coeffs=None,
                 min_area=100, max_area_ratio=0.5):
        """
        参数:
            family: Tag家族类型
            marker_length: 标记物理边长(米)
            camera_matrix: 3x3相机内参矩阵
            dist_coeffs: 畸变系数
            min_area: 最小轮廓面积(像素)
            max_area_ratio: 最大面积占画面比例
        """
        self.marker_length = marker_length
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros((5, 1))
        self.min_area = min_area
        self.max_area_ratio = max_area_ratio

        # 初始化ArUco检测器(AprilTag使用ArUco API)
        dict_id = self.TAG_FAMILIES.get(family, cv2.aruco.DICT_APRILTAG_36h11)
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        try:
            params = cv2.aruco.DetectorParameters()
            # AprilTag优化参数
            params.adaptiveThreshWinMinSize = 10
            params.adaptiveThreshWinMaxSize = 50
            params.adaptiveThreshConstant = 7
            params.minMarkerPerimeterRate = 0.03
            params.maxMarkerPerimeterRate = 4.0
            params.polygonalApproxAccuracyRate = 0.05
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            self.use_new_api = True
        except AttributeError:
            self.use_new_api = False

    def detect(self, frame):
        """
        检测AprilTag标记
        参数:
            frame: BGR图像
        返回:
            results: list of dict, 每个包含:
                - id: 标记ID
                - corners: 4个角点坐标
                - center: 中心点
                - rvec: 旋转向量
                - tvec: 平移向量
                - area: 轮廓面积
        """
        if frame is None:
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame

        if self.use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            )

        results = []
        if ids is None or len(ids) == 0:
            return results

        h, w = frame.shape[:2]
        max_area = h * w * self.max_area_ratio

        for i, marker_id in enumerate(ids.flatten()):
            pts = corners[i][0]  # shape: (4, 2)

            # 面积过滤
            area = cv2.contourArea(pts.astype(np.float32))
            if area < self.min_area or area > max_area:
                continue

            # 计算中心点
            cx = float(np.mean(pts[:, 0]))
            cy = float(np.mean(pts[:, 1]))

            result = {
                'id': int(marker_id),
                'corners': pts.astype(int).tolist(),
                'center': (int(cx), int(cy)),
                'area': float(area),
                'rvec': None,
                'tvec': None,
            }

            # 位姿估计
            if self.camera_matrix is not None:
                obj_points = np.array([
                    [-self.marker_length / 2,  self.marker_length / 2, 0],
                    [ self.marker_length / 2,  self.marker_length / 2, 0],
                    [ self.marker_length / 2, -self.marker_length / 2, 0],
                    [-self.marker_length / 2, -self.marker_length / 2, 0],
                ], dtype=np.float32)

                img_points = pts.astype(np.float32)
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points,
                    self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if success:
                    result['rvec'] = rvec
                    result['tvec'] = tvec
                    result['distance'] = float(np.linalg.norm(tvec))

            results.append(result)

        return results

    def draw_detections(self, frame, results):
        """在图像上绘制检测结果"""
        vis = frame.copy()
        for r in results:
            pts = np.array(r['corners'], dtype=np.int32)
            cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
            cx, cy = r['center']
            label = f"ID:{r['id']}"
            if 'distance' in r and r['distance'] is not None:
                label += f" {r['distance']:.2f}m"
            cv2.putText(vis, label, (cx - 30, cy - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return vis

    def get_tags_by_id(self, results, target_id):
        """按ID过滤检测结果"""
        return [r for r in results if r['id'] == target_id]


def demo():
    """演示AprilTag检测"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    detector = AprilTagDetector(family='tag36h11', marker_length=0.05)
    print("AprilTag检测器启动 (按q退出)")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = detector.detect(frame)
        vis = detector.draw_detections(frame, results)

        cv2.putText(vis, f"Tags: {len(results)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("AprilTag Detector", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo()
