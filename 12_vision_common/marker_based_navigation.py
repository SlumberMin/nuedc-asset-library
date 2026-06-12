#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于标记的导航模块
===================
功能：使用 ArUco 标记进行位姿估计与导航，适用于电赛定点巡航/机械臂视觉引导等场景。

核心能力：
    1. ArUco 标记检测（支持多种字典）
    2. 基于标记的相机位姿估计（solvePnP）
    3. 多标记融合定位
    4. 导航控制（航向/距离 PID）
    5. 坐标系变换与标定工具

依赖：
    pip install opencv-python opencv-contrib-python numpy

使用示例：
    python marker_based_navigation.py                    # 摄像头实时检测
    python marker_based_navigation.py --generate 5       # 生成5个ArUco标记图
    python marker_based_navigation.py --target 42        # 导航到标记ID=42

作者：电赛视觉通用代码库
"""

import cv2
import numpy as np
import time
import threading
from collections import deque
from typing import Tuple, Optional, List, Dict


# ============================================================
#  ArUco 字典映射
# ============================================================
ARUCO_DICTS = {
    '4X4_50': cv2.aruco.DICT_4X4_50,
    '4X4_100': cv2.aruco.DICT_4X4_100,
    '4X4_250': cv2.aruco.DICT_4X4_250,
    '4X4_1000': cv2.aruco.DICT_4X4_1000,
    '5X5_50': cv2.aruco.DICT_5X5_50,
    '5X5_100': cv2.aruco.DICT_5X5_100,
    '6X6_50': cv2.aruco.DICT_6X6_50,
    '6X6_100': cv2.aruco.DICT_6X6_100,
    '7X7_50': cv2.aruco.DICT_7X7_50,
}

# 兼容不同 OpenCV 版本的 ArUco API
def _get_aruco_detector(dict_name: str = '4X4_50'):
    """获取 ArUco 检测器（兼容 OpenCV 4.7+ 和旧版 API）"""
    dict_id = ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
    
    try:
        # OpenCV >= 4.7
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        detector_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
        return detector, aruco_dict, detector_params
    except AttributeError:
        pass
    
    try:
        # OpenCV 4.6 及以下
        aruco_dict = cv2.aruco.Dictionary_get(dict_id)
        detector_params = cv2.aruco.DetectorParameters_create()
        return None, aruco_dict, detector_params
    except AttributeError:
        pass
    
    raise RuntimeError("OpenCV 版本不支持 ArUco，请安装 opencv-contrib-python")


def _detect_markers(detector, aruco_dict, params, gray):
    """统一 ArUco 检测接口"""
    try:
        if detector is not None:
            # OpenCV >= 4.7
            corners, ids, rejected = detector.detectMarkers(gray)
        else:
            # OpenCV <= 4.6
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
        return corners, ids, rejected
    except Exception as e:
        print(f"[ArUco 检测异常] {e}")
        return [], None, []


class CameraCalibrator:
    """
    相机标定工具
    
    使用棋盘格标定相机内参和畸变系数。
    """
    
    def __init__(self, chessboard_size: Tuple[int, int] = (9, 6), 
                 square_size: float = 0.025):
        """
        Args:
            chessboard_size: 棋盘格内角点数 (列, 行)
            square_size: 棋盘格方块边长（米）
        """
        self.chessboard_size = chessboard_size
        self.square_size = square_size
        
        # 标定点
        self.obj_points = []  # 3D 世界坐标
        self.img_points = []  # 2D 图像坐标
        
        # 准备棋盘格 3D 坐标模板
        self.objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:chessboard_size[0], 
                                      0:chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
    
    def add_frame(self, frame: np.ndarray) -> bool:
        """
        添加一帧标定图像
        
        Args:
            frame: BGR 图像
        Returns:
            是否成功检测到棋盘格
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
        
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            self.obj_points.append(self.objp)
            self.img_points.append(corners_refined)
            
            return True
        return False
    
    def calibrate(self, image_size: Tuple[int, int]) -> Tuple[np.ndarray, np.ndarray]:
        """
        执行标定
        
        Args:
            image_size: 图像尺寸 (宽, 高)
        Returns:
            (camera_matrix, dist_coeffs)
        """
        if len(self.obj_points) < 3:
            raise ValueError(f"标定图像不足（需至少3张，当前{len(self.obj_points)}张）")
        
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, image_size, None, None
        )
        
        print(f"[标定完成] 重投影误差: {ret:.4f}")
        print(f"相机矩阵:\n{mtx}")
        print(f"畸变系数: {dist.flatten()}")
        
        return mtx, dist


class ArUcoNavigator:
    """
    ArUco 标记导航器
    
    功能：
    1. 检测 ArUco 标记
    2. 估计标记位姿（相对于相机）
    3. 计算到标记的距离和方位
    4. 输出导航控制指令
    """
    
    def __init__(self, 
                 dict_name: str = '4X4_50',
                 marker_size: float = 0.05,
                 camera_matrix: Optional[np.ndarray] = None,
                 dist_coeffs: Optional[np.ndarray] = None):
        """
        Args:
            dict_name: ArUco 字典名称
            marker_size: 标记物理尺寸（米）
            camera_matrix: 相机内参矩阵 (3, 3)，None 则使用默认值
            dist_coeffs: 畸变系数，None 则使用默认值
        """
        self.marker_size = marker_size
        
        # 初始化 ArUco 检测器
        self._detector, self._aruco_dict, self._params = _get_aruco_detector(dict_name)
        
        # 相机参数（如果未提供，使用默认近似值）
        if camera_matrix is not None:
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5)
        else:
            # 默认内参（需要标定后替换！）
            self.camera_matrix = np.array([
                [600, 0, 320],
                [0, 600, 240],
                [0, 0, 1]
            ], dtype=np.float64)
            self.dist_coeffs = np.zeros(5, dtype=np.float64)
            print("[警告] 使用默认相机参数，请进行标定以获得准确位姿")
        
        # 导航目标
        self.target_marker_id: Optional[int] = None
        self.target_distance: float = 0.5  # 目标距离（米）
        self.target_heading: float = 0.0   # 目标航向偏角（度）
        
        # 导航 PID
        self._pid_distance = {'kp': 50.0, 'ki': 1.0, 'kd': 10.0, 
                               'integral': 0.0, 'prev_error': 0.0}
        self._pid_heading = {'kp': 2.0, 'ki': 0.05, 'kd': 0.5, 
                              'integral': 0.0, 'prev_error': 0.0}
        
        # 标记地图（ID -> 世界坐标）
        self.marker_map: Dict[int, np.ndarray] = {}
        
        # 结果缓存
        self._lock = threading.Lock()
        self._last_result: Optional[Dict] = None
        
        # FPS
        self.fps = 0.0
        self._frame_count = 0
        self._fps_timer = time.time()
        
        # 轨迹
        self._position_history = deque(maxlen=100)
    
    def set_camera_params(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        """设置标定后的相机参数"""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
    
    def set_marker_map(self, marker_map: Dict[int, Tuple[float, float, float]]):
        """
        设置标记地图
        
        Args:
            marker_map: {marker_id: (x, y, z)} 世界坐标
        """
        self.marker_map = {k: np.array(v, dtype=np.float64) for k, v in marker_map.items()}
    
    def set_target(self, marker_id: int, distance: float = 0.5):
        """设置导航目标标记"""
        self.target_marker_id = marker_id
        self.target_distance = distance
    
    def _pid_compute(self, pid: Dict, error: float, dt: float) -> float:
        """PID 计算"""
        if dt <= 0:
            dt = 0.01
        
        pid['integral'] += error * dt
        pid['integral'] = np.clip(pid['integral'], -100, 100)
        
        derivative = (error - pid['prev_error']) / dt
        output = pid['kp'] * error + pid['ki'] * pid['integral'] + pid['kd'] * derivative
        output = np.clip(output, -100, 100)
        
        pid['prev_error'] = error
        return output
    
    def detect_and_estimate(self, frame: np.ndarray) -> Dict:
        """
        检测标记并估计位姿
        
        Args:
            frame: BGR 输入图像
        Returns:
            {
                'corners': 标记角点,
                'ids': 标记 ID,
                'markers': List[Dict] - 每个标记的详细信息
                'target_info': 目标标记信息（如果设置了目标）
                'navigation': 导航控制指令
            }
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 检测标记
        corners, ids, rejected = _detect_markers(
            self._detector, self._aruco_dict, self._params, gray
        )
        
        markers = []
        
        if ids is not None and len(ids) > 0:
            for i, (corner, marker_id) in enumerate(zip(corners, ids.flatten())):
                # 估计单个标记的位姿
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corner, self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                
                rvec = rvec[0][0]
                tvec = tvec[0][0]
                
                # 计算距离
                distance = np.linalg.norm(tvec)
                
                # 计算方位角（相对于相机光轴）
                heading_x = np.degrees(np.arctan2(tvec[0], tvec[2]))  # 水平偏角
                heading_y = np.degrees(np.arctan2(tvec[1], tvec[2]))  # 垂直偏角
                
                # 旋转矩阵 -> 欧拉角
                rmat, _ = cv2.Rodrigues(rvec)
                euler = self._rotation_matrix_to_euler(rmat)
                
                marker_info = {
                    'id': int(marker_id),
                    'corners': corner[0],
                    'rvec': rvec,
                    'tvec': tvec,
                    'distance': distance,
                    'heading_x': heading_x,
                    'heading_y': heading_y,
                    'rotation_matrix': rmat,
                    'euler': euler,  # (roll, pitch, yaw) 度
                    'center': (int(np.mean(corner[0][:, 0])), 
                              int(np.mean(corner[0][:, 1])))
                }
                
                markers.append(marker_info)
        
        # 目标标记信息
        target_info = None
        navigation = {'forward': 0, 'turn': 0, 'arrived': False, 'found': False}
        
        if self.target_marker_id is not None:
            target_markers = [m for m in markers if m['id'] == self.target_marker_id]
            
            if target_markers:
                target = target_markers[0]
                target_info = target
                navigation['found'] = True
                
                # 计算导航误差
                distance_error = target['distance'] - self.target_distance
                heading_error = target['heading_x'] - self.target_heading
                
                # 是否到达
                if abs(distance_error) < 0.05 and abs(heading_error) < 5:
                    navigation['arrived'] = True
                    self._pid_distance['integral'] = 0
                    self._pid_heading['integral'] = 0
                else:
                    current_time = time.time()
                    dt = current_time - getattr(self, '_last_nav_time', current_time)
                    self._last_nav_time = current_time
                    
                    # PID 控制
                    navigation['forward'] = self._pid_compute(
                        self._pid_distance, distance_error, dt)
                    navigation['turn'] = self._pid_compute(
                        self._pid_heading, heading_error, dt)
        
        result = {
            'corners': corners,
            'ids': ids,
            'markers': markers,
            'target_info': target_info,
            'navigation': navigation,
            'n_markers': len(markers)
        }
        
        with self._lock:
            self._last_result = result
        
        self._update_fps()
        return result
    
    def _rotation_matrix_to_euler(self, R: np.ndarray) -> Tuple[float, float, float]:
        """旋转矩阵转欧拉角 (roll, pitch, yaw)，单位：度"""
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        
        if sy > 1e-6:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0
        
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)
    
    def detect_threaded(self, frame: np.ndarray, callback=None):
        """多线程检测"""
        frame_copy = frame.copy()
        
        def _worker():
            result = self.detect_and_estimate(frame_copy)
            if callback:
                callback(result)
        
        t = threading.Thread(target=_worker, daemon=True)
        t.start()
        return t
    
    def draw(self, frame: np.ndarray, result: Optional[Dict] = None) -> np.ndarray:
        """
        绘制检测与导航结果
        
        Args:
            frame: 输入图像
            result: 检测结果
        Returns:
            可视化图像
        """
        vis = frame.copy()
        h, w = vis.shape[:2]
        
        if result is None:
            with self._lock:
                result = self._last_result
        
        if result is None:
            return vis
        
        # 绘制检测到的标记
        if result['ids'] is not None:
            cv2.aruco.drawDetectedMarkers(vis, result['corners'], result['ids'])
        
        # 绘制每个标记的位姿
        for marker in result['markers']:
            # 绘制坐标轴
            cv2.drawFrameAxes(vis, self.camera_matrix, self.dist_coeffs,
                            marker['rvec'], marker['tvec'], self.marker_size * 0.5)
            
            # 标记信息
            cx, cy = marker['center']
            info_text = [
                f"ID:{marker['id']}",
                f"D:{marker['distance']:.3f}m",
                f"H:{marker['heading_x']:.1f}deg"
            ]
            
            # 高亮目标标记
            is_target = (self.target_marker_id is not None and 
                        marker['id'] == self.target_marker_id)
            color = (0, 255, 0) if is_target else (255, 255, 0)
            
            for j, text in enumerate(info_text):
                cv2.putText(vis, text, (cx + 10, cy - 20 + j * 18),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        
        # 导航信息
        if self.target_marker_id is not None:
            nav = result['navigation']
            y_start = 30
            
            if nav['arrived']:
                cv2.putText(vis, "ARRIVED!", (w // 2 - 60, h // 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            elif not nav['found']:
                cv2.putText(vis, f"Searching for marker {self.target_marker_id}...",
                           (10, y_start), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                info = [
                    f"Target: Marker {self.target_marker_id}",
                    f"Forward: {nav['forward']:.1f}",
                    f"Turn: {nav['turn']:.1f}",
                ]
                for i, text in enumerate(info):
                    cv2.putText(vis, text, (10, y_start + i * 22),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # FPS
        cv2.putText(vis, f"FPS: {self.fps:.1f} | Markers: {result['n_markers']}",
                    (w - 250, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return vis
    
    def _update_fps(self):
        self._frame_count += 1
        elapsed = time.time() - self._fps_timer
        if elapsed >= 1.0:
            self.fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_timer = time.time()


def generate_markers(dict_name: str = '4X4_50', 
                     marker_ids: Optional[List[int]] = None,
                     marker_size_px: int = 200,
                     output_dir: str = '.'):
    """
    生成 ArUco 标记图片
    
    Args:
        dict_name: ArUco 字典名称
        marker_ids: 要生成的标记 ID 列表
        marker_size_px: 标记图片尺寸（像素）
        output_dir: 输出目录
    """
    import os
    
    if marker_ids is None:
        marker_ids = [0, 1, 2, 3, 4]
    
    dict_id = ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
    
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    except AttributeError:
        aruco_dict = cv2.aruco.Dictionary_get(dict_id)
    
    os.makedirs(output_dir, exist_ok=True)
    
    for marker_id in marker_ids:
        try:
            img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)
        except AttributeError:
            img = np.zeros((marker_size_px, marker_size_px), dtype=np.uint8)
            cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_px, img, 1)
        
        # 添加白边和 ID 标注
        border = 30
        img_border = np.ones((marker_size_px + 2 * border, marker_size_px + 2 * border), 
                              dtype=np.uint8) * 255
        img_border[border:border + marker_size_px, border:border + marker_size_px] = img
        
        # 标注 ID
        img_color = cv2.cvtColor(img_border, cv2.COLOR_GRAY2BGR)
        cv2.putText(img_color, f"ID: {marker_id}", (border, marker_size_px + border + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        filename = os.path.join(output_dir, f"aruco_{dict_name}_id{marker_id}.png")
        cv2.imwrite(filename, img_color)
        print(f"[生成] {filename}")
    
    print(f"\n共生成 {len(marker_ids)} 个标记图片到 {os.path.abspath(output_dir)}")
    print("请打印后用于标定和导航。")


def run_camera_demo(target_id: Optional[int] = None):
    """摄像头实时检测/导航演示"""
    print("=" * 60)
    print("  ArUco 标记导航")
    print("  按 'q' 退出 | 按 'c' 标定相机")
    if target_id is not None:
        print(f"  导航目标: 标记 ID={target_id}")
    print("=" * 60)
    
    navigator = ArUcoNavigator(dict_name='4X4_50', marker_size=0.05)
    
    if target_id is not None:
        navigator.set_target(target_id, distance=0.3)
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[错误] 无法打开摄像头")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        result = navigator.detect_and_estimate(frame)
        vis = navigator.draw(frame, result)
        
        cv2.imshow("ArUco Navigation", vis)
        
        # 打印检测结果
        for m in result['markers']:
            print(f"\r  ID:{m['id']} D:{m['distance']:.3f}m "
                  f"H:{m['heading_x']:.1f}deg", end='', flush=True)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            # 简易标定流程
            cap.release()
            cv2.destroyAllWindows()
            run_calibration()
            cap = cv2.VideoCapture(0)
    
    print()
    cap.release()
    cv2.destroyAllWindows()


def run_calibration():
    """简易相机标定流程"""
    print("\n=== 相机标定 ===")
    print("将棋盘格放在摄像头前，按空格采集，至少采集10张，按 'q' 完成")
    
    calibrator = CameraCalibrator(chessboard_size=(9, 6), square_size=0.025)
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        return
    
    count = 0
    while count < 20:
        ret, frame = cap.read()
        if not ret:
            break
        
        vis = frame.copy()
        
        # 实时检测棋盘格
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (9, 6), None)
        if found:
            cv2.drawChessboardCorners(vis, (9, 6), corners, found)
        
        cv2.putText(vis, f"Samples: {count}/20 | SPACE=capture | Q=finish",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("Calibration", vis)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' ') and found:
            calibrator.add_frame(frame)
            count += 1
            print(f"  采集 #{count}")
        elif key == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    
    if count >= 5:
        try:
            mtx, dist = calibrator.calibrate((640, 480))
            np.savez('camera_calib.npz', camera_matrix=mtx, dist_coeffs=dist)
            print("[保存] camera_calib.npz")
        except ValueError as e:
            print(f"[错误] {e}")
    else:
        print("[错误] 标定图像不足")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description="ArUco 标记导航")
    parser.add_argument('--generate', type=int, nargs='?', const=5, 
                        help='生成标记图片（默认5个）')
    parser.add_argument('--target', type=int, help='导航目标标记 ID')
    parser.add_argument('--dict', type=str, default='4X4_50', 
                        choices=list(ARUCO_DICTS.keys()),
                        help='ArUco 字典')
    args = parser.parse_args()
    
    if args.generate:
        generate_markers(dict_name=args.dict, 
                        marker_ids=list(range(args.generate)))
    else:
        run_camera_demo(target_id=args.target)
