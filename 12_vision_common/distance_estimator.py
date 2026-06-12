#!/usr/bin/env python3
"""
单目测距
功能：基于已知目标尺寸 + 相机标定参数的单目距离估计
适用：OpenCV + Orange Pi 5
原理：D = (W_real × f) / W_pixel
"""

import cv2
import numpy as np
import json
import os


class DistanceEstimator:
    """单目测距器"""

    def __init__(self, focal_length=None, known_width=None,
                 camera_matrix=None, dist_coeffs=None):
        """
        参数:
            focal_length: 焦距(像素) - 可由标定得到或已知值
            known_width: 已知目标宽度(米)
            camera_matrix: 3x3相机内参 (如果有则自动计算focal_length)
            dist_coeffs: 畸变系数
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

        if camera_matrix is not None:
            # 从相机矩阵提取焦距
            self.focal_length = (camera_matrix[0, 0] + camera_matrix[1, 1]) / 2
        else:
            self.focal_length = focal_length

        self.known_width = known_width

        # 参考距离数据 (用于线性拟合)
        self.calibration_data = []  # [(pixel_width, real_distance), ...]

    def load_camera_params(self, filepath):
        """加载相机参数(.npz)"""
        if not os.path.exists(filepath):
            print(f"[警告] 文件不存在: {filepath}")
            return False
        data = np.load(filepath)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']
        self.focal_length = (self.camera_matrix[0, 0] + self.camera_matrix[1, 1]) / 2
        print(f"[加载] 焦距: {self.focal_length:.2f}px")
        return True

    def calibrate_focal_length(self, known_distance, pixel_width):
        """
        标定焦距
        参数:
            known_distance: 已知距离(米)
            pixel_width: 该距离下目标像素宽度
        """
        if self.known_width is None:
            print("[错误] 需要设置已知目标宽度(known_width)")
            return
        self.focal_length = (pixel_width * known_distance) / self.known_width
        print(f"[标定] 焦距 = {self.focal_length:.2f}px")

    def add_calibration_point(self, pixel_width, real_distance):
        """添加标定点(用于多点拟合)"""
        self.calibration_data.append((pixel_width, real_distance))

    def fit_calibration(self):
        """用标定数据拟合 distance = a/pixel_width + b 模型"""
        if len(self.calibration_data) < 2:
            print("[警告] 至少需要2个标定点")
            return

        pixels = np.array([d[0] for d in self.calibration_data])
        distances = np.array([d[1] for d in self.calibration_data])

        # 拟合: D = k / pixel + b
        inv_pixels = 1.0 / pixels
        coeffs = np.polyfit(inv_pixels, distances, 1)
        self._fit_k = coeffs[0]
        self._fit_b = coeffs[1]
        self._use_fit = True

        print(f"[拟合] D = {self._fit_k:.4f}/pixel + {self._fit_b:.4f}")

    def estimate_distance(self, pixel_width):
        """
        估计距离

        参数:
            pixel_width: 目标在图像中的像素宽度

        返回:
            distance: 估计距离(米)
        """
        if pixel_width <= 0:
            return None

        # 优先使用拟合模型
        if hasattr(self, '_use_fit') and self._use_fit:
            return self._fit_k / pixel_width + self._fit_b

        # 使用简单模型: D = (W_real × f) / W_pixel
        if self.focal_length and self.known_width:
            return (self.known_width * self.focal_length) / pixel_width

        return None

    def estimate_distance_from_bbox(self, bbox):
        """
        从边界框估计距离

        参数:
            bbox: (x, y, w, h) 边界框

        返回:
            distance, info_dict
        """
        x, y, w, h = bbox
        distance = self.estimate_distance(w)

        return {
            'distance': distance,
            'pixel_width': w,
            'pixel_height': h,
            'bbox': bbox,
            'center': (x + w//2, y + h//2),
        }

    def estimate_distance_from_contour(self, contour):
        """从轮廓估计距离"""
        x, y, w, h = cv2.boundingRect(contour)
        return self.estimate_distance_from_bbox((x, y, w, h))

    def estimate_from_aruco(self, tvec):
        """
        从ArUco标记的平移向量估计距离
        参数:
            tvec: 3x1平移向量(来自solvePnP)

        返回:
            distance(米), x_offset, y_offset
        """
        tvec = tvec.flatten()
        distance = np.linalg.norm(tvec)
        return {
            'distance': distance,
            'x_offset': tvec[0],
            'y_offset': tvec[1],
            'z_offset': tvec[2],
        }

    def undistort_point(self, point):
        """校正畸变点坐标"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return point

        pts = np.array([[point]], dtype=np.float32)
        undistorted = cv2.undistortPoints(pts, self.camera_matrix, self.dist_coeffs,
                                          P=self.camera_matrix)
        return tuple(undistorted[0][0])

    def draw_distance(self, frame, result):
        """在画面上绘制测距结果"""
        vis = frame.copy()

        if result is None:
            return vis

        bbox = result.get('bbox')
        if bbox:
            x, y, w, h = bbox
            cv2.rectangle(vis, (x, y), (x+w, y+h), (0, 255, 0), 2)

        cx, cy = result.get('center', (0, 0))
        dist = result.get('distance')

        if dist is not None:
            text = f"Dist: {dist:.3f}m"
            cv2.putText(vis, text, (cx - 40, cy - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # 距离颜色指示
            if dist < 0.3:
                color = (0, 0, 255)  # 红色：很近
            elif dist < 1.0:
                color = (0, 165, 255)  # 橙色：近
            else:
                color = (0, 255, 0)  # 绿色：远
            cv2.circle(vis, (cx, cy), 8, color, -1)

        # 显示焦距信息
        if self.focal_length:
            cv2.putText(vis, f"f={self.focal_length:.0f}px", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        return vis


class MultiTargetDistanceEstimator:
    """多目标测距管理器"""

    def __init__(self):
        self.targets = {}  # name -> {estimator, detector_params}

    def add_target(self, name, known_width, color_lower, color_upper,
                   focal_length=None, camera_matrix=None):
        """添加一个目标类型"""
        estimator = DistanceEstimator(
            focal_length=focal_length,
            known_width=known_width,
            camera_matrix=camera_matrix
        )
        self.targets[name] = {
            'estimator': estimator,
            'color_lower': np.array(color_lower),
            'color_upper': np.array(color_upper),
            'known_width': known_width,
        }

    def detect_and_estimate(self, frame):
        """检测所有目标并估计距离"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        results = {}

        for name, cfg in self.targets.items():
            mask = cv2.inRange(hsv, cfg['color_lower'], cfg['color_upper'])
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    est_result = cfg['estimator'].estimate_from_contour(cnt)
                    est_result['target_name'] = name
                    results[name] = est_result
                    break  # 每个目标取最大轮廓

        return results


def run_demo(camera_id=0, known_width=0.05, focal_length=600):
    """实时演示"""
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    estimator = DistanceEstimator(focal_length=focal_length, known_width=known_width)

    print("=" * 50)
    print(f"单目测距 - 目标宽度: {known_width}m, 焦距: {focal_length}px")
    print("q/ESC: 退出 | c: 标定焦距")
    print("=" * 50)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 简单检测：找最大轮廓
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        vis = frame.copy()
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                result = estimator.estimate_from_contour(largest)
                vis = estimator.draw_distance(frame, result)

        cv2.imshow('Distance Estimator', vis)

        key = cv2.waitKey(1) & 0xFF
        if key in [ord('q'), 27]:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='单目测距')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--known-width', type=float, default=0.05,
                       help='已知目标宽度(米)')
    parser.add_argument('--focal-length', type=float, default=600,
                       help='焦距(像素)')
    parser.add_argument('--camera-params', type=str, default=None)
    args = parser.parse_args()

    run_demo(args.camera, args.known_width, args.focal_length)
