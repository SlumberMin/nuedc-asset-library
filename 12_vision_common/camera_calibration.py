#!/usr/bin/env python3
"""
相机标定工具
功能：棋盘格标定 + 畸变校正
适用：OpenCV + Orange Pi 5 / USB摄像头
输出：camera_matrix, dist_coeffs (.npz格式)
"""

import cv2
import numpy as np
import os
import glob
import time


class CameraCalibrator:
    """相机标定器"""

    def __init__(self, chessboard_size=(9, 6), square_size=0.025):
        """
        参数:
            chessboard_size: 棋盘格内角点数 (列, 行)
            square_size: 每格物理边长(米)
        """
        self.chessboard_size = chessboard_size
        self.square_size = square_size

        # 准备棋盘格3D坐标
        self.obj_points = []  # 3D点
        self.img_points = []  # 2D点
        self.objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:chessboard_size[0],
                                      0:chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size

        # 标定结果
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None
        self.reprojection_error = None

    def find_corners(self, image):
        """
        在图像中查找棋盘格角点

        返回:
            found: 是否找到
            corners: 角点坐标
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image

        # 查找角点
        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH +
                 cv2.CALIB_CB_NORMALIZE_IMAGE +
                 cv2.CALIB_CB_FAST_CHECK)
        found, corners = cv2.findChessboardCorners(gray, self.chessboard_size, flags)

        if found:
            # 亚像素精化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        return found, corners

    def add_image(self, image):
        """
        添加一张标定图像

        返回:
            success: 是否成功添加
        """
        found, corners = self.find_corners(image)
        if found:
            self.img_points.append(corners)
            self.obj_points.append(self.objp)
            return True
        return False

    def add_images_from_dir(self, image_dir, pattern='*.jpg'):
        """
        从目录批量添加标定图像

        返回:
            count: 成功添加的图像数
        """
        search_pattern = os.path.join(image_dir, pattern)
        image_files = glob.glob(search_pattern)

        count = 0
        for filepath in image_files:
            image = cv2.imread(filepath)
            if image is not None:
                if self.add_image(image):
                    count += 1
                    print(f"  [+] {os.path.basename(filepath)}")
                else:
                    print(f"  [-] {os.path.basename(filepath)} (未找到角点)")

        return count

    def calibrate(self, image_size=None):
        """
        执行标定

        参数:
            image_size: (width, height) 图像尺寸

        返回:
            success: 是否成功
        """
        if len(self.img_points) < 3:
            print(f"[错误] 标定图像不足(需至少3张，当前{len(self.img_points)}张)")
            return False

        if image_size is None:
            print("[错误] 需要指定图像尺寸")
            return False

        print(f"\n[标定] 使用 {len(self.img_points)} 张图像...")

        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = \
            cv2.calibrateCamera(
                self.obj_points, self.img_points, image_size,
                None, None,
                flags=cv2.CALIB_FIX_ASPECT_RATIO
            )

        self.reprojection_error = ret
        self.image_size = image_size

        print(f"[标定] 完成!")
        print(f"  重投影误差: {ret:.4f} px")
        print(f"  焦距: fx={self.camera_matrix[0,0]:.2f}, fy={self.camera_matrix[1,1]:.2f}")
        print(f"  光心: cx={self.camera_matrix[0,2]:.2f}, cy={self.camera_matrix[1,2]:.2f}")
        print(f"  畸变系数: {self.dist_coeffs.flatten()}")

        return True

    def undistort(self, image):
        """
        校正图像畸变

        参数:
            image: 畸变图像

        返回:
            校正后的图像
        """
        if self.camera_matrix is None:
            print("[错误] 未标定")
            return image

        return cv2.undistort(image, self.camera_matrix, self.dist_coeffs)

    def undistort_with_roi(self, image):
        """校正畸变并裁剪有效区域"""
        if self.camera_matrix is None:
            return image

        h, w = image.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )

        undistorted = cv2.undistort(image, self.camera_matrix, self.dist_coeffs, None, new_mtx)

        # 裁剪
        x, y, w, h = roi
        if w > 0 and h > 0:
            undistorted = undistorted[y:y+h, x:x+w]

        return undistorted

    def save(self, filepath='camera_params.npz'):
        """保存标定参数"""
        if self.camera_matrix is None:
            print("[错误] 未标定，无法保存")
            return

        np.savez(filepath,
                 camera_matrix=self.camera_matrix,
                 dist_coeffs=self.dist_coeffs,
                 image_size=np.array(self.image_size),
                 reprojection_error=self.reprojection_error)
        print(f"[保存] 标定参数: {filepath}")

    def load(self, filepath='camera_params.npz'):
        """加载标定参数"""
        if not os.path.exists(filepath):
            print(f"[错误] 文件不存在: {filepath}")
            return False

        data = np.load(filepath)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']
        if 'image_size' in data:
            self.image_size = tuple(data['image_size'])
        if 'reprojection_error' in data:
            self.reprojection_error = float(data['reprojection_error'])

        print(f"[加载] 标定参数: {filepath}")
        print(f"  焦距: fx={self.camera_matrix[0,0]:.2f}, fy={self.camera_matrix[1,1]:.2f}")
        return True

    def draw_corners(self, image, corners, found):
        """绘制角点"""
        vis = image.copy()
        if found:
            cv2.drawChessboardCorners(vis, self.chessboard_size, corners, found)
        return vis

    def get_undistort_maps(self):
        """获取畸变校正映射表(可缓存加速)"""
        if self.camera_matrix is None:
            return None, None

        h, w = self.image_size[1], self.image_size[0]
        map1, map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix, self.dist_coeffs, None, None, (w, h), cv2.CV_32FC1
        )
        return map1, map2

    def print_report(self):
        """打印标定报告"""
        print("\n" + "=" * 50)
        print("相机标定报告")
        print("=" * 50)

        if self.camera_matrix is not None:
            print(f"\n相机内参矩阵:")
            print(self.camera_matrix)
            print(f"\n畸变系数:")
            print(self.dist_coeffs.flatten())
            print(f"\n焦距: fx={self.camera_matrix[0,0]:.2f}, fy={self.camera_matrix[1,1]:.2f}")
            print(f"光心: cx={self.camera_matrix[0,2]:.2f}, cy={self.camera_matrix[1,2]:.2f}")
            if self.reprojection_error is not None:
                print(f"重投影误差: {self.reprojection_error:.4f} px")
                if self.reprojection_error < 0.5:
                    print("评价: 优秀 ✓")
                elif self.reprojection_error < 1.0:
                    print("评价: 良好")
                else:
                    print("评价: 建议重新标定")
        else:
            print("未标定")

        print("=" * 50)


class RealtimeCalibrator:
    """实时标定流程"""

    def __init__(self, camera_id=0, chessboard_size=(9, 6), square_size=0.025):
        self.camera_id = camera_id
        self.calibrator = CameraCalibrator(chessboard_size, square_size)
        self.save_dir = 'calibration_images'
        os.makedirs(self.save_dir, exist_ok=True)

    def capture_and_calibrate(self, n_images=20, auto_capture=True):
        """
        实时采集标定图像并标定

        参数:
            n_images: 需要采集的图像数
            auto_capture: 自动采集(检测到角点就保存)
        """
        cap = cv2.VideoCapture(self.camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not cap.isOpened():
            print(f"[错误] 无法打开摄像头 {self.camera_id}")
            return

        print("=" * 50)
        print("相机标定 - 实时采集")
        print("=" * 50)
        print(f"需要 {n_images} 张标定图像")
        print("操作: Space=手动拍摄 | a=自动模式 | q=开始标定")
        print("=" * 50)

        captured = 0
        auto_mode = auto_capture
        last_capture_time = 0

        while captured < n_images:
            ret, frame = cap.read()
            if not ret:
                break

            found, corners = self.calibrator.find_corners(frame)
            vis = self.calibrator.draw_corners(frame, corners, found)

            # 状态信息
            status = f"Captured: {captured}/{n_images}"
            cv2.putText(vis, status, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(vis, f"Auto: {'ON' if auto_mode else 'OFF'}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                       (0, 255, 0) if auto_mode else (0, 0, 255), 1)

            if found:
                cv2.putText(vis, "FOUND", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow('Camera Calibration', vis)

            # 自动采集
            if auto_mode and found:
                current_time = time.time()
                if current_time - last_capture_time > 1.0:  # 每秒最多1张
                    if self.calibrator.add_image(frame):
                        filepath = os.path.join(self.save_dir, f'calib_{captured:03d}.jpg')
                        cv2.imwrite(filepath, frame)
                        captured += 1
                        last_capture_time = current_time
                        print(f"  自动拍摄 {captured}/{n_images}")

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or captured >= n_images:
                break
            elif key == ord(' '):  # 手动拍摄
                if found:
                    if self.calibrator.add_image(frame):
                        filepath = os.path.join(self.save_dir, f'calib_{captured:03d}.jpg')
                        cv2.imwrite(filepath, frame)
                        captured += 1
                        print(f"  手动拍摄 {captured}/{n_images}")
                else:
                    print("  未检测到棋盘格")
            elif key == ord('a'):
                auto_mode = not auto_mode
                print(f"  自动模式: {'开' if auto_mode else '关'}")

        cap.release()
        cv2.destroyAllWindows()

        # 执行标定
        if captured >= 3:
            image_size = (640, 480)
            if self.calibrator.calibrate(image_size):
                self.calibrator.print_report()
                self.calibrator.save('camera_params.npz')

                # 演示校正效果
                self._demo_undistort()
        else:
            print("[错误] 标定图像不足")

    def _demo_undistort(self):
        """演示畸变校正效果"""
        cap = cv2.VideoCapture(self.camera_id)
        if not cap.isOpened():
            return

        print("\n畸变校正预览 (q退出)")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            undistorted = self.calibrator.undistort(frame)

            # 并排显示
            h, w = frame.shape[:2]
            combined = np.zeros((h, w*2, 3), dtype=np.uint8)
            combined[:, :w] = frame
            combined[:, w:] = undistorted

            cv2.putText(combined, "Original", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(combined, "Undistorted", (w+10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow('Undistortion Comparison', combined)

            if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
                break

        cap.release()
        cv2.destroyAllWindows()


def run_calibration(camera_id=0, chessboard=(9, 6), square_size=0.025, n_images=20):
    """运行标定流程"""
    calibrator = RealtimeCalibrator(camera_id, chessboard, square_size)
    calibrator.capture_and_calibrate(n_images)


def undistort_image(image_path, params_path='camera_params.npz'):
    """用已标定参数校正图像"""
    calibrator = CameraCalibrator()
    if not calibrator.load(params_path):
        return

    image = cv2.imread(image_path)
    if image is None:
        print(f"[错误] 无法读取: {image_path}")
        return

    result = calibrator.undistort(image)
    output_path = image_path.replace('.', '_undistorted.')
    cv2.imwrite(output_path, result)
    print(f"[保存] 校正后图像: {output_path}")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='相机标定工具')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--chessboard', type=str, default='9,6',
                       help='棋盘格内角点数 (列,行)')
    parser.add_argument('--square-size', type=float, default=0.025,
                       help='棋盘格边长(米)')
    parser.add_argument('--n-images', type=int, default=20,
                       help='标定图像数')
    parser.add_argument('--undistort', type=str, default=None,
                       help='校正指定图像')
    parser.add_argument('--params', type=str, default='camera_params.npz',
                       help='标定参数文件')
    args = parser.parse_args()

    chessboard = tuple(map(int, args.chessboard.split(',')))

    if args.undistort:
        undistort_image(args.undistort, args.params)
    else:
        run_calibration(args.camera, chessboard, args.square_size, args.n_images)
