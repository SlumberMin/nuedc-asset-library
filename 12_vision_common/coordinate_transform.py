#!/usr/bin/env python3
"""
坐标变换工具
功能：像素坐标 → 世界坐标转换(透视变换 + 仿射变换)
适用：OpenCV + Orange Pi 5
"""

import cv2
import numpy as np
import json
import os


class CoordinateTransform:
    """坐标变换器"""

    def __init__(self):
        self.perspective_matrix = None  # 3x3透视变换矩阵
        self.perspective_matrix_inv = None  # 逆矩阵
        self.affine_matrix = None  # 2x3仿射变换矩阵
        self.affine_matrix_inv = None

    # ============ 透视变换 ============

    def calibrate_perspective(self, src_points, dst_points):
        """
        用4对点标定透视变换

        参数:
            src_points: 源(像素)坐标 [(x,y),...] 4个点
            dst_points: 目标(世界)坐标 [(X,Y),...] 4个点

        坐标系建议:
            - src: 图像像素坐标 (左上角为原点)
            - dst: 世界坐标 (如实际场地坐标,单位cm或m)
        """
        src = np.float32(src_points)
        dst = np.float32(dst_points)

        self.perspective_matrix = cv2.getPerspectiveTransform(src, dst)
        self.perspective_matrix_inv = cv2.getPerspectiveTransform(dst, src)

        print(f"[透视变换] 标定完成")
        print(f"  源点: {src_points}")
        print(f"  目标: {dst_points}")

    def pixel_to_world_perspective(self, point):
        """
        透视变换：像素坐标 → 世界坐标

        参数:
            point: (x, y) 像素坐标

        返回:
            (X, Y) 世界坐标
        """
        if self.perspective_matrix is None:
            print("[错误] 透视变换未标定")
            return None

        pts = np.array([[[point[0], point[1]]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pts, self.perspective_matrix)
        return (transformed[0][0][0], transformed[0][0][1])

    def world_to_pixel_perspective(self, point):
        """
        透视变换：世界坐标 → 像素坐标

        参数:
            point: (X, Y) 世界坐标

        返回:
            (x, y) 像素坐标
        """
        if self.perspective_matrix_inv is None:
            print("[错误] 透视变换未标定")
            return None

        pts = np.array([[[point[0], point[1]]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pts, self.perspective_matrix_inv)
        return (int(transformed[0][0][0]), int(transformed[0][0][1]))

    def transform_image_perspective(self, image, output_size):
        """
        透视变换整张图像(鸟瞰图)

        参数:
            image: 输入图像
            output_size: (width, height) 输出尺寸

        返回:
            变换后的图像
        """
        if self.perspective_matrix is None:
            print("[错误] 透视变换未标定")
            return image

        return cv2.warpPerspective(image, self.perspective_matrix, output_size)

    # ============ 仿射变换 ============

    def calibrate_affine(self, src_points, dst_points):
        """
        用3对点标定仿射变换

        参数:
            src_points: 源坐标 [(x,y),...] 3个点
            dst_points: 目标坐标 [(X,Y),...] 3个点
        """
        src = np.float32(src_points)
        dst = np.float32(dst_points)

        self.affine_matrix = cv2.getAffineTransform(src, dst)

        # 计算逆矩阵
        # 对于2x3仿射矩阵 [A|b]，逆为 [A^-1 | -A^-1*b]
        A = self.affine_matrix[:, :2]
        b = self.affine_matrix[:, 2]
        A_inv = np.linalg.inv(A)
        self.affine_matrix_inv = np.column_stack([A_inv, -A_inv @ b])

        print(f"[仿射变换] 标定完成")

    def pixel_to_world_affine(self, point):
        """仿射变换：像素坐标 → 世界坐标"""
        if self.affine_matrix is None:
            print("[错误] 仿射变换未标定")
            return None

        pts = np.array([[[point[0], point[1]]]], dtype=np.float32)
        transformed = cv2.transform(pts, self.affine_matrix)
        return (transformed[0][0][0], transformed[0][0][1])

    def world_to_pixel_affine(self, point):
        """仿射变换：世界坐标 → 像素坐标"""
        if self.affine_matrix_inv is None:
            print("[错误] 仿射变换未标定")
            return None

        pts = np.array([[[point[0], point[1]]]], dtype=np.float32)
        transformed = cv2.transform(pts, self.affine_matrix_inv)
        return (int(transformed[0][0][0]), int(transformed[0][0][1]))

    # ============ 射影变换 + Z估计 ============

    def calibrate_with_camera(self, camera_matrix, dist_coeffs, rvec, tvec):
        """
        使用相机参数进行完整的2D→3D映射
        (需要目标在同一平面上)

        参数:
            camera_matrix: 相机内参
            dist_coeffs: 畸变系数
            rvec: 旋转向量
            tvec: 平移向量
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.rvec = rvec
        self.tvec = tvec

        # 计算单应性矩阵
        rot_mat, _ = cv2.Rodrigues(rvec)
        # H = K * [r1 r2 t]
        H = camera_matrix @ np.column_stack([rot_mat[:, 0], rot_mat[:, 1], tvec])
        self.perspective_matrix = H
        self.perspective_matrix_inv = np.linalg.inv(H)

    def pixel_to_world_plane(self, point):
        """
        像素坐标 → 平面世界坐标(Z=0)

        使用相机参数和单应性矩阵

        返回:
            (X, Y) 世界坐标 or None
        """
        if self.perspective_matrix is None:
            return None

        pts = np.array([[[point[0], point[1], 1]]], dtype=np.float32)
        world = self.perspective_matrix_inv @ pts[0].T
        world /= world[2]  # 归一化
        return (float(world[0]), float(world[1]))

    # ============ 批量转换 ============

    def batch_pixel_to_world(self, points, method='perspective'):
        """
        批量坐标转换

        参数:
            points: [(x,y), ...] 像素坐标列表
            method: 'perspective' | 'affine'

        返回:
            [(X,Y), ...] 世界坐标列表
        """
        results = []
        for pt in points:
            if method == 'perspective':
                result = self.pixel_to_world_perspective(pt)
            else:
                result = self.pixel_to_world_affine(pt)
            results.append(result)
        return results

    # ============ 配置保存/加载 ============

    def save_config(self, filepath):
        """保存变换配置"""
        data = {}
        if self.perspective_matrix is not None:
            data['perspective_matrix'] = self.perspective_matrix.tolist()
        if self.affine_matrix is not None:
            data['affine_matrix'] = self.affine_matrix.tolist()

        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"[保存] 变换配置: {filepath}")

    def load_config(self, filepath):
        """加载变换配置"""
        if not os.path.exists(filepath):
            print(f"[错误] 文件不存在: {filepath}")
            return False

        with open(filepath, 'r') as f:
            data = json.load(f)

        if 'perspective_matrix' in data:
            self.perspective_matrix = np.array(data['perspective_matrix'])
            self.perspective_matrix_inv = np.linalg.inv(self.perspective_matrix)

        if 'affine_matrix' in data:
            self.affine_matrix = np.array(data['affine_matrix'])
            A = self.affine_matrix[:, :2]
            b = self.affine_matrix[:, 2]
            A_inv = np.linalg.inv(A)
            self.affine_matrix_inv = np.column_stack([A_inv, -A_inv @ b])

        print(f"[加载] 变换配置: {filepath}")
        return True

    # ============ 可视化 ============

    def draw_calibration(self, frame, src_points, dst_points=None):
        """绘制标定点和变换区域"""
        vis = frame.copy()

        # 绘制源点
        pts = np.array(src_points, dtype=np.int32)
        cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
        for i, pt in enumerate(src_points):
            cv2.circle(vis, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)
            cv2.putText(vis, f"P{i}({int(pt[0])},{int(pt[1])})",
                       (int(pt[0])+10, int(pt[1])-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        # 显示世界坐标
        if dst_points:
            for i, pt in enumerate(dst_points):
                cv2.putText(vis, f"W{i}({pt[0]:.1f},{pt[1]:.1f})",
                           (int(src_points[i][0])+10, int(src_points[i][1])+15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        return vis

    def draw_grid(self, frame, n_rows=4, n_cols=4):
        """在图像上绘制变换后的网格"""
        if self.perspective_matrix_inv is None:
            return frame

        vis = frame.copy()
        h, w = frame.shape[:2]

        # 在世界坐标系中生成网格点
        for i in range(n_rows + 1):
            for j in range(n_cols + 1):
                # 这里假设世界坐标范围
                pass

        return vis


class InteractiveCalibrator:
    """交互式标定工具(鼠标点击选取标定点)"""

    def __init__(self):
        self.points = []
        self.window_name = 'Calibration'
        self.callback_done = False

    def _mouse_callback(self, event, x, y, flags, param):
        """鼠标回调"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((x, y))
            print(f"点 {len(self.points)}: ({x}, {y})")

    def collect_points(self, frame, n_points=4, prompt="点击选择标定点"):
        """
        交互式收集标定点

        参数:
            frame: 显示图像
            n_points: 需要的点数

        返回:
            points: [(x,y), ...]
        """
        self.points = []
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)

        print(f"\n{prompt}")
        print(f"请在图像上点击 {n_points} 个点")

        while len(self.points) < n_points:
            vis = frame.copy()

            # 绘制已选点
            for i, pt in enumerate(self.points):
                cv2.circle(vis, pt, 5, (0, 0, 255), -1)
                cv2.putText(vis, f"P{i+1}", (pt[0]+10, pt[1]-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # 绘制连线
            if len(self.points) > 1:
                pts = np.array(self.points, dtype=np.int32)
                cv2.polylines(vis, [pts], False, (0, 255, 0), 1)

            # 提示信息
            remaining = n_points - len(self.points)
            cv2.putText(vis, f"Click {remaining} more points (ESC to cancel)",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow(self.window_name, vis)
            key = cv2.waitKey(30)
            if key == 27:  # ESC
                self.points = []
                break

        cv2.destroyWindow(self.window_name)
        return self.points


def run_interactive_demo(camera_id=0):
    """交互式标定演示"""
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    ret, frame = cap.read()
    if not ret:
        print("[错误] 无法打开摄像头")
        return

    transformer = CoordinateTransform()
    calibrator = InteractiveCalibrator()

    print("=" * 50)
    print("坐标变换交互标定")
    print("=" * 50)

    # 收集4个像素坐标点
    print("\n步骤1: 选择4个像素坐标点")
    src_points = calibrator.collect_points(frame, 4, "选择4个像素坐标点(左上→右上→右下→左下)")

    if len(src_points) != 4:
        print("[取消] 点数不足")
        cap.release()
        return

    # 输入对应的世界坐标
    print("\n步骤2: 输入对应的世界坐标(单位cm)")
    dst_points = []
    labels = ['左上', '右上', '右下', '左下']
    default_world = [(0, 0), (30, 0), (30, 30), (0, 30)]

    for i, label in enumerate(labels):
        inp = input(f"  {label} 世界坐标 (默认{default_world[i]}): ").strip()
        if inp:
            x, y = map(float, inp.split(','))
            dst_points.append((x, y))
        else:
            dst_points.append(default_world[i])

    # 标定
    transformer.calibrate_perspective(src_points, dst_points)

    # 测试转换
    print("\n步骤3: 测试转换 (点击图像上的点，按q退出)")
    calibrator.points = []
    cv2.namedWindow('Transform Test')
    test_transformer = CoordinateTransform()
    test_transformer.calibrate_perspective(src_points, dst_points)

    def test_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            world = test_transformer.pixel_to_world_perspective((x, y))
            if world:
                print(f"  像素({x}, {y}) → 世界({world[0]:.2f}, {world[1]:.2f})")

    cv2.setMouseCallback('Transform Test', test_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        vis = transformer.draw_calibration(frame, src_points, dst_points)
        cv2.imshow('Transform Test', vis)
        if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
            break

    # 保存
    save = input("\n保存配置? (y/n): ").strip().lower()
    if save == 'y':
        transformer.save_config('coordinate_transform.json')

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='坐标变换工具')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--config', type=str, default=None, help='加载配置文件')
    parser.add_argument('--interactive', action='store_true', help='交互式标定')
    args = parser.parse_args()

    if args.interactive or args.config is None:
        run_interactive_demo(args.camera)
    else:
        ct = CoordinateTransform()
        ct.load_config(args.config)
        # 测试
        test_pt = (320, 240)
        result = ct.pixel_to_world_perspective(test_pt)
        print(f"测试: 像素{test_pt} → 世界{result}")
