"""
轻量级AR模块 - ArUco标记检测 + 3D渲染 + 虚实叠加
适用于：AR交互、姿态估计、电赛AR题等场景
依赖：pip install opencv-python opencv-contrib-python numpy
"""
import cv2
import numpy as np


class ARLite:
    """轻量级增强现实引擎"""

    def __init__(self, marker_size=0.05, camera_matrix=None, dist_coeffs=None):
        """
        初始化AR引擎
        Args:
            marker_size: ArUco标记物理尺寸（米）
            camera_matrix: 相机内参矩阵 3x3
            dist_coeffs: 畸变系数
        """
        self.marker_size = marker_size

        # ArUco字典：4x4_50 适合小型标记
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # 相机参数（默认值，实际应标定获取）
        if camera_matrix is not None:
            self.camera_matrix = camera_matrix
        else:
            # 默认内参（需根据实际相机调整）
            self.camera_matrix = np.array([
                [600, 0, 320],
                [0, 600, 240],
                [0, 0, 1]
            ], dtype=np.float32)

        if dist_coeffs is not None:
            self.dist_coeffs = dist_coeffs
        else:
            self.dist_coeffs = np.zeros(5, dtype=np.float32)

    def calibrate_camera(self, images, board_size=(9, 6), square_size=0.025):
        """
        棋盘格相机标定
        Args:
            images: 棋盘格图像列表
            board_size: 棋盘格内角点数（列, 行）
            square_size: 方格物理尺寸（米）
        Returns:
            camera_matrix, dist_coeffs, reproj_error
        """
        obj_points = []  # 3D世界坐标
        img_points = []  # 2D图像坐标

        # 生成棋盘格3D坐标
        objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) * square_size

        gray = None
        for img in images:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
            ret, corners = cv2.findChessboardCorners(gray, board_size, None)
            if ret:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                obj_points.append(objp)
                img_points.append(corners_refined)

        if len(obj_points) > 0 and gray is not None:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                obj_points, img_points, gray.shape[::-1], None, None
            )
            self.camera_matrix = mtx
            self.dist_coeffs = dist
            print(f"标定完成，重投影误差: {ret:.4f}")
            print(f"相机矩阵:\n{mtx}")
            return mtx, dist, ret
        else:
            print("未找到足够的棋盘格角点")
            return None, None, -1

    def detect_markers(self, image):
        """
        检测ArUco标记
        Args:
            image: BGR图像
        Returns:
            corners: 角点列表, ids: 标记ID列表, rejected: 被拒绝的候选
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        corners, ids, rejected = self.detector.detectMarkers(gray)
        return corners, ids, rejected

    def estimate_pose(self, corners, ids):
        """
        估计标记的6DoF姿态
        Args:
            corners: 检测到的角点
            ids: 标记ID
        Returns:
            poses: {id: {'rvec': ..., 'tvec': ..., 'rmat': ...}, ...}
        """
        if ids is None or len(ids) == 0:
            return {}

        # 单个标记的3D坐标（以标记中心为原点）
        half = self.marker_size / 2
        obj_points = np.array([
            [-half,  half, 0],  # 左上
            [ half,  half, 0],  # 右上
            [ half, -half, 0],  # 右下
            [-half, -half, 0],  # 左下
        ], dtype=np.float32)

        poses = {}
        for i, marker_id in enumerate(ids.flatten()):
            ret, rvec, tvec = cv2.solvePnP(
                obj_points, corners[i],
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if ret:
                rmat, _ = cv2.Rodrigues(rvec)
                poses[marker_id] = {
                    'rvec': rvec,
                    'tvec': tvec,
                    'rmat': rmat
                }
        return poses

    def draw_markers(self, image, corners, ids, draw_axes=True):
        """
        绘制检测到的标记和坐标轴
        Args:
            image: BGR图像
            corners: 角点
            ids: ID
            draw_axes: 是否绘制3D坐标轴
        Returns:
            标注后的图像
        """
        vis = image.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

            if draw_axes:
                poses = self.estimate_pose(corners, ids)
                for marker_id, pose in poses.items():
                    cv2.drawFrameAxes(
                        vis, self.camera_matrix, self.dist_coeffs,
                        pose['rvec'], pose['tvec'],
                        self.marker_size * 0.5, 2
                    )
        return vis

    def create_cube_vertices(self, size=0.03, offset=(0, 0, 0)):
        """
        生成立方体顶点（标记坐标系）
        Args:
            size: 立方体边长（米）
            offset: 偏移量 (x, y, z)
        Returns:
            8个顶点坐标 + 6个面的索引
        """
        s = size / 2
        ox, oy, oz = offset
        vertices = np.float32([
            [-s + ox, -s + oy, -s + oz],  # 0: 左下后
            [ s + ox, -s + oy, -s + oz],  # 1: 右下后
            [ s + ox,  s + oy, -s + oz],  # 2: 右上后
            [-s + ox,  s + oy, -s + oz],  # 3: 左上后
            [-s + ox, -s + oy,  s + oz],  # 4: 左下前
            [ s + ox, -s + oy,  s + oz],  # 5: 右下前
            [ s + ox,  s + oy,  s + oz],  # 6: 右上前
            [-s + ox,  s + oy,  s + oz],  # 7: 左上前
        ])
        # 6个面（每面4个顶点索引）
        faces = [
            [4, 5, 6, 7],  # 前
            [0, 1, 5, 4],  # 下
            [1, 2, 6, 5],  # 右
            [2, 3, 7, 6],  # 上
            [3, 0, 4, 7],  # 左
            [0, 1, 2, 3],  # 后
        ]
        return vertices, faces

    def project_3d_to_2d(self, points_3d, rvec, tvec):
        """
        将3D点投影到2D图像平面
        Args:
            points_3d: Nx3 3D点
            rvec: 旋转向量
            tvec: 平移向量
        Returns:
            points_2d: Nx2 2D像素坐标
        """
        points_2d, _ = cv2.projectPoints(
            points_3d, rvec, tvec,
            self.camera_matrix, self.dist_coeffs
        )
        return points_2d.reshape(-1, 2).astype(np.int32)

    def render_cube(self, image, pose, size=0.03, color=(0, 200, 0), alpha=0.6):
        """
        在标记位置渲染半透明立方体
        Args:
            image: BGR图像
            pose: {'rvec': ..., 'tvec': ..., 'rmat': ...}
            size: 立方体大小
            color: 颜色 (B,G,R)
            alpha: 透明度
        Returns:
            渲染后的图像
        """
        vis = image.copy()
        overlay = image.copy()

        rvec, tvec = pose['rvec'], pose['tvec']
        vertices, faces = self.create_cube_vertices(size, offset=(0, 0, size / 2))
        pts_2d = self.project_3d_to_2d(vertices, rvec, tvec)

        # 画家算法：按z深度排序面（远处先画）
        center_3d = np.mean(vertices, axis=0)
        face_centers_z = []
        for face in faces:
            face_pts_3d = vertices[face]
            face_center = np.mean(face_pts_3d, axis=0)
            # 变换到相机坐标系算深度
            rmat, _ = cv2.Rodrigues(rvec)
            cam_pos = rmat @ face_center.reshape(3, 1) + tvec
            face_centers_z.append(cam_pos[2, 0])

        sorted_faces = sorted(zip(faces, face_centers_z), key=lambda x: -x[1])

        colors = [
            (0, 200, 0),    # 前-绿
            (0, 150, 200),  # 下-橙
            (200, 100, 0),  # 右-蓝
            (0, 200, 200),  # 上-黄
            (200, 0, 200),  # 左-紫
            (100, 100, 100), # 后-灰
        ]

        for idx, (face, _) in enumerate(sorted_faces):
            face_pts = pts_2d[face]
            cv2.fillConvexPoly(overlay, face_pts, colors[idx % len(colors)])
            cv2.polylines(vis, [face_pts], True, (0, 0, 0), 1)

        # 混合半透明效果
        cv2.addWeighted(overlay, alpha, vis, 1 - alpha, 0, vis)
        return vis

    def render_pyramid(self, image, pose, base_size=0.04, height=0.05, color=(200, 0, 0)):
        """渲染金字塔形状"""
        vis = image.copy()
        overlay = image.copy()

        rvec, tvec = pose['rvec'], pose['tvec']
        half = base_size / 2
        # 金字塔顶点：底面4个 + 顶点1个
        vertices = np.float32([
            [-half, -half, 0],
            [ half, -half, 0],
            [ half,  half, 0],
            [-half,  half, 0],
            [0, 0, height],  # 顶点
        ])
        faces = [
            [0, 1, 4],  # 前
            [1, 2, 4],  # 右
            [2, 3, 4],  # 后
            [3, 0, 4],  # 左
            [0, 1, 2, 3],  # 底
        ]

        pts_2d = self.project_3d_to_2d(vertices, rvec, tvec)

        pyramid_colors = [
            (0, 0, 200), (0, 180, 0), (200, 0, 0), (0, 200, 200), (80, 80, 80)
        ]
        for idx, face in enumerate(faces):
            face_pts = pts_2d[face]
            cv2.fillConvexPoly(overlay, face_pts, pyramid_colors[idx % len(pyramid_colors)])

        cv2.addWeighted(overlay, 0.6, vis, 0.4, 0, vis)

        # 画边线
        for face in faces:
            face_pts = pts_2d[face]
            cv2.polylines(vis, [face_pts], True, (0, 0, 0), 1)

        return vis

    def render_text_3d(self, image, pose, text, font_scale=0.02, color=(255, 255, 255)):
        """
        在3D空间中渲染文字（浮在标记上方）
        Args:
            image: BGR图像
            pose: 姿态
            text: 文字内容
            font_scale: 字体大小（相对于标记尺寸）
            color: 颜色
        Returns:
            渲染后的图像
        """
        vis = image.copy()
        rvec, tvec = pose['rvec'], pose['tvec']

        # 文字位置（标记上方）
        text_pos_3d = np.float32([
            [-self.marker_size * 0.4, self.marker_size * 0.4, self.marker_size * 0.5],
            [ self.marker_size * 0.4, self.marker_size * 0.4, self.marker_size * 0.5],
        ])
        text_pts_2d = self.project_3d_to_2d(text_pos_3d, rvec, tvec)

        org = tuple(text_pts_2d[0])
        # 计算字体大小（基于两点距离）
        dist = np.linalg.norm(text_pts_2d[1] - text_pts_2d[0])
        scale = max(0.3, dist / (len(text) * 20))

        cv2.putText(vis, text, org, cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, max(1, int(scale * 2)), cv2.LINE_AA)
        return vis

    def get_marker_info(self, image):
        """
        获取完整的标记检测信息
        Args:
            image: BGR图像
        Returns:
            info: {'count': int, 'ids': list, 'poses': dict, 'annotated': image}
        """
        corners, ids, rejected = self.detect_markers(image)
        vis = self.draw_markers(image, corners, ids)

        poses = {}
        if ids is not None and len(ids) > 0:
            poses = self.estimate_pose(corners, ids)
            # 添加距离信息
            for mid, pose in poses.items():
                dist = np.linalg.norm(pose['tvec'])
                pose['distance'] = float(dist)
                cv2.putText(vis, f'ID:{mid} D:{dist:.2f}m', (10, 30 + mid * 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return {
            'count': len(ids) if ids is not None else 0,
            'ids': ids.flatten().tolist() if ids is not None else [],
            'poses': poses,
            'annotated': vis
        }


def generate_aruco_marker(marker_id=0, size_pixels=200, dict_type=cv2.aruco.DICT_4X4_50):
    """
    生成ArUco标记图像（用于打印）
    Args:
        marker_id: 标记ID（0-49 for 4x4_50）
        size_pixels: 输出图像像素大小
        dict_type: 字典类型
    Returns:
        marker_image: 标记图像（黑白）
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
    marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size_pixels)
    # 添加白色边框
    border = size_pixels // 5
    bordered = np.ones((size_pixels + 2 * border, size_pixels + 2 * border), dtype=np.uint8) * 255
    bordered[border:border + size_pixels, border:border + size_pixels] = marker
    return bordered


# ============== 使用示例 ==============
if __name__ == '__main__':
    print("=== 轻量级AR示例 ===")

    # 步骤1：生成ArUco标记用于打印
    print("\n1. 生成ArUco标记...")
    for mid in range(4):
        marker = generate_aruco_marker(marker_id=mid, size_pixels=300)
        fname = f'aruco_marker_{mid}.png'
        cv2.imwrite(fname, marker)
        print(f"   已生成: {fname} (ID={mid})")

    # 步骤2：创建AR引擎
    ar = ARLite(marker_size=0.05)  # 5cm标记

    # 步骤3：创建模拟场景（标记 + 背景）
    print("\n2. 创建模拟AR场景...")

    # 读取生成的标记
    marker_img = cv2.imread('aruco_marker_0.png')
    if marker_img is None:
        marker_img = generate_aruco_marker(0, 300)
        marker_img = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

    # 将标记放入大图中（模拟相机视角）
    scene = np.ones((480, 640, 3), dtype=np.uint8) * 200
    # 缩放标记
    marker_resized = cv2.resize(marker_img, (150, 150))
    scene[150:300, 240:390] = marker_resized

    # 检测标记
    info = ar.get_marker_info(scene)
    print(f"   检测到 {info['count']} 个标记")
    print(f"   标记ID: {info['ids']}")

    # 如果检测到标记，叠加3D物体
    result = info['annotated']
    if info['poses']:
        for mid, pose in info['poses'].items():
            if mid == 0:
                result = ar.render_cube(result, pose, size=0.04, alpha=0.7)
                result = ar.render_text_3d(result, pose, "Cube")
            elif mid == 1:
                result = ar.render_pyramid(result, pose)
            else:
                result = ar.render_cube(result, pose, size=0.03)

    cv2.imwrite('ar_result.jpg', result)
    print("   AR结果已保存到 ar_result.jpg")

    # 步骤4：实时AR演示
    print("\n3. 实时AR演示（按 'q' 退出）")
    print("   提示：打印 aruco_marker_0.png 并对准摄像头")

    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            info = ar.get_marker_info(frame)
            result = info['annotated']

            for mid, pose in info['poses'].items():
                if mid % 2 == 0:
                    result = ar.render_cube(result, pose, size=0.04, alpha=0.6)
                    result = ar.render_text_3d(result, pose, f"AR-{mid}")
                else:
                    result = ar.render_pyramid(result, pose, base_size=0.04, height=0.05)

            cv2.putText(result, f'Markers: {info["count"]}', (10, 470),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow('AR Lite', result)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                cv2.imwrite('ar_capture.jpg', result)
                print("   截图已保存")

        cap.release()
        cv2.destroyAllWindows()
    else:
        print("   无法打开摄像头，请打印标记后手动测试")

    print("\n=== 使用说明 ===")
    print("1. 打印生成的 aruco_marker_*.png 文件")
    print("2. 将标记放在摄像头前")
    print("3. 程序会自动检测标记并叠加3D虚拟物体")
    print("4. 可通过修改 marker_size 参数适配不同大小的标记")
