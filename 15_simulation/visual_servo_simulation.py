#!/usr/bin/env python3
"""
视觉伺服仿真V2 - IBVS + PBVS
==============================
对比两种经典视觉伺服方法：
- IBVS (Image-Based Visual Servoing): 基于图像特征的视觉伺服
- PBVS (Position-Based Visual Servoing): 基于位姿估计的视觉伺服

运行: python visual_servo_simulation.py
依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ========== 相机模型 ==========
class PinholeCamera:
    """针孔相机模型"""

    def __init__(self, f=800, cx=320, cy=240, width=640, height=480):
        self.f = f          # 焦距(像素)
        self.cx = cx        # 主点x
        self.cy = cy        # 主点y
        self.width = width
        self.height = height

        # 内参矩阵
        self.K = np.array([
            [f,  0, cx],
            [0,  f, cy],
            [0,  0,  1]
        ])

    def project(self, point_3d):
        """3D点投影到图像平面 (归一化坐标)"""
        if point_3d[2] <= 0.01:
            return None
        x = self.f * point_3d[0] / point_3d[2] + self.cx
        y = self.f * point_3d[1] / point_3d[2] + self.cy
        return np.array([x, y])

    def project_normalized(self, point_3d):
        """投影到归一化图像平面"""
        if point_3d[2] <= 0.01:
            return None
        x = point_3d[0] / point_3d[2]
        y = point_3d[1] / point_3d[2]
        return np.array([x, y])

    def get_interaction_matrix(self, point_3d, Z=None):
        """计算图像雅可比矩阵 (Interaction Matrix / Feature Jacobian)
        L = [ -f/Z,  0,    x/Z,   xy/f,      -(f + x²/f), y  ]
            [  0,   -f/Z,  y/Z,   f + y²/f,  -xy/f,      -x  ]
        其中 x,y 为归一化坐标, Z 为深度
        """
        if Z is None:
            Z = point_3d[2]
        x = point_3d[0] / Z
        y = point_3d[1] / Z
        f = self.f

        L = np.array([
            [-f/Z, 0,    x*f/Z,  x*y,     -(f + x**2*f), y*f],
            [0,   -f/Z,  y*f/Z,  f + y**2*f, -x*y*f,    -x*f]
        ]) / f

        return L


# ========== 机器人模型 (6DOF) ==========
class Robot6DOF:
    """简化的6自由度机器人(相机安装在末端)"""

    def __init__(self):
        self.pose = np.eye(4)  # 齐次变换矩阵 T_cw (相机到世界)
        self.position = np.zeros(3)
        self.orientation = np.zeros(3)  # roll, pitch, yaw (简化)

    def set_pose(self, T):
        """设置位姿"""
        self.pose = T.copy()
        self.position = T[:3, 3]
        # 简化: 从旋转矩阵提取欧拉角
        R = T[:3, :3]
        self.orientation = np.array([
            np.arctan2(R[2, 1], R[2, 2]),
            np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2)),
            np.arctan2(R[1, 0], R[0, 0])
        ])

    def get_pose(self):
        return self.pose.copy()

    def move(self, velocity, dt):
        """执行速度指令 (在相机坐标系下)
        velocity: [vx, vy, vz, wx, wy, wz]
        """
        v = velocity[:3]
        w = velocity[3:]

        # 平移
        self.position += self.pose[:3, :3] @ v * dt

        # 旋转 (小角度近似)
        theta = np.linalg.norm(w) * dt
        if theta > 1e-6:
            axis = w / np.linalg.norm(w)
            # Rodrigues公式
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])
            R_inc = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * K @ K
            self.pose[:3, :3] = R_inc @ self.pose[:3, :3]

        self.pose[:3, 3] = self.position

    def transform_to_camera(self, point_world):
        """世界坐标点转到相机坐标系"""
        T_cw = np.linalg.inv(self.pose)
        p_h = np.append(point_world, 1)
        p_cam = T_cw @ p_h
        return p_cam[:3]


# ========== IBVS 控制器 ==========
class IBVSController:
    """图像视觉伺服控制器"""

    def __init__(self, camera, lambda_gain=0.5):
        self.camera = camera
        self.lambda_gain = lambda_gain

    def compute_velocity(self, current_features, desired_features, points_3d_cam):
        """计算相机速度指令

        current_features: 当前图像特征 [(u1,v1), (u2,v2), ...]
        desired_features: 期望图像特征
        points_3d_cam: 对应3D点(相机坐标系)
        """
        n = len(current_features)
        L_full = np.zeros((2 * n, 6))
        error = np.zeros(2 * n)

        for i in range(n):
            # 归一化坐标
            x_cur = (current_features[i][0] - self.camera.cx) / self.camera.f
            y_cur = (current_features[i][1] - self.camera.cy) / self.camera.f
            Z = points_3d_cam[i][2]

            # 交互矩阵
            L = self.camera.get_interaction_matrix(
                np.array([x_cur * Z, y_cur * Z, Z]), Z
            )
            L_full[2*i:2*i+2, :] = L

            # 误差
            error[2*i:2*i+2] = current_features[i] - desired_features[i]

        # 伪逆求解
        L_pinv = np.linalg.pinv(L_full)
        velocity = -self.lambda_gain * L_pinv @ error

        return velocity


# ========== PBVS 控制器 ==========
class PBVSController:
    """位姿视觉伺服控制器"""

    def __init__(self, lambda_gain=0.8):
        self.lambda_gain = lambda_gain

    def compute_velocity(self, T_current, T_desired):
        """计算相机速度指令

        T_current: 当前相机位姿 (4x4齐次矩阵)
        T_desired: 期望相机位姿
        """
        # 计算相对位姿误差
        T_error = np.linalg.inv(T_current) @ T_desired

        # 平移误差
        t_error = T_error[:3, 3]

        # 旋转误差 (从旋转矩阵提取轴角)
        R_error = T_error[:3, :3]
        theta = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))
        if abs(theta) < 1e-6:
            r_error = np.zeros(3)
        else:
            r_error = theta / (2 * np.sin(theta)) * np.array([
                R_error[2, 1] - R_error[1, 2],
                R_error[0, 2] - R_error[2, 0],
                R_error[1, 0] - R_error[0, 1]
            ])

        # 组合误差
        error = np.concatenate([t_error, r_error])

        # 简单比例控制 (负号确保误差收敛)
        velocity = -self.lambda_gain * error

        return velocity


# ========== 仿真场景 ==========
def create_feature_points():
    """创建目标特征点 (平面正方形)"""
    points = np.array([
        [0.1, 0.1, 1.0],
        [-0.1, 0.1, 1.0],
        [-0.1, -0.1, 1.0],
        [0.1, -0.1, 1.0],
        [0.0, 0.0, 1.0],  # 中心点
    ])
    return points


def run_visual_servo(method='IBVS', duration=10.0, dt=0.01):
    """运行视觉伺服仿真"""
    camera = PinholeCamera()

    # 目标特征点(世界坐标)
    target_points = create_feature_points()

    # 期望位姿 (相机正对目标)
    T_desired = np.eye(4)
    T_desired[2, 3] = 1.0  # 距离目标1m

    # 初始位姿 (有偏移)
    robot = Robot6DOF()
    T_init = np.eye(4)
    T_init[0, 3] = 0.15    # x偏移
    T_init[1, 3] = 0.10    # y偏移
    T_init[2, 3] = 1.3     # 深度偏移
    # 添加旋转误差
    theta_err = 0.15  # ~8.6度
    R_err = np.array([
        [np.cos(theta_err), 0, np.sin(theta_err)],
        [0, 1, 0],
        [-np.sin(theta_err), 0, np.cos(theta_err)]
    ])
    T_init[:3, :3] = R_err
    robot.set_pose(T_init)

    if method == 'IBVS':
        controller = IBVSController(camera, lambda_gain=0.5)
    else:
        controller = PBVSController(lambda_gain=0.8)

    # 期望特征
    desired_features = []
    for p in target_points:
        p_cam = np.linalg.inv(T_desired) @ np.append(p, 1)
        feat = camera.project(p_cam[:3])
        desired_features.append(feat)

    # 仿真
    steps = int(duration / dt)
    history = {
        'time': [], 'position': [], 'orientation': [],
        'features': [], 'error_feature': [], 'error_pose': [],
        'velocity': []
    }

    for i in range(steps):
        t = i * dt

        # 当前特征
        current_features = []
        points_cam = []
        for p in target_points:
            p_cam = robot.transform_to_camera(p)
            points_cam.append(p_cam)
            feat = camera.project(p_cam)
            if feat is not None:
                current_features.append(feat)
            else:
                current_features.append(np.array([camera.cx, camera.cy]))

        # 计算控制律
        if method == 'IBVS':
            velocity = controller.compute_velocity(current_features, desired_features, points_cam)
        else:
            velocity = controller.compute_velocity(robot.get_pose(), T_desired)

        # 限幅
        velocity = np.clip(velocity, -2.0, 2.0)

        # 更新机器人
        robot.move(velocity, dt)

        # 记录
        history['time'].append(t)
        history['position'].append(robot.position.copy())
        history['orientation'].append(robot.orientation.copy())
        history['features'].append([f.copy() for f in current_features])
        history['velocity'].append(velocity.copy())

        # 特征误差
        feat_err = 0
        for cf, df in zip(current_features, desired_features):
            feat_err += np.linalg.norm(cf - df)
        history['error_feature'].append(feat_err / len(current_features))

        # 位姿误差
        T_err = np.linalg.inv(robot.get_pose()) @ T_desired
        pos_err = np.linalg.norm(T_err[:3, 3])
        history['error_pose'].append(pos_err)

        # 收敛判断
        if feat_err / len(current_features) < 0.5 and pos_err < 0.01:
            print(f"    [{method}] 收敛于 t={t:.2f}s")
            break

    return history, desired_features, target_points


# ========== 可视化 ==========
def plot_comparison(ibvs_hist, pbvs_hist, desired_features, target_points):
    """对比IBVS和PBVS"""
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('视觉伺服仿真V2 - IBVS vs PBVS 对比', fontsize=16, fontweight='bold')

    # 1. 特征误差
    ax = axes[0, 0]
    ax.plot(ibvs_hist['time'], ibvs_hist['error_feature'], 'b-', linewidth=2, label='IBVS')
    ax.plot(pbvs_hist['time'], pbvs_hist['error_feature'], 'r-', linewidth=2, label='PBVS')
    ax.set_title('图像特征误差')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('平均特征误差 (pixels)')
    ax.legend(); ax.grid(True, alpha=0.3)

    # 2. 位姿误差
    ax = axes[0, 1]
    ax.plot(ibvs_hist['time'], np.array(ibvs_hist['error_pose']) * 100, 'b-', linewidth=2, label='IBVS')
    ax.plot(pbvs_hist['time'], np.array(pbvs_hist['error_pose']) * 100, 'r-', linewidth=2, label='PBVS')
    ax.set_title('位姿误差')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('位置误差 (cm)')
    ax.legend(); ax.grid(True, alpha=0.3)

    # 3. 控制速度
    ax = axes[0, 2]
    ibvs_speed = [np.linalg.norm(v) for v in ibvs_hist['velocity']]
    pbvs_speed = [np.linalg.norm(v) for v in pbvs_hist['velocity']]
    ax.plot(ibvs_hist['time'], ibvs_speed, 'b-', linewidth=2, label='IBVS')
    ax.plot(pbvs_hist['time'], pbvs_speed, 'r-', linewidth=2, label='PBVS')
    ax.set_title('控制速度大小')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('速度')
    ax.legend(); ax.grid(True, alpha=0.3)

    # 4. 相机轨迹 (2D投影)
    ax = axes[1, 0]
    ibvs_pos = np.array(ibvs_hist['position'])
    pbvs_pos = np.array(pbvs_hist['position'])
    ax.plot(ibvs_pos[:, 0], ibvs_pos[:, 2], 'b-', linewidth=2, label='IBVS')
    ax.plot(pbvs_pos[:, 0], pbvs_pos[:, 2], 'r-', linewidth=2, label='PBVS')
    ax.plot(ibvs_pos[0, 0], ibvs_pos[0, 2], 'go', markersize=10, label='起始')
    ax.plot(0, 1.0, 'k*', markersize=15, label='目标')
    ax.set_title('相机轨迹 (X-Z平面)')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Z (m)')
    ax.legend(); ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # 5. IBVS特征轨迹
    ax = axes[1, 1]
    n_points = len(desired_features)
    colors_pts = plt.cm.tab10(np.linspace(0, 1, n_points))
    for pi in range(n_points):
        fx = [hist[pi][0] for hist in ibvs_hist['features']]
        fy = [hist[pi][1] for hist in ibvs_hist['features']]
        ax.plot(fx, fy, '-', color=colors_pts[pi], linewidth=1)
        ax.plot(fx[0], fy[0], 'o', color=colors_pts[pi], markersize=8)
        ax.plot(desired_features[pi][0], desired_features[pi][1],
                's', color=colors_pts[pi], markersize=10)
    ax.set_title('IBVS 特征轨迹 (图像平面)')
    ax.set_xlabel('u (pixels)'); ax.set_ylabel('v (pixels)')
    ax.invert_yaxis()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # 6. 收敛性能
    ax = axes[1, 2]
    metrics = ['收敛时间(s)', '最终特征误差\n(pixels)', '最终位姿误差\n(cm)', '平均速度']
    ibvs_vals = [
        ibvs_hist['time'][-1],
        ibvs_hist['error_feature'][-1],
        ibvs_hist['error_pose'][-1] * 100,
        np.mean([np.linalg.norm(v) for v in ibvs_hist['velocity']])
    ]
    pbvs_vals = [
        pbvs_hist['time'][-1],
        pbvs_hist['error_feature'][-1],
        pbvs_hist['error_pose'][-1] * 100,
        np.mean([np.linalg.norm(v) for v in pbvs_hist['velocity']])
    ]

    x_pos = np.arange(len(metrics))
    width = 0.35
    ax.bar(x_pos - width/2, ibvs_vals, width, label='IBVS', color='#2196F3')
    ax.bar(x_pos + width/2, pbvs_vals, width, label='PBVS', color='#F44336')
    ax.set_title('性能指标对比')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(metrics, fontsize=9)
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig('visual_servo_v2_comparison.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("结果已保存: visual_servo_v2_comparison.png")


# ========== 主程序 ==========
if __name__ == '__main__':
    print("=" * 60)
    print("视觉伺服仿真V2 - IBVS + PBVS")
    print("=" * 60)

    print("\n运行 IBVS 仿真...")
    ibvs_hist, desired_features, target_points = run_visual_servo('IBVS', duration=15.0)
    print(f"  最终特征误差: {ibvs_hist['error_feature'][-1]:.2f} pixels")
    print(f"  最终位姿误差: {ibvs_hist['error_pose'][-1]*100:.2f} cm")

    print("\n运行 PBVS 仿真...")
    pbvs_hist, _, _ = run_visual_servo('PBVS', duration=15.0)
    print(f"  最终特征误差: {pbvs_hist['error_feature'][-1]:.2f} pixels")
    print(f"  最终位姿误差: {pbvs_hist['error_pose'][-1]*100:.2f} cm")

    print("\n绘制对比图...")
    plot_comparison(ibvs_hist, pbvs_hist, desired_features, target_points)

    print("\n仿真完成！")
