#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
差速小车运动学仿真 - 路径跟踪 (Pure Pursuit / Stanley)
=========================================================
数学模型:
  v = (vL + vR) / 2
  ω = (vR - vL) / d   (d为轮距)
  x' = v*cos(θ)
  y' = v*sin(θ)
  θ' = ω

控制策略:
  1. Pure Pursuit (纯追踪)
  2. Stanley Controller
  3. PID线跟踪

仿真内容:
  - 参考路径生成 (直线、圆弧、S形)
  - 两种控制器路径跟踪对比
  - 跟踪误差分析
  - 不同速度下的性能
  - 参数影响分析
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.patches import FancyArrowPatch
import warnings
warnings.filterwarnings('ignore')

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


# ==============================================================================
# 差速小车模型
# ==============================================================================
class DifferentialDriveRobot:
    """差速驱动小车"""
    def __init__(self, wheel_base=0.15, dt=0.02):
        """
        wheel_base: 轮距 d [m]
        dt: 控制周期 [s]
        """
        self.d = wheel_base
        self.dt = dt

        # 状态: [x, y, theta]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 速度限制
        self.v_max = 1.0     # m/s
        self.omega_max = 3.0  # rad/s

    def reset(self, x0=0.0, y0=0.0, theta0=0.0):
        self.x = x0
        self.y = y0
        self.theta = theta0

    def update(self, v, omega):
        """运动学更新"""
        v = np.clip(v, -self.v_max, self.v_max)
        omega = np.clip(omega, -self.omega_max, self.omega_max)

        self.x += v * np.cos(self.theta) * self.dt
        self.y += v * np.sin(self.theta) * self.dt
        self.theta += omega * self.dt

        # 归一化角度到 [-pi, pi]
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

    def get_state(self):
        return np.array([self.x, self.y, self.theta])


# ==============================================================================
# 参考路径
# ==============================================================================
class ReferencePath:
    """参考路径生成器"""
    def __init__(self):
        self.points = None
        self.tangent_angles = None

    def generate_straight(self, length=5.0, n_points=500):
        """直线路径"""
        t = np.linspace(0, length, n_points)
        self.points = np.column_stack([t, np.zeros(n_points)])
        self.tangent_angles = np.zeros(n_points)
        return self.points, self.tangent_angles

    def generate_circle(self, radius=3.0, n_points=500):
        """圆形路径"""
        theta = np.linspace(0, 2 * np.pi, n_points)
        self.points = np.column_stack([radius * np.cos(theta), radius * np.sin(theta)])
        self.tangent_angles = theta + np.pi / 2  # 切线方向
        return self.points, self.tangent_angles

    def generate_s_curve(self, length=10.0, amplitude=3.0, n_points=500):
        """S形路径"""
        t = np.linspace(0, length, n_points)
        x = t
        y = amplitude * np.sin(2 * np.pi * t / length)
        self.points = np.column_stack([x, y])

        # 计算切线角度
        dx = np.gradient(x)
        dy = np.gradient(y)
        self.tangent_angles = np.arctan2(dy, dx)
        return self.points, self.tangent_angles

    def generate_l_shape(self, n_points=500):
        """L形路径"""
        seg1_len = 3.0
        seg2_len = 3.0
        n_seg = n_points // 2

        t1 = np.linspace(0, seg1_len, n_seg)
        t2 = np.linspace(0, seg2_len, n_points - n_seg)

        x = np.concatenate([t1, np.full(n_points - n_seg, seg1_len)])
        y = np.concatenate([np.zeros(n_seg), t2])

        self.points = np.column_stack([x, y])

        dx = np.gradient(x)
        dy = np.gradient(y)
        self.tangent_angles = np.arctan2(dy, dx)
        return self.points, self.tangent_angles

    def get_closest_point(self, robot_pos):
        """获取路径上最近点及索引"""
        distances = np.linalg.norm(self.points - robot_pos[:2], axis=1)
        idx = np.argmin(distances)
        return self.points[idx], idx, self.tangent_angles[idx]


# ==============================================================================
# Pure Pursuit 控制器
# ==============================================================================
class PurePursuitController:
    """纯追踪控制器"""
    def __init__(self, lookahead_distance=0.5, k_velocity=0.5):
        """
        lookahead_distance: 前视距离 [m]
        k_velocity: 速度增益
        """
        self.Ld = lookahead_distance
        self.k_v = k_velocity
        self.path = None

    def set_path(self, path):
        self.path = path

    def compute(self, robot_state, v_nominal=0.5):
        """计算控制量 (v, omega)"""
        x, y, theta = robot_state

        # 找到目标点 (前视距离)
        if self.path is None:
            return v_nominal, 0.0

        distances = np.linalg.norm(self.path.points - np.array([x, y]), axis=1)

        # 找到距离>=前视距离的第一个点
        candidates = np.where(distances >= self.Ld)[0]
        if len(candidates) == 0:
            target_idx = len(self.path.points) - 1
        else:
            target_idx = candidates[0]

        target = self.path.points[target_idx]

        # 计算到目标点的距离
        dx = target[0] - x
        dy = target[1] - y
        dist = np.sqrt(dx**2 + dy**2)

        if dist < 0.01:
            return v_nominal, 0.0

        # 计算目标点在车体坐标系中的角度
        angle_to_target = np.arctan2(dy, dx)
        alpha = angle_to_target - theta
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        # Pure Pursuit公式: omega = v * 2 * sin(alpha) / Ld
        curvature = 2.0 * np.sin(alpha) / max(dist, 0.01)
        omega = v_nominal * curvature

        # 速度调整 (转弯时减速)
        v = v_nominal * np.cos(alpha)
        v = max(v, 0.1)  # 最小速度

        return v, omega


# ==============================================================================
# Stanley 控制器
# ==============================================================================
class StanleyController:
    """Stanley控制器 (前轮转向控制)"""
    def __init__(self, k_cross=1.0, k_heading=2.0):
        """
        k_cross: 横向误差增益
        k_heading: 航向误差增益
        """
        self.k_cross = k_cross
        self.k_heading = k_heading
        self.path = None

    def set_path(self, path):
        self.path = path

    def compute(self, robot_state, v_nominal=0.5):
        """计算控制量"""
        x, y, theta = robot_state

        if self.path is None:
            return v_nominal, 0.0

        # 找到最近点
        closest, closest_idx, closest_angle = self.path.get_closest_point(robot_state)

        # 横向误差 (有符号)
        dx = x - closest[0]
        dy = y - closest[1]

        # 路径法向量
        normal = np.array([-np.sin(closest_angle), np.cos(closest_angle)])
        cross_error = dx * normal[0] + dy * normal[1]

        # 航向误差
        heading_error = theta - closest_angle
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        # Stanley控制律
        v = v_nominal

        # 横向误差补偿 (带饱和)
        cross_term = np.arctan2(self.k_cross * cross_error, max(v, 0.1))

        # 总转向角
        delta = -heading_error - cross_term

        # 转换为角速度 (简化)
        omega = delta * v / 0.1  # 近似

        return v, omega


# ==============================================================================
# PID 线跟踪控制器
# ==============================================================================
class PIDLineFollower:
    """PID线跟踪控制器"""
    def __init__(self, Kp=2.0, Ki=0.1, Kd=0.5):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.path = None

    def set_path(self, path):
        self.path = path

    def compute(self, robot_state, v_nominal=0.5, dt=0.02):
        x, y, theta = robot_state

        if self.path is None:
            return v_nominal, 0.0

        closest, closest_idx, closest_angle = self.path.get_closest_point(robot_state)

        # 横向误差
        dx = x - closest[0]
        dy = y - closest[1]
        cross_error = dx * np.cos(closest_angle) + dy * np.sin(closest_angle)

        # PID
        self.integral += cross_error * dt
        derivative = (cross_error - self.prev_error) / dt
        self.prev_error = cross_error

        omega = -(self.Kp * cross_error + self.Ki * self.integral + self.Kd * derivative)

        v = v_nominal * (1.0 - 0.3 * abs(cross_error))
        v = max(v, 0.1)

        return v, omega


# ==============================================================================
# 性能指标
# ==============================================================================
class TrackingMetrics:
    @staticmethod
    def compute(path_points, robot轨迹):
        """计算路径跟踪性能指标"""
        errors = []
        for pos in robot轨迹:
            dist = np.linalg.norm(path_points - pos[:2], axis=1)
            errors.append(np.min(dist))

        errors = np.array(errors)
        return {
            'mean_error': np.mean(errors),
            'max_error': np.max(errors),
            'rms_error': np.sqrt(np.mean(errors**2)),
            'final_error': errors[-1],
        }


# ==============================================================================
# 仿真引擎
# ==============================================================================
class PathTrackingSimulation:
    """路径跟踪仿真引擎"""
    def __init__(self, dt=0.02):
        self.dt = dt
        self.robot = DifferentialDriveRobot(wheel_base=0.15, dt=dt)

    def simulate_tracking(self, controller, path, v_nominal=0.5, T=10.0,
                          noise_std=0.0, delay_steps=0):
        """仿真路径跟踪"""
        n_steps = int(T / self.dt)
        self.robot.reset(x0=path.points[0, 0],
                         y0=path.points[0, 1],
                         theta0=path.tangent_angles[0])

        controller.set_path(path)

        states = []
        controls = []
        errors = []
        time_hist = []

        for i in range(n_steps):
            state = self.robot.get_state()
            states.append(state.copy())

            # 传感器噪声
            noisy_state = state.copy()
            if noise_std > 0:
                noisy_state[:2] += np.random.randn(2) * noise_std
                noisy_state[2] += np.random.randn() * noise_std * 0.1

            # 控制计算
            v, omega = controller.compute(noisy_state, v_nominal)
            controls.append([v, omega])

            # 计算跟踪误差
            closest, _, _ = path.get_closest_point(state)
            error = np.linalg.norm(state[:2] - closest)
            errors.append(error)

            time_hist.append(i * self.dt)

            # 更新机器人
            self.robot.update(v, omega)

        return (np.array(time_hist), np.array(states),
                np.array(controls), np.array(errors))


# ==============================================================================
# 主仿真与绘图
# ==============================================================================
def run_diff_drive_simulation():
    """运行差速小车路径跟踪仿真"""
    print("=" * 70)
    print("差速小车路径跟踪仿真 - Pure Pursuit / Stanley / PID")
    print("=" * 70)

    sim = PathTrackingSimulation(dt=0.02)

    # === 路径定义 ===
    print("\n[1/3] 生成参考路径...")
    paths = {
        '直线': ReferencePath(),
        '圆形': ReferencePath(),
        'S形': ReferencePath(),
    }
    paths['直线'].generate_straight(length=5.0)
    paths['圆形'].generate_circle(radius=2.0)
    paths['S形'].generate_s_curve(length=10.0, amplitude=2.0)

    # === 控制器 ===
    controllers = {
        'Pure Pursuit': PurePursuitController(lookahead_distance=0.5),
        'Stanley': StanleyController(k_cross=1.0, k_heading=2.0),
        'PID': PIDLineFollower(Kp=3.0, Ki=0.1, Kd=0.8),
    }

    # === 仿真 ===
    print("\n[2/3] 执行路径跟踪仿真...")
    all_results = {}
    v_nominal = 0.5

    for path_name, path in paths.items():
        all_results[path_name] = {}
        for ctrl_name, ctrl in controllers.items():
            t, states, controls, errors = sim.simulate_tracking(
                ctrl, path, v_nominal=v_nominal, T=8.0
            )
            metrics = TrackingMetrics.compute(path.points, states)
            all_results[path_name][ctrl_name] = {
                't': t, 'states': states, 'controls': controls,
                'errors': errors, 'metrics': metrics
            }
            print(f"  {path_name} + {ctrl_name}: "
                  f"平均误差={metrics['mean_error']:.4f}m, "
                  f"最大误差={metrics['max_error']:.4f}m")

    # === 速度影响分析 ===
    print("\n  速度影响分析...")
    speed_results = {}
    for v in [0.2, 0.5, 0.8, 1.0]:
        speed_results[v] = {}
        for ctrl_name, ctrl in controllers.items():
            t, states, controls, errors = sim.simulate_tracking(
                ctrl, paths['S形'], v_nominal=v, T=8.0
            )
            metrics = TrackingMetrics.compute(paths['S形'].points, states)
            speed_results[v][ctrl_name] = metrics

    # === 绘图 ===
    print("\n[3/3] 生成图表...")

    # 图1: S形路径跟踪对比
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('差速小车路径跟踪仿真', fontsize=14, fontweight='bold')

    # 1.1 路径跟踪轨迹
    ax = axes[0, 0]
    path_s = paths['S形']
    ax.plot(path_s.points[:, 0], path_s.points[:, 1], 'k--', linewidth=2, label='参考路径')
    for ctrl_name in ['Pure Pursuit', 'Stanley', 'PID']:
        states = all_results['S形'][ctrl_name]['states']
        ax.plot(states[:, 0], states[:, 1], linewidth=1.5, label=ctrl_name)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('S形路径跟踪轨迹')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # 1.2 跟踪误差
    ax = axes[0, 1]
    for ctrl_name in ['Pure Pursuit', 'Stanley', 'PID']:
        t = all_results['S形'][ctrl_name]['t']
        errors = all_results['S形'][ctrl_name]['errors']
        ax.plot(t, errors, linewidth=1.2, label=ctrl_name)
    ax.set_xlabel('时间 [s]')
    ax.set_ylabel('跟踪误差 [m]')
    ax.set_title('跟踪误差随时间变化')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 1.3 圆形路径跟踪
    ax = axes[1, 0]
    path_c = paths['圆形']
    ax.plot(path_c.points[:, 0], path_c.points[:, 1], 'k--', linewidth=2, label='参考路径')
    for ctrl_name in ['Pure Pursuit', 'Stanley', 'PID']:
        states = all_results['圆形'][ctrl_name]['states']
        ax.plot(states[:, 0], states[:, 1], linewidth=1.5, label=ctrl_name)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('圆形路径跟踪轨迹')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # 1.4 速度对误差的影响
    ax = axes[1, 1]
    speeds = sorted(speed_results.keys())
    for ctrl_name in ['Pure Pursuit', 'Stanley', 'PID']:
        mean_errors = [speed_results[v][ctrl_name]['mean_error'] for v in speeds]
        ax.plot(speeds, mean_errors, 'o-', linewidth=1.5, label=ctrl_name)
    ax.set_xlabel('目标速度 [m/s]')
    ax.set_ylabel('平均跟踪误差 [m]')
    ax.set_title('速度对跟踪精度的影响')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'diff_drive_tracking.png'),
                dpi=150, bbox_inches='tight')
    print("  图1已保存: diff_drive_tracking.png")

    # 图2: 性能指标对比
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axis('off')

    headers = ['路径', '控制器', '平均误差[m]', '最大误差[m]', 'RMS误差[m]', '末端误差[m]']
    table_data = []
    for path_name in paths:
        for ctrl_name in controllers:
            m = all_results[path_name][ctrl_name]['metrics']
            table_data.append([
                path_name, ctrl_name,
                f'{m["mean_error"]:.4f}',
                f'{m["max_error"]:.4f}',
                f'{m["rms_error"]:.4f}',
                f'{m["final_error"]:.4f}',
            ])

    table = ax.table(cellText=table_data, colLabels=headers,
                     cellLoc='center', loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.2, 1.8)
    ax.set_title('路径跟踪性能指标对比', pad=20, fontsize=14)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'diff_drive_metrics.png'),
                dpi=150, bbox_inches='tight')
    print("  图2已保存: diff_drive_metrics.png")

    # === 汇总 ===
    print("\n" + "=" * 70)
    print("差速小车仿真完成!")
    print("=" * 70)
    for path_name in paths:
        print(f"\n{path_name}路径:")
        for ctrl_name in controllers:
            m = all_results[path_name][ctrl_name]['metrics']
            print(f"  {ctrl_name}: 平均误差={m['mean_error']:.4f}m, "
                  f"最大误差={m['max_error']:.4f}m")

    return all_results


if __name__ == '__main__':
    results = run_diff_drive_simulation()
    plt.close('all')
