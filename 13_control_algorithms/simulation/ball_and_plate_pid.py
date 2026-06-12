#!/usr/bin/env python3
"""
滚球平衡系统 PID 仿真 - 双轴独立控制

系统描述:
  - 钢球在倾斜平板上滚动
  - 两个舵机分别控制平板 X/Y 方向倾斜角
  - 目标: 将球控制到平板中心

物理模型:
  球在倾斜平面上的运动(小角度近似):
    x_ddot = (5/7) * g * sin(θ_x) ≈ (5/7) * g * θ_x
    y_ddot = (5/7) * g * sin(θ_y) ≈ (5/7) * g * θ_y

  其中 θ_x, θ_y 是平板两个方向的倾斜角(作为控制输入)

控制结构:
  - 外环: 位置PID -> 期望倾斜角
  - 内环: 角度PID -> 舵机PWM (可选,本仿真假设舵机理想)
  - X/Y 轴独立控制(解耦)

依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class PIDController:
    """PID 控制器(带抗积分饱和)"""

    def __init__(self, kp, ki, kd, output_min, output_max, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.dt = dt

        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, setpoint, measurement):
        error = setpoint - measurement

        p = self.kp * error

        # 条件积分抗饱和
        self.integral += error * self.dt
        i = self.ki * self.integral

        d = self.kd * (error - self.prev_error) / self.dt
        self.prev_error = error

        output = p + i + d

        # 输出限幅
        if output > self.output_max:
            output = self.output_max
            # 反向积分退饱和
            self.integral -= error * self.dt
        elif output < self.output_min:
            output = self.output_min
            self.integral -= error * self.dt

        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class BallAndPlate:
    """滚球-平板系统模型"""

    def __init__(self, g=9.81, plate_size=0.3):
        self.g = g
        self.plate_size = plate_size  # 平板半径/边长 (m)

        # 球的状态: [x, x_dot, y, y_dot]
        self.state = np.zeros(4)

        # 物理参数
        self.r_ball = 0.01      # 球半径 (m)
        self.inertia_factor = 5.0 / 7.0  # 纯滚动的惯性因子

    def reset(self, x0=None):
        if x0 is None:
            self.state = np.zeros(4)
        else:
            self.state = np.array(x0, dtype=float)

    def step(self, theta_x, theta_y, dt):
        """
        仿真一步

        Parameters:
            theta_x: X方向倾斜角 (rad)
            theta_y: Y方向倾斜角 (rad)
            dt: 时间步长 (s)
        """
        x, x_dot, y, y_dot = self.state

        # 加速度(小角度近似)
        x_ddot = self.inertia_factor * self.g * theta_x
        y_ddot = self.inertia_factor * self.g * theta_y

        # 前向欧拉积分
        x_dot_new = x_dot + x_ddot * dt
        y_dot_new = y_dot + y_ddot * dt
        x_new = x + x_dot_new * dt
        y_new = y + y_dot_new * dt

        # 边界检测(球掉出平板)
        half = self.plate_size / 2
        if abs(x_new) > half:
            x_new = np.clip(x_new, -half, half)
            x_dot_new = 0  # 碰撞停止
        if abs(y_new) > half:
            y_new = np.clip(y_new, -half, half)
            y_dot_new = 0

        self.state = np.array([x_new, x_dot_new, y_new, y_dot_new])
        return self.state.copy()

    def get_position(self):
        return self.state[0], self.state[2]

    def get_velocity(self):
        return self.state[1], self.state[3]


def generate_trajectory(t, mode='step'):
    """生成目标轨迹"""
    if mode == 'step':
        # 阶跃: 从中心跳到 (0.1, 0.05) m
        target_x = np.where(t > 1.0, 0.1, 0.0)
        target_y = np.where(t > 2.0, 0.05, 0.0)
    elif mode == 'circle':
        # 圆形轨迹
        omega = 0.5 * 2 * np.pi  # 0.5 Hz
        r = 0.08  # 半径 8cm
        target_x = r * np.cos(omega * t)
        target_y = r * np.sin(omega * t)
    elif mode == 'figure8':
        # 8字形轨迹
        omega = 0.3 * 2 * np.pi
        target_x = 0.08 * np.sin(omega * t)
        target_y = 0.08 * np.sin(omega * t) * np.cos(omega * t)
    else:
        target_x = np.zeros_like(t)
        target_y = np.zeros_like(t)

    return target_x, target_y


def run_simulation(pid_x_params, pid_y_params, x0, target_mode, duration, dt=0.001):
    """运行完整仿真"""
    t = np.arange(0, duration, dt)
    n = len(t)

    # 目标轨迹
    ref_x, ref_y = generate_trajectory(t, target_mode)

    # 系统和控制器
    system = BallAndPlate()
    system.reset(x0)

    pid_x = PIDController(*pid_x_params, dt=dt)
    pid_y = PIDController(*pid_y_params, dt=dt)

    # 记录
    pos_x = np.zeros(n)
    pos_y = np.zeros(n)
    ctrl_x = np.zeros(n)
    ctrl_y = np.zeros(n)

    max_tilt = 15 * np.pi / 180  # 最大倾斜角 15°

    for i in range(n):
        bx, by = system.get_position()

        # PID 计算 -> 倾斜角指令
        theta_x_cmd = pid_x.update(ref_x[i], bx)
        theta_y_cmd = pid_y.update(ref_y[i], by)

        # 限幅
        theta_x_cmd = np.clip(theta_x_cmd, -max_tilt, max_tilt)
        theta_y_cmd = np.clip(theta_y_cmd, -max_tilt, max_tilt)

        # 仿真一步
        system.step(theta_x_cmd, theta_y_cmd, dt)

        pos_x[i], pos_y[i] = bx, by
        ctrl_x[i] = theta_x_cmd
        ctrl_y[i] = theta_y_cmd

    return t, pos_x, pos_y, ctrl_x, ctrl_y, ref_x, ref_y


def main():
    print("=" * 60)
    print("  滚球平衡系统 PID 双轴控制仿真")
    print("=" * 60)

    dt = 0.001

    # PID 参数整定
    # 位置环: 需要适度的超调抑制和快速响应
    # Kp: 决定倾斜角大小  Ki: 消除稳态误差  Kd: 阻尼
    pid_params = (8.0, 2.0, 5.0)  # kp, ki, kd
    tilt_limit = 15 * np.pi / 180

    # === 仿真1: 阶跃响应 ===
    print("\n--- 仿真1: 阶跃响应 ---")
    t1, px1, py1, cx1, cy1, rx1, ry1 = run_simulation(
        pid_params, pid_params,
        x0=[-0.05, 0, -0.03, 0],
        target_mode='step',
        duration=5, dt=dt
    )

    # === 仿真2: 圆形轨迹跟踪 ===
    print("--- 仿真2: 圆形轨迹跟踪 ---")
    t2, px2, py2, cx2, cy2, rx2, ry2 = run_simulation(
        pid_params, pid_params,
        x0=[0, 0, 0, 0],
        target_mode='circle',
        duration=10, dt=dt
    )

    # === 仿真3: 8字轨迹跟踪 ===
    print("--- 仿真3: 8字轨迹跟踪 ---")
    t3, px3, py3, cx3, cy3, rx3, ry3 = run_simulation(
        pid_params, pid_params,
        x0=[0, 0, 0, 0],
        target_mode='figure8',
        duration=10, dt=dt
    )

    # 性能指标
    err_x1 = np.abs(rx1 - px1)
    err_y1 = np.abs(ry1 - py1)
    print(f"\n阶跃响应性能:")
    print(f"  X轴 稳态误差: {err_x1[-1000:].mean()*1000:.2f} mm")
    print(f"  Y轴 稳态误差: {err_y1[-1000:].mean()*1000:.2f} mm")

    # 绘图
    fig, axes = plt.subplots(3, 3, figsize=(18, 14))

    # 行1: 阶跃响应
    axes[0, 0].plot(t1, px1 * 1000, 'b-', label='实际', linewidth=1.5)
    axes[0, 0].plot(t1, rx1 * 1000, 'r--', label='目标', linewidth=1.5)
    axes[0, 0].set_title('阶跃响应 - X轴位置')
    axes[0, 0].set_ylabel('位置 (mm)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(t1, py1 * 1000, 'b-', label='实际', linewidth=1.5)
    axes[0, 1].plot(t1, ry1 * 1000, 'r--', label='目标', linewidth=1.5)
    axes[0, 1].set_title('阶跃响应 - Y轴位置')
    axes[0, 1].set_ylabel('位置 (mm)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    axes[0, 2].plot(px1 * 1000, py1 * 1000, 'b-', alpha=0.7, label='轨迹')
    axes[0, 2].plot(rx1 * 1000, ry1 * 1000, 'r*', markersize=15, label='目标')
    axes[0, 2].set_title('阶跃响应 - XY平面轨迹')
    axes[0, 2].set_xlabel('X (mm)')
    axes[0, 2].set_ylabel('Y (mm)')
    axes[0, 2].legend()
    axes[0, 2].grid(True, alpha=0.3)
    axes[0, 2].set_aspect('equal')

    # 行2: 圆形跟踪
    axes[1, 0].plot(t2, px2 * 1000, 'b-', label='实际', linewidth=1)
    axes[1, 0].plot(t2, rx2 * 1000, 'r--', label='目标', linewidth=1)
    axes[1, 0].set_title('圆形跟踪 - X轴')
    axes[1, 0].set_ylabel('位置 (mm)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(t2, py2 * 1000, 'b-', label='实际', linewidth=1)
    axes[1, 1].plot(t2, ry2 * 1000, 'r--', label='目标', linewidth=1)
    axes[1, 1].set_title('圆形跟踪 - Y轴')
    axes[1, 1].set_ylabel('位置 (mm)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    axes[1, 2].plot(px2 * 1000, py2 * 1000, 'b-', alpha=0.7, label='实际')
    axes[1, 2].plot(rx2 * 1000, ry2 * 1000, 'r--', alpha=0.7, label='目标')
    axes[1, 2].set_title('圆形跟踪 - XY平面')
    axes[1, 2].set_xlabel('X (mm)')
    axes[1, 2].set_ylabel('Y (mm)')
    axes[1, 2].legend()
    axes[1, 2].grid(True, alpha=0.3)
    axes[1, 2].set_aspect('equal')

    # 行3: 8字跟踪 + 控制量
    axes[2, 0].plot(px3 * 1000, py3 * 1000, 'b-', alpha=0.7, label='实际')
    axes[2, 0].plot(rx3 * 1000, ry3 * 1000, 'r--', alpha=0.7, label='目标')
    axes[2, 0].set_title('8字跟踪 - XY平面')
    axes[2, 0].set_xlabel('X (mm)')
    axes[2, 0].set_ylabel('Y (mm)')
    axes[2, 0].legend()
    axes[2, 0].grid(True, alpha=0.3)
    axes[2, 0].set_aspect('equal')

    axes[2, 1].plot(t2, np.degrees(cx2), 'b-', label='θx', linewidth=1)
    axes[2, 1].plot(t2, np.degrees(cy2), 'r-', label='θy', linewidth=1)
    axes[2, 1].set_title('圆形跟踪 - 倾斜角指令')
    axes[2, 1].set_xlabel('时间 (s)')
    axes[2, 1].set_ylabel('角度 (°)')
    axes[2, 1].legend()
    axes[2, 1].grid(True, alpha=0.3)

    # 误差分析
    err_circle = np.sqrt((rx2 - px2)**2 + (ry2 - py2)**2) * 1000
    axes[2, 2].plot(t2, err_circle, 'g-', linewidth=1)
    axes[2, 2].set_title('圆形跟踪 - 位置误差')
    axes[2, 2].set_xlabel('时间 (s)')
    axes[2, 2].set_ylabel('误差 (mm)')
    axes[2, 2].grid(True, alpha=0.3)

    plt.suptitle('滚球平衡系统 PID 双轴控制', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('ball_and_plate_pid.png', dpi=150)
    plt.close('all')

    print(f"\n圆形跟踪 RMS误差: {np.sqrt(np.mean(err_circle**2)):.2f} mm")
    print("\n仿真完成! 图形已保存为 ball_and_plate_pid.png")


if __name__ == "__main__":
    main()
