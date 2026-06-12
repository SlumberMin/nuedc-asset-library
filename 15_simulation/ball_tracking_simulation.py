#!/usr/bin/env python3
"""
小球追踪仿真 - 视觉追踪 + 云台控制
==============================================
功能：
  1. 模拟小球在画面中的运动（圆形轨迹/随机轨迹）
  2. 模拟摄像头检测延迟和噪声
  3. PID 云台控制追踪小球
  4. 分析追踪误差和响应特性
使用：
  python ball_tracking_simulation.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.gridspec as gridspec

# ========== 参数 ==========


def main():
    dt = 0.02           # 控制周期 (50Hz，模拟摄像头帧率)
    total_time = 20.0
    steps = int(total_time / dt)

    # 摄像头参数
    CAMERA_W, CAMERA_H = 640, 480  # 画面分辨率
    DETECT_DELAY = 2                # 检测延迟帧数
    NOISE_STD = 5                   # 像素检测噪声 (标准差)

    # 云台 PID 参数
    PID_PAN  = {'kp': 0.15, 'ki': 0.02, 'kd': 0.08}   # 水平
    PID_TILT = {'kp': 0.15, 'ki': 0.02, 'kd': 0.08}   # 垂直

    # ========== 小球运动模型 ==========
    class BallMotion:
        """小球运动轨迹生成器"""
        def __init__(self, mode='circle'):
            self.mode = mode
            self.t = 0
            self.cx, self.cy = CAMERA_W/2, CAMERA_H/2

        def update(self, dt):
            self.t += dt
            if self.mode == 'circle':
                r = 120
                freq = 0.3
                x = self.cx + r * np.cos(2 * np.pi * freq * self.t)
                y = self.cy + r * np.sin(2 * np.pi * freq * self.t)
            elif self.mode == 'figure8':
                r = 100
                x = self.cx + r * np.sin(2 * np.pi * 0.3 * self.t)
                y = self.cy + r * np.sin(2 * np.pi * 0.6 * self.t) * 0.5
            elif self.mode == 'random':
                x = self.cx + 150 * np.sin(0.2 * self.t) + 50 * np.sin(0.7 * self.t)
                y = self.cy + 100 * np.cos(0.3 * self.t) + 40 * np.cos(0.9 * self.t)
            else:  # step
                x = self.cx + (100 if int(self.t) % 4 < 2 else -100)
                y = self.cy + (80 if int(self.t) % 6 < 3 else -80)
            return x, y

    # ========== PID 控制器 ==========
    class PID:
        def __init__(self, kp, ki, kd, output_limit=100, integral_limit=500):
            self.kp, self.ki, self.kd = kp, ki, kd
            self.output_limit = output_limit
            self.integral_limit = integral_limit
            self.integral = 0
            self.prev_error = 0

        def update(self, error, dt):
            self.integral += error * dt
            self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
            derivative = (error - self.prev_error) / dt if dt > 0 else 0
            self.prev_error = error
            output = self.kp * error + self.ki * self.integral + self.kd * derivative
            return np.clip(output, -self.output_limit, self.output_limit)

    # ========== 云台模型 ==========
    class Gimbal:
        """二阶系统云台模型"""
        def __init__(self):
            self.pan_angle = 0     # 当前水平角度
            self.tilt_angle = 0    # 当前垂直角度
            self.pan_vel = 0
            self.tilt_vel = 0
            self.omega = 15.0      # 自然频率
            self.zeta = 0.7        # 阻尼比

        def update(self, pan_cmd, tilt_cmd, dt):
            # 二阶系统响应
            for angle, vel, cmd, attr in [
                ('pan_angle', 'pan_vel', pan_cmd, 'pan'),
                ('tilt_angle', 'tilt_vel', tilt_cmd, 'tilt')
            ]:
                acc = self.omega**2 * (cmd - getattr(self, angle)) - 2 * self.zeta * self.omega * getattr(self, vel)
                new_vel = getattr(self, vel) + acc * dt
                new_angle = getattr(self, angle) + new_vel * dt
                setattr(self, vel, new_vel)
                setattr(self, angle, new_angle)

        def get_pixel_pos(self):
            """将角度映射到像素位置"""
            px = CAMERA_W/2 + self.pan_angle * 10   # 简单线性映射
            py = CAMERA_H/2 + self.tilt_angle * 10
            return px, py

    # ========== 主仿真 ==========
    print("开始小球追踪仿真...")

    modes = ['circle', 'figure8', 'random', 'step']
    mode_names = ['圆形轨迹', '8字轨迹', '随机轨迹', '阶跃响应']
    results = {}

    for mode, name in zip(modes, mode_names):
        ball = BallMotion(mode)
        gimbal = Gimbal()
        pid_pan = PID(**PID_PAN)
        pid_tilt = PID(**PID_TILT)

        # 延迟缓冲
        detect_buffer_x = [CAMERA_W/2] * (DETECT_DELAY + 1)
        detect_buffer_y = [CAMERA_H/2] * (DETECT_DELAY + 1)

        hist = {'t': [], 'bx': [], 'by': [], 'gx': [], 'gy': [], 'err': []}

        for i in range(steps):
            t = i * dt
            bx, by = ball.update(dt)

            # 加噪声
            nx = bx + np.random.normal(0, NOISE_STD)
            ny = by + np.random.normal(0, NOISE_STD)

            # 延迟
            detect_buffer_x.append(nx)
            detect_buffer_y.append(ny)
            dx = detect_buffer_x.pop(0)
            dy = detect_buffer_y.pop(0)

            # PID 计算
            err_x = dx - gimbal.get_pixel_pos()[0]
            err_y = dy - gimbal.get_pixel_pos()[1]
            pan_cmd = pid_pan.update(err_x, dt)
            tilt_cmd = pid_tilt.update(err_y, dt)

            gimbal.update(pan_cmd, tilt_cmd, dt)
            gx, gy = gimbal.get_pixel_pos()
            err = np.sqrt((bx - gx)**2 + (by - gy)**2)

            hist['t'].append(t)
            hist['bx'].append(bx)
            hist['by'].append(by)
            hist['gx'].append(gx)
            hist['gy'].append(gy)
            hist['err'].append(err)

        results[name] = hist
        mean_err = np.mean(hist['err'][500:])  # 忽略初始瞬态
        print(f"  [{name}] 平均追踪误差: {mean_err:.1f} px")

    # ========== 可视化 ==========
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('小球追踪仿真 - 视觉追踪 + 云台控制', fontsize=14, fontweight='bold')
    gs = gridspec.GridSpec(3, 2, hspace=0.35, wspace=0.3)

    for idx, (name, hist) in enumerate(results.items()):
        ax = fig.add_subplot(gs[idx // 2, idx % 2])
        ax.plot(hist['bx'], hist['by'], 'b-', linewidth=0.8, alpha=0.5, label='小球轨迹')
        ax.plot(hist['gx'], hist['gy'], 'r-', linewidth=0.8, alpha=0.7, label='云台追踪')
        ax.set_xlim(0, CAMERA_W)
        ax.set_ylim(0, CAMERA_H)
        ax.set_title(name)
        ax.set_xlabel('X (px)')
        ax.set_ylabel('Y (px)')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.invert_yaxis()

    # 追踪误差对比
    ax_err = fig.add_subplot(gs[2, 0])
    for name, hist in results.items():
        ax_err.plot(hist['t'], hist['err'], linewidth=0.5, alpha=0.7, label=name)
    ax_err.set_xlabel('时间 (s)')
    ax_err.set_ylabel('追踪误差 (px)')
    ax_err.set_title('追踪误差对比')
    ax_err.legend(fontsize=7)
    ax_err.grid(True, alpha=0.3)

    # 阶跃响应详细分析
    ax_step = fig.add_subplot(gs[2, 1])
    hist = results['阶跃响应']
    ax_step.plot(hist['t'], hist['bx'], 'b-', linewidth=1, label='目标X')
    ax_step.plot(hist['t'], hist['gx'], 'r-', linewidth=1, label='云台X')
    ax_step.set_xlabel('时间 (s)')
    ax_step.set_ylabel('X 坐标 (px)')
    ax_step.set_title('阶跃响应详细分析')
    ax_step.legend(fontsize=7)
    ax_step.grid(True, alpha=0.3)

    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ball_tracking_result.png'), dpi=150)
    print("图表已保存: ball_tracking_result.png")
    plt.close('all')



if __name__ == '__main__':
    main()
