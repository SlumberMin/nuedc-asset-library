"""
四旋翼动力学仿真 - 6DOF + 电机模型 + PID控制
===============================================
仿真四旋翼无人机完整飞行动力学：
- 6自由度刚体动力学（位置+姿态）- ENU坐标系(z向上)
- 电机+螺旋桨模型（推力/扭矩/惯性）
- 级联PID控制器（外环位置/内环姿态）
- 轨迹跟踪仿真（圆形轨迹）
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class PID1D:
    """单轴PID控制器（带抗积分饱和）"""
    def __init__(self, Kp, Ki, Kd, dt, limit=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target, current):
        error = target - current
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        # 只在输出未饱和时积分（抗积分饱和）
        output_unclamped = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        if self.limit is not None and abs(output_unclamped) >= self.limit:
            # 饱和时冻结积分
            pass
        else:
            self.integral += error * self.dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        if self.limit is not None:
            output = np.clip(output, -self.limit, self.limit)
        return output


class Quadrotor:
    """四旋翼6DOF模型 (ENU坐标系: z向上)"""
    def __init__(self):
        self.m = 1.5
        self.g = 9.81
        self.Jx = 0.029
        self.Jy = 0.029
        self.Jz = 0.055
        self.arm = 0.25
        self.k_t = 5.0e-6      # 推力系数 N/(rpm²)
        self.k_q = 1.0e-7      # 扭矩系数 Nm/(rpm²)

        # 状态 (ENU: z向上)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        self.Cd = 0.3

    def rotation_matrix(self):
        """本体->世界 (ENU, ZYX Euler)"""
        cphi, sphi = np.cos(self.phi), np.sin(self.phi)
        cthe, sthe = np.cos(self.theta), np.sin(self.theta)
        cpsi, spsi = np.cos(self.psi), np.sin(self.psi)
        return np.array([
            [cpsi*cthe, cpsi*sthe*sphi - spsi*cphi, cpsi*sthe*cphi + spsi*sphi],
            [spsi*cthe, spsi*sthe*sphi + cpsi*cphi, spsi*sthe*cphi - cpsi*sphi],
            [-sthe,     cthe*sphi,                    cthe*cphi]
        ])

    def step(self, rpm_vec, dt):
        """rpm_vec: [ω1, ω2, ω3, ω4] 四个电机转速 (rpm)"""
        # 推力和力矩
        omega2 = rpm_vec**2
        thrusts = self.k_t * omega2
        total_thrust = np.sum(thrusts)

        # 十字构型: 1=前(+x), 2=右(+y), 3=后(-x), 4=左(-y)
        tau_phi   = self.arm * self.k_t * (omega2[1] - omega2[3])
        tau_theta = self.arm * self.k_t * (omega2[2] - omega2[0])
        tau_psi   = self.k_q * (omega2[0] - omega2[1] + omega2[2] - omega2[3])

        # 位置动力学 (ENU: z向上)
        R = self.rotation_matrix()
        # 推力沿本体z轴(向上)
        thrust_body = np.array([0, 0, total_thrust])
        thrust_world = R @ thrust_body
        gravity = np.array([0, 0, -self.m * self.g])  # 向下
        drag = -self.Cd * self.vel * np.abs(self.vel)
        accel = (thrust_world + gravity + drag) / self.m
        self.vel += accel * dt
        self.pos += self.vel * dt

        # 姿态动力学
        p_dot = (tau_phi + (self.Jy - self.Jz) * self.q * self.r) / self.Jx
        q_dot = (tau_theta + (self.Jz - self.Jx) * self.p * self.r) / self.Jy
        r_dot = (tau_psi + (self.Jx - self.Jy) * self.p * self.q) / self.Jz
        self.p += p_dot * dt
        self.q += q_dot * dt
        self.r += r_dot * dt

        # 欧拉角积分
        cphi, sphi = np.cos(self.phi), np.sin(self.phi)
        cthe, sthe = np.cos(self.theta), np.sin(self.theta)
        tthe = sthe / cthe if abs(cthe) > 1e-6 else 0

        self.phi   += (self.p + self.q * sphi * tthe + self.r * cphi * tthe) * dt
        self.theta += (self.q * cphi - self.r * sphi) * dt
        self.psi   += ((self.q * sphi + self.r * cphi) / max(cthe, 1e-6)) * dt

        self.phi   = np.clip(self.phi, -np.pi/3, np.pi/3)
        self.theta = np.clip(self.theta, -np.pi/3, np.pi/3)

        # 地面约束
        if self.pos[2] < 0:
            self.pos[2] = 0
            self.vel[2] = max(self.vel[2], 0)


class CascadedController:
    """级联PID控制器"""
    def __init__(self, dt=0.005):
        self.dt = dt
        # 外环: 位置
        self.pid_x = PID1D(Kp=2.0, Ki=0.05, Kd=1.5, dt=dt, limit=5.0)
        self.pid_y = PID1D(Kp=2.0, Ki=0.05, Kd=1.5, dt=dt, limit=5.0)
        self.pid_z = PID1D(Kp=4.0, Ki=0.0, Kd=3.0, dt=dt, limit=8.0)
        # 内环: 姿态
        self.pid_roll  = PID1D(Kp=6.0, Ki=0.05, Kd=2.0, dt=dt, limit=20.0)
        self.pid_pitch = PID1D(Kp=6.0, Ki=0.05, Kd=2.0, dt=dt, limit=20.0)
        self.pid_yaw   = PID1D(Kp=1.0, Ki=0.01, Kd=0.5, dt=dt, limit=2.0)

    def compute(self, quad, target_pos, target_yaw=0.0):
        """计算四个电机RPM"""
        # 位置PID -> 期望加速度
        ax = self.pid_x.compute(target_pos[0], quad.pos[0])
        ay = self.pid_y.compute(target_pos[1], quad.pos[1])
        az = self.pid_z.compute(target_pos[2], quad.pos[2])

        # 总推力: T = m*(g + az) / cos(phi)*cos(theta) 补偿倾斜
        cphi = np.cos(quad.phi)
        cthe = np.cos(quad.theta)
        tilt_comp = max(cphi * cthe, 0.7)  # 防止除以过小值
        T_total = quad.m * (quad.g + az) / tilt_comp
        T_total = max(T_total, 0)

        # 期望姿态 (小角度近似)
        desired_theta = np.clip(ax / quad.g, -0.5, 0.5)
        desired_phi   = np.clip(-ay / quad.g, -0.5, 0.5)

        # 姿态PID
        tau_phi   = self.pid_roll.compute(desired_phi, quad.phi)
        tau_theta = self.pid_pitch.compute(desired_theta, quad.theta)
        tau_psi   = self.pid_yaw.compute(target_yaw, quad.psi)

        # 分配到电机推力 (十字构型: 1=前, 2=右, 3=后, 4=左)
        # tau_phi = arm * (F2 - F4)
        # tau_theta = arm * (F3 - F1)
        # tau_psi = (k_q/k_t) * (F1 - F2 + F3 - F4)
        dF_roll  = tau_phi / quad.arm / 2
        dF_pitch = tau_theta / quad.arm / 2
        # Yaw: dF_yaw contributes via k_q/k_t ratio
        dF_yaw = tau_psi / (2 * quad.k_q / quad.k_t) if quad.k_q > 0 else 0

        t1 = T_total/4 - dF_pitch + dF_yaw  # 前(-theta, +yaw)
        t2 = T_total/4 + dF_roll  - dF_yaw  # 右(+phi, -yaw)
        t3 = T_total/4 + dF_pitch + dF_yaw  # 后(+theta, +yaw)
        t4 = T_total/4 - dF_roll  - dF_yaw  # 左(-phi, -yaw)

        rpms = np.sqrt(np.clip([t1, t2, t3, t4], 0, None) / quad.k_t)
        return np.clip(rpms, 0, 5000)


def run_hover_test(duration=8, dt=0.005):
    """悬停测试: 起飞到2m高度"""
    quad = Quadrotor()
    ctrl = CascadedController(dt)

    target = np.array([0.0, 0.0, 2.0])

    steps = int(duration / dt)
    t_arr = np.arange(steps) * dt
    pos_log = np.zeros((steps, 3))
    euler_log = np.zeros((steps, 3))

    for i in range(steps):
        rpm = ctrl.compute(quad, target)
        quad.step(rpm, dt)
        pos_log[i] = quad.pos
        euler_log[i] = np.degrees([quad.phi, quad.theta, quad.psi])

    err = np.linalg.norm(pos_log[-1] - target)
    print(f"[悬停] 最终位置: [{pos_log[-1,0]:.3f}, {pos_log[-1,1]:.3f}, {pos_log[-1,2]:.3f}] m")
    print(f"[悬停] 位置误差: {err:.4f} m")

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('四旋翼悬停测试 (ENU)', fontsize=14)

    for j, l in enumerate(['X', 'Y', 'Z']):
        axes[0, 0].plot(t_arr, pos_log[:, j], label=l)
    axes[0, 0].axhline(2, color='k', linestyle='--', alpha=0.3, label='目标高度')
    axes[0, 0].set_ylabel('位置 (m)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    for j, l in enumerate(['Roll', 'Pitch', 'Yaw']):
        axes[0, 1].plot(t_arr, euler_log[:, j], label=l)
    axes[0, 1].set_ylabel('欧拉角 (°)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    ax3d = fig.add_subplot(2, 2, 3, projection='3d')
    ax3d.plot(pos_log[:, 0], pos_log[:, 1], pos_log[:, 2], 'b-', linewidth=0.5)
    ax3d.scatter([0], [0], [2], c='r', marker='*', s=100, label='目标')
    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')
    ax3d.set_title('3D轨迹')
    ax3d.legend()

    err_log = np.linalg.norm(pos_log - target, axis=1)
    axes[1, 1].plot(t_arr, err_log, 'r-')
    axes[1, 1].set_ylabel('位置误差 (m)')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/quadrotor_hover.png', dpi=150)
    plt.close()
    print("[悬停] 图表已保存")


def run_trajectory_tracking(duration=20, dt=0.005):
    """圆形轨迹跟踪"""
    quad = Quadrotor()
    ctrl = CascadedController(dt)

    radius, omega_traj, height = 2.0, 0.5, 3.0

    steps = int(duration / dt)
    t_arr = np.arange(steps) * dt
    pos_log = np.zeros((steps, 3))
    target_log = np.zeros((steps, 3))

    for i in range(steps):
        t = t_arr[i]
        target = np.array([
            radius * np.cos(omega_traj * t),
            radius * np.sin(omega_traj * t),
            height
        ])
        target_log[i] = target
        rpm = ctrl.compute(quad, target, target_yaw=omega_traj * t)
        quad.step(rpm, dt)
        pos_log[i] = quad.pos

    rmse = np.sqrt(np.mean(np.sum((pos_log - target_log)**2, axis=1)))
    print(f"[轨迹跟踪] RMSE: {rmse:.4f} m")

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(target_log[:, 0], target_log[:, 1], target_log[:, 2],
            'r--', label='目标轨迹', linewidth=2)
    ax.plot(pos_log[:, 0], pos_log[:, 1], pos_log[:, 2],
            'b-', label='实际轨迹', linewidth=0.8)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'四旋翼圆形轨迹跟踪 (RMSE={rmse:.3f}m)')
    ax.legend()
    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/quadrotor_trajectory.png', dpi=150)
    plt.close()
    print("[轨迹跟踪] 图表已保存")


if __name__ == '__main__':
    print("=" * 60)
    print("四旋翼动力学仿真 - 6DOF + 电机模型 + PID控制")
    print("=" * 60)
    run_hover_test()
    run_trajectory_tracking()
    print("\n✅ 四旋翼仿真完成！")
