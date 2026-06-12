"""
卫星姿态控制仿真 - 四元数 + 反作用轮 + 扰动
==============================================
仿真卫星三轴姿态控制系统：
- 四元数姿态表示（避免万向锁）
- 反作用轮执行器模型（饱和/摩擦/转速限制）
- 多种空间扰动模型（重力梯度、地磁、太阳光压）
- PD控制律
- 姿态机动与稳定保持仿真
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


# ─────────────────────────────────────────────
# 四元数工具
# ─────────────────────────────────────────────
def quat_multiply(q1, q2):
    """四元数乘法 (Hamilton: [w,x,y,z])"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_normalize(q):
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1, 0, 0, 0])

def quat_to_euler(q):
    """四元数转欧拉角 [roll, pitch, yaw] (rad)"""
    w, x, y, z = q
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])

def euler_to_quat(euler):
    """欧拉角 -> 四元数"""
    r, p, y = euler / 2
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    ])


# ─────────────────────────────────────────────
# 卫星动力学模型
# ─────────────────────────────────────────────
class Satellite:
    def __init__(self):
        # 转动惯量 kg·m²
        self.J = np.diag([0.05, 0.06, 0.07])
        self.J_inv = np.linalg.inv(self.J)
        # 姿态四元数 [w,x,y,z]
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        # 角速度 rad/s
        self.omega = np.array([0.01, -0.005, 0.008])
        # 轨道参数
        self.orbit_rate = 0.0011  # rad/s

    def step(self, tau_total, dt):
        """使用RK4积分器更新状态"""
        # 定义状态导数
        def deriv(state):
            q = state[:4]
            w = state[4:7]
            # 陀螺力矩: ω × (Jω)
            gyro = np.cross(w, self.J @ w)
            w_dot = self.J_inv @ (tau_total - gyro)
            # 四元数导数
            w_quat = np.array([0, w[0], w[1], w[2]])
            q_dot = 0.5 * quat_multiply(q, w_quat)
            return np.concatenate([q_dot, w_dot])

        state = np.concatenate([self.q, self.omega])
        k1 = deriv(state)
        k2 = deriv(state + 0.5 * dt * k1)
        k3 = deriv(state + 0.5 * dt * k2)
        k4 = deriv(state + dt * k3)
        state = state + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

        self.q = quat_normalize(state[:4])
        self.omega = state[4:7]


# ─────────────────────────────────────────────
# 反作用轮模型
# ─────────────────────────────────────────────
class ReactionWheel:
    def __init__(self, axis, h_max=0.2, J_rw=3e-4, friction_coeff=1e-4):
        self.axis = axis / np.linalg.norm(axis)
        self.h_max = h_max
        self.J_rw = J_rw
        self.friction = friction_coeff
        self.momentum = 0.0  # 角动量 Nms

    def apply_torque(self, torque_cmd, dt):
        """
        输入: 期望施加在卫星上的控制力矩 (在该轴分量)
        返回: (实际施加在卫星上的力矩向量, 内部角动量变化)
        """
        # 转换为作用在轮上的力矩（反作用）
        tau_wheel = -torque_cmd  # 轮上的力矩
        # 摩擦
        tau_friction = -self.friction * self.momentum / self.J_rw
        # 积分角动量
        self.momentum += (tau_wheel + tau_friction) * dt
        # 饱和
        if abs(self.momentum) > self.h_max:
            self.momentum = np.sign(self.momentum) * self.h_max
        # 实际施加在卫星上的力矩 = -dH_rw/dt (近似)
        tau_actual = torque_cmd * self.axis
        return tau_actual

    def get_omega(self):
        return self.momentum / self.J_rw


# ─────────────────────────────────────────────
# 空间扰动模型
# ─────────────────────────────────────────────
class SpaceDisturbances:
    def __init__(self, J, orbit_rate):
        self.J = J
        self.n = orbit_rate

    def gravity_gradient(self, q):
        """重力梯度力矩: τ = 3n² (r× Jr)"""
        # 地心方向在本体坐标系 (通过四元数旋转)
        q_inv = quat_conjugate(q)
        # 世界坐标系地心方向 (NED: z向下)
        r_world = np.array([0, 0, 1])
        # 旋转到本体
        r_quat = np.array([0, *r_world])
        r_body_quat = quat_multiply(quat_multiply(q_inv, r_quat), q)
        r_hat = r_body_quat[1:4]
        r_hat = r_hat / (np.linalg.norm(r_hat) + 1e-12)
        return 3 * self.n**2 * np.cross(r_hat, self.J @ r_hat)

    def magnetic_disturbance(self, t):
        """地磁扰动"""
        B_0 = 2e-5
        B_body = B_0 * np.array([
            np.sin(self.n * t),
            0.5 * np.cos(self.n * t),
            0.3 * np.sin(2 * self.n * t)
        ])
        m_res = np.array([0.001, -0.002, 0.001])  # 残余磁矩
        return np.cross(m_res, B_body)

    def solar_radiation(self, t):
        """太阳光压力矩"""
        sun_angle = self.n * t * 0.01
        sun_dir = np.array([np.cos(sun_angle), np.sin(sun_angle), 0])
        cp_offset = np.array([0.002, -0.001, 0.003])
        F_srp = 4.5e-6 * sun_dir
        return np.cross(cp_offset, F_srp)

    def total(self, q, t):
        return (self.gravity_gradient(q) +
                self.magnetic_disturbance(t) +
                self.solar_radiation(t))


# ─────────────────────────────────────────────
# 姿态控制器 (PD)
# ─────────────────────────────────────────────
class AttitudeController:
    def __init__(self, Kp, Kd, max_torque=0.05):
        self.Kp = np.array(Kp)
        self.Kd = np.array(Kd)
        self.max_torque = max_torque

    def compute(self, q_current, q_desired, omega):
        """计算控制力矩"""
        # 姿态误差: q_err = q_des* ⊗ q_curr
        q_err = quat_multiply(quat_conjugate(q_desired), q_current)
        if q_err[0] < 0:
            q_err = -q_err
        # 小角度误差向量
        e_att = 2.0 * q_err[1:4]
        e_rate = omega  # 期望角速度为0
        # PD控制律
        torque = -self.Kp * e_att - self.Kd * e_rate
        # 饱和限制
        torque = np.clip(torque, -self.max_torque, self.max_torque)
        return torque


# ─────────────────────────────────────────────
# 仿真主函数
# ─────────────────────────────────────────────
def run_simulation(duration=500, dt=0.5):
    """卫星姿态控制仿真"""
    sat = Satellite()
    dist = SpaceDisturbances(sat.J, sat.orbit_rate)
    ctrl = AttitudeController(
        Kp=[0.005, 0.005, 0.005],
        Kd=[0.02, 0.02, 0.02],
        max_torque=0.03
    )

    # 三轴反作用轮
    rw = [ReactionWheel(np.array([1,0,0])),
          ReactionWheel(np.array([0,1,0])),
          ReactionWheel(np.array([0,0,1]))]

    # 目标: 30度偏航
    angle = np.radians(30)
    q_target = np.array([np.cos(angle/2), 0, 0, np.sin(angle/2)])

    steps = int(duration / dt)
    t_arr = np.arange(steps) * dt
    euler_log = np.zeros((steps, 3))
    omega_log = np.zeros((steps, 3))
    rw_mom_log = np.zeros((steps, 3))
    dist_log = np.zeros((steps, 3))

    for i in range(steps):
        euler_log[i] = np.degrees(quat_to_euler(sat.q))
        omega_log[i] = sat.omega
        rw_mom_log[i] = [rw[j].momentum for j in range(3)]

        # 扰动
        tau_dist = dist.total(sat.q, t_arr[i])
        dist_log[i] = tau_dist

        # 控制
        tau_ctrl = ctrl.compute(sat.q, q_target, sat.omega)

        # 分配到反作用轮
        tau_rw_total = np.zeros(3)
        for j in range(3):
            tau_rw_total += rw[j].apply_torque(tau_ctrl[j], dt)

        # 动力学更新 (扰动 + 反作用轮力矩)
        sat.step(tau_dist + tau_rw_total, dt)

    # 性能
    euler_target = np.degrees(quat_to_euler(q_target))
    final_err = np.abs(euler_log[-1] - euler_target)
    print(f"[姿态控制] 目标欧拉角: [{euler_target[0]:.1f}, {euler_target[1]:.1f}, {euler_target[2]:.1f}]°")
    print(f"[姿态控制] 最终欧拉角: [{euler_log[-1][0]:.3f}, {euler_log[-1][1]:.3f}, {euler_log[-1][2]:.3f}]°")
    print(f"[姿态控制] 姿态误差: [{final_err[0]:.4f}, {final_err[1]:.4f}, {final_err[2]:.4f}]°")

    # 绘图
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('卫星姿态控制仿真 (四元数+反作用轮+扰动)', fontsize=14)

    labels = ['Roll(φ)', 'Pitch(θ)', 'Yaw(ψ)']
    colors = ['r', 'g', 'b']
    for j in range(3):
        axes[0].plot(t_arr, euler_log[:, j], colors[j], label=labels[j], linewidth=0.8)
    axes[0].axhline(euler_target[2], color='b', linestyle='--', alpha=0.5, label='Yaw目标')
    axes[0].set_ylabel('欧拉角 (°)')
    axes[0].legend(loc='right')
    axes[0].grid(True, alpha=0.3)

    for j in range(3):
        axes[1].plot(t_arr, np.degrees(omega_log[:, j]), colors[j],
                     label=f'ω{["x","y","z"][j]}', linewidth=0.8)
    axes[1].set_ylabel('角速度 (°/s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    for j in range(3):
        axes[2].plot(t_arr, rw_mom_log[:, j] * 1000, colors[j],
                     label=f'RW{["x","y","z"][j]}', linewidth=0.8)
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('角动量 (mNms)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/satellite_attitude.png', dpi=150)
    plt.close()
    print("[姿态控制] 图表已保存")

    return t_arr, euler_log, omega_log


if __name__ == '__main__':
    print("=" * 60)
    print("卫星姿态控制仿真 - 四元数 + 反作用轮 + 扰动")
    print("=" * 60)
    run_simulation()
    print("\n✅ 卫星姿态控制仿真完成！")
