# -*- coding: utf-8 -*-
"""
Mahony滤波器仿真 — 四元数姿态估计
====================================
实现 Mahony 互补滤波器（AHRS），融合陀螺仪/加速度计/磁力计。
对比有/无磁力计校正、不同 Kp/Ki 增益的效果。

用法：python mahony_filter_simulation.py
"""

import numpy as np
import matplotlib.pyplot as plt
import os

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ── 四元数工具 ────────────────────────────────────────────
def quat_normalize(q):
    n = np.linalg.norm(q)
    return q / n if n > 1e-10 else np.array([1, 0, 0, 0], dtype=float)

def quat_multiply(q, r):
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1])

def quat_to_euler(q):
    w, x, y, z = q
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

def rotate_vector_by_quat(v, q):
    """q * v * q_conj"""
    qv = np.array([0, v[0], v[1], v[2]])
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    tmp = quat_multiply(q, qv)
    res = quat_multiply(tmp, q_conj)
    return res[1:4]

# ── Mahony滤波器 ──────────────────────────────────────────
class MahonyFilter:
    def __init__(self, kp=10.0, ki=0.0, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.q = np.array([1.0, 0, 0, 0])
        self.ei = np.zeros(3)  # 积分误差

    def update(self, gyro, acc, mag=None):
        """
        gyro: [gx, gy, gz] rad/s
        acc:  [ax, ay, az] m/s² (归一化)
        mag:  [mx, my, mz] (可选)
        """
        ax, ay, az = acc
        norm_a = np.sqrt(ax*ax + ay*ay + az*az)
        if norm_a < 1e-10:
            return self.q
        ax, ay, az = ax/norm_a, ay/norm_a, az/norm_a

        w, x, y, z = self.q

        # 估计的重力方向
        vx = 2*(x*z - w*y)
        vy = 2*(w*x + y*z)
        vz = w*w - x*x - y*y + z*z

        # 加速度计误差（叉积）
        ex = ay*vz - az*vy
        ey = az*vx - ax*vz
        ez = ax*vy - ay*vx

        # 磁力计校正
        if mag is not None:
            mx, my, mz = mag
            norm_m = np.sqrt(mx*mx + my*my + mz*mz)
            if norm_m > 1e-10:
                mx, my, mz = mx/norm_m, my/norm_m, mz/norm_m
                # 估计磁场方向
                qm = rotate_vector_by_quat([mx, my, mz], self.q)
                bx = np.sqrt(qm[0]**2 + qm[1]**2)
                bz = qm[2]
                # 估计磁场参考
                hx = 2*(bx*(0.5 - y*y - z*z) + bz*(x*z - w*y))
                hy = 2*(bx*(x*y - w*z) + bz*(w*x + y*z))
                hz = 2*(bx*(w*y + x*z) + bz*(0.5 - x*x - y*y))
                # 磁场误差
                ex += my*hz - mz*hy
                ey += mz*hx - mx*hz
                ez += mx*hy - my*hx

        # PI校正
        self.ei += np.array([ex, ey, ez]) * self.dt * self.ki
        gx = gyro[0] + self.kp * ex + self.ei[0]
        gy = gyro[1] + self.kp * ey + self.ei[1]
        gz = gyro[2] + self.kp * ez + self.ei[2]

        # 四元数微分
        q_dot = 0.5 * quat_multiply(self.q, np.array([0, gx, gy, gz]))
        self.q = quat_normalize(self.q + q_dot * self.dt)
        return self.q.copy()

# ── 仿真 ──────────────────────────────────────────────────
DT = 0.01
T_END = 15.0
N = int(T_END / DT)
t = np.linspace(0, T_END, N)
np.random.seed(42)

# 真实姿态
true_roll  = 30 * np.sin(2*np.pi*0.3*t)
true_pitch = 20 * np.sin(2*np.pi*0.2*t + 0.5)
true_yaw   = 45 * np.sin(2*np.pi*0.1*t)

# 传感器数据
gyro_noise = 0.02
acc_noise = 0.5

def true_acc(r, p):
    """重力在体轴的投影"""
    cr, sr = np.cos(np.radians(r)), np.sin(np.radians(r))
    cp, sp = np.cos(np.radians(p)), np.sin(np.radians(p))
    return np.array([-sp, sr*cp, cr*cp])

def true_gyro(r, p, y, dt):
    return np.array([
        np.radians(np.gradient(r, dt)),
        np.radians(np.gradient(p, dt)),
        np.radians(np.gradient(y, dt))
    ])

# 对比不同参数
configs = {
    'Kp=10, Ki=0':   {'kp': 10.0, 'ki': 0.0},
    'Kp=10, Ki=0.1': {'kp': 10.0, 'ki': 0.1},
    'Kp=50, Ki=0':   {'kp': 50.0, 'ki': 0.0},
    'Kp=5, Ki=0.3':  {'kp': 5.0,  'ki': 0.3},
}

results = {}
for name, cfg in configs.items():
    filt = MahonyFilter(kp=cfg['kp'], ki=cfg['ki'], dt=DT)
    rolls, pitches, yaws = [], [], []
    for i in range(N):
        acc = true_acc(true_roll[i], true_pitch[i]) + np.random.randn(3) * acc_noise
        gyro = np.array([
            np.radians(np.gradient(true_roll, DT)[i]),
            np.radians(np.gradient(true_pitch, DT)[i]),
            np.radians(np.gradient(true_yaw, DT)[i])
        ]) + np.random.randn(3) * gyro_noise
        q = filt.update(gyro, acc)
        r, p, y = quat_to_euler(q)
        rolls.append(r)
        pitches.append(p)
        yaws.append(y)
    results[name] = (np.array(rolls), np.array(pitches), np.array(yaws))

# ── 绘图 ──────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
colors = ['#2196F3', '#4CAF50', '#FF9800', '#F44336']

for ax_idx, (label, true_val) in enumerate([
    ('Roll (°)', true_roll), ('Pitch (°)', true_pitch), ('Yaw (°)', true_yaw)
]):
    ax = axes[ax_idx]
    ax.plot(t, true_val, 'k--', lw=2, label='真实值')
    for (name, (r, p, y)), color in zip(results.items(), colors):
        val = [r, p, y][ax_idx]
        ax.plot(t, val, color=color, lw=0.8, alpha=0.8, label=name)
    ax.set_ylabel(label)
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

axes[0].set_title('Mahony互补滤波器 — 不同Kp/Ki参数对比')
axes[2].set_xlabel('时间 (s)')

plt.tight_layout()
out_dir = os.path.dirname(os.path.abspath(__file__))
out_path = os.path.join(out_dir, 'mahony_filter_simulation.png')
plt.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"图表已保存: {out_path}")
plt.show()

# 误差统计
print("\n" + "=" * 60)
print("Mahony滤波器 — Roll RMSE 对比")
print("=" * 60)
for name, (r, p, y) in results.items():
    rmse_r = np.sqrt(np.mean((r - true_roll)**2))
    rmse_p = np.sqrt(np.mean((p - true_pitch)**2))
    print(f"  {name:20s}  Roll RMSE={rmse_r:.2f}°  Pitch RMSE={rmse_p:.2f}°")
