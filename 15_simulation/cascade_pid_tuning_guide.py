# -*- coding: utf-8 -*-
"""
串级PID调参指南仿真
====================
演示串级PID（外环+内环）的调参过程：
1. 先调内环（速度环/电流环）
2. 再调外环（位置环）
3. 对比不同内外环带宽比的效果
4. 展示常见调参错误及修正

适用场景：电机位置控制、平衡车、四旋翼姿态控制等。

用法：python cascade_pid_tuning_guide.py
"""

import numpy as np
import matplotlib.pyplot as plt
import os

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ── 被控对象 ──────────────────────────────────────────────
class MotorPlant:
    """简化的电机-负载模型
    内环：电流/速度环 → 二阶系统
    外环：位置环 → 积分（速度的积分=位置）
    """
    def __init__(self, dt=0.001):
        self.dt = dt
        self.velocity = 0.0
        self.position = 0.0
        self.tau_motor = 0.05  # 电机时间常数 50ms
        self.K_motor = 10.0    # 电机增益

    def update(self, u):
        """u: 控制量 → 加速度"""
        # 一阶惯性：dv/dt = (K*u - v) / tau
        accel = (self.K_motor * u - self.velocity) / self.tau_motor
        self.velocity += accel * self.dt
        self.position += self.velocity * self.dt
        return self.velocity, self.position

    def reset(self):
        self.velocity = 0.0
        self.position = 0.0

# ── PID控制器 ─────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, dt, u_max=None, d_filter_tau=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.u_max = u_max
        self.d_filter_tau = d_filter_tau

        self.integral = 0.0
        self.prev_error = 0.0
        self.d_filtered = 0.0
        self.prev_output = 0.0

    def compute(self, error, feedback_rate=0.0):
        # P
        p_term = self.kp * error

        # I (带限幅)
        self.integral += error * self.dt
        if self.u_max is not None and self.ki > 0:
            max_i = self.u_max / self.ki
            self.integral = np.clip(self.integral, -max_i, max_i)
        i_term = self.ki * self.integral

        # D (一阶低通滤波，避免微分噪声)
        d_raw = (error - self.prev_error) / self.dt
        alpha = self.dt / (self.d_filter_tau + self.dt)
        self.d_filtered = alpha * d_raw + (1 - alpha) * self.d_filtered
        d_term = self.kd * self.d_filtered

        self.prev_error = error

        output = p_term + i_term + d_term
        if self.u_max is not None:
            output = np.clip(output, -self.u_max, self.u_max)
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.d_filtered = 0.0

# ── 串级PID ───────────────────────────────────────────────
def run_cascade(setpoint, kp_outer, ki_outer, kd_outer,
                kp_inner, ki_inner, kd_inner,
                dt=0.001, t_end=2.0, label=""):
    N = int(t_end / dt)
    t = np.linspace(0, t_end, N)

    plant = MotorPlant(dt=dt)
    outer_pid = PID(kp_outer, ki_outer, kd_outer, dt, u_max=100)
    inner_pid = PID(kp_inner, ki_inner, kd_inner, dt, u_max=50)

    positions = []
    velocities = []
    inner_refs = []
    u_inners = []

    for i in range(N):
        # 外环：位置误差 → 速度参考
        pos_error = setpoint[i] - plant.position
        vel_ref = outer_pid.compute(pos_error)

        # 内环：速度误差 → 控制量
        vel_error = vel_ref - plant.velocity
        u_inner = inner_pid.compute(vel_error)

        # 更新被控对象
        v, p = plant.update(u_inner)

        positions.append(p)
        velocities.append(v)
        inner_refs.append(vel_ref)
        u_inners.append(u_inner)

    return t, np.array(positions), np.array(velocities), np.array(inner_refs), np.array(u_inners)

# ── 仿真场景 ──────────────────────────────────────────────
DT = 0.001
T_END = 3.0
N = int(T_END / DT)
t_base = np.linspace(0, T_END, N)

# 阶跃设定值
setpoint = np.zeros(N)
setpoint[int(0.2/DT):] = 1.0  # 0.2s后阶跃到1.0

# 场景配置
scenarios = {
    'Step1_先内环后外环': {
        'desc': '正确调参：先调好内环，再调外环',
        'kp_o': 20, 'ki_o': 5,  'kd_o': 0.5,
        'kp_i': 8,  'ki_i': 20, 'kd_i': 0,
    },
    'Step2_内环过弱': {
        'desc': '内环Kp太小，跟踪慢',
        'kp_o': 20, 'ki_o': 5,  'kd_o': 0.5,
        'kp_i': 1,  'ki_i': 5,  'kd_i': 0,
    },
    'Step3_外环过强': {
        'desc': '外环增益过高，内环跟不上→振荡',
        'kp_o': 80, 'ki_o': 20, 'kd_o': 1,
        'kp_i': 8,  'ki_i': 20, 'kd_i': 0,
    },
    'Step4_内外环均衡': {
        'desc': '优化后的参数（带宽比约5:1）',
        'kp_o': 15, 'ki_o': 8,  'kd_o': 0.3,
        'kp_i': 10, 'ki_i': 30, 'kd_i': 0,
    },
}

results = {}
for name, cfg in scenarios.items():
    t, pos, vel, vel_ref, u = run_cascade(
        setpoint,
        cfg['kp_o'], cfg['ki_o'], cfg['kd_o'],
        cfg['kp_i'], cfg['ki_i'], cfg['kd_i'],
        dt=DT, t_end=T_END)
    results[name] = (t, pos, vel, vel_ref, u, cfg['desc'])

# ── 绘图1：调参步骤对比 ──────────────────────────────────
fig1, axes1 = plt.subplots(2, 2, figsize=(14, 10))
colors = ['#F44336', '#2196F3', '#4CAF50', '#FF9800']

for ax, ((name, (t, pos, vel, vel_ref, u, desc)), color) in \
        zip(axes1.flatten(), zip(results.items(), colors)):
    ax.plot(t, setpoint, 'k--', lw=1.5, label='设定值')
    ax.plot(t, pos, color=color, lw=1.5, label='位置')
    ax.set_title(f'{name}\n{desc}', fontsize=10)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('位置')
    ax.legend(loc='lower right', fontsize=8)
    ax.grid(True, alpha=0.3)

fig1.suptitle('串级PID调参指南 — 4个调参阶段', fontsize=14, fontweight='bold')
plt.tight_layout()

out_dir = os.path.dirname(os.path.abspath(__file__))
path1 = os.path.join(out_dir, 'cascade_pid_tuning_guide.png')
fig1.savefig(path1, dpi=150, bbox_inches='tight')
print(f"图1已保存: {path1}")

# ── 绘图2：带宽比影响 ────────────────────────────────────
bandwidth_ratios = [2, 5, 10, 20]
fig2, axes2 = plt.subplots(2, 2, figsize=(14, 10))

inner_kps = [5, 10, 20, 40]
for ax, inner_kp, ratio in zip(axes2.flatten(), inner_kps, bandwidth_ratios):
    t, pos, vel, vel_ref, u, _ = run_cascade(
        setpoint, kp_outer=15, ki_outer=8, kd_outer=0.3,
        kp_inner=inner_kp, ki_inner=30, kd_inner=0,
        dt=DT, t_end=T_END)

    ax.plot(t, setpoint, 'k--', lw=1.5, label='设定值')
    ax.plot(t, pos, 'b-', lw=1.2, label='位置')

    # 性能指标
    rise_idx = np.argmax(pos > 0.9)
    rise_time = rise_idx * DT if rise_idx > 0 else T_END
    overshoot = (np.max(pos) - 1.0) * 100 if np.max(pos) > 1.0 else 0

    ax.set_title(f'内环Kp={inner_kp} (带宽比≈{ratio}:1)\n'
                 f'上升时间={rise_time:.3f}s, 超调={overshoot:.1f}%', fontsize=10)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('位置')
    ax.legend(loc='lower right', fontsize=8)
    ax.grid(True, alpha=0.3)

fig2.suptitle('串级PID — 内外环带宽比影响', fontsize=14, fontweight='bold')
plt.tight_layout()

path2 = os.path.join(out_dir, 'cascade_pid_bandwidth_ratio.png')
fig2.savefig(path2, dpi=150, bbox_inches='tight')
print(f"图2已保存: {path2}")

# ── 绘图3：内环响应细节 ──────────────────────────────────
fig3, axes3 = plt.subplots(2, 1, figsize=(14, 7), sharex=True)

# 用最优参数
t, pos, vel, vel_ref, u, _ = run_cascade(
    setpoint, kp_outer=15, ki_outer=8, kd_outer=0.3,
    kp_inner=10, ki_inner=30, kd_inner=0,
    dt=DT, t_end=T_END)

ax = axes3[0]
ax.plot(t, setpoint, 'k--', lw=1.5, label='位置设定值')
ax.plot(t, pos, 'b-', lw=1.2, label='实际位置')
ax.plot(t, vel, 'r-', lw=1, alpha=0.7, label='实际速度')
ax.plot(t, vel_ref, 'g--', lw=1, alpha=0.7, label='速度参考(外环输出)')
ax.set_ylabel('幅值')
ax.set_title('串级PID — 最优参数下的详细响应')
ax.legend(loc='lower right', fontsize=9)
ax.grid(True, alpha=0.3)

ax = axes3[1]
ax.plot(t, u, 'm-', lw=1, label='控制量(内环输出)')
ax.set_xlabel('时间 (s)')
ax.set_ylabel('控制量')
ax.legend(loc='upper right', fontsize=9)
ax.grid(True, alpha=0.3)

plt.tight_layout()
path3 = os.path.join(out_dir, 'cascade_pid_detail_response.png')
fig3.savefig(path3, dpi=150, bbox_inches='tight')
print(f"图3已保存: {path3}")

plt.show()

# ── 调参指南文字总结 ──────────────────────────────────────
print("\n" + "=" * 65)
print("串级PID调参指南")
print("=" * 65)
print("""
步骤1：先调内环
  - 断开外环，给内环阶跃参考
  - 增大内环Kp直到出现振荡，取50%~70%的值
  - 适当加Ki消除稳态误差
  - 目标：内环响应时间 < 外环期望响应时间的1/5

步骤2：再调外环
  - 内环调好后，将内环视为"快速执行器"
  - 外环Kp从小到大，观察位置响应
  - 外环通常需要Ki（消除位置误差）
  - 外环Kd用于抑制超调

步骤3：带宽比优化
  - 内外环带宽比建议 5:1 ~ 10:1
  - 比值太小(<3)：内外环耦合，容易振荡
  - 比值太大(>20)：内环过快，可能放大噪声

步骤4：微调
  - 加入前馈：位置→速度前馈可加快响应
  - 加入滤波：D项低通滤波，避免噪声放大
  - 加入抗饱和：积分限幅或条件积分
""")
