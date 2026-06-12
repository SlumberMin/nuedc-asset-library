#!/usr/bin/env python3
"""电机制动仿真 — 能耗制动 / 回馈制动 / 反接制动"""
import numpy as np, matplotlib.pyplot as plt, os
plt.rcParams['font.sans-serif'] = ['SimHei','Microsoft YaHei','DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 电机参数
J = 0.01       # 转动惯量 kg·m²
B = 0.001      # 摩擦系数
Kt = 0.1       # 转矩常数 Nm/A
Ke = 0.1       # 反电势常数 V/(rad/s)
R = 1.0        # 电枢电阻 Ω
V_bus = 48     # 母线电压 V
omega0 = 300   # 初始角速度 rad/s
dt = 0.001; T = 2.0
t = np.arange(0, T, dt)
N = len(t)

def simulate_braking(brake_fn):
    omega = np.zeros(N); omega[0] = omega0
    i_a = np.zeros(N); T_brake = np.zeros(N)
    P_brake = np.zeros(N); P_regen = np.zeros(N)
    for k in range(N-1):
        i_cmd, mode = brake_fn(omega[k], t[k])
        e_back = Ke * omega[k]
        if mode == 'resistive':  # 能耗制动：电流经电阻
            v_r = min(e_back, V_bus)
            i_a[k] = v_r / R
            T_brake[k] = -Kt * i_a[k]
            P_brake[k] = i_a[k]**2 * R
        elif mode == 'regen':  # 回馈制动：能量返回母线
            i_a[k] = i_cmd
            T_brake[k] = -Kt * abs(i_a[k])
            P_regen[k] = e_back * abs(i_a[k])
            P_brake[k] = 0
        elif mode == 'plugging':  # 反接制动：施加反向电压
            v_applied = -V_bus
            i_a[k] = (v_applied - e_back) / R
            T_brake[k] = Kt * i_a[k]  # 可能为负(制动)
            P_brake[k] = i_a[k]**2 * R
        # 运动方程
        d_omega = (T_brake[k] - B * omega[k]) * dt / J
        omega[k+1] = max(omega[k] + d_omega, 0)
        if omega[k+1] <= 0.1:
            omega[k+1:] = 0; break
    return omega, T_brake, P_brake, P_regen, i_a

# 能耗制动
def brake_resistive(w, _t):
    return (0, 'resistive')
# 回馈制动 (限流)
def brake_regen(w, _t):
    i_max = 5.0
    return (min(i_max, w * Ke / R), 'regen')
# 反接制动
def brake_plugging(w, _t):
    return (0, 'plugging')

results = {}
for name, fn in [('能耗制动', brake_resistive), ('回馈制动', brake_regen), ('反接制动', brake_plugging)]:
    omega, Tb, Pb, Pr, ia = simulate_braking(fn)
    settle_idx = np.argmax(omega <= 0.1) if np.any(omega <= 0.1) else N-1
    t_stop = t[settle_idx]
    energy_dissipated = np.sum(Pb) * dt
    energy_recovered = np.sum(Pr) * dt
    results[name] = dict(omega=omega, Tb=Tb, Pb=Pb, Pr=Pr, ia=ia,
                         t_stop=t_stop, E_diss=energy_dissipated, E_rec=energy_recovered)

# ── 绘图 ──
fig, axes = plt.subplots(2, 3, figsize=(16, 9))
fig.suptitle('电机制动方式对比仿真', fontsize=14, fontweight='bold')
colors = {'能耗制动':'#E91E63', '回馈制动':'#2196F3', '反接制动':'#FF9800'}

for name, res in results.items():
    c = colors[name]
    axes[0,0].plot(t, res['omega'], c, label=name, alpha=0.8)
    axes[0,1].plot(t, res['Tb'], c, label=name, alpha=0.8)
    axes[0,2].plot(t, res['ia'], c, label=name, alpha=0.8)
    axes[1,0].plot(t, res['Pb'], c, label=f"{name}(耗散)", alpha=0.8)
    axes[1,1].plot(t, res['Pr'], c, label=f"{name}(回收)", alpha=0.8)

axes[0,0].set_title('转速响应'); axes[0,0].set_ylabel('ω (rad/s)'); axes[0,0].legend(fontsize=8)
axes[0,1].set_title('制动转矩'); axes[0,1].set_ylabel('T (Nm)'); axes[0,1].legend(fontsize=8)
axes[0,2].set_title('电枢电流'); axes[0,2].set_ylabel('I (A)'); axes[0,2].legend(fontsize=8)
axes[1,0].set_title('耗散功率'); axes[1,0].set_ylabel('P (W)'); axes[1,0].legend(fontsize=8)
axes[1,1].set_title('回收功率'); axes[1,1].set_ylabel('P (W)'); axes[1,1].legend(fontsize=8)

# 汇总表
ax = axes[1,2]; ax.axis('off')
cell_text = [[name, f"{res['t_stop']:.3f}s", f"{res['E_diss']:.1f}J", f"{res['E_rec']:.1f}J"]
             for name, res in results.items()]
table = ax.table(cellText=cell_text, colLabels=['制动方式','停机时间','耗散能量','回收能量'],
                 loc='center', cellLoc='center')
table.auto_set_font_size(False); table.set_fontsize(10); table.scale(1.2, 1.5)
ax.set_title('制动性能对比')

for ax in axes.flat[:5]:
    ax.set_xlabel('t (s)'); ax.grid(True, alpha=0.3)
plt.tight_layout()
out = os.path.join(os.path.dirname(__file__), 'motor_braking_simulation_result.png')
plt.savefig(out, dpi=150); print(f'已保存: {out}')
for name, res in results.items():
    print(f"{name}: 停机={res['t_stop']:.3f}s, 耗散={res['E_diss']:.1f}J, 回收={res['E_rec']:.1f}J")
