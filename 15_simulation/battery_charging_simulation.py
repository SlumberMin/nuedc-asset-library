#!/usr/bin/env python3
"""电池充电仿真 — CC-CV / 脉冲充电 / 快充策略"""
import numpy as np, matplotlib.pyplot as plt, os
plt.rcParams['font.sans-serif'] = ['SimHei','Microsoft YaHei','DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 电池参数 (简化Rint模型)
Q_nom = 3.0       # Ah
R_int = 0.05      # 内阻 Ω
V_full = 4.2; V_cut = 3.0; V_nom = 3.7
dt = 1.0  # 1秒步长

def ocv_from_soc(soc):
    """开路电压-SOC曲线"""
    return 3.0 + 1.2*soc - 0.5*soc**2 + 0.3*soc**3

def simulate_cc_cv(I_cc, V_max=4.2, I_min_ratio=0.05):
    """CC-CV充电"""
    soc = np.zeros(100000); v_term = np.zeros(100000); i_ch = np.zeros(100000)
    soc[0] = 0.1; phase = ['CC']
    for k in range(1, len(soc)):
        ocv = ocv_from_soc(soc[k-1])
        v_batt = ocv + I_cc * R_int
        if v_batt >= V_max:
            I_cv = max((V_max - ocv) / R_int, 0)
            I_use = min(I_cv, I_cc)
            if I_use < I_cc * I_min_ratio:
                soc[k:] = soc[k-1]; v_term[k:] = V_max; i_ch[k:] = 0
                break
        else:
            I_use = I_cc
        i_ch[k] = I_use; v_term[k] = ocv + I_use * R_int
        soc[k] = soc[k-1] + I_use * dt / 3600 / Q_nom
        if soc[k] >= 0.999:
            soc[k] = 0.999; break
    n = k + 1
    return soc[:n], v_term[:n], i_ch[:n], np.arange(n)*dt/3600

def simulate_pulse(I_pulse, duty=0.5, freq=1.0):
    """脉冲充电"""
    soc = np.zeros(200000); v_term = np.zeros(200000); i_ch = np.zeros(200000)
    soc[0] = 0.1; period = int(1.0/freq/dt*1000)
    for k in range(1, len(soc)):
        in_pulse = (k % period) < (period * duty)
        I_use = I_pulse if in_pulse else 0
        ocv = ocv_from_soc(soc[k-1])
        i_ch[k] = I_use; v_term[k] = ocv + I_use * R_int
        soc[k] = soc[k-1] + I_use * dt / 3600 / Q_nom
        if soc[k] >= 0.999:
            soc[k] = 0.999; break
    n = k + 1
    return soc[:n], v_term[:n], i_ch[:n], np.arange(n)*dt/3600

def simulate_fast_charge():
    """多阶段快充"""
    soc = np.zeros(200000); v_term = np.zeros(200000); i_ch = np.zeros(200000)
    soc[0] = 0.05
    for k in range(1, len(soc)):
        s = soc[k-1]
        if s < 0.2:
            I_use = 0.5 * Q_nom  # 预充
        elif s < 0.7:
            I_use = 2.0 * Q_nom  # 快充CC
        elif s < 0.9:
            I_use = 1.0 * Q_nom  # 中速
        else:
            ocv = ocv_from_soc(s)
            I_use = max((4.2 - ocv) / R_int, 0)  # CV
            if I_use < 0.1 * Q_nom:
                soc[k:] = s; break
        ocv = ocv_from_soc(s)
        v_term[k] = ocv + I_use * R_int
        i_ch[k] = I_use
        soc[k] = s + I_use * dt / 3600 / Q_nom
        if soc[k] >= 0.999:
            soc[k] = 0.999; break
    n = k + 1
    return soc[:n], v_term[:n], i_ch[:n], np.arange(n)*dt/3600

# 仿真运行
r1 = simulate_cc_cv(1.5)
r2 = simulate_cc_cv(3.0)
r3 = simulate_pulse(3.0, duty=0.5)
r4 = simulate_fast_charge()

# 绘图
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('电池充电策略对比仿真', fontsize=14, fontweight='bold')
names = ['CC-CV (0.5C)', 'CC-CV (1C)', '脉冲充电 (1C)', '多阶段快充']
colors = ['#2196F3', '#4CAF50', '#FF9800', '#E91E63']
results = [r1, r2, r3, r4]

for i, (name, res, c) in enumerate(zip(names, results, colors)):
    soc, vt, ic, th = res
    axes[0,0].plot(th, soc*100, c, label=name)
    axes[0,1].plot(th, vt, c, label=name)
    axes[1,0].plot(th, ic, c, label=name)

axes[0,0].set_title('SOC曲线'); axes[0,0].set_ylabel('SOC (%)'); axes[0,0].legend(fontsize=8)
axes[0,1].set_title('端电压'); axes[0,1].set_ylabel('V (V)'); axes[0,1].axhline(4.2, ls='--', c='r', alpha=0.3)
axes[1,0].set_title('充电电流'); axes[1,0].set_ylabel('I (A)')

# 充电时间对比
ax = axes[1,1]
times = [r[3][-1] for r in results]
bars = ax.bar(names, times, color=colors)
ax.set_title('充电至100%时间对比'); ax.set_ylabel('时间 (h)')
for bar, tm in zip(bars, times):
    ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.02, f'{tm:.2f}h', ha='center', fontsize=9)

for ax in axes.flat[:3]:
    ax.set_xlabel('时间 (h)'); ax.grid(True, alpha=0.3)
plt.tight_layout()
out = os.path.join(os.path.dirname(__file__), 'battery_charging_simulation_result.png')
plt.savefig(out, dpi=150); print(f'已保存: {out}')
for name, tm in zip(names, times):
    print(f"{name}: 充电时间 = {tm:.2f} h")
