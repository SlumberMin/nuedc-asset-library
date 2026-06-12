"""
ADRC+PID级联控制仿真 (Cascade ADRC-PID Simulation)
==============================================
外环: PID控制（位置/速度环）
内环: ADRC（自抗扰控制，含ESO扩张状态观测器）

典型应用: 电机控制（外环速度PID + 内环电流ADRC）

系统（二阶积分串联型 + 外扰）:
  ẋ₁ = x₂
  ẋ₂ = b₀*u + d(t)
  d(t) 为外部扰动
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams



def main():
    rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'STHeiti']
    rcParams['axes.unicode_minus'] = False

    # ============================================================
    # 仿真参数
    # ============================================================
    dt = 1e-4
    T_sim = 3.0
    N = int(T_sim / dt)
    t = np.arange(N) * dt

    # ============================================================
    # 被控对象: 双积分器 + 扰动
    # ============================================================
    b0 = 10.0  # 控制增益

    # 外部扰动: 阶跃 + 正弦
    def disturbance(t_val):
        d = 0.0
        if t_val > 1.0:
            d += 3.0  # 阶跃扰动
        d += 0.5 * np.sin(5.0 * t_val)  # 正弦扰动
        return d

    # ============================================================
    # ADRC 内环 (二阶)
    # ============================================================
    class ADRCInnerLoop:
        """二阶ADRC: ESO + NLSEF"""
        def __init__(self, dt, b0, omega_o=100, omega_c=50):
            self.dt = dt
            self.b0 = b0
            # ESO参数 (带宽 ω_o)
            self.beta1 = 3 * omega_o
            self.beta2 = 3 * omega_o**2
            self.beta3 = omega_o**3
            # 控制器参数 (带宽 ω_c)
            self.kp = omega_c**2
            self.kd = 2 * omega_c
            # ESO状态
            self.z1 = 0.0
            self.z2 = 0.0
            self.z3 = 0.0

        def update(self, y, u):
            """ESO更新 + 控制律"""
            # ESO (扩张状态观测器)
            e_obs = self.z1 - y
            self.z1 += self.dt * (self.z2 - self.beta1 * e_obs)
            self.z2 += self.dt * (self.z3 - self.beta2 * e_obs + self.b0 * u)
            self.z3 += self.dt * (-self.beta3 * e_obs)

        def control(self, ref, y):
            """ADRC控制律: u = (kp*(ref-z1) + kd*(0-z2) - z3) / b0"""
            e = ref - self.z1
            de = 0 - self.z2  # 参考信号导数近似为0
            u0 = self.kp * e + self.kd * de - self.z3
            u = u0 / self.b0
            return u

    # ============================================================
    # PID外环
    # ============================================================
    class PIDOuterLoop:
        def __init__(self, Kp, Ki, Kd, dt):
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd
            self.dt = dt
            self.e_int = 0.0
            self.e_prev = 0.0

        def control(self, ref, fb):
            e = ref - fb
            self.e_int += e * self.dt
            self.e_int = np.clip(self.e_int, -10, 10)  # 抗积分饱和
            de = (e - self.e_prev) / self.dt
            self.e_prev = e
            return self.Kp * e + self.Ki * self.e_int + self.Kd * de

    # ============================================================
    # 仿真运行
    # ============================================================
    def run_cascade_adrc_pid():
        """级联: 外环PID(位置) + 内环ADRC(速度/电流)"""
        x1 = np.zeros(N)  # 位置
        x2 = np.zeros(N)  # 速度
        u = np.zeros(N)

        # 位置参考: 梯形轨迹
        ref_pos = np.zeros(N)
        for k in range(N):
            if t[k] < 0.3:
                ref_pos[k] = 0.0
            elif t[k] < 1.0:
                ref_pos[k] = 5.0 * (t[k] - 0.3) / 0.7
            elif t[k] < 2.0:
                ref_pos[k] = 5.0
            elif t[k] < 2.5:
                ref_pos[k] = 5.0 - 5.0 * (t[k] - 2.0) / 0.5
            else:
                ref_pos[k] = 0.0

        # 外环PID输出作为内环ADRC的速度参考
        pid_outer = PIDOuterLoop(Kp=8.0, Ki=2.0, Kd=0.5, dt=dt)
        adrc_inner = ADRCInnerLoop(dt=dt, b0=b0, omega_o=80, omega_c=40)

        for k in range(N - 1):
            # 外环: PID输出 = 速度参考
            vel_ref = pid_outer.control(ref_pos[k], x1[k])
            vel_ref = np.clip(vel_ref, -10, 10)  # 限幅

            # 内环: ADRC跟踪速度参考
            u[k] = adrc_inner.control(vel_ref, x2[k])
            u[k] = np.clip(u[k], -50, 50)  # 控制量限幅

            # 系统更新
            d = disturbance(t[k])
            x1[k+1] = x1[k] + dt * x2[k]
            x2[k+1] = x2[k] + dt * (b0 * u[k] + d)

            # ESO更新
            adrc_inner.update(x2[k], u[k])

        return x1, x2, u, ref_pos

    def run_cascade_pid_pid():
        """级联PID-PID（对比）"""
        x1 = np.zeros(N)
        x2 = np.zeros(N)
        u = np.zeros(N)

        ref_pos = np.zeros(N)
        for k in range(N):
            if t[k] < 0.3:
                ref_pos[k] = 0.0
            elif t[k] < 1.0:
                ref_pos[k] = 5.0 * (t[k] - 0.3) / 0.7
            elif t[k] < 2.0:
                ref_pos[k] = 5.0
            elif t[k] < 2.5:
                ref_pos[k] = 5.0 - 5.0 * (t[k] - 2.0) / 0.5
            else:
                ref_pos[k] = 0.0

        pid_outer = PIDOuterLoop(Kp=8.0, Ki=2.0, Kd=0.5, dt=dt)
        pid_inner = PIDOuterLoop(Kp=20.0, Ki=50.0, Kd=0.1, dt=dt)

        for k in range(N - 1):
            vel_ref = pid_outer.control(ref_pos[k], x1[k])
            vel_ref = np.clip(vel_ref, -10, 10)
            u[k] = pid_inner.control(vel_ref, x2[k])
            u[k] = np.clip(u[k], -50, 50)

            d = disturbance(t[k])
            x1[k+1] = x1[k] + dt * x2[k]
            x2[k+1] = x2[k] + dt * (b0 * u[k] + d)

        return x1, x2, u, ref_pos

    # ============================================================
    # 运行仿真
    # ============================================================
    x1_adrc, x2_adrc, u_adrc, ref_pos = run_cascade_adrc_pid()
    x1_pid, x2_pid, u_pid, _ = run_cascade_pid_pid()

    # ============================================================
    # 性能指标
    # ============================================================
    def calc_pos_metrics(x1, ref, t_start=1.5):
        s = int(t_start / dt)
        e = x1[s:] - ref[s:]
        rmse = np.sqrt(np.mean(e**2))
        mae = np.mean(np.abs(e))
        max_e = np.max(np.abs(e))
        return rmse, mae, max_e

    rmse_adrc, mae_adrc, max_adrc = calc_pos_metrics(x1_adrc, ref_pos)
    rmse_pid, mae_pid, max_pid = calc_pos_metrics(x1_pid, ref_pos)

    print("=" * 60)
    print("    级联ADRC-PID vs 级联PID-PID — 性能对比")
    print("=" * 60)
    print(f"{'指标':<16} {'ADRC+PID':>14} {'PID+PID':>14}")
    print("-" * 60)
    print(f"{'RMSE(稳态+扰动)':<15} {rmse_adrc:>14.6f} {rmse_pid:>14.6f}")
    print(f"{'MAE (稳态+扰动)':<15} {mae_adrc:>14.6f} {mae_pid:>14.6f}")
    print(f"{'最大误差':<16} {max_adrc:>14.6f} {max_pid:>14.6f}")
    print("=" * 60)
    print(f"\n扰动说明: t>1s时叠加阶跃扰动(3.0)+持续正弦扰动(0.5sin(5t))")

    # ============================================================
    # 绘图
    # ============================================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('级联ADRC+PID vs 级联PID+PID — 扰动抑制', fontsize=15, fontweight='bold')

    ax = axes[0, 0]
    ax.plot(t, ref_pos, 'k--', lw=1.2, label='位置参考')
    ax.plot(t, x1_adrc, 'b-', lw=0.9, label='外环PID+内环ADRC')
    ax.plot(t, x1_pid, 'r-', lw=0.9, alpha=0.7, label='外环PID+内环PID')
    ax.axvline(x=1.0, color='green', ls=':', alpha=0.5, label='扰动开始')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('位置')
    ax.set_title('(a) 位置跟踪对比')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, x1_adrc - ref_pos, 'b-', lw=0.6, label='ADRC+PID误差')
    ax.plot(t, x1_pid - ref_pos, 'r-', lw=0.6, alpha=0.7, label='PID+PID误差')
    ax.axvline(x=1.0, color='green', ls=':', alpha=0.5)
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('位置误差')
    ax.set_title('(b) 位置跟踪误差')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(t, x2_adrc, 'b-', lw=0.6, label='ADRC+PID 速度')
    ax.plot(t, x2_pid, 'r-', lw=0.6, alpha=0.7, label='PID+PID 速度')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('速度')
    ax.set_title('(c) 速度响应')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t, u_adrc, 'b-', lw=0.6, label='ADRC+PID 控制量')
    ax.plot(t, u_pid, 'r-', lw=0.6, alpha=0.7, label='PID+PID 控制量')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('控制量')
    ax.set_title('(d) 控制量对比')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('sim_cascade_adrc_pid.png', dpi=150, bbox_inches='tight')
    print("图表已保存: sim_cascade_adrc_pid.png")



if __name__ == '__main__':
    main()
