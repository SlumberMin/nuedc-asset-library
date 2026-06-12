"""
PID自整定仿真
方法1：Ziegler-Nichols临界比例法
方法2：继电反馈法（Relay Feedback / Åström-Hägglund）
应用：自动获取PID参数 Kp, Ki, Kd
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt



def main():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False

    # ============================================================
    # 被控对象：二阶系统 G(s) = K / ((tau1*s+1)*(tau2*s+1))
    # ============================================================
    K_plant = 2.0
    tau1 = 0.3
    tau2 = 0.1

    dt = 0.001
    t_end = 5.0
    t = np.arange(0, t_end, dt)

    def simulate_plant(u_history, dt):
        """模拟被控对象响应"""
        n = len(u_history)
        y = np.zeros(n)
        x1, x2 = 0.0, 0.0
        for i in range(1, n):
            x1 += dt / tau1 * (-x1 + K_plant * u_history[i-1])
            x2 += dt / tau2 * (-x2 + x1)
            y[i] = x2
        return y

    # ============================================================
    # 方法1：Ziegler-Nichols临界比例法
    # ============================================================
    def find_critical_gain(Kp_start=0.5, Kp_max=20.0, step=0.1):
        """逐步增大比例增益直到系统等幅振荡"""
        setpoint = np.ones(len(t))
        results = []

        for Kp in np.arange(Kp_start, Kp_max, step):
            y = np.zeros(len(t))
            u = np.zeros(len(t))
            for i in range(1, len(t)):
                error = setpoint[i] - y[i-1]
                u[i] = Kp * error
                # 限制输出
                u[i] = np.clip(u[i], -10, 10)
                # 一阶近似模拟
                y[i] = y[i-1] + dt/tau1 * (-y[i-1] + K_plant * u[i])
                y[i] = y[i] + dt/tau2 * (-y[i] + K_plant * u[i])

            # 检查后半段是否等幅振荡
            y_tail = y[len(t)//2:]
            peaks = []
            for j in range(1, len(y_tail)-1):
                if y_tail[j] > y_tail[j-1] and y_tail[j] > y_tail[j+1]:
                    peaks.append(y_tail[j])

            if len(peaks) >= 3:
                peak_var = np.var(peaks[-4:]) if len(peaks) >= 4 else np.var(peaks)
                peak_mean = np.mean(peaks[-4:]) if len(peaks) >= 4 else np.mean(peaks)
                # 判断是否接近等幅振荡
                if peak_var < 0.01 and 0.3 < peak_mean < 3.0:
                    # 计算振荡周期
                    peak_times = []
                    for j in range(1, len(y_tail)-1):
                        if y_tail[j] > y_tail[j-1] and y_tail[j] > y_tail[j+1]:
                            peak_times.append(j * dt)
                    if len(peak_times) >= 2:
                        Tu = np.mean(np.diff(peak_times))
                        return Kp, Tu, True

        return None, None, False

    # 简化：直接用已知参数计算临界增益和周期（解析法）
    # 对于 G(s) = 2 / ((0.3s+1)(0.1s+1))，可计算临界点
    # 令相位=-180°: -arctan(0.3w) - arctan(0.1w) = -π
    # 解得 w_u ≈ 5.77 rad/s, K_u = 1/|G(jw_u)| ≈ ...

    # 用仿真法找临界增益
    print("=" * 60)
    print("方法1: Ziegler-Nichols 临界比例法")
    print("=" * 60)

    # 仿真法扫描
    best_Kp = None
    best_Tu = None

    for Kp in np.arange(0.5, 15.0, 0.05):
        y = np.zeros(len(t))
        x1, x2 = 0.0, 0.0
        for i in range(1, len(t)):
            error = 1.0 - y[i-1]
            u_val = np.clip(Kp * error, -10, 10)
            x1 += dt / tau1 * (-x1 + K_plant * u_val)
            x2 += dt / tau2 * (-x2 + x1)
            y[i] = x2

        # 检查振荡
        y_tail = y[int(2.0/dt):]
        peak_idx = []
        for j in range(1, len(y_tail)-1):
            if y_tail[j] > y_tail[j-1] and y_tail[j] > y_tail[j+1]:
                peak_idx.append(j)

        if len(peak_idx) >= 4:
            peak_vals = y_tail[peak_idx[-4:]]
            if np.std(peak_vals) / (np.mean(peak_vals) + 1e-10) < 0.05:
                Tu = (peak_idx[-1] - peak_idx[-3]) * 2 * dt  # 2个周期
                best_Kp = Kp
                best_Tu = Tu
                break

    if best_Kp is not None:
        Ku = best_Kp
        Tu = best_Tu
        print(f"  临界增益 Ku = {Ku:.2f}")
        print(f"  临界周期 Tu = {Tu:.3f} s")

        # Ziegler-Nichols整定公式
        zn_params = {
            'P':   (0.5 * Ku, 0, 0),
            'PI':  (0.45 * Ku, 0.45 * Ku / (Tu/1.2), 0),
            'PID': (0.6 * Ku, 0.6 * Ku / (Tu/2), 0.6 * Ku * Tu/8),
        }
        for name, (kp, ki, kd) in zn_params.items():
            print(f"  {name}: Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}")
    else:
        print("  未找到临界振荡点，使用估算值")
        Ku, Tu = 5.0, 1.1
        zn_params = {
            'P':   (0.5 * Ku, 0, 0),
            'PI':  (0.45 * Ku, 0.45 * Ku / (Tu/1.2), 0),
            'PID': (0.6 * Ku, 0.6 * Ku / (Tu/2), 0.6 * Ku * Tu/8),
        }

    # ============================================================
    # 方法2：继电反馈法（Relay Feedback）
    # ============================================================
    print("\n" + "=" * 60)
    print("方法2: 继电反馈法（Relay Feedback）")
    print("=" * 60)

    d_relay = 1.0   # 继电器幅值
    hysteresis = 0.1 # 滞环宽度

    y_relay = np.zeros(len(t))
    u_relay = np.zeros(len(t))
    relay_state = 1

    x1, x2 = 0.0, 0.0
    for i in range(1, len(t)):
        # 带滞环的继电器
        if relay_state == 1 and y_relay[i-1] > hysteresis:
            relay_state = -1
        elif relay_state == -1 and y_relay[i-1] < -hysteresis:
            relay_state = 1

        u_relay[i] = relay_state * d_relay
        x1 += dt / tau1 * (-x1 + K_plant * u_relay[i])
        x2 += dt / tau2 * (-x2 + x1)
        y_relay[i] = x2

    # 从稳态段提取振荡参数
    y_steady = y_relay[int(1.0/dt):]
    peak_indices = []
    for j in range(1, len(y_steady)-1):
        if y_steady[j] > y_steady[j-1] and y_steady[j] > y_steady[j+1]:
            peak_indices.append(j)

    if len(peak_indices) >= 2:
        Tu_relay = (peak_indices[-1] - peak_indices[-2]) * dt * 2  # 周期
        a_relay = np.mean([y_steady[p] for p in peak_indices[-3:]])  # 振幅
        # 继电反馈法整定公式
        Ku_relay = 4 * d_relay / (np.pi * a_relay)
        print(f"  振荡周期 Tu = {Tu_relay:.3f} s")
        print(f"  振荡幅值 a = {a_relay:.3f}")
        print(f"  临界增益 Ku = {Ku_relay:.3f}")

        rf_params = {
            'P':   (0.5 * Ku_relay, 0, 0),
            'PI':  (0.45 * Ku_relay, 0.45 * Ku_relay / (Tu_relay/1.2), 0),
            'PID': (0.6 * Ku_relay, 0.6 * Ku_relay / (Tu_relay/2), 0.6 * Ku_relay * Tu_relay/8),
        }
        for name, (kp, ki, kd) in rf_params.items():
            print(f"  {name}: Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}")

    # ============================================================
    # 用整定参数进行PID控制仿真对比
    # ============================================================
    def pid_simulate(Kp, Ki, Kd, setpoint, dt):
        """PID控制仿真"""
        n = len(setpoint)
        y = np.zeros(n)
        u = np.zeros(n)
        integral = 0.0
        prev_error = 0.0
        x1, x2 = 0.0, 0.0

        for i in range(1, n):
            error = setpoint[i] - y[i-1]
            integral += error * dt
            derivative = (error - prev_error) / dt
            u[i] = Kp * error + Ki * integral + Kd * derivative
            u[i] = np.clip(u[i], -10, 10)
            prev_error = error

            x1 += dt / tau1 * (-x1 + K_plant * u[i])
            x2 += dt / tau2 * (-x2 + x1)
            y[i] = x2

        return y, u

    setpoint = np.ones(len(t))
    setpoint[int(2.5/dt):] = 0.5  # 2.5s时设定值变化

    # ============================================================
    # 绘图
    # ============================================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 图1：继电反馈信号
    ax = axes[0, 0]
    t_plot = t[:int(3.0/dt)]
    ax.plot(t_plot, y_relay[:len(t_plot)], 'b-', label='输出')
    ax.plot(t_plot, u_relay[:len(t_plot)] * 0.3, 'r-', alpha=0.5, label='继电器输入(缩放)')
    ax.axhline(hysteresis, color='g', linestyle='--', alpha=0.5)
    ax.axhline(-hysteresis, color='g', linestyle='--', alpha=0.5)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('幅值')
    ax.set_title('继电反馈法自整定过程')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 图2：不同PID参数的阶跃响应对比
    ax = axes[0, 1]
    for name, (kp, ki, kd) in zn_params.items():
        y_pid, _ = pid_simulate(kp, ki, kd, setpoint, dt)
        ax.plot(t, y_pid, linewidth=1.5, label=f'{name}: Kp={kp:.2f}')
    ax.plot(t, setpoint, 'k--', linewidth=1, label='设定值')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.set_title('Ziegler-Nichols整定效果对比')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 4])

    # 图3：继电反馈法整定效果对比
    ax = axes[1, 0]
    if len(peak_indices) >= 2:
        for name, (kp, ki, kd) in rf_params.items():
            y_pid, _ = pid_simulate(kp, ki, kd, setpoint, dt)
            ax.plot(t, y_pid, linewidth=1.5, label=f'{name}: Kp={kp:.2f}')
        ax.plot(t, setpoint, 'k--', linewidth=1, label='设定值')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.set_title('继电反馈法整定效果对比')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 4])

    # 图4：控制量对比
    ax = axes[1, 1]
    kp_zn, ki_zn, kd_zn = zn_params['PID']
    y_zn, u_zn = pid_simulate(kp_zn, ki_zn, kd_zn, setpoint, dt)
    if len(peak_indices) >= 2:
        kp_rf, ki_rf, kd_rf = rf_params['PID']
        y_rf, u_rf = pid_simulate(kp_rf, ki_rf, kd_rf, setpoint, dt)
        ax.plot(t, u_rf, 'g-', linewidth=1, label='继电反馈法')
    ax.plot(t, u_zn, 'b-', linewidth=1, label='Z-N法')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制量')
    ax.set_title('PID控制量对比')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 4])

    plt.suptitle('PID自整定仿真（Ziegler-Nichols + 继电反馈法）', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pid_auto_tuning_sim.png'), dpi=150, bbox_inches='tight')
    print("\n图表已保存: pid_auto_tuning_sim.png")
    plt.close('all')



if __name__ == '__main__':
    main()
