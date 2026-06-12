"""
采样数据系统仿真 - ZOH / 采样率 / 混叠 / 量化效应
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

# ========== 连续系统 ==========
def continuous_system_step(A, B, C, D, t, u=1.0):
    """连续系统阶跃响应"""
    n = A.shape[0]
    dt = t[1] - t[0]
    x = np.zeros((len(t), n))
    y = np.zeros(len(t))
    for i in range(1, len(t)):
        xdot = A @ x[i-1] + B.flatten() * u
        x[i] = x[i-1] + xdot * dt
        y[i] = (C @ x[i] + D.flatten() * u)[0]
    return y

# ========== 1. ZOH等效 vs 连续系统 ==========
def zoh_equivalence_demo():
    """ZOH离散化与连续系统对比"""
    # 二阶系统: G(s) = 1 / (s^2 + s + 1)
    num_c = [1]
    den_c = [1, 1, 1]
    sys_c = signal.TransferFunction(num_c, den_c)

    t_cont = np.linspace(0, 10, 1000)
    _, y_cont = signal.step(sys_c, T=t_cont)

    sampling_rates = [0.1, 0.5, 1.0, 2.0]
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('ZOH等效离散化对比', fontsize=14, fontweight='bold')

    for ax, Ts in zip(axes.flat, sampling_rates):
        # ZOH离散化
        sys_d = signal.cont2discrete((num_c, den_c), Ts, method='zoh')
        b_d, a_d, *_ = sys_d
        t_disc = np.arange(0, 10, Ts)
        _, y_disc = signal.dstep((b_d, a_d, Ts), n=len(t_disc))
        y_disc = y_disc[0].flatten()

        ax.plot(t_cont, y_cont, 'b-', linewidth=2, label='连续系统')
        ax.stem(t_disc, y_disc, linefmt='r-', markerfmt='ro', basefmt='k-', label=f'ZOH (Ts={Ts}s)')
        ax.set_title(f'采样周期 Ts = {Ts}s', fontsize=12)
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('输出')
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('zoh_equivalence_result.png', dpi=150)
    plt.close()
    print("✅ ZOH等效对比完成")

# ========== 2. 采样率对系统性能影响 ==========
def sampling_rate_effect():
    """不同采样率对控制系统的影响"""
    # 连续PID控制
    Kp, Ki, Kd = 2.0, 1.0, 0.5
    plant_num = [1]
    plant_den = [1, 2, 1]

    sampling_rates = [0.01, 0.05, 0.1, 0.2, 0.5]
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    fig.suptitle('采样率对闭环系统性能的影响', fontsize=14, fontweight='bold')

    settling_times = []
    overshoots = []

    for Ts in sampling_rates:
        # 离散化被控对象 (Tustin)
        sys_d_plant = signal.cont2discrete((plant_num, plant_den), Ts, method='bilinear')
        b_p, a_p, *_ = sys_d_plant
        b_p = b_p.flatten()
        a_p = a_p.flatten()

        # 离散PID (Tustin变换)
        # C(z) = Kp + Ki*Ts/(2*(z-1)/(z+1)) + Kd*(2*(z-1))/(Ts*(z+1))
        N = int(30 / Ts)
        y = np.zeros(N)
        r = np.ones(N)
        e = np.zeros(N)
        u = np.zeros(N)

        # 简化离散PID积分和微分
        ei = 0
        e_prev = 0
        u_buf = [0.0] * max(len(a_p), len(b_p))
        y_buf = [0.0] * max(len(a_p), len(b_p))

        for i in range(1, N):
            e[i] = r[i] - y[i-1]
            ei += e[i] * Ts
            ed = (e[i] - e_prev) / Ts
            u[i] = Kp * e[i] + Ki * ei + Kd * ed
            # 饱和
            u[i] = np.clip(u[i], -10, 10)
            # 系统响应 (差分方程)
            y_new = 0
            for j in range(1, len(a_p)):
                if i - j >= 0:
                    y_new -= a_p[j] * y[i-j]
            for j in range(len(b_p)):
                if i - j >= 0:
                    y_new += b_p[j] * u[i-j]
            y[i] = y_new
            e_prev = e[i]

        t = np.arange(N) * Ts
        axes[0].plot(t, y, linewidth=1.5, label=f'Ts={Ts}s')

        # 计算性能指标
        ss_val = y[-1] if len(y) > 0 else 1.0
        overshoot = max(0, (np.max(y) - 1.0) / 1.0 * 100)
        overshoots.append(overshoot)
        # 调节时间 (2%准则)
        ss_mask = np.abs(y - 1.0) > 0.02
        if np.any(ss_mask):
            st_idx = np.where(ss_mask)[0][-1]
            settling_times.append(t[st_idx] if st_idx < len(t) else t[-1])
        else:
            settling_times.append(0)

    axes[0].plot([0, 30], [1, 1], 'k--', alpha=0.3)
    axes[0].set_title('不同采样率的闭环阶跃响应', fontsize=12)
    axes[0].set_xlabel('时间 (s)')
    axes[0].set_ylabel('输出')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 性能指标柱状图
    x = np.arange(len(sampling_rates))
    width = 0.35
    axes[1].bar(x - width/2, overshoots, width, label='超调量 (%)', color='salmon')
    axes[1].bar(x + width/2, settling_times, width, label='调节时间 (s)', color='skyblue')
    axes[1].set_xticks(x)
    axes[1].set_xticklabels([f'Ts={s}' for s in sampling_rates])
    axes[1].set_title('采样率对性能指标的影响', fontsize=12)
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('sampling_rate_effect_result.png', dpi=150)
    plt.close()
    print("✅ 采样率影响分析完成")

# ========== 3. 混叠效应演示 ==========
def aliasing_demo():
    """奈奎斯特采样定理与混叠"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('采样混叠效应演示', fontsize=14, fontweight='bold')

    # 信号: 两个不同频率正弦波叠加
    f1, f2 = 5, 45  # Hz
    t_cont = np.linspace(0, 1, 10000)
    x_cont = np.sin(2*np.pi*f1*t_cont) + 0.5*np.sin(2*np.pi*f2*t_cont)

    # 频谱
    fft_cont = np.fft.rfft(x_cont)
    freq_cont = np.fft.rfftfreq(len(t_cont), t_cont[1]-t_cont[0])

    # 不同采样率
    fs_list = [100, 60, 30, 15]  # Hz

    for ax_idx, fs in enumerate(fs_list):
        ax = axes.flat[ax_idx]
        Ts = 1.0 / fs
        t_sample = np.arange(0, 1, Ts)
        x_sample = np.sin(2*np.pi*f1*t_sample) + 0.5*np.sin(2*np.pi*f2*t_sample)

        # 采样信号频谱
        fft_sample = np.fft.rfft(x_sample)
        freq_sample = np.fft.rfftfreq(len(t_sample), Ts)

        ax.plot(freq_cont, np.abs(fft_cont)/len(t_cont)*2, 'b-', linewidth=1, alpha=0.5, label='连续信号频谱')
        ax.stem(freq_sample, np.abs(fft_sample)/len(t_sample)*2, linefmt='r-', markerfmt='ro', basefmt='k-', label=f'采样 fs={fs}Hz')

        # 标注奈奎斯特频率
        nyq = fs / 2
        ax.axvline(nyq, color='green', linestyle='--', linewidth=2, label=f'奈奎斯特={nyq}Hz')
        ax.set_title(f'采样率 fs = {fs} Hz (f₂={f2}Hz {"会" if fs < 2*f2 else "不会"}混叠)',
                     fontsize=11, color='red' if fs < 2*f2 else 'green')
        ax.set_xlabel('频率 (Hz)')
        ax.set_ylabel('幅度')
        ax.set_xlim(0, 80)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('aliasing_demo_result.png', dpi=150)
    plt.close()
    print("✅ 混叠效应演示完成")

# ========== 4. 量化效应 ==========
def quantization_effect():
    """ADC量化对控制精度的影响"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('量化效应对控制精度的影响', fontsize=14, fontweight='bold')

    t = np.linspace(0, 10, 1000)
    signal_true = np.sin(2*np.pi*0.5*t) * 5  # 模拟信号

    bits_list = [16, 12, 8, 4]
    vref = 10.0  # 参考电压 ±10V

    for ax, bits in zip(axes.flat, bits_list):
        levels = 2**bits
        step_size = 2 * vref / levels

        # 量化
        signal_quant = np.round(signal_true / step_size) * step_size
        quant_error = signal_true - signal_quant

        ax.plot(t, signal_true, 'b-', linewidth=1, label='真实信号')
        ax.plot(t, signal_quant, 'r.', markersize=1, label=f'{bits}bit量化')
        ax.fill_between(t, signal_true - step_size/2, signal_true + step_size/2,
                        alpha=0.2, color='red', label=f'量化步长={step_size*1000:.2f}mV')
        ax.set_title(f'{bits}位ADC (分辨率: {levels}级, SNR≈{6.02*bits+1.76:.0f}dB)', fontsize=11)
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('电压 (V)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('quantization_effect_result.png', dpi=150)
    plt.close()

    # 量化位数 vs 信噪比
    fig, ax = plt.subplots(figsize=(10, 6))
    bits_range = np.arange(1, 25)
    snr_db = 6.02 * bits_range + 1.76
    enob = (snr_db - 1.76) / 6.02

    ax.plot(bits_range, snr_db, 'bo-', linewidth=2, markersize=8, label='理论SNR')
    ax.set_xlabel('量化位数 (bits)')
    ax.set_ylabel('信噪比 SNR (dB)')
    ax.set_title('ADC量化位数与信噪比关系', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()

    for b in [8, 10, 12, 16]:
        ax.annotate(f'{b}bit: {6.02*b+1.76:.0f}dB', xy=(b, 6.02*b+1.76),
                    fontsize=10, ha='left', va='bottom',
                    arrowprops=dict(arrowstyle='->', color='gray'))

    plt.tight_layout()
    plt.savefig('quantization_snr_result.png', dpi=150)
    plt.close()
    print("✅ 量化效应分析完成")

# ========== 主程序 ==========
if __name__ == '__main__':
    print("=" * 60)
    print("  采样数据系统仿真 - V3迭代")
    print("=" * 60)
    zoh_equivalence_demo()
    sampling_rate_effect()
    aliasing_demo()
    quantization_effect()
    print("\n✅ 所有采样数据系统仿真完成!")
