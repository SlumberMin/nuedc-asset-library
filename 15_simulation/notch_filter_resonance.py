# -*- coding: utf-8 -*-
"""
陷波滤波器谐振抑制仿真
=====================
使用陷波滤波器抑制机械谐振峰，对比滤波前后效果
应用场景: 电机驱动系统中的谐振抑制
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os



def main():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    # ========== 系统参数 ==========
    fs = 10000   # 采样率 10kHz
    dt = 1.0 / fs
    T = 0.5
    t = np.arange(0, T, dt)
    N = len(t)

    # ========== 含谐振的系统传递函数 ==========
    # 系统: 一阶惯性 + 谐振峰
    # G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2) * 1/(taus*s + 1)
    # 谐振频率 500Hz, 阻尼比0.05 (尖锐谐振)
    f_res = 500  # 谐振频率 Hz
    w_res = 2 * np.pi * f_res
    zeta = 0.05  # 小阻尼比 -> 尖锐谐振峰

    # 频率响应计算
    f_axis = np.linspace(10, fs/2, 5000)
    w_axis = 2 * np.pi * f_axis

    # 谐振环节频率响应
    H_res = w_res**2 / (-w_axis**2 + 2j*zeta*w_res*w_axis + w_res**2)
    # 低通环节 (截止100Hz)
    tau = 1 / (2*np.pi*100)
    H_lp = 1 / (1j*w_axis*tau + 1)
    # 总系统
    H_total = H_res * H_lp

    # ========== 设计陷波滤波器 ==========
    # 陷波滤波器: H(z) = (1 - 2*cos(w0)*z^-1 + z^-2) / (1 - 2*r*cos(w0)*z^-1 + r^2*z^-2)
    # r 接近1时陷波越窄
    w0_digital = 2 * np.pi * f_res / fs  # 数字角频率
    r = 0.98  # 陷波窄度

    b_notch = [1, -2*np.cos(w0_digital), 1]
    a_notch = [1, -2*r*np.cos(w0_digital), r**2]

    # 陷波滤波器频率响应
    z = np.exp(1j * w_axis * dt)
    H_notch = (b_notch[0] + b_notch[1]*z**(-1) + b_notch[2]*z**(-2)) / \
              (a_notch[0] + a_notch[1]*z**(-1) + a_notch[2]*z**(-2))

    # 不同阻尼的陷波滤波器
    r_values = [0.95, 0.98, 0.995]

    # ========== 时域信号仿真 ==========
    # 输入: 白噪声 + 谐振频率正弦
    np.random.seed(42)
    x_in = np.random.randn(N) * 0.1 + 0.5 * np.sin(2*np.pi*f_res*t)

    # 通过系统 (谐振环节) - 使用二阶差分方程
    # H(s) = w_res^2 / (s^2 + 2*zeta*w_res*s + w_res^2)
    # 双线性变换离散化
    y_resonant = np.zeros(N)
    x1_r = x2_r = 0.0
    for i in range(N):
        y_resonant[i] = x1_r
        # 状态空间: x1_dot = x2, x2_dot = -w_res^2*x1 - 2*zeta*w_res*x2 + w_res^2*u
        x2_dot = -w_res**2 * x1_r - 2*zeta*w_res * x2_r + w_res**2 * x_in[i]
        x1_dot = x2_r
        # 使用较小的有效步长防止数值问题
        effective_dt = min(dt, 1e-6)
        steps = max(1, int(dt / effective_dt))
        for _ in range(steps):
            x2_r += x2_dot * effective_dt
            x1_r += x1_dot * effective_dt
        # 限幅防止发散
        x1_r = np.clip(x1_r, -10, 10)
        x2_r = np.clip(x2_r, -1e6, 1e6)

    # 陷波滤波
    def apply_filter(signal, b, a):
        """IIR滤波器实现"""
        y = np.zeros(len(signal))
        for i in range(len(signal)):
            y[i] = b[0] * signal[i]
            for j in range(1, len(b)):
                if i - j >= 0:
                    y[i] += b[j] * signal[i-j] - a[j] * y[i-j]
        return y

    y_filtered = apply_filter(y_resonant, b_notch, a_notch)

    # ========== 性能指标 ==========
    # 谐振频率处的衰减
    H_total_mag = 20 * np.log10(np.abs(H_total) + 1e-10)
    H_filtered_mag = 20 * np.log10(np.abs(H_total * H_notch) + 1e-10)
    res_idx = np.argmin(np.abs(f_axis - f_res))

    print("=== 陷波滤波器谐振抑制性能 ===")
    print(f"  谐振频率: {f_res} Hz")
    print(f"  谐振峰增益: {H_total_mag[res_idx]:.1f} dB")
    print(f"  滤波后增益: {H_filtered_mag[res_idx]:.1f} dB")
    print(f"  谐振抑制: {H_total_mag[res_idx] - H_filtered_mag[res_idx]:.1f} dB")

    # 时域RMS
    rms_before = np.sqrt(np.mean(y_resonant**2))
    rms_after = np.sqrt(np.mean(y_filtered**2))
    print(f"  时域RMS (滤波前): {rms_before:.4f}")
    print(f"  时域RMS (滤波后): {rms_after:.4f}")

    # ========== 绘图 ==========
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 频率响应 - 原始系统
    axes[0, 0].semilogx(f_axis, H_total_mag, 'b-', linewidth=1.5, label='含谐振系统')
    axes[0, 0].axvline(f_res, color='r', linestyle='--', alpha=0.5, label=f'谐振频率 {f_res}Hz')
    axes[0, 0].set_ylabel('增益 (dB)')
    axes[0, 0].set_title('原始系统频率响应（含谐振峰）')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_xlim([10, 2000])

    # 陷波滤波器特性 (不同阻尼)
    for r_val in r_values:
        b_n = [1, -2*np.cos(w0_digital), 1]
        a_n = [1, -2*r_val*np.cos(w0_digital), r_val**2]
        z = np.exp(1j * w_axis * dt)
        H_n = (b_n[0] + b_n[1]*z**(-1) + b_n[2]*z**(-2)) / \
              (a_n[0] + a_n[1]*z**(-1) + a_n[2]*z**(-2))
        axes[0, 1].semilogx(f_axis, 20*np.log10(np.abs(H_n)+1e-10), linewidth=1.2, label=f'r={r_val}')
    axes[0, 1].set_ylabel('增益 (dB)')
    axes[0, 1].set_title('不同阻尼参数的陷波滤波器')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_xlim([200, 2000])
    axes[0, 1].set_ylim([-40, 5])

    # 滤波前后频率响应对比
    axes[1, 0].semilogx(f_axis, H_total_mag, 'b-', linewidth=1.2, label='滤波前')
    axes[1, 0].semilogx(f_axis, H_filtered_mag, 'r-', linewidth=1.2, label='滤波后')
    axes[1, 0].axvline(f_res, color='gray', linestyle='--', alpha=0.5)
    axes[1, 0].set_xlabel('频率 (Hz)')
    axes[1, 0].set_ylabel('增益 (dB)')
    axes[1, 0].set_title('谐振抑制前后频率响应对比')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_xlim([10, 2000])

    # 时域波形对比
    t_show = t[:2000]
    axes[1, 1].plot(t_show*1000, y_resonant[:2000], 'b-', linewidth=0.8, alpha=0.7, label='滤波前')
    axes[1, 1].plot(t_show*1000, y_filtered[:2000], 'r-', linewidth=0.8, alpha=0.7, label='滤波后')
    axes[1, 1].set_xlabel('时间 (ms)')
    axes[1, 1].set_ylabel('幅值')
    axes[1, 1].set_title('时域波形对比')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'notch_filter_resonance.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"\n已保存: {out}")



if __name__ == '__main__':
    main()
