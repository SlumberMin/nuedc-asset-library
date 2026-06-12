#!/usr/bin/env python3
"""数字滤波器对比仿真 — FIR / IIR / 自适应 / 卡尔曼"""
import numpy as np, matplotlib.pyplot as plt, os
plt.rcParams['font.sans-serif'] = ['SimHei','Microsoft YaHei','DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

np.random.seed(42)
fs = 1000; t = np.arange(0, 1, 1/fs)
# 纯净信号 + 噪声
clean = np.sin(2*np.pi*5*t) + 0.5*np.sin(2*np.pi*50*t)
noise = np.random.randn(len(t)) * 0.8
x = clean + noise

# ── FIR (窗函数法 50阶低通) ──
from numpy.fft import fft, ifft
n_fir = 51; fc = 30
k = np.arange(n_fir)
h_fir = np.sinc(2*fc/fs*(k-(n_fir-1)/2)) * np.blackman(n_fir)
h_fir /= h_fir.sum()
fir_out = np.convolve(x, h_fir, mode='same')

# ── IIR (双二阶 Butterworth 低通) ──
from scipy.signal import butter, filtfilt
b_iir, a_iir = butter(4, 30/(fs/2), btype='low')
iir_out = filtfilt(b_iir, a_iir, x)

# ── LMS 自适应滤波 ──
mu = 0.01; n_lms = 32
w = np.zeros(n_lms); lms_out = np.zeros_like(x)
ref = np.sin(2*np.pi*5*t)  # 参考信号
for i in range(n_lms, len(x)):
    xvec = ref[i-n_lms:i][::-1]
    y_hat = w @ xvec
    e = x[i] - y_hat
    w += 2 * mu * e * xvec
    lms_out[i] = y_hat

# ── 卡尔曼滤波 ──
Q_k = 1e-4; R_k = 0.64
x_est = np.zeros_like(x); P = 1.0
for i in range(len(x)):
    x_pred = x_est[i-1] if i > 0 else 0
    P_pred = P + Q_k
    K = P_pred / (P_pred + R_k)
    x_est[i] = x_pred + K * (x[i] - x_pred)
    P = (1 - K) * P_pred

# ── 性能指标 ──
def snr_metric(clean, filt):
    e = clean - filt
    return 10*np.log10(np.sum(clean**2)/(np.sum(e**2)+1e-12))

filters = {'FIR': fir_out, 'IIR': iir_out, 'LMS自适应': lms_out, '卡尔曼': x_est}
metrics = {k: snr_metric(clean, v) for k, v in filters.items()}

# ── 绘图 ──
fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle('数字滤波器对比仿真', fontsize=14, fontweight='bold')
axes[0,0].plot(t, x, alpha=0.5, label='含噪信号'); axes[0,0].plot(t, clean, 'k', label='纯净信号')
axes[0,0].set_title('原始信号'); axes[0,0].legend(fontsize=8); axes[0,0].set_xlabel('t (s)')

for ax, (name, filt) in zip(axes.flat[1:5], filters.items()):
    ax.plot(t, clean, 'k--', alpha=0.4, label='纯净')
    ax.plot(t, filt, 'r', label=name)
    ax.set_title(f'{name}  SNR={metrics[name]:.1f} dB'); ax.legend(fontsize=8); ax.set_xlabel('t (s)')

axes[2,1].bar(metrics.keys(), metrics.values(), color=['#2196F3','#4CAF50','#FF9800','#E91E63'])
axes[2,1].set_title('输出SNR对比 (dB)'); axes[2,1].set_ylabel('SNR (dB)')
for i, (k,v) in enumerate(metrics.items()):
    axes[2,1].text(i, v+0.3, f'{v:.1f}', ha='center', fontsize=9)

plt.tight_layout()
out = os.path.join(os.path.dirname(__file__), 'digital_filter_comparison_result.png')
plt.savefig(out, dpi=150); print(f'已保存: {out}')
print('SNR对比:', {k: f'{v:.1f}dB' for k,v in metrics.items()})
