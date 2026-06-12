#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通用仿真结果可视化工具
============================================
支持多种图表类型，可加载CSV/JSON数据，适合电赛仿真结果展示
图表类型: 时域响应、频域分析、相平面、阶跃响应指标、3D曲面、热力图等
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import json
import os
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class SimVisualizer:
    """仿真结果可视化工具"""

    def __init__(self, data=None):
        """
        data: dict, key为信号名, value为numpy数组
              必须包含 't' (时间轴)
        """
        self.data = data or {}
        self.fig = None

    def load_csv(self, filepath, delimiter=','):
        """从CSV加载数据 (第一列为时间)"""
        try:
            raw = np.loadtxt(filepath, delimiter=delimiter, skiprows=1)
            headers = []
            with open(filepath, 'r', encoding='utf-8') as f:
                headers = f.readline().strip().split(delimiter)
            self.data['t'] = raw[:, 0]
            for i, h in enumerate(headers[1:], 1):
                self.data[h.strip()] = raw[:, i]
            print(f"已加载 {filepath}: {len(headers)-1} 个信号, {len(raw)} 个数据点")
        except Exception as e:
            print(f"加载失败: {e}")

    def load_json(self, filepath):
        """从JSON加载数据"""
        with open(filepath, 'r', encoding='utf-8') as f:
            raw = json.load(f)
        for k, v in raw.items():
            self.data[k] = np.array(v)
        print(f"已加载 {filepath}: {len(self.data)} 个信号")

    def add_signal(self, name, values, t=None):
        """手动添加信号"""
        if t is not None:
            self.data['t'] = t
        self.data[name] = np.array(values)

    def generate_demo_data(self):
        """生成演示数据"""
        t = np.linspace(0, 5, 1000)
        self.data['t'] = t
        self.data['PID响应'] = 1 - np.exp(-2*t) * np.cos(3*t) - 0.3*np.exp(-2*t)*np.sin(3*t)
        self.data['模糊PID响应'] = 1 - np.exp(-3*t) * np.cos(4*t) - 0.15*np.exp(-3*t)*np.sin(4*t)
        self.data['参考信号'] = np.ones_like(t)
        self.data['误差PID'] = self.data['参考信号'] - self.data['PID响应']
        self.data['误差模糊PID'] = self.data['参考信号'] - self.data['模糊PID响应']
        self.data['控制量PID'] = 0.5 * np.exp(-t) + 0.3 * np.sin(5*t) * np.exp(-0.5*t) + 0.2
        self.data['控制量模糊PID'] = 0.4 * np.exp(-1.5*t) + 0.2 * np.sin(6*t) * np.exp(-t) + 0.2
        print("已生成演示数据")

    def plot_time_response(self, signals=None, title='时域响应', ylabel='幅值',
                           show_metrics=True):
        """时域响应图"""
        if signals is None:
            signals = [k for k in self.data if k != 't']

        t = self.data['t']
        fig, axes = plt.subplots(2 if show_metrics else 1, 1,
                                 figsize=(12, 4 * (2 if show_metrics else 1)),
                                 sharex=True)
        if not show_metrics:
            axes = [axes]

        for s in signals:
            axes[0].plot(t, self.data[s], linewidth=1.2, label=s)
        axes[0].set_ylabel(ylabel)
        axes[0].set_title(title)
        axes[0].legend(loc='best')
        axes[0].grid(True, alpha=0.3)

        if show_metrics:
            # 计算误差 (假设第一个信号为参考)
            ref_name = [s for s in signals if '参考' in s or 'setpoint' in s.lower() or 'ref' in s.lower()]
            if ref_name:
                ref = self.data[ref_name[0]]
                for s in signals:
                    if s != ref_name[0] and '误差' not in s and '控制' not in s:
                        error = ref - self.data[s]
                        axes[1].plot(t, error, linewidth=1, label=f'{s}误差')
                axes[1].set_ylabel('误差')
                axes[1].set_title('跟踪误差')
                axes[1].legend(loc='best')
                axes[1].grid(True, alpha=0.3)
            else:
                for s in signals:
                    if '误差' in s:
                        axes[1].plot(t, self.data[s], linewidth=1, label=s)
                axes[1].set_ylabel('误差')
                axes[1].set_title('误差信号')
                axes[1].legend(loc='best')
                axes[1].grid(True, alpha=0.3)
            axes[1].set_xlabel('时间 (s)')

        plt.tight_layout()
        return fig

    def plot_step_metrics(self, signal, ref_signal=None, title='阶跃响应性能指标'):
        """阶跃响应指标标注图"""
        t = self.data['t']
        y = self.data[signal]
        if ref_signal and ref_signal in self.data:
            ref = self.data[ref_signal]
        else:
            ref = np.ones_like(t)

        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(t, ref, 'k--', linewidth=1.5, label='参考')
        ax.plot(t, y, 'b-', linewidth=1.5, label=signal)

        # 计算指标
        final_val = ref[-1]
        y_final = np.mean(y[-50:])

        # 上升时间 (10%~90%)
        idx_10 = np.argmax(y > 0.1 * final_val)
        idx_90 = np.argmax(y > 0.9 * final_val)
        t_rise = t[idx_90] - t[idx_10]

        # 超调量
        y_max = np.max(y)
        overshoot = max(0, (y_max - final_val) / final_val * 100)

        # 调节时间 (2%误差带)
        settling_idx = len(t) - 1
        for j in range(len(y)-1, 0, -1):
            if abs(y[j] - final_val) > 0.02 * final_val:
                settling_idx = min(j + 1, len(t) - 1)
                break
        t_settle = t[settling_idx]

        # 标注
        ax.axhline(y=final_val, color='gray', linestyle=':', alpha=0.5)
        ax.axhline(y=y_max, color='red', linestyle=':', alpha=0.3)
        ax.annotate(f'上升时间 Tr={t_rise:.3f}s',
                    xy=(t[idx_90], y[idx_90]), xytext=(t[idx_90]+0.3, y[idx_90]+0.1),
                    arrowprops=dict(arrowstyle='->', color='green'),
                    fontsize=10, color='green')
        ax.annotate(f'超调 {overshoot:.1f}%',
                    xy=(t[np.argmax(y)], y_max), xytext=(t[np.argmax(y)]+0.2, y_max+0.05),
                    arrowprops=dict(arrowstyle='->', color='red'),
                    fontsize=10, color='red')
        ax.annotate(f'调节时间 Ts={t_settle:.3f}s',
                    xy=(t_settle, y[settling_idx]),
                    xytext=(t_settle+0.1, y[settling_idx]-0.1),
                    arrowprops=dict(arrowstyle='->', color='blue'),
                    fontsize=10, color='blue')

        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('幅值')
        ax.set_title(title)
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        return fig

    def plot_phase_plane(self, x_signal, y_signal, title='相平面图'):
        """相平面图"""
        fig, ax = plt.subplots(figsize=(8, 8))
        x = self.data[x_signal]
        y = self.data[y_signal]

        # 颜色渐变表示时间
        points = ax.scatter(x[::5], y[::5], c=self.data['t'][::5],
                            cmap='viridis', s=2, alpha=0.7)
        ax.plot(x[0], y[0], 'go', markersize=10, label='起点')
        ax.plot(x[-1], y[-1], 'r*', markersize=15, label='终点')

        plt.colorbar(points, ax=ax, label='时间 (s)')
        ax.set_xlabel(x_signal)
        ax.set_ylabel(y_signal)
        ax.set_title(title)
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', adjustable='datalim')
        plt.tight_layout()
        return fig

    def plot_frequency_response(self, signal, title='频率响应', fs=None):
        """频域分析 (FFT)"""
        t = self.data['t']
        y = self.data[signal]

        if fs is None:
            fs = 1.0 / (t[1] - t[0])

        N = len(y)
        Y = np.fft.rfft(y)
        freq = np.fft.rfftfreq(N, d=1.0/fs)
        magnitude = 20 * np.log10(np.abs(Y) + 1e-10)
        phase = np.angle(Y, deg=True)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        ax1.plot(freq, magnitude, 'b-', linewidth=1)
        ax1.set_ylabel('幅值 (dB)')
        ax1.set_title(title)
        ax1.grid(True, alpha=0.3)

        ax2.plot(freq, phase, 'r-', linewidth=1)
        ax2.set_xlabel('频率 (Hz)')
        ax2.set_ylabel('相位 (°)')
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        return fig

    def plot_bode(self, num, den, title='Bode图'):
        """绘制传递函数的Bode图"""
        from scipy.signal import bode, TransferFunction
        sys = TransferFunction(num, den)
        w, mag, phase = bode(sys)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        ax1.semilogx(w, mag, 'b-', linewidth=1.5)
        ax1.set_ylabel('幅值 (dB)')
        ax1.set_title(title)
        ax1.grid(True, which='both', alpha=0.3)

        ax2.semilogx(w, phase, 'r-', linewidth=1.5)
        ax2.set_xlabel('角频率 (rad/s)')
        ax2.set_ylabel('相位 (°)')
        ax2.grid(True, which='both', alpha=0.3)

        plt.tight_layout()
        return fig

    def plot_comparison_table(self, signals, ref_signal=None):
        """多信号性能对比表"""
        t = self.data['t']
        if ref_signal and ref_signal in self.data:
            ref = self.data[ref_signal]
        else:
            ref = np.ones_like(t)

        print("\n" + "=" * 70)
        print(f"{'信号':15s} | {'MAE':>8s} | {'RMSE':>8s} | {'最大误差':>8s} | {'超调%':>8s} | {'Ts(2%)':>8s}")
        print("-" * 70)

        for s in signals:
            if s == ref_signal:
                continue
            y = self.data[s]
            if len(y) != len(ref):
                continue
            error = ref - y
            mae = np.mean(np.abs(error))
            rmse = np.sqrt(np.mean(error**2))
            max_err = np.max(np.abs(error))

            final_val = ref[-1]
            y_max = np.max(y)
            overshoot = max(0, (y_max - final_val) / abs(final_val) * 100) if final_val != 0 else 0

            # 调节时间
            settling = t[-1]
            for j in range(len(y)-1, 0, -1):
                if abs(ref[j] - y[j]) > 0.02 * abs(final_val):
                    settling = t[min(j+1, len(t)-1)]
                    break

            print(f"{s:15s} | {mae:8.4f} | {rmse:8.4f} | {max_err:8.4f} | {overshoot:7.1f}% | {settling:7.3f}s")

        print("=" * 70)

    def plot_3d_surface(self, x_key, y_key, z_key, title='3D曲面图'):
        """3D曲面图"""
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        x = self.data[x_key]
        y = self.data[y_key]
        z = self.data[z_key]

        # 需要网格数据，尝试reshape
        n = int(np.sqrt(len(x)))
        if n * n == len(x):
            X = x.reshape(n, n)
            Y = y.reshape(n, n)
            Z = z.reshape(n, n)
        else:
            X, Y = np.meshgrid(x[:100], y[:100])
            Z = z[:10000].reshape(100, 100) if len(z) >= 10000 else None

        if Z is not None:
            surf = ax.plot_surface(X, Y, Z, cmap='viridis', alpha=0.8)
            fig.colorbar(surf, ax=ax, shrink=0.5)

        ax.set_xlabel(x_key)
        ax.set_ylabel(y_key)
        ax.set_zlabel(z_key)
        ax.set_title(title)
        plt.tight_layout()
        return fig

    def plot_heatmap(self, matrix, xlabels=None, ylabels=None, title='热力图'):
        """热力图"""
        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(matrix, cmap='RdYlBu_r', aspect='auto')
        fig.colorbar(im, ax=ax)

        if xlabels:
            ax.set_xticks(range(len(xlabels)))
            ax.set_xticklabels(xlabels, rotation=45)
        if ylabels:
            ax.set_yticks(range(len(ylabels)))
            ax.set_yticklabels(ylabels)

        ax.set_title(title)
        plt.tight_layout()
        return fig

    def comprehensive_report(self, ref_signal='参考信号'):
        """综合报告: 生成所有图表"""
        signal_names = [k for k in self.data if k != 't' and '误差' not in k and '控制' not in k]

        # 1. 时域响应
        self.plot_time_response(title='综合时域响应分析')

        # 2. 性能对比表
        response_signals = [s for s in signal_names if '参考' not in s]
        self.plot_comparison_table(response_signals, ref_signal)

        # 3. 阶跃响应指标
        if response_signals:
            self.plot_step_metrics(response_signals[0], ref_signal)

        # 4. 相平面 (如果有误差信号)
        error_signals = [k for k in self.data if '误差' in k]
        if error_signals and response_signals:
            t = self.data['t']
            error_dot = np.gradient(self.data[error_signals[0]], t[1]-t[0])
            self.add_signal('误差导数', error_dot)
            self.plot_phase_plane(error_signals[0], '误差导数', '误差相平面')

        # 5. 频域分析
        if response_signals:
            self.plot_frequency_response(response_signals[0], '频率响应分析')

        plt.close('all')

    def save_all(self, prefix='sim_result', dpi=150):
        """保存所有打开的图表"""
        for i, num in enumerate(plt.get_fignums()):
            fig = plt.figure(num)
            filename = f'{prefix}_{i+1}.png'
            fig.savefig(filename, dpi=dpi, bbox_inches='tight')
            print(f"已保存: {filename}")


def demo():
    """演示: 综合分析"""
    print("=" * 60)
    print("  仿真结果可视化工具 - 综合演示")
    print("=" * 60)

    viz = SimVisualizer()
    viz.generate_demo_data()

    # 综合报告
    viz.comprehensive_report()

    # 示例: 热力图 (PID参数扫描结果)
    kp_range = np.linspace(0.5, 5, 10)
    ki_range = np.linspace(0.01, 1, 10)
    cost = np.zeros((10, 10))
    for i, kp in enumerate(kp_range):
        for j, ki in enumerate(ki_range):
            # 模拟代价函数 (简化)
            cost[i, j] = abs(kp - 2.5)**2 + abs(ki - 0.3)**2 + 0.1 * np.random.randn()
    viz.plot_heatmap(cost,
                     xlabels=[f'{v:.1f}' for v in ki_range],
                     ylabels=[f'{v:.1f}' for v in kp_range],
                     title='PID参数优化热力图 (Kp vs Ki)')
    plt.close('all')

    print("\n提示: 使用 SimVisualizer 类可以:")
    print("  - load_csv() 加载CSV数据文件")
    print("  - load_json() 加载JSON数据文件")
    print("  - plot_time_response() 时域响应图")
    print("  - plot_step_metrics() 阶跃响应指标图")
    print("  - plot_phase_plane() 相平面图")
    print("  - plot_frequency_response() 频域分析")
    print("  - plot_bode() 传递函数Bode图")
    print("  - plot_3d_surface() 3D曲面图")
    print("  - plot_heatmap() 热力图")
    print("  - comprehensive_report() 综合报告")


if __name__ == '__main__':
    demo()
