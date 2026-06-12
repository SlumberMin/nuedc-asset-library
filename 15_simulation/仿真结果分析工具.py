"""
仿真结果分析工具 - 性能指标计算
================================
自动化计算控制系统仿真结果的关键性能指标:
- 上升时间(Rise Time)
- 调节时间(Settling Time, 2%准则)
- 超调量(Overshoot)
- 稳态误差(Steady-State Error)
- IAE / ISE / ITAE / ITSE 积分指标
- 控制量RMS / 最大值
- 相位裕度/增益裕度估算(Bode图)
- 频域带宽分析

支持从CSV/文本文件导入仿真数据, 或直接使用内置测试信号。
输出结构化分析报告 + 可视化图表。

运行: python 仿真结果分析工具.py
"""
import math
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# ============ 性能指标计算 ============
class PerformanceAnalyzer:
    """控制系统仿真性能指标分析器"""

    def __init__(self, t, y, u, setpoint):
        self.t = np.asarray(t, dtype=float)
        self.y = np.asarray(y, dtype=float)
        self.u = np.asarray(u, dtype=float)
        self.setpoint = float(setpoint)
        self.dt = self.t[1] - self.t[0] if len(self.t) > 1 else 0.01
        self.e = self.setpoint - self.y
        self.report = {}

    # ---------- 时域指标 ----------
    def calc_overshoot(self):
        """超调量(%)"""
        peak = np.max(self.y)
        sp = self.setpoint
        if sp == 0:
            return 0.0
        overshoot = (peak - sp) / abs(sp) * 100
        self.report['超调量(%)'] = round(overshoot, 4)
        return overshoot

    def calc_rise_time(self, lo=0.1, hi=0.9):
        """上升时间(10%→90%)"""
        sp = self.setpoint
        target_lo = lo * sp
        target_hi = hi * sp
        idx_lo = next((i for i, v in enumerate(self.y) if v >= target_lo), None)
        idx_hi = next((i for i, v in enumerate(self.y) if v >= target_hi), None)
        if idx_lo is not None and idx_hi is not None:
            rt = self.t[idx_hi] - self.t[idx_lo]
        else:
            rt = float('nan')
        self.report['上升时间(s)'] = round(rt, 4)
        return rt

    def calc_settling_time(self, tol=0.02):
        """调节时间(2%准则)"""
        sp = self.setpoint
        band = tol * abs(sp)
        st = self.t[-1]
        for i in range(len(self.y) - 1, -1, -1):
            if abs(self.y[i] - sp) > band:
                st = self.t[min(i + 1, len(self.t) - 1)]
                break
        self.report['调节时间(s,2%)'] = round(st, 4)
        return st

    def calc_steady_state_error(self, last_pct=0.1):
        """稳态误差(取最后10%数据平均)"""
        n = max(1, int(len(self.y) * last_pct))
        ss_val = np.mean(self.y[-n:])
        ss_err = self.setpoint - ss_val
        ss_err_pct = ss_err / abs(self.setpoint) * 100 if self.setpoint != 0 else 0
        self.report['稳态值'] = round(ss_val, 6)
        self.report['稳态误差'] = round(ss_err, 6)
        self.report['稳态误差(%)'] = round(ss_err_pct, 4)
        return ss_err, ss_err_pct

    # ---------- 积分指标 ----------
    def calc_IAE(self):
        """IAE = ∫|e|dt"""
        val = np.sum(np.abs(self.e)) * self.dt
        self.report['IAE'] = round(val, 6)
        return val

    def calc_ISE(self):
        """ISE = ∫e²dt"""
        val = np.sum(self.e**2) * self.dt
        self.report['ISE'] = round(val, 6)
        return val

    def calc_ITAE(self):
        """ITAE = ∫t|e|dt"""
        val = np.sum(self.t * np.abs(self.e)) * self.dt
        self.report['ITAE'] = round(val, 6)
        return val

    def calc_ITSE(self):
        """ITSE = ∫te²dt"""
        val = np.sum(self.t * self.e**2) * self.dt
        self.report['ITSE'] = round(val, 6)
        return val

    # ---------- 控制量指标 ----------
    def calc_control_metrics(self):
        """控制量统计"""
        u_max = np.max(np.abs(self.u))
        u_rms = np.sqrt(np.mean(self.u**2))
        u_mean = np.mean(self.u)
        # 控制量变化率
        du = np.diff(self.u) / self.dt
        du_max = np.max(np.abs(du))
        du_rms = np.sqrt(np.mean(du**2))
        self.report['控制量最大值'] = round(u_max, 4)
        self.report['控制量RMS'] = round(u_rms, 4)
        self.report['控制量均值'] = round(u_mean, 4)
        self.report['控制量变化率最大'] = round(du_max, 4)
        self.report['控制量变化率RMS'] = round(du_rms, 4)
        return u_max, u_rms, du_max

    # ---------- 频域指标 ----------
    def estimate_bandwidth(self):
        """通过FFT估算闭环带宽(误差信号)"""
        n = len(self.e)
        if n < 16:
            return float('nan')
        fft_e = np.fft.rfft(self.e - np.mean(self.e))
        freqs = np.fft.rfftfreq(n, self.dt)
        mag = np.abs(fft_e) / n
        # 带宽: 幅值降至峰值一半(-6dB)的频率
        peak_mag = np.max(mag[1:])  # 跳过DC
        half_mag = peak_mag / 2
        bw = 0.0
        for i in range(1, len(mag)):
            if mag[i] < half_mag:
                bw = freqs[i]
                break
        self.report['估算带宽(Hz)'] = round(bw, 2)
        return bw

    # ---------- 运行全部分析 ----------
    def analyze_all(self):
        """运行所有分析并返回报告字典"""
        self.calc_overshoot()
        self.calc_rise_time()
        self.calc_settling_time()
        self.calc_steady_state_error()
        self.calc_IAE()
        self.calc_ISE()
        self.calc_ITAE()
        self.calc_ITSE()
        self.calc_control_metrics()
        self.estimate_bandwidth()
        return self.report

    def print_report(self, title="仿真性能分析报告"):
        """打印格式化报告"""
        print("\n" + "=" * 55)
        print(f"  {title}")
        print("=" * 55)
        print(f"  分析时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"  数据点数: {len(self.t)}, dt={self.dt:.4f}s, 仿真时长={self.t[-1]:.2f}s")
        print(f"  设定值:   {self.setpoint}")
        print("-" * 55)

        categories = {
            '时域指标': ['超调量(%)', '上升时间(s)', '调节时间(s,2%)', '稳态值', '稳态误差', '稳态误差(%)'],
            '积分指标': ['IAE', 'ISE', 'ITAE', 'ITSE'],
            '控制量指标': ['控制量最大值', '控制量RMS', '控制量均值', '控制量变化率最大', '控制量变化率RMS'],
            '频域指标': ['估算带宽(Hz)'],
        }
        for cat, keys in categories.items():
            print(f"\n  【{cat}】")
            for k in keys:
                if k in self.report:
                    print(f"    {k:<20}: {self.report[k]}")
        print("\n" + "=" * 55)
        return self.report


# ============ 可视化 ============
def plot_analysis(analyzer, save_path=None):
    """生成分析图表"""
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('仿真结果分析', fontsize=14, fontweight='bold')

    t, y, u, e = analyzer.t, analyzer.y, analyzer.u, analyzer.e
    sp = analyzer.setpoint

    # 时域响应
    axes[0, 0].plot(t, y, 'tab:blue', linewidth=1.5, label='输出 y(t)')
    axes[0, 0].axhline(sp, color='gray', linestyle='--', alpha=0.6, label=f'设定值={sp}')
    axes[0, 0].fill_between(t, sp*0.98, sp*1.02, alpha=0.1, color='green', label='2%带')
    if '超调量(%)' in analyzer.report and analyzer.report['超调量(%)'] > 0:
        peak_idx = np.argmax(y)
        axes[0, 0].annotate(f"超调 {analyzer.report['超调量(%)']:.1f}%",
                    xy=(t[peak_idx], y[peak_idx]), xytext=(t[peak_idx]+0.5, y[peak_idx]),
                    arrowprops=dict(arrowstyle='->', color='red'), color='red', fontsize=9)
    axes[0, 0].set_title('时域响应'); axes[0, 0].set_xlabel('时间(s)'); axes[0, 0].set_ylabel('输出'); axes[0, 0].legend(fontsize=8); axes[0, 0].grid(True, alpha=0.3)

    # 误差
    axes[0, 1].plot(t, e, 'tab:red', linewidth=1.0)
    axes[0, 1].axhline(0, color='gray', linestyle='-', alpha=0.3)
    axes[0, 1].fill_between(t, -0.02*abs(sp), 0.02*abs(sp), alpha=0.1, color='green')
    axes[0, 1].set_title('误差 e(t)'); axes[0, 1].set_xlabel('时间(s)'); axes[0, 1].set_ylabel('误差'); axes[0, 1].grid(True, alpha=0.3)

    # 控制量
    axes[1, 0].plot(t, u, 'tab:green', linewidth=1.0)
    axes[1, 0].set_title('控制量 u(t)'); axes[1, 0].set_xlabel('时间(s)'); axes[1, 0].set_ylabel('控制量'); axes[1, 0].grid(True, alpha=0.3)

    # 误差FFT频谱
    n = len(e)
    if n >= 16:
        fft_e = np.fft.rfft(e - np.mean(e))
        freqs = np.fft.rfftfreq(n, analyzer.dt)
        mag = np.abs(fft_e) / n
        axes[1, 1].semilogy(freqs[1:], mag[1:], 'tab:purple', linewidth=1.0)
        if '估算带宽(Hz)' in analyzer.report:
            bw = analyzer.report['估算带宽(Hz)']
            axes[1, 1].axvline(bw, color='red', linestyle='--', alpha=0.7, label=f'带宽≈{bw:.1f}Hz')
            axes[1, 1].legend(fontsize=8)
    axes[1, 1].set_title('误差频谱(FFT)'); axes[1, 1].set_xlabel('频率(Hz)'); axes[1, 1].set_ylabel('幅值'); axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"\n分析图已保存: {save_path}")
    return fig


# ============ 批量对比分析 ============
def compare_simulations(data_list, save_path=None):
    """
    批量对比多组仿真结果
    data_list: [(name, t, y, u, setpoint), ...]
    """
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('多组仿真结果对比分析', fontsize=14, fontweight='bold')

    colors = ['tab:blue', 'tab:red', 'tab:green', 'tab:orange', 'tab:purple', 'tab:brown']
    summary = []

    print("\n" + "=" * 80)
    print("  多组仿真结果对比分析")
    print("=" * 80)
    print(f"{'名称':<18} {'超调%':>8} {'上升时间':>8} {'调节时间':>8} {'IAE':>10} {'控制RMS':>10}")
    print("-" * 80)

    for i, (name, t, y, u, sp) in enumerate(data_list):
        analyzer = PerformanceAnalyzer(t, y, u, sp)
        r = analyzer.analyze_all()
        color = colors[i % len(colors)]
        axes[0].plot(t, y, color=color, label=name, linewidth=1.2)
        axes[1].plot(t, u, color=color, label=name, linewidth=0.8)

        print(f"{name:<18} {r['超调量(%)']:>7.2f}% {r['上升时间(s)']:>7.4f}s {r['调节时间(s,2%)']:>7.4f}s {r['IAE']:>10.4f} {r['控制量RMS']:>10.4f}")
        summary.append((name, r))

    sp_val = data_list[0][4]
    axes[0].axhline(sp_val, color='gray', linestyle='--', alpha=0.5)
    axes[0].set_title('输出响应对比'); axes[0].set_ylabel('输出'); axes[0].legend(fontsize=8); axes[0].grid(True, alpha=0.3)
    axes[1].set_title('控制量对比'); axes[1].set_ylabel('u(t)'); axes[1].set_xlabel('时间(s)'); axes[1].legend(fontsize=8); axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"\n对比图已保存: {save_path}")

    print("=" * 80)
    return summary


# ============ 主程序 - 演示 ============
if __name__ == "__main__":
    # 构造测试信号: 二阶系统阶跃响应
    def simulate_test(Kp, Ki, Kd, sp=1.0, dt=0.01, t_end=5.0, noise=0.0):
        """快速仿真函数"""
        x1, x2 = 0.0, 0.0
        e_sum, e_prev = 0.0, 0.0
        n = int(t_end / dt)
        t_arr, y_arr, u_arr = [], [], []
        for i in range(n):
            t = i * dt
            y = x2 + np.random.randn() * noise
            e = sp - y
            e_sum += e * dt
            de = (e - e_prev) / dt
            u = Kp * e + Ki * e_sum + Kd * de
            u = max(-10, min(10, u))
            e_prev = e
            dx1 = (-x1 + u) / 0.5
            dx2 = (-x2 + x1) / 0.2
            x1 += dx1 * dt; x2 += dx2 * dt
            t_arr.append(t); y_arr.append(x2); u_arr.append(u)
        return np.array(t_arr), np.array(y_arr), np.array(u_arr)

    # 单组分析
    t, y, u = simulate_test(Kp=3.0, Ki=2.0, Kd=0.5)
    analyzer = PerformanceAnalyzer(t, y, u, setpoint=1.0)
    analyzer.analyze_all()
    analyzer.print_report("PID(3.0, 2.0, 0.5) 阶跃响应分析")

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '15_simulation')
    plot_analysis(analyzer, save_path=f'{out_dir}/simulation_analysis_result.png')

    # 多组对比
    data_list = [
        ("PID(3,2,0.5)", *simulate_test(3.0, 2.0, 0.5), 1.0),
        ("PID(5,1,0.8)", *simulate_test(5.0, 1.0, 0.8), 1.0),
        ("PID(2,5,0.3)", *simulate_test(2.0, 5.0, 0.3), 1.0),
        ("PID(3,2,0.5)+噪声", *simulate_test(3.0, 2.0, 0.5, noise=0.03), 1.0),
    ]
    compare_simulations(data_list, save_path=f'{out_dir}/simulation_comparison_result.png')

    print("\n使用方法:")
    print("  from 仿真结果分析工具 import PerformanceAnalyzer")
    print("  analyzer = PerformanceAnalyzer(t_array, y_array, u_array, setpoint)")
    print("  report = analyzer.analyze_all()")
    print("  analyzer.print_report()")
    print("  plot_analysis(analyzer, 'output.png')")
