"""
数字PID仿真 (Digital PID Simulation)
=====================================
离散化PID的工程实现问题：
- 位置式 vs 增量式PID
- 采样率对控制性能的影响
- 量化效应（ADC/DAC位数）
- 积分饱和与抗饱和

Author: nuedc-asset-library
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class DigitalPID:
    """数字PID控制器（支持位置式/增量式）"""

    def __init__(self, Kp, Ki, Kd, dt, mode='position', output_bits=None, anti_windup=True):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.mode = mode
        self.output_bits = output_bits  # DAC位数，None表示不限制
        self.anti_windup = anti_windup

        self.e = [0.0, 0.0, 0.0]  # e[k], e[k-1], e[k-2]
        self.u = 0.0
        self.u_prev = 0.0
        self.integral = 0.0

    def quantize(self, value, bits):
        """模拟DAC量化"""
        if bits is None:
            return value
        levels = 2 ** bits
        max_val = 10.0
        min_val = -10.0
        step = (max_val - min_val) / levels
        return np.round((value - min_val) / step) * step + min_val

    def compute(self, setpoint, measurement):
        self.e[2] = self.e[1]
        self.e[1] = self.e[0]
        self.e[0] = setpoint - measurement

        if self.mode == 'position':
            # 位置式PID
            self.integral += self.e[0] * self.dt
            derivative = (self.e[0] - self.e[1]) / self.dt
            u_raw = self.Kp * self.e[0] + self.Ki * self.integral + self.Kd * derivative

            # 抗积分饱和
            if self.anti_windup:
                u_sat = self.quantize(np.clip(u_raw, -10, 10), self.output_bits)
                if u_raw != u_sat:
                    self.integral -= self.e[0] * self.dt  # 回退积分
            else:
                u_sat = self.quantize(u_raw, self.output_bits)

        elif self.mode == 'increment':
            # 增量式PID
            delta_u = self.Kp * (self.e[0] - self.e[1]) \
                     + self.Ki * self.e[0] * self.dt \
                     + self.Kd * (self.e[0] - 2*self.e[1] + self.e[2]) / self.dt
            u_raw = self.u_prev + delta_u
            u_sat = self.quantize(np.clip(u_raw, -10, 10), self.output_bits)

        self.u_prev = u_sat
        return u_sat

    def reset(self):
        self.e = [0.0, 0.0, 0.0]
        self.u = 0.0
        self.u_prev = 0.0
        self.integral = 0.0


def analog_pid_sim(Kp, Ki, Kd, ref, dt_plant, T, K=1.0, tau=1.0):
    """连续PID（高精度参考）"""
    N = int(T / dt_plant)
    y = np.zeros(N)
    e_int = 0.0
    e_prev = 0.0
    for i in range(N):
        e = ref[min(i, len(ref)-1)] - y[i]
        e_int += e * dt_plant
        de = (e - e_prev) / dt_plant
        e_prev = e
        u = np.clip(Kp * e + Ki * e_int + Kd * de, -10, 10)
        y[i] += dt_plant / tau * (K * u - y[i])
    return y


def digital_pid_sim(Kp, Ki, Kd, ref, dt_plant, dt_ctrl, T, K=1.0, tau=1.0,
                    mode='position', output_bits=None, anti_windup=True):
    """离散PID仿真"""
    N_plant = int(T / dt_plant)
    N_ctrl = int(T / dt_ctrl)
    ratio = max(1, int(dt_ctrl / dt_plant))

    y = np.zeros(N_plant)
    u = np.zeros(N_plant)
    ctrl = DigitalPID(Kp, Ki, Kd, dt_ctrl, mode, output_bits, anti_windup)

    for i in range(1, N_plant):
        if i % ratio == 0:
            ref_idx = min(i, len(ref) - 1)
            u[i] = ctrl.compute(ref[ref_idx], y[i-1])
        else:
            u[i] = u[i-1]
        y[i] = y[i-1] + dt_plant / tau * (K * u[i] - y[i-1])

    return y, u


if __name__ == '__main__':
    Kp, Ki, Kd = 5.0, 2.0, 1.0
    K, tau = 1.0, 1.0
    T = 10.0
    dt_plant = 0.001  # 被控对象仿真步长

    N = int(T / dt_plant)
    t = np.linspace(0, T, N)
    ref = np.ones(N)

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    # ========== 1. 采样率影响 ==========
    dt_ctrls = [0.001, 0.01, 0.05, 0.1, 0.2]
    colors = ['b', 'g', 'orange', 'r', 'purple']

    for i_ctrl, dt_c in enumerate(dt_ctrls):
        N_c = int(T / dt_c)
        t_c = np.linspace(0, T, N_c)
        ref_c = np.ones(N_c)
        y, u = digital_pid_sim(Kp, Ki, Kd, ref_c, dt_plant, dt_c, T)
        axes[0, 0].plot(t, y, colors[i_ctrl], lw=1.2, label=f'Ts={dt_c}s')
        axes[0, 1].plot(t, u, colors[i_ctrl], lw=0.8, label=f'Ts={dt_c}s')

    axes[0, 0].plot(t, ref, 'k--', lw=1.5, label='参考')
    axes[0, 0].set_title('采样率对响应的影响')
    axes[0, 0].set_ylabel('输出')
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].set_title('采样率对控制量的影响')
    axes[0, 1].set_ylabel('控制量')
    axes[0, 1].legend(fontsize=8)
    axes[0, 1].grid(True, alpha=0.3)

    # ========== 2. 量化效应 ==========
    bits_list = [None, 16, 12, 8, 6]
    labels = ['无量化', '16bit', '12bit', '8bit', '6bit']

    for idx, bits in enumerate(bits_list):
        dt_c = 0.01
        N_c = int(T / dt_c)
        ref_c = np.ones(N_c)
        y, u = digital_pid_sim(Kp, Ki, Kd, ref_c, dt_plant, dt_c, T, output_bits=bits)
        axes[1, 0].plot(t, y, colors[idx], lw=1.2, label=labels[idx])
        axes[1, 1].plot(t, u, colors[idx], lw=0.8, label=labels[idx])

    axes[1, 0].plot(t, ref, 'k--', lw=1.5, label='参考')
    axes[1, 0].set_title('DAC量化位数对响应的影响')
    axes[1, 0].set_ylabel('输出')
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].set_title('DAC量化对控制量的影响')
    axes[1, 1].set_ylabel('控制量')
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(True, alpha=0.3)

    # ========== 3. 位置式 vs 增量式 ==========
    dt_c = 0.01
    N_c = int(T / dt_c)
    ref_c = np.ones(N_c)

    y_pos, u_pos = digital_pid_sim(Kp, Ki, Kd, ref_c, dt_plant, dt_c, T, mode='position')
    y_inc, u_inc = digital_pid_sim(Kp, Ki, Kd, ref_c, dt_plant, dt_c, T, mode='increment')

    axes[2, 0].plot(t, ref, 'k--', lw=1.5, label='参考')
    axes[2, 0].plot(t, y_pos, 'b-', lw=1.2, label='位置式PID')
    axes[2, 0].plot(t, y_inc, 'r-', lw=1.2, label='增量式PID')
    axes[2, 0].set_title('位置式 vs 增量式PID')
    axes[2, 0].set_xlabel('时间 (s)')
    axes[2, 0].set_ylabel('输出')
    axes[2, 0].legend()
    axes[2, 0].grid(True, alpha=0.3)

    axes[2, 1].plot(t, u_pos, 'b-', lw=1, label='位置式PID')
    axes[2, 1].plot(t, u_inc, 'r-', lw=1, label='增量式PID')
    axes[2, 1].set_title('控制量对比')
    axes[2, 1].set_xlabel('时间 (s)')
    axes[2, 1].set_ylabel('控制量')
    axes[2, 1].legend()
    axes[2, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('digital_pid_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')

    # 性能总结
    print(f"\n{'配置':<25} {'IAE':>10} {'最大超调%':>10}")
    print('-' * 48)
    for dt_c in dt_ctrls:
        N_c = int(T / dt_c)
        ref_c = np.ones(N_c)
        y, _ = digital_pid_sim(Kp, Ki, Kd, ref_c, dt_plant, dt_c, T)
        iae = np.sum(np.abs(ref - y)) * dt_plant
        overshoot = (np.max(y) - 1.0) * 100
        print(f"Ts={dt_c}s{'':<20} {iae:>10.4f} {overshoot:>10.2f}")
