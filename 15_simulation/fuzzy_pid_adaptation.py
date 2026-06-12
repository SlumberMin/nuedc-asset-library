#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模糊PID自适应仿真
=================
仿真内容：模糊控制器在线调整PID参数Kp, Ki, Kd
被控对象：二阶系统
展示参数在线调整过程
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False




# ============ 被控对象 ============
def plant_update(x, u, dt):
    """二阶系统: G(s) = 1 / (s^2 + s + 1)"""
    pos, vel = x
    acc = (u - vel - pos) / 1.0
    vel_new = vel + acc * dt
    pos_new = pos + vel_new * dt
    return np.array([pos_new, vel_new])

# ============ 模糊控制器 ============
class FuzzyPIDController:
    """
    模糊PID控制器
    输入：误差e, 误差变化率ec
    输出：delta_Kp, delta_Ki, delta_Kd
    """
    def __init__(self, Kp0, Ki0, Kd0):
        # 初始PID参数
        self.Kp = Kp0
        self.Ki = Ki0
        self.Kd = Kd0
        self.Kp0 = Kp0
        self.Ki0 = Ki0
        self.Kd0 = Kd0
        
        self.integral = 0.0
        self.prev_error = 0.0
        
        # 模糊集：NB, NM, NS, ZO, PS, PM, PB
        # 量化到[-3, 3]
        self.levels = np.array([-3, -2, -1, 0, 1, 2, 3])
        
        # 模糊规则表（7x7）用于 delta_Kp
        # 行: e (NB->PB), 列: ec (NB->PB)
        self.rule_Kp = np.array([
            [ 3,  3,  2,  2,  1,  0,  0],
            [ 3,  3,  2,  1,  1,  0, -1],
            [ 2,  2,  2,  1,  0, -1, -1],
            [ 2,  2,  1,  0, -1, -2, -2],
            [ 1,  1,  0, -1, -1, -2, -2],
            [ 1,  0, -1, -2, -2, -2, -3],
            [ 0,  0, -2, -2, -2, -3, -3]
        ])
        
        # 模糊规则表用于 delta_Ki
        self.rule_Ki = np.array([
            [-3, -3, -2, -2, -1,  0,  0],
            [-3, -3, -2, -1, -1,  0,  0],
            [-3, -2, -1, -1,  0,  1,  1],
            [-2, -2, -1,  0,  1,  2,  2],
            [-2, -1,  0,  1,  1,  2,  3],
            [-1,  0,  1,  1,  2,  3,  3],
            [ 0,  0,  1,  2,  2,  3,  3]
        ])
        
        # 模糊规则表用于 delta_Kd
        self.rule_Kd = np.array([
            [ 3,  2,  1,  1,  0, -1, -2],
            [ 3,  2,  1,  0,  0, -1, -2],
            [ 2,  1,  0,  0,  0, -1, -2],
            [ 2,  1,  0,  0,  0, -1, -2],
            [ 2,  1,  0,  0,  0, -1, -2],
            [ 3,  2,  1,  0,  0, -1, -2],
            [ 3,  2,  1,  1,  0, -1, -2]
        ])
    
    def _quantize(self, value, range_val):
        """量化到[-3, 3]"""
        return np.clip(value / range_val * 3, -3, 3)
    
    def _membership(self, x, center, width=1.0):
        """三角隶属度函数"""
        return max(0, 1 - abs(x - center) / width)
    
    def _fuzzy_infer(self, e_fuzzy, ec_fuzzy, rule_table):
        """模糊推理（加权平均法）"""
        result = 0.0
        weight_sum = 0.0
        
        for i, ei in enumerate(self.levels):
            for j, ecj in enumerate(self.levels):
                w = self._membership(e_fuzzy, ei) * self._membership(ec_fuzzy, ecj)
                if w > 0:
                    result += w * rule_table[i, j]
                    weight_sum += w
        
        if weight_sum > 0:
            return result / weight_sum
        return 0.0
    
    def compute(self, ref, y, dt):
        error = ref - y
        d_error = (error - self.prev_error) / dt
        
        # 量化
        e_fuzzy = self._quantize(error, 2.0)      # 假设误差范围[-2, 2]
        ec_fuzzy = self._quantize(d_error, 10.0)   # 假设误差变化率范围[-10, 10]
        
        # 模糊推理
        d_Kp = self._fuzzy_infer(e_fuzzy, ec_fuzzy, self.rule_Kp)
        d_Ki = self._fuzzy_infer(e_fuzzy, ec_fuzzy, self.rule_Ki)
        d_Kd = self._fuzzy_infer(e_fuzzy, ec_fuzzy, self.rule_Kd)
        
        # 参数自适应
        self.Kp = max(0, self.Kp0 + d_Kp * 0.5)
        self.Ki = max(0, self.Ki0 + d_Ki * 0.1)
        self.Kd = max(0, self.Kd0 + d_Kd * 0.1)
        
        # PID计算
        self.integral += error * dt
        derivative = d_error
        self.prev_error = error
        
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return u

# ============ 普通PID ============
class StandardPID:
    """标准PID控制器（用于与模糊PID对比）"""
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, ref, y, dt):
        error = ref - y
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


def ref_func(t):
    """参考信号：阶跃 + 正弦 + 阶跃变化"""
    if t < 2:
        return 1.0
    elif t < 5:
        return 1.0 + 0.5 * np.sin(2 * np.pi * 0.5 * (t - 2))
    elif t < 7:
        return 0.5
    else:
        return 1.5


def run_simulation(dt=0.001, T_total=10.0, save_path=None):
    """运行模糊PID自适应仿真

    Parameters
    ----------
    dt : float
        仿真步长 (s)
    T_total : float
        仿真总时长 (s)
    save_path : str or None
        图表保存路径，默认保存到脚本同目录
    """
    N = int(T_total / dt)
    t = np.arange(N) * dt

    fuzzy_pid = FuzzyPIDController(Kp0=5.0, Ki0=2.0, Kd0=1.0)
    std_pid = StandardPID(Kp=5.0, Ki=2.0, Kd=1.0)

    x_fuzzy = np.array([0.0, 0.0])
    x_std = np.array([0.0, 0.0])

    y_fuzzy = np.zeros(N)
    y_std = np.zeros(N)
    Kp_log = np.zeros(N)
    Ki_log = np.zeros(N)
    Kd_log = np.zeros(N)
    ref_log = np.zeros(N)

    for i in range(N):
        ref = ref_func(i * dt)
        ref_log[i] = ref
        u_f = fuzzy_pid.compute(ref, x_fuzzy[0], dt)
        x_fuzzy = plant_update(x_fuzzy, u_f, dt)
        y_fuzzy[i] = x_fuzzy[0]
        Kp_log[i] = fuzzy_pid.Kp
        Ki_log[i] = fuzzy_pid.Ki
        Kd_log[i] = fuzzy_pid.Kd
        u_s = std_pid.compute(ref, x_std[0], dt)
        x_std = plant_update(x_std, u_s, dt)
        y_std[i] = x_std[0]

    err_fuzzy = np.abs(ref_log - y_fuzzy)
    err_std = np.abs(ref_log - y_std)
    print(f"模糊PID - IAE: {np.sum(err_fuzzy)*dt:.3f}, ISE: {np.sum((ref_log-y_fuzzy)**2)*dt:.3f}")
    print(f"标准PID - IAE: {np.sum(err_std)*dt:.3f}, ISE: {np.sum((ref_log-y_std)**2)*dt:.3f}")

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('模糊PID自适应控制仿真', fontsize=16, fontweight='bold')

    ax = axes[0, 0]
    ax.plot(t, ref_log, 'k--', linewidth=2, label='参考值')
    ax.plot(t, y_fuzzy, 'r-', linewidth=1.5, label='模糊PID')
    ax.plot(t, y_std, 'b-', linewidth=1.5, label='标准PID')
    ax.set_title('跟踪响应对比')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('输出')
    ax.legend(); ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, err_fuzzy, 'r-', linewidth=1, label='模糊PID')
    ax.plot(t, err_std, 'b-', linewidth=1, label='标准PID')
    ax.set_title('跟踪误差对比')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('|误差|')
    ax.legend(); ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(t, Kp_log, 'r-', linewidth=1.5, label='Kp (模糊自适应)')
    ax.axhline(y=5.0, color='b', linestyle='--', alpha=0.5, label='Kp (标准PID)')
    ax.plot(t, Ki_log, 'g-', linewidth=1.5, label='Ki (模糊自适应)')
    ax.axhline(y=2.0, color='b', linestyle=':', alpha=0.5, label='Ki (标准PID)')
    ax.set_title('PID参数在线调整过程')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('参数值')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t, Kd_log, 'm-', linewidth=1.5, label='Kd (模糊自适应)')
    ax.axhline(y=1.0, color='b', linestyle='--', alpha=0.5, label='Kd (标准PID)')
    ax.set_title('Kd参数在线调整过程')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('参数值')
    ax.legend(); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path is None:
        save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 'fuzzy_pid_adaptation_result.png')
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n图表已保存: {save_path}")
    plt.close()
    print("模糊PID自适应仿真完成!")


if __name__ == '__main__':
    run_simulation()
