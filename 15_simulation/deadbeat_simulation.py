#!/usr/bin/env python3
"""
无差拍控制仿真 (Deadbeat Control)
====================================
无差拍控制原理：
  根据被控对象的离散数学模型，计算出使下一采样时刻
  输出精确等于参考值的控制量。

  对于二阶系统: y(k+1) = a1*y(k) + a2*y(k-1) + b*u(k)
  控制律: u(k) = (r(k+1) - a1*y(k) - a2*y(k-1)) / b

对比：无差拍控制 vs 常规PID

运行方式: python deadbeat_simulation.py
输出图表: deadbeat_result.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class DiscreteSecondOrderPlant:
    """离散二阶被控对象"""
    def __init__(self, a1=1.5, a2=-0.7, b=0.3):
        self.a1 = a1
        self.a2 = a2
        self.b = b
        self.y = [0.0, 0.0]  # y(k), y(k-1)

    def update(self, u):
        y_new = self.a1 * self.y[0] + self.a2 * self.y[1] + self.b * u
        self.y[1] = self.y[0]
        self.y[0] = y_new
        return y_new

    def reset(self):
        self.y = [0.0, 0.0]


class DeadbeatController:
    """无差拍控制器"""
    def __init__(self, a1, a2, b, u_min=-10, u_max=10):
        self.a1 = a1
        self.a2 = a2
        self.b = b
        self.u_min = u_min
        self.u_max = u_max

    def compute(self, ref, y_k, y_km1):
        """计算控制量使 y(k+1) = ref"""
        u = (ref - self.a1 * y_k - self.a2 * y_km1) / self.b
        return np.clip(u, self.u_min, self.u_max)


class PIDController:
    def __init__(self, kp, ki, kd, out_min=-10, out_max=10):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt=1.0):
        self.integral += error * dt
        self.integral = np.clip(self.integral,
                                self.out_min / max(self.ki, 1e-8),
                                self.out_max / max(self.ki, 1e-8))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return np.clip(out, self.out_min, self.out_max)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


def run_simulation():
    N = 200  # 采样点数

    # 被控对象参数
    a1, a2, b_coeff = 1.5, -0.7, 0.3

    # 参考信号
    ref = np.zeros(N)
    ref[20:60] = 1.0
    ref[80:120] = -0.5
    ref[140:180] = 0.8

    # ===== 无差拍控制 =====
    plant_db = DiscreteSecondOrderPlant(a1, a2, b_coeff)
    db_ctrl = DeadbeatController(a1, a2, b_coeff)
    y_db = np.zeros(N)
    u_db = np.zeros(N)

    for k in range(N):
        ref_next = ref[min(k+1, N-1)]
        u_db[k] = db_ctrl.compute(ref_next, plant_db.y[0], plant_db.y[1])
        y_db[k] = plant_db.update(u_db[k])

    # ===== 常规PID =====
    plant_pid = DiscreteSecondOrderPlant(a1, a2, b_coeff)
    pid = PIDController(kp=3.0, ki=0.5, kd=0.3)
    y_pid = np.zeros(N)
    u_pid = np.zeros(N)

    for k in range(N):
        u_pid[k] = pid.compute(ref[k] - plant_pid.y[0])
        y_pid[k] = plant_pid.update(u_pid[k])

    # ===== 模型失配下的无差拍 =====
    # 实际对象参数偏移
    plant_mis = DiscreteSecondOrderPlant(a1=1.6, a2=-0.65, b=0.35)
    y_mis = np.zeros(N)
    u_mis = np.zeros(N)

    for k in range(N):
        ref_next = ref[min(k+1, N-1)]
        u_mis[k] = db_ctrl.compute(ref_next, plant_mis.y[0], plant_mis.y[1])
        y_mis[k] = plant_mis.update(u_mis[k])

    k_axis = np.arange(N)

    # ===== 绘图 =====
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle('无差拍控制仿真 (Deadbeat Control)', fontsize=14, fontweight='bold')

    axes[0].plot(k_axis, ref, 'k--', linewidth=2, label='参考信号')
    axes[0].plot(k_axis, y_db, 'r-o', markersize=2, linewidth=1.5, label='无差拍控制')
    axes[0].plot(k_axis, y_pid, 'b-', linewidth=1.2, label='常规PID')
    axes[0].plot(k_axis, y_mis, 'g--', linewidth=1.2, label='模型失配无差拍')
    axes[0].set_ylabel('输出 y(k)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('输出响应对比')

    # 误差
    err_db = ref - y_db
    err_pid = ref - y_pid
    err_mis = ref - y_mis
    axes[1].plot(k_axis, err_db, 'r-', linewidth=1.0, label=f'无差拍 (RMSE={np.sqrt(np.mean(err_db**2)):.4f})')
    axes[1].plot(k_axis, err_pid, 'b-', linewidth=1.0, label=f'PID (RMSE={np.sqrt(np.mean(err_pid**2)):.4f})')
    axes[1].plot(k_axis, err_mis, 'g-', linewidth=1.0, label=f'失配无差拍 (RMSE={np.sqrt(np.mean(err_mis**2)):.4f})')
    axes[1].set_ylabel('误差 e(k)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('跟踪误差')

    axes[2].plot(k_axis, u_db, 'r-', linewidth=1.0, label='无差拍控制量', alpha=0.8)
    axes[2].plot(k_axis, u_pid, 'b-', linewidth=1.0, label='PID控制量', alpha=0.8)
    axes[2].set_ylabel('控制量 u(k)')
    axes[2].set_xlabel('采样步 k')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title('控制输出')

    plt.tight_layout()
    plt.savefig('deadbeat_result.png', dpi=150, bbox_inches='tight')
    print('[OK] 仿真完成，图表已保存: deadbeat_result.png')
    plt.close()


if __name__ == '__main__':
    run_simulation()
