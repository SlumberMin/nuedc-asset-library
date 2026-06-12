#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通用仿真代码模板
================
使用方法:
  1. 复制本文件, 重命名为你的仿真名
  2. 修改 Plant 类为你的被控对象
  3. 修改 Controller 类为你的控制算法
  4. 在 run_scenario() 中配置仿真参数
  5. 运行查看结果
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
from datetime import datetime

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ============================================================
# 1. 被控对象 (Plant)
# ============================================================
class Plant:
    """
    被控对象模型 - 根据实际需求修改
    示例: 一阶惯性环节 G(s) = K / (τs + 1)
    """
    def __init__(self, K=1.0, tau=1.0, dt=0.01):
        self.K = K
        self.tau = tau
        self.dt = dt
        self.state = 0.0

    def update(self, u):
        """输入u, 返回输出y"""
        self.state += (self.K * u - self.state) / self.tau * self.dt
        return self.state

    def reset(self):
        self.state = 0.0


# ============================================================
# 2. 控制器 (Controller)
# ============================================================
class Controller:
    """
    控制器 - 根据实际需求修改
    示例: PID控制器
    """
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, reference, measurement):
        """计算控制输出"""
        error = reference - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ============================================================
# 3. 设定值/参考信号生成
# ============================================================
def reference_signal(t, signal_type='step'):
    """
    生成参考信号
    signal_type: 'step' / 'ramp' / 'sine' / 'square'
    """
    if signal_type == 'step':
        return 1.0 if t > 0.5 else 0.0
    elif signal_type == 'ramp':
        return min(t, 5.0)
    elif signal_type == 'sine':
        return np.sin(2 * np.pi * 0.5 * t)
    elif signal_type == 'square':
        return 1.0 if np.sin(2 * np.pi * 0.5 * t) > 0 else 0.0
    else:
        return 1.0


# ============================================================
# 4. 仿真主循环
# ============================================================
def run_scenario(scenario_name='default', dt=0.01, T=10.0,
                 plant_params=None, ctrl_params=None, ref_type='step',
                 disturbance_func=None):
    """
    运行单个仿真场景

    Parameters:
        scenario_name: 场景名称
        dt: 仿真步长
        T: 仿真时长
        plant_params: 被控对象参数 dict
        ctrl_params: 控制器参数 dict
        ref_type: 参考信号类型
        disturbance_func: 干扰函数 d(t), None表示无干扰

    Returns:
        dict with keys: t, reference, output, control, error
    """
    plant_params = plant_params or {}
    ctrl_params = ctrl_params or {}

    plant = Plant(dt=dt, **plant_params)
    ctrl = Controller(dt=dt, **ctrl_params)

    t_arr = np.arange(0, T, dt)
    results = {'t': t_arr, 'ref': [], 'y': [], 'u': [], 'e': []}

    for t in t_arr:
        ref = reference_signal(t, ref_type)
        y = plant.state

        u = ctrl.compute(ref, y)

        # 可选: 添加干扰
        d = disturbance_func(t) if disturbance_func else 0.0
        plant.update(u + d)

        results['ref'].append(ref)
        results['y'].append(y)
        results['u'].append(u)
        results['e'].append(ref - y)

    for k in ['ref', 'y', 'u', 'e']:
        results[k] = np.array(results[k])

    return results


# ============================================================
# 5. 性能指标计算
# ============================================================
def compute_metrics(results, dt):
    """计算常用性能指标"""
    e = results['e']
    ref = results['ref']

    metrics = {
        'ISE':  np.sum(e**2) * dt,
        'IAE':  np.sum(np.abs(e)) * dt,
        'ITAE': np.sum(results['t'] * np.abs(e)) * dt,
        'RMSE': np.sqrt(np.mean(e**2)),
        'MaxError': np.max(np.abs(e)),
        'SteadyStateError': np.mean(np.abs(e[-100:])),
    }
    return metrics


# ============================================================
# 6. 绘图
# ============================================================
def plot_results(results_list, labels, title='仿真结果', save_path=None):
    """
    绘制多组结果对比图

    Parameters:
        results_list: list of result dicts
        labels: list of str
        title: 图标题
        save_path: 保存路径 (None则不保存)
    """
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    for res, label in zip(results_list, labels):
        axes[0].plot(res['t'], res['ref'], 'k--', lw=1, alpha=0.5) if label == labels[0] else None
        axes[0].plot(res['t'], res['y'], label=label)
        axes[1].plot(res['t'], res['u'], label=label)
        axes[2].plot(res['t'], res['e'], label=label)

    axes[0].set_ylabel('输出')
    axes[0].set_title(f'{title} - 系统响应')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('控制量')
    axes[1].set_title('控制信号')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel('误差')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_title('跟踪误差')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close('all')


# ============================================================
# 7. 主程序
# ============================================================
def main():
    dt = 0.01
    T = 10.0

    # 场景1: 基本PID
    r1 = run_scenario('PID', dt=dt, T=T,
                       plant_params={'K': 1.0, 'tau': 1.0},
                       ctrl_params={'kp': 5.0, 'ki': 2.0, 'kd': 0.5},
                       ref_type='step')

    # 场景2: 增大Ki
    r2 = run_scenario('PI大增益', dt=dt, T=T,
                       plant_params={'K': 1.0, 'tau': 1.0},
                       ctrl_params={'kp': 5.0, 'ki': 8.0, 'kd': 0.5},
                       ref_type='step')

    # 场景3: 带干扰
    r3 = run_scenario('带干扰', dt=dt, T=T,
                       plant_params={'K': 1.0, 'tau': 1.0},
                       ctrl_params={'kp': 5.0, 'ki': 2.0, 'kd': 0.5},
                       ref_type='step',
                       disturbance_func=lambda t: 0.5 * np.sin(10*t) if t > 5 else 0)

    # 绘图对比
    plot_results([r1, r2, r3],
                 ['PID基础', 'PI大增益', 'PID+干扰'],
                 title='仿真模板演示',
                 save_path='simulation_template_result.png')

    # 打印性能指标
    print("\n===== 性能指标对比 =====")
    for name, r in [('PID基础', r1), ('PI大增益', r2), ('PID+干扰', r3)]:
        m = compute_metrics(r, dt)
        print(f"\n[{name}]")
        for k, v in m.items():
            print(f"  {k:20s} = {v:.6f}")


if __name__ == '__main__':
    main()
