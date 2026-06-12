#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
水位控制仿真 - PID + 串级PID
============================================
适用于电赛水位/液位控制类题目
包含单回路PID和串级PID控制对比
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class WaterTankModel:
    """水箱液位模型 (基于流量平衡)
    A * dh/dt = Qin - Qout
    Qout = Cv * sqrt(h)  (非线性出水)
    线性化后: A * dh/dt ≈ Qin - k*h
    """
    def __init__(self, A=1.0, k=0.5, h0=0.0, dt=0.01):
        self.A = A          # 水箱截面积 (m²)
        self.k = k          # 出水系数
        self.h = h0         # 液位 (m)
        self.dt = dt
        self.h_max = 2.0    # 最大液位

    def update(self, flow_in, disturbance=0.0):
        """更新液位, flow_in: 进水流量(m³/s), disturbance: 扰动流量"""
        # 非线性出水
        flow_out = self.k * np.sqrt(max(self.h, 0))
        dh = (flow_in - flow_out + disturbance) / self.A * self.dt
        self.h = max(0, min(self.h_max, self.h + dh))
        return self.h


class ValveModel:
    """阀门/水泵模型 (一阶惯性)
    位置/转速响应: G(s) = 1 / (Tv*s + 1)
    """
    def __init__(self, Tv=0.5, dt=0.01):
        self.Tv = Tv
        self.dt = dt
        self.position = 0.0

    def update(self, command):
        """command: 0~1 的阀门开度指令"""
        command = np.clip(command, 0, 1)
        dp = (command - self.position) / self.Tv * self.dt
        self.position += dp
        return self.position


class PIDController:
    """PID控制器"""
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, dt=0.01,
                 output_min=0, output_max=1.0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.error_prev = 0.0
        self.output = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt
        integral_limit = self.output_max / max(self.Ki, 0.0001)
        self.integral = np.clip(self.integral, -integral_limit, integral_limit)

        derivative = (error - self.error_prev) / self.dt
        self.error_prev = error

        self.output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.output = np.clip(self.output, self.output_min, self.output_max)
        return self.output


class CascadePIDController:
    """串级PID控制器
    外环: 液位PID (输出为流量设定值)
    内环: 流量PID (输出为阀门开度)
    """
    def __init__(self, dt=0.01,
                 outer_Kp=1.5, outer_Ki=0.05, outer_Kd=0.3,
                 inner_Kp=3.0, inner_Ki=0.5, inner_Kd=0.05,
                 output_max=1.0):
        self.dt = dt
        # 外环 (液位环) - 输出为内环设定值(流量)
        self.outer_pid = PIDController(
            outer_Kp, outer_Ki, outer_Kd, dt,
            output_min=0, output_max=output_max * 2
        )
        # 内环 (流量环) - 输出为阀门开度
        self.inner_pid = PIDController(
            inner_Kp, inner_Ki, inner_Kd, dt,
            output_min=0, output_max=output_max
        )
        self.flow_setpoint = 0.0

    def compute(self, level_setpoint, level_measurement, flow_measurement):
        # 外环: 液位 -> 流量设定
        self.flow_setpoint = self.outer_pid.compute(level_setpoint, level_measurement)
        # 内环: 流量设定 -> 阀门开度
        valve_cmd = self.inner_pid.compute(self.flow_setpoint, flow_measurement)
        return valve_cmd, self.flow_setpoint


def simulate(duration=60.0, dt=0.01):
    """运行仿真"""
    steps = int(duration / dt)
    t = np.arange(steps) * dt

    # 设定液位曲线
    setpoint = np.ones(steps) * 0.5
    setpoint[int(15/dt):int(30/dt)] = 1.0   # 升到1.0m
    setpoint[int(30/dt):int(45/dt)] = 0.3   # 降到0.3m
    setpoint[int(45/dt):] = 0.8              # 升到0.8m

    # 方案1: 单回路PID
    tank1 = WaterTankModel(A=1.0, k=0.5, dt=dt)
    valve1 = ValveModel(Tv=0.5, dt=dt)
    pid1 = PIDController(Kp=0.8, Ki=0.08, Kd=0.2, dt=dt)

    # 方案2: 串级PID
    tank2 = WaterTankModel(A=1.0, k=0.5, dt=dt)
    valve2 = ValveModel(Tv=0.5, dt=dt)
    cascade = CascadePIDController(
        dt=dt, outer_Kp=1.5, outer_Ki=0.05, outer_Kd=0.3,
        inner_Kp=3.0, inner_Ki=0.5, inner_Kd=0.05
    )

    # 结果存储
    res = {
        't': t, 'setpoint': setpoint,
        'level_pid': np.zeros(steps), 'level_cascade': np.zeros(steps),
        'valve_pid': np.zeros(steps), 'valve_cascade': np.zeros(steps),
        'flow_sp': np.zeros(steps), 'disturbance': np.zeros(steps),
    }

    for i in range(steps):
        # 扰动: t=20~22s 放水扰动
        dist = -0.3 if (20 < t[i] < 22) else 0.0
        res['disturbance'][i] = dist

        # 方案1: 单回路PID
        u1 = pid1.compute(setpoint[i], tank1.h)
        v1 = valve1.update(u1)
        flow1 = v1 * 2.0  # 最大流量2m³/s
        res['level_pid'][i] = tank1.update(flow1, dist)
        res['valve_pid'][i] = v1

        # 方案2: 串级PID (用液位变化率近似流量测量)
        if i > 0:
            flow_meas = (tank2.h - prev_h) / dt + 0.5 * np.sqrt(max(tank2.h, 0))
        else:
            flow_meas = 0
        flow_meas = max(0, flow_meas)

        v2_cmd, flow_sp = cascade.compute(setpoint[i], tank2.h, flow_meas)
        v2 = valve2.update(v2_cmd)
        flow2 = v2 * 2.0
        prev_h = tank2.h
        res['level_cascade'][i] = tank2.update(flow2, dist)
        res['valve_cascade'][i] = v2
        res['flow_sp'][i] = flow_sp

    return res


def plot_results(r):
    """绘制结果"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # 液位响应
    axes[0].plot(r['t'], r['setpoint'], 'k--', linewidth=2, label='设定液位')
    axes[0].plot(r['t'], r['level_pid'], 'b-', linewidth=1, label='单回路PID', alpha=0.8)
    axes[0].plot(r['t'], r['level_cascade'], 'r-', linewidth=1.5, label='串级PID')
    # 扰动标注
    dist_mask = r['disturbance'] != 0
    if np.any(dist_mask):
        axes[0].axvspan(r['t'][dist_mask][0], r['t'][dist_mask][-1],
                        alpha=0.15, color='orange', label='扰动区间')
    axes[0].set_ylabel('液位 (m)')
    axes[0].set_title('水位控制仿真 - 单回路PID vs 串级PID')
    axes[0].legend(loc='best')
    axes[0].grid(True, alpha=0.3)

    # 阀门开度
    axes[1].plot(r['t'], r['valve_pid'], 'b-', label='单回路PID阀门', alpha=0.7)
    axes[1].plot(r['t'], r['valve_cascade'], 'r-', label='串级PID阀门', alpha=0.7)
    axes[1].set_ylabel('阀门开度')
    axes[1].set_title('阀门动作对比')
    axes[1].legend(loc='best')
    axes[1].grid(True, alpha=0.3)

    # 扰动
    axes[2].plot(r['t'], r['disturbance'], 'r-', linewidth=2)
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('扰动流量 (m³/s)')
    axes[2].set_title('外部扰动')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('water_level_control_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')

    for name, y in [('单回路PID', r['level_pid']), ('串级PID', r['level_cascade'])]:
        error = r['setpoint'] - y
        mae = np.mean(np.abs(error))
        rmse = np.sqrt(np.mean(error**2))
        print(f"{name:10s} | MAE={mae:.4f}m | RMSE={rmse:.4f}m")


if __name__ == '__main__':
    print("=" * 60)
    print("  水位控制仿真 (单回路PID vs 串级PID)")
    print("=" * 60)
    results = simulate()
    plot_results(results)
