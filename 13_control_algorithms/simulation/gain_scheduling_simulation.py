#!/usr/bin/env python3
"""
增益调度PID仿真 - 对比硬切换 vs 插值法

仿真场景: 二阶系统，其参数随工作点变化
  低速段: 惯量小, 响应快
  高速段: 惯量大, 响应慢

对比:
  1. 固定PID (取中等参数)
  2. 硬切换增益调度PID
  3. 插值法增益调度PID
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ============================================================
# 被控对象: 变参数二阶系统
# ============================================================
class VariablePlant:
    """增益和时间常数随工作点变化的二阶系统"""

    def __init__(self):
        self.x1 = 0.0  # 位置
        self.x2 = 0.0  # 速度

    def params_at(self, x1):
        """根据当前位置返回系统参数"""
        # 低速段: K=1.0, tau=0.05
        # 高速段: K=0.6, tau=0.15
        ratio = np.clip(abs(x1) / 10.0, 0, 1)
        K = 1.0 - 0.4 * ratio      # 增益下降
        tau = 0.05 + 0.1 * ratio    # 时间常数增大
        return K, tau

    def update(self, u, dt):
        K, tau = self.params_at(self.x1)
        # dx2/dt = (K*u - x2) / tau
        self.x2 += (K * u - self.x2) / tau * dt
        self.x1 += self.x2 * dt
        return self.x1

    def reset(self):
        self.x1 = 0.0
        self.x2 = 0.0


# ============================================================
# PID控制器
# ============================================================
class PID:
    def __init__(self, kp, ki, kd, out_min=-100, out_max=100):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, setpoint, fb, dt):
        err = setpoint - fb
        self.integral += err * dt
        deriv = (err - self.prev_error) / dt
        self.prev_error = err
        out = self.kp * err + self.ki * self.integral + self.kd * deriv
        # anti-windup
        if out > self.out_max:
            self.integral -= err * dt
            out = self.out_max
        elif out < self.out_min:
            self.integral -= err * dt
            out = self.out_min
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ============================================================
# 硬切换增益调度PID
# ============================================================
class ScheduledPID:
    def __init__(self, regions, out_min=-100, out_max=100):
        """regions: list of (threshold, kp, ki, kd)"""
        self.regions = sorted(regions, key=lambda r: r[0])
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.active_idx = 0

    def _find_region(self, sched_var):
        for i in range(len(self.regions) - 1, -1, -1):
            if sched_var >= self.regions[i][0]:
                return i
        return 0

    def update(self, setpoint, fb, sched_var, dt):
        new_idx = self._find_region(abs(sched_var))
        if new_idx != self.active_idx:
            self.integral = 0.0  # 切换时重置积分
            self.active_idx = new_idx

        _, kp, ki, kd = self.regions[self.active_idx]
        err = setpoint - fb
        self.integral += err * dt
        deriv = (err - self.prev_error) / dt
        self.prev_error = err
        out = kp * err + ki * self.integral + kd * deriv
        if out > self.out_max:
            self.integral -= err * dt
            out = self.out_max
        elif out < self.out_min:
            self.integral -= err * dt
            out = self.out_min
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.active_idx = 0


# ============================================================
# 插值法增益调度PID
# ============================================================
class InterpolatingPID:
    def __init__(self, table, out_min=-100, out_max=100):
        """table: list of (sched_val, kp, ki, kd), sorted by sched_val"""
        self.table = sorted(table, key=lambda r: r[0])
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_deriv = 0.0
        self.deriv_alpha = 0.15

    def _interpolate(self, v):
        vals = [r[0] for r in self.table]
        if v <= vals[0]:
            return self.table[0][1], self.table[0][2], self.table[0][3]
        if v >= vals[-1]:
            return self.table[-1][1], self.table[-1][2], self.table[-1][3]
        for i in range(len(vals) - 1):
            if vals[i] <= v < vals[i + 1]:
                t = (v - vals[i]) / (vals[i + 1] - vals[i])
                kp = self.table[i][1] + t * (self.table[i + 1][1] - self.table[i][1])
                ki = self.table[i][2] + t * (self.table[i + 1][2] - self.table[i][2])
                kd = self.table[i][3] + t * (self.table[i + 1][3] - self.table[i][3])
                return kp, ki, kd
        return self.table[-1][1], self.table[-1][2], self.table[-1][3]

    def update(self, setpoint, fb, sched_var, dt):
        kp, ki, kd = self._interpolate(abs(sched_var))
        err = setpoint - fb
        self.integral += err * dt
        raw_d = (err - self.prev_error) / dt
        filt_d = self.deriv_alpha * raw_d + (1 - self.deriv_alpha) * self.prev_deriv
        self.prev_deriv = filt_d
        self.prev_error = err
        out = kp * err + ki * self.integral + kd * filt_d
        if out > self.out_max:
            self.integral -= err * dt
            out = self.out_max
        elif out < self.out_min:
            self.integral -= err * dt
            out = self.out_min
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_deriv = 0.0


# ============================================================
# 仿真主程序
# ============================================================
def run_simulation():
    dt = 0.001
    T = 3.0
    steps = int(T / dt)
    t = np.linspace(0, T, steps)

    # 目标: 阶跃 0→5 (t=0.5s), 然后 5→8 (t=1.5s)
    setpoint = np.zeros(steps)
    setpoint[int(0.5 / dt):] = 5.0
    setpoint[int(1.5 / dt):] = 8.0

    # --- 固定PID (中等参数) ---
    plant1 = VariablePlant()
    pid_fixed = PID(kp=8.0, ki=50.0, kd=0.3, out_min=-100, out_max=100)
    y_fixed = np.zeros(steps)

    # --- 硬切换增益调度 ---
    plant2 = VariablePlant()
    pid_sched = ScheduledPID(
        regions=[
            (0.0,  12.0, 80.0, 0.2),   # 低速: 大增益
            (3.0,  8.0,  50.0, 0.3),    # 中速: 中等增益
            (6.0,  5.0,  30.0, 0.5),    # 高速: 小增益
        ],
        out_min=-100, out_max=100
    )
    y_sched = np.zeros(steps)

    # --- 插值法增益调度 ---
    plant3 = VariablePlant()
    pid_interp = InterpolatingPID(
        table=[
            (0.0,  12.0, 80.0, 0.2),
            (2.0,  10.0, 65.0, 0.25),
            (4.0,  8.0,  50.0, 0.3),
            (6.0,  6.0,  40.0, 0.4),
            (8.0,  5.0,  30.0, 0.5),
        ],
        out_min=-100, out_max=100
    )
    y_interp = np.zeros(steps)

    for i in range(steps):
        sp = setpoint[i]
        y_fixed[i] = plant1.update(pid_fixed.update(sp, plant1.x1, dt), dt)
        y_sched[i] = plant2.update(pid_sched.update(sp, plant2.x1, plant2.x1, dt), dt)
        y_interp[i] = plant3.update(pid_interp.update(sp, plant3.x1, plant3.x1, dt), dt)

    # ============================================================
    # 绘图
    # ============================================================
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # 输出对比
    axes[0].plot(t, setpoint, 'k--', lw=2, label='设定值')
    axes[0].plot(t, y_fixed, 'b-', lw=1.2, label='固定PID')
    axes[0].plot(t, y_sched, 'r-', lw=1.2, label='硬切换增益调度')
    axes[0].plot(t, y_interp, 'g-', lw=1.2, label='插值法增益调度')
    axes[0].set_ylabel('输出')
    axes[0].set_title('增益调度PID仿真对比')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 误差
    axes[1].plot(t, setpoint - y_fixed, 'b-', lw=1, label='固定PID')
    axes[1].plot(t, setpoint - y_sched, 'r-', lw=1, label='硬切换')
    axes[1].plot(t, setpoint - y_interp, 'g-', lw=1, label='插值法')
    axes[1].set_ylabel('误差')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # 有效Kp
    kp_fixed_arr = np.full(steps, 8.0)
    kp_sched_arr = np.zeros(steps)
    kp_interp_arr = np.zeros(steps)
    for i in range(steps):
        idx = pid_sched._find_region(abs(y_sched[i]))
        kp_sched_arr[i] = pid_sched.regions[idx][1]
        kp_interp_arr[i] = pid_interp._interpolate(abs(y_interp[i]))[0]

    axes[2].plot(t, kp_fixed_arr, 'b--', lw=1.2, label='固定Kp=8.0')
    axes[2].plot(t, kp_sched_arr, 'r-', lw=1.2, label='硬切换Kp')
    axes[2].plot(t, kp_interp_arr, 'g-', lw=1.2, label='插值Kp')
    axes[2].set_ylabel('有效Kp')
    axes[2].set_xlabel('时间 (s)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('gain_scheduling_result.png', dpi=150)
    print("仿真完成，结果已保存为 gain_scheduling_result.png")

    # 性能指标
    print("\n性能指标对比:")
    for name, y in [('固定PID', y_fixed), ('硬切换', y_sched), ('插值法', y_interp)]:
        err = setpoint - y
        mae = np.mean(np.abs(err[int(0.6 / dt):]))
        settling_idx = None
        for i in range(steps - 1, int(0.5 / dt), -1):
            if abs(err[i]) > 0.05:
                settling_idx = i
                break
        settling_time = t[settling_idx] - 0.5 if settling_idx else 0
        overshoot = (np.max(y[int(0.5 / dt):int(1.5 / dt)]) - 5.0) / 5.0 * 100
        print(f"  {name}: MAE={mae:.4f}, 阶跃响应超调={overshoot:.1f}%")


if __name__ == '__main__':
    run_simulation()
