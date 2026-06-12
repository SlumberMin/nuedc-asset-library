#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
温度控制仿真 - PID + 模糊PID + Smith预估器
============================================
适用于电赛温度控制类题目(如加热炉、恒温箱)
包含三种控制策略对比
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class TemperaturePlant:
    """温度被控对象模型 (带纯滞后的一阶惯性环节)
    G(s) = K * exp(-L*s) / (T*s + 1)
    """
    def __init__(self, K=2.0, T=30.0, L=5.0, dt=0.1, T_ambient=25.0):
        self.K = K              # 增益 (°C/W)
        self.T = T              # 时间常数 (s)
        self.L = L              # 纯滞后时间 (s)
        self.dt = dt
        self.T_ambient = T_ambient

        self.temperature = T_ambient
        self.buffer = [0.0] * int(L / dt)  # 滞后缓冲

    def update(self, power):
        """更新温度，power为加热功率(W)"""
        # 纯滞后: 将输入延迟L秒
        self.buffer.append(power)
        delayed_power = self.buffer.pop(0)

        # 一阶惯性: dT/dt = (K*u - (T - T_ambient)) / tau
        dT = (self.K * delayed_power - (self.temperature - self.T_ambient)) / self.T
        self.temperature += dT * self.dt
        return self.temperature


class PIDController:
    """PID控制器(带积分抗饱和)"""
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, dt=0.1, output_min=0, output_max=100):
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

        # 积分限幅 (抗饱和)
        integral_max = self.output_max / max(self.Ki, 0.001)
        self.integral = np.clip(self.integral, -integral_max, integral_max)

        derivative = (error - self.error_prev) / self.dt
        self.error_prev = error

        self.output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.output = np.clip(self.output, self.output_min, self.output_max)
        return self.output


class FuzzyPIDController:
    """模糊PID控制器
    根据误差e和误差变化率ec，模糊推理调整Kp, Ki, Kd
    """
    # 模糊规则表 (NB=负大, NS=负小, ZO=零, PS=正小, PB=正大)
    # e: 误差等级, ec: 误差变化率等级
    DELTA_KP = [
        # ec:  NB    NS    ZO    PS    PB
        [  2,   2,   1,   1,   0],  # e=NB
        [  2,   1,   1,   0,  -1],  # e=NS
        [  1,   1,   0,  -1,  -1],  # e=ZO
        [  1,   0,  -1,  -1,  -2],  # e=PS
        [  0,  -1,  -1,  -2,  -2],  # e=PB
    ]
    DELTA_KI = [
        [-2,  -2,  -1,  -1,   0],
        [-2,  -1,  -1,   0,   1],
        [-1,  -1,   0,   1,   1],
        [-1,   0,   1,   1,   2],
        [ 0,   1,   1,   2,   2],
    ]
    DELTA_KD = [
        [ 2,   1,   0,  -1,  -2],
        [ 1,   1,   0,  -1,  -1],
        [ 0,   0,   0,   0,   0],
        [-1,  -1,   0,   1,   1],
        [-2,  -1,   0,   1,   2],
    ]

    def __init__(self, Kp=5.0, Ki=0.1, Kd=2.0, dt=0.1,
                 output_min=0, output_max=100,
                 e_range=(-10, 10), ec_range=(-5, 5)):
        self.Kp_base, self.Ki_base, self.Kd_base = Kp, Ki, Kd
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.output_min = output_min
        self.output_max = output_max
        self.e_range = e_range
        self.ec_range = ec_range
        self.integral = 0.0
        self.error_prev = 0.0
        self.output = 0.0

        # 量化因子
        self.ke = 4.0 / (e_range[1] - e_range[0])  # 映射到[-4,4]
        self.kec = 4.0 / (ec_range[1] - ec_range[0])

    def _fuzzy_index(self, x, n=5):
        """将连续值映射到模糊集索引"""
        # x in [-4, 4] -> index in [0, 4]
        idx = int((x + 4) / 8 * (n - 1))
        return max(0, min(n - 1, idx))

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        d_error = (error - self.error_prev) / self.dt
        self.error_prev = error

        # 模糊化
        e_fuzzy = np.clip(self.ke * error, -4, 4)
        ec_fuzzy = np.clip(self.kec * d_error, -4, 4)

        ei = self._fuzzy_index(e_fuzzy)
        eci = self._fuzzy_index(ec_fuzzy)

        # 模糊推理 + 解模糊 (查表)
        scale = 0.3  # 调整因子
        self.Kp = self.Kp_base + scale * self.Kp_base * self.DELTA_KP[ei][eci] / 2
        self.Ki = self.Ki_base + scale * self.Ki_base * self.DELTA_KI[ei][eci] / 2
        self.Kd = self.Kd_base + scale * self.Kd_base * self.DELTA_KD[ei][eci] / 2
        self.Kp = max(0, self.Kp)
        self.Ki = max(0, self.Ki)
        self.Kd = max(0, self.Kd)

        self.integral += error * self.dt
        integral_max = self.output_max / max(self.Ki, 0.001)
        self.integral = np.clip(self.integral, -integral_max, integral_max)

        self.output = (self.Kp * error + self.Ki * self.integral
                       + self.Kd * d_error)
        self.output = np.clip(self.output, self.output_min, self.output_max)
        return self.output


class SmithPredictor:
    """Smith预估器 - 补偿纯滞后
    在PID控制器的基础上，利用模型预测消除滞后影响
    """
    def __init__(self, K=2.0, T=30.0, L=5.0, dt=0.1,
                 Kp=5.0, Ki=0.15, Kd=3.0, T_ambient=25.0):
        self.dt = dt
        self.T_ambient = T_ambient

        # 内部模型 (无滞后)
        self.model_output = T_ambient
        # 滞后补偿缓冲
        self.delay_buffer = [T_ambient] * int(L / dt)

        self.K, self.T = K, T
        self.pid = PIDController(Kp, Ki, Kd, dt, output_min=0, output_max=100)

    def compute(self, setpoint, measurement):
        """Smith预估器补偿计算，消除纯滞后对控制品质的影响。

        补偿原理：
            1. 利用内部模型计算无滞后输出 model_output（一阶惯性响应）
            2. 将无滞后输出经过延迟缓冲得到 model_delayed（模拟实际滞后响应）
            3. 计算补偿量 = 无滞后输出 - 滞后输出，叠加到实际测量值上
            4. 用补偿后的反馈信号送入PID控制器，使PID"看到"的反馈
               等效于无滞后系统，从而避免因纯滞后引起的超调和振荡

        Args:
            setpoint (float): 设定温度值 (°C)
            measurement (float): 实际测量温度值 (°C)，来自带滞后的被控对象

        Returns:
            float: PID控制器输出的加热功率百分比，范围 [output_min, output_max]
        """
        # 模型无滞后输出
        model_ff = self.K * self.pid.output
        dT_model = (model_ff - (self.model_output - self.T_ambient)) / self.T
        self.model_output += dT_model * self.dt

        # 模型滞后输出 (从缓冲区读取)
        model_delayed = self.delay_buffer.pop(0)
        self.delay_buffer.append(self.model_output)

        # Smith补偿: 调整反馈信号
        compensated_feedback = measurement + (self.model_output - model_delayed)

        # PID计算
        return self.pid.compute(setpoint, compensated_feedback)


def simulate(duration=200.0, dt=0.1):
    """运行温度控制仿真"""
    steps = int(duration / dt)
    t = np.arange(steps) * dt

    # 设置点
    setpoint = np.ones(steps) * 25.0
    setpoint[int(5/dt):] = 60.0   # t=5s: 升温到60°C
    setpoint[int(80/dt):] = 45.0  # t=80s: 降温到45°C
    setpoint[int(130/dt):] = 70.0 # t=130s: 升温到70°C

    # 创建三个控制器 + 三个被控对象
    plant1 = TemperaturePlant(K=2.0, T=30.0, L=5.0, dt=dt)
    plant2 = TemperaturePlant(K=2.0, T=30.0, L=5.0, dt=dt)
    plant3 = TemperaturePlant(K=2.0, T=30.0, L=5.0, dt=dt)

    pid = PIDController(Kp=5.0, Ki=0.15, Kd=3.0, dt=dt)
    fuzzy_pid = FuzzyPIDController(Kp=5.0, Ki=0.15, Kd=3.0, dt=dt)
    smith = SmithPredictor(K=2.0, T=30.0, L=5.0, dt=dt, Kp=6.0, Ki=0.2, Kd=4.0)

    # 结果
    results = {
        't': t, 'setpoint': setpoint,
        'pid_temp': np.zeros(steps), 'fuzzy_temp': np.zeros(steps),
        'smith_temp': np.zeros(steps),
        'pid_u': np.zeros(steps), 'fuzzy_u': np.zeros(steps),
        'smith_u': np.zeros(steps),
        'fuzzy_kp': np.zeros(steps), 'fuzzy_ki': np.zeros(steps),
        'fuzzy_kd': np.zeros(steps),
    }

    for i in range(steps):
        sp = setpoint[i]

        u1 = pid.compute(sp, plant1.temperature)
        results['pid_temp'][i] = plant1.update(u1)
        results['pid_u'][i] = u1

        u2 = fuzzy_pid.compute(sp, plant2.temperature)
        results['fuzzy_temp'][i] = plant2.update(u2)
        results['fuzzy_u'][i] = u2
        results['fuzzy_kp'][i] = fuzzy_pid.Kp
        results['fuzzy_ki'][i] = fuzzy_pid.Ki
        results['fuzzy_kd'][i] = fuzzy_pid.Kd

        u3 = smith.compute(sp, plant3.temperature)
        results['smith_temp'][i] = plant3.update(u3)
        results['smith_u'][i] = u3

    return results


def plot_results(r):
    """绘制结果"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # 温度响应
    axes[0].plot(r['t'], r['setpoint'], 'k--', linewidth=2, label='设定温度')
    axes[0].plot(r['t'], r['pid_temp'], 'b-', linewidth=1, label='PID', alpha=0.8)
    axes[0].plot(r['t'], r['fuzzy_temp'], 'r-', linewidth=1, label='模糊PID', alpha=0.8)
    axes[0].plot(r['t'], r['smith_temp'], 'g-', linewidth=1.5, label='Smith预估+PID')
    axes[0].set_ylabel('温度 (°C)')
    axes[0].set_title('温度控制仿真 - PID vs 模糊PID vs Smith预估')
    axes[0].legend(loc='best')
    axes[0].grid(True, alpha=0.3)

    # 控制量
    axes[1].plot(r['t'], r['pid_u'], 'b-', label='PID', alpha=0.7)
    axes[1].plot(r['t'], r['fuzzy_u'], 'r-', label='模糊PID', alpha=0.7)
    axes[1].plot(r['t'], r['smith_u'], 'g-', label='Smith预估', alpha=0.7)
    axes[1].set_ylabel('加热功率 (%)')
    axes[1].set_title('控制量对比')
    axes[1].legend(loc='best')
    axes[1].grid(True, alpha=0.3)

    # 模糊PID参数自适应
    axes[2].plot(r['t'], r['fuzzy_kp'], 'r-', label='Kp')
    axes[2].plot(r['t'], r['fuzzy_ki'] * 10, 'b-', label='Ki×10')
    axes[2].plot(r['t'], r['fuzzy_kd'], 'g-', label='Kd')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('参数值')
    axes[2].set_title('模糊PID参数自适应过程')
    axes[2].legend(loc='best')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('temperature_control_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')

    # 性能指标
    for name, temp in [('PID', r['pid_temp']), ('模糊PID', r['fuzzy_temp']),
                        ('Smith预估', r['smith_temp'])]:
        error = r['setpoint'] - temp
        mae = np.mean(np.abs(error[100:]))
        rmse = np.sqrt(np.mean(error[100:]**2))
        print(f"{name:10s} | MAE={mae:.2f}°C | RMSE={rmse:.2f}°C")


if __name__ == '__main__':
    print("=" * 60)
    print("  温度控制仿真 (PID + 模糊PID + Smith预估器)")
    print("=" * 60)
    results = simulate()
    plot_results(results)
