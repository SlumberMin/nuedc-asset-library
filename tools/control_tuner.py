#!/usr/bin/env python3
"""
控制参数整定工具 - nuedc-asset-library
功能：PID自整定、ADRC参数设计、MPC控制仿真、Ziegler-Nichols整定
作者：电赛自动迭代引擎 V3
"""

import argparse
import json
import math
import sys


# ─── PID控制器 ────────────────────────────────────────────────────

class PIDController:
    """离散PID控制器（带抗积分饱和）"""

    def __init__(self, kp, ki, kd, dt, output_min=-1e6, output_max=1e6):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_min = output_min
        self.output_max = output_max
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def update(self, setpoint, measurement):
        """计算PID输出"""
        error = setpoint - measurement
        # P项
        p_term = self.kp * error
        # I项（梯形积分）
        self._integral += 0.5 * (error + self._prev_error) * self.dt
        i_term = self.ki * self._integral
        # D项
        if self._first:
            d_term = 0.0
            self._first = False
        else:
            d_term = self.kd * (error - self._prev_error) / self.dt
        self._prev_error = error

        output = p_term + i_term + d_term
        # 抗积分饱和
        if output > self.output_max:
            output = self.output_max
            self._integral -= 0.5 * (error + self._prev_error) * self.dt
        elif output < self.output_min:
            output = self.output_min
            self._integral -= 0.5 * (error + self._prev_error) * self.dt

        return output

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True


# ─── Ziegler-Nichols整定 ─────────────────────────────────────────

def ziegler_nichols(ku, tu, method='classic'):
    """
    Ziegler-Nichols临界比例法整定
    ku: 临界增益, tu: 临界振荡周期
    method: classic / pessen / some_overshoot / no_overshoot
    """
    rules = {
        'classic': {'kp': 0.6, 'ti': 0.5, 'td': 0.125},
        'pessen': {'kp': 0.7, 'ti': 0.4, 'td': 0.15},
        'some_overshoot': {'kp': 0.33, 'ti': 0.5, 'td': 0.33},
        'no_overshoot': {'kp': 0.2, 'ti': 0.5, 'td': 0.33},
    }
    r = rules.get(method, rules['classic'])
    kp = r['kp'] * ku
    ti = r['ti'] * tu
    td = r['td'] * tu
    ki = kp / ti
    kd = kp * td
    return {'kp': kp, 'ki': ki, 'kd': kd, 'ti': ti, 'td': td, 'method': method}


def lambda_tuning(k, tau, theta, closed_loop_bw=0.5):
    """
    Lambda整定法（基于一阶加纯滞后模型）
    k: 过程增益, tau: 时间常数, theta: 纯滞后
    """
    lambd = max(tau / closed_loop_bw, 4 * theta)  # 闭环时间常数
    kp = tau / (k * (lambd + theta))
    ti = tau
    td = theta / 2
    ki = kp / ti
    kd = kp * td
    return {'kp': kp, 'ki': ki, 'kd': kd, 'lambda': lambd}


def cohen_coon(k, tau, theta):
    """
    Cohen-Coon整定法
    k: 过程增益, tau: 时间常数, theta: 纯滞后
    """
    r = theta / tau
    kp = (1 / k) * (1 + 0.35 * r / (1 - r))
    ti = theta * (3.3 - 3.0 * r) / (1 + 1.2 * r)
    td = theta * 0.37 * (1 - r) / (1 + 0.2 * r)
    ki = kp / ti
    kd = kp * td
    return {'kp': kp, 'ki': ki, 'kd': kd, 'r': r}


# ─── ADRC (自抗扰控制) ────────────────────────────────────────────

class ADRCController:
    """
    一阶ADRC控制器
    结构：跟踪微分器(TD) + 扩张状态观测器(ESO) + 非线性反馈(NLSEF)
    """

    def __init__(self, wc, wo, b0, dt, zeta=1.0):
        """
        wc: 控制器带宽, wo: 观测器带宽, b0: 控制增益估计
        dt: 采样时间, zeta: 阻尼比
        """
        self.wc = wc
        self.wo = wo
        self.b0 = b0
        self.dt = dt
        self.zeta = zeta
        # ESO状态
        self.z1 = 0.0  # 位置估计
        self.z2 = 0.0  # 速度/总扰动估计
        # TD状态
        self.v1 = 0.0  # 跟踪信号
        # 控制器增益
        self.kp = wc
        # ESO增益
        self.beta1 = 2 * wo
        self.beta2 = wo * wo

    def _fal(self, e, alpha, delta):
        """非线性函数"""
        if abs(e) > delta:
            return math.copysign(abs(e)**alpha, e)
        else:
            return e / (delta**(1 - alpha))

    def update(self, setpoint, measurement, u_prev=0.0):
        """ADRC一步计算"""
        y = measurement
        # ESO更新
        e_eso = self.z1 - y
        self.z1 += self.dt * (self.z2 + self.beta1 * e_eso + self.b0 * u_prev)
        self.z2 += self.dt * (self.beta2 * e_eso)

        # TD更新（一阶跟踪）
        self.v1 += self.dt * self.wc * (setpoint - self.v1)

        # 控制律
        e_ctrl = self.v1 - self.z1
        u0 = self.kp * e_ctrl
        u = (u0 - self.z2) / self.b0

        return u

    def reset(self):
        self.z1 = 0.0
        self.z2 = 0.0
        self.v1 = 0.0


def design_adrc_params(settling_time, overshoot_limit=0.1, k_est=1.0):
    """
    根据性能指标设计ADRC参数
    settling_time: 期望调节时间(s)
    overshoot_limit: 超调量限制(0~1)
    k_est: 控制增益估计
    """
    wc = 4.0 / settling_time  # 控制器带宽
    wo = 3.0 * wc  # 观测器带宽（通常为控制器带宽的3~5倍）
    b0 = k_est
    zeta = max(0.7, 1.0 - overshoot_limit)
    return {'wc': wc, 'wo': wo, 'b0': b0, 'zeta': zeta,
            'kp': wc, 'beta1': 2*wo, 'beta2': wo*wo}


# ─── MPC (模型预测控制，简化版) ───────────────────────────────────

class MPCController:
    """
    简化MPC控制器（基于一阶模型）
    假设模型: y(k+1) = a*y(k) + b*u(k)
    """

    def __init__(self, a, b, Np, Nc, Q, R, dt,
                 u_min=-1e6, u_max=1e6, du_min=-1e6, du_max=1e6):
        """
        a, b: 模型参数, Np: 预测步长, Nc: 控制步长
        Q: 输出权重, R: 控制增量权重
        """
        self.a = a
        self.b = b
        self.Np = Np
        self.Nc = Nc
        self.Q = Q
        self.R = R
        self.dt = dt
        self.u_min = u_min
        self.u_max = u_max
        self.du_min = du_min
        self.du_max = du_max
        self.y_state = 0.0
        self.u_prev = 0.0

    def _predict(self, y0, du_seq):
        """预测Np步输出"""
        y = y0
        u = self.u_prev
        predictions = []
        for k in range(self.Np):
            if k < len(du_seq):
                u += du_seq[k]
            y = self.a * y + self.b * u
            predictions.append(y)
        return predictions

    def update(self, setpoint, measurement):
        """MPC一步优化（简化梯度下降法）"""
        self.y_state = measurement
        # 初始化du序列
        du = [0.0] * self.Nc
        # 简单优化：逐步调整使预测输出接近设定值
        for iteration in range(50):
            preds = self._predict(self.y_state, du)
            # 计算梯度并更新
            for k in range(self.Nc):
                # 数值梯度
                du_test = list(du)
                du_test[k] += 0.001
                preds_test = self._predict(self.y_state, du_test)
                # 目标函数差分
                cost_diff = 0.0
                for j in range(self.Np):
                    err_new = (setpoint - preds_test[j])**2 * self.Q
                    err_old = (setpoint - preds[j])**2 * self.Q
                    cost_diff += (err_new - err_old)
                cost_diff += self.R * (2*du[k]*0.001 + 0.001**2)
                # 梯度下降
                du[k] -= 0.5 * cost_diff / 0.001
                # 约束
                du[k] = max(self.du_min, min(self.du_max, du[k]))

        # 应用第一个控制增量
        self.u_prev += du[0]
        self.u_prev = max(self.u_min, min(self.u_max, self.u_prev))
        return self.u_prev, du

    def reset(self):
        self.y_state = 0.0
        self.u_prev = 0.0


def identify_fopdt(y_data, u_data, dt):
    """
    一阶加纯滞后(FOPDT)模型辨识
    y_data: 输出数据, u_data: 输入数据, dt: 采样时间
    返回: {k, tau, theta}
    """
    # 简单两点法
    y_ss = y_data[-1]
    u_ss = u_data[-1]
    if abs(u_ss) < 1e-10:
        return {'k': 1.0, 'tau': 1.0, 'theta': 0.0}

    k = y_ss / u_ss  # 增益

    # 找63.2%响应点
    y_target = y_ss * 0.632
    tau_idx = 0
    for i in range(len(y_data)):
        if y_data[i] >= y_target:
            tau_idx = i
            break
    tau = tau_idx * dt

    # 找10%响应点作为滞后
    y_10 = y_ss * 0.1
    theta_idx = 0
    for i in range(len(y_data)):
        if y_data[i] >= y_10:
            theta_idx = i
            break
    theta = theta_idx * dt

    return {'k': k, 'tau': max(tau, dt), 'theta': max(theta, 0.0)}


# ─── 闭环仿真 ────────────────────────────────────────────────────

def simulate_pid(kp, ki, kd, setpoint, plant_k, plant_tau, plant_theta,
                 dt, t_end, u_min=-100, u_max=100):
    """
    PID闭环仿真（一阶加纯滞后对象）
    返回: {time[], setpoint[], output[], control[]}
    """
    pid = PIDController(kp, ki, kd, dt, u_min, u_max)
    # 对象状态缓冲
    delay_steps = max(1, int(plant_theta / dt))
    state_buffer = [0.0] * delay_steps
    y = 0.0
    results = {"time": [], "setpoint": [], "output": [], "control": []}
    steps = int(t_end / dt)

    for i in range(steps):
        t = i * dt
        u = pid.update(setpoint, y)
        # 一阶对象
        dy = (-y + plant_k * state_buffer[0]) / plant_tau * dt
        y += dy
        # 更新延迟缓冲
        state_buffer.append(u)
        state_buffer.pop(0)

        results["time"].append(t)
        results["setpoint"].append(setpoint)
        results["output"].append(y)
        results["control"].append(u)

    return results


def simulate_adrc(wc, wo, b0, setpoint, plant_k, plant_tau, dt, t_end):
    """ADRC闭环仿真"""
    adrc = ADRCController(wc, wo, b0, dt)
    y = 0.0
    u = 0.0
    results = {"time": [], "setpoint": [], "output": [], "control": []}
    steps = int(t_end / dt)

    for i in range(steps):
        t = i * dt
        u = adrc.update(setpoint, y, u)
        dy = (-y + plant_k * u) / plant_tau * dt
        y += dy

        results["time"].append(t)
        results["setpoint"].append(setpoint)
        results["output"].append(y)
        results["control"].append(u)

    return results


# ─── 性能指标计算 ─────────────────────────────────────────────────

def calc_performance(results):
    """计算阶跃响应性能指标"""
    setpoint = results["setpoint"][-1]
    output = results["output"]
    time = results["time"]
    dt = time[1] - time[0] if len(time) > 1 else 0.01

    # 上升时间（10%到90%）
    y_10 = setpoint * 0.1
    y_90 = setpoint * 0.9
    t_10, t_90 = None, None
    for i, y in enumerate(output):
        if t_10 is None and y >= y_10:
            t_10 = time[i]
        if t_90 is None and y >= y_90:
            t_90 = time[i]

    rise_time = (t_90 - t_10) if (t_10 and t_90) else None

    # 超调量
    y_max = max(output)
    overshoot = max(0, (y_max - setpoint) / max(abs(setpoint), 1e-10) * 100)

    # 调节时间（进入±2%带）
    settling_time = None
    band = abs(setpoint) * 0.02
    for i in range(len(output) - 1, -1, -1):
        if abs(output[i] - setpoint) > band:
            settling_time = time[min(i + 1, len(time) - 1)]
            break

    # 稳态误差
    ss_error = abs(output[-1] - setpoint)

    # IAE / ISE
    iae = sum(abs(setpoint - output[i]) * dt for i in range(len(output)))
    ise = sum((setpoint - output[i])**2 * dt for i in range(len(output)))

    return {
        "rise_time": rise_time,
        "overshoot_percent": overshoot,
        "settling_time": settling_time,
        "steady_state_error": ss_error,
        "iae": iae,
        "ise": ise
    }


# ─── CLI ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='控制参数整定工具 - 电赛资产库')
    sub = parser.add_subparsers(dest='command')

    # PID整定
    p_pid = sub.add_parser('pid', help='PID参数整定')
    p_pid.add_argument('--ku', type=float, help='临界增益(ZN法)')
    p_pid.add_argument('--tu', type=float, help='临界周期(ZN法)')
    p_pid.add_argument('--zn-method', default='classic',
                       choices=['classic', 'pessen', 'some_overshoot', 'no_overshoot'])
    p_pid.add_argument('--k', type=float, help='过程增益(Lambda/Cohen-Coon)')
    p_pid.add_argument('--tau', type=float, help='时间常数')
    p_pid.add_argument('--theta', type=float, help='纯滞后')
    p_pid.add_argument('--method', default='ziegler_nichols',
                       choices=['ziegler_nichols', 'lambda', 'cohen_coon'])

    # ADRC设计
    p_adrc = sub.add_parser('adrc', help='ADRC参数设计')
    p_adrc.add_argument('--settling-time', type=float, required=True, help='期望调节时间(s)')
    p_adrc.add_argument('--overshoot', type=float, default=0.1, help='超调限制(0~1)')
    p_adrc.add_argument('--k-est', type=float, default=1.0, help='控制增益估计')

    # 仿真
    p_sim = sub.add_parser('simulate', help='闭环仿真')
    p_sim.add_argument('--controller', choices=['pid', 'adrc'], default='pid')
    p_sim.add_argument('--kp', type=float, default=1.0)
    p_sim.add_argument('--ki', type=float, default=0.0)
    p_sim.add_argument('--kd', type=float, default=0.0)
    p_sim.add_argument('--wc', type=float, default=5.0)
    p_sim.add_argument('--wo', type=float, default=15.0)
    p_sim.add_argument('--b0', type=float, default=1.0)
    p_sim.add_argument('--setpoint', type=float, default=1.0, help='设定值')
    p_sim.add_argument('--plant-k', type=float, default=1.0, help='对象增益')
    p_sim.add_argument('--plant-tau', type=float, default=1.0, help='对象时间常数')
    p_sim.add_argument('--plant-theta', type=float, default=0.0, help='对象纯滞后')
    p_sim.add_argument('--dt', type=float, default=0.01, help='仿真步长(s)')
    p_sim.add_argument('--t-end', type=float, default=10.0, help='仿真时长(s)')
    p_sim.add_argument('--output', '-o', help='输出JSON')

    # 模型辨识
    p_id = sub.add_parser('identify', help='FOPDT模型辨识')
    p_id.add_argument('--input', '-i', required=True, help='输入CSV(y,u列)')
    p_id.add_argument('--dt', type=float, default=0.01, help='采样时间(s)')

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        return

    if args.command == 'pid':
        if args.method == 'ziegler_nichols':
            if not args.ku or not args.tu:
                print('错误：Ziegler-Nichols法需要 --ku 和 --tu')
                return
            result = ziegler_nichols(args.ku, args.tu, args.zn_method)
        elif args.method == 'lambda':
            result = lambda_tuning(args.k, args.tau, args.theta)
        else:
            result = cohen_coon(args.k, args.tau, args.theta)
        print(f'\n{args.method} 整定结果:')
        for k, v in result.items():
            if isinstance(v, float):
                print(f'  {k}: {v:.6f}')
            else:
                print(f'  {k}: {v}')

    elif args.command == 'adrc':
        result = design_adrc_params(args.settling_time, args.overshoot, args.k_est)
        print(f'\nADRC参数设计结果 (调节时间={args.settling_time}s):')
        for k, v in result.items():
            print(f'  {k}: {v:.4f}')

    elif args.command == 'simulate':
        if args.controller == 'pid':
            result = simulate_pid(args.kp, args.ki, args.kd, args.setpoint,
                                  args.plant_k, args.plant_tau, args.plant_theta,
                                  args.dt, args.t_end)
        else:
            result = simulate_adrc(args.wc, args.wo, args.b0, args.setpoint,
                                   args.plant_k, args.plant_tau, args.dt, args.t_end)
        perf = calc_performance(result)
        print(f'\n仿真结果 ({args.controller.upper()}):')
        print(f'  上升时间: {perf["rise_time"]:.4f}s' if perf["rise_time"] else '  上升时间: N/A')
        print(f'  超调量: {perf["overshoot_percent"]:.2f}%')
        print(f'  调节时间: {perf["settling_time"]:.4f}s' if perf["settling_time"] else '  调节时间: N/A')
        print(f'  稳态误差: {perf["steady_state_error"]:.6f}')
        print(f'  IAE: {perf["iae"]:.4f}')
        print(f'  ISE: {perf["ise"]:.6f}')
        if args.output:
            with open(args.output, 'w') as f:
                json.dump({"results": {k: v[:200] for k, v in result.items()},
                          "performance": perf}, f, indent=2)
            print(f'\n仿真数据已保存至 {args.output}')

    elif args.command == 'identify':
        data = []
        with open(args.input, 'r') as f:
            for i, line in enumerate(f):
                if i == 0:
                    continue
                parts = line.strip().split(',')
                if len(parts) >= 2:
                    try:
                        data.append((float(parts[0]), float(parts[1])))
                    except ValueError:
                        continue
        y_data = [d[0] for d in data]
        u_data = [d[1] for d in data]
        result = identify_fopdt(y_data, u_data, args.dt)
        print(f'\nFOPDT模型辨识结果:')
        print(f'  增益 K: {result["k"]:.4f}')
        print(f'  时间常数 τ: {result["tau"]:.4f}s')
        print(f'  纯滞后 θ: {result["theta"]:.4f}s')


if __name__ == '__main__':
    main()
