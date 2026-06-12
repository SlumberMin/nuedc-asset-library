#!/usr/bin/env python3
"""
PID自动整定工具 - 继电反馈法 + ZN规则
=============================================
功能：
  - 继电反馈法(Relay Feedback)自动获取临界增益和临界周期
  - Ziegler-Nichols整定规则（经典法、Pessen法、超调量法、PID-2法）
  - Cohen-Coon整定规则
  - IMC(内模控制)整定规则
  - PID仿真验证（一阶/二阶/高阶系统）
  - 参数优化（ITAE/ISE/IAE/ITSE指标最小化）
  - 生成整定报告

用法：
  python pid_auto_tuner.py relay --gain 1.0 --dead-time 0.5 --tau 2.0
  python pid_auto_tuner.py zn --ku 4.0 --tu 3.0
  python pid_auto_tuner.py simulate --kp 2.0 --ki 0.5 --kd 0.1 --plant first-order
  python pid_auto_tuner.py optimize --plant second-order --target overshoot=10,settling=5
"""

import argparse
import math
import sys
from dataclasses import dataclass
from typing import List, Tuple, Optional


# ============================================================
# PID 控制器
# ============================================================

@dataclass
class PIDParams:
    """PID参数"""
    kp: float = 0.0     # 比例增益
    ki: float = 0.0     # 积分增益
    kd: float = 0.0     # 微分增益
    name: str = ""

    @property
    def ti(self) -> float:
        """积分时间"""
        return self.kp / self.ki if self.ki != 0 else float('inf')

    @property
    def td(self) -> float:
        """微分时间"""
        return self.kd / self.kp if self.kp != 0 else 0.0

    def to_parallel(self) -> Tuple[float, float, float]:
        """转为并行形式 (Kp, Ki, Kd)"""
        return (self.kp, self.ki, self.kd)

    def to_standard(self) -> Tuple[float, float, float]:
        """转为标准形式 (Kp, Ti, Td)"""
        return (self.kp, self.ti, self.td)

    def __str__(self):
        return (f"Kp={self.kp:.4f}, Ki={self.ki:.4f}, Kd={self.kd:.4f} "
                f"(Ti={self.ti:.4f}s, Td={self.td:.4f}s)")


class PIDController:
    """PID控制器（带抗积分饱和）"""

    def __init__(self, params: PIDParams, dt: float = 0.01,
                 output_min: float = -100, output_max: float = 100):
        self.params = params
        self.dt = dt
        self.output_min = output_min
        self.output_max = output_max

        # 内部状态
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.first_run = True

    def reset(self):
        """重置控制器状态"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.first_run = True

    def update(self, setpoint: float, measurement: float) -> float:
        """计算控制输出"""
        error = setpoint - measurement

        # 比例项
        p_term = self.params.kp * error

        # 积分项（带抗饱和）
        self.integral += error * self.dt
        i_term = self.params.ki * self.integral

        # 微分项（对测量值微分，避免阶跃干扰）
        if self.first_run:
            d_term = 0.0
            self.first_run = False
        else:
            d_measurement = (measurement - self.prev_measurement) / self.dt
            d_term = -self.params.kd * d_measurement

        # 计算输出
        output = p_term + i_term + d_term

        # 输出限幅
        if output > self.output_max:
            output = self.output_max
            # 抗积分饱和：回退积分
            if error > 0:
                self.integral -= error * self.dt
        elif output < self.output_min:
            output = self.output_min
            if error < 0:
                self.integral -= error * self.dt

        # 保存状态
        self.prev_error = error
        self.prev_measurement = measurement

        return output


# ============================================================
# 被控对象模型
# ============================================================

class PlantModel:
    """被控对象模型"""

    @staticmethod
    def first_order(y: float, u: float, dt: float,
                    gain: float = 1.0, tau: float = 1.0,
                    dead_time: float = 0.0) -> float:
        """一阶系统: G(s) = K * e^(-Ls) / (τs + 1)"""
        # 简化处理：用一阶差分近似
        dy = (gain * u - y) / tau * dt
        return y + dy

    @staticmethod
    def second_order(y: float, dy: float, u: float, dt: float,
                     gain: float = 1.0, omega: float = 1.0,
                     zeta: float = 0.7) -> Tuple[float, float]:
        """二阶系统: G(s) = K * ω² / (s² + 2ζωs + ω²)"""
        ddy = gain * omega * omega * u - 2 * zeta * omega * dy - omega * omega * y
        new_dy = dy + ddy * dt
        new_y = y + new_dy * dt
        return new_y, new_dy

    @staticmethod
    def integrator(y: float, u: float, dt: float, gain: float = 1.0) -> float:
        """积分系统: G(s) = K / s"""
        return y + gain * u * dt

    @staticmethod
    def high_order(y_list: List[float], u: float, dt: float,
                   gains: List[float], taus: List[float]) -> float:
        """高阶系统（串联多个一阶环节）"""
        current_input = u
        new_y_list = []
        for i, (y, gain, tau) in enumerate(zip(y_list, gains, taus)):
            y_new = y + (gain * current_input - y) / tau * dt
            new_y_list.append(y_new)
            current_input = y_new
        return new_y_list, current_input


# ============================================================
# 继电反馈法整定
# ============================================================

class RelayFeedbackTuner:
    """继电反馈法自动整定

    原理：在被控对象输入端施加继电（Bang-Bang）控制，
    系统将产生等幅振荡，从中提取临界增益Ku和临界周期Tu。
    """

    @staticmethod
    def simulate_relay(gain: float = 1.0, tau: float = 1.0,
                       dead_time: float = 0.0, relay_amp: float = 1.0,
                       hysteresis: float = 0.1, dt: float = 0.01,
                       sim_time: float = 50.0) -> dict:
        """模拟继电反馈实验

        Args:
            gain: 系统增益
            tau: 系统时间常数
            dead_time: 死区时间
            relay_amp: 继电器幅值
            hysteresis: 滞环宽度
            dt: 仿真步长
            sim_time: 仿真时长

        Returns:
            dict: 包含Ku, Tu和振荡数据
        """
        n_steps = int(sim_time / dt)

        # 状态变量
        y = 0.0          # 输出
        u = relay_amp     # 输入（继电器输出）
        relay_state = True  # 继电器状态

        # 振荡检测
        crossings = []     # 过零点时间
        peaks = []         # 峰值

        # 死区延迟缓冲
        delay_buffer = [0.0] * max(1, int(dead_time / dt))

        # 数据记录
        time_data = []
        y_data = []
        u_data = []

        for i in range(n_steps):
            t = i * dt

            # 从延迟缓冲获取控制量
            delayed_u = delay_buffer[0] if delay_buffer else u
            delay_buffer.append(u)
            if len(delay_buffer) > max(1, int(dead_time / dt)):
                delay_buffer.pop(0)

            # 一阶系统响应
            dy = (gain * delayed_u - y) / tau * dt
            y += dy

            # 继电器逻辑（带滞环）
            if relay_state and y > hysteresis:
                relay_state = False
                u = -relay_amp
            elif not relay_state and y < -hysteresis:
                relay_state = True
                u = relay_amp

            # 检测过零点
            if i > 0 and len(y_data) > 0:
                if (y_data[-1] < 0 and y >= 0) or (y_data[-1] > 0 and y <= 0):
                    crossings.append(t)

            # 检测峰值
            if i > 1 and len(y_data) > 1:
                if y_data[-2] < y_data[-1] and y_data[-1] > y:
                    peaks.append((t - dt, y_data[-1]))
                elif y_data[-2] > y_data[-1] and y_data[-1] < y:
                    peaks.append((t - dt, y_data[-1]))

            time_data.append(t)
            y_data.append(y)
            u_data.append(u)

        # 从振荡数据提取Ku和Tu
        result = {
            "time": time_data,
            "output": y_data,
            "input": u_data,
            "crossings": crossings,
            "peaks": peaks,
        }

        # 计算临界周期Tu
        if len(crossings) >= 2:
            # 使用过零点间距的两倍作为周期
            periods = [crossings[i+1] - crossings[i] for i in range(len(crossings)-1)]
            if periods:
                # 取稳定后的平均值
                stable_periods = periods[len(periods)//3:]
                if stable_periods:
                    half_period = sum(stable_periods) / len(stable_periods)
                    result["Tu"] = 2 * half_period
                else:
                    result["Tu"] = periods[-1] * 2
            else:
                result["Tu"] = None
        else:
            # 尝试从峰值计算
            if len(peaks) >= 2:
                peak_times = [p[0] for p in peaks]
                periods = [peak_times[i+2] - peak_times[i]
                          for i in range(len(peak_times)-2)]
                if periods:
                    result["Tu"] = sum(periods) / len(periods)
                else:
                    result["Tu"] = None
            else:
                result["Tu"] = None

        # 计算振荡幅度a
        if peaks:
            stable_peaks = [abs(p[1]) for p in peaks[len(peaks)//3:]]
            if stable_peaks:
                result["a"] = sum(stable_peaks) / len(stable_peaks)
            else:
                result["a"] = max(abs(p[1]) for p in peaks)
        else:
            result["a"] = max(abs(v) for v in y_data[len(y_data)//2:])

        # 计算临界增益 Ku = 4d / (π*a)
        # d = 继电器幅值, a = 振荡幅值
        if result.get("a") and result["a"] > 0:
            result["Ku"] = (4 * relay_amp) / (math.pi * result["a"])
        else:
            result["Ku"] = None

        return result


# ============================================================
# Ziegler-Nichols 整定规则
# ============================================================

class ZieglerNichols:
    """Ziegler-Nichols整定规则

    基于临界增益Ku和临界周期Tu计算PID参数
    """

    @staticmethod
    def classic(Ku: float, Tu: float) -> PIDParams:
        """经典ZN规则"""
        kp = 0.6 * Ku
        ti = 0.5 * Tu
        td = 0.125 * Tu
        return PIDParams(kp=kp, ki=kp/ti, kd=kp*td, name="ZN经典法")

    @staticmethod
    def pessen(Ku: float, Tu: float) -> PIDParams:
        """Pessen积分规则（改进版）"""
        kp = 0.7 * Ku
        ti = 0.4 * Tu
        td = 0.15 * Tu
        return PIDParams(kp=kp, ki=kp/ti, kd=kp*td, name="Pessen积分法")

    @staticmethod
    def some_overshoot(Ku: float, Tu: float) -> PIDParams:
        """超调量略大的ZN规则"""
        kp = 0.33 * Ku
        ti = 0.5 * Tu
        td = 0.33 * Tu
        return PIDParams(kp=kp, ki=kp/ti, kd=kp*td, name="ZN超调法")

    @staticmethod
    def no_overshoot(Ku: float, Tu: float) -> PIDParams:
        """无超调ZN规则"""
        kp = 0.2 * Ku
        ti = 0.5 * Tu
        td = 0.33 * Tu
        return PIDParams(kp=kp, ki=kp/ti, kd=kp*td, name="ZN无超调法")

    @staticmethod
    def pid2(Ku: float, Tu: float) -> PIDParams:
        """PID-2法（修正版）"""
        kp = 0.5 * Ku
        ti = 0.33 * Tu
        td = 0.167 * Tu
        return PIDParams(kp=kp, ki=kp/ti, kd=kp*td, name="PID-2法")


# ============================================================
# Cohen-Coon 整定规则
# ============================================================

class CohenCoon:
    """Cohen-Coon整定规则

    基于过程响应曲线的参数K, L(死区), T(时间常数)
    """

    @staticmethod
    def from_step_response(K: float, L: float, T: float) -> PIDParams:
        """从阶跃响应参数计算PID

        Args:
            K: 过程增益
            L: 死区时间
            T: 时间常数
        """
        if L <= 0 or T <= 0 or K <= 0:
            raise ValueError("K, L, T 必须为正数")

        r = L / T  # 死区时间比

        kp = (1.0 / K) * (1.0 + r / 3.0) / (1.0 if r > 1 else (1 + r * (r > 0.1)))
        kp = (1.0 / K) * (T / L) * (1.0 + L / (3.0 * T))

        ti = L * (32.0 + 6.0 * L / T) / (13.0 + 8.0 * L / T)

        td = L * 4.0 / (11.0 + 2.0 * L / T)

        return PIDParams(kp=kp, ki=kp/ti, kd=kp*td, name="Cohen-Coon法")

    @staticmethod
    def p_controller(K: float, L: float, T: float) -> PIDParams:
        """Cohen-Coon P控制器"""
        kp = (1.0 / K) * (T / L) * (1.0 + L / (3.0 * T))
        return PIDParams(kp=kp, ki=0, kd=0, name="Cohen-Coon P")

    @staticmethod
    def pi_controller(K: float, L: float, T: float) -> PIDParams:
        """Cohen-Coon PI控制器"""
        kp = (1.0 / K) * (T / L) * (0.9 + L / (12.0 * T))
        ti = L * (30.0 + 3.0 * L / T) / (9.0 + 20.0 * L / T)
        return PIDParams(kp=kp, ki=kp/ti, kd=0, name="Cohen-Coon PI")


# ============================================================
# IMC 整定规则
# ============================================================

class IMCTuner:
    """内模控制(IMC)整定规则

    基于期望闭环时间常数λ计算PID参数
    """

    @staticmethod
    def first_order(K: float, L: float, T: float,
                    lambda_c: float = None) -> PIDParams:
        """一阶系统IMC整定

        Args:
            K: 过程增益
            L: 死区时间
            T: 时间常数
            lambda_c: 期望闭环时间常数（默认取T的一半）
        """
        if lambda_c is None:
            lambda_c = T * 0.5

        kp = T / (K * (lambda_c + L))
        ti = T
        td = 0.0

        return PIDParams(kp=kp, ki=kp/ti, kd=0, name=f"IMC法(λ={lambda_c:.2f})")

    @staticmethod
    def second_order(K: float, L: float, T1: float, T2: float,
                     lambda_c: float = None) -> PIDParams:
        """二阶系统IMC整定"""
        if lambda_c is None:
            lambda_c = max(T1, T2) * 0.5

        kp = (T1 + T2) / (K * (lambda_c + L))
        ti = T1 + T2
        td = T1 * T2 / (T1 + T2)

        return PIDParams(kp=kp, ki=kp/ti, kd=kp*td, name=f"IMC法(λ={lambda_c:.2f})")


# ============================================================
# PID 仿真器
# ============================================================

class PIDSimulator:
    """PID闭环仿真器"""

    @staticmethod
    def simulate_first_order(pid_params: PIDParams, K: float = 1.0,
                             tau: float = 1.0, setpoint: float = 1.0,
                             dt: float = 0.01, sim_time: float = 10.0,
                             disturbance: float = 0.0) -> dict:
        """一阶系统PID仿真"""
        n_steps = int(sim_time / dt)
        controller = PIDController(pid_params, dt=dt)
        y = 0.0

        time_data = [0.0]
        y_data = [0.0]
        u_data = [0.0]
        sp_data = [setpoint]

        for i in range(1, n_steps):
            t = i * dt

            # 添加扰动
            sp = setpoint
            if disturbance != 0 and t > sim_time * 0.6:
                sp += disturbance

            # PID计算
            u = controller.update(sp, y)

            # 一阶系统响应
            dy = (K * u - y) / tau * dt
            y += dy

            time_data.append(t)
            y_data.append(y)
            u_data.append(u)
            sp_data.append(sp)

        # 计算性能指标
        metrics = PIDSimulator._calculate_metrics(time_data, y_data, setpoint)

        return {
            "time": time_data,
            "output": y_data,
            "input": u_data,
            "setpoint": sp_data,
            "metrics": metrics,
            "params": str(pid_params),
        }

    @staticmethod
    def simulate_second_order(pid_params: PIDParams, K: float = 1.0,
                              omega: float = 1.0, zeta: float = 0.7,
                              setpoint: float = 1.0, dt: float = 0.01,
                              sim_time: float = 10.0) -> dict:
        """二阶系统PID仿真"""
        n_steps = int(sim_time / dt)
        controller = PIDController(pid_params, dt=dt)
        y = 0.0
        dy = 0.0

        time_data = [0.0]
        y_data = [0.0]
        u_data = [0.0]

        for i in range(1, n_steps):
            t = i * dt
            u = controller.update(setpoint, y)

            ddy = K * omega * omega * u - 2 * zeta * omega * dy - omega * omega * y
            dy += ddy * dt
            y += dy * dt

            time_data.append(t)
            y_data.append(y)
            u_data.append(u)

        metrics = PIDSimulator._calculate_metrics(time_data, y_data, setpoint)

        return {
            "time": time_data,
            "output": y_data,
            "input": u_data,
            "setpoint": [setpoint] * len(time_data),
            "metrics": metrics,
            "params": str(pid_params),
        }

    @staticmethod
    def _calculate_metrics(time_data: List[float], y_data: List[float],
                           setpoint: float) -> dict:
        """计算性能指标"""
        n = len(y_data)

        # 上升时间（10%到90%）
        val_10 = setpoint * 0.1
        val_90 = setpoint * 0.9
        t_10 = t_90 = None
        for i in range(n):
            if t_10 is None and y_data[i] >= val_10:
                t_10 = time_data[i]
            if t_90 is None and y_data[i] >= val_90:
                t_90 = time_data[i]
                break
        rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else None

        # 最大超调量
        y_max = max(y_data)
        overshoot = ((y_max - setpoint) / setpoint * 100) if setpoint != 0 else 0

        # 稳态误差
        ss_start = int(n * 0.8)
        y_ss = sum(y_data[ss_start:]) / (n - ss_start)
        ss_error = setpoint - y_ss

        # 调节时间（±2%带内）
        settling_time = None
        band = setpoint * 0.02
        for i in range(n - 1, -1, -1):
            if abs(y_data[i] - setpoint) > band:
                if i < n - 1:
                    settling_time = time_data[i + 1]
                break

        # ISE/IAE/ITAE/ITSE指标
        ise = sum((setpoint - y) ** 2 * (time_data[1] - time_data[0])
                  for y in y_data)
        iae = sum(abs(setpoint - y) * (time_data[1] - time_data[0])
                  for y in y_data)
        itae = sum(t * abs(setpoint - y) * (time_data[1] - time_data[0])
                   for t, y in zip(time_data, y_data))
        itse = sum(t * (setpoint - y) ** 2 * (time_data[1] - time_data[0])
                   for t, y in zip(time_data, y_data))

        return {
            "rise_time": round(rise_time, 4) if rise_time else "N/A",
            "overshoot": round(overshoot, 2),
            "settling_time": round(settling_time, 4) if settling_time else "N/A",
            "steady_state_error": round(ss_error, 4),
            "final_value": round(y_ss, 4),
            "ISE": round(ise, 4),
            "IAE": round(iae, 4),
            "ITAE": round(itae, 4),
            "ITSE": round(itse, 4),
        }


# ============================================================
# CLI 接口
# ============================================================

def cmd_relay(args):
    """继电反馈法整定"""
    print("\n  继电反馈法自动整定")
    print("  " + "=" * 50)
    print(f"  系统参数: K={args.gain}, τ={args.tau}s, L={args.dead_time}s")
    print(f"  继电器: 幅值={args.relay_amp}, 滞环={args.hysteresis}")

    result = RelayFeedbackTuner.simulate_relay(
        gain=args.gain, tau=args.tau, dead_time=args.dead_time,
        relay_amp=args.relay_amp, hysteresis=args.hysteresis,
        dt=args.dt, sim_time=args.sim_time,
    )

    print(f"\n  继电反馈实验结果:")
    print(f"  {'-' * 40}")

    if result.get("Tu"):
        print(f"  临界周期 Tu = {result['Tu']:.4f} s")
        print(f"  振荡幅值 a  = {result['a']:.4f}")

        if result.get("Ku"):
            Ku = result["Ku"]
            Tu = result["Tu"]
            print(f"  临界增益 Ku = {Ku:.4f}")

            # 应用各种整定规则
            print(f"\n  整定结果:")
            print(f"  {'-' * 65}")
            print(f"  {'方法':12s} {'Kp':>10s} {'Ki':>10s} {'Kd':>10s} {'Ti':>10s} {'Td':>10s}")
            print(f"  {'-' * 65}")

            methods = [
                ZieglerNichols.classic(Ku, Tu),
                ZieglerNichols.pessen(Ku, Tu),
                ZieglerNichols.some_overshoot(Ku, Tu),
                ZieglerNichols.no_overshoot(Ku, Tu),
                ZieglerNichols.pid2(Ku, Tu),
            ]

            for p in methods:
                print(f"  {p.name:12s} {p.kp:10.4f} {p.ki:10.4f} {p.kd:10.4f} "
                      f"{p.ti:10.4f} {p.td:10.4f}")

            # Cohen-Coon
            cc = CohenCoon.from_step_response(args.gain, args.dead_time, args.tau)
            print(f"  {cc.name:12s} {cc.kp:10.4f} {cc.ki:10.4f} {cc.kd:10.4f} "
                  f"{cc.ti:10.4f} {cc.td:10.4f}")

            # IMC
            imc = IMCTuner.first_order(args.gain, args.dead_time, args.tau)
            print(f"  {imc.name:12s} {imc.kp:10.4f} {imc.ki:10.4f} {imc.kd:10.4f} "
                  f"{imc.ti:10.4f} {imc.td:10.4f}")

            # 仿真推荐参数
            if args.simulate:
                print(f"\n  仿真验证 (ZN经典法):")
                print(f"  {'-' * 40}")
                best = methods[0]
                sim = PIDSimulator.simulate_first_order(
                    best, K=args.gain, tau=args.tau,
                    setpoint=1.0, sim_time=args.sim_time
                )
                for k, v in sim["metrics"].items():
                    print(f"  {k:20s}: {v}")
    else:
        print("  ⚠ 未能检测到稳定振荡，请调整参数重试")


def cmd_zn(args):
    """ZN规则整定"""
    Ku = args.ku
    Tu = args.tu

    print("\n  Ziegler-Nichols 整定计算")
    print("  " + "=" * 55)
    print(f"  输入: Ku = {Ku:.4f}, Tu = {Tu:.4f}s")
    print()

    methods = [
        ZieglerNichols.classic(Ku, Tu),
        ZieglerNichols.pessen(Ku, Tu),
        ZieglerNichols.some_overshoot(Ku, Tu),
        ZieglerNichols.no_overshoot(Ku, Tu),
        ZieglerNichols.pid2(Ku, Tu),
    ]

    print(f"  {'方法':12s} {'Kp':>10s} {'Ki':>10s} {'Kd':>10s} {'Ti(s)':>10s} {'Td(s)':>10s}")
    print(f"  {'-' * 65}")

    for p in methods:
        print(f"  {p.name:12s} {p.kp:10.4f} {p.ki:10.4f} {p.kd:10.4f} "
              f"{p.ti:10.4f} {p.td:10.4f}")

    if args.simulate:
        print(f"\n  仿真对比:")
        print(f"  {'-' * 50}")
        for p in methods:
            sim = PIDSimulator.simulate_first_order(
                p, K=1.0, tau=Tu/(2*math.pi), setpoint=1.0, sim_time=Tu*3
            )
            m = sim["metrics"]
            print(f"  {p.name}: 超调={m['overshoot']:.1f}%  "
                  f"调节时间={m['settling_time']}s  "
                  f"稳态误差={m['steady_state_error']:.4f}")


def cmd_simulate(args):
    """PID仿真"""
    params = PIDParams(kp=args.kp, ki=args.ki, kd=args.kd, name="用户参数")

    print(f"\n  PID 闭环仿真")
    print("  " + "=" * 50)
    print(f"  PID参数: {params}")
    print(f"  仿真: 时长={args.duration}s, 步长={args.dt}s")

    if args.plant == "first-order":
        sim = PIDSimulator.simulate_first_order(
            params, K=args.plant_gain, tau=args.plant_tau,
            setpoint=args.setpoint, dt=args.dt, sim_time=args.duration,
            disturbance=args.disturbance,
        )
    elif args.plant == "second-order":
        sim = PIDSimulator.simulate_second_order(
            params, K=args.plant_gain, omega=args.plant_omega,
            zeta=args.plant_zeta, setpoint=args.setpoint,
            dt=args.dt, sim_time=args.duration,
        )
    else:
        sim = PIDSimulator.simulate_first_order(
            params, K=args.plant_gain, tau=args.plant_tau,
            setpoint=args.setpoint, dt=args.dt, sim_time=args.duration,
        )

    m = sim["metrics"]
    print(f"\n  性能指标:")
    print(f"  {'-' * 40}")
    for k, v in m.items():
        print(f"  {k:20s}: {v}")

    # 绘制ASCII图
    if args.ascii_plot:
        _ascii_plot(sim["time"], sim["output"], sim["setpoint"],
                    title="PID响应曲线", width=70, height=20)


def cmd_optimize(args):
    """参数优化"""
    print(f"\n  PID参数优化")
    print("  " + "=" * 50)
    print(f"  目标: {args.target}")
    print(f"  被控对象: {args.plant}")

    # 解析目标
    targets = {}
    for t in args.target.split(","):
        k, v = t.split("=")
        targets[k.strip()] = float(v.strip())

    best_params = None
    best_score = float('inf')
    results = []

    # 网格搜索
    kp_range = [0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 10.0]
    ki_range = [0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 5.0]
    kd_range = [0.0, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0]

    print(f"  搜索空间: Kp={len(kp_range)}, Ki={len(ki_range)}, Kd={len(kd_range)}")
    total = len(kp_range) * len(ki_range) * len(kd_range)
    print(f"  总组合: {total}")

    count = 0
    for kp in kp_range:
        for ki in ki_range:
            for kd in kd_range:
                params = PIDParams(kp=kp, ki=ki, kd=kd)

                if args.plant == "first-order":
                    sim = PIDSimulator.simulate_first_order(
                        params, K=1.0, tau=1.0,
                        setpoint=1.0, dt=0.01, sim_time=10.0,
                    )
                else:
                    sim = PIDSimulator.simulate_second_order(
                        params, K=1.0, omega=1.0, zeta=0.7,
                        setpoint=1.0, dt=0.01, sim_time=10.0,
                    )

                m = sim["metrics"]
                count += 1

                # 计算综合得分
                score = 0
                if "overshoot" in targets and isinstance(m["overshoot"], (int, float)):
                    score += abs(m["overshoot"] - targets["overshoot"]) * 2
                if "settling" in targets and isinstance(m["settling_time"], (int, float)):
                    score += abs(m["settling_time"] - targets["settling"]) * 5

                # 使用ITAE作为额外权重
                if isinstance(m.get("ITAE"), (int, float)):
                    score += m["ITAE"]

                results.append((params, score, m))

                if score < best_score:
                    best_score = score
                    best_params = params

    # 显示最优结果
    results.sort(key=lambda x: x[1])

    print(f"\n  最优PID参数 (得分: {best_score:.4f}):")
    print(f"  {best_params}")

    print(f"\n  Top 5 参数组合:")
    print(f"  {'排名':>4s} {'Kp':>8s} {'Ki':>8s} {'Kd':>8s} {'得分':>10s} "
          f"{'超调%':>8s} {'调节时间':>10s}")
    print(f"  {'-' * 60}")

    for i, (p, s, m) in enumerate(results[:5]):
        ot = m['overshoot'] if isinstance(m['overshoot'], (int, float)) else 'N/A'
        st = m['settling_time'] if isinstance(m['settling_time'], (int, float)) else 'N/A'
        print(f"  {i+1:4d} {p.kp:8.4f} {p.ki:8.4f} {p.kd:8.4f} {s:10.4f} "
              f"{ot:>8} {st:>10}")


def _ascii_plot(time_data: List[float], y_data: List[float],
                setpoint_data: List[float], title: str = "",
                width: int = 70, height: int = 20):
    """ASCII绘图"""
    if not y_data:
        return

    y_min = min(min(y_data), min(setpoint_data))
    y_max = max(max(y_data), max(setpoint_data))

    if y_max == y_min:
        y_max = y_min + 1

    # 采样到屏幕宽度
    n = len(y_data)
    step = max(1, n // width)
    sampled_y = [y_data[min(i * step, n - 1)] for i in range(width)]
    sampled_sp = [setpoint_data[min(i * step, n - 1)] for i in range(width)]

    print(f"\n  {title}")
    print(f"  {'=' * (width + 10)}")

    for row in range(height, -1, -1):
        y_val = y_min + (y_max - y_min) * row / height
        line = f"  {y_val:7.2f} |"

        for i in range(width):
            y_pixel = y_min + (y_max - y_min) * (height - sampled_y[i] + y_min) / (y_max - y_min)
            # 简化：直接比较
            threshold = y_min + (y_max - y_min) * row / height

            if abs(sampled_y[i] - threshold) < (y_max - y_min) / height * 0.6:
                line += "*"
            elif abs(sampled_sp[i] - threshold) < (y_max - y_min) / height * 0.6:
                line += "-"
            else:
                line += " "

        print(line)

    print(f"  {' ' * 9}+" + "-" * width)
    t_start = time_data[0]
    t_end = time_data[-1]
    print(f"  {' ' * 9}{t_start:.1f}" + " " * (width - 10) + f"{t_end:.1f}")


def main():
    parser = argparse.ArgumentParser(
        description="PID自动整定工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
整定方法说明:
  继电反馈法: 自动实验获取Ku和Tu，再用ZN规则整定
  ZN规则:     基于临界增益Ku和临界周期Tu的经典整定
  Cohen-Coon: 基于阶跃响应的参数整定
  IMC法:      基于期望闭环性能的现代整定方法
        """
    )
    subparsers = parser.add_subparsers(dest="command", help="功能")

    # 继电反馈
    relay_parser = subparsers.add_parser("relay", help="继电反馈法自动整定")
    relay_parser.add_argument("--gain", type=float, default=1.0, help="系统增益")
    relay_parser.add_argument("--tau", type=float, default=1.0, help="时间常数")
    relay_parser.add_argument("--dead-time", type=float, default=0.1, help="死区时间")
    relay_parser.add_argument("--relay-amp", type=float, default=1.0, help="继电器幅值")
    relay_parser.add_argument("--hysteresis", type=float, default=0.1, help="滞环宽度")
    relay_parser.add_argument("--dt", type=float, default=0.01, help="仿真步长")
    relay_parser.add_argument("--sim-time", type=float, default=30.0, help="仿真时长")
    relay_parser.add_argument("--simulate", action="store_true", help="仿真验证")

    # ZN规则
    zn_parser = subparsers.add_parser("zn", help="ZN规则整定")
    zn_parser.add_argument("--ku", type=float, required=True, help="临界增益Ku")
    zn_parser.add_argument("--tu", type=float, required=True, help="临界周期Tu(s)")
    zn_parser.add_argument("--simulate", action="store_true", help="仿真验证")

    # 仿真
    sim_parser = subparsers.add_parser("simulate", help="PID仿真")
    sim_parser.add_argument("--kp", type=float, default=2.0, help="Kp")
    sim_parser.add_argument("--ki", type=float, default=0.5, help="Ki")
    sim_parser.add_argument("--kd", type=float, default=0.1, help="Kd")
    sim_parser.add_argument("--plant", default="first-order",
                            choices=["first-order", "second-order"], help="被控对象模型")
    sim_parser.add_argument("--plant-gain", type=float, default=1.0, help="系统增益")
    sim_parser.add_argument("--plant-tau", type=float, default=1.0, help="时间常数")
    sim_parser.add_argument("--plant-omega", type=float, default=1.0, help="固有频率")
    sim_parser.add_argument("--plant-zeta", type=float, default=0.7, help="阻尼比")
    sim_parser.add_argument("--setpoint", type=float, default=1.0, help="设定值")
    sim_parser.add_argument("--dt", type=float, default=0.01, help="仿真步长")
    sim_parser.add_argument("--duration", type=float, default=10.0, help="仿真时长")
    sim_parser.add_argument("--disturbance", type=float, default=0.0, help="扰动量")
    sim_parser.add_argument("--ascii-plot", action="store_true", help="ASCII图")

    # 优化
    opt_parser = subparsers.add_parser("optimize", help="参数优化")
    opt_parser.add_argument("--plant", default="first-order",
                            choices=["first-order", "second-order"])
    opt_parser.add_argument("--target", default="overshoot=10,settling=3",
                            help="目标: overshoot=10,settling=5")

    args = parser.parse_args()

    if args.command == "relay":
        cmd_relay(args)
    elif args.command == "zn":
        cmd_zn(args)
    elif args.command == "simulate":
        cmd_simulate(args)
    elif args.command == "optimize":
        cmd_optimize(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
