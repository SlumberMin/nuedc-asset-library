#!/usr/bin/env python3
"""
高级电机控制仿真
- FOC (磁场定向控制)
- SVPWM (空间矢量脉宽调制)
- 弱磁控制
- 最大转矩电流比 (MTPA)
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Tuple

# ============================================================
# 1. Clarke / Park 变换
# ============================================================

def clarke_transform(ia: float, ib: float, ic: float) -> Tuple[float, float]:
    """abc → αβ"""
    alpha = (2/3) * (ia - 0.5*ib - 0.5*ic)
    beta  = (2/3) * (np.sqrt(3)/2*ib - np.sqrt(3)/2*ic)
    return alpha, beta

def inv_clarke(alpha: float, beta: float) -> Tuple[float, float, float]:
    """αβ → abc"""
    ia = alpha
    ib = -0.5*alpha + np.sqrt(3)/2*beta
    ic = -0.5*alpha - np.sqrt(3)/2*beta
    return ia, ib, ic

def park_transform(alpha: float, beta: float, theta: float) -> Tuple[float, float]:
    """αβ → dq"""
    d = alpha*np.cos(theta) + beta*np.sin(theta)
    q = -alpha*np.sin(theta) + beta*np.cos(theta)
    return d, q

def inv_park(d: float, q: float, theta: float) -> Tuple[float, float]:
    """dq → αβ"""
    alpha = d*np.cos(theta) - q*np.sin(theta)
    beta  = d*np.sin(theta) + q*np.cos(theta)
    return alpha, beta


# ============================================================
# 2. PI 控制器
# ============================================================

@dataclass
class PIController:
    kp: float = 1.0
    ki: float = 0.1
    integral: float = 0.0
    out_min: float = -1e6
    out_max: float = 1e6

    def update(self, ref: float, fb: float, dt: float) -> float:
        err = ref - fb
        self.integral += err * dt
        out = self.kp * err + self.ki * self.integral
        # anti-windup
        if out > self.out_max:
            out = self.out_max
            self.integral -= err * dt
        elif out < self.out_min:
            out = self.out_min
            self.integral -= err * dt
        return out


# ============================================================
# 3. SVPWM
# ============================================================

def svpwm(v_alpha: float, v_beta: float, v_dc: float, f_pwm: float) -> Tuple[float, float, float]:
    """
    空间矢量 PWM
    返回三相占空比 (0~1)
    """
    # 逆Clarke
    va = v_alpha
    vb = -0.5*v_alpha + np.sqrt(3)/2*v_beta
    vc = -0.5*v_alpha - np.sqrt(3)/2*v_beta

    # 归一化
    vmax = v_dc / 2
    va_n = np.clip(va / vmax, -1, 1)
    vb_n = np.clip(vb / vmax, -1, 1)
    vc_n = np.clip(vc / vmax, -1, 1)

    # 扇区判断 + 占空比 (简化版)
    # 等效零序注入
    v_zero = -(max(va_n, vb_n, vc_n) + min(va_n, vb_n, vc_n)) / 2
    da = np.clip((va_n + v_zero + 1) / 2, 0, 1)
    db = np.clip((vb_n + v_zero + 1) / 2, 0, 1)
    dc = np.clip((vc_n + v_zero + 1) / 2, 0, 1)
    return da, db, dc


# ============================================================
# 4. PMSM 电机模型
# ============================================================

@dataclass
class PMSMMotor:
    Rs: float = 0.5          # 定子电阻 Ω
    Ld: float = 5e-3        # d轴电感 H
    Lq: float = 8e-3        # q轴电感 H
    psi_f: float = 0.1      # 永磁磁链 Wb
    p: int = 4              # 极对数
    J: float = 1e-4         # 转动惯量 kg·m²
    B: float = 1e-3         # 摩擦系数
    id: float = 0.0
    iq: float = 0.0
    omega_e: float = 0.0    # 电角速度
    theta_e: float = 0.0    # 电角度

    def step(self, vd: float, vq: float, tl: float, dt: float) -> Tuple[float, float]:
        """一个电气周期"""
        # dq 电压方程
        did = (vd - self.Rs*self.id + self.Lq*self.p*self.omega_e*self.iq) / self.Ld * dt
        diq = (vq - self.Rs*self.iq - self.Ld*self.p*self.omega_e*self.id
               - self.psi_f*self.p*self.omega_e) / self.Lq * dt
        self.id += did
        self.iq += diq

        # 电磁转矩
        te = 1.5 * self.p * (self.psi_f * self.iq + (self.Ld - self.Lq)*self.id*self.iq)

        # 机械方程
        omega_m = self.omega_e / self.p
        d_omega = (te - tl - self.B*omega_m) / self.J * dt
        omega_m += d_omega
        self.omega_e = omega_m * self.p

        self.theta_e += self.omega_e * dt
        self.theta_e %= 2*np.pi
        return self.omega_e, te


# ============================================================
# 5. MTPA (最大转矩电流比)
# ============================================================

def mtpa(i_ref: float, Ld: float, Lq: float, psi_f: float, p: int) -> Tuple[float, float]:
    """
    最大转矩电流比策略
    对于表贴式 (Ld≈Lq): id=0
    对于凸极式 (Ld<Lq): id = psi_f/(4*(Lq-Ld)) - sqrt((psi_f/(4*(Lq-Ld)))^2 + i_ref^2/2)
    """
    if abs(Lq - Ld) < 1e-9:
        return 0.0, i_ref
    ratio = psi_f / (4 * (Lq - Ld))
    id_ref = ratio - np.sqrt(ratio**2 + i_ref**2 / 2)
    iq_ref = np.sqrt(i_ref**2 - id_ref**2) if i_ref**2 > id_ref**2 else i_ref
    return id_ref, iq_ref


# ============================================================
# 6. 弱磁控制
# ============================================================

def flux_weakening(v_limit: float, iq: float, omega: float,
                   Rs: float, Ld: float, Lq: float, psi_f: float) -> float:
    """
    弱磁策略：当电压饱和时计算 d 轴去磁电流
    v_limit: 最大电压幅值 (V_dc/sqrt(3))
    返回 id_ref
    """
    if omega < 1e-3:
        return 0.0
    # v = sqrt((Rs*id - omega*Lq*iq)^2 + (Rs*iq + omega*Ld*id + omega*psi_f)^2)
    # 简化为电压圆约束
    emf_q = omega * psi_f
    max_vq = np.sqrt(max(v_limit**2 - (omega*Lq*iq)**2, 0))
    id_ref = (max_vq - emf_q) / (omega * Ld) if omega * Ld > 1e-9 else 0.0
    return min(id_ref, 0.0)  # 弱磁时 id 必须为负


# ============================================================
# 7. 仿真主循环
# ============================================================

def run_foc_simulation():
    """运行完整 FOC 仿真"""
    dt = 1e-5       # 10μs
    f_pwm = 20e3    # 20kHz
    t_end = 0.5
    steps = int(t_end / dt)

    motor = PMSMMotor()
    pi_id = PIController(kp=5.0, ki=200.0, out_min=-48, out_max=48)
    pi_iq = PIController(kp=5.0, ki=200.0, out_min=-48, out_max=48)
    pi_speed = PIController(kp=0.5, ki=10.0, out_min=-10, out_max=10)

    v_dc = 48.0
    speed_ref = 300.0  # rad/s (电角速度)

    # 记录
    log_t, log_id, log_iq, log_speed, log_te = [], [], [], [], []
    sample_every = int(1 / f_pwm / dt)  # 每个 PWM 周期采样

    load_torque = 0.1  # N·m

    for i in range(steps):
        t = i * dt

        # 速度环 (每10个PWM周期更新)
        if i % (10 * sample_every) == 0:
            iq_ref = pi_speed.update(speed_ref, motor.omega_e, 10/f_pwm)
            iq_ref = np.clip(iq_ref, -10, 10)

        # MTPA
        id_ref, iq_ref_adj = mtpa(iq_ref, motor.Ld, motor.Lq, motor.psi_f, motor.p)

        # 弱磁判断
        v_limit = v_dc / np.sqrt(3)
        v_needed = motor.omega_e * motor.psi_f
        if v_needed > v_limit * 0.9:
            id_ref = flux_weakening(v_limit, iq_ref_adj, motor.omega_e,
                                    motor.Rs, motor.Ld, motor.Lq, motor.psi_f)

        # dq 电流环
        vd = pi_id.update(id_ref, motor.id, dt)
        vq = pi_iq.update(iq_ref_adj, motor.iq, dt)

        # 反 Park → SVPWM
        valpha, vbeta = inv_park(vd, vq, motor.theta_e)
        da, db, dc = svpwm(valpha, vbeta, v_dc, f_pwm)

        # 等效电压 (简化: 直接用 dq 电压驱动电机)
        omega, te = motor.step(vd, vq, load_torque, dt)

        # 记录
        if i % sample_every == 0:
            log_t.append(t)
            log_id.append(motor.id)
            log_iq.append(motor.iq)
            log_speed.append(motor.omega_e)
            log_te.append(te)

        # 0.2s 加载
        if t > 0.2:
            load_torque = 0.5

    # --- 绘图 ---
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    axes[0].plot(log_t, log_speed, 'b-')
    axes[0].axhline(speed_ref, color='r', linestyle='--', label='参考')
    axes[0].set_ylabel('电角速度 (rad/s)')
    axes[0].legend()
    axes[0].set_title('FOC + SVPWM + MTPA + 弱磁 仿真')

    axes[1].plot(log_t, log_id, 'g-', label='id')
    axes[1].plot(log_t, log_iq, 'r-', label='iq')
    axes[1].set_ylabel('电流 (A)')
    axes[1].legend()

    axes[2].plot(log_t, log_te, 'm-')
    axes[2].axhline(load_torque, color='k', linestyle='--', label='负载')
    axes[2].set_ylabel('转矩 (N·m)')
    axes[2].legend()

    # SVPWM 占空比 (最后100个采样)
    axes[3].plot(log_t[-100:], [svpwm(
        *inv_park(pi_id.update(0, 0, dt), pi_iq.update(0, 0, dt), 0),
        v_dc, f_pwm)[0] for _ in range(100)], 'b-', label='da')
    axes[3].set_ylabel('占空比')
    axes[3].set_xlabel('时间 (s)')
    axes[3].legend()

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/foc_result.png', dpi=150)
    plt.show()
    print("✅ FOC 仿真完成")


# ============================================================
# 8. MTPA 曲线分析
# ============================================================

def plot_mtpa_curve():
    """绘制 MTPA 曲线"""
    i_range = np.linspace(0, 10, 100)
    Ld, Lq, psi_f = 5e-3, 8e-3, 0.1

    ids, iqs = [], []
    for i_ref in i_range:
        id_ref, iq_ref = mtpa(i_ref, Ld, Lq, psi_f, 4)
        ids.append(id_ref)
        iqs.append(iq_ref)

    plt.figure(figsize=(8, 6))
    plt.plot(ids, iqs, 'b-', linewidth=2)
    plt.xlabel('Id (A)')
    plt.ylabel('Iq (A)')
    plt.title('最大转矩电流比 (MTPA) 曲线')
    plt.grid(True)
    plt.savefig('./nuedc-asset-library/15_simulation/mtpa_curve.png', dpi=150)
    plt.show()


# ============================================================
# 9. 弱磁区域分析
# ============================================================

def plot_flux_weakening_region():
    """绘制弱磁区域工作特性"""
    omega_range = np.linspace(100, 2000, 200)
    v_dc = 48.0
    v_limit = v_dc / np.sqrt(3)
    Rs, Ld, Lq, psi_f = 0.5, 5e-3, 8e-3, 0.1

    id_fw = []
    iq_max = []
    for w in omega_range:
        iq_limit = v_limit / (w * Lq) if w * Lq > 1e-9 else 100
        iq_max.append(min(iq_limit, 10))
        id_ref = flux_weakening(v_limit, 5, w, Rs, Ld, Lq, psi_f)
        id_fw.append(id_ref)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    ax1.plot(omega_range, iq_max, 'r-', linewidth=2)
    ax1.set_xlabel('电角速度 (rad/s)')
    ax1.set_ylabel('最大 Iq (A)')
    ax1.set_title('弱磁区域 - 最大转矩')
    ax1.grid(True)

    ax2.plot(omega_range, id_fw, 'b-', linewidth=2)
    ax2.set_xlabel('电角速度 (rad/s)')
    ax2.set_ylabel('Id 弱磁电流 (A)')
    ax2.set_title('弱磁电流 vs 转速')
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/flux_weakening.png', dpi=150)
    plt.show()


if __name__ == '__main__':
    print("=" * 60)
    print("  高级电机控制仿真 (FOC/SVPWM/MTPA/弱磁)")
    print("=" * 60)
    run_foc_simulation()
    plot_mtpa_curve()
    plot_flux_weakening_region()
    print("✅ 所有仿真完成")
