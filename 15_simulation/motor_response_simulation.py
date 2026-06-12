"""
电机响应仿真
仿真不同负载/电压/参数下的电机速度和电流响应
使用wrappers.py中的PIDController进行闭环控制
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tests'))
from wrappers import PIDController, KalmanFilter


class DC_Motor_Model:
    """直流电机简化模型
    电气方程: V = R*i + L*di/dt + Ke*omega
    机械方程: J*domega/dt = Kt*i - B*omega - T_load
    """

    def __init__(self, R=1.0, L=0.001, Ke=0.01, Kt=0.01,
                 J=0.001, B=0.0001, dt=0.0001):
        self.R = R      # 电阻 (Ω)
        self.L = L      # 电感 (H)
        self.Ke = Ke    # 反电动势常数 (V/(rad/s))
        self.Kt = Kt    # 转矩常数 (N·m/A)
        self.J = J      # 转动惯量 (kg·m²)
        self.B = B      # 粘滞摩擦系数
        self.dt = dt

        self.i = 0.0        # 电流 (A)
        self.omega = 0.0    # 角速度 (rad/s)
        self.theta = 0.0    # 角度 (rad)

    def update(self, V, T_load=0.0):
        """一步仿真"""
        # 电流变化
        di = (V - self.R * self.i - self.Ke * self.omega) / self.L * self.dt
        self.i += di
        # 转矩
        T_motor = self.Kt * self.i
        # 角加速度
        domega = (T_motor - self.B * self.omega - T_load) / self.J * self.dt
        self.omega += domega
        self.theta += self.omega * self.dt
        return self.omega, self.i

    def get_rpm(self):
        return self.omega * 60.0 / (2.0 * np.pi)

    def reset(self):
        self.i = 0.0
        self.omega = 0.0
        self.theta = 0.0


def main():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('电机响应仿真', fontsize=16, fontweight='bold')

    # ═══════════════════════════════════════════════════════
    # 1. 不同电压下的开环响应
    # ═══════════════════════════════════════════════════════
    ax = axes[0, 0]
    voltages = [5.0, 12.0, 24.0, 36.0]
    dt_sim = 0.0001
    t_end = 0.5
    t = np.arange(0, t_end, dt_sim)

    for V in voltages:
        motor = DC_Motor_Model(dt=dt_sim)
        rpms = []
        for _ in range(len(t)):
            motor.update(V)
            rpms.append(motor.get_rpm())
        ax.plot(t * 1000, rpms, label=f'V={V}V')

    ax.set_title('不同电压下的开环速度响应')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('转速 (RPM)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ═══════════════════════════════════════════════════════
    # 2. 不同负载下的响应
    # ═══════════════════════════════════════════════════════
    ax = axes[0, 1]
    loads = [0.0, 0.001, 0.005, 0.01]
    for T_load in loads:
        motor = DC_Motor_Model(dt=dt_sim)
        rpms = []
        for _ in range(len(t)):
            motor.update(24.0, T_load)
            rpms.append(motor.get_rpm())
        ax.plot(t * 1000, rpms, label=f'T_load={T_load*1000:.1f}mN·m')

    ax.set_title('不同负载下的速度响应 (V=24V)')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('转速 (RPM)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ═══════════════════════════════════════════════════════
    # 3. PID闭环速度控制
    # ═══════════════════════════════════════════════════════
    ax = axes[1, 0]
    target_rpm = 3000
    dt_ctrl = 0.001  # 控制周期1ms
    t_ctrl = np.arange(0, 1.0, dt_ctrl)

    pid_params = [
        (0.005, 0.1, 0.0, '欠阻尼'),
        (0.002, 0.05, 0.0, '适中'),
        (0.001, 0.02, 0.0001, '带微分'),
    ]

    for kp, ki, kd, label in pid_params:
        pid = PIDController(kp=kp, ki=ki, kd=kd, output_min=0.0, output_max=48.0,
                            integral_max=100.0)
        motor = DC_Motor_Model(dt=dt_ctrl / 10)  # 电机模型10倍速
        rpms = []
        for _ in range(len(t_ctrl)):
            rpm = motor.get_rpm()
            u = pid.calc(target_rpm, rpm)
            # 电机子步
            for _ in range(10):
                motor.update(u)
            rpms.append(motor.get_rpm())
        ax.plot(t_ctrl * 1000, rpms, label=label)

    ax.axhline(y=target_rpm, color='k', linestyle='--', alpha=0.5, label=f'目标={target_rpm}RPM')
    ax.set_title('PID闭环速度控制')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('转速 (RPM)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ═══════════════════════════════════════════════════════
    # 4. 负载突变响应
    # ═══════════════════════════════════════════════════════
    ax = axes[1, 1]
    pid = PIDController(kp=0.002, ki=0.05, kd=0.0001, output_min=0.0, output_max=48.0,
                        integral_max=100.0)
    motor = DC_Motor_Model(dt=dt_ctrl / 10)
    rpms = []
    loads_timeline = []
    for i in range(len(t_ctrl)):
        # 突加负载在0.5s
        if t_ctrl[i] < 0.5:
            T_load = 0.0
        else:
            T_load = 0.005
        rpm = motor.get_rpm()
        u = pid.calc(target_rpm, rpm)
        for _ in range(10):
            motor.update(u, T_load)
        rpms.append(motor.get_rpm())
        loads_timeline.append(T_load * 1000)

    ax.plot(t_ctrl * 1000, rpms, 'b-', label='转速')
    ax.axhline(y=target_rpm, color='k', linestyle='--', alpha=0.5)
    ax.axvline(x=500, color='r', linestyle=':', alpha=0.5, label='负载突变')
    ax.set_title('负载突变响应 (0→5mN·m @0.5s)')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('转速 (RPM)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ═══════════════════════════════════════════════════════
    # 5. 不同电阻（电机参数）的影响
    # ═══════════════════════════════════════════════════════
    ax = axes[2, 0]
    resistances = [0.5, 1.0, 2.0, 5.0]
    for R in resistances:
        motor = DC_Motor_Model(R=R, dt=dt_sim)
        rpms = []
        for _ in range(len(t)):
            motor.update(12.0)
            rpms.append(motor.get_rpm())
        ax.plot(t * 1000, rpms, label=f'R={R}Ω')

    ax.set_title('不同电机电阻的影响 (V=12V)')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('转速 (RPM)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ═══════════════════════════════════════════════════════
    # 6. PID + Kalman滤波
    # ═══════════════════════════════════════════════════════
    ax = axes[2, 1]
    pid_raw = PIDController(kp=0.002, ki=0.05, kd=0.0001, output_min=0.0, output_max=48.0,
                            integral_max=100.0)
    pid_kf = PIDController(kp=0.002, ki=0.05, kd=0.0001, output_min=0.0, output_max=48.0,
                           integral_max=100.0)
    kf = KalmanFilter(dt=0.001, proc_noise=1.0, meas_noise=100.0)

    motor1 = DC_Motor_Model(dt=dt_ctrl / 10)
    motor2 = DC_Motor_Model(dt=dt_ctrl / 10)

    rpms_raw = []
    rpms_kf = []

    import random
    random.seed(42)
    for i in range(len(t_ctrl)):
        # 带噪声的测量
        rpm1 = motor1.get_rpm() + random.gauss(0, 50)
        rpm2 = motor2.get_rpm() + random.gauss(0, 50)

        u1 = pid_raw.calc(target_rpm, rpm1)
        filtered, _ = kf.step(rpm2)
        u2 = pid_kf.calc(target_rpm, filtered)

        for _ in range(10):
            motor1.update(u1)
            motor2.update(u2)
        rpms_raw.append(motor1.get_rpm())
        rpms_kf.append(motor2.get_rpm())

    ax.plot(t_ctrl * 1000, rpms_raw, 'r-', alpha=0.5, label='原始反馈')
    ax.plot(t_ctrl * 1000, rpms_kf, 'b-', label='Kalman滤波反馈')
    ax.axhline(y=target_rpm, color='k', linestyle='--', alpha=0.5)
    ax.set_title('PID + Kalman滤波降噪效果')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('转速 (RPM)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(out_dir, 'motor_response.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"[OK] 图表已保存: {out_path}")


if __name__ == '__main__':
    main()
