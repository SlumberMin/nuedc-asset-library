#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机+舵机串级控制仿真
外环：位置/角度控制 → 内环：速度/电流控制
nuedc-asset-library V3
"""

import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# 1. PID控制器
# ============================================================

class PID:
    def __init__(self, kp, ki, kd, out_min=-100, out_max=100, integ_limit=50):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integ_limit = integ_limit
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integ_limit, self.integ_limit)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return np.clip(output, self.out_min, self.out_max)

    def reset(self):
        self.integral = 0
        self.prev_error = 0

# ============================================================
# 2. 电机模型（直流电机）
# ============================================================

class DC_Motor:
    def __init__(self, J=0.01, b=0.1, Kt=0.1, Ke=0.1, R=1.0, L=0.01):
        """J:转动惯量, b:摩擦, Kt:力矩常数, Ke:反电动势常数, R:电阻, L:电感"""
        self.J, self.b, self.Kt, self.Ke, self.R, self.L = J, b, Kt, Ke, R, L
        self.omega = 0  # 角速度 (rad/s)
        self.current = 0  # 电流 (A)

    def update(self, voltage, dt, load_torque=0):
        """电压输入→角速度输出"""
        # 电气方程: L*di/dt = V - R*i - Ke*omega
        # 机械方程: J*domega/dt = Kt*i - b*omega - Tload
        emf = self.Ke * self.omega
        di = (voltage - self.R * self.current - emf) / self.L
        self.current += di * dt

        torque = self.Kt * self.current - self.b * self.omega - load_torque
        domega = torque / self.J
        self.omega += domega * dt
        return self.omega

# ============================================================
# 3. 舵机模型
# ============================================================

class Servo:
    def __init__(self, speed=60, deadband=0.5, delay=0.05):
        """speed: °/s, deadband: 死区(°), delay: 响应延迟(s)"""
        self.speed = speed
        self.deadband = deadband
        self.delay = delay
        self.angle = 0  # 当前角度(°)
        self.target = 0
        self.buffer = []

    def set_target(self, target_deg):
        self.target = np.clip(target_deg, -90, 90)

    def update(self, dt):
        """模拟舵机响应（含死区和延迟）"""
        error = self.target - self.angle
        # 死区
        if abs(error) < self.deadband:
            return self.angle
        # 有限速度响应
        max_step = self.speed * dt
        step = np.clip(error, -max_step, max_step)
        self.angle += step
        return self.angle

# ============================================================
# 4. 串级控制器
# ============================================================

class CascadeMotorController:
    """电机串级控制：外环位置→内环速度"""
    def __init__(self):
        self.outer_pid = PID(kp=2.0, ki=0.5, kd=0.1, out_min=-50, out_max=50)
        self.inner_pid = PID(kp=10.0, ki=5.0, kd=0.01, out_min=-24, out_max=24)
        self.motor = DC_Motor()
        self.position = 0

    def update(self, target_pos, dt, load_torque=0):
        # 外环：位置→速度指令
        pos_error = target_pos - self.position
        speed_cmd = self.outer_pid.compute(pos_error, dt)
        # 内环：速度→电压
        vel_error = speed_cmd - self.motor.omega
        voltage = self.inner_pid.compute(vel_error, dt)
        # 电机响应
        omega = self.motor.update(voltage, dt, load_torque)
        self.position += omega * dt
        return self.position, omega, voltage

class CascadeServoController:
    """舵机串级控制：外环角度→内环角速度"""
    def __init__(self):
        self.outer_pid = PID(kp=3.0, ki=1.0, kd=0.2, out_min=-200, out_max=200)
        self.servo = Servo(speed=120, deadband=0.3)

    def update(self, target_angle, dt):
        self.servo.set_target(target_angle)
        angle = self.servo.update(dt)
        return angle

# ============================================================
# 5. 仿真
# ============================================================

def run_cascade_simulation():
    np.random.seed(42)
    dt = 0.001
    t_total = 5.0
    steps = int(t_total / dt)
    t = np.arange(steps) * dt

    # 目标：阶跃+正弦
    motor_target = np.where(t < 1.0, 0, np.where(t < 2.0, 10, 10 + 5*np.sin(2*np.pi*1.0*(t-2))))
    servo_target = 30 * np.sin(2 * np.pi * 0.5 * t)

    # 电机串级
    motor_ctrl = CascadeMotorController()
    motor_pos = np.zeros(steps)
    motor_vel = np.zeros(steps)
    motor_volt = np.zeros(steps)

    # 舵机
    servo_ctrl = CascadeServoController()
    servo_ang = np.zeros(steps)

    # 负载扰动
    load = np.zeros(steps)
    load[int(2/dt):int(3/dt)] = 0.5  # 2-3s施加负载

    for i in range(steps):
        motor_pos[i], motor_vel[i], motor_volt[i] = motor_ctrl.update(motor_target[i], dt, load[i])
        servo_ang[i] = servo_ctrl.update(servo_target[i], dt)

    # 可视化
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))

    # 电机位置
    ax = axes[0, 0]
    ax.plot(t, motor_target, 'k--', linewidth=1.5, label='目标')
    ax.plot(t, motor_pos, 'b-', linewidth=1, label='实际')
    ax.set_ylabel('位置 (rad)'); ax.set_title('电机串级控制 - 位置'); ax.legend(); ax.grid(True, alpha=0.3)

    # 电机速度
    ax = axes[1, 0]
    ax.plot(t, motor_vel, 'g-', linewidth=1)
    ax.set_ylabel('角速度 (rad/s)'); ax.set_title('电机角速度'); ax.grid(True, alpha=0.3)

    # 电机电压+负载
    ax = axes[2, 0]
    ax.plot(t, motor_volt, 'r-', linewidth=1, label='电压')
    ax.plot(t, load * 50, 'k--', linewidth=1, label='负载×50')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('电压 (V)')
    ax.set_title('控制电压'); ax.legend(); ax.grid(True, alpha=0.3)

    # 舵机角度
    ax = axes[0, 1]
    ax.plot(t, servo_target, 'k--', linewidth=1.5, label='目标')
    ax.plot(t, servo_ang, 'r-', linewidth=1, label='实际')
    ax.set_ylabel('角度 (°)'); ax.set_title('舵机跟踪'); ax.legend(); ax.grid(True, alpha=0.3)

    # 电机误差
    ax = axes[1, 1]
    motor_err = motor_target - motor_pos
    ax.plot(t, motor_err, 'b-', linewidth=1)
    ax.set_ylabel('误差 (rad)'); ax.set_title('电机位置误差'); ax.grid(True, alpha=0.3)

    # 舵机误差
    ax = axes[2, 1]
    servo_err = servo_target - servo_ang
    ax.plot(t, servo_err, 'r-', linewidth=1)
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('误差 (°)')
    ax.set_title('舵机角度误差'); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('motor_servo_cascade.png', dpi=150)
    plt.show()

    print("=" * 50)
    print("电机+舵机串级控制仿真结果")
    print("=" * 50)
    print(f"电机位置 RMSE: {np.sqrt(np.mean(motor_err[1000:]**2)):.4f} rad")
    print(f"舵机角度 RMSE: {np.sqrt(np.mean(servo_err[1000:]**2)):.4f} °")
    print(f"电机最大超调: {np.max(motor_pos[int(1/dt):int(2/dt)]) - 10:.4f} rad")

if __name__ == '__main__':
    run_cascade_simulation()
