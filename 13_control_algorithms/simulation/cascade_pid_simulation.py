"""
串级PID仿真 (Cascade PID Simulation)

仿真场景: 电机位置控制
- 外环: 位置环PID
- 内环: 速度环PID
- 被控对象: 直流电机模型 (含惯量+摩擦)

对比: 串级PID vs 单环PID

依赖: numpy, matplotlib
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ============== 仿真参数 ==============
dt = 0.001          # 1ms
T_total = 2.0
steps = int(T_total / dt)

# ============== 电机模型参数 ==============
J = 0.01            # 转动惯量 kg·m²
b = 0.1             # 粘性摩擦系数
Kt = 0.1            # 转矩常数 Nm/A
R = 1.0             # 电枢电阻
L = 0.001           # 电枢电感


class DCMotor:
    """直流电机简化模型"""
    def __init__(self):
        self.position = 0.0
        self.velocity = 0.0
        self.current = 0.0

    def step(self, voltage, dt):
        """输入电压，更新状态"""
        # 电流环: L*di/dt + R*i = V - Ke*omega
        Ke = Kt  # 反电动势常数
        back_emf = Ke * self.velocity
        di = (voltage - R * self.current - back_emf) / L * dt
        self.current += di

        # 力矩
        torque = Kt * self.current

        # 机械: J*dω/dt = torque - b*omega
        dv = (torque - b * self.velocity) / J * dt
        self.velocity += dv
        self.position += self.velocity * dt

        return self.position, self.velocity


class PIDController:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def calc(self, setpoint, feedback, dt):
        error = setpoint - feedback
        self.integral += error * dt
        self.integral = np.clip(self.integral, self.out_min / self.ki if self.ki > 0 else -1e6,
                                self.out_max / self.ki if self.ki > 0 else 1e6)
        d = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * d
        return np.clip(output, self.out_min, self.out_max)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


def simulate_cascade_pid():
    """串级PID: 位置环(外) + 速度环(内)"""
    motor = DCMotor()
    # 外环 - 位置环 (输出为速度指令)
    pos_pid = PIDController(kp=50.0, ki=1.0, kd=5.0, out_min=-100, out_max=100)
    # 内环 - 速度环 (输出为电压)
    vel_pid = PIDController(kp=10.0, ki=50.0, kd=0.1, out_min=-24, out_max=24)

    positions = []
    velocities = []
    vel_setpoints = []

    # 目标位置: 阶跃
    target_pos = 1.0  # rad

    for i in range(steps):
        t = i * dt

        # 外环: 位置 -> 速度指令
        vel_cmd = pos_pid.calc(target_pos, motor.position, dt)

        # 内环: 速度指令 -> 电压
        voltage = vel_pid.calc(vel_cmd, motor.velocity, dt)

        # 电机更新
        pos, vel = motor.step(voltage, dt)

        positions.append(pos)
        velocities.append(vel)
        vel_setpoints.append(vel_cmd)

    return np.array(positions), np.array(velocities), np.array(vel_setpoints)


def simulate_single_pid():
    """单环PID: 只有位置环，直接输出电压"""
    motor = DCMotor()
    pos_pid = PIDController(kp=80.0, ki=2.0, kd=10.0, out_min=-24, out_max=24)

    positions = []
    velocities = []

    target_pos = 1.0

    for i in range(steps):
        voltage = pos_pid.calc(target_pos, motor.position, dt)
        pos, vel = motor.step(voltage, dt)
        positions.append(pos)
        velocities.append(vel)

    return np.array(positions), np.array(velocities)


def simulate_with_disturbance():
    """串级PID带扰动"""
    motor = DCMotor()
    pos_pid = PIDController(kp=50.0, ki=1.0, kd=5.0, out_min=-100, out_max=100)
    vel_pid = PIDController(kp=10.0, ki=50.0, kd=0.1, out_min=-24, out_max=24)

    positions = []
    target_pos = 1.0

    for i in range(steps):
        t = i * dt
        vel_cmd = pos_pid.calc(target_pos, motor.position, dt)
        voltage = vel_pid.calc(vel_cmd, motor.velocity, dt)

        # 在t=1.0s施加阶跃扰动负载
        if t >= 1.0:
            motor.current -= 0.5 * dt  # 模拟负载扰动

        pos, vel = motor.step(voltage, dt)
        positions.append(pos)

    return np.array(positions)


def compute_metrics(pos, target=1.0):
    """计算性能指标"""
    time = np.arange(len(pos)) * dt
    overshoot = (np.max(pos) - target) / target * 100

    # 上升时间 (10%~90%)
    idx_10 = np.argmax(pos >= 0.1 * target)
    idx_90 = np.argmax(pos >= 0.9 * target)
    rise_time = (idx_90 - idx_10) * dt

    # 调节时间 (进入±2%带)
    settling_idx = len(pos) - 1
    for j in range(len(pos) - 1, -1, -1):
        if abs(pos[j] - target) > 0.02 * target:
            settling_idx = j + 1
            break
    settling_time = settling_idx * dt

    # 稳态误差
    ss_error = abs(pos[-1] - target)

    return {
        'overshoot': overshoot,
        'rise_time': rise_time,
        'settling_time': settling_time,
        'ss_error': ss_error
    }


def main():
    time = np.arange(steps) * dt

    # 运行仿真
    cas_pos, cas_vel, cas_vel_sp = simulate_cascade_pid()
    sin_pos, sin_vel = simulate_single_pid()
    dist_pos = simulate_with_disturbance()

    # 性能指标
    cas_metrics = compute_metrics(cas_pos)
    sin_metrics = compute_metrics(sin_pos)

    print("=" * 60)
    print("Cascade PID vs Single PID Performance")
    print("=" * 60)
    print(f"{'Metric':<20} {'Cascade PID':>12} {'Single PID':>12}")
    print(f"{'Overshoot (%)':<20} {cas_metrics['overshoot']:>12.2f} {sin_metrics['overshoot']:>12.2f}")
    print(f"{'Rise Time (s)':<20} {cas_metrics['rise_time']:>12.4f} {sin_metrics['rise_time']:>12.4f}")
    print(f"{'Settling Time (s)':<20} {cas_metrics['settling_time']:>12.4f} {sin_metrics['settling_time']:>12.4f}")
    print(f"{'SS Error':<20} {cas_metrics['ss_error']:>12.6f} {sin_metrics['ss_error']:>12.6f}")

    # 绘图
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Cascade PID vs Single PID Simulation', fontsize=14)

    # 1. 位置对比
    ax1 = axes[0, 0]
    ax1.plot(time, cas_pos, 'b-', label='Cascade PID', linewidth=1.5)
    ax1.plot(time, sin_pos, 'r--', label='Single PID', linewidth=1.5)
    ax1.axhline(y=1.0, color='k', linestyle=':', alpha=0.5, label='Target')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (rad)')
    ax1.set_title('Position Response')
    ax1.legend()
    ax1.grid(True)

    # 2. 速度对比
    ax2 = axes[0, 1]
    ax2.plot(time, cas_vel, 'b-', label='Cascade velocity', linewidth=1)
    ax2.plot(time, cas_vel_sp, 'g--', label='Velocity setpoint', linewidth=1, alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.set_title('Velocity (Cascade PID)')
    ax2.legend()
    ax2.grid(True)

    # 3. 扰动响应
    ax3 = axes[1, 0]
    ax3.plot(time, cas_pos, 'b-', label='No disturbance', linewidth=1)
    ax3.plot(time, dist_pos, 'r-', label='With disturbance at t=1s', linewidth=1)
    ax3.axhline(y=1.0, color='k', linestyle=':', alpha=0.5)
    ax3.axvline(x=1.0, color='orange', linestyle='--', alpha=0.5, label='Disturbance')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (rad)')
    ax3.set_title('Disturbance Rejection')
    ax3.legend()
    ax3.grid(True)

    # 4. 误差
    ax4 = axes[1, 1]
    ax4.plot(time, 1.0 - cas_pos, 'b-', label='Cascade PID error', linewidth=1)
    ax4.plot(time, 1.0 - sin_pos, 'r--', label='Single PID error', linewidth=1)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Error (rad)')
    ax4.set_title('Position Error')
    ax4.legend()
    ax4.grid(True)

    plt.tight_layout()
    plt.savefig('cascade_pid_simulation.png', dpi=150)
    plt.close('all')


if __name__ == '__main__':
    main()
