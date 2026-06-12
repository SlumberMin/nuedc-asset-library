"""
电机模型仿真 - 一阶惯性+死区+饱和
用法: python motor_simulation.py
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


class MotorModel:
    """直流电机模型: 一阶惯性 + 死区 + 饱和"""
    def __init__(self, tau=0.3, gain=1.0, dead_zone=0.5, saturation=12.0,
                 coulomb_friction=0.1, viscous_friction=0.05):
        self.tau = tau          # 时间常数 (s)
        self.gain = gain        # 增益
        self.dead_zone = dead_zone    # 死区电压 (V)
        self.saturation = saturation  # 饱和电压 (V)
        self.cf = coulomb_friction    # 库仑摩擦
        self.vf = viscous_friction    # 粘性摩擦系数
        self.speed = 0.0
        self.position = 0.0

    def _apply_nonlinearity(self, u):
        # 饱和
        u = max(-self.saturation, min(self.saturation, u))
        # 死区
        if abs(u) < self.dead_zone:
            u = 0
        elif u > 0:
            u -= self.dead_zone
        else:
            u += self.dead_zone
        return u

    def update(self, voltage, dt):
        u = self._apply_nonlinearity(voltage)
        # 摩擦力
        friction = 0
        if abs(self.speed) > 0.01:
            friction = self.cf * np.sign(self.speed) + self.vf * self.speed
        elif abs(u) < self.cf:
            u = 0
        # 一阶惯性
        self.speed += dt / self.tau * (self.gain * u - self.speed) - friction * dt
        self.position += self.speed * dt
        return self.speed, self.position

    def reset(self):
        self.speed = 0.0
        self.position = 0.0


class PIDController:
    def __init__(self, kp, ki, kd, out_min=-12, out_max=12):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        deriv = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error

        # 积分抗饱和: 仅在输出未饱和时累积积分，或误差方向有利于解除饱和时才积分
        candidate_integral = self.integral + error * dt
        p = self.kp * error
        i = self.ki * candidate_integral
        d = self.kd * deriv
        out = p + i + d

        if self.out_min < out < self.out_max:
            # 输出未饱和，正常更新积分
            self.integral = candidate_integral
        else:
            # 输出饱和，仅当误差能帮助解除饱和时才更新积分(conditional integration)
            if (out >= self.out_max and error < 0) or (out <= self.out_min and error > 0):
                self.integral = candidate_integral
            # 否则冻结积分项，防止windup

        out = p + self.ki * self.integral + d
        return max(self.out_min, min(self.out_max, out))


def simulate_speed_control(target_speed, duration=3.0, dt=0.001):
    """速度环仿真"""
    motor = MotorModel(tau=0.3, gain=10, dead_zone=1.0, saturation=12.0)
    pid = PIDController(1.5, 5.0, 0.01, out_min=-12, out_max=12)

    t_list, speed_list, voltage_list, target_list = [], [], [], []
    for i in range(int(duration / dt)):
        t = i * dt
        error = target_speed - motor.speed
        voltage = pid.compute(error, dt)
        motor.update(voltage, dt)
        t_list.append(t)
        speed_list.append(motor.speed)
        voltage_list.append(voltage)
        target_list.append(target_speed)
    return np.array(t_list), np.array(speed_list), np.array(voltage_list), np.array(target_list)


def simulate_position_control(target_pos, duration=5.0, dt=0.001):
    """位置环仿真"""
    motor = MotorModel(tau=0.3, gain=10, dead_zone=1.0, saturation=12.0)
    pid_speed = PIDController(1.0, 3.0, 0.005, out_min=-12, out_max=12)
    pid_pos = PIDController(2.0, 0.0, 0.1, out_min=-5, out_max=5)  # 位置环限速

    t_list, pos_list, speed_list, voltage_list = [], [], [], []
    for i in range(int(duration / dt)):
        t = i * dt
        pos_error = target_pos - motor.position
        target_speed = pid_pos.compute(pos_error, dt)
        speed_error = target_speed - motor.speed
        voltage = pid_speed.compute(speed_error, dt)
        motor.update(voltage, dt)
        t_list.append(t)
        pos_list.append(motor.position)
        speed_list.append(motor.speed)
        voltage_list.append(voltage)
    return np.array(t_list), np.array(pos_list), np.array(speed_list), np.array(voltage_list)


def main():
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 速度环
    for idx, target in enumerate([5.0, 10.0, 15.0]):
        t, spd, volt, tgt = simulate_speed_control(target)
        axes[0, 0].plot(t, spd, label=f'目标={target}')
    axes[0, 0].set_title('速度控制响应')
    axes[0, 0].set_ylabel('转速 (rad/s)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 速度控制 - 电压
    t, spd, volt, _ = simulate_speed_control(10.0)
    axes[0, 1].plot(t, volt)
    axes[0, 1].set_title('速度控制-驱动电压')
    axes[0, 1].set_ylabel('电压 (V)')
    axes[0, 1].grid(True, alpha=0.3)

    # 位置环
    t, pos, spd, volt = simulate_position_control(10.0)
    axes[1, 0].plot(t, pos)
    axes[1, 0].axhline(10.0, color='k', linestyle='--')
    axes[1, 0].set_title('位置控制响应')
    axes[1, 0].set_ylabel('位置 (rad)')
    axes[1, 0].grid(True, alpha=0.3)

    # 位置环速度曲线
    axes[1, 1].plot(t, spd)
    axes[1, 1].set_title('位置控制-速度曲线')
    axes[1, 1].set_ylabel('转速 (rad/s)')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].grid(True, alpha=0.3)

    for ax in axes.flatten():
        ax.set_xlabel('时间 (s)')

    plt.suptitle('电机模型仿真 (一阶惯性+死区+饱和+摩擦)', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('motor_simulation_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("仿真结果已保存: motor_simulation_result.png")


if __name__ == '__main__':
    main()
