#!/usr/bin/env python3
"""
轨迹生成器仿真
支持梯形速度曲线、S曲线、五次多项式轨迹
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import dataclass
from enum import Enum

class TrajectoryType(Enum):
    TRAPEZOIDAL = 0
    S_CURVE = 1
    POLYNOMIAL_5TH = 2

@dataclass
class TrajectoryConfig:
    start_pos: float = 0.0
    target_pos: float = 10.0
    max_velocity: float = 5.0
    max_acceleration: float = 2.0
    max_jerk: float = 10.0
    total_time: float = 5.0

class TrajectoryGenerator:
    def __init__(self, dt=0.001):
        self.dt = dt
        self.current_time = 0.0
        self.initial_position = 0.0
        self.target_position = 0.0
        self.max_velocity = 0.0
        self.max_acceleration = 0.0
        self.max_jerk = 0.0
        self.total_time = 0.0
        self.trajectory_type = TrajectoryType.TRAPEZOIDAL
        
    def set_trapezoidal(self, start_pos, target_pos, max_vel, max_accel):
        """配置梯形轨迹"""
        self.trajectory_type = TrajectoryType.TRAPEZOIDAL
        self.initial_position = start_pos
        self.target_position = target_pos
        self.max_velocity = abs(max_vel)
        self.max_acceleration = abs(max_accel)
        self.current_time = 0.0
        
        # 计算各段时间
        distance = abs(target_pos - start_pos)
        ta = max_vel / max_accel
        accel_dist = 0.5 * max_accel * ta * ta
        
        if 2 * accel_dist >= distance:
            # 三角形曲线
            self.ta = np.sqrt(distance / max_accel)
            self.tc = 0.0
            self.td = self.ta
        else:
            # 梯形曲线
            self.ta = ta
            cruise_dist = distance - 2 * accel_dist
            self.tc = cruise_dist / max_vel
            self.td = self.ta
            
        self.total_time = self.ta + self.tc + self.td
        
    def set_s_curve(self, start_pos, target_pos, max_vel, max_accel, max_jerk):
        """配置S曲线轨迹"""
        self.trajectory_type = TrajectoryType.S_CURVE
        self.initial_position = start_pos
        self.target_position = target_pos
        self.max_velocity = abs(max_vel)
        self.max_acceleration = abs(max_accel)
        self.max_jerk = abs(max_jerk)
        self.current_time = 0.0
        
        # 使用五次多项式近似S曲线
        distance = abs(target_pos - start_pos)
        self.total_time = 3.0 * distance / max_vel  # 简化计算
        
    def set_polynomial_5th(self, start_pos, target_pos, start_vel, target_vel, 
                           start_accel, target_accel, total_time):
        """配置五次多项式轨迹"""
        self.trajectory_type = TrajectoryType.POLYNOMIAL_5TH
        self.initial_position = start_pos
        self.target_position = target_pos
        self.start_vel = start_vel
        self.target_vel = target_vel
        self.start_accel = start_accel
        self.target_accel = target_accel
        self.total_time = total_time
        self.current_time = 0.0
        
        # 求解五次多项式系数
        p0, p1 = start_pos, target_pos
        v0, v1 = start_vel, target_vel
        a0, a1 = start_accel, target_accel
        T = total_time
        
        self.c0 = p0
        self.c1 = v0
        self.c2 = a0 / 2.0
        self.c3 = (20*(p1-p0) - (8*v1+12*v0)*T - (3*a0-a1)*T*T) / (2*T**3)
        self.c4 = (-30*(p1-p0) + (14*v1+16*v0)*T + (3*a0-2*a1)*T*T) / (2*T**4)
        self.c5 = (12*(p1-p0) - 6*(v1+v0)*T + (a1-a0)*T*T) / (2*T**5)
        
    def calculate(self, t):
        """计算t时刻的轨迹点"""
        if self.trajectory_type == TrajectoryType.TRAPEZOIDAL:
            return self._calc_trapezoidal(t)
        elif self.trajectory_type == TrajectoryType.S_CURVE:
            return self._calc_s_curve(t)
        elif self.trajectory_type == TrajectoryType.POLYNOMIAL_5TH:
            return self._calc_polynomial_5th(t)
            
    def _calc_trapezoidal(self, t):
        """梯形曲线计算"""
        direction = 1.0 if self.target_position > self.initial_position else -1.0
        
        if t <= self.ta:
            # 加速段
            pos = self.initial_position + 0.5 * self.max_acceleration * t * t * direction
            vel = self.max_acceleration * t * direction
            accel = self.max_acceleration * direction
            state = 'accel'
        elif t <= self.ta + self.tc:
            # 匀速段
            t_cruise = t - self.ta
            accel_dist = 0.5 * self.max_acceleration * self.ta * self.ta
            pos = self.initial_position + (accel_dist + self.max_velocity * t_cruise) * direction
            vel = self.max_velocity * direction
            accel = 0.0
            state = 'cruise'
        elif t <= self.total_time:
            # 减速段
            t_decel = t - self.ta - self.tc
            accel_dist = 0.5 * self.max_acceleration * self.ta * self.ta
            cruise_dist = self.max_velocity * self.tc
            decel_dist = self.max_velocity * t_decel - 0.5 * self.max_acceleration * t_decel * t_decel
            pos = self.initial_position + (accel_dist + cruise_dist + decel_dist) * direction
            vel = (self.max_velocity - self.max_acceleration * t_decel) * direction
            accel = -self.max_acceleration * direction
            state = 'decel'
        else:
            pos = self.target_position
            vel = 0.0
            accel = 0.0
            state = 'done'
            
        return {'position': pos, 'velocity': vel, 'acceleration': accel, 'state': state}
        
    def _calc_s_curve(self, t):
        """S曲线计算(五次多项式近似)"""
        if t >= self.total_time:
            return {'position': self.target_position, 'velocity': 0.0, 
                    'acceleration': 0.0, 'state': 'done'}
                    
        normalized_time = t / self.total_time
        
        # 五次多项式
        s = 10 * normalized_time**3 - 15 * normalized_time**4 + 6 * normalized_time**5
        s_dot = (30 * normalized_time**2 - 60 * normalized_time**3 + 30 * normalized_time**4) / self.total_time
        s_ddot = (60 * normalized_time - 180 * normalized_time**2 + 120 * normalized_time**3) / (self.total_time**2)
        
        distance = self.target_position - self.initial_position
        
        pos = self.initial_position + distance * s
        vel = distance * s_dot
        accel = distance * s_ddot
        
        # 状态判断
        if normalized_time < 0.2:
            state = 'jerk_accel'
        elif normalized_time < 0.5:
            state = 'accel'
        elif normalized_time < 0.8:
            state = 'decel'
        else:
            state = 'jerk_decel'
            
        return {'position': pos, 'velocity': vel, 'acceleration': accel, 'state': state}
        
    def _calc_polynomial_5th(self, t):
        """五次多项式计算"""
        if t >= self.total_time:
            return {'position': self.target_position, 'velocity': 0.0,
                    'acceleration': 0.0, 'state': 'done'}
                    
        pos = self.c0 + self.c1*t + self.c2*t**2 + self.c3*t**3 + self.c4*t**4 + self.c5*t**5
        vel = self.c1 + 2*self.c2*t + 3*self.c3*t**2 + 4*self.c4*t**3 + 5*self.c5*t**4
        accel = 2*self.c2 + 6*self.c3*t + 12*self.c4*t**2 + 20*self.c5*t**3
        jerk = 6*self.c3 + 24*self.c4*t + 60*self.c5*t**2
        
        return {'position': pos, 'velocity': vel, 'acceleration': accel, 
                'jerk': jerk, 'state': 'polynomial'}
    
    def generate_trajectory(self):
        """生成完整轨迹数据"""
        times = np.arange(0, self.total_time + self.dt, self.dt)
        positions = []
        velocities = []
        accelerations = []
        jerks = []
        
        for t in times:
            result = self.calculate(t)
            positions.append(result['position'])
            velocities.append(result['velocity'])
            accelerations.append(result['acceleration'])
            jerks.append(result.get('jerk', 0.0))
            
        return {
            'time': times,
            'position': np.array(positions),
            'velocity': np.array(velocities),
            'acceleration': np.array(accelerations),
            'jerk': np.array(jerks)
        }

def plot_trajectory_comparison(config):
    """绘制三种轨迹对比图"""
    dt = 0.001
    
    # 梯形轨迹
    trap = TrajectoryGenerator(dt)
    trap.set_trapezoidal(config.start_pos, config.target_pos, 
                        config.max_velocity, config.max_acceleration)
    trap_data = trap.generate_trajectory()
    
    # S曲线
    s_curve = TrajectoryGenerator(dt)
    s_curve.set_s_curve(config.start_pos, config.target_pos,
                       config.max_velocity, config.max_acceleration, config.max_jerk)
    s_curve_data = s_curve.generate_trajectory()
    
    # 五次多项式
    poly = TrajectoryGenerator(dt)
    poly.set_polynomial_5th(config.start_pos, config.target_pos,
                           0.0, 0.0, 0.0, 0.0, config.total_time)
    poly_data = poly.generate_trajectory()
    
    # 绘图
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('Trajectory Generator Comparison', fontsize=16)
    
    # 位置
    ax1 = axes[0, 0]
    ax1.plot(trap_data['time'], trap_data['position'], 'b-', label='Trapezoidal', linewidth=2)
    ax1.plot(s_curve_data['time'], s_curve_data['position'], 'r-', label='S-Curve', linewidth=2)
    ax1.plot(poly_data['time'], poly_data['position'], 'g-', label='5th Polynomial', linewidth=2)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position')
    ax1.set_title('Position Profile')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 速度
    ax2 = axes[0, 1]
    ax2.plot(trap_data['time'], trap_data['velocity'], 'b-', label='Trapezoidal', linewidth=2)
    ax2.plot(s_curve_data['time'], s_curve_data['velocity'], 'r-', label='S-Curve', linewidth=2)
    ax2.plot(poly_data['time'], poly_data['velocity'], 'g-', label='5th Polynomial', linewidth=2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity')
    ax2.set_title('Velocity Profile')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 加速度
    ax3 = axes[1, 0]
    ax3.plot(trap_data['time'], trap_data['acceleration'], 'b-', label='Trapezoidal', linewidth=2)
    ax3.plot(s_curve_data['time'], s_curve_data['acceleration'], 'r-', label='S-Curve', linewidth=2)
    ax3.plot(poly_data['time'], poly_data['acceleration'], 'g-', label='5th Polynomial', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Acceleration')
    ax3.set_title('Acceleration Profile')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 加加速度(Jerk)
    ax4 = axes[1, 1]
    ax4.plot(trap_data['time'], trap_data['jerk'], 'b-', label='Trapezoidal', linewidth=2)
    ax4.plot(s_curve_data['time'], s_curve_data['jerk'], 'r-', label='S-Curve', linewidth=2)
    ax4.plot(poly_data['time'], poly_data['jerk'], 'g-', label='5th Polynomial', linewidth=2)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Jerk')
    ax4.set_title('Jerk Profile')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('trajectory_comparison.png', dpi=150, bbox_inches='tight')
    plt.close('all')

def plot_trajectory_details(config):
    """绘制单个轨迹的详细图"""
    dt = 0.001
    
    gen = TrajectoryGenerator(dt)
    gen.set_trapezoidal(config.start_pos, config.target_pos,
                       config.max_velocity, config.max_acceleration)
    
    data = gen.generate_trajectory()
    
    fig, axes = plt.subplots(4, 1, figsize=(10, 12))
    fig.suptitle('Trapezoidal Trajectory Details', fontsize=16)
    
    # 位置
    ax1 = axes[0]
    ax1.plot(data['time'], data['position'], 'b-', linewidth=2)
    ax1.set_ylabel('Position')
    ax1.set_title('Position Profile')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=config.target_pos, color='r', linestyle='--', alpha=0.5, label='Target')
    ax1.legend()
    
    # 速度
    ax2 = axes[1]
    ax2.plot(data['time'], data['velocity'], 'r-', linewidth=2)
    ax2.set_ylabel('Velocity')
    ax2.set_title('Velocity Profile')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=config.max_velocity, color='g', linestyle='--', alpha=0.5, label='Max Velocity')
    ax2.legend()
    
    # 加速度
    ax3 = axes[2]
    ax3.plot(data['time'], data['acceleration'], 'g-', linewidth=2)
    ax3.set_ylabel('Acceleration')
    ax3.set_title('Acceleration Profile')
    ax3.grid(True, alpha=0.3)
    
    # 相图(位置-速度)
    ax4 = axes[3]
    ax4.plot(data['position'], data['velocity'], 'm-', linewidth=2)
    ax4.set_xlabel('Position')
    ax4.set_ylabel('Velocity')
    ax4.set_title('Phase Portrait (Position vs Velocity)')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('trajectory_details.png', dpi=150, bbox_inches='tight')
    plt.close('all')

def demo_trajectory_with_pid():
    """演示轨迹跟踪PID控制"""
    dt = 0.001
    total_time = 8.0
    
    # 生成参考轨迹
    config = TrajectoryConfig(start_pos=0.0, target_pos=10.0, 
                            max_velocity=5.0, max_acceleration=2.0)
    
    gen = TrajectoryGenerator(dt)
    gen.set_trapezoidal(config.start_pos, config.target_pos,
                       config.max_velocity, config.max_acceleration)
    
    ref_data = gen.generate_trajectory()
    
    # 简单PID控制器
    class SimplePID:
        def __init__(self, Kp, Ki, Kd):
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd
            self.integral = 0.0
            self.prev_error = 0.0
            
        def update(self, error, dt):
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            self.prev_error = error
            return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
    
    pid = SimplePID(Kp=10.0, Ki=1.0, Kd=2.0)
    
    # 仿真
    times = np.arange(0, total_time + dt, dt)
    actual_positions = []
    actual_velocities = []
    control_signals = []
    
    position = 0.0
    velocity = 0.0
    inertia = 1.0  # 惯性
    damping = 0.5  # 阻尼
    
    for t in times:
        # 获取参考轨迹
        if t < gen.total_time:
            ref = gen.calculate(t)
            ref_pos = ref['position']
            ref_vel = ref['velocity']
        else:
            ref_pos = config.target_pos
            ref_vel = 0.0
            
        # 计算误差
        error = ref_pos - position
        
        # PID控制
        control = pid.update(error, dt)
        
        # 系统模型(二阶系统)
        acceleration = (control - damping * velocity) / inertia
        velocity += acceleration * dt
        position += velocity * dt
        
        actual_positions.append(position)
        actual_velocities.append(velocity)
        control_signals.append(control)
    
    # 绘图
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))
    fig.suptitle('Trajectory Tracking with PID', fontsize=16)
    
    # 位置跟踪
    ax1 = axes[0]
    ax1.plot(times, ref_data['position'][:len(times)], 'b-', label='Reference', linewidth=2)
    ax1.plot(times, actual_positions, 'r-', label='Actual', linewidth=2)
    ax1.set_ylabel('Position')
    ax1.set_title('Position Tracking')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 跟踪误差
    ax2 = axes[1]
    tracking_error = np.array(ref_data['position'][:len(times)]) - np.array(actual_positions)
    ax2.plot(times, tracking_error, 'g-', linewidth=2)
    ax2.set_ylabel('Error')
    ax2.set_title('Tracking Error')
    ax2.grid(True, alpha=0.3)
    
    # 控制信号
    ax3 = axes[2]
    ax3.plot(times, control_signals, 'm-', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Control')
    ax3.set_title('Control Signal')
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('trajectory_tracking.png', dpi=150, bbox_inches='tight')
    plt.close('all')

if __name__ == '__main__':
    # 配置参数
    config = TrajectoryConfig(
        start_pos=0.0,
        target_pos=10.0,
        max_velocity=5.0,
        max_acceleration=2.0,
        max_jerk=10.0,
        total_time=5.0
    )
    
    print("=" * 60)
    print("轨迹生成器仿真")
    print("=" * 60)
    
    print("\n1. 绘制轨迹对比图...")
    plot_trajectory_comparison(config)
    
    print("\n2. 绘制梯形轨迹详细图...")
    plot_trajectory_details(config)
    
    print("\n3. 演示轨迹跟踪PID控制...")
    demo_trajectory_with_pid()
    
    print("\n仿真完成！图片已保存。")