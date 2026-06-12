#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电赛控制算法仿真对比工具

统一测试平台：二阶振荡系统（弹簧-阻尼系统）
对比算法：PID, LADRC, ADRC, LQR, SMC, 超螺旋SMC, 模糊PID

测试场景：
1. 阶跃响应 - 测试系统响应速度和超调量
2. 正弦跟踪 - 测试轨迹跟踪性能
3. 阶跃扰动抑制 - 测试抗扰动能力
4. 参数摄动鲁棒性 - 测试对系统参数变化的敏感度

输出：性能指标对比表 + matplotlib可视化对比图
"""

import numpy as np
_trapz = np.trapezoid if hasattr(np, 'trapezoid') else np.trapezoid  # V2审计: numpy兼容
# numpy 2.x 兼容: np.trapezoid → np.trapezoid
if not hasattr(np, 'trapz'):
    np.trapezoid = np.trapezoid
import matplotlib
matplotlib.use('Agg')  # 无头模式，适用于服务器环境
import matplotlib.pyplot as plt

# ==================== 系统模型 ====================

class SecondOrderSystem:
    """
    二阶振荡系统（弹簧-阻尼系统）

    数学模型：
    d²x/dt² + 2*ζ*ωn*dx/dt + ωn²*x = ωn²*u

    参数：
    - ωn: 自然频率 (rad/s)
    - ζ: 阻尼比 (0 < ζ < 1 为振荡系统)
    """

    def __init__(self, wn=10.0, zeta=0.3, dt=0.001):
        """
        初始化系统

        Args:
            wn: 自然频率 (rad/s)
            zeta: 阻尼比
            dt: 采样时间 (s)
        """
        self.wn = wn
        self.zeta = zeta
        self.dt = dt

        # 系统状态 [位置, 速度]
        self.x1 = 0.0  # 位置
        self.x2 = 0.0  # 速度

    def reset(self):
        """重置系统状态"""
        self.x1 = 0.0
        self.x2 = 0.0

    def step(self, u, disturbance=0.0):
        """
        系统一步仿真（欧拉前向法）

        Args:
            u: 控制输入
            disturbance: 扰动输入

        Returns:
            系统输出（位置）
        """
        # 系统动态：d²x/dt² = -2*ζ*ωn*dx/dt - ωn²*x + ωn²*u + disturbance
        dx1 = self.x2
        dx2 = -2 * self.zeta * self.wn * self.x2 - self.wn**2 * self.x1 + self.wn**2 * u + disturbance

        # 离散化
        self.x1 += dx1 * self.dt
        self.x2 += dx2 * self.dt

        return self.x1


# ==================== 控制算法实现 ====================

class PIDController:
    """经典PID控制器"""

    def __init__(self, Kp=1.0, Ki=0.5, Kd=0.1, dt=0.001):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0.0
        self.error_last = 0.0

    def reset(self):
        self.integral = 0.0
        self.error_last = 0.0

    def compute(self, target, measurement):
        error = target - measurement

        # P
        p_out = self.Kp * error

        # I
        self.integral += error * self.dt
        i_out = self.Ki * self.integral

        # D
        d_out = self.Kd * (error - self.error_last) / self.dt
        self.error_last = error

        return p_out + i_out + d_out


class LADRCController:
    """
    线性自抗扰控制（简化版）

    只需两个参数：控制带宽wc和观测器带宽wo
    """

    def __init__(self, wc=10.0, wo=30.0, dt=0.001, b0=1.0):
        self.wc = wc
        self.wo = wo
        self.dt = dt
        self.b0 = b0

        # 状态观测器
        self.z1 = 0.0
        self.z2 = 0.0

    def reset(self):
        self.z1 = 0.0
        self.z2 = 0.0

    def compute(self, target, measurement):
        # 跟踪微分器
        e_td = target - self.z1

        # 扩展状态观测器（ESO）
        e_o = measurement - self.z1
        self.z1 += self.dt * (self.z2 + self.wo * 3 * e_o)
        self.z2 += self.dt * (self.b0 * (measurement - self.z1) + self.wo**3 * e_o)

        # 状态反馈
        u = (self.wc**2 * (target - self.z1) - 2 * self.wc * self.z2) / self.b0

        return u


class LQRController:
    """
    线性二次调节器（LQR）

    需要精确的系统模型
    """

    def __init__(self, A, B, Q, R, dt=0.001):
        """
        初始化LQR

        Args:
            A: 系统矩阵
            B: 控制矩阵
            Q: 状态权重矩阵
            R: 控制权重
            dt: 采样时间
        """
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.dt = dt

        # 简化的增益计算（实际应该用代数Riccati方程）
        # 这里用近似方法
        self.K = np.linalg.inv(R + B.T @ Q @ B) @ B.T @ Q @ A

    def reset(self):
        pass

    def compute(self, target, measurement):
        # 状态反馈：u = -K * x（这里简化为基于误差的反馈）
        error = target - measurement
        return self.K[0][0] * error


class SMController:
    """
    滑模控制（SMC）

    使用趋近律方法
    """

    def __init__(self, lambda_smc=10.0, epsilon=0.5, k=2.0, dt=0.001):
        self.lambda_smc = lambda_smc
        self.epsilon = epsilon
        self.k = k
        self.dt = dt
        self.prev_error = 0.0

    def reset(self):
        self.prev_error = 0.0

    def compute(self, target, measurement):
        error = target - measurement
        de = (error - self.prev_error) / self.dt
        self.prev_error = error

        # 滑模面：s = e + λ*∫e
        s = error + self.lambda_smc * error * self.dt

        # 趋近律：ṡ = -ε*sign(s) - k*s
        if abs(s) > self.epsilon:
            ds = -self.epsilon * np.sign(s) - self.k * s
        else:
            ds = -self.k * s

        return ds


class SuperTwistingSMC:
    """
    超螺旋滑模控制

    二阶滑模，抑制抖振
    """

    def __init__(self, lambda_st=5.0, alpha=2.0, beta=3.0, dt=0.001):
        self.lambda_st = lambda_st
        self.alpha = alpha
        self.beta = beta
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target, measurement):
        error = target - measurement
        de = (error - self.prev_error) / self.dt
        self.prev_error = error

        # 滑模面
        s = error + self.lambda_st * de

        # 超螺旋控制律
        self.integral += np.sign(s) * self.dt
        u = -self.alpha * np.sign(s) * np.sqrt(np.abs(s)) - self.beta * self.integral

        return u


class FuzzyPIDController:
    """
    模糊PID控制器

    使用简化的模糊规则
    """

    def __init__(self, Kp=1.0, Ki=0.5, Kd=0.1, dt=0.001):
        self.Kp_base = Kp
        self.Ki_base = Ki
        self.Kd_base = Kd
        self.dt = dt
        self.integral = 0.0
        self.error_last = 0.0

    def reset(self):
        self.integral = 0.0
        self.error_last = 0.0

    def _fuzzy_adjust(self, error, de):
        """简单的模糊调整规则"""
        # 模糊化
        e_norm = np.clip(error / 10.0, -1, 1)
        de_norm = np.clip(de / 10.0, -1, 1)

        # 简单的模糊规则
        if abs(e_norm) > 0.5:
            # 大误差 → 增大Kp, 减小Ki, 增大Kd
            Kp = self.Kp_base * 1.5
            Ki = self.Ki_base * 0.5
            Kd = self.Kd_base * 1.5
        elif abs(de_norm) > 0.5:
            # 快速变化 → 增大Kd
            Kp = self.Kp_base * 1.2
            Ki = self.Ki_base * 0.8
            Kd = self.Kd_base * 2.0
        else:
            # 正常范围
            Kp = self.Kp_base
            Ki = self.Ki_base
            Kd = self.Kd_base

        return Kp, Ki, Kd

    def compute(self, target, measurement):
        error = target - measurement
        de = (error - self.error_last) / self.dt

        Kp, Ki, Kd = self._fuzzy_adjust(error, de)

        p_out = Kp * error
        self.integral += error * self.dt
        i_out = Ki * self.integral
        d_out = Kd * de

        self.error_last = error

        return p_out + i_out + d_out


# ==================== 性能评估函数 ====================

def calculate_performance(t, y, target):
    """
    计算性能指标

    Args:
        t: 时间数组
        y: 输出数组
        target: 目标值

    Returns:
        性能指标字典
    """
    # 超调量
    overshoot = (np.max(y) - target) / target * 100 if target != 0 else 0

    # 调节时间（2%误差带）
    error_band = 0.02 * abs(target) if target != 0 else 0.1
    settled_idx = np.where(np.abs(y - target) > error_band)[0]
    if len(settled_idx) > 0:
        settling_time = t[settled_idx[-1]]
    else:
        settling_time = 0.0

    # 上升时间（10%到90%）
    rise_start_idx = np.where(y > 0.1 * target)[0]
    rise_end_idx = np.where(y > 0.9 * target)[0]
    if len(rise_start_idx) > 0 and len(rise_end_idx) > 0:
        rise_time = t[rise_end_idx[0]] - t[rise_start_idx[0]]
    else:
        rise_time = 0.0

    # 稳态误差
    steady_state_error = abs(np.mean(y[-100:]) - target) if len(y) >= 100 else abs(y[-1] - target)

    # IAE（积分绝对误差）
    iae = _trapz(np.abs(y - target), t)

    # ITAE（时间加权积分绝对误差）
    itae = _trapz(t * np.abs(y - target), t)

    return {
        'overshoot': overshoot,
        'settling_time': settling_time,
        'rise_time': rise_time,
        'steady_state_error': steady_state_error,
        'iae': iae,
        'itae': itae
    }


# ==================== 仿真场景 ====================

def simulate_step_response(controllers, dt=0.001, duration=2.0, target=10.0):
    """
    阶跃响应仿真

    Args:
        controllers: 控制器字典 {名称: 控制器对象}
        dt: 采样时间
        duration: 仿真时长
        target: 目标值

    Returns:
        results: 结果字典 {名称: {'t': 时间, 'y': 输出, 'u': 控制输入}}
    """
    results = {}

    for name, ctrl in controllers.items():
        system = SecondOrderSystem(wn=10.0, zeta=0.3, dt=dt)
        ctrl.reset()

        t = np.arange(0, duration, dt)
        y = []
        u_list = []

        for ti in t:
            output = system.step(ctrl.compute(target, system.x1))
            y.append(output)

        y = np.array(y)
        perf = calculate_performance(t, y, target)

        results[name] = {
            't': t,
            'y': y,
            'performance': perf
        }

    return results


def simulate_sine_tracking(controllers, dt=0.001, duration=5.0, freq=1.0, amplitude=10.0):
    """
    正弦轨迹跟踪仿真

    Args:
        controllers: 控制器字典
        dt: 采样时间
        duration: 仿真时长
        freq: 正弦频率
        amplitude: 正弦幅值

    Returns:
        results: 结果字典
    """
    results = {}

    for name, ctrl in controllers.items():
        system = SecondOrderSystem(wn=10.0, zeta=0.3, dt=dt)
        ctrl.reset()

        t = np.arange(0, duration, dt)
        y = []
        ref_list = []

        for ti in t:
            ref = amplitude * np.sin(2 * np.pi * freq * ti)
            output = system.step(ctrl.compute(ref, system.x1))
            y.append(output)
            ref_list.append(ref)

        y = np.array(y)
        ref = np.array(ref_list)

        # 计算跟踪误差
        tracking_error = np.sqrt(np.mean((y - ref) ** 2))

        results[name] = {
            't': t,
            'y': y,
            'ref': ref,
            'tracking_error': tracking_error
        }

    return results


def simulate_disturbance_rejection(controllers, dt=0.001, duration=3.0, target=10.0, disturbance_time=1.0, disturbance_magnitude=5.0):
    """
    阶跃扰动抑制仿真

    Args:
        controllers: 控制器字典
        dt: 采样时间
        duration: 仿真时长
        target: 目标值
        disturbance_time: 扰动施加时间
        disturbance_magnitude: 扰动幅度

    Returns:
        results: 结果字典
    """
    results = {}

    for name, ctrl in controllers.items():
        system = SecondOrderSystem(wn=10.0, zeta=0.3, dt=dt)
        ctrl.reset()

        t = np.arange(0, duration, dt)
        y = []
        disturbance_list = []

        for ti in t:
            if ti >= disturbance_time:
                dist = disturbance_magnitude
            else:
                dist = 0.0

            output = system.step(ctrl.compute(target, system.x1), dist)
            y.append(output)
            disturbance_list.append(dist)

        y = np.array(y)
        disturbance = np.array(disturbance_list)

        # 计算扰动恢复时间
        post_disturbance_idx = np.where(t >= disturbance_time)[0]
        if len(post_disturbance_idx) > 0:
            post_dist_y = y[post_disturbance_idx]
            error_band = 0.02 * target
            settled_idx = np.where(np.abs(post_dist_y - target) <= error_band)[0]
            if len(settled_idx) > 0:
                recovery_time = t[post_disturbance_idx[settled_idx[0]]] - disturbance_time
            else:
                recovery_time = duration - disturbance_time
        else:
            recovery_time = 0.0

        results[name] = {
            't': t,
            'y': y,
            'disturbance': disturbance,
            'recovery_time': recovery_time
        }

    return results


def simulate_robustness(controllers, dt=0.001, duration=2.0, target=10.0, parameter_variations=[0.5, 1.0, 1.5]):
    """
    参数摄动鲁棒性仿真

    Args:
        controllers: 控制器字典
        dt: 采样时间
        duration: 仿真时长
        target: 目标值
        parameter_variations: 参数变化倍数列表

    Returns:
        results: 结果字典
    """
    results = {}

    for name, ctrl in controllers.items():
        results[name] = {}

        for var in parameter_variations:
            # 修改系统参数
            system = SecondOrderSystem(wn=10.0 * var, zeta=0.3, dt=dt)
            ctrl.reset()

            t = np.arange(0, duration, dt)
            y = []

            for ti in t:
                output = system.step(ctrl.compute(target, system.x1))
                y.append(output)

            y = np.array(y)
            perf = calculate_performance(t, y, target)

            results[name][var] = {
                't': t,
                'y': y,
                'performance': perf
            }

    return results


# ==================== 可视化函数 ====================

def plot_step_response(results, save_path='step_response.png'):
    """绘制阶跃响应对比图"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 10))

    # 响应曲线
    ax1 = axes[0]
    for name, data in results.items():
        ax1.plot(data['t'], data['y'], label=name, linewidth=1.5)
    ax1.axhline(y=10, color='k', linestyle='--', label='Target', alpha=0.5)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Output')
    ax1.set_title('Step Response Comparison')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 性能指标对比
    ax2 = axes[1]
    algorithms = list(results.keys())
    metrics = ['overshoot', 'settling_time', 'rise_time']
    metric_names = ['Overshoot (%)', 'Settling Time (s)', 'Rise Time (s)']

    x = np.arange(len(algorithms))
    width = 0.25

    for i, (metric, metric_name) in enumerate(zip(metrics, metric_names)):
        values = [results[algo]['performance'][metric] for algo in algorithms]
        ax2.bar(x + i * width, values, width, label=metric_name)

    ax2.set_xlabel('Algorithm')
    ax2.set_ylabel('Value')
    ax2.set_title('Performance Metrics Comparison')
    ax2.set_xticks(x + width)
    ax2.set_xticklabels(algorithms, rotation=45, ha='right')
    ax2.legend()
    ax2.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✓ 阶跃响应对比图已保存: {save_path}")


def plot_sine_tracking(results, save_path='sine_tracking.png'):
    """绘制正弦跟踪对比图"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 10))

    # 跟踪曲线
    ax1 = axes[0]
    for name, data in results.items():
        ax1.plot(data['t'], data['y'], label=name, linewidth=1.5)
    ax1.plot(results[list(results.keys())[0]]['t'],
             results[list(results.keys())[0]]['ref'],
             'k--', label='Reference', alpha=0.5)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Output')
    ax1.set_title('Sine Tracking Comparison')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 跟踪误差对比
    ax2 = axes[1]
    algorithms = list(results.keys())
    tracking_errors = [results[algo]['tracking_error'] for algo in algorithms]

    bars = ax2.bar(algorithms, tracking_errors, color='steelblue')
    ax2.set_xlabel('Algorithm')
    ax2.set_ylabel('RMSE')
    ax2.set_title('Tracking Error Comparison')
    ax2.grid(True, alpha=0.3, axis='y')

    # 在柱状图上显示数值
    for bar, val in zip(bars, tracking_errors):
        ax2.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.1,
                f'{val:.3f}', ha='center', va='bottom')

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✓ 正弦跟踪对比图已保存: {save_path}")


def plot_disturbance_rejection(results, save_path='disturbance_rejection.png'):
    """绘制扰动抑制对比图"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 10))

    # 响应曲线
    ax1 = axes[0]
    for name, data in results.items():
        ax1.plot(data['t'], data['y'], label=name, linewidth=1.5)
    ax1.axhline(y=10, color='k', linestyle='--', label='Target', alpha=0.5)
    ax1.axvline(x=1.0, color='r', linestyle='--', label='Disturbance', alpha=0.5)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Output')
    ax1.set_title('Disturbance Rejection Comparison')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 恢复时间对比
    ax2 = axes[1]
    algorithms = list(results.keys())
    recovery_times = [results[algo]['recovery_time'] for algo in algorithms]

    bars = ax2.bar(algorithms, recovery_times, color='coral')
    ax2.set_xlabel('Algorithm')
    ax2.set_ylabel('Recovery Time (s)')
    ax2.set_title('Disturbance Recovery Time Comparison')
    ax2.grid(True, alpha=0.3, axis='y')

    for bar, val in zip(bars, recovery_times):
        ax2.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.01,
                f'{val:.3f}', ha='center', va='bottom')

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✓ 扰动抑制对比图已保存: {save_path}")


def plot_robustness(results, save_path='robustness.png'):
    """绘制鲁棒性对比图"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 10))

    # 不同参数下的响应曲线
    ax1 = axes[0]
    variations = list(results[list(results.keys())[0]].keys())
    algorithms = list(results.keys())

    for algo in algorithms:
        for var in variations:
            label = f'{algo} (wn×{var})'
            ax1.plot(results[algo][var]['t'],
                    results[algo][var]['y'],
                    label=label, linewidth=1.2, alpha=0.7)
    ax1.axhline(y=10, color='k', linestyle='--', label='Target', alpha=0.5)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Output')
    ax1.set_title('Robustness to Parameter Variation')
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
    ax1.grid(True, alpha=0.3)

    # 超调量对比
    ax2 = axes[1]
    x = np.arange(len(algorithms))
    width = 0.25

    for i, var in enumerate(variations):
        overshoots = [results[algo][var]['performance']['overshoot'] for algo in algorithms]
        ax2.bar(x + i * width, overshoots, width, label=f'wn×{var}')

    ax2.set_xlabel('Algorithm')
    ax2.set_ylabel('Overshoot (%)')
    ax2.set_title('Overshoot Under Parameter Variation')
    ax2.set_xticks(x + width)
    ax2.set_xticklabels(algorithms, rotation=45, ha='right')
    ax2.legend()
    ax2.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✓ 鲁棒性对比图已保存: {save_path}")


def print_performance_table(step_results, sine_results, disturbance_results):
    """打印性能指标对比表"""
    print("\n" + "="*80)
    print("控制算法性能对比表")
    print("="*80)

    # 阶跃响应
    print("\n【阶跃响应性能指标】")
    print(f"{'算法':<20} {'超调量(%)':<12} {'调节时间(s)':<14} {'上升时间(s)':<14} {'稳态误差':<12}")
    print("-"*80)

    for name, data in step_results.items():
        perf = data['performance']
        print(f"{name:<20} {perf['overshoot']:<12.2f} {perf['settling_time']:<14.4f} "
              f"{perf['rise_time']:<14.4f} {perf['steady_state_error']:<12.4f}")

    # 正弦跟踪
    print("\n【正弦跟踪性能指标】")
    print(f"{'算法':<20} {'跟踪误差(RMSE)':<15}")
    print("-"*50)

    for name, data in sine_results.items():
        print(f"{name:<20} {data['tracking_error']:<15.4f}")

    # 扰动抑制
    print("\n【扰动抑制性能指标】")
    print(f"{'算法':<20} {'恢复时间(s)':<15}")
    print("-"*50)

    for name, data in disturbance_results.items():
        print(f"{name:<20} {data['recovery_time']:<15.4f}")

    print("\n" + "="*80)


# ==================== 主程序 ====================

def main():
    """主函数"""
    print("="*80)
    print("电赛控制算法仿真对比工具")
    print("="*80)

    # 定义控制算法
    controllers = {
        'PID': PIDController(Kp=1.0, Ki=0.5, Kd=0.1),
        'LADRC': LADRCController(wc=10.0, wo=30.0),
        'LQR': LQRController(
            A=np.array([[0, 1], [-100, -6]]),
            B=np.array([[0], [100]]),
            Q=np.diag([10, 1]),
            R=np.array([[1]])
        ),
        'SMC': SMController(lambda_smc=10.0, epsilon=0.5, k=2.0),
        'SuperTwisting': SuperTwistingSMC(lambda_st=5.0, alpha=2.0, beta=3.0),
        'FuzzyPID': FuzzyPIDController(Kp=1.0, Ki=0.5, Kd=0.1),
    }

    print(f"\n测试算法: {', '.join(controllers.keys())}")
    print("系统模型: 二阶振荡系统 (ωn=10, ζ=0.3)")
    print("采样时间: 0.001s")

    # 1. 阶跃响应
    print("\n[1/4] 执行阶跃响应仿真...")
    step_results = simulate_step_response(controllers)

    # 2. 正弦跟踪
    print("[2/4] 执行正弦跟踪仿真...")
    sine_results = simulate_sine_tracking(controllers)

    # 3. 扰动抑制
    print("[3/4] 执行扰动抑制仿真...")
    disturbance_results = simulate_disturbance_rejection(controllers)

    # 4. 鲁棒性
    print("[4/4] 执行鲁棒性仿真...")
    robustness_results = simulate_robustness(controllers)

    # 打印性能对比表
    print_performance_table(step_results, sine_results, disturbance_results)

    # 生成可视化图表
    print("\n生成可视化图表...")
    plot_step_response(step_results, 'step_response.png')
    plot_sine_tracking(sine_results, 'sine_tracking.png')
    plot_disturbance_rejection(disturbance_results, 'disturbance_rejection.png')
    plot_robustness(robustness_results, 'robustness.png')

    print("\n" + "="*80)
    print("仿真完成！")
    print("输出文件:")
    print("  - step_response.png (阶跃响应对比)")
    print("  - sine_tracking.png (正弦跟踪对比)")
    print("  - disturbance_rejection.png (扰动抑制对比)")
    print("  - robustness.png (鲁棒性对比)")
    print("="*80)


if __name__ == "__main__":
    main()
