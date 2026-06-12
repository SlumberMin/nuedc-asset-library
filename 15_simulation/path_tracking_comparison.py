"""
路径跟踪算法对比仿真
方法：Pure Pursuit + Stanley + 简化MPC
应用：自动驾驶/机器人路径跟踪控制
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

# ============================================================
# 1. 参考路径生成（S形曲线）
# ============================================================
def generate_reference_path():
    """生成参考路径（S形弯道）"""
    t = np.linspace(0, 2*np.pi, 500)
    x = np.linspace(0, 50, 500)
    y = 5 * np.sin(0.15 * x) + 0.5 * np.sin(0.3 * x)
    
    # 计算路径切线角度
    dx = np.gradient(x)
    dy = np.gradient(y)
    yaw = np.arctan2(dy, dx)
    
    # 计算曲率
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
    
    return x, y, yaw, curvature

ref_x, ref_y, ref_yaw, ref_curvature = generate_reference_path()

# ============================================================
# 2. 车辆运动学模型（自行车模型）
# ============================================================
class BicycleModel:
    """自行车模型"""
    def __init__(self, x=0, y=-2, yaw=0, v=2.0, L=2.5):
        self.x = x          # 位置x
        self.y = y          # 位置y
        self.yaw = yaw      # 航向角
        self.v = v          # 速度(m/s)
        self.L = L          # 轴距(m)
        self.trajectory = [(x, y, yaw)]
    
    def update(self, delta, dt=0.05):
        """更新车辆状态 delta为前轮转角"""
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / self.L * np.tan(delta) * dt
        self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))  # 归一化
        self.trajectory.append((self.x, self.y, self.yaw))
    
    def get_trajectory(self):
        traj = np.array(self.trajectory)
        return traj[:, 0], traj[:, 1], traj[:, 2]

# ============================================================
# 3. Pure Pursuit算法
# ============================================================
def pure_pursuit(vehicle, ref_x, ref_y, look_ahead=5.0):
    """
    Pure Pursuit路径跟踪
    计算使车辆跟踪前方look_ahead距离的目标点所需的转向角
    """
    # 找最近点
    dx = ref_x - vehicle.x
    dy = ref_y - vehicle.y
    dist = np.sqrt(dx**2 + dy**2)
    
    # 找到前瞻距离处的目标点
    target_idx = None
    for i in np.argsort(dist):
        if dist[i] >= look_ahead:
            target_idx = i
            break
    
    if target_idx is None:
        target_idx = np.argmin(dist)
    
    # 目标点在车辆坐标系中的位置
    target_x = ref_x[target_idx] - vehicle.x
    target_y = ref_y[target_idx] - vehicle.y
    
    # 转换到车辆坐标系
    local_x = target_x * np.cos(vehicle.yaw) + target_y * np.sin(vehicle.yaw)
    local_y = -target_x * np.sin(vehicle.yaw) + target_y * np.cos(vehicle.yaw)
    
    # 计算转向角
    ld = np.sqrt(local_x**2 + local_y**2)
    if ld < 0.1:
        return 0.0, target_idx
    
    curvature_cmd = 2 * local_y / (ld**2)
    delta = np.arctan(curvature_cmd * vehicle.L)
    delta = np.clip(delta, -np.radians(30), np.radians(30))
    
    return delta, target_idx

# ============================================================
# 4. Stanley算法
# ============================================================
def stanley(vehicle, ref_x, ref_y, ref_yaw, k_e=2.0):
    """
    Stanley路径跟踪
    结合航向误差和横向偏差
    """
    # 找最近点
    dx = ref_x - vehicle.x
    dy = ref_y - vehicle.y
    dist = np.sqrt(dx**2 + dy**2)
    nearest_idx = np.argmin(dist)
    
    # 横向偏差（带符号）
    front_axle_x = vehicle.x + vehicle.L/2 * np.cos(vehicle.yaw)
    front_axle_y = vehicle.y + vehicle.L/2 * np.sin(vehicle.yaw)
    
    dx_front = ref_x - front_axle_x
    dy_front = ref_y - front_axle_y
    
    # 横向偏差（叉积判断左右）
    cross = dx_front * np.sin(ref_yaw) - dy_front * np.cos(ref_yaw)
    e_y = -cross[nearest_idx]
    
    # 航向误差
    theta_e = ref_yaw[nearest_idx] - vehicle.yaw
    theta_e = np.arctan2(np.sin(theta_e), np.cos(theta_e))
    
    # Stanley公式
    if vehicle.v < 0.1:
        delta = theta_e
    else:
        delta = theta_e + np.arctan(k_e * e_y / vehicle.v)
    
    delta = np.clip(delta, -np.radians(30), np.radians(30))
    return delta, nearest_idx, e_y

# ============================================================
# 5. 简化MPC（模型预测控制）
# ============================================================
def simple_mpc(vehicle, ref_x, ref_y, ref_yaw, horizon=15, k_path=1.0, k_yaw=2.0, k_smooth=0.5):
    """
    简化MPC：基于自行车模型预测+优化
    使用网格搜索代替完整QP求解（简化实现）
    """
    dt_mpc = 0.05
    v = vehicle.v
    L = vehicle.L
    
    best_delta = 0.0
    best_cost = float('inf')
    
    # 找最近参考点
    dx = ref_x - vehicle.x
    dy = ref_y - vehicle.y
    dist = np.sqrt(dx**2 + dy**2)
    nearest_idx = np.argmin(dist)
    
    # 搜索最优转向角（简化：只搜索单步）
    delta_candidates = np.linspace(-np.radians(25), np.radians(25), 21)
    
    for delta in delta_candidates:
        # 前向模拟
        x, y, yaw = vehicle.x, vehicle.y, vehicle.yaw
        total_cost = 0.0
        
        for step in range(horizon):
            # 预测一步
            x += v * np.cos(yaw) * dt_mpc
            y += v * np.sin(yaw) * dt_mpc
            yaw += v / L * np.tan(delta) * dt_mpc
            
            # 对应的参考点
            ref_idx = min(nearest_idx + step * 2, len(ref_x) - 1)
            
            # 路径跟踪误差
            path_err = (x - ref_x[ref_idx])**2 + (y - ref_y[ref_idx])**2
            yaw_err = (yaw - ref_yaw[ref_idx])**2
            
            total_cost += k_path * path_err + k_yaw * yaw_err
        
        total_cost += k_smooth * delta**2  # 平滑惩罚
        
        if total_cost < best_cost:
            best_cost = total_cost
            best_delta = delta
    
    return best_delta, nearest_idx

# ============================================================
# 6. 仿真运行
# ============================================================
def run_simulation(controller_func, controller_name, **kwargs):
    """运行仿真"""
    vehicle = BicycleModel(x=0, y=-2, yaw=0.3, v=3.0)
    n_steps = 1000
    errors = []
    
    for step in range(n_steps):
        if controller_name == 'Pure Pursuit':
            delta, target_idx = controller_func(vehicle, ref_x, ref_y, **kwargs)
        elif controller_name == 'Stanley':
            delta, nearest_idx, e_y = controller_func(vehicle, ref_x, ref_y, ref_yaw, **kwargs)
            errors.append(abs(e_y))
        elif controller_name == 'MPC':
            delta, nearest_idx = controller_func(vehicle, ref_x, ref_y, ref_yaw, **kwargs)
        
        vehicle.update(delta, dt=0.05)
        
        # 计算跟踪误差
        if controller_name != 'Stanley':
            dx = ref_x - vehicle.x
            dy = ref_y - vehicle.y
            errors.append(np.min(np.sqrt(dx**2 + dy**2)))
    
    return vehicle, errors

print("=" * 60)
print("路径跟踪算法对比仿真")
print("=" * 60)

# 运行三种算法
veh_pp, err_pp = run_simulation(pure_pursuit, 'Pure Pursuit', look_ahead=4.0)
print(f"[Pure Pursuit] 平均误差: {np.mean(err_pp):.3f}m, 最大误差: {np.max(err_pp):.3f}m")

veh_st, err_st = run_simulation(stanley, 'Stanley', k_e=2.5)
print(f"[Stanley]      平均误差: {np.mean(err_st):.3f}m, 最大误差: {np.max(err_st):.3f}m")

veh_mpc, err_mpc = run_simulation(simple_mpc, 'MPC')
print(f"[MPC]          平均误差: {np.mean(err_mpc):.3f}m, 最大误差: {np.max(err_mpc):.3f}m")

# ============================================================
# 7. 绘图
# ============================================================
fig, axes = plt.subplots(2, 2, figsize=(16, 12))

# 图1：路径跟踪对比
ax = axes[0, 0]
ax.plot(ref_x, ref_y, 'k--', linewidth=2, label='参考路径')
traj_pp = veh_pp.get_trajectory()
traj_st = veh_st.get_trajectory()
traj_mpc = veh_mpc.get_trajectory()
ax.plot(traj_pp[0], traj_pp[1], 'b-', linewidth=1.5, label='Pure Pursuit')
ax.plot(traj_st[0], traj_st[1], 'r-', linewidth=1.5, label='Stanley')
ax.plot(traj_mpc[0], traj_mpc[1], 'g-', linewidth=1.5, label='MPC')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('路径跟踪轨迹对比')
ax.legend()
ax.grid(True, alpha=0.3)
ax.set_aspect('equal')

# 图2：跟踪误差随时间变化
ax = axes[0, 1]
t = np.arange(len(err_pp)) * 0.05
ax.plot(t, err_pp, 'b-', linewidth=1, label=f'Pure Pursuit (avg={np.mean(err_pp):.2f}m)')
ax.plot(t, err_st, 'r-', linewidth=1, label=f'Stanley (avg={np.mean(err_st):.2f}m)')
ax.plot(t, err_mpc, 'g-', linewidth=1, label=f'MPC (avg={np.mean(err_mpc):.2f}m)')
ax.set_xlabel('时间 (s)')
ax.set_ylabel('跟踪误差 (m)')
ax.set_title('跟踪误差对比')
ax.legend()
ax.grid(True, alpha=0.3)

# 图3：航向角对比
ax = axes[1, 0]
t_pp = np.arange(len(traj_pp[2])) * 0.05
t_st = np.arange(len(traj_st[2])) * 0.05
t_mpc = np.arange(len(traj_mpc[2])) * 0.05
ref_yaw_interp_pp = np.interp(t_pp, np.linspace(0, t_pp[-1], len(ref_yaw)), ref_yaw)
ax.plot(t_pp, np.degrees(traj_pp[2]), 'b-', label='Pure Pursuit')
ax.plot(t_st, np.degrees(traj_st[2]), 'r-', label='Stanley')
ax.plot(t_mpc, np.degrees(traj_mpc[2]), 'g-', label='MPC')
ax.plot(t_pp, np.degrees(ref_yaw_interp_pp), 'k--', alpha=0.5, label='参考航向')
ax.set_xlabel('时间 (s)')
ax.set_ylabel('航向角 (°)')
ax.set_title('航向角对比')
ax.legend()
ax.grid(True, alpha=0.3)

# 图4：误差统计
ax = axes[1, 1]
names = ['Pure\nPursuit', 'Stanley', 'MPC']
avg_errors = [np.mean(err_pp), np.mean(err_st), np.mean(err_mpc)]
max_errors = [np.max(err_pp), np.max(err_st), np.max(err_mpc)]

x_pos = np.arange(len(names))
width = 0.35
bars1 = ax.bar(x_pos - width/2, avg_errors, width, label='平均误差', color='steelblue')
bars2 = ax.bar(x_pos + width/2, max_errors, width, label='最大误差', color='coral')

for bar, val in zip(bars1, avg_errors):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
            f'{val:.3f}', ha='center', va='bottom', fontsize=10)
for bar, val in zip(bars2, max_errors):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
            f'{val:.3f}', ha='center', va='bottom', fontsize=10)

ax.set_xticks(x_pos)
ax.set_xticklabels(names)
ax.set_ylabel('误差 (m)')
ax.set_title('跟踪误差统计')
ax.legend()
ax.grid(True, alpha=0.3, axis='y')

plt.suptitle('路径跟踪算法对比仿真（Pure Pursuit + Stanley + MPC）', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'path_tracking_comparison.png'), dpi=150, bbox_inches='tight')
print("\n图表已保存: path_tracking_comparison.png")
plt.close('all')

if __name__ == '__main__':
    run_simulation()
