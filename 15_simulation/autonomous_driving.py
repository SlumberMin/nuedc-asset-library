"""
自动驾驶仿真 - 感知 + 规划 + 控制 + 交通流
==============================================
仿真自动驾驶全栈功能：
1. 感知层：传感器模拟（激光雷达/摄像头/雷达）、目标检测、跟踪
2. 规划层：路径规划（A*）、行为决策、轨迹优化
3. 控制层：纵向控制（速度PID）、横向控制（Stanley/Pure Pursuit）
4. 交通流：多车交互、跟车模型（IDM）、换道模型（MOBIL）
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from collections import deque
import heapq
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


# ═══════════════════════════════════════════════
# 1. 道路与地图
# ═══════════════════════════════════════════════
class Road:
    """道路基础设施"""
    def __init__(self, length=200, lanes=3, lane_width=3.5, speed_limit=30):
        self.length = length
        self.lanes = lanes
        self.lane_width = lane_width
        self.speed_limit = speed_limit  # m/s
        self.width = lanes * lane_width

    def get_lane_center(self, lane):
        return (lane + 0.5) * self.lane_width


class GridMap:
    """网格地图（用于路径规划）"""
    def __init__(self, size=50, resolution=1.0):
        self.size = size
        self.res = resolution
        self.grid = np.zeros((size, size), dtype=int)
        self._generate_obstacles()

    def _generate_obstacles(self):
        """生成随机障碍物"""
        np.random.seed(42)
        # 墙壁
        self.grid[15:35, 20:22] = 1
        self.grid[10:12, 10:40] = 1
        self.grid[30:45, 30:32] = 1
        # 随机障碍
        for _ in range(20):
            x, y = np.random.randint(0, self.size, 2)
            self.grid[x, y] = 1
            self.grid[x, min(y+1, self.size-1)] = 1

    def is_valid(self, x, y):
        if 0 <= x < self.size and 0 <= y < self.size:
            return self.grid[x, y] == 0
        return False


# ═══════════════════════════════════════════════
# 2. 车辆模型
# ═══════════════════════════════════════════════
class Vehicle:
    """单车动力学模型（自行车模型）"""
    def __init__(self, x=0, y=0, yaw=0, v=0, L=2.7):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.L = L          # 轴距
        self.width = 1.8
        self.length = 4.5
        self.steer = 0
        self.accel = 0
        self.max_steer = np.radians(30)
        self.max_accel = 3.0
        self.max_decel = -8.0

    def update(self, accel, steer, dt):
        self.steer = np.clip(steer, -self.max_steer, self.max_steer)
        self.accel = np.clip(accel, self.max_decel, self.max_accel)
        self.v = max(self.v + self.accel * dt, 0)
        self.yaw += self.v / self.L * np.tan(self.steer) * dt
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt


# ═══════════════════════════════════════════════
# 3. 感知模块
# ═══════════════════════════════════════════════
class LidarSensor:
    """激光雷达模拟"""
    def __init__(self, range_max=80, angular_res=1.0, noise_std=0.1):
        self.range_max = range_max
        self.angular_res = angular_res
        self.noise_std = noise_std

    def scan(self, ego, obstacles):
        """返回 [(angle, range, intensity)]"""
        angles = np.arange(0, 360, self.angular_res)
        ranges = np.full_like(angles, self.range_max, dtype=float)
        for obs in obstacles:
            dx = obs['x'] - ego.x
            dy = obs['y'] - ego.y
            dist = np.sqrt(dx**2 + dy**2)
            angle = np.degrees(np.arctan2(dy, dx)) % 360
            idx = int(angle / self.angular_res) % len(angles)
            if dist < self.range_max:
                ranges[idx] = min(ranges[idx], dist + np.random.normal(0, self.noise_std))
        return angles, ranges


class ObjectDetector:
    """目标检测与跟踪"""
    def __init__(self, detection_range=50):
        self.range = detection_range
        self.tracked_objects = {}

    def detect(self, ego, all_vehicles):
        detected = []
        for v in all_vehicles:
            if v is ego:
                continue
            dx = v.x - ego.x
            dy = v.y - ego.y
            dist = np.sqrt(dx**2 + dy**2)
            if dist < self.range:
                detected.append({
                    'x': v.x, 'y': v.y, 'v': v.v, 'yaw': v.yaw,
                    'dist': dist, 'id': id(v)
                })
        return detected


# ═══════════════════════════════════════════════
# 4. 路径规划 (A*)
# ═══════════════════════════════════════════════
def astar(grid_map, start, goal):
    """A*路径规划"""
    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for dx, dy in neighbors:
            neighbor = (current[0]+dx, current[1]+dy)
            if not grid_map.is_valid(neighbor[0], neighbor[1]):
                continue
            cost = g_score[current] + (1.414 if dx and dy else 1.0)
            if cost < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = cost
                f = cost + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))
                came_from[neighbor] = current
    return []


# ═══════════════════════════════════════════════
# 5. 行为决策 (有限状态机)
# ═══════════════════════════════════════════════
class BehaviorPlanner:
    """行为规划有限状态机"""
    STATES = ['LANE_KEEP', 'FOLLOW', 'LANE_CHANGE_LEFT', 'LANE_CHANGE_RIGHT', 'EMERGENCY_BRAKE']

    def __init__(self, safe_dist=15.0, emergency_dist=5.0):
        self.state = 'LANE_KEEP'
        self.safe_dist = safe_dist
        self.emergency_dist = emergency_dist

    def decide(self, ego, detected_objects):
        """基于感知信息做行为决策"""
        # 找前方最近车辆
        front_dist = float('inf')
        front_v = 0
        for obj in detected_objects:
            # 简化：只看距离
            if obj['dist'] < front_dist and obj['x'] > ego.x - 2:
                front_dist = obj['dist']
                front_v = obj['v']

        if front_dist < self.emergency_dist:
            self.state = 'EMERGENCY_BRAKE'
        elif front_dist < self.safe_dist:
            if front_v < ego.v - 2:
                self.state = 'LANE_CHANGE_LEFT'
            else:
                self.state = 'FOLLOW'
        else:
            self.state = 'LANE_KEEP'

        return self.state, front_dist, front_v


# ═══════════════════════════════════════════════
# 6. 纵向+横向控制
# ═══════════════════════════════════════════════
class LongitudinalController:
    """纵向速度PID控制"""
    def __init__(self, Kp=1.0, Ki=0.1, Kd=0.05, dt=0.1):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.integral = 0
        self.prev_err = 0

    def compute(self, v_target, v_actual):
        err = v_target - v_actual
        self.integral += err * self.dt
        deriv = (err - self.prev_err) / self.dt
        self.prev_err = err
        return self.Kp * err + self.Ki * self.integral + self.Kd * deriv


class LateralController:
    """横向控制 - Stanley + Pure Pursuit混合"""
    def __init__(self, k_stanley=2.5, L=2.7):
        self.k = k_stanley
        self.L = L

    def stanley(self, ego, path_x, path_y, path_yaw):
        """Stanley横向控制"""
        # 找最近路径点
        dists = np.sqrt((path_x - ego.x)**2 + (path_y - ego.y)**2)
        idx = np.argmin(dists)
        # 横向误差
        dx = path_x[idx] - ego.x
        dy = path_y[idx] - ego.y
        cross_track_err = -dx * np.sin(ego.yaw) + dy * np.cos(ego.yaw)
        # 航向误差
        heading_err = path_yaw[idx] - ego.yaw
        heading_err = np.arctan2(np.sin(heading_err), np.cos(heading_err))
        # Stanley公式
        if ego.v > 0.5:
            steer = heading_err + np.arctan2(self.k * cross_track_err, ego.v)
        else:
            steer = heading_err
        return np.clip(steer, -np.radians(30), np.radians(30))


# ═══════════════════════════════════════════════
# 7. 跟车模型 (IDM)
# ═══════════════════════════════════════════════
class IDMModel:
    """智能驾驶员模型 (Intelligent Driver Model)"""
    def __init__(self, v0=30.0, T=1.5, a=1.5, b=2.0, s0=2.0):
        self.v0 = v0      # 期望速度
        self.T = T          # 安全车头时距
        self.a = a          # 最大加速度
        self.b = b          # 舒适减速度
        self.s0 = s0        # 最小间距

    def compute_accel(self, v, delta_v, s):
        """v: 自车速度, delta_v: 速度差(前-自), s: 间距"""
        s_star = self.s0 + max(0, v * self.T + v * delta_v / (2 * np.sqrt(self.a * self.b)))
        accel = self.a * (1 - (v / max(self.v0, 0.1))**4 - (s_star / max(s, 0.1))**2)
        return np.clip(accel, -self.b * 3, self.a)


# ═══════════════════════════════════════════════
# 8. 交通流仿真
# ═══════════════════════════════════════════════
class TrafficSimulation:
    """多车交通流仿真"""
    def __init__(self, road, n_vehicles=15, dt=0.1):
        self.road = road
        self.dt = dt
        self.vehicles = []
        self.idm = IDMModel()
        self._init_vehicles(n_vehicles)

    def _init_vehicles(self, n):
        np.random.seed(123)
        for i in range(n):
            lane = np.random.randint(0, self.road.lanes)
            x = np.random.uniform(10, self.road.length - 20)
            y = self.road.get_lane_center(lane)
            v = np.random.uniform(15, 28)
            v_obj = Vehicle(x=x, y=y, yaw=0, v=v)
            v_obj.lane = lane
            self.vehicles.append(v_obj)

    def step(self):
        """所有车辆一步仿真"""
        # 按车道和位置排序
        for lane in range(self.road.lanes):
            lane_vehicles = sorted(
                [v for v in self.vehicles if hasattr(v, 'lane') and v.lane == lane],
                key=lambda v: v.x
            )
            for i, v in enumerate(lane_vehicles):
                # 前车距离
                if i < len(lane_vehicles) - 1:
                    leader = lane_vehicles[i + 1]
                    s = leader.x - v.x - v.length
                    dv = v.v - leader.v
                else:
                    s = 200
                    dv = 0
                accel = self.idm.compute_accel(v.v, dv, s)
                # 车道保持
                target_y = self.road.get_lane_center(v.lane)
                steer = 0.3 * (target_y - v.y)
                v.update(accel, steer, self.dt)

        return self.vehicles


# ═══════════════════════════════════════════════
# 9. 仿真主函数
# ═══════════════════════════════════════════════
def run_autonomous_driving(duration=60, dt=0.1):
    """完整自动驾驶仿真"""
    road = Road(length=200, lanes=3, lane_width=3.5, speed_limit=30)
    traffic = TrafficSimulation(road, n_vehicles=12, dt=dt)

    # 自车
    ego = Vehicle(x=10, y=road.get_lane_center(1), yaw=0, v=20)

    # 模块实例
    lidar = LidarSensor(range_max=60)
    detector = ObjectDetector(detection_range=50)
    planner = BehaviorPlanner(safe_dist=20, emergency_dist=6)
    lon_ctrl = LongitudinalController(Kp=1.5, Ki=0.2, Kd=0.05, dt=dt)
    lat_ctrl = LateralController(k_stanley=3.0, L=2.7)

    # 参考路径
    path_x = np.linspace(0, road.length, 500)
    path_y = np.full(500, road.get_lane_center(1))
    path_yaw = np.zeros(500)

    steps = int(duration / dt)
    t_arr = np.arange(steps) * dt
    ego_log = np.zeros((steps, 5))  # x, y, v, yaw, steer
    state_log = []

    for i in range(steps):
        # 感知
        detected = detector.detect(ego, traffic.vehicles)
        angles, ranges = lidar.scan(ego, [{'x': v.x, 'y': v.y} for v in traffic.vehicles])

        # 行为决策
        state, front_dist, front_v = planner.decide(ego, detected)
        state_log.append(state)

        # 目标速度
        if state == 'EMERGENCY_BRAKE':
            v_target = 0
        elif state == 'FOLLOW':
            v_target = min(front_v, road.speed_limit)
        else:
            v_target = road.speed_limit

        # 纵向控制
        accel = lon_ctrl.compute(v_target, ego.v)

        # 横向控制
        steer = lat_ctrl.stanley(ego, path_x, path_y, path_yaw)

        # 更新
        ego.update(accel, steer, dt)
        traffic.step()

        ego_log[i] = [ego.x, ego.y, ego.v, np.degrees(ego.yaw), np.degrees(ego.steer)]

    print(f"[自动驾驶] 行驶距离: {ego.x:.1f} m")
    print(f"[自动驾驶] 最终速度: {ego.v:.1f} m/s ({ego.v*3.6:.1f} km/h)")
    print(f"[自动驾驶] 平均速度: {np.mean(ego_log[:,2]):.1f} m/s")
    state_counts = {}
    for s in state_log:
        state_counts[s] = state_counts.get(s, 0) + 1
    print(f"[自动驾驶] 状态分布: {state_counts}")

    # 绘图
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle('自动驾驶仿真 - 感知/规划/控制/交通流', fontsize=14)

    # 俯视图
    ax = axes[0, 0]
    ax.set_xlim(0, min(road.length, 150))
    ax.set_ylim(-1, road.width + 1)
    for l in range(road.lanes + 1):
        ax.axhline(l * road.lane_width, color='gray', linestyle='--', linewidth=0.5)
    ax.plot(ego_log[:, 0], ego_log[:, 1], 'r-', linewidth=2, label='自车轨迹')
    # 其他车辆最终位置
    for v in traffic.vehicles:
        ax.plot(v.x, v.y, 's', color='blue', markersize=4)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('俯视图')
    ax.legend()
    ax.set_aspect('equal')

    # 速度
    axes[0, 1].plot(t_arr, ego_log[:, 2] * 3.6, 'b-', label='自车速度')
    axes[0, 1].axhline(road.speed_limit * 3.6, color='r', linestyle='--', label='限速')
    axes[0, 1].set_ylabel('速度 (km/h)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # 横向位置
    axes[1, 0].plot(t_arr, ego_log[:, 1], 'g-')
    for l in range(road.lanes):
        axes[1, 0].axhline((l + 0.5) * road.lane_width, color='gray', linestyle=':', alpha=0.5)
    axes[1, 0].set_ylabel('横向位置 (m)')
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].grid(True, alpha=0.3)

    # 转向角
    axes[1, 1].plot(t_arr, ego_log[:, 4], 'r-')
    axes[1, 1].set_ylabel('转向角 (°)')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].grid(True, alpha=0.3)

    # 行为状态
    state_map = {s: i for i, s in enumerate(BehaviorPlanner.STATES)}
    state_vals = [state_map.get(s, -1) for s in state_log]
    axes[2, 0].plot(t_arr, state_vals, 'k-', linewidth=0.5)
    axes[2, 0].set_yticks(list(state_map.values()))
    axes[2, 0].set_yticklabels(list(state_map.keys()), fontsize=7)
    axes[2, 0].set_xlabel('时间 (s)')
    axes[2, 0].set_title('行为决策状态')

    # 雷达扫描（最后一帧）
    ax = axes[2, 1]
    angles_final, ranges_final = lidar.scan(ego, [{'x': v.x, 'y': v.y} for v in traffic.vehicles])
    ax.plot(np.radians(angles_final), ranges_final, 'b.', markersize=1)
    ax.set_xlabel('角度 (rad)')
    ax.set_ylabel('距离 (m)')
    ax.set_title('激光雷达扫描')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/autonomous_driving.png', dpi=150)
    plt.close()
    print("[自动驾驶] 图表已保存")


def run_path_planning():
    """A*路径规划演示"""
    grid_map = GridMap(size=50)
    start, goal = (2, 2), (47, 47)
    path = astar(grid_map, start, goal)

    if path:
        print(f"[A*规划] 路径长度: {len(path)} 步")
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(grid_map.grid.T, origin='lower', cmap='binary', alpha=0.5)
        px, py = zip(*path)
        ax.plot(px, py, 'r-', linewidth=2, label='A*路径')
        ax.plot(*start, 'go', markersize=10, label='起点')
        ax.plot(*goal, 'r*', markersize=15, label='终点')
        ax.set_title('A*路径规划')
        ax.legend()
        plt.tight_layout()
        plt.savefig('./nuedc-asset-library/15_simulation/path_planning.png', dpi=150)
        plt.close()
        print("[A*规划] 图表已保存")
    else:
        print("[A*规划] 未找到路径！")


if __name__ == '__main__':
    print("=" * 60)
    print("自动驾驶仿真 - 感知 + 规划 + 控制 + 交通流")
    print("=" * 60)
    run_path_planning()
    run_autonomous_driving()
    print("\n✅ 自动驾驶仿真完成！")
