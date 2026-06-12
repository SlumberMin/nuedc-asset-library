#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
传感器阵列仿真 - 多传感器布局+覆盖+融合
==========================================
功能：
  - 传感器布局生成（线阵、圆阵、网格、随机）
  - 覆盖率分析与覆盖空洞检测
  - 多传感器数据融合算法（加权平均、卡尔曼、D-S证据理论）
  - 检测概率与虚警概率分析
  - 3D可视化

适用场景：多传感器网络设计、目标检测、环境监测
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib import cm
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import warnings
warnings.filterwarnings('ignore')

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ======================== 传感器与环境模型 ========================

@dataclass
class Sensor:
    """单个传感器模型"""
    sensor_id: int
    position: np.ndarray          # [x, y] 位置
    sensing_range: float          # 感知半径
    detection_prob: float = 0.9   # 检测概率Pd
    false_alarm_prob: float = 0.01  # 虚警概率Pfa
    noise_std: float = 0.5        # 测量噪声标准差
    sensor_type: str = "generic"
    battery: float = 100.0        # 电池电量%

    def is_in_range(self, point: np.ndarray) -> bool:
        return np.linalg.norm(point - self.position) <= self.sensing_range

    def measure(self, true_value: float) -> float:
        """带噪声的测量"""
        if np.random.random() < self.detection_prob:
            return true_value + np.random.normal(0, self.noise_std)
        return np.nan  # 漏检

    def detect_target(self, target_pos: np.ndarray) -> Tuple[bool, bool]:
        """检测目标，返回 (检测到, 是否虚警)"""
        dist = np.linalg.norm(target_pos - self.position)
        if dist <= self.sensing_range:
            detected = np.random.random() < self.detection_prob
            return detected, False
        else:
            false_alarm = np.random.random() < self.false_alarm_prob
            return false_alarm, true


@dataclass
class Target:
    """目标模型"""
    position: np.ndarray
    true_value: float = 1.0
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))


# ======================== 传感器布局生成器 ========================

class LayoutGenerator:
    """传感器布局生成器"""

    @staticmethod
    def linear_array(n: int, spacing: float, y: float = 0) -> np.ndarray:
        """线阵布局"""
        x = np.linspace(-(n-1)*spacing/2, (n-1)*spacing/2, n)
        return np.column_stack([x, np.full(n, y)])

    @staticmethod
    def circular_array(n: int, radius: float, center: np.ndarray = None) -> np.ndarray:
        """圆形布局"""
        if center is None:
            center = np.zeros(2)
        angles = np.linspace(0, 2*np.pi, n, endpoint=False)
        return center + radius * np.column_stack([np.cos(angles), np.sin(angles)])

    @staticmethod
    def grid_array(rows: int, cols: int, spacing: float) -> np.ndarray:
        """网格布局"""
        x = np.arange(cols) * spacing
        y = np.arange(rows) * spacing
        xx, yy = np.meshgrid(x, y)
        return np.column_stack([xx.ravel(), yy.ravel()])

    @staticmethod
    def random_array(n: int, area_size: float, seed: int = 42) -> np.ndarray:
        """随机布局"""
        rng = np.random.RandomState(seed)
        return rng.uniform(0, area_size, (n, 2))


# ======================== 覆盖分析 ========================

class CoverageAnalyzer:
    """覆盖率与覆盖质量分析"""

    def __init__(self, sensors: List[Sensor], area: Tuple[float, float]):
        self.sensors = sensors
        self.area = area  # (width, height)

    def compute_coverage_map(self, resolution: float = 0.5) -> np.ndarray:
        """计算覆盖地图（每个点被多少传感器覆盖）"""
        x = np.arange(0, self.area[0], resolution)
        y = np.arange(0, self.area[1], resolution)
        xx, yy = np.meshgrid(x, y)
        coverage = np.zeros_like(xx)

        for s in self.sensors:
            dist = np.sqrt((xx - s.position[0])**2 + (yy - s.position[1])**2)
            coverage += (dist <= s.sensing_range).astype(float)

        return xx, yy, coverage

    def coverage_ratio(self, resolution: float = 0.5) -> float:
        """计算覆盖率"""
        _, _, coverage = self.compute_coverage_map(resolution)
        return np.mean(coverage > 0)

    def find_coverage_holes(self, resolution: float = 0.5) -> List[Tuple[float, float]]:
        """查找覆盖空洞"""
        xx, yy, coverage = self.compute_coverage_map(resolution)
        holes_x = xx[coverage == 0]
        holes_y = yy[coverage == 0]
        return list(zip(holes_x, holes_y))

    def connectivity_graph(self) -> Tuple[List, List]:
        """传感器连通性图（通信范围=2倍感知范围）"""
        edges = []
        comm_range = 2.0  # 通信倍数
        for i, s1 in enumerate(self.sensors):
            for j, s2 in enumerate(self.sensors):
                if i < j:
                    dist = np.linalg.norm(s1.position - s2.position)
                    if dist <= comm_range * s1.sensing_range:
                        edges.append((i, j))
        nodes = [s.position for s in self.sensors]
        return nodes, edges


# ======================== 数据融合算法 ========================

class FusionAlgorithms:
    """多传感器数据融合算法"""

    @staticmethod
    def weighted_average(measurements: List[float], weights: List[float]) -> float:
        """加权平均融合"""
        valid = [(m, w) for m, w in zip(measurements, weights) if not np.isnan(m)]
        if not valid:
            return np.nan
        vals, ws = zip(*valid)
        return np.sum(np.array(vals) * np.array(ws)) / np.sum(ws)

    @staticmethod
    def weighted_by_inverse_variance(measurements: List[float],
                                      variances: List[float]) -> float:
        """逆方差加权融合"""
        valid = [(m, v) for m, v in zip(measurements, variances) if not np.isnan(m)]
        if not valid:
            return np.nan
        vals, vars_ = zip(*valid)
        weights = [1.0/v for v in vars_]
        return np.sum(np.array(vals)*np.array(weights)) / np.sum(weights)

    @staticmethod
    def kalman_fusion(states: List[float], covariances: List[float]) -> Tuple[float, float]:
        """多传感器卡尔曼融合（协方差交叉简化版）"""
        valid = [(s, c) for s, c in zip(states, covariances)
                 if not np.isnan(s) and c > 0]
        if not valid:
            return np.nan, np.inf
        # 信息融合
        info_sum = sum(1.0/c for _, c in valid)
        fused_cov = 1.0 / info_sum
        fused_state = fused_cov * sum(s/c for s, c in valid)
        return fused_state, fused_cov

    @staticmethod
    def ds_evidence(mass_functions: List[dict], hypotheses: list) -> dict:
        """Dempster-Shafer 证据理论融合"""
        # mass_functions: [{hyp: mass, ...}, ...]
        combined = mass_functions[0].copy()
        for mf in mass_functions[1:]:
            new_combined = {}
            for h1, m1 in combined.items():
                for h2, m2 in mf.items():
                    if h1 == h2:
                        key = h1
                    elif h1 == 'uncertainty' or h2 == 'uncertainty':
                        key = h1 if h2 == 'uncertainty' else h2
                    else:
                        key = 'conflict'
                    new_combined[key] = new_combined.get(key, 0) + m1 * m2
            # 处理冲突
            conflict = new_combined.pop('conflict', 0)
            if conflict < 1.0:
                for k in new_combined:
                    new_combined[k] /= (1.0 - conflict)
            combined = new_combined
        return combined


# ======================== 目标跟踪仿真 ========================

class TargetTracker:
    """基于传感器阵列的目标跟踪"""

    def __init__(self, sensors: List[Sensor]):
        self.sensors = sensors
        self.fusion = FusionAlgorithms()

    def estimate_position_multilateration(self, measurements: List[Tuple[float, np.ndarray]]) -> np.ndarray:
        """基于距离测量的多边定位"""
        # measurements: [(distance, sensor_position), ...]
        valid = [(d, p) for d, p in measurements if not np.isnan(d) and d > 0]
        if len(valid) < 2:
            return np.array([np.nan, np.nan])
        # 最小二乘法
        n = len(valid)
        A = np.zeros((n-1, 2))
        b = np.zeros(n-1)
        d0, p0 = valid[0]
        for i in range(1, n):
            di, pi = valid[i]
            A[i-1] = 2 * (pi - p0)
            b[i-1] = d0**2 - di**2 + np.dot(pi, pi) - np.dot(p0, p0)
        try:
            pos = np.linalg.lstsq(A, b, rcond=None)[0]
        except np.linalg.LinAlgError:
            pos = np.array([np.nan, np.nan])
        return pos

    def track_target_over_time(self, target_trajectory: np.ndarray,
                                time_steps: int) -> Tuple[np.ndarray, np.ndarray]:
        """跟踪运动目标"""
        estimates = []
        uncertainties = []

        for t in range(time_steps):
            true_pos = target_trajectory[t]
            # 收集测量
            measured_ranges = []
            sensor_positions = []
            state_vars = []
            for s in self.sensors:
                dist = np.linalg.norm(true_pos - s.position)
                meas = dist + np.random.normal(0, s.noise_std) if np.random.random() < s.detection_prob else np.nan
                measured_ranges.append(meas)
                sensor_positions.append(s.position)
                state_vars.append(s.noise_std**2)

            # 位置估计
            est = self.estimate_position_multilateration(
                list(zip(measured_ranges, sensor_positions)))
            estimates.append(est)

            # 融合不确定性
            _, cov = self.fusion.kalman_fusion(
                [m if not np.isnan(m) else np.nan for m in measured_ranges],
                state_vars)
            uncertainties.append(cov)

        return np.array(estimates), np.array(uncertainties)


# ======================== 仿真与可视化 ========================

def simulate_sensor_array():
    """完整传感器阵列仿真"""
    print("=" * 60)
    print("传感器阵列仿真系统")
    print("=" * 60)

    # 1. 生成传感器布局
    print("\n[1] 生成传感器布局...")
    positions_grid = LayoutGenerator.grid_array(5, 5, spacing=3.0)
    positions_circle = LayoutGenerator.circular_array(8, radius=8.0, center=np.array([7.5, 7.5]))

    sensors = []
    for i, pos in enumerate(positions_grid):
        sensors.append(Sensor(
            sensor_id=i, position=pos, sensing_range=2.5,
            detection_prob=0.9, noise_std=0.3, sensor_type="temperature"
        ))
    for i, pos in enumerate(positions_circle):
        sensors.append(Sensor(
            sensor_id=25+i, position=pos, sensing_range=3.0,
            detection_prob=0.85, noise_std=0.5, sensor_type="humidity"
        ))

    print(f"   共部署 {len(sensors)} 个传感器")

    # 2. 覆盖分析
    print("\n[2] 覆盖率分析...")
    area = (15, 15)
    analyzer = CoverageAnalyzer(sensors, area)
    ratio = analyzer.coverage_ratio(resolution=0.3)
    print(f"   覆盖率: {ratio:.1%}")

    nodes, edges = analyzer.connectivity_graph()
    print(f"   连通边数: {len(edges)}")

    # 3. 数据融合对比
    print("\n[3] 多传感器数据融合对比...")
    true_value = 25.0  # 真实温度
    measurements = [s.measure(true_value) for s in sensors[:10]]
    weights = [1.0/s.noise_std**2 for s in sensors[:10]]
    variances = [s.noise_std**2 for s in sensors[:10]]

    avg_result = FusionAlgorithms.weighted_average(measurements, [1.0]*len(measurements))
    inv_var_result = FusionAlgorithms.weighted_by_inverse_variance(measurements, variances)
    kf_state, kf_cov = FusionAlgorithms.kalman_fusion(measurements, variances)

    print(f"   真实值:        {true_value:.2f}")
    print(f"   简单平均:      {avg_result:.2f} (误差={abs(avg_result-true_value):.3f})")
    print(f"   逆方差加权:    {inv_var_result:.2f} (误差={abs(inv_var_result-true_value):.3f})")
    print(f"   卡尔曼融合:    {kf_state:.2f} (误差={abs(kf_state-true_value):.3f}, 方差={kf_cov:.4f})")

    # D-S证据融合示例
    print("\n[4] D-S证据理论融合...")
    mass_funcs = [
        {"目标A": 0.6, "目标B": 0.2, "uncertainty": 0.2},
        {"目标A": 0.5, "目标B": 0.3, "uncertainty": 0.2},
        {"目标A": 0.7, "目标B": 0.1, "uncertainty": 0.2},
    ]
    ds_result = FusionAlgorithms.ds_evidence(mass_funcs, ["目标A", "目标B"])
    for hyp, mass in sorted(ds_result.items(), key=lambda x: -x[1]):
        print(f"   {hyp}: {mass:.3f}")

    # 4. 目标跟踪
    print("\n[5] 目标跟踪仿真...")
    T = 50
    t = np.linspace(0, 2*np.pi, T)
    trajectory = np.column_stack([
        7.5 + 5*np.cos(t) + 0.3*np.random.randn(T),
        7.5 + 5*np.sin(t) + 0.3*np.random.randn(T)
    ])

    tracker = TargetTracker(sensors)
    estimates, uncertainties = tracker.track_target_over_time(trajectory, T)

    valid_mask = ~np.isnan(estimates[:, 0])
    if valid_mask.any():
        rmse = np.sqrt(np.mean(np.sum((estimates[valid_mask] - trajectory[valid_mask])**2, axis=1)))
        print(f"   跟踪RMSE: {rmse:.3f}")
        print(f"   有效估计率: {valid_mask.mean():.1%}")

    # ======================== 可视化 ========================
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('传感器阵列仿真系统', fontsize=16, fontweight='bold')

    # (1) 传感器布局与覆盖
    ax = axes[0, 0]
    xx, yy, coverage = analyzer.compute_coverage_map(resolution=0.3)
    ax.contourf(xx, yy, coverage, levels=20, cmap='YlOrRd', alpha=0.6)
    for s in sensors:
        color = 'blue' if s.sensor_type == 'temperature' else 'green'
        ax.plot(s.position[0], s.position[1], 'o', color=color, markersize=4)
        circle = Circle(s.position, s.sensing_range, fill=False, color=color, alpha=0.3, linewidth=0.5)
        ax.add_patch(circle)
    ax.set_title(f'传感器布局与覆盖 (率={ratio:.1%})')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')
    ax.set_xlim(-1, area[0]+1)
    ax.set_ylim(-1, area[1]+1)

    # (2) 连通性图
    ax = axes[0, 1]
    for i, j in edges:
        ax.plot([nodes[i][0], nodes[j][0]], [nodes[i][1], nodes[j][1]],
                'b-', alpha=0.2, linewidth=0.5)
    for n_pos in nodes:
        ax.plot(n_pos[0], n_pos[1], 'ro', markersize=5)
    ax.set_title(f'传感器连通性 ({len(edges)} 条边)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')

    # (3) 覆盖热力图
    ax = axes[0, 2]
    c = ax.pcolormesh(xx, yy, coverage, cmap='hot', shading='auto')
    fig.colorbar(c, ax=ax, label='覆盖传感器数')
    ax.set_title('覆盖密度热力图')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')

    # (4) 融合算法对比
    ax = axes[1, 0]
    individual_errors = [abs(m - true_value) for m in measurements if not np.isnan(m)]
    methods = ['单传感器\n(均值)', '简单平均', '逆方差加权', '卡尔曼融合']
    errors = [np.mean(individual_errors), abs(avg_result-true_value),
              abs(inv_var_result-true_value), abs(kf_state-true_value)]
    colors = ['#ff6b6b', '#ffa726', '#66bb6a', '#42a5f5']
    bars = ax.bar(methods, errors, color=colors)
    ax.set_ylabel('绝对误差')
    ax.set_title('融合算法精度对比')
    for bar, err in zip(bars, errors):
        ax.text(bar.get_x()+bar.get_width()/2, bar.get_height(),
                f'{err:.4f}', ha='center', va='bottom', fontsize=9)

    # (5) 目标跟踪轨迹
    ax = axes[1, 1]
    ax.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, label='真实轨迹')
    ax.plot(estimates[valid_mask, 0], estimates[valid_mask, 1],
            'r--', linewidth=1, alpha=0.8, label='估计轨迹')
    for s in sensors[:5]:  # 只画部分传感器
        ax.plot(s.position[0], s.position[1], 'g^', markersize=6)
    ax.legend()
    ax.set_title('目标跟踪轨迹')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # (6) 跟踪误差时序
    ax = axes[1, 2]
    errors_t = np.sqrt(np.sum((estimates - trajectory)**2, axis=1))
    errors_t[~valid_mask] = np.nan
    ax.plot(errors_t, 'b-', linewidth=1)
    ax.axhline(y=np.nanmean(errors_t), color='r', linestyle='--', label=f'均值={np.nanmean(errors_t):.2f}')
    ax.set_xlabel('时间步')
    ax.set_ylabel('定位误差 (m)')
    ax.set_title('跟踪误差时序')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = './nuedc-asset-library/15_simulation/sensor_array_result.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n图表已保存: {save_path}")
    plt.show()
    print("\n仿真完成！")


if __name__ == '__main__':
    simulate_sensor_array()
