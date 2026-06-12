#!/usr/bin/env python3
"""
ToF 激光测距仿真 (Time-of-Flight Laser Ranging Simulation)
============================================================
仿真内容:
  - 基于 d = c·t/2 的ToF测距原理
  - 高斯噪声 + 暗计数噪声 + 环境光散粒噪声
  - 多目标回波信号仿真 (距离+反射率)
  - 环境光干扰建模 (太阳光/人工光源)
  - 距离精度与信噪比分析

依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Tuple

# ── 物理常量 ─────────────────────────────────────────────
C_LIGHT = 3e8          # 光速 m/s
PLANCK  = 6.626e-34    # 普朗克常数 J·s
WAVELENGTH = 905e-9    # 激光波长 905nm (典型ToF传感器)


@dataclass
class ToFConfig:
    """ToF传感器配置"""
    laser_power: float = 75e-3        # 激光发射功率 75mW
    wavelength: float = WAVELENGTH
    detector_area: float = 1e-6       # 探测器面积 1mm²
    detector_efficiency: float = 0.6  # 量子效率
    fov: float = np.deg2rad(25)       # 视场角 25°
    aperture: float = 10e-3           # 光圈直径 10mm
    noise_equivalent_power: float = 1e-12  # NEP W/√Hz
    bandwidth: float = 10e6           # 检测带宽 10MHz
    dark_count_rate: float = 100      # 暗计数率 Hz
    timing_resolution: float = 50e-12 # 时间分辨率 50ps


@dataclass
class Target:
    """测量目标"""
    distance: float        # 距离 m
    reflectivity: float    # 反射率 0~1
    size: float = 0.1      # 目标尺寸 m


@dataclass
class AmbientLight:
    """环境光条件"""
    name: str = "indoor"
    irradiance: float = 1e-3  # W/m² (1mW/m² 室内)

    @staticmethod
    def outdoor_shade():
        return AmbientLight("outdoor_shade", irradiance=1e-2)
    @staticmethod
    def outdoor_sun():
        return AmbientLight("outdoor_sun", irradiance=0.1)
    @staticmethod
    def indoor():
        return AmbientLight("indoor", irradiance=1e-3)
    @staticmethod
    def dark_room():
        return AmbientLight("dark_room", irradiance=1e-6)


class ToFLaserSimulation:
    """ToF激光测距仿真引擎"""

    def __init__(self, config: ToFConfig = None):
        self.config = config or ToFConfig()

    # ── 信号模型 ──────────────────────────────────────────
    def target_echo_power(self, target: Target) -> float:
        """计算目标回波功率 (激光雷达方程)
        P_echo = (P_tx * ρ * A_det) / (π * R²)
        简化模型: 假设朗伯反射, 目标大于光斑
        """
        cfg = self.config
        R = target.distance
        if R <= 0:
            return 0.0
        # 光斑在目标处的面积 (基于FOV)
        spot_area = np.pi * (R * np.tan(cfg.fov / 2)) ** 2
        effective_area = min(target.size ** 2, spot_area)
        # 回波功率
        P_echo = (cfg.laser_power * target.reflectivity * cfg.detector_area
                  * cfg.detector_efficiency) / (np.pi * R ** 2)
        return P_echo

    def ambient_noise_power(self, ambient: AmbientLight) -> float:
        """环境光产生的散粒噪声等效功率"""
        cfg = self.config
        P_amb = ambient.irradiance * cfg.detector_area * cfg.detector_efficiency
        # 散粒噪声: σ = √(2q·I_ph·B), 等效为 NEP·√B
        shot_noise = np.sqrt(2 * 1.6e-19 * P_amb / 1.6e-19 * cfg.bandwidth) * PLANCK * C_LIGHT / cfg.wavelength
        # 简化: 直接用 NEP 模型
        noise_power = cfg.noise_equivalent_power * np.sqrt(cfg.bandwidth)
        # 加上环境光散粒噪声
        env_noise = np.sqrt(2 * 1.6e-19 * (P_amb / (PLANCK * C_LIGHT / cfg.wavelength)) * cfg.bandwidth)
        env_noise_energy = env_noise * PLANCK * C_LIGHT / cfg.wavelength
        return noise_power + env_noise_energy

    # ── 距离测量 ──────────────────────────────────────────
    def measure_single(self, target: Target, ambient: AmbientLight,
                       n_samples: int = 1000) -> dict:
        """单目标测距仿真, 返回统计结果"""
        cfg = self.config
        true_distance = target.distance
        true_time = 2 * true_distance / C_LIGHT

        # 信号功率
        P_echo = self.target_echo_power(target)
        P_noise = self.ambient_noise_power(ambient)

        # 信噪比
        snr_linear = P_echo / P_noise if P_noise > 0 else np.inf
        snr_db = 10 * np.log10(snr_linear) if snr_linear > 0 else -np.inf

        # 生成含噪声的测量值
        # 时间抖动: σ_t ∝ 1/√(SNR) + timing_resolution
        sigma_t = cfg.timing_resolution * (1 + 1 / np.sqrt(max(snr_linear, 1e-10)))
        measured_times = true_time + np.random.normal(0, sigma_t, n_samples)

        # 暗计数引起的随机测量 (泊松分布)
        dark_events = np.random.poisson(cfg.dark_count_rate * (n_samples / cfg.bandwidth))
        if dark_events > 0:
            dark_idx = np.random.choice(n_samples, min(dark_events, n_samples), replace=False)
            measured_times[dark_idx] = np.random.uniform(0, 2 * 10 / C_LIGHT, len(dark_idx))

        measured_distances = measured_times * C_LIGHT / 2

        return {
            "true_distance": true_distance,
            "mean_measured": np.mean(measured_distances),
            "std_measured": np.std(measured_distances),
            "rmse": np.sqrt(np.mean((measured_distances - true_distance) ** 2)),
            "snr_db": snr_db,
            "echo_power": P_echo,
            "noise_power": P_noise,
            "measured_distances": measured_distances,
            "n_dark_events": dark_events,
        }

    def multi_target_measure(self, targets: List[Target],
                             ambient: AmbientLight,
                             n_samples: int = 1000) -> dict:
        """多目标测距仿真 - 模拟回波叠加"""
        cfg = self.config
        results = []

        for tgt in targets:
            res = self.measure_single(tgt, ambient, n_samples)
            results.append(res)

        # 多目标时, 近距离强目标可能掩盖远距离弱目标 (串扰)
        # 计算目标间干扰因子
        interference = np.zeros((len(targets), len(targets)))
        for i, ti in enumerate(targets):
            for j, tj in enumerate(targets):
                if i != j:
                    # 近目标对远目标的干扰 (拖尾效应)
                    delta_t = abs(2 * (tj.distance - ti.distance) / C_LIGHT)
                    if delta_t < 5 * cfg.timing_resolution and self.target_echo_power(ti) > self.target_echo_power(tj):
                        interference[i][j] = 0.1  # 10%干扰

        return {
            "per_target": results,
            "interference_matrix": interference,
            "targets": targets,
            "ambient": ambient,
        }

    # ── 扫描仿真 ──────────────────────────────────────────
    def line_scan(self, targets: List[Target], ambient: AmbientLight,
                  scan_range: Tuple[float, float] = (0, 5.0),
                  n_points: int = 200) -> dict:
        """一维线扫描仿真 (模拟激光雷达扫描)"""
        distances = np.linspace(scan_range[0], scan_range[1], n_points)
        measured = np.zeros(n_points)
        snr = np.zeros(n_points)

        for i, d in enumerate(distances):
            if d < 0.01:
                measured[i] = 0
                snr[i] = -np.inf
                continue
            # 找最近的目标
            closest_tgt = None
            min_diff = float('inf')
            for t in targets:
                diff = abs(t.distance - d)
                if diff < min_diff:
                    min_diff = diff
                    closest_tgt = t

            if closest_tgt and min_diff < 0.05:  # 5cm范围内视为命中
                res = self.measure_single(closest_tgt, ambient, 100)
                measured[i] = res["mean_measured"]
                snr[i] = res["snr_db"]
            else:
                # 无目标, 测量噪声底
                measured[i] = d + np.random.normal(0, 0.01)
                snr[i] = -20

        return {"scan_distances": distances, "measured": measured, "snr": snr, "targets": targets}


def run_demo():
    """运行完整仿真演示"""
    print("=" * 70)
    print("ToF 激光测距仿真")
    print("=" * 70)

    sim = ToFLaserSimulation()
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    # ── 1. 单目标: 不同距离的精度 ─────────────────────────
    print("\n[1] 不同距离测距精度...")
    distances = np.arange(0.1, 5.1, 0.2)
    ambient = AmbientLight.indoor()
    rmses = []
    snrs = []
    for d in distances:
        tgt = Target(distance=d, reflectivity=0.9, size=0.1)
        res = sim.measure_single(tgt, ambient, 500)
        rmses.append(res["rmse"] * 100)  # cm
        snrs.append(res["snr_db"])

    ax = axes[0, 0]
    ax.plot(distances, rmses, 'b-o', markersize=3, label='RMSE (cm)')
    ax.set_xlabel('目标距离 (m)')
    ax.set_ylabel('测距误差 RMSE (cm)')
    ax.set_title('不同距离的测距精度')
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax2 = ax.twinx()
    ax2.plot(distances, snrs, 'r--s', markersize=3, label='SNR (dB)')
    ax2.set_ylabel('信噪比 SNR (dB)')
    ax2.legend(loc='upper right')

    # ── 2. 不同环境光条件 ────────────────────────────────
    print("[2] 不同环境光条件...")
    ambients = [AmbientLight.dark_room(), AmbientLight.indoor(),
                AmbientLight.outdoor_shade(), AmbientLight.outdoor_sun()]
    ambient_names = ['暗室', '室内', '户外阴影', '户外阳光']
    tgt = Target(distance=2.0, reflectivity=0.5, size=0.1)

    ax = axes[0, 1]
    for amb, name in zip(ambients, ambient_names):
        res = sim.measure_single(tgt, amb, 1000)
        ax.hist(res["measured_distances"] * 100, bins=50, alpha=0.6,
                label=f'{name}\nSNR={res["snr_db"]:.1f}dB')
    ax.axvline(200, color='k', linestyle='--', label='真实距离=200cm')
    ax.set_xlabel('测量距离 (cm)')
    ax.set_ylabel('计数')
    ax.set_title('不同环境光下的测量分布')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ── 3. 不同反射率目标 ────────────────────────────────
    print("[3] 不同反射率目标...")
    reflectivities = np.arange(0.05, 1.05, 0.05)
    rmses_by_reflect = []
    snrs_by_reflect = []
    for rho in reflectivities:
        tgt = Target(distance=3.0, reflectivity=rho, size=0.1)
        res = sim.measure_single(tgt, ambient, 500)
        rmses_by_reflect.append(res["rmse"] * 100)
        snrs_by_reflect.append(res["snr_db"])

    ax = axes[0, 2]
    ax.plot(reflectivities * 100, rmses_by_reflect, 'g-o', markersize=3)
    ax.set_xlabel('目标反射率 (%)')
    ax.set_ylabel('测距误差 RMSE (cm)')
    ax.set_title('反射率对精度的影响 (R=3m)')
    ax.grid(True, alpha=0.3)

    # ── 4. 多目标回波仿真 ────────────────────────────────
    print("[4] 多目标回波仿真...")
    multi_targets = [
        Target(distance=1.0, reflectivity=0.9, size=0.2),
        Target(distance=2.5, reflectivity=0.5, size=0.1),
        Target(distance=4.0, reflectivity=0.3, size=0.15),
    ]
    multi_res = sim.multi_target_measure(multi_targets, AmbientLight.indoor(), 1000)

    ax = axes[1, 0]
    colors = ['blue', 'green', 'red']
    for i, (res, tgt) in enumerate(zip(multi_res["per_target"], multi_targets)):
        ax.hist(res["measured_distances"] * 100, bins=60, alpha=0.5,
                color=colors[i],
                label=f'R={tgt.distance}m, ρ={tgt.reflectivity}')
        ax.axvline(tgt.distance * 100, color=colors[i], linestyle='--', alpha=0.8)
    ax.set_xlabel('测量距离 (cm)')
    ax.set_ylabel('计数')
    ax.set_title('多目标测距仿真')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ── 5. 一维线扫描 ────────────────────────────────────
    print("[5] 一维线扫描仿真...")
    scan_targets = [
        Target(distance=1.5, reflectivity=0.8, size=0.3),
        Target(distance=3.0, reflectivity=0.4, size=0.2),
    ]
    scan_res = sim.line_scan(scan_targets, AmbientLight.indoor(), (0, 5), 300)

    ax = axes[1, 1]
    ax.plot(scan_res["scan_distances"], scan_res["measured"] * 100,
            'b.', markersize=2, alpha=0.5, label='测量值')
    for t in scan_targets:
        ax.axvline(t.distance, color='r', linestyle='--', alpha=0.7)
    ax.set_xlabel('扫描距离 (m)')
    ax.set_ylabel('测量距离 (cm)')
    ax.set_title('一维线扫描仿真')
    ax.grid(True, alpha=0.3)

    # ── 6. SNR vs 距离+反射率 热力图 ─────────────────────
    print("[6] SNR热力图...")
    dist_range = np.linspace(0.2, 5, 50)
    refl_range = np.linspace(0.05, 1.0, 50)
    snr_map = np.zeros((len(refl_range), len(dist_range)))

    for i, rho in enumerate(refl_range):
        for j, d in enumerate(dist_range):
            tgt = Target(distance=d, reflectivity=rho, size=0.1)
            res = sim.measure_single(tgt, AmbientLight.indoor(), 50)
            snr_map[i, j] = res["snr_db"]

    ax = axes[1, 2]
    im = ax.imshow(snr_map, extent=[0.2, 5, 5, 100], aspect='auto',
                   cmap='RdYlGn', origin='lower')
    ax.set_xlabel('目标距离 (m)')
    ax.set_ylabel('反射率 (%)')
    ax.set_title('SNR 热力图 (dB)')
    plt.colorbar(im, ax=ax, label='SNR (dB)')

    plt.tight_layout()
    plt.savefig('tof_laser_simulation.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("\n仿真完成! 图表已保存为 tof_laser_simulation.png")


if __name__ == "__main__":
    run_demo()
