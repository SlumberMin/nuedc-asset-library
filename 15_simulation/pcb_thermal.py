#!/usr/bin/env python3
"""PCB热仿真V2 — 多层板 / 铜皮覆盖率 / 散热过孔 / 热耦合效应"""
import numpy as np, matplotlib.pyplot as plt, os
plt.rcParams['font.sans-serif'] = ['SimHei','Microsoft YaHei','DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class PCBThermalV2:
    def __init__(self, nx=60, ny=60, layers=4):
        self.nx, self.ny, self.nl = nx, ny, layers
        self.dx = 0.5e-3  # 0.5mm网格
        self.T = np.ones((layers, ny, nx)) * 25.0  # 环境温度
        self.k_layer = np.array([0.3, 385, 0.3, 385])[:layers]  # FR4/Cu交替
        self.cu_coverage = np.ones((layers, ny, nx)) * 0.3  # 铜皮覆盖率
        self.via_map = np.zeros((ny, nx))  # 散热过孔密度
        self.sources = []  # (layer, y, x, power_W)
        self.h_conv = 10  # 对流系数 W/(m²·K)
        self.T_amb = 25

    def add_heat_source(self, layer, cy, cx, radius, power):
        """添加热源（高斯分布）"""
        yy, xx = np.ogrid[:self.ny, :self.nx]
        mask = np.exp(-((xx-cx)**2 + (yy-cy)**2) / (2*radius**2))
        self.sources.append((layer, mask, power))

    def set_copper(self, layer, coverage):
        self.cu_coverage[layer] = np.clip(coverage, 0, 1)

    def add_thermal_vias(self, cy, cx, radius, density):
        """添加散热过孔区域"""
        yy, xx = np.ogrid[:self.ny, :self.nx]
        via_d = np.exp(-((xx-cx)**2 + (yy-cy)**2) / (2*radius**2)) * density
        self.via_map = np.maximum(self.via_map, via_d)

    def run(self, n_iter=3000, dt=0.001):
        alpha_base = 0.3  # FR4热扩散率
        for it in range(n_iter):
            T_new = self.T.copy()
            for l in range(self.nl):
                k_eff = self.k_layer[l] * (0.3 + 0.7 * self.cu_coverage[l])
                # Effective volumetric heat capacity (J/m³·K)
                rho_cp = 1.7e6  # FR4 ~1.7 MJ/m³K
                alpha = k_eff * dt / (self.dx**2 * rho_cp)
                # 2D扩散
                lap = (np.roll(self.T[l], -1, 0) + np.roll(self.T[l], 1, 0) +
                       np.roll(self.T[l], -1, 1) + np.roll(self.T[l], 1, 1) - 4*self.T[l])
                T_new[l] += alpha * lap
                # 层间热传导 (通过过孔增强)
                via_coupling = 0.001 + self.via_map * 0.05  # base + via enhanced
                if l > 0:
                    coupling = (self.T[l-1] - self.T[l]) * via_coupling * dt
                    T_new[l] += coupling; T_new[l-1] -= coupling
                if l < self.nl - 1:
                    coupling = (self.T[l+1] - self.T[l]) * via_coupling * dt
                    T_new[l] += coupling; T_new[l+1] -= coupling
                # 热源
                for sl, mask, pw in self.sources:
                    if sl == l:
                        # Source spread over area*dx thickness, normalize by volume heat cap
                        T_new[l] += mask * pw * dt / (rho_cp * self.dx)
                # 表面对流散热 (顶层和底层)
                if l == 0 or l == self.nl - 1:
                    T_new[l] -= self.h_conv * (self.T[l] - self.T_amb) * dt / (rho_cp * self.dx)
            self.T = np.clip(T_new, self.T_amb, 500)
        return self.T

# ── 场景1: 双层板 vs 4层板 ──
pcb2 = PCBThermalV2(layers=2)
pcb2.add_heat_source(0, 30, 30, 5, 2.0)
pcb2.add_heat_source(0, 15, 45, 4, 1.0)
T2 = pcb2.run(3000)

pcb4 = PCBThermalV2(layers=4)
pcb4.add_heat_source(0, 30, 30, 5, 2.0)
pcb4.add_heat_source(0, 15, 45, 4, 1.0)
T4 = pcb4.run(3000)

# ── 场景2: 铜皮覆盖率影响 ──
pcb_lo_cu = PCBThermalV2(layers=4)
pcb_lo_cu.set_copper(1, 0.1); pcb_lo_cu.set_copper(3, 0.1)
pcb_lo_cu.add_heat_source(0, 30, 30, 5, 2.0)
T_lo = pcb_lo_cu.run(3000)

pcb_hi_cu = PCBThermalV2(layers=4)
pcb_hi_cu.set_copper(1, 0.8); pcb_hi_cu.set_copper(3, 0.8)
pcb_hi_cu.add_heat_source(0, 30, 30, 5, 2.0)
T_hi = pcb_hi_cu.run(3000)

# ── 场景3: 散热过孔效果 ──
pcb_via = PCBThermalV2(layers=4)
pcb_via.add_heat_source(0, 30, 30, 5, 2.0)
pcb_via.add_thermal_vias(30, 30, 8, 0.8)
T_via = pcb_via.run(3000)

# ── 绘图 ──
fig, axes = plt.subplots(2, 3, figsize=(16, 10))
fig.suptitle('PCB热仿真V2 — 多层板/铜皮/散热过孔', fontsize=14, fontweight='bold')

ims = []
for ax, data, title in [(axes[0,0], T2[0], '双层板 顶层'),
                          (axes[0,1], T4[0], '4层板 顶层'),
                          (axes[0,2], T4[1], '4层板 内层1'),
                          (axes[1,0], T_lo[0], '低铜皮(10%) 顶层'),
                          (axes[1,1], T_hi[0], '高铜皮(80%) 顶层'),
                          (axes[1,2], T_via[0], '散热过孔 顶层')]:
    im = ax.imshow(data, cmap='hot', vmin=25, vmax=max(data.max(), 60), origin='lower')
    plt.colorbar(im, ax=ax, fraction=0.046)
    ax.set_title(f'{title}\nMax={data.max():.1f}°C')

plt.tight_layout()
out = os.path.join(os.path.dirname(__file__), 'pcb_thermal_v2_result.png')
plt.savefig(out, dpi=150); print(f'已保存: {out}')
print(f'双层板Max={T2[0].max():.1f}°C, 4层板Max={T4[0].max():.1f}°C')
print(f'低铜皮Max={T_lo[0].max():.1f}°C, 高铜皮Max={T_hi[0].max():.1f}°C')
print(f'散热过孔Max={T_via[0].max():.1f}°C')
