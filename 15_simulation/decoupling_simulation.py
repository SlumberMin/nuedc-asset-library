#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
解耦控制仿真 - 2×2 MIMO系统
=============================
功能：静态解耦、动态解耦、相对增益矩阵(RGA)分析
作者：nuedc-asset-library
"""

import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ============================================================
# 1. 2×2 MIMO 系统定义
# ============================================================
class MIMOSystem:
    """2×2 MIMO传递函数矩阵（SISO传递函数组合）"""

    def __init__(self):
        # G11 = 2/(s+1)
        self.G11_num, self.G11_den = [2], [1, 1]
        # G12 = 1/(s+2)
        self.G12_num, self.G12_den = [1], [1, 2]
        # G21 = 1.5/(s+3)
        self.G21_num, self.G21_den = [1.5], [1, 3]
        # G22 = 3/(s+1)
        self.G22_num, self.G22_den = [3], [1, 1]

    def dc_gain(self):
        """计算稳态增益矩阵"""
        G11 = np.polyval(self.G11_num, 0) / np.polyval(self.G11_den, 0)
        G12 = np.polyval(self.G12_num, 0) / np.polyval(self.G12_den, 0)
        G21 = np.polyval(self.G21_num, 0) / np.polyval(self.G21_den, 0)
        G22 = np.polyval(self.G22_num, 0) / np.polyval(self.G22_den, 0)
        return np.array([[G11, G12], [G21, G22]])

    def step_response(self, t, u1, u2):
        """计算2输入2输出的阶跃响应"""
        sys11 = signal.TransferFunction(self.G11_num, self.G11_den)
        sys12 = signal.TransferFunction(self.G12_num, self.G12_den)
        sys21 = signal.TransferFunction(self.G21_num, self.G21_den)
        sys22 = signal.TransferFunction(self.G22_num, self.G22_den)

        _, y11 = signal.step(sys11, T=t)
        _, y12 = signal.step(sys12, T=t)
        _, y21 = signal.step(sys21, T=t)
        _, y22 = signal.step(sys22, T=t)

        y1 = u1 * y11 + u2 * y12
        y2 = u1 * y21 + u2 * y22
        return y1, y2


# ============================================================
# 2. RGA (相对增益矩阵)
# ============================================================
def compute_rga(G):
    """计算相对增益矩阵 Λ = G ⊗ (G^{-T})"""
    G_inv = np.linalg.inv(G)
    RGA = G * G_inv.T  # Hadamard product
    return RGA


# ============================================================
# 3. 静态解耦
# ============================================================
def static_decoupler(G):
    """静态解耦矩阵 D = G(0)^{-1}"""
    return np.linalg.inv(G)


# ============================================================
# 4. 主程序
# ============================================================
def main():
    plant = MIMOSystem()
    G = plant.dc_gain()

    print("=" * 60)
    print("2×2 MIMO 解耦控制仿真")
    print("=" * 60)
    print(f"\n稳态增益矩阵 G(0):\n{G}")

    # --- RGA ---
    rga = compute_rga(G)
    print(f"\n相对增益矩阵 RGA:\n{np.round(rga, 4)}")
    print("RGA元素接近1 → 该变量配对合适")
    print("RGA元素接近0 → 该耦合可忽略")

    # --- 静态解耦 ---
    D = static_decoupler(G)
    print(f"\n静态解耦矩阵 D = G(0)^{{-1}}:\n{np.round(D, 4)}")
    G_dec = G @ D
    print(f"\n解耦后稳态增益 (应≈I):\n{np.round(G_dec, 4)}")

    # --- 仿真 ---
    t = np.linspace(0, 10, 500)

    # 未解耦：u1=1, u2=0
    y1_coupled, y2_coupled = plant.step_response(t, 1.0, 0.0)

    # 静态解耦后
    u_dec = D @ np.array([1.0, 0.0])
    y1_dec, y2_dec = plant.step_response(t, u_dec[0], u_dec[1])

    # --- 绘图 ---
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('2×2 MIMO 解耦控制仿真', fontsize=14, fontweight='bold')

    # 未解耦响应
    axes[0, 0].plot(t, y1_coupled, 'b-', linewidth=1.5, label='$y_1$')
    axes[0, 0].plot(t, y2_coupled, 'r--', linewidth=1.5, label='$y_2$')
    axes[0, 0].set_title('未解耦响应 ($u_1=1, u_2=0$)')
    axes[0, 0].set_ylabel('输出')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 解耦后响应
    axes[0, 1].plot(t, y1_dec, 'b-', linewidth=1.5, label='$y_1$ (解耦后)')
    axes[0, 1].plot(t, y2_dec, 'r--', linewidth=1.5, label='$y_2$ (解耦后)')
    axes[0, 1].set_title('静态解耦后响应')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # 交叉耦合对比
    axes[1, 0].plot(t, y2_coupled, 'r-', linewidth=1.5, label='未解耦 $y_2$')
    axes[1, 0].plot(t, y2_dec, 'g-', linewidth=1.5, label='解耦后 $y_2$')
    axes[1, 0].set_title('交叉耦合对比 (通道2应为0)')
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].set_ylabel('输出')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # RGA热力图
    im = axes[1, 1].imshow(rga, cmap='RdYlBu_r', vmin=-1, vmax=2)
    axes[1, 1].set_title('RGA 热力图')
    for i in range(2):
        for j in range(2):
            axes[1, 1].text(j, i, f'{rga[i, j]:.3f}', ha='center', va='center', fontsize=14)
    axes[1, 1].set_xticks([0, 1])
    axes[1, 1].set_yticks([0, 1])
    axes[1, 1].set_xticklabels(['$u_1$', '$u_2$'])
    axes[1, 1].set_yticklabels(['$y_1$', '$y_2$'])
    plt.colorbar(im, ax=axes[1, 1])

    plt.tight_layout()
    plt.savefig('decoupling_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("\n仿真结果已保存: decoupling_result.png")


if __name__ == '__main__':
    main()
