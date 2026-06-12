#!/usr/bin/env python3
"""
数字时钟仿真
功能：七段数码管显示、时间同步仿真、温度补偿晶振漂移
适用：电赛数字时钟/计时器类题目
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec

# ============== 时钟参数 ==============
CRYSTAL_FREQ = 32768    # 32.768 kHz晶振
TEMP_COEFF = -0.035     # ppm/°C² (典型AT切晶振)
T_REF = 25.0            # 参考温度 (°C)
ADC_BITS = 12           # 温度ADC分辨率
TEMP_RANGE = (-40, 85)  # 工作温度范围 (°C)

# 七段编码 (a,b,c,d,e,f,g) — 共阳极，0=亮
SEVEN_SEG = {
    0: [0, 0, 0, 0, 0, 0, 1],
    1: [1, 0, 0, 1, 1, 1, 1],
    2: [0, 0, 1, 0, 0, 1, 0],
    3: [0, 0, 0, 0, 1, 1, 0],
    4: [1, 0, 0, 1, 1, 0, 0],
    5: [0, 1, 0, 0, 1, 0, 0],
    6: [0, 1, 0, 0, 0, 0, 0],
    7: [0, 0, 0, 1, 1, 1, 1],
    8: [0, 0, 0, 0, 0, 0, 0],
    9: [0, 0, 0, 0, 1, 0, 0],
}

# 七段笔画坐标 (起始x, 起始y, 方向, 长度)
SEG_GEOMETRY = {
    'a': (0.15, 0.85, 'h', 0.70),  # 上横
    'b': (0.85, 0.50, 'v', 0.35),  # 右上竖
    'c': (0.85, 0.10, 'v', 0.35),  # 右下竖
    'd': (0.15, 0.05, 'h', 0.70),  # 下横
    'e': (0.05, 0.10, 'v', 0.35),  # 左下竖
    'f': (0.05, 0.50, 'v', 0.35),  # 左上竖
    'g': (0.15, 0.45, 'h', 0.70),  # 中横
}
SEG_NAMES = ['a', 'b', 'c', 'd', 'e', 'f', 'g']


def crystal_freq_error(temp, t_ref=T_REF, coeff=TEMP_COEFF):
    """晶振频率随温度的漂移 (ppm)"""
    dt = temp - t_ref
    return coeff * dt ** 2  # 抛物线模型


def draw_seven_seg(ax, x, y, w, h, digit, color_on='#FF0000', color_off='#220000'):
    """绘制单个七段数码管"""
    states = SEVEN_SEG.get(digit, [1]*7)
    seg_w = w
    seg_h = h

    for i, seg_name in enumerate(SEG_NAMES):
        sx, sy, direction, slen = SEG_GEOMETRY[seg_name]
        color = color_on if states[i] == 0 else color_off

        if direction == 'h':
            rect = patches.FancyBboxPatch(
                (x + sx * seg_w, y + sy * seg_h),
                slen * seg_w, 0.10 * seg_h,
                boxstyle="round,pad=0.01",
                facecolor=color, edgecolor='#333', linewidth=0.5
            )
        else:
            rect = patches.FancyBboxPatch(
                (x + sx * seg_w, y + sy * seg_h),
                0.10 * seg_w, slen * seg_h,
                boxstyle="round,pad=0.01",
                facecolor=color, edgecolor='#333', linewidth=0.5
            )
        ax.add_patch(rect)


def draw_clock_display(ax, hours, minutes, seconds, color='#FF0000'):
    """绘制完整的时钟显示 (HH:MM:SS)"""
    ax.clear()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 2)
    ax.set_aspect('equal')
    ax.set_facecolor('#0a0a0a')

    digits = [
        hours // 10, hours % 10,
        minutes // 10, minutes % 10,
        seconds // 10, seconds % 10,
    ]

    positions = [0.2, 1.4, 3.0, 4.2, 5.8, 7.0]
    for i, (d, px) in enumerate(zip(digits, positions)):
        draw_seven_seg(ax, px, 0.2, 1.0, 1.6, d, color_on=color)

    # 冒号分隔符
    for cy in [0.75, 1.25]:
        circle = patches.Circle((2.5, cy), 0.08, color=color)
        ax.add_patch(circle)
        circle = patches.Circle((5.3, cy), 0.08, color=color)
        ax.add_patch(circle)

    ax.set_title(f'{hours:02d}:{minutes:02d}:{seconds:02d}', fontsize=14,
                 color='white', fontfamily='monospace')
    ax.axis('off')


def simulate_temperature_compensation(days=30):
    """模拟温度补偿效果"""
    # 模拟一天内温度变化
    t = np.arange(0, days * 24 * 3600, 10)  # 每10秒一个点
    hours_of_day = (t % 86400) / 3600

    # 温度模型: 日变化 + 季节漂移 + 随机
    temp_daily = 8 * np.sin(2 * np.pi * (hours_of_day - 6) / 24)
    temp_season = 15 * np.sin(2 * np.pi * t / (365.25 * 86400))
    temp_noise = 2 * np.random.randn(len(t))
    temperature = T_REF + temp_daily + temp_season + temp_noise

    # 无补偿：累计误差
    freq_ppm_no_comp = crystal_freq_error(temperature)
    freq_ppm_cum = np.cumsum(freq_ppm_no_comp) * 10 / 1e6  # 累计秒误差

    # 有补偿：用ADC读温度 → 查表补偿
    temp_quantized = np.round(temperature / 0.1) * 0.1  # 0.1°C分辨率
    freq_ppm_comp = crystal_freq_error(temp_quantized)
    freq_ppm_comp_cum = np.cumsum(freq_ppm_comp) * 10 / 1e6

    return t / 86400, temperature, freq_ppm_no_comp, freq_ppm_cum, freq_ppm_comp_cum


def run_simulation():
    print("=" * 60)
    print("数字时钟仿真系统")
    print("=" * 60)

    # ---- 晶振频率特性 ----
    temps = np.linspace(-40, 85, 200)
    freq_err = crystal_freq_error(temps)
    print(f"晶振: {CRYSTAL_FREQ/1e3} kHz | 参考温度: {T_REF}°C")
    print(f"温度系数: {TEMP_COEFF} ppm/°C²")
    print(f"-40°C 频率误差: {crystal_freq_error(-40):.2f} ppm")
    print(f"+25°C 频率误差: {crystal_freq_error(25):.2f} ppm")
    print(f"+85°C 频率误差: {crystal_freq_error(85):.2f} ppm")

    # ---- 不同温度下的日误差 ----
    print("\n--- 不同温度下的日误差 (无补偿) ---")
    for temp in [-20, 0, 10, 25, 40, 60]:
        err_ppm = crystal_freq_error(temp)
        err_sec_day = err_ppm * 86400 / 1e6
        print(f"  {temp:+4.0f}°C: {err_ppm:+.2f} ppm → {err_sec_day:+.3f} 秒/天")

    # ---- 温度补偿仿真 ----
    days_arr, temp_arr, err_no, cum_no, cum_comp = simulate_temperature_compensation(30)

    # ---- 绘图 ----
    fig = plt.figure(figsize=(18, 16))
    fig.suptitle("数字时钟仿真系统", fontsize=16, fontweight='bold')
    gs = GridSpec(4, 2, figure=fig, hspace=0.4, wspace=0.3)

    # (0,0) 七段数码管显示
    ax = fig.add_subplot(gs[0, 0])
    draw_clock_display(ax, 12, 34, 56, color='#FF3300')
    ax.set_title('七段数码管显示 12:34:56', color='black', fontsize=12)

    # (0,1) 七段编码表
    ax = fig.add_subplot(gs[0, 1])
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 3)
    ax.set_aspect('equal')
    ax.set_facecolor('#0a0a0a')
    for d in range(10):
        draw_seven_seg(ax, d * 1.0, 0.5, 0.8, 2.0, d, color_on='#00FF00')
        ax.text(d * 1.0 + 0.4, 0.2, str(d), ha='center', fontsize=10, color='white')
    ax.set_title('0-9 七段编码', color='white', fontsize=12)
    ax.axis('off')

    # (1,0) 晶振频率误差 vs 温度
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(temps, freq_err, 'r-', linewidth=2)
    ax.axvline(x=T_REF, color='g', linestyle='--', alpha=0.5, label=f'参考温度 {T_REF}°C')
    ax.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
    ax.fill_between(temps, freq_err, alpha=0.2, color='red')
    ax.set_title('晶振频率误差 vs 温度')
    ax.set_xlabel('温度 (°C)')
    ax.set_ylabel('频率误差 (ppm)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (1,1) 日误差 vs 温度
    ax = fig.add_subplot(gs[1, 1])
    err_sec = freq_err * 86400 / 1e6
    ax.plot(temps, err_sec, 'b-', linewidth=2)
    ax.axhline(y=1, color='r', linestyle='--', alpha=0.5, label='±1秒/天')
    ax.axhline(y=-1, color='r', linestyle='--', alpha=0.5)
    ax.set_title('日计时误差 vs 温度')
    ax.set_xlabel('温度 (°C)')
    ax.set_ylabel('误差 (秒/天)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (2,0) 30天温度变化
    ax = fig.add_subplot(gs[2, 0])
    ax.plot(days_arr, temp_arr, 'orange', linewidth=0.3, alpha=0.5)
    # 平滑
    window = min(1000, len(temp_arr) // 10)
    if window > 1:
        temp_smooth = np.convolve(temp_arr, np.ones(window)/window, mode='same')
        ax.plot(days_arr, temp_smooth, 'r-', linewidth=2, label='平均温度')
    ax.set_title('30天温度变化仿真')
    ax.set_xlabel('天数')
    ax.set_ylabel('温度 (°C)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (2,1) 累计误差对比
    ax = fig.add_subplot(gs[2, 1])
    ax.plot(days_arr, cum_no, 'r-', linewidth=1.5, label='无补偿')
    ax.plot(days_arr, cum_comp, 'b-', linewidth=1.5, label='有补偿')
    ax.set_title('累计计时误差 (30天)')
    ax.set_xlabel('天数')
    ax.set_ylabel('累计误差 (秒)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (3,0) 补偿前后误差对比
    ax = fig.add_subplot(gs[3, 0])
    ax.plot(days_arr, np.abs(cum_no), 'r-', linewidth=1, alpha=0.7, label='无补偿')
    ax.plot(days_arr, np.abs(cum_comp), 'b-', linewidth=1, alpha=0.7, label='有补偿')
    ax.set_yscale('log')
    ax.set_title('累计误差绝对值 (对数)')
    ax.set_xlabel('天数')
    ax.set_ylabel('|误差| (秒)')
    ax.legend()
    ax.grid(True, alpha=0.3, which='both')

    # (3,1) ADC量化温度误差
    ax = fig.add_subplot(gs[3, 1])
    # 不同ADC精度下的温度量化误差
    for bits in [8, 10, 12, 16]:
        lsb_temp = (TEMP_RANGE[1] - TEMP_RANGE[0]) / (2 ** bits)
        freq_err_quant = TEMP_COEFF * lsb_temp ** 2
        err_sec_quant = freq_err_quant * 86400 / 1e6
        ax.plot(bits, err_sec_quant * 1000, 'o', markersize=10, label=f'{bits}bit ({lsb_temp:.2f}°C)')
    ax.set_title('温度ADC分辨率 vs 补偿残余误差')
    ax.set_xlabel('ADC位数')
    ax.set_ylabel('残余日误差 (ms)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('digital_clock_simulation.png', dpi=150, bbox_inches='tight')
    print("\n图像已保存: digital_clock_simulation.png")

    # ---- 数码管扫描时序 ----
    print("\n--- 数码管扫描时序设计 ---")
    digits = 6
    scan_period_ms = 2  # 每位扫描时间
    refresh_rate = 1000 / (digits * scan_period_ms)
    print(f"扫描位数: {digits} | 每位: {scan_period_ms}ms")
    print(f"刷新率: {refresh_rate:.0f} Hz (>50Hz无闪烁)")
    print(f"段驱动电流: 10~20mA | 限流电阻: {(3.3-2.0)/0.015:.0f}Ω (3.3V, 2V压降)")

    plt.show()


if __name__ == '__main__':
    run_simulation()
