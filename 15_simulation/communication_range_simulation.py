#!/usr/bin/env python3
"""
通信距离仿真 - 蓝牙/WiFi/LoRa路径损耗模型
适用于电赛无线通信模块选型与链路预算分析
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams

rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial']
rcParams['axes.unicode_minus'] = False


class PropagationModel:
    """无线信道传播模型"""

    @staticmethod
    def free_space(d, f_mhz, Gtx_dBi=0, Grx_dBi=0):
        """自由空间路径损耗 (Friis)
        d: 距离(m), f_mhz: 频率(MHz)
        """
        f_hz = f_mhz * 1e6
        PL = 20*np.log10(d) + 20*np.log10(f_hz) + 20*np.log10(4*np.pi/3e8)
        return PL - Gtx_dBi - Grx_dBi

    @staticmethod
    def log_distance(d, f_mhz, n=2.0, d0=1.0, PL_d0=None, sigma=0):
        """对数距离路径损耗模型
        n: 路径损耗指数 (2=自由空间, 2.7~3.5=城市, 4~6=室内)
        sigma: 对数正态阴影衰落标准差 (dB)
        """
        if PL_d0 is None:
            PL_d0 = PropagationModel.free_space(d0, f_mhz)
        PL = PL_d0 + 10 * n * np.log10(d / d0)
        if sigma > 0 and np.isscalar(d):
            PL += np.random.normal(0, sigma)
        elif sigma > 0:
            PL += np.random.normal(0, sigma, len(d))
        return PL

    @staticmethod
    def two_ray(d, htx, hrx, f_mhz):
        """双径模型 (地面反射)
        适用于视距开阔环境
        """
        f_hz = f_mhz * 1e6
        lam = 3e8 / f_hz
        d = np.maximum(d, 1.0)

        # 交叉距离
        d_cross = 4 * htx * hrx / lam

        PL = np.where(
            d < d_cross,
            PropagationModel.free_space(d, f_mhz),  # 近距离用FSL
            40 * np.log10(d) - 10*np.log10(htx**2 * hrx**2)  # 远距离 d^-4
        )
        return PL

    @staticmethod
    def itu_indoor(d, f_mhz, n_floors=0, n_walls=0):
        """ITU室内传播模型 (P.1238)
        适用于室内WiFi/蓝牙
        """
        f_ghz = f_mhz / 1000
        N = 28  # 路径损耗系数 (2.4GHz)
        Lf = 15 + 4 * (n_floors - 1)  # 楼层穿透损耗
        Lw = 5 * n_walls               # 墙壁穿透损耗
        PL = 20*np.log10(f_mhz) + N*np.log10(d) + Lf + Lw - 28
        return PL


# ==================== 蓝牙仿真 ====================
class BluetoothSim:
    def __init__(self):
        self.freq_mhz = 2440
        self.tx_power_dBm = 0       # BLE典型0dBm (Class 3)
        self.rx_sensitivity = -90    # BLE接收灵敏度 dBm
        self.antenna_gain = 0        # dBi (PCB天线约0~2)
        self.data_rate = 1e6         # 1Mbps
        self.bandwidth = 2e6         # 2MHz信道

    def link_budget(self, d):
        """链路预算"""
        PL = PropagationModel.log_distance(d, self.freq_mhz, n=3.0, sigma=4)
        Rx = self.tx_power_dBm + self.antenna_gain * 2 - PL
        margin = Rx - self.rx_sensitivity
        return Rx, margin

    def indoor_range(self):
        """室内覆盖范围分析"""
        d = np.logspace(-1, 2.5, 300)  # 0.1m ~ 300m
        PL_free = PropagationModel.free_space(d, self.freq_mhz)
        PL_indoor_office = PropagationModel.log_distance(d, self.freq_mhz, n=3.5, sigma=5)
        PL_indoor_factory = PropagationModel.log_distance(d, self.freq_mhz, n=4.0, sigma=7)

        return d, PL_free, PL_indoor_office, PL_indoor_factory


# ==================== WiFi仿真 ====================
class WiFiSim:
    def __init__(self, band='2.4GHz'):
        if band == '2.4GHz':
            self.freq_mhz = 2437
            self.rx_sensitivity = -82    # 54Mbps
            self.tx_power_dBm = 18       # 典型AP
        else:  # 5GHz
            self.freq_mhz = 5500
            self.rx_sensitivity = -75
            self.tx_power_dBm = 17
        self.band = band
        self.antenna_gain = 3  # dBi (AP天线)

    def throughput_vs_distance(self, d):
        """吞吐量 vs 距离"""
        PL = PropagationModel.log_distance(d, self.freq_mhz, n=3.2, sigma=5)
        Rx = self.tx_power_dBm + self.antenna_gain - PL
        snr_db = Rx - (-174 + 10*np.log10(20e6))  # 热噪声 @20MHz BW

        # 简化MCS选择 (根据SNR)
        mcs_table = [
            (0,    6.5,   'MCS0'),
            (5,    13,    'MCS1'),
            (9,    19.5,  'MCS2'),
            (12,   26,    'MCS3'),
            (16,   39,    'MCS4'),
            (18,   52,    'MCS5'),
            (20,   58.5,  'MCS6'),
            (22,   65,    'MCS7'),
        ]
        throughput = np.zeros_like(d)
        mcs_label = np.full_like(d, '', dtype=object)
        for i, (snr_th, tp, label) in enumerate(mcs_table):
            mask = snr_db >= snr_th
            throughput[mask] = tp
            mcs_label[mask] = label

        return throughput, snr_db


# ==================== LoRa仿真 ====================
class LoRaSim:
    def __init__(self, sf=7, bw=125e3, cr=1, freq_mhz=470):
        self.sf = sf
        self.bw = bw
        self.cr = cr  # 编码率 1=4/5, 2=4/6 ...
        self.freq_mhz = freq_mhz
        self.tx_power_dBm = 17   # dBm
        self.rx_sensitivity = self._calc_sensitivity()
        self.antenna_gain = 2.0  # dBi

    def _calc_sensitivity(self):
        """LoRa接收灵敏度 (基于SF和BW)"""
        # 简化公式: SNR_threshold ≈ -20dB (SF7) to -28dB (SF12)
        snr_th = -20 - (self.sf - 7) * (8 / 5)
        noise = -174 + 10 * np.log10(self.bw)
        return noise + snr_th

    @property
    def data_rate(self):
        """数据速率 (bps)"""
        return self.sf * (self.bw / 2**self.sf) * (4 / (4 + self.cr))

    @property
    def time_on_air(self):
        """单包空中时间 (ms), 假设20字节payload"""
        payload_bytes = 20
        de = 1 if self.sf >= 11 else 0
        n_sym = 8 + max(np.ceil((8*payload_bytes - 4*self.sf + 28 + 16) / (4*self.sf)) * (4+self.cr), 0)
        t_sym = 2**self.sf / self.bw
        return n_sym * t_sym * 1000

    def link_budget(self, d):
        PL = PropagationModel.log_distance(d, self.freq_mhz, n=3.5, sigma=6)
        Rx = self.tx_power_dBm + self.antenna_gain * 2 - PL
        margin = Rx - self.rx_sensitivity
        return Rx, margin

    def range_estimation(self, margin_target=10):
        """计算可达距离 (留余量margin_target dB)"""
        d = np.logspace(0, 5, 1000)
        _, margin = self.link_budget(d)
        # 找margin=margin_target的位置
        idx = np.argmin(np.abs(margin - margin_target))
        return d[idx]


# ==================== 综合绘图 ====================
def plot_communication_range():
    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    fig.suptitle('无线通信距离仿真 - 链路预算与覆盖分析', fontsize=14)

    bt = BluetoothSim()
    wifi24 = WiFiSim('2.4GHz')
    wifi5 = WiFiSim('5GHz')

    # 1) 路径损耗对比
    ax = axes[0, 0]
    d = np.logspace(0, 3, 200)
    ax.semilogx(d, PropagationModel.free_space(d, 2440), 'b-', label='2.4GHz自由空间', linewidth=2)
    ax.semilogx(d, PropagationModel.log_distance(d, 2440, n=3.0), 'g-', label='2.4GHz室内(η=3)', linewidth=2)
    ax.semilogx(d, PropagationModel.log_distance(d, 2440, n=4.0), 'r-', label='2.4GHz密集室内(η=4)', linewidth=2)
    ax.semilogx(d, PropagationModel.itu_indoor(d, 2440, n_walls=3), 'm-', label='ITU室内(3墙)', linewidth=2)
    ax.set_xlabel('距离 (m)')
    ax.set_ylabel('路径损耗 (dB)')
    ax.set_title('不同环境的路径损耗模型')
    ax.legend(fontsize=9)
    ax.grid(True, which='both', alpha=0.3)

    # 2) 链路余量对比
    ax = axes[0, 1]
    d = np.logspace(0, 4, 200)

    # BLE
    _, margin_bt = bt.link_budget(d)
    ax.semilogx(d, margin_bt, 'g-', linewidth=2, label='BLE (0dBm, SF=3)')

    # WiFi 2.4GHz
    PL_wifi = PropagationModel.log_distance(d, wifi24.freq_mhz, n=3.2)
    Rx_wifi = wifi24.tx_power_dBm + wifi24.antenna_gain - PL_wifi
    margin_wifi = Rx_wifi - wifi24.rx_sensitivity
    ax.semilogx(d, margin_wifi, 'b-', linewidth=2, label='WiFi 2.4G (18dBm)')

    # WiFi 5GHz
    PL_wifi5 = PropagationModel.log_distance(d, wifi5.freq_mhz, n=3.5)
    Rx_wifi5 = wifi5.tx_power_dBm + wifi5.antenna_gain - PL_wifi5
    margin_wifi5 = Rx_wifi5 - wifi5.rx_sensitivity
    ax.semilogx(d, margin_wifi5, 'c-', linewidth=2, label='WiFi 5G (17dBm)')

    # LoRa SF7~SF12
    for sf in [7, 9, 12]:
        lora = LoRaSim(sf=sf)
        _, margin_lora = lora.link_budget(d)
        ax.semilogx(d, margin_lora, linewidth=2, label=f'LoRa SF{sf}', linestyle='--')

    ax.axhline(y=0, color='red', linestyle=':', linewidth=2, alpha=0.7)
    ax.set_xlabel('距离 (m)')
    ax.set_ylabel('链路余量 (dB)')
    ax.set_title('链路余量 vs 距离 (0dB以下=通信中断)')
    ax.legend(fontsize=8)
    ax.grid(True, which='both', alpha=0.3)
    ax.set_ylim(-20, 80)

    # 3) WiFi吞吐量 vs 距离
    ax = axes[1, 0]
    d_wifi = np.linspace(1, 100, 200)
    tp24, snr24 = wifi24.throughput_vs_distance(d_wifi)
    tp5, snr5 = wifi5.throughput_vs_distance(d_wifi)

    ax.plot(d_wifi, tp24, 'b-', linewidth=2, label='WiFi 2.4GHz')
    ax.plot(d_wifi, tp5, 'c-', linewidth=2, label='WiFi 5GHz')
    ax.set_xlabel('距离 (m)')
    ax.set_ylabel('吞吐量 (Mbps)')
    ax.set_title('WiFi吞吐量 vs 距离 (含MCS自适应)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 100)

    # 4) LoRa参数权衡 (距离 vs 速率 vs 功耗)
    ax = axes[1, 1]
    sf_range = range(7, 13)
    distances = []
    data_rates = []
    toa_list = []

    for sf in sf_range:
        lora = LoRaSim(sf=sf)
        d_est = lora.range_estimation(margin_target=10)
        distances.append(d_est)
        data_rates.append(lora.data_rate / 1000)  # kbps
        toa_list.append(lora.time_on_air)

    x = np.arange(len(sf_range))
    width = 0.35
    bars1 = ax.bar(x - width/2, [d/1000 for d in distances], width, label='通信距离 (km)', color='steelblue')
    ax2 = ax.twinx()
    bars2 = ax2.bar(x + width/2, data_rates, width, label='数据速率 (kbps)', color='coral', alpha=0.7)

    ax.set_xlabel('扩频因子 (SF)')
    ax.set_ylabel('通信距离 (km)', color='steelblue')
    ax2.set_ylabel('数据速率 (kbps)', color='coral')
    ax.set_xticks(x)
    ax.set_xticklabels([f'SF{s}' for s in sf_range])
    ax.set_title('LoRa SF vs 距离/速率权衡')
    ax.grid(True, alpha=0.3, axis='y')

    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='center left')

    plt.tight_layout()
    plt.savefig('communication_range.png', dpi=150, bbox_inches='tight')
    plt.show()


def demo():
    print("=" * 60)
    print("无线通信距离仿真 - 蓝牙/WiFi/LoRa")
    print("=" * 60)

    # 距离估算
    bt = BluetoothSim()
    d_bt = np.linspace(1, 100, 100)
    _, margin_bt = bt.link_budget(d_bt)
    valid_bt = d_bt[margin_bt > 0]
    print(f"\n=== BLE链路预算 ===")
    print(f"  发射功率: {bt.tx_power_dBm} dBm")
    print(f"  接收灵敏度: {bt.rx_sensitivity} dBm")
    print(f"  预计室内范围: ~{valid_bt[-1]:.0f}m (η=3)")

    wifi = WiFiSim('2.4GHz')
    print(f"\n=== WiFi 2.4GHz ===")
    print(f"  发射功率: {wifi.tx_power_dBm} dBm")
    print(f"  接收灵敏度: {wifi.rx_sensitivity} dBm")

    print(f"\n=== LoRa参数对比 ===")
    print(f"  {'SF':>4} {'距离(km)':>10} {'速率(kbps)':>12} {'灵敏度(dBm)':>14}")
    for sf in [7, 8, 9, 10, 11, 12]:
        lora = LoRaSim(sf=sf)
        d = lora.range_estimation(margin_target=10)
        print(f"  SF{sf:<2} {d/1000:>10.1f} {lora.data_rate/1000:>12.2f} {lora.rx_sensitivity:>14.1f}")

    plot_communication_range()
    print("\n仿真完成！")


if __name__ == '__main__':
    demo()
