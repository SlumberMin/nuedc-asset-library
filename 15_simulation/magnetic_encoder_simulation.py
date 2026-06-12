#!/usr/bin/env python3
"""
磁编码器仿真 (Magnetic Encoder Simulation)
============================================
仿真内容:
  - 旋转磁铁的磁场分布 (偶极子模型)
  - 霍尔传感器阵列信号生成
  - 角度解码算法 (反正切法 + 查表法)
  - 误差分析: 偏心误差、倾斜误差、温度漂移、非线性误差
  - ABZ正交编码器输出仿真

依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Tuple


# ── 常量 ──────────────────────────────────────────────────
MU_0 = 4 * np.pi * 1e-7  # 真空磁导率


@dataclass
class MagnetConfig:
    """磁铁配置"""
    remanence: float = 1.2       # 剩磁 T (NdFeB ~1.2T)
    radius: float = 5e-3         # 磁铁半径 5mm
    thickness: float = 3e-3      # 磁铁厚度 3mm
    pole_pairs: int = 1          # 极对数
    temperature_coeff: float = -0.12  # 温度系数 %/°C


@dataclass
class SensorConfig:
    """传感器配置"""
    n_sensors: int = 4           # 传感器数量 (正交排列)
    sensor_radius: float = 8e-3  # 传感器安装半径 8mm
    sensitivity: float = 50e-3   # 灵敏度 50mV/mT
    offset_voltage: float = 2.5  # 零点电压 2.5V (单电源)
    noise_density: float = 5e-6  # 噪声密度 V/√Hz
    adc_bits: int = 12           # ADC分辨率
    adc_vref: float = 3.3        # ADC参考电压


@dataclass
class ErrorConfig:
    """误差源配置"""
    eccentricity: float = 0.1e-3     # 偏心距 0.1mm
    tilt_angle: float = np.deg2rad(2) # 倾斜角 2°
    temperature_offset: float = 25    # 参考温度 25°C
    nonlinearity_pct: float = 0.5    # 非线性度 0.5%


class MagneticEncoderSimulation:
    """磁编码器仿真引擎"""

    def __init__(self, magnet: MagnetConfig = None, sensor: SensorConfig = None,
                 errors: ErrorConfig = None):
        self.magnet = magnet or MagnetConfig()
        self.sensor = sensor or SensorConfig()
        self.errors = errors or ErrorConfig()

    # ── 磁场模型 ──────────────────────────────────────────
    def dipole_field(self, angle: float, sensor_angle: float,
                     r_sensor: float, z_gap: float = 2e-3) -> Tuple[float, float]:
        """计算偶极子磁场在传感器位置的径向和切向分量
        angle: 磁铁旋转角 (rad)
        sensor_angle: 传感器安装角 (rad)
        r_sensor: 传感器安装半径
        z_gap: 气隙距离
        """
        mag = self.magnet
        # 磁矩方向
        theta = angle * mag.pole_pairs

        # 传感器相对于磁铁中心的极坐标
        phi = sensor_angle - angle  # 相对角

        # 等效磁偶极矩
        m = mag.remanence * np.pi * mag.radius ** 2 * mag.thickness / MU_0

        # 到传感器的距离
        r = np.sqrt(r_sensor ** 2 + z_gap ** 2)

        # 偶极子磁场分量 (简化为面磁铁, 多极对)
        Br = (MU_0 / (2 * np.pi)) * m * np.cos(mag.pole_pairs * phi) / r ** 3 * z_gap
        Bt = (MU_0 / (2 * np.pi)) * m * np.sin(mag.pole_pairs * phi) / r ** 3 * r_sensor

        return Br, Bt

    def generate_sensor_signals(self, angle: float, temperature: float = 25,
                                add_errors: bool = True) -> np.ndarray:
        """生成传感器阵列信号"""
        cfg = self.sensor
        errors = self.errors
        signals = np.zeros(cfg.n_sensors)

        for i in range(cfg.n_sensors):
            sensor_angle = 2 * np.pi * i / cfg.n_sensors

            # 偏心误差
            if add_errors and errors.eccentricity > 0:
                ecc_offset = errors.eccentricity * np.cos(sensor_angle) / self.sensor.sensor_radius
                effective_angle = sensor_angle + ecc_offset
            else:
                effective_angle = sensor_angle

            # 倾斜误差
            tilt_factor = 1.0
            if add_errors and errors.tilt_angle > 0:
                tilt_factor = np.cos(errors.tilt_angle) * (
                    1 + 0.1 * np.sin(errors.tilt_angle) * np.cos(sensor_angle))

            # 计算磁场
            Br, Bt = self.dipole_field(angle, effective_angle, self.sensor.sensor_radius)
            B_total = np.sqrt(Br ** 2 + Bt ** 2) * np.sign(Br) * tilt_factor

            # 霍尔电压
            V_hall = cfg.offset_voltage + cfg.sensitivity * B_total * 1000  # mT

            # 温度漂移
            if add_errors:
                temp_drift = self.magnet.temperature_coeff / 100 * (temperature - errors.temperature_offset)
                V_hall *= (1 + temp_drift)

                # 非线性误差
                if errors.nonlinearity_pct > 0:
                    V_hall += V_hall * errors.nonlinearity_pct / 100 * np.sin(3 * angle)

                # 噪声
                V_hall += np.random.normal(0, cfg.noise_density * np.sqrt(1e6))

            # ADC量化
            V_hall = np.clip(V_hall, 0, cfg.adc_vref)
            if cfg.adc_bits > 0:
                V_hall = np.round(V_hall / cfg.adc_vref * (2 ** cfg.adc_bits)) * cfg.adc_vref / (2 ** cfg.adc_bits)

            signals[i] = V_hall

        return signals

    # ── 角度解码 ──────────────────────────────────────────
    def decode_angle_atan2(self, signals: np.ndarray) -> float:
        """反正切法解码角度"""
        cfg = self.sensor
        # 归一化到 [-1, 1]
        s = (signals - cfg.offset_voltage)
        # 两路正交信号 (X, Y)
        if cfg.n_sensors >= 2:
            X = s[0] - s[2] if cfg.n_sensors >= 4 else s[0]
            Y = s[1] - s[3] if cfg.n_sensors >= 4 else s[1]
        else:
            X, Y = s[0], 0

        angle = np.arctan2(Y, X) / self.magnet.pole_pairs
        return angle % (2 * np.pi)

    def decode_angle_lut(self, signals: np.ndarray, lut_size: int = 3600) -> float:
        """查表法解码角度 (基于预标定)"""
        # 生成查找表
        if not hasattr(self, '_lut_angles') or len(self._lut_angles) != lut_size:
            self._lut_angles = np.linspace(0, 2 * np.pi, lut_size, endpoint=False)
            self._lut_signals = np.array([
                self.generate_sensor_signals(a, add_errors=False) for a in self._lut_angles
            ])

        # 最小距离匹配
        diff = self._lut_signals - signals
        distances = np.sum(diff ** 2, axis=1)
        best_idx = np.argmin(distances)
        return self._lut_angles[best_idx]

    # ── ABZ编码器输出 ─────────────────────────────────────
    def abz_output(self, angle: float, ppr: int = 1024) -> Tuple[int, int, int]:
        """生成ABZ正交编码信号
        ppr: 每转脉冲数
        """
        pulses_per_rev = ppr * self.magnet.pole_pairs
        pulse_angle = 2 * np.pi / pulses_per_rev

        # A相
        A = int((angle % (2 * np.pi)) / pulse_angle) % 2
        # B相 (滞后90°)
        B = int(((angle % (2 * np.pi)) + pulse_angle / 2) / pulse_angle) % 2
        # Z相 (每转一个)
        Z = 1 if (angle % (2 * np.pi)) < pulse_angle else 0

        return A, B, Z

    # ── 全角度扫描分析 ────────────────────────────────────
    def full_rotation_analysis(self, n_points: int = 3600,
                               temperature: float = 25) -> dict:
        """全角度扫描, 分析误差"""
        angles_true = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
        angles_decoded_atan = np.zeros(n_points)
        angles_decoded_lut = np.zeros(n_points)
        all_signals = np.zeros((n_points, self.sensor.n_sensors))

        for i, angle in enumerate(angles_true):
            signals = self.generate_sensor_signals(angle, temperature)
            all_signals[i] = signals
            angles_decoded_atan[i] = self.decode_angle_atan2(signals)
            angles_decoded_lut[i] = self.decode_angle_lut(signals)

        # 误差计算 (处理环绕)
        error_atan = (angles_decoded_atan - angles_true)
        error_atan = (error_atan + np.pi) % (2 * np.pi) - np.pi
        error_atan_deg = np.rad2deg(error_atan)

        error_lut = (angles_decoded_lut - angles_true)
        error_lut = (error_lut + np.pi) % (2 * np.pi) - np.pi
        error_lut_deg = np.rad2deg(error_lut)

        return {
            "angles_true": angles_true,
            "angles_decoded_atan": angles_decoded_atan,
            "angles_decoded_lut": angles_decoded_lut,
            "signals": all_signals,
            "error_atan_deg": error_atan_deg,
            "error_lut_deg": error_lut_deg,
            "rms_error_atan": np.sqrt(np.mean(error_atan_deg ** 2)),
            "rms_error_lut": np.sqrt(np.mean(error_lut_deg ** 2)),
            "max_error_atan": np.max(np.abs(error_atan_deg)),
            "max_error_lut": np.max(np.abs(error_lut_deg)),
            "temperature": temperature,
        }


def run_demo():
    """运行完整仿真演示"""
    print("=" * 70)
    print("磁编码器仿真")
    print("=" * 70)

    encoder = MagneticEncoderSimulation()
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    # ── 1. 传感器信号波形 ────────────────────────────────
    print("\n[1] 传感器信号波形...")
    angles = np.linspace(0, 4 * np.pi, 1000)
    signals_0 = [encoder.generate_sensor_signals(a, add_errors=False) for a in angles]
    signals_e = [encoder.generate_sensor_signals(a, add_errors=True) for a in angles]

    ax = axes[0, 0]
    for ch in range(4):
        ax.plot(np.rad2deg(angles), [s[ch] for s in signals_0],
                '-', alpha=0.7, label=f'CH{ch} (理想)')
        ax.plot(np.rad2deg(angles), [s[ch] for s in signals_e],
                '--', alpha=0.5, label=f'CH{ch} (含误差)')
    ax.set_xlabel('旋转角度 (°)')
    ax.set_ylabel('传感器电压 (V)')
    ax.set_title('传感器信号波形 (2转)')
    ax.legend(fontsize=6, ncol=2)
    ax.grid(True, alpha=0.3)

    # ── 2. 李萨如图 (XY散点图) ───────────────────────────
    print("[2] 李萨如图形...")
    ax = axes[0, 1]
    s_arr = np.array(signals_0)
    ax.plot(s_arr[:, 0] - 2.5, s_arr[:, 1] - 2.5, 'b-', alpha=0.5, label='理想')
    s_arr_e = np.array(signals_e)
    ax.plot(s_arr_e[:, 0] - 2.5, s_arr_e[:, 1] - 2.5, 'r.', alpha=0.2, markersize=1, label='含误差')
    ax.set_xlabel('X (CH0 - Vref)')
    ax.set_ylabel('Y (CH1 - Vref)')
    ax.set_title('李萨如图形 (XY散点)')
    ax.set_aspect('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ── 3. 角度误差分析 ──────────────────────────────────
    print("[3] 角度误差分析...")
    result = encoder.full_rotation_analysis(3600, temperature=25)

    ax = axes[0, 2]
    ax.plot(np.rad2deg(result["angles_true"]), result["error_atan_deg"],
            'b-', alpha=0.7, label=f'反正切法 (RMS={result["rms_error_atan"]:.2f}°)')
    ax.plot(np.rad2deg(result["angles_true"]), result["error_lut_deg"],
            'r-', alpha=0.7, label=f'查表法 (RMS={result["rms_error_lut"]:.2f}°)')
    ax.set_xlabel('真实角度 (°)')
    ax.set_ylabel('角度误差 (°)')
    ax.set_title('角度解码误差')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ── 4. 温度漂移影响 ──────────────────────────────────
    print("[4] 温度漂移影响...")
    temps = [-40, -20, 0, 25, 60, 85, 125]
    rms_by_temp = []
    for t in temps:
        res = encoder.full_rotation_analysis(1800, temperature=t)
        rms_by_temp.append(res["rms_error_atan"])

    ax = axes[1, 0]
    ax.plot(temps, rms_by_temp, 'ro-', linewidth=2, markersize=6)
    ax.set_xlabel('温度 (°C)')
    ax.set_ylabel('RMS角度误差 (°)')
    ax.set_title('温度漂移对精度的影响')
    ax.grid(True, alpha=0.3)

    # ── 5. ABZ正交编码 ──────────────────────────────────
    print("[5] ABZ正交编码输出...")
    abz_angles = np.linspace(0, 0.02 * np.pi, 500)
    A_list, B_list, Z_list = [], [], []
    for a in abz_angles:
        A, B, Z = encoder.abz_output(a, ppr=512)
        A_list.append(A)
        B_list.append(B)
        Z_list.append(Z)

    ax = axes[1, 1]
    ax.plot(np.rad2deg(abz_angles), A_list, 'b-', label='A相', linewidth=1.5)
    ax.plot(np.rad2deg(abz_angles), [b + 1.2 for b in B_list], 'r-', label='B相 (+1.2)', linewidth=1.5)
    ax.plot(np.rad2deg(abz_angles), [z + 2.4 for z in Z_list], 'g-', label='Z相 (+2.4)', linewidth=1.5)
    ax.set_xlabel('角度 (°)')
    ax.set_ylabel('电平')
    ax.set_title('ABZ正交编码器输出')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_yticks([0, 1, 1.2, 2.2, 2.4, 3.4])
    ax.set_yticklabels(['0', '1', '0', '1', '0', '1'])

    # ── 6. 频谱分析 (INL/DNL) ────────────────────────────
    print("[6] 误差频谱分析...")
    error_fft = np.fft.fft(result["error_atan_deg"])
    freqs = np.fft.fftfreq(len(error_fft), d=1 / len(error_fft))
    magnitude = np.abs(error_fft) / len(error_fft)

    ax = axes[1, 2]
    mask = freqs > 0
    ax.semilogy(freqs[mask], magnitude[mask] * 2, 'b-', linewidth=0.8)
    ax.set_xlabel('谐波次数 (机械周期)')
    ax.set_ylabel('幅度 (°)')
    ax.set_title('角度误差频谱分析')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 50)

    # 标注主要谐波
    peak_idx = np.argsort(magnitude[1:len(magnitude) // 2])[-3:] + 1
    for idx in peak_idx:
        ax.annotate(f'{freqs[idx]:.0f}阶\n{magnitude[idx] * 2:.3f}°',
                    xy=(freqs[idx], magnitude[idx] * 2),
                    fontsize=8, ha='center', va='bottom',
                    arrowprops=dict(arrowstyle='->', color='red'))

    plt.tight_layout()
    plt.savefig('magnetic_encoder_simulation.png', dpi=150, bbox_inches='tight')
    plt.show()
    print(f"\n仿真完成!")
    print(f"  反正切法 RMS误差: {result['rms_error_atan']:.3f}°, 最大: {result['max_error_atan']:.3f}°")
    print(f"  查表法   RMS误差: {result['rms_error_lut']:.3f}°, 最大: {result['max_error_lut']:.3f}°")
    print("  图表已保存为 magnetic_encoder_simulation.png")


if __name__ == "__main__":
    run_demo()
