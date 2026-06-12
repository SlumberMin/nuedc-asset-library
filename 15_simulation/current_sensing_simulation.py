#!/usr/bin/env python3
"""
电流检测仿真 (Current Sensing Simulation)
==========================================
仿真内容:
  - 分流电阻检测原理 (欧姆定律)
  - 高侧/低侧检测拓扑
  - 仪表放大器增益模型 (INA219/INA226)
  - 滤波器设计 (RC/有源滤波)
  - 过流保护逻辑
  - 温度漂移与精度分析
  - PWM电流纹波仿真

依赖: numpy, matplotlib, scipy (可选)
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Tuple


@dataclass
class ShuntResistor:
    """分流电阻参数"""
    resistance: float = 0.01       # 阻值 10mΩ
    tolerance: float = 0.01        # 精度 1%
    temp_coeff: float = 50e-6      # 温度系数 50ppm/°C (锰铜)
    power_rating: float = 1.0      # 额定功率 1W
    inductance: float = 5e-9       # 寄生电感 5nH
    tcr_2nd: float = 1e-6          # 二次温度系数


@dataclass
class AmplifierConfig:
    """放大器配置"""
    gain: float = 100              # 增益 100V/V
    gain_error: float = 0.001      # 增益误差 0.1%
    offset_voltage: float = 25e-6  # 输入失调电压 25μV
    offset_drift: float = 0.3e-6   # 失调漂移 0.3μV/°C
    cmrr_db: float = 120           # 共模抑制比 120dB
    bandwidth: float = 800e3       # 带宽 800kHz
    slew_rate: float = 2e6         # 压摆率 2V/μs
    supply_voltage: float = 3.3    # 供电电压
    noise_density: float = 10e-9   # 输入噪声密度 10nV/√Hz


@dataclass
class ADCConfig:
    """ADC配置"""
    bits: int = 12
    vref: float = 3.3
    sampling_rate: float = 1e6     # 1MSPS
    inl: float = 1.0               # INL ±1 LSB
    dnl: float = 0.5               # DNL ±0.5 LSB
    input_noise: float = 0.5e-3    # 输入噪声 0.5mV RMS


@dataclass
class FilterConfig:
    """滤波器配置"""
    rc_cutoff: float = 100e3       # RC截止频率 100kHz
    r_value: float = 1e3           # 电阻 1kΩ
    order: int = 2                 # 滤波器阶数


class CurrentSensingSimulation:
    """电流检测仿真引擎"""

    def __init__(self, shunt: ShuntResistor = None, amp: AmplifierConfig = None,
                 adc: ADCConfig = None, filt: FilterConfig = None):
        self.shunt = shunt or ShuntResistor()
        self.amp = amp or AmplifierConfig()
        self.adc = adc or ADCConfig()
        self.filt = filt or FilterConfig()

    # ── 基本检测模型 ─────────────────────────────────────
    def shunt_voltage(self, current: float, temperature: float = 25) -> float:
        """计算分流电阻上的压降 (含温度漂移)"""
        s = self.shunt
        # 温度修正
        dt = temperature - 25
        r_actual = s.resistance * (1 + s.temp_coeff * dt + s.tcr_2nd * dt ** 2)
        # 容差
        r_actual *= (1 + np.random.normal(0, s.tolerance / 3))  # 3σ
        return current * r_actual

    def amplifier_output(self, v_shunt: float, v_cm: float = 12,
                         temperature: float = 25) -> float:
        """放大器输出电压"""
        a = self.amp
        # 增益误差
        gain_actual = a.gain * (1 + np.random.normal(0, a.gain_error / 3))
        # 失调电压 + 温漂
        dt = temperature - 25
        v_os = a.offset_voltage + a.offset_drift * dt + np.random.normal(0, a.offset_voltage / 3)
        # CMRR影响
        cmrr_linear = 10 ** (a.cmrr_db / 20)
        v_cm_error = v_cm / cmrr_linear
        # 输出
        v_out = (v_shunt + v_os + v_cm_error) * gain_actual
        # 饱和限幅
        v_out = np.clip(v_out, 0, a.supply_voltage)
        return v_out

    def adc_conversion(self, v_analog: float) -> Tuple[int, float]:
        """ADC转换"""
        a = self.adc
        # 加噪声
        v_noisy = v_analog + np.random.normal(0, a.input_noise)
        # INL误差
        lsb = a.vref / (2 ** a.bits)
        inl_error = np.random.uniform(-a.inl, a.inl) * lsb
        v_noisy += inl_error
        # 量化
        v_noisy = np.clip(v_noisy, 0, a.vref)
        code = int(v_noisy / a.vref * (2 ** a.bits))
        code = min(code, 2 ** a.bits - 1)
        # 还原电压
        v_reconstructed = code * lsb
        return code, v_reconstructed

    def measure_current(self, current: float, v_cm: float = 12,
                        temperature: float = 25) -> dict:
        """完整测量链: 电流→分流电压→放大→滤波→ADC→数字值"""
        s = self.shunt
        a = self.amp

        # 分流电压
        v_shunt = self.shunt_voltage(current, temperature)
        # 功耗检查
        p_dissipated = current ** 2 * s.resistance
        # 放大
        v_amp = self.amplifier_output(v_shunt, v_cm, temperature)
        # ADC
        code, v_reconstructed = self.adc_conversion(v_amp)
        # 反推电流
        measured_current = v_reconstructed / a.gain / s.resistance

        return {
            "true_current": current,
            "measured_current": measured_current,
            "shunt_voltage": v_shunt,
            "amp_output": v_amp,
            "adc_code": code,
            "adc_voltage": v_reconstructed,
            "shunt_power": p_dissipated,
            "error_pct": abs(measured_current - current) / max(abs(current), 1e-10) * 100,
            "temperature": temperature,
        }

    # ── 频率响应 ──────────────────────────────────────────
    def frequency_response(self, freq_range: Tuple[float, float] = (1, 10e6),
                           n_points: int = 1000) -> dict:
        """计算检测链路的频率响应"""
        freqs = np.logspace(np.log10(freq_range[0]), np.log10(freq_range[1]), n_points)

        # 分流电阻 (含寄生电感)
        s = self.shunt
        z_shunt = np.sqrt(s.resistance ** 2 + (2 * np.pi * freqs * s.inductance) ** 2)

        # 放大器 (单极点)
        a = self.amp
        amp_gain = a.gain / np.sqrt(1 + (freqs / a.bandwidth) ** 2)

        # RC滤波器
        f = self.filt
        filter_gain = 1 / np.sqrt(1 + (freqs / f.rc_cutoff) ** 2) ** f.order

        # 总传递函数
        total_gain = z_shunt / s.resistance * amp_gain * filter_gain
        total_gain_db = 20 * np.log10(np.abs(total_gain) + 1e-20)

        return {
            "freqs": freqs,
            "shunt_z": z_shunt,
            "amp_gain": amp_gain,
            "filter_gain": filter_gain,
            "total_gain_db": total_gain_db,
        }

    # ── 过流保护 ──────────────────────────────────────────
    def overcurrent_protection(self, currents: np.ndarray,
                               threshold: float = 10.0,
                               response_time_us: float = 1.0) -> dict:
        """过流保护仿真"""
        n = len(currents)
        fault_detected = np.zeros(n, dtype=bool)
        protection_active = np.zeros(n, dtype=bool)
        output_current = np.copy(currents)

        # 检测延迟
        delay_samples = max(1, int(response_time_us * self.adc.sampling_rate / 1e6))

        fault_counter = 0
        for i in range(n):
            # 测量
            result = self.measure_current(currents[i])
            measured = result["measured_current"]

            if abs(measured) > threshold:
                fault_counter += 1
                if fault_counter >= 3:  # 连续3次触发
                    fault_detected[i] = True
            else:
                fault_counter = 0

            # 保护动作 (延迟后)
            if i >= delay_samples and fault_detected[i - delay_samples]:
                protection_active[i] = True
                output_current[i] = 0  # 关断

        return {
            "time": np.arange(n) / self.adc.sampling_rate * 1e6,
            "input_current": currents,
            "output_current": output_current,
            "fault_detected": fault_detected,
            "protection_active": protection_active,
            "threshold": threshold,
        }

    # ── PWM电流纹波 ──────────────────────────────────────
    def pwm_current_ripple(self, v_supply: float = 12, resistance: float = 10,
                           inductance: float = 1e-3, duty: float = 0.5,
                           pwm_freq: float = 20e3, n_cycles: int = 5) -> dict:
        """PWM驱动下的电流纹波仿真"""
        period = 1 / pwm_freq
        dt = period / 200  # 每周期200个采样点
        total_time = n_cycles * period
        n_points = int(total_time / dt)
        t = np.linspace(0, total_time, n_points)

        current = np.zeros(n_points)
        voltage = np.zeros(n_points)

        for i in range(1, n_points):
            phase = (t[i] % period) / period
            if phase < duty:
                v = v_supply
            else:
                v = 0  # 续流

            voltage[i] = v
            # L di/dt + Ri = V
            di_dt = (v - resistance * current[i - 1]) / inductance
            current[i] = current[i - 1] + di_dt * dt
            current[i] = max(current[i], 0)

        # 理论纹波
        delta_i_theory = v_supply * duty * (1 - duty) / (inductance * pwm_freq)

        return {
            "time": t * 1000,  # ms
            "current": current,
            "voltage": voltage,
            "ripple_measured": np.max(current[int(n_points * 0.5):]) - np.min(current[int(n_points * 0.5):]),
            "ripple_theory": delta_i_theory,
            "average_current": np.mean(current[int(n_points * 0.5):]),
            "pwm_freq": pwm_freq,
            "duty": duty,
        }


def run_demo():
    """运行完整仿真演示"""
    print("=" * 70)
    print("电流检测仿真")
    print("=" * 70)

    sim = CurrentSensingSimulation()
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    # ── 1. 传递特性 (线性度) ─────────────────────────────
    print("\n[1] 传递特性分析...")
    currents = np.linspace(0, 15, 200)
    measured = []
    errors = []
    for I in currents:
        res = sim.measure_current(I)
        measured.append(res["measured_current"])
        errors.append(res["error_pct"])

    ax = axes[0, 0]
    ax.plot(currents, measured, 'b-', label='测量值')
    ax.plot(currents, currents, 'r--', label='理想值')
    ax.set_xlabel('真实电流 (A)')
    ax.set_ylabel('测量电流 (A)')
    ax.set_title('电流检测传递特性')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax2 = ax.twinx()
    ax2.plot(currents, errors, 'g-', alpha=0.5, label='误差%')
    ax2.set_ylabel('相对误差 (%)')
    ax2.legend(loc='upper right')

    # ── 2. 频率响应 ──────────────────────────────────────
    print("[2] 频率响应...")
    fr = sim.frequency_response()

    ax = axes[0, 1]
    ax.semilogx(fr["freqs"], fr["total_gain_db"], 'b-', linewidth=2, label='总增益')
    ax.semilogx(fr["freqs"], 20 * np.log10(fr["amp_gain"] + 1e-20), 'r--', label='放大器')
    ax.semilogx(fr["freqs"], 20 * np.log10(fr["filter_gain"] + 1e-20), 'g--', label='滤波器')
    ax.axhline(-3, color='k', linestyle=':', alpha=0.5)
    ax.set_xlabel('频率 (Hz)')
    ax.set_ylabel('增益 (dB)')
    ax.set_title('检测链路频率响应')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(1, 10e6)

    # ── 3. 温度影响 ──────────────────────────────────────
    print("[3] 温度影响分析...")
    temps = np.linspace(-40, 125, 50)
    errors_by_temp = []
    for t in temps:
        errs = [sim.measure_current(5.0, temperature=t)["error_pct"] for _ in range(20)]
        errors_by_temp.append(np.mean(errs))

    ax = axes[0, 2]
    ax.plot(temps, errors_by_temp, 'r-o', markersize=3)
    ax.set_xlabel('温度 (°C)')
    ax.set_ylabel('测量误差 (%)')
    ax.set_title('温度对测量精度的影响 (I=5A)')
    ax.grid(True, alpha=0.3)

    # ── 4. 过流保护仿真 ──────────────────────────────────
    print("[4] 过流保护仿真...")
    n_points = 5000
    t = np.arange(n_points) / 1e6 * 1e6  # μs
    # 电流波形: 正常→过流→恢复
    test_current = np.ones(n_points) * 5.0
    test_current[1000:1500] = 15.0   # 过流事件
    test_current[3000:3500] = 18.0   # 更严重的过流
    # 加噪声
    test_current += np.random.normal(0, 0.1, n_points)

    prot = sim.overcurrent_protection(test_current, threshold=10.0, response_time_us=2)

    ax = axes[1, 0]
    ax.plot(prot["time"], prot["input_current"], 'b-', alpha=0.7, label='实际电流')
    ax.plot(prot["time"], prot["output_current"], 'r-', linewidth=2, label='保护后输出')
    ax.axhline(10, color='orange', linestyle='--', label='阈值=10A')
    ax.fill_between(prot["time"], 0, 20,
                    where=prot["protection_active"], alpha=0.2, color='red', label='保护动作')
    ax.set_xlabel('时间 (μs)')
    ax.set_ylabel('电流 (A)')
    ax.set_title('过流保护仿真')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 22)

    # ── 5. PWM电流纹波 ──────────────────────────────────
    print("[5] PWM电流纹波...")
    ripple = sim.pwm_current_ripple(duty=0.5, pwm_freq=20e3)

    ax = axes[1, 1]
    ax.plot(ripple["time"], ripple["current"], 'b-', linewidth=0.8)
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('电流 (A)')
    ax.set_title(f'PWM电流纹波 (f={ripple["pwm_freq"]/1e3:.0f}kHz, D={ripple["duty"]*100:.0f}%)\n'
                 f'纹波实测={ripple["ripple_measured"]:.3f}A, 理论={ripple["ripple_theory"]:.3f}A')
    ax.grid(True, alpha=0.3)

    ax2 = ax.twinx()
    ax2.plot(ripple["time"], ripple["voltage"], 'r-', alpha=0.3, linewidth=0.5)
    ax2.set_ylabel('电压 (V)', color='red')

    # ── 6. 不同分流电阻的精度对比 ────────────────────────
    print("[6] 分流电阻对比...")
    shunt_values = [1e-3, 5e-3, 10e-3, 20e-3, 50e-3, 100e-3]
    test_currents = [0.1, 1, 5, 10]
    error_matrix = np.zeros((len(test_currents), len(shunt_values)))

    for i, I_test in enumerate(test_currents):
        for j, R_shunt in enumerate(shunt_values):
            sim_temp = CurrentSensingSimulation(
                shunt=ShuntResistor(resistance=R_shunt),
                amp=AmplifierConfig(gain=min(3.3 / (I_test * R_shunt * 1.1), 1000))
            )
            errs = [sim_temp.measure_current(I_test)["error_pct"] for _ in range(30)]
            error_matrix[i, j] = np.mean(errs)

    ax = axes[1, 2]
    x = np.arange(len(shunt_values))
    width = 0.2
    for i, I_test in enumerate(test_currents):
        ax.bar(x + i * width, error_matrix[i, :], width, label=f'{I_test}A')
    ax.set_xticks(x + width * 1.5)
    ax.set_xticklabels([f'{r * 1000:.0f}mΩ' for r in shunt_values])
    ax.set_xlabel('分流电阻值')
    ax.set_ylabel('平均误差 (%)')
    ax.set_title('不同分流电阻的测量精度')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('current_sensing_simulation.png', dpi=150, bbox_inches='tight')
    plt.show()

    print(f"\n仿真完成!")
    print(f"  典型测量 (5A, 25°C): 误差 {sim.measure_current(5.0)['error_pct']:.3f}%")
    print(f"  频率响应 -3dB带宽: ~{fr['freqs'][np.argmin(np.abs(fr['total_gain_db'] + 3))]/1e3:.0f}kHz")
    print("  图表已保存为 current_sensing_simulation.png")


if __name__ == "__main__":
    run_demo()
