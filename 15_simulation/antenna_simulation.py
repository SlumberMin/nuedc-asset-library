"""
天线仿真模块 - Antenna Simulation
====================================
功能: 偶极子方向图、阵列因子、增益、阻抗匹配仿真
适用: 电赛天线设计与射频前端辅助
"""

import numpy as np
from scipy.special import spherical_jn, spherical_yn
from typing import Tuple, List, Optional
import warnings
warnings.filterwarnings('ignore')


# ─────────────────────────────────────────────
# 1. 偶极子天线方向图
# ─────────────────────────────────────────────

class DipoleAntenna:
    """偶极子天线仿真"""
    
    def __init__(self, length: float, frequency: float, impedance: float = 73.0):
        """
        Args:
            length: 天线长度 (m)
            frequency: 工作频率 (Hz)
            impedance: 馈电阻抗 (Ω), 默认半波偶极子73Ω
        """
        self.L = length
        self.f = frequency
        self.c = 3e8
        self.wavelength = self.c / frequency
        self.k = 2 * np.pi / self.wavelength
        self.Z0 = impedance
    
    def radiation_pattern(self, theta: np.ndarray) -> np.ndarray:
        """
        偶极子天线辐射方向图 E(θ)
        
        Args:
            theta: 俯仰角数组 (rad), 0=天线轴向
        Returns:
            归一化辐射强度
        """
        # 通用偶极子公式
        kL = self.k * self.L / 2
        
        if abs(kL) < 1e-10:
            return np.ones_like(theta)
        
        # E(θ) = [cos(kL*cos(θ)) - cos(kL)] / sin(θ)
        sin_theta = np.sin(theta)
        numerator = np.cos(kL * np.cos(theta)) - np.cos(kL)
        
        # 避免 sin(θ)=0 处的奇点
        E = np.where(np.abs(sin_theta) < 1e-10, 0, numerator / sin_theta)
        E = np.abs(E)
        E_max = E.max()
        if E_max > 0:
            E = E / E_max
        return E
    
    def gain_pattern(self, theta: np.ndarray, phi: np.ndarray = None) -> np.ndarray:
        """
        增益方向图 G(θ,φ)
        偶极子天线在φ方向是均匀的
        """
        E = self.radiation_pattern(theta)
        # 辐射强度 U = U_max * |E|²
        U = E**2
        return U
    
    def directivity(self) -> float:
        """
        方向性系数 D
        半波偶极子: D ≈ 1.64 (2.15 dBi)
        """
        E_pattern = lambda theta: self.radiation_pattern(np.array([theta]))[0]
        
        # 数值积分 D = 4π*U_max / P_rad
        theta = np.linspace(0.01, np.pi - 0.01, 1000)
        U = self.radiation_pattern(theta)**2
        P_rad = np.trapz(U * np.sin(theta), theta) * 2 * np.pi
        D = 4 * np.pi * U.max() / P_rad if P_rad > 0 else 1.5
        return D
    
    def gain_dBi(self, efficiency: float = 0.95) -> float:
        """增益 (dBi)"""
        D = self.directivity()
        G = D * efficiency
        return 10 * np.log10(G)
    
    def input_impedance(self) -> complex:
        """
        输入阻抗 (简化模型)
        半波偶极子: R_in ≈ 73Ω, X_in ≈ 42.5Ω
        """
        kL = self.k * self.L
        
        # 辐射电阻 (近似)
        if abs(kL - np.pi) < 0.1:  # 半波偶极子附近
            R_rad = 73.0
            X_rad = 42.5
        elif abs(kL - np.pi/2) < 0.1:  # 1/4波长
            R_rad = 36.5
            X_rad = 21.25
        else:
            # 通用近似
            R_rad = 20 * (np.pi * self.L / self.wavelength)**2
            X_rad = -120 / (np.tan(kL) + 1e-10)
        
        return complex(R_rad, X_rad)
    
    def vswr(self, Z_load: complex = None, Z_line: float = 50.0) -> float:
        """
        电压驻波比 VSWR
        
        Args:
            Z_load: 负载阻抗 (默认天线输入阻抗)
            Z_line: 传输线特性阻抗
        """
        Z_L = Z_load if Z_load is not None else self.input_impedance()
        Gamma = (Z_L - Z_line) / (Z_L + Z_line)
        rho = abs(Gamma)
        if rho >= 1:
            return float('inf')
        return (1 + rho) / (1 - rho)
    
    def return_loss_dB(self, Z_line: float = 50.0) -> float:
        """回波损耗 (dB)"""
        Z_L = self.input_impedance()
        Gamma = (Z_L - Z_line) / (Z_L + Z_line)
        return -20 * np.log10(abs(Gamma)) if abs(Gamma) > 0 else float('inf')
    
    def effective_length(self) -> float:
        """有效长度"""
        kL = self.k * self.L
        return self.wavelength / np.pi * abs(np.sin(kL/2)) if abs(np.sin(kL/2)) > 0 else 0


class ShortDipole(DipoleAntenna):
    """短偶极子 (L << λ)"""
    
    def __init__(self, length: float, frequency: float):
        super().__init__(length, frequency)
        if length > self.wavelength / 10:
            warnings.warn(f"长度 {length*1000:.1f}mm > λ/10={self.wavelength/10*1000:.1f}mm, "
                         f"短偶极子模型不准确")
    
    def radiation_pattern(self, theta: np.ndarray) -> np.ndarray:
        """短偶极子: E(θ) = sin(θ)"""
        return np.abs(np.sin(theta))
    
    def radiation_resistance(self) -> float:
        """辐射电阻 R_r = 20*(πL/λ)²"""
        return 20 * (np.pi * self.L / self.wavelength)**2


class MonopoleAntenna(DipoleAntenna):
    """单极子天线 (地面上)"""
    
    def __init__(self, length: float, frequency: float, ground_loss: float = 0.5):
        super().__init__(length * 2, frequency)  # 等效偶极子长度加倍
        self.monopole_length = length
        self.ground_loss = ground_loss
    
    def radiation_pattern(self, theta: np.ndarray) -> np.ndarray:
        """单极子: 只有上半空间, E(θ) = sin(θ) for θ ∈ [0, π/2]"""
        # 等效偶极子的方向图在上半空间
        E = super().radiation_pattern(theta)
        # 地面以下为0
        E = np.where(theta <= np.pi/2, E, 0)
        return E
    
    def input_impedance(self) -> complex:
        """单极子输入阻抗 = 偶极子的一半"""
        Z_dipole = super().input_impedance()
        return Z_dipole / 2
    
    def gain_dBi(self, efficiency: float = None) -> float:
        """单极子增益比偶极子高3dB (半空间辐射)"""
        eff = efficiency if efficiency else (1 - self.ground_loss)
        D = self.directivity() * 2  # 半空间辐射
        return 10 * np.log10(D * eff)


# ─────────────────────────────────────────────
# 2. 天线阵列
# ─────────────────────────────────────────────

class AntennaArray:
    """天线阵列仿真"""
    
    def __init__(self, frequency: float, element_spacing: float,
                 num_elements: int = 4):
        """
        Args:
            frequency: 工作频率 (Hz)
            element_spacing: 阵元间距 (m)
            num_elements: 阵元数
        """
        self.f = frequency
        self.c = 3e8
        self.wl = self.c / frequency
        self.d = element_spacing
        self.N = num_elements
        self.k = 2 * np.pi / self.wl
        
        # 默认均匀激励
        self.weights = np.ones(num_elements, dtype=complex)
        self.phases = np.zeros(num_elements)
    
    def set_weights(self, amplitudes: np.ndarray = None, phases: np.ndarray = None):
        """设置阵元激励"""
        if amplitudes is not None:
            self.weights = amplitudes.astype(complex)
        if phases is not None:
            self.phases = phases
            self.weights = np.abs(self.weights) * np.exp(1j * phases)
    
    def array_factor_1d(self, theta: np.ndarray) -> np.ndarray:
        """
        一维线阵阵列因子 AF(θ)
        
        Args:
            theta: 俯仰角数组 (rad)
        Returns:
            阵列因子 (复数)
        """
        AF = np.zeros(len(theta), dtype=complex)
        for n in range(self.N):
            phase = self.k * n * self.d * np.cos(theta) + np.angle(self.weights[n])
            AF += np.abs(self.weights[n]) * np.exp(1j * phase)
        return AF
    
    def array_factor_2d(self, theta: np.ndarray, phi: np.ndarray) -> np.ndarray:
        """
        二维面阵阵列因子
        theta: (M,) 俯仰角
        phi: (K,) 方位角
        Returns: (M, K) 阵列因子
        """
        THETA, PHI = np.meshgrid(theta, phi, indexing='ij')
        AF = np.zeros_like(THETA, dtype=complex)
        
        # 假设均匀矩形阵列 (简化为N元线阵在θ方向)
        for n in range(self.N):
            phase = self.k * n * self.d * np.sin(THETA) * np.cos(PHI)
            AF += self.weights[n] * np.exp(1j * phase)
        
        return AF
    
    def beam_direction(self, scan_angle_rad: float):
        """
        设置波束指向 (相控阵扫描)
        
        Args:
            scan_angle_rad: 波束指向角 (rad), 0=法向
        """
        phases = np.array([self.k * n * self.d * np.sin(scan_angle_rad) 
                          for n in range(self.N)])
        self.phases = phases
        self.weights = np.abs(self.weights) * np.exp(1j * phases)
    
    def half_power_beamwidth(self) -> float:
        """半功率波束宽度 (rad), 均匀线阵近似"""
        # θ_3dB ≈ 0.886*λ/(N*d) (radians)
        return 0.886 * self.wl / (self.N * self.d)
    
    def sidelobe_level(self) -> float:
        """第一副瓣电平 (dB), 均匀线阵"""
        return -13.2  # 均匀线阵理论值
    
    def directivity(self) -> float:
        """阵列方向性 D ≈ 2*N*d/λ (窄波束近似)"""
        if self.d / self.wl < 0.5:
            return 2 * self.N * self.d / self.wl
        return self.N
    
    def array_gain_dBi(self) -> float:
        """阵列增益 (dBi), 假设单元增益0dBi"""
        D = self.directivity()
        return 10 * np.log10(D)


class UniformLinearArray(AntennaArray):
    """均匀线阵 (ULA) - 带权重设计方法"""
    
    @staticmethod
    def chebyshev_weights(num_elements: int, sidelobe_dB: float) -> np.ndarray:
        """
        切比雪夫加权 (低副瓣)
        
        Args:
            num_elements: 阵元数
            sidelobe_dB: 副瓣电平 (dB)
        Returns:
            加权系数
        """
        from scipy.special import chebyt
        
        N = num_elements
        R = 10**(abs(sidelobe_dB) / 20)
        
        # 计算Chebyshev权值
        x0 = np.cosh(np.arccosh(R) / (N - 1))
        
        weights = np.zeros(N)
        for m in range(N):
            s = 0
            for q in range(N):
                psi_q = 2 * np.pi * q / N
                xd = x0 * np.cos(psi_q / 2)
                if abs(xd) <= 1:
                    Tn = np.cos((N-1) * np.arccos(xd))
                else:
                    Tn = np.cosh((N-1) * np.arccosh(xd))
                s += Tn * np.cos(m * psi_q)
            weights[m] = s
        
        # 归一化
        weights = weights / weights.max()
        return np.abs(weights)
    
    @staticmethod
    def taylor_weights(num_elements: int, nbar: int = 4,
                       sidelobe_dB: float = -30) -> np.ndarray:
        """
        Taylor加权
        
        Args:
            num_elements: 阵元数
            nbar: Taylor参数
            sidelobe_dB: 副瓣电平
        Returns:
            加权系数
        """
        N = num_elements
        A = np.arccosh(10**(abs(sidelobe_dB)/20)) / np.pi
        
        weights = np.ones(N)
        for m in range(1, nbar):
            # 计算修正的零点位置
            sigma2 = nbar**2 / (A**2 + (nbar - 0.5)**2)
            numerator = 1
            denominator = 1
            for p in range(1, nbar):
                num = 1 - m**2 / (sigma2 * (A**2 + (p - 0.5)**2))
                den = 1 - m**2 / p**2
                if abs(den) > 1e-10:
                    numerator *= num
                    denominator *= den
            
            if abs(denominator) > 1e-10:
                Fm = numerator / denominator
            else:
                Fm = 1
            
            for n in range(N):
                weights[n] += 2 * Fm * np.cos(2 * np.pi * m * (n - (N-1)/2) / N)
        
        weights = weights / weights.max()
        return np.abs(weights)
    
    @staticmethod
    def binomial_weights(num_elements: int) -> np.ndarray:
        """二项式加权 (无副瓣, 但主瓣宽)"""
        from scipy.special import comb
        N = num_elements
        weights = np.array([comb(N-1, k, exact=True) for k in range(N)])
        return weights / weights.max()


# ─────────────────────────────────────────────
# 3. 阻抗匹配网络
# ─────────────────────────────────────────────

class ImpedanceMatching:
    """天线阻抗匹配网络设计"""
    
    def __init__(self, Z_load: complex, Z0: float = 50.0, frequency: float = 1e9):
        """
        Args:
            Z_load: 负载阻抗 (天线输入阻抗)
            Z0: 系统特性阻抗
            frequency: 工作频率
        """
        self.Z_L = Z_load
        self.Z0 = Z0
        self.f = frequency
        self.wl = 3e8 / frequency
    
    def reflection_coefficient(self) -> complex:
        """反射系数 Γ"""
        return (self.Z_L - self.Z0) / (self.Z_L + self.Z0)
    
    def vswr(self) -> float:
        """VSWR"""
        rho = abs(self.reflection_coefficient())
        if rho >= 1:
            return float('inf')
        return (1 + rho) / (1 - rho)
    
    def return_loss_dB(self) -> float:
        """回波损耗"""
        rho = abs(self.reflection_coefficient())
        return -20 * np.log10(rho) if rho > 0 else float('inf')
    
    def mismatch_loss_dB(self) -> float:
        """失配损耗"""
        rho = abs(self.reflection_coefficient())
        return -10 * np.log10(1 - rho**2) if rho < 1 else float('inf')
    
    def l_network_design(self) -> dict:
        """
        L型匹配网络设计
        
        Returns:
            dict: 包含匹配网络元件值
        """
        ZL = self.Z_L
        Z0 = self.Z0
        omega = 2 * np.pi * self.f
        
        RL = ZL.real
        XL = ZL.imag
        
        result = {}
        
        # 判断匹配可行性
        Q_match = np.sqrt(RL / Z0 - 1) if RL > Z0 else np.sqrt(Z0 / RL - 1)
        
        if RL > Z0:
            # 高阻抗到低阻抗: 串联-并联型
            # 串联电感/电容 + 并联电容/电感
            X_s = Q_match * Z0  # 串联电抗
            X_p = RL / Q_match   # 并联电抗
            
            if XL >= 0:  # 需要串联电容抵消
                result['type'] = 'LC-串联并联'
                result['series_C'] = -1 / (omega * (X_s - XL)) if abs(X_s - XL) > 1e-10 else None
                result['shunt_L'] = X_p / omega
            else:
                result['type'] = 'CL-串联并联'
                result['series_L'] = (X_s - XL) / omega
                result['shunt_C'] = -1 / (omega * X_p) if abs(X_p) > 1e-10 else None
        else:
            # 低阻抗到高阻抗: 并联-串联型
            X_p = Z0 / Q_match
            X_s = Q_match * RL
            
            if XL <= 0:
                result['type'] = 'CL-并联串联'
                result['shunt_L'] = X_p / omega
                result['series_C'] = -1 / (omega * (X_s + XL)) if abs(X_s + XL) > 1e-10 else None
            else:
                result['type'] = 'LC-并联串联'
                result['shunt_C'] = -1 / (omega * X_p) if abs(X_p) > 1e-10 else None
                result['series_L'] = (X_s + XL) / omega
        
        result['Q'] = Q_match
        result['bandwidth_f0_over_Q'] = self.f / Q_match
        return result
    
    def stub_matching(self, line_Z0: float = 50.0) -> dict:
        """
        短截线匹配 (单stub)
        
        Args:
            line_Z0: 传输线特性阻抗
        Returns:
            匹配参数
        """
        Y_L = 1 / self.Z_L
        Y0 = 1 / line_Z0
        
        # 归一化导纳
        g = Y_L.real / Y0
        b = Y_L.imag / Y0
        
        # 短截线位置和长度
        # 简化: 在smith圆图上求解
        if g > 0:
            # 匹配距离
            d_wavelengths = np.arctan(b / (g - 1)) / (2 * np.pi) if abs(g - 1) > 1e-6 else 0.25
            if d_wavelengths < 0:
                d_wavelengths += 0.5
            
            # stub导纳
            Y_stub = complex(0, -Y0 * b)
            stub_length = np.arctan(Y_stub.imag / Y0) / (2 * np.pi)
            if stub_length < 0:
                stub_length += 0.5
            
            return {
                'stub_position': d_wavelengths * self.wl,
                'stub_position_wavelengths': d_wavelengths,
                'stub_length': stub_length * self.wl,
                'stub_length_wavelengths': stub_length,
                'type': 'short_stub'
            }
        return {'error': '无法匹配, 归一化电导g<0'}
    
    def smith_chart_impedance(self, Z: complex = None) -> Tuple[float, float]:
        """
        计算Smith圆图上的阻抗坐标
        
        Returns:
            (r, x) 归一化阻抗
        """
        Z = Z if Z is not None else self.Z_L
        z = Z / self.Z0
        return z.real, z.imag
    
    def gamma_to_impedance(self, Gamma: complex) -> complex:
        """从反射系数计算阻抗"""
        return self.Z0 * (1 + Gamma) / (1 - Gamma)
    
    def impedance_to_gamma(self, Z: complex) -> complex:
        """从阻抗计算反射系数"""
        return (Z - self.Z0) / (Z + self.Z0)


# ─────────────────────────────────────────────
# 4. 天线增益与链路计算
# ─────────────────────────────────────────────

class AntennaLinkBudget:
    """天线链路预算"""
    
    @staticmethod
    def friis_transmission(P_tx_dBm: float, G_tx_dBi: float, G_rx_dBi: float,
                           distance_m: float, frequency_Hz: float,
                           additional_losses_dB: float = 0) -> dict:
        """
        Friis传输公式
        
        P_rx = P_tx + G_tx + G_rx - FSPL - L_additional
        """
        c = 3e8
        wl = c / frequency_Hz
        
        # 自由空间路径损耗
        FSPL_dB = 20 * np.log10(4 * np.pi * distance_m / wl)
        
        P_rx_dBm = P_tx_dBm + G_tx_dBi + G_rx_dBi - FSPL_dB - additional_losses_dB
        
        return {
            'P_tx_dBm': P_tx_dBm,
            'G_tx_dBi': G_tx_dBi,
            'G_rx_dBi': G_rx_dBi,
            'FSPL_dB': FSPL_dB,
            'additional_losses_dB': additional_losses_dB,
            'P_rx_dBm': P_rx_dBm,
            'distance_m': distance_m,
            'frequency_Hz': frequency_Hz
        }
    
    @staticmethod
    def radar_range(P_tx_dBm: float, G_dBi: float, wavelength_m: float,
                    rcs_m2: float, sensitivity_dBm: float) -> float:
        """
        雷达方程: 最大探测距离
        
        R_max = [(P_tx * G² * λ² * σ) / ((4π)³ * S_min)]^(1/4)
        """
        P_tx_W = 10**(P_tx_dBm/10) / 1000
        G = 10**(G_dBi/10)
        S_min_W = 10**(sensitivity_dBm/10) / 1000
        
        R4 = P_tx_W * G**2 * wavelength_m**2 * rcs_m2 / ((4*np.pi)**3 * S_min_W)
        R_max = R4**0.25
        return R_max


# ─────────────────────────────────────────────
# 综合演示
# ─────────────────────────────────────────────

def demo_dipole():
    """偶极子天线演示"""
    print("=" * 60)
    print("  偶极子天线仿真演示")
    print("=" * 60)
    
    # 半波偶极子
    freq = 2.4e9
    wl = 3e8 / freq
    dipole = DipoleAntenna(length=wl/2, frequency=freq)
    
    print(f"  半波偶极子 @ {freq/1e9:.1f}GHz (λ={wl*100:.2f}mm)")
    print(f"    方向性: {dipole.directivity():.3f} ({10*np.log10(dipole.directivity()):.2f} dBi)")
    print(f"    增益: {dipole.gain_dBi():.2f} dBi")
    print(f"    输入阻抗: {dipole.input_impedance():.1f} Ω")
    print(f"    VSWR (50Ω): {dipole.vswr():.2f}")
    print(f"    回波损耗: {dipole.return_loss_dB():.2f} dB")
    print(f"    有效长度: {dipole.effective_length()*1000:.2f}mm")
    print(f"    半功率波束宽度: {np.degrees(2*np.arccos(1/np.sqrt(2))):.1f}°")
    
    # 不同长度偶极子
    print(f"\n  不同长度偶极子对比:")
    for L_ratio in [0.25, 0.5, 0.75, 1.0, 1.25]:
        d = DipoleAntenna(length=L_ratio * wl, frequency=freq)
        D = d.directivity()
        Z = d.input_impedance()
        print(f"    L/λ={L_ratio:.2f}: D={10*np.log10(D):.2f}dBi, Z={Z.real:.1f}+j{Z.imag:.1f}Ω")


def demo_array():
    """天线阵列演示"""
    print("\n" + "=" * 60)
    print("  天线阵列仿真演示")
    print("=" * 60)
    
    freq = 2.4e9
    wl = 3e8 / freq
    
    # 8元均匀线阵
    N = 8
    d = wl / 2  # 半波长间距
    array = UniformLinearArray(frequency=freq, element_spacing=d, num_elements=N)
    
    print(f"  {N}元均匀线阵 (d=λ/2={d*1000:.1f}mm)")
    print(f"    半功率波束宽度: {np.degrees(array.half_power_beamwidth()):.1f}°")
    print(f"    副瓣电平: {array.sidelobe_level():.1f} dB")
    print(f"    方向性: {10*np.log10(array.directivity()):.2f} dBi")
    print(f"    增益: {array.array_gain_dBi():.2f} dBi")
    
    # 切比雪夫加权
    cheb_w = UniformLinearArray.chebyshev_weights(N, sidelobe_dB=-25)
    print(f"\n  切比雪夫加权 (-25dB副瓣):")
    print(f"    权重: {np.round(cheb_w, 3)}")
    
    # 波束扫描
    print(f"\n  波束扫描演示:")
    for scan_deg in [0, 15, 30, 45]:
        scan_rad = np.radians(scan_deg)
        array.beam_direction(scan_rad)
        theta = np.linspace(0, np.pi, 360)
        AF = array.array_factor_1d(theta)
        AF_dB = 20 * np.log10(np.abs(AF) / np.abs(AF).max() + 1e-10)
        main_idx = np.argmax(AF_dB)
        actual_angle = np.degrees(theta[main_idx])
        print(f"    设定{scan_deg}° → 实际{actual_angle:.1f}°, 峰值增益={AF_dB.max():.1f}dB")


def demo_matching():
    """阻抗匹配演示"""
    print("\n" + "=" * 60)
    print("  阻抗匹配仿真演示")
    print("=" * 60)
    
    # 天线阻抗 (例如偶极子)
    Z_antenna = complex(73, 42.5)
    freq = 2.4e9
    
    match = ImpedanceMatching(Z_load=Z_antenna, Z0=50, frequency=freq)
    
    print(f"  天线阻抗: {Z_antenna} Ω")
    print(f"  系统阻抗: 50 Ω")
    print(f"  反射系数: {match.reflection_coefficient():.4f}")
    print(f"  VSWR: {match.vswr():.2f}")
    print(f"  回波损耗: {match.return_loss_dB():.2f} dB")
    print(f"  失配损耗: {match.mismatch_loss_dB():.2f} dB")
    
    # L型匹配网络
    l_net = match.l_network_design()
    print(f"\n  L型匹配网络:")
    print(f"    类型: {l_net['type']}")
    if l_net.get('series_L'):
        print(f"    串联电感: {l_net['series_L']*1e9:.2f} nH")
    if l_net.get('series_C'):
        print(f"    串联电容: {l_net['series_C']*1e12:.2f} pF")
    if l_net.get('shunt_L'):
        print(f"    并联电感: {l_net['shunt_L']*1e9:.2f} nH")
    if l_net.get('shunt_C'):
        print(f"    并联电容: {l_net['shunt_C']*1e12:.2f} pF")
    print(f"    Q值: {l_net['Q']:.2f}")
    print(f"    带宽: {l_net['bandwidth_f0_over_Q']/1e6:.1f} MHz")


def demo_link_budget():
    """链路预算演示"""
    print("\n" + "=" * 60)
    print("  链路预算演示")
    print("=" * 60)
    
    link = AntennaLinkBudget.friis_transmission(
        P_tx_dBm=20, G_tx_dBi=6, G_rx_dBi=2,
        distance_m=100, frequency_Hz=2.4e9
    )
    
    print(f"  2.4GHz 链路 (100m):")
    print(f"    发射功率: {link['P_tx_dBm']} dBm")
    print(f"    发射增益: {link['G_tx_dBi']} dBi")
    print(f"    接收增益: {link['G_rx_dBi']} dBi")
    print(f"    路径损耗: {link['FSPL_dB']:.1f} dB")
    print(f"    接收功率: {link['P_rx_dBm']:.1f} dBm")


if __name__ == "__main__":
    demo_dipole()
    demo_array()
    demo_matching()
    demo_link_budget()
    print("\n✓ 天线仿真演示完成")
