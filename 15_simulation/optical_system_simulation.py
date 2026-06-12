"""
光学系统仿真模块 - Optical System Simulation
================================================
功能: 透镜成像、衍射、干涉、光纤传输仿真
适用: 电赛光学题目辅助设计与验证
"""

import numpy as np
from numpy.fft import fft2, ifft2, fftshift
from typing import Tuple, Optional
import warnings
warnings.filterwarnings('ignore')

# ─────────────────────────────────────────────
# 1. 透镜成像仿真
# ─────────────────────────────────────────────

class ThinLens:
    """薄透镜成像模型"""
    
    def __init__(self, focal_length: float, diameter: float = None, n: float = 1.5):
        """
        Args:
            focal_length: 焦距 (m)
            diameter: 透镜直径 (m), 用于计算数值孔径
            n: 透镜折射率
        """
        self.f = focal_length
        self.d = diameter
        self.n = n
    
    def image_distance(self, object_dist: float) -> float:
        """薄透镜公式: 1/v - 1/u = 1/f, u<0 (实物)"""
        u = -abs(object_dist)
        v = 1.0 / (1.0/self.f + 1.0/u)
        return v
    
    def magnification(self, object_dist: float) -> float:
        """横向放大率 M = v/u"""
        u = -abs(object_dist)
        v = self.image_distance(object_dist)
        return v / u
    
    def ray_transfer_matrix(self) -> np.ndarray:
        """薄透镜的光线传输矩阵 [[1,0],[-1/f,1]]"""
        return np.array([[1, 0], [-1/self.f, 1]])
    
    def numerical_aperture(self) -> float:
        """数值孔径 NA = d/(2f) (近轴近似)"""
        if self.d is None:
            raise ValueError("需设置透镜直径")
        return self.d / (2 * self.f)
    
    def airy_disk_radius(self, wavelength: float) -> float:
        """艾里斑半径 r = 1.22*λ*f/D"""
        if self.d is None:
            raise ValueError("需设置透镜直径")
        return 1.22 * wavelength * self.f / self.d
    
    def depth_of_field(self, wavelength: float, coc: float) -> Tuple[float, float]:
        """景深计算 (circle of confusion)"""
        N = self.f / self.d if self.d else None
        if N is None:
            raise ValueError("需设置透镜直径")
        dof_near = 2 * N * coc
        dof_far = 2 * N * coc
        return dof_near, dof_far
    
    def simulate_imaging(self, object_func: np.ndarray, z_obj: float,
                         wavelength: float, pixel_size: float) -> np.ndarray:
        """
        基于角谱法的透镜成像仿真
        
        Args:
            object_func: 物面复振幅分布 (2D array)
            z_obj: 物距 (m)
            wavelength: 波长 (m)
            pixel_size: 像素尺寸 (m)
        Returns:
            像面强度分布
        """
        z_img = self.image_distance(z_obj)
        k = 2 * np.pi / wavelength
        M = self.magnification(z_obj)
        
        Ny, Nx = object_func.shape
        # 频率网格
        dfx = 1.0 / (Nx * pixel_size)
        dfy = 1.0 / (Ny * pixel_size)
        fx = np.arange(-Nx//2, Nx//2) * dfx
        fy = np.arange(-Ny//2, Ny//2) * dfy
        FX, FY = np.meshgrid(fx, fy)
        
        # 物面到透镜的自由空间传播 (角谱法)
        H_free = np.exp(1j * k * abs(z_obj) * np.sqrt(1 - (wavelength*FX)**2 - (wavelength*FY)**2))
        H_free[np.isnan(H_free)] = 0
        
        U_lens = fftshift(fft2(fftshift(object_func))) * H_free
        U_lens = fftshift(ifft2(fftshift(U_lens)))
        
        # 透镜相位调制
        x = np.arange(-Nx//2, Nx//2) * pixel_size
        y = np.arange(-Ny//2, Ny//2) * pixel_size
        X, Y = np.meshgrid(x, y)
        lens_phase = np.exp(-1j * k * (X**2 + Y**2) / (2 * self.f))
        U_after_lens = U_lens * lens_phase
        
        # 透镜到像面传播
        H_img = np.exp(1j * k * abs(z_img) * np.sqrt(1 - (wavelength*FX)**2 - (wavelength*FY)**2))
        H_img[np.isnan(H_img)] = 0
        
        U_img = fftshift(fft2(fftshift(U_after_lens))) * H_img
        U_img = fftshift(ifft2(fftshift(U_img)))
        
        return np.abs(U_img)**2


class ThickLens:
    """厚透镜模型 (考虑色差)"""
    
    def __init__(self, R1: float, R2: float, thickness: float, n: float, n_func=None):
        """
        Args:
            R1, R2: 前后曲率半径 (m), 凸为正
            thickness: 中心厚度 (m)
            n: 折射率
            n_func: 色散函数 n(λ), 可选
        """
        self.R1 = R1
        self.R2 = R2
        self.t = thickness
        self.n = n
        self.n_func = n_func
    
    def focal_length(self, wavelength: float = None) -> float:
        """厚透镜焦距"""
        n = self.n_func(wavelength) if (self.n_func and wavelength) else self.n
        # 透镜制造者公式 (厚透镜)
        power = (n - 1) * (1/self.R1 - 1/self.R2 + (n-1)*self.t / (n*self.R1*self.R2))
        return 1.0 / power if power != 0 else np.inf
    
    def ray_transfer_matrix(self) -> np.ndarray:
        """厚透镜光线传输矩阵"""
        n = self.n
        # 界面1
        M1 = np.array([[1, 0], [-(n-1)/self.R1, 1/n]])
        # 厚度传播
        Mt = np.array([[1, self.t], [0, 1]])
        # 界面2
        M2 = np.array([[1, 0], [-(n-1)/self.R2, n]])
        return M2 @ Mt @ M1
    
    def chromatic_aberration(self, wl1: float, wl2: float) -> float:
        """色差: Δf = f(λ1) - f(λ2)"""
        if self.n_func is None:
            raise ValueError("需要色散函数 n_func")
        return self.focal_length(wl1) - self.focal_length(wl2)


# ─────────────────────────────────────────────
# 2. 衍射仿真
# ─────────────────────────────────────────────

class DiffractionSimulator:
    """衍射光学仿真器"""
    
    def __init__(self, wavelength: float, grid_size: Tuple[int, int],
                 pixel_size: float):
        """
        Args:
            wavelength: 波长 (m)
            grid_size: (Ny, Nx)
            pixel_size: 物面像素尺寸 (m)
        """
        self.wl = wavelength
        self.Ny, self.Nx = grid_size
        self.dx = pixel_size
        self.k = 2 * np.pi / wavelength
    
    def angular_spectrum_propagation(self, U0: np.ndarray, z: float) -> np.ndarray:
        """
        角谱衍射传播
        适用于近场/远场, z 不太大时准确
        
        Args:
            U0: 初始复振幅
            z: 传播距离 (m)
        Returns:
            传播后的复振幅
        """
        dfx = 1.0 / (self.Nx * self.dx)
        dfy = 1.0 / (self.Ny * self.dx)
        fx = np.arange(-self.Nx//2, self.Nx//2) * dfx
        fy = np.arange(-self.Ny//2, self.Ny//2) * dfy
        FX, FY = np.meshgrid(fx, fy)
        
        # 频域传播函数
        arg = 1 - (self.wl * FX)**2 - (self.wl * FY)**2
        H = np.exp(1j * self.k * z * np.sqrt(np.maximum(arg, 0)))
        
        U0_shifted = fftshift(U0)
        U_freq = fft2(U0_shifted)
        U_out = ifft2(U_freq * fftshift(H))
        return fftshift(U_out)
    
    def fresnel_diffraction(self, U0: np.ndarray, z: float) -> np.ndarray:
        """
        菲涅尔衍射 (傍轴近似)
        适用条件: z³ >> π/(4λ) * (x²+y²)²_max
        
        Args:
            U0: 初始复振幅
            z: 传播距离
        Returns:
            衍射后的复振幅
        """
        x = np.arange(-self.Nx//2, self.Nx//2) * self.dx
        y = np.arange(-self.Ny//2, self.Ny//2) * self.dx
        X, Y = np.meshgrid(x, y)
        
        # 菲涅尔相位因子
        phase = np.exp(1j * self.k / (2*z) * (X**2 + Y**2))
        
        # 用FFT加速
        df = 1.0 / (self.Nx * self.dx)
        fx = np.arange(-self.Nx//2, self.Nx//2) * df
        fy = np.arange(-self.Ny//2, self.Ny//2) * df
        FX, FY = np.meshgrid(fx, fy)
        
        quad_phase = np.exp(-1j * np.pi * self.wl * z * (FX**2 + FY**2))
        prefactor = np.exp(1j * self.k * z) / (1j * self.wl * z)
        
        U_in = U0 * phase
        U_freq = fftshift(fft2(fftshift(U_in)))
        U_out = fftshift(ifft2(fftshift(U_freq * quad_phase))) * prefactor * self.dx**2
        
        return U_out
    
    def fraunhofer_diffraction(self, aperture: np.ndarray, z: float) -> np.ndarray:
        """
        夫琅禾费衍射 (远场)
        适用条件: z >> π*D²/λ
        
        Args:
            aperture: 孔径函数 (复振幅)
            z: 观察距离
        Returns:
            远场衍射图样
        """
        U_freq = fftshift(fft2(fftshift(aperture)))
        # 远场缩放
        scale = self.dx**2 / (1j * self.wl * z)
        return U_freq * scale
    
    def single_slit_diffraction(self, slit_width: float, z: float,
                                 screen_width: float = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        单缝衍射图样
        
        Args:
            slit_width: 缝宽 (m)
            z: 观察距离 (m)
            screen_width: 屏幕宽度 (m)
        Returns:
            (x坐标, 强度分布)
        """
        if screen_width is None:
            screen_width = 20 * self.wl * z / slit_width
        
        x = np.linspace(-screen_width/2, screen_width/2, self.Nx)
        # 解析解: I = I0 * sinc²(a*sin(θ)/λ), 近似 sin(θ) ≈ x/z
        theta = np.arctan(x / z)
        beta = np.pi * slit_width * np.sin(theta) / self.wl
        I = np.sinc(beta / np.pi)**2  # np.sinc(x) = sin(πx)/(πx)
        return x, I
    
    def circular_aperture_diffraction(self, radius: float, z: float,
                                      screen_size: float = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        圆孔衍射 (艾里图样)
        
        Returns:
            (x坐标, y坐标, 强度2D分布)
        """
        if screen_size is None:
            screen_size = 30 * self.wl * z / radius
        
        x = np.linspace(-screen_size/2, screen_size/2, self.Nx)
        y = np.linspace(-screen_size/2, screen_size/2, self.Ny)
        X, Y = np.meshgrid(x, y)
        R = np.sqrt(X**2 + Y**2)
        
        theta = np.arctan(R / z)
        v = 2 * np.pi * radius * np.sin(theta) / self.wl
        
        # Airy pattern: [2*J1(v)/v]²
        from scipy.special import j1
        I = np.where(v == 0, 1.0, (2 * j1(v) / v)**2)
        return x, y, I
    
    def double_slit_diffraction(self, slit_width: float, slit_sep: float,
                                 z: float, screen_width: float = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        双缝衍射 = 单缝衍射因子 × 双缝干涉因子
        """
        if screen_width is None:
            screen_width = 30 * self.wl * z / slit_width
        
        x = np.linspace(-screen_width/2, screen_width/2, self.Nx)
        theta = np.arctan(x / z)
        
        # 单缝因子
        beta = np.pi * slit_width * np.sin(theta) / self.wl
        single = np.sinc(beta / np.pi)**2
        
        # 双缝干涉因子
        delta = np.pi * slit_sep * np.sin(theta) / self.wl
        interference = np.cos(delta)**2
        
        I = single * interference
        return x, I


# ─────────────────────────────────────────────
# 3. 干涉仿真
# ─────────────────────────────────────────────

class InterferometerSimulation:
    """干涉仪仿真"""
    
    def __init__(self, wavelength: float, grid_size: Tuple[int, int],
                 pixel_size: float):
        self.wl = wavelength
        self.Ny, self.Nx = grid_size
        self.dx = pixel_size
    
    def young_double_slit(self, slit_sep: float, z: float, num_fringes: int = 20) -> Tuple[np.ndarray, np.ndarray]:
        """
        杨氏双缝干涉
        
        Args:
            slit_sep: 双缝间距 (m)
            z: 观察屏距离 (m)
            num_fringes: 显示条纹数
        Returns:
            (屏幕坐标, 强度)
        """
        fringe_period = self.wl * z / slit_sep
        screen_width = num_fringes * fringe_period
        x = np.linspace(-screen_width/2, screen_width/2, self.Nx)
        
        # 干涉条纹: I = 4*I0*cos²(π*d*sin(θ)/λ)
        theta = np.arctan(x / z)
        phase_diff = 2 * np.pi * slit_sep * np.sin(theta) / self.wl
        I = 4 * np.cos(phase_diff / 2)**2
        return x, I
    
    def michelson_interferometer(self, mirror1_angle: float = 0, mirror2_angle: float = 0,
                                  path_diff: float = 0, beam_radius: float = 1e-3) -> np.ndarray:
        """
        迈克尔逊干涉仪仿真
        
        Args:
            mirror1_angle: 镜1倾斜角 (rad)
            mirror2_angle: 镜2倾斜角 (rad)
            path_diff: 光程差 (m)
            beam_radius: 光束半径 (m)
        Returns:
            干涉图样 (2D)
        """
        x = np.linspace(-beam_radius, beam_radius, self.Nx)
        y = np.linspace(-beam_radius, beam_radius, self.Ny)
        X, Y = np.meshgrid(x, y)
        
        # 两束光的相位
        phi1 = 2 * np.pi / self.wl * (2 * mirror1_angle * X + path_diff/2)
        phi2 = 2 * np.pi / self.wl * (2 * mirror2_angle * X - path_diff/2)
        
        # 光束包络 (高斯)
        envelope = np.exp(-(X**2 + Y**2) / (beam_radius/2)**2)
        
        # 干涉
        E1 = envelope * np.exp(1j * phi1)
        E2 = envelope * np.exp(1j * phi2)
        I = np.abs(E1 + E2)**2
        return I
    
    def fabry_perot(self, R: float, d: float, theta: float = 0,
                    wavelength_range: Tuple[float, float] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        法布里-珀罗干涉仪透射谱
        
        Args:
            R: 镜面反射率
            d: 腔长 (m)
            theta: 入射角 (rad)
            wavelength_range: (λ_min, λ_max)
        Returns:
            (波长数组, 透射率)
        """
        if wavelength_range is None:
            wl_center = self.wl
            wavelength_range = (wl_center * 0.95, wl_center * 1.05)
        
        wl = np.linspace(wavelength_range[0], wavelength_range[1], self.Nx)
        
        # 自由光谱范围
        n = 1.0  # 腔内折射率
        FSR = wl_center**2 / (2 * n * d * np.cos(theta))
        
        # 精细度
        F = 4 * R / (1 - R)**2
        
        # 透射率
        delta = 4 * np.pi * n * d * np.cos(theta) / wl
        T = 1.0 / (1 + F * np.sin(delta/2)**2)
        
        return wl, T
    
    def newton_rings(self, R_lens: float, wavelength: float = None,
                     n_gap: float = 1.0, max_radius: float = 5e-3) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        牛顿环仿真
        
        Args:
            R_lens: 透镜曲率半径 (m)
            wavelength: 波长
            n_gap: 间隙折射率
            max_radius: 最大观察半径
        Returns:
            (x, y, 强度2D)
        """
        wl = wavelength or self.wl
        x = np.linspace(-max_radius, max_radius, self.Nx)
        y = np.linspace(-max_radius, max_radius, self.Ny)
        X, Y = np.meshgrid(x, y)
        r = np.sqrt(X**2 + Y**2)
        
        # 空气膜厚度 h = r²/(2R)
        h = r**2 / (2 * R_lens)
        
        # 光程差 Δ = 2nh + λ/2 (半波损失)
        delta = 2 * n_gap * h + wl / 2
        
        # 干涉强度
        I = 4 * np.cos(2 * np.pi * delta / wl)**2
        return x, y, I


# ─────────────────────────────────────────────
# 4. 光纤传输仿真
# ─────────────────────────────────────────────

class OpticalFiber:
    """光纤传输仿真"""
    
    def __init__(self, core_radius: float, n_core: float, n_cladding: float,
                 length: float, loss_dB_km: float = 0.2):
        """
        Args:
            core_radius: 纤芯半径 (m)
            n_core: 纤芯折射率
            n_cladding: 包层折射率
            length: 光纤长度 (m)
            loss_dB_km: 损耗 (dB/km)
        """
        self.a = core_radius
        self.n1 = n_core
        self.n2 = n_cladding
        self.L = length
        self.loss = loss_dB_km
        
        self.NA = np.sqrt(n_core**2 - n_cladding**2)
        self.delta = (n_core - n_cladding) / n_core
    
    def v_number(self, wavelength: float) -> float:
        """归一化频率 V = (2πa/λ)*NA"""
        return 2 * np.pi * self.a * self.NA / wavelength
    
    def num_modes(self, wavelength: float) -> int:
        """支持的模式数 (弱导近似)"""
        V = self.v_number(wavelength)
        if V < 2.405:
            return 1  # 单模
        return int(V**2 / 2)
    
    def is_single_mode(self, wavelength: float) -> bool:
        """判断是否单模传输"""
        return self.v_number(wavelength) < 2.405
    
    def cutoff_wavelength(self) -> float:
        """单模截止波长"""
        return 2 * np.pi * self.a * self.NA / 2.405
    
    def mode_field_diameter(self, wavelength: float) -> float:
        """模场直径 (Marcuse近似公式)"""
        V = self.v_number(wavelength)
        w = self.a * (0.65 + 1.619/V**1.5 + 2.879/V**6)
        return 2 * w
    
    def attenuation(self, wavelength: float) -> float:
        """
        计算总衰减
        包含瑞利散射、OH吸收、红外吸收
        """
        wl_um = wavelength * 1e6  # 转换为μm
        
        # 瑞利散射 (~λ^-4)
        rayleigh = 0.7 / wl_um**4
        
        # OH吸收峰 (~1.39μm)
        oh_peak = 0.5 * np.exp(-((wl_um - 1.39) / 0.05)**2)
        
        # 红外吸收
        ir = 0.1 * np.exp((wl_um - 1.6) / 0.1) if wl_um > 1.5 else 0
        
        alpha_dB_km = rayleigh + oh_peak + ir + self.loss * 0.01
        return alpha_dB_km
    
    def transmission_loss(self, wavelength: float) -> float:
        """总传输损耗 (dB)"""
        alpha = self.attenuation(wavelength) + self.loss
        return alpha * self.L / 1000  # 转换为dB
    
    def output_power(self, input_power_mW: float, wavelength: float) -> float:
        """输出光功率 (mW)"""
        loss_dB = self.transmission_loss(wavelength)
        return input_power_mW * 10**(-loss_dB / 10)
    
    def dispersion_parameter(self, wavelength: float) -> float:
        """
        色散参数 D (ps/(nm·km))
        近似公式 (标准单模光纤)
        """
        wl_nm = wavelength * 1e9
        # 零色散波长 ~1310nm
        lambda_0 = 1310
        S0 = 0.09  # 色散斜率 ps/(nm²·km)
        D = S0 / 4 * (wl_nm - lambda_0**4 / wl_nm**3)
        return D
    
    def pulse_broadening(self, pulse_width_ps: float, spectral_width_nm: float,
                         wavelength: float) -> float:
        """
        色散导致的脉冲展宽
        
        Args:
            pulse_width_ps: 输入脉冲宽度 (ps)
            spectral_width_nm: 光谱宽度 (nm)
            wavelength: 中心波长 (m)
        Returns:
            输出脉冲宽度 (ps)
        """
        D = self.dispersion_parameter(wavelength)  # ps/(nm·km)
        L_km = self.L / 1000
        broadening_ps = abs(D) * spectral_width_nm * L_km
        return np.sqrt(pulse_width_ps**2 + broadening_ps**2)
    
    def simulate_field_propagation(self, input_field: np.ndarray,
                                    wavelength: float) -> np.ndarray:
        """
        简化的光纤模式传播仿真 (LP01模式)
        
        Args:
            input_field: 输入场分布
            wavelength: 波长
        Returns:
            输出场分布
        """
        V = self.v_number(wavelength)
        
        # LP01模式的传播常数
        # 近似: b ≈ (1.1428 - 0.996/V)^2 for 1.5 < V < 2.5
        b = max(0, min(1, (1.1428 - 0.996/V)**2)) if V > 1.1428 else 0
        beta = self.n2 * 2*np.pi/wavelength + b * (self.n1 - self.n2) * 2*np.pi/wavelength
        
        # 传输损耗
        loss_dB = self.transmission_loss(wavelength)
        loss_linear = 10**(-loss_dB / 20)  # 振幅衰减
        
        # 输出 = 输入 × 传播因子 × 损耗 (简化)
        output = input_field * loss_linear
        return output
    
    def bend_loss(self, bend_radius: float, wavelength: float) -> float:
        """
        弯曲损耗估算 (dB/turn)
        
        Args:
            bend_radius: 弯曲半径 (m)
            wavelength: 波长 (m)
        Returns:
            每圈弯曲损耗 (dB)
        """
        V = self.v_number(wavelength)
        gamma = (2*np.pi/wavelength) * np.sqrt(self.n1**2 - self.n2**2)
        u = 2.405 * np.sqrt(max(0, 1 - (V/2.405 - 1)**2)) / self.a if V > 2.405 else 1
        W = np.sqrt(gamma**2 - u**2/self.a**2) if gamma > u/self.a else gamma
        
        # 简化弯曲损耗公式
        C = np.exp(-2/3 * W * bend_radius * (
            1 + 2*np.log(self.a * W) - np.log(self.a * W * bend_radius * (self.n1**2 - self.n2**2))
        ))
        loss_per_turn_dB = 4.343 * C  # 10*log10(e) * C
        return max(0, loss_per_turn_dB)


# ─────────────────────────────────────────────
# 综合演示
# ─────────────────────────────────────────────

def demo_lens_imaging():
    """演示: 透镜成像"""
    print("=" * 60)
    print("  透镜成像仿真演示")
    print("=" * 60)
    
    lens = ThinLens(focal_length=50e-3, diameter=25e-3)
    
    # 成像计算
    for obj_dist in [0.1, 0.2, 0.5, 1.0, 5.0]:
        img_dist = lens.image_distance(obj_dist)
        mag = lens.magnification(obj_dist)
        print(f"  物距={obj_dist*1000:.0f}mm → 像距={img_dist*1000:.1f}mm, "
              f"放大率={mag:.3f}x")
    
    # 艾里斑
    airy_r = lens.airy_disk_radius(550e-9)
    print(f"\n  λ=550nm 艾里斑半径: {airy_r*1e6:.2f}μm")
    print(f"  数值孔径 NA = {lens.numerical_aperture():.3f}")


def demo_diffraction():
    """演示: 衍射仿真"""
    print("\n" + "=" * 60)
    print("  衍射仿真演示")
    print("=" * 60)
    
    sim = DiffractionSimulator(wavelength=632.8e-9, grid_size=(256, 256), pixel_size=10e-6)
    
    # 单缝衍射
    x, I = sim.single_slit_diffraction(slit_width=50e-6, z=0.5)
    peak_idx = np.argmax(I > 0.5)
    print(f"  单缝衍射 (缝宽50μm, z=0.5m):")
    print(f"    中央亮纹宽度: {2*x[np.argmin(np.abs(I-0))]:.2e}m")
    
    # 双缝衍射
    x, I = sim.double_slit_diffraction(slit_width=30e-6, slit_sep=200e-6, z=0.5)
    print(f"\n  双缝衍射 (缝宽30μm, 间距200μm):")
    print(f"    图样计算完成, {len(I)}个采样点")


def demo_interference():
    """演示: 干涉仿真"""
    print("\n" + "=" * 60)
    print("  干涉仿真演示")
    print("=" * 60)
    
    interf = InterferometerSimulation(wavelength=632.8e-9, grid_size=(256, 256), pixel_size=10e-6)
    
    # 杨氏双缝
    x, I = interf.young_double_slit(slit_sep=0.5e-3, z=1.0)
    fringe_spacing = 632.8e-9 * 1.0 / 0.5e-3
    print(f"  杨氏双缝干涉 (d=0.5mm, z=1m):")
    print(f"    理论条纹间距: {fringe_spacing*1e3:.3f}mm")
    
    # 法布里-珀罗
    wl, T = interf.fabry_perot(R=0.9, d=1e-3)
    finesse = np.pi * np.sqrt(0.9) / (1 - 0.9)
    print(f"\n  法布里-珀罗 (R=0.9, d=1mm):")
    print(f"    精细度 F = {finesse:.1f}")
    print(f"    最大透射率: {T.max():.4f}")
    
    # 牛顿环
    x, y, I = interf.newton_rings(R_lens=5.0)
    print(f"\n  牛顿环 (R=5m): 图样 {I.shape} 计算完成")


def demo_fiber():
    """演示: 光纤传输"""
    print("\n" + "=" * 60)
    print("  光纤传输仿真演示")
    print("=" * 60)
    
    # 单模光纤
    fiber = OpticalFiber(core_radius=4.5e-6, n_core=1.4504, n_cladding=1.4469,
                         length=10000, loss_dB_km=0.2)
    
    wl = 1550e-9
    V = fiber.v_number(wl)
    print(f"  单模光纤参数 (λ=1550nm):")
    print(f"    V数 = {V:.2f}")
    print(f"    单模: {fiber.is_single_mode(wl)}")
    print(f"    截止波长: {fiber.cutoff_wavelength()*1e9:.0f}nm")
    print(f"    模场直径: {fiber.mode_field_diameter(wl)*1e6:.2f}μm")
    print(f"    10km传输损耗: {fiber.transmission_loss(wl):.2f}dB")
    print(f"    输出功率 (1mW输入): {fiber.output_power(1.0, wl):.4f}mW")
    
    # 色散
    D = fiber.dispersion_parameter(wl)
    print(f"    色散参数: {D:.2f}ps/(nm·km)")
    
    # 脉冲展宽
    out_pulse = fiber.pulse_broadening(pulse_width_ps=10, spectral_width_nm=0.1, wavelength=wl)
    print(f"    脉冲展宽: 10ps → {out_pulse:.2f}ps")


if __name__ == "__main__":
    demo_lens_imaging()
    demo_diffraction()
    demo_interference()
    demo_fiber()
    print("\n✓ 光学系统仿真演示完成")
