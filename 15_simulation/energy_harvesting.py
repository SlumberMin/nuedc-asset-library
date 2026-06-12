"""
能量收集仿真 - 太阳能/振动/热电/RF能量转换
==============================================
仿真多种能量收集(Energy Harvesting)技术的能量转换特性：
1. 太阳能光伏：I-V特性曲线、MPPT算法（P&O/INC）、遮挡效应
2. 压电振动能量：悬臂梁模型、非线性整流、阻抗匹配
3. 热电发电(TEG)：塞贝克效应、热阻网络、负载匹配
4. RF能量收集：天线模型、阻抗匹配网络、倍压整流
5. 多源混合收集与能量管理
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


# ═══════════════════════════════════════════════
# 1. 太阳能光伏模型
# ═══════════════════════════════════════════════
class SolarCell:
    """
    单二极管光伏模型
    I = Iph - I0 * [exp((V + I*Rs)/(n*Vt)) - 1] - (V + I*Rs)/Rp
    """
    def __init__(self, n_series=1, n_parallel=1, Isc_ref=3.5, Voc_ref=0.6,
                 Ki=0.0032, Kv=-0.0022, T_ref=298.15, Rs=0.005, Rp=10.0, n_diode=1.3):
        self.ns = n_series
        self.np = n_parallel
        self.Isc = Isc_ref * n_parallel
        self.Voc = Voc_ref * n_series
        self.Ki = Ki
        self.Kv = Kv
        self.T_ref = T_ref
        self.Rs = Rs * n_series / n_parallel
        self.Rp = Rp * n_series / n_parallel
        self.n = n_diode

    def current(self, V, G=1000, T=298.15):
        """给定电压V，辐射G(W/m²)，温度T(K)，返回电流"""
        Vt = 1.381e-23 * T / 1.602e-19  # 热电压
        dT = T - self.T_ref
        Iph = (self.Isc + self.Ki * dT) * G / 1000
        I0 = (self.Isc + self.Ki * dT) / (np.exp((self.Voc + self.Kv * dT) / (self.n * Vt * self.ns)) - 1)

        # 迭代求解隐式方程 (Newton-Raphson)
        I = Iph  # 初始猜测
        for _ in range(50):
            f = Iph - I0 * (np.exp((V + I * self.Rs) / (self.n * Vt * self.ns)) - 1) - (V + I * self.Rs) / self.Rp - I
            df = -I0 * self.Rs / (self.n * Vt * self.ns) * np.exp((V + I * self.Rs) / (self.n * Vt * self.ns)) - self.Rs / self.Rp - 1
            dI = -f / df
            I += dI
            if abs(dI) < 1e-8:
                break
        return max(I, 0)

    def iv_curve(self, G=1000, T=298.15, n_points=200):
        """计算完整I-V曲线"""
        V_range = np.linspace(0, self.Voc + 0.1, n_points)
        I_range = np.array([self.current(V, G, T) for V in V_range])
        P_range = V_range * I_range
        return V_range, I_range, P_range

    def mpp(self, G=1000, T=298.15):
        """最大功率点"""
        V, I, P = self.iv_curve(G, T)
        idx = np.argmax(P)
        return V[idx], I[idx], P[idx]


class MPPTController:
    """最大功率点追踪(MPPT)控制器"""
    def __init__(self, method='P&O', dV=0.01):
        self.method = method
        self.dV = dV
        self.V_prev = 0.0
        self.P_prev = 0.0
        self.V_ref = 0.0

    def update(self, V, P):
        if self.method == 'P&O':
            # 扰动观察法
            dP = P - self.P_prev
            dV = V - self.V_prev
            if dV == 0:
                pass
            elif dP > 0:
                self.V_ref = V + self.dV * np.sign(dV)
            else:
                self.V_ref = V - self.dV * np.sign(dV)
            self.V_prev = V
            self.P_prev = P
        elif self.method == 'INC':
            # 增量电导法
            if abs(V - self.V_prev) > 1e-6:
                dI = (P / V - self.P_prev / max(self.V_prev, 1e-6))
                dV = V - self.V_prev
                I = P / max(V, 1e-6)
                if abs(dI / max(abs(I), 1e-6) + 1 / max(V, 1e-6)) < 1e-3:
                    self.V_ref = V
                elif dI / max(abs(dV), 1e-6) > -1 / max(V, 1e-6):
                    self.V_ref = V + self.dV
                else:
                    self.V_ref = V - self.dV
            self.V_prev = V
            self.P_prev = P

        return self.V_ref


# ═══════════════════════════════════════════════
# 2. 压电振动能量收集
# ═══════════════════════════════════════════════
class PiezoelectricHarvester:
    """
    压电悬臂梁振动能量收集器
    等效电路模型：电流源 + 电容 + 电阻
    I_piezo = Θ * ẏ (机电耦合)
    """
    def __init__(self, mass=0.015, fn=50.0, zeta=0.02, d33=400e-12,
                 C_p=25e-9, R_p=500e3, Theta=1e-3):
        self.m = mass        # 等效质量 kg
        self.fn = fn         # 固有频率 Hz
        self.zeta = zeta     # 阻尼比
        self.omega_n = 2 * np.pi * fn
        self.C_p = C_p       # 压电电容
        self.R_p = R_p       # 压电内阻
        self.Theta = Theta   # 机电耦合系数

    def response(self, f_exc, A_exc, R_load, n_cycles=200):
        """激励频率f_exc，加速度幅值A_exc，负载R_load"""
        omega = 2 * np.pi * f_exc
        dt = 1 / (f_exc * 100)
        n_steps = int(n_cycles / f_exc / dt)

        # 解析解（稳态）
        r = f_exc / self.fn
        H = 1 / np.sqrt((1 - r**2)**2 + (2 * self.zeta * r)**2)
        x_amp = A_exc / (self.omega_n**2) * H  # 位移幅值
        v_amp = omega * x_amp                   # 速度幅值

        # 开路电压
        V_oc = self.Theta * v_amp / (omega * self.C_p)
        # 负载电压
        Z_c = 1 / (omega * self.C_p)
        Z_p = self.R_p
        Z_l = R_load
        # 等效阻抗
        Z_parallel = 1 / (1/Z_p + 1/Z_l + 1/(1j * Z_c))
        V_load = V_oc * abs(Z_parallel) / abs(Z_parallel + 1j * Z_c)

        P_avg = V_load**2 / (2 * R_load)
        return V_load, P_avg, x_amp

    def frequency_response(self, f_range, A_exc=1.0, R_load=100e3):
        """频率响应曲线"""
        V_list, P_list = [], []
        for f in f_range:
            V, P, _ = self.response(f, A_exc, R_load)
            V_list.append(V)
            P_list.append(P)
        return np.array(V_list), np.array(P_list)

    def optimal_load(self, f_exc, A_exc, R_range=None):
        """最优负载匹配"""
        if R_range is None:
            R_range = np.logspace(2, 7, 200)
        P_list = [self.response(f_exc, A_exc, R)[1] for R in R_range]
        idx = np.argmax(P_list)
        return R_range[idx], P_list[idx]


# ═══════════════════════════════════════════════
# 3. 热电发电(TEG)
# ═══════════════════════════════════════════════
class ThermoElectricGenerator:
    """
    热电发电模块模型
    基于塞贝克效应：V = α * ΔT
    热阻网络模型
    """
    def __init__(self, n_couples=127, alpha=200e-6, R_internal=2.0,
                 K_thermal=0.5, A_cross=1.44e-4, L_leg=1.6e-3):
        self.n = n_couples
        self.alpha = alpha      # 塞贝克系数 V/K (模块级)
        self.R_int = R_internal # 内阻 ohm
        self.K = K_thermal      # 热导 W/K
        self.A = A_cross
        self.L = L_leg

    def generate(self, T_hot, T_cold, R_load):
        """热端温度、冷端温度、负载电阻 -> (电压, 电流, 功率)"""
        dT = T_hot - T_cold
        if dT <= 0:
            return 0, 0, 0
        V_oc = self.alpha * dT
        V_load = V_oc * R_load / (R_load + self.R_int)
        I = V_load / R_load
        P = V_load * I
        return V_load, I, P

    def characteristic(self, T_hot, T_cold, R_range=None):
        """TEG特性曲线"""
        if R_range is None:
            R_range = np.linspace(0.1, 20, 200)
        V_list, I_list, P_list = [], [], []
        for R in R_range:
            V, I, P = self.generate(T_hot, T_cold, R)
            V_list.append(V)
            I_list.append(I)
            P_list.append(P)
        return R_range, np.array(V_list), np.array(I_list), np.array(P_list)

    def max_power(self, T_hot, T_cold):
        """最大功率点"""
        R_opt = self.R_int  # 阻抗匹配
        V, I, P = self.generate(T_hot, T_cold, R_opt)
        return R_opt, P


# ═══════════════════════════════════════════════
# 4. RF能量收集
# ═══════════════════════════════════════════════
class RFHarvester:
    """
    RF能量收集器
    天线 -> 阻抗匹配 -> 倍压整流 -> DC输出
    """
    def __init__(self, freq=2.4e9, gain_dBi=6, rect_efficiency=0.5,
                 n_stages=3, V_threshold=0.2):
        self.freq = freq
        self.gain = 10**(gain_dBi/10)
        self.eff = rect_efficiency
        self.n_stages = n_stages
        self.V_th = V_threshold  # 二极管阈值
        self.c = 3e8

    def received_power(self, P_tx, distance, freq=None):
        """Friis方程：接收功率"""
        f = freq or self.freq
        lam = self.c / f
        G_tx = 1.0  # 发射天线增益
        G_rx = self.gain
        # 自由空间路径损耗
        FSPL = (lam / (4 * np.pi * distance))**2
        P_rx = P_tx * G_tx * G_rx * FSPL
        return P_rx

    def rectenna_output(self, P_rx_dbm, R_load=10e3):
        """接收功率(dBm) -> DC输出"""
        P_rx = 10**(P_rx_dbm / 10) * 1e-3  # 转W
        # 整流效率随输入功率变化
        if P_rx < 1e-6:
            eff_actual = 0.1
        elif P_rx < 1e-3:
            eff_actual = self.eff * 0.5
        else:
            eff_actual = self.eff

        P_dc = P_rx * eff_actual
        # 倍压整流电压
        V_dc = self.n_stages * np.sqrt(2 * P_rx * R_load) - self.n_stages * self.V_th
        V_dc = max(V_dc, 0)
        I_dc = P_dc / max(V_dc, 1e-12) if V_dc > 0 else 0
        return V_dc, I_dc, P_dc

    def distance_sweep(self, P_tx=1.0, d_range=None, R_load=10e3):
        """距离vs收集功率"""
        if d_range is None:
            d_range = np.logspace(0, 3, 200)  # 1m ~ 1km
        P_list, V_list = [], []
        for d in d_range:
            P_rx = self.received_power(P_tx, d)
            P_rx_dbm = 10 * np.log10(max(P_rx, 1e-15) / 1e-3)
            V, I, P = self.rectenna_output(P_rx_dbm, R_load)
            P_list.append(P)
            V_list.append(V)
        return d_range, np.array(P_list), np.array(V_list)


# ═══════════════════════════════════════════════
# 5. 多源混合能量管理
# ═══════════════════════════════════════════════
class HybridEnergyManager:
    """多源能量收集管理器"""
    def __init__(self, battery_capacity_Wh=10.0):
        self.battery_Wh = battery_capacity_Wh
        self.battery_V = 3.7
        self.soc = 0.5  # 初始50%

    def manage(self, P_solar, P_vib, P_teg, P_rf, P_load, dt_hours):
        """输入各源功率、负载功率、时间步"""
        P_total = P_solar + P_vib + P_teg + P_rf
        P_net = P_total - P_load
        energy_net = P_net * dt_hours  # Wh
        self.soc += energy_net / self.battery_Wh
        self.soc = np.clip(self.soc, 0, 1)
        return P_total, P_net, self.soc


# ═══════════════════════════════════════════════
# 仿真函数
# ═══════════════════════════════════════════════
def run_solar_simulation():
    """太阳能仿真"""
    cell = SolarCell(n_series=36, n_parallel=2)

    # I-V曲线在不同辐射下
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle('太阳能光伏仿真', fontsize=14)

    for G in [200, 400, 600, 800, 1000]:
        V, I, P = cell.iv_curve(G=G, T=318)
        axes[0].plot(V, I, label=f'{G} W/m²')
        axes[1].plot(V, P, label=f'{G} W/m²')
    axes[0].set_xlabel('电压 (V)')
    axes[0].set_ylabel('电流 (A)')
    axes[0].set_title('I-V特性曲线')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel('电压 (V)')
    axes[1].set_ylabel('功率 (W)')
    axes[1].set_title('P-V特性曲线')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # MPPT跟踪
    mppt = MPPTController(method='P&O', dV=0.5)
    t = np.linspace(0, 3600, 500)
    G_profile = 500 + 300 * np.sin(2 * np.pi * t / 3600)  # 一天辐射变化
    P_mppt = []
    P_actual = []
    V_curr = 15.0
    for G in G_profile:
        Vmpp, Impp, Pmpp = cell.mpp(G=G, T=318)
        V_ref = mppt.update(V_curr, V_curr * cell.current(V_curr, G, 318))
        V_curr = V_ref
        P_mppt.append(V_curr * cell.current(V_curr, G, 318))
        P_actual.append(Pmpp)

    axes[2].plot(t/60, P_mppt, 'b-', label='MPPT功率')
    axes[2].plot(t/60, P_actual, 'r--', label='理论最大')
    axes[2].set_xlabel('时间 (min)')
    axes[2].set_ylabel('功率 (W)')
    axes[2].set_title('MPPT跟踪 (P&O)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/solar_harvesting.png', dpi=150)
    plt.close()
    eff = np.mean(P_mppt) / max(np.mean(P_actual), 1e-6) * 100
    print(f"[太阳能] MPPT平均效率: {eff:.1f}%")
    print(f"[太阳能] 最大输出功率: {max(P_actual):.1f} W")


def run_piezo_simulation():
    """压电振动能量收集仿真"""
    harvester = PiezoelectricHarvester(fn=50, zeta=0.02)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle('压电振动能量收集仿真', fontsize=14)

    # 频率响应
    f_range = np.linspace(20, 100, 300)
    V, P = harvester.frequency_response(f_range, A_exc=1.0, R_load=100e3)
    axes[0].plot(f_range, P * 1e6, 'b-')
    axes[0].axvline(50, color='r', linestyle='--', label='fn=50Hz')
    axes[0].set_xlabel('频率 (Hz)')
    axes[0].set_ylabel('功率 (μW)')
    axes[0].set_title('频率响应')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 最优负载
    R_opt, P_max = harvester.optimal_load(50, 1.0)
    print(f"[压电] 最优负载: {R_opt/1e3:.1f} kΩ, 最大功率: {P_max*1e6:.2f} μW")
    R_range = np.logspace(2, 7, 200)
    P_load = [harvester.response(50, 1.0, R)[1] for R in R_range]
    axes[1].semilogx(R_range/1e3, np.array(P_load)*1e6, 'g-')
    axes[1].axvline(R_opt/1e3, color='r', linestyle='--', label=f'R_opt={R_opt/1e3:.0f}kΩ')
    axes[1].set_xlabel('负载电阻 (kΩ)')
    axes[1].set_ylabel('功率 (μW)')
    axes[1].set_title('负载匹配')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # 加速度影响
    A_range = np.linspace(0.1, 5, 50)
    P_acc = [harvester.response(50, A, R_opt)[1] for A in A_range]
    axes[2].plot(A_range, np.array(P_acc)*1e6, 'r-')
    axes[2].set_xlabel('加速度 (m/s²)')
    axes[2].set_ylabel('功率 (μW)')
    axes[2].set_title('加速度vs功率')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/piezo_harvesting.png', dpi=150)
    plt.close()
    print("[压电] 图表已保存")


def run_teg_simulation():
    """热电发电仿真"""
    teg = ThermoElectricGenerator()

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle('热电发电(TEG)仿真', fontsize=14)

    # 不同温差下的特性
    for dT in [20, 40, 60, 80, 100]:
        R, V, I, P = teg.characteristic(25 + dT, 25)
        axes[0].plot(R, P * 1000, label=f'ΔT={dT}K')
    axes[0].set_xlabel('负载电阻 (Ω)')
    axes[0].set_ylabel('功率 (mW)')
    axes[0].set_title('TEG P-R特性')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # 最大功率vs温差
    dT_range = np.linspace(5, 150, 100)
    P_max = [teg.max_power(25 + dt, 25)[1] * 1000 for dt in dT_range]
    axes[1].plot(dT_range, P_max, 'r-')
    axes[1].set_xlabel('温差 ΔT (K)')
    axes[1].set_ylabel('最大功率 (mW)')
    axes[1].set_title('最大功率 vs 温差')
    axes[1].grid(True, alpha=0.3)

    # 最优负载vs温差
    R_opt = [teg.max_power(25 + dt, 25)[0] for dt in dT_range]
    axes[2].plot(dT_range, R_opt, 'b-')
    axes[2].set_xlabel('温差 ΔT (K)')
    axes[2].set_ylabel('最优负载 (Ω)')
    axes[2].set_title('最优负载 vs 温差')
    axes[2].grid(True, alpha=0.3)

    R_opt60, P_max60 = teg.max_power(85, 25)
    print(f"[TEG] ΔT=60K 最优负载: {R_opt60:.1f}Ω, 最大功率: {P_max60*1000:.1f}mW")

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/teg_harvesting.png', dpi=150)
    plt.close()
    print("[TEG] 图表已保存")


def run_rf_simulation():
    """RF能量收集仿真"""
    rf = RFHarvester(freq=2.4e9, gain_dBi=6, n_stages=3)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle('RF能量收集仿真', fontsize=14)

    # 距离vs功率
    for P_tx in [0.1, 1.0, 10.0]:
        d, P, V = rf.distance_sweep(P_tx=P_tx)
        axes[0].loglog(d, P * 1e6, label=f'Pt={P_tx}W')
    axes[0].set_xlabel('距离 (m)')
    axes[0].set_ylabel('收集功率 (μW)')
    axes[0].set_title('距离 vs 收集功率 (2.4GHz)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 不同频率
    for freq in [900e6, 1.8e9, 2.4e9, 5.8e9]:
        d = np.logspace(0, 3, 100)
        P_list = []
        for dist in d:
            P_rx = rf.received_power(1.0, dist, freq)
            P_rx_dbm = 10 * np.log10(max(P_rx, 1e-15) / 1e-3)
            _, _, P = rf.rectenna_output(P_rx_dbm)
            P_list.append(P)
        axes[1].loglog(d, np.array(P_list)*1e6, label=f'{freq/1e9:.1f}GHz')
    axes[1].set_xlabel('距离 (m)')
    axes[1].set_ylabel('收集功率 (μW)')
    axes[1].set_title('频率影响 (Pt=1W)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # 负载匹配
    R_load_range = np.logspace(2, 6, 100)
    P_rx_dbm = -20  # -20dBm接收
    P_dc_list = []
    for R in R_load_range:
        _, _, P = rf.rectenna_output(P_rx_dbm, R)
        P_dc_list.append(P)
    axes[2].semilogx(R_load_range/1e3, np.array(P_dc_list)*1e6, 'g-')
    axes[2].set_xlabel('负载电阻 (kΩ)')
    axes[2].set_ylabel('DC功率 (μW)')
    axes[2].set_title(f'负载匹配 (Pr={P_rx_dbm}dBm)')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/rf_harvesting.png', dpi=150)
    plt.close()
    print("[RF] 图表已保存")


def run_hybrid_simulation():
    """多源混合能量管理仿真"""
    manager = HybridEnergyManager(battery_capacity_Wh=20.0)

    # 24小时仿真
    hours = np.arange(0, 24, 0.5)
    n = len(hours)
    P_solar = np.maximum(0, 5 * np.sin(np.pi * (hours - 6) / 12)) * (hours > 6) * (hours < 18)
    P_vib = 0.05 + 0.03 * np.random.randn(n)  # 振动（近恒定）
    P_teg = 0.1 + 0.05 * np.sin(2 * np.pi * hours / 24)  # TEG（昼夜温差）
    P_rf = 0.02 + 0.01 * np.random.randn(n)   # RF（微弱）
    P_load = 0.3 + 0.1 * (hours > 8) * (hours < 20)  # 负载

    P_total_log, P_net_log, soc_log = [], [], []
    for i in range(n):
        P_s = max(P_solar[i], 0)
        P_v = max(P_vib[i], 0)
        P_t = max(P_teg[i], 0)
        P_r = max(P_rf[i], 0)
        P_t, P_n, soc = manager.manage(P_s, P_v, P_t, P_r, P_load[i], 0.5)
        P_total_log.append(P_t)
        P_net_log.append(P_n)
        soc_log.append(soc)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('多源混合能量收集管理', fontsize=14)

    axes[0].stackplot(hours,
                      np.maximum(P_solar, 0), np.maximum(P_vib, 0),
                      np.maximum(P_teg, 0), np.maximum(P_rf, 0),
                      labels=['太阳能', '振动', '热电', 'RF'],
                      alpha=0.7)
    axes[0].plot(hours, P_load, 'k-', linewidth=2, label='负载')
    axes[0].set_ylabel('功率 (W)')
    axes[0].set_title('各源功率输出')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(hours, np.array(soc_log) * 100, 'g-', linewidth=2)
    axes[1].set_xlabel('时间 (h)')
    axes[1].set_ylabel('电池SOC (%)')
    axes[1].set_title('电池荷电状态')
    axes[1].set_ylim(0, 100)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/hybrid_energy.png', dpi=150)
    plt.close()
    print(f"[混合] 最终SOC: {soc_log[-1]*100:.1f}%")
    print(f"[混合] 平均净功率: {np.mean(P_net_log):.3f} W")
    print("[混合] 图表已保存")


if __name__ == '__main__':
    print("=" * 60)
    print("能量收集仿真 - 太阳能/振动/热电/RF能量转换")
    print("=" * 60)
    run_solar_simulation()
    run_piezo_simulation()
    run_teg_simulation()
    run_rf_simulation()
    run_hybrid_simulation()
    print("\n✅ 全部能量收集仿真完成！")
