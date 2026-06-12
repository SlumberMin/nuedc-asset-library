#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    python_simulation_template.py
@brief   Python仿真代码模板 — 基于47个已知错误模式的标准化防护
@version 2.0
@date    2026-06-12

使用说明:
    1. 搜索替换 YOUR_MODULE 为实际模块名
    2. 根据仿真需求修改仿真参数和绘图逻辑
    3. 保留所有防护段落

本模板覆盖的错误模式 (来自错误经验库):
    #2   硬编码绝对路径         → os.path相对路径
    #3   顶层执行无守卫         → __main__守卫
    #4   缺失import            → 完整import声明
    #10  变量名拼写错误         → 类型注解+命名规范
    #27  npz加载后getattr失败   → 正确的dict读取
    #36  函数内重复import       → 统一在顶层import
    #37  numpy 2.x移除旧API     → 兼容shim
    #38  plt.show()在Agg下阻塞  → plt.close('all')
    #39  matplotlib.use()顺序   → 必须在pyplot之前
    #28  循环内.close()         → with语句或循环外关闭
    #29  bare except            → 指定具体异常类型
    #30  条件导入库在共享函数使用 → 守卫检查
"""

# =========================================================================
#  1. matplotlib后端设置 — 必须在import pyplot之前 (错误经验 #38, #39)
#
#  !! matplotlib.use('Agg') 必须在 import matplotlib.pyplot 之前调用
#  !! 否则后端已初始化，use()无效，plt.show()会阻塞
# =========================================================================
import matplotlib
matplotlib.use('Agg')  # 无头模式: 仅生成图片文件，不弹窗

# =========================================================================
#  2. 标准库import
# =========================================================================
import os
import sys
import time
import math
import warnings
from typing import Tuple, List, Optional, Dict, Any

# =========================================================================
#  3. 第三方库import (错误经验 #4: 缺失import)
# =========================================================================
import numpy as np

# !! 错误经验 #36: 不要在函数内重复import已顶层导入的模块
# !! 比如函数内又写 import numpy as np 会导致 UnboundLocalError
# !! 只在顶层import一次，函数中直接使用

import matplotlib.pyplot as plt

# =========================================================================
#  4. numpy 2.x兼容shim (错误经验 #37)
#
#  np.trapz在numpy 2.0+中被移除，替代为np.trapezoid
#  float, int, bool, complex 等也被移除
# =========================================================================
_trapz = np.trapezoid if hasattr(np, 'trapezoid') else np.trapezoid
"""兼容函数: numpy 2.x用trapezoid, 1.x用trapz (错误经验 #37)"""

# !! 下面这些旧别名在numpy 2.x中已移除，不要使用:
# !! float  → 使用 float
# !! int    → 使用 int
# !! bool   → 使用 bool
# !! complex → 使用 complex

# =========================================================================
#  5. 路径处理 — 使用相对路径 (错误经验 #2)
#
#  !! 绝对不要写死 /path/to/your/project/... 这样的路径
#  !! 使用 os.path.dirname(os.path.abspath(__file__)) 获取脚本所在目录
# =========================================================================
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
"""当前脚本所在目录 (错误经验 #2: 替代硬编码绝对路径)"""

# 数据/输出目录 (相对于脚本位置)
_DATA_DIR = os.path.join(_SCRIPT_DIR, '..', 'data')
_OUTPUT_DIR = os.path.join(_SCRIPT_DIR, '..', 'output', 'your_module')

# 自动创建输出目录
os.makedirs(_OUTPUT_DIR, exist_ok=True)


# =========================================================================
#  6. 核心仿真类/函数
# =========================================================================

class YourSimulation:
    """
    仿真模型类

    示例:
        >>> sim = YourSimulation(dt=0.001)
        >>> sim.run(duration=5.0)
        >>> sim.plot(os.path.join(_OUTPUT_DIR, 'result.png'))
    """

    def __init__(self, dt: float = 0.001, **kwargs):
        """
        初始化仿真模型

        Args:
            dt: 仿真步长(秒), 必须>0 (错误经验 #26)
            **kwargs: 额外参数

        Raises:
            ValueError: dt<=0 或其他参数非法
        """
        # --- 参数校验 (错误经验 #16, #26) ---
        if dt <= 0:
            raise ValueError(f"dt必须>0, 收到: {dt}")

        self.dt: float = dt
        self.time_log: List[float] = []
        self.state_log: List[float] = []
        self.output_log: List[float] = []

        # 仿真状态
        self._state: float = 0.0
        self._step: int = 0
        self._initialized: bool = True

    def step(self, control_input: float) -> float:
        """
        执行一个仿真步

        Args:
            control_input: 控制器输出

        Returns:
            当前状态值
        """
        # --- 一阶惯性模型示例 ---
        tau = 0.1  # 时间常数(秒)

        # 错误经验 #1: dt已在__init__校验, 但再次确认
        dt = self.dt if self.dt > 0 else 0.001

        # 错误经验 #1: tau不能为0
        if abs(tau) < 1e-10:
            tau = 0.1

        # 一阶系统离散化
        alpha = dt / (tau + dt)
        self._state += alpha * (control_input - self._state)

        # 记录日志
        self.time_log.append(self._step * dt)
        self.state_log.append(self._state)
        self.output_log.append(control_input)

        self._step += 1
        return self._state

    def run(self, duration: float = 5.0) -> Dict[str, np.ndarray]:
        """
        运行完整仿真

        Args:
            duration: 仿真时长(秒)

        Returns:
            包含 'time', 'state', 'output' 的字典
        """
        n_steps = int(duration / self.dt)
        setpoint = 1.0  # 阶跃输入

        for _ in range(n_steps):
            # 简单P控制器
            error = setpoint - self._state
            control = 10.0 * error
            self.step(control)

        return {
            'time': np.array(self.time_log),
            'state': np.array(self.state_log),
            'output': np.array(self.output_log),
        }

    def plot(self, save_path: Optional[str] = None):
        """
        绘制仿真结果

        Args:
            save_path: 图片保存路径。None则使用默认路径。

        注意:
            - 错误经验 #38: Agg模式下不调用plt.show()
            - 使用plt.close('all')释放内存
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        time_arr = np.array(self.time_log)
        state_arr = np.array(self.state_log)
        output_arr = np.array(self.output_log)

        # 状态曲线
        ax1.plot(time_arr, state_arr, 'b-', linewidth=1.5, label='State')
        ax1.axhline(y=1.0, color='r', linestyle='--', alpha=0.5, label='Setpoint')
        ax1.set_ylabel('State')
        ax1.set_title('仿真结果')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # 控制输出曲线
        ax2.plot(time_arr, output_arr, 'g-', linewidth=1.5, label='Control')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Control Output')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_path is None:
            save_path = os.path.join(_OUTPUT_DIR, 'simulation_result.png')

        # 错误经验 #38: 保存图片而非plt.show()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')

        # !! 永远不要在Agg模式下调用 plt.show()
        # !! plt.show()  # ← 错误! 在无头环境下会阻塞或报警告

        # 正确做法: 关闭figure释放内存
        plt.close('all')


def load_npz_data(npz_path: str) -> Dict[str, np.ndarray]:
    """
    安全加载npz文件 (错误经验 #27)

    Args:
        npz_path: npz文件路径 (使用相对路径, 错误经验 #2)

    Returns:
        数据字典

    注意:
        错误经验 #27: npz加载后必须用data['key']读取，
        不能用getattr(self, 'key', None)，后者永远返回None
    """
    if not os.path.exists(npz_path):
        raise FileNotFoundError(f"文件不存在: {npz_path}")

    data = np.load(npz_path, allow_pickle=True)

    # !! 错误写法 (错误经验 #27):
    # !! image_size = getattr(self, 'image_size', None)  # 永远返回None!

    # !! 正确写法:
    result = {}
    for key in data.files:
        result[key] = data[key]

    return result


def load_serial_data(port: str = 'COM3', baudrate: int = 115200,
                     timeout: float = 1.0) -> List[float]:
    """
    串口数据采集示例 (错误经验 #28, #29)

    Args:
        port: 串口号
        baudrate: 波特率
        timeout: 超时时间(秒)

    Returns:
        采集的数据列表

    注意:
        错误经验 #28: ser.close()不能放在循环内部
        错误经验 #29: except必须指定具体异常类型
    """
    try:
        import serial
    except ImportError:
        warnings.warn("未安装pyserial, 跳过串口采集")
        return []

    values: List[float] = []
    ser = None

    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(0.5)  # 等待串口稳定

        # !! 错误经验 #28: close() 不能放在循环内
        # !! 错误写法:
        # !! while True:
        # !!     line = ser.readline()
        # !!     values.append(float(line))
        # !!     ser.close()  # ← 第一次就关了! 后续全部失败

        # 正确写法: close在循环外
        for _ in range(100):
            line = ser.readline()
            if line:
                try:
                    values.append(float(line.decode('utf-8').strip()))
                except (ValueError, UnicodeDecodeError):
                    pass  # 跳过无效数据行

    # !! 错误经验 #29: 不要用bare except
    # !! 错误: except:  # 会吞掉KeyboardInterrupt
    # !! 正确: 指定具体异常类型
    except (serial.SerialException, OSError) as e:
        warnings.warn(f"串口错误: {e}")
    except KeyboardInterrupt:
        raise  # 不要吞掉Ctrl+C
    finally:
        if ser is not None and ser.is_open:
            ser.close()  # 在循环外关闭 (错误经验 #28)

    return values


def process_with_conditional_import(data: np.ndarray) -> np.ndarray:
    """
    处理数据 — 条件导入库的安全使用 (错误经验 #30)

    Args:
        data: 输入数据

    Returns:
        处理后的数据
    """
    # 错误经验 #30: 条件导入的库，在函数内使用前必须检查
    try:
        import scipy.signal as signal
        HAS_SCIPY = True
    except ImportError:
        HAS_SCIPY = False

    if HAS_SCIPY:
        # scipy可用时使用高级滤波
        b, a = signal.butter(4, 0.1)
        return signal.filtfilt(b, a, data)
    else:
        # scipy不可用时使用简单移动平均
        window = 5
        kernel = np.ones(window) / window
        return np.convolve(data, kernel, mode='same')


# =========================================================================
#  7. 主程序守卫 (错误经验 #3)
#
#  !! 所有仿真代码必须封装在函数中
#  !! 顶层只保留 __main__ 守卫
#  !! 否则 import此模块时就会触发仿真和绘图
# =========================================================================

def main():
    """
    主函数 — 仿真入口

    注意: 所有仿真逻辑必须在这里调用 (错误经验 #3)
    """
    print("=" * 60)
    print("  仿真启动")
    print("=" * 60)

    # --- 创建并运行仿真 ---
    sim = YourSimulation(dt=0.001)
    results = sim.run(duration=5.0)

    print(f"  仿真完成: {len(results['time'])} 步")
    print(f"  最终状态: {results['state'][-1]:.4f}")

    # --- 绘图 (保存到文件, 不弹窗) ---
    sim.plot()
    print(f"  图片已保存到: {_OUTPUT_DIR}")

    # --- 保存数据 ---
    data_path = os.path.join(_OUTPUT_DIR, 'sim_data.npz')
    np.savez(data_path,
             time=results['time'],
             state=results['state'],
             output=results['output'])
    print(f"  数据已保存到: {data_path}")

    # --- 演示npz加载 (错误经验 #27) ---
    loaded = load_npz_data(data_path)
    print(f"  验证加载: keys={list(loaded.keys())}")

    print("=" * 60)
    print("  仿真结束")
    print("=" * 60)


# =========================================================================
#  8. 入口守卫 (错误经验 #3: 必须)
# =========================================================================
if __name__ == '__main__':
    main()

    # !! 错误经验 #3: 下面这种顶层直接执行的写法是禁止的:
    # !! sim = YourSimulation()
    # !! sim.run()
    # !! sim.plot()  # ← import此模块时就会执行! 导致副作用
