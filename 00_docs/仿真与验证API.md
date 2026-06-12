# 仿真与验证 API 文档

> 版本: 1.0 | 更新日期: 2026-06-11  
> 路径: `15_simulation/`  
> 用途: 算法仿真、性能验证、参数整定的完整工具集

---

## 目录

1. [概述](#概述)
2. [环境准备](#环境准备)
3. [PID算法仿真](#pid算法仿真)
4. [电机模型仿真](#电机模型仿真)
5. [循迹算法仿真](#循迹算法仿真)
6. [滚球控制仿真](#滚球控制仿真)
7. [倒立摆仿真](#倒立摆仿真)
8. [高级算法仿真](#高级算法仿真)
9. [算法性能基准测试](#算法性能基准测试)
10. [视觉算法测试数据集](#视觉算法测试数据集)
11. [自定义修改指南](#自定义修改指南)

---

## 概述

仿真与验证模块提供电赛常用控制算法和系统的仿真程序，用于**赛前算法验证**和**参数预整定**。所有仿真均为纯Python实现，无需硬件依赖。

### 文件结构

```
15_simulation/
├── pid_simulation.py              # PID算法仿真
├── motor_simulation.py            # 电机模型仿真
├── line_tracking_simulation.py    # 循迹算法仿真
├── ball_plate_simulation.py       # 滚球控制仿真
├── pendulum_simulation.py         # 倒立摆仿真
├── fuzzy_pid_simulation.py        # 模糊PID仿真
├── adrc_simulation.py             # ADRC自抗扰仿真
├── mpc_simulation.py              # MPC模型预测仿真
├── kalman_filter_simulation.py    # 卡尔曼滤波仿真
├── sensor_fusion_simulation.py    # 传感器融合仿真
├── 仿真使用指南.md                # 使用说明
├── 算法性能基准测试.md            # 各算法性能对比
└── 视觉算法测试数据集说明.md      # 测试数据集说明
```

---

## 环境准备

### 依赖安装

```bash
pip install numpy matplotlib scipy
```

| 库 | 用途 | 必需 |
|----|------|------|
| numpy | 数值计算 | ✅ |
| matplotlib | 可视化 | ✅ |
| scipy | 倒立摆LQR求解 | 仅倒立摆仿真 |

### 快速验证

```bash
cd 15_simulation/
python pid_simulation.py
# 应生成 pid_simulation_result.png
```

---

## PID算法仿真

**文件**: `pid_simulation.py`

### 功能

对比4种PID变种在不同系统下的表现：

| PID类型 | 特点 | 适用场景 |
|---------|------|---------|
| 位置式PID | 标准实现 | 通用 |
| 增量式PID | 输出增量，无积分饱和 | 执行器增量控制 |
| 积分分离PID | 大误差时关闭积分 | 减少超调 |
| 抗积分饱和PID | 输出饱和时回推积分 | 防止积分累积 |

### 运行

```bash
python pid_simulation.py
```

### 仿真场景

| 场景 | 参数 | 说明 |
|------|------|------|
| 标准系统 | τ=0.5 | 基准一阶惯性 |
| 慢速系统 | τ=2.0 | 大惯量系统 |
| 带时延 | delay=0.3s | 通信/执行延迟 |
| 带噪声 | σ=0.05 | 传感器噪声 |

### 可调参数

```python
# 代码内修改
pid_params = (2.0, 0.5, 0.3)  # Kp, Ki, Kd
setpoint = 1.0                  # 目标值
duration = 10.0                 # 仿真时长(s)
dt = 0.01                       # 时间步长(s)
```

### 类API

#### `PIDController` 类

```python
pid = PIDController(kp=2.0, ki=0.5, kd=0.3, output_min=-100, output_max=100)
output = pid.compute(error, dt)
```

| 方法 | 参数 | 返回 | 说明 |
|------|------|------|------|
| `__init__(kp, ki, kd, output_min, output_max)` | PID增益和限幅 | - | 初始化 |
| `compute(error, dt)` | 误差, 时间步 | float | 计算输出 |

#### `IncrementalPID` 类

```python
pid = IncrementalPID(kp=2.0, ki=0.5, kd=0.3)
output = pid.compute(error, dt)
```

#### `IntegralSeparationPID` 类

```python
pid = IntegralSeparationPID(kp=2.0, ki=0.5, kd=0.3, threshold=10)
output = pid.compute(error, dt)
```

| 额外参数 | 说明 |
|---------|------|
| `threshold` | 积分分离阈值，误差大于此值时关闭积分 |

#### `AntiWindupPID` 类

```python
pid = AntiWindupPID(kp=2.0, ki=0.5, kd=0.3)
output = pid.compute(error, dt)
```

### 输出

`pid_simulation_result.png` — 4子图对比（标准/慢速/时延/噪声场景）

---

## 电机模型仿真

**文件**: `motor_simulation.py`

### 功能

模拟带死区、饱和、摩擦的直流电机，进行速度环和位置环PID控制。

### 运行

```bash
python motor_simulation.py
```

### 电机模型参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| tau | 0.3s | 电气时间常数 |
| gain | 10 | 速度增益 |
| dead_zone | 1.0V | 死区电压 |
| saturation | 12.0V | 饱和电压 |
| coulomb_friction | 0.1 | 库仑摩擦 |
| viscous_friction | 0.05 | 粘性摩擦 |

### 使用示例

```python
from motor_simulation import MotorModel

motor = MotorModel(tau=0.3, gain=10, dead_zone=1.0, saturation=12.0)
speed = motor.update(voltage=6.0, dt=0.001)
```

### 输出

`motor_simulation_result.png` — 速度响应、电压曲线、位置控制

---

## 循迹算法仿真

**文件**: `line_tracking_simulation.py`

### 功能

模拟7路红外传感器循迹小车，含弯道、S弯赛道。

### 运行

```bash
python line_tracking_simulation.py
```

### 可调参数

| 参数 | 位置 | 默认值 | 说明 |
|------|------|--------|------|
| n_sensors | SensorArray | 7 | 传感器数量 |
| spacing | SensorArray | 3.0mm | 传感器间距 |
| base_speed | CarModel | 30 mm/s | 基础速度 |
| Kp, Ki, Kd | PID | 0.08, 0.001, 0.05 | PID参数 |

### 使用示例

```python
from line_tracking_simulation import SensorArray, CarModel, Track, PIDController

track = Track()
sensors = SensorArray(n_sensors=7, spacing=3.0)
car = CarModel(base_speed=30)
pid = PIDController(kp=0.08, ki=0.001, kd=0.05)
```

### 输出

`line_tracking_result.png` — 赛道轨迹+偏差曲线

---

## 滚球控制仿真

**文件**: `ball_plate_simulation.py`

### 功能

双轴PID控制球在平板上的位置，含阶跃跟踪和圆形轨迹跟踪。

### 运行

```bash
python ball_plate_simulation.py
```

### 系统参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| radius | 0.15m | 板半径 |
| g | 9.81 m/s² | 重力加速度 |
| mu | 0.01 | 摩擦系数 |
| Kp, Ki, Kd | 2.0, 0.5, 0.8 | PID参数 |

### 自定义目标轨迹

```python
# 编辑 simulate_trajectory() 中的 target_points
target_points = [
    (0.05, 0.05),   # 第1个目标
    (0.10, 0.00),   # 第2个目标
    (0.00, 0.10),   # 第3个目标
]
```

### 输出

`ball_plate_result.png` — XY轨迹、阶跃响应、速度、跟踪误差

---

## 倒立摆仿真

**文件**: `pendulum_simulation.py`

### 功能

LQR最优控制倒立摆，含噪声鲁棒性分析。

### 运行

```bash
python pendulum_simulation.py
```

> **注意**: 需要 `scipy` 库 (`pip install scipy`)

### 系统参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| M | 0.5 kg | 小车质量 |
| m | 0.2 kg | 摆杆质量 |
| l | 0.3 m | 摆杆半长 |
| b | 0.1 | 摩擦系数 |

### LQR权重矩阵

```python
Q = np.diag([10, 1, 100, 1])  # 状态权重 [位置, 速度, 角度, 角速度]
R = np.array([[0.1]])          # 控制权重
```

### 输出

`pendulum_result.png` — 角度响应、噪声对比、位移、控制力

---

## 高级算法仿真

### 模糊PID仿真

**文件**: `fuzzy_pid_simulation.py`

7×7规则表在线自整定Kp/Ki/Kd，对比标准PID。

### ADRC自抗扰仿真

**文件**: `adrc_simulation.py`

自抗扰控制器仿真，含扩张状态观测器(ESO)。

### MPC模型预测仿真

**文件**: `mpc_simulation.py`

模型预测控制仿真，对比PID的跟踪性能。

### 卡尔曼滤波仿真

**文件**: `kalman_filter_simulation.py`

一维/二维卡尔曼滤波，演示噪声抑制效果。

### 传感器融合仿真

**文件**: `sensor_fusion_simulation.py`

加速度计+陀螺仪融合，演示互补滤波和卡尔曼融合。

---

## 算法性能基准测试

**文件**: `算法性能基准测试.md`

### 测试指标

| 指标 | 说明 | 计算方法 |
|------|------|---------|
| 上升时间 | 从10%到90%目标值 | 时间差 |
| 超调量 | 超过目标值的最大百分比 | (峰值-目标)/目标×100% |
| 稳定时间 | 进入±2%误差带的时间 | 时间 |
| 稳态误差 | 稳定后与目标的偏差 | 均值误差 |
| ITAE | 时间加权绝对误差积分 | Σ(t×|e|)×dt |

### 各算法对比

| 算法 | 上升时间 | 超调量 | 稳定时间 | 抗干扰 | 实现难度 |
|------|---------|--------|---------|--------|---------|
| 标准PID | 快 | 中 | 中 | 中 | ★☆☆ |
| 模糊PID | 中 | 小 | 快 | 强 | ★★☆ |
| ADRC | 快 | 小 | 快 | 很强 | ★★★ |
| LQR | 中 | 无 | 快 | 强 | ★★☆ |
| MPC | 中 | 小 | 快 | 很强 | ★★★ |

---

## 视觉算法测试数据集

**文件**: `视觉算法测试数据集说明.md`

### 数据集内容

| 数据集 | 内容 | 用途 |
|--------|------|------|
| 颜色目标 | 不同光照下的红/蓝/绿目标 | 颜色追踪测试 |
| 循迹线 | 直线/弯道/S弯赛道图 | 循迹算法测试 |
| 棋盘格 | 标定板图片 | 相机标定 |
| ArUco | 不同角度的标记图 | 标记检测测试 |

---

## 自定义修改指南

### 修改PID参数

所有PID控制器参数均在代码中显式定义，搜索 `kp`, `ki`, `kd` 即可修改。

```python
# 示例：修改pid_simulation.py中的参数
pid_params = (2.0, 0.5, 0.3)  # 改为你的参数
```

### 修改系统参数

```python
# 电机：修改 MotorModel(...) 构造参数
motor = MotorModel(tau=0.3, gain=10, dead_zone=1.0)

# 球板：修改 BallOnPlate(...) 和 AxisPID(...)
plate = BallOnPlate(radius=0.15, mu=0.01)

# 倒立摆：修改 InvertedPendulum(...) 和 LQR权重
Q = np.diag([10, 1, 100, 1])
R = np.array([[0.1]])
```

### 添加新赛道

在 `line_tracking_simulation.py` 的 `Track._generate_track()` 中添加新点段：

```python
def _generate_track(self):
    # 已有: 直线、弯道、S弯
    # 添加新赛道段:
    points = [
        (0, 0), (100, 0),        # 直线段
        (150, 50), (100, 100),   # 弯道段
        (50, 50), (0, 0),        # 返回段
    ]
```

### 导出数据

所有仿真内部均有数据数组，可在 `main()` 函数末尾添加：

```python
# 导出CSV
np.savetxt('data.csv', np.column_stack([t, y, u]),
           delimiter=',', header='t,y,u')

# 导出JSON
import json
data = {'time': t.tolist(), 'output': y.tolist(), 'control': u.tolist()}
with open('data.json', 'w') as f:
    json.dump(data, f)
```

### 输出文件清单

| 文件 | 仿真程序 | 内容 |
|------|---------|------|
| `pid_simulation_result.png` | pid_simulation.py | PID对比图 |
| `motor_simulation_result.png` | motor_simulation.py | 电机响应图 |
| `line_tracking_result.png` | line_tracking_simulation.py | 循迹轨迹图 |
| `ball_plate_result.png` | ball_plate_simulation.py | 滚球控制图 |
| `pendulum_result.png` | pendulum_simulation.py | 倒立摆响应图 |

### 快速使用流程

```
1. 选择仿真场景
   └── 根据比赛题目选择对应的仿真程序

2. 运行仿真
   └── python xxx_simulation.py

3. 查看结果
   └── 打开生成的 .png 图片

4. 调整参数
   └── 修改代码中的PID/系统参数

5. 重新运行
   └── 对比不同参数的效果

6. 记录最优参数
   └── 保存到配置文件，比赛时直接使用
```
