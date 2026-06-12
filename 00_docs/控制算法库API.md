# 控制算法库 API 文档

> 版本: 1.0 | 更新日期: 2026-06-10  
> 路径: `11_控制算法库/`  
> 支持平台: STM32 (C语言) / Orange Pi 5 (Python)

---

## 目录

1. [概述](#概述)
2. [PID 控制器 (完整版)](#pid-控制器完整版)
3. [模糊 PID](#模糊-pid)
4. [ADRC 自抗扰](#adrc-自抗扰)
5. [卡尔曼滤波器](#卡尔曼滤波器)
6. [LQR 最优控制](#lqr-最优控制)
7. [滑模控制](#滑模控制)
8. [MPC 模型预测控制](#mpc-模型预测控制)
9. [自动整定](#自动整定)
10. [算法选型指南](#算法选型指南)

---

## 概述

控制算法库提供从经典 PID 到先进控制算法的完整实现，覆盖电赛常见控制需求。

### 文件结构

```
11_控制算法库/
├── common/                    # 通用算法 (纯C，跨平台)
│   ├── pid_full.h / .c        # PID完整版(含多种高级功能)
│   ├── fuzzy_pid.h / .c       # 模糊PID
│   ├── adrc.h / .c            # ADRC自抗扰
│   ├── kalman.h / .c          # 卡尔曼滤波
│   ├── lqr.h / .c             # LQR最优控制
│   ├── sliding_mode.h / .c    # 滑模控制
│   └── mpc_simple.h / .c      # 简化MPC
├── stm32/
│   └── pid_stm32.h / .c       # STM32平台适配版
├── orangepi5/
│   └── pid_orangepi5.py       # Orange Pi 5 Python版
├── simulation/
│   ├── pid_simulation.py      # PID仿真
│   └── auto_tune.py           # 自动整定(Ziegler-Nichols等)
└── docs/
    ├── 控制算法选型指南.md
    └── 现场快速参数整定方法.md
```

---

## PID 控制器 (完整版)

**文件**: `common/pid_full.h` / `common/pid_full.c`

标准 PID 的增强版，支持所有高级特性。

### 类型定义

```c
typedef struct {
    float kp, ki, kd;           // PID增益
    float target;               // 目标值
    float output;               // 输出值
    float output_min, output_max;  // 输出限幅
    float integral;             // 积分项
    float integral_max;         // 积分限幅
    float dead_zone;            // 死区
    float feedforward;          // 前馈
    float derivative_filter;    // 微分滤波系数
    float dt;                   // 控制周期
    // ... 内部状态
} PID_Full_t;
```

### API 函数

| 函数 | 说明 |
|---|---|
| `PID_Full_Init(pid, kp, ki, kd, dt)` | 初始化 |
| `PID_Full_SetTarget(pid, target)` | 设置目标值 |
| `PID_Full_SetOutputLimit(pid, min, max)` | 设置输出限幅 |
| `PID_Full_SetIntegralLimit(pid, max)` | 设置积分限幅 |
| `PID_Full_SetDeadZone(pid, zone)` | 设置死区 |
| `PID_Full_SetFilter(pid, alpha)` | 设置微分滤波系数 |
| `PID_Full_SetFeedforward(pid, ff)` | 设置前馈 |
| `PID_Full_Calculate(pid, feedback)` | 计算输出 |
| `PID_Full_Reset(pid)` | 重置状态 |

---

## 模糊 PID

**文件**: `common/fuzzy_pid.h` / `common/fuzzy_pid.c`

7×7 规则表在线自整定 Kp/Ki/Kd，适合非线性、时变系统。

### 原理

根据误差 `e` 和误差变化率 `ec` 的模糊语言值 (NB/NM/NS/ZO/PS/PM/PB)，通过规则表查表得到 PID 参数的调整量 ΔKp、ΔKi、ΔKd。

### 类型定义

```c
typedef struct {
    float kp_base, ki_base, kd_base;  // 基准参数
    float kp, ki, kd;                 // 当前调整后参数
    float target, output;
    float delta_kp_max, delta_ki_max, delta_kd_max;  // 调整量范围
    float e_scale, ec_scale;          // 量化因子
    int8_t rule_kp[7][7];             // Kp规则表
    int8_t rule_ki[7][7];             // Ki规则表
    int8_t rule_kd[7][7];             // Kd规则表
    float output_max, output_min;
    float integral_max;
    // ... 内部状态
} FuzzyPID_t;
```

### API 函数

| 函数 | 说明 |
|---|---|
| `FuzzyPID_Init(fuzzy, kp, ki, kd)` | 初始化 (使用默认规则表) |
| `FuzzyPID_SetDefaultRules(fuzzy)` | 设置默认 7×7 规则表 |
| `FuzzyPID_SetDeltaRange(fuzzy, dkp, dki, dkd)` | 设置调整量范围 |
| `FuzzyPID_SetScale(fuzzy, e_scale, ec_scale)` | 设置量化因子 |
| `FuzzyPID_SetTarget(fuzzy, target)` | 设置目标值 |
| `FuzzyPID_SetOutputLimit(fuzzy, min, max)` | 设置输出限幅 |
| `FuzzyPID_Calculate(fuzzy, measurement)` | 计算输出 |
| `FuzzyPID_Reset(fuzzy)` | 重置 |
| `FuzzyPID_GetParams(fuzzy, &kp, &ki, &kd)` | 获取当前参数 (调试) |

### 使用场景

- 温度控制 (非线性加热/散热)
- 液位控制
- 电机调速 (负载变化大)

---

## ADRC 自抗扰

**文件**: `common/adrc.h` / `common/adrc.c`

由跟踪微分器 (TD) + 扩张状态观测器 (ESO) + 非线性状态误差反馈 (NLSEF) 三部分组成。

### 原理

```
目标 → TD → 安排过渡过程
           ↘
             NLSEF → 控制输出 → 被控对象 → 输出
           ↗                ↓
     ESO ← 观测总扰动 ← 补偿
```

### 类型定义

```c
typedef struct {
    ADRC_TD_t td;       // 跟踪微分器
    ADRC_ESO_t eso;     // 扩张状态观测器
    ADRC_NLSEF_t nlsef; // 非线性状态误差反馈
    float h;            // 采样步长
    float b;            // 系统增益估计
    float output;
} ADRC_t;
```

### API 函数

| 函数 | 说明 |
|---|---|
| `ADRC_Init(adrc, h, b)` | 初始化 (h=采样步长, b=系统增益估计) |
| `ADRC_SetTD(adrc, r)` | 设置 TD 参数 (r=速度因子, 推荐 5~100) |
| `ADRC_SetESO(adrc, b1, b2, b3)` | 设置 ESO 增益 (推荐 10~100) |
| `ADRC_SetNLSEF(adrc, b0, b1, a0, a1)` | 设置 NLSEF 参数 |
| `ADRC_Calculate(adrc, target, measurement)` | 计算输出 |
| `ADRC_Reset(adrc)` | 重置 |

### 调参指南

| 参数 | 推荐范围 | 作用 |
|---|---|---|
| `r` (TD) | 10~50 | 越大跟踪越快，但噪声敏感 |
| `beta1` (ESO) | 30~100 | 观测器带宽 |
| `beta2` (ESO) | 300~1000 | 速度观测 |
| `beta3` (ESO) | 1000~5000 | 扰动观测 |
| `b` | 系统增益 | 需要估计 (可通过实验获得) |

---

## 卡尔曼滤波器

**文件**: `common/kalman.h` / `common/kalman.c`

标准卡尔曼滤波 (2 状态: 位置 + 速度) 和一阶互补滤波。

### `Kalman_t` 结构体

```c
typedef struct {
    float x[2];     // 状态: [位置, 速度]
    float P[2][2];  // 协方差矩阵
    float Q[2][2];  // 过程噪声
    float R;         // 测量噪声
    float H[2];      // 观测矩阵
    float dt;        // 采样周期
} Kalman_t;
```

### API 函数

| 函数 | 说明 |
|---|---|
| `Kalman_Init(kf, dt, process_noise, measure_noise)` | 初始化 |
| `Kalman_SetNoise(kf, Q_pos, Q_vel, R)` | 设置噪声参数 |
| `Kalman_Update(kf, measurement)` | 更新 (返回滤波后位置) |
| `Kalman_GetPosition(kf)` | 获取估计位置 |
| `Kalman_GetVelocity(kf)` | 获取估计速度 |
| `Kalman_Reset(kf)` | 重置 |

### 互补滤波

| 函数 | 说明 |
|---|---|
| `Complementary_Init(cf, alpha)` | 初始化 (alpha=0~1) |
| `Complementary_Update(cf, value)` | 更新滤波值 |

**互补滤波**: `output = alpha * new_value + (1 - alpha) * old_value`

---

## LQR 最优控制

**文件**: `common/lqr.h` / `common/lqr.c`

离散时间 LQR，最小化二次型代价函数。

### 类型定义

```c
typedef struct {
    uint8_t n;                              // 状态维度 (最大4)
    float A[4][4];                          // 状态矩阵
    float B[4];                             // 输入矩阵
    float Q[4][4];                          // 状态权重
    float R;                                // 控制权重
    float K[4];                             // 反馈增益 (自动计算)
    float x[4];                             // 状态向量
    float output, output_max, output_min;
} LQR_t;
```

### API 函数

| 函数 | 说明 |
|---|---|
| `LQR_Init(lqr, n)` | 初始化 (n=状态维度) |
| `LQR_SetSystem(lqr, A, B)` | 设置系统模型 |
| `LQR_SetWeight(lqr, Q, R)` | 设置权重矩阵 |
| `LQR_ComputeGain(lqr)` | 离线迭代求解 Riccati 方程，得到 K |
| `LQR_Calculate(lqr, state, target)` | 计算输出: `u = -K * (x - target)` |
| `LQR_SetOutputLimit(lqr, min, max)` | 设置输出限幅 |
| `LQR_Reset(lqr)` | 重置 |

### 使用场景

- 倒立摆平衡
- 姿态控制
- 小车平衡

---

## 滑模控制

**文件**: `common/sliding_mode.h` / `common/sliding_mode.c`

滑模控制 + 趋近律 + 抖振抑制。

### 趋近律类型

| 类型 | 公式 | 说明 |
|---|---|---|
| `SMC_REACH_RATE` | `u = -k*sgn(s)` | 等速趋近律 |
| `SMC_EXP_RATE` | `u = -k*sgn(s) - ε*s` | 指数趋近律 |
| `SMC_POW_RATE` | `u = -k*|s|^α*sgn(s)` | 幂次趋近律 |

### API 函数

| 函数 | 说明 |
|---|---|
| `SMC_Init(smc, c, k)` | 初始化 (c=滑模面斜率, k=趋近速度) |
| `SMC_SetReachingLaw(smc, law, k, epsilon, alpha)` | 设置趋近律 |
| `SMC_SetBoundaryLayer(smc, boundary)` | 设置边界层厚度 (抖振抑制) |
| `SMC_SetOutputLimit(smc, min, max)` | 设置输出限幅 |
| `SMC_Calculate(smc, target, measurement, measurement_dot)` | 计算输出 |
| `SMC_Reset(smc)` | 重置 |

### 调参建议

- `c` (滑模面): 根据期望收敛速度设定，通常 5~20
- `k` (趋近速度): 过大会加剧抖振，推荐 5~50
- `boundary_layer`: 抖振抑制，推荐 0.1~1.0

---

## MPC 模型预测控制

**文件**: `common/mpc_simple.h` / `common/mpc_simple.c`

简化 MPC，梯度法 QP 求解，适合嵌入式。

### 类型定义

```c
typedef struct {
    float A[4];         // 状态矩阵 (2×2)
    float B[2];         // 输入矩阵 (2×1)
    float C[2];         // 输出矩阵 (1×2)
    uint8_t Np;         // 预测时域
    uint8_t Nc;         // 控制时域
    float Q;            // 误差权重
    float R;            // 控制增量权重
    float u_min, u_max; // 控制量约束
    float du_min, du_max; // 控制增量约束
    float learning_rate;
    uint8_t max_iterations;
    float x[2], u_last, output;
} MPC_t;
```

### API 函数

| 函数 | 说明 |
|---|---|
| `MPC_Init(mpc, A11, A12, A21, A22, B1, B2, C1, C2)` | 初始化模型 |
| `MPC_SetHorizon(mpc, Np, Nc)` | 设置预测/控制时域 |
| `MPC_SetWeight(mpc, Q, R)` | 设置权重 |
| `MPC_SetConstraint(mpc, u_min, u_max, du_min, du_max)` | 设置约束 |
| `MPC_SetSolver(mpc, lr, max_iter)` | 设置求解器参数 |
| `MPC_Calculate(mpc, target, measurement)` | 计算输出 |
| `MPC_Reset(mpc)` | 重置 |

---

## 自动整定

**文件**: `simulation/auto_tune.py`

### 支持方法

| 方法 | 函数 | 说明 |
|---|---|---|
| Ziegler-Nichols | `ziegler_nichols(Ku, Tu)` | 经典临界振荡法 |
| Cohen-Coon | `cohen_coon(K, L, T)` | 基于阶跃响应 |
| 继电反馈 | `relay_feedback(system)` | 自动获取临界参数 |

---

## 算法选型指南

| 场景 | 推荐算法 | 理由 |
|---|---|---|
| 电机调速 | PID / FuzzyPID | 简单有效，模糊PID应对负载变化 |
| 循迹小车 | PID + 前馈 | 线路已知可加前馈 |
| 平衡车 | LQR / ADRC | 多状态耦合，需要状态观测 |
| 倒立摆 | LQR / SMC | 需要强鲁棒性 |
| 机械臂 | MPC | 有约束，需轨迹规划 |
| 温度控制 | FuzzyPID | 非线性，大滞后 |
| 高精度位置 | ADRC | 自动补偿扰动 |
| 传感器融合 | Kalman | 最优估计 |

> 详细选型指南参见 `11_控制算法库/docs/控制算法选型指南.md`
