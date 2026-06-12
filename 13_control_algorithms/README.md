# 先进控制算法库 - nuedc-asset-library

## 目录结构

| 编号 | 算法 | 文件 | 说明 |
|------|------|------|------|
| 01 | ADRC自抗扰控制 | adrc.c/h, adrc_sim.py | 含ESO、NLSEF、TD完整实现 |
| 02 | MPC模型预测控制 | mpc.c/h, mpc_sim.py | 嵌入式简化实现，梯度下降求解 |
| 03 | LQR线性二次调节器 | lqr.c/h, lqr_sim.py | Riccati方程迭代求解 |
| 04 | SMC滑模控制 | smc.c/h, smc_sim.py | 4种趋近律+边界层法 |
| 05 | NN-PID神经网络PID | nn_pid.c/h, nn_pid_sim.py | 3层BP网络在线调参 |
| 06 | 模糊自适应PID | fuzzy_pid.c/h, fuzzy_pid_sim.py | 7×7模糊规则表 |
| 07 | H∞鲁棒控制 | hinf.c/h, hinf_sim.py | 修改Riccati方程求解 |
| 08 | ILC迭代学习控制 | ilc.c/h, ilc_sim.py | P/D/PD/PID四种学习律 |

## 快速使用

### C语言嵌入式
```c
#include "adrc.h"
ADRC_t adrc;
ADRC_Init(&adrc, 100, 0.01, 1.0, 15, 60, 0.05, 0.001);
ADRC_SetOutputLimit(&adrc, 100);
float u = ADRC_Update(&adrc, reference, measured_value);
```

### Python仿真
```bash
python adrc_sim.py   # 运行对应算法仿真
```

## 算法选型指南

| 场景 | 推荐算法 | 理由 |
|------|---------|------|
| 电机转速/位置控制 | ADRC | 不需要精确模型，抗扰能力强 |
| 约束优化控制 | MPC | 可处理输入/状态约束 |
| 线性系统最优控制 | LQR | 理论最优，增益固定 |
| 强扰动/不确定性系统 | SMC | 对参数变化和扰动不敏感 |
| 非线性/时变系统 | NN-PID | 在线自适应调参 |
| 模型不确定的非线性系统 | 模糊PID | 无需精确模型，利用专家经验 |
| 未建模动态系统 | H∞ | 最坏情况下保证性能 |
| 重复执行任务 | ILC | 逐次迭代逼近完美跟踪 |

## 参数整定速查

### ADRC
- omega_c (控制器带宽): 10~30，响应速度
- omega_o (观测器带宽): 3~5倍omega_c
- b0: 系统增益估计，需较准确

### MPC
- Np (预测步长): 10~30
- Nc (控制步长): 3~5
- Q/R: 状态/控制权重比

### LQR
- Q对角线: 状态惩罚，越大响应越快
- R: 控制惩罚，越大控制越小

### SMC
- c (滑模面斜率): 5~20
- eps (切换增益): 1~10
- phi (边界层): 0.01~0.1

### 模糊PID
- ke, kec: 量化因子，决定输入灵敏度
- kup, kui, kud: 输出比例因子
