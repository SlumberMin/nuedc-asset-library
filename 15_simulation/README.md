# 15_simulation

## 模块概述

本模块收录电赛控制算法的仿真验证程序和结果图，涵盖 PID 及其变种（模糊PID、自适应PID、串级PID、前馈PID）、ADRC、LQR、MPC、滑模控制、迭代学习控制、神经网络PID、Smith预估器、无差拍控制、重复控制等算法的仿真对比。同时提供控制系统设计检查清单和仿真工具。

## 目录结构

```
15_simulation/
├── README.md                          # 本文档
├── 仿真使用指南.md                    # 仿真程序使用指南
├── # === PID系列仿真 ===
├── cascade_pid_simulation.py          # 串级PID仿真
├── cascade_pid_result.png             # 串级PID仿真结果
├── feedforward_pid_simulation.py      # 前馈PID仿真
├── feedforward_pid_result.png         # 前馈PID仿真结果
├── fuzzy_pid_adaptation.py            # 模糊PID自适应仿真
├── fuzzy_pid_adaptation_result.png    # 模糊PID自适应结果
├── neural_pid_learning.py             # 神经网络PID学习仿真
├── neural_pid_learning_result.png     # 神经网络PID结果
├── # === 先进控制算法仿真 ===
├── adrc_vs_pid_simulation.py          # ADRC vs PID对比仿真
├── adrc_vs_pid_result.png             # ADRC vs PID对比结果
├── lqr_inverted_pendulum.py           # LQR倒立摆仿真
├── lqr_inverted_pendulum_result.png   # LQR倒立摆结果
├── mpc_trajectory_tracking.py         # MPC轨迹跟踪仿真
├── mpc_trajectory_tracking_result.png # MPC轨迹跟踪结果
├── smc_robustness_simulation.py       # 滑模控制鲁棒性仿真
├── smc_robustness_result.png          # 滑模控制鲁棒性结果
├── ilc_repetitive_task.py             # 迭代学习控制仿真
├── ilc_repetitive_task_result.png     # 迭代学习控制结果
├── # === 其他控制策略仿真 ===
├── smith_predictor_simulation.py      # Smith预估器仿真
├── smith_predictor_result.png         # Smith预估器结果
├── deadbeat_simulation.py             # 无差拍控制仿真
├── deadbeat_result.png                # 无差拍控制结果
├── repetitive_control_simulation.py   # 重复控制仿真
├── repetitive_control_result.png      # 重复控制结果
├── advanced_algorithms_comparison.py  # 高级算法综合对比
└── advanced_algorithms_comparison_result.png  # 综合对比结果
```

## 文件清单和说明

### PID系列仿真

| 文件 | 说明 |
|------|------|
| cascade_pid_simulation.py | 串级PID（位置环+速度环）仿真 |
| feedforward_pid_simulation.py | 前馈+PID组合控制仿真 |
| fuzzy_pid_adaptation.py | 模糊自适应PID参数调整仿真 |
| neural_pid_learning.py | 神经网络自学习PID仿真 |

### 先进控制算法仿真

| 文件 | 说明 |
|------|------|
| adrc_vs_pid_simulation.py | ADRC与传统PID性能对比 |
| lqr_inverted_pendulum.py | LQR控制倒立摆稳定仿真 |
| mpc_trajectory_tracking.py | MPC轨迹跟踪控制仿真 |
| smc_robustness_simulation.py | 滑模控制抗干扰鲁棒性仿真 |
| ilc_repetitive_task.py | 迭代学习控制重复任务仿真 |

### 其他控制策略仿真

| 文件 | 说明 |
|------|------|
| smith_predictor_simulation.py | Smith预估器处理大滞后系统 |
| deadbeat_simulation.py | 无差拍控制快速响应仿真 |
| repetitive_control_simulation.py | 重复控制周期信号跟踪仿真 |
| advanced_algorithms_comparison.py | 多种先进算法综合性能对比 |

## 使用方法

### 运行单个仿真

```bash
cd 15_simulation/
python cascade_pid_simulation.py
```

### 查看结果

仿真结果图（`*_result.png`）会在运行后自动生成，也可直接查看已有图片。

### 算法选型参考

```bash
# 运行综合对比
python advanced_algorithms_comparison.py
```

根据对比结果选择适合题目的控制算法：

| 题目特征 | 推荐算法 | 仿真文件 |
|---------|---------|---------|
| 快速响应 | 无差拍控制 | deadbeat_simulation.py |
| 大滞后系统 | Smith预估器 | smith_predictor_simulation.py |
| 姿态平衡 | LQR | lqr_inverted_pendulum.py |
| 轨迹跟踪 | MPC | mpc_trajectory_tracking.py |
| 抗干扰 | 滑模控制 | smc_robustness_simulation.py |
| 重复任务 | 迭代学习/重复控制 | ilc_repetitive_task.py |
| 通用调参 | 模糊PID | fuzzy_pid_adaptation.py |

## 依赖说明

- Python 3.8+
- NumPy
- Matplotlib
- SciPy（部分仿真需要）

```bash
pip install numpy matplotlib scipy
```

## 常见问题

**Q: 仿真结果和实际效果差很多怎么办？**
A: 仿真模型和实际系统总有差异。仿真用于验证算法思路，实际调试需重新整定参数。

**Q: 如何将仿真参数应用到实际代码？**
A: 仿真中的PID参数可直接移植到 `13_control_algorithms/` 中的C代码，但需根据实际采样周期调整。

**Q: 图片看不清怎么办？**
A: 运行对应 `.py` 文件重新生成高清图，或修改 `matplotlib` 的 DPI 参数。
