<div align="center">

<img src="https://img.icons8.com/fluency/96/cpu.png" width="80" alt="logo"/>

# ⚡ 电赛综合资产库

**全国大学生电子设计竞赛 · 控制+视觉 综合资产库**

[![English](https://img.shields.io/badge/English-README_EN-blue?style=flat-square&logo=googletranslate&logoColor=white)](README_EN.md)
[![中文](https://img.shields.io/badge/中文-README-red?style=flat-square&logo=googletranslate&logoColor=white)](README.md)

---

[![release](https://img.shields.io/badge/release-v1.0.0-blue?style=flat-square&logo=github)](https://github.com/SlumberMin/nuedc-asset-library/releases)
[![license](https://img.shields.io/badge/license-MIT-green?style=flat-square)](LICENSE)
[![platform](https://img.shields.io/badge/platform-STM32%20%7C%20MSPM0%20%7C%20TM4C%20%7C%20OPi5-orange?style=flat-square)]()
[![vision](https://img.shields.io/badge/vision-NPU%20%7C%20GPU%20%7C%20RGA-9cf?style=flat-square&logo=opencv)]()

[![files](https://img.shields.io/badge/files-1%2C966-purple?style=flat-square&logo=files)]()
[![algorithms](https://img.shields.io/badge/algorithms-21-blueviolet?style=flat-square&logo=brain)]()
[![solutions](https://img.shields.io/badge/solutions-37-yellow?style=flat-square&logo=lightning)]()
[![tests](https://img.shields.io/badge/tests-3%2C000%2B-brightgreen?style=flat-square&logo=flask)]()

<br/>

**一套覆盖 STM32 / MSPM0G3507 / TM4C123 三平台 + Orange Pi 5 视觉的完整电赛备赛资源库**

[📦 快速开始](#-快速开始) •
[📂 目录结构](#-目录结构) •
[🧮 算法库](#-控制算法库) •
[📝 赛题方案](#-历年赛题方案) •
[🤝 贡献](#-贡献)

</div>

---

## ✨ 特性一览

<table>
<tr>
<td width="50%">

### 🔌 三平台驱动库
- **STM32F103** — 18 个驱动
- **MSPM0G3507** — 79 个驱动
- **TM4C123** — 12 个驱动
- 同一传感器，三套驱动代码
- 详细中文注释，可直接编译

</td>
<td width="50%">

### 👁️ Orange Pi 5 视觉系统
- **NPU** — RKNN 模型推理
- **GPU** — OpenGL 加速
- **RGA** — 硬件图像缩放/旋转
- **相机驱动** — USB3.0 全局快门
- YOLOv8 / 语义分割 / 目标跟踪

</td>
</tr>
<tr>
<td width="50%">

### 🧠 21 种控制算法
- PID 全家族（位置式/增量式/串级/自适应/模糊）
- ADRC / LADRC 自抗扰
- MPC 模型预测 / LQR 线性二次
- SMC 滑模 / 超螺旋滑模
- 卡尔曼 / 扩展卡尔曼 / 龙贝格观测器

</td>
<td width="50%">

### 📝 37 道历年赛题方案
- 2009—2025 年控制类赛题
- 完整 STM32 工程代码
- 系统方案 + 硬件原理图 + 设计报告
- 测试数据 + 误差分析

</td>
</tr>
<tr>
<td width="50%">

### 🧪 3000+ 测试用例
- 驱动层单元测试
- 算法层功能测试
- 系统集成测试
- 边界条件 & 异常测试

</td>
<td width="50%">

### 🔧 59 个自动化工具
- 代码质量审计
- 编译检查 & 自动修复
- SysConfig 配置转换
- 性能分析 & 评分

</td>
</tr>
</table>

---

## 🚀 快速开始

### 1️⃣ 克隆仓库

```bash
git clone https://github.com/SlumberMin/nuedc-asset-library.git
cd nuedc-asset-library
```

### 2️⃣ 电机驱动示例（MSPM0G3507 + TB6612）

```c
#include "drivers/tb6612.h"

int main(void)
{
    TB6612_Init();
    TB6612_MotorInit(MOTOR_A, GPIO_PWM_0_C0_IDX);
    TB6612_SetSpeed(MOTOR_A, MOTOR_FORWARD, 500);  // 50% 速度

    while (1) { /* 主循环 */ }
}
```

### 3️⃣ 串级 PID 示例

```c
#include "common/cascade_pid.h"

CascadePID_t pid;

void setup(void) {
    CascadePID_Init(&pid);
    pid.outer.Kp = 2.0f;  pid.outer.Ki = 0.5f;  pid.outer.Kd = 0.1f;
    pid.inner.Kp = 5.0f;  pid.inner.Ki = 1.0f;  pid.inner.Kd = 0.05f;
}

float control_loop(float pos_ref, float pos_fb, float vel_fb) {
    return CascadePID_Calc(&pid, pos_ref, pos_fb, vel_fb);
}
```

### 4️⃣ Python 仿真

```bash
cd 15_simulation
pip install numpy matplotlib
python pid_simulation.py
```

---

## 📂 目录结构

```
nuedc-asset-library/
├── 00_docs/                      📖 使用指南 & API 文档
├── 01_stm32/                     🔧 STM32F103 驱动 & 算法
├── 02_mspm0g3507/                🔧 MSPM0G3507 驱动 & 示例
├── 03_hardware_modules/          🔩 硬件模块原理图 & 封装
├── 04_tm4c123/                   🔧 TM4C123 驱动
├── 05_orangepi5/                 🖥️ OPi5 控制通信框架
├── 06_circuit_templates/         ⚡ 电机/传感器/电源电路
├── 07_report_templates/          📄 电赛设计报告模板
├── 08_vision_drivers/            📷 USB 相机 / GStreamer
├── 09_hw_acceleration/           ⚡ RGA / GPU / NPU 加速
├── 10_opencv/                    👁️ OpenCV 交叉编译
├── 11_npu_models/                🧠 YOLOv8 / 分割 / 跟踪
├── 12_vision_common/             👁️ 标定 / 检测 / 跟踪 / 决策
├── 13_control_algorithms/        🎯 21 种控制算法 C 实现
├── 14_solutions/                 💡 典型工程问题方案
├── 15_simulation/                📊 Python 仿真（控制 + 电机 + 传感器）
├── 16_templates/                 📝 C / Python 项目模板
├── 17_competition_solutions/     🏆 37 道历年赛题完整代码
├── 18_robotics/                  🤖 RoboMaster / WRC 机器人竞赛
├── 19_opensource_analysis/       📚 GitHub 高星项目分析
├── tests/                        🧪 3000+ 测试用例
└── tools/                        🔧 59 个自动化工具
```

---

## 🧮 控制算法库

<table>
<tr>
<td>

| 算法 | 文件 | 场景 |
|------|------|------|
| PID（位置式） | `pid_full.c` | 通用 |
| PID（增量式） | `incremental_pid.c` | 防冲击 |
| 串级 PID | `cascade_pid.c` | 双环控制 |
| 自适应 PID | `adaptive_pid.c` | 变参数 |
| 模糊 PID | `fuzzy_pid.c` | 非线性 |
| ADRC | `adrc.c` | 强扰动 |
| LADRC | `ladrc.c` | 简化 ADRC |
| MPC | `mpc_simple.c` | 约束优化 |
| LQR | `lqr.c` | 最优控制 |
| 滑模 SMC | `sliding_mode.c` | 鲁棒控制 |
| 超螺旋 SMC | `super_twisting_smc.c` | 消抖振 |

</td>
<td>

| 算法 | 文件 | 场景 |
|------|------|------|
| Smith 预估器 | `smith_predictor.c` | 大滞后 |
| 卡尔曼滤波 | `kalman.c` | 状态估计 |
| 扩展卡尔曼 | `ekf.c` | 非线性估计 |
| 龙贝格观测器 | `luenberger_observer.c` | 状态观测 |
| 扰动观测器 | `disturbance_observer.c` | 扰动补偿 |
| 前馈控制 | `feedforward.c` | 消除稳态误差 |
| 重复控制 | `repetitive_control.c` | 周期扰动 |
| 轨迹生成器 | `trajectory_generator.c` | 运动规划 |
| 路径跟踪 | `path_tracker.c` | Pure Pursuit |
| 运动控制 | `motion_control.c` | 梯形/S 曲线 |
| 状态反馈 | `state_feedback.c` | 极点配置 |

</td>
</tr>
</table>

---

## 📝 历年赛题方案

> 每题包含：**完整 STM32 工程代码** + **系统方案设计** + **硬件原理图** + **设计报告** + **测试数据**

| 年份 | 赛题 | 核心算法 | 报告 |
|:----:|------|:--------:|:----:|
| 2025 | E 题 · 简易自行瞄准装置 | 视觉追踪 + 云台 PID | ✅ |
| 2024 | H 题 · 自动行驶小车 | 循迹 PID + 编码器 | ✅ |
| 2024 | F 题 · 磁悬浮实验装置 | PID + 电磁控制 | ✅ |
| 2023 | E 题 · 运动目标控制与追踪 | 视觉 + ADRC | ✅ |
| 2022 | H 题 · 小车跟随行驶 | 超声波 + 串级 PID | ✅ |
| 2021 | F 题 · 智能送药小车 | 循迹 + 避障 + PID | ✅ |
| 2020 | C 题 · 坡道行驶电动小车 | 编码器 + PID | ✅ |
| 2019 | H 题 · 模拟电磁曲射炮 | 弹道计算 + PID | ✅ |
| 2017 | B 题 · 滚球控制系统 | MPU6050 + LQR | ✅ |
| 2015 | B 题 · 风力摆控制系统 | IMU + PID + ADRC | ✅ |
| 2013 | C 题 · 旋转倒立摆 | LQR + 卡尔曼 | ✅ |
| 2011 | F 题 · 帆板控制系统 | 角度 + PID | ✅ |
| 2009 | E 题 · 模拟路灯控制系统 | 光感 + 定时 | ✅ |
| ... | 更多赛题见 `17_competition_solutions/` | ... | ... |

---

## 📊 硬件支持

### 主控平台

| 平台 | 芯片 | 内核 | 主频 | 驱动数 |
|:----:|------|:----:|:----:|:------:|
| 🔵 STM32 | STM32F103C8T6 | Cortex-M3 | 72MHz | 18 |
| 🟢 MSPM0 | TI MSPM0G3507 | Cortex-M0+ | 80MHz | 79 |
| 🟠 TM4C | TI TM4C123GH6PZ | Cortex-M4F | 80MHz | 12 |
| 🟣 OPi5 | RK3588S | 4×A76 + 4×A55 | 2.4GHz | 视觉 |

### 传感器 & 执行器

| 类别 | 器件 | 接口 | 平台 |
|:----:|------|:----:|:----:|
| 🎯 IMU | JY901S | UART | 全平台 |
| ⬛ 灰度 | 愿为 8 路 | ADC | MSPM0 |
| 📡 超声波 | HC-SR04 | GPIO | 全平台 |
| 🎨 颜色 | TCS34725 | I2C | STM32/MSPM0 |
| 🌡️ 温湿度 | SHT20/SHT30 | I2C | STM32/MSPM0 |
| 💨 空气 | SGP30 | I2C | MSPM0 |
| ⚙️ 电机 | N20 编码电机 | PWM+编码器 | 全平台 |
| 🔌 驱动 | TB6612 / L298N | PWM+GPIO | 全平台 |
| 🦾 舵机 | SG90 | PWM | STM32/MSPM0 |
| 🎛️ 舵机驱动 | PCA9685 | I2C | 全平台 |
| 📺 显示 | SSD1306 OLED | I2C | STM32/MSPM0 |
| 📶 蓝牙 | HC-05 | UART | STM32/MSPM0 |

---

## 🔧 工具链

| 平台 | 工具链 | IDE |
|:----:|--------|-----|
| STM32 | arm-none-eabi-gcc | Keil / STM32CubeIDE |
| MSPM0G3507 | ti-cgt-armllvm 4.0.2 | CCS / SysConfig |
| TM4C123 | arm-none-eabi-gcc | CCS / Keil |
| Python | Python 3.8+ | 任意 |

---

## 📖 文档索引

| 文档 | 说明 |
|------|------|
| 📖 [快速使用指南](00_docs/电赛资产库快速使用指南.md) | 入门必读 |
| 📘 [STM32 API](00_docs/STM32代码库API.md) | 驱动接口文档 |
| 📗 [控制算法 API](00_docs/控制算法库API.md) | 算法接口文档 |
| 📙 [视觉代码 API](00_docs/视觉代码库API.md) | 视觉模块文档 |
| 📕 [硬件模块 API](00_docs/硬件模块库API.md) | 硬件驱动文档 |
| 📋 [三平台对照表](00_docs/三单片机平台差异对照表.md) | 跨平台参考 |
| 🤝 [贡献指南](CONTRIBUTING.md) | 如何参与贡献 |

---

## 🤝 贡献

欢迎 Issue 和 PR！请阅读 [📄 贡献指南](CONTRIBUTING.md)。

**贡献方向：**
- 🏆 补充更多历年赛题方案
- 🔌 增加新传感器/执行器驱动
- ⚡ 优化控制算法性能
- 🧪 完善测试用例
- 📖 改进文档

---

## 📄 许可证

本项目采用 [MIT 许可证](LICENSE) 开源。

```
MIT License — 自由使用、修改、分发
```

---

## 🙏 致谢

<div align="center">

![TI](https://img.shields.io/badge/Texas%20Instruments-E31837?style=flat-square&logo=texasinstruments&logoColor=white)
![ST](https://img.shields.io/badge/STMicroelectronics-03234B?style=flat-square&logo=stmicroelectronics&logoColor=white)
![ARM](https://img.shields.io/badge/ARM%20Cortex-0091BD?style=flat-square&logo=arm&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=flat-square&logo=opencv&logoColor=white)

全国大学生电子设计竞赛组委会 · TI MSPM0/TM4C 社区 · ST STM32 HAL · 开源控制算法贡献者

</div>

---

<div align="center">

**⚡ 备赛提示：从 `00_docs/电赛资产库快速使用指南.md` 开始，选择你的硬件平台对应的驱动目录。**

<br/>

![visitors](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FSlumberMin%2Fnuedc-asset-library.json&style=flat-square&color=blue&label=visitors&logo=github)

</div>
