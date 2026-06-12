<div align="center">

<img src="https://img.icons8.com/fluency/96/cpu.png" width="80" alt="logo"/>

# ⚡ NUEDC Asset Library

**National Electronic Design Competition · Control & Vision Comprehensive Asset Library**

[![中文](https://img.shields.io/badge/中文-README-red?style=flat-square&logo=googletranslate&logoColor=white)](README.md)
[![English](https://img.shields.io/badge/English-README_EN-blue?style=flat-square&logo=googletranslate&logoColor=white)](README_EN.md)

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

**A complete competition resource library covering STM32 / MSPM0G3507 / TM4C123 + Orange Pi 5 vision**

[📦 Quick Start](#-quick-start) •
[📂 Structure](#-directory-structure) •
[🧮 Algorithms](#-control-algorithms) •
[📝 Solutions](#-competition-solutions-20092025) •
[🤝 Contributing](#-contributing)

</div>

---

## ✨ Highlights

<table>
<tr>
<td width="50%">

### 🔌 3 MCU Platform Drivers
- **STM32F103** — 18 drivers
- **MSPM0G3507** — 79 drivers
- **TM4C123** — 12 drivers
- Same sensor, 3 driver implementations
- Detailed Chinese comments, directly compilable

</td>
<td width="50%">

### 👁️ Orange Pi 5 Vision System
- **NPU** — RKNN model inference
- **GPU** — OpenGL acceleration
- **RGA** — Hardware image scaling/rotation
- **Camera** — USB3.0 global shutter
- YOLOv8 / Segmentation / Tracking

</td>
</tr>
<tr>
<td width="50%">

### 🧠 21 Control Algorithms
- PID family (position/incremental/cascade/adaptive/fuzzy)
- ADRC / LADRC (Active Disturbance Rejection)
- MPC / LQR (Optimal Control)
- SMC / Super-Twisting SMC
- Kalman / EKF / Luenberger Observer

</td>
<td width="50%">

### 📝 37 Past Competition Solutions
- 2009–2025 control-type problems
- Complete STM32 project code
- System design + schematics + reports
- Test data + error analysis

</td>
</tr>
<tr>
<td width="50%">

### 🧪 3000+ Test Cases
- Driver unit tests
- Algorithm functional tests
- System integration tests
- Boundary & exception tests

</td>
<td width="50%">

### 🔧 59 Automation Tools
- Code quality auditing
- Build checks & auto-fix
- SysConfig converter
- Performance analysis & scoring

</td>
</tr>
</table>

---

## 📦 Quick Start

### 1️⃣ Clone Repository

```bash
git clone https://github.com/SlumberMin/nuedc-asset-library.git
cd nuedc-asset-library
```

### 2️⃣ Motor Driver Example (MSPM0G3507 + TB6612)

```c
#include "drivers/tb6612.h"

int main(void)
{
    TB6612_Init();
    TB6612_MotorInit(MOTOR_A, GPIO_PWM_0_C0_IDX);
    TB6612_SetSpeed(MOTOR_A, MOTOR_FORWARD, 500);  // 50% speed

    while (1) { /* main loop */ }
}
```

### 3️⃣ Cascade PID Example

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

### 4️⃣ Python Simulation

```bash
cd 15_simulation
pip install numpy matplotlib
python pid_simulation.py
```

---

## 📂 Directory Structure

```
nuedc-asset-library/
├── 00_docs/                      📖 Guides & API docs
├── 01_stm32/                     🔧 STM32F103 drivers & algorithms
├── 02_mspm0g3507/                🔧 MSPM0G3507 drivers & examples
├── 03_hardware_modules/          🔩 Hardware schematics & footprints
├── 04_tm4c123/                   🔧 TM4C123 drivers
├── 05_orangepi5/                 🖥️ OPi5 control & communication
├── 06_circuit_templates/         ⚡ Motor/sensor/power circuits
├── 07_report_templates/          📄 Design report templates
├── 08_vision_drivers/            📷 USB camera / GStreamer
├── 09_hw_acceleration/           ⚡ RGA / GPU / NPU acceleration
├── 10_opencv/                    👁️ OpenCV cross-compile
├── 11_npu_models/                🧠 YOLOv8 / segmentation / tracking
├── 12_vision_common/             👁️ Calibration / detection / tracking
├── 13_control_algorithms/        🎯 21 control algorithms in C
├── 14_solutions/                 💡 Engineering problem solutions
├── 15_simulation/                📊 Python simulations
├── 16_templates/                 📝 C / Python project templates
├── 17_competition_solutions/     🏆 37 competition solutions
├── 18_robotics/                  🤖 RoboMaster / WRC robotics
├── 19_opensource_analysis/       📚 GitHub project analysis
├── tests/                        🧪 3000+ test cases
└── tools/                        🔧 59 automation tools
```

---

## 🧮 Control Algorithms

<table>
<tr>
<td>

| Algorithm | File | Use Case |
|-----------|------|----------|
| PID (position) | `pid_full.c` | General |
| PID (incremental) | `incremental_pid.c` | Anti-shock |
| Cascade PID | `cascade_pid.c` | Dual-loop |
| Adaptive PID | `adaptive_pid.c` | Varying params |
| Fuzzy PID | `fuzzy_pid.c` | Nonlinear |
| ADRC | `adrc.c` | Strong disturbance |
| LADRC | `ladrc.c` | Simplified ADRC |
| MPC | `mpc_simple.c` | Constrained |
| LQR | `lqr.c` | Optimal control |
| SMC | `sliding_mode.c` | Robust control |
| Super-Twisting | `super_twisting_smc.c` | No chattering |

</td>
<td>

| Algorithm | File | Use Case |
|-----------|------|----------|
| Smith Predictor | `smith_predictor.c` | Large delay |
| Kalman Filter | `kalman.c` | State estimation |
| Extended Kalman | `ekf.c` | Nonlinear est. |
| Luenberger | `luenberger_observer.c` | State observer |
| Disturbance Obs. | `disturbance_observer.c` | Disturbance comp. |
| Feedforward | `feedforward.c` | Steady-state error |
| Repetitive Ctrl | `repetitive_control.c` | Periodic distur. |
| Trajectory Gen. | `trajectory_generator.c` | Motion planning |
| Path Tracker | `path_tracker.c` | Pure Pursuit |
| Motion Control | `motion_control.c` | Trapezoid/S-curve |
| State Feedback | `state_feedback.c` | Pole placement |

</td>
</tr>
</table>

---

## 📝 Competition Solutions (2009–2025)

> Each solution includes: **complete STM32 project** + **system design** + **schematics** + **design report** + **test data**

| Year | Problem | Core Algorithm | Report |
|:----:|---------|:--------------:|:------:|
| 2025 | E · Auto-aiming Device | Vision tracking + gimbal PID | ✅ |
| 2024 | H · Auto-driving Car | Line following + encoder PID | ✅ |
| 2024 | F · Magnetic Levitation | PID + electromagnetic control | ✅ |
| 2023 | E · Moving Target Tracking | Vision + ADRC | ✅ |
| 2022 | H · Car Following | Ultrasonic + cascade PID | ✅ |
| 2021 | F · Medicine Delivery Robot | Line following + obstacle avoidance | ✅ |
| 2020 | C · Hill-climbing Car | Encoder + PID | ✅ |
| 2019 | H · Electromagnetic Cannon | Ballistics + PID | ✅ |
| 2017 | B · Ball Rolling Control | MPU6050 + LQR | ✅ |
| 2015 | B · Wind Pendulum | IMU + PID + ADRC | ✅ |
| 2013 | C · Rotary Inverted Pendulum | LQR + Kalman | ✅ |
| 2011 | F · Sail Board Control | Angle + PID | ✅ |
| 2009 | E · Street Light Control | Light sensor + timer | ✅ |
| ... | More in `17_competition_solutions/` | ... | ... |

---

## 📊 Hardware Support

### MCU Platforms

| Platform | Chip | Core | Freq | Drivers |
|:--------:|------|:----:|:----:|:-------:|
| 🔵 STM32 | STM32F103C8T6 | Cortex-M3 | 72MHz | 18 |
| 🟢 MSPM0 | TI MSPM0G3507 | Cortex-M0+ | 80MHz | 79 |
| 🟠 TM4C | TI TM4C123GH6PZ | Cortex-M4F | 80MHz | 12 |
| 🟣 OPi5 | RK3588S | 4×A76 + 4×A55 | 2.4GHz | Vision |

### Sensors & Actuators

| Category | Device | Interface | Platforms |
|:--------:|--------|:---------:|:---------:|
| 🎯 IMU | JY901S | UART | All |
| ⬛ Line Sensor | 8-ch grayscale | ADC | MSPM0 |
| 📡 Ultrasonic | HC-SR04 | GPIO | All |
| 🎨 Color | TCS34725 | I2C | STM32/MSPM0 |
| 🌡️ Temp/Humidity | SHT20/SHT30 | I2C | STM32/MSPM0 |
| 💨 Air Quality | SGP30 | I2C | MSPM0 |
| ⚙️ Motor | N20 encoder motor | PWM+Encoder | All |
| 🔌 Driver | TB6612 / L298N | PWM+GPIO | All |
| 🦾 Servo | SG90 | PWM | STM32/MSPM0 |
| 🎛️ Servo Driver | PCA9685 | I2C | All |
| 📺 Display | SSD1306 OLED | I2C | STM32/MSPM0 |
| 📶 Bluetooth | HC-05 | UART | STM32/MSPM0 |

---

## 📖 Documentation

| Doc | Description |
|-----|-------------|
| 📖 [Quick Start Guide](00_docs/电赛资产库快速使用指南.md) | Getting started |
| 📘 [STM32 API](00_docs/STM32代码库API.md) | Driver API reference |
| 📗 [Control Algorithm API](00_docs/控制算法库API.md) | Algorithm API reference |
| 📙 [Vision API](00_docs/视觉代码库API.md) | Vision module reference |
| 📕 [Hardware API](00_docs/硬件模块库API.md) | Hardware driver reference |
| 📋 [Platform Comparison](00_docs/三单片机平台差异对照表.md) | Cross-platform reference |
| 🤝 [Contributing](CONTRIBUTING.md) | How to contribute |

---

## 🤝 Contributing

See [📄 Contributing Guide](CONTRIBUTING.md).

**Ways to contribute:**
- 🏆 Add more competition solutions
- 🔌 Add new sensor/actuator drivers
- ⚡ Optimize algorithm performance
- 🧪 Improve test coverage
- 📖 Enhance documentation

---

## 📄 License

[MIT License](LICENSE) — Free to use, modify, and distribute.

---

## 🙏 Acknowledgments

<div align="center">

![TI](https://img.shields.io/badge/Texas%20Instruments-E31837?style=flat-square&logo=texasinstruments&logoColor=white)
![ST](https://img.shields.io/badge/STMicroelectronics-03234B?style=flat-square&logo=stmicroelectronics&logoColor=white)
![ARM](https://img.shields.io/badge/ARM%20Cortex-0091BD?style=flat-square&logo=arm&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=flat-square&logo=opencv&logoColor=white)

National Electronic Design Competition Committee · TI MSPM0/TM4C Community · ST STM32 HAL · Open-source Contributors

</div>

---

<div align="center">

**⚡ Start with `00_docs/电赛资产库快速使用指南.md` and choose the driver directory matching your hardware.**

<br/>

![visitors](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FSlumberMin%2Fnuedc-asset-library.json&style=flat-square&color=blue&label=visitors&logo=github)

</div>
