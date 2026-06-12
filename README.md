# 🏆 NUEDC Control & Vision Asset Library

> 全国大学生电子设计竞赛 · 控制+视觉 综合资产库
>
> A complete competition resource library covering STM32 / MSPM0G3507 / TM4C123 + Orange Pi 5 vision

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-STM32%20%7C%20MSPM0%20%7C%20TM4C-blue.svg)]()
[![Vision](https://img.shields.io/badge/Vision-Orange%20Pi%205%20(RK3588S)-green.svg)]()

---

## 📦 Features

- **3 MCU platform drivers**: STM32F103 / MSPM0G3507 / TM4C123 — same sensor, 3 drivers
- **21 control algorithms**: PID family, ADRC, MPC, LQR, SMC, Smith predictor, Kalman filter, etc.
- **37 past competition solutions**: 2009–2025 control-type problems with code + design reports
- **Orange Pi 5 vision system**: NPU/GPU/RGA hardware acceleration, camera + OpenCV + inference
- **3000+ test cases**: drivers, algorithms, integration tests
- **59 automation tools**: build checks, code generation, debugging helpers

---

## 🗂️ Directory Structure

```
nuedc-asset-library/
├── 00_docs/                      # Guides, API docs, reports
├── 01_stm32/                     # STM32F103 drivers & algorithms
├── 02_mspm0g3507/                # TI MSPM0G3507 drivers & examples
├── 03_hardware_modules/          # Hardware module schematics & footprints
├── 04_tm4c123/                   # TI TM4C123 drivers
├── 05_orangepi5/                 # Orange Pi 5 control & communication
├── 06_circuit_templates/         # Motor/sensor/power circuit templates
├── 07_report_templates/          # Competition design report templates
├── 08_vision_drivers/            # USB camera / GStreamer / ISP drivers
├── 09_hw_acceleration/           # RGA / GPU / NPU hardware acceleration
├── 10_opencv/                    # OpenCV cross-compile + RKNN backend
├── 11_npu_models/                # YOLOv8 / segmentation / tracking models
├── 12_vision_common/             # Vision algorithms: calibration, detection, tracking
├── 13_control_algorithms/        # 21 control algorithms in C (portable)
├── 14_solutions/                 # Typical engineering problem solutions
├── 15_simulation/                # Python simulations: control, motors, sensors
├── 16_templates/                 # C / Python project templates
├── 17_competition_solutions/     # 37 past competition solutions (code + reports)
├── 18_robotics/                  # RoboMaster / WRC robotics competition
├── 19_opensource_analysis/       # GitHub high-star project analysis
├── tests/                        # 3000+ test cases
└── tools/                        # 59 automation tools
```

---

## 🚀 Quick Start

### Motor Driver Example (MSPM0G3507 + TB6612)

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

### Cascade PID Example

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

### Python Simulation

```bash
cd 15_simulation
pip install numpy matplotlib
python pid_simulation.py
```

---

## 📊 Supported Hardware

### MCU Platforms

| Platform | Chip | Core | Freq | Drivers |
|----------|------|------|------|---------|
| STM32 | STM32F103C8T6 | Cortex-M3 | 72MHz | 18 |
| MSPM0G3507 | TI MSPM0G3507 | Cortex-M0+ | 80MHz | 79 |
| TM4C123 | TI TM4C123GH6PZT7 | Cortex-M4F | 80MHz | 12 |
| Orange Pi 5 | RK3588S | 4×A76+4×A55 | 2.4GHz | Vision |

### Sensors & Actuators

| Category | Device | Platforms |
|----------|--------|-----------|
| IMU | JY901S (UART) | STM32/MSPM0/TM4C |
| Line sensor | 8-ch grayscale | MSPM0 |
| Ultrasonic | HC-SR04 | STM32/MSPM0/TM4C |
| Color | TCS34725 (I2C) | STM32/MSPM0 |
| Temp/Humidity | SHT20/SHT30 | STM32/MSPM0 |
| Air quality | SGP30 | MSPM0 |
| Motor | N20 encoder motor | STM32/MSPM0/TM4C |
| Driver | TB6612FNG / L298N | STM32/MSPM0/TM4C |
| Servo | SG90 (PWM) | STM32/MSPM0 |
| Servo driver | PCA9685 (I2C) | STM32/MSPM0/TM4C |
| Display | SSD1306 OLED (I2C) | STM32/MSPM0 |
| Bluetooth | HC-05 | STM32/MSPM0 |

---

## 🧮 Control Algorithms

| Algorithm | File | Use Case |
|-----------|------|----------|
| PID (position/incremental) | `pid_full.c` | General |
| Cascade PID | `cascade_pid.c` | Position-velocity loop |
| Adaptive PID | `adaptive_pid.c` | Varying parameters |
| Fuzzy PID | `fuzzy_pid.c` | Nonlinear systems |
| ADRC | `adrc.c` | Strong disturbance |
| LADRC | `ladrc.c` | Simplified ADRC |
| MPC | `mpc_simple.c` | Constrained optimization |
| LQR | `lqr.c` | Optimal control |
| Sliding Mode (SMC) | `sliding_mode.c` | Robust control |
| Super-Twisting SMC | `super_twisting_smc.c` | Chattering elimination |
| Smith Predictor | `smith_predictor.c` | Large delay systems |
| Kalman Filter | `kalman.c` | State estimation |
| Extended Kalman | `ekf.c` | Nonlinear estimation |
| Luenberger Observer | `luenberger_observer.c` | State observation |
| Disturbance Observer | `disturbance_observer.c` | Disturbance compensation |
| Feedforward | `feedforward.c` | Steady-state error elimination |
| Repetitive Control | `repetitive_control.c` | Periodic disturbance |
| Trajectory Generator | `trajectory_generator.c` | Motion planning |
| Path Tracker | `path_tracker.c` | Pure Pursuit |
| Motion Control | `motion_control.c` | Trapezoidal/S-curve |

---

## 📝 Competition Solutions (2009–2025)

Each solution includes: complete STM32 project, system design doc, schematics, test data.

| Year | Problem | Core Algorithm |
|------|---------|----------------|
| 2025 | E · Auto-aiming Device | Vision tracking + gimbal PID |
| 2024 | H · Auto-driving Car | Line following + encoder PID |
| 2024 | F · Magnetic Levitation | PID + electromagnetic control |
| 2023 | E · Moving Target Tracking | Vision + ADRC |
| 2022 | H · Car Following System | Ultrasonic + cascade PID |
| 2021 | F · Medicine Delivery Robot | Line following + obstacle avoidance |
| 2017 | B · Ball Rolling Control | MPU6050 + LQR |
| 2015 | B · Wind Pendulum | IMU + PID + ADRC |
| 2013 | C · Rotary Inverted Pendulum | LQR + Kalman |
| ... | See `17_competition_solutions/` | ... |

---

## 🔧 Build & Toolchain

| Platform | Toolchain | IDE |
|----------|-----------|-----|
| STM32 | arm-none-eabi-gcc | Keil / STM32CubeIDE |
| MSPM0G3507 | ti-cgt-armllvm 4.0.2 | CCS/Theia + SysConfig |
| TM4C123 | arm-none-eabi-gcc | CCS / Keil |
| Python | Python 3.8+ | Any |

---

## 📖 Documentation

| Doc | Path | Description |
|-----|------|-------------|
| Quick Start | `00_docs/电赛资产库快速使用指南.md` | Getting started |
| STM32 API | `00_docs/STM32代码库API.md` | Driver API reference |
| Control Algorithm API | `00_docs/控制算法库API.md` | Algorithm API reference |
| Vision API | `00_docs/视觉代码库API.md` | Vision module reference |
| Hardware API | `00_docs/硬件模块库API.md` | Hardware driver reference |
| Platform Comparison | `00_docs/三单片机平台差异对照表.md` | Cross-platform reference |
| Contributing | `CONTRIBUTING.md` | How to contribute |

---

## 🤝 Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## 📄 License

[MIT License](LICENSE)

---

## 🙏 Acknowledgments

- National Electronic Design Competition Committee (全国大学生电子设计竞赛组委会)
- Texas Instruments MSPM0/TM4C open-source community
- STMicroelectronics STM32 HAL library
- Open-source control algorithm contributors

---

> **⚡ Tip**: Start with `00_docs/电赛资产库快速使用指南.md` and choose the driver directory matching your hardware platform.
