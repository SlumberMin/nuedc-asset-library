# 📋 nuedc-asset-library - 更新日志

---

## 2026-06-11

### 新增模块
- **MSPM0G3507 驱动库大幅扩充**：新增 11 个外设驱动，覆盖电机控制、传感器、通信、显示全链路
  - TB6612 电机驱动 (`tb6612.c/.h`)
  - L298N 电机驱动 (`l298n.c/.h`)
  - 舵机驱动 (`servo_mspm0.h`)
  - GPIO 编码器 (`encoder_gpio_mspm0.c/.h`)
  - 定时器编码器 (`encoder_mspm0.c/.c`)
  - JY901S 九轴 IMU (`jy901s_mspm0.c/.h`)
  - 感为 8 路灰度传感器 (`grayscale_mspm0.c/.h`)
  - SR04 超声波传感器 (`ultrasonic_mspm0.c/.h`)
  - HC-05 蓝牙模块 (`bluetooth_hc05_mspm0.c/.h`)
  - SSD1306 OLED 显示 (`oled_ssd1306_mspm0.c/.h`)
  - TCS34725 颜色传感器 (`tcs34725.c/.h`)
  - AT24C02 EEPROM (`at24c02.c/.h`)
  - PCA9685 16 路舵机驱动 (`pca9685.c/.h`)

### 文档更新
- 创建 `02_mspm0g3507/README.md` 完整文档（含全部 API 说明、系统集成示例、FAQ）
- 创建 `02_mspm0g3507/API速查表.md`（全部驱动 API 速查）
- 更新主 `README.md`，添加新增模块索引
- 更新 `已买器件适配_无限迭代提示词_V2.md`，添加最佳实践
- 创建本 `CHANGELOG.md`

### 目录整理
- 修复模块编号冲突
- 归档旧版本到 `archive/`

---

## 2026-06-10

- 完成 42 轮并行迭代，产出 1,447 个文件
- 新增控制算法库（ADRC/LQR/MPC/滑模/模糊PID/神经网络PID 等 15+ 种）
- 新增视觉通用代码库（147 个文件）
- 新增 NPU 模型库（YOLOv5/v8、PicoDet、MobileNet 等）
- 新增系统架构模块（进程通信、实时性优化、融合接口）
- 新增仿真与验证模块（143 个文件）

---

## 2026-06-09

- 开始构建nuedc-asset-library
- 建立目录结构（18 个主模块）
- 初始导入 STM32/TM4C/OrangePi5 代码库
- 导入历年赛题解决方案（2009-2025，33 个赛题）

---
