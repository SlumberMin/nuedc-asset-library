# MSPM0G3507 驱动命名规范

## 1. 命名规则总览

| 后缀 | 含义 | 引脚配置方式 | 适用场景 |
|------|------|-------------|---------|
| `xxx.c` / `xxx.h` | SysConfig版本 | TI SysConfig工具 `.syscfg` 文件中配置 | 快速原型、单一项目 |
| `xxx_mspm0.c` / `xxx_mspm0.h` | 自定义版本 | 运行时传入配置结构体 | 多项目复用、引脚灵活 |

## 2. SysConfig版本 vs 自定义版本

### 2.1 SysConfig版本 (`xxx.c`)

**特点：**
- 引脚在 SysConfig GUI 中可视化配置，生成 `ti_msp_dl_config.h`
- 代码中直接使用 SysConfig 宏（如 `I2C_0_INST`、`ENCODER_PORT`、`TB6612_AIN1_PIN`）
- 初始化函数无参数（引脚已在 SysConfig 中绑定）
- 代码简洁，但引脚写死在 `.syscfg` 文件中

**示例：**
```c
// oled_ssd1306.c — 直接使用 SysConfig 宏
#define OLED_I2C    I2C_0_INST
// 引脚: PB2=SCL, PB3=SDA (在 .syscfg 中配置)
```

**典型文件：**
- `oled_ssd1306.c` — OLED显示
- `tcs34725.c` — 颜色传感器
- `pca9685.c` — 16路PWM舵机板
- `at24c02.c` — EEPROM
- `encoder.c` — 编码器
- `tb6612.c` — 电机驱动
- `l298n.c` — 电机驱动
- `servo.c` — 舵机
- `bluetooth.c` — 蓝牙
- `ultrasonic.c` — 超声波
- `grayscale.c` — 灰度传感器
- `jy901s.c` — IMU

### 2.2 自定义版本 (`xxx_mspm0.c`)

**特点：**
- 引脚在代码中通过 Config 结构体传入，运行时绑定
- 不依赖 SysConfig 宏，可移植性强
- Init 函数接收配置参数
- 适合引脚复用、多实例、跨项目复用

**示例：**
```c
// oled_ssd1306_mspm0.c — 运行时传入 I2C 实例
void OLED_Init(I2C_Regs *i2c);  // 调用时传入 I2C_0 或 I2C_1

// ultrasonic_mspm0.c — 运行时传入引脚配置
typedef struct {
    GPIO_Regs *trig_port;
    uint32_t   trig_pin;
    GPIO_Regs *echo_port;
    uint32_t   echo_pin;
} UltrasonicConfig;
void Ultrasonic_Init(const UltrasonicConfig *cfg);
```

**典型文件：**
- `oled_ssd1306_mspm0.c` — OLED（传入 I2C_Regs*）
- `jy901s_mspm0.c` — IMU（传入 JY901S_Config）
- `ultrasonic_mspm0.c` — 超声波（传入 UltrasonicConfig）
- `grayscale_mspm0.c` — 灰度（传入 GrayscaleConfig）
- `sensor_ir_mspm0.c` — 红外循迹（传入 IRConfig）

### 2.3 纯算法模块（无后缀，无硬件依赖）

这些模块不涉及硬件引脚，无版本区分：
- `advanced_pid.c` — PID控制器
- `moving_average.c` — 滑动平均滤波
- `kalman_filter.c` — 卡尔曼滤波
- `state_machine.c` — 状态机
- `ring_buffer.c` — 环形缓冲区
- `watchdog.c` — 看门狗
- `event_system.c` — 事件系统
- `task_scheduler.c` — 任务调度器
- `motor_protect.c` — 电机保护
- `speed_estimator.c` — 速度估计
- `foc_simple.c` — 简易FOC

## 3. 推荐使用版本

| 场景 | 推荐版本 | 理由 |
|------|---------|------|
| **电赛快速开发** | SysConfig版本 `xxx.c` | 配置简单，SysConfig可视化，代码量少 |
| **多项目复用** | 自定义版本 `xxx_mspm0.c` | 引脚灵活，不依赖SysConfig，可跨工程复用 |
| **引脚冲突调优** | 自定义版本 `xxx_mspm0.c` | 运行时可切换引脚，方便调试 |
| **新手入门** | SysConfig版本 `xxx.c` | 学习曲线低，TI官方推荐 |

**默认推荐：SysConfig版本**（适合电赛4天限时开发）

## 4. 引脚分配对照表

### 4.1 SysConfig版本默认引脚分配

| 外设 | 引脚 | SysConfig宏名 | 备注 |
|------|------|--------------|------|
| **I2C0 SCL** | PB2 | `I2C_0_SCL` | OLED/TCS34725/PCA9685/AT24C02 共用 |
| **I2C0 SDA** | PB3 | `I2C_0_SDA` | OLED/TCS34725/PCA9685/AT24C02 共用 |
| **UART1 TX** | PA17 | `UART_1_TX` | JY901S/Bluetooth 共用(互斥) |
| **UART1 RX** | PA18 | `UART_1_RX` | JY901S/Bluetooth 共用(互斥) |
| **TB6612 AIN1** | PA0 | `TB6612_AIN1_PIN` | 电机A方向 |
| **TB6612 AIN2** | PA1 | `TB6612_AIN2_PIN` | 电机A方向 |
| **TB6612 BIN1** | PA2 | `TB6612_BIN1_PIN` | 电机B方向 |
| **TB6612 BIN2** | PA3 | `TB6612_BIN2_PIN` | 电机B方向 |
| **TB6612 PWMA** | PA12 | `GPIO_PWM_0_C0_IDX` | TIMA0 CH0 |
| **TB6612 PWMB** | PA13 | `GPIO_PWM_0_C3_IDX` | TIMA0 CH3 |
| **L298N IN1** | PA4 | `L298N_IN1_PIN` | 电机A方向 |
| **L298N IN2** | PA5 | `L298N_IN2_PIN` | 电机A方向 |
| **L298N IN3** | PA6 | `L298N_IN3_PIN` | 电机B方向 |
| **L298N IN4** | PA7 | `L298N_IN4_PIN` | 电机B方向 |
| **L298N ENA** | PA8 | `GPIO_PWM_0_C0_IDX` | TIMA0 CH0 PWM |
| **L298N ENB** | PA9 | `GPIO_PWM_0_C1_IDX` | TIMA0 CH1 PWM |
| **Servo Signal** | PA8 | `GPIO_SERVO_C0_IDX` | TIMA0 CH0 PWM |
| **Encoder E1A** | PB0 | `ENCODER_E1A_PIN` | 左轮A相(中断) |
| **Encoder E1B** | PB1 | `ENCODER_E1B_PIN` | 左轮B相(输入) |
| **Encoder E2A** | PB4 | `ENCODER_E2A_PIN` | 右轮A相(中断) |
| **Encoder E2B** | PB5 | `ENCODER_E2B_PIN` | 右轮B相(输入) |
| **Ultrasonic Trig** | PB6 | `ENCODER_TRIG_PIN` | 超声波触发 |
| **Ultrasonic Echo** | PB7 | `ENCODER_ECHO_PIN` | 超声波回波(中断) |
| **Grayscale G0~G7** | PB0~PB7 | `GRAY_G0~G7_PIN` | 8路数字输入 |
| **Bluetooth EN** | PA16 | `BT_EN` | 蓝牙使能(高=AT) |

### 4.2 I2C从机地址表（I2C0总线，可共存）

| 设备 | I2C地址 | 地址引脚 |
|------|---------|---------|
| SSD1306 OLED | 0x3C | 固定 |
| TCS34725 颜色传感器 | 0x29 | 固定 |
| PCA9685 舵机板 | 0x40 | A0~A5可调 |
| AT24C02 EEPROM | 0x50 | A2=A1=A0=0 |

> **I2C设备可同时挂载在同一总线上**，只要地址不冲突即可。
> 推荐使用 `i2c_bus.h` 总线管理层统一调度。

## 5. 驱动共存/互斥矩阵

### 5.1 互斥关系（不能同时使用）

| 驱动A | 驱动B | 冲突原因 |
|-------|-------|---------|
| **TB6612** | **L298N** | 同类电机驱动，且PA引脚冲突 |
| **L298N ENA** | **Servo** | 共用PA8 (TIMA0 CH0) |
| **JY901S** | **Bluetooth** | 共用UART1 (PA17/PA18) |
| **Grayscale** | **Encoder** | 共用PB0, PB1, PB4, PB5 |
| **Grayscale** | **Ultrasonic** | 共用PB6, PB7 |
| **Encoder** | **Ultrasonic** | 共用ENCODER_PORT中断组 |

### 5.2 可共存关系（可同时使用）

| 驱动组合 | 共存条件 |
|---------|---------|
| OLED + TCS34725 + PCA9685 + AT24C02 | I2C0总线，地址不同 |
| TB6612 + Servo | 电机用PA12/PA13 PWM，舵机用PA8 PWM |
| TB6612 + Encoder | 电机PA0~PA3 + PA12/PA13，编码器PB0~PB5 |
| TB6612 + Bluetooth | 电机PA0~PA3，蓝牙PA16~PA18 |
| 任意I2C设备 + 任意非I2C设备 | 总线独立 |

### 5.3 典型比赛方案引脚分配

**方案A：智能小车（循迹+避障+蓝牙遥控）**
```
电机驱动: TB6612 (PA0~PA3, PA12, PA13)
编码器:   PB0, PB1, PB4, PB5
灰度:     不可用（与编码器冲突）→ 改用红外循迹 sensor_ir
超声波:   PB6, PB7
蓝牙:     PA16~PA18
OLED:     PB2, PB3 (I2C0)
```

**方案B：机械臂（多舵机+颜色分拣）**
```
舵机板:   PCA9685 (I2C0, 0x40) — 16路舵机
颜色传感器: TCS34725 (I2C0, 0x29)
OLED:     PB2, PB3 (I2C0, 0x3C)
EEPROM:   AT24C02 (I2C0, 0x50)
蓝牙:     PA16~PA18
```

**方案C：平衡小车（IMU+电机+编码器）**
```
电机驱动: TB6612 (PA0~PA3, PA12, PA13)
编码器:   PB0, PB1, PB4, PB5
IMU:      JY901S (UART1: PA17, PA18)
OLED:     PB2, PB3 (I2C0)
超声波:   PB6, PB7
```
