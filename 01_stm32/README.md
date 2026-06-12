# STM32 电赛通用代码库 使用手册

## 📋 目录

1. [概述](#概述)
2. [目录结构](#目录结构)
3. [快速开始](#快速开始)
4. [模块说明](#模块说明)
   - [platform/hal_stm32.h — HAL统一抽象层](#1-platformhal_stm32h)
   - [drivers/motor — 电机驱动](#2-driversmotor)
   - [drivers/encoder — 编码器驱动](#3-driversencoder)
   - [drivers/servo — 舵机驱动](#4-driversservo)
   - [drivers/sensor_ir — 红外循迹](#5-drivessensor_ir)
   - [drivers/oled — OLED显示](#6-driversoled)
   - [drivers/key — 按键驱动](#7-driverskey)
   - [algorithm/pid — PID控制器](#8-algorithmpid)
   - [utils/ring_buffer — 环形缓冲区](#9-utilsring_buffer)
   - [utils/math_utils — 数学工具](#10-utilsmath_utils)
5. [移植指南](#移植指南)
6. [编码规范](#编码规范)

---

## 概述

本代码库为全国大学生电子设计竞赛（电赛）设计的 **STM32通用底层驱动库**，覆盖常见外设驱动和常用算法，遵循以下设计原则：

- **模块化**：每个模块独立，可单独使用
- **统一接口**：`Init/Get/Set/Process/DeInit` 统一命名
- **错误码返回**：所有函数返回 `ErrorCode_t`
- **详细注释**：每个函数均有中文注释说明功能、参数、返回值
- **即插即用**：配合STM32CubeMX使用，仅需少量配置

---

## 目录结构

```
01_stm32/
├── platform/
│   └── hal_stm32.h           # HAL统一抽象层（宏定义）
├── drivers/
│   ├── motor.h / motor.c     # 电机驱动（L298N/TB6612FNG）
│   ├── encoder.h / encoder.c # 编码器驱动（四倍频、速度、距离）
│   ├── servo.h / servo.c     # 舵机驱动（SG90/MG996R）
│   ├── sensor_ir.h / sensor_ir.c  # 红外循迹（TCRT5000阵列）
│   ├── oled.h / oled.c       # OLED显示（SSD1306 I2C）
│   └── key.h / key.c         # 按键驱动（消抖+边沿检测）
├── algorithm/
│   ├── pid.h / pid.c         # PID控制器（位置式/增量式）
├── utils/
│   ├── ring_buffer.h / ring_buffer.c  # 环形缓冲区（无锁）
│   ├── math_utils.h / math_utils.c    # 数学工具（滤波/限幅/映射）
└── README.md                 # 本文件
```

---

## 快速开始

### 1. 工程配置

在STM32CubeMX中配置好所需外设（GPIO、TIM、I2C、ADC、UART等），生成代码后将本库文件加入工程。

### 2. 添加头文件路径

在Keil/IAR中将 `platform/`、`drivers/`、`algorithm/`、`utils/` 目录加入头文件搜索路径。

### 3. 示例：电机+编码器+PID

```c
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "algorithm/pid.h"

Motor_t   motor_left;
Encoder_t encoder_left;
PID_t     pid_speed;

void System_Init(void)
{
    // 初始化电机（L298N驱动）
    Motor_Init(&motor_left, MOTOR_DRV_L298N,
               GPIOA, GPIO_PIN_0,  // IN1
               GPIOA, GPIO_PIN_1,  // IN2
               &htim2, TIM_CHANNEL_1);

    // 初始化编码器
    Encoder_Init(&encoder_left, &htim3,
                 13,     // 13线编码器
                 6.5f,   // 轮径6.5cm
                 30.0f); // 减速比30:1

    // 初始化PID（位置式，10ms周期）
    PID_Init(&pid_speed, PID_MODE_POSITION,
             8.0f, 0.3f, 0.0f, 0.01f);
    PID_SetOutputLimit(&pid_speed, -1000, 1000);
    PID_SetTarget(&pid_speed, 50.0f);  // 目标速度50cm/s
}

// 在10ms定时中断中调用
void Control_Loop(void)
{
    Encoder_Update(&encoder_left);
    float speed = Encoder_GetSpeed(&encoder_left);
    float output = PID_Calculate(&pid_speed, speed);
    Motor_SetSpeed(&motor_left, (int16_t)output);
}
```

### 4. 示例：红外循迹

```c
#include "drivers/sensor_ir.h"
#include "algorithm/pid.h"

SensorIR_t ir_sensor;
PID_t      pid_track;

void Track_Init(void)
{
    uint16_t adc_ch[] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2,
                         ADC_CHANNEL_3, ADC_CHANNEL_4};
    SensorIR_InitADC(&ir_sensor, &hadc1, adc_ch, 5, 2000);

    PID_Init(&pid_track, PID_MODE_POSITION, 30.0f, 0.0f, 10.0f, 0.005f);
    PID_SetOutputLimit(&pid_track, -500, 500);
}

void Track_Loop(void)
{
    SensorIR_Update(&ir_sensor);
    float pos = SensorIR_GetPosition(&ir_sensor);
    PID_SetTarget(&pid_track, 0.0f);
    float steer = PID_Calculate(&pid_track, pos);
    // steer用于差速转向
}
```

### 5. 示例：OLED显示

```c
#include "drivers/oled.h"

OLED_t oled;

void Display_Init(void)
{
    OLED_Init(&oled, &hi2c1, OLED_I2C_ADDR);
    OLED_Clear(&oled);
    OLED_ShowString(&oled, "Hello E-Design!", 0, 0);
    OLED_ShowString(&oled, "Speed:", 0, 1);
}

void Display_Update(float speed)
{
    OLED_ShowFloat(&oled, speed, 1, 7, 1);
}
```

---

## 模块说明

### 1. platform/hal_stm32.h

**功能**：HAL统一抽象层，提供宏定义简化HAL库操作。

| 宏 | 功能 | 示例 |
|---|---|---|
| `GPIO_SET/CLR/TOGGLE/READ` | GPIO操作 | `GPIO_SET(GPIOA, GPIO_PIN_5)` |
| `PWM_START/STOP/SET` | PWM操作 | `PWM_SET(&htim2, TIM_CHANNEL_1, 500)` |
| `ADC_READ` | 读取ADC值 | `uint16_t val = ADC_READ(&hadc1)` |
| `UART_SEND_STR` | 串口发送字符串 | `UART_SEND_STR(&huart1, "OK", 100)` |
| `CLAMP/MIN/MAX/ABS` | 数学工具 | `CLAMP(val, -100, 100)` |
| `MAP_RANGE` | 范围映射 | `MAP_RANGE(val, 0, 4095, 0, 100)` |
| `ErrorCode_t` | 统一错误码 | `return HAL_OK_CODE` |

---

### 2. drivers/motor

**功能**：L298N/TB6612FNG电机驱动。

| 函数 | 说明 |
|---|---|
| `Motor_Init()` | 初始化电机，配置驱动芯片类型、GPIO、PWM |
| `Motor_SetSpeed(speed)` | 设置速度 -1000~+1000 |
| `Motor_GetSpeed()` | 获取当前速度 |
| `Motor_Brake()` | 制动（快速停止） |
| `Motor_Coast()` | 滑行（自由停止） |
| `Motor_DeInit()` | 反初始化 |

**注意**：速度绝对值<10时自动归零（内置死区）。

---

### 3. drivers/encoder

**功能**：编码器驱动，四倍频测速测距。

| 函数 | 说明 |
|---|---|
| `Encoder_Init()` | 初始化，需传入ppr、轮径、减速比 |
| `Encoder_Update()` | 更新数据（建议10~20ms调用一次） |
| `Encoder_GetSpeed()` | 获取速度(cm/s) |
| `Encoder_GetDistance()` | 获取累计距离(cm) |
| `Encoder_Reset()` | 重置距离和计数 |

**物理参数**：
- `cm_per_pulse = π × diameter / (ppr × 4 × gear_ratio)`
- 定时器需配置为 **Encoder Mode**（TI1+TI2四倍频）

---

### 4. drivers/servo

**功能**：SG90/MG996R舵机驱动。

| 函数 | 说明 |
|---|---|
| `Servo_Init()` | 初始化（SG90/MG996R预设参数） |
| `Servo_InitCustom()` | 自定义角度范围和脉宽 |
| `Servo_SetAngle(angle)` | 设置角度(°) |
| `Servo_GetAngle()` | 获取当前角度 |
| `Servo_SetPulse(pulse_us)` | 直接设置脉宽(μs) |
| `Servo_Center()` | 归中(90°) |

**PWM参数**：50Hz (ARR=19999)，脉宽 500~2500μs 对应 0°~180°。

---

### 5. drivers/sensor_ir

**功能**：TCRT5000红外循迹传感器阵列。

| 函数 | 说明 |
|---|---|
| `SensorIR_InitADC()` | ADC模式初始化 |
| `SensorIR_InitGPIO()` | GPIO模式初始化 |
| `SensorIR_SetWeights()` | 自定义权重 |
| `SensorIR_Update()` | 更新数据（建议5~10ms） |
| `SensorIR_GetPosition()` | 获取位置（0=中心，负=偏左） |
| `SensorIR_IsOnLine()` | 是否在线上 |
| `SensorIR_IsCrossDetected()` | 是否十字路口 |

**位置计算**：加权平均法，权重默认 {-4,-2,0,2,4}（5路时）。

---

### 6. drivers/oled

**功能**：SSD1306 128×64 I2C OLED显示。

| 函数 | 说明 |
|---|---|
| `OLED_Init()` | 初始化OLED |
| `OLED_Clear()` | 清屏 |
| `OLED_ShowString(str, x, y)` | 显示字符串（字符坐标） |
| `OLED_ShowInt(num, x, y)` | 显示整数 |
| `OLED_ShowFloat(num, dec, x, y)` | 显示浮点数 |
| `OLED_DrawPoint(x, y)` | 画点（像素坐标） |
| `OLED_DrawLine(x1,y1,x2,y2)` | 画线（Bresenham算法） |
| `OLED_DrawRect(x,y,w,h)` | 画矩形 |
| `OLED_Refresh()` | 刷新显存到屏幕 |

**I2C配置**：地址0x3C，建议Fast Mode(400kHz)。

---

### 7. drivers/key

**功能**：按键驱动，20ms消抖 + 边沿检测。

| 函数 | 说明 |
|---|---|
| `Key_Init()` | 初始化单个按键 |
| `KeyMgr_Init/Add()` | 管理器：初始化/添加按键 |
| `KeyMgr_Scan()` | 扫描所有按键（建议5~10ms） |
| `Key_GetEvent()` | 获取事件（读后清除） |
| `Key_IsPressed()` | 是否按下 |
| `Key_IsLongPressed()` | 是否长按(>1s) |

**事件类型**：
- `KEY_EVENT_PRESS` — 按下（单次触发）
- `KEY_EVENT_RELEASE` — 松开（单次触发）
- `KEY_EVENT_LONG_PRESS` — 长按（持续>1s触发）

---

### 8. algorithm/pid

**功能**：PID控制器，支持位置式和增量式。

| 函数 | 说明 |
|---|---|
| `PID_Init(mode, kp, ki, kd, dt)` | 初始化 |
| `PID_SetTarget(target)` | 设置目标值 |
| `PID_Calculate(feedback)` | 计算输出（周期调用） |
| `PID_SetOutputLimit(min, max)` | 输出限幅 |
| `PID_SetIntegralLimit(max)` | 积分限幅 |
| `PID_SetDeadZone(zone)` | 设置死区 |
| `PID_SetDerivativeFilter(alpha)` | 微分滤波系数 |
| `PID_SetConditionalIntegral(enable, threshold)` | 条件积分抗饱和 |
| `PID_SetFeedforward(ff)` | 前馈值 |
| `PID_Reset()` | 重置状态 |

**高级特性**：
- **微分滤波**：一阶低通，alpha=0.1（默认），减少高频噪声
- **条件积分抗饱和**：输出饱和时不累积积分，防止超调
- **死区**：误差小时不输出，减少抖动
- **前馈**：已知扰动补偿

---

### 9. utils/ring_buffer

**功能**：无锁单生产者单消费者环形缓冲区。

| 函数 | 说明 |
|---|---|
| `RingBuffer_Init(capacity)` | 动态分配初始化 |
| `RingBuffer_InitStatic(buf, size)` | 静态缓冲区初始化（推荐） |
| `RingBuffer_Put/Get()` | 单字节读写 |
| `RingBuffer_Write/Read()` | 多字节读写 |
| `RingBuffer_Available()` | 已存数据量 |
| `RingBuffer_Free()` | 剩余空间 |
| `RingBuffer_Flush()` | 清空 |

**典型用途**：串口接收中断 → 环形缓冲区 → 主循环解析。

---

### 10. utils/math_utils

**功能**：常用数学工具。

| 函数/类型 | 说明 |
|---|---|
| `Math_ClampF/I()` | 限幅 |
| `Math_MapF/I()` | 范围映射 |
| `Math_DeadZoneF/I()` | 死区处理 |
| `Math_DeadZoneCompensateF()` | 死区补偿（电机用） |
| `MovingAvg_t` | 滑动平均滤波器 |
| `LowPassFilter_t` | 一阶低通滤波器 |

---

## 移植指南

### STM32F1 → STM32F4/F0/L0

1. 修改 `hal_stm32.h` 中的 `#include "stm32f1xx_hal.h"` 为对应芯片系列
2. ADC通道配置方式可能不同，修改 `sensor_ir.c` 中的 `SensorIR_ReadADC()`
3. 其他模块不依赖具体芯片，可直接使用

### CubeMX配置清单

| 模块 | CubeMX需配置 |
|---|---|
| motor | TIM PWM输出 + GPIO Output |
| encoder | TIM Encoder Mode (TI1+TI2) |
| servo | TIM PWM输出 (50Hz, ARR=19999) |
| sensor_ir(ADC) | ADC + Scan/Regular模式 |
| sensor_ir(GPIO) | GPIO Input |
| oled | I2C (Fast Mode 400kHz) |
| key | GPIO Input (Pull-up/Pull-down) |
| ring_buffer | 无需硬件配置 |

---

## 编码规范

| 规则 | 说明 |
|---|---|
| 命名 | `模块名_函数名()`，如 `Motor_SetSpeed()` |
| 接口 | 统一 `Init/Get/Set/Process/DeInit` |
| 返回值 | `ErrorCode_t`，0=成功，负值=错误 |
| 注释 | 每个函数均有功能、参数、返回值说明 |
| 头文件保护 | `#ifndef __模块名_H` |
| 静态函数 | 内部函数加 `static` 前缀 |
| 调试输出 | 使用 `DBG_PRINTF()` 宏 |

---

## 版本信息

- **版本**：1.0
- **日期**：2026-06
- **适用**：STM32F1xx / STM32F4xx 系列
- **许可**：电赛开源，自由使用
