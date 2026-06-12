# STM32 代码库 API 文档

> 版本: 1.0 | 更新日期: 2026-06-10  
> 路径: `01_stm32/`

---

## 目录

1. [概述](#概述)
2. [平台抽象层](#平台抽象层)
3. [PID 控制器](#pid-控制器)
4. [电机驱动](#电机驱动)
5. [舵机控制](#舵机控制)
6. [编码器读取](#编码器读取)
7. [红外传感器](#红外传感器)
8. [OLED 显示](#oled-显示)
9. [按键驱动](#按键驱动)
10. [数学工具](#数学工具)
11. [环形缓冲区](#环形缓冲区)

---

## 概述

STM32 通用代码库为电赛提供 HAL 层封装的硬件驱动与控制算法。支持 STM32F103/F407/F411 等主流型号。

### 文件结构

```
01_stm32/
├── platform/
│   └── hal_stm32.h          # 平台抽象层(宏定义、类型)
├── drivers/
│   ├── motor.h / motor.c    # 电机驱动
│   ├── servo.h / servo.c    # 舵机控制
│   ├── encoder.h / encoder.c # 编码器
│   ├── sensor_ir.h / .c     # 红外传感器
│   ├── oled.h / oled.c      # OLED显示
│   └── key.h / key.c        # 按键
├── algorithm/
│   └── pid.h / pid.c        # PID控制器
└── utils/
    ├── math_utils.h / .c    # 数学工具
    └── ring_buffer.h / .c   # 环形缓冲区
```

### 错误码定义

```c
typedef enum {
    HAL_OK_CODE     = 0,   // 成功
    HAL_ERR_PARAM   = 1,   // 参数错误
    HAL_ERR_NOT_INIT = 2,  // 未初始化
    HAL_ERR_TIMEOUT  = 3,  // 超时
    HAL_ERR_BUSY    = 4,   // 忙碌
} ErrorCode_t;
```

### 平台宏

| 宏 | 说明 |
|---|---|
| `GPIO_SET(port, pin)` | 设置 GPIO 引脚高电平 |
| `GPIO_CLR(port, pin)` | 设置 GPIO 引脚低电平 |
| `GPIO_READ(port, pin)` | 读取 GPIO 引脚状态 |
| `PWM_SET(htim, ch, val)` | 设置 PWM 比较值 |
| `PWM_START(htim, ch)` | 启动 PWM 输出 |
| `PWM_STOP(htim, ch)` | 停止 PWM 输出 |
| `CLAMP(val, min, max)` | 数值限幅 |
| `ABS(x)` | 绝对值 |
| `DBG_PRINTF(...)` | 调试打印 |

---

## PID 控制器

**头文件**: `algorithm/pid.h`  
**源文件**: `algorithm/pid.c`

### 类型定义

#### `PID_Mode_t`

| 枚举值 | 说明 |
|---|---|
| `PID_MODE_POSITION` | 位置式 PID: `output = Kp*e + Ki*∫e + Kd*de` |
| `PID_MODE_INCREMENTAL` | 增量式 PID: `Δu = Kp*Δe + Ki*e + Kd*Δ²e` |

#### `PID_t` 结构体

| 字段 | 类型 | 说明 |
|---|---|---|
| `mode` | `PID_Mode_t` | PID 模式 |
| `kp, ki, kd` | `float` | PID 参数 |
| `target` | `float` | 目标值 |
| `output` | `float` | 输出值 |
| `output_min, output_max` | `float` | 输出限幅范围 (默认 -1000 ~ +1000) |
| `integral` | `float` | 积分累积 |
| `integral_max` | `float` | 积分限幅 (默认 500) |
| `derivative_filter_alpha` | `float` | 微分滤波系数 (默认 0.1) |
| `conditional_integral` | `bool` | 条件积分抗饱和开关 |
| `dead_zone` | `float` | 死区大小 |
| `feedforward` | `float` | 前馈值 |
| `dt_s` | `float` | 控制周期 (秒) |
| `initialized` | `bool` | 初始化标志 |

### API 函数

#### `PID_Init`

```c
ErrorCode_t PID_Init(PID_t *pid, PID_Mode_t mode,
                     float kp, float ki, float kd, float dt_s);
```

- **功能**: 初始化 PID 控制器
- **参数**: `pid` - PID 结构体指针; `mode` - 工作模式; `kp/ki/kd` - 增益; `dt_s` - 控制周期(秒)
- **返回**: `HAL_OK_CODE` 成功; `HAL_ERR_PARAM` 参数错误
- **默认值**: 输出范围 [-1000, 1000]，积分限幅 500，微分滤波 alpha=0.1

#### `PID_SetParams`

```c
ErrorCode_t PID_SetParams(PID_t *pid, float kp, float ki, float kd);
```

- **功能**: 运行时修改 PID 参数

#### `PID_SetTarget`

```c
ErrorCode_t PID_SetTarget(PID_t *pid, float target);
```

- **功能**: 设置目标值

#### `PID_GetTarget`

```c
float PID_GetTarget(const PID_t *pid);
```

- **功能**: 获取当前目标值

#### `PID_SetOutputLimit`

```c
ErrorCode_t PID_SetOutputLimit(PID_t *pid, float min, float max);
```

- **功能**: 设置输出限幅 (必须 min < max)

#### `PID_SetIntegralLimit`

```c
ErrorCode_t PID_SetIntegralLimit(PID_t *pid, float max_value);
```

- **功能**: 设置积分项限幅 (抗饱和)

#### `PID_SetDeadZone`

```c
ErrorCode_t PID_SetDeadZone(PID_t *pid, float dead_zone);
```

- **功能**: 设置死区 (误差绝对值小于此值时输出为 0)

#### `PID_SetDerivativeFilter`

```c
ErrorCode_t PID_SetDerivativeFilter(PID_t *pid, float alpha);
```

- **功能**: 设置微分低通滤波系数
- **说明**: `alpha` 范围 [0, 1]，0=完全滤除，1=无滤波

#### `PID_SetConditionalIntegral`

```c
ErrorCode_t PID_SetConditionalIntegral(PID_t *pid, bool enable, float threshold);
```

- **功能**: 启用/禁用条件积分抗饱和
- **说明**: 当输出已饱和且误差方向会加剧饱和时停止积分

#### `PID_SetFeedforward`

```c
ErrorCode_t PID_SetFeedforward(PID_t *pid, float feedforward);
```

- **功能**: 设置前馈值，直接加到输出中

#### `PID_Calculate`

```c
float PID_Calculate(PID_t *pid, float feedback);
```

- **功能**: 计算 PID 输出 (需周期性调用)
- **参数**: `feedback` - 当前反馈值 (传感器读数)
- **返回**: PID 输出值 (已限幅)
- **公式 (位置式)**: `output = Kp*e + Ki*∫e*dt + Kd*de/dt + feedforward`
- **公式 (增量式)**: `Δu = Kp*Δe + Ki*e*dt + Kd*Δ²e/dt`，`output += Δu`

#### `PID_Reset`

```c
ErrorCode_t PID_Reset(PID_t *pid);
```

- **功能**: 重置内部状态 (不清除参数和目标值)

#### `PID_GetError` / `PID_GetOutput`

```c
float PID_GetError(const PID_t *pid);
float PID_GetOutput(const PID_t *pid);
```

### 使用示例

```c
PID_t pid;
PID_Init(&pid, PID_MODE_POSITION, 1.0f, 0.1f, 0.01f, 0.01f);
PID_SetTarget(&pid, 100.0f);
PID_SetOutputLimit(&pid, -500.0f, 500.0f);

while (1) {
    float sensor = ReadSensor();
    float output = PID_Calculate(&pid, sensor);
    Motor_SetSpeed(&motor, (int16_t)output);
    HAL_Delay(10);  // 10ms 周期
}
```

---

## 电机驱动

**头文件**: `drivers/motor.h`  
**源文件**: `drivers/motor.c`

### 类型定义

#### `MotorDriver_t`

| 枚举值 | 说明 |
|---|---|
| `MOTOR_DRV_L298N` | L298N 双 H 桥驱动 |
| `MOTOR_DRV_TB6612` | TB6612FNG 驱动 |

#### `Motor_t` 结构体

| 字段 | 类型 | 说明 |
|---|---|---|
| `driver` | `MotorDriver_t` | 驱动芯片类型 |
| `in1_port, in1_pin` | `GPIO_TypeDef*, uint16_t` | IN1 引脚 |
| `in2_port, in2_pin` | `GPIO_TypeDef*, uint16_t` | IN2 引脚 |
| `pwm_htim` | `TIM_HandleTypeDef*` | PWM 定时器 |
| `pwm_channel` | `uint32_t` | PWM 通道 |
| `speed` | `int16_t` | 当前速度 [-1000, 1000] |
| `initialized` | `bool` | 初始化标志 |

### API 函数

#### `Motor_Init`

```c
ErrorCode_t Motor_Init(Motor_t *motor, MotorDriver_t driver,
                       GPIO_TypeDef *in1_port, uint16_t in1_pin,
                       GPIO_TypeDef *in2_port, uint16_t in2_pin,
                       TIM_HandleTypeDef *htim, uint32_t channel);
```

- **功能**: 初始化电机驱动
- **默认**: 停止状态 (COAST)，PWM=0

#### `Motor_SetSpeed`

```c
ErrorCode_t Motor_SetSpeed(Motor_t *motor, int16_t speed);
```

- **功能**: 设置电机速度
- **范围**: [-1000, +1000]，正值正转，负值反转，0 停止
- **特性**: 速度绝对值 < 10 时自动归零

#### `Motor_GetSpeed`

```c
int16_t Motor_GetSpeed(const Motor_t *motor);
```

#### `Motor_Brake`

```c
ErrorCode_t Motor_Brake(Motor_t *motor);
```

- **功能**: 快速制动 (IN1=HIGH, IN2=HIGH)

#### `Motor_Coast`

```c
ErrorCode_t Motor_Coast(Motor_t *motor);
```

- **功能**: 滑行停止 (IN1=LOW, IN2=LOW)

#### `Motor_DeInit`

```c
ErrorCode_t Motor_DeInit(Motor_t *motor);
```

- **功能**: 反初始化 (停止 PWM 输出)

### 使用示例

```c
Motor_t motor;
Motor_Init(&motor, MOTOR_DRV_TB6612,
           GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1,
           &htim2, TIM_CHANNEL_1);

Motor_SetSpeed(&motor, 500);   // 正转 50%
Motor_SetSpeed(&motor, -300);  // 反转 30%
Motor_Brake(&motor);           // 制动
```

---

## 舵机控制

**头文件**: `drivers/servo.h`  
**源文件**: `drivers/servo.c`

### API 概览

| 函数 | 说明 |
|---|---|
| `Servo_Init(servo, htim, channel)` | 初始化舵机 (默认 90° 中位) |
| `Servo_SetAngle(servo, angle)` | 设置角度 (0~180°) |
| `Servo_GetAngle(servo)` | 获取当前角度 |
| `Servo_SetRange(servo, min_us, max_us)` | 设置脉宽范围 (默认 500~2500μs) |
| `Servo_DeInit(servo)` | 反初始化 |

---

## 编码器读取

**头文件**: `drivers/encoder.h`  
**源文件**: `drivers/encoder.c`

### API 概览

| 函数 | 说明 |
|---|---|
| `Encoder_Init(enc, htim)` | 初始化编码器 (定时器编码器模式) |
| `Encoder_Read(enc)` | 读取计数值 (带溢出处理) |
| `Encoder_GetSpeed(enc)` | 计算转速 (脉冲/秒) |
| `Encoder_Reset(enc)` | 清零计数 |
| `Encoder_SetPPR(enc, ppr)` | 设置每转脉冲数 |

---

## 红外传感器

**头文件**: `drivers/sensor_ir.h`  
**源文件**: `drivers/sensor_ir.c`

### API 概览

| 函数 | 说明 |
|---|---|
| `IRSensor_Init(adc_handle, channel)` | 初始化 (ADC 通道) |
| `IRSensor_Read(adc_handle, channel)` | 读取 ADC 原始值 (0~4095) |
| `IRSensor_ReadDistance(adc_handle, channel)` | 读取距离 (cm) |
| `IRSensor_IsOnLine(adc_handle, channel, threshold)` | 判断是否在线上 |

---

## OLED 显示

**头文件**: `drivers/oled.h`  
**源文件**: `drivers/oled.c`

### API 概览

| 函数 | 说明 |
|---|---|
| `OLED_Init()` | 初始化 OLED (I2C/SPI) |
| `OLED_Clear()` | 清屏 |
| `OLED_ShowString(x, y, str, size)` | 显示字符串 |
| `OLED_ShowNum(x, y, num, len, size)` | 显示数字 |
| `OLED_ShowChinese(x, y, index)` | 显示汉字 |
| `OLED_DrawBMP(x0, y0, x1, y1, bmp)` | 显示位图 |
| `OLED_Refresh()` | 刷新显示缓冲区 |

---

## 按键驱动

**头文件**: `drivers/key.h`  
**源文件**: `drivers/key.c`

### API 概览

| 函数 | 说明 |
|---|---|
| `Key_Init()` | 初始化按键 GPIO |
| `Key_Scan()` | 扫描按键 (支持消抖) |
| `Key_WaitPress()` | 阻塞等待按键按下 |

---

## 数学工具

**头文件**: `utils/math_utils.h`

| 函数/宏 | 说明 |
|---|---|
| `CLAMP(val, min, max)` | 数值限幅 |
| `ABS(x)` | 绝对值 |
| `MAP(x, in_min, in_max, out_min, out_max)` | 范围映射 |
| `DEG_TO_RAD(deg)` | 角度转弧度 |
| `RAD_TO_DEG(rad)` | 弧度转角度 |
| `MovingAvg_Init(buf, size)` | 初始化滑动平均 |
| `MovingAvg_Update(avg, value)` | 更新滑动平均 |
| `MovingAvg_Get(avg)` | 获取平均值 |

---

## 环形缓冲区

**头文件**: `utils/ring_buffer.h`

| 函数 | 说明 |
|---|---|
| `RingBuf_Init(rb, buf, size)` | 初始化 |
| `RingBuf_Put(rb, data)` | 写入一个字节 |
| `RingBuf_Get(rb, data)` | 读取一个字节 |
| `RingBuf_IsFull(rb)` | 是否满 |
| `RingBuf_IsEmpty(rb)` | 是否空 |
| `RingBuf_Count(rb)` | 可读数据量 |
| `RingBuf_Flush(rb)` | 清空 |

---

## 快速参考

### 常用组合: 电机 + PID 闭环

```c
// 初始化
PID_t pid;
Motor_t motor;
Encoder_t encoder;

PID_Init(&pid, PID_MODE_POSITION, 1.5f, 0.2f, 0.05f, 0.01f);
PID_SetTarget(&pid, 300.0f);  // 目标转速
Motor_Init(&motor, MOTOR_DRV_TB6612, ...);
Encoder_Init(&encoder, &htim3);

// 主循环 (10ms)
while (1) {
    float speed = (float)Encoder_GetSpeed(&encoder);
    float output = PID_Calculate(&pid, speed);
    Motor_SetSpeed(&motor, (int16_t)output);
    HAL_Delay(10);
}
```

### 平台移植

本代码库通过 `platform/hal_stm32.h` 实现硬件抽象。移植到其他 MCU 时需修改该文件中的宏定义。
