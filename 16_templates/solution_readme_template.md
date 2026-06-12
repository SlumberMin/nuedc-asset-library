# 方案名称: [XXX年电赛XX题] — [简要描述]

> **版本**: 2.0  
> **日期**: 2026-XX-XX  
> **平台**: MSPM0G3507 / TM4C123G / STM32F4xx  
> **状态**: ✅ 已验证

---

## 目录

1. [题目要求](#1-题目要求)
2. [系统架构](#2-系统架构)
3. [硬件设计](#3-硬件设计)
4. [软件设计](#4-软件设计)
5. [核心算法](#5-核心算法)
6. [调试记录](#6-调试记录)
7. [API映射表](#7-api映射表)

---

## 1. 题目要求

### 1.1 基本要求
- [ ] 要求1: ...
- [ ] 要求2: ...
- [ ] 要求3: ...

### 1.2 发挥部分
- [ ] 发挥1: ...
- [ ] 发挥2: ...

### 1.3 关键指标
| 指标 | 要求 | 实测 | 达标 |
|------|------|------|------|
| 精度 | ±1mm | ±0.8mm | ✅ |
| 响应时间 | <1s | 0.6s | ✅ |
| 稳定性 | 无振荡 | 小超调 | ✅ |

---

## 2. 系统架构

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   传感器层   │───→│   控制层     │───→│   执行层    │
│ (编码器/IMU) │    │ (PID/ADRC)  │    │ (电机/舵机) │
└─────────────┘    └─────────────┘    └─────────────┘
       │                  │                  │
       └──────────────────┴──────────────────┘
                    通信总线 (UART/I2C/SPI)
```

### 2.1 模块清单
| 模块 | 文件 | 功能 |
|------|------|------|
| PID控制器 | `drivers/pid.c/h` | 位置式/增量式PID |
| 电机驱动 | `drivers/tb6612.c/h` | 双路电机驱动 |
| 编码器 | `drivers/encoder.c/h` | 正交解码计数 |
| OLED显示 | `drivers/oled_ssd1306.c/h` | I2C显示屏 |

---

## 3. 硬件设计

### 3.1 引脚分配 (参照 pin_config.h)

> **重要**: 引脚分配必须与pin_config.h一致，不能凭记忆分配 (错误经验 #13)

| 功能 | 引脚 | 备注 |
|------|------|------|
| 电机A PWM | PA12 (TIMA0, CC_0_INDEX) | **必须用TIMA0高级定时器** |
| 电机B PWM | PA13 (TIMA0, CC_3_INDEX) | 同上 |
| 编码器A | PB4/PB5 | **不要用PB6/PB7(与超声波冲突)** |
| 超声波Trig | PB6 | 与编码器分时复用 |
| 超声波Echo | PB7 | 同上 |
| OLED SDA | PB2 | I2C |
| OLED SCL | PB3 | I2C |

### 3.2 电路图
<!-- 插入电路图或链接 -->

### 3.3 注意事项
- ⚠️ 右编码器引脚不要使用PB6/PB7，与超声波Trig/Echo冲突 (错误经验 #17)
- ⚠️ 电机PWM必须使用TIMA0(高级定时器)，不能用TIMG0 (错误经验 #13)

---

## 4. 软件设计

### 4.1 目录结构
```
project/
├── drivers/          # 驱动层
│   ├── pid.c/h       # PID控制器
│   ├── tb6612.c/h    # 电机驱动
│   ├── encoder.c/h   # 编码器
│   └── oled_ssd1306.c/h
├── platform/         # 平台层
│   └── driverlib_mspm0.c/h
├── examples/         # 示例代码
│   └── balance_car_demo.c
└── tests/            # Python测试
    ├── wrappers.py
    └── test_*.py
```

### 4.2 初始化流程
```c
/* 必要的头文件包含 */
#include "drivers/pid.h"
#include "drivers/tb6612.h"
#include "drivers/encoder.h"
#include "drivers/oled_ssd1306.h"
#include "platform/driverlib_mspm0.h"

/* 必要的宏定义 (错误经验 #42) */
#define CLAMP(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define DEG2RAD(d)        ((d) * 0.0174532925f)
#define RAD2DEG(r)        ((r) * 57.29577951f)

/* 使用自定义驱动架构 (不是ti_msp_dl_config.h) (错误经验 #19) */
int main(void)
{
    /* 系统初始化 */
    SYSCFG_DL_init();  /* SysConfig生成的引脚配置 */

    /* 外设初始化 — 使用自定义驱动API */
    PID_Init(&pid_left, 1.0f, 0.1f, 0.01f);    /* kp, ki, kd */
    Motor_Init();         /* TB6612电机驱动 */
    Encoder_Init();       /* 编码器初始化 */
    OLED_Init();          /* OLED显示初始化 */

    /* 主循环 */
    while (1) {
        int16_t speed = Encoder_GetSpeed();      /* 读取编码器 */
        float output = PID_Calc(&pid_left, target, speed);  /* PID计算 */
        TB6612_SetMotor(output);                 /* 驱动电机 */
    }
}
```

### 4.3 关键设计决策
| 决策 | 选择 | 原因 |
|------|------|------|
| PID模式 | 位置式 | 响应快，适合实时控制 |
| PWM频率 | 20kHz | 超出人耳频率，无噪声 |
| 通信协议 | UART 115200 | 调试数据传输 |

---

## 5. 核心算法

### 5.1 算法概述
<!-- 算法原理说明 -->

### 5.2 代码实现

> **重要**: 以下代码使用**实际驱动API**，可直接编译 (错误经验 #40, #43)

#### 必要的头文件和宏定义

```c
/* 头文件 */
#include "drivers/pid.h"        /* PID_Controller, PID_Init, PID_Calc */
#include "drivers/tb6612.h"     /* TB6612_SetMotor */
#include "drivers/encoder.h"    /* Encoder_GetSpeed */
#include "drivers/servo.h"      /* Servo_SetAngle, Servo_SetPulse_us */
#include <math.h>               /* fabsf, sinf, cosf (错误经验 #41: 用f后缀) */
#include <stdint.h>

/* 宏定义 (错误经验 #42) */
#ifndef CLAMP
#define CLAMP(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

#ifndef M_PI_F
#define M_PI_F 3.14159265f
#endif

#ifndef GRAVITY
#define GRAVITY 9.80665f
#endif
```

#### PID控制回路 (使用实际API)

```c
/* PID控制器实例 (类型必须与头文件一致, 错误经验 #14, #43) */
PID_Controller pid_speed;
PID_Controller pid_angle;

void control_loop_init(void)
{
    /* PID_Init(PID_Param *param, float kp, float ki, float kd) */
    /* 注意: 第一个参数是PID_Param, 不是PID_Controller */
    PID_Param param_speed = { .kp = 2.0f, .ki = 0.5f, .kd = 0.1f };
    PID_Init(&pid_speed, &param_speed);

    PID_Param param_angle = { .kp = 8.0f, .ki = 0.2f, .kd = 0.5f };
    PID_Init(&pid_angle, &param_angle);
}

float control_loop_step(float target_angle, float current_angle)
{
    /* 外环: 角度PID → 目标速度 */
    float target_speed = PID_Calc(&pid_angle, target_angle, current_angle);

    /* 内环: 速度PID → 电机输出 */
    float current_speed = (float)Encoder_GetSpeed();  /* 实际API */
    float motor_output = PID_Calc(&pid_speed, target_speed, current_speed);

    /* 输出钳位 (错误经验 #1: 防御性编程) */
    motor_output = CLAMP(motor_output, -100.0f, 100.0f);

    /* 驱动电机 (实际API: TB6612_SetMotor) */
    TB6612_SetMotor(motor_output);

    return motor_output;
}
```

#### 舵机控制 (使用实际API)

```c
void steer_control(float angle_deg)
{
    /* 角度范围校验 (错误经验 #34: 除零保护) */
    angle_deg = CLAMP(angle_deg, -45.0f, 45.0f);

    /* 实际API: Servo_SetAngle */
    Servo_SetAngle(angle_deg);

    /* 或使用脉宽控制: Servo_SetPulse_us(500~2500) */
}
```

#### 浮点数学函数 (错误经验 #41)

```c
/* !! 错误写法 (double版本, 无FPU时慢2-4倍): */
/* float y = fabs(x); */
/* float y = sin(x); */
/* float y = atan2(y, x); */
/* float y = pow(x, 2); */

/* !! 正确写法 (float版本): */
float x = 1.5f;
float y;

y = fabsf(x);         /* fabs → fabsf */
y = sinf(x);          /* sin → sinf */
y = cosf(x);          /* cos → cosf */
y = atan2f(1.0f, x);  /* atan2 → atan2f */
y = sqrtf(x);         /* sqrt → sqrtf */

/* powf(x, 2) 可替换为乘法 (错误经验 #44) */
float x_squared = x * x;           /* 替代 powf(x, 2) */
float x_cubed   = x * x * x;       /* 替代 powf(x, 3) */
```

### 5.3 数学公式验证清单

> **重要**: 复制公式时必须与原始论文逐项核对 (错误经验 #20, #21)

| 公式 | 来源 | 核对状态 |
|------|------|----------|
| PID离散化 | 教材 | ✅ |
| Cohen-Coon整定 | Cohen & Coon 1953 | ⬜ 待核对 |
| 互补滤波 | 原始论文 | ⬜ 待核对 |

### 5.4 除零防护清单

> 所有除法都必须检查除数 (错误经验 #1, #45)

```c
/* 示例: 带除零保护的平均值计算 */
float safe_average(float *data, int len)
{
    /* 错误经验 #1: 除数校验 */
    if (data == NULL || len <= 0) {
        return 0.0f;  /* 安全默认值 */
    }

    float sum = 0.0f;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }

    /* !! 错误: return sum / len; (len已在上面检查，但防御性编程) */
    /* !! 正确: */
    return sum / (float)len;  /* len已校验>0 */
}
```

---

## 6. 调试记录

### 6.1 调试日志

| 日期 | 问题 | 原因 | 解决方案 |
|------|------|------|----------|
| 2026-XX-XX | 电机不转 | PWM用的TIMG0 | 改用TIMA0 (错误经验 #13) |
| 2026-XX-XX | 编码器跳变 | PB6/PB7与超声波冲突 | 改用PB4/PB5 (错误经验 #17) |
| 2026-XX-XX | 控制发散 | 正反馈 | state -= output*dt (错误经验 #18) |

### 6.2 性能测试结果

| 测试项 | 结果 | 截图 |
|--------|------|------|
| 阶跃响应 | 上升时间0.3s, 超调5% | <!-- 链接 --> |
| 稳态精度 | ±0.5mm | <!-- 链接 --> |

---

## 7. API映射表

> **本节解决错误经验 #40**: README伪代码与实际驱动API不匹配

### 7.1 功能映射

| 伪代码中的函数 | 实际驱动API | 头文件 | 说明 |
|---------------|-------------|--------|------|
| `motor_set_speed(speed)` | `TB6612_SetMotor(speed)` | `drivers/tb6612.h` | 设置电机速度 |
| `servo_set_angle(angle)` | `Servo_SetAngle(angle)` | `drivers/servo.h` | 设置舵机角度 |
| `set_esc_pwm(pulse_us)` | `Servo_SetPulse_us(pulse_us)` | `drivers/servo.h` | ESC脉宽控制 |
| `encoder_get_speed()` | `Encoder_GetSpeed()` | `drivers/encoder.h` | 读取编码器速度 |
| `pid_calculate(...)` | `PID_Calc(pid, target, feedback)` | `drivers/pid.h` | PID计算 |
| `pid_init(kp, ki, kd)` | `PID_Init(pid, &param)` | `drivers/pid.h` | PID初始化 |
| `oled_print(str)` | `OLED_ShowString(x, y, str)` | `drivers/oled_ssd1306.h` | OLED显示字符串 |

### 7.2 类型映射

| 伪代码中的类型 | 实际驱动类型 | 头文件 |
|---------------|-------------|--------|
| `PID_t` | `PID_Controller` + `PID_Param` | `drivers/pid.h` |
| `Motor_t` | `TB6612_Motor_t` | `drivers/tb6612.h` |
| `Servo_t` | `Servo_Config_t` | `drivers/servo.h` |

### 7.3 两套驱动架构说明

| 架构 | 头文件 | API风格 | 适用场景 |
|------|--------|---------|----------|
| 自定义驱动 | `drivers/*.h` | `Module_Function()` | **推荐: 统一使用** |
| SysConfig | `ti_msp_dl_config.h` | `DL_XXX_YYY()` | 底层配置 |

> **建议**: 示例代码统一使用自定义驱动架构 (错误经验 #19)

### 7.4 引脚冲突速查

| 外设A | 外设B | 冲突引脚 | 解决方案 |
|-------|-------|----------|----------|
| 编码器 | 超声波 | PB6/PB7 | 编码器改用PB4/PB5 |
| 灰度传感器 | 编码器 | PB0~PB5 | 分时复用或选择其一 |
| 灰度传感器 | 超声波 | PB6~PB7 | 分时复用 |

---

## 附录

### A. 依赖列表
- MSPM0 SDK (或对应平台SDK)
- pin_config.h (引脚配置)
- drivers/ (驱动库)
- tests/ (Python测试)

### B. 编译命令
```bash
# 使用CCS (Code Composer Studio)
# 或使用makefile
make clean && make all
```

### C. 测试命令
```bash
cd tests/
python -m pytest test_*.py -v
```

---

<!-- 
模板说明 (使用时删除本注释):

本模板覆盖的错误模式:
  #14 示例代码与驱动API不匹配 → API映射表(第7节)
  #40 README函数名与驱动API不匹配 → 统一使用实际API名
  #41 浮点函数缺f后缀 → 使用fabsf/sinf/cosf
  #42 缺少宏定义 → 提供必要宏定义区块
  #43 伪代码→可编译代码 → 所有代码可直接编译
  #44 powf(x,2)→乘法 → x*x替代

章节结构:
  1. 题目要求 — 明确目标和指标
  2. 系统架构 — 模块关系和数据流
  3. 硬件设计 — 引脚分配(参照pin_config.h)
  4. 软件设计 — 目录结构和初始化流程
  5. 核心算法 — 可编译的代码实现
  6. 调试记录 — 问题和解决方案
  7. API映射表 — 伪代码→实际API对照
-->
