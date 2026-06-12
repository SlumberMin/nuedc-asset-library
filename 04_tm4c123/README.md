# TM4C123GH6PZT7 电赛通用代码库

## 📋 概述

本代码库为**全国大学生电子设计竞赛**提供TM4C123GH6PZT7平台的通用驱动和算法模块。

**MCU特性:**
- ARM Cortex-M4F @ 80MHz
- 硬件FPU (单精度浮点)
- 内置QEI正交编码器接口
- 8路ADC (12位, 1MSPS)
- 16路PWM输出

## 📁 目录结构

```
03_通用代码库_TM4C123/
├── platform/                   # 平台层
│   ├── tivaware.h             # TivaWare封装层(统一宏/内联函数)
│   ├── system_tm4c.h          # 系统初始化头文件
│   └── system_tm4c.c          # 系统初始化实现
├── drivers/                    # 驱动层
│   ├── motor_tm4c.h           # 电机驱动头文件
│   ├── motor_tm4c.c           # 电机驱动实现
│   ├── encoder_tm4c.h         # 编码器驱动头文件
│   ├── encoder_tm4c.c         # 编码器驱动实现(QEI)
│   ├── servo_tm4c.h           # 舵机驱动头文件
│   ├── servo_tm4c.c           # 舵机驱动实现
│   ├── sensor_ir_tm4c.h       # 红外循迹头文件
│   └── sensor_ir_tm4c.c       # 红外循迹实现(ADC)
├── algorithm/                  # 算法层
│   ├── pid_tm4c.h             # PID控制器头文件
│   └── pid_tm4c.c             # PID控制器实现(FPU加速)
├── README.md                   # 本文件
└── STM32_vs_TM4C_PLATFORM_DIFF.md  # 平台差异对比
```

## 🔧 模块使用指南

### 1. 系统初始化

```c
#include "platform/system_tm4c.h"

int main(void)
{
    system_init();  // 80MHz时钟 + FPU + SysTick 1ms
    
    // 你的代码...
    while(1) {
        delay_ms(10);
    }
}
```

**初始化内容:**
- 16MHz晶振 → PLL → 80MHz系统时钟
- FPU硬件浮点使能
- SysTick 1ms中断 (提供 `system_get_tick()`)

### 2. 电机驱动 (PWM)

```c
#include "drivers/motor_tm4c.h"

int main(void)
{
    system_init();
    motor_init();   // 初始化PWM0 + GPIO方向控制
    
    motor_set(MOTOR_CH_A, 500);   // 左电机正转 50%
    motor_set(MOTOR_CH_B, -300);  // 右电机反转 30%
    
    delay_ms(1000);
    motor_stop_all();
}
```

**默认引脚:**
| 通道 | PWM引脚 | IN1 | IN2 |
|------|---------|-----|-----|
| CH_A (左) | PB6 (M0PWM0) | PA2 | PA3 |
| CH_B (右) | PB7 (M0PWM1) | PA4 | PA5 |

**参数:** speed = -1000 ~ +1000 (负值反转)

### 3. 编码器 (QEI硬件解码)

```c
#include "drivers/encoder_tm4c.h"

int main(void)
{
    system_init();
    encoder_init();  // 初始化QEI0/QEI1
    
    while(1) {
        int32_t pos_l = encoder_get_position(ENC_CH_LEFT);
        int32_t vel_r = encoder_get_velocity(ENC_CH_RIGHT);
        float rpm = encoder_get_rpm(ENC_CH_LEFT);
        
        delay_ms(10);
    }
}
```

**默认引脚:**
| 通道 | QEI模块 | Phase A | Phase B |
|------|---------|---------|---------|
| 左轮 | QEI0 | PD6 (PHA0) | PD7 (PHB0) |
| 右轮 | QEI1 | PC5 (PHA1) | PC6 (PHB1) |

**特性:** 硬件自动4倍频正交解码，零CPU开销

### 4. 舵机驱动

```c
#include "drivers/servo_tm4c.h"

int main(void)
{
    system_init();
    servo_init();
    
    servo_set_angle(SERVO_CH_1, 90.0f);   // 归中
    servo_set_angle(SERVO_CH_2, 45.0f);   // 转到45°
    servo_set_pulse(SERVO_CH_1, 1500);    // 直接设置脉宽1500us
}
```

**默认引脚:** PF2 (M1PWM4), PF3 (M1PWM5)

**参数:** 0°~180° 或 500~2500us脉宽

### 5. 红外循迹 (ADC)

```c
#include "drivers/sensor_ir_tm4c.h"

int main(void)
{
    system_init();
    ir_sensor_init();
    ir_sensor_calibrate(200);  // 采集200次校准
    
    while(1) {
        int16_t pos = ir_sensor_get_position();  // -3500~+3500
        
        if (ir_sensor_line_lost(&data)) {
            // 丢线处理
        }
    }
}
```

**默认引脚:** PE0~PE3 (AIN3~AIN0), PD0~PD3 (AIN7~AIN4)

### 6. PID控制器 (FPU加速)

```c
#include "algorithm/pid_tm4c.h"

pid_t speed_pid;
pid_t steer_pid;

int main(void)
{
    system_init();
    
    // 速度环PID
    pid_init(&speed_pid, 2.0f, 0.5f, 0.1f);
    pid_set_output_limit(&speed_pid, -1000, 1000);
    
    // 转向环PID
    pid_init(&steer_pid, 1.0f, 0.0f, 0.3f);
    pid_set_output_limit(&steer_pid, -500, 500);
    
    while(1) {
        float speed_out = pid_calc(&speed_pid, target_speed, actual_speed);
        float steer_out = pid_calc(&steer_pid, 0, ir_position);
        
        motor_set(MOTOR_CH_A, (int16_t)(speed_out + steer_out));
        motor_set(MOTOR_CH_B, (int16_t)(speed_out - steer_out));
        
        delay_ms(5);
    }
}
```

**特性:**
- FPU硬件浮点运算 (单精度)
- 位置式/增量式PID
- 积分限幅 (抗饱和)
- 微分低通滤波 (抗噪声)

## ⚙️ TivaWare依赖

本代码库依赖TI官方TivaWare库。需要在工程中包含:
```
TivaWare_C_Series-2.2.0.295/
├── driverlib/       # 驱动库 (.lib 或源码)
├── inc/             # 头文件
└── usblib/          # (可选) USB库
```

**CCS工程配置:**
- Include Path: 添加 `TivaWare/` 目录
- 预定义宏: `TARGET_IS_TM4C123_RB1` (或 `PART_TM4C123GH6PZ`)
- Link: `driverlib.lib`

## 📌 注意事项

1. **引脚冲突**: 电机PWM用PB6/PB7, 编码器用PD6/PD7/PC5/PC6, 舵机用PF2/PF3, 互不冲突
2. **中断优先级**: SysTick最低, QEI/ADC按需配置
3. **FPU**: 系统初始化时自动使能, float运算零额外开销
4. **PWM频率**: 电机20kHz(静音), 舵机50Hz(标准)
5. **ADC**: 12位精度, 最大1MSPS采样率

## 🔗 参考资料

- [TM4C123GH6PZ数据手册](https://www.ti.com/lit/ds/symlink/tm4c123gh6pz.pdf)
- [TivaWare API文档](https://www.ti.com/lit/ug/spmu298/spmu298.pdf)
- [TM4C123 LaunchPad原理图](https://www.ti.com/lit/ug/spmu372/spmu372.pdf)
