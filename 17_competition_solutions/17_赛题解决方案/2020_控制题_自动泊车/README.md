# 2020年全国大学生电子设计竞赛 - 控制题（C题）坡道行驶电动小车 完整解决方案

## 一、题目分析

### 1.1 题目要求概述

设计坡道行驶电动小车，要求：
- **坡道行驶**：在不同角度坡道上稳定行驶
- **速度控制**：上坡、下坡速度可调
- **定点停车**：在坡道上指定位置停车
- **自动启停**：坡道上停车后能重新启动
- **角度测量（发挥部分）**：实时测量坡道角度

### 1.2 核心技术指标

| 指标 | 基本要求 | 发挥部分 |
|------|---------|---------|
| 坡道角度 | 15° | 30° |
| 上坡速度 | 0.2-0.5m/s | 0.1-1.0m/s |
| 下坡速度 | 0.2-0.5m/s | 0.1-1.0m/s |
| 停车精度 | ±10cm | ±5cm |
| 角度测量 | 无 | ±1°精度 |
| 坡道启停 | 能启停 | 平稳启停无溜车 |

### 1.3 难点分析

1. **重力补偿**：上坡需要额外扭矩，下坡需要制动力
2. **防溜车**：坡道停车和启动时防止溜车
3. **角度测量精度**：IMU噪声和漂移问题
4. **速度稳定性**：坡道上保持匀速行驶

## 二、系统方案设计

### 2.1 方案论证

#### 2.1.1 角度测量方案

| 方案 | 传感器 | 优点 | 缺点 | 选择 |
|------|--------|------|------|------|
| **方案A：MPU6050** | 6轴IMU | 成本低、响应快 | 有漂移 | **主方案** |
| 方案B：倾角传感器 | SCA100T | 精度高 | 成本高 | 备用 |
| 方案C：编码器+计算 | 编码器 | 无需额外传感器 | 需要速度信息 | 辅助 |

#### 2.1.2 电机驱动方案

| 方案 | 驱动芯片 | 优点 | 缺点 | 选择 |
|------|---------|------|------|------|
| **方案A：TB6612** | TB6612FNG | 效率高、支持刹车 | 电流1.2A | **主方案** |
| 方案B：L298N | L298N | 驱动能力强 | 效率低、发热 | 备用 |
| 方案C：DRV8833 | DRV8833 | 低电压驱动 | 电流小 | 放弃 |

#### 2.1.3 速度控制方案

| 方案 | 传感器 | 优点 | 缺点 | 选择 |
|------|--------|------|------|------|
| **方案A：编码器** | 500线光电 | 精度高、直接测量 | 需要安装 | **主方案** |
| 方案B：霍尔传感器 | 霍尔编码器 | 简单 | 精度低 | 备用 |
| 方案C：电流检测 | 电流采样 | 间接测量 | 不准确 | 放弃 |

### 2.2 系统总体架构

```
┌─────────────────────────────────────────────────────────────┐
│                    坡道行驶电动小车系统                        │
│                                                              │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────┐ │
│  │ MPU6050     │───▶│  STM32F407  │───▶│  电机驱动       │ │
│  │ 6轴IMU     │    │  主控制器    │    │  TB6612FNG      │ │
│  │ 角度测量    │    │  PID控制    │    │  双电机         │ │
│  └─────────────┘    └──────┬──────┘    └─────────────────┘ │
│                            │                                 │
│  ┌─────────────┐    ┌──────┴──────┐    ┌─────────────────┐ │
│  │ 编码器       │───▶│  速度环     │───▶│  电源管理       │ │
│  │ 500线光电    │    │  闭环控制    │    │  12V锂电池     │ │
│  └─────────────┘    └──────┬──────┘    └─────────────────┘ │
│                            │                                 │
│  ┌─────────────┐    ┌──────┴──────┐    ┌─────────────────┐ │
│  │ 超声波       │───▶│  定点停车    │───▶│  OLED显示       │ │
│  │ HC-SR04     │    │  距离检测    │    │  角度/速度显示   │ │
│  └─────────────┘    └─────────────┘    └─────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## 三、硬件选型表

| 模块 | 型号 | 参数 | 数量 | 单价(元) | 备注 |
|------|------|------|------|---------|------|
| 主控MCU | STM32F407VET6 | 168MHz, 512KB Flash | 1 | 25 | 核心控制器 |
| 姿态传感器 | MPU6050 | 6轴IMU | 1 | 8 | 角度测量 |
| 超声波 | HC-SR04 | 2cm-400cm | 1 | 5 | 定点停车 |
| 红外对管 | TCRT5000 | 数字输出 | 5 | 2 | 循线辅助 |
| 电机驱动 | TB6612FNG | 双路1.2A | 1 | 8 | H桥驱动 |
| 直流电机 | JGA25-370 | 12V, 300RPM | 2 | 25 | 带编码器 |
| 编码器 | 500线增量式 | AB相 | 2 | 15 | 速度反馈 |
| OLED显示 | SSD1306 | 0.96" 128×64 | 1 | 8 | 参数显示 |
| 按键模块 | 4按键 | 独立按键 | 1 | 2 | 参数设置 |
| 电源 | 12V锂电池 | 3S 2200mAh | 1 | 60 | 主电源 |
| 稳压模块 | LM2596 | 5V/3.3V输出 | 1 | 5 | 降压供电 |
| **总计** | | | | **~165** | |

## 四、软件架构图

```
┌──────────────────────────────────────────────────────┐
│                    主程序(main.c)                      │
│  ┌──────────────────────────────────────────────────┐│
│  │              初始化层                              ││
│  │  GPIO / UART / TIM / ADC / I2C                  ││
│  └──────────────────────────────────────────────────┘│
│  ┌──────────────────────────────────────────────────┐│
│  │              驱动层                               ││
│  │  Motor.c    Encoder.c   MPU6050.c  Ultrasonic.c ││
│  └──────────────────────────────────────────────────┘│
│  ┌──────────────────────────────────────────────────┐│
│  │              算法层                               ││
│  │  PID.c      Kalman.c    AngleCompute.c          ││
│  └──────────────────────────────────────────────────┘│
│  ┌──────────────────────────────────────────────────┐│
│  │              应用层                               ││
│  │  SlopeTask.c — 坡道行驶 / 速度控制 / 定点停车     ││
│  └──────────────────────────────────────────────────┘│
└──────────────────────────────────────────────────────┘
```

## 五、核心算法说明

### 5.1 角度测量算法（互补滤波）

> ⚠️ 以下代码基于MSPM0G3507平台，使用实际驱动API，可直接编译。
> 需要链接: `tb6612.h`, `encoder.h`, `advanced_pid.h`

```c
/* ═══ 公共宏定义 ═══ */
#include <math.h>
#include "tb6612.h"        /* TB6612_SetMotor */
#include "encoder.h"       /* Encoder_GetSpeed */
#include "advanced_pid.h"  /* PID_Init, PID_Calc */

#define PI            3.14159265f
#define GRAVITY_COEFF 9.8f
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define BASE_SPEED    2000
```

```c
/* MPU6050互补滤波测角度 */
/* 修复#41: atan2 → atan2f, alpha加f后缀 */
typedef struct {
    float angle;      // 融合角度
    float gyro_bias;  // 陀螺仪零偏
} Angle_t;

float complementary_filter(Angle_t *att, float accel_angle, float gyro_rate, float dt) {
    if (dt <= 0.0f) dt = 0.001f;  /* 除零保护 */
    float alpha = 0.98f;

    float angle_gyro = att->angle + (gyro_rate - att->gyro_bias) * dt;
    att->angle = alpha * angle_gyro + (1 - alpha) * accel_angle;
    return att->angle;
}

/* 修复#41: atan2 → atan2f */
float accel_to_angle(float accel_x, float accel_z) {
    return atan2f(accel_x, accel_z) * 180.0f / PI;
}
```

### 5.2 坡道重力补偿算法

```c
/* 坡道行驶PID控制（含重力补偿） */
/* 修复C1: pid_calculate → PID_Calc (使用实际驱动API) */
/* 修复#41: sin → sinf */
/* 修复: constrain → CLAMP */

static PID_Controller slope_pid;
static int slope_pid_inited = 0;

float slope_speed_control(float target_speed, float actual_speed,
                          float slope_angle) {
    if (!slope_pid_inited) {
        PID_Param param = { .kp = 5.0f, .ki = 0.5f, .kd = 1.0f,
                            .output_min = -100.0f, .output_max = 100.0f,
                            .integral_max = 80.0f, .dead_zone = 0.5f };
        PID_Init(&slope_pid, &param);
        slope_pid_inited = 1;
    }
    /* 使用实际API: PID_Calc(pid, ref, feedback) */
    float pid_output = PID_Calc(&slope_pid, target_speed, actual_speed);

    /* 重力补偿: 上坡需要额外扭矩 T = mg*sin(θ) */
    float gravity_comp = GRAVITY_COEFF * sinf(slope_angle * PI / 180.0f);

    float output = pid_output + gravity_comp;
    output = CLAMP(output, -100.0f, 100.0f);
    return output;
}
```

### 5.3 坡道启停控制算法

```c
/* 修复C1: motor_set_speed → TB6612_SetMotor */
/* 修复C1: encoder_get_speed → Encoder_GetSpeed */
/* 修复#41: sin → sinf */
/* 注意: delay_ms() 需使用 HAL_Delay() 或等效实现 */

/* 坡道启动防溜车 */
void slope_start(float slope_angle) {
    /* 先施加刹车力 */
    TB6612_SetMotor(MOTOR_CH_A, MOTOR_DIR_BACKWARD, 20);
    TB6612_SetMotor(MOTOR_CH_B, MOTOR_DIR_BACKWARD, 20);
    /* delay_ms(100); — 需实现: HAL_Delay(100) */

    /* 逐渐增加驱动力 */
    for (int duty = 0; duty <= 60; duty += 5) {
        float comp = GRAVITY_COEFF * sinf(slope_angle * PI / 180.0f);
        int spd = CLAMP(duty + (int)comp, 0, 100);
        TB6612_SetMotor(MOTOR_CH_A, MOTOR_DIR_FORWARD, (uint32_t)spd);
        TB6612_SetMotor(MOTOR_CH_B, MOTOR_DIR_FORWARD, (uint32_t)spd);
        /* delay_ms(50); — 需实现 */
    }
}

/* 坡道停车防溜车 */
/* 修复: pid_brake 改为独立 PID_Controller 实例 */
void slope_stop(float slope_angle) {
    static PID_Controller pid_brake;
    static int brake_inited = 0;
    if (!brake_inited) {
        PID_Param bp = { .kp = 3.0f, .ki = 0.1f, .kd = 0.5f,
                         .output_min = -100.0f, .output_max = 100.0f,
                         .integral_max = 50.0f, .dead_zone = 0.0f };
        PID_Init(&pid_brake, &bp);
        brake_inited = 1;
    }

    /* 逐渐减速 */
    int32_t current_speed = Encoder_GetSpeed(ENC_LEFT);
    while (current_speed > 10) {
        float decel = PID_Calc(&pid_brake, 0.0f, (float)current_speed);
        int spd = CLAMP((int)decel, -100, 100);
        TB6612_SetMotor(MOTOR_CH_A, spd >= 0 ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD, (uint32_t)abs(spd));
        TB6612_SetMotor(MOTOR_CH_B, spd >= 0 ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD, (uint32_t)abs(spd));
        current_speed = Encoder_GetSpeed(ENC_LEFT);
        /* delay_ms(10); — 需实现 */
    }

    /* 施加保持力矩 */
    float hold_torque = GRAVITY_COEFF * sinf(slope_angle * PI / 180.0f) * 0.8f;
    int hold = CLAMP((int)hold_torque, 0, 100);
    TB6612_SetMotor(MOTOR_CH_A, MOTOR_DIR_FORWARD, (uint32_t)hold);
    TB6612_SetMotor(MOTOR_CH_B, MOTOR_DIR_FORWARD, (uint32_t)hold);
}
```

## 六、测试方案

### 6.1 测试项目

| 测试项 | 测试方法 | 合格标准 |
|--------|---------|---------|
| 角度测量精度 | 标准角度尺对比 | 误差<1° |
| 平地速度控制 | 匀速行驶 | 速度误差<5% |
| 15°坡道行驶 | 坡道上下行 | 速度稳定不溜车 |
| 30°坡道行驶 | 坡道上下行 | 能通过不翻车 |
| 坡道停车 | 坡道定点停车 | 停车精度<5cm |
| 坡道启停 | 坡道停车后启动 | 无溜车现象 |
| 长时间运行 | 坡道往复10次 | 角度无累积漂移 |

### 6.2 测试流程

```
Step1: 传感器标定
  ├── MPU6050零偏校准
  ├── 编码器方向确认
  └── 超声波测距标定

Step2: 角度测量测试
  ├── 静态角度精度
  ├── 动态角度响应
  └── 长时间漂移测试

Step3: 速度控制测试
  ├── 平地PID调参
  ├── 上坡速度控制
  └── 下坡速度控制

Step4: 坡道启停测试
  ├── 15°坡道启停
  ├── 30°坡道启停
  └── 多次启停重复性

Step5: 综合测试
  ├── 完整赛道测试
  ├── 定点停车测试
  └── 长时间稳定性测试
```

## 七、评分要点分析

### 7.1 基本要求评分（50分）

| 评分项 | 分值 | 要点 | 策略 |
|--------|------|------|------|
| 15°坡道行驶 | 20分 | 能稳定行驶 | 重力补偿PID |
| 速度可调 | 15分 | 上下坡速度可调 | 速度闭环控制 |
| 定点停车 | 15分 | 坡道上停车准确 | 编码器+超声波 |

### 7.2 发挥部分评分（50分）

| 评分项 | 分值 | 要点 | 策略 |
|--------|------|------|------|
| 30°坡道行驶 | 20分 | 陡坡稳定行驶 | 增大驱动力+防滑 |
| 角度测量 | 15分 | 实时显示坡道角度 | MPU6050+互补滤波 |
| 坡道启停 | 15分 | 停车后能重新启动 | 防溜车算法 |

### 7.3 得分策略

1. **保基础**：先确保15°坡道稳定行驶，拿稳30分
2. **优化重力补偿**：这是坡道行驶的核心算法
3. **角度测量**：发挥部分最大加分项
4. **防溜车**：坡道启停是难点，重点攻克

### 7.4 常见失分点

| 失分点 | 原因 | 解决方案 |
|--------|------|---------|
| 坡道溜车 | 重力补偿不足 | 增大补偿系数 |
| 速度不稳 | PID参数不当 | 针对坡道单独调参 |
| 角度漂移 | 陀螺仪漂移 | 定期零偏校准 |
| 翻车 | 重心太高 | 降低重心+增加配重 |

## 八、调试经验总结

### 8.1 MPU6050调试

```
1. 零偏校准：静止状态下采集1000个点取平均
2. 互补滤波系数：0.95-0.98之间选择
3. 采样频率：至少100Hz以上
4. 安装方向：确认X/Y/Z轴方向与坡道方向一致
```

### 8.2 重力补偿调参

```
1. 先在15°坡道测试基础补偿系数
2. 在30°坡道验证补偿效果
3. 上下坡分别调整补偿系数
4. 考虑摩擦力的影响，适当增加补偿
```

### 8.3 防溜车技巧

```
1. 停车时先减速再刹车
2. 启动时先施加保持力矩再加速
3. 使用编码器检测是否溜车
4. 溜车时立即增加制动力
```

### 8.4 硬件注意事项

```
1. 重心尽量低，电池放底部
2. 轮胎要有足够摩擦力（橡胶轮）
3. 电机扭矩要足够（考虑坡道最大扭矩）
4. 编码器安装要牢固，避免震动干扰
```

---
*本方案基于2020年C题（坡道行驶电动小车）完整解析，适用于电赛备战参考。*
