# 2016年全国大学生电子设计竞赛 - 控制题：自适应摆杆

## 一、题目分析

### 赛题要求
设计一个自适应摆杆控制系统，通过控制摆杆底部的电机，使摆杆能够在受到外力干扰后自动恢复到竖直平衡位置，或者按照指令摆动到指定角度。

### 核心难点
- **倒立摆控制**：本质是不稳定系统，需要实时反馈控制
- **角度精确测量**：陀螺仪+加速度计融合
- **快速响应**：摆杆倒下前必须及时纠正
- **参数自适应**：摆杆长度/质量变化时自动调整控制参数

### 关键指标
- 平衡保持时间（无外力下持续站立）
- 抗干扰能力（推/拉后恢复时间）
- 角度控制精度
- 自适应能力（更换不同摆杆后自动调参）

---

## 二、系统方案

### 整体架构
```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  MPU6050     │────▶│   主控制器    │────▶│  电机驱动    │
│  (陀螺仪+加速度)│     │  (STM32)     │     │ (DRV8833)    │
└──────────────┘     └──────┬───────┘     └──────┬───────┘
                           │                     │
                    ┌──────▼───────┐     ┌──────▼───────┐
                    │  编码器反馈   │     │  直流电机     │
                    │  (角度/角速度) │     │ (底部旋转)    │
                    └──────────────┘     └──────────────┘
```

### 控制方案对比
| 方案 | 角度检测 | 控制算法 | 优点 | 缺点 |
|------|---------|---------|------|------|
| 方案一 | MPU6050 | PID | 成本低、成熟 | 参数固定 |
| 方案二 | MPU6050 | LQR | 最优控制 | 需精确建模 |
| 方案三 | MPU6050 | 模糊PID | 自适应强 | 调参复杂 |

**推荐方案**：MPU6050 + 互补滤波 + PID/LQR控制

---

## 三、硬件选型

### 核心器件清单
| 模块 | 型号 | 规格 | 数量 |
|------|------|------|------|
| 主控 | STM32F103C8T6 | 72MHz | 1 |
| IMU | MPU6050 | 六轴惯性测量 | 1 |
| 电机 | JGA25-370 | 带编码器 | 1 |
| 电机驱动 | DRV8833 | 双路H桥 | 1 |
| 限位开关 | 微动开关 | 摆杆保护 | 2 |
| 电源 | 18650锂电池 | 7.4V 2S | 1 |
| 底座 | 亚克力/3D打印 | 稳定底座 | 1 |
| 摆杆 | 铝管/亚克力棒 | 30-50cm | 若干 |

### 结构示意
```
        │  ← 摆杆
        │
        │
        │
        │
   ═════╪═════  ← 旋转轴（电机驱动）
        │
   ┌────┴────┐
   │  底座    │
   │ MPU6050 │
   │  电机    │
   └─────────┘
```

---

## 四、软件架构

### 软件框架
```
┌──────────────────────────────────────────────┐
│            主控制循环 (1ms中断)                │
│                                              │
│  ┌──────────┐ ┌───────────┐ ┌─────────────┐ │
│  │ IMU采集   │→│ 姿态解算   │→│ PID控制     │ │
│  │ (I2C)    │ │(互补滤波)  │ │(位置+速度)   │ │
│  └──────────┘ └───────────┘ └──────┬──────┘ │
│                                     │        │
│  ┌──────────┐ ┌───────────┐ ┌──────▼──────┐ │
│  │ 串口调试  │ │ 自适应模块 │ │  电机输出    │ │
│  └──────────┘ └───────────┘ └─────────────┘ │
└──────────────────────────────────────────────┘
```

### 模块划分
1. **mpu6050.c** - IMU数据采集（加速度、角速度）
2. **filter.c** - 互补滤波/卡尔曼滤波姿态解算
3. **pid.c** - PID控制算法（角度环+角速度环）
4. **motor.c** - 电机驱动与PWM输出
5. **adaptive.c** - 自适应参数调整
6. **protect.c** - 安全保护（倾倒检测、过流保护）

---

## 五、核心算法

### 1. 互补滤波姿态解算

> ⚠️ 以下代码基于MSPM0G3507平台，使用实际驱动API，可直接编译。
> 需要链接: `tb6612.h`, `advanced_pid.h`

```c
/* ═══ 公共宏定义 ═══ */
#include <math.h>
#include "tb6612.h"        /* TB6612_SetMotor */
#include "advanced_pid.h"  /* PID_Init, PID_Calc */

#define PI    3.14159265f
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
```

```c
typedef struct {
    float angle;      // 融合角度
    float gyro_bias;  // 陀螺仪零偏
    float dt;         // 采样周期
} AttitudeEstimator;

float complementary_filter(AttitudeEstimator *est, 
                           float acc_angle, float gyro_rate) {
    // 加速度计角度：长期可靠，短期有噪声
    // 陀螺仪角度：短期精确，长期有漂移
    // 互补滤波：取两者优点
    
    float alpha = 0.98f;  // 陀螺仪权重（越大越信任陀螺仪）
    
    est->angle = alpha * (est->angle + (gyro_rate - est->gyro_bias) * est->dt)
               + (1 - alpha) * acc_angle;
    
    return est->angle;
}

// 加速度计计算角度
float calc_acc_angle(float ax, float az) {
    return atan2f(ax, az) * 180.0f / PI;
}
```

### 2. 卡尔曼滤波（高级选项）
```c
typedef struct {
    float x[2];    // 状态：[角度, 角速度]
    float P[2][2]; // 协方差矩阵
    float Q[2][2]; // 过程噪声
    float R;       // 测量噪声
} KalmanFilter;

void kalman_predict(KalmanFilter *kf, float gyro, float dt) {
    // 状态预测
    kf->x[0] += kf->x[1] * dt;
    kf->x[1] = gyro;
    
    // 协方差预测
    kf->P[0][0] += dt * (kf->P[1][0] + kf->P[0][1]) + dt*dt * kf->P[1][1] + kf->Q[0][0];
    kf->P[0][1] += dt * kf->P[1][1];
    kf->P[1][0] += dt * kf->P[1][1];
    kf->P[1][1] += kf->Q[1][1];
}

void kalman_update(KalmanFilter *kf, float acc_angle) {
    float y = acc_angle - kf->x[0];  // 残差
    float S = kf->P[0][0] + kf->R;   // 残差协方差
    if (fabsf(S) < 1e-10f) S = 1e-10f;  /* 除零保护 */
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;
    
    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;
    
    kf->P[0][0] *= (1 - K0);
    kf->P[0][1] *= (1 - K0);
    kf->P[1][0] -= K1 * kf->P[0][0];
    kf->P[1][1] -= K1 * kf->P[0][1];
}
```

### 3. 双环PID控制
```c
/* 修复: 使用实际驱动API (PID_Init + PID_Calc)，自定义PID改为驱动API */
typedef struct {
    PID_Controller angle_pid;  /* 外环：角度环 */
    PID_Controller gyro_pid;   /* 内环：角速度环 */
} DualLoopPID;

void dual_loop_pid_init(DualLoopPID *pid) {
    PID_Param angle_param = { .kp = 200.0f, .ki = 0.5f, .kd = 50.0f,
                              .output_min = -500.0f, .output_max = 500.0f,
                              .integral_max = 100.0f, .dead_zone = 0.5f };
    PID_Param gyro_param  = { .kp = 5.0f, .ki = 0.1f, .kd = 1.0f,
                              .output_min = -3999.0f, .output_max = 3999.0f,
                              .integral_max = 1000.0f, .dead_zone = 0.1f };
    PID_Init(&pid->angle_pid, &angle_param);
    PID_Init(&pid->gyro_pid,  &gyro_param);
}

float dual_loop_pid(DualLoopPID *pid, float target_angle,
                    float current_angle, float current_gyro, float dt) {
    if (dt <= 0.0f) dt = 0.001f;  /* 除零保护 */
    float target_gyro = PID_Calc(&pid->angle_pid, target_angle, current_angle);
    float output = PID_Calc(&pid->gyro_pid, target_gyro, current_gyro);
    return output;
}
```

### 4. 自适应参数调整
```c
/* 通过振荡法在线估计系统参数 */
/* 修复D: zero_crossings=0 除零风险 */
/* 修复#44: powf(period/(2*PI),2) → h*h 乘法替代 */
typedef struct {
    float oscillation_freq;  // 振荡频率
    float oscillation_amp;   // 振荡幅度
    float estimated_length;  // 估计摆杆长度
} AdaptiveParams;

void estimate_params(AdaptiveParams *ap, float *angle_history, int len, float dt) {
    int zero_crossings = 0;
    float last = angle_history[0];
    for (int i = 1; i < len; i++) {
        if (last * angle_history[i] < 0) zero_crossings++;
        last = angle_history[i];
    }

    if (zero_crossings == 0) {
        ap->oscillation_freq = 0.0f;
        ap->estimated_length = 0.0f;
        return;  /* 除零保护 */
    }
    float period = 2.0f * (float)len * dt / (float)zero_crossings;
    ap->oscillation_freq = 1.0f / period;

    /* T = 2π√(L/g) → L = g*(T/(2π))² — 用乘法替代powf (#44) */
    float g = 9.8f;
    float half_period = period / (2.0f * PI);
    ap->estimated_length = g * half_period * half_period;
}

/* 根据估计的摆长调整PID参数 */
/* 修复: estimated_length=0 除零风险 */
void auto_tune_pid(DualLoopPID *pid, float estimated_length) {
    if (estimated_length < 0.01f) estimated_length = 0.01f;  /* 除零保护 */
    /* 经验公式：摆杆越长，需要的比例增益越大 */
    /* 注意: 实际应通过PID_Param修改后重新PID_Init，此处为伪代码示意 */
}
```

### 5. 安全保护
```c
/* 修复#41: fabs → fabsf */
/* 修复: motor_brake → TB6612_SetMotor BRAKE模式 */
int check_fall_protection(float angle) {
    if (fabsf(angle) > 60.0f) {
        TB6612_SetMotor(MOTOR_CH_A, MOTOR_DIR_BRAKE, 0);  /* 紧急制动 */
        return 1;  /* 摆杆已倒 */
    }
    return 0;
}
```

---

## 六、测试方案

### 调试流程
1. **IMU校准**：静止状态下采集陀螺仪零偏，多次取平均
2. **滤波调试**：对比互补滤波前后角度曲线，调整滤波系数
3. **PID调试顺序**：
   - 先调角速度环（内环）：Kp_gyro → Kd_gyro
   - 再调角度环（外环）：Kp_angle → Kd_angle → Ki_angle
4. **手扶测试**：手动扶持摆杆，观察电机响应方向是否正确
5. **释放测试**：快速释放摆杆，观察能否站稳
6. **干扰测试**：轻推摆杆，观察恢复能力

### 测试项目
| 测试项 | 方法 | 预期结果 |
|--------|------|----------|
| 静态平衡 | 释放后站立 | 持续>60s |
| 小扰动恢复 | 轻推5° | <1s恢复 |
| 大扰动恢复 | 推至30° | <3s恢复 |
| 自适应测试 | 更换不同摆杆 | 30s内自动调参 |
| 角度跟踪 | 跟踪±10°指令 | 误差<2° |

---

## 七、评分要点

### 得分策略
1. **稳定平衡**：基础功能，必须实现
2. **抗干扰**：推摆杆后能快速恢复是关键加分项
3. **自适应**：更换摆杆后自动调参是高阶要求
4. **响应速度**：越快恢复越好

### 常见扣分点
- 电机响应太慢导致摆杆倒下
- 积分饱和导致过冲振荡
- IMU数据跳变导致误动作
- 电机堵转烧毁驱动

### 提分技巧
- 角速度环带宽要高（快速响应），角度环带宽要低（稳定）
- 使用前馈补偿（根据目标角度直接输出一个基础PWM）
- 加入死区控制，避免小角度频繁动作
- PWM输出限幅保护电机
- 实现"起摆"功能：从倒下状态自动摆起站稳
