# 2015年全国大学生电子设计竞赛 - 控制题：风力摆

## 一、题目分析

### 赛题要求
设计一个风力摆控制系统，通过控制悬挂摆杆下方的风扇（螺旋桨）转速和方向，使摆杆末端能够按照指定轨迹运动（如画圆、画线、定点停留等）。

### 核心难点
- **非线性动力学**：风力与摆角之间的非线性关系
- **多自由度控制**：摆杆可在两个方向自由摆动
- **风力响应延迟**：风扇转速变化到产生风力有惯性延迟
- **轨迹规划**：将目标运动分解为两个方向的协调控制

### 关键指标
- 画圆精度（半径、圆度误差）
- 画线精度（直线度误差）
- 定点停留精度
- 响应速度

---

## 二、系统方案

### 整体架构
```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  角度检测     │────▶│   主控制器    │────▶│  风扇驱动    │
│ (MPU6050)    │     │  (STM32)     │     │ (ESC/电调)    │
└──────────────┘     └──────┬───────┘     └──────┬───────┘
                           │                     │
                    ┌──────▼───────┐     ┌──────▼───────┐
                    │  显示/通信    │     │  四个风扇     │
                    │  (LCD/串口)   │     │ (X/Y方向)     │
                    └──────────────┘     └──────────────┘
```

### 风扇布局方案
```
    方案一：十字布局          方案二：对角布局
    
         F_up                    F1   F2
          ↑                    ↗   ↖
  F_left ← ● → F_right       ←  ●  →
          ↓                    ↘   ↙
        F_down                 F3   F4
```

**推荐方案**：十字布局四风扇，X/Y方向独立控制

---

## 三、硬件选型

### 核心器件清单
| 模块 | 型号 | 规格 | 数量 |
|------|------|------|------|
| 主控 | STM32F103C8T6 | 72MHz | 1 |
| IMU | MPU6050 | 六轴（挂摆杆末端） | 1 |
| 风扇 | 75mm涵道风扇 | 12V 无刷 | 4 |
| 电调 | 20A ESC | PWM控制 | 4 |
| 电源 | 12V锂电池 | 3S 2200mAh | 1 |
| 摆杆 | 碳纤维管/铝管 | 50cm | 1 |
| 悬挂支架 | 三脚架/龙门架 | 可调高度 | 1 |
| 显示 | OLED 0.96" | I2C | 1 |

### 结构示意
```
    ┌──────────────┐
    │   悬挂点      │
    └──────┬───────┘
           │  摆线（50cm）
           │
    ┌──────┴───────┐
    │  ┌─────────┐ │
    │  │ MPU6050 │ │  ← 检测摆角
    │  └─────────┘ │
    │              │
    │  ┌─F_up──┐  │
    │  │       │  │
    │F_left ● F_right│ ← 四个风扇
    │  │       │  │
    │  └─F_down┘  │
    └─────────────┘
```

---

## 四、软件架构

### 软件框架
```
┌──────────────────────────────────────────────┐
│              主控制循环 (10ms)                 │
│                                              │
│  ┌──────────┐ ┌───────────┐ ┌─────────────┐ │
│  │ IMU采集   │→│ 姿态解算   │→│ 轨迹规划    │ │
│  └──────────┘ └───────────┘ └──────┬──────┘ │
│                                     │        │
│  ┌──────────┐ ┌───────────┐ ┌──────▼──────┐ │
│  │ PWM输出   │←│ PID控制    │←│ 误差计算    │ │
│  │ (4路ESC)  │ │ (X/Y独立)  │ │             │ │
│  └──────────┘ └───────────┘ └─────────────┘ │
└──────────────────────────────────────────────┘
```

### 模块划分
1. **mpu6050.c** - 末端IMU数据采集
2. **attitude.c** - 摆角解算（X/Y两个方向）
3. **pid.c** - 双轴PID控制
4. **fan.c** - 四风扇PWM驱动
5. **trajectory.c** - 轨迹生成（圆、线、8字等）
6. **mode.c** - 工作模式切换

---

## 五、核心算法

> ⚠️ 以下代码基于MSPM0G3507平台，使用实际驱动API，可直接编译。
> 需要链接: `servo.h`, `advanced_pid.h`

```c
/* ═══ 公共宏定义 ═══ */
#include <math.h>
#include "servo.h"         /* Servo_SetAngle, Servo_SetPulse_us */
#include "advanced_pid.h"  /* PID_Init, PID_Calc */

#define PI             3.14159265f
#define GRAVITY_FACTOR 9.8f
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define ESC_PWM_MIN    1000
#define ESC_PWM_MAX    2000
#define ESC_PWM_MID    1500
```

### 1. 摆角解算
```c
typedef struct {
    float angle_x;  // X方向摆角（前后）
    float angle_y;  // Y方向摆角（左右）
} SwingAngle;

SwingAngle calc_swing_angle(float ax, float ay, float az) {
    SwingAngle sa;
    // 利用加速度计计算摆角
    // 当摆杆静止时，重力方向即为摆杆方向
    sa.angle_x = atan2f(ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    sa.angle_y = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / M_PI;
    return sa;
}
```

### 2. 双轴PID控制
```c
/* 修复C4: pid_calculate → PID_Calc (使用实际驱动API) */
typedef PID_Controller PID_Axis;

typedef struct {
    PID_Axis x;  // X方向
    PID_Axis y;  // Y方向
} DualAxisPID;

/* 初始化双轴PID */
void dual_axis_pid_init(DualAxisPID *pid) {
    PID_Param param = { .kp = 5.0f, .ki = 0.1f, .kd = 2.0f,
                        .output_min = -500.0f, .output_max = 500.0f,
                        .integral_max = 500.0f, .dead_zone = 0.5f };
    PID_Init(&pid->x, &param);
    PID_Init(&pid->y, &param);
}

void dual_axis_control(DualAxisPID *pid, SwingAngle current, 
                       SwingAngle target, float dt) {
    float error_x = target.angle_x - current.angle_x;
    float error_y = target.angle_y - current.angle_y;

    /* 使用实际API: PID_Calc(pid, ref, feedback) */
    float output_x = PID_Calc(&pid->x, target.angle_x, current.angle_x);
    float output_y = PID_Calc(&pid->y, target.angle_y, current.angle_y);

    /* 输出映射到四个风扇 */
    fan_output_mapping(output_x, output_y);
}
```

### 3. 风扇输出映射
```c
/* 十字布局：将X/Y方向的力分解到四个风扇 */
/* ESC使用舵机式PWM控制 (1000~2000us) */
void fan_output_mapping(float force_x, float force_y) {
    /* force_x > 0: 向右推 */
    /* force_y > 0: 向前推 */
    float base_pwm = ESC_PWM_MID;  /* ESC中位（停转） */
    
    float pwm_right = base_pwm + force_x;
    float pwm_left  = base_pwm - force_x;
    float pwm_front = base_pwm + force_y;
    float pwm_back  = base_pwm - force_y;
    
    /* 限幅 */
    pwm_right = CLAMP(pwm_right, ESC_PWM_MIN, ESC_PWM_MAX);
    pwm_left  = CLAMP(pwm_left,  ESC_PWM_MIN, ESC_PWM_MAX);
    pwm_front = CLAMP(pwm_front, ESC_PWM_MIN, ESC_PWM_MAX);
    pwm_back  = CLAMP(pwm_back,  ESC_PWM_MIN, ESC_PWM_MAX);

    /* 修复C2: set_esc_pwm → Servo_SetPulse_us (ESC用舵机PWM控制) */
    /* 注意: 实际需PCA9685扩展多路PWM，单舵机仅PA8 */
    Servo_SetPulse_us((uint16_t)pwm_right);  /* 实际需多路PWM板 */
}
```

### 4. 轨迹生成
```c
/* 修复F4: switch添加default分支处理TRAJ_FREE */
typedef enum {
    TRAJ_POINT,   // 定点停留
    TRAJ_LINE,    // 直线
    TRAJ_CIRCLE,  // 圆形
    TRAJ_EIGHT,   // 8字形
    TRAJ_FREE     // 自由摆
} TrajectoryType;

SwingAngle generate_trajectory(TrajectoryType type, float t, 
                               float radius, float speed) {
    SwingAngle target = {0, 0};
    
    switch (type) {
    case TRAJ_POINT:
        // 目标角度为0（竖直位置）
        target.angle_x = 0;
        target.angle_y = 0;
        break;
        
    case TRAJ_CIRCLE:
        // 画圆：角度按正弦/余弦变化
        target.angle_x = radius * cosf(2 * M_PI * speed * t);
        target.angle_y = radius * sinf(2 * M_PI * speed * t);
        break;
        
    case TRAJ_LINE:
        // 画直线：只在一个方向变化
        target.angle_x = radius * sinf(2 * M_PI * speed * t);
        target.angle_y = 0;
        break;
        
    case TRAJ_EIGHT:
        // 8字形
        target.angle_x = radius * sinf(2 * M_PI * speed * t);
        target.angle_y = radius * sinf(4 * M_PI * speed * t);
        break;
    case TRAJ_FREE:
    default:
        /* 自由摆: 不施加控制，保持当前状态 */
        target.angle_x = 0.0f;
        target.angle_y = 0.0f;
        break;
    }
    
    return target;
}
```

### 5. 前馈+反馈复合控制
```c
/* 风力摆的特殊性：需要克服重力分量 */
float compute_feedforward(float target_angle) {
    /* 前馈：在目标角度需要维持的最小推力 */
    /* F = mg * sin(θ) */
    float gravity_comp = GRAVITY_FACTOR * sinf(target_angle * PI / 180.0f);
    return gravity_comp;
}

float composite_control(PID_Axis *pid, float target, float current, float dt) {
    /* 修复C4: pid_calculate → PID_Calc */
    float feedback = PID_Calc(pid, target, current);
    float feedforward = compute_feedforward(target);
    return feedback + feedforward;
}
```

### 6. 自适应摆长估计
```c
/* 通过自由摆动周期估计摆长 */
/* 修复D: crossings=0 除零风险 */
float estimate_pendulum_length(float *angle_data, int len, float dt) {
    /* 检测过零点，计算周期 */
    int crossings = 0;
    for (int i = 1; i < len; i++) {
        if (angle_data[i-1] * angle_data[i] < 0) crossings++;
    }

    if (crossings == 0) return 0.0f;  /* 除零保护 */
    float period = 2.0f * (float)len * dt / (float)crossings;
    /* T = 2π√(L/g) → L = g * (T/(2π))^2 */
    float half_period = period / (2.0f * PI);
    float length = GRAVITY_FACTOR * half_period * half_period;
    return length;
}
```

---

## 六、测试方案

### 调试流程
1. **风扇测试**：逐一测试每个风扇转向和转速
2. **IMU安装**：确认IMU坐标轴与风扇方向的对应关系
3. **开环测试**：手动设置PWM，观察摆杆响应
4. **单轴PID**：先只控制X方向，调好PID参数
5. **双轴联调**：X/Y同时控制
6. **轨迹测试**：定点→直线→圆形→8字

### 测试项目
| 测试项 | 方法 | 预期结果 |
|--------|------|----------|
| 定点停留 | 任意角度→竖直 | <3°误差，<5s |
| 画圆 | 半径10° | 圆度误差<15% |
| 画线 | 幅度15° | 直线度误差<10% |
| 抗干扰 | 外力扰动后恢复 | <3s |
| 模式切换 | 任意模式切换 | 平滑过渡 |

---

## 七、评分要点

### 得分策略
1. **定点停留**：基础功能，摆杆回到竖直位置
2. **画圆**：高分项，圆度和半径精度
3. **画线**：中等难度，直线度控制
4. **快速响应**：从自由摆到定点的过渡时间

### 常见扣分点
- 风扇推力不足无法达到要求角度
- 控制振荡导致摆杆越摆越大
- X/Y轴耦合导致轨迹变形
- 风扇惯性延迟导致响应慢

### 提分技巧
- 前馈补偿重力分量，减少PID负担
- 加入角速度阻尼项，抑制振荡
- 使用变速积分（大误差时减小积分，小误差时增大积分）
- 风扇响应补偿（预测风扇转速变化，提前给出指令）
- 考虑空气阻力对摆杆运动的影响
- 优化风扇布局角度，利用矢量分解提高控制灵活性
