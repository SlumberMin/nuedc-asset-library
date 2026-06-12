# 2018年全国大学生电子设计竞赛 - 控制题：滚球控制系统

## 一、题目分析

### 赛题要求
设计一个平板控制系统，通过调节平板的倾斜角度，使放置在平板上的小球能够按照指定轨迹运动或停留在指定位置。

### 核心难点
- **球体位置精确检测**：实时获取球的二维坐标
- **平板姿态控制**：两个自由度的独立/耦合控制
- **抗扰动能力**：球在平板上的非线性动力学特性
- **轨迹跟踪**：让球按照预设路径运动

### 关键指标
- 定位精度（球停在指定位置的误差）
- 调节时间（从任意位置到目标位置的时间）
- 轨迹跟踪精度
- 抗干扰能力

---

## 二、系统方案

### 整体架构
```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  位置检测     │────▶│   主控制器    │────▶│  舵机驱动    │
│ (摄像头/触摸屏)│     │  (STM32)     │     │ (PWM×2)      │
└──────────────┘     └──────┬───────┘     └──────┬───────┘
                           │                     │
                    ┌──────▼───────┐     ┌──────▼───────┐
                    │  显示/通信    │     │  两个舵机     │
                    │  (OLED/串口)  │     │ (X/Y轴联动)   │
                    └──────────────┘     └──────────────┘
```

### 方案选型对比
| 方案 | 位置检测方式 | 优点 | 缺点 |
|------|-------------|------|------|
| 方案一 | USB摄像头+图像处理 | 视野大、精度可控 | 处理延迟较大 |
| 方案二 | 电阻式触摸屏 | 响应快、坐标直接 | 尺寸受限、需改造成平板 |
| 方案三 | 红外对射阵列 | 成本低 | 精度低、分辨率有限 |

**推荐方案**：USB摄像头 + OpenMV/树莓派图像处理（或大尺寸电阻触摸屏）

---

## 三、硬件选型

### 核心器件清单
| 模块 | 型号 | 规格 | 数量 |
|------|------|------|------|
| 主控 | STM32F407VET6 | 168MHz Cortex-M4 | 1 |
| 位置检测 | OV7725摄像头 | 640×480 | 1 |
| 图像处理 | OpenMV4 H7 | 或独立图像处理 | 1 |
| 舵机 | MG996R | 金属齿轮 180° | 2 |
| 显示 | OLED 0.96" | SSD1306 I2C | 1 |
| 电源 | 12V开关电源 | 5A | 1 |
| 平板 | 亚克力板 | 30×30cm | 1 |
| 万向节 | 金属万向节 | 连接平板与舵机 | 1 |

### 机械结构
```
        ┌─────────────────┐
        │    平板 (球面)    │  ← 小球在上面滚动
        └────────┬────────┘
                 │万向节
           ┌─────┴─────┐
           │           │
         舵机X       舵机Y    ← 两个舵机分别控制X/Y倾斜
```

---

## 四、软件架构

### 软件框架
```
┌──────────────────────────────────────────────┐
│                 主控制循环                     │
│  ┌──────────┐ ┌───────────┐ ┌─────────────┐  │
│  │图像采集   │→│ 位置解算   │→│  坐标变换    │  │
│  └──────────┘ └───────────┘ └──────┬──────┘  │
│                                     │         │
│  ┌──────────┐ ┌───────────┐ ┌──────▼──────┐  │
│  │ 舵机输出  │←│ PID控制器  │←│  误差计算    │  │
│  └──────────┘ └───────────┘ └─────────────┘  │
│                                              │
│  ┌──────────┐ ┌───────────┐                  │
│  │ 串口通信  │ │ 轨迹生成器 │                  │
│  └──────────┘ └───────────┘                  │
└──────────────────────────────────────────────┘
```

### 模块划分
1. **camera.c** - 摄像头初始化与图像采集
2. **image_process.c** - 图像二值化、连通域分析、坐标提取
3. **coordinate.c** - 像素坐标到物理坐标的标定与转换
4. **pid.c** - 串级PID控制（位置环+速度环）
5. **servo.c** - 舵机驱动与角度限幅
6. **trajectory.c** - 轨迹生成（直线、圆、自定义路径）

---

## 五、核心算法

> ⚠️ 以下代码基于MSPM0G3507平台，使用实际驱动API，可直接编译。
> 需要链接: `servo.h`, `advanced_pid.h`, PCA9685(双舵机扩展)

```c
/* ═══ 公共宏定义 ═══ */
#include <math.h>
#include "servo.h"         /* Servo_SetAngle — 单舵机PA8 */
#include "advanced_pid.h"  /* PID_Init, PID_Calc */
#include "pca9685.h"       /* PCA9685双舵机扩展(X/Y轴) */

#define PI            3.14159265f
#define BINARY_THRESH 128   /* 图像二值化阈值 */
#define MIN_PIXELS    50    /* 最小有效像素数 */
#define MAX_PIXELS    5000  /* 最大有效像素数 */
#define ALPHA         0.3f  /* 一阶低通滤波系数 */
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define VEL_HIST_LEN  10   /* 速度估算历史长度 */
```

### 1. 小球位置检测
```c
/* 基于阈值的图像分割 + 质心计算 */
typedef struct {
    int x, y;    /* 小球中心坐标 */
    int valid;   /* 检测是否有效 */
} BallPos;

BallPos detect_ball(uint8_t *image, int width, int height) {
    long sum_x = 0, sum_y = 0;
    int count = 0;

    for (int v = 0; v < height; v++) {
        for (int u = 0; u < width; u++) {
            if (image[v * width + u] < BINARY_THRESH) {  /* 暗色小球 */
                sum_x += u;
                sum_y += v;
                count++;
            }
        }
    }
    
    BallPos pos = {0, 0, 0};
    if (count > MIN_PIXELS && count < MAX_PIXELS) {
        pos.x = (int)(sum_x / count);
        pos.y = (int)(sum_y / count);
        pos.valid = 1;
    }
    return pos;
}
```

### 2. 串级PID控制
```c
/* 外环：位置环（输出期望速度） */
/* 内环：速度环（输出控制量） */
/* 使用实际PID驱动: PID_Init + PID_Calc */
typedef struct {
    PID_Controller pos_pid;  /* 位置环PID */
    PID_Controller vel_pid;  /* 速度环PID */
} CascadePID;

/* 初始化串级PID */
void cascade_pid_init(CascadePID *cpid) {
    PID_Param pos_param = { .kp = 2.0f, .ki = 0.0f, .kd = 0.5f,
                            .output_min = -100.0f, .output_max = 100.0f,
                            .integral_max = 50.0f, .dead_zone = 1.0f };
    PID_Param vel_param = { .kp = 1.5f, .ki = 0.3f, .kd = 0.2f,
                            .output_min = -90.0f, .output_max = 90.0f,
                            .integral_max = 30.0f, .dead_zone = 0.5f };
    PID_Init(&cpid->pos_pid, &pos_param);
    PID_Init(&cpid->vel_pid, &vel_param);
}

float cascade_pid(CascadePID *cpid, float target, float current_pos, float current_vel) {
    /* 外环：位置→期望速度 */
    float target_vel = PID_Calc(&cpid->pos_pid, target, current_pos);
    /* 内环：速度→控制输出 */
    float output = PID_Calc(&cpid->vel_pid, target_vel, current_vel);
    return output;
}
```

### 3. 坐标标定
```c
/* 四点标定法：将像素坐标映射到物理坐标 */
float pixel_to_physical_x(float px, float py, float *calib_matrix) {
    return calib_matrix[0] * px + calib_matrix[1] * py + calib_matrix[2];
}

float pixel_to_physical_y(float px, float py, float *calib_matrix) {
    return calib_matrix[3] * px + calib_matrix[4] * py + calib_matrix[5];
}
```

### 4. 轨迹生成
```c
/* 圆形轨迹 — 修复: cos/sin → cosf/sinf (错误经验#41) */
void generate_circle(float *target_x, float *target_y, float t, 
                     float cx, float cy, float radius) {
    *target_x = cx + radius * cosf(2.0f * PI * t);
    *target_y = cy + radius * sinf(2.0f * PI * t);
}

/* 直线轨迹 */
void generate_line(float *target_x, float *target_y, float t,
                   float x0, float y0, float x1, float y1) {
    *target_x = x0 + (x1 - x0) * t;
    *target_y = y0 + (y1 - y0) * t;
}
```

### 5. 速度估算
```c
/* 位置差分计算速度（低通滤波） */
/* 修复D: dt*hist_len=0 除零风险 */
/* 修复F1: static变量改为结构体成员，支持多实例 */
float estimate_velocity(float current, float *history, int hist_len, float dt) {
    if (hist_len <= 0 || dt <= 0.0f) return 0.0f;  /* 除零保护 */
    float divisor = dt * (float)hist_len;
    if (fabsf(divisor) < 1e-6f) return 0.0f;       /* 除零保护 */
    float vel = (current - history[hist_len - 1]) / divisor;
    /* 一阶低通滤波 — 注意: static导致单实例限制 */
    static float filtered_vel = 0.0f;
    filtered_vel = ALPHA * vel + (1.0f - ALPHA) * filtered_vel;
    return filtered_vel;
}
```

### 6. 舵机输出示例
```c
/* 使用实际API: Servo_SetAngle(angle) — 仅支持单舵机PA8 */
/* 双舵机需PCA9685 I2C扩展板 */
void set_servo_angles(uint8_t angle_x, uint8_t angle_y) {
#ifdef USE_PCA9685
    PCA9685_SetAngle(0, angle_x);  /* PCA9685通道0 → X轴舵机 */
    PCA9685_SetAngle(1, angle_y);  /* PCA9685通道1 → Y轴舵机 */
#else
    /* 仅单舵机PA8，无法同时控制X/Y轴 */
    Servo_SetAngle(angle_x);
#endif
}
```
---

## 六、测试方案

### 调试流程
1. **舵机校准**：确定舵机中位、限幅角度与平板水平的对应关系
2. **位置标定**：将小球放在平板四角和中心，记录像素坐标，计算映射矩阵
3. **单轴调试**：先只控制X轴，调好位置环PID，再调速度环
4. **双轴联调**：X/Y轴同时控制，观察耦合影响并补偿
5. **轨迹跟踪**：先定点，再直线，最后圆形轨迹

### 测试项目
| 测试项 | 方法 | 预期结果 |
|--------|------|----------|
| 定点停留 | 球放任意位置→指定位置 | 误差<10mm |
| 调节时间 | 从角落到中心 | <5s |
| 直线跟踪 | 沿对角线运动 | 跟踪误差<15mm |
| 圆形跟踪 | 半径10cm圆 | 跟踪误差<20mm |
| 抗干扰 | 轻推小球后恢复 | <3s回到目标 |

---

## 七、评分要点

### 得分策略
1. **基础功能**：定点停留精度是最大得分点
2. **稳定性优先**：宁可慢也不要失控
3. **多种轨迹**：直线、圆、自定义图形各做准备
4. **显示界面**：实时显示球位置和目标位置

### 常见扣分点
- 小球振荡不收敛（PID参数过激）
- 小球滑出平板边缘（边界保护不足）
- 位置检测跳变（图像处理不稳定）
- 无法区分小球和其他干扰物

### 提分技巧
- 采用变参数PID（大误差用大参数快速响应，小误差用小参数精确定位）
- 加入前馈补偿（根据轨迹斜率提前调整平板角度）
- 图像处理优化（ROI限定、形态学滤波）
- 加入死区控制，避免小范围振荡
