# 2019年全国大学生电子设计竞赛 - 控制题：巡线机器人

## 一、题目分析

### 赛题要求
设计一个巡线机器人，能够在指定路径（黑线/白线）上自主循迹行驶，完成规定的巡线任务。

### 核心难点
- **路径识别精度**：在不同光照、地面条件下稳定识别线路
- **高速巡线稳定性**：速度与循迹精度的平衡
- **弯道处理**：急弯、直角弯、S弯等多种弯道类型的平滑过渡
- **起停控制**：精确定位起止点，按规定路线行驶

### 关键指标
- 巡线速度要求（越快越好或规定时间内完成）
- 偏离线路惩罚（脱线扣分）
- 路径类型覆盖（直道、弯道、交叉路口等）

---

## 二、系统方案

### 整体架构
```
┌─────────────┐     ┌──────────────┐     ┌──────────────┐
│  路径传感器  │────▶│   主控制器    │────▶│  电机驱动    │
│ (红外/光电)  │     │  (STM32)     │     │ (TB6612/L298N)│
└─────────────┘     └──────┬───────┘     └──────┬───────┘
                           │                     │
                    ┌──────▼───────┐     ┌──────▼───────┐
                    │  编码器反馈   │     │   直流电机    │
                    │  (速度环)     │     │  (左右差速)   │
                    └──────────────┘     └──────────────┘
```

### 方案选型对比
| 方案 | 传感器类型 | 优点 | 缺点 |
|------|-----------|------|------|
| 方案一 | 红外对管阵列 | 成本低、响应快 | 探测距离短、受环境光影响 |
| 方案二 | CCD摄像头 | 视野宽、信息量大 | 处理复杂、延迟较大 |
| 方案三 | 激光传感器 | 精度高、抗干扰强 | 成本较高 |

**推荐方案**：红外对管阵列 + 编码器闭环控制

---

## 三、硬件选型

### 核心器件清单
| 模块 | 型号 | 规格 | 数量 |
|------|------|------|------|
| 主控 | STM32F103C8T6 | 72MHz Cortex-M3 | 1 |
| 循迹传感器 | TCRT5000 | 红外反射式 | 8~16路 |
| 电机驱动 | TB6612FNG | 双路H桥 | 1 |
| 直流电机 | JGA25-370 | 带编码器 12V | 2 |
| 编码器 | 增量式光电编码器 | 13PPR | 2 |
| 电源 | 18650锂电池 | 7.4V 2S | 1组 |
| 稳压 | AMS1117-3.3 | 3.3V | 1 |
| 底盘 | 四轮/两驱 | 亚克力 | 1 |

### 传感器布局
```
      □ □ □ □ □ □ □ □
      7 6 5 4 3 2 1 0
              ↑
           黑线位置
```
- 8路红外传感器，间距10mm
- 安装高度距地面5~10mm
- 中间4路用于直线检测，外侧用于弯道预判

---

## 四、软件架构

### 软件框架
```
┌─────────────────────────────────────────┐
│              主控制循环 (main loop)       │
│  ┌─────────┐ ┌──────────┐ ┌──────────┐  │
│  │传感器采集│→│ 巡线算法  │→│ 电机输出  │  │
│  └─────────┘ └──────────┘ └──────────┘  │
│       ↑           ↑                      │
│  ┌─────────┐ ┌──────────┐               │
│  │定时中断  │ │ 编码器   │               │
│  │(10ms)   │ │ 速度计算  │               │
│  └─────────┘ └──────────┘               │
└─────────────────────────────────────────┘
```

### 模块划分
1. **sensor.c** - 传感器数据采集与滤波
2. **line_detect.c** - 路径识别与位置计算
3. **motor.c** - 电机驱动与PWM输出
4. **pid.c** - PID控制算法
5. **main.c** - 主控制逻辑与状态机

---

## 五、核心算法

> ⚠️ 以下代码基于MSPM0G3507平台，使用实际驱动API，可直接编译。
> 需要链接: `tb6612.h`, `encoder.h`, `advanced_pid.h`

```c
/* ═══ 公共宏定义 ═══ */
#include <math.h>
#include <string.h>
#include "tb6612.h"
#include "encoder.h"
#include "advanced_pid.h"

#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define HIGH_SPEED    3000
#define MEDIUM_SPEED  2000
#define LOW_SPEED     1000
#define SENSOR_COUNT  8
#define MEDIAN_BUF_MAX  16  /* 中值滤波最大通道数，避免VLA */
```

### 1. 位置偏差计算
```c
/* 加权平均法计算偏差 — 修复: last_error改用static局部变量 */
int calculate_error(int *sensor_values, int num_sensors) {
    long weighted_sum = 0;
    int active_count = 0;
    static int last_error = 0;  /* 修复A2: 原代码last_error未声明 */

    for (int i = 0; i < num_sensors; i++) {
        if (sensor_values[i]) {  /* 检测到黑线 */
            weighted_sum += (i - num_sensors / 2) * 1000;
            active_count++;
        }
    }
    if (active_count == 0) return last_error;  /* 保持上次偏差 */
    last_error = (int)(weighted_sum / active_count);
    return last_error;
}
```

### 2. PD巡线控制
```c
typedef struct {
    float Kp;       /* 比例系数 */
    float Kd;       /* 微分系数 */
    int base_speed; /* 基础速度 */
    int last_error;
} LineTracker;

/* 修复A1: 返回类型改为void (原声明int但无return) */
/* 修复C1: set_motors → TB6612_SetMotor */
void line_pid(LineTracker *tracker, int error) {
    int derivative = error - tracker->last_error;
    int correction = (int)(tracker->Kp * error + tracker->Kd * derivative);
    tracker->last_error = error;

    int left_speed  = CLAMP(tracker->base_speed + correction, 0, 3999);
    int right_speed = CLAMP(tracker->base_speed - correction, 0, 3999);

    /* 使用实际驱动API: TB6612_SetMotor(ch, dir, speed) */
    TB6612_SetMotor(MOTOR_CH_A, MOTOR_DIR_FORWARD, (uint32_t)left_speed);
    TB6612_SetMotor(MOTOR_CH_B, MOTOR_DIR_FORWARD, (uint32_t)right_speed);
}
```

### 3. 差速转弯策略
```c
/* 根据弯道程度调整速度 */
static int base_speed = MEDIUM_SPEED;  /* 修复: 需声明 */

void adaptive_speed(int error) {
    int abs_error = abs(error);  /* abs()在stdlib.h中 */
    if (abs_error < 500) {
        base_speed = HIGH_SPEED;    /* 直道高速 */
    } else if (abs_error < 2000) {
        base_speed = MEDIUM_SPEED;  /* 小弯中速 */
    } else {
        base_speed = LOW_SPEED;     /* 急弯低速 */
    }
}
```

### 4. 滤波算法
```c
/* 中值滤波去除毛刺 — 修复F3: VLA改为固定大小数组 */
#include <stdlib.h>

static void _swap_int(int *a, int *b) { int t = *a; *a = t; *b = t; } /* 修复: swap未定义 */

int median_filter(int *buf, int len) {
    if (len <= 0 || len > MEDIAN_BUF_MAX) return 0;  /* 边界保护 */
    int temp[MEDIAN_BUF_MAX];  /* 修复F3: 固定大小，避免VLA栈溢出 */
    memcpy(temp, buf, len * sizeof(int));
    /* 简单冒泡排序取中值 */
    for (int i = 0; i < len - 1; i++)
        for (int j = 0; j < len - i - 1; j++)
            if (temp[j] > temp[j + 1]) {
                int t = temp[j]; temp[j] = temp[j + 1]; temp[j + 1] = t;
            }
    return temp[len / 2];
}

/* ═══ 编码器速度读取示例 ═══ */
/* 使用实际API: Encoder_GetSpeed(EncoderChannel ch) */
void read_wheel_speeds(int32_t *left, int32_t *right) {
    *left  = Encoder_GetSpeed(ENC_LEFT);
    *right = Encoder_GetSpeed(ENC_RIGHT);
}
```

---

## 六、测试方案

### 调试流程
1. **传感器标定**：分别在白色和黑色表面采集ADC值，确定阈值
2. **直道调试**：先调Kp使机器人在直道上平稳行驶，再加Kd抑制振荡
3. **弯道调试**：在各种弯道上测试，调整参数适应不同曲率
4. **速度优化**：逐步提高基础速度，找到速度与稳定性的平衡点

### 测试项目
| 测试项 | 方法 | 预期结果 |
|--------|------|----------|
| 直线巡线 | 2m直道 | 偏差<5mm |
| S弯巡线 | 标准S弯 | 不脱线 |
| 急转弯 | 90°弯道 | 平滑通过 |
| 长时间运行 | 连续5圈 | 无累积误差 |
| 不同地面 | 瓷砖/木板 | 自适应调节 |

---

## 七、评分要点

### 得分策略
1. **基础功能优先**：确保稳定巡线不脱线（占分最多）
2. **速度优化**：在稳定性基础上提升速度
3. **适应性**：适应不同光照和地面条件
4. **鲁棒性**：长时间运行不退化

### 常见扣分点
- 频繁脱线（传感器布局不合理）
- 弯道速度过快冲出赛道
- 起停定位不准
- 传感器间距过大导致检测盲区

### 提分技巧
- 传感器数量适当增加（10~16路）提高分辨率
- 采用变速策略，直道快弯道慢
- 加入预测算法，提前预判弯道
- 使用编码器闭环提高速度稳定性
