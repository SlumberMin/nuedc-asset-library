# 2017年全国大学生电子设计竞赛 - 控制题：电磁曲射炮

## 一、题目分析

### 赛题要求
设计一个电磁曲射炮系统，能够将小铁球发射到指定距离和高度的目标区域，通过控制电磁线圈的充电量和发射角度来实现精确投射。

### 核心难点
- **充电量精确控制**：电容储能与发射初速度的关系（非线性）
- **发射角度控制**：步进电机或舵机精确调节仰角
- **弹道计算**：抛体运动与空气阻力
- **一致性**：多次发射的重复精度

### 关键指标
- 发射距离精度（目标距离±误差范围）
- 命中率（多次发射的统计命中率）
- 发射角度范围
- 充电/发射周期

---

## 二、系统方案

### 整体架构
```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  参数输入     │────▶│   主控制器    │────▶│  充电控制    │
│ (目标距离/角度)│     │  (STM32)     │     │ (升压+电容)   │
└──────────────┘     └──────┬───────┘     └──────┬───────┘
                           │                     │
                    ┌──────▼───────┐     ┌──────▼───────┐
                    │  角度控制     │     │  电磁线圈    │
                    │ (步进电机)    │     │ (放电开关)    │
                    └──────────────┘     └──────────────┘
```

### 方案选型对比
| 方案 | 储能方式 | 优点 | 缺点 |
|------|---------|------|------|
| 方案一 | 大电容直接充电 | 简单可靠 | 充电慢、能量控制粗 |
| 方案二 | 升压模块+电容 | 充电快、可控 | 电路复杂 |
| 方案三 | 多级电容切换 | 多档位精确 | 继电器切换延迟 |

**推荐方案**：升压模块充电 + 电压检测 + 可控硅/IGBT放电

---

## 三、硬件选型

### 核心器件清单
| 模块 | 型号 | 规格 | 数量 |
|------|------|------|------|
| 主控 | STM32F103C8T6 | 72MHz Cortex-M3 | 1 |
| 升压模块 | XL6009 | 5V→300V可调 | 1 |
| 储能电容 | 电解电容 | 450V 1000μF | 2~4 |
| 放电开关 | IGBT模块 | IKW40N120H3 | 1 |
| 角度控制 | 42步进电机 | 1.8° + 细分 | 1 |
| 步进驱动 | A4988 | 1/16细分 | 1 |
| 电压检测 | 分压电阻+ADC | 0-300V→0-3.3V | 1 |
| 电磁线圈 | 漆包线绕制 | 0.5mm 200匝 | 1 |
| 显示 | LCD1602/0.96"OLED | - | 1 |

### 电磁炮结构
```
         线圈（炮管尾部）
            ┌──┐
     ───────┤  ├─────── ← 炮管（PVC管）
            └──┘
              │
        ┌─────┴─────┐
        │   储能电容  │
        └─────┬─────┘
              │
        ┌─────┴─────┐
        │   升压电路  │
        └───────────┘
              │
         ═══════════  ← 旋转轴（步进电机控制仰角）
```

---

## 四、软件架构

### 软件框架
```
┌──────────────────────────────────────────────┐
│              主控制状态机                      │
│                                              │
│  IDLE → CHARGING → READY → AIMING → FIRE    │
│   ↑                              │           │
│   └──────────────────────────────┘           │
│                                              │
│  ┌──────────┐ ┌───────────┐ ┌─────────────┐ │
│  │ 充电控制  │ │ 角度控制   │ │ 弹道计算     │ │
│  └──────────┘ └───────────┘ └─────────────┘ │
│  ┌──────────┐ ┌───────────┐                  │
│  │ 按键输入  │ │ 显示更新   │                  │
│  └──────────┘ └───────────┘                  │
└──────────────────────────────────────────────┘
```

### 模块划分
1. **boost.c** - 升压充电控制与电压检测
2. **cap_charge.c** - 电容充电状态机
3. **fire.c** - 放电发射控制
4. **angle.c** - 步进电机角度控制
5. **ballistic.c** - 弹道计算与参数映射
6. **ui.c** - 用户界面与参数设置

---

## 五、核心算法

### 1. 弹道计算（抛体运动）

> ⚠️ 以下代码基于MSPM0G3507平台，算法伪代码，实际硬件操作需根据具体电路实现。

```c
#include <math.h>

#define PI  3.14159265f
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define TABLE_SIZE  6  /* 标定表条目数 */

/* 已知：发射高度h0，目标距离d，发射角度θ
 * 求：所需初速度v0
 * 公式：d = v0*cos(θ)*t
 *       h0 + v0*sin(θ)*t - 0.5*g*t² = 0 */
typedef struct {
    float angle;       // 发射角度(度)
    float velocity;    // 初速度(m/s)
    float charge_volt; // 充电电压(V)
} FireParams;

/* 修复#41: cos/tan → cosf/tanf */
/* 修复: v_sq可能为负(角度过大或距离过远)，需检查 */
float calc_velocity(float distance, float angle_deg, float height) {
    float theta = angle_deg * PI / 180.0f;
    float g = 9.8f;

    float cos_t = cosf(theta);
    float denom = 2.0f * cos_t * cos_t * (height + distance * tanf(theta));
    if (fabsf(denom) < 1e-6f) return 0.0f;  /* 除零保护 */

    float v_sq = g * distance * distance / denom;
    if (v_sq < 0.0f) return 0.0f;  /* 物理上无解 */
    return sqrtf(v_sq);
}
```

### 2. 充电电压与初速度映射
```c
/* 通过标定实验建立 V_charge → v0 的映射表 */
/* 修复: TABLE_SIZE已定义; 除零保护(标定间距) */
typedef struct {
    float voltage;    // 充电电压
    float velocity;   // 实测初速度
} CalibPoint;

CalibPoint calib_table[] = {
    {50.0f,  1.2f},
    {100.0f, 2.5f},
    {150.0f, 3.8f},
    {200.0f, 5.0f},
    {250.0f, 6.2f},
    {300.0f, 7.3f},
};

float velocity_to_voltage(float target_vel) {
    for (int i = 0; i < TABLE_SIZE - 1; i++) {
        if (target_vel >= calib_table[i].velocity && 
            target_vel <= calib_table[i+1].velocity) {
            float dv = calib_table[i+1].velocity - calib_table[i].velocity;
            if (fabsf(dv) < 1e-6f) return calib_table[i].voltage;  /* 除零保护 */
            float ratio = (target_vel - calib_table[i].velocity) / dv;
            return calib_table[i].voltage + ratio * 
                   (calib_table[i+1].voltage - calib_table[i].voltage);
        }
    }
    return calib_table[TABLE_SIZE - 1].voltage;  /* 超范围取最大值 */
}
```

### 3. 充电控制
```c
#define TARGET_VOLTAGE_TOLERANCE 2.0f  // ±2V

typedef enum { CHG_IDLE, CHG_BOOSTING, CHG_DONE } ChargeState;

ChargeState charge_control(float target_v) {
    float current_v = read_cap_voltage();
    
    if (current_v < target_v - TARGET_VOLTAGE_TOLERANCE) {
        enable_boost(1);   // 开启升压
        return CHG_BOOSTING;
    } else if (current_v >= target_v) {
        enable_boost(0);   // 关闭升压
        return CHG_DONE;
    }
    return CHG_BOOSTING;
}
```

### 4. 步进电机角度控制
```c
/* 1.8°步进电机，16细分 → 每步 0.1125° */
/* 注意: step_forward/step_backward/delay_us 需根据实际步进驱动芯片(A4988等)实现 */
#define STEPS_PER_DEG  (200.0f * 16.0f / 360.0f)  /* ≈ 8.89步/度 */

void set_angle(float target_deg) {
    static float current_deg = 0;
    int steps = (int)((target_deg - current_deg) * STEPS_PER_DEG);

    if (steps > 0) {
        for (int i = 0; i < steps; i++) {
            /* step_forward();  — 需实现: A4988 STEP脉冲 */
            /* delay_us(500);   — 需实现: HAL_Delay_us 或 定时器 */
        }
    } else {
        for (int i = 0; i < -steps; i++) {
            /* step_backward(); — 需实现: A4988 DIR反向+STEP脉冲 */
        }
    }
    current_deg = target_deg;
}
```

### 5. 发射时序控制
```c
/* 注意: 以下函数需根据实际硬件电路实现 */
/* enable_boost(), read_cap_voltage(), fire_igbt_pulse(), discharge_safety() */
/* delay_ms() 需使用 HAL_Delay() 或等效实现 */

void fire_sequence(float charge_volt, float angle) {
    /* 1. 调整角度 */
    set_angle(angle);
    /* delay_ms(500); — 需实现: HAL_Delay(500) */

    /* 2. 充电 */
    while (charge_control(charge_volt) != CHG_DONE) {
        /* delay_ms(10); — 需实现 */
    }
    /* delay_ms(200); — 充电稳定等待 */

    /* 3. 放电发射 */
    /* fire_igbt_pulse(10); — 需实现: GPIO脉冲驱动IGBT */

    /* 4. 安全放电（残余电荷泄放） */
    /* discharge_safety(); — 需实现: 泄放电阻电路 */
}
```

---

## 六、测试方案

### 调试流程
1. **安全第一**：高压电路调试必须佩戴绝缘手套，先用低电压测试
2. **升压模块调试**：逐步调节输出电压，验证ADC检测准确性
3. **单次发射测试**：固定角度，不同充电电压，记录发射距离
4. **标定实验**：建立充电电压-发射距离映射表
5. **角度测试**：固定充电量，不同角度，验证弹道计算
6. **综合测试**：输入目标距离，自动计算参数并发射

### 测试项目
| 测试项 | 方法 | 预期结果 |
|--------|------|----------|
| 充电精度 | 目标200V | 实际200±2V |
| 发射距离1 | 固定距离2m | 误差<10cm |
| 发射距离2 | 固定距离3m | 误差<15cm |
| 角度精度 | 目标45° | 实际45±0.5° |
| 连续发射 | 5次同参数 | 标准差<5cm |

---

## 七、评分要点

### 得分策略
1. **基础发射功能**：能将球发射出去并落入目标区域
2. **距离精度**：多次发射平均误差最小化
3. **快速响应**：缩短充电和瞄准时间
4. **安全性**：电路保护完善，不会烧毁器件

### 常见扣分点
- 高压电路短路烧毁（缺乏保护）
- 发射不一致（电容残余电荷、线圈温度影响）
- 弹道计算偏差大（未考虑炮管长度、摩擦等）
- 充电速度过慢超时

### 提分技巧
- 多次标定取平均，建立精确的查表+插值映射
- 加入温度补偿（线圈电阻随温度变化）
- 考虑炮管内加速距离对初速度的影响
- 设计快速充电策略（多段充电）
- 使用光电传感器测速反馈校准
