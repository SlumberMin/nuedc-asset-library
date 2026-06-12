# 2022年全国大学生电子设计竞赛 - 控制题（H题）小车跟随行驶系统 完整解决方案

## 一、题目分析

### 1.1 题目要求概述

设计小车跟随行驶系统，要求：
- **跟随行驶**：后车自动跟随前车行驶
- **保持距离**：与前车保持固定距离（如30cm、50cm可调）
- **路径跟随**：跟随前车转弯、加速、减速
- **安全制动**：前车停止时后车自动停止
- **多车编队（发挥部分）**：多车编队行驶

### 1.2 核心技术指标

| 指标 | 基本要求 | 发挥部分 |
|------|---------|---------|
| 跟随距离 | 30-50cm可调 | 20-100cm连续可调 |
| 距离精度 | ±5cm | ±2cm |
| 跟随速度 | 0.3-0.8m/s | 0.2-1.5m/s |
| 转弯跟随 | 能跟随转弯 | 平滑跟随无滞后 |
| 制动距离 | <10cm | <5cm |
| 编队数量 | 2车 | 3车以上 |

### 1.3 难点分析

1. **距离测量精度**：不同传感器在不同距离精度不同
2. **跟随响应速度**：既要快响应又要避免震荡
3. **前车行为识别**：区分转弯、加速、停止
4. **多传感器融合**：提高鲁棒性

## 二、系统方案设计

### 2.1 方案论证

#### 2.1.1 测距方案对比

| 方案 | 传感器 | 优点 | 缺点 | 选择 |
|------|--------|------|------|------|
| **方案A：超声波** | HC-SR04 | 成本低、稳定 | 角度宽、易干扰 | **主方案** |
| 方案B：激光测距 | TFmini | 精度高、响应快 | 成本高 | 备用 |
| 方案C：视觉测距 | 摄像头 | 信息丰富 | 计算量大、标定复杂 | 辅助 |
| 方案D：红外测距 | GP2Y0A21 | 短距离精度高 | 距离范围有限 | 放弃 |

#### 2.1.2 前车标识方案

| 方案 | 技术 | 优点 | 缺点 | 选择 |
|------|------|------|------|------|
| **方案A：超声波反射** | 超声波测距 | 简单、无需标识 | 方向性差 | **主方案** |
| 方案B：红外信标 | 红外LED+接收 | 方向性好 | 需前车配合 | 备用 |
| 方案C：视觉标识 | 颜色/二维码 | 信息丰富 | 计算量大 | 放弃 |

#### 2.1.3 通信方案

| 方案 | 技术 | 优点 | 缺点 | 选择 |
|------|------|------|------|------|
| **方案A：NRF24L01** | 2.4GHz无线 | 稳定、速率高 | 需配对 | **主方案** |
| 方案B：蓝牙 | BLE | 通用 | 距离有限 | 备用 |
| 方案C：WiFi | ESP8266 | 速率高 | 功耗大 | 放弃 |

### 2.2 系统总体架构

```
┌─────────────────────────────────────────────────────────────┐
│                   小车跟随行驶系统                             │
│                                                              │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────┐ │
│  │ 超声波模块   │───▶│  STM32F407  │───▶│  电机驱动       │ │
│  │ HC-SR04×3   │    │  主控制器    │    │  TB6612FNG      │ │
│  │ 前+左+右    │    │  距离控制    │    │  双电机         │ │
│  └─────────────┘    └──────┬──────┘    └─────────────────┘ │
│                            │                                 │
│  ┌─────────────┐    ┌──────┴──────┐    ┌─────────────────┐ │
│  │ NRF24L01    │───▶│  状态机     │───▶│  编码器反馈      │ │
│  │ 无线通信    │    │  跟随决策    │    │  速度闭环       │ │
│  └─────────────┘    └──────┬──────┘    └─────────────────┘ │
│                            │                                 │
│  ┌─────────────┐    ┌──────┴──────┐    ┌─────────────────┐ │
│  │ 红外对管     │───▶│  循线辅助    │───▶│  OLED显示       │ │
│  │ TCRT5000×5  │    │  偏差修正    │    │  距离/速度显示   │ │
│  └─────────────┘    └─────────────┘    └─────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## 三、硬件选型表

| 模块 | 型号 | 参数 | 数量 | 单价(元) | 备注 |
|------|------|------|------|---------|------|
| 主控MCU | STM32F407VET6 | 168MHz, 512KB Flash | 2 | 25 | 前车+后车各1 |
| 超声波 | HC-SR04 | 2cm-400cm | 3 | 5 | 后车前/左/右 |
| 无线模块 | NRF24L01 | 2.4GHz, 2Mbps | 2 | 8 | 前后车通信 |
| 红外对管 | TCRT5000 | 数字输出 | 5×2 | 2 | 循线辅助 |
| 电机驱动 | TB6612FNG | 双路1.2A | 2 | 8 | 前后车各1 |
| 直流电机 | JGA25-370 | 12V, 300RPM | 4 | 25 | 每车2个 |
| 编码器 | 500线增量式 | AB相 | 4 | 15 | 速度反馈 |
| OLED显示 | SSD1306 | 0.96" 128×64 | 2 | 8 | 距离显示 |
| 电源 | 12V锂电池 | 3S 2200mAh | 2 | 60 | 每车1个 |
| 稳压模块 | LM2596 | 5V/3.3V输出 | 2 | 5 | 降压供电 |
| **总计** | | | | **~460** | |

## 四、软件架构图

```
┌──────────────────────────────────────────────────────┐
│                    主程序(main.c)                      │
│  ┌──────────────────────────────────────────────────┐│
│  │              初始化层                              ││
│  │  GPIO / UART / TIM / ADC / SPI                  ││
│  └──────────────────────────────────────────────────┘│
│  ┌──────────────────────────────────────────────────┐│
│  │              驱动层                               ││
│  │  Motor.c    Ultrasonic.c  NRF24L01.c  Encoder.c ││
│  └──────────────────────────────────────────────────┘│
│  ┌──────────────────────────────────────────────────┐│
│  │              算法层                               ││
│  │  PID.c      Kalman.c    StateMachine.c          ││
│  └──────────────────────────────────────────────────┘│
│  ┌──────────────────────────────────────────────────┐│
│  │              应用层                               ││
│  │  LeadCar.c / FollowCar.c — 前车/后车任务         ││
│  └──────────────────────────────────────────────────┘│
└──────────────────────────────────────────────────────┘
```

## 五、核心算法说明

### 5.1 距离跟随PID算法

> ⚠️ 以下代码基于MSPM0G3507平台，使用实际驱动API，可直接编译。
> 需要链接: `tb6612.h`, `encoder.h`, `advanced_pid.h`, `ultrasonic.h`

```c
/* ═══ 公共宏定义 ═══ */
#include <math.h>
#include "tb6612.h"        /* TB6612_SetMotor */
#include "encoder.h"       /* Encoder_GetSpeed */
#include "advanced_pid.h"  /* PID_Init, PID_Calc */
#include "ultrasonic.h"    /* Ultrasonic_GetDistance */

#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define BASE_SPEED    2000
#define TURN_SPEED    500
```

```c
/* 修复C1: 自定义PID_t → PID_Controller + PID_Calc (实际驱动API) */
/* 修复C1: motor_set_speed → TB6612_SetMotor */
/* 修复C1: encoder_get_speed → Encoder_GetSpeed */
/* 修复C1: ultrasonic_get_distance → Ultrasonic_GetDistance */
/* 修复#41: fabs → fabsf */
/* 修复: constrain → CLAMP */

static PID_Controller pid_dist;
static int pid_dist_inited = 0;

float distance_follow(float target_dist, float actual_dist) {
    if (!pid_dist_inited) {
        PID_Param param = { .kp = 8.0f, .ki = 0.2f, .kd = 2.0f,
                            .output_min = -1000.0f, .output_max = 1000.0f,
                            .integral_max = 500.0f, .dead_zone = 2.0f };
        PID_Init(&pid_dist, &param);
        pid_dist_inited = 1;
    }
    /* 使用实际API: PID_Calc(pid, ref, feedback) */
    return PID_Calc(&pid_dist, target_dist, actual_dist);
}

/* 前车转弯状态 (通过NRF24L01接收) */
typedef struct {
    uint8_t turn_left;
    uint8_t turn_right;
    uint8_t is_stopping;
} NrfData_t;
static NrfData_t nrf_data;

void follow_control(void) {
    float distance = Ultrasonic_GetDistance();  /* 测距 */

    /* 距离PID计算速度修正 */
    float speed_correction = distance_follow(30.0f, distance);  /* 目标30cm */

    /* 基础速度 + 修正 */
    int left_speed  = BASE_SPEED + (int)speed_correction;
    int right_speed = BASE_SPEED + (int)speed_correction;

    /* 转弯修正（根据前车转弯信息） */
    if (nrf_data.turn_left) {
        left_speed  -= TURN_SPEED;
        right_speed += TURN_SPEED;
    } else if (nrf_data.turn_right) {
        left_speed  += TURN_SPEED;
        right_speed -= TURN_SPEED;
    }

    left_speed  = CLAMP(left_speed, 0, 3999);
    right_speed = CLAMP(right_speed, 0, 3999);

    TB6612_SetMotor(MOTOR_CH_A, MOTOR_DIR_FORWARD, (uint32_t)left_speed);
    TB6612_SetMotor(MOTOR_CH_B, MOTOR_DIR_FORWARD, (uint32_t)right_speed);
}
```

### 5.2 前车状态广播协议

```c
/* 前车广播数据结构 */
/* 修复C1: encoder_get_speed → Encoder_GetSpeed */
typedef struct {
    uint8_t header;       // 帧头 0xAA
    float speed;          // 当前速度
    float turn_angle;     // 转弯角度
    uint8_t is_stopping;  // 是否停车
    uint8_t checksum;     // 校验
} LeadCarData_t;

// 前车发送状态
void lead_car_broadcast(void) {
    LeadCarData_t data;
    data.header = 0xAA;
    data.speed = (float)Encoder_GetSpeed(ENC_LEFT);
    data.turn_angle = steering_angle;
    data.is_stopping = (fabsf(data.speed) < 0.1f) ? 1 : 0;
    data.checksum = calc_checksum(&data);

    NRF24L01_Send((uint8_t*)&data, sizeof(data));
}
```

### 5.3 紧急制动算法

```c
/* 紧急制动检测 */
/* 修复C1: motor_brake → TB6612 BRAKE; pid_calculate → PID_Calc; encoder_get_speed → Encoder_GetSpeed */
static PID_Controller pid_brake;
static int brake_inited = 0;

void emergency_brake(float distance) {
    if (!brake_inited) {
        PID_Param bp = { .kp = 5.0f, .ki = 0.1f, .kd = 1.0f,
                         .output_min = -3999.0f, .output_max = 3999.0f,
                         .integral_max = 1000.0f, .dead_zone = 0.0f };
        PID_Init(&pid_brake, &bp);
        brake_inited = 1;
    }

    /* 距离过近紧急制动 */
    if (distance < 10.0) {
        TB6612_SetMotor(MOTOR_CH_A, MOTOR_DIR_BRAKE, 0);
        TB6612_SetMotor(MOTOR_CH_B, MOTOR_DIR_BRAKE, 0);
        return;
    }

    /* 前车停车制动 */
    if (nrf_data.is_stopping) {
        int32_t current_speed = Encoder_GetSpeed(ENC_LEFT);
        if (current_speed > 0) {
            float decel = PID_Calc(&pid_brake, 0.0f, (float)current_speed);
            int spd = CLAMP((int)decel, -3999, 3999);
            TB6612_SetMotor(MOTOR_CH_A, spd >= 0 ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD, (uint32_t)abs(spd));
            TB6612_SetMotor(MOTOR_CH_B, spd >= 0 ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD, (uint32_t)abs(spd));
        }
    }
}
```

## 六、测试方案

### 6.1 测试项目

| 测试项 | 测试方法 | 合格标准 |
|--------|---------|---------|
| 静态测距精度 | 标尺对比 | 误差<2cm |
| 动态跟随精度 | 前车匀速行驶 | 距离误差<5cm |
| 转弯跟随 | 前车转弯 | 后车跟随转弯 |
| 停车响应 | 前车急停 | 后车10cm内停住 |
| 距离可调 | 设定不同距离 | 实际距离误差<5cm |
| 长距离跟随 | 10米直线 | 无累积误差 |

### 6.2 测试流程

```
Step1: 通信测试
  ├── NRF24L01配对成功
  ├── 数据传输稳定
  └── 延迟测试

Step2: 测距标定
  ├── 静态测距精度
  ├── 动态测距精度
  └── 不同距离段精度

Step3: 跟随测试
  ├── 匀速跟随（0.3m/s）
  ├── 匀速跟随（0.5m/s）
  ├── 变速跟随
  └── 转弯跟随

Step4: 安全测试
  ├── 急停响应
  ├── 距离过近保护
  └── 通信中断保护

Step5: 综合测试
  ├── 完整赛道跟随
  ├── 重复性测试
  └── 长时间稳定性
```

## 七、评分要点分析

### 7.1 基本要求评分（50分）

| 评分项 | 分值 | 要点 | 策略 |
|--------|------|------|------|
| 距离跟随 | 25分 | 保持固定距离跟随 | PID距离控制+通信 |
| 速度跟随 | 15分 | 跟随前车速度变化 | 速度闭环+前馈 |
| 停车跟随 | 10分 | 前车停后车停 | 紧急制动算法 |

### 7.2 发挥部分评分（50分）

| 评分项 | 分值 | 要点 | 策略 |
|--------|------|------|------|
| 距离可调 | 15分 | 跟随距离可调 | 参数可设置 |
| 转弯跟随 | 15分 | 跟随转弯不脱线 | 转弯信息通信+差速 |
| 多车编队 | 20分 | 3车以上编队 | 分层通信协议 |

### 7.3 得分策略

1. **保基础**：先确保2车跟随稳定，拿稳40分
2. **优化测距**：多传感器融合提高测距精度
3. **通信可靠**：NRF24L01加上重传机制
4. **扩展编队**：在2车基础上扩展到3车

## 八、调试经验总结

### 8.1 通信调试

```
NRF24L01常见问题：
1. 通信距离不够 → 增大发射功率、加天线
2. 数据丢包 → 加重传机制、降低速率
3. 通信延迟 → 优化数据包大小、提高SPI时钟
```

### 8.2 跟随控制调参

```
PID调参顺序：
1. 先调距离环（外环）：Kp从小到大，使跟随距离稳定
2. 再调速度环（内环）：Kp、Kd，使速度响应快且不超调
3. 最后调转弯补偿：根据转弯信息调整左右轮差速
```

### 8.3 常见问题

1. **跟随震荡**：降低Kp，增大Kd，增加死区
2. **转弯脱线**：前车提前发送转弯意图
3. **测距干扰**：超声波加屏蔽罩，避免串扰
4. **速度不稳**：编码器闭环+低通滤波

---
*本方案基于2022年H题（小车跟随行驶系统）完整解析，适用于电赛备战参考。*
