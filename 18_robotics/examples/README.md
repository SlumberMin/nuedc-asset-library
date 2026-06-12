# 电赛平台适配综合示例

本示例展示了如何将机器人竞赛优秀方案适配到电赛平台（MSPM0G3507）。

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      应用层                                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐    │
│  │ 任务调度  │  │ 状态机   │  │ 策略控制  │  │ 用户接口  │    │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘    │
├─────────────────────────────────────────────────────────────┤
│                      算法层                                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐    │
│  │ 电机控制  │  │ 路径规划  │  │ 视觉算法  │  │ 数据融合  │    │
│  │ (PID/FOC) │  │ (A*)     │  │ (车道线)  │  │ (卡尔曼)  │    │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘    │
├─────────────────────────────────────────────────────────────┤
│                      驱动层                                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐    │
│  │ 电机驱动  │  │ 编码器   │  │ 摄像头   │  │ 传感器   │    │
│  │ (TB6612)  │  │ (TIM)    │  │ (OV7670) │  │ (ADC)    │    │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘    │
├─────────────────────────────────────────────────────────────┤
│                      硬件层                                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐    │
│  │ MSPM0G3507│  │ 电源管理  │  │ 通信接口  │  │ 调试接口  │    │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘    │
└─────────────────────────────────────────────────────────────┘
```

## 文件结构

```
examples/
├── main.c                      # 主程序
├── system_init.c/h             # 系统初始化
├── task_scheduler.c/h          # 任务调度器
├── motor_control.c/h           # 电机控制
├── path_tracking.c/h           # 路径跟踪
├── vision_processing.c/h       # 视觉处理
├── debug_interface.c/h         # 调试接口
└── README.md                   # 本文件
```

## 快速开始

### 1. 硬件准备
- MSPM0G3507开发板
- 电机驱动模块（TB6612/L298N）
- 编码器电机
- 摄像头模块（OV7670）
- 超声波传感器
- 蓝牙/WiFi模块

### 2. 软件配置
- 安装CCS或Keil开发环境
- 导入项目模板
- 配置引脚和外设

### 3. 编译烧录
- 编译项目
- 烧录到开发板
- 连接调试器

### 4. 运行测试
- 上电运行
- 使用上位机查看数据
- 调整参数

## 代码示例

### 主程序框架
```c
#include "system_init.h"
#include "task_scheduler.h"
#include "motor_control.h"
#include "vision_processing.h"
#include "debug_interface.h"

int main(void)
{
    // 系统初始化
    System_Init();
    
    // 任务调度器初始化
    TaskScheduler_Init();
    
    // 主循环
    while (1) {
        // 运行任务调度器
        TaskScheduler_Run();
    }
}
```

### 任务调度器
```c
void TaskScheduler_Init(void)
{
    // 注册任务
    TaskScheduler_RegisterTask(1, MotorControl_Task);      // 1ms周期
    TaskScheduler_RegisterTask(10, VisionProcessing_Task);  // 10ms周期
    TaskScheduler_RegisterTask(100, DebugInterface_Task);   // 100ms周期
}
```

## 调试方法

### 1. 使用数据可视化工具
```c
// 初始化数据可视化
DataVis_Init(&hvis);
DataVis_RegisterChannel(&hvis, 0, DATA_TYPE_FLOAT, "速度");
DataVis_RegisterChannel(&hvis, 1, DATA_TYPE_FLOAT, "角度");
DataVis_RegisterChannel(&hvis, 2, DATA_TYPE_FLOAT, "偏移量");

// 主循环中更新数据
DataVis_SetValue(&hvis, 0, motor_speed);
DataVis_SetValue(&hvis, 1, encoder_angle);
DataVis_SetValue(&hvis, 2, lane_offset);
DataVis_SendData(&hvis);
```

### 2. 使用串口打印
```c
// 调试信息输出
printf("Speed: %.2f rpm\n", motor_speed);
printf("Angle: %.2f deg\n", encoder_angle);
printf("Offset: %.2f px\n", lane_offset);
```

### 3. 使用LED指示
```c
// 状态指示
LED_SetColor(GREEN);   // 正常运行
LED_SetColor(YELLOW);  // 警告
LED_SetColor(RED);     // 错误
```

## 性能优化

### 1. 中断优先级
```c
// 设置中断优先级
NVIC_SetPriority(TIMER0_IRQn, 0);  // 最高优先级
NVIC_SetPriority(UART0_IRQn, 1);   // 次高优先级
NVIC_SetPriority(ADC0_IRQn, 2);    // 中等优先级
```

### 2. 内存优化
```c
// 使用const修饰常量
const float PID_KP = 0.5f;
const float PID_KI = 0.01f;
const float PID_KD = 0.1f;

// 使用位域节省内存
typedef struct {
    uint8_t flag1 : 1;
    uint8_t flag2 : 1;
    uint8_t flag3 : 1;
    uint8_t reserved : 5;
} Flags;
```

### 3. 代码优化
```c
// 使用inline函数
inline float Clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// 使用查表法
const float sin_table[360] = {...};
float FastSin(int angle)
{
    return sin_table[angle % 360];
}
```

## 常见问题

### 1. 编译错误
- 检查头文件路径
- 检查库文件是否正确链接
- 检查编译器版本

### 2. 运行异常
- 检查时钟配置
- 检查外设初始化
- 检查中断配置

### 3. 性能不足
- 优化算法复杂度
- 使用DMA传输
- 降低采样率

## 参考资源

1. MSPM0G3507技术手册
2. TI官方例程
3. RoboMaster开源项目
4. 飞思卡尔智能车竞赛资料
