# 数据可视化工具使用文档

## 1. 功能介绍

数据可视化工具用于实时显示嵌入式系统的运行数据，帮助开发者调试和优化系统。

### 1.1 主要功能

- **实时波形显示**: 将变量值以波形图形式显示
- **多通道支持**: 同时显示多个变量
- **参数调整**: 通过上位机实时修改系统参数
- **数据记录**: 保存历史数据用于分析

### 1.2 支持的上位机

- **VOFA+**: 功能强大的调试工具，支持多种协议
- **SerialPlot**: 轻量级波形显示工具
- **Serial Studio**: 开源串口调试工具
- **自定义上位机**: 使用Python/C#等开发

## 2. 硬件连接

### 2.1 串口连接
```
MSPM0G3507 TX → USB转TTL RX
MSPM0G3507 RX → USB转TTL TX
MSPM0G3507 GND → USB转TTL GND
```

### 2.2 波特率设置
- 推荐波特率：115200 或 921600
- 数据位：8
- 停止位：1
- 校验位：无

## 3. 使用步骤

### 3.1 初始化
```c
#include "data_visualization.h"

DataVisualization_HandleTypeDef hvis;

void main()
{
    // 初始化串口
    UART_Init(115200);
    
    // 初始化数据可视化
    DataVis_Init(&hvis);
    
    // 注册通道
    DataVis_RegisterChannel(&hvis, 0, DATA_TYPE_FLOAT, "速度");
    DataVis_RegisterChannel(&hvis, 1, DATA_TYPE_FLOAT, "角度");
    DataVis_RegisterChannel(&hvis, 2, DATA_TYPE_FLOAT, "电流");
    DataVis_RegisterChannel(&hvis, 3, DATA_TYPE_FLOAT, "温度");
    
    // 设置发送回调
    DataVis_SetSendCallback(&hvis, UART_SendData);
    
    // 主循环
    while (1) {
        // 更新数据
        DataVis_SetValue(&hvis, 0, motor_speed);
        DataVis_SetValue(&hvis, 1, encoder_angle);
        DataVis_SetValue(&hvis, 2, motor_current);
        DataVis_SetValue(&hvis, 3, temperature);
        
        // 发送数据
        DataVis_SendData(&hvis);
        
        // 延时
        HAL_Delay(10);
    }
}
```

### 3.2 使用VOFA+查看数据

1. 下载并安装VOFA+
2. 选择串口和波特率
3. 选择协议：RawData或FireWater
4. 添加通道并设置显示范围
5. 开始接收数据

### 3.3 使用SerialPlot查看数据

1. 下载并安装SerialPlot
2. 配置串口参数
3. 设置数据格式
4. 开始接收数据

## 4. 协议说明

### 4.1 RawData协议
```
帧格式：[帧头][通道数][通道数据...][帧尾]

帧头：0xAA
通道数：1字节
通道数据：[通道号][数据类型][数据值(4字节)]
帧尾：0x55
```

### 4.2 FireWater协议
```
格式：通道1值,通道2值,通道3值,...\n

示例：1.23,4.56,7.89\n
```

### 4.3 JustFloat协议
```
帧格式：[数据...][帧尾]

数据：多个float值（小端序）
帧尾：0x00 0x00 0x80 0x7F
```

## 5. 高级功能

### 5.1 参数调整
```c
// 接收回调函数
void OnReceiveParam(uint8_t *data, uint16_t length)
{
    // 解析参数
    uint8_t param_id = data[0];
    float param_value;
    memcpy(&param_value, &data[1], 4);
    
    // 设置参数
    switch (param_id) {
        case 0: pid.kp = param_value; break;
        case 1: pid.ki = param_value; break;
        case 2: pid.kd = param_value; break;
        case 3: target_speed = param_value; break;
    }
}

// 注册接收回调
DataVis_SetReceiveCallback(&hvis, OnReceiveParam);
```

### 5.2 数据记录
```c
// 记录数据到文件
void RecordData(DataVisualization_HandleTypeDef *hvis, uint32_t duration_ms)
{
    uint32_t start_time = HAL_GetTick();
    uint32_t sample_count = 0;
    
    while (HAL_GetTick() - start_time < duration_ms) {
        // 发送数据
        DataVis_SendData(hvis);
        
        // 增加采样计数
        sample_count++;
        
        // 延时
        HAL_Delay(10);
    }
    
    // 输出采样信息
    printf("Sample count: %lu\n", sample_count);
    printf("Sample rate: %.1f Hz\n", (float)sample_count / (duration_ms / 1000.0f));
}
```

### 5.3 性能监控
```c
// 性能监控通道
DataVis_RegisterChannel(&hvis, 4, DATA_TYPE_FLOAT, "CPU负载");
DataVis_RegisterChannel(&hvis, 5, DATA_TYPE_FLOAT, "循环时间");
DataVis_RegisterChannel(&hvis, 6, DATA_TYPE_FLOAT, "堆内存");
DataVis_RegisterChannel(&hvis, 7, DATA_TYPE_FLOAT, "栈内存");

// 在主循环中更新性能数据
void UpdatePerformanceData(void)
{
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 计算循环时间
    float loop_time = (float)(current_time - last_time);
    DataVis_SetValue(&hvis, 5, loop_time);
    
    // 计算CPU负载
    float cpu_load = CalculateCPULoad();
    DataVis_SetValue(&hvis, 4, cpu_load);
    
    // 获取内存使用情况
    DataVis_SetValue(&hvis, 6, (float)GetHeapUsage());
    DataVis_SetValue(&hvis, 7, (float)GetStackUsage());
    
    last_time = current_time;
}
```

## 6. 调试技巧

### 6.1 选择合适的采样率
- **低速信号**: 10-100 Hz
- **中速信号**: 100-1000 Hz
- **高速信号**: 1000-10000 Hz

### 6.2 设置合适的显示范围
```c
// 根据数据范围设置Y轴
// 速度：-1000 ~ 1000 rpm
// 角度：-180 ~ 180 度
// 电流：-10 ~ 10 A
// 温度：0 ~ 100 °C
```

### 6.3 使用触发功能
```c
// 当数据超过阈值时触发记录
#define TRIGGER_THRESHOLD 100.0f

void CheckTrigger(float value)
{
    static bool triggered = false;
    
    if (value > TRIGGER_THRESHOLD && !triggered) {
        // 开始记录
        StartRecording();
        triggered = true;
    } else if (value < TRIGGER_THRESHOLD * 0.9f && triggered) {
        // 停止记录
        StopRecording();
        triggered = false;
    }
}
```

## 7. 常见问题

### 7.1 数据乱码
- 检查波特率设置
- 检查数据格式
- 确保帧头帧尾正确

### 7.2 数据不同步
- 添加帧头帧尾
- 使用校验和
- 降低发送频率

### 7.3 显示卡顿
- 减少通道数量
- 降低采样率
- 使用更快的串口波特率

## 8. 示例项目

### 8.1 PID参数调试
```c
void PID_Debug(void)
{
    // 注册PID相关通道
    DataVis_RegisterChannel(&hvis, 0, DATA_TYPE_FLOAT, "目标值");
    DataVis_RegisterChannel(&hvis, 1, DATA_TYPE_FLOAT, "实际值");
    DataVis_RegisterChannel(&hvis, 2, DATA_TYPE_FLOAT, "误差");
    DataVis_RegisterChannel(&hvis, 3, DATA_TYPE_FLOAT, "输出");
    
    while (1) {
        // 更新数据
        DataVis_SetValue(&hvis, 0, target);
        DataVis_SetValue(&hvis, 1, measured);
        DataVis_SetValue(&hvis, 2, target - measured);
        DataVis_SetValue(&hvis, 3, output);
        
        // 发送数据
        DataVis_SendData(&hvis);
        
        HAL_Delay(10);
    }
}
```

### 8.2 电机性能测试
```c
void Motor_Test(void)
{
    // 注册电机相关通道
    DataVis_RegisterChannel(&hvis, 0, DATA_TYPE_FLOAT, "速度");
    DataVis_RegisterChannel(&hvis, 1, DATA_TYPE_FLOAT, "电流");
    DataVis_RegisterChannel(&hvis, 2, DATA_TYPE_FLOAT, "温度");
    DataVis_RegisterChannel(&hvis, 3, DATA_TYPE_FLOAT, "PWM占空比");
    
    // 速度扫描测试
    for (float speed = 0; speed <= 1000; speed += 100) {
        Motor_SetSpeed(speed);
        HAL_Delay(1000);
        
        // 记录数据
        for (int i = 0; i < 100; i++) {
            DataVis_SetValue(&hvis, 0, Motor_GetSpeed());
            DataVis_SetValue(&hvis, 1, Motor_GetCurrent());
            DataVis_SetValue(&hvis, 2, Motor_GetTemperature());
            DataVis_SetValue(&hvis, 3, speed / 1000.0f);
            DataVis_SendData(&hvis);
            HAL_Delay(10);
        }
    }
}
```
