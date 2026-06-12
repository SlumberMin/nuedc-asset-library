/*
 * 数据可视化工具
 * 来源：RoboMaster 调试系统
 * 适配平台：MSPM0G3507
 * 
 * 设计思路：
 * 1. 通过串口发送数据到上位机
 * 2. 上位机使用波形显示工具（如SerialPlot、VOFA+）
 * 3. 支持多通道数据同时显示
 * 4. 支持实时调整参数
 */

#ifndef DATA_VISUALIZATION_H
#define DATA_VISUALIZATION_H

#include <stdint.h>
#include <stdbool.h>

// 通道数量
#define MAX_CHANNELS    8

// 数据帧格式
#define FRAME_HEADER    0xAA
#define FRAME_TAIL      0x55

// 数据类型
typedef enum {
    DATA_TYPE_INT8 = 0,
    DATA_TYPE_UINT8,
    DATA_TYPE_INT16,
    DATA_TYPE_UINT16,
    DATA_TYPE_INT32,
    DATA_TYPE_UINT32,
    DATA_TYPE_FLOAT
} DataType;

// 通道配置
typedef struct {
    bool enabled;           // 是否启用
    DataType type;          // 数据类型
    const char *name;       // 通道名称
    float value;            // 当前值
} ChannelConfig;

// 数据可视化结构体
typedef struct {
    ChannelConfig channels[MAX_CHANNELS];
    uint8_t channel_count;
    
    // 发送缓冲区
    uint8_t tx_buffer[128];
    uint16_t tx_length;
    
    // 接收缓冲区
    uint8_t rx_buffer[128];
    uint16_t rx_length;
    
    // 回调函数
    void (*send_callback)(uint8_t *data, uint16_t length);
    void (*receive_callback)(uint8_t *data, uint16_t length);
    
} DataVisualization_HandleTypeDef;

// 函数声明
void DataVis_Init(DataVisualization_HandleTypeDef *hvis);
void DataVis_RegisterChannel(DataVisualization_HandleTypeDef *hvis, uint8_t channel, DataType type, const char *name);
void DataVis_SetValue(DataVisualization_HandleTypeDef *hvis, uint8_t channel, float value);
void DataVis_SendData(DataVisualization_HandleTypeDef *hvis);
void DataVis_SendFrame(DataVisualization_HandleTypeDef *hvis, uint8_t *data, uint16_t length);
void DataVis_ReceiveData(DataVisualization_HandleTypeDef *hvis, uint8_t *data, uint16_t length);
void DataVis_SetSendCallback(DataVisualization_HandleTypeDef *hvis, void (*callback)(uint8_t *, uint16_t));
void DataVis_SetReceiveCallback(DataVisualization_HandleTypeDef *hvis, void (*callback)(uint8_t *, uint16_t));

#endif // DATA_VISUALIZATION_H
