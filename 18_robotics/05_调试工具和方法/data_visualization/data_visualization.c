/*
 * 数据可视化工具实现
 * 来源：RoboMaster 调试系统
 * 适配平台：MSPM0G3507
 */

#include "data_visualization.h"
#include <string.h>

/**
 * @brief 初始化数据可视化工具
 * @param hvis 可视化句柄
 */
void DataVis_Init(DataVisualization_HandleTypeDef *hvis)
{
    memset(hvis, 0, sizeof(DataVisualization_HandleTypeDef));
}

/**
 * @brief 注册数据通道
 * @param hvis 可视化句柄
 * @param channel 通道号
 * @param type 数据类型
 * @param name 通道名称
 */
void DataVis_RegisterChannel(DataVisualization_HandleTypeDef *hvis, uint8_t channel, DataType type, const char *name)
{
    if (channel < MAX_CHANNELS) {
        hvis->channels[channel].enabled = true;
        hvis->channels[channel].type = type;
        hvis->channels[channel].name = name;
        hvis->channels[channel].value = 0.0f;
        
        if (channel >= hvis->channel_count) {
            hvis->channel_count = channel + 1;
        }
    }
}

/**
 * @brief 设置通道数据值
 * @param hvis 可视化句柄
 * @param channel 通道号
 * @param value 数据值
 */
void DataVis_SetValue(DataVisualization_HandleTypeDef *hvis, uint8_t channel, float value)
{
    if (channel < MAX_CHANNELS && hvis->channels[channel].enabled) {
        hvis->channels[channel].value = value;
    }
}

/**
 * @brief 将float转换为字节数组
 * @param value float值
 * @param buffer 输出缓冲区
 */
static void FloatToBytes(float value, uint8_t *buffer)
{
    memcpy(buffer, &value, 4);
}

/**
 * @brief 发送数据帧
 * @param hvis 可视化句柄
 */
void DataVis_SendData(DataVisualization_HandleTypeDef *hvis)
{
    uint16_t index = 0;
    
    // 帧头
    hvis->tx_buffer[index++] = FRAME_HEADER;
    
    // 通道数量
    hvis->tx_buffer[index++] = hvis->channel_count;
    
    // 各通道数据
    for (uint8_t i = 0; i < hvis->channel_count; i++) {
        if (hvis->channels[i].enabled) {
            // 通道号
            hvis->tx_buffer[index++] = i;
            
            // 数据类型
            hvis->tx_buffer[index++] = (uint8_t)hvis->channels[i].type;
            
            // 数据值（统一转换为float）
            FloatToBytes(hvis->channels[i].value, &hvis->tx_buffer[index]);
            index += 4;
        }
    }
    
    // 帧尾
    hvis->tx_buffer[index++] = FRAME_TAIL;
    
    // 记录长度
    hvis->tx_length = index;
    
    // 调用发送回调
    if (hvis->send_callback != NULL) {
        hvis->send_callback(hvis->tx_buffer, hvis->tx_length);
    }
}

/**
 * @brief 发送原始数据帧
 * @param hvis 可视化句柄
 * @param data 数据
 * @param length 长度
 */
void DataVis_SendFrame(DataVisualization_HandleTypeDef *hvis, uint8_t *data, uint16_t length)
{
    if (hvis->send_callback != NULL) {
        hvis->send_callback(data, length);
    }
}

/**
 * @brief 接收数据处理
 * @param hvis 可视化句柄
 * @param data 数据
 * @param length 长度
 */
void DataVis_ReceiveData(DataVisualization_HandleTypeDef *hvis, uint8_t *data, uint16_t length)
{
    // 检查帧头
    if (data[0] != FRAME_HEADER) {
        return;
    }
    
    // 检查帧尾
    if (data[length - 1] != FRAME_TAIL) {
        return;
    }
    
    // 解析通道数量
    uint8_t channel_count = data[1];
    
    // 解析各通道数据
    uint16_t index = 2;
    for (uint8_t i = 0; i < channel_count; i++) {
        if (index + 6 > length - 1) {
            break;
        }
        
        // 通道号
        uint8_t channel = data[index++];
        
        // 数据类型
        DataType type = (DataType)data[index++];
        
        // 数据值
        float value;
        memcpy(&value, &data[index], 4);
        index += 4;
        
        // 存储数据
        if (channel < MAX_CHANNELS) {
            hvis->channels[channel].value = value;
        }
    }
    
    // 调用接收回调
    if (hvis->receive_callback != NULL) {
        hvis->receive_callback(data, length);
    }
}

/**
 * @brief 设置发送回调函数
 * @param hvis 可视化句柄
 * @param callback 回调函数
 */
void DataVis_SetSendCallback(DataVisualization_HandleTypeDef *hvis, void (*callback)(uint8_t *, uint16_t))
{
    hvis->send_callback = callback;
}

/**
 * @brief 设置接收回调函数
 * @param hvis 可视化句柄
 * @param callback 回调函数
 */
void DataVis_SetReceiveCallback(DataVisualization_HandleTypeDef *hvis, void (*callback)(uint8_t *, uint16_t))
{
    hvis->receive_callback = callback;
}
