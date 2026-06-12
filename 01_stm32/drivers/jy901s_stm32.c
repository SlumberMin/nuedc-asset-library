/**
 * @file    jy901s_stm32.c
 * @brief   JY901S IMU 驱动模块实现 — STM32 HAL库版本
 */

#include "drivers/jy901s_stm32.h"
#include <string.h>

/* ========================================================================== */
/*                              内部函数                                       */
/* ========================================================================== */

/**
 * @brief 校验帧数据
 * @param buf  11字节帧缓冲区
 * @return true=校验通过
 */
static bool JY901S_CheckFrame(const uint8_t *buf)
{
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += buf[i];
    }
    return (sum == buf[10]);
}

/**
 * @brief 解析单帧数据
 * @param dev  JY901S设备结构体指针
 * @param buf  11字节帧缓冲区
 */
static void JY901S_ParseFrame(JY901S_Dev_t *dev, const uint8_t *buf)
{
    int16_t raw[3];

    /* 提取3个int16_t原始值（小端序） */
    raw[0] = (int16_t)(buf[2] | (buf[3] << 8));
    raw[1] = (int16_t)(buf[4] | (buf[5] << 8));
    raw[2] = (int16_t)(buf[6] | (buf[7] << 8));

    switch (buf[1]) {
        case JY901S_ID_ACCEL:  /* 0x51 加速度 */
            dev->data.accel_x = (float)raw[0] / 32768.0f * 16.0f;  /* ±16g */
            dev->data.accel_y = (float)raw[1] / 32768.0f * 16.0f;
            dev->data.accel_z = (float)raw[2] / 32768.0f * 16.0f;
            break;

        case JY901S_ID_GYRO:  /* 0x52 角速度 */
            dev->data.gyro_x = (float)raw[0] / 32768.0f * 2000.0f;  /* ±2000°/s */
            dev->data.gyro_y = (float)raw[1] / 32768.0f * 2000.0f;
            dev->data.gyro_z = (float)raw[2] / 32768.0f * 2000.0f;
            break;

        case JY901S_ID_ANGLE:  /* 0x53 角度 */
            dev->data.roll  = (float)raw[0] / 32768.0f * 180.0f;  /* ±180° */
            dev->data.pitch = (float)raw[1] / 32768.0f * 180.0f;
            dev->data.yaw   = (float)raw[2] / 32768.0f * 180.0f;
            break;

        default:
            break;
    }

    dev->data.update_tick = HAL_GetTick();
    dev->data.frame_count++;
    dev->data.new_data = true;
}

/* ========================================================================== */
/*                              接口函数实现                                   */
/* ========================================================================== */

HAL_StatusTypeDef JY901S_Init(JY901S_Dev_t *dev, UART_HandleTypeDef *huart)
{
    if (dev == NULL || huart == NULL) return HAL_ERROR;

    dev->huart    = huart;
    dev->rx_index = 0;
    dev->rx_state = 0;
    memset(&dev->data, 0, sizeof(JY901S_Data_t));
    dev->initialized = true;

    return HAL_OK;
}

HAL_StatusTypeDef JY901S_StartReceive(JY901S_Dev_t *dev)
{
    if (dev == NULL || !dev->initialized) return HAL_ERROR;

    /* 启动单字节中断接收 */
    return HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

void JY901S_RxCallback(JY901S_Dev_t *dev)
{
    if (dev == NULL || !dev->initialized) return;

    uint8_t byte = dev->rx_byte;

    switch (dev->rx_state) {
        case 0:  /* 等待帧头 0x55 */
            if (byte == JY901S_HEADER) {
                dev->rx_buf[0] = byte;
                dev->rx_index  = 1;
                dev->rx_state  = 1;
            }
            break;

        case 1:  /* 接收后续10字节 */
            dev->rx_buf[dev->rx_index++] = byte;
            if (dev->rx_index >= JY901S_FRAME_LEN) {
                /* 帧接收完毕，校验并解析 */
                if (JY901S_CheckFrame(dev->rx_buf)) {
                    JY901S_ParseFrame(dev, dev->rx_buf);
                }
                dev->rx_index = 0;
                dev->rx_state = 0;
            }
            break;

        default:
            dev->rx_index = 0;
            dev->rx_state = 0;
            break;
    }

    /* 继续下一字节接收 */
    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

bool JY901S_GetData(JY901S_Dev_t *dev, JY901S_Data_t *data)
{
    if (dev == NULL || data == NULL) return false;
    if (!dev->data.new_data) return false;

    *data = dev->data;
    dev->data.new_data = false;
    return true;
}

float JY901S_GetYaw(JY901S_Dev_t *dev)
{
    if (dev == NULL) return 0.0f;
    dev->data.new_data = false;
    return dev->data.yaw;
}
