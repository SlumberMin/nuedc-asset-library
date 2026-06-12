/**
 * @file    bluetooth_stm32.c
 * @brief   HC-05 蓝牙模块驱动实现 — STM32 HAL库版本
 */

#include "drivers/bluetooth_stm32.h"
#include <string.h>

/* ── 初始化 ─────────────────────────────────────────────── */
HAL_StatusTypeDef BT_Init(BT_Dev_t *dev, UART_HandleTypeDef *huart)
{
    dev->huart = huart;
    dev->rx_head = 0;
    dev->rx_tail = 0;
    dev->initialized = true;
    return HAL_OK;
}

/* ── 启动中断接收 ───────────────────────────────────────── */
HAL_StatusTypeDef BT_StartReceive(BT_Dev_t *dev)
{
    return HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

/* ── 接收回调 ───────────────────────────────────────────── */
void BT_RxCallback(BT_Dev_t *dev)
{
    /* 将接收到的字节存入环形缓冲区 */
    uint16_t next_head = (dev->rx_head + 1) % BT_RX_BUF_SIZE;
    if (next_head != dev->rx_tail) {
        /* 缓冲区未满 */
        dev->rx_buf[dev->rx_head] = dev->rx_byte;
        dev->rx_head = next_head;
    }
    /* 否则丢弃该字节(缓冲区满) */

    /* 重新启动接收 */
    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

/* ── 读取一个字节 ───────────────────────────────────────── */
bool BT_ReadByte(BT_Dev_t *dev, uint8_t *byte)
{
    if (dev->rx_head == dev->rx_tail)
        return false;

    *byte = dev->rx_buf[dev->rx_tail];
    dev->rx_tail = (dev->rx_tail + 1) % BT_RX_BUF_SIZE;
    return true;
}

/* ── 可读字节数 ─────────────────────────────────────────── */
uint16_t BT_Available(BT_Dev_t *dev)
{
    return (dev->rx_head - dev->rx_tail + BT_RX_BUF_SIZE) % BT_RX_BUF_SIZE;
}

/* ── 发送数据 ───────────────────────────────────────────── */
HAL_StatusTypeDef BT_Send(BT_Dev_t *dev, const uint8_t *data, uint16_t len)
{
    return HAL_UART_Transmit(dev->huart, (uint8_t *)data, len, HAL_MAX_DELAY);
}

/* ── 发送字符串 ─────────────────────────────────────────── */
HAL_StatusTypeDef BT_SendString(BT_Dev_t *dev, const char *str)
{
    return BT_Send(dev, (const uint8_t *)str, (uint16_t)strlen(str));
}

/* ── 清空缓冲区 ─────────────────────────────────────────── */
void BT_Flush(BT_Dev_t *dev)
{
    dev->rx_tail = dev->rx_head;
}
