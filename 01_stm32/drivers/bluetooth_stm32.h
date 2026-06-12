/**
 * @file    bluetooth_stm32.h
 * @brief   HC-05 蓝牙模块驱动 — STM32 HAL库版本（UART中断收发）
 * @details 默认波特率 9600-8-N-1, 支持中断接收和环形缓冲区。
 * @version 1.0
 * @date    2026-06
 */

#ifndef __BLUETOOTH_STM32_H
#define __BLUETOOTH_STM32_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── 缓冲区大小 ─────────────────────────────────────────── */
#ifndef BT_RX_BUF_SIZE
#define BT_RX_BUF_SIZE          (256)
#endif

#ifndef BT_TX_BUF_SIZE
#define BT_TX_BUF_SIZE          (256)
#endif

/* ── 设备结构体 ─────────────────────────────────────────── */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t  rx_byte;                           /* 单字节接收缓冲 */
    uint8_t  rx_buf[BT_RX_BUF_SIZE];           /* 环形接收缓冲区 */
    volatile uint16_t rx_head;                  /* 写入位置 (ISR) */
    volatile uint16_t rx_tail;                  /* 读取位置 (主循环) */
    bool     initialized;
} BT_Dev_t;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化蓝牙模块
 * @param dev   设备结构体指针
 * @param huart UART句柄指针（需已初始化为9600-8-N-1）
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef BT_Init(BT_Dev_t *dev, UART_HandleTypeDef *huart);

/**
 * @brief 启动UART中断接收（调用一次即可）
 * @param dev   设备结构体指针
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef BT_StartReceive(BT_Dev_t *dev);

/**
 * @brief UART接收完成回调（在HAL_UART_RxCpltCallback中调用）
 * @param dev   设备结构体指针
 */
void BT_RxCallback(BT_Dev_t *dev);

/**
 * @brief 从环形缓冲区读取一个字节
 * @param dev   设备结构体指针
 * @param byte  输出字节
 * @return true=有数据, false=缓冲区空
 */
bool BT_ReadByte(BT_Dev_t *dev, uint8_t *byte);

/**
 * @brief 检查接收缓冲区中是否有数据
 * @param dev   设备结构体指针
 * @return 可读字节数
 */
uint16_t BT_Available(BT_Dev_t *dev);

/**
 * @brief 通过蓝牙发送数据
 * @param dev   设备结构体指针
 * @param data  发送数据指针
 * @param len   数据长度
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef BT_Send(BT_Dev_t *dev, const uint8_t *data, uint16_t len);

/**
 * @brief 通过蓝牙发送字符串
 * @param dev   设备结构体指针
 * @param str   以'\0'结尾的字符串
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef BT_SendString(BT_Dev_t *dev, const char *str);

/**
 * @brief 清空接收缓冲区
 * @param dev   设备结构体指针
 */
void BT_Flush(BT_Dev_t *dev);

#endif /* __BLUETOOTH_STM32_H */
