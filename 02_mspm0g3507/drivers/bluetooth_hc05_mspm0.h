/**
 * @file    bluetooth_hc05_mspm0.h
 * @brief   HC-05 蓝牙模块驱动 — MSPM0G3507
 * @note    适用于HC-05主从一体蓝牙模块
 *          UART通信，支持数据收发和连接状态检测
 *
 * 接线示例:
 *   MSPM0 PA9 → HC-05 TX (MCU的RX)
 *   MSPM0 PA8 → HC-05 RX (MCU的TX)
 *   MSPM0 PA7 → HC-05 STATE (连接状态)
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#ifndef __BLUETOOTH_HC05_MSPM0_H
#define __BLUETOOTH_HC05_MSPM0_H

#include <stdint.h>
#include "platform/driverlib_mspm0.h"

/* ── 配置常量 ──────────────────────────────────────────────── */
#define BT_RX_BUF_SIZE  256     /* 接收缓冲区大小 */

/* ── 连接状态 ──────────────────────────────────────────────── */
typedef enum {
    BT_DISCONNECTED = 0,    /* 未连接 */
    BT_CONNECTED = 1        /* 已连接 */
} BT_ConnectState;

/* ── 配置结构 ──────────────────────────────────────────────── */
typedef struct {
    UART_Regs *uart;        /* UART实例 (如 UART_2_INST) */
    GPIO_Regs *state_port;  /* STATE引脚端口 */
    uint32_t   state_pin;   /* STATE引脚 */
    uint32_t   baudrate;    /* 波特率 (默认9600) */
} BT_HC05_Config;

/* ── 数据结构 ──────────────────────────────────────────────── */
typedef struct {
    uint8_t  rx_buf[BT_RX_BUF_SIZE]; /* 接收缓冲区 */
    volatile uint16_t rx_len;        /* 接收数据长度 */
    volatile uint8_t  rx_flag;       /* 接收完成标志 */
    BT_ConnectState connect_state;   /* 连接状态 */
    uint32_t rx_count;               /* 接收计数 */
    uint32_t tx_count;               /* 发送计数 */
} BT_HC05_Data;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化HC-05蓝牙模块
 * @param cfg  指向BT_HC05_Config结构
 */
void BT_HC05_Init(const BT_HC05_Config *cfg);

/**
 * @brief 发送单个字节
 * @param ch  要发送的字节
 */
void BT_HC05_SendByte(uint8_t ch);

/**
 * @brief 发送字符串
 * @param str  要发送的字符串
 */
void BT_HC05_SendString(const char *str);

/**
 * @brief 发送数据
 * @param data  数据指针
 * @param len   数据长度
 */
void BT_HC05_SendData(const uint8_t *data, uint16_t len);

/**
 * @brief 检查是否收到数据
 * @return 1=收到数据, 0=未收到
 */
uint8_t BT_HC05_IsDataReceived(void);

/**
 * @brief 获取接收到的数据
 * @param buf   存储数据的缓冲区
 * @param max_len  缓冲区最大长度
 * @return 实际接收的数据长度
 */
uint16_t BT_HC05_GetReceivedData(uint8_t *buf, uint16_t max_len);

/**
 * @brief 清除接收缓冲区
 */
void BT_HC05_ClearRxBuffer(void);

/**
 * @brief 获取连接状态
 * @return BT_CONNECTED 或 BT_DISCONNECTED
 */
BT_ConnectState BT_HC05_GetConnectState(void);

/**
 * @brief 检查是否已连接
 * @return 1=已连接, 0=未连接
 */
uint8_t BT_HC05_IsConnected(void);

/**
 * @brief 发送数据（自动检查连接状态）
 * @param data  数据指针
 * @param len   数据长度
 * @return 1=发送成功, 0=未连接
 */
uint8_t BT_HC05_SendIfConnected(const uint8_t *data, uint16_t len);

/**
 * @brief 获取接收计数
 * @return 接收字节数
 */
uint32_t BT_HC05_GetRxCount(void);

/**
 * @brief 获取发送计数
 * @return 发送字节数
 */
uint32_t BT_HC05_GetTxCount(void);

#endif /* __BLUETOOTH_HC05_MSPM0_H */