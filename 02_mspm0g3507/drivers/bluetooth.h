/**
 * @file    bluetooth.h
 * @brief   HC-05 蓝牙模块 UART 驱动 — MSPM0G3507
 *
 * HC-05 通信协议:
 *   - UART 9600 8N1 (默认透传模式)
 *   - AT指令模式: EN引脚拉高, 波特率38400
 *   - 透传模式:   EN引脚拉低, 波特率9600
 *   - 支持数据收发、AT指令配置
 *
 * 硬件连接:
 *   MSPM0 PA17(UART1_TX) → HC-05 RX
 *   MSPM0 PA18(UART1_RX) → HC-05 TX
 *   MSPM0 PA16(GPIO)      → HC-05 EN (高=AT模式, 低=透传)
 *
 * 依赖: ti_msp_dl_config.h (SysConfig生成, 需配置UART_1 + BT EN GPIO)
 */

#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ── 配置 ──────────────────────────────────────────────────── */

#define BT_RX_BUF_SIZE      256     /* 接收缓冲区大小 */
#define BT_TX_BUF_SIZE      256     /* 发送缓冲区大小 */
#define BT_AT_TIMEOUT_MS    1000    /* AT指令响应超时(ms) */

/* ── 蓝牙状态 ──────────────────────────────────────────────── */

typedef enum {
    BT_MODE_TRANSPARENT = 0,    /* 透传模式 (EN=LOW) */
    BT_MODE_AT,                 /* AT指令模式 (EN=HIGH) */
} BT_Mode;

typedef enum {
    BT_STATE_IDLE = 0,
    BT_STATE_CONNECTED,
    BT_STATE_AT_PENDING,
} BT_State;

/* ── 蓝牙数据结构 ──────────────────────────────────────────── */

typedef struct {
    /* 接收环形缓冲区 */
    volatile uint8_t  rx_buf[BT_RX_BUF_SIZE];
    volatile uint16_t rx_head;          /* 写入位置 (中断) */
    volatile uint16_t rx_tail;          /* 读取位置 (主循环) */

    /* 状态 */
    volatile BT_State state;
    volatile BT_Mode  mode;

    /* AT模式响应缓冲区 */
    volatile char     at_response[128];
    volatile uint16_t at_resp_len;
    volatile bool     at_resp_ready;

    /* 统计 */
    volatile uint32_t rx_count;
    volatile uint32_t tx_count;
    volatile uint32_t overflow_count;
} BT_Handle;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化蓝牙驱动
 *        配置UART1接收中断, 设置EN引脚为输出, 进入透传模式
 */
void BT_Init(void);

/**
 * @brief 切换到AT指令模式
 *        拉高EN引脚, 等待200ms, 切换UART波特率到38400
 * @note  HC-05在EN拉高后需要约200ms进入AT模式
 */
void BT_EnterATMode(void);

/**
 * @brief 切换回透传模式
 *        拉低EN引脚, 切换UART波特率到9600
 */
void BT_EnterTransparentMode(void);

/**
 * @brief 发送AT指令并等待响应
 * @param cmd AT指令字符串 (不含\r\n)
 * @param response 响应缓冲区
 * @param resp_size 缓冲区大小
 * @param timeout_ms 超时时间(ms)
 * @return true=收到响应, false=超时
 */
bool BT_SendATCommand(const char *cmd, char *response, uint16_t resp_size, uint32_t timeout_ms);

/**
 * @brief 获取蓝牙当前模式
 * @return BT_MODE_TRANSPARENT 或 BT_MODE_AT
 */
BT_Mode BT_GetMode(void);

/**
 * @brief 发送单个字节 (非阻塞)
 * @param data 要发送的字节
 */
void BT_SendByte(uint8_t data);

/**
 * @brief 发送数据缓冲区
 * @param data 数据指针
 * @param len 数据长度
 */
void BT_SendData(const uint8_t *data, uint16_t len);

/**
 * @brief 发送字符串
 * @param str 以'\0'结尾的字符串
 */
void BT_SendString(const char *str);

/**
 * @brief 检查接收缓冲区中是否有数据
 * @return 可读字节数
 */
uint16_t BT_Available(void);

/**
 * @brief 读取一个字节
 * @return 字节数据, 无数据返回-1
 */
int16_t BT_Read(void);

/**
 * @brief 读取指定长度数据
 * @param buf 输出缓冲区
 * @param len 期望读取长度
 * @return 实际读取字节数
 */
uint16_t BT_ReadData(uint8_t *buf, uint16_t len);

/**
 * @brief 读取一行 (以\r\n或\n结尾)
 * @param buf 输出缓冲区
 * @param max_len 缓冲区大小
 * @return 读取到的字符数, 无完整行返回0
 */
uint16_t BT_ReadLine(char *buf, uint16_t max_len);

/**
 * @brief 清空接收缓冲区
 */
void BT_Flush(void);

/**
 * @brief 获取蓝牙连接状态
 * @return BT_State
 */
BT_State BT_GetState(void);

/**
 * @brief UART1中断处理函数
 *        在UART1_IRQHandler中调用
 */
void BT_UART_IRQHandler(void);

#endif /* __BLUETOOTH_H */
