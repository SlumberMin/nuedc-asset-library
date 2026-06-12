/**
 * @file    bluetooth_tm4c.h
 * @brief   HC-05蓝牙模块驱动 头文件 (TM4C123 UART)
 * @details 经典蓝牙SPP透传模块，支持AT指令配置和数据收发
 *
 * 硬件接线:
 *   HC-05          TM4C123
 *   -----          --------
 *   TX  ---------->  PC6 (U3RX)  或  PE4 (U5RX)
 *   RX  ---------->  PC7 (U3TX)  或  PE5 (U5TX)
 *   EN  ---------->  (可选) PA6 (高电平使能)
 *   VCC ---------->  5V
 *   GND ---------->  GND
 *
 * @note    默认波特率: 9600 (AT模式38400)
 * @note    AT指令需要EN引脚拉高或KEY引脚拉高
 * @note    数据传输使用UART中断+环形缓冲区
 */

#ifndef BLUETOOTH_TM4C_H
#define BLUETOOTH_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 缓冲区大小 ========== */
#define BT_RX_BUF_SIZE      256     /* 接收环形缓冲区大小 */
#define BT_TX_BUF_SIZE      256     /* 发送缓冲区大小 */
#define BT_LINE_MAX_LEN     64      /* 单行最大长度 */

/* ========== UART模块选择 ========== */
typedef enum {
    BT_UART3 = 3,   /* UART3: PC6(RX)/PC7(TX) */
    BT_UART5 = 5    /* UART5: PE4(RX)/PE5(TX) */
} BT_UART_t;

/* ========== 蓝牙状态 ========== */
typedef enum {
    BT_STATE_IDLE = 0,      /* 空闲 */
    BT_STATE_CONNECTED,     /* 已连接 */
    BT_STATE_AT_MODE        /* AT指令模式 */
} BT_State_t;

/* ========== 配置结构体 ========== */
typedef struct {
    BT_UART_t       uart_module;    /* UART模块选择 */
    uint32_t        baudrate;       /* 波特率 (默认9600) */
    uint32_t        sys_clock_hz;   /* 系统时钟频率 */
} Bluetooth_Config_t;

/* ========== 函数声明 ========== */

/**
 * @brief  初始化HC-05蓝牙模块
 * @param  cfg  配置结构体指针
 */
void Bluetooth_Init(const Bluetooth_Config_t *cfg);

/**
 * @brief  发送单个字节
 * @param  ch  字节数据
 */
void Bluetooth_SendByte(uint8_t ch);

/**
 * @brief  发送字符串
 * @param  str  以'\0'结尾的字符串
 */
void Bluetooth_SendString(const char *str);

/**
 * @brief  发送指定长度的数据
 * @param  data  数据缓冲区
 * @param  len   数据长度
 */
void Bluetooth_SendData(const uint8_t *data, uint16_t len);

/**
 * @brief  检查是否有可读数据
 * @return 可读字节数
 */
uint16_t Bluetooth_Available(void);

/**
 * @brief  读取一个字节
 * @return 读取的字节 (0~255), 无数据时返回-1
 */
int16_t Bluetooth_Read(void);

/**
 * @brief  读取一行 (以\n结尾)
 * @param  buf      输出缓冲区
 * @param  max_len  最大长度
 * @return 读取的字节数, 无完整行返回0
 */
uint16_t Bluetooth_ReadLine(char *buf, uint16_t max_len);

/**
 * @brief  获取蓝牙状态
 * @return 当前状态
 */
BT_State_t Bluetooth_GetState(void);

/**
 * @brief  清空接收缓冲区
 */
void Bluetooth_Flush(void);

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_TM4C_H */
