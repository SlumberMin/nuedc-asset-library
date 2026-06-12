/**
 * @file    at24c02_stm32.h
 * @brief   AT24C02 EEPROM 驱动 — STM32 HAL库版本（I2C）
 * @details 2Kbit (256字节) I2C EEPROM，页大小8字节。
 * @version 1.0
 * @date    2026-06
 */

#ifndef __AT24C02_STM32_H
#define __AT24C02_STM32_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── AT24C02 参数 ───────────────────────────────────────── */
#define AT24C02_ADDR            (0x50)  /* A0=A1=A2=GND时的默认地址 */
#define AT24C02_SIZE            (256)   /* 总容量256字节 */
#define AT24C02_PAGE_SIZE       (8)     /* 页大小8字节 */

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化AT24C02驱动
 * @param hi2c  I2C句柄指针
 * @param addr  I2C 7位地址（默认0x50）
 */
void AT24C02_Init(I2C_HandleTypeDef *hi2c, uint8_t addr);

/**
 * @brief 读取单字节
 * @param mem_addr  存储地址 0~255
 * @param val       输出值
 * @return true=成功
 */
bool AT24C02_ReadByte(uint8_t mem_addr, uint8_t *val);

/**
 * @brief 写入单字节
 * @param mem_addr  存储地址 0~255
 * @param val       写入值
 * @return true=成功
 */
bool AT24C02_WriteByte(uint8_t mem_addr, uint8_t val);

/**
 * @brief 读取多个字节
 * @param mem_addr  起始地址
 * @param buf       读取缓冲区
 * @param len       字节数
 * @return true=成功
 */
bool AT24C02_Read(uint8_t mem_addr, uint8_t *buf, uint16_t len);

/**
 * @brief 写入多个字节（自动处理跨页）
 * @param mem_addr  起始地址
 * @param buf       写入数据
 * @param len       字节数
 * @return true=全部成功
 */
bool AT24C02_Write(uint8_t mem_addr, const uint8_t *buf, uint16_t len);

/**
 * @brief 检测EEPROM是否就绪（ACK轮询）
 * @return true=就绪
 */
bool AT24C02_IsReady(void);

#endif /* __AT24C02_STM32_H */
