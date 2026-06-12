/**
 * @file    at24c02.h
 * @brief   AT24C02 EEPROM I2C驱动 — MSPM0G3507
 *
 * 硬件连接:
 *   I2C0: PB2=SCL, PB3=SDA
 *   AT24C02 I2C地址: 0x50 (A2=A1=A0=0)
 *
 * AT24C02参数:
 *   容量: 256字节 (地址 0x00~0xFF)
 *   页大小: 8字节
 *   写入周期: 最大5ms
 *
 * 依赖: ti_msp_dl_config.h (SysConfig生成, 需配置I2C_0)
 */

#ifndef __AT24C02_H
#define __AT24C02_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ── AT24C02 参数 ─────────────────────────────────────────── */
#define AT24C02_ADDR            (0x50)      /* I2C从机地址 */
#define AT24C02_SIZE            (256)       /* 总容量(字节) */
#define AT24C02_PAGE_SIZE       (8)         /* 页大小(字节) */
#define AT24C02_WRITE_CYCLE_MS  (5)         /* 写入周期(ms) */

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 写入单个字节到EEPROM
 * @param addr  字节地址 0x00~0xFF
 * @param data  写入的数据
 * @return true=成功, false=I2C通信失败
 */
bool AT24C02_WriteByte(uint8_t addr, uint8_t data);

/**
 * @brief 从EEPROM读取单个字节
 * @param addr  字节地址 0x00~0xFF
 * @param data  输出: 读取的数据
 * @return true=成功, false=I2C通信失败
 */
bool AT24C02_ReadByte(uint8_t addr, uint8_t *data);

/**
 * @brief 写入多个字节 (自动处理跨页)
 * @param addr   起始地址 0x00~0xFF
 * @param data   数据缓冲区
 * @param len    数据长度
 * @return true=成功, false=失败
 *
 * @note  自动处理页边界, 每页写入后等待5ms
 */
bool AT24C02_Write(uint8_t addr, const uint8_t *data, uint8_t len);

/**
 * @brief 从EEPROM读取多个字节
 * @param addr   起始地址 0x00~0xFF
 * @param data   输出缓冲区
 * @param len    读取长度
 * @return true=成功, false=失败
 */
bool AT24C02_Read(uint8_t addr, uint8_t *data, uint8_t len);

/**
 * @brief 写入一整页 (不超过页边界)
 * @param addr   起始地址 (应页对齐, 或确保不跨页)
 * @param data   数据缓冲区
 * @param len    数据长度 (1~8)
 * @return true=成功, false=失败
 *
 * @note  调用者需确保 addr~addr+len-1 不跨越页边界
 *        写入后需等待 AT24C02_WRITE_CYCLE_MS
 */
bool AT24C02_WritePage(uint8_t addr, const uint8_t *data, uint8_t len);

/**
 * @brief 检测AT24C02是否在线
 * @return true=设备应答, false=无应答
 */
bool AT24C02_IsReady(void);

#endif /* __AT24C02_H */
