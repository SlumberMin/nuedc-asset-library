/**
 * @file    tcs34725_stm32.h
 * @brief   TCS34725 颜色传感器 I2C 驱动 — STM32 HAL库版本
 *
 * 硬件连接:
 *   I2C1: PB6=SCL, PB7=SDA (与OLED共享I2C总线)
 *   TCS34725 I2C地址: 0x29
 *
 * 寄存器映射 (带命令位 0x80):
 *   ENABLE  (0x00) — ADC使能
 *   CDATAL  (0x14) — Clear通道低字节
 *   CDATAH  (0x15) — Clear通道高字节
 *   RDATAL  (0x16) — Red通道低字节
 *   RDATAH  (0x17) — Red通道高字节
 *   GDATAL  (0x18) — Green通道低字节
 *   GDATAH  (0x19) — Green通道高字节
 *   BDATAL  (0x1A) — Blue通道低字节
 *   BDATAH  (0x1B) — Blue通道高字节
 */

#ifndef __TCS34725_STM32_H
#define __TCS34725_STM32_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── TCS34725 I2C 地址 ──────────────────────────────────── */
#define TCS34725_ADDR           (0x29)

/* ── TCS34725 寄存器地址 (带命令位 0x80) ────────────────── */
#define TCS34725_REG_ENABLE     (0x00)
#define TCS34725_REG_CDATAL     (0x14)
#define TCS34725_REG_CDATAH     (0x15)
#define TCS34725_REG_RDATAL     (0x16)
#define TCS34725_REG_RDATAH     (0x17)
#define TCS34725_REG_GDATAL     (0x18)
#define TCS34725_REG_GDATAH     (0x19)
#define TCS34725_REG_BDATAL     (0x1A)
#define TCS34725_REG_BDATAH     (0x1B)

/* ── TCS34725 ENABLE 寄存器位 ───────────────────────────── */
#define TCS34725_ENABLE_PON     (0x01)   /* 上电 */
#define TCS34725_ENABLE_AEN     (0x02)   /* ADC使能 */

/* ── 命令字节 ────────────────────────────────────────────── */
#define TCS34725_CMD_BIT        (0x80)   /* 命令位 */
#define TCS34725_CMD_AUTO_INC   (0xA0)   /* 命令+自动递增 */

/* ── RGBC 数据结构 ───────────────────────────────────────── */
typedef struct {
    uint16_t clear;    /* Clear (透明) 通道 */
    uint16_t red;      /* Red 通道 */
    uint16_t green;    /* Green 通道 */
    uint16_t blue;     /* Blue 通道 */
} TCS34725_RGBC;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化 TCS34725
 *        上电并使能ADC
 * @param hi2c I2C句柄指针 (I2C1, PB6=SCL, PB7=SDA)
 * @return true=成功, false=I2C通信失败
 */
bool TCS34725_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief 读取 RGBC 四通道数据
 * @param data  输出: RGBC数据结构指针
 * @return true=成功, false=I2C通信失败
 */
bool TCS34725_ReadRGBC(TCS34725_RGBC *data);

/**
 * @brief 写 TCS34725 寄存器
 * @param reg   寄存器地址 (不含命令位)
 * @param val   写入值
 * @return true=成功, false=失败
 */
bool TCS34725_WriteReg(uint8_t reg, uint8_t val);

/**
 * @brief 读 TCS34725 寄存器
 * @param reg   寄存器地址 (不含命令位)
 * @param val   输出: 读取的值
 * @return true=成功, false=失败
 */
bool TCS34725_ReadReg(uint8_t reg, uint8_t *val);

#endif /* __TCS34725_STM32_H */
