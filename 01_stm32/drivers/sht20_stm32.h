/**
 * @file    sht20_stm32.h
 * @brief   SHT20 温湿度传感器驱动 — STM32 HAL库版本（I2C）
 * @details 精度: ±0.3°C (温度), ±1.8%RH (湿度)
 *          I2C地址固定 0x40
 * @version 1.0
 * @date    2026-06
 */

#ifndef __SHT20_STM32_H
#define __SHT20_STM32_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── SHT20 I2C 地址 ─────────────────────────────────────── */
#define SHT20_ADDR              (0x40)

/* ── SHT20 命令 ──────────────────────────────────────────── */
#define SHT20_CMD_TRIG_T_HM     (0xE3)  /* 触发温度测量，保持主机 */
#define SHT20_CMD_TRIG_RH_HM    (0xE5)  /* 触发湿度测量，保持主机 */
#define SHT20_CMD_TRIG_T_NHM    (0xF3)  /* 触发温度测量，不保持主机 */
#define SHT20_CMD_TRIG_RH_NHM   (0xF5)  /* 触发湿度测量，不保持主机 */
#define SHT20_CMD_SOFT_RESET    (0xFE)  /* 软复位 */
#define SHT20_CMD_WRITE_REG     (0xE6)  /* 写用户寄存器 */
#define SHT20_CMD_READ_REG      (0xE7)  /* 读用户寄存器 */

/* ── SHT20 数据结构 ──────────────────────────────────────── */
typedef struct {
    float temperature;      /* 温度 (°C) */
    float humidity;         /* 相对湿度 (%RH) */
} SHT20_Data_t;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化SHT20
 * @param hi2c  I2C句柄指针
 * @return true=成功
 */
bool SHT20_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief 读取温度（°C）
 * @param temp  输出温度值
 * @return true=成功
 */
bool SHT20_ReadTemperature(float *temp);

/**
 * @brief 读取湿度（%RH）
 * @param humi  输出湿度值
 * @return true=成功
 */
bool SHT20_ReadHumidity(float *humi);

/**
 * @brief 同时读取温湿度
 * @param data  输出数据结构
 * @return true=成功
 */
bool SHT20_ReadAll(SHT20_Data_t *data);

/**
 * @brief 软复位
 * @return true=成功
 */
bool SHT20_SoftReset(void);

#endif /* __SHT20_STM32_H */
