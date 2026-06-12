/**
 * @file    tcs34725_stm32.c
 * @brief   TCS34725 颜色传感器 I2C 驱动实现 — STM32 HAL库版本
 *
 * 使用 STM32 HAL I2C 库进行通信。
 * 与 OLED 共享 I2C1 总线 (PB6=SCL, PB7=SDA)。
 */

#include "drivers/tcs34725_stm32.h"

/* ── 内部变量 ─────────────────────────────────────────────── */
static I2C_HandleTypeDef *g_tcs_hi2c = NULL;

/* ── 写寄存器 ─────────────────────────────────────────────── */
bool TCS34725_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t txBuf[2];
    txBuf[0] = TCS34725_CMD_BIT | reg;   /* 命令字节 + 寄存器地址 */
    txBuf[1] = val;

    if (HAL_I2C_Master_Transmit(g_tcs_hi2c, TCS34725_ADDR << 1,
                                txBuf, 2, 50 /* [修复#6] 原50→有限超时防止I2C锁死 */) != HAL_OK) {
        return false;
    }
    return true;
}

/* ── 读寄存器 (单字节) ────────────────────────────────────── */
bool TCS34725_ReadReg(uint8_t reg, uint8_t *val)
{
    uint8_t txBuf[1];
    txBuf[0] = TCS34725_CMD_BIT | reg;   /* 命令字节 + 寄存器地址 */

    /* 阶段1: 写寄存器地址 */
    if (HAL_I2C_Master_Transmit(g_tcs_hi2c, TCS34725_ADDR << 1,
                                txBuf, 1, 50 /* [修复#6] 原50→有限超时防止I2C锁死 */) != HAL_OK) {
        return false;
    }

    /* 阶段2: 读1字节 */
    if (HAL_I2C_Master_Receive(g_tcs_hi2c, TCS34725_ADDR << 1,
                               val, 1, 50 /* [修复#6] 原50→有限超时防止I2C锁死 */) != HAL_OK) {
        return false;
    }

    return true;
}

/* ── 初始化 ───────────────────────────────────────────────── */
bool TCS34725_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) {
        return false;
    }

    g_tcs_hi2c = hi2c;

    /* 上电 */
    if (!TCS34725_WriteReg(TCS34725_REG_ENABLE, TCS34725_ENABLE_PON)) {
        return false;
    }
    HAL_Delay(10);  /* 等待约10ms */

    /* 使能ADC */
    if (!TCS34725_WriteReg(TCS34725_REG_ENABLE,
            TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)) {
        return false;
    }
    HAL_Delay(10);  /* 等待ADC稳定 */

    return true;
}

/* ── 读取 RGBC 四通道 ────────────────────────────────────── */
bool TCS34725_ReadRGBC(TCS34725_RGBC *data)
{
    uint8_t txBuf[1];
    uint8_t rxBuf[8];

    /* 阶段1: 发送起始寄存器地址 (CDATAL, 自动递增模式) */
    txBuf[0] = TCS34725_CMD_AUTO_INC | TCS34725_REG_CDATAL;

    if (HAL_I2C_Master_Transmit(g_tcs_hi2c, TCS34725_ADDR << 1,
                                txBuf, 1, 50 /* [修复#6] 原50→有限超时防止I2C锁死 */) != HAL_OK) {
        return false;
    }

    /* 阶段2: 连续读取8字节 (C_L, C_H, R_L, R_H, G_L, G_H, B_L, B_H) */
    if (HAL_I2C_Master_Receive(g_tcs_hi2c, TCS34725_ADDR << 1,
                               rxBuf, 8, 50 /* [修复#6] 原50→有限超时防止I2C锁死 */) != HAL_OK) {
        return false;
    }

    /* 组合16位数据 (低字节在前) */
    data->clear = (uint16_t)(rxBuf[1] << 8) | rxBuf[0];
    data->red   = (uint16_t)(rxBuf[3] << 8) | rxBuf[2];
    data->green = (uint16_t)(rxBuf[5] << 8) | rxBuf[4];
    data->blue  = (uint16_t)(rxBuf[7] << 8) | rxBuf[6];

    return true;
}
