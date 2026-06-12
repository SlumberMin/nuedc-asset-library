/**
 * @file    tcs34725.c
 * @brief   TCS34725 颜色传感器 I2C 驱动实现 — MSPM0G3507
 *
 * SysConfig生成的宏:
 *   I2C_0_INST
 */

#include "drivers/tcs34725.h"

/* ── I2C 实例 (由SysConfig生成) ──────────────────────────── */
#define TCS_I2C     I2C_0_INST

/* ── I2C超时计数 ─────────────────────────────────────────── */
#ifndef I2C_TIMEOUT_COUNT
#define I2C_TIMEOUT_COUNT   100000
#endif

/* ── 内部: 等待 I2C 空闲 ─────────────────────────────────── */
static bool WaitIdle(void)
{
    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while (!(DL_I2C_getControllerStatus(TCS_I2C) & DL_I2C_CONTROLLER_STATUS_IDLE)
           && --timeout)
        ;
    return (timeout != 0);
}

/* ── 内部: 等待传输完成 ──────────────────────────────────── */
static bool WaitBusy(void)
{
    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while ((DL_I2C_getControllerStatus(TCS_I2C) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
           && --timeout)
        ;
    return (timeout != 0);
}

/* ── 写寄存器 ────────────────────────────────────────────── */
bool TCS34725_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t txBuf[2];

    txBuf[0] = TCS34725_CMD_BIT | reg;   /* 命令字节 + 寄存器地址 */
    txBuf[1] = val;

    /* 填充TX FIFO */
    DL_I2C_fillControllerTXFIFO(TCS_I2C, txBuf, 2);

    if (!WaitIdle()) {
        DL_I2C_flushControllerTXFIFO(TCS_I2C);
        return false;
    }

    /* 发送: Start + Addr(W) + 2字节 + Stop */
    DL_I2C_startControllerTransfer(TCS_I2C, TCS34725_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    if (!WaitBusy()) {
        DL_I2C_flushControllerTXFIFO(TCS_I2C);
        return false;
    }

    /* 检查错误 */
    if (DL_I2C_getControllerStatus(TCS_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(TCS_I2C);
        return false;
    }

    /* 清空TX FIFO */
    DL_I2C_flushControllerTXFIFO(TCS_I2C);

    return true;
}

/* ── 读寄存器 (单字节) ───────────────────────────────────── */
bool TCS34725_ReadReg(uint8_t reg, uint8_t *val)
{
    uint8_t txBuf[1];

    txBuf[0] = TCS34725_CMD_BIT | reg;   /* 命令字节 + 寄存器地址 */

    /* 阶段1: 写寄存器地址 */
    DL_I2C_fillControllerTXFIFO(TCS_I2C, txBuf, 1);

    if (!WaitIdle()) {
        DL_I2C_flushControllerTXFIFO(TCS_I2C);
        return false;
    }

    DL_I2C_startControllerTransfer(TCS_I2C, TCS34725_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    if (!WaitBusy()) {
        DL_I2C_flushControllerTXFIFO(TCS_I2C);
        return false;
    }

    if (DL_I2C_getControllerStatus(TCS_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(TCS_I2C);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(TCS_I2C);
    delay_cycles(100);

    /* 阶段2: 读1字节 */
    if (!WaitIdle()) return false;

    DL_I2C_startControllerTransfer(TCS_I2C, TCS34725_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1);

    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while (DL_I2C_isControllerRXFIFOEmpty(TCS_I2C) && --timeout)
        ;
    if (timeout == 0) return false;
    *val = (uint8_t)DL_I2C_receiveControllerData(TCS_I2C);

    return true;
}

/* ── 初始化 ──────────────────────────────────────────────── */
bool TCS34725_Init(void)
{
    /* 上电 */
    if (!TCS34725_WriteReg(TCS34725_REG_ENABLE, TCS34725_ENABLE_PON)) {
        return false;
    }
    delay_cycles(32000);  /* 等待约1ms @32MHz */

    /* 使能ADC */
    if (!TCS34725_WriteReg(TCS34725_REG_ENABLE,
            TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)) {
        return false;
    }
    delay_cycles(320000);  /* 等待ADC稳定 ~10ms */

    return true;
}

/* ── 读取 RGBC 四通道 ───────────────────────────────────── */
bool TCS34725_ReadRGBC(TCS34725_RGBC *data)
{
    uint8_t txBuf[1];
    uint8_t rxBuf[8];

    /* 阶段1: 发送起始寄存器地址 (CDATAL, 自动递增模式) */
    txBuf[0] = TCS34725_CMD_AUTO_INC | TCS34725_REG_CDATAL;

    DL_I2C_fillControllerTXFIFO(TCS_I2C, txBuf, 1);

    if (!WaitIdle()) {
        DL_I2C_flushControllerTXFIFO(TCS_I2C);
        return false;
    }

    DL_I2C_startControllerTransfer(TCS_I2C, TCS34725_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    if (!WaitBusy()) {
        DL_I2C_flushControllerTXFIFO(TCS_I2C);
        return false;
    }

    if (DL_I2C_getControllerStatus(TCS_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(TCS_I2C);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(TCS_I2C);
    delay_cycles(100);

    /* 阶段2: 连续读取8字节 (C_L, C_H, R_L, R_H, G_L, G_H, B_L, B_H) */
    if (!WaitIdle()) return false;

    DL_I2C_startControllerTransfer(TCS_I2C, TCS34725_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, 8);

    for (uint8_t i = 0; i < 8; i++) {
        uint32_t timeout = I2C_TIMEOUT_COUNT;
        while (DL_I2C_isControllerRXFIFOEmpty(TCS_I2C) && --timeout)
            ;
        if (timeout == 0) return false;
        rxBuf[i] = (uint8_t)DL_I2C_receiveControllerData(TCS_I2C);
    }

    /* 组合16位数据 (低字节在前)
     * BugFix: 运算符优先级 — (uint16_t)(rxBuf[1] << 8) | rxBuf[0]
     *   rxBuf[1]<<8 在int(32位)上计算，高字节>=0x80时结果为0xFFFFxx00
     *   先cast到uint16_t会截断为0x__00，再OR低字节=错误值0x__xx(正数)
     * 正确做法: 先组合为uint32_t再截断 */
    data->clear = (uint16_t)((uint32_t)rxBuf[1] << 8 | rxBuf[0]);
    data->red   = (uint16_t)((uint32_t)rxBuf[3] << 8 | rxBuf[2]);
    data->green = (uint16_t)((uint32_t)rxBuf[5] << 8 | rxBuf[4]);
    data->blue  = (uint16_t)((uint32_t)rxBuf[7] << 8 | rxBuf[6]);

    return true;
}
