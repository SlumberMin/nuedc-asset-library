/**
 * @file    at24c02.c
 * @brief   AT24C02 EEPROM I2C驱动实现 — MSPM0G3507
 *
 * SysConfig生成的宏:
 *   I2C_0_INST
 *
 * AT24C02写入协议:
 *   写1字节: Start + Addr(W) + MemAddr + Data + Stop → 等5ms
 *   读1字节: Start + Addr(W) + MemAddr + Stop + Start + Addr(R) + 1字节 + Stop
 *   页写入:  Start + Addr(W) + MemAddr + Data[0..N] + Stop → 等5ms (N≤7,不跨页)
 */

#include "drivers/at24c02.h"

/* ── I2C 实例 (由SysConfig生成) ──────────────────────────── */
#define EEPROM_I2C     I2C_0_INST

/* ── I2C超时计数 ─────────────────────────────────────────── */
#ifndef I2C_TIMEOUT_COUNT
#define I2C_TIMEOUT_COUNT   100000
#endif

/* ── 等待写入周期完成 (ms级延迟) ──────────────────────────── */
static void WriteWait(void)
{
    /* 简单阻塞延时: 5ms @ 32MHz = 160000 cycles */
    delay_cycles(160000);
}

/* ── 内部: 等待 I2C 空闲 ─────────────────────────────────── */
static bool WaitIdle(void)
{
    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while (!(DL_I2C_getControllerStatus(EEPROM_I2C) & DL_I2C_CONTROLLER_STATUS_IDLE)
           && --timeout)
        ;
    return (timeout != 0);
}

/* ── 内部: 等待传输完成 ──────────────────────────────────── */
static bool WaitBusy(void)
{
    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while ((DL_I2C_getControllerStatus(EEPROM_I2C) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
           && --timeout)
        ;
    return (timeout != 0);
}

/* ── 写入单个字节 ────────────────────────────────────────── */
bool AT24C02_WriteByte(uint8_t addr, uint8_t data)
{
    uint8_t txBuf[2];

    txBuf[0] = addr;    /* 内存地址 */
    txBuf[1] = data;    /* 数据 */

    /* 填充TX FIFO */
    DL_I2C_fillControllerTXFIFO(EEPROM_I2C, txBuf, 2);

    if (!WaitIdle()) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
        return false;
    }

    /* 发送: Start + Addr(W) + 2字节 + Stop */
    DL_I2C_startControllerTransfer(EEPROM_I2C, AT24C02_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    if (!WaitBusy()) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
        return false;
    }

    /* 检查错误 */
    if (DL_I2C_getControllerStatus(EEPROM_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
        return false;
    }

    /* 清空TX FIFO */
    DL_I2C_flushControllerTXFIFO(EEPROM_I2C);

    /* 等待EEPROM写入周期完成 */
    WriteWait();

    return true;
}

/* ── 读取单个字节 ────────────────────────────────────────── */
bool AT24C02_ReadByte(uint8_t addr, uint8_t *data)
{
    uint8_t txBuf[1];

    txBuf[0] = addr;    /* 内存地址 */

    /* 阶段1: 发送内存地址 */
    DL_I2C_fillControllerTXFIFO(EEPROM_I2C, txBuf, 1);

    if (!WaitIdle()) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
        return false;
    }

    DL_I2C_startControllerTransfer(EEPROM_I2C, AT24C02_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    if (!WaitBusy()) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
        return false;
    }

    if (DL_I2C_getControllerStatus(EEPROM_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
    delay_cycles(100);

    /* 阶段2: 读1字节 */
    if (!WaitIdle()) return false;

    DL_I2C_startControllerTransfer(EEPROM_I2C, AT24C02_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1);

    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while (DL_I2C_isControllerRXFIFOEmpty(EEPROM_I2C) && --timeout)
        ;
    if (timeout == 0) return false;
    *data = (uint8_t)DL_I2C_receiveControllerData(EEPROM_I2C);

    return true;
}

/* ── 页写入 (不跨页) ─────────────────────────────────────── */
bool AT24C02_WritePage(uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t txBuf[9];   /* 1字节地址 + 最多8字节数据 */

    if (len == 0 || len > AT24C02_PAGE_SIZE) return false;

    /* 首字节为内存地址 */
    txBuf[0] = addr;
    for (uint8_t i = 0; i < len; i++) {
        txBuf[1 + i] = data[i];
    }

    /* 填充TX FIFO */
    DL_I2C_fillControllerTXFIFO(EEPROM_I2C, txBuf, 1 + len);

    if (!WaitIdle()) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
        return false;
    }

    /* 发送: Start + Addr(W) + (1+len)字节 + Stop */
    DL_I2C_startControllerTransfer(EEPROM_I2C, AT24C02_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1 + len);

    if (!WaitBusy()) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
        return false;
    }

    if (DL_I2C_getControllerStatus(EEPROM_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(EEPROM_I2C);

    /* 等待EEPROM写入周期完成 */
    WriteWait();

    return true;
}

/* ── 写入多个字节 (自动处理跨页) ─────────────────────────── */
bool AT24C02_Write(uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t written = 0;

    while (written < len) {
        /* 计算当前页剩余空间 */
        uint8_t pageRemain = AT24C02_PAGE_SIZE - ((addr + written) % AT24C02_PAGE_SIZE);
        uint8_t chunk = len - written;

        if (chunk > pageRemain) {
            chunk = pageRemain;
        }

        if (!AT24C02_WritePage(addr + written, data + written, chunk)) {
            return false;
        }

        written += chunk;
    }

    return true;
}

/* ── 读取多个字节 ────────────────────────────────────────── */
bool AT24C02_Read(uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t txBuf[1];

    if (len == 0) return true;

    /* 阶段1: 发送起始内存地址 */
    txBuf[0] = addr;

    DL_I2C_fillControllerTXFIFO(EEPROM_I2C, txBuf, 1);

    if (!WaitIdle()) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
        return false;
    }

    DL_I2C_startControllerTransfer(EEPROM_I2C, AT24C02_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    if (!WaitBusy()) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
        return false;
    }

    if (DL_I2C_getControllerStatus(EEPROM_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
    delay_cycles(100);

    /* 阶段2: 连续读取 len 字节 */
    if (!WaitIdle()) return false;

    DL_I2C_startControllerTransfer(EEPROM_I2C, AT24C02_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);

    for (uint8_t i = 0; i < len; i++) {
        uint32_t timeout = I2C_TIMEOUT_COUNT;
        while (DL_I2C_isControllerRXFIFOEmpty(EEPROM_I2C) && --timeout)
            ;
        if (timeout == 0) return false;
        data[i] = (uint8_t)DL_I2C_receiveControllerData(EEPROM_I2C);
    }

    return true;
}

/* ── 设备就绪检测 ────────────────────────────────────────── */
bool AT24C02_IsReady(void)
{
    uint8_t dummy;

    /* 发送地址字节, 检查是否有ACK */
    DL_I2C_flushControllerTXFIFO(EEPROM_I2C);

    DL_I2C_fillControllerTXFIFO(EEPROM_I2C, &dummy, 0);

    if (!WaitIdle()) return false;

    DL_I2C_startControllerTransfer(EEPROM_I2C, AT24C02_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 0);

    if (!WaitBusy()) return false;

    bool error = (DL_I2C_getControllerStatus(EEPROM_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0;

    if (error) {
        DL_I2C_flushControllerTXFIFO(EEPROM_I2C);
    }

    return !error;
}
