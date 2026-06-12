/**
 * @file    i2c_bus.c
 * @brief   I2C 总线管理层实现 — MSPM0G3507
 *
 * 实现要点:
 *   1. 软件互斥锁 (volatile bool)
 *   2. 每次传输带超时检测
 *   3. 传输失败自动 flush FIFO + 重试
 *   4. 统计计数器用于调试
 */

#include "drivers/i2c_bus.h"

/* ── 内部: 等待I2C空闲 ────────────────────────────────────── */

static bool WaitIdle(I2C_Regs *inst, uint32_t timeout)
{
    while (!(DL_I2C_getControllerStatus(inst) & DL_I2C_CONTROLLER_STATUS_IDLE)
           && --timeout)
        ;
    return (timeout != 0);
}

/* ── 内部: 等待传输完成 (BUSY释放) ─────────────────────────── */

static bool WaitBusyDone(I2C_Regs *inst, uint32_t timeout)
{
    while ((DL_I2C_getControllerStatus(inst) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
           && --timeout)
        ;
    return (timeout != 0);
}

/* ── 内部: 检查错误 ───────────────────────────────────────── */

static bool CheckError(I2C_Regs *inst)
{
    return (DL_I2C_getControllerStatus(inst) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0;
}

/* ── 内部: 恢复总线 ───────────────────────────────────────── */

static void BusRecover(I2C_Regs *inst)
{
    DL_I2C_flushControllerTXFIFO(inst);
    /* 短暂延时让总线释放 */
    delay_cycles(I2C_BUS_RETRY_DELAY);
}

/* ── 内部: 底层写操作 (不带锁, 不带重试) ───────────────────── */

static I2C_Bus_Error RawWrite(I2C_Bus *bus, uint8_t addr,
                               const uint8_t *buf, uint8_t len)
{
    I2C_Regs *inst = bus->inst;

    /* 填充TX FIFO */
    DL_I2C_flushControllerTXFIFO(inst);
    DL_I2C_fillControllerTXFIFO(inst, (uint8_t *)buf, len);

    /* 等待空闲 */
    if (!WaitIdle(inst, bus->timeout)) {
        BusRecover(inst);
        return I2C_BUS_ERR_TIMEOUT;
    }

    /* 启动传输 */
    DL_I2C_startControllerTransfer(inst, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, len);

    /* 等待完成 */
    if (!WaitBusyDone(inst, bus->timeout)) {
        BusRecover(inst);
        return I2C_BUS_ERR_TIMEOUT;
    }

    /* 检查错误 */
    if (CheckError(inst)) {
        BusRecover(inst);
        return I2C_BUS_ERR_NACK;
    }

    DL_I2C_flushControllerTXFIFO(inst);
    return I2C_BUS_OK;
}

/* ── 内部: 底层读操作 (不带锁, 不带重试) ────────────────────── */

static I2C_Bus_Error RawRead(I2C_Bus *bus, uint8_t addr,
                              uint8_t *buf, uint8_t len)
{
    I2C_Regs *inst = bus->inst;

    /* 等待空闲 */
    if (!WaitIdle(inst, bus->timeout)) {
        return I2C_BUS_ERR_TIMEOUT;
    }

    /* 启动读传输 */
    DL_I2C_startControllerTransfer(inst, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);

    /* 等待完成 */
    if (!WaitBusyDone(inst, bus->timeout)) {
        BusRecover(inst);
        return I2C_BUS_ERR_TIMEOUT;
    }

    /* 检查错误 */
    if (CheckError(inst)) {
        BusRecover(inst);
        return I2C_BUS_ERR_NACK;
    }

    /* 读取RX FIFO */
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = DL_I2C_receiveControllerData(inst);
    }

    return I2C_BUS_OK;
}

/* ── 内部: 带重试的写 ──────────────────────────────────────── */

static I2C_Bus_Error WriteWithRetry(I2C_Bus *bus, uint8_t addr,
                                     const uint8_t *buf, uint8_t len)
{
    I2C_Bus_Error err;
    uint8_t retry = 0;

    do {
        err = RawWrite(bus, addr, buf, len);
        if (err == I2C_BUS_OK) {
            bus->tx_count++;
            return I2C_BUS_OK;
        }
        retry++;
        bus->retry_count++;
        if (retry < bus->max_retry) {
            delay_cycles(I2C_BUS_RETRY_DELAY);
        }
    } while (retry < bus->max_retry);

    bus->err_count++;
    return err;
}

/* ── 内部: 带重试的读 ──────────────────────────────────────── */

static I2C_Bus_Error ReadWithRetry(I2C_Bus *bus, uint8_t addr,
                                    uint8_t *buf, uint8_t len)
{
    I2C_Bus_Error err;
    uint8_t retry = 0;

    do {
        err = RawRead(bus, addr, buf, len);
        if (err == I2C_BUS_OK) {
            bus->rx_count++;
            return I2C_BUS_OK;
        }
        retry++;
        bus->retry_count++;
        if (retry < bus->max_retry) {
            delay_cycles(I2C_BUS_RETRY_DELAY);
        }
    } while (retry < bus->max_retry);

    bus->err_count++;
    return err;
}

/* ═══════════════════════════════════════════════════════════════
 *  公开 API
 * ═══════════════════════════════════════════════════════════════ */

void I2C_Bus_Init(I2C_Bus *bus, I2C_Regs *inst)
{
    bus->inst        = inst;
    bus->locked      = false;
    bus->timeout     = I2C_BUS_TIMEOUT;
    bus->max_retry   = I2C_BUS_MAX_RETRY;
    bus->tx_count    = 0;
    bus->rx_count    = 0;
    bus->err_count   = 0;
    bus->retry_count = 0;
}

bool I2C_Bus_Lock(I2C_Bus *bus)
{
    if (bus->locked) {
        return false;   /* 已被占用 */
    }
    bus->locked = true;
    return true;
}

void I2C_Bus_Unlock(I2C_Bus *bus)
{
    bus->locked = false;
}

I2C_Bus_Error I2C_Bus_Write(I2C_Bus *bus, uint8_t addr,
                             const uint8_t *tx_buf, uint8_t tx_len)
{
    if (!I2C_Bus_Lock(bus)) {
        return I2C_BUS_ERR_BUSY;
    }

    I2C_Bus_Error err = WriteWithRetry(bus, addr, tx_buf, tx_len);

    I2C_Bus_Unlock(bus);
    return err;
}

I2C_Bus_Error I2C_Bus_Read(I2C_Bus *bus, uint8_t addr,
                            uint8_t *rx_buf, uint8_t rx_len)
{
    if (!I2C_Bus_Lock(bus)) {
        return I2C_BUS_ERR_BUSY;
    }

    I2C_Bus_Error err = ReadWithRetry(bus, addr, rx_buf, rx_len);

    I2C_Bus_Unlock(bus);
    return err;
}

I2C_Bus_Error I2C_Bus_WriteRead(I2C_Bus *bus, uint8_t addr,
                                 const uint8_t *tx_buf, uint8_t tx_len,
                                 uint8_t *rx_buf, uint8_t rx_len)
{
    if (!I2C_Bus_Lock(bus)) {
        return I2C_BUS_ERR_BUSY;
    }

    I2C_Bus_Error err;
    uint8_t retry = 0;

    do {
        /* 写阶段 */
        err = RawWrite(bus, addr, tx_buf, tx_len);
        if (err != I2C_BUS_OK) {
            retry++;
            bus->retry_count++;
            if (retry < bus->max_retry) {
                delay_cycles(I2C_BUS_RETRY_DELAY);
                continue;
            }
            bus->err_count++;
            I2C_Bus_Unlock(bus);
            return err;
        }

        /* 读阶段 */
        err = RawRead(bus, addr, rx_buf, rx_len);
        if (err == I2C_BUS_OK) {
            bus->tx_count++;
            bus->rx_count++;
            I2C_Bus_Unlock(bus);
            return I2C_BUS_OK;
        }

        retry++;
        bus->retry_count++;
        if (retry < bus->max_retry) {
            delay_cycles(I2C_BUS_RETRY_DELAY);
        }
    } while (retry < bus->max_retry);

    bus->err_count++;
    I2C_Bus_Unlock(bus);
    return err;
}

I2C_Bus_Error I2C_Bus_WriteReg(I2C_Bus *bus, uint8_t addr,
                                uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return I2C_Bus_Write(bus, addr, buf, 2);
}

I2C_Bus_Error I2C_Bus_ReadReg(I2C_Bus *bus, uint8_t addr,
                               uint8_t reg, uint8_t *val)
{
    return I2C_Bus_WriteRead(bus, addr, &reg, 1, val, 1);
}

I2C_Bus_Error I2C_Bus_ReadMulti(I2C_Bus *bus, uint8_t addr,
                                 uint8_t reg, uint8_t *buf, uint8_t len)
{
    return I2C_Bus_WriteRead(bus, addr, &reg, 1, buf, len);
}

void I2C_Bus_GetStats(const I2C_Bus *bus,
                       uint32_t *tx, uint32_t *rx,
                       uint32_t *err, uint32_t *retry)
{
    if (tx)    *tx    = bus->tx_count;
    if (rx)    *rx    = bus->rx_count;
    if (err)   *err   = bus->err_count;
    if (retry) *retry = bus->retry_count;
}

const char *I2C_Bus_ErrorStr(I2C_Bus_Error err)
{
    switch (err) {
    case I2C_BUS_OK:          return "OK";
    case I2C_BUS_ERR_BUSY:    return "BUSY (locked)";
    case I2C_BUS_ERR_TIMEOUT: return "TIMEOUT";
    case I2C_BUS_ERR_NACK:    return "NACK (no response)";
    case I2C_BUS_ERR_RETRY:   return "RETRY exhausted";
    default:                  return "UNKNOWN";
    }
}
