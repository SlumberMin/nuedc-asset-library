/**
 * @file    pca9685.c
 * @brief   PCA9685 16路PWM舵机驱动板 I2C驱动实现 — MSPM0G3507
 *
 * SysConfig生成的宏:
 *   I2C_0_INST
 */

#include "drivers/pca9685.h"

/* ── I2C 实例 (由SysConfig生成) ─────────────────────────────── */
#define PCA_I2C     I2C_0_INST

/* ── I2C超时计数 ─────────────────────────────────────────── */
#ifndef I2C_TIMEOUT_COUNT
#define I2C_TIMEOUT_COUNT   100000
#endif

/* ── 内部: 等待 I2C 空闲 ────────────────────────────────────── */
static bool PCA9685_WaitIdle(void)
{
    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while (!(DL_I2C_getControllerStatus(PCA_I2C) & DL_I2C_CONTROLLER_STATUS_IDLE)
           && --timeout)
        ;
    return (timeout != 0);
}

/* ── 内部: 等待传输完成 ─────────────────────────────────────── */
static bool PCA9685_WaitBusy(void)
{
    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while ((DL_I2C_getControllerStatus(PCA_I2C) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
           && --timeout)
        ;
    return (timeout != 0);
}

/* ── 写寄存器 ───────────────────────────────────────────────── */
bool PCA9685_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t txBuf[2];

    txBuf[0] = reg;
    txBuf[1] = val;

    DL_I2C_flushControllerTXFIFO(PCA_I2C);
    DL_I2C_fillControllerTXFIFO(PCA_I2C, txBuf, 2);

    if (!PCA9685_WaitIdle()) {
        DL_I2C_flushControllerTXFIFO(PCA_I2C);
        return false;
    }

    DL_I2C_startControllerTransfer(PCA_I2C, PCA9685_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    if (!PCA9685_WaitBusy()) {
        DL_I2C_flushControllerTXFIFO(PCA_I2C);
        return false;
    }

    if (DL_I2C_getControllerStatus(PCA_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(PCA_I2C);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(PCA_I2C);
    return true;
}

/* ── 读寄存器 (单字节) ──────────────────────────────────────── */
bool PCA9685_ReadReg(uint8_t reg, uint8_t *val)
{
    uint8_t txBuf[1];

    txBuf[0] = reg;

    DL_I2C_flushControllerTXFIFO(PCA_I2C);
    DL_I2C_fillControllerTXFIFO(PCA_I2C, txBuf, 1);

    if (!PCA9685_WaitIdle()) {
        DL_I2C_flushControllerTXFIFO(PCA_I2C);
        return false;
    }

    DL_I2C_startControllerTransfer(PCA_I2C, PCA9685_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    if (!PCA9685_WaitBusy()) {
        DL_I2C_flushControllerTXFIFO(PCA_I2C);
        return false;
    }

    if (DL_I2C_getControllerStatus(PCA_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(PCA_I2C);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(PCA_I2C);
    delay_cycles(100);

    /* 阶段2: 读1字节 */
    if (!PCA9685_WaitIdle()) return false;

    DL_I2C_startControllerTransfer(PCA_I2C, PCA9685_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1);

    uint32_t timeout = I2C_TIMEOUT_COUNT;
    while (DL_I2C_isControllerRXFIFOEmpty(PCA_I2C) && --timeout)
        ;
    if (timeout == 0) return false;
    *val = (uint8_t)DL_I2C_receiveControllerData(PCA_I2C);

    return true;
}

/* ── 初始化 ─────────────────────────────────────────────────── */
bool PCA9685_Init(void)
{
    uint8_t mode1;

    /* 软复位: 写0x00到MODE1 */
    if (!PCA9685_WriteReg(PCA9685_REG_MODE1, 0x00)) {
        return false;
    }
    delay_cycles(16000);  /* ~0.5ms @32MHz */

    /* 读取MODE1确认 */
    if (!PCA9685_ReadReg(PCA9685_REG_MODE1, &mode1)) {
        return false;
    }

    /* 配置MODE2: 推挽输出 */
    if (!PCA9685_WriteReg(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV)) {
        return false;
    }

    /* 设置50Hz舵机频率 */
    if (!PCA9685_SetPWMFreq(50)) {
        return false;
    }

    /* 唤醒: 清除SLEEP位, 设置自动递增 */
    if (!PCA9685_WriteReg(PCA9685_REG_MODE1,
            PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL)) {
        return false;
    }
    delay_cycles(16000);  /* 等待振荡器稳定 */

    return true;
}

/* ── 设置PWM频率 ────────────────────────────────────────────── */
bool PCA9685_SetPWMFreq(uint16_t freq_hz)
{
    uint8_t mode1;
    uint32_t prescale;

    if (freq_hz < 24) freq_hz = 24;
    if (freq_hz > 1526) freq_hz = 1526;

    /* prescale = round(25MHz / (4096 * freq)) - 1 */
    prescale = (PCA9685_OSC_FREQ + (PCA9685_PWM_STEPS / 2) * freq_hz) /
               (PCA9685_PWM_STEPS * freq_hz) - 1;

    if (prescale < 3) prescale = 3;

    /* 进入睡眠模式才能修改PRE_SCALE */
    if (!PCA9685_ReadReg(PCA9685_REG_MODE1, &mode1)) {
        return false;
    }

    if (!PCA9685_WriteReg(PCA9685_REG_MODE1,
            (mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP)) {
        return false;
    }

    if (!PCA9685_WriteReg(PCA9685_REG_PRE_SCALE, (uint8_t)prescale)) {
        return false;
    }

    /* 恢复MODE1 */
    if (!PCA9685_WriteReg(PCA9685_REG_MODE1, mode1)) {
        return false;
    }
    delay_cycles(16000);

    /* 使能RESTART */
    if (!PCA9685_WriteReg(PCA9685_REG_MODE1, mode1 | PCA9685_MODE1_RESTART)) {
        return false;
    }

    return true;
}

/* ── 设置单通道PWM值 ────────────────────────────────────────── */
bool PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    uint8_t reg;
    uint8_t txBuf[5];

    if (channel > 15) return false;

    /* LED0_ON_L + channel*4 */
    reg = PCA9685_REG_LED0_ON_L + (channel << 2);

    txBuf[0] = reg;
    txBuf[1] = (uint8_t)(on & 0xFF);        /* ON_L */
    txBuf[2] = (uint8_t)((on >> 8) & 0x0F); /* ON_H */
    txBuf[3] = (uint8_t)(off & 0xFF);        /* OFF_L */
    txBuf[4] = (uint8_t)((off >> 8) & 0x0F);/* OFF_H */

    DL_I2C_flushControllerTXFIFO(PCA_I2C);
    DL_I2C_fillControllerTXFIFO(PCA_I2C, txBuf, 5);

    if (!PCA9685_WaitIdle()) {
        DL_I2C_flushControllerTXFIFO(PCA_I2C);
        return false;
    }

    DL_I2C_startControllerTransfer(PCA_I2C, PCA9685_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 5);

    if (!PCA9685_WaitBusy()) {
        DL_I2C_flushControllerTXFIFO(PCA_I2C);
        return false;
    }

    if (DL_I2C_getControllerStatus(PCA_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(PCA_I2C);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(PCA_I2C);
    return true;
}

/* ── 设置舵机角度 ───────────────────────────────────────────── */
bool PCA9685_SetAngle(uint8_t channel, uint16_t angle)
{
    uint16_t pulse;

    if (angle > 180) angle = 180;

    /*
     * 脉宽映射 (50Hz, 20ms周期):
     *   0°   → 0.5ms → 0.5/20 * 4096 = 102
     *   90°  → 1.5ms → 1.5/20 * 4096 = 307
     *   180° → 2.5ms → 2.5/20 * 4096 = 512
     *
     * 线性插值: pulse = 102 + angle * (512 - 102) / 180
     */
    pulse = (uint16_t)(102 + (uint32_t)angle * 410 / 180);

    return PCA9685_SetPWM(channel, 0, pulse);
}

/* ── 关闭所有通道 ───────────────────────────────────────────── */
bool PCA9685_AllOff(void)
{
    uint8_t txBuf[5];

    txBuf[0] = PCA9685_REG_ALL_LED_ON_L;
    txBuf[1] = 0x00;  /* ALL_LED_ON_L */
    txBuf[2] = 0x00;  /* ALL_LED_ON_H */
    txBuf[3] = 0x00;  /* ALL_LED_OFF_L */
    txBuf[4] = 0x10;  /* ALL_LED_OFF_H bit4=1 → full OFF */

    DL_I2C_flushControllerTXFIFO(PCA_I2C);
    DL_I2C_fillControllerTXFIFO(PCA_I2C, txBuf, 5);

    if (!PCA9685_WaitIdle()) {
        DL_I2C_flushControllerTXFIFO(PCA_I2C);
        return false;
    }

    DL_I2C_startControllerTransfer(PCA_I2C, PCA9685_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 5);

    if (!PCA9685_WaitBusy()) {
        DL_I2C_flushControllerTXFIFO(PCA_I2C);
        return false;
    }

    if (DL_I2C_getControllerStatus(PCA_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(PCA_I2C);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(PCA_I2C);
    return true;
}
