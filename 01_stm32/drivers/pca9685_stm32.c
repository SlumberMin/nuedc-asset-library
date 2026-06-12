/**
 * @file    pca9685_stm32.c
 * @brief   PCA9685 16路PWM舵机驱动板 实现 — STM32 HAL库版本
 */

#include "drivers/pca9685_stm32.h"

static I2C_HandleTypeDef *g_pca9685_hi2c = NULL;
static uint8_t g_pca9685_addr = PCA9685_ADDR_DEFAULT;
static uint16_t g_pca9685_pwm_freq = 50;  /* 默认50Hz，适合舵机 */

/* ── 内部: 写寄存器 ─────────────────────────────────────── */
static bool PCA9685_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(g_pca9685_hi2c, g_pca9685_addr << 1,
                                   buf, 2, HAL_MAX_DELAY) == HAL_OK;
}

/* ── 内部: 读寄存器 ─────────────────────────────────────── */
static bool PCA9685_ReadReg(uint8_t reg, uint8_t *val)
{
    if (HAL_I2C_Master_Transmit(g_pca9685_hi2c, g_pca9685_addr << 1,
                                &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return false;
    return HAL_I2C_Master_Receive(g_pca9685_hi2c, g_pca9685_addr << 1,
                                  val, 1, HAL_MAX_DELAY) == HAL_OK;
}

/* ── 初始化 ─────────────────────────────────────────────── */
bool PCA9685_Init(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    g_pca9685_hi2c = hi2c;
    g_pca9685_addr = addr;

    /* 软复位: RESTART + AI */
    if (!PCA9685_WriteReg(PCA9685_REG_MODE1, PCA9685_MODE1_RESTART | PCA9685_MODE1_AI))
        return false;
    HAL_Delay(5);

    /* 设置默认50Hz */
    return PCA9685_SetFreq(50);
}

/* ── 设置PWM频率 ────────────────────────────────────────── */
bool PCA9685_SetFreq(uint16_t freq_hz)
{
    if (freq_hz < 24) freq_hz = 24;
    if (freq_hz > 1526) freq_hz = 1526;

    /* prescale = round(25MHz / (4096 * freq)) - 1 */
    uint8_t prescale = (uint8_t)((PCA9685_OSC_FREQ / (PCA9685_PWM_STEPS * freq_hz)) - 1);

    /* 进入睡眠模式才能修改prescale */
    uint8_t oldmode;
    if (!PCA9685_ReadReg(PCA9685_REG_MODE1, &oldmode))
        return false;

    uint8_t newmode = (oldmode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
    if (!PCA9685_WriteReg(PCA9685_REG_MODE1, newmode))
        return false;

    if (!PCA9685_WriteReg(PCA9685_REG_PRESCALE, prescale))
        return false;

    /* 恢复模式: AI + RESTART */
    if (!PCA9685_WriteReg(PCA9685_REG_MODE1, oldmode))
        return false;
    HAL_Delay(5);

    /* 使能restart */
    if (!PCA9685_WriteReg(PCA9685_REG_MODE1, oldmode | PCA9685_MODE1_RESTART))
        return false;

    g_pca9685_pwm_freq = freq_hz;
    return true;
}

/* ── 设置PWM ────────────────────────────────────────────── */
bool PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel > 15) return false;

    uint8_t reg = PCA9685_REG_LED0_ON_L + 4 * channel;
    uint8_t buf[5] = {
        reg,
        (uint8_t)(on & 0xFF),
        (uint8_t)(on >> 8),
        (uint8_t)(off & 0xFF),
        (uint8_t)(off >> 8)
    };
    return HAL_I2C_Master_Transmit(g_pca9685_hi2c, g_pca9685_addr << 1,
                                   buf, 5, HAL_MAX_DELAY) == HAL_OK;
}

/* ── 设置舵机脉宽（微秒）────────────────────────────────── */
bool PCA9685_SetServoPulse(uint8_t channel, uint16_t pulse_us)
{
    /* pulse_us → 12位计数值:
       一个PWM周期 = 1e6/freq Hz 微秒, 分辨率4096步 */
    uint32_t period_us = 1000000UL / g_pca9685_pwm_freq;
    uint32_t pulse_count = ((uint32_t)pulse_us * PCA9685_PWM_STEPS) / period_us;
    if (pulse_count > 4095) pulse_count = 4095;

    return PCA9685_SetPWM(channel, 0, (uint16_t)pulse_count);
}

/* ── 全通道设置 ─────────────────────────────────────────── */
bool PCA9685_SetAllPWM(uint16_t on, uint16_t off)
{
    uint8_t buf[5] = {
        PCA9685_REG_LED0_ON_L,
        (uint8_t)(on & 0xFF),
        (uint8_t)(on >> 8),
        (uint8_t)(off & 0xFF),
        (uint8_t)(off >> 8)
    };
    return HAL_I2C_Master_Transmit(g_pca9685_hi2c, g_pca9685_addr << 1,
                                   buf, 5, HAL_MAX_DELAY) == HAL_OK;
}
