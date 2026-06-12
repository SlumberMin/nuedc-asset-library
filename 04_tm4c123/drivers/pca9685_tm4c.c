/**
 * @file    pca9685_tm4c.c
 * @brief   PCA9685舵机驱动板驱动 实现文件 (TM4C123 I2C)
 */

#include "pca9685_tm4c.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

/* ========== 内部变量 ========== */
static uint32_t g_i2c_base = 0;
static uint8_t  g_dev_addr = PCA9685_ADDR;

/* ========== 内部: I2C写寄存器 ========== */
static void PCA9685_WriteReg(uint8_t reg, uint8_t val)
{
    I2CMasterSlaveAddrSet(g_i2c_base, g_dev_addr, false);
    I2CMasterDataPut(g_i2c_base, reg);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterDataPut(g_i2c_base, val);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(g_i2c_base)) {}
}

static uint8_t PCA9685_ReadReg(uint8_t reg)
{
    I2CMasterSlaveAddrSet(g_i2c_base, g_dev_addr, false);
    I2CMasterDataPut(g_i2c_base, reg);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterSlaveAddrSet(g_i2c_base, g_dev_addr, true);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(g_i2c_base)) {}

    return (uint8_t)I2CMasterDataGet(g_i2c_base);
}

/* ========== 内部: 计算预分频值 ========== */
static uint8_t PCA9685_CalcPrescale(float freq_hz)
{
    /* prescale = round(25MHz / (4096 * freq)) - 1 */
    float prescale = 25000000.0f / (4096.0f * freq_hz) - 1.0f;
    if (prescale < 3.0f)  prescale = 3.0f;
    if (prescale > 255.0f) prescale = 255.0f;
    return (uint8_t)(prescale + 0.5f);
}

/* ========================================================================== */
/*                              公共接口实现                                    */
/* ========================================================================== */

void PCA9685_Init(const PCA9685_Config_t *cfg)
{
    g_i2c_base = cfg->i2c_base;
    g_dev_addr = cfg->dev_addr;

    /* 1. 使能外设时钟 */
    SysCtlPeripheralEnable(cfg->gpio_periph);
    while (!SysCtlPeripheralReady(cfg->gpio_periph)) {}
    SysCtlPeripheralEnable(cfg->i2c_periph);
    while (!SysCtlPeripheralReady(cfg->i2c_periph)) {}

    /* 2. 配置GPIO */
    GPIOPinTypeI2CSCL(cfg->gpio_base, cfg->scl_pin);
    GPIOPinTypeI2C(cfg->gpio_base, cfg->sda_pin);
    GPIOPinConfigure(cfg->scl_config);
    GPIOPinConfigure(cfg->sda_config);

    /* 3. 初始化I2C主模块 */
    I2CMasterInitExpClk(cfg->i2c_base, cfg->sys_clock_hz, false);

    /* 4. 软复位 */
    PCA9685_WriteReg(PCA9685_REG_MODE1, 0x00);

    /* 5. 设置PWM频率 */
    PCA9685_SetFrequency(cfg->pwm_freq > 0 ? cfg->pwm_freq : 50.0f);

    /* 6. 配置MODE2: 推挽输出, 自动递增 */
    PCA9685_WriteReg(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV);
}

void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel > 15) return;

    uint8_t reg_base = PCA9685_REG_LED0_ON_L + 4 * channel;

    I2CMasterSlaveAddrSet(g_i2c_base, g_dev_addr, false);
    I2CMasterDataPut(g_i2c_base, reg_base);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterDataPut(g_i2c_base, (uint8_t)(on & 0xFF));
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_CONT);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterDataPut(g_i2c_base, (uint8_t)(on >> 8));
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_CONT);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterDataPut(g_i2c_base, (uint8_t)(off & 0xFF));
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_CONT);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterDataPut(g_i2c_base, (uint8_t)(off >> 8));
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(g_i2c_base)) {}
}

void PCA9685_SetServoAngle(uint8_t channel, float angle)
{
    /* 50Hz下: 0.5ms=102, 2.5ms=512 (4096/20ms * 脉宽ms) */
    if (angle < 0.0f)  angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    uint16_t off_val = (uint16_t)(102.0f + (angle / 180.0f) * 410.0f);
    PCA9685_SetPWM(channel, 0, off_val);
}

void PCA9685_SetDuty(uint8_t channel, float duty_pct)
{
    if (duty_pct < 0.0f)   duty_pct = 0.0f;
    if (duty_pct > 100.0f) duty_pct = 100.0f;

    uint16_t off_val = (uint16_t)(duty_pct * 4095.0f / 100.0f);
    PCA9685_SetPWM(channel, 0, off_val);
}

void PCA9685_SetFrequency(float freq_hz)
{
    uint8_t prescale = PCA9685_CalcPrescale(freq_hz);

    /* 进入睡眠模式才能修改预分频器 */
    uint8_t old_mode = PCA9685_ReadReg(PCA9685_REG_MODE1);
    uint8_t new_mode = (old_mode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
    PCA9685_WriteReg(PCA9685_REG_MODE1, new_mode);

    PCA9685_WriteReg(PCA9685_REG_PRESCALE, prescale);

    /* 恢复: 清除SLEEP, 等待振荡器稳定 */
    PCA9685_WriteReg(PCA9685_REG_MODE1, old_mode);
    SysCtlDelay(10000);

    /* 使能自动递增 + RESTART */
    PCA9685_WriteReg(PCA9685_REG_MODE1,
                     old_mode | PCA9685_MODE1_AI | PCA9685_MODE1_RESTART);
}

void PCA9685_AllOff(void)
{
    /* 全通道OFF寄存器地址0xFA, 写0x10=全关 */
    I2CMasterSlaveAddrSet(g_i2c_base, g_dev_addr, false);
    I2CMasterDataPut(g_i2c_base, 0xFA);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterDataPut(g_i2c_base, 0x00);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_CONT);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterDataPut(g_i2c_base, 0x00);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_CONT);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterDataPut(g_i2c_base, 0x10);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_CONT);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterDataPut(g_i2c_base, 0x00);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(g_i2c_base)) {}
}
