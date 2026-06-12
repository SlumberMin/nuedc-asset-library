/**
 * @file    tcs34725_tm4c.c
 * @brief   TCS34725颜色传感器驱动 实现文件 (TM4C123 I2C)
 */

#include "tcs34725_tm4c.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

/* ========== 内部变量 ========== */
static uint32_t g_i2c_base = 0;
static uint8_t  g_int_time = 0xD5;   /* 默认101ms */
static TCS34725_Gain_t g_gain = TCS34725_GAIN_4X;

/* ========== 内部: I2C读写辅助函数 ========== */
static void I2C_WriteReg(uint8_t reg, uint8_t val)
{
    I2CMasterSlaveAddrSet(g_i2c_base, TCS34725_ADDR, false);
    I2CMasterDataPut(g_i2c_base, TCS34725_CMD_AUTO_INC | reg);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterDataPut(g_i2c_base, val);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(g_i2c_base)) {}
}

static uint8_t I2C_ReadReg(uint8_t reg)
{
    I2CMasterSlaveAddrSet(g_i2c_base, TCS34725_ADDR, false);
    I2CMasterDataPut(g_i2c_base, TCS34725_CMD_BIT | reg);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterSlaveAddrSet(g_i2c_base, TCS34725_ADDR, true);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(g_i2c_base)) {}

    return (uint8_t)I2CMasterDataGet(g_i2c_base);
}

/* 读取16位寄存器 (低字节在前) */
static uint16_t I2C_ReadReg16(uint8_t reg)
{
    uint8_t lo, hi;

    I2CMasterSlaveAddrSet(g_i2c_base, TCS34725_ADDR, false);
    I2CMasterDataPut(g_i2c_base, TCS34725_CMD_AUTO_INC | reg);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterSlaveAddrSet(g_i2c_base, TCS34725_ADDR, true);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while (I2CMasterBusy(g_i2c_base)) {}
    lo = (uint8_t)I2CMasterDataGet(g_i2c_base);

    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (I2CMasterBusy(g_i2c_base)) {}
    hi = (uint8_t)I2CMasterDataGet(g_i2c_base);

    return (uint16_t)((hi << 8) | lo);
}

/* ========================================================================== */
/*                              公共接口实现                                    */
/* ========================================================================== */

bool TCS34725_Init(const TCS34725_Config_t *cfg)
{
    g_i2c_base = cfg->i2c_base;
    g_int_time = cfg->integration_time;
    g_gain = cfg->gain;

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

    /* 4. 验证器件ID */
    uint8_t id = TCS34725_GetID();
    if (id != 0x44 && id != 0x4D) {
        return false;
    }

    /* 5. 配置传感器 */
    TCS34725_Enable(false);
    I2C_WriteReg(TCS34725_REG_ATIME, g_int_time);
    I2C_WriteReg(TCS34725_REG_CONTROL, (uint8_t)g_gain);
    TCS34725_Enable(true);

    return true;
}

void TCS34725_Read(TCS34725_Color_t *color)
{
    color->clear = I2C_ReadReg16(TCS34725_REG_CDATAL);
    color->red   = I2C_ReadReg16(TCS34725_REG_RDATAL);
    color->green = I2C_ReadReg16(TCS34725_REG_GDATAL);
    color->blue  = I2C_ReadReg16(TCS34725_REG_BDATAL);

    /* 计算归一化百分比 */
    if (color->clear > 0) {
        color->r_pct = (float)color->red   / (float)color->clear * 100.0f;
        color->g_pct = (float)color->green / (float)color->clear * 100.0f;
        color->b_pct = (float)color->blue  / (float)color->clear * 100.0f;
    } else {
        color->r_pct = 0;
        color->g_pct = 0;
        color->b_pct = 0;
    }
}

void TCS34725_SetGain(TCS34725_Gain_t gain)
{
    g_gain = gain;
    I2C_WriteReg(TCS34725_REG_CONTROL, (uint8_t)gain);
}

void TCS34725_SetIntegrationTime(uint8_t atime)
{
    g_int_time = atime;
    I2C_WriteReg(TCS34725_REG_ATIME, atime);
}

void TCS34725_Enable(bool enable)
{
    if (enable) {
        I2C_WriteReg(TCS34725_REG_ENABLE, TCS34725_ENABLE_PON);
        /* 等待上电稳定 */
        SysCtlDelay(10000);
        I2C_WriteReg(TCS34725_REG_ENABLE,
                     TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    } else {
        I2C_WriteReg(TCS34725_REG_ENABLE, 0x00);
    }
}

uint8_t TCS34725_GetID(void)
{
    return I2C_ReadReg(TCS34725_REG_ID);
}
