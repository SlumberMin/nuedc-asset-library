/**
 * @file    at24c02_tm4c.c
 * @brief   AT24C02 EEPROM驱动 实现文件 (TM4C123 I2C)
 */

#include "at24c02_tm4c.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

/* ========== 内部变量 ========== */
static uint32_t g_i2c_base = 0;
static uint32_t g_sys_clock = 0;
static uint8_t  g_dev_addr = AT24C02_ADDR;

/* ========== 内部: 延时函数 ========== */
static void AT24C02_WaitWrite(void)
{
    /* 等待写周期完成 (~5ms) */
    SysCtlDelay(g_sys_clock / 3000 * AT24C02_WRITE_DELAY_MS);
}

/* ========== 内部: 写一页数据 ========== */
static void AT24C02_WritePage(uint8_t addr, const uint8_t *data, uint8_t len)
{
    /* 发送器件地址 + 字节地址 */
    I2CMasterSlaveAddrSet(g_i2c_base, g_dev_addr, false);
    I2CMasterDataPut(g_i2c_base, addr);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(g_i2c_base)) {}

    /* 发送数据字节 */
    uint8_t i;
    for (i = 0; i < len - 1; i++) {
        I2CMasterDataPut(g_i2c_base, data[i]);
        I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_CONT);
        while (I2CMasterBusy(g_i2c_base)) {}
    }

    /* 最后一个字节 */
    I2CMasterDataPut(g_i2c_base, data[len - 1]);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(g_i2c_base)) {}
}

/* ========================================================================== */
/*                              公共接口实现                                    */
/* ========================================================================== */

void AT24C02_Init(const AT24C02_Config_t *cfg)
{
    g_i2c_base  = cfg->i2c_base;
    g_sys_clock = cfg->sys_clock_hz;
    g_dev_addr  = cfg->dev_addr;

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
}

uint8_t AT24C02_ReadByte(uint8_t addr)
{
    /* 写入字节地址 */
    I2CMasterSlaveAddrSet(g_i2c_base, g_dev_addr, false);
    I2CMasterDataPut(g_i2c_base, addr);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(g_i2c_base)) {}

    /* 读取数据 */
    I2CMasterSlaveAddrSet(g_i2c_base, g_dev_addr, true);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(g_i2c_base)) {}

    return (uint8_t)I2CMasterDataGet(g_i2c_base);
}

void AT24C02_WriteByte(uint8_t addr, uint8_t data)
{
    I2CMasterSlaveAddrSet(g_i2c_base, g_dev_addr, false);
    I2CMasterDataPut(g_i2c_base, addr);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(g_i2c_base)) {}

    I2CMasterDataPut(g_i2c_base, data);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(g_i2c_base)) {}

    AT24C02_WaitWrite();
}

void AT24C02_Read(uint8_t start_addr, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    /* 写入起始地址 */
    I2CMasterSlaveAddrSet(g_i2c_base, g_dev_addr, false);
    I2CMasterDataPut(g_i2c_base, start_addr);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(g_i2c_base)) {}

    /* 连续读取 */
    I2CMasterSlaveAddrSet(g_i2c_base, g_dev_addr, true);

    for (i = 0; i < len; i++) {
        if (i < len - 1) {
            I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        } else {
            I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        }
        while (I2CMasterBusy(g_i2c_base)) {}
        buf[i] = (uint8_t)I2CMasterDataGet(g_i2c_base);
    }
}

void AT24C02_Write(uint8_t start_addr, const uint8_t *data, uint8_t len)
{
    uint8_t written = 0;

    while (written < len) {
        /* 计算当前页剩余空间 */
        uint8_t page_remain = AT24C02_PAGE_SIZE -
                              ((start_addr + written) % AT24C02_PAGE_SIZE);
        uint8_t chunk = len - written;
        if (chunk > page_remain) {
            chunk = page_remain;
        }

        AT24C02_WritePage(start_addr + written, &data[written], chunk);
        AT24C02_WaitWrite();
        written += chunk;
    }
}

bool AT24C02_IsReady(void)
{
    I2CMasterSlaveAddrSet(g_i2c_base, g_dev_addr, false);
    I2CMasterDataPut(g_i2c_base, 0x00);
    I2CMasterControl(g_i2c_base, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(g_i2c_base)) {}

    /* 检查错误标志 */
    uint32_t err = I2CMasterErr(g_i2c_base);
    return (err == I2_MASTER_ERR_NONE);
}
