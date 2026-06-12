/**
 * @file    at24c02_stm32.c
 * @brief   AT24C02 EEPROM 驱动实现 — STM32 HAL库版本
 */

#include "drivers/at24c02_stm32.h"

static I2C_HandleTypeDef *g_eeprom_hi2c = NULL;
static uint8_t g_eeprom_addr = AT24C02_ADDR;

/* ── 初始化 ─────────────────────────────────────────────── */
void AT24C02_Init(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    g_eeprom_hi2c = hi2c;
    g_eeprom_addr = addr;
}

/* ── ACK轮询检测就绪 ───────────────────────────────────── */
bool AT24C02_IsReady(void)
{
    return HAL_I2C_IsDeviceReady(g_eeprom_hi2c, g_eeprom_addr << 1, 3, 10) == HAL_OK;
}

/* ── 读单字节 ───────────────────────────────────────────── */
bool AT24C02_ReadByte(uint8_t mem_addr, uint8_t *val)
{
    /* 发送存储地址 */
    if (HAL_I2C_Master_Transmit(g_eeprom_hi2c, g_eeprom_addr << 1,
                                &mem_addr, 1, HAL_MAX_DELAY) != HAL_OK)
        return false;
    /* 读取数据 */
    return HAL_I2C_Master_Receive(g_eeprom_hi2c, g_eeprom_addr << 1,
                                  val, 1, HAL_MAX_DELAY) == HAL_OK;
}

/* ── 写单字节 ───────────────────────────────────────────── */
bool AT24C02_WriteByte(uint8_t mem_addr, uint8_t val)
{
    uint8_t buf[2] = {mem_addr, val};
    if (HAL_I2C_Master_Transmit(g_eeprom_hi2c, g_eeprom_addr << 1,
                                buf, 2, HAL_MAX_DELAY) != HAL_OK)
        return false;
    /* 等待写入完成（ACK轮询） */
    while (!AT24C02_IsReady()) { /* 等待 */ }
    return true;
}

/* ── 读多字节 ───────────────────────────────────────────── */
bool AT24C02_Read(uint8_t mem_addr, uint8_t *buf, uint16_t len)
{
    if (mem_addr + len > AT24C02_SIZE) return false;

    if (HAL_I2C_Master_Transmit(g_eeprom_hi2c, g_eeprom_addr << 1,
                                &mem_addr, 1, HAL_MAX_DELAY) != HAL_OK)
        return false;
    return HAL_I2C_Master_Receive(g_eeprom_hi2c, g_eeprom_addr << 1,
                                  buf, len, HAL_MAX_DELAY) == HAL_OK;
}

/* ── 写多字节（跨页处理）────────────────────────────────── */
bool AT24C02_Write(uint8_t mem_addr, const uint8_t *buf, uint16_t len)
{
    if (mem_addr + len > AT24C02_SIZE) return false;

    while (len > 0) {
        /* 计算当前页剩余空间 */
        uint8_t page_remain = AT24C02_PAGE_SIZE - (mem_addr % AT24C02_PAGE_SIZE);
        uint8_t chunk = (len < page_remain) ? (uint8_t)len : page_remain;

        /* 发送: [地址] [数据...] */
        uint8_t tmp[AT24C02_PAGE_SIZE + 1];
        tmp[0] = mem_addr;
        for (uint8_t i = 0; i < chunk; i++) {
            tmp[i + 1] = buf[i];
        }
        if (HAL_I2C_Master_Transmit(g_eeprom_hi2c, g_eeprom_addr << 1,
                                    tmp, chunk + 1, HAL_MAX_DELAY) != HAL_OK)
            return false;
        while (!AT24C02_IsReady()) { /* 等待写入完成 */ }

        mem_addr += chunk;
        buf += chunk;
        len -= chunk;
    }
    return true;
}
