/**
 * @file    sht20_stm32.c
 * @brief   SHT20 温湿度传感器驱动实现 — STM32 HAL库版本
 */

#include "drivers/sht20_stm32.h"

static I2C_HandleTypeDef *g_sht20_hi2c = NULL;

/* ── 内部: 发送命令并读取原始数据(2字节+CRC) ────────────── */
static bool SHT20_Measure(uint8_t cmd, float *result, bool is_temp)
{
    uint8_t tx_cmd = cmd;
    uint8_t rx_buf[3];

    /* 发送测量命令 */
    if (HAL_I2C_Master_Transmit(g_sht20_hi2c, SHT20_ADDR << 1,
                                &tx_cmd, 1, HAL_MAX_DELAY) != HAL_OK)
        return false;

    /* 等待测量完成: 温度约85ms, 湿度约29ms */
    if (is_temp)
        HAL_Delay(100);
    else
        HAL_Delay(40);

    /* 读取2字节数据 + 1字节CRC */
    if (HAL_I2C_Master_Receive(g_sht20_hi2c, SHT20_ADDR << 1,
                               rx_buf, 3, HAL_MAX_DELAY) != HAL_OK)
        return false;

    /* 组合16位原始值 (高字节在前) */
    uint16_t raw = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    raw &= 0xFFFC;  /* 清除状态位 bit1, bit0 */

    /* 转换 */
    if (is_temp) {
        *result = -46.85f + 175.72f * (float)raw / 65536.0f;
    } else {
        *result = -6.0f + 125.0f * (float)raw / 65536.0f;
        if (*result > 100.0f) *result = 100.0f;
        if (*result < 0.0f)   *result = 0.0f;
    }
    return true;
}

/* ── 初始化 ─────────────────────────────────────────────── */
bool SHT20_Init(I2C_HandleTypeDef *hi2c)
{
    g_sht20_hi2c = hi2c;

    /* 软复位 */
    return SHT20_SoftReset();
}

/* ── 软复位 ─────────────────────────────────────────────── */
bool SHT20_SoftReset(void)
{
    uint8_t cmd = SHT20_CMD_SOFT_RESET;
    if (HAL_I2C_Master_Transmit(g_sht20_hi2c, SHT20_ADDR << 1,
                                &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
        return false;
    HAL_Delay(15);
    return true;
}

/* ── 读温度 ─────────────────────────────────────────────── */
bool SHT20_ReadTemperature(float *temp)
{
    return SHT20_Measure(SHT20_CMD_TRIG_T_HM, temp, true);
}

/* ── 读湿度 ─────────────────────────────────────────────── */
bool SHT20_ReadHumidity(float *humi)
{
    return SHT20_Measure(SHT20_CMD_TRIG_RH_HM, humi, false);
}

/* ── 同时读温湿度 ───────────────────────────────────────── */
bool SHT20_ReadAll(SHT20_Data_t *data)
{
    if (!SHT20_ReadTemperature(&data->temperature))
        return false;
    return SHT20_ReadHumidity(&data->humidity);
}
