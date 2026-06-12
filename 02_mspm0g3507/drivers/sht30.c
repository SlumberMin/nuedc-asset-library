/**
 * @file sht30.c
 * @brief SHT30 温湿度传感器驱动实现
 *
 * 依赖 SysConfig 生成的 I2C 宏。如使用不同的 I2C 实例，
 * 请修改下方 SHT30_I2C_INST 宏。
 */

#include "sht30.h"
#include <ti/driverlib/dl_i2c.h>
#include "ti_msp_dl_config.h"

/* ============================================================
 *  I2C 实例适配 —— 根据 SysConfig 配置修改
 * ============================================================ */
#define SHT30_I2C_INST      I2C_0_INST
#define SHT30_I2C_TARGET    SHT30_I2C_ADDR

/* ============================================================
 *  内部延时函数
 * ============================================================ */
static void sht30_delay_ms(uint32_t ms)
{
    delay_cycles(ms * 32000);  /* 假设32MHz主频 */
}

/* ============================================================
 *  CRC-8 校验（多项式 0x31，初值 0xFF）
 * ============================================================ */
static uint8_t sht30_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = SHT30_CRC_INIT;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ SHT30_CRC_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

/* ============================================================
 *  I2C 写2字节命令
 * ============================================================ */
static bool sht30_write_cmd(uint16_t cmd)
{
    DL_I2C_flushTXFIFO(SHT30_I2C_INST);
    DL_I2C_transmitData(SHT30_I2C_INST, (uint8_t)(cmd >> 8));
    DL_I2C_transmitData(SHT30_I2C_INST, (uint8_t)(cmd & 0xFF));

    DL_I2C_sendStartCondition(SHT30_I2C_INST);
    DL_I2C_setTargetAddress(SHT30_I2C_INST, SHT30_I2C_TARGET);

    uint32_t timeout = SHT30_TIMEOUT_MS * 32000;
    while (!(DL_I2C_getControllerStatus(SHT30_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }
    DL_I2C_sendStopCondition(SHT30_I2C_INST);
    return true;
}

/* ============================================================
 *  I2C 读取6字节（温度2字节+CRC + 湿度2字节+CRC）
 * ============================================================ */
static bool sht30_read_raw(uint16_t *temp_raw, uint16_t *humi_raw)
{
    uint8_t buf[6];

    DL_I2C_flushRXFIFO(SHT30_I2C_INST);
    DL_I2C_sendStartCondition(SHT30_I2C_INST);
    DL_I2C_setTargetAddress(SHT30_I2C_INST, SHT30_I2C_TARGET);
    DL_I2C_enableControllerRead(SHT30_I2C_INST, 6);

    for (int i = 0; i < 6; i++) {
        uint32_t timeout = SHT30_TIMEOUT_MS * 32000;
        while (DL_I2C_isRXFIFOEmpty(SHT30_I2C_INST)) {
            if (--timeout == 0) {
                DL_I2C_sendStopCondition(SHT30_I2C_INST);
                return false;
            }
        }
        buf[i] = DL_I2C_receiveData(SHT30_I2C_INST);
    }
    DL_I2C_sendStopCondition(SHT30_I2C_INST);

    /* CRC 校验：温度字节 */
    if (sht30_crc8(&buf[0], 2) != buf[2]) return false;
    /* CRC 校验：湿度字节 */
    if (sht30_crc8(&buf[3], 2) != buf[5]) return false;

    *temp_raw = ((uint16_t)buf[0] << 8) | buf[1];
    *humi_raw = ((uint16_t)buf[3] << 8) | buf[4];
    return true;
}

/* ============================================================
 *  将原始值转换为实际物理量
 * ============================================================ */
static void sht30_convert(uint16_t temp_raw, uint16_t humi_raw,
                          float *temperature, float *humidity)
{
    /* 温度转换公式：T = -45 + 175 * raw / 65535 */
    *temperature = -45.0f + 175.0f * (float)temp_raw / 65535.0f;
    /* 湿度转换公式：RH = 100 * raw / 65535 */
    *humidity = 100.0f * (float)humi_raw / 65535.0f;

    /* 饱和限幅 */
    if (*humidity > 100.0f) *humidity = 100.0f;
    if (*humidity < 0.0f)   *humidity = 0.0f;
}

/* ============================================================
 *  公开 API
 * ============================================================ */

bool sht30_init(void)
{
    /* 软复位 */
    if (!sht30_write_cmd(SHT30_CMD_SOFT_RESET))
        return false;
    sht30_delay_ms(2);  /* 软复位需要约1.5ms */

    /* 读取状态寄存器验证通信正常 */
    uint16_t status;
    if (!sht30_read_status(&status))
        return false;

    return true;
}

bool sht30_measure_single(sht30_data_t *data)
{
    if (data == NULL) return false;

    /* 发送单次高精度测量命令（带时钟拉伸） */
    if (!sht30_write_cmd(SHT30_CMD_SINGLE_HIGH_CS_EN))
        return false;

    /* 高精度测量需要约15ms */
    sht30_delay_ms(20);

    uint16_t temp_raw, humi_raw;
    if (!sht30_read_raw(&temp_raw, &humi_raw))
        return false;

    sht30_convert(temp_raw, humi_raw, &data->temperature, &data->humidity);
    return true;
}

bool sht30_start_continuous(uint16_t cmd)
{
    return sht30_write_cmd(cmd);
}

bool sht30_stop_continuous(void)
{
    return sht30_write_cmd(0x3093);  /* 停止连续测量命令 */
}

bool sht30_read_data(sht30_data_t *data)
{
    if (data == NULL) return false;

    uint16_t temp_raw, humi_raw;
    if (!sht30_read_raw(&temp_raw, &humi_raw))
        return false;

    sht30_convert(temp_raw, humi_raw, &data->temperature, &data->humidity);
    return true;
}

bool sht30_read_status(uint16_t *status)
{
    if (status == NULL) return false;

    if (!sht30_write_cmd(SHT30_CMD_READ_STATUS))
        return false;
    sht30_delay_ms(5);

    uint8_t buf[3];
    DL_I2C_flushRXFIFO(SHT30_I2C_INST);
    DL_I2C_sendStartCondition(SHT30_I2C_INST);
    DL_I2C_setTargetAddress(SHT30_I2C_INST, SHT30_I2C_TARGET);
    DL_I2C_enableControllerRead(SHT30_I2C_INST, 3);

    for (int i = 0; i < 3; i++) {
        uint32_t timeout = SHT30_TIMEOUT_MS * 32000;
        while (DL_I2C_isRXFIFOEmpty(SHT30_I2C_INST)) {
            if (--timeout == 0) {
                DL_I2C_sendStopCondition(SHT30_I2C_INST);
                return false;
            }
        }
        buf[i] = DL_I2C_receiveData(SHT30_I2C_INST);
    }
    DL_I2C_sendStopCondition(SHT30_I2C_INST);

    if (sht30_crc8(&buf[0], 2) != buf[2]) return false;

    *status = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

bool sht30_heater_on(void)
{
    return sht30_write_cmd(SHT30_CMD_HEATER_ON);
}

bool sht30_heater_off(void)
{
    return sht30_write_cmd(SHT30_CMD_HEATER_OFF);
}

bool sht30_soft_reset(void)
{
    if (!sht30_write_cmd(SHT30_CMD_SOFT_RESET))
        return false;
    sht30_delay_ms(2);
    return true;
}
