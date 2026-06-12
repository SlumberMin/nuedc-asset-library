/**
 * @file sgp30.c
 * @brief SGP30 空气质量传感器驱动实现
 *
 * 依赖 SysConfig 生成的 I2C 宏（如 gI2c0 或 I2C_0_INST）。
 * 如使用不同的 I2C 实例，请修改下方 SGP30_I2C_* 宏。
 */

#include "sgp30.h"
#include <ti/driverlib/dl_i2c.h>
#include "ti_msp_dl_config.h"  /* SysConfig 生成的配置 */

/* ============================================================
 *  I2C 实例适配 —— 根据 SysConfig 配置修改
 * ============================================================ */
#define SGP30_I2C_INST      I2C_0_INST   /* SysConfig生成的I2C实例 */
#define SGP30_I2C_TARGET    SGP30_I2C_ADDR

/* ============================================================
 *  内部延时函数
 * ============================================================ */
static void sgp30_delay_ms(uint32_t ms)
{
    /* 使用 SysConfig 生成的 delay_cycles 或自行实现 */
    delay_cycles(ms * 32000);  /* 假设32MHz主频，粗略延时 */
}

/* ============================================================
 *  CRC-8 校验（多项式 0x31，初值 0xFF）
 * ============================================================ */
static uint8_t sgp30_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = SGP30_CRC_INIT;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ SGP30_CRC_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

/* ============================================================
 *  I2C 写命令（发送2字节命令 + CRC，可选）
 * ============================================================ */
static bool sgp30_write_cmd(uint16_t cmd)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(cmd >> 8);    /* 命令高字节 */
    buf[1] = (uint8_t)(cmd & 0xFF);  /* 命令低字节 */

    DL_I2C_flushTXFIFO(SGP30_I2C_INST);
    DL_I2C_transmitData(SGP30_I2C_INST, buf[0]);
    DL_I2C_transmitData(SGP30_I2C_INST, buf[1]);

    DL_I2C_sendStartCondition(SGP30_I2C_INST);
    DL_I2C_setTargetAddress(SGP30_I2C_INST, SGP30_I2C_TARGET);

    /* 等待传输完成或超时 */
    uint32_t timeout = SGP30_TIMEOUT_MS * 32000;
    while (!(DL_I2C_getControllerStatus(SGP30_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }
    DL_I2C_sendStopCondition(SGP30_I2C_INST);
    return true;
}

/* ============================================================
 *  I2C 读取多个字（每字2字节 + 1字节CRC）
 * ============================================================ */
static bool sgp30_read_words(uint8_t *buf, uint8_t num_words)
{
    uint8_t total = num_words * 3;  /* 每个word = 2字节数据 + 1字节CRC */

    DL_I2C_flushRXFIFO(SGP30_I2C_INST);
    DL_I2C_sendStartCondition(SGP30_I2C_INST);
    DL_I2C_setTargetAddress(SGP30_I2C_INST, SGP30_I2C_TARGET);
    DL_I2C_enableControllerRead(SGP30_I2C_INST, total);

    for (uint8_t i = 0; i < total; i++) {
        uint32_t timeout = SGP30_TIMEOUT_MS * 32000;
        while (DL_I2C_isRXFIFOEmpty(SGP30_I2C_INST)) {
            if (--timeout == 0) {
                DL_I2C_sendStopCondition(SGP30_I2C_INST);
                return false;
            }
        }
        buf[i] = DL_I2C_receiveData(SGP30_I2C_INST);
    }
    DL_I2C_sendStopCondition(SGP30_I2C_INST);

    /* CRC 校验 */
    for (uint8_t i = 0; i < num_words; i++) {
        uint8_t *p = &buf[i * 3];
        uint8_t crc = sgp30_crc8(p, 2);
        if (crc != p[2]) return false;
    }
    return true;
}

/* ============================================================
 *  公开 API
 * ============================================================ */

bool sgp30_init(void)
{
    /* 发送初始化命令 */
    if (!sgp30_write_cmd(SGP30_CMD_INIT_AIR_QUALITY))
        return false;

    /* SGP30初始化需要约10ms */
    sgp30_delay_ms(12);
    return true;
}

bool sgp30_measure(sgp30_data_t *data)
{
    if (data == NULL) return false;

    if (!sgp30_write_cmd(SGP30_CMD_MEASURE_AIR_QUALITY))
        return false;

    /* 等待测量完成（约12ms） */
    sgp30_delay_ms(15);

    uint8_t buf[6];  /* 2 words = 6 bytes (含CRC) */
    if (!sgp30_read_words(buf, 2))
        return false;

    data->eco2_ppm = ((uint16_t)buf[0] << 8) | buf[1];
    data->tvoc_ppb = ((uint16_t)buf[3] << 8) | buf[4];
    return true;
}

bool sgp30_set_humidity(uint8_t humidity_percent, int8_t temperature_c)
{
    /* 将湿度和温度编码为定点数：abs_humidity = (RH/100) * 256 */
    /* 简化：直接用比例换算 */
    uint16_t abs_humidity = (uint16_t)(humidity_percent * 256 / 100);

    uint8_t buf[5];
    buf[0] = (uint8_t)(SGP30_CMD_SET_HUMIDITY >> 8);
    buf[1] = (uint8_t)(SGP30_CMD_SET_HUMIDITY & 0xFF);
    buf[2] = (uint8_t)(abs_humidity >> 8);
    buf[3] = (uint8_t)(abs_humidity & 0xFF);
    buf[4] = sgp30_crc8(&buf[2], 2);

    DL_I2C_flushTXFIFO(SGP30_I2C_INST);
    for (int i = 0; i < 5; i++)
        DL_I2C_transmitData(SGP30_I2C_INST, buf[i]);

    DL_I2C_sendStartCondition(SGP30_I2C_INST);
    DL_I2C_setTargetAddress(SGP30_I2C_INST, SGP30_I2C_TARGET);

    uint32_t timeout = SGP30_TIMEOUT_MS * 32000;
    while (!(DL_I2C_getControllerStatus(SGP30_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }
    DL_I2C_sendStopCondition(SGP30_I2C_INST);
    (void)temperature_c;  /* 温度信息编码在abs_humidity中（此处简化） */
    return true;
}

bool sgp30_get_baseline(uint16_t *tvoc_base, uint16_t *eco2_base)
{
    if (!sgp30_write_cmd(SGP30_CMD_GET_TVOC_BASELINE))
        return false;
    sgp30_delay_ms(15);

    uint8_t buf[6];
    if (!sgp30_read_words(buf, 2))
        return false;

    *tvoc_base = ((uint16_t)buf[0] << 8) | buf[1];
    *eco2_base = ((uint16_t)buf[3] << 8) | buf[4];
    return true;
}

bool sgp30_set_baseline(uint16_t tvoc_base, uint16_t eco2_base)
{
    /* 注意：eco2_base 先发，tvoc_base 后发 */
    uint8_t buf[8];
    buf[0] = (uint8_t)(SGP30_CMD_SET_TVOC_BASELINE >> 8);
    buf[1] = (uint8_t)(SGP30_CMD_SET_TVOC_BASELINE & 0xFF);
    buf[2] = (uint8_t)(eco2_base >> 8);
    buf[3] = (uint8_t)(eco2_base & 0xFF);
    buf[4] = sgp30_crc8(&buf[2], 2);
    buf[5] = (uint8_t)(tvoc_base >> 8);
    buf[6] = (uint8_t)(tvoc_base & 0xFF);
    buf[7] = sgp30_crc8(&buf[5], 2);

    DL_I2C_flushTXFIFO(SGP30_I2C_INST);
    for (int i = 0; i < 8; i++)
        DL_I2C_transmitData(SGP30_I2C_INST, buf[i]);

    DL_I2C_sendStartCondition(SGP30_I2C_INST);
    DL_I2C_setTargetAddress(SGP30_I2C_INST, SGP30_I2C_TARGET);

    uint32_t timeout = SGP30_TIMEOUT_MS * 32000;
    while (!(DL_I2C_getControllerStatus(SGP30_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }
    DL_I2C_sendStopCondition(SGP30_I2C_INST);
    return true;
}

bool sgp30_selftest(void)
{
    if (!sgp30_write_cmd(SGP30_CMD_MEASURE_TEST))
        return false;
    sgp30_delay_ms(220);  /* 自检需要约220ms */

    uint8_t buf[3];
    if (!sgp30_read_words(buf, 1))
        return false;

    uint16_t result = ((uint16_t)buf[0] << 8) | buf[1];
    return (result == 0x00D4);  /* 0x00D4 = 自检通过 */
}
