/**
 * @file mpu6050.c
 * @brief MPU6050 六轴IMU驱动实现
 *
 * 依赖 SysConfig 生成的 I2C 宏。
 */

#include "mpu6050.h"
#include <ti/driverlib/dl_i2c.h>
#include "ti_msp_dl_config.h"

/* ============================================================
 *  I2C 实例适配
 * ============================================================ */
#define MPU6050_I2C_INST      I2C_0_INST
#define MPU6050_I2C_TARGET    MPU6050_I2C_ADDR

/* 当前量程的灵敏度（静态变量，在init时设置） */
static float s_accel_sensitivity = 16384.0f;  /* 默认 ±2g */
static float s_gyro_sensitivity  = 131.0f;    /* 默认 ±250°/s */

/* ============================================================
 *  延时函数
 * ============================================================ */
static void mpu6050_delay_ms(uint32_t ms)
{
    delay_cycles(ms * 32000);
}

/* ============================================================
 *  I2C 写寄存器
 * ============================================================ */
static bool mpu6050_write_reg(uint8_t reg, uint8_t value)
{
    DL_I2C_flushTXFIFO(MPU6050_I2C_INST);
    DL_I2C_transmitData(MPU6050_I2C_INST, reg);
    DL_I2C_transmitData(MPU6050_I2C_INST, value);

    DL_I2C_sendStartCondition(MPU6050_I2C_INST);
    DL_I2C_setTargetAddress(MPU6050_I2C_INST, MPU6050_I2C_TARGET);

    uint32_t timeout = MPU6050_TIMEOUT_MS * 32000;
    while (!(DL_I2C_getControllerStatus(MPU6050_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }
    DL_I2C_sendStopCondition(MPU6050_I2C_INST);
    return true;
}

/* ============================================================
 *  I2C 读寄存器（连续读取）
 * ============================================================ */
static bool mpu6050_read_regs(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    /* 写寄存器地址 */
    DL_I2C_flushTXFIFO(MPU6050_I2C_INST);
    DL_I2C_transmitData(MPU6050_I2C_INST, start_reg);
    DL_I2C_sendStartCondition(MPU6050_I2C_INST);
    DL_I2C_setTargetAddress(MPU6050_I2C_INST, MPU6050_I2C_TARGET);

    uint32_t timeout = MPU6050_TIMEOUT_MS * 32000;
    while (!(DL_I2C_getControllerStatus(MPU6050_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }

    /* 重复起始 + 读取 */
    DL_I2C_flushRXFIFO(MPU6050_I2C_INST);
    DL_I2C_sendStartCondition(MPU6050_I2C_INST);
    DL_I2C_setTargetAddress(MPU6050_I2C_INST, MPU6050_I2C_TARGET);
    DL_I2C_enableControllerRead(MPU6050_I2C_INST, len);

    for (uint8_t i = 0; i < len; i++) {
        timeout = MPU6050_TIMEOUT_MS * 32000;
        while (DL_I2C_isRXFIFOEmpty(MPU6050_I2C_INST)) {
            if (--timeout == 0) {
                DL_I2C_sendStopCondition(MPU6050_I2C_INST);
                return false;
            }
        }
        buf[i] = DL_I2C_receiveData(MPU6050_I2C_INST);
    }
    DL_I2C_sendStopCondition(MPU6050_I2C_INST);
    return true;
}

/* ============================================================
 *  将原始值转为物理量
 * ============================================================ */
static void mpu6050_convert(const uint8_t *raw14, mpu6050_data_t *data)
{
    /* 14字节布局：AX_H AX_L AY_H AY_L AZ_H AZ_L
     *             T_H T_L GX_H GX_L GY_H GY_L GZ_H GZ_L */

    /* 加速度原始值 */
    data->accel_x_raw = (int16_t)((uint16_t)raw14[0]  << 8 | raw14[1]);
    data->accel_y_raw = (int16_t)((uint16_t)raw14[2]  << 8 | raw14[3]);
    data->accel_z_raw = (int16_t)((uint16_t)raw14[4]  << 8 | raw14[5]);

    /* 温度原始值 */
    int16_t temp_raw = (int16_t)((uint16_t)raw14[6] << 8 | raw14[7]);
    /* 温度转换公式：T = temp_raw / 340.0 + 36.53 */
    data->temperature = (float)temp_raw / 340.0f + 36.53f;

    /* 陀螺仪原始值 */
    data->gyro_x_raw = (int16_t)((uint16_t)raw14[8]  << 8 | raw14[9]);
    data->gyro_y_raw = (int16_t)((uint16_t)raw14[10] << 8 | raw14[11]);
    data->gyro_z_raw = (int16_t)((uint16_t)raw14[12] << 8 | raw14[13]);

    /* 转换为物理单位 */
    data->accel_x_g  = (float)data->accel_x_raw / s_accel_sensitivity;
    data->accel_y_g  = (float)data->accel_y_raw / s_accel_sensitivity;
    data->accel_z_g  = (float)data->accel_z_raw / s_accel_sensitivity;
    data->gyro_x_dps = (float)data->gyro_x_raw / s_gyro_sensitivity;
    data->gyro_y_dps = (float)data->gyro_y_raw / s_gyro_sensitivity;
    data->gyro_z_dps = (float)data->gyro_z_raw / s_gyro_sensitivity;
}

/* ============================================================
 *  公开 API
 * ============================================================ */

bool mpu6050_init(mpu6050_accel_range_t accel_range,
                   mpu6050_gyro_range_t gyro_range)
{
    /* 1. 复位芯片 */
    if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_RESET))
        return false;
    mpu6050_delay_ms(100);  /* 复位需要约100ms */

    /* 2. 验证芯片ID */
    uint8_t id;
    if (!mpu6050_read_id(&id))
        return false;
    if (id != MPU6050_WHO_AM_I_VALUE)
        return false;

    /* 3. 唤醒，选择时钟源为PLL with X axis gyro（更稳定） */
    if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_PLL_X))
        return false;
    mpu6050_delay_ms(10);

    /* 4. 配置加速度量程 */
    if (!mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, (uint8_t)accel_range))
        return false;

    /* 5. 配置陀螺仪量程 */
    if (!mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, (uint8_t)gyro_range))
        return false;

    /* 6. 设置灵敏度 */
    switch (accel_range) {
        case MPU6050_ACCEL_RANGE_2G:  s_accel_sensitivity = 16384.0f; break;
        case MPU6050_ACCEL_RANGE_4G:  s_accel_sensitivity = 8192.0f;  break;
        case MPU6050_ACCEL_RANGE_8G:  s_accel_sensitivity = 4096.0f;  break;
        case MPU6050_ACCEL_RANGE_16G: s_accel_sensitivity = 2048.0f;  break;
    }

    switch (gyro_range) {
        case MPU6050_GYRO_RANGE_250DPS:  s_gyro_sensitivity = 131.0f;   break;
        case MPU6050_GYRO_RANGE_500DPS:  s_gyro_sensitivity = 65.5f;    break;
        case MPU6050_GYRO_RANGE_1000DPS: s_gyro_sensitivity = 32.8f;    break;
        case MPU6050_GYRO_RANGE_2000DPS: s_gyro_sensitivity = 16.4f;    break;
    }

    return true;
}

bool mpu6050_read_all(mpu6050_data_t *data)
{
    if (data == NULL) return false;

    /* 一次性读取14字节：加速度(6) + 温度(2) + 陀螺仪(6) */
    uint8_t buf[14];
    if (!mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, buf, 14))
        return false;

    mpu6050_convert(buf, data);
    return true;
}

bool mpu6050_read_accel(mpu6050_data_t *data)
{
    if (data == NULL) return false;

    uint8_t buf[6];
    if (!mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, buf, 6))
        return false;

    data->accel_x_raw = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
    data->accel_y_raw = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
    data->accel_z_raw = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);

    data->accel_x_g = (float)data->accel_x_raw / s_accel_sensitivity;
    data->accel_y_g = (float)data->accel_y_raw / s_accel_sensitivity;
    data->accel_z_g = (float)data->accel_z_raw / s_accel_sensitivity;

    return true;
}

bool mpu6050_read_gyro(mpu6050_data_t *data)
{
    if (data == NULL) return false;

    uint8_t buf[6];
    if (!mpu6050_read_regs(MPU6050_REG_GYRO_XOUT_H, buf, 6))
        return false;

    data->gyro_x_raw = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
    data->gyro_y_raw = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
    data->gyro_z_raw = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);

    data->gyro_x_dps = (float)data->gyro_x_raw / s_gyro_sensitivity;
    data->gyro_y_dps = (float)data->gyro_y_raw / s_gyro_sensitivity;
    data->gyro_z_dps = (float)data->gyro_z_raw / s_gyro_sensitivity;

    return true;
}

bool mpu6050_read_temperature(float *temperature)
{
    if (temperature == NULL) return false;

    uint8_t buf[2];
    if (!mpu6050_read_regs(MPU6050_REG_TEMP_OUT_H, buf, 2))
        return false;

    int16_t raw = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
    *temperature = (float)raw / 340.0f + 36.53f;
    return true;
}

bool mpu6050_sleep(void)
{
    /* 设置 SLEEP 位 */
    return mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_SLEEP);
}

bool mpu6050_wake_up(void)
{
    /* 清除 SLEEP 位，选择时钟源 */
    if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_PLL_X))
        return false;
    mpu6050_delay_ms(10);
    return true;
}

bool mpu6050_set_sample_rate(uint16_t rate_hz)
{
    if (rate_hz < 4)   rate_hz = 4;
    if (rate_hz > 1000) rate_hz = 1000;

    /* 分频器 = 1000 / rate_hz - 1 */
    uint8_t div = (uint8_t)(1000 / rate_hz - 1);
    return mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, div);
}

bool mpu6050_read_id(uint8_t *id)
{
    if (id == NULL) return false;
    return mpu6050_read_regs(MPU6050_REG_WHO_AM_I, id, 1);
}
