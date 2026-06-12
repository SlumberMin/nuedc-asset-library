/**
 * @file qmc5883l.c
 * @brief QMC5883L 三轴电子罗盘驱动实现
 *
 * 依赖 SysConfig 生成的 I2C 宏。
 */

#include "qmc5883l.h"
#include <math.h>
#include <ti/driverlib/dl_i2c.h>
#include "ti_msp_dl_config.h"

/* ============================================================
 *  I2C 实例适配
 * ============================================================ */
#define QMC5883L_I2C_INST      I2C_0_INST
#define QMC5883L_I2C_TARGET    QMC5883L_I2C_ADDR

/* 当前量程对应的灵敏度（LSB/Gauss） */
static float s_sensitivity = 12000.0f;  /* 默认 ±2G */

/* ============================================================
 *  延时函数
 * ============================================================ */
static void qmc5883l_delay_ms(uint32_t ms)
{
    delay_cycles(ms * 32000);
}

/* ============================================================
 *  I2C 写寄存器（地址1字节 + 数据1字节）
 * ============================================================ */
static bool qmc5883l_write_reg(uint8_t reg, uint8_t value)
{
    DL_I2C_flushTXFIFO(QMC5883L_I2C_INST);
    DL_I2C_transmitData(QMC5883L_I2C_INST, reg);
    DL_I2C_transmitData(QMC5883L_I2C_INST, value);

    DL_I2C_sendStartCondition(QMC5883L_I2C_INST);
    DL_I2C_setTargetAddress(QMC5883L_I2C_INST, QMC5883L_I2C_TARGET);

    uint32_t timeout = QMC5883L_TIMEOUT_MS * 32000;
    while (!(DL_I2C_getControllerStatus(QMC5883L_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }
    DL_I2C_sendStopCondition(QMC5883L_I2C_INST);
    return true;
}

/* ============================================================
 *  I2C 读寄存器（发送寄存器地址，读取N字节）
 * ============================================================ */
static bool qmc5883l_read_regs(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    /* 先写寄存器地址 */
    DL_I2C_flushTXFIFO(QMC5883L_I2C_INST);
    DL_I2C_transmitData(QMC5883L_I2C_INST, start_reg);
    DL_I2C_sendStartCondition(QMC5883L_I2C_INST);
    DL_I2C_setTargetAddress(QMC5883L_I2C_INST, QMC5883L_I2C_TARGET);

    uint32_t timeout = QMC5883L_TIMEOUT_MS * 32000;
    while (!(DL_I2C_getControllerStatus(QMC5883L_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }

    /* 重复起始条件，读取数据 */
    DL_I2C_flushRXFIFO(QMC5883L_I2C_INST);
    DL_I2C_sendStartCondition(QMC5883L_I2C_INST);
    DL_I2C_setTargetAddress(QMC5883L_I2C_INST, QMC5883L_I2C_TARGET);
    DL_I2C_enableControllerRead(QMC5883L_I2C_INST, len);

    for (uint8_t i = 0; i < len; i++) {
        timeout = QMC5883L_TIMEOUT_MS * 32000;
        while (DL_I2C_isRXFIFOEmpty(QMC5883L_I2C_INST)) {
            if (--timeout == 0) {
                DL_I2C_sendStopCondition(QMC5883L_I2C_INST);
                return false;
            }
        }
        buf[i] = DL_I2C_receiveData(QMC5883L_I2C_INST);
    }
    DL_I2C_sendStopCondition(QMC5883L_I2C_INST);
    return true;
}

/* ============================================================
 *  等待数据就绪
 * ============================================================ */
static bool qmc5883l_wait_drdy(uint32_t timeout_ms)
{
    uint32_t deadline = timeout_ms * 32000;
    while (deadline-- > 0) {
        uint8_t status;
        if (qmc5883l_read_regs(QMC5883L_REG_STATUS, &status, 1)) {
            if (status & QMC5883L_STATUS_DRDY)
                return true;
        }
    }
    return false;
}

/* ============================================================
 *  读取原始数据并转换
 * ============================================================ */
static bool qmc5883l_read_raw(qmc5883l_data_t *data)
{
    uint8_t buf[6];

    if (!qmc5883l_read_regs(QMC5883L_REG_DATA_X_LSB, buf, 6))
        return false;

    /* QMC5883L 输出格式：低字节在前，高字节在后 */
    data->x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    data->y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    data->z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

    /* 转换为 Gauss */
    data->x_gauss = (float)data->x / s_sensitivity;
    data->y_gauss = (float)data->y / s_sensitivity;
    data->z_gauss = (float)data->z / s_sensitivity;

    /* 计算航向角（仅用X和Y轴） */
    float heading = atan2f(data->y_gauss, data->x_gauss) * 180.0f / 3.14159265f;
    if (heading < 0.0f)
        heading += 360.0f;
    data->heading_deg = heading;

    return true;
}

/* ============================================================
 *  公开 API
 * ============================================================ */

bool qmc5883l_init(qmc5883l_range_t range)
{
    /* 软复位 */
    if (!qmc5883l_write_reg(QMC5883L_REG_CTRL2, 0x80))
        return false;
    qmc5883l_delay_ms(10);

    /* 验证芯片ID */
    uint8_t id;
    if (!qmc5883l_read_chip_id(&id))
        return false;
    if (id != QMC5883L_CHIP_ID_VALUE)
        return false;

    /* 设置复位寄存器（推荐值0x01） */
    if (!qmc5883l_write_reg(QMC5883L_REG_SET_RESET, 0x01))
        return false;

    /* 配置控制寄存器1：连续模式 + 200Hz + 指定量程 + OSR=512 */
    uint8_t ctrl1 = QMC5883L_CTRL1_MODE_CONTI |
                    QMC5883L_CTRL1_ODR_200HZ |
                    (uint8_t)range |
                    QMC5883L_CTRL1_OSR_512;
    if (!qmc5883l_write_reg(QMC5883L_REG_CTRL1, ctrl1))
        return false;

    /* 设置灵敏度 */
    if (range == QMC5883L_RANGE_2G)
        s_sensitivity = 12000.0f;
    else
        s_sensitivity = 3000.0f;

    return true;
}

bool qmc5883l_measure_single(qmc5883l_data_t *data)
{
    if (data == NULL) return false;

    /* 设置为单次测量模式（保持其他配置不变） */
    uint8_t ctrl1_current;
    if (!qmc5883l_read_regs(QMC5883L_REG_CTRL1, &ctrl1_current, 1))
        return false;

    /* 仅修改模式位为单次测量 */
    ctrl1_current = (ctrl1_current & ~QMC5883L_CTRL1_MODE_MASK) | 0x01;
    if (!qmc5883l_write_reg(QMC5883L_REG_CTRL1, ctrl1_current))
        return false;

    /* 等待数据就绪 */
    if (!qmc5883l_wait_drdy(QMC5883L_TIMEOUT_MS))
        return false;

    /* 读取数据 */
    return qmc5883l_read_raw(data);
}

bool qmc5883l_read_data(qmc5883l_data_t *data)
{
    if (data == NULL) return false;

    /* 等待数据就绪 */
    if (!qmc5883l_wait_drdy(QMC5883L_TIMEOUT_MS))
        return false;

    /* 读取磁场数据 */
    if (!qmc5883l_read_raw(data))
        return false;

    /* 读取温度 */
    uint8_t temp_buf[2];
    if (qmc5883l_read_regs(QMC5883L_REG_TEMP_LSB, temp_buf, 2)) {
        int16_t temp_raw = (int16_t)((uint16_t)temp_buf[1] << 8 | temp_buf[0]);
        data->temperature = (float)temp_raw / 100.0f;  /* 灵敏度约100 LSB/°C */
    } else {
        data->temperature = 0.0f;
    }

    return true;
}

bool qmc5883l_set_standby(void)
{
    return qmc5883l_write_reg(QMC5883L_REG_CTRL1, 0x00);
}

bool qmc5883l_read_chip_id(uint8_t *id)
{
    if (id == NULL) return false;
    return qmc5883l_read_regs(QMC5883L_REG_CHIP_ID, id, 1);
}

bool qmc5883l_soft_reset(void)
{
    if (!qmc5883l_write_reg(QMC5883L_REG_CTRL2, 0x80))
        return false;
    qmc5883l_delay_ms(10);
    return true;
}
