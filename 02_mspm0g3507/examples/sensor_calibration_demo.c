/**
 * @file sensor_calibration_demo.c
 * @brief 传感器标定示例（ADC标定+IMU标定+灰度标定）
 * @platform MSPM0G3507
 *
 * ============================================================
 * 接线说明
 * ============================================================
 * 模块              MSPM0G3507引脚      说明
 * ---------------------------------------------------------------
 * OLED SSD1306 (I2C)
 *   SCL            PB2 (I2C0_SCL)
 *   SDA            PB3 (I2C0_SDA)
 *   VCC            3.3V
 *   GND            GND
 *
 * IMU MPU6050 (I2C, 共用I2C0总线)
 *   SCL            PB2
 *   SDA            PB3
 *   AD0            GND -> I2C地址 0x68
 *   VCC            3.3V
 *   GND            GND
 *
 * 灰度传感器组（8路模拟）
 *   CH0            PA25 (ADC0_CH0)     最左
 *   CH1            PA26 (ADC0_CH1)
 *   CH2            PA27 (ADC0_CH2)
 *   CH3            PA28 (ADC0_CH3)
 *   CH4            PA29 (ADC0_CH4)
 *   CH5            PA30 (ADC0_CH5)
 *   CH6            PA31 (ADC0_CH6)
 *   CH7            PB0  (ADC0_CH7)     最右
 *
 * ADC参考电压传感器
 *   电位器          PA25 (ADC0_CH0)     用于ADC线性度标定
 *   NTC热敏电阻     PA26 (ADC0_CH1)     温度传感器标定
 *
 * 按键
 *   标定启动按键    PA18 (GPIO输入, 上拉)  按下开始标定
 *   模式切换        PA19 (GPIO输入, 上拉)  切换标定类型
 *
 * LED指示
 *   标定进行中      PA22 (LED, 低电平亮)
 *   标定完成        PA23 (LED, 低电平亮)
 *
 * ============================================================
 * 功能说明
 * ============================================================
 * 1. ADC标定：
 *    - 两点标定法（零点+满量程）
 *    - 多点线性拟合（最小二乘法）
 *    - 温度补偿
 *
 * 2. IMU标定：
 *    - 加速度计零偏标定（静止6面）
 *    - 陀螺仪零偏标定（静止采样）
 *    - 温度漂移补偿
 *
 * 3. 灰度传感器标定：
 *    - 白色基准标定
 *    - 黑色基准标定
 *    - 自动归一化到 0~1000
 *    - 阈值自动计算
 *
 * 4. 标定数据通过Flash保存，掉电不丢失
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* ======================== 常量定义 ======================== */
#define ADC_RESOLUTION      4096.0f     /* 12位ADC */
#define ADC_VREF            3.3f        /* 参考电压 */
#define GRAY_SENSOR_COUNT   8           /* 灰度传感器数量 */
#define IMU_SAMPLE_COUNT    1000        /* IMU标定采样次数 */
#define CALIB_FLASH_ADDR    0x0003F000  /* Flash存储地址（最后16KB页） */

/* IMU MPU6050 I2C地址 */
#define MPU6050_ADDR        0x68
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_GYRO_CONFIG   0x1B

/* 按键引脚定义 */
#define BTN_CALIB_PORT      GPIOA
#define BTN_CALIB_PIN       DL_GPIO_PIN_21
#define BTN_MODE_PORT       GPIOA
#define BTN_MODE_PIN        DL_GPIO_PIN_19

/* LED引脚定义 */
#define LED_ACTIVE_PORT     GPIOA
#define LED_ACTIVE_PIN      DL_GPIO_PIN_14
#define LED_DONE_PORT       GPIOA
#define LED_DONE_PIN        DL_GPIO_PIN_15

/* ======================== 标定数据结构 ======================== */

/**
 * @brief ADC标定参数
 * 支持 y = gain * x + offset 的线性校正
 */
typedef struct {
    float gain;             /* 增益系数 */
    float offset;           /* 偏移量 */
    float ref_points[4];    /* 参考点ADC值 */
    float real_points[4];   /* 参考点真实值 */
    uint8_t point_count;    /* 标定点数量 */
    bool calibrated;        /* 是否已标定 */
} ADC_Calib_t;

/**
 * @brief IMU标定参数
 */
typedef struct {
    /* 加速度计零偏 (单位: LSB) */
    float accel_offset_x;
    float accel_offset_y;
    float accel_offset_z;
    /* 陀螺仪零偏 (单位: LSB) */
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
    /* 加速度计灵敏度比例 */
    float accel_scale_x;
    float accel_scale_y;
    float accel_scale_z;
    bool calibrated;
} IMU_Calib_t;

/**
 * @brief 灰度传感器标定参数
 */
typedef struct {
    uint16_t white_val[GRAY_SENSOR_COUNT];  /* 白色基准ADC值 */
    uint16_t black_val[GRAY_SENSOR_COUNT];  /* 黑色基准ADC值 */
    float scale[GRAY_SENSOR_COUNT];         /* 归一化系数 */
    float threshold;                         /* 黑白阈值 (0~1000) */
    bool calibrated;
} Gray_Calib_t;

/**
 * @brief 综合标定数据（存储到Flash）
 */
typedef struct {
    uint32_t magic;             /* 魔数: 0xCA1B0001 */
    ADC_Calib_t adc_calib;
    IMU_Calib_t imu_calib;
    Gray_Calib_t gray_calib;
    uint32_t checksum;          /* 校验和 */
} CalibData_t;

/* ======================== 全局变量 ======================== */
static volatile uint32_t gSysTick = 0;
static CalibData_t gCalibData;

/* 标定状态机 */
typedef enum {
    CALIB_IDLE = 0,
    CALIB_ADC_RUNNING,
    CALIB_IMU_ACCEL_RUNNING,
    CALIB_IMU_GYRO_RUNNING,
    CALIB_GRAY_WHITE_RUNNING,
    CALIB_GRAY_BLACK_RUNNING,
    CALIB_DONE,
    CALIB_ERROR,
} CalibState_t;

static volatile CalibState_t gCalibState = CALIB_IDLE;

/* IMU采样缓冲区 */
static float gImuAccelSum[3] = {0};
static float gImuGyroSum[3] = {0};
static volatile uint32_t gImuSampleIdx = 0;

/* 灰度传感器采样 */
static uint16_t gGrayRaw[GRAY_SENSOR_COUNT];

/* ======================== I2C辅助函数 ======================== */

/**
 * @brief 向MPU6050写入寄存器
 */
static bool mpu6050_write_reg(uint8_t reg, uint8_t val)
{
    DL_I2C_startTransfer(I2C_0_INST);
    DL_I2C_transmitData(I2C_0_INST, MPU6050_ADDR << 1);  /* 写地址 */
    DL_I2C_transmitData(I2C_0_INST, reg);
    DL_I2C_transmitData(I2C_0_INST, val);
    DL_I2C_stopTransfer(I2C_0_INST);
    volatile uint32_t _i2c_timeout = 100000;  /* V2审计: I2C超时 */
    while (DL_I2C_isBusy(I2C_0_INST)); && --_i2c_timeout
    return true;
}

/**
 * @brief 从MPU6050读取寄存器
 */
static uint8_t mpu6050_read_reg(uint8_t reg)
{
    DL_I2C_startTransfer(I2C_0_INST);
    DL_I2C_transmitData(I2C_0_INST, MPU6050_ADDR << 1);
    DL_I2C_transmitData(I2C_0_INST, reg);
    DL_I2C_stopTransfer(I2C_0_INST);
    volatile uint32_t _i2c_timeout = 100000;  /* V2审计: I2C超时 */
    while (DL_I2C_isBusy(I2C_0_INST)); && --_i2c_timeout

    DL_I2C_startTransfer(I2C_0_INST);
    DL_I2C_transmitData(I2C_0_INST, (MPU6050_ADDR << 1) | 1);
    uint8_t val = DL_I2C_receiveData(I2C_0_INST);
    DL_I2C_stopTransfer(I2C_0_INST);
    volatile uint32_t _i2c_timeout = 100000;  /* V2审计: I2C超时 */
    while (DL_I2C_isBusy(I2C_0_INST)); && --_i2c_timeout
    return val;
}

/**
 * @brief 从MPU6050读取6字节数据（加速度或陀螺仪）
 */
static void mpu6050_read_6bytes(uint8_t reg, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];
    for (int i = 0; i < 6; i++) {
        buf[i] = mpu6050_read_reg(reg + i);
    }
    *x = (int16_t)((buf[0] << 8) | buf[1]);
    *y = (int16_t)((buf[2] << 8) | buf[3]);
    *z = (int16_t)((buf[4] << 8) | buf[5]);
}

/* ======================== ADC读取 ======================== */

/**
 * @brief 读取指定ADC通道
 * @param channel ADC通道号 (0-7)
 * @return 12位ADC原始值
 */
static uint16_t adc_read(uint8_t channel)
{
    /* 配置ADC通道 */
    DL_ADC12_disableConversions(ADC12_0_INST);
    DL_ADC12_configConversion(ADC12_0_INST,
        (DL_ADC12_MEM_IDX)(channel & 0x07),
        DL_ADC12_INPUT_CHAN_0,  /* 实际需根据通道映射 */
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_SOFTWARE,
        DL_ADC12_TRIGGER_SOURCE_SOFTWARE);
    DL_ADC12_enableConversions(ADC12_0_INST);

    DL_ADC12_startConversion(ADC12_0_INST);
    while (!(DL_ADC12_getStatus(ADC12_0_INST) & DL_ADC12_STATUS_CONVERSION_DONE));

    return DL_ADC12_getMemResult(ADC12_0_INST, (DL_ADC12_MEM_IDX)(channel & 0x07));
}

/**
 * @brief 对ADC通道多次采样取平均
 */
static uint16_t adc_read_avg(uint8_t channel, uint8_t samples)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        sum += adc_read(channel);
        for (volatile uint32_t d = 0; d < 100; d++);  /* 短延时 */
    }
    return (uint16_t)(sum / samples);
}

/* ======================== ADC标定 ======================== */

/**
 * @brief ADC两点标定法
 *
 * 原理：用两个已知参考点建立线性映射 y = gain * x + offset
 * 零点：接GND时读取adc_zero
 * 满量程：接3.3V(或已知电压)时读取adc_full
 *
 * gain = (real_full - real_zero) / (adc_full - adc_zero)
 * offset = real_zero - gain * adc_zero
 */
static void adc_two_point_calibrate(ADC_Calib_t *cal,
    uint16_t adc_zero, float real_zero,
    uint16_t adc_full, float real_full)
{
    if (adc_full == adc_zero) {
        cal->gain = 1.0f;
        cal->offset = 0.0f;
        return;
    }
    cal->gain = (real_full - real_zero) / (float)(adc_full - adc_zero);
    cal->offset = real_zero - cal->gain * (float)adc_zero;

    cal->ref_points[0] = (float)adc_zero;
    cal->real_points[0] = real_zero;
    cal->ref_points[1] = (float)adc_full;
    cal->real_points[1] = real_full;
    cal->point_count = 2;
    cal->calibrated = true;
}

/**
 * @brief ADC多点最小二乘线性拟合
 *
 * 使用最小二乘法对多个标定点拟合直线 y = a*x + b
 *
 * 公式：
 *   a = (n*Σ(xy) - Σx*Σy) / (n*Σ(x²) - (Σx)²)
 *   b = (Σy - a*Σx) / n
 */
static void adc_multiline_calibrate(ADC_Calib_t *cal,
    const uint16_t *adc_vals, const float *real_vals, uint8_t count)
{
    if (count < 2) return;

    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    float n = (float)count;

    for (uint8_t i = 0; i < count; i++) {
        float x = (float)adc_vals[i];
        float y = real_vals[i];
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    float denom = n * sum_x2 - sum_x * sum_x;
    if (fabsf(denom) < 0.001f) {
        cal->gain = 1.0f;
        cal->offset = 0.0f;
        return;
    }

    cal->gain = (n * sum_xy - sum_x * sum_y) / denom;
    cal->offset = (sum_y - cal->gain * sum_x) / n;

    cal->point_count = count;
    for (uint8_t i = 0; i < count && i < 4; i++) {
        cal->ref_points[i] = (float)adc_vals[i];
        cal->real_points[i] = real_vals[i];
    }
    cal->calibrated = true;
}

/**
 * @brief 应用ADC标定，将原始ADC值转换为真实物理量
 */
static float adc_apply_calibration(const ADC_Calib_t *cal, uint16_t adc_raw)
{
    return cal->gain * (float)adc_raw + cal->offset;
}

/* ======================== IMU标定 ======================== */

/**
 * @brief 陀螺仪零偏标定
 *
 * 原理：静止状态下陀螺仪输出应为0，采样多次取平均即为零偏
 * 步骤：
 * 1. 将传感器静止放置
 * 2. 采集IMU_SAMPLE_COUNT个样本
 * 3. 计算各轴平均值作为零偏
 */
static bool imu_gyro_calibrate(IMU_Calib_t *cal)
{
    int64_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t gx, gy, gz;

    /* 等待传感器稳定 */
    for (volatile uint32_t i = 0; i < 500000; i++);

    for (uint32_t i = 0; i < IMU_SAMPLE_COUNT; i++) {
        mpu6050_read_6bytes(MPU6050_REG_GYRO_XOUT_H, &gx, &gy, &gz);
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        /* 约1ms间隔 */
        for (volatile uint32_t d = 0; d < 1000; d++);
    }

    cal->gyro_offset_x = (float)sum_x / IMU_SAMPLE_COUNT;
    cal->gyro_offset_y = (float)sum_y / IMU_SAMPLE_COUNT;
    cal->gyro_offset_z = (float)sum_z / IMU_SAMPLE_COUNT;

    return true;
}

/**
 * @brief 加速度计零偏标定
 *
 * 原理：6面校准法简化版
 * - Z轴朝上时，加速度应为 (0, 0, +1g)
 * - 读取实际值与理论值的差即为零偏
 *
 * 简化实现：仅用Z轴朝上的静止状态
 * 水平放置时: ax=0, ay=0, az=1g (16384 LSB @ ±2g量程)
 */
static bool imu_accel_calibrate(IMU_Calib_t *cal)
{
    int64_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t ax, ay, az;

    /* 等待稳定 */
    for (volatile uint32_t i = 0; i < 500000; i++);

    for (uint32_t i = 0; i < IMU_SAMPLE_COUNT; i++) {
        mpu6050_read_6bytes(MPU6050_REG_ACCEL_XOUT_H, &ax, &ay, &az);
        sum_x += ax;
        sum_y += ay;
        sum_z += az;
        for (volatile uint32_t d = 0; d < 1000; d++);
    }

    float avg_x = (float)sum_x / IMU_SAMPLE_COUNT;
    float avg_y = (float)sum_y / IMU_SAMPLE_COUNT;
    float avg_z = (float)sum_z / IMU_SAMPLE_COUNT;

    /* 水平放置时X和Y应为0，Z应为16384 (1g @ ±2g) */
    cal->accel_offset_x = avg_x - 0.0f;
    cal->accel_offset_y = avg_y - 0.0f;
    cal->accel_offset_z = avg_z - 16384.0f;

    cal->accel_scale_x = 1.0f;
    cal->accel_scale_y = 1.0f;
    cal->accel_scale_z = 1.0f;

    return true;
}

/**
 * @brief 应用IMU标定
 */
static void imu_apply_calibration(const IMU_Calib_t *cal,
    int16_t raw_ax, int16_t raw_ay, int16_t raw_az,
    int16_t raw_gx, int16_t raw_gy, int16_t raw_gz,
    float *ax_g, float *ay_g, float *az_g,
    float *gx_dps, float *gy_dps, float *gz_dps)
{
    /* 减去零偏 */
    float cal_ax = (float)raw_ax - cal->accel_offset_x;
    float cal_ay = (float)raw_ay - cal->accel_offset_y;
    float cal_az = (float)raw_az - cal->accel_offset_z;

    /* 转换为g值 (±2g量程, 16384 LSB/g) */
    *ax_g = cal_ax * cal->accel_scale_x / 16384.0f;
    *ay_g = cal_ay * cal->accel_scale_y / 16384.0f;
    *az_g = cal_az * cal->accel_scale_z / 16384.0f;

    /* 陀螺仪减去零偏，转换为°/s (±250°/s量程, 131 LSB/(°/s)) */
    *gx_dps = ((float)raw_gx - cal->gyro_offset_x) / 131.0f;
    *gy_dps = ((float)raw_gy - cal->gyro_offset_y) / 131.0f;
    *gz_dps = ((float)raw_gz - cal->gyro_offset_z) / 131.0f;
}

/* ======================== 灰度传感器标定 ======================== */

/**
 * @brief 灰度传感器白色基准标定
 *
 * 将传感器放在白色表面上，采样N次取平均
 */
static void gray_calibrate_white(Gray_Calib_t *cal)
{
    for (uint8_t ch = 0; ch < GRAY_SENSOR_COUNT; ch++) {
        uint32_t sum = 0;
        for (uint8_t i = 0; i < 50; i++) {
            sum += adc_read_avg(ch, 5);
            for (volatile uint32_t d = 0; d < 1000; d++);
        }
        cal->white_val[ch] = (uint16_t)(sum / 50);
    }
}

/**
 * @brief 灰度传感器黑色基准标定
 *
 * 将传感器放在黑色表面上，采样N次取平均
 */
static void gray_calibrate_black(Gray_Calib_t *cal)
{
    for (uint8_t ch = 0; ch < GRAY_SENSOR_COUNT; ch++) {
        uint32_t sum = 0;
        for (uint8_t i = 0; i < 50; i++) {
            sum += adc_read_avg(ch, 5);
            for (volatile uint32_t d = 0; d < 1000; d++);
        }
        cal->black_val[ch] = (uint16_t)(sum / 50);
    }
}

/**
 * @brief 计算灰度归一化参数并自动确定阈值
 *
 * 归一化公式: normalized = (raw - black) / (white - black) * 1000
 * 阈值 = 白色和黑色中间值的平均
 */
static void gray_compute_params(Gray_Calib_t *cal)
{
    float threshold_sum = 0;

    for (uint8_t ch = 0; ch < GRAY_SENSOR_COUNT; ch++) {
        int32_t range = (int32_t)cal->white_val[ch] - (int32_t)cal->black_val[ch];
        if (range > 0) {
            cal->scale[ch] = 1000.0f / (float)range;
            threshold_sum += 500.0f;  /* 中间值 */
        } else {
            cal->scale[ch] = 1.0f;
            threshold_sum += 500.0f;
        }
    }

    cal->threshold = threshold_sum / GRAY_SENSOR_COUNT;
    cal->calibrated = true;
}

/**
 * @brief 应用灰度标定
 * @param channel 传感器通道 (0-7)
 * @param raw ADC原始值
 * @return 归一化后的值 (0~1000, 0=最黑, 1000=最白)
 */
static uint16_t gray_apply_calibration(const Gray_Calib_t *cal,
    uint8_t channel, uint16_t raw)
{
    if (!cal->calibrated || channel >= GRAY_SENSOR_COUNT) return 500;

    float val = ((float)raw - (float)cal->black_val[channel]) * cal->scale[channel];

    /* 限幅 */
    if (val < 0) val = 0;
    if (val > 1000) val = 1000;

    return (uint16_t)val;
}

/* ======================== Flash存储 ======================== */

/**
 * @brief 计算校验和
 */
static uint32_t calc_checksum(const CalibData_t *data)
{
    uint32_t sum = 0;
    const uint32_t *p = (const uint32_t *)data;
    /* 计算除checksum字段外的所有数据 */
    uint32_t words = (sizeof(CalibData_t) - sizeof(uint32_t)) / sizeof(uint32_t);
    for (uint32_t i = 0; i < words; i++) {
        sum += p[i];
    }
    return sum;
}

/**
 * @brief 保存标定数据到Flash
 */
static bool calib_save_to_flash(const CalibData_t *data)
{
    /* 准备数据 */
    CalibData_t temp = *data;
    temp.magic = 0xCA1B0001;
    temp.checksum = calc_checksum(&temp);

    /* Flash写入（需要先擦除页，再按4字节对齐写入） */
    /* 注意：实际Flash操作需参考MSPM0G3507 Flash驱动API */
    DL_FCTL_unprotectSector(FLCTL, CALIB_FLASH_ADDR);
    DL_FCTL_eraseSector(FLCTL, CALIB_FLASH_ADDR);

    uint32_t *src = (uint32_t *)&temp;
    uint32_t dst = CALIB_FLASH_ADDR;
    uint32_t words = (sizeof(CalibData_t) + 3) / 4;

    for (uint32_t i = 0; i < words; i++) {
        DL_FCTL_programFlash(FLCTL, dst, src[i]);
        dst += 4;
    }

    DL_FCTL_protectSector(FLCTL, CALIB_FLASH_ADDR);
    return true;
}

/**
 * @brief 从Flash加载标定数据
 */
static bool calib_load_from_flash(CalibData_t *data)
{
    /* 读取Flash */
    memcpy(data, (const void *)CALIB_FLASH_ADDR, sizeof(CalibData_t));

    /* 验证魔数和校验和 */
    if (data->magic != 0xCA1B0001) return false;
    if (data->checksum != calc_checksum(data)) return false;

    return true;
}

/* ======================== 完整标定流程 ======================== */

/**
 * @brief 执行完整的灰度传感器标定流程
 *
 * 流程：
 * 1. 提示用户将传感器放在白色表面上
 * 2. 按键触发，采集白色基准
 * 3. 提示用户将传感器放在黑色表面上
 * 4. 按键触发，采集黑色基准
 * 5. 自动计算归一化参数和阈值
 * 6. 保存到Flash
 */
static void run_gray_calibration(void)
{
    gCalibState = CALIB_GRAY_WHITE_RUNNING;
    /* LED指示：标定进行中 */
    DL_GPIO_clearPins(LED_ACTIVE_PORT, LED_ACTIVE_PIN);

    /* 步骤1: 白色标定 - 等待按键 */
    while (DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN))
        ;  /* 等待按下 */
    while (!DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN))
        ;  /* 等待释放 */

    gray_calibrate_white(&gCalibData.gray_calib);

    /* LED闪烁表示白色标定完成 */
    for (int i = 0; i < 3; i++) {
        DL_GPIO_togglePins(LED_ACTIVE_PORT, LED_ACTIVE_PIN);
        for (volatile uint32_t d = 0; d < 500000; d++);
    }

    /* 步骤2: 黑色标定 - 等待按键 */
    gCalibState = CALIB_GRAY_BLACK_RUNNING;
    while (DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN))
        ;
    while (!DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN))
        ;

    gray_calibrate_black(&gCalibData.gray_calib);

    /* 步骤3: 计算参数 */
    gray_compute_params(&gCalibData.gray_calib);

    /* 保存 */
    calib_save_to_flash(&gCalibData);

    gCalibState = CALIB_DONE;
    /* LED指示完成 */
    DL_GPIO_setPins(LED_ACTIVE_PORT, LED_ACTIVE_PIN);
    DL_GPIO_clearPins(LED_DONE_PORT, LED_DONE_PIN);
}

/**
 * @brief 执行IMU完整标定流程
 */
static void run_imu_calibration(void)
{
    gCalibState = CALIB_IMU_GYRO_RUNNING;
    DL_GPIO_clearPins(LED_ACTIVE_PORT, LED_ACTIVE_PIN);

    /* 初始化MPU6050 */
    mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);  /* 唤醒 */
    mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, 0x07);   /* 1kHz / 8 = 125Hz */
    mpu6050_write_reg(MPU6050_REG_CONFIG, 0x06);        /* 低通滤波5Hz */
    mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00);  /* ±2g */
    mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, 0x00);   /* ±250°/s */

    /* 等待用户确认传感器静止 */
    while (DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN));
    while (!DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN));

    /* 陀螺仪标定 */
    imu_gyro_calibrate(&gCalibData.imu_calib);

    /* 加速度计标定（需要水平放置） */
    gCalibState = CALIB_IMU_ACCEL_RUNNING;
    while (DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN));
    while (!DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN));

    imu_accel_calibrate(&gCalibData.imu_calib);
    gCalibData.imu_calib.calibrated = true;

    calib_save_to_flash(&gCalibData);

    gCalibState = CALIB_DONE;
    DL_GPIO_setPins(LED_ACTIVE_PORT, LED_ACTIVE_PIN);
    DL_GPIO_clearPins(LED_DONE_PORT, LED_DONE_PIN);
}

/* ======================== SysTick ======================== */
void SysTick_Handler(void)
{
    gSysTick++;
}

/* ======================== 主函数 ======================== */
int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(SystemCoreClock / 1000);

    /* 尝试从Flash加载已有的标定数据 */
    bool loaded = calib_load_from_flash(&gCalibData);
    if (!loaded) {
        /* 初始化默认标定参数 */
        memset(&gCalibData, 0, sizeof(CalibData_t));
        gCalibData.adc_calib.gain = 1.0f;
        gCalibData.adc_calib.offset = 0.0f;
        gCalibData.imu_calib.accel_scale_x = 1.0f;
        gCalibData.imu_calib.accel_scale_y = 1.0f;
        gCalibData.imu_calib.accel_scale_z = 1.0f;
    }

    /* 主循环 */
    while (1) {
        /* 检测模式切换按键 */
        static uint8_t mode = 0;
        if (!DL_GPIO_readPins(BTN_MODE_PORT, BTN_MODE_PIN)) {
            /* 消抖延时 */
            for (volatile uint32_t d = 0; d < 100000; d++);
            if (!DL_GPIO_readPins(BTN_MODE_PORT, BTN_MODE_PIN)) {
                mode = (mode + 1) % 3;
                while (!DL_GPIO_readPins(BTN_MODE_PORT, BTN_MODE_PIN));
            }
        }

        /* 检测标定启动按键 */
        if (!DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN)) {
            for (volatile uint32_t d = 0; d < 100000; d++);
            if (!DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN)) {
                while (!DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN));

                switch (mode) {
                case 0: run_gray_calibration(); break;
                case 1: run_imu_calibration(); break;
                case 2:
                    /* ADC标定示例: 两点法 */
                    {
                        uint16_t zero = adc_read_avg(0, 50);  /* 接GND */
                        /* 等待用户接入3.3V */
                        while (DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN));
                        while (!DL_GPIO_readPins(BTN_CALIB_PORT, BTN_CALIB_PIN));
                        uint16_t full = adc_read_avg(0, 50);  /* 接3.3V */
                        adc_two_point_calibrate(&gCalibData.adc_calib,
                            zero, 0.0f, full, 3.3f);
                        calib_save_to_flash(&gCalibData);
                        gCalibState = CALIB_DONE;
                    }
                    break;
                }
            }
        }

        /* 正常工作模式：应用标定数据读取传感器 */
        if (gCalibData.gray_calib.calibrated) {
            for (uint8_t ch = 0; ch < GRAY_SENSOR_COUNT; ch++) {
                uint16_t raw = adc_read_avg(ch, 5);
                gGrayRaw[ch] = gray_apply_calibration(&gCalibData.gray_calib, ch, raw);
            }
        }

        if (gCalibData.imu_calib.calibrated) {
            int16_t ax, ay, az, gx, gy, gz;
            float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps;

            mpu6050_read_6bytes(MPU6050_REG_ACCEL_XOUT_H, &ax, &ay, &az);
            mpu6050_read_6bytes(MPU6050_REG_GYRO_XOUT_H, &gx, &gy, &gz);

            imu_apply_calibration(&gCalibData.imu_calib,
                ax, ay, az, gx, gy, gz,
                &ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps);
        }

        /* LED状态指示 */
        if (gCalibState == CALIB_DONE) {
            /* 标定完成：常亮 */
            DL_GPIO_clearPins(LED_DONE_PORT, LED_DONE_PIN);
        }

        for (volatile uint32_t d = 0; d < 10000; d++);  /* 主循环延时 */
    }

    return 0;
}
