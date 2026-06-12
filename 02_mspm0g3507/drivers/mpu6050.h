/**
 * @file mpu6050.h
 * @brief MPU6050 六轴IMU驱动（I2C接口）
 *
 * MPU6050 是 InvenSense 出品的六轴惯性测量单元（IMU）：
 *   - 三轴加速度计：±2g / ±4g / ±8g / ±16g
 *   - 三轴陀螺仪：±250 / ±500 / ±1000 / ±2000 °/s
 *   - 内置温度传感器
 *   - 内置16位ADC
 *   - 内置DMP（数字运动处理器，本驱动不使用）
 *
 * 通信接口：I2C，地址 0x68（AD0接GND）或 0x69（AD0接VDD）
 *
 * 硬件接线（典型）：
 *   MPU6050 VCC  → 3.3V
 *   MPU6050 GND  → GND
 *   MPU6050 SDA  → MSPM0 SDA（需4.7kΩ上拉）
 *   MPU6050 SCL  → MSPM0 SCL（需4.7kΩ上拉）
 *   MPU6050 AD0  → GND（地址0x68）或 VDD（地址0x69）
 *   MPU6050 INT  → 可选，数据就绪中断引脚
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include <stdbool.h>

/* MPU6050 I2C 地址 */
#define MPU6050_I2C_ADDR_LOW    0x68  /* AD0 接 GND */
#define MPU6050_I2C_ADDR_HIGH   0x69  /* AD0 接 VDD */
#define MPU6050_I2C_ADDR        MPU6050_I2C_ADDR_LOW

/* ============================================================
 *  寄存器地址定义
 * ============================================================ */
#define MPU6050_REG_SMPLRT_DIV     0x19  /* 采样率分频器 */
#define MPU6050_REG_CONFIG         0x1A  /* 配置寄存器 */
#define MPU6050_REG_GYRO_CONFIG    0x1B  /* 陀螺仪配置 */
#define MPU6050_REG_ACCEL_CONFIG   0x1C  /* 加速度计配置 */
#define MPU6050_REG_INT_ENABLE     0x38  /* 中断使能 */
#define MPU6050_REG_INT_STATUS     0x3A  /* 中断状态 */
#define MPU6050_REG_ACCEL_XOUT_H   0x3B  /* 加速度X高字节 */
#define MPU6050_REG_ACCEL_XOUT_L   0x3C
#define MPU6050_REG_ACCEL_YOUT_H   0x3D
#define MPU6050_REG_ACCEL_YOUT_L   0x3E
#define MPU6050_REG_ACCEL_ZOUT_H   0x3F
#define MPU6050_REG_ACCEL_ZOUT_L   0x40
#define MPU6050_REG_TEMP_OUT_H     0x41  /* 温度高字节 */
#define MPU6050_REG_TEMP_OUT_L     0x42
#define MPU6050_REG_GYRO_XOUT_H    0x43  /* 陀螺仪X高字节 */
#define MPU6050_REG_GYRO_XOUT_L    0x44
#define MPU6050_REG_GYRO_YOUT_H    0x45
#define MPU6050_REG_GYRO_YOUT_L    0x46
#define MPU6050_REG_GYRO_ZOUT_H    0x47
#define MPU6050_REG_GYRO_ZOUT_L    0x48
#define MPU6050_REG_PWR_MGMT_1     0x6B  /* 电源管理1 */
#define MPU6050_REG_PWR_MGMT_2     0x6C  /* 电源管理2 */
#define MPU6050_REG_WHO_AM_I       0x75  /* 芯片ID（应为0x68） */

/* 电源管理1位定义 */
#define MPU6050_PWR1_RESET         0x80  /* 复位所有寄存器 */
#define MPU6050_PWR1_SLEEP         0x40  /* 睡眠模式 */
#define MPU6050_PWR1_CLKSEL_INT    0x00  /* 内部8MHz振荡器 */
#define MPU6050_PWR1_CLKSEL_PLL_X  0x01  /* PLL with X axis gyro */

/* 陀螺仪量程 */
#define MPU6050_GYRO_FS_250DPS     0x00  /* ±250 °/s, 灵敏度131 LSB/°/s */
#define MPU6050_GYRO_FS_500DPS     0x08  /* ±500 °/s, 灵敏度65.5 LSB/°/s */
#define MPU6050_GYRO_FS_1000DPS    0x10  /* ±1000 °/s, 灵敏度32.8 LSB/°/s */
#define MPU6050_GYRO_FS_2000DPS    0x18  /* ±2000 °/s, 灵敏度16.4 LSB/°/s */

/* 加速度计量程 */
#define MPU6050_ACCEL_FS_2G        0x00  /* ±2g, 灵敏度16384 LSB/g */
#define MPU6050_ACCEL_FS_4G        0x08  /* ±4g, 灵敏度8192 LSB/g */
#define MPU6050_ACCEL_FS_8G        0x10  /* ±8g, 灵敏度4096 LSB/g */
#define MPU6050_ACCEL_FS_16G       0x18  /* ±16g, 灵敏度2048 LSB/g */

/* 芯片ID */
#define MPU6050_WHO_AM_I_VALUE     0x68

/* 超时 */
#define MPU6050_TIMEOUT_MS         100

/**
 * @brief 加速度量程枚举
 */
typedef enum {
    MPU6050_ACCEL_RANGE_2G  = MPU6050_ACCEL_FS_2G,
    MPU6050_ACCEL_RANGE_4G  = MPU6050_ACCEL_FS_4G,
    MPU6050_ACCEL_RANGE_8G  = MPU6050_ACCEL_FS_8G,
    MPU6050_ACCEL_RANGE_16G = MPU6050_ACCEL_FS_16G
} mpu6050_accel_range_t;

/**
 * @brief 陀螺仪量程枚举
 */
typedef enum {
    MPU6050_GYRO_RANGE_250DPS  = MPU6050_GYRO_FS_250DPS,
    MPU6050_GYRO_RANGE_500DPS  = MPU6050_GYRO_FS_500DPS,
    MPU6050_GYRO_RANGE_1000DPS = MPU6050_GYRO_FS_1000DPS,
    MPU6050_GYRO_RANGE_2000DPS = MPU6050_GYRO_FS_2000DPS
} mpu6050_gyro_range_t;

/**
 * @brief MPU6050 测量数据结构体
 */
typedef struct {
    int16_t accel_x_raw;  /* 加速度X原始值 */
    int16_t accel_y_raw;  /* 加速度Y原始值 */
    int16_t accel_z_raw;  /* 加速度Z原始值 */
    int16_t gyro_x_raw;   /* 陀螺仪X原始值 */
    int16_t gyro_y_raw;   /* 陀螺仪Y原始值 */
    int16_t gyro_z_raw;   /* 陀螺仪Z原始值 */

    float accel_x_g;      /* 加速度X，单位 g */
    float accel_y_g;      /* 加速度Y，单位 g */
    float accel_z_g;      /* 加速度Z，单位 g */
    float gyro_x_dps;     /* 角速度X，单位 °/s */
    float gyro_y_dps;     /* 角速度Y，单位 °/s */
    float gyro_z_dps;     /* 角速度Z，单位 °/s */
    float temperature;    /* 温度，单位 °C */
} mpu6050_data_t;

/**
 * @brief 初始化MPU6050
 *
 * 唤醒芯片，设置时钟源，配置量程。
 *
 * @param accel_range 加速度量程
 * @param gyro_range  陀螺仪量程
 * @return true 初始化成功
 */
bool mpu6050_init(mpu6050_accel_range_t accel_range,
                   mpu6050_gyro_range_t gyro_range);

/**
 * @brief 读取全部传感器数据
 *
 * 一次性读取加速度（6字节）+ 温度（2字节）+ 陀螺仪（6字节）共14字节。
 * 自动转换为物理单位。
 *
 * @param data 输出结构体指针
 * @return true 读取成功
 */
bool mpu6050_read_all(mpu6050_data_t *data);

/**
 * @brief 仅读取加速度计数据
 * @param data 输出结构体指针
 * @return true 读取成功
 */
bool mpu6050_read_accel(mpu6050_data_t *data);

/**
 * @brief 仅读取陀螺仪数据
 * @param data 输出结构体指针
 * @return true 读取成功
 */
bool mpu6050_read_gyro(mpu6050_data_t *data);

/**
 * @brief 读取温度
 * @param temperature 输出温度值（°C）
 * @return true 读取成功
 */
bool mpu6050_read_temperature(float *temperature);

/**
 * @brief 进入睡眠模式（低功耗）
 * @return true 设置成功
 */
bool mpu6050_sleep(void);

/**
 * @brief 唤醒（退出睡眠）
 * @return true 唤醒成功
 */
bool mpu6050_wake_up(void);

/**
 * @brief 设置采样率
 * @param rate_hz 期望采样率（Hz），范围 4~1000
 * @return true 设置成功
 *
 * 实际采样率 = 1000 / (1 + div)
 */
bool mpu6050_set_sample_rate(uint16_t rate_hz);

/**
 * @brief 读取芯片ID
 * @param id 输出ID（应为0x68）
 * @return true 读取成功
 */
bool mpu6050_read_id(uint8_t *id);

#endif /* __MPU6050_H */
