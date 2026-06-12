/**
 * @file qmc5883l.h
 * @brief QMC5883L 三轴电子罗盘驱动（I2C接口）
 *
 * QMC5883L 是一款三轴磁传感器，可用于电子罗盘/指南针应用：
 *   - 测量范围：±2 Gauss / ±8 Gauss（可配置）
 *   - 分辨率：12位 / 16位（可配置）
 *   - 输出速率：10Hz / 50Hz / 100Hz / 200Hz
 *   - 内置温度传感器
 *
 * 通信接口：I2C，固定地址 0x0D
 *
 * 注意：QMC5883L 与 HMC5883L 地址不同（HMC为0x1E），不要混淆。
 *
 * 硬件接线（典型）：
 *   QMC5883L VDD → 3.3V
 *   QMC5883L GND → GND
 *   QMC5883L SDA → MSPM0 SDA（需4.7kΩ上拉）
 *   QMC5883L SCL → MSPM0 SCL（需4.7kΩ上拉）
 *   QMC5883L DRDY → 可选，数据就绪中断引脚
 */

#ifndef __QMC5883L_H
#define __QMC5883L_H

#include <stdint.h>
#include <stdbool.h>

/* QMC5883L I2C 固定地址 */
#define QMC5883L_I2C_ADDR       0x0D

/* 寄存器地址定义 */
#define QMC5883L_REG_DATA_X_LSB 0x00  /* X轴数据低字节 */
#define QMC5883L_REG_DATA_X_MSB 0x01  /* X轴数据高字节 */
#define QMC5883L_REG_DATA_Y_LSB 0x02
#define QMC5883L_REG_DATA_Y_MSB 0x03
#define QMC5883L_REG_DATA_Z_LSB 0x04
#define QMC5883L_REG_DATA_Z_MSB 0x05
#define QMC5883L_REG_STATUS     0x06  /* 状态寄存器 */
#define QMC5883L_REG_TEMP_LSB   0x07  /* 温度低字节 */
#define QMC5883L_REG_TEMP_MSB   0x08  /* 温度高字节 */
#define QMC5883L_REG_CTRL1      0x09  /* 控制寄存器1 */
#define QMC5883L_REG_CTRL2      0x0A  /* 控制寄存器2 */
#define QMC5883L_REG_SET_RESET  0x0B  /* 复位寄存器 */
#define QMC5883L_REG_CHIP_ID    0x0D  /* 芯片ID（应为0xFF） */

/* 控制寄存器1 位定义 */
#define QMC5883L_CTRL1_MODE_MASK    0x03
#define QMC5883L_CTRL1_MODE_STANDBY 0x00  /* 待机模式 */
#define QMC5883L_CTRL1_MODE_CONTI   0x01  /* 连续测量模式 */
#define QMC5883L_CTRL1_MODE_SINGLE  0x11  /* 单次测量模式 */

#define QMC5883L_CTRL1_ODR_MASK     0x0C
#define QMC5883L_CTRL1_ODR_10HZ     0x00  /* 10Hz输出 */
#define QMC5883L_CTRL1_ODR_50HZ     0x04  /* 50Hz输出 */
#define QMC5883L_CTRL1_ODR_100HZ    0x08  /* 100Hz输出 */
#define QMC5883L_CTRL1_ODR_200HZ    0x0C  /* 200Hz输出 */

#define QMC5883L_CTRL1_RANGE_MASK   0x30
#define QMC5883L_CTRL1_RANGE_2G     0x00  /* ±2 Gauss */
#define QMC5883L_CTRL1_RANGE_8G     0x10  /* ±8 Gauss */

#define QMC5883L_CTRL1_OSR_MASK     0xC0
#define QMC5883L_CTRL1_OSR_512      0x00  /* 过采样率 512 */
#define QMC5883L_CTRL1_OSR_256      0x40  /* 过采样率 256 */
#define QMC5883L_CTRL1_OSR_128      0x80  /* 过采样率 128 */
#define QMC5883L_CTRL1_OSR_64       0xC0  /* 过采样率 64 */

/* 状态寄存器位定义 */
#define QMC5883L_STATUS_DRDY        0x01  /* 数据就绪 */
#define QMC5883L_STATUS_OVL         0x02  /* 数据溢出 */
#define QMC5883L_STATUS_DOR         0x04  /* 数据跳过 */

/* 芯片ID */
#define QMC5883L_CHIP_ID_VALUE      0xFF

/* 超时时间 */
#define QMC5883L_TIMEOUT_MS         100

/**
 * @brief QMC5883L 量程枚举
 */
typedef enum {
    QMC5883L_RANGE_2G = QMC5883L_CTRL1_RANGE_2G,  /* ±2 Gauss，灵敏度12000 LSB/Gauss */
    QMC5883L_RANGE_8G = QMC5883L_CTRL1_RANGE_8G   /* ±8 Gauss，灵敏度3000 LSB/Gauss */
} qmc5883l_range_t;

/**
 * @brief QMC5883L 测量数据结构体
 */
typedef struct {
    int16_t x;           /* X轴磁场原始值（有符号16位） */
    int16_t y;           /* Y轴磁场原始值 */
    int16_t z;           /* Z轴磁场原始值 */
    float x_gauss;       /* X轴磁场，单位 Gauss */
    float y_gauss;       /* Y轴磁场，单位 Gauss */
    float z_gauss;       /* Z轴磁场，单位 Gauss */
    float heading_deg;   /* 航向角 0~360°（相对于X轴正方向，顺时针） */
    float temperature;   /* 温度，单位 °C */
} qmc5883l_data_t;

/**
 * @brief 初始化QMC5883L
 *
 * 验证芯片ID，配置量程、输出速率、过采样率，设置连续测量模式。
 *
 * @param range 量程选择（±2G 或 ±8G）
 * @return true 初始化成功
 */
bool qmc5883l_init(qmc5883l_range_t range);

/**
 * @brief 单次测量
 *
 * 切换到单次测量模式，等待DRDY，读取数据。
 *
 * @param data 输出结构体指针
 * @return true 测量成功
 */
bool qmc5883l_measure_single(qmc5883l_data_t *data);

/**
 * @brief 读取连续测量数据
 *
 * 从连续测量模式读取数据。必须先调用 qmc5883l_init 进入连续模式。
 *
 * @param data 输出结构体指针
 * @return true 读取成功
 */
bool qmc5883l_read_data(qmc5883l_data_t *data);

/**
 * @brief 设置待机模式（低功耗）
 * @return true 设置成功
 */
bool qmc5883l_set_standby(void);

/**
 * @brief 读取芯片ID
 * @param id 输出芯片ID
 * @return true 读取成功
 */
bool qmc5883l_read_chip_id(uint8_t *id);

/**
 * @brief 软复位
 * @return true 复位成功
 */
bool qmc5883l_soft_reset(void);

#endif /* __QMC5883L_H */
