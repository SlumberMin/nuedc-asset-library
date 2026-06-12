/**
 * @file sht30.h
 * @brief SHT30 温湿度传感器驱动（I2C接口）
 *
 * SHT30 是 Sensirion 出品的高精度数字温湿度传感器：
 *   - 温度精度：±0.3°C（典型），范围 -40~125°C
 *   - 湿度精度：±2%RH（典型），范围 0~100%RH
 *   - 分辨率：16位
 *
 * 通信接口：I2C，地址 0x44（ADDR引脚接GND）或 0x45（ADDR引脚接VDD）
 *
 * 硬件接线（典型）：
 *   SHT30 VDD → 3.3V
 *   SHT30 GND → GND
 *   SHT30 SDA → MSPM0 SDA（需4.7kΩ上拉）
 *   SHT30 SCL → MSPM0 SCL（需4.7kΩ上拉）
 *   SHT30 ADDR → GND（地址0x44）或 VDD（地址0x45）
 */

#ifndef __SHT30_H
#define __SHT30_H

#include <stdint.h>
#include <stdbool.h>

/* SHT30 I2C 地址（ADDR接GND=0x44，接VDD=0x45） */
#define SHT30_I2C_ADDR_LOW   0x44  /* ADDR 引脚接 GND */
#define SHT30_I2C_ADDR_HIGH  0x45  /* ADDR 引脚接 VDD */

/* 默认地址（ADDR接GND） */
#define SHT30_I2C_ADDR       SHT30_I2C_ADDR_LOW

/* SHT30 命令定义 */
#define SHT30_CMD_SINGLE_HIGH_CS_EN   0x2C06  /* 单次高精度，时钟拉伸使能 */
#define SHT30_CMD_SINGLE_HIGH_CS_DIS  0x2400  /* 单次高精度，时钟拉伸禁用 */
#define SHT30_CMD_SINGLE_MED_CS_EN    0x2C0D  /* 单次中精度，时钟拉伸使能 */
#define SHT30_CMD_SINGLE_LOW_CS_EN    0x2C10  /* 单次低精度，时钟拉伸使能 */
#define SHT30_CMD_CONTI_HIGH_0_5      0x2032  /* 连续高精度 0.5 mps */
#define SHT30_CMD_CONTI_HIGH_1        0x2400  /* 连续高精度 1 mps（即单次高精度CS_DIS） */
#define SHT30_CMD_CONTI_HIGH_2        0x2737  /* 连续高精度 2 mps */
#define SHT30_CMD_CONTI_HIGH_4        0x2629  /* 连续高精度 4 mps */
#define SHT30_CMD_CONTI_HIGH_10       0x2032  /* 连续高精度 10 mps */
#define SHT30_CMD_READ_SERIAL         0x3780  /* 读序列号 */
#define SHT30_CMD_SOFT_RESET          0x30A2  /* 软复位 */
#define SHT30_CMD_HEATER_ON           0x306D  /* 加热器开 */
#define SHT30_CMD_HEATER_OFF          0x3066  /* 加热器关 */
#define SHT30_CMD_READ_STATUS         0xF32D  /* 读状态寄存器 */

/* CRC 参数 */
#define SHT30_CRC_POLYNOMIAL          0x31
#define SHT30_CRC_INIT                0xFF

/* 超时时间（ms） */
#define SHT30_TIMEOUT_MS              100

/**
 * @brief SHT30 测量结果结构体
 */
typedef struct {
    float temperature;  /* 温度，单位 °C */
    float humidity;     /* 相对湿度，单位 %RH */
} sht30_data_t;

/**
 * @brief 初始化SHT30传感器
 *
 * 发送软复位命令，等待复位完成，然后读取状态寄存器验证通信正常。
 *
 * @return true 初始化成功
 */
bool sht30_init(void);

/**
 * @brief 单次测量（高精度模式）
 *
 * 发送单次高精度测量命令，等待测量完成，读取温湿度数据。
 *
 * @param data 输出结构体指针
 * @return true 测量成功
 */
bool sht30_measure_single(sht30_data_t *data);

/**
 * @brief 开启连续测量模式
 *
 * @param cmd 连续测量命令（如 SHT30_CMD_CONTI_HIGH_4）
 * @return true 设置成功
 */
bool sht30_start_continuous(uint16_t cmd);

/**
 * @brief 停止连续测量模式
 * @return true 停止成功
 */
bool sht30_stop_continuous(void);

/**
 * @brief 从连续测量模式读取数据
 * @param data 输出结构体指针
 * @return true 读取成功
 */
bool sht30_read_data(sht30_data_t *data);

/**
 * @brief 读取状态寄存器
 * @param status 输出状态值（16位）
 * @return true 读取成功
 */
bool sht30_read_status(uint16_t *status);

/**
 * @brief 开启片上加热器（用于去湿）
 * @return true 设置成功
 */
bool sht30_heater_on(void);

/**
 * @brief 关闭片上加热器
 * @return true 设置成功
 */
bool sht30_heater_off(void);

/**
 * @brief 软复位
 * @return true 复位成功
 */
bool sht30_soft_reset(void);

#endif /* __SHT30_H */
