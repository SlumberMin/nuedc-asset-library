/**
 * @file sgp30.h
 * @brief SGP30 空气质量传感器驱动（I2C接口）
 *
 * SGP30 是 Sensirion 出品的金属氧化物气体传感器，可测量：
 *   - TVOC（总挥发性有机化合物）：0~60000 ppb
 *   - eCO2（等效二氧化碳）：400~60000 ppm
 *
 * 通信接口：I2C，固定地址 0x58
 * 需要上电后发送 init_air_quality 命令，然后每秒读取一次 measure_air_quality
 *
 * 硬件接线（典型）：
 *   SGP30 VDD  → 3.3V
 *   SGP30 GND  → GND
 *   SGP30 SDA  → MSPM0 SDA（需4.7kΩ上拉）
 *   SGP30 SCL  → MSPM0 SCL（需4.7kΩ上拉）
 */

#ifndef __SGP30_H
#define __SGP30_H

#include <stdint.h>
#include <stdbool.h>

/* SGP30 I2C 固定地址 */
#define SGP30_I2C_ADDR              0x58

/* SGP30 命令定义（2字节，大端） */
#define SGP30_CMD_INIT_AIR_QUALITY  0x2003  /* 初始化空气质量测量 */
#define SGP30_CMD_MEASURE_AIR_QUALITY   0x2008  /* 测量空气质量（TVOC+eCO2） */
#define SGP30_CMD_GET_FEATURE_SET   0x202F  /* 获取特性集版本 */
#define SGP30_CMD_MEASURE_TEST      0x2032  /* 自检命令 */
#define SGP30_CMD_GET_TVOC_BASELINE 0x20B3  /* 获取TVOC基线 */
#define SGP30_CMD_SET_TVOC_BASELINE 0x2077  /* 设置TVOC基线 */
#define SGP30_CMD_SET_HUMIDITY      0x2061  /* 设置湿度补偿 */

/* CRC 参数 */
#define SGP30_CRC_POLYNOMIAL        0x31
#define SGP30_CRC_INIT              0xFF

/* 超时时间（ms） */
#define SGP30_TIMEOUT_MS            200

/**
 * @brief SGP30 测量结果结构体
 */
typedef struct {
    uint16_t tvoc_ppb;   /* TVOC浓度，单位 ppb */
    uint16_t eco2_ppm;   /* eCO2浓度，单位 ppm */
} sgp30_data_t;

/**
 * @brief 初始化SGP30传感器
 *
 * 发送 init_air_quality 命令，等待传感器初始化完成。
 * 必须在上电后调用一次，之后每秒调用 measure 读取数据。
 *
 * @return true 初始化成功，false 通信失败
 */
bool sgp30_init(void);

/**
 * @brief 读取空气质量数据（TVOC + eCO2）
 *
 * 发送 measure_air_quality 命令并读取结果。
 * 注意：SGP30要求每秒至少调用一次此函数。
 *
 * @param data 输出结构体指针
 * @return true 读取成功，false 通信失败或CRC校验失败
 */
bool sgp30_measure(sgp30_data_t *data);

/**
 * @brief 设置湿度补偿值
 *
 * 将当前相对湿度和温度发送给SGP30以提高测量精度。
 *
 * @param humidity_percent  相对湿度 0~100%
 * @param temperature_c     温度 -40~85°C
 * @return true 设置成功
 */
bool sgp30_set_humidity(uint8_t humidity_percent, int8_t temperature_c);

/**
 * @brief 获取基线值（用于保存/恢复）
 * @param tvoc_base 输出TVOC基线
 * @param eco2_base 输出eCO2基线
 * @return true 成功
 */
bool sgp30_get_baseline(uint16_t *tvoc_base, uint16_t *eco2_base);

/**
 * @brief 恢复基线值
 * @param tvoc_base TVOC基线
 * @param eco2_base eCO2基线
 * @return true 成功
 */
bool sgp30_set_baseline(uint16_t tvoc_base, uint16_t eco2_base);

/**
 * @brief SGP30自检
 * @return true 自检通过，false 自检失败
 */
bool sgp30_selftest(void);

#endif /* __SGP30_H */
