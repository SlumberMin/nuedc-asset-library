/**
 * @file    sensor_ir.h
 * @brief   红外循迹传感器模块 — STM32电赛通用代码库
 * @details 支持TCRT5000红外反射式传感器阵列(5路)。
 *          功能：加权平均计算位置、跨线检测、单路状态读取。
 *          传感器输出：黑线上=0(低电平/低ADC)，白色地面=1(高电平/高ADC)。
 * @author  电赛通用代码库
 * @version 1.0
 * @date    2026-06
 */

#ifndef __SENSOR_IR_H
#define __SENSOR_IR_H

#include "platform/hal_stm32.h"

/* ========================================================================== */
/*                              常量定义                                       */
/* ========================================================================== */

/** @brief 最大支持传感器路数 */
#define IR_SENSOR_MAX_CH    8

/** @brief 默认传感器路数 */
#define IR_SENSOR_DEFAULT_CH 5

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/**
 * @brief 红外传感器工作模式
 */
typedef enum {
    IR_MODE_ADC = 0,    /**< ADC模拟量模式（推荐，精度高） */
    IR_MODE_GPIO,       /**< GPIO数字量模式（简单，但无加权） */
} IRMode_t;

/**
 * @brief 红外循迹传感器配置结构体
 */
typedef struct {
    IRMode_t mode;              /**< 工作模式 */
    uint8_t  channel_count;     /**< 传感器路数(1~8) */

    /* ADC模式配置 */
    ADC_HandleTypeDef *hadc;    /**< ADC句柄（仅ADC模式） */
    uint16_t adc_channel[IR_SENSOR_MAX_CH]; /**< 各路ADC通道号 */

    /* GPIO模式配置 */
    GPIO_TypeDef *gpio_port[IR_SENSOR_MAX_CH]; /**< GPIO端口 */
    uint16_t      gpio_pin[IR_SENSOR_MAX_CH];  /**< GPIO引脚 */

    /* 阈值配置 */
    uint16_t threshold;         /**< 黑白判定阈值（ADC模式：0~4095；GPIO模式不用） */

    /* 权重（默认对称分布，可自定义） */
    int8_t   weights[IR_SENSOR_MAX_CH]; /**< 各传感器权重，如 -4,-2,0,2,4 */

    /* 状态数据 */
    uint16_t raw_value[IR_SENSOR_MAX_CH];  /**< 各路原始ADC值 */
    bool     digital[IR_SENSOR_MAX_CH];    /**< 各路数字状态（1=白地，0=黑线） */
    float    position;           /**< 当前位置：0=中心，负=偏左，正=偏右 */
    bool     on_line;            /**< 是否检测到线 */
    bool     cross_detected;     /**< 是否检测到十字路口 */
    bool     initialized;        /**< 是否已初始化 */
} SensorIR_t;

/* ========================================================================== */
/*                              接口函数                                       */
/* ========================================================================== */

/**
 * @brief 初始化红外循迹传感器（ADC模式）
 * @param ir       传感器结构体指针
 * @param hadc     ADC句柄
 * @param channels 各路ADC通道号数组
 * @param count    传感器路数(1~8)
 * @param threshold 黑白判定阈值
 * @return ErrorCode_t: HAL_OK_CODE=成功
 * @note   默认权重：5路时为 {-4, -2, 0, 2, 4}
 *         position = Σ(weight[i]*digital[i]) / Σ(digital[i]为1的个数)
 */
ErrorCode_t SensorIR_InitADC(SensorIR_t *ir, ADC_HandleTypeDef *hadc,
                             const uint16_t *channels, uint8_t count,
                             uint16_t threshold);

/**
 * @brief 初始化红外循迹传感器（GPIO模式）
 * @param ir       传感器结构体指针
 * @param ports    GPIO端口数组
 * @param pins     GPIO引脚数组
 * @param count    传感器路数
 * @return ErrorCode_t
 */
ErrorCode_t SensorIR_InitGPIO(SensorIR_t *ir, GPIO_TypeDef **ports,
                              const uint16_t *pins, uint8_t count);

/**
 * @brief 设置自定义权重
 * @param ir       传感器结构体指针
 * @param weights  权重数组
 * @return ErrorCode_t
 */
ErrorCode_t SensorIR_SetWeights(SensorIR_t *ir, const int8_t *weights);

/**
 * @brief 读取传感器数据并计算位置（需周期性调用，建议5~10ms）
 * @param ir  传感器结构体指针
 * @return ErrorCode_t
 * @details ADC模式：读取各路ADC值，与阈值比较得到数字状态，
 *          加权平均计算位置。
 *          GPIO模式：直接读取各路GPIO状态。
 *          更新 on_line 和 cross_detected 标志。
 */
ErrorCode_t SensorIR_Update(SensorIR_t *ir);

/**
 * @brief 获取当前位置
 * @param ir  传感器结构体指针
 * @return float: 位置值，0=中心，负=偏左，正=偏右
 *         5路权重{-4,-2,0,2,4}时，范围约-4~+4
 */
float SensorIR_GetPosition(const SensorIR_t *ir);

/**
 * @brief 是否在线上
 * @param ir  传感器结构体指针
 * @return bool: true=至少一个传感器检测到黑线
 */
bool SensorIR_IsOnLine(const SensorIR_t *ir);

/**
 * @brief 是否检测到十字路口（所有传感器都在黑线上）
 * @param ir  传感器结构体指针
 * @return bool: true=十字路口
 */
bool SensorIR_IsCrossDetected(const SensorIR_t *ir);

/**
 * @brief 获取某路传感器原始ADC值
 * @param ir     传感器结构体指针
 * @param index  传感器索引(0起)
 * @return uint16_t: ADC原始值(ADC模式) 或 0/1(GPIO模式)
 */
uint16_t SensorIR_GetRaw(const SensorIR_t *ir, uint8_t index);

/**
 * @brief 获取某路传感器数字状态
 * @param ir     传感器结构体指针
 * @param index  传感器索引(0起)
 * @return bool: true=白地(未在线上), false=黑线(在线上)
 * @note   注意极性：TCRT5000在黑线上输出低电平
 */
bool SensorIR_GetDigital(const SensorIR_t *ir, uint8_t index);

#endif /* __SENSOR_IR_H */
