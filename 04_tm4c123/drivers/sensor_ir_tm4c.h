/**
 * @file sensor_ir_tm4c.h
 * @brief TM4C123 红外循迹传感器驱动 (ADC序列采样)
 *
 * 使用ADC0的SS3(Sequence 3)进行8通道扫描。
 * 支持5路/8路红外传感器阵列(如TCRT5000)。
 * PE0~PE5 (AIN3~AIN8) 用于传感器输入。
 *
 * 提供原始值、归一化值、加权位置计算。
 */
#ifndef __SENSOR_IR_TM4C_H
#define __SENSOR_IR_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 配置 ======================== */
#define IR_SENSOR_COUNT         8       /* 传感器数量 */
#define IR_ADC_BASE             ADC0_BASE
#define IR_ADC_SEQUENCE         3       /* 使用SS3 */

/* 传感器通道(AIN编号) -> PE引脚 */
/* AIN0=PE3, AIN1=PE2, AIN2=PE1, AIN3=PE0, AIN4=PD3, AIN5=PD2, AIN6=PD1, AIN7=PD0 */
#define IR_ADC_CHANNELS         { ADC_CTL_CH3, ADC_CTL_CH2, ADC_CTL_CH1, ADC_CTL_CH0, \
                                  ADC_CTL_CH7, ADC_CTL_CH6, ADC_CTL_CH5, ADC_CTL_CH4 }

/* 加权循迹位置 */
#define IR_POS_LEFT_MAX     (-3500)     /* 最左 */
#define IR_POS_RIGHT_MAX    (3500)      /* 最右 */
#define IR_POS_CENTER       (0)         /* 中心 */

/* ======================== 数据结构 ======================== */
typedef struct {
    uint16_t raw[IR_SENSOR_COUNT];      /* 原始ADC值 (0~4095) */
    float    norm[IR_SENSOR_COUNT];     /* 归一化值 (0.0~1.0) */
    int16_t  position;                  /* 加权位置 (-3500~+3500) */
    uint8_t  active_mask;               /* 激活传感器位掩码 */
} ir_data_t;

/* ======================== API ======================== */

/**
 * @brief 初始化红外循迹ADC
 */
void ir_sensor_init(void);

/**
 * @brief 读取所有传感器并更新数据
 * @param data 输出数据结构
 */
void ir_sensor_read(ir_data_t *data);

/**
 * @brief 读取单个传感器原始值
 * @param index 传感器索引 0~7
 * @return ADC原始值 0~4095
 */
uint16_t ir_sensor_read_single(uint8_t index);

/**
 * @brief 获取加权位置(用于PID输入)
 * @return 位置值 -3500(最左) ~ +3500(最右), 0=居中
 */
int16_t ir_sensor_get_position(void);

/**
 * @brief 校准传感器(采集N次取最大/最小值)
 */
void ir_sensor_calibrate(uint16_t samples);

/**
 * @brief 判断是否完全丢失线(全白/全黑)
 */
bool ir_sensor_line_lost(const ir_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_IR_TM4C_H */
