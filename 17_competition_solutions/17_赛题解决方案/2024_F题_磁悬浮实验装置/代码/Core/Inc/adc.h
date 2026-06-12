/**
 * @file    adc.h
 * @brief   ADC模块头文件
 * @version 1.0
 */

#ifndef __ADC_H
#define __ADC_H

#include "stm32f1xx_hal.h"

/* ADC通道定义 */
#define ADC_CH_HALL1        ADC_CHANNEL_0   // PA0 - 霍尔传感器1
#define ADC_CH_HALL2        ADC_CHANNEL_1   // PA1 - 霍尔传感器2
#define ADC_CH_HALL3        ADC_CHANNEL_2   // PA2 - 霍尔传感器3
#define ADC_CH_HALL4        ADC_CHANNEL_3   // PA3 - 霍尔传感器4
#define ADC_CHANNEL_COUNT   4

/* 外部变量声明 */
extern ADC_HandleTypeDef hadc1;

/* 函数声明 */
void MX_ADC1_Init(void);
void ADC_ReadAll(uint16_t *adc_values);
uint16_t ADC_ReadChannel(uint32_t channel);

#endif /* __ADC_H */
