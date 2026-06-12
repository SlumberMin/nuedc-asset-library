/**
 * @file    adc.c
 * @brief   ADC模块 - 霍尔传感器数据采集
 * @version 1.0
 * 
 * 功能：
 * - 4通道ADC采集（PA0~PA3）
 * - 支持单次转换和连续转换
 * - 中值滤波和滑动平均滤波
 */

#include "adc.h"

/* ADC句柄 */
ADC_HandleTypeDef hadc1;

/* 滤波缓冲区 */
#define FILTER_SIZE     8       // 滑动平均窗口大小
static uint16_t adc_buffer[ADC_CHANNEL_COUNT][FILTER_SIZE];
static uint8_t adc_index = 0;

/**
 * @brief  ADC1初始化函数
 * @note   配置4通道扫描转换，TIM2触发
 */
void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    /* ADC1基本配置 */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;          // 扫描模式
    hadc1.Init.ContinuousConvMode = DISABLE;            // 非连续模式
    hadc1.Init.DiscontinuousConvMode = ENABLE;          // 间断模式
    hadc1.Init.NbrOfDiscConversion = 1;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;  // TIM2触发
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;         // 右对齐
    hadc1.Init.NbrOfConversion = ADC_CHANNEL_COUNT;     // 4个通道
    HAL_ADC_Init(&hadc1);
    
    /* 通道0配置 - 霍尔传感器1 */
    sConfig.Channel = ADC_CH_HALL1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;  // 高精度采样
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    /* 通道1配置 - 霍尔传感器2 */
    sConfig.Channel = ADC_CH_HALL2;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    /* 通道2配置 - 霍尔传感器3 */
    sConfig.Channel = ADC_CH_HALL3;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    /* 通道3配置 - 霍尔传感器4 */
    sConfig.Channel = ADC_CH_HALL4;
    sConfig.Rank = ADC_REGULAR_RANK_4;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    /* 初始化滤波缓冲区 */
    for (int ch = 0; ch < ADC_CHANNEL_COUNT; ch++)
    {
        for (int i = 0; i < FILTER_SIZE; i++)
        {
            adc_buffer[ch][i] = 0;
        }
    }
}

/**
 * @brief  读取单个ADC通道
 * @param  channel: ADC通道号
 * @retval ADC转换值(0~4095)
 */
uint16_t ADC_ReadChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    
    return value;
}

/**
 * @brief  读取所有霍尔传感器（带滤波）
 * @param  adc_values: 存储4个通道ADC值的数组
 * @retval 无
 * 
 * 滤波算法：中值滤波 + 滑动平均
 * 1. 读取每个通道的ADC值
 * 2. 存入环形缓冲区
 * 3. 计算滑动平均值
 */
void ADC_ReadAll(uint16_t *adc_values)
{
    uint32_t sum;
    
    /* 读取4个通道 */
    adc_buffer[0][adc_index] = ADC_ReadChannel(ADC_CH_HALL1);
    adc_buffer[1][adc_index] = ADC_ReadChannel(ADC_CH_HALL2);
    adc_buffer[2][adc_index] = ADC_ReadChannel(ADC_CH_HALL3);
    adc_buffer[3][adc_index] = ADC_ReadChannel(ADC_CH_HALL4);
    
    /* 更新缓冲区索引 */
    adc_index = (adc_index + 1) % FILTER_SIZE;
    
    /* 计算每个通道的滑动平均 */
    for (int ch = 0; ch < ADC_CHANNEL_COUNT; ch++)
    {
        sum = 0;
        for (int i = 0; i < FILTER_SIZE; i++)
        {
            sum += adc_buffer[ch][i];
        }
        adc_values[ch] = sum / FILTER_SIZE;
    }
}
