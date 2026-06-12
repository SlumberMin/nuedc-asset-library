/**
 * @file    height.c
 * @brief   高度检测模块 - 霍尔传感器数据转换
 * @version 1.0
 * 
 * 功能：
 * - 将ADC值转换为高度值(cm)
 * - 支持标定数据和线性插值
 * - 支持倾斜检测
 */

#include "height.h"
#include <math.h>

/* 标定数据表（需实际标定） */
/* 格式：{ADC值, 高度(cm)} */
static const float calib_height[CALIB_POINTS] = {
    0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f
};

static const uint16_t calib_adc[CALIB_POINTS] = {
    3500, 2800, 2200, 1700, 1300, 1000, 800
};

/* 滤波缓冲区 */
#define HEIGHT_FILTER_SIZE  8
static float height_buffer[HEIGHT_FILTER_SIZE];
static uint8_t height_index = 0;

/**
 * @brief  高度检测模块初始化
 * @retval 无
 */
void Height_Init(void)
{
    /* 初始化滤波缓冲区 */
    for (int i = 0; i < HEIGHT_FILTER_SIZE; i++)
    {
        height_buffer[i] = 0.0f;
    }
    height_index = 0;
}

/**
 * @brief  ADC值转换为高度值（线性插值）
 * @param  adc_val: ADC值(0~4095)
 * @retval 高度值(cm)
 * 
 * 算法：
 * 1. 在标定数据中找到adc_val所在的区间
 * 2. 使用线性插值计算高度
 * 3. 超出范围时返回边界值
 */
static float ADC_To_Height(uint16_t adc_val)
{
    /* 超出标定范围的处理 */
    if (adc_val >= calib_adc[0])
    {
        return calib_height[0];  // 最小高度
    }
    if (adc_val <= calib_adc[CALIB_POINTS - 1])
    {
        return calib_height[CALIB_POINTS - 1];  // 最大高度
    }
    
    /* 线性插值 */
    for (int i = 0; i < CALIB_POINTS - 1; i++)
    {
        if (adc_val <= calib_adc[i] && adc_val >= calib_adc[i + 1])
        {
            float ratio = (float)(calib_adc[i] - adc_val) / 
                         (float)(calib_adc[i] - calib_adc[i + 1]);
            return calib_height[i] + ratio * (calib_height[i + 1] - calib_height[i]);
        }
    }
    
    return 0.0f;  // 不应到达此处
}

/**
 * @brief  计算悬浮盘高度
 * @param  adc_values: 4个霍尔传感器的ADC值数组
 * @retval 高度值(cm)
 * 
 * 算法：
 * 1. 取4个传感器的平均值
 * 2. 转换为高度
 * 3. 滑动平均滤波
 */
float Height_Calculate(uint16_t *adc_values)
{
    uint32_t adc_sum = 0;
    float height;
    
    /* 计算4个传感器的平均ADC值 */
    for (int i = 0; i < 4; i++)
    {
        adc_sum += adc_values[i];
    }
    uint16_t adc_avg = adc_sum / 4;
    
    /* 转换为高度 */
    height = ADC_To_Height(adc_avg);
    
    /* 滑动平均滤波 */
    height_buffer[height_index] = height;
    height_index = (height_index + 1) % HEIGHT_FILTER_SIZE;
    
    float sum = 0.0f;
    for (int i = 0; i < HEIGHT_FILTER_SIZE; i++)
    {
        sum += height_buffer[i];
    }
    
    return sum / HEIGHT_FILTER_SIZE;
}

/**
 * @brief  获取平均高度（滤波后）
 * @retval 高度值(cm)
 */
float Height_GetAverage(void)
{
    float sum = 0.0f;
    for (int i = 0; i < HEIGHT_FILTER_SIZE; i++)
    {
        sum += height_buffer[i];
    }
    return sum / HEIGHT_FILTER_SIZE;
}

/**
 * @brief  计算倾斜角度
 * @retval 倾斜角度(度)
 * 
 * 算法：通过4个传感器的差值计算倾斜
 * 传感器布局：H1(左前) H2(右前) H3(左后) H4(右后)
 */
float Height_GetTilt(void)
{
    /* 此函数需要单独获取各传感器高度，暂留接口 */
    return 0.0f;
}
