/**
 * @file    encoder.c
 * @brief   编码器模块实现
 * 
 * 使用TIM3和TIM4的编码器模式，四倍频计数
 * 每10ms更新一次速度（在TIM2中断中调用Encoder_Update）
 * 
 * 速度计算公式：
 * speed(cm/s) = (pulse_count / PPR_TOTAL) * π * D * (1000/update_period_ms)
 */

#include "encoder.h"
#include "microcontroller_hal.h"
#include <math.h>

/* 编码器内部状态 */
static volatile int32_t g_left_count = 0;       // 左编码器累计脉冲
static volatile int32_t g_right_count = 0;      // 右编码器累计脉冲
static volatile int32_t g_left_count_prev = 0;  // 上次左编码器计数
static volatile int32_t g_right_count_prev = 0; // 上次右编码器计数
static volatile float g_left_speed = 0.0f;      // 左轮速度(cm/s)
static volatile float g_right_speed = 0.0f;     // 右轮速度(cm/s)
static volatile float g_left_distance = 0.0f;   // 左轮累计距离(cm)
static volatile float g_right_distance = 0.0f;  // 右轮累计距离(cm)

/* 距离常量 */
#define DISTANCE_PER_PULSE  ((float)(WHEEL_DIAMETER_MM * 3.14159f) / ENCODER_PPR_TOTAL / 10.0f)  // cm/pulse
#define UPDATE_PERIOD_MS    10      // 更新周期(ms)

/**
 * @brief  编码器初始化
 * 
 * 配置TIM3和TIM4为编码器模式
 * TIM3: 左编码器（PA6/PA7）
 * TIM4: 右编码器（PB6/PB7）
 */
void Encoder_Init(void)
{
    /* TIM3编码器模式初始化 */
    #ifdef PLATFORM_STM32
    /* STM32 HAL编码器模式配置 */
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;  // 四倍频
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0x0F;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0x0F;
    HAL_TIM_Encoder_Init(&htim3, &sConfig);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    
    /* TIM4编码器模式初始化（类似TIM3） */
    /* ... 省略类似配置 ... */
    #endif
    
    /* 初始化状态 */
    g_left_count = 0;
    g_right_count = 0;
    g_left_count_prev = 0;
    g_right_count_prev = 0;
    g_left_speed = 0;
    g_right_speed = 0;
    g_left_distance = 0;
    g_right_distance = 0;
}

/**
 * @brief  更新编码器数据（在定时中断中调用，每10ms）
 * 
 * 功能：
 * 1. 读取编码器计数值
 * 2. 计算速度（脉冲差 → cm/s）
 * 3. 累计距离
 */
void Encoder_Update(void)
{
    #ifdef PLATFORM_STM32
    g_left_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    g_right_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    #endif
    
    /* 计算脉冲增量 */
    int32_t left_delta = g_left_count - g_left_count_prev;
    int32_t right_delta = g_right_count - g_right_count_prev;
    
    /* 处理计数器溢出 */
    if(left_delta > 32767) left_delta -= 65536;
    if(left_delta < -32768) left_delta += 65536;
    if(right_delta > 32767) right_delta -= 65536;
    if(right_delta < -32768) right_delta += 65536;
    
    /* 计算速度(cm/s) */
    g_left_speed = (float)left_delta * DISTANCE_PER_PULSE * (1000.0f / UPDATE_PERIOD_MS);
    g_right_speed = (float)right_delta * DISTANCE_PER_PULSE * (1000.0f / UPDATE_PERIOD_MS);
    
    /* 累计距离(cm) */
    g_left_distance += (float)left_delta * DISTANCE_PER_PULSE;
    g_right_distance += (float)right_delta * DISTANCE_PER_PULSE;
    
    /* 保存当前计数 */
    g_left_count_prev = g_left_count;
    g_right_count_prev = g_right_count;
}

/**
 * @brief  获取左轮速度
 * @retval float: 左轮速度(cm/s)
 */
float Encoder_GetLeftSpeed(void)
{
    return g_left_speed;
}

/**
 * @brief  获取右轮速度
 * @retval float: 右轮速度(cm/s)
 */
float Encoder_GetRightSpeed(void)
{
    return g_right_speed;
}

/**
 * @brief  获取平均速度
 * @retval float: 平均速度(cm/s)
 */
float Encoder_GetAvgSpeed(void)
{
    return (g_left_speed + g_right_speed) * 0.5f;
}

/**
 * @brief  获取左轮累计距离
 * @retval float: 左轮累计距离(cm)
 */
float Encoder_GetLeftDistance(void)
{
    return g_left_distance;
}

/**
 * @brief  获取右轮累计距离
 * @retval float: 右轮累计距离(cm)
 */
float Encoder_GetRightDistance(void)
{
    return g_right_distance;
}

/**
 * @brief  获取平均累计距离
 * @retval float: 平均累计距离(cm)
 */
float Encoder_GetAvgDistance(void)
{
    return (g_left_distance + g_right_distance) * 0.5f;
}

/**
 * @brief  重置累计距离
 */
void Encoder_ResetDistance(void)
{
    g_left_distance = 0;
    g_right_distance = 0;
}

/**
 * @brief  获取左编码器原始计数
 */
int32_t Encoder_GetLeftCount(void)
{
    return g_left_count;
}

/**
 * @brief  获取右编码器原始计数
 */
int32_t Encoder_GetRightCount(void)
{
    return g_right_count;
}
