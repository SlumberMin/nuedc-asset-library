/**
 * @file    encoder.c
 * @brief   编码器模块 - 增量式霍尔编码器接口
 * @author  电赛团队
 * @date    2024
 * @note    左编码器: TIM2 CH1/CH2 (PA0/PA1)
 *          右编码器: TIM3 CH1/CH2 (PA6/PA7)
 *          使用STM32编码器模式，硬件四倍频
 *
 * 编码器参数：
 *   线数: 200线/转
 *   四倍频后: 800脉冲/转
 *   减速比: 34:1
 *   输出轴每转: 800 × 34 = 27200脉冲
 *   轮径: 65mm
 *   每脉冲距离: π × 65 / 27200 ≈ 0.0075mm
 */

#include "encoder.h"

/* ========================================================================== */
/*                              私有变量                                       */
/* ========================================================================== */

static TIM_HandleTypeDef htim2;         /* 左编码器定时器句柄 */
static TIM_HandleTypeDef htim3;         /* 右编码器定时器句柄 */

static Encoder_t encoder_left;          /* 左轮编码器数据 */
static Encoder_t encoder_right;         /* 右轮编码器数据 */

/* 更新周期(ms)，在Encoder_Init中设置 */
static float update_period_ms = 10.0f;

/* ========================================================================== */
/*                              私有函数                                       */
/* ========================================================================== */

/**
 * @brief  配置编码器输入GPIO
 */
static void Encoder_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIOA时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*
     * PA0 -> TIM2_CH1 (左编码器A相)
     * PA1 -> TIM2_CH2 (左编码器B相)
     * PA6 -> TIM3_CH1 (右编码器A相)
     * PA7 -> TIM3_CH2 (右编码器B相)
     *
     * 配置为浮空输入模式（编码器接口需要）
     */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;     /* 编码器有上拉，无需内部上拉 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief  配置TIM2为编码器接口模式（左编码器）
 */
static void Encoder_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};

    /* 使能TIM2时钟 */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2基本配置 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;                          /* 不分频 */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF;                         /* 最大计数值 65535 */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* 编码器模式配置: TI1和TI2双边沿计数（四倍频） */
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;         /* 四倍频模式 */
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0x06;                           /* 滤波，消除毛刺 */
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0x06;

    HAL_TIM_Encoder_Init(&htim2, &sConfig);

    /* 清零计数器 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* 启动编码器 */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

/**
 * @brief  配置TIM3为编码器接口模式（右编码器）
 */
static void Encoder_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};

    /* 使能TIM3时钟 */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* TIM3基本配置 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* 编码器模式配置: TI1和TI2双边沿计数（四倍频） */
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0x06;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0x06;

    HAL_TIM_Encoder_Init(&htim3, &sConfig);

    /* 清零计数器 */
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    /* 启动编码器 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

/* ========================================================================== */
/*                              公有函数                                       */
/* ========================================================================== */

/**
 * @brief  编码器模块初始化
 */
void Encoder_Init(void)
{
    /* 初始化数据结构 */
    encoder_left.count = 0;
    encoder_left.last_count = 0;
    encoder_left.delta = 0;
    encoder_left.speed = 0.0f;
    encoder_left.total_distance = 0.0f;
    encoder_left.direction = 1;

    encoder_right.count = 0;
    encoder_right.last_count = 0;
    encoder_right.delta = 0;
    encoder_right.speed = 0.0f;
    encoder_right.total_distance = 0.0f;
    encoder_right.direction = 1;

    /* 初始化硬件 */
    Encoder_GPIO_Init();
    Encoder_TIM2_Init();
    Encoder_TIM3_Init();
}

/**
 * @brief  更新编码器数据
 * @note   在定时中断中调用，建议10ms周期
 *         处理计数器溢出，计算增量和速度
 */
void Encoder_Update(void)
{
    int16_t temp_count;
    float distance_per_pulse = DISTANCE_PER_PULSE_MM / 10.0f;  /* 转换为cm */

    /* ===== 更新左轮编码器 ===== */
    temp_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);

    /* 处理计数器溢出（16位有符号处理） */
    encoder_left.count += temp_count;
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* 计算增量 */
    encoder_left.delta = temp_count;

    /* 判断方向 */
    encoder_left.direction = (encoder_left.delta >= 0) ? 1 : 0;

    /* 累积距离（取绝对值） */
    encoder_left.total_distance += (float)abs(encoder_left.delta) * distance_per_pulse;

    /* 计算瞬时速度 (cm/s) */
    encoder_left.speed = (float)encoder_left.delta * distance_per_pulse
                         / (update_period_ms / 1000.0f);

    /* ===== 更新右轮编码器 ===== */
    temp_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    encoder_right.count += temp_count;
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    encoder_right.delta = temp_count;
    encoder_right.direction = (encoder_right.delta >= 0) ? 1 : 0;
    encoder_right.total_distance += (float)abs(encoder_right.delta) * distance_per_pulse;
    encoder_right.speed = (float)encoder_right.delta * distance_per_pulse
                          / (update_period_ms / 1000.0f);
}

/**
 * @brief  获取左轮编码器数据
 */
Encoder_t* Encoder_GetLeft(void)
{
    return &encoder_left;
}

/**
 * @brief  获取右轮编码器数据
 */
Encoder_t* Encoder_GetRight(void)
{
    return &encoder_right;
}

/**
 * @brief  获取左右轮平均行驶距离(cm)
 */
float Encoder_GetAvgDistance(void)
{
    return (encoder_left.total_distance + encoder_right.total_distance) / 2.0f;
}

/**
 * @brief  重置编码器累积距离
 */
void Encoder_ResetDistance(void)
{
    encoder_left.total_distance = 0.0f;
    encoder_right.total_distance = 0.0f;
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

/**
 * @brief  获取瞬时速度(cm/s)
 */
float Encoder_GetSpeed(void)
{
    return (encoder_left.speed + encoder_right.speed) / 2.0f;
}
