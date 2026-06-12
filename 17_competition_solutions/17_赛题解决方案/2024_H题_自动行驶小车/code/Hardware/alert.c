/**
 * @file    alert.c
 * @brief   声光提示模块 - 蜂鸣器和LED控制
 * @author  电赛团队
 * @date    2024
 * @note    蜂鸣器: PB8 (NPN三极管驱动有源蜂鸣器)
 *          LED:    PB9 (红色LED，220Ω限流电阻)
 *
 * 电路说明：
 *   PB8 → 1K电阻 → S8050基极
 *         S8050发射极 → GND
 *         S8050集电极 → 蜂鸣器 → 5V
 *
 *   PB9 → 220Ω电阻 → LED → GND
 */

#include "alert.h"

/* ========================================================================== */
/*                              私有变量                                       */
/* ========================================================================== */

static uint8_t  alert_active = 0;       /* 提示激活标志 */
static uint32_t alert_timer = 0;        /* 提示计时器(ms) */
static uint32_t alert_duration = 0;     /* 提示持续时间(ms) */

/* ========================================================================== */
/*                              公有函数                                       */
/* ========================================================================== */

/**
 * @brief  声光提示模块初始化
 */
void Alert_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIOB时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 配置PB8(蜂鸣器)和PB9(LED)为推挽输出 */
    GPIO_InitStruct.Pin = BUZZER_PIN | LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 初始状态：关闭 */
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

    alert_active = 0;
    alert_timer = 0;
    alert_duration = 0;
}

/**
 * @brief  触发声光提示
 * @param  duration_ms: 提示持续时间(ms)
 */
void Alert_Start(uint32_t duration_ms)
{
    alert_active = 1;
    alert_duration = duration_ms;
    alert_timer = 0;

    /* 立即开启蜂鸣器和LED */
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

/**
 * @brief  停止声光提示
 */
void Alert_Stop(void)
{
    alert_active = 0;
    alert_timer = 0;
    alert_duration = 0;

    /* 关闭蜂鸣器和LED */
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  声光提示处理函数
 * @note   在主循环或定时中断中调用（建议1ms或10ms周期）
 *         管理提示定时，时间到自动关闭
 */
void Alert_Process(void)
{
    if (alert_active)
    {
        alert_timer += 10;      /* 假设每10ms调用一次 */

        if (alert_timer >= alert_duration)
        {
            /* 提示时间到，自动关闭 */
            Alert_Stop();
        }
    }
}

/**
 * @brief  检查是否正在提示
 */
uint8_t Alert_IsActive(void)
{
    return alert_active;
}

/**
 * @brief  蜂鸣器开启
 */
void Alert_BuzzerOn(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

/**
 * @brief  蜂鸣器关闭
 */
void Alert_BuzzerOff(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  LED开启
 */
void Alert_LEDOn(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

/**
 * @brief  LED关闭
 */
void Alert_LEDOff(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  LED闪烁
 * @param  times: 闪烁次数
 * @param  interval_ms: 每次亮/灭间隔(ms)
 * @note   阻塞函数，用于初始化阶段的状态指示
 */
void Alert_LEDBlink(uint8_t times, uint16_t interval_ms)
{
    for (uint8_t i = 0; i < times; i++)
    {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(interval_ms);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(interval_ms);
    }
}
