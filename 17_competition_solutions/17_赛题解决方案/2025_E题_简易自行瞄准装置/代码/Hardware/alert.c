/**
 * @file    alert.c
 * @brief   声光提示模块实现
 * 
 * 硬件连接：
 * Buzzer → PB8 (TIM4_CH3 PWM驱动)
 * LED1(红) → PB9
 * LED2(绿) → PB10
 * LED3(蓝) → PB11
 */

#include "alert.h"
#include "msp.h"

#define BUZZER_PORT GPIOB
#define BUZZER_PIN  BIT8
#define LED_PORT    GPIOB
#define LED1_PIN    BIT9
#define LED2_PIN    BIT10
#define LED3_PIN    BIT11

/**
 * @brief  声光模块初始化
 * @param  无
 * @retval 无
 */
void Alert_Init(void)
{
    /* LED引脚配置为输出 */
    LED_PORT->DIR |= (LED1_PIN|LED2_PIN|LED3_PIN);
    LED_PORT->OUT &= ~(LED1_PIN|LED2_PIN|LED3_PIN);  // 初始关闭
    
    /* 蜂鸣器引脚配置为输出 */
    BUZZER_PORT->DIR |= BUZZER_PIN;
    BUZZER_PORT->OUT &= ~BUZZER_PIN;   // 初始静音
}

/**
 * @brief  蜂鸣器鸣响指定时间
 * @param  ms: 鸣响时间(ms)
 * @retval 无
 */
void Alert_Beep(uint16_t ms)
{
    BUZZER_PORT->OUT |= BUZZER_PIN;    // 开启蜂鸣器
    /* 延时 */
    volatile uint32_t count = (uint32_t)ms * 8000;  // 约80MHz下的循环次数
    while(count--);
    BUZZER_PORT->OUT &= ~BUZZER_PIN;   // 关闭蜂鸣器
}

/**
 * @brief  短提示音(50ms)
 * @param  无
 * @retval 无
 */
void Alert_ShortBeep(void)
{
    Alert_Beep(50);
}

/**
 * @brief  错误报警（连续短促鸣响）
 * @param  无
 * @retval 无
 */
void Alert_Error(void)
{
    for(uint8_t i = 0; i < 3; i++)
    {
        Alert_Beep(100);
        /* 间隔100ms */
        volatile uint32_t count = 800000;
        while(count--);
    }
}

/**
 * @brief  点亮指定LED
 * @param  led: LED编号(1-3)
 * @retval 无
 */
void Alert_LED_On(uint8_t led)
{
    switch(led)
    {
        case 1: LED_PORT->OUT |= LED1_PIN; break;
        case 2: LED_PORT->OUT |= LED2_PIN; break;
        case 3: LED_PORT->OUT |= LED3_PIN; break;
    }
}

/**
 * @brief  关闭指定LED
 * @param  led: LED编号(1-3)
 * @retval 无
 */
void Alert_LED_Off(uint8_t led)
{
    switch(led)
    {
        case 1: LED_PORT->OUT &= ~LED1_PIN; break;
        case 2: LED_PORT->OUT &= ~LED2_PIN; break;
        case 3: LED_PORT->OUT &= ~LED3_PIN; break;
    }
}
