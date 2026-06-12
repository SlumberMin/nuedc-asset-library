/**
 * @file    key.c
 * @brief   按键模块实现 - 磁悬浮实验装置
 * 
 * 引脚：PB12-PB15，上拉输入，低电平有效
 * 含20ms消抖处理
 */

#include "key.h"

#define KEY_PORT    GPIOB
#define KEY1_PIN    GPIO_PIN_12
#define KEY2_PIN    GPIO_PIN_13
#define KEY3_PIN    GPIO_PIN_14
#define KEY4_PIN    GPIO_PIN_15
#define KEY_ALL     (KEY1_PIN|KEY2_PIN|KEY3_PIN|KEY4_PIN)

void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = KEY_ALL;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY_PORT, &GPIO_InitStruct);
}

uint8_t Key_Scan(void)
{
    static uint8_t key_last = 0xFF;
    uint8_t key_now = 0;
    uint8_t key_press;
    
    /* 读取当前状态 */
    if(HAL_GPIO_ReadPin(KEY_PORT, KEY1_PIN) == GPIO_PIN_RESET) key_now |= 0x01;
    if(HAL_GPIO_ReadPin(KEY_PORT, KEY2_PIN) == GPIO_PIN_RESET) key_now |= 0x02;
    if(HAL_GPIO_ReadPin(KEY_PORT, KEY3_PIN) == GPIO_PIN_RESET) key_now |= 0x04;
    if(HAL_GPIO_ReadPin(KEY_PORT, KEY4_PIN) == GPIO_PIN_RESET) key_now |= 0x08;
    
    /* 边沿检测（下降沿） */
    key_press = key_now & (~key_last);
    key_last = key_now;
    
    /* 消抖：检测到按下后延时20ms再确认 */
    if(key_press)
    {
        HAL_Delay(20);
        key_now = 0;
        if(HAL_GPIO_ReadPin(KEY_PORT, KEY1_PIN) == GPIO_PIN_RESET) key_now |= 0x01;
        if(HAL_GPIO_ReadPin(KEY_PORT, KEY2_PIN) == GPIO_PIN_RESET) key_now |= 0x02;
        if(HAL_GPIO_ReadPin(KEY_PORT, KEY3_PIN) == GPIO_PIN_RESET) key_now |= 0x04;
        if(HAL_GPIO_ReadPin(KEY_PORT, KEY4_PIN) == GPIO_PIN_RESET) key_now |= 0x08;
        key_press = key_now & (~key_last);
        key_last = key_now;
    }
    
    if(key_press & 0x01) return KEY1_PRESS;
    if(key_press & 0x02) return KEY2_PRESS;
    if(key_press & 0x04) return KEY3_PRESS;
    if(key_press & 0x08) return KEY4_PRESS;
    return KEY_NONE;
}
