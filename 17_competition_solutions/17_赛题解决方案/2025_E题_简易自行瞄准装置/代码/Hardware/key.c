/**
 * @file    key.c
 * @brief   按键模块实现
 * 
 * 硬件连接：
 * KEY1 → PB12 (切换圈数)
 * KEY2 → PB13 (启动/停止)
 * KEY3 → PB14 (切换模式)
 * KEY4 → PB15 (手动激光)
 * 按键接地，GPIO配置上拉输入
 */

#include "key.h"
#include "msp.h"

#define KEY_PORT    GPIOB
#define KEY1_PIN    BIT12
#define KEY2_PIN    BIT13
#define KEY3_PIN    BIT14
#define KEY4_PIN    BIT15
#define KEY_ALL     (KEY1_PIN|KEY2_PIN|KEY3_PIN|KEY4_PIN)

/* 消抖时间(ms) */
#define KEY_DEBOUNCE_MS  20

/**
 * @brief  按键初始化
 * @param  无
 * @retval 无
 */
void Key_Init(void)
{
    /* PB12-PB15配置为输入，上拉 */
    KEY_PORT->DIR &= ~KEY_ALL;
    KEY_PORT->REN |= KEY_ALL;
    KEY_PORT->OUT |= KEY_ALL;
}

/**
 * @brief  按键扫描（带消抖）
 * @param  无
 * @retval uint8_t: 按键编号(KEY1_PRESS~KEY4_PRESS)，无按键返回KEY_NONE
 * 
 * 扫描逻辑：
 * 1. 读取GPIO状态
 * 2. 检测下降沿（按下）
 * 3. 延时消抖
 * 4. 再次确认
 */
uint8_t Key_Scan(void)
{
    static uint8_t key_last = 0xFF;
    uint8_t key_now;
    uint8_t key_val = KEY_NONE;
    
    /* 读取按键状态（低电平有效） */
    key_now = (uint8_t)((KEY_PORT->IN & KEY_ALL) >> 12);
    
    /* 检测下降沿 */
    if((key_now & KEY1_PIN >> 12) == 0 && (key_last & KEY1_PIN >> 12))
        key_val = KEY1_PRESS;
    else if((key_now & KEY2_PIN >> 12) == 0 && (key_last & KEY2_PIN >> 12))
        key_val = KEY2_PRESS;
    else if((key_now & KEY3_PIN >> 12) == 0 && (key_last & KEY3_PIN >> 12))
        key_val = KEY3_PRESS;
    else if((key_now & KEY4_PIN >> 12) == 0 && (key_last & KEY4_PIN >> 12))
        key_val = KEY4_PRESS;
    
    key_last = key_now;
    
    return key_val;
}
