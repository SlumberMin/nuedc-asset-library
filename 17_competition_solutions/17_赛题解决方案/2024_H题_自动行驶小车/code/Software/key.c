/**
 * @file    key.c
 * @brief   按键模块 - 模式选择和启动控制
 * @author  电赛团队
 * @date    2024
 * @note    模式选择按键: PA4 (上拉，按下为低电平)
 *          启动按键:     PA5 (上拉，按下为低电平)
 */

#include "key.h"

/* ========================================================================== */
/*                              私有变量                                       */
/* ========================================================================== */

static RunMode_t current_mode = MODE_IDLE;     /* 当前运行模式 */
static uint8_t key_mode_state = 0;             /* 模式按键状态（消抖后） */
static uint8_t key_start_state = 0;            /* 启动按键状态（消抖后） */
static uint8_t key_mode_last = 0;              /* 模式按键上次状态 */
static uint8_t key_start_last = 0;             /* 启动按键上次状态 */

/* ========================================================================== */
/*                              公有函数                                       */
/* ========================================================================== */

/**
 * @brief  按键模块初始化
 */
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIOA时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 配置PA4(模式键)和PA5(启动键)为上拉输入 */
    GPIO_InitStruct.Pin = KEY_MODE_PIN | KEY_START_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    current_mode = MODE_IDLE;
    key_mode_state = 0;
    key_start_state = 0;
    key_mode_last = 0;
    key_start_last = 0;
}

/**
 * @brief  按键扫描（带消抖和边沿检测）
 * @note   在主循环中调用，建议10ms周期
 * @retval KeyID_t 返回按下边沿的按键ID
 */
KeyID_t Key_Scan(void)
{
    KeyID_t result = KEY_NONE;
    uint8_t key_mode_raw, key_start_raw;

    /* 读取原始按键状态（按下为0，上拉输入） */
    key_mode_raw = (HAL_GPIO_ReadPin(KEY_MODE_PORT, KEY_MODE_PIN) == GPIO_PIN_RESET) ? 1 : 0;
    key_start_raw = (HAL_GPIO_ReadPin(KEY_START_PORT, KEY_START_PIN) == GPIO_PIN_RESET) ? 1 : 0;

    /* 模式按键：检测下降沿（按下瞬间） */
    if (key_mode_raw && !key_mode_last)
    {
        result = KEY_MODE;
    }
    key_mode_last = key_mode_raw;

    /* 启动按键：检测下降沿（按下瞬间） */
    if (key_start_raw && !key_start_last)
    {
        result = KEY_START;
    }
    key_start_last = key_start_raw;

    return result;
}

/**
 * @brief  获取当前运行模式
 */
RunMode_t Key_GetMode(void)
{
    return current_mode;
}

/**
 * @brief  切换到下一个运行模式
 * @retval RunMode_t 切换后的模式
 */
RunMode_t Key_NextMode(void)
{
    current_mode = (RunMode_t)((current_mode + 1) % 5);  /* 4个模式循环 */
    return current_mode;
}
