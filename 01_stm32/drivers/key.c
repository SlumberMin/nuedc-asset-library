/**
 * @file    key.c
 * @brief   按键驱动模块实现
 * @details 状态机实现消抖和边沿检测。
 *          状态转移：IDLE→DEBOUNCE→PRESSED→LONG_PRESS
 *                                  ↘ RELEASE→IDLE
 */

#include "drivers/key.h"

/* ========================================================================== */
/*                              内部函数                                       */
/* ========================================================================== */

/**
 * @brief 读取按键原始电平并判断是否有效按下（内部函数）
 * @param key  按键结构体指针
 * @return bool: true=当前处于有效按下状态
 */
static bool Key_ReadRaw(const Key_t *key)
{
    GPIO_PinState state = GPIO_READ(key->port, key->pin);
    bool level = (state == GPIO_PIN_SET);

    if (key->active_level == KEY_ACTIVE_HIGH) {
        return level;
    } else {
        return !level;
    }
}

/* ========================================================================== */
/*                              接口函数实现                                   */
/* ========================================================================== */

ErrorCode_t Key_Init(Key_t *key, GPIO_TypeDef *port, uint16_t pin,
                     KeyActiveLevel_t active_level)
{
    if (key == NULL || port == NULL) {
        return HAL_ERR_PARAM;
    }

    key->port         = port;
    key->pin          = pin;
    key->active_level = active_level;
    key->state        = KEY_STATE_IDLE;
    key->event        = KEY_EVENT_NONE;
    key->press_tick   = 0;
    key->debounce_tick = 0;
    key->last_raw     = false;
    key->initialized  = true;

    return HAL_OK_CODE;
}

ErrorCode_t KeyMgr_Init(KeyMgr_t *mgr)
{
    if (mgr == NULL) return HAL_ERR_PARAM;

    mgr->count = 0;
    memset(mgr->keys, 0, sizeof(mgr->keys));

    return HAL_OK_CODE;
}

ErrorCode_t KeyMgr_Add(KeyMgr_t *mgr, GPIO_TypeDef *port, uint16_t pin,
                       KeyActiveLevel_t active_level)
{
    if (mgr == NULL || mgr->count >= KEY_MAX_COUNT) {
        return HAL_ERR_OVERFLOW;
    }

    ErrorCode_t ret = Key_Init(&mgr->keys[mgr->count], port, pin, active_level);
    if (ret == HAL_OK_CODE) {
        mgr->count++;
    }

    return ret;
}

ErrorCode_t KeyMgr_Scan(KeyMgr_t *mgr)
{
    if (mgr == NULL) return HAL_ERR_PARAM;

    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < mgr->count; i++) {
        Key_t *key = &mgr->keys[i];
        if (!key->initialized) continue;

        bool pressed = Key_ReadRaw(key);

        switch (key->state) {
            case KEY_STATE_IDLE:
                if (pressed) {
                    key->state        = KEY_STATE_DEBOUNCE;
                    key->debounce_tick = now;
                }
                break;

            case KEY_STATE_DEBOUNCE:
                if ((now - key->debounce_tick) >= KEY_DEBOUNCE_MS) {
                    if (pressed) {
                        /* 消抖确认：确实按下了 */
                        key->state      = KEY_STATE_PRESSED;
                        key->press_tick  = now;
                        key->event       = KEY_EVENT_PRESS;
                    } else {
                        /* 消抖失败：回到空闲 */
                        key->state = KEY_STATE_IDLE;
                    }
                }
                break;

            case KEY_STATE_PRESSED:
                if (pressed) {
                    /* 持续按下，检查是否达到长按阈值 */
                    if ((now - key->press_tick) >= KEY_LONG_PRESS_MS) {
                        key->state = KEY_STATE_LONG_PRESS;
                        key->event = KEY_EVENT_LONG_PRESS;
                    }
                } else {
                    /* 松开 */
                    key->state = KEY_STATE_IDLE;
                    key->event = KEY_EVENT_RELEASE;
                }
                break;

            case KEY_STATE_LONG_PRESS:
                if (!pressed) {
                    key->state = KEY_STATE_IDLE;
                    key->event = KEY_EVENT_RELEASE;
                }
                /* 长按期间不重复触发长按事件 */
                break;
        }

        key->last_raw = pressed;
    }

    return HAL_OK_CODE;
}

KeyEvent_t Key_GetEvent(Key_t *key)
{
    if (key == NULL) return KEY_EVENT_NONE;

    KeyEvent_t evt = key->event;
    key->event = KEY_EVENT_NONE; /* 读后清除 */
    return evt;
}

bool Key_IsPressed(const Key_t *key)
{
    if (key == NULL) return false;
    return (key->state == KEY_STATE_PRESSED || key->state == KEY_STATE_LONG_PRESS);
}

bool Key_IsLongPressed(const Key_t *key)
{
    if (key == NULL) return false;
    return (key->state == KEY_STATE_LONG_PRESS);
}
