/**
 * @file    key.h
 * @brief   按键驱动模块 — STM32电赛通用代码库
 * @details GPIO输入 + 20ms软件消抖 + 边沿检测。
 *          支持按下/松开/长按事件检测。
 * @author  电赛通用代码库
 * @version 1.0
 * @date    2026-06
 */

#ifndef __KEY_H
#define __KEY_H

#include "platform/hal_stm32.h"

/* ========================================================================== */
/*                              常量定义                                       */
/* ========================================================================== */

/** @brief 消抖时间(ms) */
#define KEY_DEBOUNCE_MS     20

/** @brief 长按判定时间(ms) */
#define KEY_LONG_PRESS_MS   1000

/** @brief 最大支持按键数量 */
#define KEY_MAX_COUNT       8

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/**
 * @brief 按键有效电平
 */
typedef enum {
    KEY_ACTIVE_LOW  = 0,    /**< 低电平有效（按下为低） */
    KEY_ACTIVE_HIGH,        /**< 高电平有效（按下为高） */
} KeyActiveLevel_t;

/**
 * @brief 按键事件类型
 */
typedef enum {
    KEY_EVENT_NONE      = 0x00, /**< 无事件 */
    KEY_EVENT_PRESS     = 0x01, /**< 按下事件（边沿触发，仅一次） */
    KEY_EVENT_RELEASE   = 0x02, /**< 松开事件（边沿触发，仅一次） */
    KEY_EVENT_LONG_PRESS = 0x04, /**< 长按事件（持续按住超过阈值） */
} KeyEvent_t;

/**
 * @brief 按键状态
 */
typedef enum {
    KEY_STATE_IDLE = 0,     /**< 空闲 */
    KEY_STATE_DEBOUNCE,     /**< 消抖中 */
    KEY_STATE_PRESSED,      /**< 已按下（消抖确认） */
    KEY_STATE_LONG_PRESS,   /**< 长按状态 */
} KeyState_t;

/**
 * @brief 单个按键实例
 */
typedef struct {
    GPIO_TypeDef       *port;          /**< GPIO端口 */
    uint16_t            pin;           /**< GPIO引脚 */
    KeyActiveLevel_t    active_level;  /**< 有效电平 */
    KeyState_t          state;         /**< 当前状态 */
    KeyEvent_t          event;         /**< 当前事件（读取后自动清除） */
    uint32_t            press_tick;    /**< 按下时刻(ms) */
    uint32_t            debounce_tick; /**< 进入消抖时刻(ms) */
    bool                last_raw;      /**< 上次原始电平 */
    bool                initialized;   /**< 是否已初始化 */
} Key_t;

/**
 * @brief 按键管理器
 */
typedef struct {
    Key_t   keys[KEY_MAX_COUNT];
    uint8_t count;
} KeyMgr_t;

/* ========================================================================== */
/*                              接口函数                                       */
/* ========================================================================== */

/**
 * @brief 初始化单个按键
 * @param key           按键结构体指针
 * @param port          GPIO端口
 * @param pin           GPIO引脚
 * @param active_level  有效电平（按下时的电平）
 * @return ErrorCode_t: HAL_OK_CODE=成功
 * @note   GPIO需在CubeMX中配置为输入模式（上拉/下拉根据电路决定）
 *         有效低电平(active_level=KEY_ACTIVE_LOW)：内部上拉，按下接地
 */
ErrorCode_t Key_Init(Key_t *key, GPIO_TypeDef *port, uint16_t pin,
                     KeyActiveLevel_t active_level);

/**
 * @brief 扫描更新所有按键状态（需周期性调用，建议5~10ms调用一次）
 * @param mgr  按键管理器指针
 * @return ErrorCode_t
 * @note   此函数实现状态机：
 *         IDLE → 检测到有效电平 → DEBOUNCE → 20ms后确认 → PRESSED
 *         PRESSED → 持续按下超阈值 → LONG_PRESS
 *         PRESSED → 检测到松开 → RELEASE → IDLE
 */
ErrorCode_t KeyMgr_Scan(KeyMgr_t *mgr);

/**
 * @brief 初始化按键管理器
 * @param mgr  按键管理器指针
 * @return ErrorCode_t
 */
ErrorCode_t KeyMgr_Init(KeyMgr_t *mgr);

/**
 * @brief 向管理器添加按键
 * @param mgr           按键管理器指针
 * @param port          GPIO端口
 * @param pin           GPIO引脚
 * @param active_level  有效电平
 * @return ErrorCode_t
 */
ErrorCode_t KeyMgr_Add(KeyMgr_t *mgr, GPIO_TypeDef *port, uint16_t pin,
                       KeyActiveLevel_t active_level);

/**
 * @brief 获取指定按键的当前事件
 * @param key    按键结构体指针
 * @return KeyEvent_t: 事件类型（读取后自动清除为KEY_EVENT_NONE）
 */
KeyEvent_t Key_GetEvent(Key_t *key);

/**
 * @brief 按键是否处于按下状态
 * @param key  按键结构体指针
 * @return bool: true=当前按下
 */
bool Key_IsPressed(const Key_t *key);

/**
 * @brief 按键是否处于长按状态
 * @param key  按键结构体指针
 * @return bool: true=长按
 */
bool Key_IsLongPressed(const Key_t *key);

#endif /* __KEY_H */
