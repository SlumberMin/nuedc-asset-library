/**
 * @file    event_system.h
 * @brief   模块间事件通信系统 — MSPM0G3507
 *
 * 参考国奖作品系统架构:
 *   - 2021年F题 智能送药小车: 模块间事件驱动通信
 *   - 2023年H题 三子棋博弈: 视觉→决策→执行 事件链
 *
 * 特性:
 *   - 事件注册(监听)、触发(发布)、分发(订阅)
 *   - 支持多监听者注册同一事件
 *   - 事件携带可选数据 (指针+长度)
 *   - 同步分发（在Event_Dispatch中执行）
 *   - 事件计数和统计
 *
 * 依赖: <stdint.h>, <stdbool.h>
 */

#ifndef __EVENT_SYSTEM_H
#define __EVENT_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 配置参数 ─────────────────────────────────────────── */
#ifndef EVT_MAX_TYPES
#define EVT_MAX_TYPES           32      /* 最大事件类型数 */
#endif

#ifndef EVT_MAX_LISTENERS
#define EVT_MAX_LISTENERS       32      /* 最大监听者总数 */
#endif

/* ── 预定义系统事件 (用户可扩展, 从EVT_USER_START开始) ── */
typedef enum {
    EVT_NONE = 0,                       /* 无效事件 */
    EVT_SYSTEM_BOOT,                    /* 系统启动 */
    EVT_SYSTEM_ERROR,                   /* 系统错误 */
    EVT_SENSOR_UPDATE,                  /* 传感器数据更新 */
    EVT_MOTOR_CMD,                      /* 电机控制命令 */
    EVT_LINE_LOST,                      /* 循线丢失 */
    EVT_LINE_FOUND,                     /* 循线恢复 */
    EVT_OBSTACLE_DETECTED,              /* 障碍物检测 */
    EVT_OBSTACLE_CLEARED,               /* 障碍物清除 */
    EVT_BUTTON_PRESS,                   /* 按键按下 */
    EVT_UART_RX_DATA,                   /* UART收到数据 */
    EVT_TIMEOUT,                        /* 超时事件 */
    EVT_WATCHDOG_WARN,                  /* 看门狗警告 */
    EVT_WATCHDOG_RESET,                 /* 看门狗复位 */
    EVT_USER_START = 16,                /* 用户自定义事件起始 */
    EVT_TYPE_MAX = EVT_MAX_TYPES
} EventType;

/* ── 事件数据 ──────────────────────────────────────────── */
typedef struct {
    EventType   type;                   /* 事件类型 */
    uint32_t    timestamp;              /* 触发时的tick */
    const void *data;                   /* 附加数据指针 (可为NULL) */
    uint16_t    data_len;               /* 附加数据长度 */
} Event;

/* ── 事件回调函数类型 ──────────────────────────────────── */
typedef void (*EventCallback)(const Event *evt, void *user_data);

/* ── 监听者句柄 ────────────────────────────────────────── */
typedef struct {
    EventType    type;                  /* 监听的事件类型 */
    EventCallback callback;             /* 回调函数 */
    void        *user_data;             /* 用户数据 */
    bool         active;                /* 是否激活 */
} EventListener;

/* ── 事件系统实例 ──────────────────────────────────────── */
typedef struct {
    EventListener   listeners[EVT_MAX_LISTENERS];
    uint8_t         listener_count;
    uint32_t        event_counts[EVT_MAX_TYPES]; /* 每种事件的触发计数 */
    uint32_t        tick_source;        /* tick来源指针 */
    bool            initialized;
} EventSystem;

/* ── 公共API ───────────────────────────────────────────── */

/**
 * @brief 初始化事件系统
 * @param es  事件系统实例指针
 */
void Event_Init(EventSystem *es);

/**
 * @brief 注册事件监听者
 * @param es        事件系统实例
 * @param type      要监听的事件类型
 * @param callback  回调函数
 * @param user_data 用户数据(会传给回调)
 * @return 监听者索引 (>=0成功, <0失败)
 */
int Event_Register(EventSystem *es, EventType type,
                   EventCallback callback, void *user_data);

/**
 * @brief 取消注册
 * @param es     事件系统实例
 * @param index  监听者索引
 */
void Event_Unregister(EventSystem *es, uint8_t index);

/**
 * @brief 触发事件 (无附加数据)
 * @param es    事件系统实例
 * @param type  事件类型
 */
void Event_Trigger(EventSystem *es, EventType type);

/**
 * @brief 触发事件 (带附加数据)
 * @param es        事件系统实例
 * @param type      事件类型
 * @param data      附加数据指针
 * @param data_len  附加数据长度
 */
void Event_TriggerWithData(EventSystem *es, EventType type,
                           const void *data, uint16_t data_len);

/**
 * @brief 获取某类型事件的触发计数
 */
uint32_t Event_GetCount(const EventSystem *es, EventType type);

/**
 * @brief 重置某类型事件计数
 */
void Event_ResetCount(EventSystem *es, EventType type);

/**
 * @brief 打印事件系统状态（调试用）
 */
void Event_PrintStatus(const EventSystem *es);

#ifdef __cplusplus
}
#endif

#endif /* __EVENT_SYSTEM_H */
