/**
 * @file    event_system.c
 * @brief   模块间事件通信系统实现
 */

#include "event_system.h"
#include <stdio.h>
#include <string.h>

/* ── 初始化 ────────────────────────────────────────────── */
void Event_Init(EventSystem *es)
{
    memset(es, 0, sizeof(EventSystem));
    es->initialized = true;
}

/* ── 注册监听者 ────────────────────────────────────────── */
int Event_Register(EventSystem *es, EventType type,
                   EventCallback callback, void *user_data)
{
    if (!es->initialized || callback == NULL) return -1;

    for (uint8_t i = 0; i < EVT_MAX_LISTENERS; i++) {
        if (!es->listeners[i].active) {
            es->listeners[i].type      = type;
            es->listeners[i].callback  = callback;
            es->listeners[i].user_data = user_data;
            es->listeners[i].active    = true;
            es->listener_count++;
            return (int)i;
        }
    }
    return -1; /* 满 */
}

/* ── 取消注册 ──────────────────────────────────────────── */
void Event_Unregister(EventSystem *es, uint8_t index)
{
    if (index >= EVT_MAX_LISTENERS) return;
    if (es->listeners[index].active) {
        es->listeners[index].active = false;
        es->listener_count--;
    }
}

/* ── 触发事件 (无数据) ─────────────────────────────────── */
void Event_Trigger(EventSystem *es, EventType type)
{
    Event_TriggerWithData(es, type, NULL, 0);
}

/* ── 触发事件 (带数据) — 同步分发 ──────────────────────── */
void Event_TriggerWithData(EventSystem *es, EventType type,
                           const void *data, uint16_t data_len)
{
    if (!es->initialized) return;

    /* 记录事件计数 */
    if (type < EVT_MAX_TYPES) {
        es->event_counts[type]++;
    }

    /* 构造事件结构 */
    Event evt;
    evt.type      = type;
    evt.data      = data;
    evt.data_len  = data_len;
    evt.timestamp = 0; /* 由应用层补充tick */

    /* 遍历所有监听者, 找到匹配类型并同步执行 */
    for (uint8_t i = 0; i < EVT_MAX_LISTENERS; i++) {
        EventListener *lis = &es->listeners[i];
        if (lis->active && lis->type == type && lis->callback != NULL) {
            lis->callback(&evt, lis->user_data);
        }
    }
}

/* ── 查询接口 ──────────────────────────────────────────── */
uint32_t Event_GetCount(const EventSystem *es, EventType type)
{
    if (type >= EVT_MAX_TYPES) return 0;
    return es->event_counts[type];
}

void Event_ResetCount(EventSystem *es, EventType type)
{
    if (type < EVT_MAX_TYPES) {
        es->event_counts[type] = 0;
    }
}

/* ── 调试输出 ──────────────────────────────────────────── */
void Event_PrintStatus(const EventSystem *es)
{
    printf("[Event] listeners=%u\r\n", es->listener_count);

    /* 打印有触发记录的事件 */
    for (uint16_t i = 0; i < EVT_MAX_TYPES; i++) {
        if (es->event_counts[i] > 0) {
            printf("  evt[%u] count=%lu\r\n", i,
                   (unsigned long)es->event_counts[i]);
        }
    }
}
