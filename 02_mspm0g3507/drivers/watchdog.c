/**
 * @file    watchdog.c
 * @brief   软件看门狗实现
 */

#include "watchdog.h"
#include <stdio.h>
#include <string.h>

/* Cortex-M0+ 系统复位: 写 AIRCR 寄存器 */
static inline void system_reset(void)
{
    /* SCB->AIRCR = VECTKEY(0x05FA) | SYSRESETREQ(1<<2) */
    *((volatile uint32_t *)0xE000ED0C) = (0x05FAUL << 16) | (1UL << 2);
    for (;;) {} /* 等待复位生效 */
}

/* ── 初始化 ────────────────────────────────────────────── */
void WDG_Init(Watchdog *wdg, uint32_t check_interval_ms)
{
    memset(wdg, 0, sizeof(Watchdog));
    wdg->check_interval_ms  = check_interval_ms > 0 ? check_interval_ms : 1000;
    wdg->system_reset_armed = false;
    wdg->initialized        = true;
}

/* ── 注册监控 ──────────────────────────────────────────── */
int WDG_Add(Watchdog *wdg, const char *name, uint32_t timeout_ms,
            uint32_t max_fails, WdgResetPolicy policy,
            WdgResetCallback reset_cb, void *reset_arg)
{
    if (!wdg->initialized) return -1;
    if (wdg->monitor_count >= WDG_MAX_MONITORS) return -1;

    WdgMonitor *m = &wdg->monitors[wdg->monitor_count];
    m->name            = name;
    m->fed             = false;
    m->timeout_ms      = timeout_ms > 0 ? timeout_ms : WDG_DEFAULT_TIMEOUT_MS;
    m->time_since_feed = 0;
    m->fail_count      = 0;
    m->max_fails       = max_fails > 0 ? max_fails : WDG_DEFAULT_MAX_FAILS;
    m->total_fails     = 0;
    m->state           = WDG_STATE_HEALTHY;
    m->reset_policy    = policy;
    m->reset_callback  = reset_cb;
    m->reset_arg       = reset_arg;
    m->active          = true;

    return (int)(wdg->monitor_count++);
}

/* ── 喂狗 ──────────────────────────────────────────────── */
void WDG_Feed(Watchdog *wdg, uint8_t index)
{
    if (index >= WDG_MAX_MONITORS) return;
    WdgMonitor *m = &wdg->monitors[index];
    if (!m->active) return;

    m->fed = true;
}

/* ── 定时检查 ──────────────────────────────────────────── */
void WDG_Update(Watchdog *wdg)
{
    if (!wdg->initialized) return;

    /*
     * 注意: elapsed_ms 由外部每ms递增
     * 这里每次调用视为经过了1ms (由Sched_TickISR间接驱动)
     */
    wdg->elapsed_ms++;
    if (wdg->elapsed_ms < wdg->check_interval_ms) return;
    wdg->elapsed_ms = 0;

    /* 检查所有监控项 */
    for (uint8_t i = 0; i < wdg->monitor_count; i++) {
        WdgMonitor *m = &wdg->monitors[i];
        if (!m->active) continue;

        if (m->fed) {
            /* 喂了狗, 重置计数器 */
            m->fed             = false;
            m->time_since_feed = 0;
            m->fail_count      = 0;
            m->state           = WDG_STATE_HEALTHY;
        } else {
            /* 没喂狗, 累加时间 */
            m->time_since_feed += wdg->check_interval_ms;

            if (m->time_since_feed >= m->timeout_ms) {
                /* 超时! */
                m->fail_count++;
                m->total_fails++;
                m->time_since_feed = 0;

                if (m->fail_count >= m->max_fails) {
                    m->state = WDG_STATE_RESET_PENDING;

                    /* 执行复位策略 */
                    switch (m->reset_policy) {
                    case WDG_RESET_NONE:
                        m->state = WDG_STATE_CRITICAL;
                        break;

                    case WDG_RESET_TASK_REINIT:
                        if (m->reset_callback) {
                            m->reset_callback(m->reset_arg);
                        }
                        m->fail_count = 0;
                        m->state = WDG_STATE_WARNING;
                        break;

                    case WDG_RESET_SYSTEM:
                        printf("[WDG] CRITICAL: %s — SYSTEM RESET!\r\n",
                               m->name ? m->name : "?");
                        /* 短暂延迟让UART输出完 */
                        for (volatile uint32_t d = 0; d < 100000; d++) {}
                        if (wdg->system_reset_armed) {
                            /* 触发系统复位 */
                            system_reset();
                        }
                        break;
                    }
                } else {
                    m->state = WDG_STATE_WARNING;
                }
            }
        }
    }
}

/* ── 查询接口 ──────────────────────────────────────────── */
WdgState WDG_GetState(const Watchdog *wdg, uint8_t index)
{
    if (index >= WDG_MAX_MONITORS) return WDG_STATE_CRITICAL;
    return wdg->monitors[index].state;
}

/* ── 系统复位使能 ──────────────────────────────────────── */
void WDG_ArmSystemReset(Watchdog *wdg, bool arm)
{
    wdg->system_reset_armed = arm;
}

/* ── 调试输出 ──────────────────────────────────────────── */
void WDG_PrintStatus(const Watchdog *wdg)
{
    printf("[WDG] monitors=%u interval=%lu ms\r\n",
           wdg->monitor_count,
           (unsigned long)wdg->check_interval_ms);

    for (uint8_t i = 0; i < wdg->monitor_count; i++) {
        const WdgMonitor *m = &wdg->monitors[i];
        if (!m->active) continue;

        const char *state_str = "?";
        switch (m->state) {
            case WDG_STATE_HEALTHY:        state_str = "OK";       break;
            case WDG_STATE_WARNING:        state_str = "WARN";     break;
            case WDG_STATE_CRITICAL:       state_str = "CRIT";     break;
            case WDG_STATE_RESET_PENDING:  state_str = "RST";      break;
        }

        printf("  [%u] %-12s %s timeout=%lu fails=%lu/%lu total=%lu\r\n",
               i,
               m->name ? m->name : "?",
               state_str,
               (unsigned long)m->timeout_ms,
               (unsigned long)m->fail_count,
               (unsigned long)m->max_fails,
               (unsigned long)m->total_fails);
    }
}
