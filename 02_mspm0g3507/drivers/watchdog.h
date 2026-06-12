/**
 * @file    watchdog.h
 * @brief   软件看门狗 — 任务健康监控与自动恢复
 *
 * 参考国奖作品可靠性设计:
 *   - 国奖作品共同特征: "全程无死机、无失控"
 *   - 异常检测 + 自动恢复机制
 *
 * 设计思路:
 *   - 每个被监控的任务需要定期"喂狗"
 *   - 若任务在超时时间内未喂狗, 视为异常
 *   - 异常处理流程: 警告事件 → 多次失败后自动复位
 *   - 复位策略: 软复位(重初始化任务) 或 硬复位(系统重启)
 *
 * 依赖: task_scheduler.h, event_system.h (可选)
 */

#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 配置参数 ─────────────────────────────────────────── */
#ifndef WDG_MAX_MONITORS
#define WDG_MAX_MONITORS        8       /* 最大监控任务数 */
#endif

#ifndef WDG_DEFAULT_TIMEOUT_MS
#define WDG_DEFAULT_TIMEOUT_MS  5000    /* 默认超时 5秒 */
#endif

#ifndef WDG_DEFAULT_MAX_FAILS
#define WDG_DEFAULT_MAX_FAILS   3       /* 默认最大连续失败次数 */
#endif

/* ── 监控状态 ──────────────────────────────────────────── */
typedef enum {
    WDG_STATE_HEALTHY,                  /* 健康 */
    WDG_STATE_WARNING,                  /* 警告 (喂狗超时一次) */
    WDG_STATE_CRITICAL,                 /* 严重 (连续多次超时) */
    WDG_STATE_RESET_PENDING             /* 即将复位 */
} WdgState;

/* ── 复位策略 ──────────────────────────────────────────── */
typedef enum {
    WDG_RESET_NONE,                     /* 不自动复位, 仅报告 */
    WDG_RESET_TASK_REINIT,              /* 调用任务重初始化回调 */
    WDG_RESET_SYSTEM                    /* 系统硬复位 (NVIC_SystemReset) */
} WdgResetPolicy;

/* ── 复位回调类型 ──────────────────────────────────────── */
typedef void (*WdgResetCallback)(void *arg);

/* ── 单个监控项 ────────────────────────────────────────── */
typedef struct {
    const char     *name;               /* 任务名称 (调试用) */
    volatile bool   fed;                /* 喂狗标志 */
    uint32_t        timeout_ms;         /* 超时时间 */
    uint32_t        time_since_feed;    /* 距上次喂狗的时间 */
    uint32_t        fail_count;         /* 连续超时计数 */
    uint32_t        max_fails;          /* 触发复位的连续失败次数 */
    uint32_t        total_fails;        /* 累计超时次数 */
    WdgState        state;              /* 当前状态 */
    WdgResetPolicy  reset_policy;       /* 复位策略 */
    WdgResetCallback reset_callback;    /* 复位回调 */
    void           *reset_arg;          /* 复位回调参数 */
    bool            active;             /* 是否激活 */
} WdgMonitor;

/* ── 看门狗实例 ────────────────────────────────────────── */
typedef struct {
    WdgMonitor      monitors[WDG_MAX_MONITORS];
    uint8_t         monitor_count;
    uint32_t        check_interval_ms;  /* 检查间隔 */
    uint32_t        elapsed_ms;         /* 距上次检查的累积时间 */
    bool            initialized;
    bool            system_reset_armed; /* 系统复位已使能 */
} Watchdog;

/* ── 公共API ───────────────────────────────────────────── */

/**
 * @brief 初始化看门狗
 * @param wdg              看门狗实例
 * @param check_interval_ms 检查间隔(ms)
 */
void WDG_Init(Watchdog *wdg, uint32_t check_interval_ms);

/**
 * @brief 注册监控任务
 * @param wdg           看门狗实例
 * @param name          任务名称
 * @param timeout_ms    超时时间(ms)
 * @param max_fails     触发复位的连续失败次数
 * @param policy        复位策略
 * @param reset_cb      复位回调 (当policy=WDG_RESET_TASK_REINIT时)
 * @param reset_arg     回调参数
 * @return 监控索引 (>=0成功, <0失败)
 */
int WDG_Add(Watchdog *wdg, const char *name, uint32_t timeout_ms,
            uint32_t max_fails, WdgResetPolicy policy,
            WdgResetCallback reset_cb, void *reset_arg);

/**
 * @brief 喂狗 — 被监控任务在其周期中调用
 * @param wdg   看门狗实例
 * @param index 监控索引
 */
void WDG_Feed(Watchdog *wdg, uint8_t index);

/**
 * @brief 看门狗定时检查 — 在调度器tick中调用
 * @param wdg 看门狗实例
 * @details 每check_interval_ms检查一次所有监控项
 */
void WDG_Update(Watchdog *wdg);

/**
 * @brief 获取监控项状态
 */
WdgState WDG_GetState(const Watchdog *wdg, uint8_t index);

/**
 * @brief 打印看门狗状态（调试用）
 */
void WDG_PrintStatus(const Watchdog *wdg);

/**
 * @brief 使能系统硬件复位 (需额外配置MSPM0看门狗外设)
 */
void WDG_ArmSystemReset(Watchdog *wdg, bool arm);

#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H */
