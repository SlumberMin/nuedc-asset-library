/**
 * @file    task_scheduler.h
 * @brief   非抢占式任务调度器 — MSPM0G3507
 *
 * 参考国奖作品系统架构:
 *   - 2021年F题 智能送药小车: 状态机任务管理
 *   - 2019年B题 巡线机器人: 定时中断驱动调度
 *
 * 特性:
 *   - 非抢占式协作调度（主循环轮询执行）
 *   - 定时中断驱动的tick计数
 *   - 支持周期任务、延时单次任务
 *   - 优先级排序（高优先级先执行）
 *   - 任务使能/禁用、运行时统计
 *
 * 依赖: <stdint.h>, <stdbool.h>
 */

#ifndef __TASK_SCHEDULER_H
#define __TASK_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 配置参数 ─────────────────────────────────────────── */
#ifndef SCHED_MAX_TASKS
#define SCHED_MAX_TASKS         16      /* 最大任务数 */
#endif

#ifndef SCHED_TICK_MS
#define SCHED_TICK_MS           1       /* 每tick毫秒数 (由定时器中断驱动) */
#endif

/* ── 优先级定义 (数值越小优先级越高) ──────────────────── */
typedef enum {
    SCHED_PRIORITY_HIGH     = 0,        /* 高优先级: 关键控制任务 */
    SCHED_PRIORITY_NORMAL   = 1,        /* 普通优先级: 常规处理 */
    SCHED_PRIORITY_LOW      = 2,        /* 低优先级: 后台任务 */
    SCHED_PRIORITY_IDLE     = 3,        /* 空闲优先级: LED闪烁等 */
    SCHED_PRIORITY_COUNT
} SchedPriority;

/* ── 任务状态 ──────────────────────────────────────────── */
typedef enum {
    TASK_STATE_IDLE,                    /* 空闲/未注册 */
    TASK_STATE_READY,                   /* 就绪等待执行 */
    TASK_STATE_RUNNING,                 /* 正在执行 */
    TASK_STATE_DISABLED                 /* 已禁用 */
} TaskState;

/* ── 任务回调函数类型 ──────────────────────────────────── */
typedef void (*TaskCallback)(void *arg);

/* ── 任务句柄 ──────────────────────────────────────────── */
typedef struct {
    TaskCallback    callback;           /* 回调函数 */
    void           *arg;                /* 回调参数 */
    uint32_t        period_ms;          /* 执行周期(ms), 0=单次延时 */
    volatile uint32_t delay_ms;         /* 剩余延时(ms) — ISR递减, 主循环读取 */
    SchedPriority   priority;           /* 优先级 */
    TaskState       state;              /* 任务状态 */
    bool            auto_reload;        /* true=周期任务, false=单次任务 */
    /* 运行统计 */
    uint32_t        run_count;          /* 累计执行次数 */
    uint32_t        last_run_tick;      /* 最后执行时的tick */
    uint32_t        max_exec_us;        /* 最大执行耗时(粗略, tick级) */
} TaskHandle;

/* ── 调度器状态 ────────────────────────────────────────── */
typedef struct {
    TaskHandle      tasks[SCHED_MAX_TASKS];
    uint8_t         task_count;         /* 已注册任务数 */
    volatile uint32_t tick_count;       /* 全局tick计数 (定时器中断递增) */
    bool            initialized;
    bool            running;            /* 调度器运行标志 */
    uint32_t        loop_count;         /* 主循环执行次数 */
} Scheduler;

/* ── 公共API ───────────────────────────────────────────── */

/**
 * @brief 初始化调度器
 * @param sched  调度器实例指针
 */
void Sched_Init(Scheduler *sched);

/**
 * @brief 定时器中断回调 — 每SCHED_TICK_MS毫秒调用一次
 * @param sched  调度器实例指针
 */
void Sched_TickISR(Scheduler *sched);

/**
 * @brief 注册周期任务
 * @param sched     调度器实例
 * @param callback  任务回调函数
 * @param arg       回调参数
 * @param period_ms 执行周期(ms)
 * @param priority  优先级
 * @return 任务索引 (>=0成功, <0失败)
 */
int Sched_AddPeriodic(Scheduler *sched, TaskCallback callback, void *arg,
                      uint32_t period_ms, SchedPriority priority);

/**
 * @brief 注册延时单次任务
 * @param sched     调度器实例
 * @param callback  任务回调函数
 * @param arg       回调参数
 * @param delay_ms  延时执行(ms)
 * @param priority  优先级
 * @return 任务索引 (>=0成功, <0失败)
 */
int Sched_AddDelayed(Scheduler *sched, TaskCallback callback, void *arg,
                     uint32_t delay_ms, SchedPriority priority);

/**
 * @brief 使能/禁用任务
 * @param sched  调度器实例
 * @param index  任务索引
 * @param enable true=使能, false=禁用
 */
void Sched_EnableTask(Scheduler *sched, uint8_t index, bool enable);

/**
 * @brief 删除任务
 * @param sched  调度器实例
 * @param index  任务索引
 */
void Sched_RemoveTask(Scheduler *sched, uint8_t index);

/**
 * @brief 重置任务延时(从现在开始重新计时)
 * @param sched  调度器实例
 * @param index  任务索引
 */
void Sched_ResetTask(Scheduler *sched, uint8_t index);

/**
 * @brief 调度器主循环 — 在main while(1)中反复调用
 * @details 按优先级顺序轮询就绪任务并执行（非抢占式）
 * @param sched  调度器实例
 */
void Sched_Run(Scheduler *sched);

/**
 * @brief 获取全局tick计数
 */
uint32_t Sched_GetTick(const Scheduler *sched);

/**
 * @brief 获取已注册任务数量
 */
uint8_t Sched_GetTaskCount(const Scheduler *sched);

/**
 * @brief 打印调度器状态（调试用, 输出到UART）
 */
void Sched_PrintStatus(const Scheduler *sched);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_SCHEDULER_H */
