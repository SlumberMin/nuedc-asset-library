/**
 * @file    task_scheduler.c
 * @brief   非抢占式任务调度器实现
 */

#include "task_scheduler.h"
#include <stdio.h>
#include <string.h>

/* ── 初始化 ────────────────────────────────────────────── */
void Sched_Init(Scheduler *sched)
{
    memset(sched, 0, sizeof(Scheduler));
    sched->initialized = true;
    sched->running     = false;
}

/* ── 定时器中断回调 ────────────────────────────────────── */
void Sched_TickISR(Scheduler *sched)
{
    sched->tick_count++;

    /* 更新所有就绪/运行任务的延时计数器 */
    for (volatile uint8_t i = 0; i < SCHED_MAX_TASKS; i++) {
        TaskHandle *t = &sched->tasks[i];
        /* BugFix: DISABLED状态的任务不应递减delay_ms，
         * 否则重新启用时可能立即触发 */
        if (t->state == TASK_STATE_READY) {
            if (t->delay_ms > 0) {
                t->delay_ms--;
            }
        }
    }
}

/* ── 内部: 查找空闲槽位 ────────────────────────────────── */
static int find_free_slot(Scheduler *sched)
{
    for (uint8_t i = 0; i < SCHED_MAX_TASKS; i++) {
        if (sched->tasks[i].state == TASK_STATE_IDLE) {
            return (int)i;
        }
    }
    return -1;
}

/* ── 注册周期任务 ──────────────────────────────────────── */
int Sched_AddPeriodic(Scheduler *sched, TaskCallback callback, void *arg,
                      uint32_t period_ms, SchedPriority priority)
{
    if (!sched->initialized || callback == NULL || period_ms == 0) return -1;

    int idx = find_free_slot(sched);
    if (idx < 0) return -1;

    TaskHandle *t = &sched->tasks[idx];
    t->callback    = callback;
    t->arg         = arg;
    t->period_ms   = period_ms;
    t->delay_ms    = period_ms;        /* 首次延迟一个周期 */
    t->priority    = priority;
    t->state       = TASK_STATE_READY;
    t->auto_reload = true;
    t->run_count   = 0;
    t->last_run_tick = 0;
    t->max_exec_us = 0;

    sched->task_count++;
    return idx;
}

/* ── 注册延时单次任务 ──────────────────────────────────── */
int Sched_AddDelayed(Scheduler *sched, TaskCallback callback, void *arg,
                     volatile uint32_t delay_ms, SchedPriority priority)
{
    if (!sched->initialized || callback == NULL) return -1;

    int idx = find_free_slot(sched);
    if (idx < 0) return -1;

    TaskHandle *t = &sched->tasks[idx];
    t->callback    = callback;
    t->arg         = arg;
    t->period_ms   = 0;
    t->delay_ms    = delay_ms;
    t->priority    = priority;
    t->state       = TASK_STATE_READY;
    t->auto_reload = false;
    t->run_count   = 0;
    t->last_run_tick = 0;
    t->max_exec_us = 0;

    sched->task_count++;
    return idx;
}

/* ── 使能/禁用任务 ─────────────────────────────────────── */
void Sched_EnableTask(Scheduler *sched, uint8_t index, bool enable)
{
    if (index >= SCHED_MAX_TASKS) return;
    TaskHandle *t = &sched->tasks[index];
    if (t->state == TASK_STATE_IDLE) return;

    t->state = enable ? TASK_STATE_READY : TASK_STATE_DISABLED;
}

/* ── 删除任务 ──────────────────────────────────────────── */
void Sched_RemoveTask(Scheduler *sched, uint8_t index)
{
    if (index >= SCHED_MAX_TASKS) return;
    if (sched->tasks[index].state != TASK_STATE_IDLE) {
        sched->task_count--;
    }
    memset(&sched->tasks[index], 0, sizeof(TaskHandle));
}

/* ── 重置任务延时 ──────────────────────────────────────── */
void Sched_ResetTask(Scheduler *sched, uint8_t index)
{
    if (index >= SCHED_MAX_TASKS) return;
    TaskHandle *t = &sched->tasks[index];
    if (t->state == TASK_STATE_IDLE) return;

    t->delay_ms = t->period_ms > 0 ? t->period_ms : 1;
    t->state    = TASK_STATE_READY;
}

/* ── 调度器主循环 ──────────────────────────────────────── */
void Sched_Run(Scheduler *sched)
{
    if (!sched->initialized) return;

    sched->running   = true;
    sched->loop_count++;

    /*
     * 按优先级从高到低遍历
     * 对每个优先级, 扫描所有就绪且到期的任务并执行
     * 非抢占: 任务执行完毕后才检查下一个
     */
    for (uint8_t prio = 0; prio < SCHED_PRIORITY_COUNT; prio++) {
        for (uint8_t i = 0; i < SCHED_MAX_TASKS; i++) {
            TaskHandle *t = &sched->tasks[i];

            if (t->state != TASK_STATE_READY) continue;
            if ((uint8_t)t->priority != prio)  continue;
            if (t->delay_ms > 0)               continue; /* 未到期 */

            /* 执行任务 */
            t->state = TASK_STATE_RUNNING;
            uint32_t tick_before = sched->tick_count;

            t->callback(t->arg);

            uint32_t elapsed = sched->tick_count - tick_before;
            if (elapsed > t->max_exec_us) {
                t->max_exec_us = elapsed;
            }

            t->run_count++;
            t->last_run_tick = sched->tick_count;

            /* 周期任务重装延时, 单次任务标记空闲 */
            if (t->auto_reload) {
                t->delay_ms = t->period_ms;
                t->state    = TASK_STATE_READY;
            } else {
                t->state = TASK_STATE_IDLE;
                sched->task_count--;
            }
        }
    }
}

/* ── 查询接口 ──────────────────────────────────────────── */
uint32_t Sched_GetTick(const Scheduler *sched)
{
    return sched->tick_count;
}

uint8_t Sched_GetTaskCount(const Scheduler *sched)
{
    return sched->task_count;
}

/* ── 调试输出 ──────────────────────────────────────────── */
void Sched_PrintStatus(const Scheduler *sched)
{
    printf("[Sched] tick=%lu loops=%lu tasks=%u\r\n",
           (unsigned long)sched->tick_count,
           (unsigned long)sched->loop_count,
           sched->task_count);

    for (uint8_t i = 0; i < SCHED_MAX_TASKS; i++) {
        const TaskHandle *t = &sched->tasks[i];
        if (t->state == TASK_STATE_IDLE) continue;

        const char *state_str = "?";
        switch (t->state) {
            case TASK_STATE_READY:    state_str = "READY";    break;
            case TASK_STATE_RUNNING:  state_str = "RUNNING";  break;
            case TASK_STATE_DISABLED: state_str = "DISABLED"; break;
            default: break;
        }

        printf("  [%u] %s prio=%u period=%lu runs=%lu max=%lu\r\n",
               i, state_str, (unsigned)t->priority,
               (unsigned long)t->period_ms,
               (unsigned long)t->run_count,
               (unsigned long)t->max_exec_us);
    }
}
