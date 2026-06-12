/**
 * @file    state_machine.c
 * @brief   通用事件驱动状态机框架实现
 *
 * 事件分发流程:
 *   1. 将事件传递给当前状态的 on_event 处理函数
 *   2. 如果 on_event 返回 false（未处理）且有父状态，
 *      则沿父状态链向上查找，直到事件被处理或到达根状态
 *   3. 状态转换时，先调用当前状态的 on_exit，
 *      再调用新状态的 on_enter
 *
 * 使用示例:
 *   SM_Init(&sm, state_table, STATE_COUNT, STATE_INIT, &context);
 *   SM_Start(&sm);
 *
 *   while (SM_IsRunning(&sm)) {
 *       SM_Event_t evt = poll_event();
 *       SM_Dispatch(&sm, &evt);
 *       SM_Tick(&sm);
 *   }
 */
#include "drivers/state_machine.h"

/* ================================================================
 * 初始化
 * ================================================================ */
void SM_Init(SM_Machine *sm, const SM_StateDesc_t *table,
             uint8_t count, StateId_t init_state, void *user_data)
{
    sm->state_table = table;
    sm->state_count = count;
    sm->current     = init_state;
    sm->previous    = init_state;
    sm->is_running  = false;
    sm->state_ticks = 0;
    sm->user_data   = user_data;
}

/* ================================================================
 * 启动状态机
 * ================================================================ */
void SM_Start(SM_Machine *sm)
{
    sm->is_running  = true;
    sm->state_ticks = 0;

    /* 调用初始状态的 on_enter 回调 */
    if (sm->current < sm->state_count) {
        const SM_StateDesc_t *desc = &sm->state_table[sm->current];
        if (desc->on_enter) {
            SM_Event_t enter_evt = {0, 0, 0, 0};
            desc->on_enter(sm, &enter_evt);
        }
    }
}

/* ================================================================
 * 事件分发（核心逻辑）
 *
 * 事件传递链:
 *   current_state -> parent_state -> grandparent_state -> ...
 *   直到某个状态的 on_event 返回 true（已处理）
 * ================================================================ */
bool SM_Dispatch(SM_Machine *sm, const SM_Event_t *event)
{
    if (!sm->is_running || event == (void *)0) {
        return false;
    }

    StateId_t state = sm->current;

    /* 沿状态层次链向上查找处理器 */
    while (state < sm->state_count) {
        const SM_StateDesc_t *desc = &sm->state_table[state];

        if (desc->on_event) {
            bool handled = desc->on_event(sm, event);
            if (handled) {
                return true;  /* 事件已处理 */
            }
        }

        /* 事件未处理，检查父状态 */
        if (desc->parent == SM_NO_PARENT) {
            break;  /* 已到达根状态，事件丢弃 */
        }

        state = desc->parent;
    }

    return false;  /* 事件未被任何状态处理 */
}

/* ================================================================
 * 状态转换
 *
 * 转换流程:
 *   1. 校验目标状态有效性
 *   2. 调用当前状态的 on_exit
 *   3. 保存 previous，更新 current
 *   4. 重置状态计时器
 *   5. 调用新状态的 on_enter
 * ================================================================ */
void SM_Transition(SM_Machine *sm, StateId_t new_state)
{
    /* 参数校验 */
    if (new_state >= sm->state_count) {
        return;  /* 无效状态ID */
    }

    if (new_state == sm->current) {
        return;  /* 相同状态，无需转换 */
    }

    /* 1. 调用当前状态的 on_exit */
    if (sm->current < sm->state_count) {
        const SM_StateDesc_t *desc = &sm->state_table[sm->current];
        if (desc->on_exit) {
            SM_Event_t exit_evt = {0, 0, 0, 0};
            desc->on_exit(sm, &exit_evt);
        }
    }

    /* 2. 更新状态 */
    sm->previous    = sm->current;
    sm->current     = new_state;
    sm->state_ticks = 0;

    /* 3. 调用新状态的 on_enter */
    {
        const SM_StateDesc_t *desc = &sm->state_table[new_state];
        if (desc->on_enter) {
            SM_Event_t enter_evt = {0, 0, 0, 0};
            desc->on_enter(sm, &enter_evt);
        }
    }
}
