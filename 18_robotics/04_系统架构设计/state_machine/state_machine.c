/*
 * 有限状态机（FSM）实现
 * 来源：RoboMaster 比赛策略
 * 适配平台：MSPM0G3507
 */

#include "state_machine.h"
#include <string.h>

// 获取当前时间（需要根据实际平台实现）
extern uint32_t HAL_GetTick(void);

/**
 * @brief 初始化状态机
 * @param hfsm 状态机句柄
 */
void StateMachine_Init(StateMachine_HandleTypeDef *hfsm)
{
    memset(hfsm, 0, sizeof(StateMachine_HandleTypeDef));
    
    // 设置初始状态
    hfsm->current_state = STATE_IDLE;
    hfsm->previous_state = STATE_IDLE;
    hfsm->current_event = EVENT_NONE;
    hfsm->state_enter_time = HAL_GetTick();
    hfsm->initialized = true;
}

/**
 * @brief 注册状态处理函数
 * @param hfsm 状态机句柄
 * @param state 状态
 * @param handler 状态处理函数
 */
void StateMachine_RegisterState(StateMachine_HandleTypeDef *hfsm, SystemState state, StateHandler handler)
{
    if (state < STATE_COUNT) {
        hfsm->state_handlers[state] = handler;
    }
}

/**
 * @brief 注册状态转换
 * @param hfsm 状态机句柄
 * @param from 源状态
 * @param event 事件
 * @param to 目标状态
 * @param handler 事件处理函数
 */
void StateMachine_RegisterTransition(StateMachine_HandleTypeDef *hfsm, 
                                     SystemState from, SystemEvent event, 
                                     SystemState to, EventHandler handler)
{
    if (from < STATE_COUNT && event < EVENT_COUNT) {
        hfsm->transition_table[from][event].next_state = to;
        hfsm->transition_table[from][event].handler = handler;
    }
}

/**
 * @brief 设置事件
 * @param hfsm 状态机句柄
 * @param event 事件
 */
void StateMachine_SetEvent(StateMachine_HandleTypeDef *hfsm, SystemEvent event)
{
    hfsm->current_event = event;
}

/**
 * @brief 运行状态机（在主循环中调用）
 * @param hfsm 状态机句柄
 */
void StateMachine_Run(StateMachine_HandleTypeDef *hfsm)
{
    if (!hfsm->initialized) {
        return;
    }
    
    // 检查是否有事件
    if (hfsm->current_event != EVENT_NONE) {
        // 获取状态转换
        StateTransition *transition = &hfsm->transition_table[hfsm->current_state][hfsm->current_event];
        
        // 执行事件处理函数
        if (transition->handler != NULL) {
            transition->handler();
        }
        
        // 状态转换
        if (transition->next_state != hfsm->current_state) {
            // 记录上一个状态
            hfsm->previous_state = hfsm->current_state;
            
            // 更新当前状态
            hfsm->current_state = transition->next_state;
            
            // 更新状态进入时间
            hfsm->state_enter_time = HAL_GetTick();
        }
        
        // 清除事件
        hfsm->current_event = EVENT_NONE;
    }
    
    // 执行当前状态的处理函数
    if (hfsm->state_handlers[hfsm->current_state] != NULL) {
        hfsm->state_handlers[hfsm->current_state]();
    }
    
    // 更新状态持续时间
    hfsm->state_duration = HAL_GetTick() - hfsm->state_enter_time;
}

/**
 * @brief 获取当前状态
 * @param hfsm 状态机句柄
 * @return 当前状态
 */
SystemState StateMachine_GetState(StateMachine_HandleTypeDef *hfsm)
{
    return hfsm->current_state;
}

/**
 * @brief 检查是否在指定状态
 * @param hfsm 状态机句柄
 * @param state 状态
 * @return 是否在指定状态
 */
bool StateMachine_IsInState(StateMachine_HandleTypeDef *hfsm, SystemState state)
{
    return (hfsm->current_state == state);
}

/**
 * @brief 获取状态持续时间
 * @param hfsm 状态机句柄
 * @return 状态持续时间（毫秒）
 */
uint32_t StateMachine_GetStateDuration(StateMachine_HandleTypeDef *hfsm)
{
    return hfsm->state_duration;
}
