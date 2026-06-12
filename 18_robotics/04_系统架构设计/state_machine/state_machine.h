/*
 * 有限状态机（FSM）实现
 * 来源：RoboMaster 比赛策略
 * 适配平台：MSPM0G3507
 * 
 * 设计思路：
 * 1. 定义状态枚举
 * 2. 定义事件枚举
 * 3. 状态转换表
 * 4. 事件处理函数
 * 5. 状态机运行函数
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

// 状态枚举（根据实际需求修改）
typedef enum {
    STATE_IDLE = 0,         // 空闲状态
    STATE_INIT,             // 初始化状态
    STATE_RUNNING,          // 运行状态
    STATE_STOPPED,          // 停止状态
    STATE_ERROR,            // 错误状态
    STATE_COUNT             // 状态总数
} SystemState;

// 事件枚举（根据实际需求修改）
typedef enum {
    EVENT_NONE = 0,         // 无事件
    EVENT_START,            // 启动事件
    EVENT_STOP,             // 停止事件
    EVENT_ERROR,            // 错误事件
    EVENT_RESET,            // 复位事件
    EVENT_TIMEOUT,          // 超时事件
    EVENT_COUNT             // 事件总数
} SystemEvent;

// 状态处理函数类型
typedef void (*StateHandler)(void);

// 事件处理函数类型
typedef void (*EventHandler)(void);

// 状态转换结构体
typedef struct {
    SystemState next_state;     // 下一个状态
    EventHandler handler;       // 事件处理函数
} StateTransition;

// 状态机结构体
typedef struct {
    SystemState current_state;  // 当前状态
    SystemState previous_state; // 上一个状态
    SystemEvent current_event;  // 当前事件
    
    StateHandler state_handlers[STATE_COUNT];           // 状态处理函数
    StateTransition transition_table[STATE_COUNT][EVENT_COUNT];  // 状态转换表
    
    uint32_t state_enter_time;  // 状态进入时间
    uint32_t state_duration;    // 状态持续时间
    
    bool initialized;           // 是否初始化
} StateMachine_HandleTypeDef;

// 函数声明
void StateMachine_Init(StateMachine_HandleTypeDef *hfsm);
void StateMachine_RegisterState(StateMachine_HandleTypeDef *hfsm, SystemState state, StateHandler handler);
void StateMachine_RegisterTransition(StateMachine_HandleTypeDef *hfsm, SystemState from, SystemEvent event, SystemState to, EventHandler handler);
void StateMachine_SetEvent(StateMachine_HandleTypeDef *hfsm, SystemEvent event);
void StateMachine_Run(StateMachine_HandleTypeDef *hfsm);
SystemState StateMachine_GetState(StateMachine_HandleTypeDef *hfsm);
bool StateMachine_IsInState(StateMachine_HandleTypeDef *hfsm, SystemState state);
uint32_t StateMachine_GetStateDuration(StateMachine_HandleTypeDef *hfsm);

#endif // STATE_MACHINE_H
