/**
 * @file    state_machine.h
 * @brief   通用事件驱动状态机框架
 *
 * 参考: GitHub高星项目 Quantum Platform (QP) / state_machine_c 优秀实现
 *
 * 特点:
 *   - 事件驱动，支持进入/退出回调
 *   - 支持层次状态机（子状态继承父状态转换）
 *   - 纯C实现，零动态分配
 *   - 编译时确定最大状态/事件数
 *   - 支持状态转换表（查表法，高效）
 *
 * 典型应用:
 *   - 任务调度（送药小车、巡线机器人等）
 *   - 通信协议解析（帧同步、ACK等待等）
 *   - 设备控制（电机启停、传感器采样序列等）
 */
#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

/* ── 最大状态/事件数限制 ──────────────────────────────────── */
#define SM_MAX_STATES    32   /**< 最多支持32个状态 */
#define SM_MAX_EVENTS    32   /**< 最多支持32种事件 */
#define SM_NO_PARENT     0xFF /**< 无父状态标志 */

/* ── 状态/事件类型 ────────────────────────────────────────── */
typedef uint8_t  StateId_t;   /**< 状态ID类型 */
typedef uint8_t  EventId_t;   /**< 事件ID类型 */

/* ── 事件结构体 ───────────────────────────────────────────── */
typedef struct {
    EventId_t id;             /**< 事件ID */
    uint8_t   reserved;       /**< 保留对齐 */
    uint16_t  param;          /**< 事件参数（通用） */
    uint32_t  timestamp;      /**< 事件时间戳（可选） */
} SM_Event_t;

/* ── 状态机前向声明 ───────────────────────────────────────── */
typedef struct SM_Machine SM_Machine;

/* ── 状态处理函数类型 ─────────────────────────────────────── */
/**
 * 状态处理函数
 * @param  sm    状态机指针
 * @param  event 当前事件
 * @return true=事件已处理, false=事件未处理（传递给父状态）
 */
typedef bool (*SM_StateFunc_t)(SM_Machine *sm, const SM_Event_t *event);

/* ── 状态描述表项 ─────────────────────────────────────────── */
typedef struct {
    StateId_t       parent;       /**< 父状态ID（SM_NO_PARENT表示无父状态） */
    StateId_t       reserved;     /**< 保留对齐 */
    SM_StateFunc_t  on_enter;     /**< 进入状态回调（NULL表示无操作） */
    SM_StateFunc_t  on_exit;      /**< 退出状态回调（NULL表示无操作） */
    SM_StateFunc_t  on_event;     /**< 事件处理函数（NULL表示仅继承父状态） */
} SM_StateDesc_t;

/* ── 状态机控制块 ─────────────────────────────────────────── */
struct SM_Machine {
    const SM_StateDesc_t *state_table;  /**< 状态描述表 */
    StateId_t             current;      /**< 当前状态ID */
    StateId_t             previous;     /**< 上一个状态ID（用于返回） */
    uint8_t               state_count;  /**< 状态总数 */
    bool                  is_running;   /**< 状态机运行标志 */
    uint32_t              state_ticks;  /**< 当前状态持续时间（tick） */
    void                 *user_data;    /**< 用户自定义数据指针 */
};

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化状态机
 * @param sm          状态机控制块
 * @param table       状态描述表（数组，索引即状态ID）
 * @param count       状态总数
 * @param init_state  初始状态ID
 * @param user_data   用户自定义数据指针
 */
void SM_Init(SM_Machine *sm, const SM_StateDesc_t *table,
             uint8_t count, StateId_t init_state, void *user_data);

/**
 * @brief 启动状态机（触发初始状态的on_enter回调）
 */
void SM_Start(SM_Machine *sm);

/**
 * @brief 分发事件到状态机
 * @param sm    状态机控制块
 * @param event 要分发的事件
 * @return true=事件已被处理
 */
bool SM_Dispatch(SM_Machine *sm, const SM_Event_t *event);

/**
 * @brief 执行状态转换
 * @param sm       状态机控制块
 * @param new_state 目标状态ID
 *
 * 转换流程: 当前状态on_exit → 新状态on_enter → 更新current
 * 如果当前状态和目标状态相同，不做任何操作
 */
void SM_Transition(SM_Machine *sm, StateId_t new_state);

/**
 * @brief 返回上一个状态
 */
static inline void SM_Return(SM_Machine *sm)
{
    SM_Transition(sm, sm->previous);
}

/**
 * @brief 获取当前状态ID
 */
static inline StateId_t SM_GetState(const SM_Machine *sm)
{
    return sm->current;
}

/**
 * @brief 获取上一个状态ID
 */
static inline StateId_t SM_GetPreviousState(const SM_Machine *sm)
{
    return sm->previous;
}

/**
 * @brief 获取当前状态持续时间（tick）
 */
static inline uint32_t SM_GetStateTicks(const SM_Machine *sm)
{
    return sm->state_ticks;
}

/**
 * @brief 状态持续时间自增（在定时中断中调用）
 */
static inline void SM_Tick(SM_Machine *sm)
{
    if (sm->state_ticks < 0xFFFFFFFFu) {
        sm->state_ticks++;
    }
}

/**
 * @brief 状态机是否正在运行
 */
static inline bool SM_IsRunning(const SM_Machine *sm)
{
    return sm->is_running;
}

/**
 * @brief 停止状态机
 */
static inline void SM_Stop(SM_Machine *sm)
{
    sm->is_running = false;
}

#endif /* __STATE_MACHINE_H */
