/**
 * @file    pid.h
 * @brief   PID控制器模块 — STM32电赛通用代码库
 * @details 支持位置式和增量式PID，微分滤波，条件积分抗饱和，
 *          死区处理，前馈控制。
 * @author  电赛通用代码库
 * @version 1.0
 * @date    2026-06
 *
 * 使用流程:
 *   1. PID_Init() 初始化
 *   2. PID_SetTarget() 设置目标值
 *   3. 周期性调用 PID_Calculate() 计算输出
 *   4. 将输出作用到执行器
 */

#ifndef __PID_H
#define __PID_H

#include "platform/hal_stm32.h"

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/**
 * @brief PID工作模式
 */
typedef enum {
    PID_MODE_POSITION = 0,  /**< 位置式PID: output = Kp*e + Ki*∫e + Kd*de */
    PID_MODE_INCREMENTAL,   /**< 增量式PID: Δu = Kp*Δe + Ki*e + Kd*Δ²e */
} PID_Mode_t;

/**
 * @brief PID配置与状态结构体
 */
typedef struct {
    /* 模式 */
    PID_Mode_t mode;            /**< PID模式 */

    /* PID参数 */
    float kp;                   /**< 比例系数 */
    float ki;                   /**< 积分系数 */
    float kd;                   /**< 微分系数 */

    /* 目标与输入输出 */
    float target;               /**< 目标值 */
    float output;               /**< 输出值 */
    float output_min;           /**< 输出下限 */
    float output_max;           /**< 输出上限 */

    /* 内部状态（位置式） */
    float error;                /**< 当前误差 e(k) */
    float error_prev;           /**< 上次误差 e(k-1) */
    float error_prev2;          /**< 上上次误差 e(k-2) */
    float integral;             /**< 积分累积 */
    float derivative;           /**< 微分项 */
    float integral_max;         /**< 积分限幅(抗饱和) */

    /* 微分滤波 */
    float derivative_prev;      /**< 上次微分值（一阶低通滤波） */
    float derivative_filter_alpha; /**< 微分滤波系数 0~1, 越小滤波越强, 默认0.1 */

    /* 条件积分抗饱和 */
    bool  conditional_integral; /**< 是否启用条件积分 */
    float output_saturation_threshold; /**< 输出饱和阈值(接近限幅时不积分) */

    /* 死区 */
    float dead_zone;            /**< 死区大小，误差绝对值<dead_zone时不输出 */

    /* 前馈 */
    float feedforward;          /**< 前馈项（外部设置） */

    /* 时间 */
    float dt_s;                 /**< 控制周期(秒) */
    uint32_t last_tick;         /**< 上次计算时刻(ms) */

    bool initialized;           /**< 是否已初始化 */
} PID_t;

/* ========================================================================== */
/*                              接口函数                                       */
/* ========================================================================== */

/**
 * @brief 初始化PID控制器
 * @param pid       PID结构体指针
 * @param mode      PID模式(位置式/增量式)
 * @param kp        比例系数
 * @param ki        积分系数
 * @param kd        微分系数
 * @param dt_s      控制周期(秒)，如0.01=10ms
 * @return ErrorCode_t: HAL_OK_CODE=成功
 * @note   默认输出范围 -1000 ~ +1000
 *         默认无死区、无前馈、微分滤波alpha=0.1
 */
ErrorCode_t PID_Init(PID_t *pid, PID_Mode_t mode,
                     float kp, float ki, float kd, float dt_s);

/**
 * @brief 设置PID参数（运行时调参）
 * @param pid  PID结构体指针
 * @param kp   比例系数
 * @param ki   积分系数
 * @param kd   微分系数
 * @return ErrorCode_t
 */
ErrorCode_t PID_SetParams(PID_t *pid, float kp, float ki, float kd);

/**
 * @brief 设置目标值
 * @param pid     PID结构体指针
 * @param target  目标值
 * @return ErrorCode_t
 */
ErrorCode_t PID_SetTarget(PID_t *pid, float target);

/**
 * @brief 获取目标值
 * @param pid  PID结构体指针
 * @return float: 目标值
 */
float PID_GetTarget(const PID_t *pid);

/**
 * @brief 设置输出限幅
 * @param pid  PID结构体指针
 * @param min  输出下限
 * @param max  输出上限
 * @return ErrorCode_t
 */
ErrorCode_t PID_SetOutputLimit(PID_t *pid, float min, float max);

/**
 * @brief 设置积分限幅（抗饱和）
 * @param pid       PID结构体指针
 * @param max_value 积分项最大绝对值
 * @return ErrorCode_t
 */
ErrorCode_t PID_SetIntegralLimit(PID_t *pid, float max_value);

/**
 * @brief 设置死区
 * @param pid        PID结构体指针
 * @param dead_zone  死区大小（误差绝对值小于此值时输出为0）
 * @return ErrorCode_t
 */
ErrorCode_t PID_SetDeadZone(PID_t *pid, float dead_zone);

/**
 * @brief 设置微分滤波系数
 * @param pid    PID结构体指针
 * @param alpha  滤波系数 0~1, 越小滤波越强, 0=完全滤除, 1=无滤波
 * @return ErrorCode_t
 * @note   一阶低通: filtered = alpha * new + (1-alpha) * old
 */
ErrorCode_t PID_SetDerivativeFilter(PID_t *pid, float alpha);

/**
 * @brief 启用/禁用条件积分抗饱和
 * @param pid       PID结构体指针
 * @param enable    true=启用
 * @param threshold 输出饱和阈值(接近输出限幅时不累积积分)
 * @return ErrorCode_t
 * @note   条件积分：当输出已饱和且误差方向会加剧饱和时，停止积分
 */
ErrorCode_t PID_SetConditionalIntegral(PID_t *pid, bool enable, float threshold);

/**
 * @brief 设置前馈值
 * @param pid        PID结构体指针
 * @param feedforward 前馈值
 * @return ErrorCode_t
 * @note   前馈项直接加到输出中: output = PID_out + feedforward
 *         用于已知扰动的补偿
 */
ErrorCode_t PID_SetFeedforward(PID_t *pid, float feedforward);

/**
 * @brief 计算PID输出（需周期性调用）
 * @param pid    PID结构体指针
 * @param feedback 当前反馈值（传感器读数）
 * @return float: PID输出值
 * @details 位置式: output = Kp*e + Ki*∫e*dt + Kd*de/dt + feedforward
 *          增量式: Δu = Kp*(e(k)-e(k-1)) + Ki*e(k)*dt + Kd*(e(k)-2e(k-1)+e(k-2))/dt
 *                  output += Δu
 */
float PID_Calculate(PID_t *pid, float feedback);

/**
 * @brief 重置PID状态（清除积分、历史误差等）
 * @param pid  PID结构体指针
 * @return ErrorCode_t
 * @note   不清除参数(kp/ki/kd)和目标值，仅清除内部状态
 */
ErrorCode_t PID_Reset(PID_t *pid);

/**
 * @brief 获取当前误差
 * @param pid  PID结构体指针
 * @return float: 当前误差 = target - feedback
 */
float PID_GetError(const PID_t *pid);

/**
 * @brief 获取当前输出
 * @param pid  PID结构体指针
 * @return float: PID输出值
 */
float PID_GetOutput(const PID_t *pid);

#endif /* __PID_H */
