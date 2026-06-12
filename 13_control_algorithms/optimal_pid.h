/**
 * @file optimal_pid.h
 * @brief 最优PID控制器（ITAE/ISE/IAE/ISE最优参数）
 *
 * 【算法原理】
 * 基于经典最优准则（ITAE、ISE、IAE、ISTE），通过查表法或
 * 解析公式获取最优PID参数。这些参数由大量仿真优化得到，
 * 针对典型工业过程（一阶/二阶加纯滞后）提供接近最优的响应。
 *
 * 支持的最优准则：
 * - ITAE（时间加权绝对误差积分）：超调小，工程最常用
 * - ISE（误差平方积分）：响应快但超调较大
 * - IAE（绝对误差积分）：平衡型
 * - ISTE（时间加权平方误差积分）：对快速误差更敏感
 *
 * 【适用场景】
 * - 模型已知或可辨识为一阶/二阶加纯滞后
 * - 需要定量优化某项性能指标
 * - 工业过程控制（温度、压力、流量、液位）
 * - 作为其他高级控制器的初始参数
 *
 * 【参数整定指南】
 * 1. 辨识被控对象模型参数（K, T, θ）
 * 2. 选择优化准则（推荐ITAE用于一般场合）
 * 3. 调用 OptimalPID_Init() 自动计算最优参数
 * 4. 如需微调，可调整OptimalPID_SetWeight()中的权重
 */

#ifndef OPTIMAL_PID_H
#define OPTIMAL_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 最优准则枚举 */
typedef enum {
    OPT_CRITERION_ITAE = 0,  /**< 时间加权绝对误差积分（推荐） */
    OPT_CRITERION_ISE,        /**< 误差平方积分 */
    OPT_CRITERION_IAE,        /**< 绝对误差积分 */
    OPT_CRITERION_ISTE        /**< 时间加权平方误差积分 */
} OptCriterion_t;

/**
 * @brief 最优PID控制器结构体
 */
typedef struct {
    /* PID增益 */
    float Kp;
    float Ki;
    float Kd;

    /* 内部状态 */
    float integral;
    float prev_error;
    float prev_measurement;  /**< 上一次测量值（用于微分on-measurement） */
    float dt;

    /* 模型参数 */
    float model_K;
    float model_T;
    float model_theta;
    float time_ratio;        /**< θ/T 比值（决定整定公式选择） */

    /* 优化准则 */
    OptCriterion_t criterion;

    /* 积分抗饱和 */
    float integral_max;

    /* 输出限幅 */
    float output_min;
    float output_max;

    /* 性能统计（可选） */
    float itae_sum;          /**< ITAE累计值 */
    float ise_sum;           /**< ISE累计值 */
    float sim_time;          /**< 仿真/运行时间 */
} OptimalPID_t;

/**
 * @brief 初始化最优PID控制器
 *
 * @param pid           控制器结构体
 * @param dt            采样周期(s)
 * @param model_K       模型增益
 * @param model_T       时间常数(s)
 * @param model_theta   纯滞后时间(s)
 * @param criterion     优化准则
 */
void OptimalPID_Init(OptimalPID_t *pid, float dt,
                     float model_K, float model_T, float model_theta,
                     OptCriterion_t criterion);

/**
 * @brief 计算最优PID输出
 * @param setpoint  目标值
 * @param feedback  反馈值
 * @return 控制输出
 */
float OptimalPID_Compute(OptimalPID_t *pid, float setpoint, float feedback);

/**
 * @brief 重置控制器
 */
void OptimalPID_Reset(OptimalPID_t *pid);

/**
 * @brief 切换优化准则（自动重新整定）
 */
void OptimalPID_SetCriterion(OptimalPID_t *pid, OptCriterion_t criterion);

/**
 * @brief 设置输出限幅
 */
void OptimalPID_SetOutputLimits(OptimalPID_t *pid, float min_val, float max_val);

/**
 * @brief 获取当前ITAE累计值
 */
float OptimalPID_GetITAE(OptimalPID_t *pid);

/**
 * @brief 获取当前ISE累计值
 */
float OptimalPID_GetISE(OptimalPID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* OPTIMAL_PID_H */
