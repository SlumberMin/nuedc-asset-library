/**
 * @file robust_pid.h
 * @brief 鲁棒PID控制器（H∞优化的PID参数）
 *
 * 【算法原理】
 * 基于H∞鲁棒控制理论，通过频域分析优化PID参数，使控制器对模型
 * 不确定性（参数摄动、未建模动态、外部扰动）具有鲁棒性。
 *
 * 核心思想：在保证标称性能的同时，最大化系统对不确定性的容忍度。
 * 使用加权函数对灵敏度函数S(s)和补灵敏度函数T(s)进行约束：
 *   ||W1(s)*S(s)||∞ < 1  （抑制扰动）
 *   ||W2(s)*T(s)||∞ < 1  （抑制噪声）
 *
 * 【适用场景】
 * - 被控对象模型存在较大不确定性（参数变化±30%以上）
 * - 存在未建模高频动态（如机械谐振、柔性结构）
 * - 需要在大范围工况下保持稳定性的场合
 * - 电力电子变换器（参数随负载变化大）
 * - 电机控制（惯量、电阻随温度变化）
 *
 * 【参数整定指南】
 * 1. 确定被控对象标称模型G0(s) = K*exp(-θs)/(Ts+1)
 *    - K: 静态增益
 *    - T: 时间常数
 *    - θ: 纯滞后时间
 * 2. 设定不确定性边界：参数变化范围ΔK, ΔT
 * 3. 设定性能权函数W1(s)的带宽ωc（截止频率）
 *    - ωc ≈ 0.1~0.5 × 采样频率
 *    - 响应速度要求越高，ωc越大
 * 4. 调用 RobustPID_Init() 进行自动整定
 * 5. 可微调 gamma 参数平衡性能与鲁棒性
 *    - gamma < 1: 偏重鲁棒性（更保守但更安全）
 *    - gamma = 1: 平衡（推荐起点）
 *    - gamma > 1: 偏重性能（响应更快但鲁棒性降低）
 */

#ifndef ROBUST_PID_H
#define ROBUST_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 鲁棒PID控制器结构体
 */
typedef struct {
    /* PID增益（由H∞优化自动生成） */
    float Kp;           /**< 比例增益 */
    float Ki;           /**< 积分增益 */
    float Kd;           /**< 微分增益 */

    /* 内部状态 */
    float integral;     /**< 积分累加器 */
    float prev_error;   /**< 上一次误差 */
    float prev_output;  /**< 上一次输出（用于微分滤波） */
    float dt;           /**< 采样周期(s) */
    float filtered_D;   /**< 微分项低通滤波状态（避免static局部变量） */

    /* 鲁棒性参数 */
    float alpha;        /**< 微分低通滤波系数 (0~1) */
    float beta;         /**< 积分分离阈值 */
    float gamma;        /**< 性能/鲁棒性平衡因子 (0.5~2.0) */

    /* 模型参数 */
    float model_K;      /**< 标称模型增益 */
    float model_T;      /**< 标称模型时间常数(s) */
    float model_theta;  /**< 标称模型纯滞后(s) */
    float delta_K;      /**< 增益不确定性范围(0~1, 如0.3表示±30%) */
    float delta_T;      /**< 时间常数不确定性范围(0~1) */

    /* 输出限幅 */
    float output_min;   /**< 输出下限 */
    float output_max;   /**< 输出上限 */
} RobustPID_t;

/**
 * @brief 初始化鲁棒PID控制器
 *
 * @param pid           控制器结构体指针
 * @param dt            采样周期(s)
 * @param model_K       标称模型增益
 * @param model_T       标称模型时间常数(s)
 * @param model_theta   标称模型纯滞后(s)
 * @param delta_K       增益不确定性范围 (如0.3表示±30%)
 * @param delta_T       时间常数不确定性范围
 * @param desired_bw    期望闭环带宽(rad/s), 0则自动选择
 */
void RobustPID_Init(RobustPID_t *pid, float dt,
                    float model_K, float model_T, float model_theta,
                    float delta_K, float delta_T, float desired_bw);

/**
 * @brief 计算鲁棒PID输出
 * @param pid    控制器结构体指针
 * @param setpoint  目标值
 * @param feedback  当前反馈值
 * @return 控制输出
 */
float RobustPID_Compute(RobustPID_t *pid, float setpoint, float feedback);

/**
 * @brief 重置控制器状态
 */
void RobustPID_Reset(RobustPID_t *pid);

/**
 * @brief 设置输出限幅
 */
void RobustPID_SetOutputLimits(RobustPID_t *pid, float min, float max);

/**
 * @brief 手动设置PID参数（覆盖H∞优化结果）
 */
void RobustPID_SetGains(RobustPID_t *pid, float Kp, float Ki, float Kd);

/**
 * @brief 设置性能/鲁棒性平衡因子
 * @param gamma: <1偏鲁棒, =1平衡, >1偏性能
 */
void RobustPID_SetGamma(RobustPID_t *pid, float gamma);

#ifdef __cplusplus
}
#endif

#endif /* ROBUST_PID_H */
