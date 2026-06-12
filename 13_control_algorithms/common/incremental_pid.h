/**
 * @file incremental_pid.h
 * @brief 增量式PID控制器（优化版）
 * @details 输出增量Δu，适合执行器为积分环节的场合(如电机、阀门)
 *          优化: 无冲击切换、微分滤波、增量限幅
 */
#ifndef __INCREMENTAL_PID_H
#define __INCREMENTAL_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float delta_max;        /* 单步增量限幅 */
    float output_min;       /* 输出总限幅 */
    float output_max;
    float deadband;
    float d_filter_alpha;   /* 微分滤波系数 */
} IncrPID_Config_t;

typedef struct {
    IncrPID_Config_t config;
    float prev_error;
    float prev_prev_error;
    float prev_d_filtered;
    float output;
    uint8_t initialized;
} IncrPID_t;

/**
 * @brief 初始化增量式PID
 */
void IncrPID_Init(IncrPID_t *pid, const IncrPID_Config_t *config);

/**
 * @brief 增量式PID计算
 * @param pid 控制器句柄
 * @param setpoint 设定值
 * @param feedback 反馈值
 * @param dt 时间步长(秒)
 * @return 控制输出
 */
float IncrPID_Calc(IncrPID_t *pid, float setpoint, float feedback, float dt);

/**
 * @brief 增量式PID计算（误差直接输入版本）
 */
float IncrPID_CalcError(IncrPID_t *pid, float error, float dt);

/**
 * @brief 更新参数
 */
void IncrPID_SetGain(IncrPID_t *pid, const IncrPID_Config_t *config);

/**
 * @brief 重置
 */
void IncrPID_Reset(IncrPID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __INCREMENTAL_PID_H */
