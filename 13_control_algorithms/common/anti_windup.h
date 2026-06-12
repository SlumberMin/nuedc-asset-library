/**
 * @file anti_windup.h
 * @brief 抗积分饱和模块 - 支持多种抗饱和策略
 *
 * 策略:
 *   1. 条件积分 (Conditional Integration)
 *   2. 反计算抗饱和 (Back-Calculation Anti-Windup)
 *   3. 积分限幅 (Integral Clamping)
 *   4. 增量式抗饱和
 */

#ifndef ANTI_WINDUP_H
#define ANTI_WINDUP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 抗饱和策略枚举 */
typedef enum {
    AW_CONDITIONAL,       /* 条件积分: 输出饱和时停止积分 */
    AW_BACK_CALCULATION,  /* 反计算: 用饱和差值反馈修正积分项 */
    AW_CLAMPING,          /* 积分限幅: 限制积分项范围 */
    AW_INCREMENTAL        /* 增量式: 只积分增量,天然抗饱和 */
} AntiWindupMethod_e;

/* 条件积分控制器 */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float out_min, out_max;
    float dt;
} PID_Conditional_t;

/* 反计算抗饱和控制器 */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float out_min, out_max;
    float kb;              /* 反计算增益,通常取 ki 的 1~10 倍 */
    float dt;
} PID_BackCalc_t;

/* 积分限幅控制器 */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float out_min, out_max;
    float int_min, int_max; /* 积分项独立限幅 */
    float dt;
} PID_Clamp_t;

/* 增量式PID控制器 */
typedef struct {
    float kp, ki, kd;
    float prev_error;
    float prev_prev_error;
    float output;
    float out_min, out_max;
    float dt;
} PID_Incremental_t;

/* ========== 初始化 ========== */
void PID_Conditional_Init(PID_Conditional_t *pid, float kp, float ki, float kd,
                          float out_min, float out_max, float dt);
void PID_BackCalc_Init(PID_BackCalc_t *pid, float kp, float ki, float kd,
                       float kb, float out_min, float out_max, float dt);
void PID_Clamp_Init(PID_Clamp_t *pid, float kp, float ki, float kd,
                    float int_min, float int_max,
                    float out_min, float out_max, float dt);
void PID_Incremental_Init(PID_Incremental_t *pid, float kp, float ki, float kd,
                          float out_min, float out_max, float dt);

/* ========== 计算 ========== */
float PID_Conditional_Calc(PID_Conditional_t *pid, float setpoint, float measurement);
float PID_BackCalc_Calc(PID_BackCalc_t *pid, float setpoint, float measurement);
float PID_Clamp_Calc(PID_Clamp_t *pid, float setpoint, float measurement);
float PID_Incremental_Calc(PID_Incremental_t *pid, float setpoint, float measurement);

/* ========== 通用工具 ========== */
float AntiWindup_Clampf(float val, float min, float max);

#ifdef __cplusplus
}
#endif

#endif /* ANTI_WINDUP_H */
