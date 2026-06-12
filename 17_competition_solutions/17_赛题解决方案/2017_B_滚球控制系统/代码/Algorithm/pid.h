/**
 * @file    pid_v2.h
 * @brief   改进版PID控制算法 v2.1 - 统一权威版本
 * @version 2.1
 * @date    2026-06-11
 * @sync    与nuedc-asset-library/11_控制算法库/common/pid_full.h v2.0同步
 */

#ifndef __PID_V2_H
#define __PID_V2_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PID_MODE_POSITION = 0,
    PID_MODE_INCREMENTAL,
    PID_MODE_AUTO_SWITCH,
} PID_V2_Mode_t;

typedef enum {
    PID_FEATURE_NORMAL = 0,
    PID_FEATURE_INTEGRAL_SEP,
    PID_FEATURE_DERIVATIVE_LPF,
} PID_V2_Feature_t;

typedef enum {
    PID_AW_CLAMP = 0,
    PID_AW_BACK_CALC,
    PID_AW_SIMPLE_LIMIT,
} PID_V2_AntiWindup_t;

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error, prev_error2;
    float filtered_d;
    float prev_output;
    float prev_measurement;
    float out_min, out_max;
    float integral_max;
    float d_filter_alpha;
    float dead_zone;
    float feedforward;
    PID_V2_Mode_t mode;
    PID_V2_Feature_t feature;
    PID_V2_AntiWindup_t anti_windup;
    float back_calc_kb;
    float integral_sep_threshold;
    float dt;
    float auto_switch_threshold;
    float output;
} PID_V2_t;

void PID_V2_Init(PID_V2_t *pid, float kp, float ki, float kd, float out_min, float out_max, float integral_max);
float PID_V2_Calculate(PID_V2_t *pid, float target, float actual);
void PID_V2_Reset(PID_V2_t *pid);
void PID_V2_SetParams(PID_V2_t *pid, float kp, float ki, float kd);
void PID_V2_SetSampleTime(PID_V2_t *pid, float dt);
void PID_V2_SetFilterAlpha(PID_V2_t *pid, float alpha);
void PID_V2_SetDeadZone(PID_V2_t *pid, float dead_zone);
void PID_V2_SetFeedforward(PID_V2_t *pid, float ff);
void PID_V2_SetMode(PID_V2_t *pid, PID_V2_Mode_t mode, PID_V2_Feature_t feature);
void PID_V2_SetAutoSwitch(PID_V2_t *pid, float threshold);
void PID_V2_SetAntiWindup(PID_V2_t *pid, PID_V2_AntiWindup_t type, float kb);
void PID_V2_SetIntegralSeparation(PID_V2_t *pid, float threshold);

#ifdef __cplusplus
}
#endif

#endif /* __PID_V2_H */
