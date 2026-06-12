#ifndef __POSITION_PID_H
#define __POSITION_PID_H

#include <stdint.h>

/**
 * @brief 位置式PID控制器
 * @note  u = Kp*e + Ki*∫e + Kd*de/dt
 *        适合位置环、角度环等需要绝对输出的场合
 */

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float err;
    float err_last;
    float err_sum;

    /* 积分限幅 */
    float integral_max;
    float integral_min;

    /* 输出限幅 */
    float out_max;
    float out_min;

    /* 积分分离阈值: |e|>阈值时暂停积分, 防超调 */
    float integral_sep_threshold;

    /* 微分先行(对反馈微分而非误差微分, 减少设定值跳变冲击) */
    uint8_t derivative_on_feedback;
    float feedback_last;

    /* 微分滤波 */
    float d_filter_alpha;
    float d_filtered;

    /* 死区 */
    float dead_zone;

    float output;
} PositionPID_t;

void PositionPID_Init(PositionPID_t *pid,
                      float Kp, float Ki, float Kd,
                      float out_min, float out_max);

void PositionPID_SetIntegralLimit(PositionPID_t *pid, float min, float max);
void PositionPID_SetIntegralSeparation(PositionPID_t *pid, float threshold);
void PositionPID_EnableDerivativeOnFeedback(PositionPID_t *pid, uint8_t enable);
void PositionPID_SetDFilter(PositionPID_t *pid, float alpha);
void PositionPID_SetDeadZone(PositionPID_t *pid, float dead_zone);

float PositionPID_Compute(PositionPID_t *pid, float setpoint, float feedback);
void  PositionPID_Reset(PositionPID_t *pid);

#endif /* __POSITION_PID_H */
