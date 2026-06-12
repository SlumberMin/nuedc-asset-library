/**
 * @file cascade_pid.h
 * @brief 串级PID控制器
 * @details 支持位置环+速度环(或速度环+电流环)双环串级控制
 */
#ifndef __CASCADE_PID_H
#define __CASCADE_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 单环PID参数 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float output_min;
    float output_max;
    float integral_max;     /* 积分限幅 */
    float deadband;         /* 死区 */
    float d_filter_alpha;   /* 微分滤波系数 (0~1, 越大滤波越强) */
} CascadePID_Gain_t;

/* 单环PID状态 */
typedef struct {
    CascadePID_Gain_t gain;
    float integral;
    float prev_error;
    float prev_d;           /* 上一次微分值(用于滤波) */
    float output;
} CascadePID_Loop_t;

/* 串级PID控制器 */
typedef struct {
    CascadePID_Loop_t outer;    /* 外环(位置环) */
    CascadePID_Loop_t inner;    /* 内环(速度环) */
    float outer_setpoint;       /* 外环设定值 */
    float inner_setpoint;       /* 内环设定值(外环输出) */
    float outer_feedback;       /* 外环反馈 */
    float inner_feedback;       /* 内环反馈 */
    float final_output;         /* 最终输出 */
    uint8_t initialized;
} CascadePID_t;

/**
 * @brief 初始化串级PID
 */
void CascadePID_Init(CascadePID_t *pid,
                     const CascadePID_Gain_t *outer_gain,
                     const CascadePID_Gain_t *inner_gain);

/**
 * @brief 串级PID计算（外环设定值驱动）
 * @param pid 控制器句柄
 * @param outer_setpoint 外环设定值(如目标位置)
 * @param outer_feedback 外环反馈(如实际位置)
 * @param inner_feedback 内环反馈(如实际速度)
 * @param dt 时间步长(秒)
 * @return 最终输出
 */
float CascadePID_Calc(CascadePID_t *pid,
                      float outer_setpoint,
                      float outer_feedback,
                      float inner_feedback,
                      float dt);

/**
 * @brief 单环PID计算（内部使用，也可独立调用）
 */
float CascadePID_SingleLoopCalc(CascadePID_Loop_t *loop,
                                 float setpoint, float feedback,
                                 float dt);

/**
 * @brief 更新外环参数
 */
void CascadePID_SetOuterGain(CascadePID_t *pid, const CascadePID_Gain_t *gain);

/**
 * @brief 更新内环参数
 */
void CascadePID_SetInnerGain(CascadePID_t *pid, const CascadePID_Gain_t *gain);

/**
 * @brief 重置串级PID
 */
void CascadePID_Reset(CascadePID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __CASCADE_PID_H */
