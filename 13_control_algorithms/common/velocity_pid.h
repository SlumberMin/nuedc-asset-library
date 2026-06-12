#ifndef __VELOCITY_PID_H
#define __VELOCITY_PID_H

#include <stdint.h>

/**
 * @brief 速度式PID控制器
 * @note  输出直接为增量，适合电机速度环等快速响应场合
 *        Δu = Kp*(e[k]-e[k-1]) + Ki*e[k] + Kd*(e[k]-2e[k-1]+e[k-2])
 *        u[k] = u[k-1] + Δu
 */

typedef struct {
    /* 增量式PID参数 */
    float Kp;
    float Ki;
    float Kd;

    /* 历史误差 */
    float err[3];       /* [0]=当前, [1]=上一次, [2]=上上次 */

    /* 输出限幅 */
    float out_min;
    float out_max;

    /* 积分限幅(抗积分饱和) */
    float integral_min;
    float integral_max;

    /* 积分累积 */
    float integral;

    /* 一阶低通滤波系数(对D项), 0~1, 越大滤波越强 */
    float d_filter_alpha;
    float d_filtered;

    /* 死区 */
    float dead_zone;

    /* 输出 */
    float output;
} VelocityPID_t;

/**
 * @brief 初始化速度PID
 */
void VelocityPID_Init(VelocityPID_t *pid,
                      float Kp, float Ki, float Kd,
                      float out_min, float out_max);

/**
 * @brief 设置积分限幅
 */
void VelocityPID_SetIntegralLimit(VelocityPID_t *pid, float min, float max);

/**
 * @brief 设置微分滤波系数
 */
void VelocityPID_SetDFilter(VelocityPID_t *pid, float alpha);

/**
 * @brief 设置死区
 */
void VelocityPID_SetDeadZone(VelocityPID_t *pid, float dead_zone);

/**
 * @brief 计算速度PID输出
 * @param setpoint 设定值
 * @param feedback 反馈值
 * @return 控制输出(绝对值)
 */
float VelocityPID_Compute(VelocityPID_t *pid, float setpoint, float feedback);

/**
 * @brief 重置PID状态
 */
void VelocityPID_Reset(VelocityPID_t *pid);

#endif /* __VELOCITY_PID_H */
