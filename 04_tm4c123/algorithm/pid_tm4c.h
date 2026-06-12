/**
 * @file pid_tm4c.h
 * @brief PID控制器模块 (FPU硬件加速)
 *
 * 利用TM4C123的Cortex-M4F FPU进行浮点运算加速。
 * 支持位置式和增量式PID，带积分限幅、微分滤波。
 *
 * 典型用法:
 *   pid_init(&pid, 1.0f, 0.1f, 0.05f);
 *   float output = pid_calc(&pid, target, actual);
 */
#ifndef __PID_TM4C_H
#define __PID_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== PID参数结构体 ======================== */
typedef struct {
    /* PID增益 */
    float Kp;           /* 比例增益 */
    float Ki;           /* 积分增益 */
    float Kd;           /* 微分增益 */

    /* 输出限幅 */
    float out_max;      /* 输出上限 */
    float out_min;      /* 输出下限 */

    /* 积分限幅 (抗积分饱和) */
    float integral_max; /* 积分累加上限 */
    float integral_min; /* 积分累加下限 */

    /* 微分滤波系数 (0~1, 越大滤波越强) */
    float d_filter_alpha;

    /* 内部状态 */
    float integral;     /* 积分累加 */
    float prev_error;   /* 上一次误差 */
    float prev_d;       /* 上一次微分(滤波后) */
    float output;       /* 最近输出 */
    bool  first_run;    /* 首次运行标志 */
} pid_t;

/* ======================== API ======================== */

/**
 * @brief 初始化PID控制器
 * @param pid   PID实例
 * @param Kp    比例增益
 * @param Ki    积分增益
 * @param Kd    微分增益
 */
void pid_init(pid_t *pid, float Kp, float Ki, float Kd);

/**
 * @brief 设置PID增益(在线调参)
 */
void pid_set_gains(pid_t *pid, float Kp, float Ki, float Kd);

/**
 * @brief 设置输出限幅
 */
void pid_set_output_limit(pid_t *pid, float min, float max);

/**
 * @brief 设置积分限幅
 */
void pid_set_integral_limit(pid_t *pid, float max);

/**
 * @brief 计算PID输出(位置式PID)
 *
 * output = Kp*e + Ki*∫e*dt + Kd*de/dt
 *
 * 使用FPU硬件浮点运算，单精度float。
 * 典型调用周期: 1ms~10ms
 *
 * @param pid    PID实例
 * @param target 目标值
 * @param actual 实际值
 * @return PID输出
 */
float pid_calc(pid_t *pid, float target, float actual);

/**
 * @brief 计算增量式PID输出
 *
 * Δoutput = Kp*(e-e1) + Ki*e + Kd*(e-2*e1+e2)
 *
 * @param pid    PID实例
 * @param target 目标值
 * @param actual 实际值
 * @return PID输出增量
 */
float pid_calc_incremental(pid_t *pid, float target, float actual);

/**
 * @brief 重置PID状态(积分清零)
 */
void pid_reset(pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_TM4C_H */
