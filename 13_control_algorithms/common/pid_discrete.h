/**
 * @file pid_discrete.h
 * @brief 离散PID控制器 - 定点化版本
 *
 * 适用于无FPU的MCU(如STM32F0/Cortex-M0),所有运算使用Q16.16定点格式。
 * 支持位置式和增量式两种离散PID算法。
 */

#ifndef PID_DISCRETE_H
#define PID_DISCRETE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Q16.16 定点格式: 高16位整数, 低16位小数 */
#define Q16_SHIFT   16
#define Q16_ONE     (1 << Q16_SHIFT)          /* 1.0 in Q16.16 */
#define FLOAT_TO_Q16(f)  ((int32_t)((f) * (float)Q16_ONE))
#define Q16_TO_FLOAT(q)  ((float)(q) / (float)Q16_ONE)
#define Q16_MUL(a, b)    ((int32_t)(((int64_t)(a) * (b)) >> Q16_SHIFT))
#define Q16_DIV(a, b)    ((int32_t)(((int64_t)(a) << Q16_SHIFT) / (b)))

/* PID 模式 */
typedef enum {
    PID_DISCRETE_POSITION = 0,   /* 位置式PID */
    PID_DISCRETE_INCREMENT       /* 增量式PID */
} pid_discrete_mode_t;

/* PID 抗积分饱和方式 */
typedef enum {
    ANTI_WINDUP_NONE = 0,
    ANTI_WINDUP_CLAMP,           /* 钳位限幅 */
    ANTI_WINDUP_BACKCALC         /* 反馈退饱和(back-calculation) */
} pid_anti_windup_t;

/**
 * @brief 离散PID控制器句柄
 */
typedef struct {
    /* 定点化增益 (Q16.16) */
    int32_t kp;              /* 比例增益 */
    int32_t ki;              /* 积分增益 (已经乘以Ts) */
    int32_t kd;              /* 微分增益 (已经除以Ts) */

    /* 设定值 (Q16.16) */
    int32_t setpoint;

    /* 内部状态 (Q16.16) */
    int32_t integral;        /* 积分累加器 */
    int32_t prev_error;      /* 上一次误差 (用于微分) */
    int32_t prev_output;     /* 上一次输出 (增量式) */

    /* 输出限幅 (Q16.16) */
    int32_t out_min;
    int32_t out_max;

    /* 积分限幅 (Q16.16) */
    int32_t integral_min;
    int32_t integral_max;

    /* 退饱和增益 (Q16.16), 仅 ANTI_WINDUP_BACKCALC 模式 */
    int32_t kb;              /* back-calculation gain */

    /* 配置 */
    pid_discrete_mode_t mode;
    pid_anti_windup_t anti_windup;

    /* 死区 (Q16.16), |error| < deadzone 时不动作 */
    int32_t deadzone;

    /* 首次运行标志 */
    uint8_t first_run;
} pid_discrete_t;

/**
 * @brief 初始化离散PID控制器
 * @param pid       控制器句柄
 * @param kp        比例增益 (浮点)
 * @param ki        积分增益 (浮点), 已乘以采样周期
 * @param kd        微分增益 (浮点), 已除以采样周期
 * @param out_min   输出下限 (浮点)
 * @param out_max   输出上限 (浮点)
 * @param mode      PID模式(位置式/增量式)
 */
void pid_discrete_init(pid_discrete_t *pid,
                       float kp, float ki, float kd,
                       float out_min, float out_max,
                       pid_discrete_mode_t mode);

/**
 * @brief 设置目标设定值
 */
void pid_discrete_set_setpoint(pid_discrete_t *pid, float setpoint);

/**
 * @brief 设置死区
 */
void pid_discrete_set_deadzone(pid_discrete_t *pid, float deadzone);

/**
 * @brief 启用抗积分饱和 (退饱和法)
 * @param kb  退饱和增益 (通常取 1/kt, kt为积分时间常数)
 */
void pid_discrete_enable_backcalc(pid_discrete_t *pid, float kb);

/**
 * @brief 运行一次PID计算
 * @param pid     控制器句柄
 * @param measurement  当前测量值 (浮点)
 * @return 控制输出 (浮点)
 */
float pid_discrete_update(pid_discrete_t *pid, float measurement);

/**
 * @brief 重置PID控制器内部状态
 */
void pid_discrete_reset(pid_discrete_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_DISCRETE_H */
