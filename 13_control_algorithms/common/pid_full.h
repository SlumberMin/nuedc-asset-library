/**
 * @file pid_full.h
 * @brief PID全系列算法库 v2.0 - 位置式/增量式/分段/积分分离/微分先行/自适应切换
 * @version 2.0
 * @date 2026-06-11
 *
 * 适用平台: STM32 / GD32 / CH32 / Orange Pi 5 / RK3588S
 * 电赛常用: 电机控制、温度控制、平衡控制
 *
 * v2.0优化内容:
 * [OPT-1] 增加前馈补偿(Feedforward)支持
 * [OPT-2] 条件积分抗饱和(Clamping Anti-Windup)替代简单限幅
 * [OPT-3] 增加死区控制, 消除稳态抖动
 * [OPT-4] 位置式/增量式自适应切换(根据误差大小)
 * [OPT-5] 增量式PID增加微分滤波
 * [OPT-6] 增加SetBackCalculation抗饱和(回算法)
 * [OPT-7] Clamp函数改为内联, 减少函数调用开销
 * [OPT-8] 结构体增加dt字段, 微分项计算更精确
 */

#ifndef __PID_FULL_H
#define __PID_FULL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== PID模式枚举 ========== */
typedef enum {
    PID_POSITION = 0,   /* 位置式PID */
    PID_INCREMENT,      /* 增量式PID */
    PID_AUTO_SWITCH,    /* 自适应切换: 误差大用增量式, 误差小用位置式 */
} PID_Mode_t;

typedef enum {
    PID_NORMAL = 0,     /* 普通PID */
    PID_INTEGRAL_SEP,   /* 积分分离 */
    PID_DERIVATIVE_LPF, /* 微分先行(对测量值微分) */
    PID_SEGMENTED,      /* 分段PID */
} PID_Feature_t;

/* ========== 抗饱和策略 ========== */
typedef enum {
    PID_AW_CLAMP = 0,       /* 条件积分抗饱和(推荐) */
    PID_AW_BACK_CALC,       /* 回算法抗饱和 */
    PID_AW_SIMPLE_LIMIT,    /* 简单限幅(传统方法) */
} PID_AntiWindup_t;

/* ========== 分段PID阈值 ========== */
typedef struct {
    float threshold;    /* 误差阈值 */
    float kp, ki, kd;  /* 对应PID参数 */
} PID_Segment_t;

/* ========== PID主结构体 ========== */
typedef struct {
    /* 基本参数 */
    float kp, ki, kd;
    float target;           /* 目标值 */
    float output;           /* 输出值 */

    /* 模式 */
    PID_Mode_t mode;
    PID_Feature_t feature;
    PID_AntiWindup_t anti_windup;

    /* 内部状态 */
    float error;            /* 当前误差 */
    float error_last;       /* 上次误差 */
    float error_prev;       /* 上上次误差 */
    float integral;         /* 累积积分 */
    float derivative;       /* 微分项 */
    float output_last;      /* 上次输出(增量式) */
    float measurement_last; /* 上次测量值(微分先行) */

    /* 限幅 */
    float output_max;       /* 输出上限 */
    float output_min;       /* 输出下限 */
    float integral_max;     /* 积分上限 */
    float integral_min;     /* 积分下限 */

    /* 积分分离参数 */
    float integral_sep_threshold;  /* 积分分离阈值 */

    /* 微分滤波系数 */
    float derivative_filter_alpha; /* 0~1, 越小滤波越强 */

    /* 分段PID */
    PID_Segment_t segments[4];
    uint8_t segment_count;

    /* [OPT-1] 前馈补偿 */
    float feedforward;

    /* [OPT-3] 死区控制 */
    float dead_zone;

    /* [OPT-4] 自适应切换阈值 */
    float auto_switch_threshold;  /* 误差大于此值用增量式 */

    /* [OPT-6] 回算法抗饱和参数 */
    float back_calc_kb;           /* 回算增益 */
    float output_saturated;       /* 限幅后的实际输出 */

    /* [OPT-8] 采样周期(秒), 用于精确微分计算 */
    float dt;
} PID_t;

/* ========== 基本接口 ========== */

void PID_Init(PID_t *pid, float kp, float ki, float kd);
void PID_SetMode(PID_t *pid, PID_Mode_t mode, PID_Feature_t feature);
void PID_SetOutputLimit(PID_t *pid, float min, float max);
void PID_SetIntegralLimit(PID_t *pid, float min, float max);
void PID_SetIntegralSeparation(PID_t *pid, float threshold);
void PID_SetDerivativeFilter(PID_t *pid, float alpha);
void PID_SetSegments(PID_t *pid, const PID_Segment_t *segments, uint8_t count);
void PID_SetTarget(PID_t *pid, float target);

/* [OPT-1] 前馈补偿 */
void PID_SetFeedforward(PID_t *pid, float ff);
/* [OPT-3] 死区 */
void PID_SetDeadZone(PID_t *pid, float dead_zone);
/* [OPT-6] 回算法抗饱和 */
void PID_SetAntiWindup(PID_t *pid, PID_AntiWindup_t type, float kb);
/* [OPT-8] 采样周期 */
void PID_SetSampleTime(PID_t *pid, float dt);

float PID_Calculate(PID_t *pid, float measurement);
void PID_Reset(PID_t *pid);
void PID_PrintStatus(const PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_FULL_H */
