/**
 * @file    pid_mspm0.h
 * @brief   PID 控制器 — 适用于 MSPM0G3507
 * @note    位置式/增量式 PID, 支持积分限幅、输出限幅
 */

#ifndef __PID_MSPM0_H
#define __PID_MSPM0_H

#include <stdint.h>

/* ── PID 配置 ────────────────────────────────────────────── */
typedef struct {
    float kp;           /* 比例系数 */
    float ki;           /* 积分系数 */
    float kd;           /* 微分系数 */
    float integral_max; /* 积分限幅 */
    float output_max;   /* 输出限幅 */
    float dead_zone;    /* 死区 */
} PID_Params;

/* ── PID 实例 ────────────────────────────────────────────── */
typedef struct {
    PID_Params param;
    float integral;     /* 积分累加 */
    float prev_error;   /* 上次误差 */
    float prev_prev_error; /* 上上次误差 (增量式) */
    float output;       /* 上次输出 */
} PID;

/* ── API ──────────────────────────────────────────────────── */

/** 初始化 PID */
void PID_Init(PID *pid, const PID_Params *param);

/** 重置 PID 状态 */
void PID_Reset(PID *pid);

/**
 * @brief 位置式 PID 计算
 * @param pid    PID 实例
 * @param target 目标值
 * @param actual 实际值
 * @return 控制输出
 */
float PID_Calc(PID *pid, float target, float actual);

/**
 * @brief 增量式 PID 计算
 * @return 输出增量
 */
float PID_CalcIncremental(PID *pid, float target, float actual);

/** 更新 PID 参数 (运行时调参) */
void PID_SetParams(PID *pid, float kp, float ki, float kd);

#endif /* __PID_MSPM0_H */
