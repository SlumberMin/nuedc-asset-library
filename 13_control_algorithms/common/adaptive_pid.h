/**
 * @file adaptive_pid.h
 * @brief 自适应PID控制器 - 在线调整Kp, Ki, Kd参数
 * @version 1.0
 * @date 2026-06-11
 * 
 * 基于梯度下降法在线调整PID参数，使系统能适应被控对象参数变化。
 * 适用于：电机参数随温度变化、负载突变、老化等场景。
 */

#ifndef __ADAPTIVE_PID_H
#define __ADAPTIVE_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 自适应律类型 */
typedef enum {
    ADAPTIVE_GRADIENT = 0,  /* 梯度下降法 */
    ADAPTIVE_MIT             /* MIT规则 */
} AdaptiveMethod_e;

/* 自适应PID控制器结构体 */
typedef struct {
    /* PID基本参数（在线更新） */
    float Kp;
    float Ki;
    float Kd;

    /* PID参数初始值/范围 */
    float Kp_init, Kp_min, Kp_max;
    float Ki_init, Ki_min, Ki_max;
    float Kd_init, Kd_min, Kd_max;

    /* 内部状态 */
    float error;           /* 当前误差 e(k) */
    float error_last;      /* 上次误差 e(k-1) */
    float error_prev;      /* 上上次误差 e(k-2) */
    float integral;        /* 积分累积 */
    float output_last;     /* 上次输出 y(k-1) */
    float plant_output_last; /* 上次被控对象输出 */

    /* 自适应参数 */
    AdaptiveMethod_e method;
    float learning_rate_p; /* Kp学习率 */
    float learning_rate_i; /* Ki学习率 */
    float learning_rate_d; /* Kd学习率 */
    float alpha;           /* MIT规则的遗忘因子 */
    float deadband;        /* 自适应死区（避免频繁调整） */

    /* 输出限幅 */
    float out_min;
    float out_max;

    /* 积分限幅 */
    float integral_max;

    /* 采样时间(秒) */
    float dt;
} AdaptivePID_t;

/**
 * @brief 初始化自适应PID控制器
 * @param pid 控制器结构体指针
 */
void AdaptivePID_Init(AdaptivePID_t *pid);

/**
 * @brief 设置自适应方法
 */
void AdaptivePID_SetMethod(AdaptivePID_t *pid, AdaptiveMethod_e method);

/**
 * @brief 设置PID参数范围
 */
void AdaptivePID_SetParamRange(AdaptivePID_t *pid,
                                float kp_min, float kp_max,
                                float ki_min, float ki_max,
                                float kd_min, float kd_max);

/**
 * @brief 设置自适应学习率
 */
void AdaptivePID_SetLearningRate(AdaptivePID_t *pid,
                                  float lr_p, float lr_i, float lr_d);

/**
 * @brief 自适应PID计算
 * @param pid 控制器结构体指针
 * @param target 目标值
 * @param measurement 实际测量值（被控对象输出）
 * @return 控制输出
 */
float AdaptivePID_Compute(AdaptivePID_t *pid, float target, float measurement);

/**
 * @brief 复位控制器状态
 */
void AdaptivePID_Reset(AdaptivePID_t *pid);

/**
 * @brief 获取当前PID参数（用于监控/调试）
 */
void AdaptivePID_GetParams(AdaptivePID_t *pid, float *kp, float *ki, float *kd);

#ifdef __cplusplus
}
#endif

#endif /* __ADAPTIVE_PID_H */
