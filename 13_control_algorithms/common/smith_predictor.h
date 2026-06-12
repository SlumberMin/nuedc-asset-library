/**
 * @file smith_predictor.h
 * @brief Smith预估器 - 纯滞后系统补偿
 * @version 1.0
 * @date 2026-06-11
 * 
 * 解决纯滞后系统（如长管道传输、通信延迟、热传导）的控制难题。
 * 
 * 原理：
 *   在反馈回路中并联一个"预估模型"，预测无滞后时的系统输出，
 *   使控制器看到等效无滞后系统，从而可用常规PID整定。
 * 
 * 系统模型: G(s) = Kp * e^(-L*s) / (T*s + 1)
 *   Kp: 增益, T: 时间常数, L: 纯滞后时间
 * 
 * Smith预估器: Gs(s) = Gm(s) * (1 - e^(-L*s))
 *   Gm(s): 不含滞后的模型 = Kp / (T*s + 1)
 */

#ifndef __SMITH_PREDICTOR_H
#define __SMITH_PREDICTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 一阶惯性+纯滞后模型参数 */
typedef struct {
    float Kp;      /* 模型增益 */
    float T;       /* 时间常数(秒) */
    float L;       /* 纯滞后时间(秒) */
} SmithModel_t;

/* Smith预估器结构体 */
typedef struct {
    /* 被控对象模型 */
    SmithModel_t model;

    /* PID控制器参数 */
    float pid_Kp;
    float pid_Ki;
    float pid_Kd;

    /* PID内部状态 */
    float pid_error;
    float pid_error_last;
    float pid_integral;

    /* 无滞后模型状态（一阶惯性环节离散化） */
    float model_state;         /* 模型输出（无滞后） */
    float model_coeff_a;       /* 离散化系数a */
    float model_coeff_b;       /* 离散化系数b */

    /* 纯滞后环节（环形缓冲区实现） */
    float *delay_buffer;       /* 延迟缓冲区 */
    uint16_t delay_size;       /* 缓冲区大小 = ceil(L/dt) */
    uint16_t delay_index;      /* 当前写入索引 */
    float delay_output;        /* 延迟输出（含滞后模型输出） */

    /* Smith预估器输出 */
    float predictor_output;    /* 预估补偿量 = 无滞后输出 - 含滞后输出 */

    /* 修正后的反馈（送给PID的信号） */
    float compensated_feedback;

    /* 输出限幅 */
    float out_min;
    float out_max;
    float integral_max;

    /* 采样时间 */
    float dt;

    /* 内部缓冲区（预分配，避免动态内存） */
    float _static_buffer[512];
} SmithPredictor_t;

/**
 * @brief 初始化Smith预估器
 * @param pred 预估器结构体指针
 * @param dt 采样时间(秒)
 */
void Smith_Init(SmithPredictor_t *pred, float dt);

/**
 * @brief 设置被控对象模型参数
 */
void Smith_SetModel(SmithPredictor_t *pred, float Kp, float T, float L);

/**
 * @brief 设置PID参数（用于Smith预估器内部的控制器）
 */
void Smith_SetPID(SmithPredictor_t *pred, float Kp, float Ki, float Kd);

/**
 * @brief Smith预估器+PID计算
 * @param pred 预估器结构体指针
 * @param target 目标值
 * @param measurement 实际测量值（含滞后）
 * @return 控制输出
 */
float Smith_Compute(SmithPredictor_t *pred, float target, float measurement);

/**
 * @brief 复位所有状态
 */
void Smith_Reset(SmithPredictor_t *pred);

/**
 * @brief 获取预估器内部状态（调试用）
 */
void Smith_GetDebug(SmithPredictor_t *pred, float *model_out, float *delay_out, float *comp_fb);

#ifdef __cplusplus
}
#endif

#endif /* __SMITH_PREDICTOR_H */
