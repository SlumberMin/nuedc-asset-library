/**
 * @file neural_pid.h
 * @brief 神经网络PID控制器（简化版 - 单神经元自适应PID）
 * @version 1.0
 * @date 2026-06-11
 * 
 * 使用单神经元在线学习PID的三个权重(对应Kp, Ki, Kd)。
 * 神经元输入：误差e、误差积分∑e、误差微分de
 * 输出：u = w1*x1 + w2*x2 + w3*x3 (加权和)
 * 学习规则：Hebb学习 + 误差监督
 * 
 * 适用于：模型未知、非线性、时变系统
 */

#ifndef __NEURAL_PID_H
#define __NEURAL_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 学习规则类型 */
typedef enum {
    NEURAL_HEBB = 0,     /* Hebb学习规则 */
    NEURAL_DELTA,         /* Delta学习规则 */
    NEURAL_IMPROVED       /* 改进型（带监督信号） */
} NeuralLearnRule_e;

/* 神经元PID结构体 */
typedef struct {
    /* 三个连接权重（对应P, I, D） */
    float w1;  /* 比例权重 */
    float w2;  /* 积分权重 */
    float w3;  /* 微分权重 */

    /* 权重归一化前的原始值（用于计算） */
    float w1_raw, w2_raw, w3_raw;

    /* 学习率 */
    float lr_p;  /* w1学习率 */
    float lr_i;  /* w2学习率 */
    float lr_d;  /* w3学习率 */

    /* 内部状态 */
    float error;
    float error_last;
    float error_prev;
    float integral;
    float derivative;

    /* 输出限幅 */
    float out_min;
    float out_max;
    float integral_max;

    /* 学习规则 */
    NeuralLearnRule_e rule;

    /* 激活函数增益（Sigmoid缩放） */
    float activation_gain;

    /* 采样时间 */
    float dt;
} NeuralPID_t;

/**
 * @brief 初始化神经元PID
 */
void NeuralPID_Init(NeuralPID_t *pid);

/**
 * @brief 设置学习规则
 */
void NeuralPID_SetRule(NeuralPID_t *pid, NeuralLearnRule_e rule);

/**
 * @brief 设置初始权重
 */
void NeuralPID_SetWeights(NeuralPID_t *pid, float w1, float w2, float w3);

/**
 * @brief 设置学习率
 */
void NeuralPID_SetLearningRate(NeuralPID_t *pid, float lr_p, float lr_i, float lr_d);

/**
 * @brief 神经元PID计算
 * @param pid 控制器结构体指针
 * @param target 目标值
 * @param measurement 测量值
 * @return 控制输出
 */
float NeuralPID_Compute(NeuralPID_t *pid, float target, float measurement);

/**
 * @brief 复位
 */
void NeuralPID_Reset(NeuralPID_t *pid);

/**
 * @brief 获取当前权重（即等效Kp, Ki, Kd）
 */
void NeuralPID_GetWeights(NeuralPID_t *pid, float *w1, float *w2, float *w3);

#ifdef __cplusplus
}
#endif

#endif /* __NEURAL_PID_H */
