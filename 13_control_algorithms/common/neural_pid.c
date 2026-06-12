/**
 * @file neural_pid.c
 * @brief 神经元PID控制器实现
 * 
 * 单神经元自适应PID：
 if (dt <= 0.0f) dt = 0.001f;  /* 防除零 */
 *   输入向量 X = [x1, x2, x3] = [e, ∑e, de/dt]
 *   权重向量 W = [w1, w2, w3]
 *   输出 u = f(w1*x1 + w2*x2 + w3*x3)
 * 
 * 权重归一化: wi' = wi / (|w1|+|w2|+|w3|)  保证收敛
 * 
 * 学习规则:
 *   Hebb:   Δwi = lr_i * u * xi
 *   Delta:  Δwi = lr_i * e * xi
 *   改进型: Δwi = lr_i * e * u * (xi + e * Δu)
 */

#include "neural_pid.h"
#include <math.h>

#define CLAMP(val, min_v, max_v) \
    do { if ((val) < (min_v)) (val) = (min_v); \
         if ((val) > (max_v)) (val) = (max_v); } while(0)

/* Sigmoid激活函数 */
static float _Sigmoid(float x, float gain)
{
    return 2.0f / (1.0f + expf(-gain * x)) - 1.0f; /* 输出范围(-1, 1) */
}

void NeuralPID_Init(NeuralPID_t *pid)
{
    /* 初始权重 */
    pid->w1 = 0.5f;   /* P权重 */
    pid->w2 = 0.1f;   /* I权重 */
    pid->w3 = 0.05f;  /* D权重 */

    pid->w1_raw = pid->w1;
    pid->w2_raw = pid->w2;
    pid->w3_raw = pid->w3;

    /* 学习率 */
    pid->lr_p = 0.2f;
    pid->lr_i = 0.1f;
    pid->lr_d = 0.05f;

    pid->rule = NEURAL_IMPROVED;
    pid->activation_gain = 0.5f;

    pid->out_min = -1000.0f;
    pid->out_max =  1000.0f;
    pid->integral_max = 500.0f;
    pid->dt = 0.001f;

    NeuralPID_Reset(pid);
}

void NeuralPID_SetRule(NeuralPID_t *pid, NeuralLearnRule_e rule)
{
    pid->rule = rule;
}

void NeuralPID_SetWeights(NeuralPID_t *pid, float w1, float w2, float w3)
{
    pid->w1 = w1; pid->w2 = w2; pid->w3 = w3;
    pid->w1_raw = w1; pid->w2_raw = w2; pid->w3_raw = w3;
}

void NeuralPID_SetLearningRate(NeuralPID_t *pid, float lr_p, float lr_i, float lr_d)
{
    pid->lr_p = lr_p;
    pid->lr_i = lr_i;
    pid->lr_d = lr_d;
}

/* 权重归一化 */
static void _NormalizeWeights(NeuralPID_t *pid)
{
    float sum = fabsf(pid->w1_raw) + fabsf(pid->w2_raw) + fabsf(pid->w3_raw);
    if (sum > 1e-6f) {
        pid->w1 = pid->w1_raw / sum;
        pid->w2 = pid->w2_raw / sum;
        pid->w3 = pid->w3_raw / sum;
    }
}

/* 学习规则更新权重 */
static void _UpdateWeights(NeuralPID_t *pid, float error, float x1, float x2, float x3, float u_raw)
{
    float dw1 = 0, dw2 = 0, dw3 = 0;

    switch (pid->rule) {
    case NEURAL_HEBB:
        /* Δw = lr * u * x  (经典Hebb) */
        dw1 = pid->lr_p * u_raw * x1;
        dw2 = pid->lr_i * u_raw * x2;
        dw3 = pid->lr_d * u_raw * x3;
        break;

    case NEURAL_DELTA:
        /* Δw = lr * e * x  (监督学习) */
        dw1 = pid->lr_p * error * x1;
        dw2 = pid->lr_i * error * x2;
        dw3 = pid->lr_d * error * x3;
        break;

    case NEURAL_IMPROVED:
        /* Δw = lr * e * x * (1 + sign(u*error))  (带正反馈修正) */
        {
            float sign = (u_raw * error > 0) ? 1.0f : -1.0f;
            float factor = 1.0f + 0.5f * sign;
            dw1 = pid->lr_p * error * x1 * factor;
            dw2 = pid->lr_i * error * x2 * factor;
            dw3 = pid->lr_d * error * x3 * factor;
        }
        break;
    }

    pid->w1_raw += dw1;
    pid->w2_raw += dw2;
    pid->w3_raw += dw3;

    /* 防止权重为负（PID参数非负） */
    if (pid->w1_raw < 0.0f) pid->w1_raw = 0.0f;
    if (pid->w2_raw < 0.0f) pid->w2_raw = 0.0f;
    if (pid->w3_raw < 0.0f) pid->w3_raw = 0.0f;

    /* 权重上限防爆 */
    CLAMP(pid->w1_raw, 0.0f, 10.0f);
    CLAMP(pid->w2_raw, 0.0f, 10.0f);
    CLAMP(pid->w3_raw, 0.0f, 10.0f);

    _NormalizeWeights(pid);
}

float NeuralPID_Compute(NeuralPID_t *pid, float target, float measurement)
{
    float error = target - measurement;

    /* 计算三个输入 */
    float x1 = error;                                     /* P: 误差 */
    float x2 = pid->integral + error * pid->dt;           /* I: 误差积分 */
    float x3 = (error - pid->error_last) / pid->dt;       /* D: 误差微分 */

    /* 更新积分（先计算再更新，避免双重积分） */
    pid->integral += error * pid->dt;
    CLAMP(pid->integral, -pid->integral_max, pid->integral_max);
    x2 = pid->integral;

    /* 神经元加权求和 */
    float u_raw = pid->w1 * x1 + pid->w2 * x2 + pid->w3 * x3;

    /* 激活函数（可选，限制输出范围） */
    /* float u = _Sigmoid(u_raw, pid->activation_gain) * pid->out_max; */
    float u = u_raw; /* 线性输出，简单有效 */

    CLAMP(u, pid->out_min, pid->out_max);

    /* 权重学习更新 */
    _UpdateWeights(pid, error, x1, x2, x3, u_raw);

    /* 更新历史 */
    pid->error_prev = pid->error_last;
    pid->error_last = error;

    return u;
}

void NeuralPID_Reset(NeuralPID_t *pid)
{
    pid->error = 0.0f;
    pid->error_last = 0.0f;
    pid->error_prev = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
}

void NeuralPID_GetWeights(NeuralPID_t *pid, float *w1, float *w2, float *w3)
{
    if (w1) *w1 = pid->w1;
    if (w2) *w2 = pid->w2;
    if (w3) *w3 = pid->w3;
}
