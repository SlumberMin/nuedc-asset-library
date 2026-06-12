/**
 * @file luenberger_observer.h
 * @brief Luenberger观测器 - 状态估计
 * 
 * Luenberger观测器用于线性时不变系统的状态估计：
 *   x_hat_dot = A*x_hat + B*u + L*(y - C*x_hat)
 * 
 * 其中L为观测器增益矩阵，需使(A-LC)的特征值具有负实部。
 * 
 * 典型应用：
 *   - 无法直接测量的状态量估计(如电流环中的反电动势)
 *   - 传感器故障时的冗余估计
 *   - 状态反馈控制中不可测状态的重构
 *   - 无速度传感器电机控制
 * 
 * 参数整定指南：
 *   - L矩阵：使(A-LC)极点比(A-BK)极点快2~5倍
 *   - 极点太远会放大噪声，太近会收敛慢
 *   - 可用极点配置法或LQR法设计L
 */

#ifndef LUENBERGER_OBSERVER_H
#define LUENBERGER_OBSERVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最大支持8维状态/4维输出 */
#define LO_MAX_N  8
#define LO_MAX_M  4

typedef struct {
    /* --- 系统维度 --- */
    int32_t n;  /* 状态维数 */
    int32_t m;  /* 输出维数 */
    int32_t p;  /* 输入维数 */
    float   Ts; /* 采样周期(s) */

    /* --- 系统矩阵(行优先存储) --- */
    float A[LO_MAX_N * LO_MAX_N];  /* n x n */
    float B[LO_MAX_N * LO_MAX_N];  /* n x p (最大p=n) */
    float C[LO_MAX_M * LO_MAX_N];  /* m x n */
    float L[LO_MAX_N * LO_MAX_M];  /* n x m 观测器增益 */

    /* --- 运行时变量 --- */
    float x_hat[LO_MAX_N];          /* 状态估计 */
    float y_hat[LO_MAX_M];          /* 输出估计 */
} LuenbergerObs_t;

/**
 * @brief 初始化Luenberger观测器
 * @param obs 观测器句柄
 * @param n   状态维数
 * @param m   输出维数
 * @param p   输入维数
 * @param Ts  采样周期(s)
 * @return 0=成功
 */
int LO_Init(LuenbergerObs_t *obs, int32_t n, int32_t m, int32_t p, float Ts);

/**
 * @brief 设置系统矩阵
 * @param A 系统矩阵 n x n (行优先)
 * @param B 输入矩阵 n x p
 * @param C 输出矩阵 m x n
 */
void LO_SetSystemMatrices(LuenbergerObs_t *obs, const float *A, const float *B, const float *C);

/**
 * @brief 设置观测器增益矩阵
 * @param L 增益矩阵 n x m (行优先)
 */
void LO_SetGain(LuenbergerObs_t *obs, const float *L);

/**
 * @brief 观测器计算一步
 * @param obs 观测器句柄
 * @param u   输入向量(p维)
 * @param y   测量输出向量(m维)
 * @return 状态估计数组指针(内部存储)
 */
float* LO_Compute(LuenbergerObs_t *obs, const float *u, const float *y);

/**
 * @brief 获取状态估计值
 */
const float* LO_GetStateEstimate(const LuenbergerObs_t *obs);

/**
 * @brief 设置初始状态估计
 */
void LO_SetInitialState(LuenbergerObs_t *obs, const float *x0);

/**
 * @brief 重置观测器
 */
void LO_Reset(LuenbergerObs_t *obs);

#ifdef __cplusplus
}
#endif

#endif /* LUENBERGER_OBSERVER_H */
