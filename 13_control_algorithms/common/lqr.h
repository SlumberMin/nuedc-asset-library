/**
 * @file lqr.h
 * @brief LQR最优控制器
 * @version 1.0
 * @date 2026-06-10
 * 
 * 特点: 最优控制, 最小化二次型代价函数
 * 应用: 倒立摆平衡、姿态控制、小车平衡
 */

#ifndef __LQR_H
#define __LQR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LQR_MAX_STATES 4

typedef struct {
    uint8_t n;                              /* 状态维度 */
    float A[LQR_MAX_STATES][LQR_MAX_STATES]; /* 状态矩阵 */
    float B[LQR_MAX_STATES];                  /* 输入矩阵 */
    float Q[LQR_MAX_STATES][LQR_MAX_STATES]; /* 状态权重 */
    float R;                                  /* 控制权重 */
    float K[LQR_MAX_STATES];                  /* 反馈增益 */
    float x[LQR_MAX_STATES];                  /* 状态向量 */
    float output;
    float output_max, output_min;
} LQR_t;

void LQR_Init(LQR_t *lqr, uint8_t n);
void LQR_SetSystem(LQR_t *lqr, const float *A, const float *B);
void LQR_SetWeight(LQR_t *lqr, const float *Q, float R);
void LQR_ComputeGain(LQR_t *lqr);  /* 离线迭代求解Riccati方程 */
float LQR_Calculate(LQR_t *lqr, const float *state, const float *target);
void LQR_SetOutputLimit(LQR_t *lqr, float min, float max);
void LQR_Reset(LQR_t *lqr);

#ifdef __cplusplus
}
#endif

#endif /* __LQR_H */
