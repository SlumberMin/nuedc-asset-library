/**
 * @file mpc_simple.h
 * @brief 简化MPC控制器 - 梯度法QP求解
 * @version 1.0
 * @date 2026-06-10
 * 
 * 特点: 预测控制, 可处理约束, 适合嵌入式
 * 应用: 电机轨迹跟踪、运动控制
 */

#ifndef __MPC_SIMPLE_H
#define __MPC_SIMPLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPC_MAX_HORIZON 10   /* 最大预测时域 */
#define MPC_MAX_INPUTS   3   /* 最大控制量维度 */

typedef struct {
    /* 预测模型(一阶/二阶离散状态空间) */
    float A[4];              /* 状态矩阵(2×2) */
    float B[2];              /* 输入矩阵(2×1) */
    float C[2];              /* 输出矩阵(1×2) */
    
    /* 预测时域和控制时域 */
    uint8_t Np;              /* 预测时域 */
    uint8_t Nc;              /* 控制时域 */
    
    /* 权重矩阵 */
    float Q;                 /* 误差权重 */
    float R;                 /* 控制增量权重 */
    
    /* 约束 */
    float u_min, u_max;      /* 控制量约束 */
    float du_min, du_max;    /* 控制增量约束 */
    
    /* 梯度法参数 */
    float learning_rate;     /* 学习率 */
    uint8_t max_iterations;  /* 最大迭代次数 */
    
    /* 内部状态 */
    float x[2];              /* 状态向量 */
    float u_last;            /* 上次控制量 */
    float output;
} MPC_t;

void MPC_Init(MPC_t *mpc, float A11, float A12, float A21, float A22,
              float B1, float B2, float C1, float C2);
void MPC_SetHorizon(MPC_t *mpc, uint8_t Np, uint8_t Nc);
void MPC_SetWeight(MPC_t *mpc, float Q, float R);
void MPC_SetConstraint(MPC_t *mpc, float u_min, float u_max, float du_min, float du_max);
void MPC_SetSolver(MPC_t *mpc, float lr, uint8_t max_iter);
float MPC_Calculate(MPC_t *mpc, float target, float measurement);
void MPC_Reset(MPC_t *mpc);

#ifdef __cplusplus
}
#endif

#endif /* __MPC_SIMPLE_H */
