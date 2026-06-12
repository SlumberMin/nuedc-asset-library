/**
 * @file    mpc.h
 * @brief   简化MPC模型预测控制器
 * 
 * MPC优势（相比PID）：
 * 1. 显式处理约束（输入/输出/状态约束）
 * 2. 预测未来行为，提前优化
 * 3. 多变量统一处理
 * 
 * 简化实现（适合STM32实时）：
 * - 使用显式MPC或简化的QP求解
 * - 预测时域N=5~10
 * - 控制时域M=2~3
 * 
 * 适用场景：
 * - 有严格约束的系统（如电机电流限制）
 * - 需要预测的系统（如自动驾驶路径规划）
 * - 多输入多输出系统
 */

#ifndef __MPC_H
#define __MPC_H

#include <stdint.h>

#define MPC_N  10   // 预测时域
#define MPC_M  3    // 控制时域

typedef struct {
    /* 系统模型 */
    float A;        // 状态转移系数
    float B;        // 控制输入系数
    
    /* 权重 */
    float Q;        // 状态误差权重
    float R;        // 控制增量权重
    
    /* 约束 */
    float u_min;    // 控制量下限
    float u_max;    // 控制量上限
    float du_max;   // 控制增量上限
    
    /* 状态 */
    float x;        // 当前状态
    float u;        // 当前控制量
    float u_prev;   // 上一次控制量
    
    /* 输出 */
    float output;
} MPC_t;

void MPC_Init(MPC_t *mpc, float A, float B, float Q, float R,
              float u_min, float u_max, float du_max);
float MPC_Calculate(MPC_t *mpc, float ref, float measured);
void MPC_Reset(MPC_t *mpc);

#endif /* __MPC_H */
