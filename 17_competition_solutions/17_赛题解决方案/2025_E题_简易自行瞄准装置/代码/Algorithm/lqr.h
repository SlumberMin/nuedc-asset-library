/**
 * @file    lqr.h
 * @brief   LQR线性二次最优控制器
 * 
 * LQR优势（相比PID）：
 * 1. 最优控制（最小化状态+控制量的加权和）
 * 2. 多变量统一控制（不需要分解为多个PID环）
 * 3. 理论保证稳定性（通过Riccati方程）
 * 
 * 适用场景：
 * - 倒立摆（2013年C题）
 * - 滚球控制（2017年B题）
 * - 平衡车
 * - 任何可以线性化的多状态系统
 * 
 * 设计方法：
 * 1. 建立状态空间模型：x' = Ax + Bu
 * 2. 定义代价函数：J = ∫(x'Qx + u'Ru)dt
 * 3. 求解Riccati方程得到增益K
 * 4. 控制律：u = -Kx
 */

#ifndef __LQR_H
#define __LQR_H

#include <stdint.h>

/* 最大状态维度 */
#define LQR_MAX_STATES  8

typedef struct {
    uint8_t n;                      // 状态维度
    uint8_t m;                      // 控制量维度
    float K[LQR_MAX_STATES];        // 反馈增益向量
    float x[LQR_MAX_STATES];        // 状态向量
    float u_min;                     // 控制量下限
    float u_max;                     // 控制量上限
    float output;                    // 控制输出
} LQR_t;

void LQR_Init(LQR_t *lqr, uint8_t n, uint8_t m, float u_min, float u_max);
void LQR_SetGain(LQR_t *lqr, const float *K);
void LQR_SetState(LQR_t *lqr, uint8_t idx, float value);
float LQR_Calculate(LQR_t *lqr);
void LQR_Reset(LQR_t *lqr);

#endif /* __LQR_H */
