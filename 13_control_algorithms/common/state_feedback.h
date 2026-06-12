/**
 * @file state_feedback.h
 * @brief 状态反馈控制(极点配置法)
 *
 * 控制律: u = -K*x + N*r
 *   K: 状态反馈增益矩阵(通过极点配置或LQR计算)
 *   N: 前馈增益(保证稳态无差)
 *
 * 离散化(前向欧拉):
 *   x[k+1] = (A - B*K)*x[k] + B*N*r[k]
 *   y[k]   = C*x[k]
 *
 * 适用场景:
 *   - 倒立摆平衡控制
 *   - 电机位置/速度环
 *   - 直流母线电压控制
 *   - 需要指定闭环极点的场合
 */

#ifndef __STATE_FEEDBACK_H
#define __STATE_FEEDBACK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SF_MAX_STATES  8
#define SF_MAX_INPUTS  4
#define SF_MAX_OUTPUTS 4

typedef struct {
    uint8_t n;  /* 状态维度 */
    uint8_t m;  /* 输入维度 */
    uint8_t p;  /* 输出维度 */

    /* 系统模型 */
    float A[SF_MAX_STATES][SF_MAX_STATES];   /* 系统矩阵 */
    float B[SF_MAX_STATES][SF_MAX_INPUTS];    /* 输入矩阵 */
    float C[SF_MAX_OUTPUTS][SF_MAX_STATES];   /* 输出矩阵 */

    /* 状态反馈增益 */
    float K[SF_MAX_INPUTS][SF_MAX_STATES];

    /* 前馈增益(保证稳态无差) */
    float N[SF_MAX_INPUTS][SF_MAX_INPUTS];

    /* 状态和输出 */
    float x[SF_MAX_STATES];       /* 当前状态 */
    float x_hat[SF_MAX_STATES];   /* 状态估计(如配合观测器) */

    float dt;  /* 采样周期 */
} StateFeedback_t;

/**
 * @brief 初始化状态反馈控制器
 */
void SF_Init(StateFeedback_t *sf, uint8_t n, uint8_t m, uint8_t p, float dt);

/**
 * @brief 设置系统矩阵 A, B, C
 * @param A_data  n*n (行优先)
 * @param B_data  n*m (行优先)
 * @param C_data  p*n (行优先)
 */
void SF_SetSystemModel(StateFeedback_t *sf, const float *A_data,
                        const float *B_data, const float *C_data);

/**
 * @brief 设置状态反馈增益矩阵 K
 * @param K_data  m*n (行优先)
 */
void SF_SetGainMatrix(StateFeedback_t *sf, const float *K_data);

/**
 * @brief 设置前馈增益 N
 * @param N_data  m*m (行优先)
 */
void SF_SetFeedforwardGain(StateFeedback_t *sf, const float *N_data);

/**
 * @brief 二阶系统极点配置(便捷接口)
 * @param sf      控制器
 * @param zeta1   第一个闭环极点实部系数
 * @param zeta2   第二个闭环极点实部系数
 * @param wn1     第一个闭环极点虚部系数
 * @param wn2     第二个闭环极点虚部系数
 * @details 对于二阶系统 x_dot = Ax + Bu, 直接通过期望极点计算K
 *          期望特征多项式: (s^2 + 2*zeta1*wn1*s + wn1^2)(s^2 + 2*zeta2*wn2*s + wn2^2)
 *          或简化的: s^2 + a1*s + a0
 */
void SF_PolePlacement_2nd(StateFeedback_t *sf, float desired_p1, float desired_p2);

/**
 * @brief 通用极点配置(Ackermann公式)
 * @param sf            控制器
 * @param poles         期望极点数组[n] (实数极点; 复数极点用相邻两个元素存储)
 * @param num_poles     极点个数(等于n)
 */
void SF_Ackermann(StateFeedback_t *sf, const float *poles, uint8_t num_poles);

/**
 * @brief 设置初始状态
 */
void SF_SetInitialState(StateFeedback_t *sf, const float *x0);

/**
 * @brief 状态反馈控制更新
 * @param sf      控制器
 * @param ref     参考输入[m]
 * @param state   当前状态[n] (若为NULL则使用内部状态)
 * @return        控制输出[m]
 */
void SF_Update(StateFeedback_t *sf, const float *ref, const float *state, float *u_out);

/**
 * @brief 获取当前状态估计值
 */
float SF_GetState(StateFeedback_t *sf, uint8_t index);

/**
 * @brief 重置控制器状态
 */
void SF_Reset(StateFeedback_t *sf);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_FEEDBACK_H */
