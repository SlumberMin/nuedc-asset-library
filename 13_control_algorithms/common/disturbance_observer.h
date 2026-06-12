/**
 * @file disturbance_observer.h
 * @brief 扰动观测器 (Disturbance Observer, DOB)
 *
 * 原理:
 *   DOB通过估计系统输入端的等效扰动来增强鲁棒性。
 *   结构: d_hat = Q(s) * [u + G_n^-1(s) * y] - u
 *   其中 Q(s) 是低通滤波器, G_n(s) 是标称模型
 *
 * 支持:
 *   1. 一阶低通滤波器 Q(s)
 *   2. 二阶Butterworth滤波器 Q(s)
 *   3. 离散化的速度/加速度扰动估计
 */

#ifndef DISTURBANCE_OBSERVER_H
#define DISTURBANCE_OBSERVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Q滤波器类型 */
typedef enum {
    DOB_Q_FIRST_ORDER,      /* 一阶低通: Q(s) = 1/(tau*s+1) */
    DOB_Q_BUTTERWORTH_2     /* 二阶Butterworth低通 */
} DOB_QFilterType_e;

/* ========== 一阶系统DOB ========== */
typedef struct {
    /* 标称模型参数: G_n(s) = Kn / (s + an) */
    float Kn;               /* 标称模型增益 */
    float an;               /* 标称模型极点 */

    /* Q滤波器参数 (离散化后) */
    float q_tau;            /* Q滤波器时间常数 */
    float q_alpha;          /* 离散化系数: alpha = dt / (tau + dt) */
    float dt;

    /* 内部状态 */
    float q_state;          /* Q滤波器状态 */
    float disturbance_hat;  /* 估计的扰动 */
    float prev_u;           /* 上一次控制量 */
    float prev_y;           /* 上一次输出 */
    uint8_t initialized;
} DOB_FirstOrder_t;

/* ========== 二阶系统DOB ========== */
typedef struct {
    /* 标称模型: G_n(s) = Kn / (s^2 + a1*s + a0) */
    float Kn, a1, a0;

    /* Q滤波器 (二阶Butterworth) */
    float q_wn;             /* Q滤波器截止频率 */
    float q_zeta;           /* 阻尼比 (Butterworth: 0.707) */
    float dt;

    /* 离散化状态空间 */
    float x1, x2;           /* 观测器状态 */
    float y_hat;            /* 估计输出 */
    float disturbance_hat;  /* 估计扰动 */
    float prev_u;
    uint8_t initialized;
} DOB_SecondOrder_t;

/* ========== 通用速度扰动观测器 ========== */
typedef struct {
    float alpha;            /* 滤波系数 */
    float dt;
    float d_hat;            /* 扰动估计 */
    float prev_u;
    float prev_y;
    float prev_prev_y;
    float model_K;          /* 标称模型增益 */
    float model_tau;        /* 标称模型时间常数 */
    uint8_t initialized;
} DOB_Velocity_t;

/* ========== 一阶系统DOB API ========== */
void DOB_FirstOrder_Init(DOB_FirstOrder_t *dob, float Kn, float an,
                         float q_tau, float dt);
float DOB_FirstOrder_Update(DOB_FirstOrder_t *dob, float u, float y);
float DOB_FirstOrder_GetDisturbance(DOB_FirstOrder_t *dob);
void DOB_FirstOrder_Reset(DOB_FirstOrder_t *dob);

/* ========== 二阶系统DOB API ========== */
void DOB_SecondOrder_Init(DOB_SecondOrder_t *dob, float Kn, float a1, float a0,
                          float q_wn, float dt);
float DOB_SecondOrder_Update(DOB_SecondOrder_t *dob, float u, float y);
float DOB_SecondOrder_GetDisturbance(DOB_SecondOrder_t *dob);
void DOB_SecondOrder_Reset(DOB_SecondOrder_t *dob);

/* ========== 速度扰动观测器 API ========== */
void DOB_Velocity_Init(DOB_Velocity_t *dob, float model_K, float model_tau,
                       float alpha, float dt);
float DOB_Velocity_Update(DOB_Velocity_t *dob, float u, float y);
float DOB_Velocity_GetDisturbance(DOB_Velocity_t *dob);
void DOB_Velocity_Reset(DOB_Velocity_t *dob);

/* ========== 工具函数 ========== */
float DOB_ComplementFilter(float raw, float filtered, float alpha);

#ifdef __cplusplus
}
#endif

#endif /* DISTURBANCE_OBSERVER_H */
