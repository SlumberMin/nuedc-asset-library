/**
 * @file observer_controller.h
 * @brief 观测器+状态反馈控制器(分离原理)
 *
 * 结构: 观测器估计状态 -> 状态反馈计算控制量
 *
 * 观测器: x_hat[k+1] = (A-LC)*x_hat[k] + B*u[k] + L*y[k]
 * 控制律: u[k] = -K*x_hat[k] + N*r[k]
 *
 * 分离原理: 观测器设计和控制器设计可独立进行
 *
 * 适用场景:
 *   - 无法直接测量所有状态的场合
 *   - 需要状态重构 + 最优控制
 *   - 无传感器电机控制
 */

#ifndef __OBSERVER_CONTROLLER_H
#define __OBSERVER_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OC_MAX_STATES  8
#define OC_MAX_INPUTS  4
#define OC_MAX_OUTPUTS 4

typedef struct {
    uint8_t n;  /* 状态维度 */
    uint8_t m;  /* 输入维度 */
    uint8_t p;  /* 输出维度 */

    /* 系统模型 */
    float A[OC_MAX_STATES][OC_MAX_STATES];
    float B[OC_MAX_STATES][OC_MAX_INPUTS];
    float C[OC_MAX_OUTPUTS][OC_MAX_STATES];

    /* 观测器增益 L */
    float L[OC_MAX_STATES][OC_MAX_OUTPUTS];

    /* 状态反馈增益 K */
    float K[OC_MAX_INPUTS][OC_MAX_STATES];

    /* 前馈增益 N */
    float N[OC_MAX_INPUTS][OC_MAX_INPUTS];

    /* 内部状态 */
    float x_hat[OC_MAX_STATES];  /* 状态估计 */
    float u[OC_MAX_INPUTS];      /* 上一时刻控制量 */
    float dt;
} ObserverController_t;

/**
 * @brief 初始化观测器-控制器
 */
void OC_Init(ObserverController_t *oc, uint8_t n, uint8_t m, uint8_t p, float dt);

/**
 * @brief 设置系统模型 A, B, C
 */
void OC_SetModel(ObserverController_t *oc, const float *A_data,
                  const float *B_data, const float *C_data);

/**
 * @brief 设置观测器增益 L
 * @param L_data  n*p (行优先)
 */
void OC_SetObserverGain(ObserverController_t *oc, const float *L_data);

/**
 * @brief 设置状态反馈增益 K
 * @param K_data  m*n (行优先)
 */
void OC_SetControllerGain(ObserverController_t *oc, const float *K_data);

/**
 * @brief 设置前馈增益 N
 * @param N_data  m*m (行优先)
 */
void OC_SetFeedforwardGain(ObserverController_t *oc, const float *N_data);

/**
 * @brief 设置初始状态估计
 */
void OC_SetInitialState(ObserverController_t *oc, const float *x0);

/**
 * @brief 观测器-控制器更新
 *
 * 一次调用完成: 观测器更新 -> 状态反馈计算
 *
 * @param oc      控制器
 * @param ref     参考输入[m]
 * @param y_meas  测量输出[p]
 * @param u_out   控制输出[m]
 */
void OC_Update(ObserverController_t *oc, const float *ref,
               const float *y_meas, float *u_out);

/**
 * @brief 获取状态估计
 */
float OC_GetEstimatedState(ObserverController_t *oc, uint8_t index);

/**
 * @brief 重置
 */
void OC_Reset(ObserverController_t *oc);

#ifdef __cplusplus
}
#endif

#endif /* __OBSERVER_CONTROLLER_H */
