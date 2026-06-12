/**
 * @file observer_controller.c
 * @brief 观测器+状态反馈控制器实现
 * @details 实现基于Luenberger观测器的状态反馈控制器:
 *          - 观测器: x_hat_new = A*x_hat + B*u + L*(y - C*x_hat)
 *          - 控制律: u = -K*x_hat + N*r
 *          支持多输入多输出(MIMO)系统, 最大维度由宏定义控制。
 */

#include "observer_controller.h"
#include <string.h>

/**
 * @brief 初始化观测器-控制器
 * @param oc 控制器结构体指针
 * @param n 状态维度
 * @param m 输入维度
 * @param p 输出维度
 * @param dt 采样时间间隔(秒)
 */
void OC_Init(ObserverController_t *oc, uint8_t n, uint8_t m, uint8_t p, float dt)
{
    if (oc == NULL) return;
    /* 维度限幅保护 */
    if (n > OC_MAX_STATES)  n = OC_MAX_STATES;
    if (m > OC_MAX_INPUTS)  m = OC_MAX_INPUTS;
    if (p > OC_MAX_OUTPUTS) p = OC_MAX_OUTPUTS;
    if (dt <= 0.0f) dt = 0.001f;

    memset(oc, 0, sizeof(ObserverController_t));
    oc->n = n;
    oc->m = m;
    oc->p = p;
    oc->dt = dt;

    /* 前馈矩阵N默认为单位阵 */
    for (uint8_t i = 0; i < m; i++)
        oc->N[i][i] = 1.0f;
}

/**
 * @brief 设置系统模型矩阵 A, B, C
 * @param oc 控制器结构体指针
 * @param A_data 状态矩阵数据(行优先 n×n)
 * @param B_data 输入矩阵数据(行优先 n×m), 可为NULL
 * @param C_data 输出矩阵数据(行优先 p×n), 可为NULL
 */
void OC_SetModel(ObserverController_t *oc, const float *A_data,
                  const float *B_data, const float *C_data)
{
    if (oc == NULL) return;
    uint8_t n = oc->n, m = oc->m, p = oc->p;

    /* 设置状态矩阵 A */
    if (A_data)
        for (uint8_t i = 0; i < n; i++)
            for (uint8_t j = 0; j < n; j++)
                oc->A[i][j] = A_data[i * n + j];

    /* 设置输入矩阵 B */
    if (B_data)
        for (uint8_t i = 0; i < n; i++)
            for (uint8_t j = 0; j < m; j++)
                oc->B[i][j] = B_data[i * m + j];

    /* 设置输出矩阵 C */
    if (C_data)
        for (uint8_t i = 0; i < p; i++)
            for (uint8_t j = 0; j < n; j++)
                oc->C[i][j] = C_data[i * n + j];
}

/**
 * @brief 设置观测器增益矩阵 L
 * @param oc 控制器结构体指针
 * @param L_data 观测器增益数据(行优先 n×p)
 */
void OC_SetObserverGain(ObserverController_t *oc, const float *L_data)
{
    if (oc == NULL || L_data == NULL) return;
    uint8_t n = oc->n, p = oc->p;
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < p; j++)
            oc->L[i][j] = L_data[i * p + j];
}

/**
 * @brief 设置状态反馈增益矩阵 K
 * @param oc 控制器结构体指针
 * @param K_data 反馈增益数据(行优先 m×n)
 */
void OC_SetControllerGain(ObserverController_t *oc, const float *K_data)
{
    if (oc == NULL || K_data == NULL) return;
    uint8_t m = oc->m, n = oc->n;
    for (uint8_t i = 0; i < m; i++)
        for (uint8_t j = 0; j < n; j++)
            oc->K[i][j] = K_data[i * n + j];
}

/**
 * @brief 设置前馈增益矩阵 N
 * @param oc 控制器结构体指针
 * @param N_data 前馈增益数据(行优先 m×m)
 */
void OC_SetFeedforwardGain(ObserverController_t *oc, const float *N_data)
{
    if (oc == NULL || N_data == NULL) return;
    uint8_t m = oc->m;
    for (uint8_t i = 0; i < m; i++)
        for (uint8_t j = 0; j < m; j++)
            oc->N[i][j] = N_data[i * m + j];
}

/**
 * @brief 设置观测器初始状态估计
 * @param oc 控制器结构体指针
 * @param x0 初始状态数组(长度n)
 */
void OC_SetInitialState(ObserverController_t *oc, const float *x0)
{
    if (oc == NULL || x0 == NULL) return;
    for (uint8_t i = 0; i < oc->n; i++)
        oc->x_hat[i] = x0[i];
}

/**
 * @brief 执行一步观测器-控制器更新
 * @param oc 控制器结构体指针
 * @param ref 参考输入数组(长度m)
 * @param y_meas 测量输出数组(长度p)
 * @param u_out 控制输出数组(长度m)
 *
 * @details 更新流程:
 *          1. 控制律: u = -K*x_hat + N*r
 *          2. 观测器: e_y = y - C*x_hat
 *          3. 状态估计: x_hat = A*x_hat + B*u + L*e_y
 */
void OC_Update(ObserverController_t *oc, const float *ref,
               const float *y_meas, float *u_out)
{
    if (oc == NULL || ref == NULL || y_meas == NULL || u_out == NULL) return;

    uint8_t n = oc->n, m = oc->m, p = oc->p;

    /* Step 1: 计算控制量 u = -K*x_hat + N*r */
    for (uint8_t i = 0; i < m; i++) {
        float u = 0.0f;
        for (uint8_t j = 0; j < n; j++)
            u -= oc->K[i][j] * oc->x_hat[j];
        for (uint8_t j = 0; j < m; j++)
            u += oc->N[i][j] * ref[j];
        u_out[i] = u;
        oc->u[i] = u;
    }

    /* Step 2: 观测器更新
     * 计算残差: e_y = y - C*x_hat
     * x_hat_new = A*x_hat + B*u + L*e_y */
    float y_pred[OC_MAX_OUTPUTS];
    float e_y[OC_MAX_OUTPUTS];

    /* 计算预测输出 y_pred = C * x_hat */
    for (uint8_t i = 0; i < p; i++) {
        y_pred[i] = 0.0f;
        for (uint8_t j = 0; j < n; j++)
            y_pred[i] += oc->C[i][j] * oc->x_hat[j];
        /* 输出残差 */
        e_y[i] = y_meas[i] - y_pred[i];
    }

    /* 状态估计更新: x_hat = A*x_hat + B*u + L*e_y */
    float x_new[OC_MAX_STATES] = {0};
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++)
            x_new[i] += oc->A[i][j] * oc->x_hat[j];
        for (uint8_t j = 0; j < m; j++)
            x_new[i] += oc->B[i][j] * oc->u[j];
        for (uint8_t j = 0; j < p; j++)
            x_new[i] += oc->L[i][j] * e_y[j];
        oc->x_hat[i] = x_new[i];
    }
}

/**
 * @brief 获取指定索引的状态估计值
 * @param oc 控制器结构体指针
 * @param index 状态索引
 * @return 状态估计值, 索引越界返回0
 */
float OC_GetEstimatedState(ObserverController_t *oc, uint8_t index)
{
    if (oc == NULL || index >= oc->n) return 0.0f;
    return oc->x_hat[index];
}

/**
 * @brief 重置观测器和控制器状态
 * @param oc 控制器结构体指针
 */
void OC_Reset(ObserverController_t *oc)
{
    if (oc == NULL) return;
    memset(oc->x_hat, 0, sizeof(oc->x_hat));
    memset(oc->u, 0, sizeof(oc->u));
}
