/**
 * @file ekf.h
 * @brief 扩展卡尔曼滤波器 (Extended Kalman Filter, EKF)
 *
 * EKF 是卡尔曼滤波器在非线性系统上的推广。
 * 通过在当前估计点处线性化（一阶泰勒展开），将非线性问题
 * 转化为线性问题进行处理。
 *
 * 算法流程：
 *   预测步（时间更新）：
 *     x̂⁻ = f(x̂, u)              # 状态预测
 *     P⁻ = F * P * Fᵀ + Q        # 协方差预测
 *
 *   更新步（量测更新）：
 *     K = P⁻ * Hᵀ * (H * P⁻ * Hᵀ + R)⁻¹   # 卡尔曼增益
 *     x̂ = x̂⁻ + K * (z - h(x̂⁻))             # 状态更新
 *     P = (I - K * H) * P⁻                    # 协方差更新
 *
 * 参数整定指南：
 * ==========================
 * Q (过程噪声协方差)：
 *   - 反映模型不确定性，Q 越大表示越不信任模型
 *   - 增大 Q → 滤波器更信任测量值，响应快但噪声大
 *   - 典型值：根据系统扰动幅度设置对角阵
 *
 * R (量测噪声协方差)：
 *   - 反映传感器噪声，R 越大表示越不信任测量
 *   - 增大 R → 滤波器更信任模型，响应慢但平滑
 *   - 典型值：传感器 datasheet 中的噪声指标
 *
 * P0 (初始协方差)：
 *   - 通常设为较大值，表示初始估计不确定
 *   - 几步迭代后会自动收敛
 *
 * 适用场景：
 * - 非线性系统状态估计
 * - 传感器融合（IMU + GPS 等）
 * - 电机无感控制中的转速/位置估计
 */

#ifndef EKF_H
#define EKF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最大状态维度 */
#define EKF_MAX_STATES    8
#define EKF_MAX_MEASURES  6

/**
 * @brief 非线性状态转移函数指针
 * @param x  当前状态向量 [n]
 * @param u  输入向量 [m]（可为NULL）
 * @param x_new  预测状态输出 [n]
 * @param n  状态维度
 * @param m  输入维度
 */
typedef void (*EKF_StateFunc_t)(const float *x, const float *u,
                                 float *x_new, int n, int m);

/**
 * @brief 非线性量测函数指针
 * @param x  状态向量 [n]
 * @param z  量测预测输出 [p]
 * @param n  状态维度
 * @param p  量测维度
 */
typedef void (*EKF_MeasFunc_t)(const float *x, float *z, int n, int p);

/**
 * @brief 雅可比矩阵计算函数指针
 * @param x  当前状态向量 [n]
 * @param u  输入向量 [m]（可为NULL）
 * @param F  雅可比矩阵输出 [n×n]
 * @param n  状态维度
 * @param m  输入维度
 */
typedef void (*EKF_JacobianFunc_t)(const float *x, const float *u,
                                    float *F, int n, int m);

/* EKF 结构体 */
typedef struct {
    /* 维度 */
    int n;                          /* 状态维度 */
    int p;                          /* 量测维度 */

    /* 系统函数 */
    EKF_StateFunc_t    f_func;      /* 状态转移函数 x' = f(x, u) */
    EKF_MeasFunc_t     h_func;      /* 量测函数 z = h(x) */
    EKF_JacobianFunc_t F_jacobian;  /* 状态转移雅可比 ∂f/∂x */
    EKF_JacobianFunc_t H_jacobian;  /* 量测雅可比 ∂h/∂x */

    /* 状态向量 */
    float x[EKF_MAX_STATES];        /* 状态估计 */

    /* 协方差矩阵（行主序） */
    float P[EKF_MAX_STATES * EKF_MAX_STATES];

    /* 噪声协方差 */
    float Q[EKF_MAX_STATES * EKF_MAX_STATES];  /* 过程噪声 */
    float R[EKF_MAX_MEASURES * EKF_MAX_MEASURES]; /* 量测噪声 */

    /* 工作矩阵 */
    float F[EKF_MAX_STATES * EKF_MAX_STATES];  /* 状态转移雅可比 */
    float H[EKF_MAX_MEASURES * EKF_MAX_STATES]; /* 量测雅可比 */
    float K[EKF_MAX_STATES * EKF_MAX_MEASURES]; /* 卡尔曼增益 */
    float P_pred[EKF_MAX_STATES * EKF_MAX_STATES]; /* 预测协方差 */
    float x_pred[EKF_MAX_STATES];  /* 预测状态 */

    /* 采样周期 */
    float dt;
} EKF_t;

/**
 * @brief 初始化 EKF
 * @param ekf  滤波器指针
 * @param n    状态维度
 * @param p    量测维度
 * @param dt   采样周期
 */
void EKF_Init(EKF_t *ekf, int n, int p, float dt);

/**
 * @brief 设置系统函数
 */
void EKF_SetFunctions(EKF_t *ekf,
                       EKF_StateFunc_t f_func,
                       EKF_MeasFunc_t h_func,
                       EKF_JacobianFunc_t F_jac,
                       EKF_JacobianFunc_t H_jac);

/**
 * @brief 设置初始状态
 */
void EKF_SetInitialState(EKF_t *ekf, const float *x0);

/**
 * @brief 设置初始协方差 P0
 */
void EKF_SetInitialCovariance(EKF_t *ekf, const float *P0);

/**
 * @brief 设置过程噪声 Q
 */
void EKF_SetProcessNoise(EKF_t *ekf, const float *Q);

/**
 * @brief 设置量测噪声 R
 */
void EKF_SetMeasurementNoise(EKF_t *ekf, const float *R);

/**
 * @brief EKF 预测步（时间更新）
 * @param ekf  滤波器指针
 * @param u    输入向量（可为NULL）
 */
void EKF_Predict(EKF_t *ekf, const float *u);

/**
 * @brief EKF 更新步（量测更新）
 * @param ekf  滤波器指针
 * @param z    量测向量
 */
void EKF_Update(EKF_t *ekf, const float *z);

/**
 * @brief EKF 完整一步（预测 + 更新）
 * @param ekf  滤波器指针
 * @param u    输入向量（可为NULL）
 * @param z    量测向量
 */
void EKF_Step(EKF_t *ekf, const float *u, const float *z);

/**
 * @brief 获取状态估计值
 */
float EKF_GetState(const EKF_t *ekf, int index);

#ifdef __cplusplus
}
#endif

#endif /* EKF_H */
