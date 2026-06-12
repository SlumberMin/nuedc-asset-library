/**
 * @file    kalman_filter.h
 * @brief   2D卡尔曼滤波器（传感器数据融合）
 *
 * 参考: GitHub高星项目 kalman-c / EKF 优秀实现
 *
 * 特点:
 *   - 2维状态估计（位置 + 速度，或双传感器融合）
 *   - 支持可配置的观测维度（1D或2D观测）
 *   - 纯定点友好设计（全部使用float运算）
 *   - 最小内存占用
 *   - 预测-更新分离，支持无观测时纯预测
 *
 * 典型应用:
 *   - 加速度计 + 陀螺仪数据融合
 *   - 超声波 + 红外测距融合
 *   - 编码器 + IMU位置估计
 *   - 灰度传感器偏差平滑
 *
 * 数学模型:
 *   状态方程: x[k] = A * x[k-1] + B * u[k-1] + w   (w ~ N(0, Q))
 *   观测方程: z[k] = H * x[k] + v                    (v ~ N(0, R))
 *
 *   其中:
 *     x = [位置, 速度]^T   (2D状态向量)
 *     u = [加速度]          (可选控制输入)
 *     z = [观测值]          (1D或2D观测)
 *     A = 状态转移矩阵
 *     B = 控制输入矩阵
 *     H = 观测矩阵
 *     Q = 过程噪声协方差矩阵
 *     R = 观测噪声协方差矩阵
 *     P = 误差协方差矩阵
 *     K = 卡尔曼增益
 */
#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include <stdint.h>
#include <stdbool.h>

#define KF_STATE_DIM   2   /**< 状态维度 [位置, 速度] */
#define KF_OBS_DIM_MAX 2   /**< 最大观测维度 */

/**
 * 卡尔曼滤波器控制块
 *
 * 状态向量: x = [位置, 速度]^T
 * 支持1D或2D观测
 */
typedef struct {
    /* 状态向量 */
    float x[KF_STATE_DIM];              /**< 状态估计 [位置, 速度] */

    /* 误差协方差矩阵 P (2x2, 对称存储) */
    float P[2][2];

    /* 过程噪声协方差矩阵 Q (2x2) */
    float Q[2][2];

    /* 观测噪声协方差 R (最大2x2) */
    float R[KF_OBS_DIM_MAX][KF_OBS_DIM_MAX];

    /* 状态转移矩阵 A (2x2) */
    float A[2][2];

    /* 控制输入矩阵 B (2x1) */
    float B[2];

    /* 观测矩阵 H (最大2x2) */
    float H[KF_OBS_DIM_MAX][KF_STATE_DIM];

    /* 系统参数 */
    float dt;             /**< 采样周期(秒) */
    uint8_t obs_dim;      /**< 观测维度 (1或2) */
    bool initialized;     /**< 初始化标志 */
} KalmanFilter_t;

/**
 * @brief 初始化2D卡尔曼滤波器（恒速模型）
 *
 * 状态转移矩阵（恒速模型）:
 *   A = [1, dt]
 *       [0,  1]
 *
 * 控制输入矩阵:
 *   B = [0.5*dt^2]
 *       [dt      ]
 *
 * @param kf        滤波器控制块
 * @param dt        采样周期（秒）
 * @param proc_noise 过程噪声强度（越大越信任观测）
 * @param meas_noise 观测噪声强度（越大越信任预测）
 */
void Kalman_Init(KalmanFilter_t *kf, float dt, float proc_noise, float meas_noise);

/**
 * @brief 初始化2D卡尔曼滤波器（自定义A/B矩阵）
 */
void Kalman_InitCustom(KalmanFilter_t *kf, float dt,
                       const float A[2][2], const float B[2],
                       const float Q[2][2], const float R[2][2]);

/**
 * @brief 设置观测矩阵H
 * @param kf 滤波器控制块
 * @param H  观测矩阵 (obs_dim x 2)
 */
void Kalman_SetH(KalmanFilter_t *kf, const float H[2][2]);

/**
 * @brief 一步预测（无控制输入）
 *   x_pred = A * x
 *   P_pred = A * P * A^T + Q
 */
void Kalman_Predict(KalmanFilter_t *kf);

/**
 * @brief 带控制输入的一步预测
 *   x_pred = A * x + B * u
 *   P_pred = A * P * A^T + Q
 * @param u 控制输入（如加速度）
 */
void Kalman_PredictWithInput(KalmanFilter_t *kf, float u);

/**
 * @brief 用单维观测更新状态（最常用）
 *   适用场景: 只有一个传感器观测"位置"
 * @param z 观测值
 */
void Kalman_Update1D(KalmanFilter_t *kf, float z);

/**
 * @brief 用2D观测更新状态
 *   适用场景: 两个传感器分别观测"位置"和"速度"
 * @param z 观测向量 [z1, z2]
 */
void Kalman_Update2D(KalmanFilter_t *kf, float z[2]);

/**
 * @brief 获取当前状态估计（位置）
 */
static inline float Kalman_GetPosition(const KalmanFilter_t *kf)
{
    return kf->x[0];
}

/**
 * @brief 获取当前状态估计（速度）
 */
static inline float Kalman_GetVelocity(const KalmanFilter_t *kf)
{
    return kf->x[1];
}

/**
 * @brief 获取估计不确定性（P矩阵对角元素之和）
 *   值越小表示估计越确定
 */
float Kalman_GetUncertainty(const KalmanFilter_t *kf);

/**
 * @brief 设置初始状态
 * @param pos 初始位置
 * @param vel 初始速度
 */
static inline void Kalman_SetState(KalmanFilter_t *kf, float pos, float vel)
{
    kf->x[0] = pos;
    kf->x[1] = vel;
}

/**
 * @brief 重置滤波器到初始状态
 */
void Kalman_Reset(KalmanFilter_t *kf);

#endif /* __KALMAN_FILTER_H */
