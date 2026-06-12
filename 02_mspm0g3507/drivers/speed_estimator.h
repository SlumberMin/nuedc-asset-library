/**
 * @file    speed_estimator.h
 * @brief   速度估计器（M/T法+卡尔曼滤波，用于编码器测速）
 * @note    适配MSPM0G3507编码器接口
 */

#ifndef SPEED_ESTIMATOR_H
#define SPEED_ESTIMATOR_H

#include <stdint.h>

/* 速度估计器句柄 */
typedef struct {
    /* 编码器参数 */
    uint32_t ppr;               /* 编码器每转脉冲数 (PPR) */
    float    gear_ratio;        /* 减速比 */

    /* M/T法变量 */
    int32_t  pulse_count;       /* 采样周期内脉冲计数 */
    uint32_t timer_count;       /* 采样周期内定时器计数 */
    uint32_t sample_period_us;  /* 采样周期 (us) */

    /* 速度输出 */
    float    speed_rpm;         /* 估计转速 (RPM) */
    float    speed_raw;         /* 未经滤波的原始转速 */

    /* 卡尔曼滤波器状态 */
    float    x_est;             /* 状态估计值 */
    float    p_est;             /* 估计误差协方差 */
    float    q;                 /* 过程噪声协方差 */
    float    r;                 /* 测量噪声协方差 */
    float    k_gain;            /* 卡尔曼增益 */

    /* 方向 */
    int8_t   direction;         /* +1正转, -1反转 */
} SpeedEstimator_HandleTypeDef;

/* 初始化 */
void SpeedEst_Init(SpeedEstimator_HandleTypeDef *hse, uint32_t ppr,
                   float gear_ratio, uint32_t sample_period_us);

/* 更新脉冲和定时器计数（在采样中断中调用） */
void SpeedEst_Update(SpeedEstimator_HandleTypeDef *hse,
                     int32_t pulse_count, uint32_t timer_count);

/* 仅用M/T法计算原始速度 */
float SpeedEst_CalcRaw(SpeedEstimator_HandleTypeDef *hse);

/* 卡尔曼滤波 */
float SpeedEst_KalmanFilter(SpeedEstimator_HandleTypeDef *hse, float measurement);

/* 获取滤波后速度 (RPM) */
float SpeedEst_GetRPM(SpeedEstimator_HandleTypeDef *hse);

/* 获取滤波后角速度 (rad/s) */
float SpeedEst_GetRadPerSec(SpeedEstimator_HandleTypeDef *hse);

/* 配置卡尔曼滤波器噪声参数 */
void SpeedEst_SetKalmanNoise(SpeedEstimator_HandleTypeDef *hse,
                             float process_noise, float measure_noise);

#endif /* SPEED_ESTIMATOR_H */
