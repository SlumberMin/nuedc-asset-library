/**
 * @file    speed_estimator.c
 * @brief   速度估计器实现（M/T法+卡尔曼滤波）
 * @note    基于机器人竞赛优秀方案，适配MSPM0G3507
 */

#include "speed_estimator.h"

/* ---------- 初始化 ---------- */

void SpeedEst_Init(SpeedEstimator_HandleTypeDef *hse, uint32_t ppr,
                   float gear_ratio, uint32_t sample_period_us)
{
    /* 编码器参数 */
    hse->ppr              = ppr;
    hse->gear_ratio       = gear_ratio;
    hse->sample_period_us = sample_period_us;

    /* 清零 */
    hse->pulse_count   = 0;
    hse->timer_count   = 0;
    hse->speed_rpm     = 0.0f;
    hse->speed_raw     = 0.0f;
    hse->direction     = 0;

    /* 卡尔曼滤波器默认参数 */
    hse->x_est    = 0.0f;
    hse->p_est    = 1000.0f;  /* 初始不确定度较大 */
    hse->q        = 10.0f;    /* 过程噪声 */
    hse->r        = 100.0f;   /* 测量噪声 */
    hse->k_gain   = 0.0f;
}

/* ---------- 更新计数 ---------- */

void SpeedEst_Update(SpeedEstimator_HandleTypeDef *hse,
                     int32_t pulse_count, uint32_t timer_count)
{
    hse->pulse_count = pulse_count;
    hse->timer_count = timer_count;

    /* 方向判断 */
    if (pulse_count > 0)      hse->direction =  1;
    else if (pulse_count < 0) hse->direction = -1;

    /* 计算原始速度 */
    hse->speed_raw = SpeedEst_CalcRaw(hse);

    /* 卡尔曼滤波 */
    hse->speed_rpm = SpeedEst_KalmanFilter(hse, hse->speed_raw);
}

/* ---------- M/T法计算原始速度 ---------- */
/*
 * M/T法：同时使用脉冲计数（M法）和定时器计数（T法）
 *   RPM = (pulse_count * 60 * 1e6) / (ppr * 4 * timer_count_us)
 *   ×4 是因为编码器AB相四倍频
 */

float SpeedEst_CalcRaw(SpeedEstimator_HandleTypeDef *hse)
{
    if (hse->timer_count == 0 || hse->ppr == 0) {
        return 0.0f;
    }

    /* 脉冲数取绝对值 */
    int32_t pulses = hse->pulse_count;
    if (pulses < 0) pulses = -pulses;

    /* 计算RPM */
    float rpm = ((float)pulses * 60.0f * 1000000.0f) /
                ((float)hse->ppr * 4.0f * (float)hse->timer_count);

    /* 乘减速比 */
    rpm /= hse->gear_ratio;

    /* 恢复符号 */
    return hse->direction * rpm;
}

/* ---------- 卡尔曼滤波 ---------- */
/*
 * 一维卡尔曼滤波：
 *   预测: x_pred = x_est, p_pred = p_est + q
 *   更新: k = p_pred / (p_pred + r)
 *         x_est = x_pred + k * (z - x_pred)
 *         p_est = (1 - k) * p_pred
 */

float SpeedEst_KalmanFilter(SpeedEstimator_HandleTypeDef *hse, float measurement)
{
    /* 预测 */
    float x_pred = hse->x_est;
    float p_pred = hse->p_est + hse->q;

    /* 更新 */
    hse->k_gain = p_pred / (p_pred + hse->r);
    hse->x_est  = x_pred + hse->k_gain * (measurement - x_pred);
    hse->p_est  = (1.0f - hse->k_gain) * p_pred;

    return hse->x_est;
}

/* ---------- 获取速度 ---------- */

float SpeedEst_GetRPM(SpeedEstimator_HandleTypeDef *hse)
{
    return hse->speed_rpm;
}

float SpeedEst_GetRadPerSec(SpeedEstimator_HandleTypeDef *hse)
{
    return hse->speed_rpm * 2.0f * 3.14159265f / 60.0f;
}

/* ---------- 配置噪声参数 ---------- */

void SpeedEst_SetKalmanNoise(SpeedEstimator_HandleTypeDef *hse,
                             float process_noise, float measure_noise)
{
    hse->q = process_noise;
    hse->r = measure_noise;
}
