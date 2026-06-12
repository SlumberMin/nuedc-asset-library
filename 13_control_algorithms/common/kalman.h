/**
 * @file kalman.h
 * @brief 卡尔曼滤波器 - 标准/扩展/一阶互补
 * @version 1.0
 * @date 2026-06-10
 * 
 * 应用: 传感器数据融合、姿态估计、信号去噪
 */

#ifndef __KALMAN_H
#define __KALMAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 标准卡尔曼滤波(2状态) ========== */
typedef struct {
    float x[2];     /* 状态: [位置, 速度] */
    float P[2][2];  /* 协方差矩阵 */
    float Q[2][2];  /* 过程噪声 */
    float R;         /* 测量噪声 */
    float H[2];      /* 观测矩阵 */
    float dt;        /* 采样周期 */
} Kalman_t;

/* ========== 一阶互补滤波 ========== */
typedef struct {
    float alpha;     /* 滤波系数 */
    float value;     /* 滤波输出 */
    float initialized;
} Complementary_t;

/* ========== 接口 ========== */

void Kalman_Init(Kalman_t *kf, float dt, float process_noise, float measure_noise);
void Kalman_SetNoise(Kalman_t *kf, float Q_pos, float Q_vel, float R);
float Kalman_Update(Kalman_t *kf, float measurement);
float Kalman_GetPosition(Kalman_t *kf);
float Kalman_GetVelocity(Kalman_t *kf);
void Kalman_Reset(Kalman_t *kf);

void Complementary_Init(Complementary_t *cf, float alpha);
float Complementary_Update(Complementary_t *cf, float value);

#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_H */
