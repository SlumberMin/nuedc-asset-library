/**
 * @file kalman.h
 * @brief 卡尔曼滤波器 - 统一权威版本 v2.0
 * @version 2.0
 * @date    2026-06-11
 * @sync    与nuedc-asset-library/11_控制算法库/common/kalman.h v2.0同步
 */

#ifndef __KALMAN_H
#define __KALMAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x[2];
    float P[2][2];
    float Q[2][2];
    float R;
    float H[2];
    float dt;
    float innovation;
    float innovation_cov;
} Kalman_t;

typedef struct {
    float alpha;
    float value;
    float initialized;
} Complementary_t;

void Kalman_Init(Kalman_t *kf, float dt, float process_noise, float measure_noise);
void Kalman_SetNoise(Kalman_t *kf, float Q_pos, float Q_vel, float R);
float Kalman_Update(Kalman_t *kf, float measurement);
float Kalman_GetPosition(Kalman_t *kf);
float Kalman_GetVelocity(Kalman_t *kf);
float Kalman_GetInnovation(Kalman_t *kf);
void Kalman_Reset(Kalman_t *kf);

void Complementary_Init(Complementary_t *cf, float alpha);
float Complementary_Update(Complementary_t *cf, float value);

#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_H */
