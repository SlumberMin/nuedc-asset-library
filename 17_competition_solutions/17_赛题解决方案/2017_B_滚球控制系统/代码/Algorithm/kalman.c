/**
 * @file kalman.c
 * @brief 卡尔曼滤波器实现 v2.0 - 统一权威版本
 * @version 2.0
 * @date    2026-06-11
 * @sync    与nuedc-asset-library/11_控制算法库/common/kalman.c v2.0同步
 */

#include "kalman.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

void Kalman_Init(Kalman_t *kf, float dt, float process_noise, float measure_noise) {
    kf->dt=dt; kf->x[0]=0; kf->x[1]=0;
    kf->P[0][0]=1.0f; kf->P[0][1]=0; kf->P[1][0]=0; kf->P[1][1]=1.0f;
    float q=process_noise;
    kf->Q[0][0]=q; kf->Q[0][1]=0; kf->Q[1][0]=0; kf->Q[1][1]=q;
    kf->R=measure_noise; kf->H[0]=1.0f; kf->H[1]=0;
    kf->innovation=0; kf->innovation_cov=0;
}

void Kalman_SetNoise(Kalman_t *kf, float Q_pos, float Q_vel, float R) {
    kf->Q[0][0]=Q_pos; kf->Q[0][1]=0; kf->Q[1][0]=0; kf->Q[1][1]=Q_vel; kf->R=R;
}

float Kalman_Update(Kalman_t *kf, float measurement) {
    float dt=kf->dt;
    float x0_pred=kf->x[0]+dt*kf->x[1], x1_pred=kf->x[1];
    float P00=kf->P[0][0]+dt*(kf->P[0][1]+kf->P[1][0])+dt*dt*kf->P[1][1]+kf->Q[0][0];
    float P01=kf->P[0][1]+dt*kf->P[1][1], P10=P01;
    float P11=kf->P[1][1]+kf->Q[1][1];
    float S=P00+kf->R, K0=P00/S, K1=P10/S;
    kf->innovation=measurement-x0_pred; kf->innovation_cov=S;
    kf->x[0]=x0_pred+K0*kf->innovation; kf->x[1]=x1_pred+K1*kf->innovation;
    kf->P[0][0]=(1.0f-K0)*P00; kf->P[0][1]=(1.0f-K0)*P01; kf->P[1][0]=kf->P[0][1];
    kf->P[1][1]=P11-K1*P10;
    if(kf->P[0][0]<0.0f) kf->P[0][0]=0.0f;
    if(kf->P[1][1]<0.0f) kf->P[1][1]=0.0f;
    return kf->x[0];
}

float Kalman_GetPosition(Kalman_t *kf) { return kf->x[0]; }
float Kalman_GetVelocity(Kalman_t *kf) { return kf->x[1]; }
float Kalman_GetInnovation(Kalman_t *kf) { return kf->innovation; }
void Kalman_Reset(Kalman_t *kf) { kf->x[0]=0;kf->x[1]=0;kf->P[0][0]=1.0f;kf->P[0][1]=0;kf->P[1][0]=0;kf->P[1][1]=1.0f;kf->innovation=0;kf->innovation_cov=0; }
void Complementary_Init(Complementary_t *cf, float alpha) { cf->alpha=alpha; cf->value=0; cf->initialized=0; }
float Complementary_Update(Complementary_t *cf, float value) {
    if (!cf->initialized) { cf->value=value; cf->initialized=1; }
    else cf->value=cf->alpha*value+(1.0f-cf->alpha)*cf->value;
    return cf->value;
}

#ifdef __cplusplus
}
#endif
