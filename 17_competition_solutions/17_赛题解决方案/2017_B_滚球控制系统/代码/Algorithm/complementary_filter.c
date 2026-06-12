/**
 * @file    complementary_filter.c
 * @brief   互补滤波器实现
 */

#include "complementary_filter.h"

void ComplementaryFilter_Init(ComplementaryFilter_t *cf, float alpha, float dt)
{
    cf->alpha = alpha;
    cf->dt = dt;
    cf->angle = 0;
}

float ComplementaryFilter_Update(ComplementaryFilter_t *cf, float gyro, float accel_angle)
{
    /* 互补滤波 */
    cf->angle = cf->alpha * (cf->angle + gyro * cf->dt) + (1.0f - cf->alpha) * accel_angle;
    return cf->angle;
}

void ComplementaryFilter_Reset(ComplementaryFilter_t *cf)
{
    cf->angle = 0;
}
