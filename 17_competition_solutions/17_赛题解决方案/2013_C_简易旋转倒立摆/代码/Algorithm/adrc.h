/**
 * @file adrc.h
 * @brief ADRC自抗扰控制器 - 统一权威版本 v2.0
 * @version 2.0
 * @date    2026-06-11
 * @sync    与nuedc-asset-library/11_控制算法库/common/adrc.h v2.0同步
 */

#ifndef __ADRC_H
#define __ADRC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { ADRC_NONLINEAR = 0, ADRC_LINEAR } ADRC_Mode_t;

typedef struct { float r, h; float x1, x2; } ADRC_TD_t;
typedef struct { float beta1, beta2, beta3; float alpha1, alpha2; float delta, b; float z1, z2, z3; } ADRC_ESO_t;
typedef struct { float beta0, beta1; float alpha0, alpha1; float delta; } ADRC_NLSEF_t;

typedef struct {
    ADRC_TD_t td; ADRC_ESO_t eso; ADRC_NLSEF_t nlsef;
    float h, b, output; float output_min, output_max; ADRC_Mode_t mode;
} ADRC_t;

void ADRC_Init(ADRC_t *adrc, float h, float b);
void ADRC_SetTD(ADRC_t *adrc, float r);
void ADRC_SetESO(ADRC_t *adrc, float beta1, float beta2, float beta3);
void ADRC_SetNLSEF(ADRC_t *adrc, float beta0, float beta1, float alpha0, float alpha1);
void ADRC_SetBandwidth(ADRC_t *adrc, float wo, float wc);
void ADRC_SetOutputLimit(ADRC_t *adrc, float min, float max);
void ADRC_SetMode(ADRC_t *adrc, ADRC_Mode_t mode);
float ADRC_Calculate(ADRC_t *adrc, float target, float measurement);
void ADRC_Reset(ADRC_t *adrc);

#ifdef __cplusplus
}
#endif

#endif /* __ADRC_H */
