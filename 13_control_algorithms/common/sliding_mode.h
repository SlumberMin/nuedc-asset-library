/**
 * @file sliding_mode.h
 * @brief 滑模控制器 - 趋近律+抖振抑制
 * @version 1.0
 * @date 2026-06-10
 * 
 * 特点: 对参数摄动和外部干扰具有强鲁棒性
 * 应用: 电机控制、倒立摆、机器人控制
 */

#ifndef __SLIDING_MODE_H
#define __SLIDING_MODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 趋近律类型 */
typedef enum {
    SMC_REACH_RATE = 0,    /* 等速趋近律: u = -k*sgn(s) */
    SMC_EXP_RATE,          /* 指数趋近律: u = -k*sgn(s) - ε*s */
    SMC_POW_RATE,          /* 幂次趋近律: u = -k*|s|^α*sgn(s) */
} SMC_ReachingLaw_t;

typedef struct {
    /* 滑模面参数 */
    float c;               /* 滑模面斜率 s = e_dot + c*e */
    
    /* 趋近律参数 */
    SMC_ReachingLaw_t law;
    float k;               /* 趋近速度 */
    float epsilon;         /* 指数趋近系数 */
    float alpha;           /* 幂次指数(0~1) */
    
    /* 抖振抑制 */
    float boundary_layer;  /* 边界层厚度 */
    float filter_alpha;    /* 输出滤波系数 */
    
    /* 内部状态 */
    float error;
    float error_last;
    float error_dot;
    float sliding_surface;
    float output;
    float output_filtered;
    
    /* 限幅 */
    float output_max, output_min;
} SMC_t;

void SMC_Init(SMC_t *smc, float c, float k);
void SMC_SetReachingLaw(SMC_t *smc, SMC_ReachingLaw_t law, float k, float epsilon, float alpha);
void SMC_SetBoundaryLayer(SMC_t *smc, float boundary);
void SMC_SetOutputLimit(SMC_t *smc, float min, float max);
float SMC_Calculate(SMC_t *smc, float target, float measurement, float measurement_dot);
void SMC_Reset(SMC_t *smc);

#ifdef __cplusplus
}
#endif

#endif /* __SLIDING_MODE_H */
