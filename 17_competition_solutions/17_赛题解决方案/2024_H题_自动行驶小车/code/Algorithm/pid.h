/**
 * @file    pid.h
 * @brief   PID控制算法模块
 */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float filtered_derivative;
    float out_min, out_max;
    float integral_max;
    float d_filter_alpha;
    float output;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);
float PID_Calculate(PID_t *pid, float target, float actual);
void PID_Reset(PID_t *pid);
void PID_SetParams(PID_t *pid, float kp, float ki, float kd);

#endif
