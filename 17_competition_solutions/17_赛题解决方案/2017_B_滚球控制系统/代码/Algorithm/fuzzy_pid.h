/**
 * @file    fuzzy_pid.h
 * @brief   模糊自适应PID控制器
 * 
 * 核心思想：根据误差e和误差变化率ec，通过模糊规则在线调整Kp、Ki、Kd
 * 
 * 优势（相比固定参数PID）：
 * 1. 大误差时大增益快速响应
 * 2. 小误差时小增益避免超调
 * 3. 自适应不同工况（如不同负载、不同速度）
 * 
 * 适用题目：
 * - 需要宽范围工作的系统（如小车在不同路面）
 * - 负载变化大的系统（如磁悬浮加不同重量）
 * - 需要快速响应又不能超调的系统
 */

#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H

#include <stdint.h>

typedef struct {
    /* PID参数基值 */
    float Kp0, Ki0, Kd0;
    
    /* 模糊调整范围 */
    float dKp_max;      // Kp最大调整量
    float dKi_max;      // Ki最大调整量
    float dKd_max;      // Kd最大调整量
    
    /* 量化因子 */
    float e_factor;     // 误差量化因子
    float ec_factor;    // 误差变化率量化因子
    
    /* PID内部状态 */
    float integral;
    float prev_error;
    float filtered_d;
    float d_filter_alpha;
    
    /* 输出限幅 */
    float out_min, out_max;
    float integral_max;
    
    /* 输出 */
    float output;
} FuzzyPID_t;

void FuzzyPID_Init(FuzzyPID_t *fp, float kp, float ki, float kd,
                    float dKp, float dKi, float dKd,
                    float e_factor, float ec_factor,
                    float out_min, float out_max);
float FuzzyPID_Calculate(FuzzyPID_t *fp, float target, float actual);

#endif /* __FUZZY_PID_H */
