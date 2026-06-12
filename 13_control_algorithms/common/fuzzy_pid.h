/**
 * @file fuzzy_pid.h
 * @brief 模糊PID控制器 - 7×7规则表，在线自整定Kp/Ki/Kd
 * @version 1.0
 * @date 2026-06-10
 * 
 * 特点: 无需精确数学模型, 适合非线性、时变系统
 * 应用: 温度控制、液位控制、电机调速
 */

#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 模糊语言值 ========== */
/* NB=负大, NM=负中, NS=负小, ZO=零, PS=正小, PM=正中, PB=正大 */
typedef enum {
    NB = 0, NM, NS, ZO, PS, PM, PB,
    FUZZY_SET_COUNT = 7
} FuzzySet_t;

/* ========== 模糊PID结构体 ========== */
typedef struct {
    /* 基本PID参数 */
    float kp_base, ki_base, kd_base;    /* 基准参数 */
    float kp, ki, kd;                   /* 当前调整后参数 */
    float target;
    float output;
    
    /* 模糊参数调整量范围 */
    float delta_kp_max;   /* Kp最大调整量 */
    float delta_ki_max;   /* Ki最大调整量 */
    float delta_kd_max;   /* Kd最大调整量 */
    
    /* 误差和误差变化率的量化因子 */
    float e_scale;        /* 误差量化因子 */
    float ec_scale;       /* 误差变化率量化因子 */
    
    /* 7×7规则表 */
    int8_t rule_kp[7][7]; /* Kp调整规则表, 值为-3~+3 */
    int8_t rule_ki[7][7]; /* Ki调整规则表 */
    int8_t rule_kd[7][7]; /* Kd调整规则表 */
    
    /* 内部状态 */
    float error;
    float error_last;
    float error_dot;
    float integral;
    float output_last;
    
    /* 限幅 */
    float output_max, output_min;
    float integral_max;
    
    /* 滤波 */
    float derivative_filter;
} FuzzyPID_t;

/* ========== 接口函数 ========== */

/**
 * @brief 初始化模糊PID
 * @param fuzzy 模糊PID结构体
 * @param kp 基准Kp
 * @param ki 基准Ki
 * @param kd 基准Kd
 */
void FuzzyPID_Init(FuzzyPID_t *fuzzy, float kp, float ki, float kd);

/**
 * @brief 设置模糊规则表(可使用默认规则表)
 */
void FuzzyPID_SetDefaultRules(FuzzyPID_t *fuzzy);

/**
 * @brief 设置调整量范围
 */
void FuzzyPID_SetDeltaRange(FuzzyPID_t *fuzzy, float dkp, float dki, float dkd);

/**
 * @brief 设置量化因子
 */
void FuzzyPID_SetScale(FuzzyPID_t *fuzzy, float e_scale, float ec_scale);

/**
 * @brief 设置目标值
 */
void FuzzyPID_SetTarget(FuzzyPID_t *fuzzy, float target);

/**
 * @brief 设置输出限幅
 */
void FuzzyPID_SetOutputLimit(FuzzyPID_t *fuzzy, float min, float max);

/**
 * @brief 模糊PID计算
 * @param measurement 测量值
 * @return 输出值
 */
float FuzzyPID_Calculate(FuzzyPID_t *fuzzy, float measurement);

/**
 * @brief 重置模糊PID
 */
void FuzzyPID_Reset(FuzzyPID_t *fuzzy);

/**
 * @brief 获取当前PID参数(调试用)
 */
void FuzzyPID_GetParams(const FuzzyPID_t *fuzzy, float *kp, float *ki, float *kd);

#ifdef __cplusplus
}
#endif

#endif /* __FUZZY_PID_H */
