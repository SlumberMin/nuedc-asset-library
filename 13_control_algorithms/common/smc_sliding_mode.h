/**
 * @file smc_sliding_mode.h
 * @brief 滑模控制器 V2 - 支持多种趋近律
 * @version 2.0
 * @date 2026-06-11
 *
 * 滑模控制（Sliding Mode Control）核心思想：
 * 设计滑模面 s = 0，使系统状态到达滑模面后沿滑模面滑动至平衡点。
 * V2版本增加了多种趋近律以削弱抖振：
 * - 等速趋近律：ds = -ε*sign(s)
 * - 指数趋近律：ds = -ε*sign(s) - k*s
 * - 幂次趋近律：ds = -k*|s|^α*sign(s)
 * - 趋近律组合：ds = -ε*sign(s) - k*|s|^α*sign(s)
 * - 自适应趋近律：ε在线调整
 */

#ifndef __SMC_SLIDING_MODE_H
#define __SMC_SLIDING_MODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 趋近律类型 */
typedef enum {
    SMC_REACHING_CONSTANT = 0,   /* 等速趋近律 */
    SMC_REACHING_EXPONENTIAL,    /* 指数趋近律 */
    SMC_REACHING_POWER,          /* 幂次趋近律 */
    SMC_REACHING_COMBINED,       /* 组合趋近律 */
    SMC_REACHING_ADAPTIVE,       /* 自适应趋近律 */
    SMC_REACHING_SAT             /* 饱和函数替代符号函数（连续化） */
} SMCReachingLaw_e;

/* 滑模面类型 */
typedef enum {
    SMC_SURFACE_LINEAR = 0,      /* s = c*e + de */
    SMC_SURFACE_NONLINEAR,       /* s = c*|e|^α*sign(e) + de */
    SMC_SURFACE_INTEGRAL         /* s = c*e + de + ki*∫e */
} SMCSurface_e;

/* 滑模控制器结构体 */
typedef struct {
    /* 滑模面参数 */
    SMCSurface_e surface_type;
    float c;           /* 滑模面斜率 c > 0 */
    float alpha_s;     /* 非线性滑模面幂次 (0,1) */
    float ki_s;        /* 积分滑模面增益 */

    /* 趋近律参数 */
    SMCReachingLaw_e reaching_law;
    float epsilon;     /* 等速趋近律参数 ε > 0 */
    float k_reach;     /* 指数/幂次趋近律参数 k > 0 */
    float alpha_r;     /* 幂次趋近律幂次 (0,1) */
    float delta;       /* 饱和函数边界层宽度 */

    /* 自适应参数 */
    float epsilon_min; /* ε最小值 */
    float epsilon_max; /* ε最大值 */
    float adapt_rate;  /* ε自适应速率 */

    /* 内部状态 */
    float error;       /* 当前误差 e(k) */
    float error_last;  /* 上次误差 e(k-1) */
    float error_dot;   /* 误差导数 */
    float s;           /* 滑模面值 */
    float s_last;      /* 上次滑模面值 */
    float s_dot;       /* 滑模面导数 */
    float integral;    /* 积分项 */
    float epsilon_cur; /* 当前ε值（自适应模式） */

    /* 控制参数 */
    float eq_gain;     /* 等效控制增益 */
    float out_min;     /* 输出下限 */
    float out_max;     /* 输出上限 */
    float dt;          /* 采样时间(秒) */
} SMC_t;

/**
 * @brief 初始化滑模控制器
 * @param ctrl 控制器结构体指针
 */
void SMC_Init(SMC_t *ctrl);

/**
 * @brief 设置滑模面类型和参数
 */
void SMC_SetSurface(SMC_t *ctrl, SMCSurface_e type,
                     float c, float alpha_s, float ki_s);

/**
 * @brief 设置趋近律类型和参数
 */
void SMC_SetReachingLaw(SMC_t *ctrl, SMCReachingLaw_e law,
                         float epsilon, float k_reach, float alpha_r);

/**
 * @brief 设置饱和函数边界层宽度（用于连续化）
 */
void SMC_SetBoundaryLayer(SMC_t *ctrl, float delta);

/**
 * @brief 设置自适应趋近律参数
 */
void SMC_SetAdaptiveParam(SMC_t *ctrl,
                           float eps_min, float eps_max, float adapt_rate);

/**
 * @brief 设置等效控制增益（用于前馈补偿）
 */
void SMC_SetEqGain(SMC_t *ctrl, float gain);

/**
 * @brief 设置输出限幅
 */
void SMC_SetLimit(SMC_t *ctrl, float out_min, float out_max);

/**
 * @brief 滑模控制器计算
 * @param ctrl 控制器结构体指针
 * @param target 目标值
 * @param measurement 测量值
 * @param target_dot 目标值导数（NULL则自动差分估算）
 * @return 控制输出
 */
float SMC_Compute(SMC_t *ctrl, float target, float measurement,
                   float *target_dot);

/**
 * @brief 获取当前滑模面值s（用于监控）
 */
float SMC_GetSlidingSurface(SMC_t *ctrl);

/**
 * @brief 复位控制器
 */
void SMC_Reset(SMC_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* __SMC_SLIDING_MODE_H */
