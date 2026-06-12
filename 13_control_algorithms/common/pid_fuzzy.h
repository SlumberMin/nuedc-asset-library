/**
 * @file pid_fuzzy_v13.h
 * @brief 模糊PID V13 - 基于模糊规则表的PID参数在线调整
 *
 * V13特性:
 * - 7x7模糊规则表: 误差(e)和误差变化率(ec)双输入
 * - 重心法解模糊: 精确的参数调整量输出
 * - 在线规则调整: 可运行时修改模糊规则
 * - 三角隶属函数: 计算简单，适合MCU
 * - 三个输出: delta_Kp, delta_Ki, delta_Kd
 * - 归一化处理: 输入输出统一到[-6,6]论域
 */

#ifndef PID_FUZZY_V13_H
#define PID_FUZZY_V13_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 语言变量等级 */
#define FUZZY_LEVELS  7  /* NB, NM, NS, ZO, PS, PM, PB */

/* 模糊语言值 */
typedef enum {
    FUZZY_NB = 0,  /* 负大 */
    FUZZY_NM,      /* 负中 */
    FUZZY_NS,      /* 负小 */
    FUZZY_ZO,      /* 零 */
    FUZZY_PS,      /* 正小 */
    FUZZY_PM,      /* 正中 */
    FUZZY_PB       /* 正大 */
} FuzzyLevel_e;

/* 三角隶属函数 */
typedef struct {
    float center;   /* 中心点 */
    float width;    /* 半宽度 */
} TriMF_t;

/* 模糊规则表 (7x7) */
typedef int8_t FuzzyRuleTable_t[FUZZY_LEVELS][FUZZY_LEVELS];

/* 模糊PID V13句柄 */
typedef struct {
    /* 基础PID参数 */
    float Kp_base;
    float Ki_base;
    float Kd_base;

    /* 自适应后参数 */
    float Kp;
    float Ki;
    float Kd;

    /* 模糊输出范围 */
    float delta_Kp_range;   /* delta_Kp 输出范围 */
    float delta_Ki_range;   /* delta_Ki 输出范围 */
    float delta_Kd_range;   /* delta_Kd 输出范围 */

    /* 量化因子 */
    float e_quantize;       /* 误差量化因子 */
    float ec_quantize;      /* 误差变化率量化因子 */

    /* 隶属函数 */
    TriMF_t mf[FUZZY_LEVELS];

    /* 模糊规则表 */
    FuzzyRuleTable_t rule_Kp;
    FuzzyRuleTable_t rule_Ki;
    FuzzyRuleTable_t rule_Kd;

    /* 运行时状态 */
    float error;
    float error_prev;
    float error_rate;
    float integral;
    float derivative;
    float output;

    /* 输出限幅 */
    float output_max;
    float output_min;
    float integral_max;

    /* 积分分离 */
    float integral_separate_threshold;
    uint8_t integral_enable;

    /* 微分先行 */
    uint8_t derivative_on_measurement;
    float measurement;
    float measurement_prev;

    /* 步长 */
    float dt;

    /* 调试: 最近一次模糊输出 */
    float last_delta_Kp;
    float last_delta_Ki;
    float last_delta_Kd;
} PID_FuzzyV13_t;

/**
 * @brief 初始化模糊PID V13
 */
void PID_FuzzyV13_Init(PID_FuzzyV13_t *pid, float Kp, float Ki, float Kd, float dt);

/**
 * @brief 设置模糊输出范围
 */
void PID_FuzzyV13_SetDeltaRange(PID_FuzzyV13_t *pid, float dKp, float dKi, float dKd);

/**
 * @brief 设置量化因子
 */
void PID_FuzzyV13_SetQuantizeFactor(PID_FuzzyV13_t *pid, float e_factor, float ec_factor);

/**
 * @brief 设置输出限幅
 */
void PID_FuzzyV13_SetOutputLimit(PID_FuzzyV13_t *pid, float min, float max);

/**
 * @brief 设置积分限幅
 */
void PID_FuzzyV13_SetIntegralLimit(PID_FuzzyV13_t *pid, float max);

/**
 * @brief 设置积分分离阈值
 */
void PID_FuzzyV13_SetIntegralSeparate(PID_FuzzyV13_t *pid, float threshold);

/**
 * @brief 设置Kp模糊规则表
 */
void PID_FuzzyV13_SetRuleKp(PID_FuzzyV13_t *pid, const FuzzyRuleTable_t table);

/**
 * @brief 设置Ki模糊规则表
 */
void PID_FuzzyV13_SetRuleKi(PID_FuzzyV13_t *pid, const FuzzyRuleTable_t table);

/**
 * @brief 设置Kd模糊规则表
 */
void PID_FuzzyV13_SetRuleKd(PID_FuzzyV13_t *pid, const FuzzyRuleTable_t table);

/**
 * @brief 启用微分先行
 */
void PID_FuzzyV13_EnableDerivativeOnMeasurement(PID_FuzzyV13_t *pid);

/**
 * @brief 模糊PID V13计算
 * @param pid PID句柄
 * @param setpoint 设定值
 * @param measurement 测量值
 * @return 控制输出
 */
float PID_FuzzyV13_Calculate(PID_FuzzyV13_t *pid, float setpoint, float measurement);

/**
 * @brief 复位PID状态
 */
void PID_FuzzyV13_Reset(PID_FuzzyV13_t *pid);

/**
 * @brief 获取当前自适应后的参数
 */
void PID_FuzzyV13_GetAdaptiveParams(const PID_FuzzyV13_t *pid, float *Kp, float *Ki, float *Kd);

#ifdef __cplusplus
}
#endif

#endif /* PID_FUZZY_V13_H */
