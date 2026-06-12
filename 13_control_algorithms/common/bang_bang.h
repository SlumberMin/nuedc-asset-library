/**
 * @file bang_bang.h
 * @brief Bang-Bang控制器（时间最优控制）
 * @details 双模态控制: 误差为正时输出正最大值，为负时输出负最大值
 *          加入滞回区防止抖振，可与PID切换使用
 */
#ifndef __BANG_BANG_H
#define __BANG_BANG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 控制模式 */
typedef enum {
    BB_MODE_SIMPLE = 0,     /* 简单Bang-Bang */
    BB_MODE_HYSTERESIS,     /* 带滞回 */
    BB_MODE_PD_SWITCH,      /* 接近目标时切换PD */
} BangBang_Mode_t;

typedef struct {
    BangBang_Mode_t mode;
    float pos_output;       /* 正向输出值 */
    float neg_output;       /* 负向输出值 */
    float hysteresis;       /* 滞回区宽度 */
    float switch_threshold; /* 切换到PD的误差阈值 */
    float switch_kp;        /* PD模式比例增益 */
    float switch_kd;        /* PD模式微分增益 */
    float output_min;       /* 总输出限幅 */
    float output_max;
} BangBang_Config_t;

typedef struct {
    BangBang_Config_t config;
    float prev_error;
    float output;
    uint8_t in_pd_mode;     /* 是否处于PD模式 */
    uint8_t initialized;
} BangBang_t;

/**
 * @brief 初始化Bang-Bang控制器
 */
void BangBang_Init(BangBang_t *bb, const BangBang_Config_t *config);

/**
 * @brief Bang-Bang控制器计算
 * @param bb 控制器句柄
 * @param setpoint 设定值
 * @param feedback 反馈值
 * @param dt 时间步长(秒)
 * @return 控制输出
 */
float BangBang_Calc(BangBang_t *bb, float setpoint, float feedback, float dt);

/**
 * @brief 获取当前是否处于PD精细控制模式
 */
uint8_t BangBang_IsInPDMode(const BangBang_t *bb);

/**
 * @brief 重置
 */
void BangBang_Reset(BangBang_t *bb);

#ifdef __cplusplus
}
#endif

#endif /* __BANG_BANG_H */
