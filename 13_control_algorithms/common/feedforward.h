/**
 * @file feedforward.h
 * @brief 前馈控制算法
 * @details 基于系统模型的前馈控制，可与PID组合使用以提高响应速度
 */
#ifndef __FEEDFORWARD_H
#define __FEEDFORWARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 前馈控制器类型 */
typedef enum {
    FF_TYPE_NONE = 0,       /* 无前馈 */
    FF_TYPE_VELOCITY,       /* 速度前馈 (一阶) */
    FF_TYPE_ACCELERATION,   /* 加速度前馈 (二阶) */
    FF_TYPE_CUSTOM          /* 自定义查表前馈 */
} FeedForward_Type_t;

/* 前馈控制器参数 */
typedef struct {
    FeedForward_Type_t type;
    float Kv;               /* 速度前馈增益 */
    float Ka;               /* 加速度前馈增益 */
    float Kj;               /* 加加速度前馈增益 (可选) */
    float output_min;       /* 输出下限 */
    float output_max;       /* 输出上限 */
} FeedForward_Config_t;

/* 前馈控制器状态 */
typedef struct {
    FeedForward_Config_t config;
    float prev_ref;         /* 上一次参考值 */
    float prev_vel;         /* 上一次估算速度 */
    float prev_accel;       /* 上一次估算加速度 (用于jerk计算) */
    float output;           /* 当前输出 */
    uint8_t initialized;    /* 初始化标志 */
} FeedForward_t;

/* 查表前馈 */
#define FF_LUT_MAX_SIZE     64
typedef struct {
    float input[FF_LUT_MAX_SIZE];
    float output[FF_LUT_MAX_SIZE];
    uint16_t size;
} FeedForward_LUT_t;

/**
 * @brief 初始化前馈控制器
 * @param ff 控制器句柄
 * @param config 配置参数
 */
void FeedForward_Init(FeedForward_t *ff, const FeedForward_Config_t *config);

/**
 * @brief 前馈控制器计算
 * @param ff 控制器句柄
 * @param reference 当前参考值（目标位置/速度等）
 * @param dt 时间步长(秒)
 * @return 前馈输出
 */
float FeedForward_Calc(FeedForward_t *ff, float reference, float dt);

/**
 * @brief 带速度/加速度显式输入的前馈计算
 * @param ff 控制器句柄
 * @param velocity 目标速度
 * @param acceleration 目标加速度
 * @param jerk 加加速度(可选，传0则忽略)
 * @return 前馈输出
 */
float FeedForward_CalcExplicit(FeedForward_t *ff, float velocity,
                                float acceleration, float jerk);

/**
 * @brief 查表前馈计算
 * @param lut 查表数据
 * @param input 输入值
 * @return 插值输出
 */
float FeedForward_Lookup(const FeedForward_LUT_t *lut, float input);

/**
 * @brief 重置前馈控制器
 */
void FeedForward_Reset(FeedForward_t *ff);

#ifdef __cplusplus
}
#endif

#endif /* __FEEDFORWARD_H */
