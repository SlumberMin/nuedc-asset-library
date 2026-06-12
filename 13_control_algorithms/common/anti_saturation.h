#ifndef ANTI_SATURATION_H
#define ANTI_SATURATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 抗饱和策略枚举
 */
typedef enum {
    ANTI_SAT_BACK_CALCULATION,    // 反计算抗饱和
    ANTI_SAT_CONDITIONAL_INTEGRATION, // 条件积分抗饱和
    ANTI_SAT_TRACKING,            // 跟踪抗饱和
    ANTI_SAT_CLAMPING,            // 钳位抗饱和
    ANTI_SAT_INTEGRATOR_FREEZE    // 积分冻结抗饱和
} AntiSaturationMethod_t;

/**
 * @brief 抗饱和控制器结构体
 */
typedef struct {
    AntiSaturationMethod_t method; // 抗饱和策略
    
    // 输出限幅参数
    float output_min;            // 输出下限
    float output_max;            // 输出上限
    
    // 反计算抗饱和参数
    float Kb;                    // 反计算增益
    float tracking_time_constant; // 跟踪时间常数
    
    // 条件积分抗饱和参数
    float epsilon;               // 条件积分阈值
    
    // 内部状态
    float integral;              // 积分项
    float prev_error;            // 上次误差
    float prev_output;           // 上次输出
    float prev_unsaturated_output; // 上次未饱和输出
    
    // 采样时间
    float dt;                    // 采样周期(秒)
    
    // 状态标志
    uint8_t is_saturated;        // 是否饱和
} AntiSaturation_t;

/**
 * @brief 初始化抗饱和控制器
 * @param ctrl 控制器结构体指针
 * @param method 抗饱和策略
 * @param dt 采样周期(秒)
 */
void AntiSat_Init(AntiSaturation_t *ctrl, AntiSaturationMethod_t method, float dt);

/**
 * @brief 设置输出限幅
 * @param ctrl 控制器结构体指针
 * @param min 输出下限
 * @param max 输出上限
 */
void AntiSat_SetOutputLimit(AntiSaturation_t *ctrl, float min, float max);

/**
 * @brief 配置反计算抗饱和参数
 * @param ctrl 控制器结构体指针
 * @param Kb 反计算增益
 * @param Tt 跟踪时间常数(秒)
 */
void AntiSat_SetBackCalculation(AntiSaturation_t *ctrl, float Kb, float Tt);

/**
 * @brief 配置条件积分抗饱和参数
 * @param ctrl 控制器结构体指针
 * @param epsilon 条件积分阈值
 */
void AntiSat_SetConditionalIntegration(AntiSaturation_t *ctrl, float epsilon);

/**
 * @brief 计算抗饱和修正后的积分项
 * @param ctrl 控制器结构体指针
 * @param error 误差值
 * @param raw_output 原始PID输出(未限幅)
 * @return 修正后的积分项增量
 */
float AntiSat_CalculateIntegral(AntiSaturation_t *ctrl, float error, float raw_output);

/**
 * @brief 应用输出限幅并更新状态
 * @param ctrl 控制器结构体指针
 * @param raw_output 原始输出值
 * @return 限幅后的输出值
 */
float AntiSat_ApplyLimit(AntiSaturation_t *ctrl, float raw_output);

/**
 * @brief 重置抗饱和控制器状态
 * @param ctrl 控制器结构体指针
 */
void AntiSat_Reset(AntiSaturation_t *ctrl);

/**
 * @brief 检查是否处于饱和状态
 * @param ctrl 控制器结构体指针
 * @return 1表示饱和，0表示未饱和
 */
uint8_t AntiSat_IsSaturated(const AntiSaturation_t *ctrl);

/**
 * @brief 获取饱和差值(用于监控)
 * @param ctrl 控制器结构体指针
 * @return 饱和差值(原始输出与限幅输出的差)
 */
float AntiSat_GetSaturationMargin(const AntiSaturation_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* ANTI_SATURATION_H */