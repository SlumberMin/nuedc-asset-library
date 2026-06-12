/**
 * @file anti_saturation.c
 * @brief 抗积分饱和(Anti-Windup)算法实现
 * @details 提供多种抗积分饱和策略:
 *          - 反计算法(Back Calculation)
 *          - 条件积分法(Conditional Integration)
 *          - 跟踪法(Tracking Anti-Windup)
 *          - 钳位法(Clamping)
 *          - 积分冻结法(Integrator Freeze)
 *          用于PID控制器中防止积分项在执行器饱和时持续累积。
 */

#include "anti_saturation.h"
#include <math.h>

/**
 * @brief 浮点数限幅辅助函数
 * @param value 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的值
 */
static float clampf(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 初始化抗饱和控制器
 * @param ctrl 抗饱和结构体指针
 * @param method 抗饱和方法枚举
 * @param dt 采样时间间隔(秒)
 */
void AntiSat_Init(AntiSaturation_t *ctrl, AntiSaturationMethod_t method, float dt)
{
    if (ctrl == NULL) return;
    if (dt <= 0.0f) dt = 0.001f;

    ctrl->method = method;
    ctrl->dt = dt;
    
    /* 默认输出限幅范围 */
    ctrl->output_min = -1000.0f;
    ctrl->output_max = 1000.0f;
    
    /* 默认参数 */
    ctrl->Kb = 1.0f;
    ctrl->tracking_time_constant = 0.1f;
    ctrl->epsilon = 0.01f;
    
    /* 清零状态 */
    ctrl->integral = 0.0f;
    ctrl->prev_error = 0.0f;
    ctrl->prev_output = 0.0f;
    ctrl->prev_unsaturated_output = 0.0f;
    ctrl->is_saturated = 0;
}

/**
 * @brief 设置输出限幅范围
 * @param ctrl 抗饱和结构体指针
 * @param min 输出下限
 * @param max 输出上限
 */
void AntiSat_SetOutputLimit(AntiSaturation_t *ctrl, float min, float max)
{
    if (ctrl == NULL) return;
    ctrl->output_min = min;
    ctrl->output_max = max;
}

/**
 * @brief 设置反计算法参数
 * @param ctrl 抗饱和结构体指针
 * @param Kb 反计算增益
 * @param Tt 跟踪时间常数
 */
void AntiSat_SetBackCalculation(AntiSaturation_t *ctrl, float Kb, float Tt)
{
    if (ctrl == NULL) return;
    ctrl->Kb = Kb;
    ctrl->tracking_time_constant = Tt;
}

/**
 * @brief 设置条件积分法的误差阈值
 * @param ctrl 抗饱和结构体指针
 * @param epsilon 误差死区阈值
 */
void AntiSat_SetConditionalIntegration(AntiSaturation_t *ctrl, float epsilon)
{
    if (ctrl == NULL) return;
    ctrl->epsilon = epsilon;
}

/**
 * @brief 反计算法抗饱和
 * @param ctrl 抗饱和结构体指针
 * @param error 当前误差
 * @param raw_output 限幅前的原始输出
 * @return 积分增量
 *
 * @details 公式: ΔI = error + Kb*(saturated - raw)/Tt
 *          当输出饱和时, 反向修正积分项以减小饱和
 */
static float back_calculation(AntiSaturation_t *ctrl, float error, float raw_output)
{
    float saturated = clampf(raw_output, ctrl->output_min, ctrl->output_max);
    float saturation_error = saturated - raw_output;
    
    float integral_increment = error + ctrl->Kb * saturation_error / ctrl->tracking_time_constant;
    
    return integral_increment * ctrl->dt;
}

/**
 * @brief 条件积分法抗饱和
 * @param ctrl 抗饱和结构体指针
 * @param error 当前误差
 * @param raw_output 限幅前的原始输出
 * @return 积分增量
 *
 * @details 当输出饱和且误差方向会使饱和加剧时, 停止积分
 *          当误差在死区内时也停止积分
 */
static float conditional_integration(AntiSaturation_t *ctrl, float error, float raw_output)
{
    float saturated = clampf(raw_output, ctrl->output_min, ctrl->output_max);
    
    int is_upper_saturated = (raw_output >= ctrl->output_max);
    int is_lower_saturated = (raw_output <= ctrl->output_min);
    
    /* 检查是否需要冻结积分 */
    int freeze_integration = 0;
    if (is_upper_saturated && error > 0) {
        freeze_integration = 1; /* 输出已达上限且误差为正 → 冻结 */
    }
    if (is_lower_saturated && error < 0) {
        freeze_integration = 1; /* 输出已达下限且误差为负 → 冻结 */
    }
    
    /* 误差在死区内也冻结 */
    if (fabsf(error) < ctrl->epsilon) {
        freeze_integration = 1;
    }
    
    if (freeze_integration) {
        return 0.0f;
    }
    
    return error * ctrl->dt;
}

/**
 * @brief 跟踪法抗饱和
 * @param ctrl 抗饱和结构体指针
 * @param error 当前误差
 * @param raw_output 限幅前的原始输出
 * @return 积分增量
 *
 * @details 使用一阶滤波器跟踪饱和误差:
 *          α = dt/(Tt+dt), correction = α*(saturated-raw)
 */
static float tracking_anti_windup(AntiSaturation_t *ctrl, float error, float raw_output)
{
    float saturated = clampf(raw_output, ctrl->output_min, ctrl->output_max);
    float saturation_error = saturated - raw_output;
    
    /* 一阶滤波系数 */
    float alpha = ctrl->dt / (ctrl->tracking_time_constant + ctrl->dt);
    float tracking_correction = alpha * saturation_error;
    
    return (error + tracking_correction) * ctrl->dt;
}

/**
 * @brief 钳位法抗饱和
 * @param ctrl 抗饱和结构体指针
 * @param error 当前误差
 * @param raw_output 限幅前的原始输出
 * @return 积分增量
 *
 * @details 当输出饱和且误差方向与饱和方向相同时, 衰减积分
 */
static float clamping_anti_windup(AntiSaturation_t *ctrl, float error, float raw_output)
{
    float saturated = clampf(raw_output, ctrl->output_min, ctrl->output_max);
    
    if (saturated == raw_output) {
        /* 未饱和, 正常积分 */
        return error * ctrl->dt;
    }
    
    /* 已饱和: 检查误差方向 */
    if ((error > 0 && raw_output > ctrl->output_max) ||
        (error < 0 && raw_output < ctrl->output_min)) {
        /* 误差会使饱和更严重, 衰减积分 */
        return error * ctrl->dt * 0.1f;
    }
    
    return error * ctrl->dt;
}

/**
 * @brief 积分冻结法抗饱和
 * @param ctrl 抗饱和结构体指针
 * @param error 当前误差
 * @param raw_output 限幅前的原始输出
 * @return 积分增量
 *
 * @details 当输出饱和时完全冻结积分器
 */
static float integrator_freeze(AntiSaturation_t *ctrl, float error, float raw_output)
{
    float saturated = clampf(raw_output, ctrl->output_min, ctrl->output_max);
    
    if (saturated != raw_output) {
        /* 已饱和, 冻结积分 */
        return 0.0f;
    }
    
    return error * ctrl->dt;
}

/**
 * @brief 计算带抗饱和的积分值
 * @param ctrl 抗饱和结构体指针
 * @param error 当前误差
 * @param raw_output 限幅前的原始输出
 * @return 积分项值(已限幅)
 */
float AntiSat_CalculateIntegral(AntiSaturation_t *ctrl, float error, float raw_output)
{
    if (ctrl == NULL) return 0.0f;

    float integral_increment = 0.0f;
    
    /* 根据选择的方法计算积分增量 */
    switch (ctrl->method) {
        case ANTI_SAT_BACK_CALCULATION:
            integral_increment = back_calculation(ctrl, error, raw_output);
            break;
            
        case ANTI_SAT_CONDITIONAL_INTEGRATION:
            integral_increment = conditional_integration(ctrl, error, raw_output);
            break;
            
        case ANTI_SAT_TRACKING:
            integral_increment = tracking_anti_windup(ctrl, error, raw_output);
            break;
            
        case ANTI_SAT_CLAMPING:
            integral_increment = clamping_anti_windup(ctrl, error, raw_output);
            break;
            
        case ANTI_SAT_INTEGRATOR_FREEZE:
            integral_increment = integrator_freeze(ctrl, error, raw_output);
            break;
            
        default:
            integral_increment = error * ctrl->dt;
            break;
    }
    
    /* 累加积分项 */
    ctrl->integral += integral_increment;
    
    /* 积分项自身限幅 */
    float integral_max = (ctrl->output_max - ctrl->output_min) * 0.5f;
    ctrl->integral = clampf(ctrl->integral, -integral_max, integral_max);
    
    return ctrl->integral;
}

/**
 * @brief 应用输出限幅并更新饱和状态
 * @param ctrl 抗饱和结构体指针
 * @param raw_output 限幅前的原始输出
 * @return 限幅后的输出
 */
float AntiSat_ApplyLimit(AntiSaturation_t *ctrl, float raw_output)
{
    if (ctrl == NULL) return raw_output;

    float saturated = clampf(raw_output, ctrl->output_min, ctrl->output_max);
    
    /* 更新饱和状态标志 */
    ctrl->is_saturated = (saturated != raw_output) ? 1 : 0;
    
    /* 保存历史状态 */
    ctrl->prev_unsaturated_output = raw_output;
    ctrl->prev_output = saturated;
    
    return saturated;
}

/**
 * @brief 重置抗饱和控制器状态
 * @param ctrl 抗饱和结构体指针
 */
void AntiSat_Reset(AntiSaturation_t *ctrl)
{
    if (ctrl == NULL) return;
    ctrl->integral = 0.0f;
    ctrl->prev_error = 0.0f;
    ctrl->prev_output = 0.0f;
    ctrl->prev_unsaturated_output = 0.0f;
    ctrl->is_saturated = 0;
}

/**
 * @brief 查询当前是否处于饱和状态
 * @param ctrl 抗饱和结构体指针
 * @return 1=饱和, 0=未饱和
 */
uint8_t AntiSat_IsSaturated(const AntiSaturation_t *ctrl)
{
    if (ctrl == NULL) return 0;
    return ctrl->is_saturated;
}

/**
 * @brief 获取饱和裕度(饱和输出与原始输出的差)
 * @param ctrl 抗饱和结构体指针
 * @return 饱和裕度值
 */
float AntiSat_GetSaturationMargin(const AntiSaturation_t *ctrl)
{
    if (ctrl == NULL) return 0.0f;
    return ctrl->prev_output - ctrl->prev_unsaturated_output;
}
