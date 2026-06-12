#ifndef PID_FUSION_H
#define PID_FUSION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID融合控制器结构体
 * 
 * 集成PID控制、前馈补偿、扰动观测器(DOB)和自适应参数调整
 */
typedef struct {
    // PID参数
    float Kp;                    // 比例系数
    float Ki;                    // 积分系数
    float Kd;                    // 微分系数
    
    // 前馈参数
    float Kff;                   // 前馈增益
    float Kff_d;                 // 微分前馈增益
    
    // DOB参数
    float dob_gain;              // DOB增益
    float dob_cutoff;            // DOB截止频率
    float dob_estimate;          // DOB扰动估计值
    
    // 自适应参数
    float adapt_rate;            // 自适应速率
    float Kp_min, Kp_max;        // Kp自适应范围
    float Ki_min, Ki_max;        // Ki自适应范围
    float Kd_min, Kd_max;        // Kd自适应范围
    
    // 内部状态
    float integral;              // 积分项
    float prev_error;            // 上次误差
    float prev_measurement;      // 上次测量值
    float prev_derivative;       // 上次微分值
    float prev_dob_estimate;     // 上次DOB估计值
    
    // 输出限幅
    float output_min;            // 输出下限
    float output_max;            // 输出上限
    
    // 采样时间
    float dt;                    // 采样周期(秒)
    
    // 滤波参数
    float derivative_filter_coeff; // 微分滤波系数
    float filtered_derivative;     // 滤波后的微分值
} PIDFusion_t;

/**
 * @brief 初始化PID融合控制器
 * @param ctrl 控制器结构体指针
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param dt 采样周期(秒)
 */
void PIDFusion_Init(PIDFusion_t *ctrl, float Kp, float Ki, float Kd, float dt);

/**
 * @brief 配置前馈参数
 * @param ctrl 控制器结构体指针
 * @param Kff 前馈增益
 * @param Kff_d 微分前馈增益
 */
void PIDFusion_SetFeedforward(PIDFusion_t *ctrl, float Kff, float Kff_d);

/**
 * @brief 配置DOB参数
 * @param ctrl 控制器结构体指针
 * @param gain DOB增益
 * @param cutoff 截止频率(Hz)
 */
void PIDFusion_SetDOB(PIDFusion_t *ctrl, float gain, float cutoff);

/**
 * @brief 配置自适应参数
 * @param ctrl 控制器结构体指针
 * @param rate 自适应速率
 * @param Kp_min, Kp_max Kp范围
 * @param Ki_min, Ki_max Ki范围
 * @param Kd_min, Kd_max Kd范围
 */
void PIDFusion_SetAdaptive(PIDFusion_t *ctrl, float rate, 
                           float Kp_min, float Kp_max,
                           float Ki_min, float Ki_max,
                           float Kd_min, float Kd_max);

/**
 * @brief 设置输出限幅
 * @param ctrl 控制器结构体指针
 * @param min 输出下限
 * @param max 输出上限
 */
void PIDFusion_SetOutputLimit(PIDFusion_t *ctrl, float min, float max);

/**
 * @brief 设置微分滤波系数
 * @param ctrl 控制器结构体指针
 * @param coeff 滤波系数(0-1, 0为无滤波，1为强滤波)
 */
void PIDFusion_SetDerivativeFilter(PIDFusion_t *ctrl, float coeff);

/**
 * @brief 执行PID融合控制计算
 * @param ctrl 控制器结构体指针
 * @param setpoint 目标值
 * @param measurement 测量值
 * @param feedforward 前馈值(可为0)
 * @param feedforward_d 微分前馈值(可为0)
 * @return 控制输出
 */
float PIDFusion_Calculate(PIDFusion_t *ctrl, float setpoint, float measurement,
                         float feedforward, float feedforward_d);

/**
 * @brief 重置控制器状态
 * @param ctrl 控制器结构体指针
 */
void PIDFusion_Reset(PIDFusion_t *ctrl);

/**
 * @brief 获取当前扰动估计值
 * @param ctrl 控制器结构体指针
 * @return DOB估计的扰动值
 */
float PIDFusion_GetDisturbanceEstimate(const PIDFusion_t *ctrl);

/**
 * @brief 获取当前PID参数(用于监控)
 * @param ctrl 控制器结构体指针
 * @param Kp 比例系数输出
 * @param Ki 积分系数输出
 * @param Kd 微分系数输出
 */
void PIDFusion_GetCurrentParams(const PIDFusion_t *ctrl, float *Kp, float *Ki, float *Kd);

#ifdef __cplusplus
}
#endif

#endif /* PID_FUSION_H */