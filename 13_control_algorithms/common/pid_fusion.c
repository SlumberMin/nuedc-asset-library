#include "pid_fusion.h"
#include <math.h>

/* 内部辅助函数：限幅 */
static float clampf(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/* 内部辅助函数：一阶低通滤波 */
static float low_pass_filter(float current, float new_value, float alpha)
{
    return alpha * current + (1.0f - alpha) * new_value;
}

void PIDFusion_Init(PIDFusion_t *ctrl, float Kp, float Ki, float Kd, float dt)
{
    ctrl->Kp = Kp;
    ctrl->Ki = Ki;
    ctrl->Kd = Kd;
    ctrl->dt = dt;
    
    ctrl->Kff = 0.0f;
    ctrl->Kff_d = 0.0f;
    
    ctrl->dob_gain = 0.0f;
    ctrl->dob_cutoff = 10.0f;
    ctrl->dob_estimate = 0.0f;
    
    ctrl->adapt_rate = 0.0f;
    ctrl->Kp_min = 0.0f;
    ctrl->Kp_max = 100.0f;
    ctrl->Ki_min = 0.0f;
    ctrl->Ki_max = 100.0f;
    ctrl->Kd_min = 0.0f;
    ctrl->Kd_max = 100.0f;
    
    ctrl->integral = 0.0f;
    ctrl->prev_error = 0.0f;
    ctrl->prev_measurement = 0.0f;
    ctrl->prev_derivative = 0.0f;
    ctrl->prev_dob_estimate = 0.0f;
    
    ctrl->output_min = -1000.0f;
    ctrl->output_max = 1000.0f;
    
    ctrl->derivative_filter_coeff = 0.0f;
    ctrl->filtered_derivative = 0.0f;
}

void PIDFusion_SetFeedforward(PIDFusion_t *ctrl, float Kff, float Kff_d)
{
    ctrl->Kff = Kff;
    ctrl->Kff_d = Kff_d;
}

void PIDFusion_SetDOB(PIDFusion_t *ctrl, float gain, float cutoff)
{
    ctrl->dob_gain = gain;
    ctrl->dob_cutoff = cutoff;
}

void PIDFusion_SetAdaptive(PIDFusion_t *ctrl, float rate, 
                           float Kp_min, float Kp_max,
                           float Ki_min, float Ki_max,
                           float Kd_min, float Kd_max)
{
    ctrl->adapt_rate = rate;
    ctrl->Kp_min = Kp_min;
    ctrl->Kp_max = Kp_max;
    ctrl->Ki_min = Ki_min;
    ctrl->Ki_max = Ki_max;
    ctrl->Kd_min = Kd_min;
    ctrl->Kd_max = Kd_max;
}

void PIDFusion_SetOutputLimit(PIDFusion_t *ctrl, float min, float max)
{
    ctrl->output_min = min;
    ctrl->output_max = max;
}

void PIDFusion_SetDerivativeFilter(PIDFusion_t *ctrl, float coeff)
{
    ctrl->derivative_filter_coeff = clampf(coeff, 0.0f, 0.99f);
}

/* 自适应参数调整 */
static void update_adaptive_params(PIDFusion_t *ctrl, float error, float error_dot)
{
    if (ctrl->adapt_rate <= 0.0f) return;
    
    float abs_error = fabsf(error);
    float abs_error_dot = fabsf(error_dot);
    
    /* 根据误差大小调整Kp：误差大时增大Kp */
    float Kp_adapt = ctrl->Kp + ctrl->adapt_rate * abs_error;
    ctrl->Kp = clampf(Kp_adapt, ctrl->Kp_min, ctrl->Kp_max);
    
    /* 根据误差变化率调整Kd：变化快时增大Kd */
    float Kd_adapt = ctrl->Kd + ctrl->adapt_rate * abs_error_dot;
    ctrl->Kd = clampf(Kd_adapt, ctrl->Kd_min, ctrl->Kd_max);
    
    /* 根据稳态误差调整Ki：稳态误差大时增大Ki */
    float steady_state_error = fabsf(ctrl->integral) * ctrl->dt;
    float Ki_adapt = ctrl->Ki + ctrl->adapt_rate * steady_state_error;
    ctrl->Ki = clampf(Ki_adapt, ctrl->Ki_min, ctrl->Ki_max);
}

/* DOB扰动估计更新 */
static void update_dob_estimate(PIDFusion_t *ctrl, float measurement, float control_output)
{
    if (ctrl->dob_gain <= 0.0f) return;
    
    /* 简化的DOB模型：假设系统为一阶惯性环节
     * 扰动估计 = 测量值变化 - 控制作用产生的变化 */
    float measurement_rate = (measurement - ctrl->prev_measurement) / ctrl->dt;
    float expected_rate = control_output; // 简化：假设增益为1
    
    float raw_estimate = measurement_rate - expected_rate;
    
    /* 一阶低通滤波 */
    float alpha = expf(-2.0f * M_PI * ctrl->dob_cutoff * ctrl->dt);
    ctrl->dob_estimate = low_pass_filter(ctrl->prev_dob_estimate, raw_estimate, alpha);
    
    ctrl->prev_dob_estimate = ctrl->dob_estimate;
}

float PIDFusion_Calculate(PIDFusion_t *ctrl, float setpoint, float measurement,
                         float feedforward, float feedforward_d)
{
    float error = setpoint - measurement;
    
    /* 计算微分项 */
    float derivative = (measurement - ctrl->prev_measurement) / ctrl->dt;
    
    /* 微分滤波 */
    if (ctrl->derivative_filter_coeff > 0.0f) {
        ctrl->filtered_derivative = low_pass_filter(
            ctrl->filtered_derivative, derivative, ctrl->derivative_filter_coeff);
        derivative = ctrl->filtered_derivative;
    }
    
    /* 自适应参数更新 */
    float error_dot = (error - ctrl->prev_error) / ctrl->dt;
    update_adaptive_params(ctrl, error, error_dot);
    
    /* PID分量计算 */
    float P_term = ctrl->Kp * error;
    float I_term = ctrl->Ki * ctrl->integral;
    float D_term = -ctrl->Kd * derivative; // 注意取负号，因为微分的是measurement
    
    /* 前馈分量 */
    float FF_term = ctrl->Kff * feedforward;
    float FF_d_term = ctrl->Kff_d * feedforward_d;
    
    /* 总输出（不含DOB） */
    float pid_output = P_term + I_term + D_term + FF_term + FF_d_term;
    
    /* 更新DOB */
    update_dob_estimate(ctrl, measurement, pid_output);
    
    /* DOB补偿 */
    float dob_compensation = ctrl->dob_gain * ctrl->dob_estimate;
    
    /* 最终输出 */
    float output = pid_output - dob_compensation;
    
    /* 输出限幅 */
    output = clampf(output, ctrl->output_min, ctrl->output_max);
    
    /* 积分项更新（含抗饱和） */
    if ((output > ctrl->output_min && output < ctrl->output_max) ||
        (error > 0 && output < ctrl->output_max) ||
        (error < 0 && output > ctrl->output_min)) {
        ctrl->integral += error * ctrl->dt;
    }
    
    /* 更新状态 */
    ctrl->prev_error = error;
    ctrl->prev_measurement = measurement;
    ctrl->prev_derivative = derivative;
    
    return output;
}

void PIDFusion_Reset(PIDFusion_t *ctrl)
{
    ctrl->integral = 0.0f;
    ctrl->prev_error = 0.0f;
    ctrl->prev_measurement = 0.0f;
    ctrl->prev_derivative = 0.0f;
    ctrl->prev_dob_estimate = 0.0f;
    ctrl->dob_estimate = 0.0f;
    ctrl->filtered_derivative = 0.0f;
}

float PIDFusion_GetDisturbanceEstimate(const PIDFusion_t *ctrl)
{
    return ctrl->dob_estimate;
}

void PIDFusion_GetCurrentParams(const PIDFusion_t *ctrl, float *Kp, float *Ki, float *Kd)
{
    if (Kp) *Kp = ctrl->Kp;
    if (Ki) *Ki = ctrl->Ki;
    if (Kd) *Kd = ctrl->Kd;
}