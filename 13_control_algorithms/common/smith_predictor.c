/**
 * @file smith_predictor.c
 * @brief Smith预估器实现
 * 
 * Smith预估器结构：
 * 
 *          ┌─────────┐
 *   r(k)──►│  PID    │──►u(k)──────────────────────────► 被控对象(含滞后) ──► y(k)
 *          └─────────┘      │                                      │
 *               ▲           │                                      │
 *               │           ▼                                      │
 *               │    ┌──────────────┐                              │
 *               │    │ 无滞后模型    │──► ym(k)                     │
 *               │    │ Kp/(Ts+1)   │                              │
 *               │    └──────────────┘                              │
 *               │           │                                      │
 *               │           ▼                                      │
 *               │    ┌──────────────┐                              │
 *               │    │ 延迟模型      │──► ym_delayed(k)             │
 *               │    │ e^(-Ls)     │                              │
 *               │    └──────────────┘                              │
 *               │           │                                      │
 *               │     ym(k) - ym_delayed(k) = 补偿量               │
 *               │           │                                      │
 *               └─── y(k) + [ym(k) - ym_delayed(k)] ← 反馈修正     │
 *                                                                     │
 *   修正反馈 = y(k) + ym(k) - ym_delayed(k)
 *   近似于 = y_without_delay(k)  (消除了滞后影响)
 */

#include "smith_predictor.h"
#include <math.h>
#include <string.h>

#define CLAMP(val, min_v, max_v) \
    do { if ((val) < (min_v)) (val) = (min_v); \
         if ((val) > (max_v)) (val) = (max_v); } while(0)

void Smith_Init(SmithPredictor_t *pred, float dt)
{
    memset(pred, 0, sizeof(SmithPredictor_t));
    pred->dt = (dt > 0.0f) ? dt : 0.001f;  /* [审计修复] 防止除零 */

    /* 默认模型参数（需根据实际系统设置） */
    pred->model.Kp = 1.0f;
    pred->model.T = 1.0f;
    pred->model.L = 0.5f;

    /* 默认PID */
    pred->pid_Kp = 1.0f;
    pred->pid_Ki = 0.5f;
    pred->pid_Kd = 0.1f;

    pred->out_min = -1000.0f;
    pred->out_max =  1000.0f;
    pred->integral_max = 500.0f;

    /* 使用静态缓冲区 */
    pred->delay_buffer = pred->_static_buffer;
    pred->delay_size = 512;
    pred->delay_index = 0;

    Smith_Reset(pred);

    /* 重新计算模型系数（在Reset之后） */
    Smith_SetModel(pred, pred->model.Kp, pred->model.T, pred->model.L);
}

void Smith_SetModel(SmithPredictor_t *pred, float Kp, float T, float L)
{
    pred->model.Kp = Kp;
    pred->model.T = T;
    pred->model.L = L;

    /* 一阶惯性环节离散化（双线性变换 / 后向差分） */
    /* G(s) = Kp / (Ts+1) => G(z) = Kp*dt / (T + dt) * 1/(1 - T/(T+dt)*z^-1) */
    float dt = pred->dt;
    pred->model_coeff_b = Kp * dt / (T + dt);   /* 输入系数 */
    pred->model_coeff_a = T / (T + dt);          /* 状态反馈系数 */

    /* 计算延迟缓冲区大小 */
    if (dt <= 0.0f) dt = 0.001f;  /* 防止除零 */
    uint16_t delay_samples = (uint16_t)(L / dt) + 1;
    if (delay_samples > 512) delay_samples = 512;
    pred->delay_size = delay_samples;
    pred->delay_index = 0;
}

void Smith_SetPID(SmithPredictor_t *pred, float Kp, float Ki, float Kd)
{
    pred->pid_Kp = Kp;
    pred->pid_Ki = Ki;
    pred->pid_Kd = Kd;
}

float Smith_Compute(SmithPredictor_t *pred, float target, float measurement)
{
    /* ---- Step 1: 更新无滞后模型 ---- */
    /* 输入上次控制量 u，计算无滞后模型输出 */
    /* 这里用measurement的增量作为驱动（自回归模型） */
    pred->model_state = pred->model_coeff_a * pred->model_state
                      + pred->model_coeff_b * measurement;
    float ym = pred->model_state;  /* 无滞后模型输出 */

    /* ---- Step 2: 更新延迟模型（环形缓冲区） ---- */
    /* 将模型输出写入延迟缓冲区 */
    pred->delay_buffer[pred->delay_index] = ym;
    /* 读取 delay_size 步之前的值 */
    uint16_t read_index = (pred->delay_index + 1) % pred->delay_size;
    float ym_delayed = pred->delay_buffer[read_index];
    pred->delay_index = read_index;

    /* ---- Step 3: 计算预估补偿 ---- */
    pred->predictor_output = ym - ym_delayed;

    /* ---- Step 4: 修正反馈信号 ---- */
    /* compensated_feedback = measurement + (ym - ym_delayed) */
    /* 当模型完美时，measurement = ym_delayed，反馈 = ym（无滞后输出） */
    pred->compensated_feedback = measurement + pred->predictor_output;

    /* ---- Step 5: PID控制器 ---- */
    float error = target - pred->compensated_feedback;

    pred->pid_integral += error * pred->dt;
    CLAMP(pred->pid_integral, -pred->integral_max, pred->integral_max);

    float derivative = (error - pred->pid_error_last) / pred->dt;

    float output = pred->pid_Kp * error
                 + pred->pid_Ki * pred->pid_integral
                 + pred->pid_Kd * derivative;

    CLAMP(output, pred->out_min, pred->out_max);

    pred->pid_error_last = error;

    return output;
}

void Smith_Reset(SmithPredictor_t *pred)
{
    pred->pid_error = 0.0f;
    pred->pid_error_last = 0.0f;
    pred->pid_integral = 0.0f;
    pred->model_state = 0.0f;
    pred->delay_output = 0.0f;
    pred->predictor_output = 0.0f;
    pred->compensated_feedback = 0.0f;
    pred->delay_index = 0;

    /* 清零延迟缓冲区 */
    memset(pred->delay_buffer, 0, pred->delay_size * sizeof(float));
}

void Smith_GetDebug(SmithPredictor_t *pred, float *model_out, float *delay_out, float *comp_fb)
{
    if (model_out) *model_out = pred->model_state;
    if (delay_out) *delay_out = pred->delay_output;
    if (comp_fb)   *comp_fb = pred->compensated_feedback;
}
