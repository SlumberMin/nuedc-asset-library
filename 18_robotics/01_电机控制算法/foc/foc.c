/*
 * FOC（磁场定向控制）算法实现
 * 来源：RoboMaster 优秀开源项目
 * 适配平台：MSPM0G3507
 */

#include "foc.h"
#include <math.h>

// 数学常数
#define PI 3.14159265358979f
#define SQRT3 1.73205080756888f
#define SQRT3_OVER_2 0.86602540378444f
#define ONE_OVER_SQRT3 0.57735026918963f
#define TWO_OVER_SQRT3 1.15470053837925f

/**
 * @brief 初始化FOC控制器
 * @param hfoc FOC句柄
 */
void FOC_Init(FOC_HandleTypeDef *hfoc)
{
    // 清零所有变量
    memset(hfoc, 0, sizeof(FOC_HandleTypeDef));
    
    // 初始化PI控制器参数（需要根据实际电机调整）
    hfoc->pi_d.kp = 0.5f;
    hfoc->pi_d.ki = 0.01f;
    hfoc->pi_d.output_limit = 12.0f;   // 电压限制
    
    hfoc->pi_q.kp = 0.5f;
    hfoc->pi_q.ki = 0.01f;
    hfoc->pi_q.output_limit = 12.0f;
    
    hfoc->pi_speed.kp = 0.1f;
    hfoc->pi_speed.ki = 0.001f;
    hfoc->pi_speed.output_limit = 5.0f; // 电流限制
}

/**
 * @brief Clark变换：三相静止坐标系 → 两相静止坐标系
 * @param i_a A相电流
 * @param i_b B相电流
 * @param i_c C相电流
 * @param i_alpha α轴输出
 * @param i_beta β轴输出
 */
void FOC_ClarkeTransform(float i_a, float i_b, float i_c, float *i_alpha, float *i_beta)
{
    // Clark变换公式（等幅值变换）
    // i_alpha = (2/3) * (i_a - 0.5*i_b - 0.5*i_c)
    // i_beta = (2/3) * (sqrt(3)/2 * i_b - sqrt(3)/2 * i_c)
    
    *i_alpha = (2.0f/3.0f) * (i_a - 0.5f*i_b - 0.5f*i_c);
    *i_beta = (2.0f/3.0f) * (SQRT3_OVER_2*i_b - SQRT3_OVER_2*i_c);
}

/**
 * @brief Park变换：两相静止坐标系 → 两相旋转坐标系
 * @param i_alpha α轴电流
 * @param i_beta β轴电流
 * @param angle 电角度 (rad)
 * @param i_d d轴输出
 * @param i_q q轴输出
 */
void FOC_ParkTransform(float i_alpha, float i_beta, float angle, float *i_d, float *i_q)
{
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
    
    // Park变换公式
    *i_d = i_alpha * cos_angle + i_beta * sin_angle;
    *i_q = -i_alpha * sin_angle + i_beta * cos_angle;
}

/**
 * @brief 反Park变换：两相旋转坐标系 → 两相静止坐标系
 * @param v_d d轴电压
 * @param v_q q轴电压
 * @param angle 电角度 (rad)
 * @param v_alpha α轴输出
 * @param v_beta β轴输出
 */
void FOC_InverseParkTransform(float v_d, float v_q, float angle, float *v_alpha, float *v_beta)
{
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
    
    // 反Park变换公式
    *v_alpha = v_d * cos_angle - v_q * sin_angle;
    *v_beta = v_d * sin_angle + v_q * cos_angle;
}

/**
 * @brief PI控制器
 * @param ref 目标值
 * @param meas 测量值
 * @param kp 比例系数
 * @param ki 积分系数
 * @param limit 输出限幅
 * @param integral 积分项指针
 * @return 控制器输出
 */
float FOC_PIController(float ref, float meas, float kp, float ki, float limit, float *integral)
{
    float error = ref - meas;
    float p_term = kp * error;
    
    // 积分项累加
    *integral += ki * error;
    
    // 积分限幅（抗积分饱和）
    if (*integral > limit) *integral = limit;
    if (*integral < -limit) *integral = -limit;
    
    float output = p_term + *integral;
    
    // 输出限幅
    if (output > limit) output = limit;
    if (output < -limit) output = -limit;
    
    return output;
}

/**
 * @brief SVPWM（空间矢量脉宽调制）
 * @param v_alpha α轴电压
 * @param v_beta β轴电压
 * @param v_bus 母线电压
 * @param duty_a A相占空比输出
 * @param duty_b B相占空比输出
 * @param duty_c C相占空比输出
 */
void FOC_SVPWM(float v_alpha, float v_beta, float v_bus, float *duty_a, float *duty_b, float *duty_c)
{
    // 限制电压矢量幅值
    float v_max = v_bus * ONE_OVER_SQRT3;
    float v_mag = sqrtf(v_alpha*v_alpha + v_beta*v_beta);
    if (v_mag > v_max) {
        v_alpha = v_alpha * v_max / v_mag;
        v_beta = v_beta * v_max / v_mag;
    }
    
    // 计算三相电压
    float v_a = v_alpha;
    float v_b = -0.5f * v_alpha + SQRT3_OVER_2 * v_beta;
    float v_c = -0.5f * v_alpha - SQRT3_OVER_2 * v_beta;
    
    // 计算占空比
    *duty_a = (v_a / v_bus + 0.5f);
    *duty_b = (v_b / v_bus + 0.5f);
    *duty_c = (v_c / v_bus + 0.5f);
    
    // 占空比限幅
    if (*duty_a > MAX_DUTY_CYCLE) *duty_a = MAX_DUTY_CYCLE;
    if (*duty_a < 0.0f) *duty_a = 0.0f;
    if (*duty_b > MAX_DUTY_CYCLE) *duty_b = MAX_DUTY_CYCLE;
    if (*duty_b < 0.0f) *duty_b = 0.0f;
    if (*duty_c > MAX_DUTY_CYCLE) *duty_c = MAX_DUTY_CYCLE;
    if (*duty_c < 0.0f) *duty_c = 0.0f;
}

/**
 * @brief FOC主控制循环（应在PWM中断中调用）
 * @param hfoc FOC句柄
 * @param i_a A相电流采样值
 * @param i_b B相电流采样值
 * @param i_c C相电流采样值
 * @param angle 编码器角度 (rad)
 */
void FOC_Update(FOC_HandleTypeDef *hfoc, float i_a, float i_b, float i_c, float angle)
{
    // 1. 存储原始数据
    hfoc->i_a = i_a;
    hfoc->i_b = i_b;
    hfoc->i_c = i_c;
    hfoc->angle = angle;
    
    // 2. Clark变换
    FOC_ClarkeTransform(i_a, i_b, i_c, &hfoc->i_alpha, &hfoc->i_beta);
    
    // 3. Park变换
    FOC_ParkTransform(hfoc->i_alpha, hfoc->i_beta, angle, &hfoc->i_d, &hfoc->i_q);
    
    // 4. 速度环PI控制（外环）
    float speed_error = hfoc->speed_ref - hfoc->speed;
    hfoc->i_q_ref = FOC_PIController(hfoc->speed_ref, hfoc->speed, 
                                      hfoc->pi_speed.kp, hfoc->pi_speed.ki,
                                      hfoc->pi_speed.output_limit, &hfoc->pi_speed.integral);
    
    // 5. d轴电流环PI控制（内环）
    hfoc->v_d = FOC_PIController(hfoc->i_d_ref, hfoc->i_d,
                                  hfoc->pi_d.kp, hfoc->pi_d.ki,
                                  hfoc->pi_d.output_limit, &hfoc->pi_d.integral);
    
    // 6. q轴电流环PI控制（内环）
    hfoc->v_q = FOC_PIController(hfoc->i_q_ref, hfoc->i_q,
                                  hfoc->pi_q.kp, hfoc->pi_q.ki,
                                  hfoc->pi_q.output_limit, &hfoc->pi_q.integral);
    
    // 7. 反Park变换
    FOC_InverseParkTransform(hfoc->v_d, hfoc->v_q, angle, &hfoc->v_alpha, &hfoc->v_beta);
    
    // 8. SVPWM生成
    FOC_SVPWM(hfoc->v_alpha, hfoc->v_beta, 12.0f, // 假设12V母线电压
               &hfoc->duty_a, &hfoc->duty_b, &hfoc->duty_c);
}
