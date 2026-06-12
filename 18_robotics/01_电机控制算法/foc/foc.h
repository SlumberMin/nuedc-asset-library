/*
 * FOC（磁场定向控制）算法实现
 * 来源：RoboMaster 优秀开源项目
 * 适配平台：MSPM0G3507
 * 
 * 设计思路：
 * 1. Clark变换：三相静止坐标系 → 两相静止坐标系（α-β）
 * 2. Park变换：两相静止坐标系 → 两相旋转坐标系（d-q）
 * 3. PI控制器：分别控制d轴和q轴电流
 * 4. 反Park变换：旋转坐标系 → 静止坐标系
 * 5. SVPWM：空间矢量脉宽调制
 */

#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include <math.h>

// 电机参数
#define MOTOR_POLE_PAIRS    7       // 极对数
#define PWM_FREQUENCY       20000   // PWM频率 (Hz)
#define MAX_DUTY_CYCLE      0.95f   // 最大占空比

// FOC状态结构体
typedef struct {
    // 传感器数据
    float angle;            // 电角度 (rad)
    float speed;            // 机械转速 (rpm)
    
    // 三相电流
    float i_a, i_b, i_c;
    
    // Clark变换结果 (α-β坐标系)
    float i_alpha, i_beta;
    
    // Park变换结果 (d-q坐标系)
    float i_d, i_q;
    
    // 目标值
    float i_d_ref;          // d轴电流目标 (通常为0)
    float i_q_ref;          // q轴电流目标 (控制转矩)
    float speed_ref;        // 速度目标
    
    // PI控制器输出
    float v_d, v_q;         // d-q轴电压
    
    // 反Park变换结果
    float v_alpha, v_beta;
    
    // SVPWM输出
    float duty_a, duty_b, duty_c;
    
    // PI控制器参数
    struct {
        float kp, ki;
        float integral;
        float output_limit;
    } pi_d, pi_q, pi_speed;
    
} FOC_HandleTypeDef;

// 函数声明
void FOC_Init(FOC_HandleTypeDef *hfoc);
void FOC_ClarkeTransform(float i_a, float i_b, float i_c, float *i_alpha, float *i_beta);
void FOC_ParkTransform(float i_alpha, float i_beta, float angle, float i_d, float i_q);
void FOC_InverseParkTransform(float v_d, float v_q, float angle, float *v_alpha, float *v_beta);
float FOC_PIController(float ref, float meas, float kp, float ki, float limit, float *integral);
void FOC_SVPWM(float v_alpha, float v_beta, float v_bus, float *duty_a, float *duty_b, float *duty_c);
void FOC_Update(FOC_HandleTypeDef *hfoc, float i_a, float i_b, float i_c, float angle);

#endif // FOC_H
