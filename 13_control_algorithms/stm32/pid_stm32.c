/**
 * @file pid_stm32.c
 * @brief STM32/GD32/CH32 移植版PID控制
 * 
 * 集成定时器中断、ADC采集、PWM输出
 * 适用于: 电机控制闭环、温度控制、平衡控制
 * 
 * 使用方法:
 *   1. 配置TIM中断(1kHz或10kHz)
 *   2. 在中断回调中调用 PID_ControlLoop()
 *   3. 或在主循环中轮询调用
 */

#include "../common/pid_full.h"
#include "../common/kalman.h"

/* ===== 根据实际平台修改以下头文件 ===== */
/*
 * STM32 HAL:
 * #include "stm32f1xx_hal.h"   // 或 f4xx, h7xx
 * 
 * GD32:
 * #include "gd32f10x.h"
 * 
 * CH32:
 * #include "ch32v10x.h"
 */

/* ===== 示例: 电机速度闭环控制 ===== */

/* 硬件句柄(需要用户配置) */
typedef struct {
    /* ADC句柄 */
    volatile uint32_t *adc_value;     /* ADC采样值指针 */
    
    /* PWM句柄 */
    volatile uint32_t *pwm_compare;   /* PWM占空比寄存器指针 */
    uint32_t pwm_period;              /* PWM周期值 */
    uint8_t pwm_channel;              /* PWM通道 */
    
    /* 编码器 */
    volatile int32_t *encoder_count;  /* 编码器计数 */
    
    /* 配置 */
    float sample_time;                /* 采样时间(s) */
} HW_Handle_t;

/* 控制器实例 */
static PID_t pid_speed;       /* 速度环PID */
static PID_t pid_position;    /* 位置环PID */
static Kalman_t kf_speed;     /* 速度卡尔曼滤波 */

/* 控制模式 */
typedef enum {
    CTRL_SPEED = 0,
    CTRL_POSITION,
    CTRL_CASCADE,  /* 串级: 位置环(外) + 速度环(内) */
} ControlMode_t;

static ControlMode_t ctrl_mode = CTRL_SPEED;
static float target_speed = 0;
static float target_position = 0;

/**
 * @brief 初始化控制系统
 * @param mode 控制模式
 */
void PID_Motor_Init(ControlMode_t mode)
{
    ctrl_mode = mode;
    
    /* 速度环PID初始化 */
    PID_Init(&pid_speed, 10.0f, 2.0f, 0.5f);
    PID_SetMode(&pid_speed, PID_POSITION, PID_INTEGRAL_SEP);
    PID_SetOutputLimit(&pid_speed, -100, 100);
    PID_SetIntegralLimit(&pid_speed, -50, 50);
    PID_SetIntegralSeparation(&pid_speed, 30);
    
    /* 位置环PID初始化 */
    PID_Init(&pid_position, 5.0f, 0.1f, 2.0f);
    PID_SetMode(&pid_position, PID_POSITION, PID_NORMAL);
    PID_SetOutputLimit(&pid_position, -200, 200);
    
    /* 速度卡尔曼滤波初始化 */
    Kalman_Init(&kf_speed, 0.001f, 0.1f, 1.0f);
}

/**
 * @brief 设置目标值
 */
void PID_Motor_SetTarget(float speed, float position)
{
    target_speed = speed;
    target_position = position;
}

/**
 * @brief 获取编码器速度(RPM)
 * @param count 编码器脉冲增量
 * @param ppr 编码器每转脉冲数
 * @param dt 采样时间(s)
 * @return 转速(RPM)
 */
static float Encoder_GetRPM(int32_t count, uint16_t ppr, float dt)
{
    /* RPM = (count / ppr) * (60 / dt) */
    if (dt <= 0.0f) dt = 0.001f;  /* V2审计: 防除零 */
    return ((float)count / (float)ppr) * (60.0f / dt);
}

/**
 * @brief 设置PWM输出(-100% ~ +100%)
 * @param duty 占空比百分比
 * @param hw 硬件句柄
 */
static void PWM_SetDuty(float duty, const HW_Handle_t *hw)
{
    /* 限幅 */
    if (duty > 100.0f) duty = 100.0f;
    if (duty < -100.0f) duty = -100.0f;
    
    /* 转换为比较值 */
    uint32_t compare = (uint32_t)((duty / 100.0f) * hw->pwm_period);
    
    /* 方向控制(需要根据实际硬件修改) */
    /*
     * if (duty >= 0) {
     *     GPIO_SetPin(MOTOR_DIR_PIN, GPIO_PIN_SET);
     * } else {
     *     GPIO_SetPin(MOTOR_DIR_PIN, GPIO_PIN_RESET);
     *     compare = hw->pwm_period - compare;
     * }
     */
    
    *hw->pwm_compare = compare;
}

/**
 * @brief 控制主循环 - 在定时器中断或主循环中调用
 * @param encoder_delta 编码器脉冲增量
 * @param hw 硬件句柄
 * @return PWM输出百分比
 */
float PID_Motor_ControlLoop(int32_t encoder_delta, const HW_Handle_t *hw)
{
    float speed_rpm, filtered_speed, output;
    
    /* 计算当前速度 */
    speed_rpm = Encoder_GetRPM(encoder_delta, 1024, hw->sample_time);
    
    /* 卡尔曼滤波去噪 */
    filtered_speed = Kalman_Update(&kf_speed, speed_rpm);
    
    switch (ctrl_mode) {
    case CTRL_SPEED:
        /* 速度单环 */
        PID_SetTarget(&pid_speed, target_speed);
        output = PID_Calculate(&pid_speed, filtered_speed);
        break;
        
    case CTRL_POSITION:
        /* 位置单环(需要外部提供位置) */
        /* output = PID_Calculate(&pid_position, current_position); */
        output = 0;
        break;
        
    case CTRL_CASCADE:
        /* 串级控制: 位置环输出作为速度环目标 */
        /* float speed_ref = PID_Calculate(&pid_position, current_position); */
        /* PID_SetTarget(&pid_speed, speed_ref); */
        /* output = PID_Calculate(&pid_speed, filtered_speed); */
        output = 0;
        break;
        
    default:
        output = 0;
        break;
    }
    
    /* 设置PWM输出 */
    PWM_SetDuty(output, hw);
    
    return output;
}

/**
 * @brief TIM中断回调示例
 * 
 * void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *     if (htim->Instance == TIM2) {
 *         int32_t delta = __HAL_TIM_GET_COUNTER(&htim_encoder) - last_count;
 *         last_count = __HAL_TIM_GET_COUNTER(&htim_encoder);
 *         PID_Motor_ControlLoop(delta, &hw);
 *     }
 * }
 */

/* ===== ADC温度控制示例 ===== */

typedef struct {
    PID_t pid;
    Kalman_t kf;
    float target_temp;
    volatile uint32_t *pwm_duty;
} TempController_t;

static TempController_t temp_ctrl;

/**
 * @brief 初始化温度控制器
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void TempController_Init(float kp, float ki, float kd)
{
    PID_Init(&temp_ctrl.pid, kp, ki, kd);
    PID_SetMode(&temp_ctrl.pid, PID_POSITION, PID_INTEGRAL_SEP);
    PID_SetOutputLimit(&temp_ctrl.pid, 0, 100);  /* 加热器只能正向输出 */
    PID_SetIntegralLimit(&temp_ctrl.pid, 0, 80);
    PID_SetIntegralSeparation(&temp_ctrl.pid, 20);
    
    Kalman_Init(&temp_ctrl.kf, 0.1f, 0.01f, 0.5f);
    
    temp_ctrl.target_temp = 50.0f;
}

/**
 * @brief 温度控制循环
 * @param adc_value ADC采样原始值
 * @return PWM占空比(0~100%)
 */
float TempController_Loop(uint16_t adc_value)
{
    /* ADC转温度(根据NTC或热电偶特性曲线修改) */
    float voltage = (float)adc_value / 4096.0f * 3.3f;
    float temperature = voltage * 100.0f;  /* 示例: 10mV/°C */
    
    /* 卡尔曼滤波 */
    temperature = Kalman_Update(&temp_ctrl.kf, temperature);
    
    /* PID计算 */
    PID_SetTarget(&temp_ctrl.pid, temp_ctrl.target_temp);
    float output = PID_Calculate(&temp_ctrl.pid, temperature);
    
    /* 输出为加热功率百分比 */
    return output;
}
