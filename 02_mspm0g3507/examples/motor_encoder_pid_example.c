/**
 * @file    motor_encoder_pid_example.c
 * @brief   电机+编码器+PID闭环控制示例 — MSPM0G3507
 * @note    演示如何使用TB6612驱动N20电机，读取霍尔编码器反馈，实现PID速度闭环控制
 *          适用于电赛智能小车、机器人等项目
 *
 * 硬件连接:
 *   TB6612电机驱动:
 *     MSPM0 PA0 → AIN1   PA1 → AIN2   PA12(PWM) → PWMA (左电机)
 *     MSPM0 PA2 → BIN1   PA3 → BIN2   PA13(PWM) → PWMB (右电机)
 *   N20电机编码器:
 *     MSPM0 PB0 → 左轮A相   PB1 → 左轮B相
 *     MSPM0 PB2 → 右轮A相   PB3 → 右轮B相
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/motor_mspm0.h"
#include "drivers/encoder_gpio_mspm0.h"
#include "algorithm/pid_mspm0.h"

/* ── 配置参数 ────────────────────────────────────────────── */
#define MOTOR_PWM_PERIOD    1000    /* PWM周期 (0~1000对应0~100%) */
#define ENCODER_SAMPLE_MS   10      /* 编码器采样周期 (ms) */
#define TARGET_SPEED        500     /* 目标速度 (脉冲/采样周期) */
#define MAX_PWM             800     /* 最大PWM输出 */

/* ── 全局变量 ────────────────────────────────────────────── */
static PID_Controller pid_left;     /* 左轮PID控制器 */
static PID_Controller pid_right;    /* 右轮PID控制器 */
static volatile uint32_t sys_tick = 0;  /* 系统时钟计数 */

/* ── 定时器中断处理 (10ms周期) ───────────────────────────── */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        sys_tick++;
        
        /* 每10ms更新编码器速度 */
        if (sys_tick % (ENCODER_SAMPLE_MS) == 0) {
            EncoderGpio_Update();
        }
    }
}

/* ── 电机初始化 ──────────────────────────────────────────── */
static void MotorSystem_Init(void)
{
    /* 电机配置 (根据实际接线修改) */
    MotorConfig motor_cfg[MOTOR_MAX] = {
        [MOTOR_A] = {
            .port_in1 = GPIOA, .pin_in1 = DL_GPIO_PIN_0,
            .port_in2 = GPIOA, .pin_in2 = DL_GPIO_PIN_1,
            .pwm_timer = TIMA0, .pwm_channel = DL_TIMER_CC_0_INDEX,
            .pwm_period = MOTOR_PWM_PERIOD
        },
        [MOTOR_B] = {
            .port_in1 = GPIOA, .pin_in1 = DL_GPIO_PIN_2,
            .port_in2 = GPIOA, .pin_in2 = DL_GPIO_PIN_3,
            .pwm_timer = TIMA0, .pwm_channel = DL_TIMER_CC_3_INDEX,
            .pwm_period = MOTOR_PWM_PERIOD
        }
    };
    
    /* 编码器配置 (根据实际接线修改) */
    EncoderGpioConfig enc_cfg[ENCODER_GPIO_MAX] = {
        [ENCODER_GPIO_LEFT] = {
            .port = GPIOB, .pin_a = DL_GPIO_PIN_0, .pin_b = DL_GPIO_PIN_1,
            .inverted = 0
        },
        [ENCODER_GPIO_RIGHT] = {
            .port = GPIOB, .pin_a = DL_GPIO_PIN_2, .pin_b = DL_GPIO_PIN_3,
            .inverted = 1  /* 右轮编码器方向可能相反 */
        }
    };
    
    /* 初始化电机驱动 */
    Motor_Init(motor_cfg);
    
    /* 初始化编码器 */
    EncoderGpio_Init(enc_cfg);
    
    /* 初始化PID控制器 */
    PID_Param pid_param = {
        .kp = 2.0f,
        .ki = 0.5f,
        .kd = 0.1f,
        .output_min = -MAX_PWM,
        .output_max = MAX_PWM,
        .integral_max = 500.0f,
        .dead_zone = 5
    };
    
    PID_Init(&pid_left, &pid_param);
    PID_Init(&pid_right, &pid_param);
    
    /* 配置定时器中断 (10ms周期) */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

/* ── 速度控制 ────────────────────────────────────────────── */
static void MotorSystem_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    /* 获取当前编码器速度 */
    int32_t left_encoder = EncoderGpio_GetSpeed(ENCODER_GPIO_LEFT);
    int32_t right_encoder = EncoderGpio_GetSpeed(ENCODER_GPIO_RIGHT);
    
    /* PID计算 */
    int16_t left_pwm = (int16_t)PID_Calc(&pid_left, left_speed, left_encoder);
    int16_t right_pwm = (int16_t)PID_Calc(&pid_right, right_speed, right_encoder);
    
    /* 设置电机PWM */
    Motor_SetSpeed(MOTOR_A, left_pwm);
    Motor_SetSpeed(MOTOR_B, right_pwm);
}

/* ── 主函数 ──────────────────────────────────────────────── */
int main(void)
{
    /* 系统初始化 */
    System_Init();
    
    /* 电机系统初始化 */
    MotorSystem_Init();
    
    printf("电机+编码器+PID闭环控制示例\n");
    printf("目标速度: %d 脉冲/采样周期\n", TARGET_SPEED);
    
    /* 主循环 */
    uint32_t last_print_tick = 0;
    while (1) {
        /* 设置目标速度 */
        MotorSystem_SetSpeed(TARGET_SPEED, TARGET_SPEED);
        
        /* 每500ms打印一次状态 */
        if (sys_tick - last_print_tick >= 500) {
            last_print_tick = sys_tick;
            
            int32_t left_speed = EncoderGpio_GetSpeed(ENCODER_GPIO_LEFT);
            int32_t right_speed = EncoderGpio_GetSpeed(ENCODER_GPIO_RIGHT);
            int32_t left_count = EncoderGpio_Read(ENCODER_GPIO_LEFT);
            int32_t right_count = EncoderGpio_Read(ENCODER_GPIO_RIGHT);
            
            printf("左轮: 速度=%ld 累计=%ld | 右轮: 速度=%ld 累计=%ld\n",
                   left_speed, left_count, right_speed, right_count);
        }
        
        /* 延时10ms */
        DELAY_MS(10);
    }
    
    return 0;
}