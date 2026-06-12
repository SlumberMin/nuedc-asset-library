/**
 * @file    tb6612_stm32.c
 * @brief   TB6612FNG 电机驱动模块实现 — STM32 HAL库版本
 */

#include "drivers/tb6612_stm32.h"
#include <math.h>

/* ========================================================================== */
/*                              内部函数                                       */
/* ========================================================================== */

/** @brief 设置方向引脚 */
static void TB6612_SetDir(TB6612_Motor_t *m, TB6612_Dir_t dir)
{
    switch (dir) {
        case TB6612_DIR_FORWARD:
            HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
            break;
        case TB6612_DIR_BACKWARD:
            HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
            break;
        case TB6612_DIR_BRAKE:
            HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
            break;
        case TB6612_DIR_COAST:
            HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
            break;
    }
}

/** @brief 设置PWM占空比 */
static void TB6612_SetPWM(TB6612_Motor_t *m, uint32_t pwm)
{
    __HAL_TIM_SET_COMPARE(m->pwm_htim, m->pwm_channel, pwm);
}

/* ========================================================================== */
/*                              接口函数实现                                   */
/* ========================================================================== */

HAL_StatusTypeDef TB6612_Init(TB6612_Dev_t *dev,
                              GPIO_TypeDef *stby_port, uint16_t stby_pin)
{
    if (dev == NULL) return HAL_ERROR;

    dev->stby_port  = stby_port;
    dev->stby_pin   = stby_pin;
    dev->motor_a.initialized = false;
    dev->motor_b.initialized = false;

    /* 使能芯片 */
    if (stby_port != NULL) {
        HAL_GPIO_WritePin(stby_port, stby_pin, GPIO_PIN_SET);
    }
    dev->initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef TB6612_MotorInit(TB6612_Motor_t *motor,
                                   GPIO_TypeDef *in1_port, uint16_t in1_pin,
                                   GPIO_TypeDef *in2_port, uint16_t in2_pin,
                                   TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (motor == NULL || in1_port == NULL || in2_port == NULL || htim == NULL)
        return HAL_ERROR;

    motor->in1_port     = in1_port;
    motor->in1_pin      = in1_pin;
    motor->in2_port     = in2_port;
    motor->in2_pin      = in2_pin;
    motor->pwm_htim     = htim;
    motor->pwm_channel  = channel;
    motor->speed        = 0;

    /* 初始状态：滑行，PWM=0 */
    TB6612_SetDir(motor, TB6612_DIR_COAST);
    TB6612_SetPWM(motor, 0);

    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(htim, channel);

    motor->initialized = true;
    return HAL_OK;
}

void TB6612_SetSpeed(TB6612_Motor_t *motor, int16_t speed)
{
    if (motor == NULL || !motor->initialized) return;

    /* 限幅 */
    if (speed > 1000)  speed = 1000;
    if (speed < -1000) speed = -1000;

    motor->speed = speed;

    uint32_t arr = motor->pwm_htim->Init.Period;

    if (speed == 0) {
        TB6612_SetDir(motor, TB6612_DIR_COAST);
        TB6612_SetPWM(motor, 0);
    } else if (speed > 0) {
        TB6612_SetDir(motor, TB6612_DIR_FORWARD);
        uint32_t pwm = (uint32_t)((float)speed / 1000.0f * arr);
        TB6612_SetPWM(motor, pwm);
    } else {
        TB6612_SetDir(motor, TB6612_DIR_BACKWARD);
        uint32_t pwm = (uint32_t)((float)(-speed) / 1000.0f * arr);
        TB6612_SetPWM(motor, pwm);
    }
}

int16_t TB6612_GetSpeed(const TB6612_Motor_t *motor)
{
    if (motor == NULL) return 0;
    return motor->speed;
}

void TB6612_Brake(TB6612_Motor_t *motor)
{
    if (motor == NULL || !motor->initialized) return;
    TB6612_SetDir(motor, TB6612_DIR_BRAKE);
    TB6612_SetPWM(motor, 0);
    motor->speed = 0;
}

void TB6612_Coast(TB6612_Motor_t *motor)
{
    if (motor == NULL || !motor->initialized) return;
    TB6612_SetDir(motor, TB6612_DIR_COAST);
    TB6612_SetPWM(motor, 0);
    motor->speed = 0;
}

void TB6612_Enable(TB6612_Dev_t *dev, bool enable)
{
    if (dev == NULL || dev->stby_port == NULL) return;
    HAL_GPIO_WritePin(dev->stby_port, dev->stby_pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
