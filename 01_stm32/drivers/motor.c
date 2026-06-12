/**
 * @file    motor.c
 * @brief   电机驱动模块实现
 * @details 支持L298N和TB6612FNG两种H桥驱动芯片。
 *          速度范围 -1000 ~ +1000 映射到PWM占空比 0~100%。
 *          正值正转，负值反转，0停止。
 */

#include "drivers/motor.h"

/* ========================================================================== */
/*                              内部函数                                       */
/* ========================================================================== */

/**
 * @brief 设置电机方向引脚（内部函数）
 * @param motor  电机结构体指针
 * @param dir    目标方向
 */
static void Motor_SetDirection(Motor_t *motor, MotorDir_t dir)
{
    switch (dir) {
        case MOTOR_DIR_FORWARD:
            GPIO_SET(motor->in1_port, motor->in1_pin);
            GPIO_CLR(motor->in2_port, motor->in2_pin);
            break;
        case MOTOR_DIR_BACKWARD:
            GPIO_CLR(motor->in1_port, motor->in1_pin);
            GPIO_SET(motor->in2_port, motor->in2_pin);
            break;
        case MOTOR_DIR_BRAKE:
            GPIO_SET(motor->in1_port, motor->in1_pin);
            GPIO_SET(motor->in2_port, motor->in2_pin);
            break;
        case MOTOR_DIR_COAST:
            GPIO_CLR(motor->in1_port, motor->in1_pin);
            GPIO_CLR(motor->in2_port, motor->in2_pin);
            break;
    }
}

/**
 * @brief 设置PWM输出值（内部函数）
 * @param motor  电机结构体指针
 * @param pwm    PWM比较值 0~ARR
 */
static void Motor_SetPWM(Motor_t *motor, uint32_t pwm)
{
    PWM_SET(motor->pwm_htim, motor->pwm_channel, pwm);
}

/* ========================================================================== */
/*                              接口函数实现                                   */
/* ========================================================================== */

ErrorCode_t Motor_Init(Motor_t *motor, MotorDriver_t driver,
                       GPIO_TypeDef *in1_port, uint16_t in1_pin,
                       GPIO_TypeDef *in2_port, uint16_t in2_pin,
                       TIM_HandleTypeDef *htim, uint32_t channel)
{
    /* 参数校验 */
    if (motor == NULL || in1_port == NULL || in2_port == NULL || htim == NULL) {
        return HAL_ERR_PARAM;
    }

    /* 保存配置 */
    motor->driver      = driver;
    motor->in1_port    = in1_port;
    motor->in1_pin     = in1_pin;
    motor->in2_port    = in2_port;
    motor->in2_pin     = in2_pin;
    motor->pwm_htim    = htim;
    motor->pwm_channel = channel;
    motor->speed       = 0;

    /* 初始状态：停止 */
    Motor_SetDirection(motor, MOTOR_DIR_COAST);
    Motor_SetPWM(motor, 0);

    /* 启动PWM */
    PWM_START(htim, channel);

    motor->initialized = true;

    DBG_PRINTF("Motor init OK: driver=%d, TIM%p CH%lu",
               driver, (void *)htim, channel);

    return HAL_OK_CODE;
}

ErrorCode_t Motor_SetSpeed(Motor_t *motor, int16_t speed)
{
    if (motor == NULL || !motor->initialized) {
        return HAL_ERR_NOT_INIT;
    }

    /* 限幅到 -1000 ~ +1000 */
    speed = (int16_t)CLAMP(speed, -1000, 1000);

    /* 死区处理：速度绝对值过小直接归零 */
    if (ABS(speed) < 10) {
        speed = 0;
    }

    motor->speed = speed;

    if (speed == 0) {
        /* 停止：滑行 */
        Motor_SetDirection(motor, MOTOR_DIR_COAST);
        Motor_SetPWM(motor, 0);
    } else if (speed > 0) {
        /* 正转 */
        Motor_SetDirection(motor, MOTOR_DIR_FORWARD);
        /* 将 0~1000 映射到 0~ARR */
        uint32_t arr = motor->pwm_htim->Init.Period;
        uint32_t pwm = (uint32_t)((float)speed / 1000.0f * arr);
        Motor_SetPWM(motor, pwm);
    } else {
        /* 反转 */
        Motor_SetDirection(motor, MOTOR_DIR_BACKWARD);
        uint32_t arr = motor->pwm_htim->Init.Period;
        uint32_t pwm = (uint32_t)((float)(-speed) / 1000.0f * arr);
        Motor_SetPWM(motor, pwm);
    }

    return HAL_OK_CODE;
}

int16_t Motor_GetSpeed(const Motor_t *motor)
{
    if (motor == NULL) {
        return 0;
    }
    return motor->speed;
}

ErrorCode_t Motor_Brake(Motor_t *motor)
{
    if (motor == NULL || !motor->initialized) {
        return HAL_ERR_NOT_INIT;
    }

    Motor_SetDirection(motor, MOTOR_DIR_BRAKE);
    Motor_SetPWM(motor, 0);
    motor->speed = 0;

    return HAL_OK_CODE;
}

ErrorCode_t Motor_Coast(Motor_t *motor)
{
    if (motor == NULL || !motor->initialized) {
        return HAL_ERR_NOT_INIT;
    }

    Motor_SetDirection(motor, MOTOR_DIR_COAST);
    Motor_SetPWM(motor, 0);
    motor->speed = 0;

    return HAL_OK_CODE;
}

ErrorCode_t Motor_DeInit(Motor_t *motor)
{
    if (motor == NULL) {
        return HAL_ERR_PARAM;
    }

    Motor_Coast(motor);
    PWM_STOP(motor->pwm_htim, motor->pwm_channel);
    motor->initialized = false;

    return HAL_OK_CODE;
}
