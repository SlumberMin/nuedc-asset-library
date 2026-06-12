/**
 * @file    motor_mspm0.c
 * @brief   TB6612FNG 电机驱动实现 — MSPM0G3507
 */

#include "motor_mspm0.h"

/* ── 私有变量 ────────────────────────────────────────────── */
static MotorConfig g_motor_cfg[MOTOR_MAX];

/* ── 内部函数 ────────────────────────────────────────────── */
static void Motor_SetDir(MotorId id, MotorDir dir)
{
    const MotorConfig *m = &g_motor_cfg[id];
    switch (dir) {
    case MOTOR_DIR_FORWARD:
        GPIO_SET(m->port_in1, m->pin_in1);
        GPIO_CLR(m->port_in2, m->pin_in2);
        break;
    case MOTOR_DIR_REVERSE:
        GPIO_CLR(m->port_in1, m->pin_in1);
        GPIO_SET(m->port_in2, m->pin_in2);
        break;
    case MOTOR_DIR_BRAKE:
        GPIO_SET(m->port_in1, m->pin_in1);
        GPIO_SET(m->port_in2, m->pin_in2);
        break;
    case MOTOR_DIR_STOP:
    default:
        GPIO_CLR(m->port_in1, m->pin_in1);
        GPIO_CLR(m->port_in2, m->pin_in2);
        break;
    }
}

/* ── 公开 API ────────────────────────────────────────────── */

void Motor_Init(const MotorConfig *cfg)
{
    for (int i = 0; i < MOTOR_MAX; i++) {
        g_motor_cfg[i] = cfg[i];
    }
    Motor_Stop(MOTOR_A);
    Motor_Stop(MOTOR_B);
}

void Motor_SetSpeed(MotorId id, int16_t speed)
{
    if (id >= MOTOR_MAX) return;

    /* 限幅 */
    if (speed > 1000)  speed = 1000;
    if (speed < -1000) speed = -1000;

    if (speed > 0) {
        Motor_SetDir(id, MOTOR_DIR_FORWARD);
    } else if (speed < 0) {
        Motor_SetDir(id, MOTOR_DIR_REVERSE);
        speed = -speed;
    } else {
        Motor_Stop(id);
        return;
    }

    /* 映射 0~1000 → 0~pwm_period */
    const MotorConfig *m = &g_motor_cfg[id];
    uint32_t duty = (uint32_t)speed * m->pwm_period / 1000;
    PWM_SET_DUTY(m->pwm_timer, m->pwm_channel, duty);
}

void Motor_Brake(MotorId id)
{
    if (id >= MOTOR_MAX) return;
    Motor_SetDir(id, MOTOR_DIR_BRAKE);
    PWM_SET_DUTY(g_motor_cfg[id].pwm_timer,
                 g_motor_cfg[id].pwm_channel, 0);
}

void Motor_Stop(MotorId id)
{
    if (id >= MOTOR_MAX) return;
    Motor_SetDir(id, MOTOR_DIR_STOP);
    PWM_SET_DUTY(g_motor_cfg[id].pwm_timer,
                 g_motor_cfg[id].pwm_channel, 0);
}

int16_t Motor_GetPWM(MotorId id)
{
    if (id >= MOTOR_MAX) return 0;
    /* TODO: 实现PWM回读，暂返回0 */
    (void)id;
    return 0;
}
