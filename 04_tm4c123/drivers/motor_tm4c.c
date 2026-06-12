/**
 * @file motor_tm4c.c
 * @brief TM4C123 电机驱动实现
 */
#include "platform/tivaware.h"
#include "drivers/motor_tm4c.h"

/* ======================== 内部变量 ======================== */
static motor_handle_t motor[MOTOR_CH_MAX] = {
    [MOTOR_CH_A] = {
        .pwm_gen = PWM_OUT_0,
        .in1_port = GPIO_PORTA_BASE, .in1_pin = GPIO_PIN_2,
        .in2_port = GPIO_PORTA_BASE, .in2_pin = GPIO_PIN_3,
    },
    [MOTOR_CH_B] = {
        .pwm_gen = PWM_OUT_1,
        .in1_port = GPIO_PORTA_BASE, .in1_pin = GPIO_PIN_4,
        .in2_port = GPIO_PORTA_BASE, .in2_pin = GPIO_PIN_5,
    },
};

/* ======================== 实现 ======================== */

void motor_init(void)
{
    /* 1. 使能外设时钟 */
    PERIPH_ENABLE(SYSCTL_PERIPH_PWM0);
    PERIPH_ENABLE(SYSCTL_PERIPH_GPIOB);
    PERIPH_ENABLE(SYSCTL_PERIPH_GPIOA);
    periph_wait_ready(SYSCTL_PERIPH_PWM0);
    periph_wait_ready(SYSCTL_PERIPH_GPIOB);
    periph_wait_ready(SYSCTL_PERIPH_GPIOA);

    /* 2. 配置PWM引脚: PB6->M0PWM0, PB7->M0PWM1 */
    MAP_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    MAP_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    MAP_GPIOPinConfigure(GPIO_PB6_M0PWM0);
    MAP_GPIOPinConfigure(GPIO_PB7_M0PWM1);

    /* 3. 配置PWM时钟分频: SYSCLK/1 = 80MHz */
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);

    /* 4. 配置PWM发生器: 下降沿对齐，递减计数 */
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    /* 5. 设置PWM周期: 80MHz / 20kHz = 4000 */
    uint32_t period = SYS_CLK_FREQ / MOTOR_PWM_FREQ_HZ;
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period);
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period);

    /* 6. 初始占空比为0 */
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);

    /* 7. 使能PWM输出 */
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    /* 8. 配置方向控制GPIO为输出 */
    for (int i = 0; i < MOTOR_CH_MAX; i++) {
        GPIO_OUTPUT_PP(motor[i].in1_port, motor[i].in1_pin);
        GPIO_OUTPUT_PP(motor[i].in2_port, motor[i].in2_pin);
        GPIO_PIN_CLR(motor[i].in1_port, motor[i].in1_pin);
        GPIO_PIN_CLR(motor[i].in2_port, motor[i].in2_pin);
    }
}

void motor_set(motor_ch_t ch, int16_t speed)
{
    if (ch >= MOTOR_CH_MAX) return;

    motor_handle_t *m = &motor[ch];
    float duty = (float)ABS(speed) / 1000.0f;
    duty = CLAMP(duty, 0.0f, 1.0f);

    /* 设置方向 */
    if (speed > 0) {
        GPIO_PIN_SET(m->in1_port, m->in1_pin);
        GPIO_PIN_CLR(m->in2_port, m->in2_pin);
        m->dir = MOTOR_DIR_FWD;
    } else if (speed < 0) {
        GPIO_PIN_CLR(m->in1_port, m->in1_pin);
        GPIO_PIN_SET(m->in2_port, m->in2_pin);
        m->dir = MOTOR_DIR_REV;
    } else {
        GPIO_PIN_CLR(m->in1_port, m->in1_pin);
        GPIO_PIN_CLR(m->in2_port, m->in2_pin);
        m->dir = MOTOR_DIR_BRAKE;
    }

    /* 设置PWM占空比 */
    uint32_t load = pwm_get_load(MOTOR_PWM_BASE, 0);
    uint32_t pulse = (uint32_t)(duty * (float)load);
    MAP_PWMPulseWidthSet(MOTOR_PWM_BASE, m->pwm_gen, pulse);
    m->duty = duty;
}

void motor_brake(motor_ch_t ch)
{
    motor_set(ch, 0);
}

void motor_stop_all(void)
{
    motor_set(MOTOR_CH_A, 0);
    motor_set(MOTOR_CH_B, 0);
}
