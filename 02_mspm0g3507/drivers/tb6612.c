/**
 * @file    tb6612.c
 * @brief   TB6612FNG 双路直流电机驱动实现 — MSPM0G3507
 *
 * SysConfig生成的宏:
 *   TB6612_PORT, TB6612_AIN1_PIN, TB6612_AIN2_PIN, TB6612_BIN1_PIN, TB6612_BIN2_PIN
 *   PWM_0_INST, GPIO_PWM_0_C0_IDX, GPIO_PWM_0_C3_IDX
 */

#include "drivers/tb6612.h"

/* ── PWM最大值 ────────────────────────────────────────────── */
#define PWM_MAX  3999U

/* ── 内部: 设置A通道方向 ─────────────────────────────────── */
static void SetDirA(MotorDirection dir)
{
    switch (dir) {
    case MOTOR_DIR_FORWARD:
        DL_GPIO_setPins(TB6612_PORT, TB6612_AIN1_PIN);
        DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN2_PIN);
        break;
    case MOTOR_DIR_REVERSE:
        DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN1_PIN);
        DL_GPIO_setPins(TB6612_PORT, TB6612_AIN2_PIN);
        break;
    case MOTOR_DIR_BRAKE:
    default:
        DL_GPIO_setPins(TB6612_PORT, TB6612_AIN1_PIN);
        DL_GPIO_setPins(TB6612_PORT, TB6612_AIN2_PIN);
        break;
    }
}

/* ── 内部: 设置B通道方向 ─────────────────────────────────── */
static void SetDirB(MotorDirection dir)
{
    switch (dir) {
    case MOTOR_DIR_FORWARD:
        DL_GPIO_setPins(TB6612_PORT, TB6612_BIN1_PIN);
        DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN2_PIN);
        break;
    case MOTOR_DIR_REVERSE:
        DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN1_PIN);
        DL_GPIO_setPins(TB6612_PORT, TB6612_BIN2_PIN);
        break;
    case MOTOR_DIR_BRAKE:
    default:
        DL_GPIO_setPins(TB6612_PORT, TB6612_BIN1_PIN);
        DL_GPIO_setPins(TB6612_PORT, TB6612_BIN2_PIN);
        break;
    }
}

/* ── 公开API ─────────────────────────────────────────────── */

void TB6612_Init(void)
{
    TB6612_StopAll();
}

void TB6612_SetMotor(MotorChannel ch, MotorDirection dir, uint32_t speed)
{
    if (speed > PWM_MAX) speed = PWM_MAX;

    if (ch == MOTOR_CH_A) {
        SetDirA(dir);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, speed, GPIO_PWM_0_C0_IDX);
    } else {
        SetDirB(dir);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, speed, GPIO_PWM_0_C3_IDX);
    }
}

void TB6612_Brake(MotorChannel ch)
{
    if (ch == MOTOR_CH_A) {
        SetDirA(MOTOR_DIR_BRAKE);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    } else {
        SetDirB(MOTOR_DIR_BRAKE);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C3_IDX);
    }
}

void TB6612_Stop(MotorChannel ch)
{
    if (ch == MOTOR_CH_A) {
        DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN1_PIN);
        DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    } else {
        DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN1_PIN);
        DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C3_IDX);
    }
}

void TB6612_StopAll(void)
{
    TB6612_Stop(MOTOR_CH_A);
    TB6612_Stop(MOTOR_CH_B);
}
