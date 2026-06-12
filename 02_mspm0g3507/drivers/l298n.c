/**
 * @file    l298n.c
 * @brief   L298N 双路直流电机驱动实现 — MSPM0G3507
 *
 * SysConfig生成的宏:
 *   L298N_PORT, L298N_IN1_PIN, L298N_IN2_PIN, L298N_IN3_PIN, L298N_IN4_PIN
 *   PWM_0_INST, GPIO_PWM_0_C0_IDX, GPIO_PWM_0_C1_IDX
 */

#include "drivers/l298n.h"

/* ── PWM最大值 ────────────────────────────────────────────── */
#define PWM_MAX  3999U

/* ── 内部: 设置A通道方向 (IN1/IN2) ───────────────────────── */
static void L298N_SetDirA(L298N_Direction dir)
{
    switch (dir) {
    case L298N_DIR_FORWARD:
        DL_GPIO_setPins(L298N_PORT, L298N_IN1_PIN);
        DL_GPIO_clearPins(L298N_PORT, L298N_IN2_PIN);
        break;
    case L298N_DIR_REVERSE:
        DL_GPIO_clearPins(L298N_PORT, L298N_IN1_PIN);
        DL_GPIO_setPins(L298N_PORT, L298N_IN2_PIN);
        break;
    case L298N_DIR_BRAKE:
        DL_GPIO_setPins(L298N_PORT, L298N_IN1_PIN);
        DL_GPIO_setPins(L298N_PORT, L298N_IN2_PIN);
        break;
    case L298N_DIR_STOP:
    default:
        DL_GPIO_clearPins(L298N_PORT, L298N_IN1_PIN);
        DL_GPIO_clearPins(L298N_PORT, L298N_IN2_PIN);
        break;
    }
}

/* ── 内部: 设置B通道方向 (IN3/IN4) ───────────────────────── */
static void L298N_SetDirB(L298N_Direction dir)
{
    switch (dir) {
    case L298N_DIR_FORWARD:
        DL_GPIO_setPins(L298N_PORT, L298N_IN3_PIN);
        DL_GPIO_clearPins(L298N_PORT, L298N_IN4_PIN);
        break;
    case L298N_DIR_REVERSE:
        DL_GPIO_clearPins(L298N_PORT, L298N_IN3_PIN);
        DL_GPIO_setPins(L298N_PORT, L298N_IN4_PIN);
        break;
    case L298N_DIR_BRAKE:
        DL_GPIO_setPins(L298N_PORT, L298N_IN3_PIN);
        DL_GPIO_setPins(L298N_PORT, L298N_IN4_PIN);
        break;
    case L298N_DIR_STOP:
    default:
        DL_GPIO_clearPins(L298N_PORT, L298N_IN3_PIN);
        DL_GPIO_clearPins(L298N_PORT, L298N_IN4_PIN);
        break;
    }
}

/* ── 公开API ─────────────────────────────────────────────── */

void L298N_Init(void)
{
    L298N_StopAll();
}

void L298N_SetMotor(L298N_Channel ch, L298N_Direction dir, uint32_t speed)
{
    if (speed > PWM_MAX) speed = PWM_MAX;

    if (ch == L298N_CH_A) {
        L298N_SetDirA(dir);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, speed, GPIO_PWM_0_C0_IDX);
    } else {
        L298N_SetDirB(dir);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, speed, GPIO_PWM_0_C1_IDX);
    }
}

void L298N_Brake(L298N_Channel ch)
{
    if (ch == L298N_CH_A) {
        L298N_SetDirA(L298N_DIR_BRAKE);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    } else {
        L298N_SetDirB(L298N_DIR_BRAKE);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
    }
}

void L298N_Stop(L298N_Channel ch)
{
    if (ch == L298N_CH_A) {
        DL_GPIO_clearPins(L298N_PORT, L298N_IN1_PIN);
        DL_GPIO_clearPins(L298N_PORT, L298N_IN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    } else {
        DL_GPIO_clearPins(L298N_PORT, L298N_IN3_PIN);
        DL_GPIO_clearPins(L298N_PORT, L298N_IN4_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
    }
}

void L298N_StopAll(void)
{
    L298N_Stop(L298N_CH_A);
    L298N_Stop(L298N_CH_B);
}
