/**
 * @file main.c
 * @brief TB6612双路电机 + N20编码器 测试程序
 *
 * SysConfig: src/motor_encoder.syscfg
 */

#include "ti_msp_dl_config.h"
#include "drivers/tb6612.h"
#include "drivers/encoder.h"
#include <stdio.h>

/* ── SysTick ──────────────────────────────────────────────── */
volatile uint32_t g_systick = 0;
void SysTick_Handler(void) { g_systick++; }

void delay_ms(uint32_t ms) {
    volatile uint32_t start = g_systick;
    while ((g_systick - start) < ms) __WFI();
}

/* ── UART printf重定向 ────────────────────────────────────── */
int fputc(int ch, FILE *f) {
    while (!DL_UART_isTXFIFOEmpty(UART_0_INST));
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)ch);
    return ch;
}

/* ── 定时器中断: 编码器采样 ───────────────────────────────── */
void TIMER_0_INST_IRQHandler(void) {
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        Encoder_SampleCallback();
    }
}

/* ── GPIO中断: 编码器脉冲 ────────────────────────────────── */
void GROUP1_IRQHandler(void) {
    Encoder_GPIO_IRQHandler();
}

/* ── 主函数 ───────────────────────────────────────────────── */
int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(80000);  /* 1ms @ 80MHz */

    TB6612_Init();
    Encoder_Init();

    printf("\r\n=== TB6612 + Encoder Test ===\r\n");

    while (1) {
        /* 电机A正转50% */
        printf("\r\n[MOTOR A] Forward 50%%\r\n");
        TB6612_SetMotor(MOTOR_CH_A, MOTOR_DIR_FORWARD, 2000);
        for (int i = 0; i < 20; i++) {
            delay_ms(100);
            printf("  L: count=%ld speed=%ld\r\n",
                   Encoder_GetCount(ENC_LEFT), Encoder_GetSpeed(ENC_LEFT));
        }

        /* 电机A刹车 */
        printf("[MOTOR A] Brake\r\n");
        TB6612_Brake(MOTOR_CH_A);
        delay_ms(500);

        /* 电机A反转50% */
        printf("[MOTOR A] Reverse 50%%\r\n");
        TB6612_SetMotor(MOTOR_CH_A, MOTOR_DIR_REVERSE, 2000);
        for (int i = 0; i < 20; i++) {
            delay_ms(100);
            printf("  L: count=%ld speed=%ld\r\n",
                   Encoder_GetCount(ENC_LEFT), Encoder_GetSpeed(ENC_LEFT));
        }

        TB6612_Stop(MOTOR_CH_A);
        delay_ms(500);

        /* 电机B正转50% */
        printf("\r\n[MOTOR B] Forward 50%%\r\n");
        TB6612_SetMotor(MOTOR_CH_B, MOTOR_DIR_FORWARD, 2000);
        for (int i = 0; i < 20; i++) {
            delay_ms(100);
            printf("  R: count=%ld speed=%ld\r\n",
                   Encoder_GetCount(ENC_RIGHT), Encoder_GetSpeed(ENC_RIGHT));
        }

        TB6612_Brake(MOTOR_CH_B);
        delay_ms(500);

        /* 电机B反转50% */
        printf("[MOTOR B] Reverse 50%%\r\n");
        TB6612_SetMotor(MOTOR_CH_B, MOTOR_DIR_REVERSE, 2000);
        for (int i = 0; i < 20; i++) {
            delay_ms(100);
            printf("  R: count=%ld speed=%ld\r\n",
                   Encoder_GetCount(ENC_RIGHT), Encoder_GetSpeed(ENC_RIGHT));
        }

        TB6612_Stop(MOTOR_CH_B);
        delay_ms(500);

        DL_GPIO_togglePins(LED_PORT, LED_PIN_PIN);
        printf("\r\n=== Loop ===\r\n");
    }
}
