/**
 * @file    test_encoder_gpio.c
 * @brief   GPIO编码器驱动测试 — MSPM0G3507
 * @note    测试N20电机编码器的GPIO中断方式读取
 *
 * 硬件连接:
 *   MSPM0 PB0 → 左轮A相   PB1 → 左轮B相
 *   MSPM0 PB2 → 右轮A相   PB3 → 右轮B相
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/encoder_gpio_mspm0.h"

/* ── 全局变量 ────────────────────────────────────────────── */
static volatile uint32_t sys_tick = 0;

/* ── 定时器中断处理 (1ms周期) ───────────────────────────── */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
        sys_tick++;
    }
}

/* ── 主函数 ──────────────────────────────────────────────── */
int main(void)
{
    /* 系统初始化 */
    System_Init();
    
    /* 编码器配置 */
    EncoderGpioConfig enc_cfg[ENCODER_GPIO_MAX] = {
        [ENCODER_GPIO_LEFT] = {
            .port = GPIOB, .pin_a = DL_GPIO_PIN_0, .pin_b = DL_GPIO_PIN_1,
            .inverted = 0
        },
        [ENCODER_GPIO_RIGHT] = {
            .port = GPIOB, .pin_a = DL_GPIO_PIN_2, .pin_b = DL_GPIO_PIN_3,
            .inverted = 1  /* 右轮编码器方向相反 */
        }
    };
    
    /* 初始化编码器 */
    EncoderGpio_Init(enc_cfg);
    
    printf("GPIO编码器驱动测试\n");
    printf("请转动电机观察编码器计数变化\n");
    
    /* 配置定时器中断 (1ms周期) */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    
    /* 主循环 */
    uint32_t last_print_tick = 0;
    uint32_t last_update_tick = 0;
    
    while (1) {
        /* 每10ms更新编码器速度 */
        if (sys_tick - last_update_tick >= 10) {
            last_update_tick = sys_tick;
            EncoderGpio_Update();
        }
        
        /* 每500ms打印一次状态 */
        if (sys_tick - last_print_tick >= 500) {
            last_print_tick = sys_tick;
            
            int32_t left_speed = EncoderGpio_GetSpeed(ENCODER_GPIO_LEFT);
            int32_t right_speed = EncoderGpio_GetSpeed(ENCODER_GPIO_RIGHT);
            int32_t left_count = EncoderGpio_Read(ENCODER_GPIO_LEFT);
            int32_t right_count = EncoderGpio_Read(ENCODER_GPIO_RIGHT);
            uint8_t left_dir = EncoderGpio_GetDirection(ENCODER_GPIO_LEFT);
            uint8_t right_dir = EncoderGpio_GetDirection(ENCODER_GPIO_RIGHT);
            
            printf("[%lu] 左轮: 速度=%ld 累计=%ld 方向=%s | 右轮: 速度=%ld 累计=%ld 方向=%s\n",
                   sys_tick,
                   left_speed, left_count, left_dir ? "反转" : "正转",
                   right_speed, right_count, right_dir ? "反转" : "正转");
        }
        
        /* 延时1ms */
        DELAY_MS(1);
    }
    
    return 0;
}