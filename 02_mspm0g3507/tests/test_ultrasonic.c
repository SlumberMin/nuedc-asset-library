/**
 * @file    test_ultrasonic.c
 * @brief   超声波传感器驱动测试 — MSPM0G3507
 * @note    测试SR04/US-016超声波测距传感器
 *
 * 硬件连接:
 *   MSPM0 PA0 → Trig (触发引脚)
 *   MSPM0 PA1 → Echo (回波引脚)
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/ultrasonic_mspm0.h"

/* ── 主函数 ──────────────────────────────────────────────── */
int main(void)
{
    /* 系统初始化 */
    System_Init();
    
    /* 超声波传感器配置 */
    UltrasonicConfig ultra_cfg = {
        .port = GPIOA,
        .trig_pin = DL_GPIO_PIN_0,
        .echo_pin = DL_GPIO_PIN_1,
        .type = ULTRASONIC_SR04,
        .filter_size = 5
    };
    
    /* 初始化超声波传感器 */
    Ultrasonic_Init(&ultra_cfg);
    
    printf("超声波传感器驱动测试\n");
    printf("请在传感器前方放置物体测试\n");
    
    /* 主循环 */
    uint32_t count = 0;
    while (1) {
        /* 测量距离 */
        float distance = Ultrasonic_Measure();
        
        /* 每500ms打印一次数据 */
        if (++count >= 500) {
            count = 0;
            
            if (distance > 0) {
                printf("距离: %.1f cm\n", distance);
            } else {
                printf("测量失败\n");
            }
        }
        
        /* 延时10ms */
        DELAY_MS(10);
    }
    
    return 0;
}