/**
 * @file    test_grayscale.c
 * @brief   灰度传感器驱动测试 — MSPM0G3507
 * @note    测试感为8路灰度循迹传感器的数据读取
 *
 * 硬件连接:
 *   MSPM0 PB0 → AD0 (ADC输入)
 *   MSPM0 PB1 → AD1 (ADC输入)
 *   MSPM0 PB2 → AD2 (ADC输入)
 *   MSPM0 PA27 → OUT (数字输出)
 *   MSPM0 PA0 → 地址位0
 *   MSPM0 PA1 → 地址位1
 *   MSPM0 PA2 → 地址位2
 *
 * 2024 电赛 · TI MSPM0G3507
 */

#include <stdio.h>
#include "platform/system_mspm0.h"
#include "platform/driverlib_mspm0.h"
#include "drivers/grayscale_mspm0.h"

/* ── 主函数 ──────────────────────────────────────────────── */
int main(void)
{
    /* 系统初始化 */
    System_Init();
    
    /* 灰度传感器配置 */
    GrayscaleConfig gray_cfg = {
        .addr_port = GPIOA,
        .addr_pin_0 = DL_GPIO_PIN_0,
        .addr_pin_1 = DL_GPIO_PIN_1,
        .addr_pin_2 = DL_GPIO_PIN_2,
        .adc = ADC12_0_INST,
        .adc_channel = DL_ADC12_MEM_IDX_0,
        .direction = 1  /* 反序 */
    };
    
    /* 初始化灰度传感器 */
    Grayscale_Init(&gray_cfg);
    
    /* 校准传感器 (根据实际黑白板调整) */
    uint16_t white_cal[8] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};
    uint16_t black_cal[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    Grayscale_Calibrate(white_cal, black_cal);
    
    printf("灰度传感器驱动测试\n");
    printf("请将传感器放在白色和黑色区域测试\n");
    
    /* 主循环 */
    uint32_t count = 0;
    while (1) {
        /* 读取传感器数据 */
        Grayscale_Read();
        
        /* 获取数据 */
        uint8_t digital = Grayscale_GetDigital();
        uint16_t analog[8];
        uint16_t normalized[8];
        Grayscale_GetAnalog(analog);
        Grayscale_GetNormalized(normalized);
        
        /* 计算循迹偏差 */
        int16_t error = Grayscale_GetTrackError();
        uint8_t off_track = Grayscale_IsOffTrack();
        uint8_t cross = Grayscale_DetectCross();
        
        /* 每500ms打印一次数据 */
        if (++count >= 500) {
            count = 0;
            
            printf("数字量: ");
            for (int i = 7; i >= 0; i--) {
                printf("%d", (digital >> i) & 1);
            }
            printf(" | 偏差: %d | %s | %s\n", 
                   error, 
                   off_track ? "脱线" : "在线",
                   cross ? "十字路口" : "");
            
            printf("模拟量: ");
            for (int i = 0; i < 8; i++) {
                printf("%d ", analog[i]);
            }
            printf("\n");
        }
        
        /* 延时1ms */
        DELAY_MS(1);
    }
    
    return 0;
}