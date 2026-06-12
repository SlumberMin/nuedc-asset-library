/**
 * @file    grayscale.c
 * @brief   感为8路灰度传感器驱动实现 — MSPM0G3507
 *
 * SysConfig生成的宏:
 *   GRAY_PORT, GRAY_G0_PIN ~ GRAY_G7_PIN
 */

#include "drivers/grayscale.h"

/* 引脚查找表，按通道号索引 */
static const uint32_t gray_pins[8] = {
    GRAY_G0_PIN,
    GRAY_G1_PIN,
    GRAY_G2_PIN,
    GRAY_G3_PIN,
    GRAY_G4_PIN,
    GRAY_G5_PIN,
    GRAY_G6_PIN,
    GRAY_G7_PIN,
};

void Grayscale_Init(void)
{
    /* GPIO方向和引脚已在SysConfig中配置，无需额外初始化 */
}

uint8_t Grayscale_Read(GrayscaleChannel ch)
{
    if (ch > GRAY_CH7) return 0xFF;
    return DL_GPIO_readPins(GRAY_PORT, gray_pins[ch]) ? 1 : 0;
}

uint8_t Grayscale_ReadAll(void)
{
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        if (DL_GPIO_readPins(GRAY_PORT, gray_pins[i])) {
            result |= (1 << i);
        }
    }
    return result;
}

uint8_t Grayscale_CountWhite(void)
{
    uint8_t bits = Grayscale_ReadAll();
    uint8_t count = 0;
    for (int i = 0; i < 8; i++) {
        if (bits & (1 << i)) count++;
    }
    return count;
}
