/**
 * @file    grayscale.h
 * @brief   感为8路灰度传感器驱动 — MSPM0G3507
 *
 * 无MCU版，8路数字输出，直接GPIO读取。
 * 黑线检测 → 低电平，白色区域 → 高电平
 *
 * 硬件连接: PB0~PB7 = G0~G7
 */

#ifndef __GRAYSCALE_H
#define __GRAYSCALE_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 8路通道编号 */
typedef enum {
    GRAY_CH0 = 0,
    GRAY_CH1 = 1,
    GRAY_CH2 = 2,
    GRAY_CH3 = 3,
    GRAY_CH4 = 4,
    GRAY_CH5 = 5,
    GRAY_CH6 = 6,
    GRAY_CH7 = 7,
} GrayscaleChannel;

/**
 * @brief 初始化灰度传感器（GPIO已在SysConfig中配置，此函数可留空或做额外配置）
 */
void Grayscale_Init(void);

/**
 * @brief 读取单路灰度传感器
 * @param ch  通道编号 (0~7)
 * @return 1=白色区域(高电平), 0=黑线(低电平), 0xFF=无效通道
 */
uint8_t Grayscale_Read(GrayscaleChannel ch);

/**
 * @brief 一次性读取全部8路，返回8位掩码
 * @return bit0=G0, bit1=G1, ... bit7=G7 (1=白, 0=黑)
 */
uint8_t Grayscale_ReadAll(void);

/**
 * @brief 计数白色通道数
 * @return 检测到白色的通道数量 (0~8)
 */
uint8_t Grayscale_CountWhite(void);

#endif /* __GRAYSCALE_H */
