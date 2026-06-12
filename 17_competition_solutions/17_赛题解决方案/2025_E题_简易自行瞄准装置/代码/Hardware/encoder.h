/**
 * @file    encoder.h
 * @brief   编码器模块头文件
 * 
 * 硬件连接：
 * 左编码器A相 → TIM3_CH1(PA6)
 * 左编码器B相 → TIM3_CH2(PA7)
 * 右编码器A相 → TIM4_CH1(PB6)
 * 右编码器B相 → TIM4_CH2(PB7)
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

/* 编码器参数 */
#define ENCODER_PPR         13      // 编码器线数（脉冲/转）
#define ENCODER_GEAR_RATIO  34      // 减速比
#define WHEEL_DIAMETER_MM   65      // 车轮直径(mm)
#define ENCODER_PPR_TOTAL   (ENCODER_PPR * 4 * ENCODER_GEAR_RATIO)  // 四倍频后总脉冲数/转

/* 函数声明 */
void Encoder_Init(void);
void Encoder_Update(void);              // 在定时中断中调用，更新速度
float Encoder_GetLeftSpeed(void);       // 获取左轮速度(cm/s)
float Encoder_GetRightSpeed(void);      // 获取右轮速度(cm/s)
float Encoder_GetAvgSpeed(void);        // 获取平均速度(cm/s)
float Encoder_GetLeftDistance(void);    // 获取左轮累计距离(cm)
float Encoder_GetRightDistance(void);   // 获取右轮累计距离(cm)
float Encoder_GetAvgDistance(void);     // 获取平均累计距离(cm)
void Encoder_ResetDistance(void);       // 重置累计距离
int32_t Encoder_GetLeftCount(void);     // 获取左编码器原始计数
int32_t Encoder_GetRightCount(void);    // 获取右编码器原始计数

#endif /* __ENCODER_H */
